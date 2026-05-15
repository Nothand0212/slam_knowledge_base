---
tags: [gtsam_points, GTSAM, 因子图, scan matching, LiDAR, 点云配准, 因子封装]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
---

# gtsam_points 因子封装模式

> gtsam_points 将多种 LiDAR 点云配准算法（ICP、GICP、VGICP、CT-ICP）统一封装为 GTSAM NonlinearFactor，使扫描匹配约束可与 IMU 预积分、GNSS、视觉重投影因子在同一个因子图中联合优化。

## 设计动机

在传统 LiDAR SLAM 系统中，点云配准通常由一个独立的前端优化器执行（如 Ceres 或手写高斯-牛顿），将配准结果作为位姿约束注入后端因子图。这种方式存在两个问题：

1. **线性化点不一致**：前端和后端使用不同的线性化点，破坏了因子图 optimize → relinearize 的一致性假设。
2. **信息矩阵丢失**：前端配准的协方差/信息矩阵难以准确传播到后端。

gtsam_points 通过将配准计算直接封装为 GTSAM factor，使配准残差与因子图变量共享同一线性化点，线性化、求解、更新变量由 GTSAM 原生流程统一管理。

## 因子类层次结构

gtsam_points 的因子体系分为两层：

### 抽象基类：`IntegratedMatchingCostFactor`

```text
gtsam::NonlinearFactor
    └── IntegratedMatchingCostFactor        (include/gtsam_points/factors/integrated_matching_cost_factor.hpp:19)
            ├── IntegratedICPFactor         (include/gtsam_points/factors/integrated_icp_factor.hpp:22)
            ├── IntegratedGICPFactor        (include/gtsam_points/factors/integrated_gicp_factor.hpp:31)
            ├── IntegratedVGICPFactor       (include/gtsam_points/factors/integrated_vgicp_factor.hpp:25)
            ├── IntegratedVGICPFactorGPU    (GPU 批量版本)
            └── IntegratedLOAMFactor        (include/gtsam_points/factors/integrated_loam_factor.hpp)
```

`IntegratedMatchingCostFactor` 是二元/一元点云配准因子的共同基类，核心职责：

- 继承 `gtsam::NonlinearFactor`，维度固定为 6（SE(3) 位姿）
- 支持**二元模式**（两个位姿变量 key）和**一元模式**（固定 target 位姿 + 可变 source 位姿 key）
- 实现 `error(values)`：从 values 中提取位姿变量 → 计算相对变换 delta → 调用虚函数 `evaluate(delta)` 得到配准残差
- 实现 `linearize(values)`：先调用 `update_correspondences(delta)` 更新数据关联 → 再调用 `evaluate(delta, H_target, H_source, ...)` 得到 Hessian 和梯度 → 组装成 `gtsam::HessianFactor` 返回

关键源码 `raw/codes/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:L37-L55`：

```cpp
gtsam::GaussianFactor::shared_ptr IntegratedMatchingCostFactor::linearize(const gtsam::Values& values) const {
    Eigen::Isometry3d delta = calc_delta(values);        // 从 values 提取 target/source 位姿求相对变换
    update_correspondences(delta);                        // 虚函数：子类实现数据关联
    Eigen::Matrix<double, 6, 6> H_target, H_source, H_target_source;
    Eigen::Matrix<double, 6, 1> b_target, b_source;
    double error = evaluate(delta, &H_target, &H_source, &H_target_source, &b_target, &b_source); // 虚函数
    if (is_binary) {
        factor.reset(new gtsam::HessianFactor(keys()[0], keys()[1],
            H_target, H_target_source, -b_target, H_source, -b_source, error));
    } else {
        factor.reset(new gtsam::HessianFactor(keys()[0], H_source, -b_source, error));
    }
    return factor;
}
```

### 两个纯虚函数接口

子类必须实现两个纯虚函数，它们定义了配准算法的核心：

| 虚函数 | 职责 | 调用时机 |
|--------|------|----------|
| `update_correspondences(delta)` | 根据当前相对位姿更新数据关联（KNN / 体素查找）并预热缓存（马氏距离矩阵等） | 每次 `linearize()` 开始时 |
| `evaluate(delta, H*, b*)` | 计算配准误差、Hessian、梯度 | `error()` 和 `linearize()` 都会调用 |

这种模板方法模式使新增一种配准因子只需实现数据关联和残差求值，无需关心 GTSAM 因子图的簿记逻辑。

### 线程安全的 mutable 成员模式

配准因子持有 **mutable** 成员变量（对应关系缓存、马氏距离缓存），因为 `error()` 和 `linearize()` 是 GTSAM 框架中**非线程安全**的接口——它们被设计为在每次线性化点改变时重新执行数据关联。

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/integrated_gicp_factor.hpp:L137-L148`：

```cpp
// I'm unhappy to have mutable members...
double correspondence_update_tolerance_rot;
double correspondence_update_tolerance_trans;
mutable Eigen::Isometry3d linearization_point;
mutable Eigen::Isometry3d last_correspondence_point;
mutable std::vector<long> correspondences;
mutable std::vector<Eigen::Matrix4d> mahalanobis_full;
mutable std::vector<Eigen::Matrix<float, 6, 1>> mahalanobis_compact;
```

**对应关系更新容忍度**机制（`correspondence_update_tolerance_*`）：当两次线性化之间的相对位姿变化小于预设阈值时，跳过 KNN 查找，复用缓存对应关系，以减少不必要的计算。

### 独立因子：`IntegratedCT_ICPFactor`

连续时间 ICP 因子**不继承** `IntegratedMatchingCostFactor`，而是直接继承 `gtsam::NonlinearFactor`。因为它需要 `t0` 和 `t1` 两个 key（扫描起止位姿），且 `error()` 和 `linearize()` 的语义不同——需要先插值中间位姿再执行点级别计算。详见[[方法-gtsam_points连续时间因子]]。

## 通用残差与雅可比模式

### Point-to-Point ICP 残差

对于 source 点 $p_i^A$ 和 target 对应点 $p_j^B$，变换到 target 坐标系后的残差为：

$$r_i = T_{BA} p_i^A - p_j^B$$

其中 $T_{BA} = T_B^{-1} T_A$ 是 source 到 target 的相对变换。对应的代价为 $\sum_i \|r_i\|^2$。

### Point-to-Plane ICP 残差

利用 target 点的法向量 $n_j^B$，将残差投影到法向方向：

$$e_i = n_j^B \cdot r_i = n_j^B \cdot (T_{BA} p_i^A - p_j^B)$$

代价为 $\sum_i \|e_i\|^2$。这是单自由度的标量残差。

### GICP 马氏距离残差

考虑 source 和 target 点各自的局部协方差 $C_i^A$、$C_j^B$：

$$\Sigma_{ij} = C_j^B + R_{BA} C_i^A R_{BA}^T$$

其中 $R_{BA}$ 是 $T_{BA}$ 的旋转部分。GICP 的马氏距离误差为：

$$d_i^2 = r_i^T \Sigma_{ij}^{-1} r_i$$

协方差对旋转的变换仅传播到 $C_i^A$ 的 3×3 空间块，齐次坐标第 4 维保持为 0。

### 雅可比结构

对 SE(3) 位姿 $T_{BA}$（或等效表示为一对位姿 $T_A, T_B$）求导，使用左乘扰动 $\exp(\delta \xi^\wedge) \cdot T$ 的标准 Lie 代数导数。

对于二元因子，target 和 source 位姿的雅可比块分别为：

$$J_{\text{target}} = \begin{bmatrix} -[T_{BA} p_i^A]_\times & I \end{bmatrix}, \quad J_{\text{source}} = \begin{bmatrix} R_{BA} [p_i^A]_\times & -R_{BA} \end{bmatrix}$$

其中 $[v]_\times$ 表示向量的反对称矩阵。对于 Point-to-Plane，两个雅可比块各乘以法向量的对角矩阵。

实际实现参见 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:L220-L238`。

### 并行 Reduction 模式

所有配准因子的 `evaluate()` 使用 `scan_matching_reduce_omp()` 或 `scan_matching_reduce_tbb()` 做并行归约。每个线程独立累加局部 $H_{6\times6}$ 和 $b_{6\times1}$，最后合并到全局矩阵。

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/scan_matching_reduction.hpp:L16-L68`：

```cpp
template <typename Transform>
double scan_matching_reduce_omp(
    const Transform f, int num_points, int num_threads,
    Eigen::Matrix<double, 6, 6>* H_target, ...) {
    std::vector<Eigen::Matrix<double, 6, 6>> Hs_target(num_threads, ...Zero());
    // ... per-thread accumulators
    #pragma omp parallel for num_threads(num_threads) schedule(guided, 8) reduction(+ : sum_errors)
    for (int i = 0; i < num_points; i++) {
        int thread_num = omp_get_thread_num();
        error = f(i, &Hs_target[thread_num], ...);
        sum_errors += error;
    }
    // Merge per-thread accumulators
    for (int i = 1; i < num_threads; i++) {
        *H_target += Hs_target[i];
        ...
    }
}
```

## VGICP 体素关联的特殊性

VGICP 因子（`IntegratedVGICPFactor`）不通过 KNN 查找对应关系，而是：

1. 将 target 点云预建为 `GaussianVoxelMap`（体素高斯分布地图）
2. 对每个 source 点 $p_i^A$，计算其体素坐标 → 哈希查找对应体素高斯 $(\mu_j, C_j)$
3. 数据关联从 `O(log N)` KNN 降为 `O(1)` 体素查找

这使得 VGICP 在扫描到地图（scan-to-map）配准中比传统 GICP 快一个数量级。详见[[方法-gtsam_points体素加速]]。

## Agent 实现提示

### 适用场景

- 需要在 GTSAM 因子图中嵌入 LiDAR scan matching 约束，与 IMU、GNSS、视觉因子联合优化
- 构建 LIO（LiDAR-Inertial Odometry）或 LVI（LiDAR-Visual-Inertial）系统时替代独立前端优化器
- 需要灵活切换 ICP/GICP/VGICP 配准算法而不改变因子图结构
- 不适合：纯位姿跟踪且不需要因子图框架的轻量级系统

### 输入输出契约

- **输入**：target/source 点云（shared_ptr 共享所有权，不复制），最近邻搜索结构（可选外部提供），GTSAM Key 对，最大对应距离，线程数，对应关系更新容忍度，协方差缓存模式
- **输出**：GTSAM HessianFactor（6×6 Hessian 块 + 6×1 梯度向量 + 标量误差），供 ISAM2/Levenberg-Marquardt 等求解器调用
- **隐含约定**：`error()` 返回 `evaluate()` 的误差值，不计算 Hessian；`linearize()` 返回包含 Hessian、梯度和误差的 GaussianFactor；factor 的 `dim()` 恒为 6；target 和 source 点云生命周期必须长于 factor 实例

### 实现骨架（伪代码）

```text
class MyScanMatchingFactor : public IntegratedMatchingCostFactor:
    data:
        target_frame: shared_ptr<PointCloud>       // 目标点云
        source_frame: shared_ptr<PointCloud>       // 源点云
        target_tree: shared_ptr<NearestNeighborSearch>  // 目标近邻结构
        correspondences: mutable vector<long>      // 对应关系缓存
        mahalanobis_cache: mutable vector<Matrix4> // 马氏距离缓存 (GICP 系)

    constructor(target_key, source_key, target, source, target_tree?):
        super(target_key, source_key)              // is_binary=true
        validate(target has points[, covs])
        validate(source has points[, covs])
        this.target_tree = target_tree || new KdTree2(target)

    virtual update_correspondences(delta):
        correspondences.resize(source.size())
        #pragma omp parallel for
        for each source point i:
            pt_transformed = delta * source.point[i]
            k_idx, k_dist = target_tree.knn_search(pt_transformed, k=1, max_dist)
            correspondences[i] = (k_dist < max_dist) ? k_idx : -1
            if has_covariances:
                C_combined = target.cov[j] + delta.R * source.cov[i] * delta.R^T
                mahalanobis_cache[i] = invert(C_combined[0:3,0:3])

    virtual evaluate(delta, H_target?, H_source?, H_ts?, b_target?, b_source?):
        sum_errors = 0
        for each source point i:
            j = correspondences[i];  if j < 0: continue
            p_src = source.point[i];  p_tgt = target.point[j]
            p_trans = delta * p_src
            r = p_trans - p_tgt

            if GICP mode:
                M = mahalanobis_cache[i]
                e = r^T * M * r
                if H_target:
                    J_tgt = [-hat(p_trans), I]   // 4x6
                    J_src = [R * hat(p_src), -R]  // 4x6
                    H_target += J_tgt^T * M * J_tgt
                    H_source += J_src^T * M * J_src
                    b_target += J_tgt^T * M * r
                    b_source += J_src^T * M * r
            else if Point-to-Plane mode:
                n = target.normal[j]
                e = (n · r)^2
                // 雅可比乘以法向量投影
            else: // Point-to-Point
                e = r^T * r
                // 标准欧氏距离雅可比

            sum_errors += e
        return sum_errors

    // error() 和 linearize() 由基类 IntegratedMatchingCostFactor 提供通用实现
    // 不需要覆写，除非有特殊需求（例如连续时间因子）
```

### 关键源码片段

1. **基类 linearize 实现** — `raw/codes/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:L37-L55`（见上文）
2. **Point-to-Plane ICP 残差与雅可比** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:L192-L247`
3. **GICP 数据关联与马氏缓存** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L132-L215`
4. **VGICP 体素查找对应** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:L99-L153`
5. **并行 Reduction 工具** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/scan_matching_reduction.hpp:L16-L68`
6. **GICP 雅可比结构** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L270-L287`

### 实现注意事项

- **协方差矩阵的正则化**：GICP 系列因子要求 target 和 source 点必须携带协方差 $C \in \mathbb{R}^{4\times4}$（最后一行为零的齐次扩展）。协方差必须可逆。实际使用时应在估计后加对角正则项 $\lambda I$ 防止 $C_j + RC_i R^T$ 接近奇异。
- **对应关系容忍度阈值**：默认值为 0（每帧都全量更新 KNN）。在 scan-to-scan 配准中，帧间运动通常不大，可设置 `set_correspondence_update_tolerance(0.1, 0.05)` 减少 KNN 调用。但在回环检测中应设为 0。
- **协方差缓存策略**：`FusedCovCacheMode::FULL`（128 bit/点，快）适合 GPU 因子；`COMPACT`（24 bit/点）适合内存受限场景；`NONE` 适合只需 error 值无需线性化的场合。切换模式影响 `memory_usage()` 报告值。
- **目标树的生命周期**：`NearestNeighborSearch` 按 `shared_ptr<const>` 传入，factor 不复制点云。如果使用 iVox 动态地图，同一地图实例可在多帧 scan-to-map 因子间共享。
- **线程安全边界**：`error()` 和 `linearize()` 被标记为非线程安全。在 ISAM2 的并行求解中，同一 factor 在两次线性化之间不会并发调用这两个方法；但不同 factor 的调用是并发的。`mutable` 成员只在 factor 内部修改，不应允许跨 factor 共享。
- **与 GTSAM 因子的区别**：GTSAM 原生因子（如 `BetweenFactor<Pose3>`）是解析的闭合形式；gtsam_points 因子是**数据驱动**的——误差和雅可比依赖于最新的数据关联结果，因此线性化点必须与数据关联计算时的 delta 一致。

### 源码检索锚点

- 抽象基类头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
- 抽象基类实现：`raw/codes/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp`
- ICP 因子实现头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp`
- GICP 因子实现头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp`
- VGICP 因子实现头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp`
- CT-ICP 因子头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_ct_icp_factor.hpp`
- 并行归约工具：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/scan_matching_reduction.hpp`
- 最近邻搜索接口：`raw/codes/gtsam_points/include/gtsam_points/ann/nearest_neighbor_search.hpp`
- PointCloud 数据布局：`raw/codes/gtsam_points/include/gtsam_points/types/point_cloud.hpp`

## 相关页面

- [[组件-gtsam_points]]
- [[方法-gtsam_points体素加速]]
- [[方法-gtsam_points连续时间因子]]
- [[方法-IntegratedMatchingCostFactor]]
- [[方法-GICP配准方法]]
- [[方法-ICP配准方法]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[方法-ICP变体族]]
- [[方法-GTSAM-API族]]
- [[组件-GTSAM]]
- [[概念-因子图]]
