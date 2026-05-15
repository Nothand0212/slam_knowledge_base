---
tags: [gtsam_points, 连续时间, CT-ICP, CT-GICP, 运动畸变, deskew, 因子图, LiDAR]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
---

# gtsam_points 连续时间因子

> gtsam_points 将激光扫描帧建模为起止两个位姿之间的连续刚体运动，通过时间参数化插值实现每点独立位姿，在因子图优化中内建消除运动畸变，无需预 deskew 步骤。

## 设计动机

传统 LiDAR SLAM 处理帧内运动畸变的典型流程是：

1. **预 deskew**：用 IMU 角速度/线速度或匀速模型，将扫描帧内各点投影到同一时刻
2. **扫描匹配**：对 deskew 后的点云执行 ICP/GICP
3. **因子图优化**：将匹配结果作为位姿约束加入图

这种方法的问题在于：deskew 线性化点（通常是 IMU 预测位姿）与因子图线性化点（优化后的位姿）不一致，导致信息损失。CT-ICP 的解法是将扫描起止位姿同时作为因子变量，让优化器直接处理每点的真实采集位姿。

## 架构设计

### 类继承关系

连续时间因子**不继承** `IntegratedMatchingCostFactor`，而是直接继承 `gtsam::NonlinearFactor`。

```text
gtsam::NonlinearFactor
    └── IntegratedCT_ICPFactor_<TargetFrame, SourceFrame>     (ct_icp_factor.hpp:21)
            └── IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>  (ct_gicp_factor.hpp:17)
```

设计理由：
- `IntegratedMatchingCostFactor` 的基类语义假设二元/一元因子通过 `calc_delta(values)` 提取单个相对变换，然后传给 `evaluate(delta)` 和 `update_correspondences(delta)`
- 连续时间因子需要两个 key（起止位姿 $T_{t0}, T_{t1}$），且 `error()` 和 `linearize()` 流程完全不同——需先插值中间位姿再执行逐点计算
- 连续时间因子的构造参数也不同——需要 `time_table` 和 `time_indices`

### 时间参数化

扫描帧内每点具有时间戳 $t_i$（相对于扫描起始时刻），归一化到 $[0, 1]$：

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L41-L55`：

```cpp
const double time_eps = 1e-3;
for (int i = 0; i < frame::size(*source); i++) {
    const double t = frame::time(*source, i);
    if (time_table.empty() || t - time_table.back() > time_eps) {
        time_table.push_back(t);       // 去重后的时间离散点
    }
    time_indices.push_back(time_table.size() - 1);  // 每点映射到时间索引
}
for (auto& t : time_table) {
    t = t / std::max(1e-9, time_table.back());  // 归一化到 [0, 1]
}
```

设计要点：
- `time_table`：去重后的时间离散点（相邻点时间差 < 1ms 的合并），用于减少插值计算量
- `time_indices`：每点映射到 `time_table` 索引，是 N 点→M 时间的降维映射（通常 M ≪ N）
- `time_table.back()` 可能为 0（所有点时间相同），此时归一化结果为 0，退化为纯匀速

### 位姿插值

给定 GTSAM values 中的起止位姿 $T_{t0}, T_{t1}$，对每个时间离散点 $t_k \in [0, 1]$：

1. 计算相对变换 $\Delta = T_{t0}^{-1} T_{t1}$ （即 $T_{t0}^{t1}$）
2. 对数映射得到李代数 $\xi = \log(\Delta)$
3. 缩放线性插值 $t_k \cdot \xi$ 后指数映射回 SE(3)
4. 组合得到 $T_{t_k} = T_{t0} \cdot \exp(t_k \cdot \xi)$

数学上，这是 SE(3) 上的**直线插值**（使用左乘形式）：

$$T_{t_k} = T_{t0} \cdot \exp\!\left(t_k \cdot \log(T_{t0}^{-1} T_{t1})\right)$$

这与 CT-ICP 论文（Bellenbach 2021）的弹性体模型一致，假设扫描帧内运动为匀速刚体变换。

### update_poses() 的完整实现

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L202-L248`：

```cpp
void update_poses(const gtsam::Values& values) const {
    gtsam::Pose3 pose0 = values.at<gtsam::Pose3>(keys_[0]);
    gtsam::Pose3 pose1 = values.at<gtsam::Pose3>(keys_[1]);

    gtsam::Matrix6 H_delta_0, H_delta_1;
    gtsam::Pose3 delta = pose0.between(pose1, H_delta_0, H_delta_1);  // Δ = T0^{-1} T1

    gtsam::Matrix6 H_vel_delta;
    gtsam::Vector6 vel = gtsam::Pose3::Logmap(delta, H_vel_delta);    // ξ = log(Δ)

    source_poses.resize(time_table.size());
    pose_derivatives_t0.resize(time_table.size());
    pose_derivatives_t1.resize(time_table.size());

    for each time_index i:
        t = time_table[i]
        gtsam::Matrix6 H_inc_vel;
        gtsam::Pose3 inc = gtsam::Pose3::Expmap(t * vel, H_inc_vel);  // exp(tξ)

        gtsam::Matrix6 H_pose_0_a, H_pose_inc;
        source_poses[i] = pose0.compose(inc, H_pose_0_a, H_pose_inc); // T0 ∘ exp(tξ)

        gtsam::Matrix6 H_pose_delta = H_pose_inc * H_inc_vel * t * H_vel_delta;
        pose_derivatives_t0[i] = H_pose_0_a + H_pose_delta * H_delta_0;
        pose_derivatives_t1[i] = H_pose_delta * H_delta_1;
    }
}
```

## 雅可比传播的链式法则

连续时间因子的核心挑战在于：误差残差对点 $i$ 的贡献需要传播到 $T_{t0}$ 和 $T_{t1}$ 两个变量。

### CT_ICP 的雅可比链

对每个 source 点 $i$ 的时间索引为 $k = \text{time\_indices}[i]$：

1. 从值提取：$T_{t_k}$、雅可比 $H_{t_k}^{t0} = \frac{\partial T_{t_k}}{\partial T_{t0}}$、$H_{t_k}^{t1} = \frac{\partial T_{t_k}}{\partial T_{t1}}$
2. 变换源点：$\tilde{p}_i = T_{t_k} \cdot p_i$，雅可比 $\frac{\partial \tilde{p}_i}{\partial T_{t_k}}$
3. 残差：$r_i = \tilde{p}_i - q_j$ （point-to-point）或 $e_i = n_j \cdot r_i$（point-to-plane）
4. 链式传播：$H_i^{t0} = \frac{\partial e_i}{\partial \tilde{p}_i} \cdot \frac{\partial \tilde{p}_i}{\partial T_{t_k}} \cdot H_{t_k}^{t0}$

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L141-L181`：

```cpp
const auto perpoint_task = [&](int i, ...) {
    const int time_index = time_indices[i];
    const auto& pose = source_poses[time_index];
    const auto& H_pose_0 = pose_derivatives_t0[time_index];  // ∂T_{tk}/∂T0
    const auto& H_pose_1 = pose_derivatives_t1[time_index];  // ∂T_{tk}/∂T1

    const auto& source_pt = frame::point(*source, i);
    const auto& target_pt = frame::point(*target, target_index);
    const auto& target_normal = frame::normal(*target, target_index);

    gtsam::Matrix36 H_transed_pose;
    gtsam::Point3 transed_source_pt = pose.transformFrom(source_pt.head<3>(), H_transed_pose);
    gtsam::Point3 residual = transed_source_pt - target_pt.head<3>();

    gtsam::Matrix13 H_error_transed;
    double error = gtsam::dot(residual, target_normal.head<3>(), H_error_transed);

    gtsam::Matrix16 H_error_pose = H_error_transed * H_transed_pose;  // ∂e/∂T_{tk}
    gtsam::Matrix16 H_0 = H_error_pose * H_pose_0;                    // ∂e/∂T0
    gtsam::Matrix16 H_1 = H_error_pose * H_pose_1;                    // ∂e/∂T1

    *H_00 += H_0.transpose() * H_0;
    *H_11 += H_1.transpose() * H_1;
    *H_01 += H_0.transpose() * H_1;
    *b_0 += H_0.transpose() * error;
    *b_1 += H_1.transpose() * error;
};
```

### 李代数导数的推导

`pose_derivatives_t0[i]` 和 `pose_derivatives_t1[i]` 的推导基于 SE(3) 上的 BCH 公式。

设 $\xi = \log(T_{t0}^{-1} T_{t1})$，$T_{t_k} = T_{t0} \cdot \exp(t_k \xi)$。使用 GTSAM 提供的伴随性质：

$$\frac{\partial T_{t_k}}{\partial T_{t0}} = \text{Ad}_{T_{t0}} \cdot \frac{\partial \exp(t_k \xi)}{\partial \xi} \cdot \frac{\partial \xi}{\partial T_{t0}} + I$$

其中 $\frac{\partial \xi}{\partial T_{t0}}$ 来自 $\log(T_{t0}^{-1} T_{t1})$ 对左操作数 $T_{t0}$ 的导数；$\frac{\partial \exp(t_k \xi)}{\partial \xi}$ 由 `GTSAM::Pose3::Expmap()` 提供。

`pose_derivatives_t1[i]` 仅通过 $\xi$ 链传播（$T_{t1}$ 不直接出现在组合的左端），即 `H_pose_delta * H_delta_1`。

## CT-GICP：连续时间 + D2D 匹配

`IntegratedCT_GICPFactor` 在 CT-ICP 基础上使用 GICP 的分布到分布（D2D）马氏距离代价。

### CT-GICP 残差与雅可比

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_gicp_factor_impl.hpp:L103-L168`：

```cpp
// 残差用马氏距离
const Eigen::Vector4d transed_source_pt = pose * source_pt;
const Eigen::Vector4d residual = transed_source_pt - target_pt;
const double error = residual.transpose() * mahalanobis[i] * residual;

// 雅可比传播
gtsam::Matrix46 H_transed_pose = gtsam::Matrix46::Zero();
H_transed_pose.block<3,3>(0,0) = pose.linear() * -gtsam::SO3::Hat(source_pt.head<3>());
H_transed_pose.block<3,3>(0,3) = pose.linear();
const Eigen::Vector4d transed_source_pt = pose * source_pt;

const gtsam::Matrix46 H_residual_pose = H_transed_pose;
const gtsam::Matrix46 H_0 = H_residual_pose * H_pose_0;
const gtsam::Matrix46 H_1 = H_residual_pose * H_pose_1;
```

CT-GICP 的 `update_correspondences()` 额外计算每点对 `(source_i, target_j)` 的马氏距离矩阵：

$$\Sigma_{ij} = C_j + T_{t_k} C_i T_{t_k}^T, \quad M_i = (\Sigma_{ij})_{\text{3x3}}^{-1}$$

缓存为 `mahalanobis[i]`。

### CT-GICP 的协方差要求

- Target 点云必须有协方差（`has_covs()` 为 true）
- Source 点云必须有协方差（`has_covs()` 为 true）
- 实践中最常见的 target 类型是 `iVoxCovarianceEstimation`（在线估计逐点协方差的增量体素地图）

## CT-ICP 因子的错误检查

构造时对输入点云做严格检查（`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L31-L38`）：

```cpp
if (!frame::has_points(*target)) {
    abort();  // target 必须有 points
}
if (!frame::has_points(*source) || !frame::has_times(*source)) {
    abort();  // source 必须有 points 和 times
}
```

`linearize()` 时额外检查法向量（`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L133-L136`）：

```cpp
if (!frame::has_normals(*target)) {
    abort();  // point-to-plane 需要 target 法向量
}
```

## 与离散时间因子的对比

| 维度 | 离散时间因子 (IntegratedICPFactor) | 连续时间因子 (IntegratedCT_ICPFactor) |
|------|-----------------------------------|--------------------------------------|
| 基类 | `IntegratedMatchingCostFactor` | `gtsam::NonlinearFactor` |
| 变量数 | 1 (unary) 或 2 (binary) | 2 (t0, t1 同帧起止) |
| 相对变换 | `calc_delta(values)` → 单个 `delta` | `update_poses(values)` → `source_poses[t]` 数组 |
| 数据关联更新接口 | `update_correspondences(delta)` | `update_correspondences()` 内部调用 `update_poses()` |
| 残差接口 | `evaluate(delta, ...)` | 内嵌在 `linearize()` 的 lambda 中 |
| 雅可比链 | 单层 Δ→残差 | 双层：$T_{t_k} \to T_{t0}, T_{t1}$ |
| 运动畸变 | 需外部 deskew | 因子内部处理（优化过程中自动适应） |
| 变量自由度 | 6 per pose | 12 (2 × 6) per scan pair |
| 计算开销 | 较低（单次线性化 1×KNN） | 较高（每帧额外插值 M 次） |
| 因子图维度 | 2-key 或 1-key | 2-key（t0 + t1） |
| 适用场景 | scan-to-map, scan-to-scan | scan-to-map 且扫描帧内运动显著 |

## deskewed_source_points() 应用

连续时间因子提供了 `deskewed_source_points(values, local)` 方法（`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L291-L308`），将每点按照插值位姿变换到统一坐标系：

```cpp
std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local) {
    update_poses(values);
    if (local) {
        for (auto& pose : source_poses) {
            pose = values.at<gtsam::Pose3>(keys_[0]).inverse() * pose;
        }
    }
    std::vector<Eigen::Vector4d> deskewed(frame::size(*source));
    for (int i = 0; i < frame::size(*source); i++) {
        const int time_index = time_indices[i];
        deskewed[i] = source_poses[time_index].matrix() * source_point[i];
    }
    return deskewed;
}
```

`local=false` 时返回世界坐标系下的 deskew 点云；`local=true` 时返回以 $T_{t0}$ 为参考系的相对坐标（可用于可视化或后续处理）。

## Agent 实现提示

### 适用场景

- LiDAR 扫描频率低（< 20 Hz）且载体运动快，帧内运动畸变不可忽略
- 不需要预先 IMU deskew 的端到端因子图优化前端
- 构建弹性连续时间轨迹表示时，将扫描帧视为一段 SE(3) 直线段
- 适合：CT-ICP 风格的 LiDAR 里程计、动态场景下的 scan-to-map 配准、与连续时间 IMU 预积分因子联合优化
- 不适合：扫描频率高（≥ 100 Hz）且畸变小（使用离散时间因子即可）、非刚体运动（如水下/软体机器人）

### 输入输出契约

- **输入**：source 点云（必须带 `times[]` 时间戳，归一化前）、target 点云（必须带 `points` 和 `normals`）、最近邻搜索结构（可选）、最大对应距离
- **source 时间戳格式**：`times[i]` 为相对于扫描首个点的秒数（单调非递减），在构造时自动归一化到 $[0, 1]$
- **变量**：`source_t0_key`（扫描起点位姿）、`source_t1_key`（扫描终点位姿），两个都是 $\text{Pose3} \in SE(3)$
- **dim()**：6（与双元因子一致，GTSAM HessianFactor 接受 -dimensional 的因子）
- **输出**：`gtsam::HessianFactor`（$6\times6$ 块 × 2），雅可比通过中间 $T_{t_k}$ 链式传播

### 实现骨架（伪代码）

```text
class IntegratedCT_ICPFactor(keys=[t0_key, t1_key], target, source, target_tree?):
    data:
        time_table: vector<double>     // M 个去重归一化时间 [0..1]
        time_indices: vector<int>      // N 个点 → M 个时间索引
        source_poses: mutable vector<Pose3>     // M 个插值位姿
        pose_derivatives_t0: mutable vector<Matrix6>  // ∂T_{tk}/∂T0
        pose_derivatives_t1: mutable vector<Matrix6>  // ∂T_{tk}/∂T1
        correspondences: mutable vector<long>  // N 个对应关系

    constructor(t0_key, t1_key, target, source, target_tree?):
        validate(source has points AND times)
        validate(target has points AND normals)
        // 构建归一化时间表
        for each point i:
            t = source.time[i]
            if t - last > 1e-3:
                time_table.push_back(t)
            time_indices.push_back(time_table.size() - 1)
        for each t in time_table:
            t /= max(1e-9, time_table.back())  // normalize to [0, 1]
        this.target_tree = target_tree || new KdTree2(target)

    // 每帧线性化前调用：插值中间位姿
    update_poses(values):
        T0 = values[keys[0]];  T1 = values[keys[1]]
        Δ = T0.between(T1)     // T0^{-1} * T1
        ξ = Log(Δ)             // se(3) velocity
        resize(source_poses, pose_derivatives_t0, pose_derivatives_t1, M)
        for each k in 0..M-1:
            inc = Exp(time_table[k] * ξ)   // se(3) → SE(3)
            source_poses[k] = T0 * inc
            // chain rule: ∂poses[k]/∂T0, ∂poses[k]/∂T1
            pose_derivatives_t0[k] = H_compose0 + H_inc * t_k * H_log * H_between0
            pose_derivatives_t1[k] = H_inc * t_k * H_log * H_between1

    // 更新近邻对应关系
    update_correspondences():
        correspondences.resize(N)
        for each source point i:
            pt_transformed = source_poses[time_indices[i]] * source.point[i]
            k_idx, k_dist = target_tree.knn_search(pt_transformed, 1, max_dist)
            correspondences[i] = (k_dist < max_dist) ? k_idx : -1

    linearize(values):
        update_poses(values)
        update_correspondences()
        H_00, H_01, H_11, b_0, b_1 = zero
        error = 0
        for each source point i:
            j = correspondences[i]; if j < 0: continue
            k = time_indices[i]
            H_tk = pose.transformFrom(source.point[i])        // ∂(T*p)/∂T
            r = (pose * source.point[i]) - target.point[j]     // 3D residual
            e = target.normal[j] · r                           // point-to-plane scalar
            H_e_pose = H_normal_residual * H_tk                // ∂e/∂T_{tk}
            H_0 = H_e_pose * pose_derivatives_t0[k]            // ∂e/∂T0
            H_1 = H_e_pose * pose_derivatives_t1[k]            // ∂e/∂T1
            H_00 += H_0^T * H_0;  H_11 += H_1^T * H_1
            H_01 += H_0^T * H_1;  b_0 += H_0^T * e;  b_1 += H_1^T * e
            error += e^2
        return HessianFactor(t0_key, t1_key, H_00, H_01, -b_0, H_11, -b_1, error)
```

### 关键源码片段

1. **CT-ICP 因子类声明** — `raw/codes/gtsam_points/include/gtsam_points/factors/integrated_ct_icp_factor.hpp:L20-L96`
2. **时间表构建与归一化** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L41-L55`
3. **位姿插值 update_poses()** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L202-L248`
4. **线性化：point-to-plane 雅可比传播** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L132-L199`
5. **CT-GICP 线性化：马氏距离雅可比** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_gicp_factor_impl.hpp:L103-L168`
6. **CT-GICP 数据关联与马氏矩阵** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_gicp_factor_impl.hpp:L171-L219`
7. **deskewed_source_points()** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L291-L308`

### 实现注意事项

- **时间基准约定**：`times[]` 必须相对于扫描起始时刻单调递增。如果使用 IMU 时间戳（绝对时间），需要在输入前做相对化处理。帧间时间 gap 通过因子图内相邻 CT-ICP 因子的变量链接处理（前一帧的 t1 通常等于下一帧的 t0）。
- **CT-ICP 的两个 key 不应在因子图中独立优化**：连续时间因子的变量对 `(t0, t1)` 应通过相邻帧的连接保证一致性。推荐使用 `t1_frame_k = t0_frame_{k+1}` 的图结构，形成连续的轨迹链。
- **错误处理的时间相关性**：如果 `time_table.size() == 1`（所有点时间相同），归一化后全为 0，退化为纯离散时间 ICP（但使用两个相同 key 没有意义——应回退到 `IntegratedICPFactor`）。
- **法向量要求**：CT-ICP 使用 point-to-plane 距离，要求 target 点云必须有法向量。CT-GICP 要求 target 和 source 均有协方差。构造时用 `abort()` 违规退出，不会静默继续。
- **与 IMU deskew 的对比**：CT-ICP 的插值是完全基于 SE(3) 直线假设的。在剧烈加速度变化的场景（如急转弯），IMU 角速度积分 deskew 可能更精确。两种方法可以共存：IMU deskew 作为初值，CT-ICP 在优化中微调起止位姿。
- **Hessian 填充**：连续时间因子的 Hessian 在 `linearize()` 末尾通过 `gtsam::HessianFactor(keys_[0], keys_[1], H_00, H_01, -b_0, H_11, -b_1, error)` 填充，这与离散二元因子一致。关键是交叉项 `H_01` 捕获了 t0 和 t1 之间的运动耦合。

### 源码检索锚点

- CT-ICP 因子头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_ct_icp_factor.hpp`
- CT-GICP 因子头文件：`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_ct_gicp_factor.hpp`
- CT-ICP 实现（模板实例化 + impl）：`raw/codes/gtsam_points/src/gtsam_points/factors/integrated_ct_icp_factor.cpp`
- CT-ICP 模板实现体：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp`
- CT-GICP 模板实现体：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_gicp_factor_impl.hpp`
- CT-GICP 实现（模板实例化）：`raw/codes/gtsam_points/src/gtsam_points/factors/integrated_ct_gicp_factor.cpp`
- iVox（常用 target 类型）：`raw/codes/gtsam_points/include/gtsam_points/ann/ivox.hpp`
- 扫描匹配并行归约：`raw/codes/gtsam_points/include/gtsam_points/factors/impl/scan_matching_reduction.hpp`

## 相关页面

- [[组件-gtsam_points]]
- [[方法-gtsam_points因子封装模式]]
- [[方法-gtsam_points体素加速]]
- [[方法-连续时间 ICP 因子]]
- [[概念-连续时间轨迹]]
- [[算法-CT-ICP]]
- [[方法-ICP变体族]]
- [[方法-IMU deskew]]
- [[方法-IntegratedMatchingCostFactor]]
- [[组件-GTSAM]]
- [[概念-因子图]]
