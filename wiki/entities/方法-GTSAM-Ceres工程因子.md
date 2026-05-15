---
type: entity
tags: [GTSAM, Ceres, 因子图, 工程因子, SLAM]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/gtsam_points
  - raw/codes/OB_GINS/src/factors
  - raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc
  - raw/codes/VINS-Fusion/vins_estimator/src/factor
---

# GTSAM / Ceres 工程因子

> 工程因子是 SLAM 后端优化中连接传感器观测与状态变量的核心抽象。本文涵盖激光配准、LiDAR BA、双目视觉 smart factor 和 GNSS 位置因子四类工程因子，对比 GTSAM 与 Ceres 两种主流后端的设计模式。

## 概述

### 因子图哲学：一切皆因子

在 GTSAM 因子图优化框架中，SLAM 问题被表达为二部图：**变量节点**（位姿、速度、bias、landmark）和**因子节点**（先验、里程计、匹配约束、重投影误差）。每个因子定义观测与估计状态之间的残差和雅可比，优化器最小化所有因子的加权平方误差：

$$\mathbf{X}^* = \arg\min_{\mathbf{X}} \sum_i \| \mathbf{e}_i(\mathbf{X}) \|_{\Sigma_i}^2$$

**工程因子**不同于教科书上的简单因子（如 `PriorFactor`、`BetweenFactor`），它们封装了复杂的传感器数据关联、特征级计算和多帧联合约束，是实际 SLAM 系统性能的关键。

### GTSAM vs Ceres 设计哲学对比

| 维度 | GTSAM | Ceres |
|------|-------|-------|
| **因子表达** | `NonlinearFactor` 子类，`error()` + `linearize()` | `CostFunction` 模板，`Evaluate()` |
| **噪声模型** | `NoiseModel` 绑定在因子内 | `LossFunction` 外挂；信息矩阵通过 `AddResidualBlock` |
| **优化器** | LM/GN/Dogleg + ISAM2 增量 | LM/Dogleg + 多种线性求解器 |
| **变量管理** | `Values`（key-value 映射） | `ParameterBlock` 数组（按索引） |
| **典型场景** | 因子图研究、VIO 后端（iSAM2） | 批量优化、滑动窗口 VINS |

### 工程因子的核心模式

1. **数据关联嵌入**：很多因子在 `linearize()` 时动态更新数据关联（如 ICP 对应关系），而非提前固定
2. **残差降维**：将高维传感器观测压缩为低维约束（如点云匹配 → 6 维 SE(3) 残差）
3. **Schur 补消元**：内部消去辅助变量减少图规模（如 smart factor 消去 landmark）
4. **鲁棒估计**：卡方粗差检验 + Huber/LossFunction 保护收敛
5. **杆臂与坐标系转换**：传感器测量必须转换到优化变量的参考系

## 变体对比表

| 因子类型 | 后端 | 残差维度 | 优化变量 | 数据关联 | 代表框架 | 特点 |
|----------|------|----------|----------|----------|----------|------|
| IntegratedMatchingCostFactor | GTSAM | 6 (SE(3)) | 1-2 个 Pose3 | ICP/GICP 点云匹配 | gtsam_points | 扫描配准基类；`update_correspondences()` + `evaluate()` 两个纯虚函数 |
| CT-ICP 因子 | GTSAM | 6 (x2) | 起止 2 个 Pose3 | slerp 插值 + ICP | gtsam_points | 帧内运动畸变内置消除 |
| PlaneEVMFactor | GTSAM | 6 (SE(3)) | 多帧 Pose3 | 面特征点云 | gtsam_points | 特征值最小化 $\lambda_0$ |
| EdgeEVMFactor | GTSAM | 6 (SE(3)) | 多帧 Pose3 | 线特征点云 | gtsam_points | 特征值最小化 $\lambda_0 + \lambda_1$ |
| LsqBundleAdjustmentFactor | GTSAM | 6 (SE(3)) | 多帧 Pose3 | 面/线特征 | gtsam_points | 代价与帧数有关，与点数无关 |
| SmartStereoFactor | GTSAM | 2 × k (k 帧观测) | k 个 Pose3（无显式 landmark） | KLT/NCC 特征跟踪 | Kimera-VIO | 内部三角化+消元；需要双目标定 |
| GNSS 位置残差因子 | Ceres | 3 (NED xyz) | 1 个 7 维 pose (+杆臂) | 时间戳对齐 | IC-GVINS, OB_GINS | RTK 绝对位置锚定；两轮优化+卡方粗差检验 |

## 各因子详解

### 1. IntegratedMatchingCostFactor — 扫描配准因子基类

#### 数学形式

二元情形给定目标帧位姿 $T_t$ 和源帧位姿 $T_s$，相对变换 $\Delta = T_t^{-1} T_s$：

$$e(T_t, T_s) = \text{evaluate}(\Delta)$$

$$\begin{aligned}
\frac{\partial e}{\partial T_t} &= -\mathbf{J}_\Delta \cdot T_t^{-1} \\
\frac{\partial e}{\partial T_s} &= \phantom{-}\mathbf{J}_\Delta \cdot T_t^{-1}
\end{aligned}$$

其中 $\mathbf{J}_\Delta$ 来自子类的 `evaluate()` 返回值（Hessian 块 + 误差向量），维度固定为 6。

#### 残差/雅可比结构

```
error() 流程:
  1. calc_delta(values)       → Isometry3d delta (target到source的相对位姿)
  2. update_correspondences(delta) → 建立点云匹配关系
  3. evaluate(delta, &H_t, &H_s, &H_ts, &b_t, &b_s) → 返回标量误差

linearize() 流程:
  等同于 error()，但通过链式法则将 evaluate 返回的 Hessian/Jacobian
  传播到目标/源位姿的 Lie 代数上，生成 GaussianFactor
```

- 残差维度：6（SE(3) 扰动）
- 雅可比维度：6 × (12 × keys_)  → 6×12（二元）或 6×6（一元固定目标）
- `evaluate()` 返回 `H_t(6×6), H_s(6×6), H_ts(6×6), b_t(6×1), b_s(6×1)` 传递给 `linearize()`

#### 工程注意事项

- **二元 vs 一元**：构造函数通过 `target_key + source_key`（二元）或 `fixed_target_pose + source_key`（一元）区分
- **非线程安全**：`update_correspondences()` 可能修改 mutable 成员，不能对同一因子并行 `linearize()`
- **子类扩展**：继承后只需实现两个纯虚函数（约 200 行代码），包括 `IntegratedICPFactor`、`IntegratedGICPFactor` 等

#### 源码锚点

`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp:L19-L89`

### 2. LiDAR 捆集调整因子 — EVM 与 LSQ

#### 2.1 PlaneEVMFactor — 平面特征值最小化

给定同一平面上来自 k 帧的 n 个点 $\{\mathbf{p}_i\}_{i=1}^{n}$，通过优化 k 个位姿使得所有点的协方差矩阵最小特征值 $\lambda_0$ 最小化：

$$e(\{T_j\}) = \lambda_0\left(\operatorname{Cov}(\{\mathbf{p}^w_i\})\right)$$

其中 $\mathbf{p}^w_i = T_{j(i)} \cdot \mathbf{p}^l_i$ 是世界坐标系下的点。

**链式法则**（`calc_pose_derivatives`）：

$$\frac{\partial e}{\partial T_j} = \frac{\partial \lambda_0}{\partial \mathbf{p}^w} \cdot \frac{\partial \mathbf{p}^w}{\partial T_j}$$

$$\frac{\partial \mathbf{p}^w}{\partial \xi} = \begin{bmatrix} -[\mathbf{p}^w]_\times & I_3 \end{bmatrix}_{3\times6}$$

其中 $[\mathbf{p}^w]_\times$ 是 hat operator。

#### 2.2 EdgeEVMFactor — 边缘特征值最小化

$$e(\{T_j\}) = \lambda_0 + \lambda_1$$

最小化协方差矩阵的最小两个特征值之和，促使所有点共线。

#### 2.3 LsqBundleAdjustmentFactor — 与点数无关的 BA

基于 EVM+EF 最优条件推导的 LSQ 形式：每个平面特征先将同帧内点聚合成局部分布（均值 + 协方差 + 法向量），然后通过全局分布（所有帧聚合）建立帧间约束。代价仅与帧数 $k$ 相关，与点数 $n$ 无关：

$$e(\{T_j\}) \sim \mathcal{O}(k) \quad \text{而非} \quad \mathcal{O}(n)$$

代价是对初始值更敏感（全局法向量不在优化变量中）。

#### 残差结构

```
EVM factor linearize() 流程:
  1. 对每个点变换到世界坐标系：p_i^w = T_j * p_i^l
  2. calc_eigenvalue(transed_points, &H, &J) → 计算特征值 + 点级 Hessian/Jacobian
  3. calc_pose_derivatives(transed_points) → 点级导数传播到位姿级 6×6 块
  4. compose_factor(H_pose, J_pose, error) → 生成 HessianFactor
```

- 每个点对所属位姿贡献 3×6 的雅可比块，拼成 3n × 6k 的导数矩阵 D
- 协方差矩阵级 Hessian（3n × 3n）通过 $D^T H D$ 传播到位姿 6k × 6k

#### 工程注意事项

- 计算量随点数增加 → 建议提前在同帧内聚合点（[Liu 2021]）
- EVM 避免了启发式平面拟合阈值，但协方差估计需要足够多样本
- LSQ 比 EVM 更快但需要更好的初始值（`LsqBundleAdjustmentFactor: public BundleAdjustmentFactorBase`）
- 适合关键帧或局部窗口优化，不适合全量点云

#### 源码锚点

- `raw/codes/gtsam_points/include/gtsam_points/factors/bundle_adjustment_factor_evm.hpp:L26-L69`
- `raw/codes/gtsam_points/src/gtsam_points/factors/bundle_adjustment_factor_evm.cpp:L47-L93`
- `raw/codes/gtsam_points/include/gtsam_points/factors/bundle_adjustment_factor_lsq.hpp:L22-L52`

### 3. SmartStereoFactor — 双目智能视觉因子

#### 数学形式

GTSAM `SmartStereoProjectionPoseFactor` 收集路标 $l$ 在 $k$ 个双目关键帧中的观测 $\{(u_{Li}, u_{Ri}, v_i)\}_{i=1}^{k}$，内部完成三角化并通过 Schur 补消去 $l$ 的坐标，最终输出只约束 k 个位姿 $T_1, \dots, T_k$ 的因子。

每个观测的重投影误差（单位球面 + 水平视差，pinhole stereo projection）：

$$\mathbf{e}_i = \pi_{stereo}(T_i^{-1} \cdot \mathbf{p}_l) - \begin{pmatrix} u_{Li} \\ u_{Ri} \\ v_i \end{pmatrix}$$

通过 Schur 补，优化问题从优化 $\{T_i\}$ 和 $\mathbf{p}_l$ 压缩为仅优化 $\{T_i\}$：

$$\min_{T_i} \min_{\mathbf{p}_l} \sum_i \| \mathbf{e}_i(\mathbf{p}_l, T_i) \|^2 = \min_{T_i} \| \tilde{\mathbf{e}}(\{T_i\}) \|^2$$

#### 雅可比结构

```python
# 外部因子图视角：只暴露位姿变量
SmartStereoFactor:
  variables: {Pose(t1), Pose(t2), ..., Pose(tk)}  # k个位姿
  NO landmark variable exposed
  
  # 内部：
  - 维护 point (triangulated position)
  - 维护 measured: [(uL, uR, v, camera_key), ...]
  - linearize() 时内部三角化 → 计算重投影雅可比 → Schur 补消元 → 返回位姿间的 HessianFactor
```

#### 关键参数

| 参数 | 作用 | 影响 |
|------|------|------|
| `rank_tolerance` | 判断三角化约束退化 | 过低 → 接受退化点；过高 → 拒绝有效点 |
| `landmark_distance_threshold` | 过滤远点 | 避免数值不稳定 |
| `retriangulation_threshold` | 何时重新三角化 | 位姿更新后重新估计深度 |
| `outlier_rejection` | 观测级外点拒绝 | 重投影误差超出阈值时剔除 |

#### 工程注意事项

- **Clone-and-add 模式**：更新已有 smart factor 时 clone 旧因子再添加新观测，不直接修改已线性化的因子
- **有效性检查链**：`factor->point().valid()` → `!isDegenerate()` → `!isFarPoint()` → `!isOutlier()` → `!isPointBehindCamera()` → 观测数是否足够
- **不要混用普通 BA**：同一 track 上不要同时创建显式 landmark 变量
- **双目优势**：双目提供更稳定的初始深度，降低三角化退化风险

#### 源码锚点

- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L478-L504` — `addLandmarkToGraph`
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L578-L604` — `updateExistingSmartFactor`
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L963-L1005` — `isSmartFactor3dPointGood`

### 4. GNSS 位置残差因子 — Ceres 绝对位置约束

#### 数学形式

GNSS 测得的天线位置（NED 坐标系）作为绝对位置约束。考虑 IMU 到天线的杆臂 $l_{ant}$：

$$e = \mathbf{W} \cdot \big( \mathbf{p}_{IMU} + \mathbf{R}(q) \cdot l_{ant} - \mathbf{p}_{GNSS} \big)$$

其中 $\mathbf{W} = \operatorname{diag}(1/\sigma_N, 1/\sigma_E, 1/\sigma_D)$ 是各向异性权重（GNSS 精度信息）。

#### 雅可比结构

Ceres `SizedCostFunction<3, 7>` 一个 7 维参数块（`p + q` 或 `p + so3`），残差维度 3：

$$\frac{\partial e}{\partial p} = \mathbf{W} \cdot I_3$$

$$\frac{\partial e}{\partial \theta} = -\mathbf{W} \cdot \mathbf{R}(q) \cdot [l_{ant}]_\times$$

#### 两轮优化 + 粗差剔除流程

```
Round 1 (25% iterations):
  solver: DENSE_SCHUR / SPARSE_NORMAL_CHOLESKY
  loss:   HuberLoss(1.0)  ← 保护收敛
  result: 收敛到粗解

Outlier detection (after Round 1):
  GNSS: Chi² test with df=3, threshold=7.815 (p=0.05)
        → 超标 → reweight 标准差 (不要完全移除)
  Vision: Chi² test with df=2, threshold=5.991
          → 超标 → 移除残差块

Round 2 (75% iterations):
  solver: DENSE_SCHUR / SPARSE_NORMAL_CHOLESKY
  loss:   Remove GNSS HuberLoss (精调)
        视觉粗差已剔除
  result: 精细解
```

#### 工程注意事项

- **杆臂必须显式补偿**：天线相位中心到 IMU 原点的杆臂 $l_{ant}$ 错误会表现为姿态依赖的位置误差
- **RTK fix/float 区分**：根据解算状态动态调整噪声协方差（fix → 小协方差；float/单点 → 大协方差 + 鲁棒核）
- **卡方检验前提**：残差维度、协方差和噪声建模必须合理；多径和非高斯误差可能导致漏检
- **GTSAM 等价**：`GPSFactor`（无杆臂）或 `PriorFactor<Pose3>` + 杆臂手动转换
- **坐标系转换**：所有 GNSS 测量必须统一到同一局部 ENU/NED 原点

#### 实现对比

| 实现 | 后端 | GNSS 数据 | 杆臂 | 粗差剔除 |
|------|------|-----------|------|----------|
| IC-GVINS | Ceres, DENSE_SCHUR | RTK 位置 | 显式 | 两轮 + Chi² |
| OB_GINS | Ceres, SPARSE_NORMAL_CHOLESKY | RTK 位置 | 显式 | 两轮 + Chi² |
| VINS-Fusion | Ceres | GPS 松耦合 | 无 | Huber 单轮 |
| LIO-SAM | GTSAM, ISAM2 | GPS 位置先验 | 无 | M-estimator |

## Agent 实现提示

### 适用场景

1. **扫描配准因子**：需要将 ICP/GICP/VGICP 匹配作为 GTSAM 因子接入因子图时，继承 `IntegratedMatchingCostFactor`
2. **LiDAR BA 因子**：多帧 LiDAR 关键帧的联合几何优化，用共面/共线特征约束位姿
3. **Smart 视觉因子**：双目 VIO 中需要消去 landmark 变量以减少图规模
4. **GNSS 位置因子**：将 RTK/GNSS 绝对位置作为全局锚定约束接入 Ceres 优化

### 输入输出契约

- **IntegratedMatchingCostFactor**：输入 = `delta (Isometry3d, T_target_source)`，输出 = `H_target(6×6), H_source(6×6), H_target_source(6×6), b_target(6×1), b_source(6×1)`
- **LiDAR BA**：输入 = 同平面/线上的多帧点 + 对应位姿 key，输出 = `GaussianFactor` (6 per pose)
- **SmartStereoFactor**：输入 = `StereoPoint2(uL, uR, v)` × k 帧 + 双目标定，输出 = 位姿因子（无 landmark 变量）
- **GNSS 位置**：输入 = 7 维 pose (p + q) + 杆臂 + GNSS 位置 + $\Sigma_{NED}$，输出 = 3 维残差

### 实现骨架（伪代码）

```python
# === 1. 扫描配准因子基类 ===
class MyScanMatchingFactor(IntegratedMatchingCostFactor):
    def update_correspondences(self, delta):
        # 用 delta 变换源点云，找最近邻作为匹配对
        self.correspondences = find_nearest_neighbors(
            self.target_cloud, self.source_cloud, delta
        )
    
    def evaluate(self, delta, H_t, H_s, H_ts, b_t, b_s):
        # 计算 point-to-plane / point-to-point 残差和雅可比
        error = 0.0
        for (pi, pj) in self.correspondences:
            residual = normal.dot(delta * pi - pj)
            J_i = normal  # (1×6) pose Jacobian for pi's frame
            H_t += J_i.T * weight * J_i  # accumulate
            b_t += J_i.T * weight * residual
            error += weight * residual**2
        return error

# === 2. LiDAR BA 因子 ===
class LiDARBAFactor:
    def __init__(self, noise_model):
        self.factors = { "plane": [], "edge": [] }
    
    def add_plane_feature(self, points_per_frame, frame_keys, noise):
        factor = PlaneEVMFactor()
        for key, pts in zip(frame_keys, points_per_frame):
            for p in pts:
                factor.add(p, key)
        self.factors["plane"].append((factor, noise))
    
    def optimize(self, initial_values):
        graph = NonlinearFactorGraph()
        for factor, noise in self.factors["plane"]:
            graph.add(noise, factor)
        return LM(graph, initial_values).optimize()

# === 3. Smart 视觉因子 ===
def manage_smart_factors(track, graph_state, params):
    if track.landmark_id not in graph_state.old_smart_factors:
        # 首次创建
        factor = SmartStereoFactor(params.noise, params.param, body_P_cam)
        for frame_id, stereo_pt in track.observations:
            factor.add(stereo_pt, Symbol("x", frame_id), stereo_cal)
        graph_state.new_factors[track.landmark_id] = factor
    else:
        # 克隆 + 追加观测
        old = graph_state.old_smart_factors[track.landmark_id]
        factor = clone(old)
        factor.add(track.latest_observation, Symbol("x", track.latest_frame), stereo_cal)
        
        # 检查因子质量
        if not is_point_valid(factor):
            return None  # 等待更多观测或降级
        graph_state.new_factors[track.landmark_id] = factor

# === 4. GNSS 位置因子 ===
class CerealGNSSPositionFactor(SizedCostFunction<3, 7>):
    def __init__(self, p_GNSS_NED, lever_arm, weight_matrix_W):
        SizedCostFunction.__init__(self)
        self.p_GNSS = p_GNSS_NED
        self.l = lever_arm      # 天线在IMU系坐标
        self.W = weight_matrix  # diag(1/σN, 1/σE, 1/σD)
    
    def Evaluate(self, parameters, residuals, jacobians):
        # parameters[0] = [p_x, p_y, p_z, q_w, q_x, q_y, q_z]
        p_imu = parameters[0:3]
        q = parameters[3:7]
        R = quaternion_to_rotation(q)
        
        # 残差
        residual_raw = p_imu + R @ self.l - self.p_GNSS
        residuals[:] = self.W @ residual_raw
        
        if jacobians:
            # dp/dp = I_3
            jacobians[0][:, 0:3] = self.W
            # dp/dθ = -W * R(q) * [l]x
            jacobians[0][:, 3:6] = -self.W @ R @ skew_symmetric(self.l)
```

### 关键源码片段

`raw/codes/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp:L47-L83`

```cpp
virtual double error(const gtsam::Values& values) const override {
  const Eigen::Isometry3d delta = calc_delta(values);
  update_correspondences(delta);
  return evaluate(delta);  // 仅需要 scalar error
}

virtual gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override {
  const Eigen::Isometry3d delta = calc_delta(values);
  update_correspondences(delta);
  Eigen::Matrix<double, 6, 6> H_target, H_source, H_target_source;
  Eigen::Matrix<double, 6, 1> b_target, b_source;
  double error = evaluate(delta, &H_target, &H_source, &H_target_source, &b_target, &b_source);
  // 通过链式法则将 H_target/source 传播到位姿 Lie 代数空间
  return compose_gaussian_factor(H_target, H_source, H_target_source, b_target, b_source, error);
}
```

`raw/codes/gtsam_points/src/gtsam_points/factors/bundle_adjustment_factor_evm.cpp:L47-L69`

```cpp
template <int k>
double EVMBundleAdjustmentFactorBase::calc_eigenvalue(
    const std::vector<Eigen::Vector3d>& transed_points,
    Eigen::MatrixXd* H, Eigen::MatrixXd* J) const {
  BALMFeature feature(transed_points);  // 计算协方差矩阵 + 特征分解
  if (H == nullptr || J == nullptr) {
    return feature.eigenvalues[k];       // 快速误差计算
  }
  *H = Eigen::MatrixXd::Zero(points.size() * 3, points.size() * 3);  // 3n × 3n
  *J = Eigen::MatrixXd(1, points.size() * 3);                          // 1  × 3n
  for (int i = 0; i < points.size(); i++) {
    for (int j = i; j < points.size(); j++) {
      H->block<3, 3>(i * 3, j * 3) = feature.Hij<k>(transed_points[i], transed_points[j], i == j);
    }
  }
  for (int i = 0; i < points.size(); i++) {
    J->block<1, 3>(0, i * 3) = feature.Ji<k>(transed_points[i]);
  }
  return feature.eigenvalues[k];
}
```

### 实现注意事项

1. **数据关联时机**：`IntegratedMatchingCostFactor` 在每次 `linearize()` 时重新建立对应关系；子类应在 `update_correspondences()` 中封装，不要在 factor 外部预计算静态匹配。
2. **EVM factor 计算量**：Hessian 为 3n × 3n（n 为点数），n > 100 时建议提前帧内聚合；首选 `LsqBundleAdjustmentFactor` 以降低规模。
3. **Smart factor 的 clone-and-add**：永远不要直接调用旧智因子的 `add()` —— 旧因子可能已在图中线性化；clone 后操作新对象再插入图。
4. **GNSS 卡方检验**：df=3 对应 3 个位置维度；阈值 7.815 基于 p=0.05，仅适用于建模良好的协方差；GNSS 多径时考虑提高阈值或动态调整噪声。
5. **所有位置因子必须处理坐标系**：GNSS 坐标准换到局部 ENU/NED；激光扫描配准因子分目标/源两帧；LiDAR BA 需要所有点在世界坐标系下聚合。
6. **鲁棒核位置**：GTSAM → `noiseModel::Robust` 绑定在 factor 上；Ceres → `LossFunction` 通过 `AddResidualBlock` 外挂。两轮优化模式（Huber → Chi² 粗差 → 无核精调）是工程已验证的最佳实践。

### 源码检索锚点

- `raw/codes/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp` — IMCF 基类
- `raw/codes/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp` — error/linearize 流水线
- `raw/codes/gtsam_points/include/gtsam_points/factors/bundle_adjustment_factor_evm.hpp` — EVM/Plane/Edge factor
- `raw/codes/gtsam_points/src/gtsam_points/factors/bundle_adjustment_factor_evm.cpp:L43-L113` — calc_eigenvalue + compose_factor
- `raw/codes/gtsam_points/include/gtsam_points/factors/bundle_adjustment_factor_lsq.hpp` — LSQ BA factor
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp` — addLandmarkToGraph:478, updateExistingSmartFactor:578, isSmartFactor3dPointGood:963
- `raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h` — SmartStereoFactor type aliases

## 相关页面

### 因子设计
- [[方法-IntegratedMatchingCostFactor]]
- [[方法-LiDAR 捆集调整因子]]
- [[方法-SmartStereoFactor]]
- [[方法-GNSS 位置残差因子]]
- [[架构-GNSS 位置因子设计模式]]

### 后端框架
- [[组件-GTSAM]]
- [[组件-gtsam_points]]
- [[组件-Ceres-Solver]]

### 优化策略
- [[方法-Ceres 两轮优化 + 粗差剔除]]
- [[方法-Ceres解析雅可比CostFunction]]
- [[概念-Schur补与边缘化]]

### 理论基础
- [[概念-因子图]]
- [[概念-位姿图优化]]
- [[概念-连续时间轨迹]]
- [[概念-三角化与深度估计]]

### 算法与系统
- [[算法-Kimera-VIO]]
- [[算法-IC-GVINS]]
- [[算法-OB_GINS]]
- [[算法-CT-ICP]]
- [[方法-连续时间 ICP 因子]]
