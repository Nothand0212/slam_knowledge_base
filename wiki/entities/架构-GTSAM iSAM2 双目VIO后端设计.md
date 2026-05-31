---
tags: [架构, GTSAM, iSAM2, 双目VIO, 因子图, 后端, SmartFactor, GenericStereoFactor, IncrementalFixedLagSmoother, 边缘化, 异常值剔除]
created: 2026-06-01
updated: 2026-06-01
type: entity
sources:
  - raw/codes/Kimera-VIO/src/backend/VioBackend.cpp
  - raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackendParams.h
  - raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h
  - raw/codes/Kimera-VIO/src/backend/VioBackendParams.cpp
  - raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.h
  - raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp
  - raw/codes/dm-vio/src/GTSAMIntegration/FEJValues.h
  - raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.cpp
  - raw/codes/ORB_SLAM3/src/Optimizer.cc
  - raw/codes/ORB_SLAM3/include/G2oTypes.h
---

# GTSAM iSAM2 双目 VIO 后端设计

> 面向双目 VIO 的 GTSAM iSAM2 因子图后端完整设计：状态向量、因子类型与噪声模型、因子注入顺序、iSAM2 参数配置、边缘化策略、异常值剔除、SmartFactor 到 GenericStereoFactor 的迁移路径、失败模式与恢复。

---

## 一、完整状态向量规范

### 1.1 每关键帧的状态变量

每个关键帧在 GTSAM `Values` 中有 **3 个变量**，使用 GTSAM Symbol 索引：

| 变量 | GTSAM 类型 | Symbol 字符 | Symbol 索引 | 维度 | 含义 |
|------|-----------|-------------|-------------|------|------|
| 位姿 | `Pose3` | `x` | KF id | 6 (SE3) | 机体系在世界系中的 6-DOF 位姿 |
| 速度 | `Vector3` | `v` | KF id | 3 | 机体系在世界系中的线速度 |
| IMU 偏置 | `imuBias::ConstantBias` | `b` | KF id | 6 (3 acc + 3 gyro) | 加速度计和陀螺仪偏置 |

**源码锚点** (`VioBackend-definitions.h:L47-L51`):
```cpp
static constexpr SymbolChar kPoseSymbolChar = 'x';       // 位姿
static constexpr SymbolChar kVelocitySymbolChar = 'v';    // 速度
static constexpr SymbolChar kImuBiasSymbolChar = 'b';     // IMU偏置
```

### 1.2 路标变量

**设计选择**：系统分两个阶段管理路标：

| 阶段 | 路标表示 | GTSAM 类型 | Symbol 字符 | 何时使用 |
|------|---------|-----------|-------------|----------|
| **Smart 阶段**（过渡） | SmartFactor 隐式管理 | `SmartStereoProjectionPoseFactor` | 无显式变量 | 路标首次加入图，观测数 < N 帧 |
| **显式阶段**（目标） | 显式 3D 点 | `Point3` | `l` | 路标三角化质量达标后转为显式 |

**SmartFactor 内部三角化**：SmartFactor 收集同一路标在多个关键帧中的双目观测 `(uL, uR, v)`，内部完成三角化并消去路标。外部因子图不暴露路标变量。可通过 `factor->point()` 获取三角化结果。

### 1.3 常量参数

| 参数 | GTSAM 类型 | 说明 |
|------|-----------|------|
| 左相机内参 | `Cal3_S2Stereo` | 包含 fx, fy, cx, cy, baseline |
| 相机到机体外参 | `Pose3` (`body_P_cam`) | 固定变换，不参与优化 |

### 1.4 滑动窗口大小

- 默认窗口：**25 个关键帧**（`nr_states_ = 25`）
- Kimera-VIO Euroc 配置使用 25，代码默认 30
- 窗口大小由 `IncrementalFixedLagSmoother` 的 `lag_seconds` 参数管理，实际效果接近维持约 25 个 Pose3 + Velocity + Bias 变量组

---

## 二、因子类型与噪声模型

### 2.1 因子分类总表

| # | 因子类型 | GTSAM 类 | 连接的变量 | 残差维度 | 噪声模型 | Sigma/协方差来源 | Huber 阈值 | 说明 |
|---|---------|----------|-----------|---------|---------|-----------------|-----------|------|
| 1 | IMU 预积分 | `CombinedImuFactor` | `x_i, v_i, b_i, x_j, v_j, b_j` | 15 (R3+V3+P3+Ba3+Bg3) | 预积分协方差逆矩阵 Cholesky | 艾伦方差标定 (`ACC_N`, `GYR_N`, `ACC_W`, `GYR_W`) | 不使用 | 连接相邻 KF 的高频运动约束 |
| 2 | 双目视觉（Smart） | `SmartStereoProjectionPoseFactor` | `x_i, x_j, ...`（多帧位姿） | 2/观测（内部消元路标） | `Isotropic.Sigma(2, 1.0)` | 像素噪声 ~1.5 px | 内置动态拒绝阈值 3.0 | 路标隐式表示，多帧观测压缩为位姿约束 |
| 3 | 双目视觉（显式） | `GenericStereoFactor3D` | `x_i, l_k` | 3 (uL, uR, v) | `Diagonal.Sigmas(px, px, px)` | 像素噪声 ~1.5 px | Huber(7.815) | 路标转为显式 Point3 后使用 |
| 4 | 相对位姿约束 | `BetweenFactor<Pose3>` | `x_i, x_j` | 6 | `Diagonal.Sigmas(...)` | 旋转 ~0.1°, 平移 ~0.01m | Huber(1.0) | KF 间立体里程计约束（可选） |
| 5 | 首帧先验 | `PriorFactor<Pose3>` + `PriorFactor<Vector3>` + `PriorFactor<ConstantBias>` | `x_0, v_0, b_0` | 6+3+6 | 弱先验 (大 sigma) | 取决于初始化质量 | 不使用 | 锚定第一帧，消除 gauge freedom |
| 6 | 偏置随机游走 | `BetweenFactor<ConstantBias>` | `b_i, b_j` | 6 | `C(9:15,9:15).inverse()` | 预积分协方差的偏置分量 | 不使用 | 偏置时变先验（可选） |

### 2.2 关键噪声参数详解

#### IMU 预积分噪声

```
ACC_N  = 加速度计测量噪声密度 (m/s²/√Hz)，典型值 ~0.01-0.1
GYR_N  = 陀螺仪测量噪声密度 (rad/s/√Hz)，典型值 ~0.0001-0.001
ACC_W  = 加速度计偏置随机游走 (m/s³/√Hz)，典型值 ~0.0001-0.001
GYR_W  = 陀螺仪偏置随机游走 (rad/s²/√Hz)，典型值 ~0.00001-0.0001
```

预积分协方差通过 18 维连续噪声协方差 Q 传播得到 15 维离散协方差 C，取逆后做 Cholesky 分解得到 `sqrt_info`：

```
Q = diag(ACC_N²·I₃, GYR_N²·I₃, ACC_N²·I₃, GYR_N²·I₃, ACC_W²·I₃, GYR_W²·I₃)
C = Σ F·C·Fᵀ + V·Q·Vᵀ  (协方差传播)
sqrt_info = cholesky(C⁻¹)ᵀ
```

**源码锚点** (`integration_base.h:L30-L36`, `L132-L135`):
```cpp
noise.block<3,3>(0,0)  = ACC_N * ACC_N * Eigen::Matrix3d::Identity();
noise.block<3,3>(3,3)  = GYR_N * GYR_N * Eigen::Matrix3d::Identity();
// 协方差传播
jacobian = F * jacobian;
covariance = F * covariance * F.transpose() + V * noise * V.transpose();
```

#### 视觉观测噪声

**立体测量 `(uL, uR, v)` 的噪声模型**：
- 采用 `Isotropic.Sigma(2, 1.0)` 用于 SmartFactor（2D 重投影，只有 uL, v）
- 采用 `Diagonal.Sigmas(1.5, 1.5, 1.5)` 用于 GenericStereoFactor3D（3D 测量，uL, uR, v）
- 像素噪声 σ ≈ 1.0-1.5 px（取决于特征提取器精度和图像分辨率）

#### Huber 损失阈值

| 因子类型 | Huber 阈值 | 理由 |
|---------|-----------|------|
| `GenericStereoFactor3D` | **7.815** | χ²₃ 95th 百分位（3-DOF 立体测量），与 ORB-SLAM3 立体 BA 一致 |
| IMU 预积分 | 不使用 | 预积分已通过协方差白化处理噪声 |
| SmartFactor | 内置拒绝阈值 3.0 | SmartFactor 内部 chi² 网关，动态拒绝 |

**7.815 阈值来源**：`χ²₃(p=0.05) = 7.815`，即 3 自由度残差的 95% 置信区间。这是 ORB-SLAM3 `EdgeStereo` 的标准 Huber 阈值（`Optimizer.cc:L132: const float thHuber3D = sqrt(7.815)`），也是用户 PFB 后端 `local_ba` 中使用的值。直接复用。

### 2.3 SmartFactor 内部参数

**源码锚点** (`VioBackend.cpp:L1811-L1830`):

| 参数 | 默认值 | 含义 |
|------|-------|------|
| `rankTolerance` | 1.0 | 三角化秩容忍度 |
| `landmarkDistanceThreshold` | 20.0 | 最大三角化距离 (m) |
| `retriangulationThreshold` | 0.001 | 重三角化触发阈值 |
| `dynamicOutlierRejectionThreshold` | 3.0 | 动态异常值拒绝 chi² 阈值（Euroc YAML） |
| `linearizationMode` | `HESSIAN` | 使用 Hessian 线性化 |
| `degeneracyMode` | `ZERO_ON_DEGENERACY` | 退化时给零约束 |
| `throwCheirality` | false | 不抛 cheirality 异常 |

---

## 三、因子注入顺序与优化流程

### 3.1 每关键帧的因子注入伪代码

这是最关键的设计约束：**SmartFactor 必须在 IMU/先验因子之前加入，因为 iSAM2 需要前 N 个 slot 来追踪 SmartFactor 的位置。**

```
procedure onKeyframe(kf_id, imu_pim, stereo_measurements, landmarks_active):
    // ===== 阶段 0: 状态初始化 =====
    kf_id += 1
    
    // ===== 阶段 1: 写入新变量初值 =====
    new_values.insert(x<kf_id>,  pose_prediction)   // 来自 IMU/视觉/PnP
    new_values.insert(v<kf_id>,  velocity_prediction)
    new_values.insert(b<kf_id>,  imu_bias_prediction)
    
    // ===== 阶段 2: 添加 IMU 因子 =====
    new_other_factors.add(CombinedImuFactor(
        x<prev>, v<prev>, b<prev>, x<kf_id>, v<kf_id>, b<kf_id>, imu_pim
    ))
    
    // ===== 阶段 3: 添加里程计因子（可选）=====
    if stereo_odometry_valid:
        new_other_factors.add(BetweenFactor<Pose3>(
            x<prev>, x<kf_id>, odom_pose, odom_noise
        ))
    
    // ===== 阶段 4: 更新路标观测 =====
    for each active_landmark:
        update_landmark_in_graph(lmk_id, kf_id, stereo_observation)
        // 内部: clone 旧 SmartFactor → 追加新观测 → 存入 new_smart_factors
    
    for each new_landmark:
        create_new_smart_factor(lmk_id, all_observations)
        // 内部: 创建 SmartStereoFactor, 添加所有历史观测 → 存入 new_smart_factors
    
    // ===== 阶段 5: 组装因子图 =====
    new_factors = NonlinearFactorGraph()
    
    // !! 顺序约束: SmartFactor 必须最先 !!
    for each smart_factor in new_smart_factors:
        new_factors.add(smart_factor)           // slot 0, 1, ..., M-1
    
    for each factor in new_other_factors:
        new_factors.add(factor)                 // slot M, M+1, ...
    
    // ===== 阶段 6: iSAM2 增量更新 =====
    result = smoother.update(new_factors, new_values, timestamp, delete_slots)
    
    // ===== 阶段 7: Cheirality 异常恢复 =====
    if cheirality_exception:
        restore smoother backup
        cleanCheiralityLmk(bad_landmark)   // 删除异常路标的所有因子
        recursive_retry(new_factors_except_bad_lmk)
    
    // ===== 阶段 8: 提取优化后状态 =====
    state = smoother.calculateEstimate()
    W_Pose_B  = state.at<Pose3>(x<kf_id>)
    W_Vel_B   = state.at<Vector3>(v<kf_id>)
    imu_bias  = state.at<ConstantBias>(b<kf_id>)
    
    return {W_Pose_B, W_Vel_B, imu_bias}
```

### 3.2 因子排序约束的原理

**核心问题**：`IncrementalFixedLagSmoother::update()` 返回的 `ISAM2Result` 中 `newFactorsIndices` 指示新因子在因子图全局 slot 数组中的位置。SmartFactor 在生命周期内会被替换（clone 后追加观测），需要根据 slot 号找到旧 SmartFactor 以标记删除。

如果 SmartFactor 不是第一批因子，IMU 因子占用了前面的 slot，会导致 SmartFactor slot 追踪错位。

**Kimera-VIO 源码注释** (`VioBackend.cpp:L1036`):
> "SMART FACTORS MUST BE FIRST, otherwise when recovering the slots for the smart factors we will mess up"

### 3.3 排序对三角化质量的影响

- SmartFactor 在 `add()` 时并不立即三角化。三角化发生在 iSAM2 线性化阶段，此时使用当前的位姿估计。
- 如果 IMU 因子先于 SmartFactor 加入，iSAM2 可能在 SmartFactor 线性化前已经更新了位姿，导致 SmartFactor 使用中间状态的位姿做三角化。
- 这是导致"ghost landmark"（三角化点始终为空）的直接原因之一——SmartFactor 在错误的线性化点处三角化失败。

---

## 四、iSAM2 参数配置

### 4.1 参数表

| 参数 | 推荐值 | Kimera-VIO 默认 | DM-VIO 参考 | 理由与调参指南 |
|------|-------|----------------|------------|--------------|
| `relinearizeThreshold` | 0.01 | 0.01 | N/A (不用iSAM2) | 控制重线性化激进程度。0.01 = 当变量变化超过 0.01 rad/m 时重线性化。双目 VIO 精度通常在此量级，不宜过大（精度退化）也不宜过小（频繁重线性化） |
| `relinearizeSkip` | 1 | 1 | N/A | 每次 `update()` 后检查是否需要重线性化。对于在线 VIO 建议 1（每次都检查），对于回环后的重新优化可增大到 5-10 |
| `wildfireThreshold` | 0.001 | 0.001 | N/A | GN/Dogleg 的 wildfire 阈值。0.001 足够小，防止过度"火焰传播" |
| `optimizationParams` | **GaussNewton** | GaussNewton (`useDogLeg_=false`) | N/A | GN 在初值良好时收敛更快。双目 VIO 的初值通常由 IMU 预积分提供，质量较高，GN 优于 Dogleg |
| `cacheLinearizedFactors` | **true** | true (硬编码) | N/A | 缓存线性化因子，重线性化时避免因子的重复线性化，显著降低计算开销 |
| `factorization` | **CHOLESKY** | CHOLESKY (硬编码) | N/A | Cholesky 分解在对称正定情况下最稳定高效 |
| `findUnusedFactorSlots` | **true** | true | N/A | 允许 SmartFactor 替换时回收旧 slot |
| `evaluateNonlinearError` | false | false | N/A | 关闭以节省计算（不每次计算全图非线性误差） |
| `enableDetailedResults` | false | false | N/A | 生产环境关闭以减少开销 |

### 4.2 ISAM2Params 初始化代码

```cpp
gtsam::ISAM2Params isam_param;
isam_param.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
isam_param.cacheLinearizedFactors = true;
isam_param.relinearizeThreshold = 0.01;
isam_param.relinearizeSkip = 1;
isam_param.findUnusedFactorSlots = true;
isam_param.evaluateNonlinearError = false;
isam_param.enableDetailedResults = false;
isam_param.factorization = gtsam::ISAM2Params::CHOLESKY;

// 创建固定滞后平滑器
auto smoother = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
    nr_states,     // 窗口大小 = 25 个关键帧
    isam_param
);
```

**源码锚点**: `VioBackendParams.cpp:L27-L54`

### 4.3 Gaudding Newton vs Dogleg 的对比如下

| | GaussNewton | Dogleg |
|---|------------|--------|
| 收敛速度 | 快（初值好时二次收敛） | 中等 |
| 鲁棒性 | 对差初值敏感 | 更鲁棒 |
| 双目 VIO 适用性 | ✅ 推荐（IMU 初值质量高） | ❌ 不需要额外鲁棒性开销 |
| 每步开销 | 较低 | 较高（需要计算 Cauchy point） |

---

## 五、边缘化策略

### 5.1 IncrementalFixedLagSmoother 的默认边缘化

`IncrementalFixedLagSmoother` 在每次 `update()` 后自动执行以下操作：
1. 检查所有变量的时间戳
2. 将时间戳超出 `lag_seconds` 窗口的变量标记为"待边缘化"
3. 使用贝叶斯树（Bayes Tree）条件化，将旧变量的信息压缩为剩余变量上的先验
4. 从活动图中移除旧变量

**关键特性**：
- **自动管理**：不需要手动指定边缘化哪些变量。按时间戳自动决定。
- **可逆边缘化**：贝叶斯树的条件化是可逆的。当回环因子连接旧变量时，可以通过添加反因子重新线性化。
- **选择性重线性化**：只重线性化与新因子相关的贝叶斯树枝（cliques），不触碰树的其余部分。

### 5.2 何时延迟边缘化（参考 DM-VIO）

**问题**：在 VIO 初始化阶段或参数未收敛时，立即边缘化旧帧会在错误的线性化点处固定信息，产生"拖拽"后续优化的先验因子。

**策略**：

| 场景 | 边缘化策略 | 理由 |
|------|-----------|------|
| **初始化阶段**（速度/偏置/重力未收敛） | **延迟边缘化** | 等待参数收敛后再压缩信息，避免错误先验锁定 |
| **正常运行**（参数已收敛） | 立即边缘化 | 窗口管理正常运作 |
| **回环检测到** | 重线性化受影响枝 | 贝叶斯树天然支持 |
| **Cheirality 异常** | 回退并删除异常因子后重试 | 避免无效因子污染边缘化先验 |

**延迟边缘化实现概要**（基于 DM-VIO 的 `DelayedMarginalization::readvanceUntilDelay`）：
```
class DelayedMarginalizationQueue:
    pending_marg_frames: deque<KeyframeState>
    delay_count: int = 5  // 延迟帧数
    
    function onNewKeyframe(kf_state):
        pending_marg_frames.push_back(kf_state)
        
        if pending_marg_frames.size() > delay_count:
            old_frame = pending_marg_frames.pop_front()
            perform_schur_complement(old_frame)
```

**关键收敛检查**（决定何时可以开始边缘化）：
- 尺度波动：`max(scale) / min(scale) - 1.0 < 0.005`（DM-VIO 经验值）
- 重力方向变化：`Log(R_grav⁻¹ * R_grav_prev).norm() < 0.001 rad`
- IMU 偏置增量：`|db|_∞ < 1e-4`
- 速度增量：`|dv|_∞ < 0.001 m/s`

### 5.3 边缘化失败的恢复

当 `IndeterminantLinearSystemException` 发生时：
1. 恢复 `smoother` 的备份（`IncrementalFixedLagSmoother` 支持拷贝）
2. 在首帧和异常帧附近添加加强的 `PriorFactor`（Pose, Velocity, Bias）
3. 重新调用 `smoother.update()`
4. 如果仍然失败，删除异常帧的所有因子并标记关键帧为无效

**源码锚点**: `VioBackend.cpp:L1383-L1550`（`updateSmoother` 异常处理）

---

## 六、异常值剔除

### 6.1 设计原则

**教训**（来自 PHAD 踩坑记录 `2026-05-18-phad-frontend-pitfalls.md` 第 6 和第 7 条）：
- ❌ **只用预过滤**：chi² 预过滤在 iSAM2 增量平滑中过度剔除观测 → 因子图失去约束 → RMSE 退化 10 倍以上
- ❌ **全局阈值**：MH（低纹理）和 V1_01（丰富纹理）需要不同阈值
- ✅ **后验剔除 + 预过滤配对**：先让 iSAM2 优化全量因子，再从优化结果中剔除真正异常的观测

### 6.2 三层异常值防御

```
层 1: 前端几何验证（KLT RANSAC + 立体 NCC）
    ↓ 筛选后的观测
层 2: iSAM2 优化内置（SmartFactor 动态拒绝 + Huber 核）
    ↓ 优化后残差
层 3: 后验 chi² 硬阈值剔除（关键！）
    ↓ 干净的因子集
层 1bis: 重新优化（可选）
```

### 6.3 后验异常值剔除伪代码

```
procedure postUpdateOutlierRejection(smoother):
    estimate = smoother.calculateEstimate()
    graph = smoother.getFactors()              // 当前活动因子图
    outlier_slots = []                          // 待删除的因子 slot
    
    // ===== 1. 检查每个 GenericStereoFactor =====
    for each factor in graph:
        if factor is GenericStereoFactor3D:
            // 计算 unwhitened error
            error = factor.unwhitenedError(estimate)
            // chi²_3, p=0.05 → 7.815（95% 置信区间）
            if error.norm() > 7.815:
                outlier_slots.append(factor.slot())
    
    // ===== 2. 检查每个 SmartStereoFactor =====
    for each factor in graph:
        if factor is SmartStereoProjectionPoseFactor:
            // SmartFactor 内部已有 outlier 标记
            if factor.isOutlier() or factor.isDegenerate():
                outlier_slots.append(factor.slot())
    
    // ===== 3. 删除异常因子（需要 iSAM2 支持）=====
    if outlier_slots is not empty:
        // 注意: ISAM2 的 removeFactors 在某些版本可能不支持
        // 替代方案: 标记因子为 inactive (set active=false)
        for each slot in outlier_slots:
            graph.at(slot).setActive(false)
        
        // 可选: 重新优化
        result = smoother.update(empty_graph, empty_values, timestamp, outlier_slots)
    
    return len(outlier_slots)  // 返回剔除数量用于诊断
```

### 6.4 Chi² 阈值选择论证

| 自由度 (DOF) | χ² 值 (p=0.05) | 来源 | 适用因子 |
|-------------|----------------|------|---------|
| 3 | **7.815** | ORB-SLAM3 `thHuber3D` | `GenericStereoFactor3D` (uL, uR, v) |
| 2 | **5.991** | ORB-SLAM3 `thHuber2D` | `GenericProjectionFactor` (单目 u, v) |
| 15 | ~25.0 | χ²₁₅(p=0.05) | `CombinedImuFactor` (P,R,V,Ba,Bg) |

**推荐配置**：
- 立体视觉：Huber(7.815) + 后验剔除阈值 7.815
- IMU 预积分：不使用 Huber（内部已通过协方差白化），后验剔除阈值 25.0
- SmartFactor：内置动态拒绝阈值 3.0，外加后验检查 `isOutlier()` / `isDegenerate()`

**自适应阈值**（ORB-SLAM3 参考）：
- 近处点（深度 < 10m）：可使用 1.5 × 标准阈值（更宽松，深度精度高 → 噪声小）
- 远点（深度 > 20m）：可使用标准阈值（深度精度低 → 噪声大）

---

## 七、SmartFactor → GenericStereoFactor 迁移路径

### 7.1 迁移的动机

PHAD 的 PFB 后端使用 `GenericStereoFactor` + 显式 `Point3` 地标，RMSE = 0.09m（EuRoC V1_01）。切换到全部使用 SmartFactor 后 RMSE 退化到 1.81m（20×）。需要在同一 iSAM2 图中将**质量达标的 SmartFactor 提升为显式 Point3 + GenericStereoFactor**。

### 7.2 迁移条件

SmartFactor → 显式地标转换的**准入条件**：

```
function readyForPromotion(smart_factor):
    // 1. 三角化结果必须有效
    if not smart_factor.point().valid():   return false
    
    // 2. 观测数达到阈值
    if smart_factor.measured().size() < MIN_OBS_FOR_PROMOTION:
        return false  // 建议 MIN_OBS_FOR_PROMOTION = 4
    
    // 3. 三角化质量检查
    if smart_factor.isDegenerate():        return false
    if smart_factor.isFarPoint():          return false  // 距离 > 20m
    if smart_factor.isOutlier():           return false
    if smart_factor.isPointBehindCamera(): return false
    
    // 4. 三角化不确定性检查
    triangulation = smart_factor.point()
    if triangulation.error() > MAX_TRI_ERROR:
        return false  // 建议 MAX_TRI_ERROR = 0.01
    
    return true
```

**源码锚点** (`VioBackend.cpp:L963-L976`: `isSmartFactor3dPointGood`)

### 7.3 迁移过程伪代码

```
procedure promoteLandmarkToExplicit(smoother, graph, lmk_id, smart_factor):
    // ===== 1. 提取三角化后的 3D 点 =====
    point3d = smart_factor.point().point()    // 获取显式 Point3
    obs_list = smart_factor.measured()        // 获取所有双目观测
    
    // ===== 2. 在 Values 中注册新的地标变量 =====
    new_values.insert(Symbol('l', lmk_id), point3d)
    
    // ===== 3. 创建 GenericStereoFactor 替换 SmartFactor =====
    new_factors = NonlinearFactorGraph()
    for each obs in obs_list:
        frame_id = obs.first
        stereo_measurement = obs.second
        factor = GenericStereoFactor3D(
            stereo_measurement,
            stereo_noise,            // Diagonal.Sigmas(1.5, 1.5, 1.5)
            Symbol('x', frame_id),    // 位姿变量
            Symbol('l', lmk_id),      // 地标变量（新！）
            stereo_cal
        )
        new_factors.add(factor)
    
    // ===== 4. 在 iSAM2 中执行替换 =====
    delete_slots = [smart_factor.slot]  // 标记旧 SmartFactor 的 slot 待删除
    
    // 将新因子和新变量传给 smoother
    result = smoother.update(new_factors, new_values, timestamp, delete_slots)
    
    // ===== 5. 更新地标状态 =====
    landmark_state[lmk_id] = EXPLICIT
    landmark_lifecycle[lmk_id].promoted_at = timestamp
```

### 7.4 迁移时机与约束

| 条件 | 策略 |
|------|------|
| **最低观测数** | 4 帧双目观测（至少两个不同视角，确保三角化稳定性） |
| **迁移频率** | 每帧最多迁移 5 个路标（防止 iSAM2 单次 update 负担过重） |
| **并发约束** | 迁移过程中 `iSAM2.update()` 不接受其他新因子（需要同步） |
| **初始化阶段** | 初始化期间禁止迁移（参数未收敛，SmartFactor 三角化不可靠） |
| **回退机制** | 如果迁移后 GenericStereoFactor 立即被后验剔除，回退为 SmartFactor |

### 7.5 回退实现

```
if postUpdateOutlierRejection(smoother) rejects a GenericStereoFactor:
    // 重新创建 SmartFactor 并替换
    revert_factor = new SmartStereoFactor(original_observations)
    delete_slots = [generic_factor.slot]
    smoother.update([revert_factor], empty_values, timestamp, delete_slots)
    landmark_state[lmk_id] = SMART  // 回退
```

---

## 八、因子图可视化结构

```
                ┌─────────────────────────────────────────────────┐
                │             GTSAM iSAM2 因子图                   │
                │          (IncrementalFixedLagSmoother)          │
                │                                                 │
                │  先验层: PriorFactor(Pose3) + PriorFactor(Vel)  │
                │          PriorFactor(ConstantBias)     ← 首帧   │
                │                                                 │
                │  运动层: CombinedImuFactor ───────────────────  │
                │          (x₀,v₀,b₀) → (x₁,v₁,b₁) → (x₂,v₂,b₂) │
                │          ───────────→ ───────────→ ───────────→ │
                │                                                 │
                │  视觉层:                                        │
                │    SmartFactor: [x₁, x₂, x₃] ───→ (隐式路标 A) │
                │                  [x₂, x₃, x₄] ───→ (隐式路标 B) │
                │    Explicit:     [x₃, x₄] + l_C ─→ GSF₃        │
                │                                                 │
                │  约束层: BetweenFactor<Pose3>                   │
                │          (x₁,x₂) ← 立体里程计                   │
                └─────────────────────────────────────────────────┘

每关键帧 KFₖ 的变量组:
  ┌──────────────────────────────────────────┐
  │ Symbol('x', k)  ← Pose3       (6-DOF)    │
  │ Symbol('v', k)  ← Vector3     (速度)     │
  │ Symbol('b', k)  ← ConstantBias (偏置)    │
  └──────────────────────────────────────────┘

显式路标 Lₘ:
  ┌──────────────────────────────────────────┐
  │ Symbol('l', m)  ← Point3      (3-DOF)    │
  └──────────────────────────────────────────┘

因子连接模式:
  IMU 链:  ──KF₀──KF₁──KF₂──KF₃──KF₄── ... ──KF₂₅──  (时间链)
            │    │    │    │    │           │
 视觉因子:  ├────┼────┼─── L₀                 (路标0)
            │    ├────┼────┼── L₁              (路标1)
            └────┼────┼── L₂                   (路标2)
                         │
               (SmartFactor 隐式: 只连接位姿，不暴露路标)
               (GenericStereoFactor 显式: 连接位姿 + Point3)
```

---

## 九、后端失败模式与恢复策略

### 9.1 失败模式总表

| # | 失败模式 | 症状 | 根因 | 检测方法 | 恢复策略 |
|---|---------|------|------|---------|---------|
| 1 | **Cheirality 异常** | `StereoCheiralityException`，iSAM2 崩溃 | 奇异三角化：点在相机后方或远点 | GTSAM 异常捕获 | 删除异常路标的所有因子，恢复 smoother 备份，递归重试（最多 5 次） |
| 2 | **不确定线性系统** | `IndeterminantLinearSystemException` | 秩亏：关键帧间约束不足（静止、纯旋转） | GTSAM 异常捕获 | 恢复备份，在首帧和异常附近帧添加增强 prior，重试 |
| 3 | **Gauge Freedom 漂移** | 轨迹整体平移/旋转，RMSE 持续增长 | 首帧先验过弱或缺失 | 检查 `state_error > 5σ` | 强化首帧 prior（缩小 sigma），或添加 GNSS 位置先验 |
| 4 | **边缘化先验"拖拽"** | 优化残差持续上升，轨迹偏离真值 | 错误线性化点处的先验污染后续优化 | 检查 `isam2.error() / n_factors > 2.0` | 重置 iSAM2（清除所有先验），从当前 KF 重新开始 |
| 5 | **SmartFactor 退化** | 大量路标 `isDegenerate() = true` | 低视差、远点、重复纹理、错误匹配 | 统计 `nDegenerate / nTotal > 0.5` | 降低 `landmarkDistanceThreshold`，增加最小观测数，前端加强 RANSAC |
| 6 | **IMU 偏置发散** | `bias.norm()` 持续增大，轨迹弯曲 | 视觉约束不足（弱纹理）→ IMU 主导 → 偏置错误估计 | `|bias_k - bias_{k-1}| > 1e-3` | 增大视觉因子信息权重，降低 IMU 因子权重（动态权重调整） |
| 7 | **尺度漂移**（单目立体退化） | 轨迹在尺度方向漂移，Z 方向 RMSE > X/Y 方向 | 远处点缺乏深度约束，双目基线不足 | `scale_error > 10% over 50 frames` | 添加高度/地面约束因子，或引入 GNSS 绝对尺度 |
| 8 | **因子槽泄漏** | SmartFactor 的 slot 被错误回收或永不回收 | `deleteSlots` 配置错误 | 因子图因子数量异常增长 | 检查 `findUnusedFactorSlots=true`，验证 SmartFactor slot 追踪逻辑 |
| 9 | **重线性化风暴** | CPU 飙升，实时性下降 | `relinearizeThreshold` 过小触发频繁重线性化 | 检查 `isam2.getRelinearized()` 频率 | 增大 `relinearizeThreshold` 到 0.05，或增大 `relinearizeSkip` 到 5 |
| 10 | **初始化阶段参数未收敛** | 前 50 帧 RMSE 大，后续改善 | 尺度/重力/偏置在初始化后继续漂移 | 检查 `|g_est - g_true| > 0.1 m/s²` 或 `|bias - bias_prev| > 5e-4` | 延迟边缘化直到参数收敛，启用多级初始化（重力对齐→偏置估计→尺度细化） |

### 9.2 恢复伪代码

```
procedure handleOptimizationFailure(smoother, error):
    switch error.type:
    
    case CheiralityException:
        // 层 1: 删除异常路标
        bad_lmk = extractLandmarkFromException(error)
        removeAllFactorsForLandmark(bad_lmk)
        retry updateSmoother()
        
        // 层 2: 如果仍然失败，扩大删除范围
        if retry_failed:
            removeAllLandmarksWithLowObservations(nObs < 3)
            retry updateSmoother()
    
    case IndeterminantLinearSystem:
        // 层 1: 添加先验
        addPriorOnRecentFrames(Pose3, Vector3, ConstantBias)
        retry updateSmoother()
        
        // 层 2: 如果仍然失败，重置 iSAM2
        if retry_failed:
            resetISAM2(current_state)  // 从当前状态重新开始
            logWarning("iSAM2 reset: Indeterminant system")
    
    case GaugeFreedomDrift:
        // 添加或强化首帧先验
        strengthenPriorOnFirstFrame(scale_factor=10.0)
        // 或在 Values 中固定首帧 (NonlinearEquality)
        setFirstFrameConstant()
    
    case PriorDrag:
        // 重置整个 iSAM2 和 smoother
        // 这是最后的手段——丢失所有边缘化信息
        resetISAM2(current_state)
        logWarning("iSAM2 reset: prior drag detected")
```

### 9.3 诊断信号

每个关键帧输出以下指标用于运行时监控：

| 指标 | 正常范围 | 告警阈值 | 含义 |
|------|---------|---------|------|
| `isam2.error() / n_factors` | < 1.0 | > 2.0 | 平均每因子残差 |
| `nDegenerate / nTotal` | < 0.3 | > 0.5 | SmartFactor 退化率 |
| `nOutlierRejected / nTotal` | < 0.1 | > 0.3 | 后验异常值剔除率 |
| `relinearizedCount` | < 5 | > 20 | 每帧重线性化的变量数 |
| `smootherLagSize` | ~25 | < 5 或 > 50 | 当前活动窗口大小 |
| `imuBiasDrift / frame` | < 1e-6 | > 1e-4 | 偏置每帧漂移率 |

---

## 十、Agent 实现提示

### 适用场景

当 Agent 需要为双目 VIO 系统构建基于 GTSAM iSAM2 的因子图后端时使用本设计。适用于：
- 从零实现或重构 VIO 后端的工程场景
- 从 SmartFactor-only 迁移到 SmartFactor + GenericStereoFactor 混合的阶段
- 需要参考 Kimera-VIO、DM-VIO、VINS-Fusion、ORB-SLAM3 的后端设计模式

### 输入输出契约

- **输入**：
  - `imu_pim: CombinedImuFactor::PreintegratedMeasurements` — 关键帧间 IMU 预积分结果
  - `stereo_measurements: vector<LandmarkId → FrameId → StereoPoint2>` — 双目特征跟踪结果
  - `landmark_states: map<LandmarkId, LandmarkState>` — 路标状态（SMART / EXPLICIT）
  - `pose_predictions: map<FrameId, Pose3>` — 由前端提供的每帧位姿初值
  - `calibration: Cal3_S2Stereo + body_P_cam` — 双目标定参数
- **输出**：
  - `W_Pose_B, W_Vel_B, imu_bias: 每关键帧的优化后状态`
  - `diag_metrics: 诊断指标（残差、退化率、剔除率等）`

### 实现骨架（伪代码）

```
class StereoVIOBackend:
    smoother: IncrementalFixedLagSmoother
    new_smart_factors: map<LandmarkId, SmartStereoFactor>
    old_smart_factors: map<LandmarkId, (SmartStereoFactor, slot)>
    explicit_landmarks: map<LandmarkId, Point3>
    
    function onKeyframe(kf_id, imu_pim, stereo_obs, pose_pred):
        // 1. 添加状态变量
        new_values = Values()
        new_values.insert(x<kf_id>, pose_pred)
        new_values.insert(v<kf_id>, predictVelocity(prev_state, imu_pim))
        new_values.insert(b<kf_id>, prev_state.bias)
        
        // 2. 添加 IMU 和里程计因子
        other_factors = NonlinearFactorGraph()
        other_factors.add(CombinedImuFactor(
            x<prev>,v<prev>,b<prev>,x<kf_id>,v<kf_id>,b<kf_id>, imu_pim
        ))
        
        // 3. 更新路标因子（SmartFactor 管理）
        smart_factors = NonlinearFactorGraph()
        for each lmk in stereo_obs:
            if lmk.state == EXPLICIT:
                // 已提升：使用显式 Point3 + GenericStereoFactor
                other_factors.add(GenericStereoFactor3D(
                    lmk.obs[kf_id], x<kf_id>, l<lmk.id>, stereo_cal
                ))
            else:
                // 仍在 Smart 阶段：更新 SmartFactor
                clone_and_add_observation(lmk, kf_id, stereo_obs)
                smart_factors.add(lmk.smart_factor)
        
        // 4. !! SmartFactor 必须最先 !!
        all_factors = NonlinearFactorGraph()
        all_factors.push_back(smart_factors)    // 先
        all_factors.push_back(other_factors)    // 后
        
        // 5. iSAM2 增量更新
        try:
            result = smoother.update(all_factors, new_values, timestamp, delete_slots)
        catch CheiralityException:
            cleanAndRetry(bad_lmk)
        catch IndeterminantLinearSystemException:
            addPriorsAndRetry(kf_id)
        
        // 6. 后验异常值剔除
        outlier_count = postUpdateOutlierRejection(smoother)
        
        // 7. 尝试提升成熟 SmartFactor → 显式 Point3
        if initializationComplete():
            for each lmk in smart_factors:
                if readyForPromotion(lmk):
                    promoteLandmarkToExplicit(lmk)
        
        return calculateEstimate()
```

### 关键源码片段

**Kimera-VIO: 因子注入顺序** (`VioBackend.cpp:L1060-L1120`):
```cpp
// SMART FACTORS MUST BE FIRST
for (const auto& new_smart_factor : new_smart_factors_) {
    if (slot != -1) {
        delete_slots.push_back(slot);           // 标记旧 slot 删除
        new_factors_tmp.push_back(new_smart_factor.second);
    } else {
        new_factors_tmp.push_back(new_smart_factor.second);
    }
}
// 随后追加 IMU/prior/other 因子
new_factors_tmp.push_back(new_imu_prior_and_other_factors_.begin(),
                          new_imu_prior_and_other_factors_.end());
```

**Kimera-VIO: SmartFactor 参数配置** (`VioBackend.cpp:L1811-L1830`):
```cpp
smart_factors_params->setRankTolerance(1.0);
smart_factors_params->setLandmarkDistanceThreshold(20.0);
smart_factors_params->setDynamicOutlierRejectionThreshold(3.0);
smart_factors_params->setLinearizationMode(gtsam::HESSIAN);
smart_factors_params->setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
```

**ORB-SLAM3: 双目 chi² 阈值** (`Optimizer.cc:L131-L132`):
```cpp
const float thHuber2D = sqrt(5.99);   // χ²_2, p=0.05
const float thHuber3D = sqrt(7.815);  // χ²_3, p=0.05 = stereo Huber delta
```

**VINS-Fusion: 后验异常值剔除** (`estimator.cpp:L1558-L1568`):
```cpp
void Estimator::outliersRejection(set<int> &removeIndex) {
    for (auto &it_per_id : f_manager.feature) {
        double ave_err = 0; int errCnt = 0;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            ave_err += reprojectionError(it_per_frame);
            errCnt++;
        }
        ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)  // 阈值: 3 像素
            removeIndex.insert(it_per_id.feature_id);
    }
}
```

### 实现注意事项

1. **因子排序约束不可违**：SmartFactor 必须在其他因子之前加入 `new_factors`。如果排序错误，SmartFactor 的 slot 追踪会错位，导致 ghost landmark 和因子泄漏。

2. **SmartFactor 更新用 clone-and-add**：不要直接修改已在线性化的 SmartFactor。必须 clone → 追加观测 → 作为新因子加入图 → 标记旧 slot 删除。直接修改会导致线性化不一致。

3. **后验异常值剔除先于边缘化检查**：在评估是否提升 SmartFactor 之前，先剔除异常观测。否则异常观测会使三角化结果不可靠，触发错误的提升决策。

4. **Chi² 阈值需要自适应**：全局阈值在低纹理场景会过度剔除（参考 PHAD 踩坑记录）。建议算法：
   ```cpp
   double adaptiveChi2Threshold(double depth, int trackLength) {
       double base = 7.815;  // χ²_3, p=0.05
       if (depth < 5.0)  return base * 1.5;   // 近点宽松
       if (depth > 20.0) return base * 0.8;   // 远点严格
       if (trackLength < 4) return base * 2.0; // 短 track 宽松
       return base;
   }
   ```

5. **初始化阶段不提升 SmartFactor**：在速度/偏置/重力方向充分收敛之前，SmartFactor 的内部三角化不可靠。提升到显式 Point3 会引入错误的 3D 先验。建议在初始化完成（偏置收敛、重力误差 < 0.01 rad）后再开始提升。

6. **iSAM2 reset 是核选项**：当边缘化先验严重污染图质量时，唯一可靠的恢复是重置整个 iSAM2。这会丢失所有历史边缘化信息，但能从干净的图重新开始。重置后需要重播最近的关键帧状态。

### 源码检索锚点

- Kimera-VIO 后端: `raw/codes/Kimera-VIO/src/backend/VioBackend.cpp` — 全部后端实现
  - 因子注入顺序: `L1036-L1120`（`optimize` 中组装因子）
  - `updateSmoother`: `L1383-L1636`（iSAM2 update + 异常处理）
  - `addVisualInertialStateAndOptimize`: `L296-L427`（每帧主入口）
  - SmartFactor 创建: `L485-L512`（`addLandmarkToGraph`）
  - SmartFactor 有效性检查: `L963-L976`（`isSmartFactor3dPointGood`）
- Kimera-VIO 参数: `raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackendParams.h:L106-L153`
  - `setIsam2Params`: `raw/codes/Kimera-VIO/src/backend/VioBackendParams.cpp:L27-L54`
- VINS-Fusion 边缘化: `raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.cpp:L183-L311`
- ORB-SLAM3 异常值剔除: `raw/codes/ORB_SLAM3/src/Optimizer.cc:L999-L1041`（Motion-only BA）、`L2853-L2883`（Local Inertial BA）
- DM-VIO 延迟边缘化: `raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp:L212-L253`
  - FEJ 管理: `raw/codes/dm-vio/src/GTSAMIntegration/FEJValues.h`
- GTSAM ISAM2 API: `raw/codes/gtsam/gtsam/nonlinear/ISAM2.h`

---

## 相关页面

- [[方法-ISAM2增量固定滞后平滑]] — ISAM2/IncrementalFixedLagSmoother 核心概念
- [[概念-因子图]] — 因子图与 GTSAM 基础
- [[概念-Schur补与边缘化]] — 边缘化的数学原理
- [[概念-延迟边缘化]] — DM-VIO 的延迟边缘化策略
- [[方法-SmartStereoFactor]] — SmartStereoProjectionPoseFactor 使用细节
- [[方法-滑动窗口边缘化]] — VINS-Fusion 的手动 Schur 补实现
- [[架构-后端适配器模式]] — 后端抽象层设计模式
- [[优化后端选型指南]] — Ceres vs g2o vs GTSAM 全景对比
- [[算法-Kimera-VIO]] — Kimera-VIO 系统概述
- [[方法-关键帧选择策略]] — 关键帧选择与窗口管理的交互
- [[方法-视觉特征跟踪]] — 前端特征跟踪（影响 SmartFactor 观测质量）
- [[phad_fusion设计总结]] — PHAD 融合系统整体设计
- [[组件-GTSAM]] — GTSAM 库概述
