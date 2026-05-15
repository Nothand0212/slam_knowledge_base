---
tags: [VIO, 初始化, IMU, SfM, 视觉惯性]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_sfm.cpp
  - raw/codes/ORB_SLAM3/src/LocalMapping.cc
  - raw/codes/open_vins/ov_init/src
---

# VIO 初始化方法

> 系统对比 VINS-Fusion、ORB-SLAM3、open_vins 三种代表性 VIO/SLAM 系统的视觉惯性初始化管线。涵盖视觉 SfM、IMU 预积分对齐、陀螺偏置估计、重力方向求解、速度与尺度恢复等核心环节。

## 初始化问题的本质

视觉惯性初始化需要从单目视觉（无尺度）和 IMU（无全局位置）的耦合数据中恢复以下未知量：

| 未知量 | 符号 | 自由度 | 困难来源 |
|--------|------|--------|----------|
| 尺度因子 | $s$ | 1 | 单目相机无法观测绝对尺度 |
| 重力方向 | $\mathbf{g}$ | 2（模长已知，仅方向未知） | 加速度计测量包含重力和本体加速度 |
| 陀螺仪偏置 | $\mathbf{b}_g$ | 3 | 旋转预积分误差在 $\Delta t$ 上累积 |
| 加速度计偏置 | $\mathbf{b}_a$ | 3 | 与重力耦合，短时窗口不可观 |
| 各帧速度 | $\mathbf{v}_k$ | $3N$ | 纯视觉不提供速度信息 |
| 全局航向角 | yaw | 1 | 重力仅约束 roll/pitch，航向不可观 |

核心矛盾：视觉提供无尺度的相对运动，IMU 提供有尺度的本体加速度和角速度，两者通过关键帧间的时间对齐建立约束。若运动激励不足（纯匀速直线、零加速度变化），则 $\mathbf{b}_a$、$\mathbf{g}$ 和 $s$ 之间的耦合难以解开，初始化会失败。

## VINS-Fusion 初始化管线

VINS-Fusion 采用 **视觉 SfM → IMU 预积分对齐** 的两阶段管线，入口为 `Estimator::initialStructure()`（`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L580`）。

### 伪代码

```pseudo
function VINSFusion_Initialize(all_image_frames):
    // Phase 1: IMU 可观性检查
    compute average g estimate from IMU delta_v/dt
    if variance < 0.25:
        warn "IMU excitation not enough"

    // Phase 2: 视觉 SfM
    collect tracked feature tracks → sfm_features
    select reference frame l with enough parallax
    compute relative_R, relative_T between frame l and latest frame
    sfm.construct(frame_count, poses Q[], trans T[], l, rel_R, rel_T, sfm_f)
    // 内部顺序: 三角化 → PnP → 全 BA (Ceres)

    // Phase 3: PnP 恢复非关键帧位姿
    for each non-keyframe:
        solvePnP using 3D-2D correspondences from SfM

    // Phase 4: 视觉-惯性对齐
    return visualInitialAlign()

function visualInitialAlign():
    VisualIMUAlignment(all_image_frame, Bgs, g, x)
    // 子步骤 1: 陀螺偏置
    solveGyroscopeBias(all_image_frame, Bgs)
    // 子步骤 2: 线性求解速度、重力、尺度
    LinearAlignment(all_image_frame, g, x)
    // 子步骤 3: 切空间重力细化
    RefineGravity(all_image_frame, g, x)

    // 恢复各帧状态
    recover scale s from x
    for each frame: P_i = s * P_i - R_i * TIC - (s * P_0 - R_0 * TIC)
    recover velocities V_i = R_i * x.segment<3>(i*3)

    // 重力对齐到世界坐标系
    R0 = g2R(g)  →  yaw = R2ypr(R0 * R0_ref).x()
    R0_aligned = ypr2R(-yaw, 0, 0) * R0
    rotate all Ps, Rs, Vs by R0_aligned

    // 重新三角化
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric)
    return true
```

### 关键源码锚点

| 环节 | 文件/函数 | 行号 |
|------|-----------|------|
| IMU 可观性检查 | `estimator.cpp:initialStructure()` | L580–L610 |
| 视觉 SfM 构建 | `initial_sfm.cpp:GlobalSFM::construct()` | L128–L323 |
| SfM PnP | `initial_sfm.cpp:solveFrameByPnP()` | L33–L83 |
| 三角化 | `initial_sfm.cpp:triangulatePoint()` | L16–L30 |
| PnP 恢复非关键帧 | `estimator.cpp:initialStructure()` | L648–L715 |
| 视觉惯性对齐入口 | `estimator.cpp:visualInitialAlign()` | L726–L784 |
| 陀螺偏置求解 | `initial_aligment.cpp:solveGyroscopeBias()` | L14–L47 |
| 线性对齐 | `initial_aligment.cpp:LinearAlignment()` | L135–L207 |
| 重力精细化 | `initial_aligment.cpp:RefineGravity()` | L65–L133 |
| 切空间基 | `initial_aligment.cpp:TangentBasis()` | L50–L63 |
| 顶层调用 | `initial_aligment.cpp:VisualIMUAlignment()` | L209–L217 |

## ORB-SLAM3 初始化管线

ORB-SLAM3 在 LocalMapping 线程中完成 IMU 初始化，核心函数为 `LocalMapping::InitializeIMU()`（`raw/codes/ORB_SLAM3/src/LocalMapping.cc:L1173`）。它依赖已有的视觉地图（ORB 特征 + 关键帧），通过 g2o 非线性优化联合估计偏置、速度、尺度和重力方向。

### 伪代码

```pseudo
function ORB_SLAM3_InitializeIMU(priorG, priorA, bFIBA):
    require at least 10 KFs and 2s (mono) / 1s (stereo) duration

    collect temporal sequence of KFs: vpKF[]

    if map not IMU-initialized:
        // 重力方向粗估计
        dirG = sum over adjacent KFs: -R_prev * Δv_ij
        dirG = dirG / |dirG|
        Rwg = rotation that aligns dirG → (0,0,-1)  // 世界 z 轴指向重力反方向
        // 速度初始化
        for each KF pair: v_i = (P_imu_j - P_imu_i) / dt_ij

    // 非线性优化 (g2o, 200 iterations)
    InertialOptimization(map, Rwg, scale, bg, ba, isMono, covInertial,
                         bFixedVel=false, bGauss=false, priorG, priorA)

    // 优化变量:
    //   - KeyFrame velocities (V_i)
    //   - Gyroscope bias (b_g)
    //   - Accelerometer bias (b_a)
    //   - Gravity direction (VertexGDir: SO(3) rotation matrix)
    //   - Scale (single scalar, fixed for stereo)

    // IMU 边 EdgeInertialGS 连接 8 个顶点:
    //   VP1, VV1, VG, VA, VP2, VV2, VGDir, VS
    //   残差 = IMU 预积分 - 视觉估计的运动

    // 检查尺度有效性
    if scale < 0.1: return failure

    // 应用优化结果
    apply scale * rotation to map (ScaledRotation)
    recover velocities and biases to each KF

    // 可选: Full Inertial BA (if bFIBA and good priors)
    if bFIBA:
        FullInertialBA(map, 100 iterations)
```

### 关键源码锚点

| 环节 | 文件/函数 | 行号 |
|------|-----------|------|
| IMU 初始化入口 | `LocalMapping.cc:InitializeIMU()` | L1173–L1319 |
| 重力方向粗估计 | `LocalMapping.cc:InitializeIMU()` | L1228–L1253 |
| 惯性优化（全变量） | `Optimizer.cc:InertialOptimization(Rwg,scale,bg,ba,...)` | L3042–L3224 |
| 惯性优化（仅偏置） | `Optimizer.cc:InertialOptimization(bg,ba,...)` | L3227–L3387 |
| 惯性优化（仅重力+尺度） | `Optimizer.cc:InertialOptimization(Rwg,scale)` | L3389 起 |
| 关键帧惯性位姿优化 | `Optimizer.cc:PoseInertialOptimizationLastKeyFrame()` | L4491 起 |

## open_vins 初始化管线

open_vins 支持 **静态初始化** 和 **动态初始化** 两种模式，由 `InertialInitializer` 根据视差检测自动切换（`raw/codes/open_vins/ov_init/src/init/InertialInitializer.cpp:L79`）。

### 静态初始化（Static Initialization）

要求设备静止，用加速度计均值估计重力方向，陀螺均值估计偏置。

```pseudo
function StaticInitialize(imu_data, wait_for_jerk):
    require two windows of IMU readings

    // 窗口 1→0: 最新半窗口，检查静止
    a_avg_1to0 = mean(accel in window)
    a_var_1to0 = std(accel in window)
    if a_var_1to0 > thresh and wait_for_jerk:
        return false  // 正在运动，等待静止

    // 窗口 2→1: 较早半窗口，估计初始状态
    a_avg_2to1 = mean(accel)
    w_avg_2to1 = mean(gyro)
    if a_var_2to1 > thresh and wait_for_jerk:
        return false  // 之前在运动

    // 重力方向对齐
    z_axis = a_avg_2to1 / |a_avg_2to1|     // 加速度指向重力反方向
    R_GtoI = gram_schmidt(z_axis)           // z 轴对齐重力方向
    q_GtoI = rot2quat(R_GtoI)

    // 偏置估计
    b_g = w_avg_2to1                        // 陀螺偏置 = 静止时角速度均值
    b_a = a_avg_2to1 - R(q_GtoI)·g          // 加计偏置 = 实测均值 - 理论重力

    return {q_GtoI, b_g, b_a, v=0}
```

源码位于 `raw/codes/open_vins/ov_init/src/static/StaticInitializer.cpp:L37–L164`。

### 动态初始化（Dynamic Initialization）

无需静止，建线性系统求解特征点位置、速度和重力方向。基于 Martinelli (2012) 的闭式解。

```pseudo
function DynamicInitialize(feature_db, imu_data, camera_extrinsics):
    // Phase 1: 选择有效的相机位姿和特征点
    filter features: ≥ min_num_meas_to_optimize observations
    check: ≥ min_valid_features features, enough camera poses
    check: angular change ≥ min_deg threshold

    // Phase 2: 预积分 IMU
    for each camera_time i:
        cpi_I0toIi: preintegrate from t0 to ti
        cpi_IitoIi+1: preintegrate from ti to ti+1

    // Phase 3: 构建线性系统 Ax = b
    // 状态: [features(3N), velocity(3), gravity(3)]
    // 约束: 特征点在每帧的归一化投影误差
    for each feature f, each observation at time t:
        H_i = projection Jacobian × R_ItoC × R_I0toIt
        H_i contributes to: feature_f, velocity, gravity columns
        A.append(H_i); b.append(b_i)

    // Phase 4: 带重力模长约束的闭式解
    solve: min |A*x - b|²  subject to |g| = gravity_mag
    → 使用 Dong-Si 系数 + 伴随矩阵求特征值
    → 选最小实特征值 λ，恢复 g = (D - λI)^{-1} d,
       feature+vel = -(A1^T A1)^{-1} A1^T (A2*g - b)

    // Phase 5: Ceres MLE 非线性优化
    problem = Ceres::Problem
    add IMU preintegration factors (Factor_ImuCPIv1)
    add visual reprojection factors (Factor_ImageReprojCalib)
    add gravity prior (Factor_GenericPrior: |g| = 9.81)
    solve with Dogleg/LM solver

    // Phase 6: 重力对齐到世界系
    R_GtoI0 = gram_schmidt(gravity_inI0)
    transform all states to gravity-aligned global frame

    return initialized state with covariance
```

源码位于 `raw/codes/open_vins/ov_init/src/dynamic/DynamicInitializer.cpp:L44–L1107`。

线性系统的关键残差（式 14, Martinelli 2012）：

$$\mathbf{p}_{F}^{C_i} = \mathbf{R}_{C_0 C_i} \mathbf{R}_{IC}^{\top} \left( \mathbf{p}_{F}^{I_0} - \mathbf{v}_{I_0}^{I_0} \Delta t - \frac{1}{2} \mathbf{g}^{I_0} \Delta t^2 - \boldsymbol{\alpha}_{I_0}^{I_i} \right) + \mathbf{p}_{I}^{C}$$

### 初始化模式切换

`InertialInitializer::initialize()`（`raw/codes/open_vins/ov_init/src/init/InertialInitializer.cpp:L79`）通过视觉视差判断运动状态：

| 条件 | 初始化模式 |
|------|-----------|
| 最近窗口静止 + 更早窗口静止 | 静态初始化 (`wait_for_jerk=false`) |
| 最近窗口运动 + 更早窗口静止 | 静态初始化 (`wait_for_jerk=true`, jerk detected) |
| 两个窗口都在运动 | 动态初始化 (if enabled) |

## 三种管线对比矩阵

| 特性 | VINS-Fusion | ORB-SLAM3 | open_vins |
|------|-------------|-----------|-----------|
| **视觉初始化** | 5 点法本质矩阵 + Global SfM + BA | ORB 特征匹配 + 无需额外 SfM（复用地图） | 特征跟踪 + 无需显式 SfM |
| **陀螺偏置** | 线性最小二乘（IMU 旋转 vs 视觉旋转残差） | g2o 非线性优化（EdgeInertialGS 边） | 静态取陀螺均值 / 动态不显式估计 |
| **速度+重力+尺度** | 线性 LDLT 求解 + 切空间重力细化（4 次迭代） | g2o 非线性 LM 优化（200 次迭代） | 线性闭式解（带 |g| 约束 + 伴随矩阵） + Ceres MLE |
| **重力处理** | 切空间 2-DoF 参数化（TangentBasis） | SO(3) 旋转矩阵参数化（VertexGDir） | 向量直接求解 + |g|=9.81 约束 + gram_schmidt 对齐 |
| **加速度计偏置** | 不优化（线性阶段未包含） | 联合优化（带先验约束） | 静态估计 / 动态包含在 MLE 中 |
| **静止要求** | 非必须（移动中即可启动） | 非必须（有足够 KF 和窗口时长即可） | 静态模式需要静止 / 动态模式不需要 |
| **优化后端** | Ceres（SfM BA）+ LDLT（对齐） | g2o LM | Ceres（MLE）+ Eigen 闭式解 |
| **输出** | 尺度 + 重力 + 速度 + 偏置 + 位姿 | 尺度 + 重力方向 + 速度 + 偏置 | 全状态 + 协方差 |

## 共通设计模式

1. **先旋转后平移**：所有三个系统都先估计陀螺偏置（旋转约束强于平移约束），再求解速度、重力和尺度。
2. **分阶段解耦**：将高维非线性问题分解为视觉位姿 → 陀螺偏置 → 速度/重力/尺度的顺序子问题。
3. **重力模长约束**：已知 $|\mathbf{g}| \approx 9.81$，利用此先验减少 1 DoF 不确定度。VINS-Fusion 用切空间参数化，ORB-SLAM3 用 SO(3) 旋转矩阵，open_vins 用约束闭式解。
4. **预积分重传播**：偏置更新后必须重新计算 IMU 预积分值，确保预积分与最新偏置估计一致。

## Agent 实现提示

### 适用场景

- 单目/双目 VIO 系统启动阶段，需要从视觉位姿和 IMU 数据中恢复尺度、重力方向、速度、偏置
- 需要对比或实现不同 VIO 系统的初始化管线（VINS-Fusion / ORB-SLAM3 / open_vins）
- 设计新的 VIO 初始化方法时，参考现有方案的分阶段解耦策略

### 输入输出契约

**VINS-Fusion 初始化**
- 输入：多帧图像特征跟踪结果（`all_image_frame`，含特征点 2D 坐标和帧位姿初值）、IMU 预积分
- 输出：尺度 `s`、重力方向（对齐到世界系）、各帧速度 `V_i`、陀螺偏置 `Bgs`、校正后的帧位姿 `Ps/Rs`
- 失败条件：IMU 激励方差 < 0.25（movement excitation insufficient）

**ORB-SLAM3 初始化**
- 输入：已有视觉地图（≥10 个关键帧，持续时间 ≥2s 单目/1s 双目）、关键帧间 IMU 预积分
- 输出：尺度、重力方向（SO(3) 旋转矩阵）、各 KF 速度、陀螺/加计偏置
- 失败条件：优化后 `scale < 0.1` 或地图关键帧数不足

**open_vins 动态初始化**
- 输入：特征点数据库（≥min_valid_features 个特征，每特征 ≥min_num_meas_to_optimize 次观测）、IMU 数据、相机外参
- 输出：全状态初始值 + 协方差矩阵（特征点位置、速度、重力方向），后续可选 Ceres MLE 精化

### 实现骨架（伪代码）

```pseudo
def VIO_Initialize(frames, imu_data, mode):
    # Phase 0: 激励检查
    if compute_imu_excitation_variance(imu_data) < 0.25:
        return FAIL("insufficient excitation")

    # Phase 1: 视觉初始化（获取无尺度位姿）
    if mode == VINS:
        poses = run_5pt_essential_matrix + Global_SfM + BA(frames)
    elif mode == ORB_SLAM3:
        poses = reuse_existing_visual_map()  # ORB 地图已就绪
    elif mode == open_vins:
        passes = True  # 动态初始化不需要显式 SfM

    # Phase 2: 陀螺偏置估计（旋转约束）
    bg = solve_gyro_bias(imu_preintegrations, visual_rotations)
    repropagate_all_preintegrations(bg)  # 必须重传播

    # Phase 3: 速度 + 重力 + 尺度联合求解
    if mode == VINS:
        v, g, s = solve_linear_alignment()       # LDLT 闭式解
        g = refine_gravity_tangent_space(g)       # 切空间 4 次迭代
    elif mode == ORB_SLAM3:
        v, g, s, bg, ba = g2o_inertial_optim()   # LM 200 次迭代
    elif mode == open_vins:
        v, g, features = solve_closed_form()      # |g|=9.81 约束 + 伴随矩阵
        v, g, features, ba = ceres_mle_refine()   # Ceres 精化

    # Phase 4: 重力对齐到世界系
    Rwg = rotation_aligning_g_to_world(g)
    rotate_all_states(Rwg)
    retriangulate_features()

    return {scale: s, gravity: g, velocities: v, biases: (bg, ba)}
```

### 关键源码片段

`raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L209-L217` — VisualIMUAlignment 顶层调用：
```cpp
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    solveGyroscopeBias(all_image_frame, Bgs);
    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else
        return false;
}
```

`raw/codes/ORB_SLAM3/src/LocalMapping.cc:L1173-L1319` — InitializeIMU 核心逻辑（非惯性优化入口）：
```cpp
// 重力方向粗估计
dirG.setZero();
for(size_t i=0; i<vpKF.size()-1; i++) {
    Velocity vel = IMU::Preintegrated::GetAverageVelocity(...);
    dirG -= rotation * vel;   // 累积多个帧对的加速度方向
}
dirG = dirG/dirG.norm();
Rwg = rotation_that_aligns(dirG, (0,0,-1));

// g2o 优化: 200 iterations
Optimizer::InertialOptimization(mpMap, Rwg, scale, bg, ba, ...);
```

### 实现注意事项

1. **顺序不可颠倒**：必须先解陀螺偏置再解重力/速度/尺度。陀螺偏置误差会在预积分中通过时间累积放大对平移的影响，若颠倒顺序会导致重力方向估计严重偏差。
2. **预积分重传播必须执行**：每次偏置更新后立即 `repropagate()` / `Reintegrate()`，否则后续速度/重力求解会使用带旧偏置的预积分值。
3. **重力模长先验不可或缺**：已知 $|\mathbf{g}| \approx 9.81$ 约束可减少 1 DoF 不确定度。VINS 用切空间参数化（2-DoF），ORB-SLAM3 用 SO(3)（2-DoF），open_vins 用等式约束闭式解。
4. **加速度计偏置短窗口不可观**：若运动激励不足（匀速直线），$\mathbf{b}_a$ 与重力耦合，短窗口内无法分离。VINS 在线性阶段不优化 $\mathbf{b}_a$，ORB-SLAM3 依赖先验约束，open_vins 依赖较长窗口和 MLE。
5. **视觉旋转质量决定初始化成败**：三种方法都依赖视觉旋转估计的精度。若视觉 SfM 失败（特征少、视差不足），后续所有阶段都会传播误差。

### 源码检索锚点

| 系统 | 文件 | 函数/位置 | 行号 |
|------|------|-----------|------|
| VINS-Fusion | `initial_aligment.cpp` | `solveGyroscopeBias()` | L14–L47 |
| VINS-Fusion | `initial_aligment.cpp` | `LinearAlignment()` | L135–L207 |
| VINS-Fusion | `initial_aligment.cpp` | `RefineGravity()` | L65–L133 |
| VINS-Fusion | `initial_sfm.cpp` | `GlobalSFM::construct()` | L128–L323 |
| VINS-Fusion | `estimator.cpp` | `visualInitialAlign()` | L726–L784 |
| ORB-SLAM3 | `LocalMapping.cc` | `InitializeIMU()` | L1173–L1319 |
| ORB-SLAM3 | `Optimizer.cc` | `InertialOptimization（全变量）` | L3042–L3224 |
| ORB-SLAM3 | `Optimizer.cc` | `InertialOptimization（仅偏置）` | L3227–L3387 |
| open_vins | `DynamicInitializer.cpp` | `initialize()` 全流程 | L44–L1107 |
| open_vins | `StaticInitializer.cpp` | `initialize()` 静止模式 | L37–L164 |
| open_vins | `InertialInitializer.cpp` | 模式切换逻辑 | L79 |

## 相关页面

- [[概念-视觉惯性初始化策略]]
- [[方法-重力精细化]]
- [[方法-IMU陀螺偏置初始化]]
- [[方法-OnlineGravityAlignment]]
- [[概念-IMU预积分]]
- [[方法-INS-centric 初始化]]
- [[概念-MSCKF]]
