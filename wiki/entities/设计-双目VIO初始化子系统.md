---
tags: [PHAD, VIO, stereo, initialization, IMU, gravity, bias, GTSAM, SmartFactor, design]
created: 2026-06-01
updated: 2026-06-01
type: entity
sources:
  - raw/codes/open_vins/ov_init/src/static/StaticInitializer.cpp
  - raw/codes/open_vins/ov_init/src/init/InertialInitializer.cpp
  - raw/codes/ORB_SLAM3/src/LocalMapping.cc
  - raw/codes/ORB_SLAM3/src/Optimizer.cc
  - raw/codes/ORB_SLAM3/include/G2oTypes.h
  - raw/codes/Kimera-VIO/src/initial/OnlineGravityAlignment.cpp
  - raw/codes/Kimera-VIO/src/initial/InitializationBackend.cpp
  - raw/codes/Kimera-VIO/src/backend/VioBackend.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp
  - wiki/entities/方法-VIO初始化方法.md
  - wiki/entities/概念-视觉惯性初始化策略.md
  - wiki/entities/架构-GTSAM iSAM2 双目VIO后端设计.md
  - wiki/entities/设计-立体VIO前端管线.md
---

# 双目 VIO 初始化子系统设计

> 完整的双目+IMU VIO 初始化子系统设计规格。双目已知基线提供绝对尺度，无需尺度估计——这是相比单目 VIO 的核心简化。初始化需估计重力方向、IMU 偏置和初始速度。采用双路径（静态 → 动态回退）策略，集成 GTSAM iSAM2 后端和路标管线。

---

## 1. 双目初始化的核心简化：无尺度估计

**关键认知**：双目视觉已知基线 `b`，特征点的绝对 3D 坐标可直接三角化：

$$Z = \frac{f_x \cdot b}{d}, \quad X = \frac{u_L - c_x}{f_x} Z, \quad Y = \frac{v - c_y}{f_y} Z$$

因此**双目 VIO 初始化不需要估计尺度因子 $s$**。这直接消除了所有参考系统中单目初始化最困难的部分：

| 参考系统 | 单目需估计的未知量 | 双目可跳过 |
|---------|------------------|-----------|
| VINS-Fusion | $\mathbf{v}_k(3N)$, $\mathbf{g}(3)$, $\mathbf{b}_g(3)$, $s(1)$ | $s$ |
| ORB-SLAM3 | $\mathbf{v}_k(3N)$, $\mathbf{g}(2)$, $\mathbf{b}_g(3)$, $\mathbf{b}_a(3)$, $s(1)$ | $s$ |
| OpenVINS | $\mathbf{v}_k(3N)$, $\mathbf{g}(3)$, $\mathbf{b}_g(3)$, features(3M) | $\mathbf{b}_a$(静态模式) |
| **本设计 (双目)** | $\mathbf{v}_k(3N)$, $\mathbf{g}(2)$, $\mathbf{b}_g(3)$ | $s$, $\mathbf{b}_a$ (短窗口), features(3M) |

双目初始化的未知量缩减为：**重力方向 (2-DOF)、陀螺仪偏置 (3-DOF)、初始速度 (3N-DOF)**。加速度计偏置在短窗口中与重力耦合且初始值小，可在后续正常 VIO 中逐步收敛。

---

## 2. 状态机设计

### 2.1 状态定义

```
UNINITIALIZED ──► 系统启动，未收集到足够数据
    │
    ├── 收集 ≥ 1.0s IMU 数据 + 前端开始产帧
    │
    ▼
STATIC_CHECK ──► 检测设备是否静止
    │              条件: 收集两个 0.5s 子窗口的 IMU 数据
    │              检查: a_var_1to0 < THRESH 且 a_var_2to1 < THRESH
    │
    ├── 静止 ──► STATIC_INIT ──► 成功 ──► INITIALIZED
    │                                │
    │                                └── 失败 ──► DYNAMIC_CHECK
    │
    └── 运动 ──► DYNAMIC_CHECK ──► 满足条件 ──► DYNAMIC_INIT ──► 成功 ──► INITIALIZED
                     │                                              │
                     │                                              └── 失败 ──► INIT_FAILED
                     │
                     └── 不满足 ──► 等待更多帧 → 重试 STATIC_CHECK 或 DYNAMIC_CHECK

INIT_FAILED ──► 重置数据积累，回到 STATIC_CHECK
```

### 2.2 状态转移条件

| 转移 | 条件 | 检测方式 |
|------|------|---------|
| UNINITIALIZED → STATIC_CHECK | IMU 数据 ≥ 1.0s，前端开始产帧 | 时间戳检查 |
| STATIC_CHECK → STATIC_INIT | `a_var_1to0 < init_imu_thresh AND a_var_2to1 < init_imu_thresh` | 加速度方差（样本标准差） |
| STATIC_INIT → INITIALIZED | 重力幅值检查通过 (`|g_est - g_true| < 0.1`) + 陀螺偏置有效 (`|b_g| < 0.1 rad/s`) | 后验校验 |
| STATIC_INIT → DYNAMIC_CHECK | 重力幅值偏差过大 或 偏置幅值超限 | 后验校验失败 |
| STATIC_CHECK → DYNAMIC_CHECK | 至少一个窗口的 `a_var > init_imu_thresh` | 检测到运动 |
| DYNAMIC_CHECK → DYNAMIC_INIT | 关键帧数 ≥ `kMinInitKFs` (默认 8) 且 时间跨度 ≥ `kMinInitTime` (默认 1.0s) | KF 计数 + 时间戳 |
| DYNAMIC_INIT → INITIALIZED | 陀螺偏置收敛 (`|δ_bg| < 1e-4` 连续 3 次迭代) + 重力幅值检查通过 + χ² 检验 < 阈值 | 优化收敛 + 后验校验 |
| DYNAMIC_INIT → INIT_FAILED | 优化不收敛 或 重力幅值偏差 > 0.5 m/s² 或 条件数过大 | 优化诊断 |
| INIT_FAILED → STATIC_CHECK | 重置所有数据积累 | 重置后重新开始 |

---

## 3. 静态初始化算法

### 3.1 算法流程

参考 OpenVINS `StaticInitializer.cpp` 和 Kimera-VIO `InitializationFromImu.cpp`。

```
function staticInitialize(imu_buffer, params):
    // === 步骤 1: 时间窗口检查 ===
    duration = imu_buffer.last().timestamp - imu_buffer.first().timestamp
    if duration < params.init_window_time:     // 默认 1.0s
        return FAIL("insufficient IMU data")

    // === 步骤 2: 划分两个子窗口 ===
    T_half = duration / 2
    window_1to0 = imu_buffer[t_cur - T_half .. t_cur]    // 最新窗口
    window_2to1 = imu_buffer[t_cur - duration .. t_cur - T_half]  // 较早窗口
    
    if window_1to0.size() < 2 or window_2to1.size() < 2:
        return FAIL("need at least 2 IMU per window")

    // === 步骤 3: 加速度方差 (样本标准差) ===
    a_avg_1to0 = mean(window_1to0.acc)     // 加速度计的 3D 向量均值
    a_var_1to0 = sqrt(Σ||acc_i - a_avg_1to0||² / (N₁-1))   // 标量，单位 m/s²

    a_avg_2to1 = mean(window_2to1.acc)
    w_avg_2to1 = mean(window_2to1.gyr)     // 角速度均值
    a_var_2to1 = sqrt(Σ||acc_i - a_avg_2to1||² / (N₂-1))

    // === 步骤 4: 静止判定 ===
    // 参考 OpenVINS init_imu_thresh = 1.0-1.5 m/s²
    if a_var_1to0 > params.init_imu_thresh:
        return FAIL("currently moving")     // → DYNAMIC_CHECK
    if a_var_2to1 > params.init_imu_thresh:
        return FAIL("was moving before")    // → DYNAMIC_CHECK

    // === 步骤 5: 重力方向估计 (Gram-Schmidt) ===
    // 静止时加速度计测量 a_meas = R_{G→I} · g_G + b_a + n
    // 忽略偏置和噪声后: a_meas ≈ R_{G→I} · g_G
    // 因此归一化加速度方向 = 传感器帧中的重力反方向
    z_axis = a_avg_2to1 / |a_avg_2to1|       // 归一化 → 重力方向在 IMU 系
    
    // Gram-Schmidt: 构建完整旋转矩阵 R_G→I
    // 选与 z_axis 内积更小的基向量做叉积，避免退化
    if |e₁·z| < |e₂·z|:
        x_axis = normalize(z_axis × e₁)       // e₁ = (1,0,0)
    else:
        x_axis = normalize(z_axis × e₂)       // e₂ = (0,1,0)
    y_axis = normalize(z_axis × x_axis)
    R_GtoI = [x_axis | y_axis | z_axis]       // 3×3 旋转矩阵
    
    // 关键: yaw (绕z轴旋转) 不可观，由 Gram-Schmidt 任意固定
    // 符合 VIO 的 4-DOF 不可观性 (全局位置3 + yaw1)

    // === 步骤 6: 偏置估计 ===
    g_G = (0, 0, -params.gravity_mag)         // 全局系重力: z轴向下的 9.81 m/s²

    b_g = w_avg_2to1                          // 陀螺偏置 = 静止时角速度均值 (预期为 0)
    b_a = a_avg_2to1 - R_GtoI · g_G           // 加计偏置 = 实测值 − 理论重力旋转到IMU系

    // === 步骤 7: 后验校验 ===
    g_est = R_GtoI.transpose() · a_avg_2to1   // 将测量旋转到全局系
    if | |g_est| - params.gravity_mag | > params.gravity_tolerance:   // 默认 0.1 m/s²
        return FAIL("gravity magnitude mismatch → DYNAMIC_CHECK")
    if |b_g| > params.max_gyro_bias:          // 默认 0.1 rad/s
        return FAIL("excessive gyro bias → DYNAMIC_CHECK")

    // === 步骤 8: 设置初始状态 ===
    timestamp = window_2to1.last().timestamp  // 使用较早窗口最后一条 IMU 的时间
    q_GtoI = rot2quat(R_GtoI)
    
    // 协方差: 参考 OpenVINS 的经验值
    // orientation: σ=0.02 rad (~1.15°), position: σ=0.05m, velocity: σ=0.01 m/s (static)
    // 偏置: gyro σ=0.01 rad/s, acc σ=0.1 m/s²
    
    return StaticInitResult{
        timestamp, q_GtoI, Vector3::Zero(), Vector3::Zero(),
        b_g, b_a,
        covariance_diag = diag(0.02²·I₃, 0.05²·I₃, 0.01²·I₃, 0.01²·I₃, 0.1²·I₃)
    }
```

### 3.2 静态初始化参数

| 参数 | 符号 | 推荐值 | 来源 |
|------|------|--------|------|
| IMU 收集窗口 | `init_window_time` | **1.0 s** | OpenVINS default; 取 1s 平衡速度与精度 |
| 静止加速度阈值 | `init_imu_thresh` | **1.0 m/s²** | OpenVINS default (EuRoC 用 1.5) |
| 重力幅值容差 | `gravity_tolerance` | **0.1 m/s²** | 比 VINS-Fusion 的 0.5 更严格（静止场景更好） |
| 最大可接受陀螺偏置 | `max_gyro_bias` | **0.1 rad/s** (~5.7°/s) | 工程经验：超过此值暗示传感器故障 |
| Gram-Schmidt 退化检测 | — | `|e₁·z| vs |e₂·z|` | OpenVINS 方法 |
| 每窗口最少 IMU 数 | — | **2** | OpenVINS |

### 3.3 静态初始化的 GTSAM 因子图

```cpp
// 步骤 1: 插入初始状态到 Values
Values init_values;
init_values.insert(X(0), Pose3(R_GtoI, Vector3::Zero()));  // 姿态 + 零位置
init_values.insert(V(0), Vector3::Zero());                   // 零速度
init_values.insert(B(0), ConstantBias(b_a, b_g));            // 估计的偏置

// 步骤 2: 添加先验因子 (固定首帧锚定)
NonlinearFactorGraph init_graph;

// 姿态先验: roll/pitch 高置信度, yaw 低置信度
// 参考 Kimera-VIO 的参数: roll/pitch σ=10°, yaw σ=0.1°
// 静态初始化姿态精度较高，可更紧 (roll/pitch σ=2°, yaw σ=0.5°)
Vector6 pose_sigmas;
pose_sigmas << 0.0087, 0.0087, 0.0087,   // 位置: σ=0.0087 m (紧先验)
               0.0349, 0.0349, 0.0087;    // roll/pitch: σ=2°, yaw: σ=0.5°
auto pose_noise = noiseModel::Diagonal::Sigmas(pose_sigmas);
init_graph.add(PriorFactor<Pose3>(X(0), Pose3(R_GtoI, Vector3::Zero()), pose_noise));

// 速度先验: 零速度, σ=0.01 m/s (静止确信度高)
auto vel_noise = noiseModel::Isotropic::Sigma(3, 0.01);
init_graph.add(PriorFactor<Vector3>(V(0), Vector3::Zero(), vel_noise));

// 偏置先验: 估计值, 陀螺 σ=0.01 rad/s, 加计 σ=0.1 m/s²
Vector6 bias_sigmas;
bias_sigmas << 0.1, 0.1, 0.1,    // 加速度计偏置 σ=0.1 m/s²
               0.01, 0.01, 0.01;  // 陀螺仪偏置 σ=0.01 rad/s
auto bias_noise = noiseModel::Diagonal::Sigmas(bias_sigmas);
init_graph.add(PriorFactor<ConstantBias>(B(0), ConstantBias(b_a, b_g), bias_noise));

// 步骤 3: 构建 iSAM2
// → 第一关键帧后继续正常注入因子
```

---

## 4. 动态初始化算法（双目简化版）

### 4.1 核心思想

双目已知尺度 → 跳过 SfM → 直接从关键帧的立体三角化 3D 点出发，估计陀螺偏置、重力方向和 KF 速度。参考 ORB-SLAM3 的三阶段非线性优化，简化为两阶段（去掉了尺度估计阶段）。

### 4.2 算法流程

```
function dynamicInitialize(kf_queue, imu_pims, params):
    // === 前置条件 ===
    if kf_queue.size() < params.kMinInitKFs:       // 默认 8
        return NEED_MORE_DATA
    time_span = kf_queue.last().timestamp - kf_queue.first().timestamp
    if time_span < params.kMinInitTime:             // 默认 1.0s
        return NEED_MORE_DATA

    // === 阶段 1: 陀螺偏置估计 (线性最小二乘) ===
    // 参考 VINS-Fusion solveGyroscopeBias + Kimera-VIO estimateGyroscopeBias
    bg = Vector3::Zero()
    for iter in 0..4:
        // 构建线性系统 A · δbg = b
        A = MatrixXd::Zero(3*N, 3)
        b = VectorXd::Zero(3*N)
        for each consecutive KF pair (i, i+1):
            delta_R_vis = R_i^T · R_{i+1}                  // 视觉旋转
            delta_R_imu = pim.deltaRij()                     // IMU 预积分旋转
            J_bg = pim.delRdelBiasOmega()                    // 旋转对陀螺偏置的 Jacobian
            // 旋转残差在 SO(3) 李代数中:
            e_R = Log(delta_R_imu^T · delta_R_vis)          // 3D 残差
            A.block(i*3, 0, 3, 3) = J_bg
            b.segment(i*3, 3) = e_R
        delta_bg = (A^T·A)^{-1}·A^T·b
        bg += delta_bg
        if |delta_bg| < 1e-4: break
        // 重传播所有预积分 (一阶修正)
        for each pim: pim.resetIntegrationAndSetBias(b_a=0, bg)

    // === 阶段 2: 速度 + 重力方向非线性优化 (GTSAM) ===
    // 参考 ORB-SLAM3 InertialOptimization (全变量形式，去掉尺度和加计偏置顶点)
    // 使用 GTSAM Levenberg-Marquardt 优化器重构

    // 2a: 重力方向粗估计
    Rwg = estimateInitialRwg(kf_queue, imu_pims)     // 见 4.3 节

    // 2b: 构建 GTSAM 因子图
    graph = NonlinearFactorGraph()
    values = Values()

    for each KF i:
        values.insert(X(i), Pose3(Ps[i], Rs[i]))     // 位姿 (来自前端的 stereo PnP，固定)
        values.insert(V(i), initialVelocity(Ps, Rs))  // 速度初值
        values.insert(B(i), ConstantBias(Vector3::Zero(), bg))  // 偏置 (固定 bg，ba=0)

    // 偏置变量 (所有 KF 共享一个偏置，加速收敛)
    values.insert(BG, bg)  // 陀螺偏置顶点 (3-DOF)
    values.insert(GR, Rwg) // 重力方向顶点 (SO(3), 2-DOF 切线空间更新)

    // 对每个相邻 KF 对添加 IMU 因子
    for each consecutive KF pair (i, j):
        error_model = noiseModel::Gaussian::SqrtInformation(pim.preintMeasCov().llt().matrixU())
        imu_factor = CombinedImuFactor(X(i), V(i), B(i), X(j), V(j), B(j), pim)
        graph.add(imu_factor)

    // 偏置先验: bg 以估计值为中心 (弱先验，允许微调)
    graph.add(PriorFactor<Vector3>(BG, bg, noiseModel::Isotropic::Sigma(3, 0.01)))

    // 重力先验: |g| = 9.81 (弱先验引导方向)
    // 通过初始 Rwg 提供方向初值

    // 2c: 优化 (Gauss-Newton, 50 次迭代)
    params_gtsam = GaussNewtonParams()
    params_gtsam.relativeErrorTol = 1e-5
    params_gtsam.maxIterations = 50
    optimizer = GaussNewtonOptimizer(graph, values, params_gtsam)
    result = optimizer.optimize()

    // 2d: 提取结果
    bg_opt = result.at<Vector3>(BG)
    Rwg_opt = result.at<Rot3>(GR)
    for each KF i:
        velocities[i] = result.at<Vector3>(V(i))

    // === 阶段 3: 后验验证 ===
    g_world = Rwg_opt · (0, 0, -GRAVITY_MAG)
    if | |g_world| - GRAVITY_MAG | > params.gravity_tolerance_dyn:  // 默认 0.1 m/s²
        return FAIL("gravity magnitude check failed")
    if |bg_opt| > params.max_gyro_bias:                              // 默认 0.1 rad/s
        return FAIL("excessive gyro bias")
    
    // 残差检查: 平均每因子 χ² 残差 < 阈值
    avg_chi2 = result.error / graph.size()
    if avg_chi2 > params.max_avg_chi2:                               // 默认 2.0
        return FAIL("high residual")

    return DynamicInitResult{
        Rwg_opt, bg_opt, velocities, result.error
    }
```

### 4.3 重力方向粗估计

参考 ORB-SLAM3 的重力方向初值计算（`LocalMapping.cc:L1228-L1252`）：

```pseudo
function estimateInitialRwg(kf_queue, imu_pims):
    dirG = Vector3::Zero()
    for each consecutive KF pair (i, i+1):
        // 加速度积分 (去偏后)
        delta_v = pim.deltaVij()        // IMU 预积分速度增量
        R_i = kf_queue[i].rotation()
        // 累加: 速度增量旋转回第 i 帧系，再映射到世界系
        // 在较短窗口中，速度变化主要来自重力
        dirG -= R_i * delta_v
    dirG = dirG / |dirG|                // 归一化得到重力方向
    
    // 构建旋转矩阵: 将 (0,0,-1) 旋转到 dirG
    gI = (0, 0, -1)                     // 理想重力在世界系
    v = gI × dirG                       // 旋转轴
    ang = acos(gI · dirG)               // 旋转角
    if |v| > 1e-6:
        Rwg = Exp_SO3(v / |v| * ang)    // 轴角 → SO(3)
    else:
        Rwg = I₃
    return Rwg
```

### 4.4 SO(3) 切线空间重力参数化

参考 ORB-SLAM3 `VertexGDir`（`G2oTypes.h:L257-L293`）—— 仅 2-DOF 更新：

```cpp
// 重力方向顶点: 在其 SO(3) 切空间中为 2 维
// 更新: Rwg ← Rwg · Exp(δθ_x, δθ_y, 0)
// 绕重力轴 (z) 的旋转不可观 → 固定为 0

Vector3 gravity_update(double dx, double dy) {
    // Exp_SO3(dx, dy, 0) 在 Rwg 右侧
    return Rot3::Expmap(Vector3(dx, dy, 0.0));
}
```

对应的重力 Jacobian（对速度/位置残差的线性化）：

```cpp
// 重力对位姿残差的影响 (∂f/∂θ_g):
// dGdTheta = Rwg · [[0, 0], [0, 0], [-g, 0]; [g, 0]; [0, 0]]

// 等效矩阵:
// Gm = [0   -g    // 只有 (0,1) 和 (1,0) 非零
//       g    0
//       0    0]
// dGdTheta = Rwg · Gm    (3×2)
```

### 4.5 动态初始化参数

| 参数 | 符号 | 推荐值 | 来源 |
|------|------|--------|------|
| 最少关键帧数 | `kMinInitKFs` | **8** | VINS-Fusion WINDOW_SIZE=10 的保守版; ORB-SLAM3 用 10 |
| 最少时间跨度 | `kMinInitTime` | **1.0 s** | ORB-SLAM3 stereo=1.0s |
| 陀螺偏置收敛阈值 | — | **1e-4 rad/s** | 连续 3 次迭代变化小于此值 |
| 陀螺偏置估计迭代上限 | — | **5** | VINS-Fusion/OpenVINS 经验 |
| 重力幅值容差 (动态) | `gravity_tolerance_dyn` | **0.3 m/s²** | 比静态模式宽松 (静态用 0.1) |
| 最大可接受陀螺偏置 | `max_gyro_bias` | **0.1 rad/s** | 与静态一致 |
| 最大平均 χ² | `max_avg_chi2` | **2.0** | 参考 ORB-SLAM3 的 chi² 阈值思路 |
| GN 相对误差容限 | — | **1e-5** | GTSAM 默认 |
| GN 最大迭代 | — | **50** | 比 ORB-SLAM3 的 200 少 (双目问题更简单) |
| 所需双目有效特征数 | — | **20 per KF** | 前端已有的质量门控阈值 |

---

## 5. 初始状态规范

### 5.1 GTSAM Values 插入

初始化完成后，在 `Values` 中插入以下变量：

```
Values init_values:
    X(0) = Pose3(R_GtoI, Vector3::Zero())  ← 首帧位姿 (IMU系在世界系)
    V(0) = Vector3::Zero()                  ← 首帧速度
    B(0) = ConstantBias(b_a_est, b_g_est)  ← 首帧 IMU 偏置

后续每插入一个 KF i:
    X(i) = Pose3(R_i, p_i)           ← 来自前端 stereo PnP 的位姿
    V(i) = v_i                        ← 来自初始化估计的速度
    B(i) = ConstantBias(b_a, b_g)    ← 共享偏置 (初始阶段不分帧独立偏置)
```

**关键设计决定**：
- 前 3 个 KF 共享同一个偏置变量 `B(0)`（通过 `BetweenFactor<ConstantBias>` 连接），避免偏置在数据不足时发散。
- 第 4 个 KF 开始，引入逐帧偏置变量（`B(1)`, `B(2)`, ...），通过 bias random walk 因子连接。

### 5.2 PriorFactor 噪声模型

| 先验 | GTSAM 因子 | Sigma 值 | 噪声模型 | 理由 |
|------|-----------|---------|---------|------|
| 首帧姿态 | `PriorFactor<Pose3>` | `(0.01, 0.01, 0.01, 0.05, 0.05, 0.001)` m/rad | `Diagonal::Sigmas` | 位置紧，roll/pitch 紧(静态)或中等(动态)，yaw 极紧 (固定 gauge) |
| 首帧速度 | `PriorFactor<Vector3>` | **0.1 m/s** (静态)/**1.0 m/s** (动态) | `Isotropic::Sigma` | 静态下确信零速；动态下弱先验 |
| 首帧偏置 (bg) | `PriorFactor<ConstantBias>` (bias 分量) | bg: **0.02 rad/s**, ba: **0.2 m/s²** | `Diagonal::Sigmas` | 中等先验，允许后续 VIO 优化 |
| 首帧偏置 (ba) | `PriorFactor<ConstantBias>` (bias 分量) | **0.2 m/s²** | — | 弱先验 (加计偏置小但不可忽略) |

**姿态先验细粒度解释**（参考 Kimera-VIO 的坐标系区分）：

```cpp
// 世界系 → 机体系旋转的噪声: 在世界系中指定 roll/pitch/yaw 的不确定度
// roll/pitch: 静态 σ=0.05 rad (~2.9°), 动态 σ=0.17 rad (~10°)
// yaw: σ=0.001 rad (~0.06°) — 极紧先验固定 gauge freedom
// 位置: σ=0.01 m (默认原点)

Vector6 pose_sigmas = (is_static) ?
    Vector6(0.01, 0.01, 0.01, 0.05, 0.05, 0.001) :   // 静态
    Vector6(0.01, 0.01, 0.01, 0.17, 0.17, 0.001);     // 动态
auto pose_noise = noiseModel::Diagonal::Sigmas(pose_sigmas);
```

### 5.3 路标因子约束

**初始化期间全部使用显式双因子**：

```cpp
// 每个立体路标在 KF i 的观测 → GenericStereoFactor3D
StereoPoint2 measurement(uL, uR, v);                          // 双目观测
auto noise = noiseModel::Isotropic::Sigma(3, 1.0);            // σ=1.0 px
auto factor = GenericStereoFactor3D(measurement, noise,
                                     X(i),                    // KF 位姿
                                     L(lmk_id),               // 3D 点
                                     stereo_cal);
graph.add(factor);

// 3D 点先验 (来自立体三角化)
values.insert(L(lmk_id), Point3(X_w, Y_w, Z_w));              // 世界系 3D 坐标
auto point_prior = noiseModel::Isotropic::Sigma(3, 0.5);      // σ=0.5 m (立体不确定性)
graph.add(PriorFactor<Point3>(L(lmk_id), Point3(X_w, Y_w, Z_w), point_prior));
```

### 5.4 SmartFactor 禁用协议

初始化期间（前 `kInitKFCount` 个关键帧，默认 10）：
- **禁止创建 SmartFactor**：前端传入的 `PendingLandmark` 全部以显式 `Point3` + `GenericStereoFactor3D` 注入图。
- **禁止路标提升**：不做 SmartFactor → GenericStereoFactor 迁移。
- **禁止异常值剔除**（后验 chi²）：初始化阶段保留所有观测，让先验和图结构自我约束。

初始化完成后（`INITIALIZED` 状态）：
- 新路标以 SmartFactor 试验期开始（参考 [[架构-GTSAM iSAM2 双目VIO后端设计]] 第 7 章）
- 启用 SmartFactor → 显式路标的迁移路径
- 启用后验异常值剔除（Huber + chi² 硬阈值）

---

## 6. 转换协议

### 6.1 INITIALIZED 状态转换动作

```
function onInitializationComplete(init_result):
    // 1. 设置标志
    initialization_complete = true
    landmark_pipeline.setInitializationComplete(true)

    // 2. 延迟边缘化结束
    // 参考 DM-VIO DelayedMarginalization: 初始化期间保持全 KF 不边缘化
    // 现在恢复正常的 IncrementalFixedLagSmoother 窗口管理
    smoother.setLagSeconds(normal_lag_seconds)  // 恢复正常窗口 (25 KF)

    // 3. 启用 SmartFactor 试验期
    landmark_pipeline.enableSmartFactorTrial(true)
    // 新路标以 SmartFactor 模式进入图
    // 成熟后提升为 GenericStereoFactor

    // 4. 启用后验异常值剔除
    backend.enableOutlierRejection(true)

    // 5. 偏置收敛监控
    // 如果动态初始化: 继续监控 bg 收敛
    // 在后续 5 个 KF 内，若 bias 变化 < 1e-4 per KF → 偏置已收敛

    // 6. 唤醒正常 VIO 管线
    return INITIALIZED
```

### 6.2 初始化期间的特殊处理

| 特性 | 初始化期间 (前 10 KF) | 正常模式 (第 11 KF+) |
|------|---------------------|-------------------|
| 路标类型 | 全部 `Point3` + `GenericStereoFactor3D` (显式) | SmartFactor 试验 → 显式迁移 |
| 边缘化 | **延迟**：不边缘化任何帧 | 正常滑动窗口边缘化 |
| 异常值剔除 | **禁用**后验 chi² | 启用 Huber + 后验 chi² |
| 偏置变量 | 共享偏置 (前 3 KF 用 `B(0)`) | 逐帧偏置 (`B(0)`, `B(1)`, ...) |
| 首帧先验 | 紧 pose 先验 + 中等 vel 先验 | 不再添加额外先验 |

### 6.3 偏置衔接

动态初始化估计的 `bg` 作为 `B(0)` 的初始值。后续 KF 的偏置通过以下方式传播：

```cpp
// KF 0 → KF i 的偏置衔接
// BetweenFactor 连接 B(0) 和 B(i):
values.insert(B(i), ConstantBias(ba_current, bg_current));  // 从上一帧复制
auto bias_walk_noise = noiseModel::Diagonal::Sigmas(
    Vector6(acc_rw*sqrt(dt), acc_rw*sqrt(dt), acc_rw*sqrt(dt),
            gyr_rw*sqrt(dt), gyr_rw*sqrt(dt), gyr_rw*sqrt(dt))
);
graph.add(BetweenFactor<ConstantBias>(B(0), B(i), ConstantBias(0,0,0,0,0,0), bias_walk_noise));
```

---

## 7. 失败检测与回退策略

### 7.1 失败模式矩阵

| # | 失败模式 | 症状 | 检测 | 恢复 |
|---|---------|------|------|------|
| F1 | **IMU 激励不足** | 静态判断全通过但动态始终不满足 | `a_var < threshold` 持续 > 5s | 降低 `init_imu_thresh` 到 0.5，重试 |
| F2 | **陀螺偏置发散** | 动态优化后 `|bg| > 0.1 rad/s` | 后验校验 | 切换到静态模式等待静止 |
| F3 | **重力方向不一致** | `|g_est - g_true| > 0.5 m/s²` | 优化后重力幅值检查 | 加更多 KF 重试；若有外部先验则用 GNSS 航向 |
| F4 | **优化不收敛** | GN/LM 未达到 `relativeErrorTol` | `optimizer.optimize()` 状态 | 增加 KF 数；若仍失败 → INIT_FAILED |
| F5 | **条件数过大** | 线性系统 (A^TA) 条件数 > 1e8 | Eigen `ldlt().info()` | 纯旋转运动 → 等待平移激励 |
| F6 | **纯旋转退化** | 位置约束不足，速度可观性差 | 连续 KF 位移 < 0.01m | 移动相机 → 等待平移 |
| F7 | **前端丢失** | KF 数不增长 | 连续 3s 无新 KF | 重置 → 重新开始 |
| F8 | **IMU 数据缺失** | 预积分窗口为空 | `pim.isNull()` | 回退到纯视觉模式 |

### 7.2 自适应重试策略

```pseudo
function handleInitFailure(failure_mode, retry_count):
    retry_count++
    max_retries = 10

    if retry_count > max_retries:
        // 最终回退: 用视觉 KF 姿态 + 零速 + 零偏置 + 默认重力方向
        // 后续 VIO 会自行修正 (可能前几十帧精度差)
        fallbackInitialization()
        return FALLBACK_INITIALIZED

    switch failure_mode:
        case F1:  // 激励不足
            waitForMoreData(2.0)  // 多等 2 秒
            → 重试 STATIC_CHECK

        case F2, F3:  // 偏置或重力不一致
            clearOldestKF()       // 丢弃最老 KF，加新 KF
            → 重试 DYNAMIC_INIT

        case F4:  // 优化不收敛
            addStrongerPrior(Rwg)  // 加强重力先验
            → 重试 DYNAMIC_INIT

        case F5, F6:  // 矩阵条件数大 / 纯旋转
            waitForTranslation(0.2)  // 等待 0.2m 平移
            → 重试 DYNAMIC_INIT

        case F7:  // 前端丢失
            resetAll()
            → UNINITIALIZED

        case F8:  // IMU 缺失
            visualOnlyFallback()
            → VISUAL_ONLY_MODE
```

### 7.3 回退初始化

当所有重试均失败时，执行回退初始化：

```cpp
// 对所有已收集的 KF，用前端姿态 + 零速 + 零偏置 + 默认重力
// 不添加速度/偏置/重力先验 → 让后续 VIO 自行收敛
// 代价: 前 20-30 帧轨迹精度可能较差，但不会完全失败
values.insert(X(0), frontend_poses[0]);
values.insert(V(0), Vector3::Zero());
values.insert(B(0), ConstantBias::Zero());
graph.add(PriorFactor<Pose3>(X(0), frontend_poses[0], weak_pose_noise));
graph.add(PriorFactor<Vector3>(V(0), Vector3::Zero(), strong_vel_noise)); // 弱姿态但强零速约束
```

---

## 8. 参数推荐表（完整）

### 8.1 静态初始化参数

| 参数 | 符号 | 推荐值 | 范围 | 来源 |
|------|------|--------|------|------|
| IMU 收集窗口 | `init_window_time` | **1.0 s** | 0.5-3.0 s | OpenVINS (1.0), EuRoC (2.0) |
| 静止加速度阈值 | `init_imu_thresh` | **1.0 m/s²** | 0.5-2.0 m/s² | OpenVINS (1.0), EuRoC (1.5) |
| 重力幅值容差 | `gravity_tolerance_static` | **0.1 m/s²** | 0.05-0.5 m/s² | VINS-Fusion 用 0.5, 静态可更严 |
| 最大陀螺偏置 | `max_gyro_bias` | **0.1 rad/s** | 0.05-0.2 rad/s | 工程经验 |
| 姿态先验 (roll/pitch) | σ | **0.05 rad (~2.9°)** | 静态 | Kimera-VIO 用 10°, 静态可更紧 |
| 姿态先验 (yaw) | σ | **0.001 rad (~0.06°)** | — | 固定 gauge freedom |
| 位置先验 | σ | **0.01 m** | — | 固定世界原点 |
| 速度先验 (静态) | σ | **0.01 m/s** | — | OpenVINS |

### 8.2 动态初始化参数

| 参数 | 符号 | 推荐值 | 范围 | 来源 |
|------|------|--------|------|------|
| 最少关键帧数 | `kMinInitKFs` | **8** | 5-15 | VINS-Fusion=10, ORB-SLAM3=10 |
| 最少时间跨度 | `kMinInitTime` | **1.0 s** | 0.5-3.0 s | ORB-SLAM3 stereo=1.0s |
| 陀螺偏置收敛 | `bg_convergence` | **1e-4 rad/s** | 连续 3 次迭代 | Kimera-VIO residuals=5e-2 的 1/500 |
| GN 最大迭代 | — | **50** | 20-100 | ORB-SLAM3=200, 双目简化后减半 |
| 重力幅值容差 (动态) | `gravity_tolerance_dyn` | **0.3 m/s²** | 0.1-0.5 m/s² | 比静态宽松 |
| 最大平均 χ² | `max_avg_chi2` | **2.0** | 1.0-5.0 | ORB-SLAM3 chi² 阈值思路 |
| 姿态先验 (roll/pitch) | σ | **0.17 rad (~10°)** | 动态 | Kimera-VIO 默认 |
| 速度先验 (动态) | σ | **1.0 m/s** | 弱先验 | — |
| 偏置先验 (bg) | σ | **0.02 rad/s** | 中等先验 | — |
| 偏置先验 (ba) | σ | **0.2 m/s²** | 弱先验 | — |
| 重试最多次数 | `max_retries` | **10** | 5-20 | 工程经验 |

### 8.3 IMU 噪声参数（与前端一致）

| 参数 | 符号 | 推荐值 | 来源 |
|------|------|--------|------|
| 陀螺噪声密度 | `GYR_N` | **1.92e-4** rad/s/√Hz | ADIS16448 (EuRoC) |
| 加速度计噪声密度 | `ACC_N` | **1.83e-3** m/s²/√Hz | ADIS16448 (EuRoC) |
| 陀螺随机游走 | `GYR_W` | **4.0e-6** rad/s²/√Hz | ADIS16448 (EuRoC) |
| 加速度计随机游走 | `ACC_W` | **2.0e-4** m/s³/√Hz | ADIS16448 (EuRoC) |
| 重力幅值 | `GRAVITY_MAG` | **9.81007** m/s² | 苏黎世 (EuRoC) |

---

## 9. 与前端和后端的显式接口契约

### 9.1 Frontend → 初始化子系统的接口

```cpp
// 初始化子系统向前端请求的数据
struct InitRequest {
    // 请求类型
    enum Type { NEED_MORE_IMU, NEED_MORE_KF, READY_STATIC, READY_DYNAMIC };
    Type type;
    double min_imu_duration;        // 需要的 IMU 持续时长 (秒)
    int min_kf_count;               // 需要的关键帧数
    double min_time_span;           // 需要的时间跨度 (秒)
};

// 前端向初始化子系统提供的数据
struct InitInput {
    // IMU 数据缓冲
    std::deque<ImuMeasurement> imu_buffer;  // [(timestamp, acc3, gyr3), ...]

    // 关键帧队列 (在初始化期间不传给后端做优化)
    std::deque<KeyframeSnapshot> kf_snapshots;

    // 每个关键帧的快照
    struct KeyframeSnapshot {
        uint64_t kf_id;
        double timestamp;
        Pose3 T_world_imu;                          // 前端估计的粗位姿
        std::vector<StereoLandmarkObs> landmarks;    // 本 KF 观测到的立体路标
        std::shared_ptr<PreintegratedCombinedMeasurements> pim_from_prev;  // 与前一个 KF 的预积分
        std::vector<std::tuple<double, Vector3, Vector3>> raw_imu_window;   // 原始 IMU 窗口
    };

    struct StereoLandmarkObs {
        uint64_t lmk_id;
        Vector3 pinhole_3d_camera;           // 立体三角化 3D 点 (相机系)
        Vector3 pinhole_3d_world;            // 立体三角化 3D 点 (世界系，用 KF 粗位姿)
        StereoPoint2 stereo_measurement;     // (uL, uR, v) 双目观测
        double stereo_ncc_score;             // NCC 匹配质量
        double stereo_depth;                 // 深度 (m)
        int track_length;                    // 跟踪长度
    };
};
```

### 9.2 初始化子系统 → 后端的接口

```cpp
// 初始化完成后的输出
struct InitOutput {
    bool success;
    InitMode mode;                     // STATIC 或 DYNAMIC 或 FALLBACK

    // 初始因子图
    NonlinearFactorGraph initial_graph;
    Values initial_values;

    // 首帧信息
    Pose3 T_world_imu_0;               // 首帧世界系位姿
    Vector3 v_world_imu_0;             // 首帧世界系速度
    ConstantBias imu_bias_0;           // 首帧偏置估计

    // 后续 KF 数据 (前端已收集但未优化的 KF)
    std::deque<InitInput::KeyframeSnapshot> pending_kfs;

    // 路标数据 (所有从立体三角化得到的初始 3D 点)
    struct InitialLandmark {
        uint64_t lmk_id;
        Point3 position_world;         // 世界系 3D 坐标
        std::vector<std::pair<uint64_t, StereoPoint2>> observations;  // (kf_id, 观测)
    };
    std::vector<InitialLandmark> landmarks;

    // 诊断信息
    struct Diagnostics {
        double gravity_error;          // |g_est - g_true| (m/s²)
        double gyro_bias_norm;         // |bg_est| (rad/s)
        double avg_chi2;               // 平均每因子 χ²
        int n_kfs_used;                // 使用的关键帧数
        double time_span;              // 时间跨度 (s)
        int retry_count;               // 重试次数
        double init_duration;          // 总初始化耗时 (s)
    } diag;
};
```

### 9.3 初始化子系统 → Landmark Pipeline 的接口

```cpp
// 初始化期间的路标生成指令
// 关键约束: 全部显式 Point3 + GenericStereoFactor3D，禁止 SmartFactor
struct InitLandmarkDirective {
    bool is_initialization_phase;      // true → 强制显式路标模式
    int init_kf_remaining;             // 剩余 KF 数（初始化结束倒计时）

    // 对每个新路标:
    //   - 禁用 SmartFactor 创建
    //   - 直接以 Point3 注入 Values
    //   - 以 GenericStereoFactor3D 注入因子图
    //   - 添加弱 Point3 先验 (σ=0.5 m，来自立体不确定性)
};
```

### 9.4 初始化后的转换指令

```cpp
// 初始化的 Landmark Pipeline 状态转换
function onInitComplete():
    landmark_pipeline.transitionToNormal() {
        // 1. 所有已存在的 InitialLandmark (显式) 保持不动
        // 2. 新路标 → SmartFactor 试验期
        //    - 创建 SmartStereoProjectionPoseFactor
        //    - 最小观测数 = 3 帧开始收集 (包含自身)
        //    - 达到 4 帧观测 → 检查提升条件
        // 3. 启用 SmartFactor → 显式 Point3 迁移路径
        // 4. 启用后验异常值剔除
    }
```

---

## 10. 初始化管线伪代码（完整入口）

```pseudo
class StereoVIOInitializer:
    state: InitState = UNINITIALIZED
    imu_buffer: deque<ImuMeas>
    kf_snapshots: deque<KeyframeSnapshot>
    retry_count: int = 0

    function onNewFrame(frontend_output, imu_window):
        imu_buffer.append(imu_window)
        
        switch state:
            case UNINITIALIZED:
                if imu_buffer.duration() >= 1.0 and frontend_output.is_keyframe:
                    kf_snapshots.append(snapshotFrom(frontend_output))
                    state = STATIC_CHECK

            case STATIC_CHECK:
                result = staticCheck(imu_buffer)
                if result == STATIC:
                    state = STATIC_INIT
                    init_result = staticInitialize(imu_buffer)
                    if validateStaticResult(init_result):
                        return buildInitOutput(init_result, STATIC)
                    else:
                        state = DYNAMIC_CHECK
                        retry_count++
                elif result == MOVING:
                    state = DYNAMIC_CHECK
                else:  // NEED_MORE_DATA
                    wait

            case DYNAMIC_CHECK:
                if frontend_output.is_keyframe:
                    kf_snapshots.append(snapshotFrom(frontend_output))
                if kf_snapshots.size() >= kMinInitKFs and
                   kf_snapshots.timeSpan() >= kMinInitTime:
                    state = DYNAMIC_INIT

            case DYNAMIC_INIT:
                init_result = dynamicInitialize(kf_snapshots, imu_buffer)
                if validateDynamicResult(init_result):
                    return buildInitOutput(init_result, DYNAMIC)
                else:
                    handleInitFailure(failure_mode, retry_count)
                    if retry_count > max_retries:
                        return fallbackInitOutput()
                    state = STATIC_CHECK   // 重试双路径

            case INITIALIZED:
                // 不应该到达这里 (调用方在上层控制)
                break
```

---

## 11. 对照参考系统总结

| 特性 | VINS-Fusion | ORB-SLAM3 | OpenVINS | Kimera-VIO | **本设计 (双目)** |
|------|------------|-----------|----------|-----------|------------------|
| **视觉初始化** | 5pt E + Global SfM + BA | 复用 ORB 视觉地图 | 特征跟踪 | 视觉 BA (Smart+Between 因子) | **立体 PnP + 三角化** (无需 SfM) |
| **陀螺偏置** | 线性 LDLT | g2o LM (200 its) | 均值 (静态) / 隐式 (动态) | Gaussian 因子图 / AHRS | **线性 A^TA δbg = A^T b** (VINS 方法) |
| **速度+重力** | 线性 LDLT + 切空间 (4 its) | g2o LM (200 its, SO(3) 切空间) | 闭式解 + Ceres MLE | 线性 + 切空间 (4 its) | **GTSAM GN (50 its, SO(3) 切空间)** |
| **尺度** | 联合求解 | VertexScale (1D) | 闭式解 | — | **跳过 (已知基线)** |
| **加计偏置** | 不优化 | 联合优化 (带先验) | 静态估计 / 动态 MLE | 静态估计 | **设为 0 (短窗口，后续收敛)** |
| **静止模式** | 无 | 无 | 双窗口判断 + Gram-Schmidt | 零速假设 | **OpenVINS 双窗口 + 静止后验校验** |
| **优化后端** | Ceres (SfM) + LDLT | g2o LM | Eigen + Ceres MLE | GTSAM GN/LM | **GTSAM GN/LM (与后端一致)** |
| **路标类型** | 逆深度 (BA 中) | ORB MapPoint | 显式 3D (动态) / 无 (静态) | SmartFactor (BA) | **全部显式 Point3 + GenericStereoFactor** |

---

## 12. Agent 实现提示

### 适用场景

- 从零实现双目 VIO 系统的初始化子系统
- 需要参考 OpenVINS (静态)、ORB-SLAM3 (非线性优化)、Kimera-VIO (在线对齐)、VINS-Fusion (立体简化) 的设计模式
- 需要在 GTSAM ISAM2 因子图框架下集成初始化 → 正常 VIO 的转换

### 输入输出契约

- **输入**: `queue<KeyframeSnapshot>` (前端产出的 KF 快照)、`deque<ImuMeas>` (IMU 原始数据)、`StereoCalibration` (双目标定)
- **输出**: `InitOutput` (含初始 `Values`, `NonlinearFactorGraph`, 路标列表, 诊断信息)
- **关键约束**: 初始化期间禁止 SmartFactor，全部路标为显式 Point3 + GenericStereoFactor3D

### 实现骨架（伪代码）

见 §10 完整入口伪代码和 §3-4 的算法伪代码。

### 关键源码片段

**OpenVINS: 静止检测 + Gram-Schmidt 重力估计**:
`raw/codes/open_vins/ov_init/src/static/StaticInitializer.cpp:L73-L131` — 加速度方差计算、静止判定、Gram-Schmidt 旋转矩阵构建。

**ORB-SLAM3: 重力方向粗估计**:
`raw/codes/ORB_SLAM3/src/LocalMapping.cc:L1228-L1254` — 从 IMU 速度积分累加重力方向，轴角对齐到 (0,0,-1)。

**ORB-SLAM3: IMU 边残差定义 (EdgeInertialGS::computeError)**:
`raw/codes/ORB_SLAM3/src/G2oTypes.cc:L617-L640` — 旋转、速度、位置的 9-DOF 残差，含尺度、重力方向、偏置的全部 Jacobian。

**Kimera-VIO: 高斯因子图陀螺偏置估计**:
`raw/codes/Kimera-VIO/src/initial/OnlineGravityAlignment.cpp:L252-L290` — 构建 `GaussianFactorGraph`，最小化帧间旋转误差。

**VINS-Fusion: 线性陀螺偏置校准**:
`raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L14-L47` — 线性最小二乘 δbg 求解 + 重传播。

**VINS-Fusion: 立体路径跳过 SfM**:
`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L480-L506` — stereo+IMU 路径直接做 PnP+三角化+陀螺偏置+非线性优化。

### 实现注意事项

1. **双目消除尺度的关键在于所有 3D 坐标都是绝对的**：立体三角化的 `Point3` 直接以米为单位，不需要 `s` 缩放的中间步骤。

2. **静态初始化的质量高度依赖静止时长**：1.0s 是最小值；如果条件允许，等待 2.0s 能显著提高偏置估计精度（IMU 的 in-run bias stability 通常在秒级才能体现）。

3. **动态初始化的 KF 选择至关重要**：连续的纯旋转 KF 会产生病态线性系统——必须确保至少前 3 个 KF 之间有 ≥ 0.2m 的平移。

4. **偏置共享策略避免过早发散**：前 3 个 KF 共享同一个 `B(0)` 变量，避免偏置在约束不足时剧烈波动。第 4 个 KF 开始才引入逐帧偏置的 random walk 模型。

5. **重力 2-DOF 参数化是标准做法**：SO(3) 切线空间固定绕 z 轴旋转为 0（`Exp(dx, dy, 0)`）。错误的 3-DOF 更新会导致重力方向的 yaw 分量与偏航姿态耦合。

6. **初始化失败不应阻塞系统**：回退初始化（弱先验 + 零速 + 零偏置 + 默认重力）是最后防线——后续 VIO 可以在 20-30 帧内自行收敛偏置和重力。

7. **因子图构建时 Point3 先验的 σ 值**：0.5m 对应典型的立体深度不确定性（在 5m 距离处，1px 视差误差 → ~0.05m 深度误差；在 15m 处 → ~0.45m）。近处点可以缩小 σ 到 0.1m。

### 源码检索锚点

| 系统 | 文件 | 函数/位置 | 行号 |
|------|------|-----------|------|
| OpenVINS | `StaticInitializer.cpp` | `initialize()` — 全过程 | L37-L164 |
| OpenVINS | `StaticInitializer.cpp` | 加速度方差 + 静止判定 | L73-L119 |
| OpenVINS | `StaticInitializer.cpp` | Gram-Schmidt 重力对齐 | L122-L125 |
| OpenVINS | `InertialInitializer.cpp` | 模式切换 (静态 vs 动态) | L79-L158 |
| OpenVINS | `helper.h` | `gram_schmidt()` | L138-L171 |
| ORB-SLAM3 | `LocalMapping.cc` | `InitializeIMU()` 入口 | L1173-L1319 |
| ORB-SLAM3 | `LocalMapping.cc` | 重力方向粗估计 | L1228-L1254 |
| ORB-SLAM3 | `Optimizer.cc` | `InertialOptimization` (全变量) | L3042-L3224 |
| ORB-SLAM3 | `G2oTypes.h` | `GDirection::Update` (SO(3) 切空间) | L257-L293 |
| ORB-SLAM3 | `G2oTypes.cc` | `EdgeInertialGS::computeError()` | L617-L640 |
| ORB-SLAM3 | `G2oTypes.cc` | 重力 Jacobian `dGdTheta` | L661-L666 |
| Kimera-VIO | `OnlineGravityAlignment.cpp` | 陀螺偏置 (高斯因子图) | L252-L290 |
| Kimera-VIO | `OnlineGravityAlignment.cpp` | 线性重力对齐 | L426-L495 |
| Kimera-VIO | `OnlineGravityAlignment.cpp` | 重力精化 (切空间 4 次) | L534-L594 |
| Kimera-VIO | `InitializationBackend.cpp` | BA+重力对齐编排 | L46-L143 |
| Kimera-VIO | `VioBackend.cpp` | 先验因子 + sigma 值 | L1254-L1319 |
| VINS-Fusion | `initial_aligment.cpp` | `solveGyroscopeBias()` | L14-L47 |
| VINS-Fusion | `initial_aligment.cpp` | `LinearAlignment()` (Ax=b) | L135-L207 |
| VINS-Fusion | `initial_aligment.cpp` | `RefineGravity()` (切空间) | L65-L133 |
| VINS-Fusion | `initial_aligment.cpp` | `TangentBasis()` | L50-L63 |
| VINS-Fusion | `estimator.cpp` | stereo+IMU 跳过 SfM | L480-L506 |
| VINS-Fusion | `estimator.cpp` | `visualInitialAlign()` 状态赋值 | L726-L785 |

---

## 13. 相关页面

- [[方法-VIO初始化方法]] — 三种参考系统的初始化管线全景对比
- [[概念-视觉惯性初始化策略]] — 初始化策略分类与失败模式概述
- [[方法-重力精细化]] — 重力方向切空间迭代精化
- [[方法-IMU陀螺偏置初始化]] — VINS-Fusion 与 ORB-SLAM3 的陀螺偏置方案对比
- [[架构-GTSAM iSAM2 双目VIO后端设计]] — 后端 ISAM2 因子图设计 (SmartFactor 管理 + 边缘化)
- [[设计-立体VIO前端管线]] — 前端特征跟踪与 IMU 预积分规格
- [[概念-IMU预积分]] — GTSAM PreintegratedCombinedMeasurements 概念与用法
- [[算法-Kimera-VIO]] — Kimera-VIO 系统概述 (OnlineGravityAlignment 上下文)
- [[算法-ORB-SLAM3]] — ORB-SLAM3 地图与初始化上下文
- [[VIO方案对比]] — 各 VIO 系统横向对比
