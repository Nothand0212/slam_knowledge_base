---
type: entity
tags: [INS, 初始化, GNSS, 对准, 零速检测, 重力调平, 双天线航向, IMU预积分, 地球自转]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/IC-GVINS/
  - raw/codes/OB_GINS/
---

# INS-centric 初始化策略

> 以惯性导航系统（INS）为核心，利用 GNSS 绝对位置和 IMU 测量通过零速检测、重力调平和动态航向估计完成系统初始化的方法，为视觉/GNSS-IMU 融合提供可靠先验。

## 一、方法动机

传统单目 VIO 初始化依赖足够的视差和纹理特征进行对极几何恢复（如 ORB-SLAM3 的 2 平面 + 单应性分解），在弱纹理、近似匀速直行、光照剧烈变化等场景容易退化。INS-centric 初始化策略将 GNSS 和 IMU 的全局信息前置：通过静止检测获取陀螺零偏和横滚/俯仰角，利用 GNSS 位置差分或双天线获取航向，使视觉系统从初始化阶段就获得有物理意义的先验，而非从零开始恢复未知尺度。

IC-GVINS 是 INS-centric 初始化的标杆实现，采用四阶段渐进式初始化。

## 二、初始化状态机

```
  GVINS_ERROR
      │
      ▼
  GVINS_INITIALIZING ───── Zero-velocity detect ────┐
      │     │ (failure)                             │
      │     └──→ 动态航向 fallback ──────────────────┤
      ▼                                              │
  gyro_bias + roll/pitch + heading                   │
      │                                              │
      ▼                                              │
  GVINS_INITIALIZING_INS ── GINS 滑窗优化 (50 iter) ─┤
      │                                              │
      ▼                                              ▼
  GVINS_INITIALIZING_VIO ── 视觉跟踪 + 三角化
      │
      ▼
  GVINS_TRACKING_INITIALIZING ── 关键帧累积至窗口满
      │
      ▼
  GVINS_TRACKING_NORMAL ── 正常 GNSS-INS-Visual 融合
```

源: `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.h:L47-L55`

## 三、阶段一：静基座粗对准（Coarse Alignment）

### 3.1 零速检测

利用一段 IMU 缓存数据（通常 2 秒）进行零速检测：

```pseudo
function detectZeroVelocity(imu_buffer, imu_rate):
    // 计算陀螺和加速度计的均值与方差
    gyro_mean = mean(imu.dtheta) * imu_rate     // rad/s
    acc_mean  = mean(imu.dvel)   * imu_rate     // m/s²
    gyro_std  = std(imu.dtheta)  * imu_rate
    acc_std   = std(imu.dvel)    * imu_rate

    // 阈值判定
    if gyro_norm < GYRO_THRESHOLD and acc_std < ACC_THRESHOLD:
        return TRUE, [gyro_mean, acc_mean]
    return FALSE
```

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L584-L626`

### 3.2 陀螺零偏估计

当检测到静止时，陀螺增量角速度的均值直接作为初始零偏：

$$b_g = \frac{1}{N} \sum_{k=1}^{N} \frac{\Delta \boldsymbol{\theta}_k}{\Delta t_k} \quad \text{(rad/s)}$$

陀螺零偏质量直接影响后续所有预积分质量。IC-GVINS 要求 IMU 缓存至少 20 个样本（200Hz 下约 0.1s），且使用重力调平前必须先获得有效的陀螺零偏。

### 3.3 重力调平（Roll/Pitch 估计）

利用加速度计比力测量在静止时的投影关系估计横滚和俯仰角。在 NED 导航系下，静止时加速度计测量为：

$$\mathbf{f}^b = \mathbf{R}_n^b \cdot [0, 0, g]^T$$

其中 $g$ 为当地重力加速度（由纬度查表获得），展开 ZYX 旋转矩阵：

$$f_x^b = -g \cdot \sin\theta$$
$$f_y^b = g \cdot \sin\phi \cos\theta$$
$$f_z^b = g \cdot \cos\phi \cos\theta$$

反解：

$$\phi = -\arcsin\left(\frac{f_y^b}{g}\right), \quad \theta = \arcsin\left(\frac{f_x^b}{g}\right)$$

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L617-L620` 和 `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L46-L53`

俯仰角的符号约定与重力定义（NED 下重力为正 Z 方向）密切相关，实现中 $initatt[0] = -\text{asin}(f_y/g)$ 体现了 NED 坐标系特有的翻转。

### 3.4 航向估计

航向是 INS 初始化中最难获取的参数，因为重力矢量提供的约束限定在俯仰和横滚平面。IC-GVINS 按优先级降级策略获取：

1. **双天线 GNSS 航向优先**：如果 GNSS 提供航向及有效标志（`isyawvalid`），直接采用
2. **GNSS 位置差分为备选**：$\psi = \text{atan2}(\Delta E, \Delta N)$，要求相邻两秒 GNSS 位置差的模 > 0.5 m/s
3. **静止时无法获取航向**：零速时无速度方向信息，只有当存在水平运动时才能从 GNSS 轨迹推断航向

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L630-L647` 和 `L128`（`MINMUM_ALIGN_VELOCITY = 0.5`）

## 四、阶段二：INS 机械编排初始化

粗对准完成后，需要将 IMU 数据从初始时刻开始重积分以获得当前 INS 状态。IC-GVINS 保留最近的 INS 窗口数据（`reserved_ins_num_ = 2`，共保留 2 个 GNSS 秒窗口长度的 INS 历史），通过 `redoInsMechanization` 使用更新后的零偏和姿态重新编排：

```pseudo
function redoInsMechanization(config, state, reserved_num, ins_window):
    for each ins_pair in ins_window:
        imu_pre, imu_cur = ins_pair
        // 补偿地球自转
        iewn = Earth.iewn(origin, state.p)  // NED 下投影
        corrected_gyro = imu_cur.dtheta/dt - Cnb * iewn - state.bg
        // 速度更新
        state.v += (Cnb * corrected_accel - cross(2*iewn+enwn, state.v) + gravity) * dt
        // 位置更新
        state.p += state.v * dt
        // 姿态更新（四元数）
        state.q *= exp(0.5 * corrected_gyro * dt)
```

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L277` — 调用 `MISC::redoInsMechanization()`

地球自转补偿项 $\boldsymbol{\omega}_{ie}^n$ 在初始化阶段通过 `Earth::iewn()` 计算，依赖当前纬度：

$$\boldsymbol{\omega}_{ie}^n = [\omega_e \cos\varphi, 0, -\omega_e \sin\varphi]^T$$

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L229-L236`

## 五、阶段三：GINS 滑窗优化初始化

在 GNSS/INS 节点积累到初始长度（默认通过 `initlength` 配置）后，执行一轮纯 GNSS-INS 优化以精化初始状态：

- **优化因子**：GNSS 位置因子 + IMU 预积分因子（不含视觉）
- **迭代次数**：50 次 LM 迭代
- **收敛判定**：`summary.termination_type == ceres::CONVERGENCE`
- **状态**：此时视觉系统尚未初始化，仅积累 GNSS 节点

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L694-L722`（`gvinsInitializationOptimization`）

OB_GINS 的初始化更为简化——直接从配置文件读取 `initvel`、`initatt`、`initbg`、`initba`，不做自动估计：

源：`raw/codes/OB_GINS/src/ob_gins.cc:L91-L101`

## 六、阶段四：视觉系统初始化

当 GINS 初始化完成后，视觉前端开始利用 INS 先验位姿进行特征跟踪。关键差异在于：

- 传统 VIO：从零开始估计地图点深度（无尺度先验）
- INS-centric：已知 IMU 在世界系下的位姿和速度，视觉只需估计特征点深度，本质上退化为已知位姿的三角化

这使视觉初始化更快、更鲁棒，且初始尺度天然与 GNSS 一致。

## 七、与 VIO-centric 初始化的对比

| 维度 | INS-centric（IC-GVINS） | VIO-centric（ORB-SLAM3/DM-VIO） |
|------|----------------------|------------------------------|
| 静止需求 | 零速检测，1-2s 静止 | 不强制，但推荐 |
| 运动需求 | 直线运动即可（GNSS 差分航向） | 需要足够视差 |
| 尺度来源 | GNSS 绝对位置 | 需要额外传感器或 IMU 恢复 |
| 退化场景 | GNSS 多径、杆臂错误 | 弱纹理、匀速直行 |
| 初始化时间 | 1-3 GNSS 秒 + 3-5 视觉帧 | 可能 5-10s |
| 全球参考 | 天然对齐 WGS84 | 局部坐标系，需外部对齐 |

## 八、OB_GINS 的手动初始化

OB_GINS 面向离线后处理，初始化参数由用户通过 YAML 配置文件直接指定：

```yaml
initvel: [0.0, 0.0, 0.0]           # 初始 NED 速度 (m/s)
initatt: [0.5, 0.3, 45.0]          # 初始姿态 RPY (deg)
initgb: [0.01, 0.02, -0.005]       # 初始陀螺零偏 (deg/hr)
initab: [100, -200, 50]            # 初始加表零偏 (mGal)
```

这种设计假设用户已有先验知识（如静止开始、已知姿态），适合数据集后处理但对自动化不友好。

源：`raw/codes/OB_GINS/src/ob_gins.cc:L188-L199`

## Agent 实现提示

### 适用场景

- GNSS+IMU+单目相机车载系统，启动阶段有短暂静止（等红绿灯即可满足）
- 不支持或不想依赖纯视觉初始化的弱纹理场景（高速公路、隧道入口）
- 需要天然全球参考系对齐的户外 SLAM
- 不适用：无法获取 GNSS 的环境、长时间无静止机会、消费级 IMU 零偏噪声过大

### 输入输出契约

- **输入**:
  - IMU: dtheta(rad)、dvel(m/s)、dt(s)，至少 20 个连续样本
  - GNSS: BLH(rad, rad, m)、航向有效性标志、双天线航向(rad)
  - 配置: 零速检测阈值、最小对齐速度(0.5 m/s)、初始窗口长度、重力常量
  - 杆臂: IMU→天线 (body 系, m)
- **输出**:
  - 初始状态: p(NED, m)、q(b→n 四元数)、v(NED, m/s)、bg(rad/s)、ba(m/s²)
  - 诊断: 零速检测标志、航向来源(双天线/GNSS差分/未知)
- **坐标系**: body 系 = 前右下(FRD)，导航系 = 站心 NED，重力 = NED Z 轴正向

### 实现骨架（伪代码）

```pseudo
function ins_centric_initialize(imu_buffer, gnss_history, config):
    // 阶段1: 零速检测 + 粗对准
    imu_window = slice(imu_buffer, last_2_gnss_epochs)

    is_static, avg = detectZeroVelocity(imu_window, config.imu_rate)
    gyro_thresh = 0.02  // rad/s 量级
    acc_thresh  = 0.05  // m/s² 量级

    if is_static:
        // 陀螺零偏: 静止时陀螺输出即为零偏
        bg = [avg[0], avg[1], avg[2]] / imu_rate

        // 重力调平: 加速度计测量 = 重力投影
        fb = [avg[3], avg[4], avg[5]] / imu_rate
        roll  = -asin(fb[1] / config.gravity)   // NED 特有符号
        pitch =  asin(fb[0] / config.gravity)

        has_zero_vel = true
    else:
        pitch = atan(-Δz / sqrt(Δx² + Δy²))  // GNSS 高程差分

    // 航向: 三选一降级策略
    if gnss.is_yaw_valid:
        yaw = gnss.yaw  // 双天线航向最优
    elif gnss_delta_norm > config.min_align_vel:
        ΔN, ΔE = gnss_curr.blh - gnss_prev.blh  (local NED)
        yaw = atan2(ΔE, ΔN)
    else:
        return FAIL  // 无法获取航向

    // 阶段2: 构建初始状态
    init_q = euler2quaternion([roll, pitch, yaw])
    // 杆臂补偿: 从GNSS天线位置反推IMU位置
    init_p  = gnss_prev.blh - init_q.toRotationMatrix() * lever_arm
    init_v  = [0, 0, 0]
    state_0 = {time: gnss_prev.time, p: init_p, q: init_q,
               v: init_v, bg: bg, ba: [0,0,0]}

    // 阶段3: 地球自转补偿 - 初始化积分环境
    if config.with_earth:
        origin = config.origin  // 站心原点
        gravity_vec = [0, 0, config.gravity]
        iewn = Earth.iewn(origin, init_p)  // [we*cosφ, 0, -we*sinφ]

    // 阶段4: 重新编排INS机械编排
    state = state_0
    for each (imu_pre, imu_cur) in ins_history:
        dt = imu_cur.dt
        dtheta = imu_cur.dtheta - bg * dt
        if config.with_earth:
            dtheta -= Cnb.transpose() * iewn * dt  // body系补偿
        dvel = imu_cur.dvel - ba * dt

        state.v += (Cnb * dvel - cross(2*iewn + enwn, state.v) + gravity_vec) * dt
        state.p += state.v * dt
        state.q = unit(state.q * quat_exp(dtheta))
        Cnb = state.q.toRotationMatrix()

    // 阶段5: GINS滑窗优化精化
    for iter in range(50):
        problem = CeresProblem()
        for node in state_window:
            problem.add_parameter(pose[node])
            problem.add_parameter(mix[node])
        for gnss in gnss_window:
            problem.add_residual(GnssFactor(gnss, lever), HuberLoss, pose[idx(gnss)])
        for k in range(window_size-1):
            problem.add_residual(PreintFactor(preint[k]), null,
                                pose[k], mix[k], pose[k+1], mix[k+1])
        solver.Solve(problem)
        if converged: break

    return state_0, state_refined, init_diagnostics
```

### 关键源码片段

**a) IC-GVINS 完整初始化函数**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L584-L691` — `gvinsInitialization()` 函数完整实现了：IMU 缓存采集 → 零速检测 → 陀螺零偏估计 → 重力调平 → 航向获取（双天线/GNSS差分 fallback）→ 初始状态构造 → 先验构造 → 重力与地球自转参数初始化 → INS 重编排 → GNSS 节点添加。注意 `L658` 状态从 `last_gnss_.time` 开始而非 `gnss_.time`，是从上一秒开始为后续预积分预留时间。

**b) 零速检测的门槛参数**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L599` — IMU 缓存至少 20 个样本（200Hz 下 ~0.1s）；`L636-L637` — 最小对齐速度 `MINMUM_ALIGN_VELOCITY = 0.5 m/s`，即相邻 GNSS 位置差分模需要足够大才能可靠估计航向。

**c) 重力常量计算**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L46-L53` — WGS84 重力模型 `gravity(blh)`：根据纬度 $\varphi$（blh[0]）和高程 $h$（blh[2]）计算当地重力 $g = 9.7803267715 (1 + 0.0052790414 \sin^2\varphi + \cdots)$。该值与标准 9.80 的差异在高精度定位（厘米级）中不可忽略。

### 实现注意事项

1. **静止判定不能仅靠陀螺**：车辆怠速时陀螺在零附近但并非真正静止，应加加速度计方差联合判断。IC-GVINS 的 `detectZeroVelocity` 同时检测陀螺和加速度计的统计特性。
2. **航向初始化优先级**：双天线 GNSS 航向 > GNSS 位置差分航向。双天线航向精度可达 0.1°（与基线长度相关），位置差分航向依赖速度和轨迹曲率，精度约为 1-5°。
3. **重力调平的符号约定**：NED 下重力沿 Z 轴正向（$\mathbf{g}^n = [0, 0, g]^T$），因此 $\mathbf{f}^b = \mathbf{C}_n^b \cdot [0, 0, g]^T$。横滚 $\phi = -\arcsin(f_y/g)$ 中的负号来自 NED 的 Z 轴方向与加速度计输出方向的关系。如果使用 ENU 必须翻转 Z 分量。
4. **IMU 缓存大小**：`MAXIMUM_INS_NUMBER = 1000`（200Hz 下约 5s）。零速检测需要足够历史数据，但过多数据会增加重编排时间。
5. **先验构造**：`constructPrior(is_zero_velocity)` 在初始阶段构造状态先验因子（`ImuPosePriorFactor` 和 `ImuMixPriorFactor`），零速时先验权重更强，动态时放松约束。源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L671`
6. **动态初始化的退化**：如果启动时车辆一直在运动且 GNSS 信号差，`is_has_zero_velocity=false` 使横滚/俯仰估计降级为近似值，初始航向完全依赖 GNSS 差分。此时初始 INS 状态质量较差，GINS 滑窗优化需要更多迭代才能收敛。

### 源码检索锚点

- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L584-L691` — `gvinsInitialization()` 完整初始化
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L694-L722` — `gvinsInitializationOptimization()` GINS 滑窗优化
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.h:L47-L55` — GVINSState 枚举（初始化状态机）
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/imu_pose_prior_factor.h:L27-L73` — 姿态先验因子
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/imu_error_factor.h` — IMU零偏先验因子
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L46-L53` — 重力模型
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L229-L236` — 地球自转角速度 iewn
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/rotation.h:L90-L94` — euler2quaternion (ZYX)
- `raw/codes/OB_GINS/src/ob_gins.cc:L91-L101` — OB_GINS 手动初始化
- `raw/codes/OB_GINS/src/ob_gins.cc:L188-L207` — 初始状态构造 + 预积分创建

## 相关页面

- 实现于：[[算法-IC-GVINS]]、[[算法-OB_GINS]]
- 对比：[[概念-视觉惯性初始化策略]]
- 核心技术：[[方法-地球自转补偿预积分]]、[[概念-IMU预积分]]
- 坐标系依赖：[[架构-坐标系管理]]
- 粗差剔除：[[方法-Ceres两轮优化粗差剔除]]
- 重力细节：[[方法-重力精细化]]
- GNSS 因子：[[方法-GNSS 位置残差因子]]
- VIO 初始化对比：[[方法-VIO初始化方法]]
