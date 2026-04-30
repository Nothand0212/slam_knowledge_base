# IMU 数据管线横向对比

> 对比 21 个主流 SLAM/VIO 项目的 IMU 数据处理全管线
> VIO (11)：open_vins, vins_fusion, orb_slam3, dm_vio, msckf_vio, schurvins, openmavis, rovio, svo_pro, kimera_vio, dso
> LiDAR-inertial (8)：fast_lio_sam, lio_sam, fusions_slam, r3live, fast_livo2, lvi_sam, cartographer, lightning_lm
> GNSS-inertial (3)：ic_gvins, ob_gins, superodom

---

## 1. 概述与分类

### 1.1 滤波系 (EKF/ESKF/IESKF) vs 优化系 (FGO) 对 IMU 的不同处理范式

| 范式 | 项目 | IMU 角色 | 核心机制 |
|------|------|---------|----------|
| **EKF 传播** | open_vins | 状态预测（Propagator） | IMU 驱动 EKF 预测步，三种积分：离散 / RK4 / ACI^2 |
| **EKF 传播** | msckf_vio | 状态预测 + 线性化 | RK4 名义 + 3 阶矩阵指数误差态，OC-MSCKF 可观测约束 |
| **EKF 传播** | rovio | 状态预测 + 路标运动学 | IMU 同时传播 robotcentric 路标的 bearing vector + depth |
| **EKF 传播** | schurvins | 状态预测（Forward） | RK4 + 三阶 Phi 矩阵，不用预积分 |
| **IESKF 迭代** | fast_lio_sam | 前端 IESKF（FAST-LIO2） | 18 维 IESKF + 反向传播去畸变 |
| **IESKF 迭代** | fusions_slam | 前端 IESKF | 18 维自实现，欧拉积分，无预积分 |
| **IESKF 迭代** | fast_livo2 | 统一 19 维 IESKF 预测 | LIO->VIO 串行链式，共享协方差 |
| **IESKF 迭代** | lightning_lm | IESKF + Anderson Acceleration | 23/24 维名义/误差态，欧拉递推，无预积分 |
| **ESIKF 迭代** | r3live | 双路 ESIKF 预测 | LIO (18 维) + VIO (29+ 维)，双 buffer 并行 |
| **因子图 (FGO)** | vins_fusion | IMU 预积分因子 | `IntegrationBase` 中值积分 15 维 |
| **因子图 (FGO)** | orb_slam3 | IMU 预积分边 | g2o `EdgeInertial`(9 维) + Bias RW 边 |
| **因子图 (FGO)** | dm_vio | IMU 预积分因子 | GTSAM `PreintegratedImuMeasurements` + 延迟边缘化 |
| **因子图 (FGO)** | openmavis | IMU 预积分边 | SE2(3) 精确预积分 (J1/J2) + g2o EdgeInertial |
| **因子图 (FGO)** | kimera_vio | IMU 预积分因子 | GTSAM `CombinedImuFactor` + iSAM2 |
| **因子图 (FGO)** | lio_sam | IMU 预积分因子 | GTSAM `ImuFactor` + ISAM2 |
| **因子图 (FGO)** | lvi_sam | 双因子图 (LIS+VIS) | GTSAM ISAM2 + Ceres 滑窗，松耦合 |
| **因子图 (FGO)** | superodom | IMU 预积分因子 | GTSAM `CombinedImuFactor` + ISAM2 |
| **因子图 (FGO)** | ic_gvins | IMU 预积分因子 | Ceres `PreintegrationFactor`(15 维)，滑窗+边缘化 |
| **因子图 (FGO)** | ob_gins | IMU 预积分因子 | Ceres `PreintegrationFactor`，1s 整秒节点 |
| **纯视觉** | dso | 无 IMU | 纯直接法光度 BA，尺度不可观 |

### 1.2 范式对比

| 方面 | 滤波系 (EKF/IESKF) | 优化系 (FGO) |
|------|-------------------|-------------|
| **IMU 角色** | 系统主线，以 IMU 频率持续传播状态 | 因子图中二元约束，在关键帧间建立预积分 |
| **更新时机** | 事件驱动（图像/点云到达时） | 关键帧频率（1-10 Hz）批量优化 |
| **状态维度** | 固定（15-29 维）+ 滑动窗口位姿 | 随窗口大小变化，但每节点含 pose+vel+bias |
| **线性化** | 传播时固定（需 FEJ/OC 保证一致性） | 每次优化重新线性化（或 FEJ 在边缘化时） |
| **协方差管理** | 显式 EKF 协方差传播+更新 | 预积分累积协方差，边缘化用 Schur complement |
| **零偏估计** | 实时估计，直接用于下一时刻传播 | 一阶 Jacobian 修正，变化大时触发重传播 |

---

## 2. 原始数据规格

| 算法 | 加速度计量程 | 陀螺仪量程 | 采样率 | 数据格式 | ROS 话题 |
|------|------------|-----------|--------|----------|----------|
| **open_vins** | 未硬编码 | 未硬编码 | 100-1000 Hz | `ImuData {t, a[3], w[3]}` | `IMU_TOPIC` |
| **vins_fusion** | 未限定 | 未限定 | 100-200 Hz | `accBuf`/`gyrBuf` deque | `IMU_TOPIC` (队列 2000) |
| **orb_slam3** | 未限定 | 未限定 | 100-200 Hz | `IMU::Point {Vector3f a, Vector3f w, double t}` | `IMU_TOPIC` |
| **dm_vio** | 未限定 | 未限定 | 100-200 Hz | `IMUMeasurement {acc[3], gyr[3], dt}` | 数据集/T265 |
| **msckf_vio** | 未限定 | 未限定 | 100-200 Hz | `sensor_msgs::Imu` -> `imu_msg_buffer` | `IMU_TOPIC` (队列 50/100) |
| **schurvins** | 未限定 | 未限定 | 100-200 Hz | `ImuMeasurement {acc[3], gyr[3], ts}` | `/imu0` |
| **openmavis** | 未限定 | 未限定 | 100-200 Hz | `IMU::Point {a, w, t}` | `imu0/data` |
| **rovio** | 未限定 | 未限定 | 100-400 Hz | `ImuMeasurement {acc(3), gyr(3), ts}` | `IMU_TOPIC` |
| **svo_pro** | 未限定 | 未限定 | 100-200 Hz | `ImuMeasurement {acc(3), gyr(3), ts}` | `/imu0` |
| **kimera_vio** | 未限定 | 未限定 | 200 Hz | `ImuMeasurement {acc(3), gyr(3), ts}` | `IMU_TOPIC` -> `ThreadsafeImuBuffer` |
| **dso** | — | — | — | — | 纯视觉，无 IMU |
| **fast_lio_sam** | 继承 FAST-LIO2 | 继承 FAST-LIO2 | >=200 Hz | 外部 FAST-LIO2 节点 | 不直接接收 |
| **lio_sam** | 未限定 | 未限定 | >=200 Hz | `sensor_msgs::Imu` (9-axis) | `imu_raw` (两节点订阅) |
| **fusions_slam** | 未限定 | 未限定 | >=100 Hz | `sensor_msgs::Imu` -> `ImuType` | `IMU_TOPIC` |
| **r3live** | 未限定 | 未限定 | 200 Hz | `imu_buffer_lio`+`imu_buffer_vio` 双缓冲 | `/imu0` |
| **fast_livo2** | 未限定 | 未限定 | >=200 Hz | `imu_buffer`+`prop_imu_buffer` | `/imu0` |
| **lvi_sam** | 未限定 | 未限定 | 200-400 Hz | `sensor_msgs::Imu` (9-axis) | `imu_raw` |
| **cartographer** | 未限定 | 未限定 | >=100 Hz | `sensor_msgs::Imu` -> `SensorCollator` | 配置 topic |
| **lightning_lm** | 未限定 | 未限定 | >=200 Hz | `IMU {ts, angvel[3], linacc[3]}` | `IMU_TOPIC` |
| **ic_gvins** | 未限定 | 未限定 | 200 Hz | `IMU {time, dt, dtheta[3], dvel[3]}` (FRD) | `/imu0` |
| **ob_gins** | 未限定 | 未限定 | 100-200 Hz | 文本文件: `dtheta[3], dvel[3]`(FRD) | 离线文件读取 |
| **superodom** | 未限定 | 未限定 | 200 Hz | `sensor_msgs::Imu` -> GTSAM | `/imu_raw` (BEST_EFFORT) |

**关键差异**：
- 所有项目均不硬编码 IMU 量程，依赖外部标定或数据集参数
- IC-GVINS/OB_GINS 要求 **FRD (前右下)** 坐标系；其他项目通过 `extRot` 外参矩阵对齐
- r3live 独有**双缓冲零拷贝**机制 (`imu_buffer_lio` + `imu_buffer_vio`)
- OB_GINS 是唯一纯离线系统（读取文本文件）

---

## 3. 标定管线

### 3.1 内参标定

| 算法 | 加速度计 scale | 陀螺仪 scale | 轴未对准 | 在线/离线 |
|------|--------------|------------|---------|----------|
| **open_vins** | `_calib_imu_da`(3) | `_calib_imu_dw`(3) | `_calib_imu_GYROtoIMU`/`_calib_imu_ACCtoIMU` | **在线估计** |
| **vins_fusion** | 无显式 scale 参数 | 无显式 scale 参数 | — | 依赖离线标定 |
| **orb_slam3** | 无 | 无 | — | 依赖离线标定 |
| **dm_vio** | 无显式标定 | 无显式标定 | — | 依赖 GTSAM IMU 噪声参数 |
| **msckf_vio** | 无显式标定 | 无显式标定 | 外参 `R_imu_cam` 在线估计 | 外参在线，内参离线 |
| **schurvins** | 无 | 无 | 外参 jext 被注释掉 | 只做离线标定 |
| **openmavis** | 无 | 无 | 链式推导 `Tlr/Tlsl/Tlsr` | 全部离线 |
| **rovio** | 无 | 无 | IMU-camera 外参在线 (`doVECalibration_`) | 外参在线，内参离线 |
| **svo_pro** | 无 | 无 | 支持 Ceres 后端在线估计 | 外参在线 |
| **kimera_vio** | 无 | 无 | `StereoCamera` 标定 | 全部离线 |
| **fast_lio_sam** | — | — | 由外部 FAST-LIO2 处理 | — |
| **lio_sam** | 无 | 无 | `extRot` + `extRPY` 坐标对齐 | 全部离线 |
| **fusions_slam** | `_imuScale = 9.81/|mean_acc|` | 无 | `T_imu_lidar` Wrapper 层处理 | scale 在线初始化 |
| **r3live** | 无显式，`m_if_acc_mul_G` 配置 | 无 | 相机-IMU 外参在线 + 内参在线 | **在线标定** |
| **fast_livo2** | `IMU_mean_acc_norm` 归一化消除 | 无 | 外参在线 (`extrinsic_est_en_`) | 外参在线 |
| **lvi_sam** | 无 | 无 | 两套外参 (LiDAR+相机) | 全部离线 |
| **cartographer** | 无 | 无 | 配置 YAML 静态外参 | 全部离线 |
| **lightning_lm** | 无 | `cov_acc *= (9.81/|mean|)^2` | `extrinsic_est_en_` 可选 | 外参可选在线 |
| **ic_gvins** | `s[3]`(PPM) 在线 | `sg[3]` 在线 | 外参[8]在线估计 | **在线估计** |
| **ob_gins** | `s[3]`(PPM) 在线 | `sg[3]` 在线 | `antlever` 杆臂，离线 | **在线估计** |
| **superodom** | 无 | 无 | LiDAR-IMU / LiDAR-Camera-IMU 两条外参链 | 全部离线 |

### 3.2 IMU 噪声参数来源

IMU 噪声参数统一来自 YAML 配置文件的 `imu_noises` 节。各项目将其转换为内部协方差矩阵：

```
open_vins:  accel_noise_density, gyro_noise_density, accel_random_walk, gyro_random_walk
vins_fusion: acc_n, gyr_n, acc_w, gyr_w -> diag(ACC_N^2, GYR_N^2) * dt
orb_slam3:  sigma_a, sigma_g, sigma_ba, sigma_bg -> 预积分协方差累积
lio_sam:    imuAccNoise, imuGyrNoise -> I * pow(noise, 2)
ic_gvins:   ARW(deg/sqrt(hr)), VRW(m/s/sqrt(hr)), gbstd(deg/hr), abstd(mGal)
            单位转换: ARW * D2R/60, VRW/60, gbstd * D2R/3600, abstd * 1e-5
fusions_slam: gyr_cov=0.1, acc_cov=0.1, b_gyr_cov=0.0001, b_acc_cov=0.0001 (NCLT)
lightning_lm: gyr_cov=0.1, acc_cov=0.1, b_gyr_cov=0.0001, b_acc_cov=0.0001 (NCLT)
```

**注意**：
- ic_gvins/ob_gins 硬编码了 `IMU_GRY_BIAS_STD = 7200 deg/hr = 2.0 rad/s` 和 `IMU_ACC_BIAS_STD = 20000 mGal = 0.2 m/s^2` 作为零偏先验
- 所有项目都不执行在线 Allan 方差估计
- 无项目做温度补偿

### 3.3 坐标系对齐

| 算法 | 对齐方式 | 关键函数 |
|------|---------|---------|
| **lio_sam** | `imuConverter()` -> 加速度/角速度左乘 `extRot` | `utility.h:252-282` |
| **fusions_slam** | Wrapper 层 `pcl::transformPointCloud(T_imu_lidar)` | `fusion_slam_ros_wrapper.cpp:109-112` |
| **ic_gvins/ob_gins** | 强制 FRD 坐标系，直接读取 | `imuCallback: dtheta = angvel*dt` |
| **fast_livo2** | `extR_Ri` 链: `IMU->LiDAR` 变换 | `IMU_Processing.cpp:521-526` |
| **cartographer** | `ImuTracker` 从加速度计量测估计重力方向 | `imu_tracker.h` |

---

## 4. 信号预处理

| 算法 | 低通滤波 | 陷波滤波 | 异常值剔除 | 备注 |
|------|---------|---------|-----------|------|
| **open_vins** | 无 | 无 | ZUPT 零速检测 | 原始数据直接使用 |
| **vins_fusion** | 无 | 无 | 无显式 IMU 异常剔除 | 依赖视觉外点剔除保护 |
| **orb_slam3** | 无 | 无 | 无 | — |
| **msckf_vio** | 无 | 无 | 前 200 条均值估计 bias | 静止初始化隐含滤波 |
| **dm_vio** | 无 | 无 | 动态 DSO 权重 (`computeDynamicDSOWeight`) | 光度 RMSE 上升时降视觉权重 |
| **fusions_slam** | 无 | 无 | 无 | — |
| **lio_sam** | 无 | 无 | 速度>30m/s 或 bias>1.0 重置 | `failureDetection()` |
| **r3live** | 无 | 无 | 加速度乘 G 配置 | 无额外滤波 |
| **fast_livo2** | 无 | 无 | `IMU_mean_acc_norm` 归一化消除传感器 scale | 加速度归一化 |
| **lightning_lm** | 无 | 无 | 20 帧 IMU 在线均值+Welford 方差估计 | `IMUInit()` |
| **cartographer** | 无 | 无 | 无 | — |
| **ic_gvins** | 无 | 无 | IMU 丢失检测: dt>1.5x周期 -> 插补虚拟 IMU | `ic_gvins.cc:171-183` |
| **ob_gins** | 无 | 无 | 整秒 IMU 线性插值拆分增量 | `ob_gins.cc:577-594` |
| **superodom** | 无 | 无 | 速度>30m/s,bias>2.0/1.0 重置 | `failureDetection()` |

**结论：所有项目均不执行 IMU 信号的数字低通或陷波滤波。**
唯一体现"预处理"概念的操作为：
1. 平均值估计（静止初始化时的零偏/重力估计）
2. 异常监测与重置（速度/bias 过大时触发）
3. IC-GVINS 的虚拟 IMU 插补（丢失数据时的时间一致性维护）

---

## 5. Bias 初始化

| 算法 | 方法 | 采集时长 | gyro bias 初始 | accel bias 初始 | 协方差初始化 |
|------|------|---------|---------------|----------------|-------------|
| **open_vins** | 静态: 静止均值 / 动态: Ceres 优化 | 静态: 数秒 / 动态: `init_window_time` | Ceres 优化 | Ceres 优化 | 先验协方差 |
| **vins_fusion** | `solveGyroscopeBias()` -> IMU预积分旋转 vs SFM视觉旋转 | 滑动窗口帧积累 | A*x=b LDLT | LinearAlignment 之后 | 0 |
| **orb_slam3** | `InertialOptimization()` -> 优化重力+scale+bg+ba+速度 | >=10 KF, >=2s | g2o 非线性优化 | g2o 非线性优化 | 预积分协方差逆 |
| **dm_vio** | `IMUInitializer` -> 速度+bias+重力+scale | 积累足够帧 | 初始化器估计 | 初始化器估计 | GTSAM prior |
| **msckf_vio** | 静止: 前 200 条 IMU 取均值 | 200 条 IMU (~1s) | `sum(w)/N` | 强制为 0 | 21x21 配置协方差 |
| **schurvins** | `InitState()` -> IMU加速度归一化 -> `FromTwoVectors` | 瞬时 | 0 | 0 | diag(1e-4,1e-3,1e-3,2e-3,1e-6) |
| **rovio** | 首次运动时 `addFeaturesByShiTomasi()` | 运动触发 | 隐式 EKF 估计 | 隐式 EKF 估计 | 配置指定 |
| **kimera_vio** | `initializeFromIMU()` -> 平均加速度推断重力方向 | 数秒 | `guessImuBias()` | `guessImuBias()` | 先验因子 |
| **fast_lio_sam** | 由外部 FAST-LIO2: 加速度方向估计重力 | 数秒 | 均值 | 均值 | FAST-LIO2 IESKF |
| **lio_sam** | IMU roll/pitch 初始化 yaw=0 | 首帧 | GTSAM ISAM2 | GTSAM ISAM2 | priorPoseNoise |
| **fusions_slam** | `ImuStaticInit`: 收集1s IMU -> `gravity = -mean/|mean|*9.81` | 1.0s | 0 (配置) | `mean_acc + gravity` | diag(1e-4,1e-3,1e-3,2e-3,1e-6) |
| **r3live** | `ImuProcess` IMU 初始化 (同 FAST-LIO2) | 积累足够 IMU | 估计 | 估计 | `set_initial_state_cov()` |
| **fast_livo2** | `IMU_init()`: Welford 在线均值 -> `gravity = -mean/|mean|*9.81` | 3 帧 IMU | 0 | 0 | `init_P.setIdentity()` |
| **lightning_lm** | `IMUInit()`: Welford 在线均值/方差，**最多 20 帧** | ~0.1s (20/200Hz) | `mean_gyr` | 强制为 0 | diag(1e-4...1e-3...0.0001..) |
| **ic_gvins** | 零速检测 -> `gravity = -mean/|mean|*9.81` -> GNSS/INS 50次LM优化 | 两个GNSS间隔+静止 | GNSS/INS优化 | GNSS/INS优化 | 配置 |
| **ob_gins** | 外部指定 `initvel/initatt/initgb/initab` | 无(配置文件) | 用户提供 `initgb` | 用户提供 `initab` | 配置 |
| **superodom** | IMU: 收集1s -> `Imu::imuInit()` 估计重力方向+bias | 1.0s | 估计 | 估计 | GTSAM prior |

---

## 6. 离散积分方法对比

| 算法 | 积分方法 | 频率 | 精度 | 备注 |
|------|---------|------|------|------|
| **open_vins** | 离散 / RK4 / ACI^2 (三种可选) | IMU 频率 | 中 / 高 / 最高 | 离散: `v += -g*dt+R^T*(a-ba)*dt`; RK4: 四阶; ACI^2: 闭式解析 |
| **vins_fusion** | 中值积分 (Mid-point, 等价 RK2) | IMU 频率 | 中 | `un_acc = 0.5*(un_acc_0 + un_acc_1)` |
| **orb_slam3** | 零阶近似 `J1 ~ dt*I, J2 ~ 0.5*dt^2*I` | IMU 频率 | 低 (高角速度时) | 假设相邻测量间角速度为零 |
| **dm_vio** | GTSAM 标准预积分 | 缓冲区 | GTSAM 内部 | Forster et al. 方法 |
| **msckf_vio** | RK4 + 3 阶矩阵指数近似 | IMU 频率 | 高 | `Phi = I + Fdt + 0.5*Fdt^2 + (1/6)*Fdt^3` |
| **schurvins** | RK4 + 三阶 Phi 矩阵 | IMU 频率 | 高 | `q_new = q * exp(w*dt)`, 直接积分原始 IMU |
| **openmavis** | SE2(3) 精确积分 (J1/J2 角速度耦合) | IMU 频率 | **最高** | `J1 = dt*I + (1-cos)/theta^2*W + (dt*theta-sin)/theta^3*W^2` |
| **rovio** | 欧拉离散 | IMU 频率 | 低 | 直接 `qWM = qWM * exp((gyr-gyb)*dt)` |
| **svo_pro** | 二阶中值预积分 | IMU 缓冲区 | 中 | `delta_t += delta_v*dt + 0.5*delta_R*a*dt^2` |
| **kimera_vio** | GTSAM 预积分 | IMU 缓冲区 | GTSAM 内部 | `PreintegratedImuMeasurements` |
| **fast_lio_sam** | FAST-LIO2 IESKF 传播 | IMU 频率 | 高 | 前向+反向传播去畸变 |
| **lio_sam** | GTSAM 预积分 | IMU 频率 | GTSAM 内部 | `PreintegratedImuMeasurements` |
| **fusions_slam** | **欧拉离散** (非预积分) | IMU 频率 | 低 | `p+=v*dt; v+=(R*(acc-ba)+g)*dt` |
| **r3live** | ESIKF 前向传播 (同 FAST-LIO2) | IMU 频率 | 高 | Rodrigues 公式 + 前向递推 |
| **fast_livo2** | Rodrigues 公式 + 加速度归一化 | IMU 频率 | 高 | `R_k+1 = R_k * Exp(angvel_avr - b_g, dt)` |
| **lvi_sam** | LIS: GTSAM + VIS: 中值预积分 | IMU 频率 | 中 | 两路不同积分方法 |
| **cartographer** | `ImuTracker` 无预积分，仅传感器朝向 | IMU 频率 | 低(仅姿态) | 加速度/陀螺估计重力+朝向 |
| **lightning_lm** | 欧拉离散 (无预积分) | IMU 频率 | 低 | `pos+=vel*dt; rot=rot*SO3::exp((gyro-bg)*dt)` |
| **ic_gvins** | Ceres 预积分 15 维 | 整秒节点间 | 高 | 地球自转可选补偿 |
| **ob_gins** | Ceres 预积分 | 1s 固定间隔 | 高 | 同 ic_gvins，固定步长 |
| **superodom** | GTSAM 预积分 | IMU 频率 | GTSAM 内部 | `CombinedImuFactor` |

---

## 7. 预积分对比（仅 FGO 系）

| 算法 | 参考论文 | 积分方法 | 协方差传播 | 偏置 Jacobian | 地球自转补偿 | 重传播触发 |
|------|---------|---------|-----------|-------------|------------|-----------|
| **vins_fusion** | Forster et al. TRO 2017 | 中值积分 | 15x15 `F * cov * F^T + V*Q*V^T` | `J_p_ba, J_p_bg, J_v_ba, J_v_bg, J_R_bg` | 无 | `repropagate()` (当前 #if 0 禁用) |
| **orb_slam3** | Campos et al. TRO 2020 | 零阶近似 | 9x9 `A*Sigma*A^T + B*Q*B^T` | `dR_db_g, dV_db_g/a, dP_db_g/a` | 无 | 一阶修正 |
| **dm_vio** | Forster et al. 2016 | GTSAM 标准 | GTSAM 内部 | GTSAM 内部 | 无 | 延迟边缘化架构 |
| **openmavis** | SE2(3) 精确 | J1/J2 积分核 | 9x9 (同 orb_slam3 结构) | 9x3/6x3 Jacobian | 无 | `SetNewBias()/Reintegrate()` |
| **kimera_vio** | Forster et al. | GTSAM 标准 | GTSAM 内部 | GTSAM 内部 | 无 | iSAM2 relinearize |
| **lio_sam** | 基于 Forster et al. | GTSAM 标准 | `PreintegrationParams` | GTSAM 内部 | 无 | ISAM2 relinearizeThreshold=0.1 |
| **lvi_sam** | 同上 (双系统) | GTSAM + 中值 | GTSAM 内部 | GTSAM + VINS Jacobian | 无 | — |
| **ic_gvins** | i2Nav 自研 | Ceres 自写 | Cholesky(cov^-1)作为 sqrt_info | 一阶 Taylor 展开 | **可选** (PreintegrationEarth) | bias 变化 >6x std -> `doReintegration()` |
| **ob_gins** | 同 ic_gvins | Ceres 自写 | 同上 | 同上 | **可选** (isearth=true) | 同上 |
| **superodom** | — | GTSAM 标准 | GTSAM 内部 | GTSAM 内部 | 无 | ISAM2 |

### 7.1 残差公式对比

**vins_fusion** (15维):
```
r_P = Q_i^-1 * (P_j - P_i - V_i*dt + 0.5*g*dt^2) - corrected_delta_p    [3]
r_R = 2 * (corrected_delta_q^-1 * (Q_i^-1 * Q_j)).vec()                 [3]
r_V = Q_i^-1 * (V_j - V_i + g*dt) - corrected_delta_v                    [3]
r_Ba = Ba_j - Ba_i                                                         [3]
r_Bg = Bg_j - Bg_i                                                         [3]
```

**orb_slam3** (9维 EdgeInertial):
```
r_R = Log(dR^T * R_wb_i^T * R_wb_j)                 [3: 旋转]
r_V = R_wb_i^T * (V_wb_j - V_wb_i - G*dt) - dV      [3: 速度]
r_P = R_wb_i^T * (P_wb_j - P_wb_i - V_wb_i*dt
                 - 0.5*G*dt^2) - dP                   [3: 位置]
```

**ic_gvins/ob_gins** (15维 Ceres):
```
e_p = R_0^T * (p_1 - p_0 - v_0*dt - 0.5*g*dt^2) - p_corrected
e_v = R_0^T * (v_1 - v_0 - g*dt) - v_corrected
e_q = 2 * (q_corrected^-1 * R_0^T * q_1).vec()
e_bg = bg_1 - bg_0
e_ba = ba_1 - ba_0
```

### 7.2 SE2(3) 精确积分（openmavis 亮点）

openmavis 使用 SE2(3) 预积分 (`src/ImuTypes.cc:206-217`)，在高角速度下显著优于零阶近似：

```
theta = |w| (角速度幅值)
J1 = dt*I + (1-cos(dt*theta))/theta^2 * [w]x + (dt*theta - sin(dt*theta))/theta^3 * [w]x^2
J2 = 0.5*dt^2*I + (dt*theta - sin(dt*theta))/theta^3 * [w]x + (0.5*dt^2*theta^2 + cos(dt*theta) - 1)/theta^4 * [w]x^2

dP += dV*dt + dR*J2*acc       // 精确位置 (vs ORB-SLAM3: dP+=dV*dt+dR*0.5*acc*dt^2)
dV += dR*J1*acc                // 精确速度 (vs ORB-SLAM3: dV+=dR*acc*dt)
```

---

## 8. 滤波传播对比（仅滤波系）

| 算法 | 滤波器类型 | 状态维度(名义) | 误差态维度 | 传播频率 | 线性化方式 | 迭代 |
|------|----------|-------------|-----------|---------|-----------|------|
| **open_vins** | EKF MSCKF | IMU(13)+Clone(6N) | 15+6N+外参 | IMU 频率 | FEJ (First Estimate Jacobian) | 无 |
| **msckf_vio** | EKF MSCKF | IMU(15)+Camera(6N) | 21+6N | IMU 频率 | OC-MSCKF + 3阶矩阵指数 | 无 |
| **rovio** | EKF/IEKF | IMU(15)+外参(6Ncam)+路标(3Nmax)+Pose(6Npose) | 变化 | IMU 频率 | IEKF 重新线性化 | **IEKF 候选生成** |
| **schurvins** | EKF + Schur Complement | IMU(15)+AugState(6*4) | 39 DOF | IMU 频率 | FEJ (quat_fej, pos_fej) | 无 |
| **fusions_slam** | IESKF | [r(3),p(3),v(3),bg(3),ba(3),g(3)] | 18 | IMU 频率 | 迭代更新中重线性化 | **10 次迭代** |
| **r3live** | ESIKF (双路) | LIO:18, VIO:29+ | 18/29+ | IMU 频率 | ESIKF 迭代 | **LIO/VIO 独立迭代** |
| **fast_livo2** | IESKF | [rot(3),pos(3),inv_exp(1),vel(3),bg(3),ba(3),g(3)] | 19 | IMU 频率 | 迭代更新中重线性化 | **金字塔迭代** |
| **lightning_lm** | IESKF + AA | [pos,rot(S3),extR,extT,vel,bg,ba,grav(S2)] | 24 | IMU 频率 | 迭代更新中重线性化 | **4 次 + AA** |

### 8.1 误差状态转移矩阵对比

**msckf_vio**: 3 阶矩阵指数近似
```
Fdt = F * dt
Phi = I + Fdt + 0.5*Fdt^2 + (1/6)*Fdt^3
```
F(21x21) 关键块: F(0:3,0:3)=-[gyro]x, F(0:3,3:6)=-I, F(6:9,0:3)=-R^T*[acc]x, F(6:9,9:12)=-R^T

**schurvins**: 三阶近似
```
F(0,12)=-rot, F(3,6)=I, F(6,0)=-Skew(rot*acc), F(6,9)=-rot
Phi = I + F*dt + 0.5*(F*dt)^2 + (1/6)*(F*dt)^3
```

**fast_livo2**: 一阶欧拉近似
```
F_x = I_19 +
  [0 0 0 0 -I*dt 0 0]    // row 0-2: d(dR)/d(bg)
  [0 0 0 I*dt 0 0 0]     // row 3-5: d(dp)/d(dv)
  [-R*[a]x*dt 0 0 0 0 -R*dt I*dt]  // row 7-9: d(dv)/d(dR),d(ba),d(dg)
```

### 8.2 OC (可观测性约束) 使用情况

| 算法 | OC 实现 | 方法 |
|------|---------|------|
| **msckf_vio** | **OC-MSCKF** (Li Mingyang 2014 TRO) | 转移矩阵 Phi 修正: `Phi(0:3,0:3)=R_k*R_{k-1}^T`, 测量 Jacobian 修正 |
| **schurvins** | FEJ | `quat_fej`, `pos_fej` 状态增广时使用首次线性化点 |
| **fast_livo2** | 通过统一协方差隐式保证 | 单一 IESKF 串行 LIO->VIO |

---

## 9. IMU 因子/残差构建

| 算法 | 残差维度 | 观测模型 | 信息矩阵 | 备注 |
|------|---------|---------|---------|------|
| **open_vins** | 无独立 IMU 残差 (EKF 传播) | 状态转移 + 协方差传播 | Qd = G*Q*G^T*dt | IMU 是预测步，非观测残差 |
| **vins_fusion** | **15** | 预积分残差 (见上文) | P^-1 (预积分协方差逆), 15x15 | `sqrt_info = LLT(cov^-1).matrixL().transpose()` |
| **orb_slam3** | **9** (EdgeInertial) | 预积分残差 + bias RW | Sigma_ij^-1 (9x9) | bias 单独用 EdgeGyroRW/EdgeAccRW (3维) |
| **dm_vio** | 15 (GTSAM ImuFactor) | GTSAM 标准 | GTSAM noiseModel | 经 PoseTransformationFactor 坐标转换 |
| **msckf_vio** | 无独立 IMU 残差 (EKF 传播) | RK4 状态传播 | Q = Phi*G*Qc*G^T*Phi^T*dt | OC-MSCKF 修正 |
| **schurvins** | 无独立 IMU 残差 (EKF 传播) | RK4 + 三阶 Phi | `P = Phi*P*Phi^T + Phi*G*Q*G^T*Phi^T*dt` | 直接积分，非预积分 |
| **openmavis** | **9** (EdgeInertial) | SE2(3) 预积分残差 | Sigma^-1 (9x9) | 特征值裁剪(<1e-12->0) |
| **rovio** | 无独立 IMU 残差 (EKF 传播) | 欧拉传播 | `K = P*F^T*(F*P*F^T+R)^-1` | robotcentric 路标运动学 |
| **svo_pro** | 15 (Ceres `imu_error.hpp`) | 预积分残差 | 预积分累积协方差 | Ceres 自动微分 |
| **kimera_vio** | 15 (CombinedImuFactor) | GTSAM 标准 | GTSAM noiseModel | iSAM2 增量 |
| **lio_sam** | 15 (ImuFactor) | GTSAM 预积分 | `imuIntegratorOpt_->deltaTij()` | LiDAR odom 作为 PriorFactor 注入 |
| **lvi_sam** | LIS:15 + VIS:15 | GTSAM + Ceres | 双系统独立信息矩阵 | 通过 pose.covariance 编码 bias/gravity |
| **fast_lio_sam** | IESKF 无独立因子 | 点到平面 (LiDAR) | `R_inv = 1/(0.001 + sigma_l)` | 由 FAST-LIO2 完成 |
| **fusions_slam** | IESKF 无独立因子 | 欧拉积分 | Q 固定，不自适应 | `K = (H^T*H + (P/0.001)^-1)^-1*H^T` |
| **r3live** | ESIKF 双预测 | LIO:点到平面 / VIO:重投影 | LIO: `P/LASER_POINT_COV`, VIO: `P*m_cam_measurement_weight` | IMU 作为先验约束 `x_propagate - x_current` |
| **fast_livo2** | IESKF 无独立因子 | LIO:点到VoxelMap / VIO:光度 | LIO: `R_inv=1/(0.001+sigma_l)`, VIO: `(P/img_point_cov)^-1` | vec = x_propagate - x_current 做先验 |
| **lvi_sam** | — | — | — | 松耦合，无统一信息矩阵 |
| **cartographer** | 无 IMU 因子 | IMU 仅用于重力估计和外推 | — | ImuTracker 输出姿态而非因子 |
| **lightning_lm** | IESKF 无独立因子 | 点到IVox3d平面 | Cauchy 鲁棒核 + 中位数平方收敛 | `K = (P/R + H^T*H)^-1 * H^T` |
| **ic_gvins** | **15** (Ceres PreintegrationFactor) | 预积分 + IMU bias 先验 | Cholesky(cov^-1) | 地球自转可选 |
| **ob_gins** | **15** (Ceres PreintegrationFactor) | 同上 | 同上 | Z 轴 scale 额外约束 |
| **superodom** | 15 (CombinedImuFactor) | GTSAM 标准 | GTSAM | LiDAR prior factor 注入 ISAM2 |

### 9.1 滤波系 IMU "残差" 本质

滤波系中 IMU 本身没有显式的残差/因子，而是通过以下机制起作用：

1. **状态预测**（预测步）：IMU 以高频将名义状态前向传播
2. **协方差传播**：误差状态协方差在每步 IMU 后增长
3. **先验正则化**（IESKF 中）：`vec = x_propagate - x_current` 将 IMU 传播信息转化为优化中的先验约束
4. **协方差连贯**：观测更新后收缩的协方差被下一轮 IMU 传播继承

---

## 10. 时间同步与偏移

### 10.1 IMU-Camera 时间偏移

| 算法 | 偏移处理 | 在线估计 | 具体实现 |
|------|---------|---------|---------|
| **open_vins** | `_calib_dt_CAMtoIMU` -> `t_imu = t_cam + t_off` | **在线** | 传播时所有时间戳加偏移 |
| **vins_fusion** | `td` (单值) -> `ProjectionTwoFrameOneCamFactor` 一阶校正 | **在线** | 利用特征速度做补偿: `pts_i_td = pts_i - (td-td_i)*velocity_i` |
| **orb_slam3** | 按时间窗口 `[last_frame, curr_frame]` 收集 IMU | 无 | 图像时间戳为主基准 |
| **dm_vio** | `delay_imu_cam` 参数补偿硬件延迟 | 无 | `IMUInterpolator` 线性插值 |
| **msckf_vio** | `timeshift_cam_imu` 配置 (EuRoC=0.0) | 无 | `batchImuProcessing` 批处理 `[last_state.time, image.time]` |
| **schurvins** | `delay_imu_cam` 配置 | 无 | `getMeasurementsContainingEdges()` -> 反序存储 |
| **openmavis** | 按时间窗口收集 | 无 | 图像帧时间为主基准 |
| **rovio** | **天然同步**：IMU 流式处理，图像到达时 EKF 已传播至最新 | **隐式** | 每条 IMU 立即执行传播 |
| **svo_pro** | 无显式变量 | 无 | `ThreadsafeQueue<ImuMeasurement>` 时间窗口对齐 |
| **kimera_vio** | Cross-correlation (`CrossCorrTimeAligner`) | **在线** | IMU预积分角速度 vs 视觉特征运动互相关 |
| **r3live** | `td_ext_i2c` (VIO状态) | **在线** | 图像+LiDAR 通过 `g_camera_lidar_queue` 时间对齐 |
| **fast_livo2** | `imu_time_offset`, `img_time_offset` | **离线配置** | `sync_packages()` 以图像时间戳为切割轴 |
| **ic_gvins** | `td_b_c` (外参一部分) | **在线** | 视觉跟踪时 `frame->stamp() + td` |

### 10.2 IMU-LiDAR 时间同步

| 算法 | 同步方式 | 关键参数 |
|------|---------|---------|
| **lio_sam** | 清除 `timeScanCur - 0.01` 之前的 IMU -> 角速度积分 deskew | `downsampleRate=1` |
| **fusions_slam** | `MeasureGroupAdd` 机制: `map<uint64_t, DataUnit>` ns 时间戳排序 | `cnt < 5 -> return false` (至少5条IMU才同步) |
| **fast_livo2** | `sync_packages()` -> 以 img_capture_time 为界切割 LiDAR 扫描 | `states_: WAIT->LIO->VIO` 状态机 |
| **r3live** | `sync_packages()` -> 取最老 LiDAR 帧 + 结束前所有 IMU | `lidar_in(t+0.1)` 补偿 |
| **fast_lio_sam** | `ApproximateTime` 同步策略 (message_filters) | Odom+PCD 对齐 |
| **lvi_sam** | 各节点独立以 LiDAR 时间戳为基准 | `cloudInfo.odomAvailable` 标志 |
| **lightning_lm** | 等待 IMU 缓冲区最新 > `lidar_end_time_` | `lidar_time_interval=0.1s` |
| **cartographer** | `SensorCollator` -> 按时间排序多种传感器 | `pose_extrapolator` 外推 |
| **superodom** | `MapRingBuffer` 时间窗口对齐 | `meas_start < lidar_start && meas_end >= lidar_end` |
| **ic_gvins** | GNSS-IMU: 动态时间对齐 + INS 速度补偿 | `MINMUM_SYNC_INTERVAL=0.025s` |
| **ob_gins** | 固定整秒节点 + IMU 线性插值拆分 | `INTEGRATION_LENGTH=1.0s` |

### 10.3 插值策略

| 策略 | 使用项目 | 说明 |
|------|---------|------|
| **线性插值 IMU** | open_vins, vins_fusion, dm_vio, kimera_vio, lio_sam, fusions_slam | 在图像/点云边界时刻线性插值生成虚拟 IMU |
| **slerp 姿态插值** | superodom, r3live | 旋转用 slerp，位移用线性 |
| **最近邻查找** | msckf_vio, schurvins | 取时间戳最近的一条 IMU |
| **IMU 增量拆分** | ob_gins | 按时间比例拆分 IMU 角增量/速度增量到整秒节点 |
| **虚拟 IMU 插补** | ic_gvins | 丢失检测：dt > 1.5*周期 -> 插补 |

### 10.4 时间基准选择

| 基准 | 项目 |
|------|------|
| **图像时间戳** | vins_fusion, orb_slam3, dm_vio, msckf_vio, openmavis, schurvins, svo_pro, kimera_vio |
| **LiDAR 时间戳** | lio_sam, lvi_sam, fast_lio_sam, fusions_slam, r3live, fast_livo2, lightning_lm |
| **多传感器混合排序** | cartographer (`SensorCollator`) |
| **GPS 周秒 (SOW)** | ic_gvins, ob_gins |

---

## 11. 设计模式总结

### 11.1 预积分 vs 滤波传播的取舍

| 维度 | 预积分 (FGO 系) | 滤波传播 (EKF/IESKF 系) |
|------|----------------|----------------------|
| **代表项目** | vins_fusion, orb_slam3, lio_sam, ic_gvins | open_vins, msckf_vio, fast_livo2, r3live |
| **计算时机** | 批量（关键帧频率） | 流式（IMU 频率） |
| **线性化** | 优化迭代中多次重线性化 | 传播时一次线性化 (FEJ/OC 保护) |
| **零偏更新** | 一阶 Jacobian 校正, 变化大时重传播 | 实时估计，直接代入下一时刻 |
| **信息损失** | 通过边缘化保留 (Schur complement) | 协方差矩阵显式传播 |
| **工程复杂度** | 中等 (需管理因子图/边缘化/ISAM2) | 高 (需推导 F/G/H 矩阵，手动协方差维护) |
| **适用场景** | 精度优先，中频输出 (1-10Hz) | 速度优先，高频输出 (100-400Hz) |

### 11.2 状态维度选择

| 状态层次 | 典型维度 | 代表性项目 |
|---------|---------|----------|
| **最小 IMU 态** | 15 (p,v,q,bg,ba) | vins_fusion, msckf_vio, schurvins |
| **+重力在线估计** | 18 (+g[3]) | fusions_slam, r3live(LIO), fast_lio_sam |
| **+曝光参数** | 19 (+inv_expo) | fast_livo2 |
| **+外参** | 23/24 (+extR,extT) | lightning_lm |
| **+外参+内参+时间偏移** | 29+ | r3live(VIO) |
| **+scale+sodo+安装角** | 最大 23+ | ic_gvins, ob_gins |
| **因子图节点** | Pose(6/7)+Vel(3)+Bias(6) per KF | lio_sam, lvi_sam, kimera_vio |

### 11.3 初始化的多样性

| 初始化范式 | 项目 | 特点 |
|-----------|------|------|
| **静止 IMU 均值** | msckf_vio, open_vins(静态), fusions_slam | 依赖静止状态，估计重力方向+bias |
| **IMU 前 N 帧在线均值** | fast_livo2(3帧), lightning_lm(20帧), superodom(1s) | 不要求静止，运动中初始化 |
| **SFM + IMU 对齐** | vins_fusion, orb_slam3(单目), lvi_sam(VIS) | 视觉 SFM -> IMU bias/重力/尺度对齐 |
| **GNSS/INS 粗对齐** | ic_gvins | 零速检测 + 重力调平 + GNSS 50次LM |
| **外部指定 (配置文件)** | ob_gins | 用户提供 initvel/initatt/initgb/initab |
| **双目直接三角化** | orb_slam3(双目), openmavis | 首帧双目匹配，无 SFM 初始化的复杂性 |
| **Ceres/GTSAM 非线性初始化** | open_vins(动态), dm_vio | 构造完整因子图做一次性初始化优化 |

### 11.4 核心架构选择矩阵

```
                         状态维度
                           ↑
                    高维度 │ ic_gvins(+scale,+sodo,23)
                           │ lightning_lm(+ext,+S2,24)
                           │ r3live(VIO,+外参,+内参,29+)
                           │
                           │ fast_livo2(+inv_expo,19)
                           │ fusions_slam(+g,18)   schurvins(+clones,39)
                    中等   │ vins_fusion(15)      open_vins(+clones,15+6N)
                           │ orb_slam3(15)        msckf_vio(+clones,21+6N)
                           │ lio_sam(因子节点)
                           │
                    低维度 │ cartographer(仅姿态)
                           │
                           └────────────────────────────────────────→ 耦合度
                             松耦合                      紧耦合
                           lvi_sam    vins_fusion    fast_livo2
                           lio_sam    orb_slam3      r3live
                           superodom  dm_vio         fusions_slam
                           carto-     kimera_vio     msckf_vio
                           grapher    openmavis      open_vins
```

### 11.5 对 phad_fusion 的综合建议

1. **IMU 预积分模块**：参考 vins_fusion 的 `IntegrationBase`（中值积分+协方差+Jacobian 传播）或 ic_gvins 的 Ceres 预积分体系
2. **因子图后端**：使用 GTSAM ISAM2（借鉴 lio_sam/kimera_vio 的模式），或 Ceres 滑窗+边缘化（借鉴 ic_gvins）
3. **初始化策略**：分级推进：IMU 静止/在线初始化 → GNSS 辅助（若有）→ 视觉 SFM 对齐
4. **时间同步**：以 IMU 时间戳为统一基准，通过插值将其他传感器对齐
5. **信号滤波**：当前所有项目都不做数字滤波，phad_fusion 可考虑加入可选的低通滤波以应对高频噪声场景
6. **偏置管理**：采用一阶 Jacobian 在线校正 + 变化超阈值时触发重传播
7. **坐标系**：内部统一使用 NED/ENU 局部切平面，传感器数据通过外参矩阵转换

---

*文档生成时间：2026-04-29*
*数据来源：/home/lin/Projects/lin_ws/slam_ws/docs/deep_dive/ 全部 21 个项目的深度分析文件*
