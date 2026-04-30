---
tags: [IMU, 预积分, GTSAM]
sources:
  - wiki/sources/2026-04-28-msckf_vio-analysis.md
  - wiki/sources/2026-04-28-open_vins-analysis.md
  - wiki/sources/2026-04-28-openmavis-analysis.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-29-open-vins-analysis.md
  - wiki/sources/2026-04-29-vins-fusion-analysis-analysis.md
created: 2026-04-29
updated: 2026-04-30
type: entity
---

# IMU预积分

> 将两关键帧间的高频IMU测量预先积分为相对运动增量（位置、速度、姿态），使优化时无需重复积分，是 VIO 和 LI-SLAM 的核心技术。

## 概述

IMU 预积分由 Forster 等人于 2015/2017 年提出，核心思想是在关键帧约束之间对加速度计和陀螺仪测量进行连续时间积分，得到一个与初始状态无关的相对增量。当关键帧状态在优化中被更新时，预积分量通过一阶泰勒展开近似修正，避免昂贵的高频 IMU 数据重新积分。GTSAM 内置 `ImuFactor` 和 `CombinedImuFactor` 直接支持。

## 在SLAM中的应用

几乎所有现代 VIO 和 LI-SLAM 系统都使用 IMU 预积分：VINS-Fusion 使用 Ceres 实现预积分作为关键帧间约束，ORB-SLAM3 在视觉-惯性模式中使用 GTSAM 预积分因子，FAST-LIO 通过 IMU 预积分在卡尔曼滤波框架中传播状态协方差并去畸变点云。预积分的偏置修正机制使得系统可以在线估计 IMU 偏置漂移。

## Agent 实现提示

### 适用场景

当系统需要把两个关键帧、两个 LiDAR 位姿修正时刻，或两个优化节点之间的高频 IMU 测量压缩成一个相对运动约束时，优先实现 IMU 预积分。它适合因子图后端、滑动窗口优化和需要异步 IMU 线程实时预测的 LIO/VIO 系统；如果只是单帧内点云去畸变，可参考 [[方法-IMU deskew]] 的轻量积分路径。

### 输入输出契约

- **输入**：按时间排序的 IMU 队列、上一关键帧状态 `prevState`、上一 bias `prevBias`、IMU 噪声参数、当前校正时间 `currentCorrectionTime`。
- **输出**：预积分对象 `preint_imu`、图优化中的 `ImuFactor`、bias 随机游走因子、用预积分预测出的当前状态初值。
- **状态约束**：每次优化完成后必须用最新 bias 重置预积分器；积分区间不能跨越尚未确认的未来校正时间。

### 实现骨架（伪代码）

```pseudo
function buildImuPreintegration(imu_queue, prev_state, prev_bias, correction_time):
    while imu_queue.front.time < correction_time - dt_guard:
        imu = imu_queue.pop_front()
        dt = last_imu_time < 0 ? nominal_dt : imu.time - last_imu_time
        preintegrator.integrate(acc=imu.acc, gyro=imu.gyro, dt=dt)
        last_imu_time = imu.time

    preint = preintegrator.as_measurement()
    graph.add(ImuFactor(X[k-1], V[k-1], X[k], V[k], B[k-1], preint))
    graph.add(BiasBetweenFactor(B[k-1], B[k], sqrt(preint.deltaT) * bias_noise))
    predicted = preintegrator.predict(prev_state, prev_bias)
    values.insert(X[k], predicted.pose)
    values.insert(V[k], predicted.velocity)
```

### 关键源码片段

`raw/codes/LIO-SAM/src/imuPreintegration.cpp:L350-L383`

```cpp
        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
```

### 实现注意事项

- `dt` 必须来自相邻 IMU 时间戳，首帧可使用标称采样周期兜底，但不能长期依赖常数周期。
- 预积分约束通常连接 `pose、velocity、bias`，缺少速度变量会导致惯性约束不可正确表达。
- bias 更新后要重置预积分器，否则新 bias 与旧预积分量不一致。
- 图优化初值可以用 `predict(prevState, prevBias)` 给出，但不要把它当作最终估计；它只是后端线性化初值。

### 源码检索锚点

- `PreintegratedImuMeasurements`
- `integrateMeasurement(`
- `ImuFactor(`
- `resetIntegrationAndSetBias`
- `noiseModelBetweenBias`

## 相关页面

- [[组件-GTSAM]], [[组件-GTSAM]]
- [[算法-FAST-LIO]], [[算法-LIO-SAM]]
- [[架构-滑动窗口优化]]
---
## (合并自: IMU预积分方法.md)
---
---
tags: [IMU, 预积分, 积分方法, VIO, LIO, GTSAM]
sources: [wiki/topics/IMU数据管线, wiki/sources/2026-04-29-imu-pipeline-comparison]
created: 2026-04-29
updated: 2026-04-29
type: entity
---

# IMU预积分方法

> 本页汇总知识库中所有算法使用的 IMU 积分/预积分方法，从离散积分到 SE2(3) 精确积分，含数学公式和选型建议。

---

## 1. 两大范式

IMU 数据进入状态估计有两种根本不同的路径：

| 范式 | 核心操作 | 更新时机 | 代表算法 |
|------|---------|---------|---------|
| **滤波传播** | 用 IMU 直接前向传播状态均值和协方差 | 每个 IMU 采样点（100-500 Hz） | MSCKF, FAST-LIO2, R3LIVE |
| **因子图预积分** | 将帧间 IMU 积分为相对运动增量，作为二元约束送入优化器 | 关键帧频率（1-10 Hz） | VINS-Fusion, ORB-SLAM3, LIO-SAM |

---

## 2. 滤波传播中的积分方法

滤波系统在每个 IMU 读数到达时执行状态前向传播，积分方法决定传播精度。

### 2.1 欧拉离散积分

最简单的积分方式，假设当前时刻的加速度和角速度在积分步长内恒定：

$$
\begin{aligned}
R_{k+1} &= R_k \cdot \text{Exp}(\omega_k \cdot \Delta t) \\
v_{k+1} &= v_k + (R_k \cdot a_k - g) \cdot \Delta t \\
p_{k+1} &= p_k + v_k \cdot \Delta t + \frac{1}{2}(R_k \cdot a_k - g) \cdot \Delta t^2
\end{aligned}
$$

- **精度**：最低，对高动态运动累积误差大
- **使用项目**：fusions_slam（`propagate.cpp` 遍历 IMU 测量零阶保持积分）、lightning_lm（前向 ESKF + 反向补偿去畸变）
- **适用场景**：LiDAR 帧率低（10 Hz）时单帧内累积误差可控，或 IESKF 的迭代更新能补偿积分误差

### 2.2 RK4 直接积分（四阶龙格-库塔）

在积分步长内取 4 个采样点求加权平均，显著降低截断误差：

$$
\begin{aligned}
k_1 &= f(t_k, x_k) \\
k_2 &= f(t_k + \frac{\Delta t}{2}, x_k + \frac{\Delta t}{2} k_1) \\
k_3 &= f(t_k + \frac{\Delta t}{2}, x_k + \frac{\Delta t}{2} k_2) \\
k_4 &= f(t_k + \Delta t, x_k + \Delta t \cdot k_3) \\
x_{k+1} &= x_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}
$$

- **精度**：高，四阶截断误差为 $O(\Delta t^5)$
- **状态转移矩阵**：三阶 $\Phi$ 矩阵近似（$\Phi \approx I + F\Delta t + \frac{1}{2}F^2\Delta t^2 + \frac{1}{6}F^3\Delta t^3$）
- **使用项目**：msckf_vio（RK4 + 三阶 Phi + 闭式积分）、schurvins（RK4 + 连续 EKF 传播，无标准预积分框架）
- **关键实现**：msckf_vio 在 `imuStatePropagate` 中调用 RK4 数值积分，然后利用解析的状态转移矩阵传播协方差矩阵 $P$

### 2.3 ACI² 解析连续时间积分

使用闭式解析解替代数值积分，对恒定加速度/角速度假设下的运动给出精确传播结果：

- **精度**：最高（在模型假设内），高角速度（>1000°/s）时远优于 RK4
- **代价**：数学推导复杂，代码量最大
- **使用项目**：open_vins（三种模式之一，`Propagator::propagate_and_record` 中选择 `aci2` 模式）
- **适用场景**：高速旋转的无人机，角速度远超 IMU 采样率时尤为重要

---

## 3. 因子图中的预积分方法

预积分的核心思想：将两关键帧 $i$ 到 $j$ 之间的所有 IMU 测量预先积分为一个与初始状态无关的相对增量，优化时直接作为二元约束，避免每次迭代重新积分。

### 3.1 中值积分（Mid-point / RK2）

在积分区间中点采样，等效于二阶龙格-库塔：

$$
\begin{aligned}
\bar{\omega} &= \frac{1}{2}(\omega_k + \omega_{k+1}) - b_g \\
\bar{a} &= \frac{1}{2}(R_k \cdot (a_k - b_a) + R_{k+1} \cdot (a_{k+1} - b_a)) \\
R_{k+1} &= R_k \cdot \text{Exp}(\bar{\omega} \cdot \Delta t) \\
v_{k+1} &= v_k + \bar{a} \cdot \Delta t \\
p_{k+1} &= p_k + v_k \cdot \Delta t + \frac{1}{2}\bar{a} \cdot \Delta t^2
\end{aligned}
$$

- **协方差传播**：使用误差状态 Jacobian $F$（15×15）和噪声 Jacobian $V$（15×18），递推预积分协方差 $\Sigma_{ij}$
- **偏置更新**：利用偏置 Jacobian $J_b = \frac{\partial \Delta X}{\partial b}$，偏置变化时通过一阶泰勒展开修正预积分量，无需重积分
- **使用项目**：VINS-Fusion（`IntegrationBase::midPointIntegration`，Ceres 实现，含完整 Jacobian/协方差传播）
- **关键实现**：`IntegrationBase::midPointIntegration` 函数，遍历区间内 IMU 数据，逐步累积预积分量 + 传播协方差

### 3.2 GTSAM 标准预积分（Forster 2015/2017 风格）

GTSAM 内置 `PreintegratedImuMeasurements`，基于流形上的 IMU 预积分：

- **参数配置**：`PreintegrationParams` 包含加速度计/陀螺仪噪声密度和随机游走参数
- **因子类型**：`ImuFactor`（标准预积分因子）和 `CombinedImuFactor`（预积分 + 偏置随机游走合并为一个因子，减少变量数）
- **积分方式**：内部使用欧拉或中点法，GTSAM 自动管理协方差传播
- **使用项目**：LIO-SAM（GTSAM ISAM2 + CombinedImuFactor）、Kimera-VIO（GTSAM ISAM2）、DM-VIO（GTSAM ImuFactor 与 DSO 光度 Hessian 联合优化）

### 3.3 零阶近似预积分

最简单的预积分形式，忽略 IMU 测量在帧间的时序信息，直接用帧间平均 IMU 读数做一次积分：

- **精度**：最低，高角速度下位置/速度误差很大（与 SE2(3) 精确积分比，误差可达 200%）
- **使用项目**：ORB-SLAM3（`IMU::Preintegrated` 类，零阶预积分 + 一阶近似偏置校正）

### 3.4 SE2(3) 精确预积分（OpenMAVIS）

在 SE2(3) 群上而非 SE(3) 上进行预积分，使用 J1(dt,θ) 和 J2(dt,θ) 精确捕捉角速度对位置和速度更新的耦合效应：

- **核心创新**：标准预积分中，位置更新公式 $p_{k+1} = p_k + v_k \Delta t + \frac{1}{2}R a \Delta t^2$ 忽略了角速度在积分区间内对姿态 $R$ 的连续影响。SE2(3) 预积分通过 J1/J2 矩阵完整捕捉这一耦合
- **精度**：最高（本知识库所有预积分方法中）。高角速度场景（>500°/s）精度显著优于零阶近似
- **使用项目**：OpenMAVIS（基于 ORB-SLAM3 架构，替换其零阶预积分为 SE2(3) 精确版本）
- **代价**：J1/J2 矩阵涉及 $\exp(\theta)/\theta$ 和 $(1 - \cos\theta)/\theta^2$ 项，接近零角速度时需 Taylor 展开避免数值奇异

### 3.5 地球自转补偿预积分（IC-GVINS / OB_GINS）

针对高精度 IMU（导航级/战术级），在预积分模型中考虑地球自转的附加项：

- **附加项**：$a_{\text{coriolis}} = -2\omega_{\text{earth}} \times v$ 和 $a_{\text{centrifugal}} = -\omega_{\text{earth}} \times (\omega_{\text{earth}} \times r)$
- **使用项目**：IC-GVINS（可选工厂模式支持）、OB_GINS（IMU 预积分默认含地球自转补偿）
- **适用场景**：仅工业级以上 MEMS 和高精度 FOG/RLG IMU 受益，消费级 MEMS 噪声远超地球自转量级
- **可插拔设计**：IC-GVINS 通过工厂模式支持 Normal / Earth / Earth+Odo 等多种预积分配置

---

## 4. 精度谱总览

按积分精度从低到高排列：

| 方法 | 精度 | 延迟 | 复杂度 | 代表项目 |
|------|------|------|--------|---------|
| 欧拉离散 | 低 | 最低 | 极简 | fusions_slam, lightning_lm |
| 零阶预积分 | 低 | 低 | 简 | ORB-SLAM3 |
| 中值积分 (RK2) | 中 | 中 | 中 | VINS-Fusion |
| GTSAM 标准预积分 | 中 | 中 | 中 | LIO-SAM, Kimera-VIO |
| RK4 + 三阶 Phi | 高 | 中 | 较高 | msckf_vio, schurvins |
| ACI² 解析积分 | 高 | 中 | 高 | open_vins |
| 地球自转补偿预积分 | 高* | 中 | 高 | IC-GVINS, OB_GINS |
| SE2(3) 精确预积分 | 最高 | 中 | 最高 | OpenMAVIS |

\* 注：地球自转补偿对消费级 IMU 几乎没有增益，仅对导航级 IMU 有意义。

---

## 5. 选型建议

| 场景 | 推荐方法 | 理由 |
|------|---------|------|
| 低算力嵌入式 VIO | 中值积分 | 精度足够，实现简单 |
| 高速无人机 | RK4 或 ACI² | 角速度 > 500°/s 时精度关键 |
| 车载多传感器 | GTSAM 标准预积分 + ISAM2 | 易扩展新传感器因子 |
| 高动态 + 高精度需求 | SE2(3) 精确预积分 | 角速度耦合不可忽略 |
| 航位推算/组合导航 | 地球自转补偿预积分 | 长时间纯惯性传播需要 |
| 纯 LiDAR SLAM | 欧拉离散 (IESKF 体系) | 10 Hz LiDAR 帧率下误差可控 |

---

## 6. 相关页面

- [[概念-IMU预积分]]
- [[IMU数据管线]]
- [[方法-IESKF滤波器]]
- [[组件-GTSAM]]
- [[因子图vs滤波]]
- [[概念-MSCKF]]
