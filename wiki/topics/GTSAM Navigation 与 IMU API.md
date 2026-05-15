---
tags: [GTSAM, navigation, IMU, GPS, VIO]
created: 2026-04-27
updated: 2026-05-15
superseded-by: [[方法-GTSAM-API族]]
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

> 本页内容已归并至 [[方法-GTSAM-API族]]。

# GTSAM Navigation 与 IMU API

> Navigation 模块覆盖 `NavState`、IMU preintegration、GPS/barometer/magnetometer factors 和 EKF 变体，是 VIO/INS/GNSS 融合的核心入口。

## 模块入口

官方文档：https://borglab.github.io/gtsam/navigation

主要类：

- `NavState`
- `imuBias::ConstantBias`
- `PreintegrationParams`
- `PreintegratedImuMeasurements`
- `PreintegratedCombinedMeasurements`
- `ImuFactor`、`ImuFactor2`
- `CombinedImuFactor`
- `GPSFactor` family
- `AHRSFactor`、`AttitudeFactor`、`MagFactor`、`BarometricFactor`

## NavState

官方文档：https://borglab.github.io/gtsam/navstate

`NavState` 表示 `(R, p, v)`：attitude、position、velocity。GTSAM 文档强调其 tangent increment 顺序是 `[dR, dP, dV]`。

常用 API：

```python
X = gtsam.NavState(gtsam.Rot3.Yaw(0.1), gtsam.Point3(1, 2, 3), np.array([0.5, 0, 0]))
R = X.attitude()
p = X.position()
v = X.velocity()
pose = X.pose()
body_v = X.bodyVelocity()
```

Lie/manifold API：

- `NavState.Expmap(xi)`
- `NavState.Logmap(X)`
- `X.retract(delta)`
- `X.localCoordinates(Y)`
- `X.inverse()`
- `X * Y`

## PreintegrationParams

官方文档：https://borglab.github.io/gtsam/preintegrationparams

常用配置：

```python
params = gtsam.PreintegrationParams.MakeSharedU(9.81)  # ENU, Z-up
params.setAccelerometerCovariance(np.eye(3) * accel_sigma**2)
params.setGyroscopeCovariance(np.eye(3) * gyro_sigma**2)
params.setIntegrationCovariance(np.eye(3) * 1e-8)
# params.setBodyPSensor(body_P_sensor)
```

注意：

- `MakeSharedU(g)`：Z-up/ENU gravity convention。
- `MakeSharedD(g)`：Z-down/NED convention。
- IMU 噪声单位要匹配连续时间 white noise 约定。
- `body_P_sensor` 处理 IMU frame 到 body frame 外参。

## PreintegratedImuMeasurements + ImuFactor

官方文档：

- https://borglab.github.io/gtsam/preintegratedimumeasurements
- https://borglab.github.io/gtsam/imufactor

`ImuFactor` 是 5-way factor，连接：

- `Pose_i`
- `Vel_i`
- `Pose_j`
- `Vel_j`
- `Bias_i`

模板：

```python
from gtsam.symbol_shorthand import X, V, B
from gtsam.imuBias import ConstantBias

bias_hat = ConstantBias()
pim = gtsam.PreintegratedImuMeasurements(params, bias_hat)

for acc, gyro, dt in imu_packets:
    pim.integrateMeasurement(acc, gyro, dt)

graph.add(gtsam.ImuFactor(X(i), V(i), X(j), V(j), B(i), pim))
```

`ImuFactor2` 使用 `NavState`，连接 `NavState_i`、`NavState_j`、`Bias_i`，图结构更紧凑。

## CombinedImuFactor

官方文档：

- https://borglab.github.io/gtsam/combinedimufactor
- https://borglab.github.io/gtsam/combined-vs-imufactor

`CombinedImuFactor` 是 6-way factor，连接 `Pose_i`、`Vel_i`、`Bias_i`、`Pose_j`、`Vel_j`、`Bias_j`，并在 15D covariance 中包含 bias random walk。

官方 navigation 文档给出的工程建议很重要：

- 默认不推荐一开始就用 combined IMU factor。
- bias 通常变化慢，单独低频 bias Markov chain 往往更合适。
- 短时实验甚至建议先用单个 constant bias，把主 pipeline 跑通后再估计 bias。
- `CombinedImuFactor` 更适合确实需要每个 interval 高精度 bias 估计的场景。

## Preintegration implementation flag

官方 navigation 文档说明默认 preintegration 类型受 `GTSAM_TANGENT_PREINTEGRATION` 编译 flag 控制：

- true：使用 tangent-space preintegration，默认。
- false：使用 Forster et al. RSS 2015 风格的 manifold preintegration。

如需非默认类型，C++ 侧使用模板类：`PreintegratedImuMeasurementsT`、`ImuFactorT`、`ImuFactor2T`、`PreintegratedCombinedMeasurementsT`、`CombinedImuFactorT`。

## GPSFactor family

官方文档：https://borglab.github.io/gtsam/gpsfactor

GPS measurement 必须先转换到 local Cartesian navigation frame，例如 ENU、NED 或 ECEF。

变体：

- `GPSFactor`：连接 `Pose3`，zero lever arm。
- `GPSFactorArm`：连接 `Pose3`，已知 body-frame lever arm。
- `GPSFactorArmCalib`：连接 `Pose3` 和 lever-arm `Point3` 变量，估计 lever arm。
- `GPSFactor2` / `GPSFactor2Arm` / `GPSFactor2ArmCalib`：对应 `NavState` 版本。

模板：

```python
gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5, 1.0]))
gps_meas = gtsam.Point3(10.5, 20.2, 5.1)  # local ENU/NED/ECEF
graph.add(gtsam.GPSFactor(X(k), gps_meas, gps_noise))

lever_arm_body = gtsam.Point3(-0.1, 0.0, 0.05)
graph.add(gtsam.GPSFactorArm(X(k), gps_meas, lever_arm_body, gps_noise))
```

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM Geometry API]]
- [[GTSAM 因子图工作流]]
