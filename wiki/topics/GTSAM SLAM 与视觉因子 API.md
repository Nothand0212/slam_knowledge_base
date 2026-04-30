---
tags: [GTSAM, SLAM, visual-SLAM, SfM, factors]
created: 2026-04-27
updated: 2026-04-27
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

# GTSAM SLAM 与视觉因子 API

> SLAM/vision 常用 factors：prior、between、projection、stereo、smart factors，以及 pose initialization/dataset utilities。

## PriorFactor

官方文档：https://borglab.github.io/gtsam/priorfactor

用途：给变量加先验，常用于锚定第一帧 pose 或给 landmark/参数提供初始约束。

```python
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
graph.addPriorPose2(X(0), gtsam.Pose2(0, 0, 0), prior_noise)
```

注意：非常强的 prior 可能恶化线性系统 condition number。需要真正固定变量时考虑 `NonlinearEquality`。

## BetweenFactor

官方文档：https://borglab.github.io/gtsam/betweenfactor

用途：两个同类型 Lie group 变量之间的相对测量，例如 odometry、relative pose、loop closure。

```python
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
graph.add(gtsam.BetweenFactorPose2(X(0), X(1), gtsam.Pose2(1, 0, 0), odom_noise))

pose3_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1]))
graph.add(gtsam.BetweenFactorPose3(X(1), X(2), measured_pose3, pose3_noise))
```

误差直觉：预测相对位姿 `X1.between(X2)` 与 measurement `Z` 的差，再映射到 tangent space。

## GenericProjectionFactor

官方文档：https://borglab.github.io/gtsam/projectionfactor

用途：单目重投影误差，连接 camera pose 和 3D landmark，calibration 固定。

```python
K = gtsam.Cal3_S2(500, 500, 0, 320, 240)
pixel_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
factor = gtsam.GenericProjectionFactorCal3_S2(
    gtsam.Point2(330, 250), pixel_noise, X(0), L(1), K
)
graph.add(factor)
```

可选参数：

- `body_P_sensor`：body frame 到 camera sensor frame 的固定外参。
- cheirality handling flags：控制 landmark 在相机后方时抛异常还是返回大误差。

## SmartProjectionFactor

官方文档：

- https://borglab.github.io/gtsam/smartprojectionfactor
- https://borglab.github.io/gtsam/smartprojectionposefactor
- https://borglab.github.io/gtsam/smartprojectionparams

用途：SfM/visual SLAM 中把 3D landmark 作为隐式变量处理。factor 内部按当前 camera estimates 三角化点，线性化后相当于边缘化掉 landmark，只连接观测它的 cameras。

选择：

- `SmartProjectionFactorPinholeCameraCal3_S2`：Values 里存 `PinholeCameraCal3_S2`，pose 和 calibration 一起作为 camera object。
- `SmartProjectionPoseFactor`：calibration 已知固定，只优化 poses，通常更高效。

模板：

```python
smart_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
params = gtsam.SmartProjectionParams()
factor = gtsam.SmartProjectionFactorPinholeCameraCal3_S2(smart_noise, params)

factor.add(gtsam.Point2(150, 505), C(0))
factor.add(gtsam.Point2(470, 495), C(1))
factor.add(gtsam.Point2(480, 150), C(2))
graph.add(factor)
```

注意：

- 对 `SmartProjectionFactor<CAMERA>`，`Values` 必须包含 CAMERA objects，不是分离的 `Pose3` 和 `Cal3_S2`。
- 退化三角化行为由 `SmartProjectionParams` 控制。

## Stereo factor

官方文档：https://borglab.github.io/gtsam/stereofactor

用途：stereo measurement `(uL, uR, v)` 约束 `Pose3` 和 `Point3`。

常见 API：

- `GenericStereoFactor3D`
- `StereoPoint2`
- `StereoCamera`
- `Cal3_S2Stereo`

典型流程：

1. 用 `StereoCamera.project(point)` 生成或检查 measurement。
2. 用 `GenericStereoFactor3D(measured, noise, pose_key, landmark_key, K)` 建图。
3. 初值里必须有 pose 和 landmark。

## 相关 examples

- Pose2 SLAM：https://borglab.github.io/gtsam/pose2slamexample
- Planar SLAM：https://borglab.github.io/gtsam/planarslamexample
- Range iSAM：https://borglab.github.io/gtsam/rangeisamexample-plaza2
- Stereo VO：https://borglab.github.io/gtsam/stereovoexample
- Stereo VO large：https://borglab.github.io/gtsam/stereovoexample-large
- Camera resectioning：https://borglab.github.io/gtsam/cameraresectioning

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM Geometry API]]
- [[GTSAM Nonlinear 优化 API]]
