---
tags: [GTSAM, geometry, Pose3, Rot3, Lie-group]
created: 2026-04-27
updated: 2026-05-15
superseded-by: [[方法-GTSAM-API族]]
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

> 本页内容已归并至 [[方法-GTSAM-API族]]。

# GTSAM Geometry API

> Geometry 模块提供优化变量的 manifold/Lie group 类型：`Pose2`、`Pose3`、`Rot2`、`Rot3`、camera calibration 和 stereo camera。

## Concepts

GTSAM 的 geometry 类型通常满足 `Manifold`、`Group`、`LieGroup` 等 concept：

- `retract(delta)`：把 tangent space 增量映射回 manifold。
- `localCoordinates(other)`：把两个 manifold 值之间的差转为 tangent vector。
- `compose` / `inverse` / `between`：group 操作。
- `Expmap` / `Logmap`：Lie algebra 和 Lie group 之间的映射。

## Pose2

官方文档：https://borglab.github.io/gtsam/pose2

用途：2D pose graph SLAM、平面机器人定位、轮式里程计约束。

常用构造：

```python
from gtsam import Pose2, Rot2, Point2

p0 = Pose2()                         # identity
p1 = Pose2(1.0, 2.0, 0.3)            # x, y, theta
p2 = Pose2(Rot2.fromDegrees(90), Point2(1, 2))
```

常用方法：

- `x()`、`y()`、`theta()`
- `translation()`、`rotation()`
- `matrix()`
- `transformTo(point)`、`transformFrom(point)`
- `bearing(point)`、`range(point)`
- `localCoordinates(q)`、`retract(delta)`
- `Pose2.Expmap(xi)`、`Pose2.Logmap(pose)`

注意：`Point2` 在 Python 中是创建 2D `np.ndarray` 的工具函数，不是独立对象类。

## Pose3

官方文档：https://borglab.github.io/gtsam/pose3

用途：3D SLAM、VIO、视觉重投影、frame transform。

常用构造：

```python
from gtsam import Pose3, Rot3, Point3
import numpy as np

P0 = Pose3()
R = Rot3.Yaw(np.deg2rad(30))
t = Point3(3, 4, 0)
P = Pose3(R, t)
```

常用方法：

- `rotation()`、`translation()`
- `matrix()`
- `inverse()`、`compose()`、`between()`
- `transformTo(point)`、`transformFrom(point)`
- `localCoordinates(q)`、`retract(delta)`
- `Pose3.Expmap(xi)`、`Pose3.Logmap(pose)`

工程注意：

- `Pose3` 同时可被当作 pose manifold 和 rigid transform Lie group 使用，但概念上 pose 与 transform 不完全相同。
- 文档强调 GTSAM 内部不要求用 4x4 homogeneous matrix 实现。
- Jacobian/tangent vector 维度是 6，使用时注意 GTSAM 的 perturbation convention。

## Rot3

官方文档：https://borglab.github.io/gtsam/rot3

用途：SO(3) 姿态、IMU、camera orientation。

常用构造：

```python
R0 = gtsam.Rot3()
R_yaw = gtsam.Rot3.Yaw(0.1)
R_ypr = gtsam.Rot3.Ypr(yaw, pitch, roll)
R_rpy = gtsam.Rot3.RzRyRx(roll, pitch, yaw)
R_exp = gtsam.Rot3.Expmap(np.array([0.01, 0.02, 0.03]))
```

常用方法：

- `matrix()`
- `quaternion()`
- `rpy()`、`yaw()`、`pitch()`、`roll()`（按 wrapper 暴露为准）
- `rotate(point)`、`unrotate(point)`
- `compose` / `inverse` / `between`
- `Rot3.Expmap` / `Rot3.Logmap`

## Cal3_S2

官方文档：https://borglab.github.io/gtsam/cal3-s2

用途：无畸变 pinhole camera intrinsic，参数为 `[fx, fy, s, u0, v0]`。

```python
K = gtsam.Cal3_S2(500.0, 500.0, 0.0, 320.0, 240.0)
fx = K.fx()
principal = K.principalPoint()
normalized = K.calibrate(gtsam.Point2(330, 250))
pixel = K.uncalibrate(normalized)
```

常用方法：

- `fx()`、`fy()`、`skew()`、`px()`、`py()`
- `principalPoint()`、`vector()`
- `K()`、`inverse()`、`aspectRatio()`
- `calibrate(p)`、`uncalibrate(p)`

## StereoCamera / Cal3_S2Stereo

官方文档：

- https://borglab.github.io/gtsam/stereocamera
- https://borglab.github.io/gtsam/cal3-s2stereo

用途：stereo VO/SLAM，处理 `(uL, uR, v)`。

```python
K = gtsam.Cal3_S2Stereo(fx, fy, s, u0, v0, baseline)
camera = gtsam.StereoCamera(left_camera_pose, K)
z = camera.project(point_world)
point_world2 = camera.backproject(z)
```

常用方法：

- `pose()`、`calibration()`、`baseline()`
- `project(point)` / `project2(point, H1, H2)`
- `backproject(stereo_point)` / `backproject2(...)`

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM SLAM 与视觉因子 API]]
- [[GTSAM Navigation 与 IMU API]]
