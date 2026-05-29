---
type: entity
tags: [GTSAM, C++ API, Geometry, PinholeCamera]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::PinholeCamera

> **类** | 头文件: `PinholeCamera.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::PinholeBaseK< Calibration >`

## 构造函数

```cpp
PinholeCamera()
```

```cpp
PinholeCamera(const Pose3 & pose)
```

```cpp
PinholeCamera(const Pose3 & pose, const Calibration & K)
```

```cpp
PinholeCamera(const Vector & v)
```
Init from vector, can be 6D (default calibration) or dim.

```cpp
PinholeCamera(const Vector & v, const Vector & K)
```
Init from Vector and calibration.

## 公开方法

### 方法

```cpp
size_t dim() const
```

```cpp
PinholeCamera retract(const Vector & d) const
```
move a cameras according to d

```cpp
VectorK6 localCoordinates(const PinholeCamera & T2) const
```
return canonical coordinate

```cpp
static size_t Dim()
```

```cpp
static PinholeCamera Identity()
```
for Canonical

```cpp
Point2 _project2(const POINT & pw, OptionalJacobian< 2, dimension > Dcamera, OptionalJacobian< 2, FixedDimension< POINT >::value > Dpoint) const
```

```cpp
Point2 project2(const Point3 & pw, OptionalJacobian< 2, dimension > Dcamera = {}, OptionalJacobian< 2, 3 > Dpoint = {}) const
```
project a 3D point from world coordinates into the image

```cpp
Point2 project2(const Unit3 & pw, OptionalJacobian< 2, dimension > Dcamera = {}, OptionalJacobian< 2, 2 > Dpoint = {}) const
```
project a point at infinity from world coordinates into the image

```cpp
double range(const Point3 & point, OptionalJacobian< 1, dimension > Dcamera = {}, OptionalJacobian< 1, 3 > Dpoint = {}) const
```

```cpp
double range(const Pose3 & pose, OptionalJacobian< 1, dimension > Dcamera = {}, OptionalJacobian< 1, 6 > Dpose = {}) const
```

```cpp
double range(const PinholeCamera< CalibrationB > & camera, OptionalJacobian< 1, dimension > Dcamera = {}, OptionalJacobian< 1, 6+CalibrationB::dimension > Dother = {}) const
```

```cpp
double range(const CalibratedCamera & camera, OptionalJacobian< 1, dimension > Dcamera = {}, OptionalJacobian< 1, 6 > Dother = {}) const
```

```cpp
Matrix34 cameraProjectionMatrix() const
```
for Linear Triangulation

```cpp
Vector defaultErrorWhenTriangulatingBehindCamera() const
```
for Nonlinear Triangulation

```cpp
bool equals(const Base & camera, double tol = 1e-9) const
```
assert equality up to a tolerance

```cpp
print(const std::string & s = "PinholeCamera") const
```
print

```cpp
const Pose3 & pose() const
```
return pose

```cpp
const Pose3 & getPose(OptionalJacobian< 6, dimension > H) const
```
return pose, with derivative

```cpp
const Calibration & calibration() const
```
return calibration

```cpp
static PinholeCamera Level(const Calibration & K, const Pose2 & pose2, double height)
```

```cpp
static PinholeCamera Level(const Pose2 & pose2, double height)
```
PinholeCamera::level with default calibration.

```cpp
static PinholeCamera Lookat(const Point3 & eye, const Point3 & target, const Point3 & upVector, const Calibration & K)
```

```cpp
static PinholeCamera Create(const Pose3 & pose, const Calibration & K, OptionalJacobian< dimension, 6 > H1 = {}, OptionalJacobian< dimension, DimK > H2 = {})
```

## 类型别名

```cpp
using VectorK6 = Eigen::Matrix< double, dimension, 1 >
```
```cpp
using Matrix2K = Eigen::Matrix< double, 2, DimK >
```
```cpp
using Measurement = Point2
```
```cpp
using MeasurementVector = Point2Vector
```

## 公开成员变量

```cpp
constexpr auto dimension
```

## 详细说明

A pinhole camera class that has a Pose3 and a Calibration. Use PinholePose if you will not be optimizing for Calibration

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`PinholeCamera` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
