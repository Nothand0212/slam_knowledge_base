---
type: entity
tags: [GTSAM, C++ API, Geometry, StereoCamera]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::StereoCamera

> **类** | 头文件: `StereoCamera.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
StereoCamera()
```
Default constructor allocates a calibration!

```cpp
StereoCamera(const Pose3 & leftCamPose, const Cal3_S2Stereo::shared_ptr K)
```
Construct from pose and shared calibration.

## 公开方法

### 方法

```cpp
const Cal3_S2Stereo & calibration() const
```
Return shared pointer to calibration.

```cpp
print(const std::string & s = "") const
```
print

```cpp
bool equals(const StereoCamera & camera, double tol = 1e-9) const
```
equals

```cpp
size_t dim() const
```
Dimensionality of the tangent space.

```cpp
StereoCamera retract(const Vector & v) const
```
Updates a with tangent space delta.

```cpp
Vector6 localCoordinates(const StereoCamera & t2) const
```
Local coordinates of manifold neighborhood around current value.

```cpp
static size_t Dim()
```
Dimensionality of the tangent space.

```cpp
const Pose3 & pose() const
```
pose

```cpp
double baseline() const
```
baseline

```cpp
StereoPoint2 project(const Point3 & point) const
```
Project 3D point to StereoPoint2 (uL,uR,v)

```cpp
StereoPoint2 project2(const Point3 & point, OptionalJacobian< 3, 6 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

```cpp
Point3 backproject(const StereoPoint2 & z) const
```
back-project a measurement

```cpp
Point3 backproject2(const StereoPoint2 & z, OptionalJacobian< 3, 6 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

```cpp
StereoPoint2 project(const Point3 & point, OptionalJacobian< 3, 6 > H1, OptionalJacobian< 3, 3 > H2 = {}, OptionalJacobian< 3, 0 > H3 = {}) const
```

```cpp
Vector defaultErrorWhenTriangulatingBehindCamera() const
```
for Nonlinear Triangulation

## 类型别名

```cpp
using Measurement = StereoPoint2
```
```cpp
using MeasurementVector = StereoPoint2Vector
```

## 公开成员变量

```cpp
constexpr auto dimension
```

## 详细说明

A stereo camera class, parameterize by left camera pose and stereo calibration

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`StereoCamera` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
