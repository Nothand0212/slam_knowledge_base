---
type: entity
tags: [GTSAM, C++ API, Geometry, Cal3_S2Stereo]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Cal3_S2Stereo

> **类** | 头文件: `Cal3_S2Stereo.h` | [在线文档](https://gtsam.org/doxygen/)

The most common 5DOF 3D->2D calibration, stereo version.

## 继承关系

- 继承自 `gtsam::Cal3_S2`

## 构造函数

```cpp
Cal3_S2Stereo()
```
default calibration leaves coordinates unchanged

```cpp
Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b)
```
constructor from doubles

```cpp
Cal3_S2Stereo(const Vector6 & d)
```
constructor from vector

```cpp
Cal3_S2Stereo(double fov, int w, int h, double b)
```
easy constructor; field-of-view in degrees, assumes zero skew

## 公开方法

### 方法

```cpp
print(const std::string & s = "") const
```
print with optional string

```cpp
bool equals(const Cal3_S2Stereo & other, double tol = 10e-9) const
```
Check if equal up to specified tolerance.

```cpp
const Cal3_S2 & calibration() const
```
return calibration, same for left and right

```cpp
Matrix3 K() const
```
return calibration matrix K, same for left and right

```cpp
double baseline() const
```
return baseline

```cpp
Vector6 vector() const
```
vectorized form (column-wise)

```cpp
size_t dim() const
```
return DOF, dimensionality of tangent space

```cpp
Cal3_S2Stereo retract(const Vector & d) const
```
Given 6-dim tangent vector, create new calibration.

```cpp
Vector6 localCoordinates(const Cal3_S2Stereo & T2) const
```
Unretraction for the calibration.

```cpp
static size_t Dim()
```
return DOF, dimensionality of tangent space

### 方法

```cpp
Point2 uncalibrate(const Point2 & p, OptionalJacobian< 2, 6 > Dcal = {}, OptionalJacobian< 2, 2 > Dp = {}) const
```

```cpp
Point2 calibrate(const Point2 & p, OptionalJacobian< 2, 6 > Dcal = {}, OptionalJacobian< 2, 2 > Dp = {}) const
```

```cpp
Vector3 calibrate(const Vector3 & p) const
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< Cal3_S2Stereo >
```

## 公开成员变量

```cpp
constexpr auto dimension
```

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Cal3_S2Stereo` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
