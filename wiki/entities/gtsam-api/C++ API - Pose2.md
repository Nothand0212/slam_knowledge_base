---
type: entity
tags: [GTSAM, C++ API, Geometry, Pose2]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Pose2

> **类** | 头文件: `Pose2.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::MatrixLieGroup< Pose2, 3, 3 >`

## 构造函数

```cpp
Pose2()
```

```cpp
Pose2(const Pose2 & pose)
```

```cpp
Pose2(double x, double y, double theta)
```

```cpp
Pose2(double theta, const Point2 & t)
```

```cpp
Pose2(const Rot2 & r, const Point2 & t)
```

```cpp
Pose2(const Matrix & T)
```

```cpp
Pose2(const Vector & v)
```

## 公开方法

### 方法

```cpp
Pose2 & operator=(const Pose2 & other)
```

```cpp
static std::optional< Pose2 > Align(const Point2Pairs & abPointPairs)
```

```cpp
static std::optional< Pose2 > Align(ConstMatrixView a, ConstMatrixView b)
```

```cpp
print(const std::string & s = "") const
```

```cpp
bool equals(const Pose2 & pose, double tol = 1e-9) const
```

```cpp
Pose2 inverse() const
```
inverse

```cpp
Pose2 operator*(const Pose2 & p2) const
```
compose syntactic sugar

```cpp
static Pose2 Identity()
```
identity for group operation

```cpp
Matrix3 AdjointMap() const
```

```cpp
static Pose2 Expmap(const Vector3 & xi, ChartJacobian H = {})
```
Exponential map at identity - create a rotation from canonical coordinates $ [T_x,T_y,\theta] $.

```cpp
static Vector3 Logmap(const Pose2 & p, ChartJacobian H = {})
```
Log map at identity - return the canonical coordinates $ [T_x,T_y,\theta] $ of this rotation.

```cpp
static Matrix3 adjointMap(const Vector3 & v)
```

```cpp
static Matrix3 adjointMap_(const Vector3 & xi)
```

```cpp
static Vector3 adjoint_(const Vector3 & xi, const Vector3 & y)
```

```cpp
static Matrix3 ExpmapDerivative(const Vector3 & v)
```
Derivative of Expmap.

```cpp
static Matrix3 LogmapDerivative(const Pose2 & v)
```
Derivative of Logmap.

```cpp
static Matrix3 Hat(const Vector3 & xi)
```
Hat maps from tangent vector to Lie algebra.

```cpp
static Vector3 Vee(const Matrix3 & X)
```
Vee maps from Lie algebra to tangent vector.

```cpp
Point2 transformTo(const Point2 & point, OptionalJacobian< 2, 3 > Dpose = {}, OptionalJacobian< 2, 2 > Dpoint = {}) const
```

```cpp
Matrix transformTo(ConstMatrixView points) const
```
transform many points in world coordinates and transform to Pose.

```cpp
Point2 transformFrom(const Point2 & point, OptionalJacobian< 2, 3 > Dpose = {}, OptionalJacobian< 2, 2 > Dpoint = {}) const
```

```cpp
Matrix transformFrom(ConstMatrixView points) const
```
transform many points in Pose coordinates and transform to world.

```cpp
Point2 operator*(const Point2 & point) const
```

```cpp
double x() const
```
get x

```cpp
double y() const
```
get y

```cpp
double theta() const
```
get theta

```cpp
const Point2 & t() const
```
translation

```cpp
const Rot2 & r() const
```
rotation

```cpp
const Point2 & translation(OptionalJacobian< 2, 3 > Hself = {}) const
```
translation

```cpp
const Rot2 & rotation(OptionalJacobian< 1, 3 > Hself = {}) const
```
rotation

```cpp
Matrix3 matrix() const
```
return transformation matrix

```cpp
Vector9 vec(OptionalJacobian< 9, 3 > H = {}) const
```
Vectorize the rotation matrix into a 9D vector.

```cpp
Rot2 bearing(const Point2 & point, OptionalJacobian< 1, 3 > H1 = {}, OptionalJacobian< 1, 2 > H2 = {}) const
```

```cpp
Rot2 bearing(const Pose2 & pose, OptionalJacobian< 1, 3 > H1 = {}, OptionalJacobian< 1, 3 > H2 = {}) const
```

```cpp
double range(const Point2 & point, OptionalJacobian< 1, 3 > H1 = {}, OptionalJacobian< 1, 2 > H2 = {}) const
```

```cpp
double range(const Pose2 & point, OptionalJacobian< 1, 3 > H1 = {}, OptionalJacobian< 1, 3 > H2 = {}) const
```

```cpp
static std::pair< size_t, size_t > translationInterval()
```

```cpp
static std::pair< size_t, size_t > rotationInterval()
```

## 类型别名

```cpp
using Rotation = Rot2
```
```cpp
using Translation = Point2
```
```cpp
using LieAlgebra = Matrix3
```

## 详细说明

A 2D pose (Point2,Rot2)

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Pose2` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
