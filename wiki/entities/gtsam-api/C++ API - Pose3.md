---
type: entity
tags: [GTSAM, C++ API, Geometry, Pose3]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Pose3

> **类** | 头文件: `Pose3.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::ExtendedPose3< 1, Pose3 >`

## 构造函数

```cpp
Pose3()
```

```cpp
Pose3(const Pose3 & pose)
```

```cpp
Pose3(const Base & other)
```

```cpp
Pose3(const Rot3 & R, const Point3 & t)
```

```cpp
Pose3(const Pose2 & pose2)
```

```cpp
Pose3(const Matrix & T)
```

## 公开方法

### 方法

```cpp
static Pose3 Expmap(const Vector6 & xi, OptionalJacobian< 6, 6 > Hxi = {})
```
Exponential map at identity.

```cpp
static Matrix6 adjointMap_(const Vector6 & xi)
```

```cpp
static Vector6 adjoint_(const Vector6 & xi, const Vector6 & y)
```

```cpp
Pose3 & operator=(const Pose3 & other)
```

```cpp
static Pose3 Create(const Rot3 & R, const Point3 & t, OptionalJacobian< 6, 3 > HR = {}, OptionalJacobian< 6, 3 > Ht = {})
```
Named constructor with derivatives.

```cpp
static Pose3 FromPose2(const Pose2 & p, OptionalJacobian< 6, 3 > H = {})
```

```cpp
static std::optional< Pose3 > Align(const Point3Pairs & abPointPairs)
```

```cpp
static std::optional< Pose3 > Align(ConstMatrixView a, ConstMatrixView b)
```

```cpp
print(const std::string & s = "") const
```
print with optional string

```cpp
bool equals(const Pose3 & pose, double tol = 1e-9) const
```
assert equality up to a tolerance

```cpp
Pose3 interpolateRt(const Pose3 & T, double t, OptionalJacobian< 6, 6 > Hself = {}, OptionalJacobian< 6, 6 > Harg = {}, OptionalJacobian< 6, 1 > Ht = {}) const
```

```cpp
Pose3 operator*(const Pose3 & T) const
```
Compose syntactic sugar.

```cpp
Point3 transformFrom(const Point3 & point, OptionalJacobian< 3, 6 > Hself = {}, OptionalJacobian< 3, 3 > Hpoint = {}) const
```
takes point in Pose coordinates and transforms it to world coordinates

```cpp
Matrix transformFrom(ConstMatrixView points) const
```
transform many points in Pose coordinates and transform to world.

```cpp
Point3 operator*(const Point3 & point) const
```

```cpp
Point3 transformTo(const Point3 & point, OptionalJacobian< 3, 6 > Hself = {}, OptionalJacobian< 3, 3 > Hpoint = {}) const
```
takes point in world coordinates and transforms it to Pose coordinates

```cpp
Matrix transformTo(ConstMatrixView points) const
```
transform many points in world coordinates and transform to Pose.

```cpp
const Point3 & translation(OptionalJacobian< 3, 6 > Hself = {}) const
```
get translation

```cpp
double x() const
```
get x

```cpp
double y() const
```
get y

```cpp
double z() const
```
get z

```cpp
Pose3 transformPoseFrom(const Pose3 & aTb, OptionalJacobian< 6, 6 > Hself = {}, OptionalJacobian< 6, 6 > HaTb = {}) const
```

```cpp
Pose3 transformPoseTo(const Pose3 & wTb, OptionalJacobian< 6, 6 > Hself = {}, OptionalJacobian< 6, 6 > HwTb = {}) const
```

```cpp
double range(const Point3 & point, OptionalJacobian< 1, 6 > Hself = {}, OptionalJacobian< 1, 3 > Hpoint = {}) const
```

```cpp
double range(const Pose3 & pose, OptionalJacobian< 1, 6 > Hself = {}, OptionalJacobian< 1, 6 > Hpose = {}) const
```

```cpp
Unit3 bearing(const Point3 & point, OptionalJacobian< 2, 6 > Hself = {}, OptionalJacobian< 2, 3 > Hpoint = {}) const
```

```cpp
Unit3 bearing(const Pose3 & pose, OptionalJacobian< 2, 6 > Hself = {}, OptionalJacobian< 2, 6 > Hpose = {}) const
```

```cpp
Pose3 slerp(double t, const Pose3 & other, OptionalJacobian< 6, 6 > Hx = {}, OptionalJacobian< 6, 6 > Hy = {}) const
```
Spherical Linear interpolation between *this and other.

```cpp
static std::pair< size_t, size_t > translationInterval()
```

```cpp
static std::pair< size_t, size_t > rotationInterval()
```

## 类型别名

```cpp
using LieAlgebra = Matrix4
```
```cpp
using Base = ExtendedPose3< 1, Pose3 >
```
```cpp
using Rotation = Rot3
```
```cpp
using Translation = Point3
```
```cpp
using Vector16 = Eigen::Matrix< double, 16, 1 >
```

## 公开成员变量

```cpp
constexpr auto dimension
```

## 详细说明

A 3D pose (R,t) : (Rot3,Point3)

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Pose3` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
