---
type: entity
tags: [GTSAM, C++ API, Geometry, Rot3]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Rot3

> **类** | 头文件: `Rot3.h` | [在线文档](https://gtsam.org/doxygen/)

Rot3 is a 3D rotation represented as a rotation matrix if the preprocessor symbol GTSAM_USE_QUATERNIONS is not defined, or as a quaternion if it is defined.

## 继承关系

- 继承自 `gtsam::MatrixLieGroup< Rot3, 3, 3 >`

## 构造函数

```cpp
Rot3()
```

```cpp
Rot3(const Point3 & col1, const Point3 & col2, const Point3 & col3)
```

```cpp
Rot3(double R11, double R12, double R13, double R21, double R22, double R23, double R31, double R32, double R33)
```
Construct from a rotation matrix, as doubles in *row-major* order !!!

```cpp
Rot3(const Eigen::MatrixBase< Derived > & R)
```

```cpp
Rot3(const Matrix3 & R)
```

```cpp
Rot3(const SO3 & R)
```

```cpp
Rot3(const Quaternion & q)
```

```cpp
Rot3(double w, double x, double y, double z)
```

## 公开方法

### 方法

```cpp
Rot3 retractCayley(const Vector & omega) const
```
Retraction from R^3 to Rot3 manifold using the Cayley transform.

```cpp
Vector3 localCayley(const Rot3 & other) const
```
Inverse of retractCayley.

```cpp
Matrix3 AdjointMap() const
```

```cpp
static Rot3 Expmap(const Vector3 & v, OptionalJacobian< 3, 3 > H = {})
```

```cpp
static Vector3 Logmap(const Rot3 & R, OptionalJacobian< 3, 3 > H = {})
```

```cpp
static Matrix3 ExpmapDerivative(const Vector3 & x)
```
Derivative of Expmap.

```cpp
static Matrix3 LogmapDerivative(const Vector3 & x)
```
Derivative of Logmap.

```cpp
static Matrix3 adjointMap(const Vector3 & xi)
```
Matrix representation of the Lie-algebra adjoint operator ad_xi on so(3).

```cpp
static Matrix3 Hat(const Vector3 & xi)
```
Hat maps from tangent vector to Lie algebra.

```cpp
static Vector3 Vee(const Matrix3 & X)
```
Vee maps from Lie algebra to tangent vector.

```cpp
Rot3 normalized() const
```

```cpp
static Rot3 Random(std::mt19937 & rng)
```

```cpp
static Rot3 Rx(double t)
```
Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.

```cpp
static Rot3 Ry(double t)
```
Rotation around Y axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.

```cpp
static Rot3 Rz(double t)
```
Rotation around Z axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.

```cpp
static Rot3 RzRyRx(double x, double y, double z, OptionalJacobian< 3, 1 > Hx = {}, OptionalJacobian< 3, 1 > Hy = {}, OptionalJacobian< 3, 1 > Hz = {})
```
Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.

```cpp
static Rot3 RzRyRx(const Vector & xyz, OptionalJacobian< 3, 3 > H = {})
```
Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.

```cpp
static Rot3 Yaw(double t)
```
Positive yaw is to right (as in aircraft heading). See ypr.

```cpp
static Rot3 Pitch(double t)
```
Positive pitch is up (increasing aircraft altitude).See ypr.

```cpp
static Rot3 Roll(double t)
```
Positive roll is to right (increasing yaw in aircraft).

```cpp
static Rot3 Ypr(double y, double p, double r, OptionalJacobian< 3, 1 > Hy = {}, OptionalJacobian< 3, 1 > Hp = {}, OptionalJacobian< 3, 1 > Hr = {})
```

```cpp
static Rot3 Quaternion(double w, double x, double y, double z)
```

```cpp
static Rot3 AxisAngle(const Point3 & axis, double angle)
```

```cpp
static Rot3 AxisAngle(const Unit3 & axis, double angle)
```

```cpp
static Rot3 Rodrigues(const Vector3 & w)
```

```cpp
static Rot3 Rodrigues(double wx, double wy, double wz)
```

```cpp
static Rot3 AlignPair(const Unit3 & axis, const Unit3 & a_p, const Unit3 & b_p)
```
Determine a rotation to bring two vectors into alignment, using the rotation axis provided.

```cpp
static Rot3 AlignTwoPairs(const Unit3 & a_p, const Unit3 & b_p, const Unit3 & a_q, const Unit3 & b_q)
```
Calculate rotation from two pairs of homogeneous points using two successive rotations.

```cpp
static Rot3 ClosestTo(const Matrix3 & M)
```

```cpp
print(const std::string & s = "") const
```

```cpp
bool equals(const Rot3 & p, double tol = 1e-9) const
```

```cpp
Rot3 operator*(const Rot3 & R2) const
```
Syntatic sugar for composing two rotations.

```cpp
Rot3 inverse() const
```
inverse of a rotation

```cpp
Rot3 conjugate(const Rot3 & cRb) const
```

```cpp
static Rot3 Identity()
```
identity rotation for group operation

```cpp
Point3 rotate(const Point3 & p, OptionalJacobian< 3, 3 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

```cpp
Point3 operator*(const Point3 & p) const
```
rotate point from rotated coordinate frame to world = R*p

```cpp
Point3 unrotate(const Point3 & p, OptionalJacobian< 3, 3 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```
rotate point from world to rotated frame $ p^c = (R_c^w)^T p^w $

```cpp
Unit3 rotate(const Unit3 & p, OptionalJacobian< 2, 3 > HR = {}, OptionalJacobian< 2, 2 > Hp = {}) const
```
rotate 3D direction from rotated coordinate frame to world frame

```cpp
Unit3 unrotate(const Unit3 & p, OptionalJacobian< 2, 3 > HR = {}, OptionalJacobian< 2, 2 > Hp = {}) const
```
unrotate 3D direction from world frame to rotated coordinate frame

```cpp
Unit3 operator*(const Unit3 & p) const
```
rotate 3D direction from rotated coordinate frame to world frame

```cpp
Matrix3 matrix() const
```

```cpp
Matrix3 transpose() const
```

```cpp
Point3 r1() const
```
first column

```cpp
Point3 r2() const
```
second column

```cpp
Point3 r3() const
```
third column

```cpp
Vector3 xyz(OptionalJacobian< 3, 3 > H = {}) const
```

```cpp
Vector3 ypr(OptionalJacobian< 3, 3 > H = {}) const
```

```cpp
Vector3 rpy(OptionalJacobian< 3, 3 > H = {}) const
```

```cpp
double roll(OptionalJacobian< 1, 3 > H = {}) const
```

```cpp
double pitch(OptionalJacobian< 1, 3 > H = {}) const
```

```cpp
double yaw(OptionalJacobian< 1, 3 > H = {}) const
```

```cpp
std::pair< Unit3, double > axisAngle() const
```

```cpp
Quaternion toQuaternion() const
```

```cpp
Rot3 slerp(double t, const Rot3 & other) const
```
Spherical Linear intERPolation between *this and other.

```cpp
Vector9 vec(OptionalJacobian< 9, 3 > H = {}) const
```
Vee maps from Lie algebra to tangent vector.

## 类型别名

```cpp
using LieAlgebra = Matrix3
```

## 公开成员变量

```cpp
constexpr size_t MatrixM
```

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Rot3` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
