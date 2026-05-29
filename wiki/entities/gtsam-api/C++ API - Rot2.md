---
type: entity
tags: [GTSAM, C++ API, Geometry, Rot2]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Rot2

> **类** | 头文件: `Rot2.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::MatrixLieGroup< Rot2, 1, 2 >`

## 构造函数

```cpp
Rot2()
```

```cpp
Rot2(const Rot2 & r)
```

```cpp
Rot2(double theta)
```
Constructor from angle in radians == exponential map at identity.

## 公开方法

### 方法

```cpp
Matrix1 AdjointMap() const
```

```cpp
static Rot2 Expmap(const Vector1 & v, ChartJacobian H = {})
```
Exponential map at identity - create a rotation from canonical coordinates.

```cpp
static Vector1 Logmap(const Rot2 & r, ChartJacobian H = {})
```
Log map at identity - return the canonical coordinates of this rotation.

```cpp
static Matrix1 adjointMap(const Vector1 & )
```
Lie-algebra adjoint (zero for abelian SO(2)).

```cpp
static Vector1 adjoint(const Vector1 & , const Vector1 & , OptionalJacobian< 1, 1 > Hxi = {}, OptionalJacobian< 1, 1 > Hy = {})
```
Apply Lie-algebra adjoint (always zero).

```cpp
static Matrix ExpmapDerivative(const Vector & )
```
Left-trivialized derivative of the exponential map.

```cpp
static Matrix LogmapDerivative(const Vector & )
```
Left-trivialized derivative inverse of the exponential map.

```cpp
static Matrix2 Hat(const Vector1 & xi)
```
Hat maps from tangent vector to Lie algebra.

```cpp
static Vector1 Vee(const Matrix2 & X)
```
Vee maps from Lie algebra to tangent vector.

```cpp
Rot2 & operator=(const Rot2 & other)
```

```cpp
static Rot2 fromAngle(double theta)
```
Named constructor from angle in radians.

```cpp
static Rot2 fromDegrees(double theta)
```
Named constructor from angle in degrees.

```cpp
static Rot2 fromCosSin(double c, double s)
```
Named constructor from cos(theta),sin(theta) pair.

```cpp
static Rot2 relativeBearing(const Point2 & d, OptionalJacobian< 1, 2 > H = {})
```

```cpp
static Rot2 atan2(double y, double x)
```

```cpp
static Rot2 Random(std::mt19937 & rng)
```

```cpp
print(const std::string & s = "theta") const
```

```cpp
bool equals(const Rot2 & R, double tol = 1e-9) const
```

```cpp
Rot2 inverse() const
```

```cpp
Rot2 operator*(const Rot2 & R) const
```

```cpp
static Rot2 Identity()
```

```cpp
Point2 rotate(const Point2 & p, OptionalJacobian< 2, 1 > H1 = {}, OptionalJacobian< 2, 2 > H2 = {}) const
```

```cpp
Point2 operator*(const Point2 & p) const
```

```cpp
Point2 unrotate(const Point2 & p, OptionalJacobian< 2, 1 > H1 = {}, OptionalJacobian< 2, 2 > H2 = {}) const
```

```cpp
Point2 unit() const
```
Creates a unit vector as a Point2.

```cpp
double theta() const
```

```cpp
double degrees() const
```

```cpp
double c() const
```

```cpp
double s() const
```

```cpp
Matrix2 matrix() const
```

```cpp
Matrix2 transpose() const
```

```cpp
Vector4 vec(OptionalJacobian< 4, 1 > H = {}) const
```

```cpp
static Rot2 ClosestTo(const Matrix2 & M)
```

## 类型别名

```cpp
using LieAlgebra = Matrix2
```

## 详细说明

Rotation matrix NOTE: the angle theta is in radians unless explicitly stated

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Rot2` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
