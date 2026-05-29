---
type: entity
tags: [GTSAM, C++ API, Navigation, NavState]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::NavState

> **类** | 头文件: `NavState.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::ExtendedPose3< 2, NavState >`

## 构造函数

```cpp
NavState()
```
Default constructor.

```cpp
NavState(const Base & other)
```

```cpp
NavState(const Rot3 & R, const Point3 & t, const Velocity3 & v)
```
Construct from attitude, position, velocity.

```cpp
NavState(const Pose3 & pose, const Velocity3 & v)
```
Construct from pose and velocity.

```cpp
NavState(const Matrix3 & R, const Vector6 & tv)
```
Construct from SO(3) and R^6.

```cpp
NavState(const Matrix5 & T)
```
Construct from Matrix5.

## 公开方法

### 方法

```cpp
static NavState Create(const Rot3 & R, const Point3 & t, const Velocity3 & v, OptionalJacobian< 9, 3 > H1 = {}, OptionalJacobian< 9, 3 > H2 = {}, OptionalJacobian< 9, 3 > H3 = {})
```
Named constructor with derivatives.

```cpp
static NavState FromPoseVelocity(const Pose3 & pose, const Vector3 & vel, OptionalJacobian< 9, 6 > H1 = {}, OptionalJacobian< 9, 3 > H2 = {})
```
Named constructor with derivatives.

```cpp
const Rot3 & attitude(OptionalJacobian< 3, 9 > H = {}) const
```

```cpp
Point3 position(OptionalJacobian< 3, 9 > H = {}) const
```

```cpp
Velocity3 velocity(OptionalJacobian< 3, 9 > H = {}) const
```

```cpp
const Pose3 pose() const
```

```cpp
double range(const Point3 & point, OptionalJacobian< 1, 9 > Hself = {}, OptionalJacobian< 1, 3 > Hpoint = {}) const
```

```cpp
Unit3 bearing(const Point3 & point, OptionalJacobian< 2, 9 > Hself = {}, OptionalJacobian< 2, 3 > Hpoint = {}) const
```

```cpp
Matrix3 R() const
```
Return rotation matrix. Induces computation in quaternion mode.

```cpp
Quaternion quaternion() const
```
Return quaternion. Induces computation in matrix mode.

```cpp
Vector3 t() const
```
Return position as Vector3.

```cpp
Vector3 v() const
```
Return velocity as Vector3.

```cpp
Velocity3 bodyVelocity(OptionalJacobian< 3, 9 > H = {}) const
```

```cpp
print(const std::string & s = "") const
```
print

```cpp
bool equals(const NavState & other, double tol = 1e-8) const
```
equals

```cpp
const Rot3 & rotation(OptionalJacobian< 3, 9 > H = {}) const
```
Syntactic sugar.

```cpp
NavState retract(const Vector9 & v, OptionalJacobian< 9, 9 > H1 = {}, OptionalJacobian< 9, 9 > H2 = {}) const
```

```cpp
Vector9 localCoordinates(const NavState & g, OptionalJacobian< 9, 9 > H1 = {}, OptionalJacobian< 9, 9 > H2 = {}) const
```

```cpp
static Eigen::Block< Vector9, 3, 1 > dR(Vector9 & v)
```

```cpp
static Eigen::Block< Vector9, 3, 1 > dP(Vector9 & v)
```

```cpp
static Eigen::Block< Vector9, 3, 1 > dV(Vector9 & v)
```

```cpp
static Eigen::Block< const Vector9, 3, 1 > dR(const Vector9 & v)
```

```cpp
static Eigen::Block< const Vector9, 3, 1 > dP(const Vector9 & v)
```

```cpp
static Eigen::Block< const Vector9, 3, 1 > dV(const Vector9 & v)
```

```cpp
NavState update(const Vector3 & b_acceleration, const Vector3 & b_omega, const double dt, OptionalJacobian< 9, 9 > F = {}, OptionalJacobian< 9, 3 > G1 = {}, OptionalJacobian< 9, 3 > G2 = {}) const
```
Uses second order integration for position, returns derivatives except dt.

```cpp
Vector9 coriolis(double dt, const Vector3 & omega, bool secondOrder, OptionalJacobian< 9, 9 > H = {}) const
```
Compute tangent space contribution due to Coriolis forces.

```cpp
Vector9 correctPIM(const Vector9 & pim, double dt, const Vector3 & n_gravity, const std::optional< Vector3 > & omegaCoriolis, bool use2ndOrderCoriolis, OptionalJacobian< 9, 9 > H1 = {}, OptionalJacobian< 9, 9 > H2 = {}) const
```

## 类型别名

```cpp
using Base = ExtendedPose3< 2, NavState >
```
```cpp
using LieAlgebra = Matrix5
```
```cpp
using Vector25 = Eigen::Matrix< double, 25, 1 >
```

## 公开成员变量

```cpp
constexpr auto dimension
```

## 详细说明

Navigation state: Pose (rotation, translation) + velocity Following Barrau20icra, this class belongs to the Lie group SE_2(3). This group is also called "double direct isometries”. NOTE: While Barrau20icra follow a R,v,t order, we use a R,t,v order to maintain backwards compatibility.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`NavState` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM Geometry API]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
