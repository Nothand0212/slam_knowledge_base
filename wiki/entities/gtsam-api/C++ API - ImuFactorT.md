---
type: entity
tags: [GTSAM, C++ API, Navigation, ImuFactorT]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::ImuFactorT

> **类** | 头文件: `ImuFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< Pose3, Vector3, Pose3, Vector3, imuBias::ConstantBias >`

## 构造函数

```cpp
ImuFactorT()
```

```cpp
ImuFactorT(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias, const PIM & preintegratedMeasurements)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & formatter) const
```
print

```cpp
bool equals(const NonlinearFactor & f, double tol = 1e-9) const
```

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
const PIM & preintegratedMeasurements() const
```

```cpp
Vector evaluateError(const Pose3 & pose_i, const Vector3 & vel_i, const Pose3 & pose_j, const Vector3 & vel_j, const imuBias::ConstantBias & bias_i, OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3, OptionalMatrixType H4, OptionalMatrixType H5) const
```
vector of errors

```cpp
OutputVec evaluateError(const ValueTypes &... x, OptionalMatrixTypeT< ValueTypes >... H) const
```

```cpp
Vector evaluateError(const ValueTypes &... x, MatrixTypeT< ValueTypes > &... H) const
```

```cpp
Vector evaluateError(const ValueTypes &... x) const
```

```cpp
AreAllMatrixRefs< Vector, OptionalJacArgs... > evaluateError(const ValueTypes &... x, OptionalJacArgs &&... H) const
```

```cpp
AreAllMatrixPtrs< Vector, OptionalJacArgs... > evaluateError(const ValueTypes &... x, OptionalJacArgs &&... H) const
```

### 静态方法

```cpp
static MethodPIMArg Merge(const MethodPIMArg & pim01, const MethodPIMArg & pim12)
```
Merge two pre-integrated measurement classes.

```cpp
static ImuFactorT< MethodPIMArg >::shared_ptr Merge(const typename ImuFactorT< MethodPIMArg >::shared_ptr & f01, const typename ImuFactorT< MethodPIMArg >::shared_ptr & f12)
```
Merge two factors.

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< This >
```

## 详细说明

ImuFactor is a 5-ways factor involving previous state (pose and velocity of the vehicle at previous time step), current state (pose and velocity at current time step), and the bias estimate. Following the preintegration scheme proposed in [2], the ImuFactor includes many IMU measurements, which are "summarized" using the PreintegratedImuMeasurements class. Note that this factor does not model "temporal consistency" of the biases (which are usually slowly varying quantities), which is up to the caller. See also CombinedImuFactor for a class that does this for you.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`ImuFactorT` 用于 GTSAM factor graph 优化流程中。

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
