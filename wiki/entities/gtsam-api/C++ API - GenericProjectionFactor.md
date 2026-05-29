---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, GenericProjectionFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::GenericProjectionFactor

> **类** | 头文件: `ProjectionFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< Pose3, Point3 >`

## 构造函数

```cpp
GenericProjectionFactor()
```
Default constructor.

```cpp
GenericProjectionFactor(const Point2 & measured, const SharedNoiseModel & model, Key poseKey, Key pointKey, const std::shared_ptr< CALIBRATION > & K, std::optional< POSE > body_P_sensor = {})
```

```cpp
GenericProjectionFactor(const Point2 & measured, const SharedNoiseModel & model, Key poseKey, Key pointKey, const std::shared_ptr< CALIBRATION > & K, bool throwCheirality, bool verboseCheirality, std::optional< POSE > body_P_sensor = {})
```

## 公开方法

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```

```cpp
bool equals(const NonlinearFactor & p, double tol = 1e-9) const
```
equals

```cpp
Vector evaluateError(const Pose3 & pose, const Point3 & point, OptionalMatrixType H1, OptionalMatrixType H2) const
```
Evaluate error h(x)-z and optionally derivatives.

```cpp
const Point2 & measured() const
```

```cpp
const std::shared_ptr< CALIBRATION > calibration() const
```

```cpp
const std::optional< POSE > & body_P_sensor() const
```

```cpp
bool verboseCheirality() const
```

```cpp
bool throwCheirality() const
```

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

## 类型别名

```cpp
using Base = NoiseModelFactorN< POSE, LANDMARK >
```
```cpp
using This = GenericProjectionFactor< POSE, LANDMARK, CALIBRATION >
```
```cpp
using shared_ptr = std::shared_ptr< This >
```

## 详细说明

Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here. The main building block for visual SLAM.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`GenericProjectionFactor` 用于 GTSAM factor graph 优化流程中。

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
