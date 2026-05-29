---
type: entity
tags: [GTSAM, C++ API, Navigation, CombinedImuFactorT]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::CombinedImuFactorT

> **类** | 头文件: `CombinedImuFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< Pose3, Vector3, Pose3, Vector3, imuBias::ConstantBias, imuBias::ConstantBias >`

## 构造函数

```cpp
CombinedImuFactorT()
```

```cpp
CombinedImuFactorT(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j, const PIM & preintegratedMeasurements)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & formatter) const
```
print

```cpp
bool equals(const NonlinearFactor & expected, double tol = 1e-9) const
```
equals

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
const PIM & preintegratedMeasurements() const
```

```cpp
Vector evaluateError(const Pose3 & pose_i, const Vector3 & vel_i, const Pose3 & pose_j, const Vector3 & vel_j, const imuBias::ConstantBias & bias_i, const imuBias::ConstantBias & bias_j, OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3, OptionalMatrixType H4, OptionalMatrixType H5, OptionalMatrixType H6) const
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

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< This >
```

## 详细说明

CombinedImuFactor is a 6-ways factor involving previous state (pose and velocity of the vehicle, as well as bias at previous time step), and current state (pose, velocity, bias at current time step). Following the pre- integration scheme proposed in [2], the CombinedImuFactor includes many IMU measurements, which are "summarized" using the PreintegratedCombinedMeasurements class. There are 3 main differences wrpt the ImuFactor class: 1) The factor is 6-ways, meaning that it also involves both biases (previous and current time step).Therefore, the factor internally imposes the biases to be slowly varying; in particular, the matrices "biasAccCovariance" and "biasOmegaCovariance" described the random walk that models bias evolution. 2) The preintegration covariance takes into account the noise in the bias estimate used for integration. 3) The covariance matrix of the PreintegratedCombinedMeasurements preserves the correlation between the bias uncertainty and the preintegrated measurements uncertainty.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`CombinedImuFactorT` 用于 GTSAM factor graph 优化流程中。

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
