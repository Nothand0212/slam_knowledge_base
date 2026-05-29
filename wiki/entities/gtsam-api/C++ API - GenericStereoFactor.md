---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, GenericStereoFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::GenericStereoFactor

> **类** | 头文件: `StereoFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< POSE, LANDMARK >`

## 构造函数

```cpp
GenericStereoFactor()
```

```cpp
GenericStereoFactor(const StereoPoint2 & measured, const SharedNoiseModel & model, Key poseKey, Key landmarkKey, const Cal3_S2Stereo::shared_ptr & K, std::optional< POSE > body_P_sensor = {})
```

```cpp
GenericStereoFactor(const StereoPoint2 & measured, const SharedNoiseModel & model, Key poseKey, Key landmarkKey, const Cal3_S2Stereo::shared_ptr & K, bool throwCheirality, bool verboseCheirality, std::optional< POSE > body_P_sensor = {})
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
bool equals(const NonlinearFactor & f, double tol = 1e-9) const
```

```cpp
Vector evaluateError(const Pose3 & pose, const Point3 & point, OptionalMatrixType H1, OptionalMatrixType H2) const
```

```cpp
const StereoPoint2 & measured() const
```

```cpp
const Cal3_S2Stereo::shared_ptr calibration() const
```

```cpp
bool verboseCheirality() const
```

```cpp
bool throwCheirality() const
```

```cpp
const std::optional< POSE > & body_P_sensor() const
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
using This = GenericStereoFactor< POSE, LANDMARK >
```
```cpp
using shared_ptr = std::shared_ptr< GenericStereoFactor >
```
```cpp
using CamPose = POSE
```

## 详细说明

A Generic Stereo Factor

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`GenericStereoFactor` 用于 GTSAM factor graph 优化流程中。

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
