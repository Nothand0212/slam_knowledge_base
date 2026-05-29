---
type: entity
tags: [GTSAM, C++ API, Navigation, ImuFactor2T]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::ImuFactor2T

> **类** | 头文件: `ImuFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< NavState, NavState, imuBias::ConstantBias >`

## 构造函数

```cpp
ImuFactor2T()
```

```cpp
ImuFactor2T(Key state_i, Key state_j, Key bias, const PIM & preintegratedMeasurements)
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
Vector evaluateError(const NavState & state_i, const NavState & state_j, const imuBias::ConstantBias & bias_i, OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3) const
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

## 详细说明

ImuFactor2 is a ternary factor that uses NavStates rather than Pose/Velocity.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`ImuFactor2T` 用于 GTSAM factor graph 优化流程中。

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
