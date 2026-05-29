---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, BetweenFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::BetweenFactor

> **类** | 头文件: `BetweenFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< VALUE, VALUE >`

## 构造函数

```cpp
BetweenFactor()
```

```cpp
BetweenFactor(Key key1, Key key2, const VALUE & measured, const SharedNoiseModel & model)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```
print with optional string

```cpp
bool equals(const NonlinearFactor & expected, double tol = 1e-9) const
```
assert equality up to a tolerance

```cpp
Vector evaluateError(const T & p1, const T & p2, OptionalMatrixType H1, OptionalMatrixType H2) const
```
evaluate error, returns vector of errors size of tangent space

```cpp
const VALUE & measured() const
```
return the measurement

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
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
using T = VALUE
```
```cpp
using shared_ptr = std::shared_ptr< BetweenFactor >
```

## 详细说明

A class for a measurement predicted by "between(config[key1],config[key2])" 

VALUE


the Value type the Value type

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`BetweenFactor` 用于 GTSAM factor graph 优化流程中。

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
