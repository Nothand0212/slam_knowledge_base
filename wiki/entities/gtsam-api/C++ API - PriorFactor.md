---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, PriorFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::PriorFactor

> **类** | 头文件: `PriorFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::ExtendedPriorFactor< VALUE >`

## 构造函数

```cpp
PriorFactor()
```
default constructor - only use for serialization

```cpp
PriorFactor(Key key, const VALUE & prior, const SharedNoiseModel & model)
```
Constructor.

```cpp
PriorFactor(Key key, const VALUE & prior, const Matrix & covariance)
```
Convenience constructor that takes a full covariance argument.

## 公开方法

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
print(const std::string & s, const KeyFormatter & keyFormatter) const
```
print

```cpp
bool equals(const NonlinearFactor & expected, double tol = 1e-9) const
```
equals

```cpp
const VALUE & prior() const
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< PriorFactor< VALUE > >
```
```cpp
using This = PriorFactor< VALUE >
```
```cpp
using T = VALUE
```
```cpp
using Base = ExtendedPriorFactor< VALUE >
```

## 详细说明

A class for a soft prior on any Value type.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`PriorFactor` 用于 GTSAM factor graph 优化流程中。

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
