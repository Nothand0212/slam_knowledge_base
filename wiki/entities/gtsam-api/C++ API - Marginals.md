---
type: entity
tags: [GTSAM, C++ API, Inference, Marginals]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Marginals

> **类** | 头文件: `Marginals.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
Marginals()
```
Default constructor only for wrappers.

```cpp
Marginals(const NonlinearFactorGraph & graph, const Values & solution, Factorization factorization)
```

```cpp
Marginals(const NonlinearFactorGraph & graph, const Values & solution, const Ordering & ordering, Factorization factorization)
```

```cpp
Marginals(const GaussianFactorGraph & graph, const Values & solution, Factorization factorization)
```

```cpp
Marginals(const GaussianFactorGraph & graph, const Values & solution, const Ordering & ordering, Factorization factorization)
```

```cpp
Marginals(const GaussianFactorGraph & graph, const VectorValues & solution, Factorization factorization)
```

```cpp
Marginals(const GaussianFactorGraph & graph, const VectorValues & solution, const Ordering & ordering, Factorization factorization)
```

```cpp
Marginals(GaussianBayesTree && bayesTree, const VectorValues & solution, Factorization factorization)
```

## 公开方法

### 方法

```cpp
print(const std::string & str = "Marginals: ", const KeyFormatter & keyFormatter) const
```

```cpp
GaussianFactor::shared_ptr marginalFactor(Key variable) const
```

```cpp
Matrix marginalInformation(Key variable) const
```
Compute the marginal information matrix of a single variable.

```cpp
Matrix marginalCovariance(Key variable) const
```
Compute the marginal covariance of a single variable.

```cpp
JointMarginal jointMarginalCovariance(const KeyVector & variables) const
```
Compute the joint marginal covariance of several variables.

```cpp
JointMarginal jointMarginalInformation(const KeyVector & variables) const
```
Compute the joint marginal information of several variables.

```cpp
deleteCachedShortcuts()
```

```cpp
VectorValues optimize() const
```

## 详细说明

A class for computing Gaussian marginals of variables in a NonlinearFactorGraph

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Marginals` 用于 GTSAM factor graph 优化流程中。

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
