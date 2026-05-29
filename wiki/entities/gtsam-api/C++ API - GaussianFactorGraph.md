---
type: entity
tags: [GTSAM, C++ API, FactorGraph, GaussianFactorGraph]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::GaussianFactorGraph

> **类** | 头文件: `GaussianFactorGraph.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::FactorGraph< GaussianFactor >`
- 继承自 `gtsam::EliminateableFactorGraph< GaussianFactorGraph >`

## 构造函数

```cpp
GaussianFactorGraph()
```

```cpp
GaussianFactorGraph(std::initializer_list< sharedFactor > factors)
```

```cpp
GaussianFactorGraph(ITERATOR firstFactor, ITERATOR lastFactor)
```

```cpp
GaussianFactorGraph(const CONTAINER & factors)
```

```cpp
GaussianFactorGraph(const FactorGraph< DERIVEDFACTOR > & graph)
```

## 公开方法

### 方法

```cpp
bool equals(const This & fg, double tol = 1e-9) const
```

```cpp
std::vector< std::tuple< int, int, double > > sparseJacobian(const Ordering & ordering, size_t & nrows, size_t & ncols) const
```

```cpp
std::vector< std::tuple< int, int, double > > sparseJacobian() const
```

```cpp
Matrix sparseJacobian_() const
```

```cpp
Matrix augmentedJacobian(const Ordering & ordering) const
```

```cpp
Matrix augmentedJacobian() const
```

```cpp
std::pair< Matrix, Vector > jacobian(const Ordering & ordering) const
```

```cpp
std::pair< Matrix, Vector > jacobian() const
```

```cpp
Matrix augmentedHessian(const Ordering & ordering) const
```

```cpp
Matrix augmentedHessian() const
```

```cpp
std::pair< Matrix, Vector > hessian(const Ordering & ordering) const
```

```cpp
std::pair< Matrix, Vector > hessian() const
```

```cpp
VectorValues hessianDiagonal() const
```

```cpp
std::map< Key, Matrix > hessianBlockDiagonal() const
```

```cpp
VectorValues optimize(const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
VectorValues optimize(const Ordering & ordering, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
VectorValues optimizeDensely() const
```

```cpp
VectorValues gradient(const VectorValues & x0) const
```

```cpp
VectorValues gradientAtZero() const
```

```cpp
VectorValues optimizeGradientSearch() const
```

```cpp
VectorValues transposeMultiply(const Errors & e) const
```

```cpp
transposeMultiplyAdd(double alpha, const Errors & e, VectorValues & x) const
```

```cpp
Errors gaussianErrors(const VectorValues & x) const
```

```cpp
Errors operator*(const VectorValues & x) const
```
** return A*x */

```cpp
multiplyHessianAdd(double alpha, const VectorValues & x, VectorValues & y) const
```
** y += alpha*A'A*x */

```cpp
multiplyInPlace(const VectorValues & x, Errors & e) const
```
** In-place version e <- A*x that overwrites e. */

```cpp
multiplyInPlace(const VectorValues & x, const Errors::iterator & e) const
```

```cpp
printErrors(const VectorValues & x, const std::string & str = "GaussianFactorGraph: ", const KeyFormatter & keyFormatter, const std::function< bool(const Factor *, double, size_t)> & printCondition = []() const
```

### 方法

```cpp
add(const GaussianFactor & factor)
```

```cpp
add(const sharedFactor & factor)
```

```cpp
add(const Vector & b)
```

```cpp
add(Key key1, const Matrix & A1, const Vector & b, const SharedDiagonal & model)
```

```cpp
add(Key key1, const Matrix & A1, Key key2, const Matrix & A2, const Vector & b, const SharedDiagonal & model)
```

```cpp
add(Key key1, const Matrix & A1, Key key2, const Matrix & A2, Key key3, const Matrix & A3, const Vector & b, const SharedDiagonal & model)
```

```cpp
add(const TERMS & terms, const Vector & b, const SharedDiagonal & model)
```

```cpp
Keys keys() const
```

```cpp
std::map< Key, size_t > getKeyDimMap() const
```

```cpp
double error(const VectorValues & x) const
```

```cpp
double deltaError(const VectorValues & x, double * oldError, double * newError) const
```

```cpp
double probPrime(const VectorValues & c) const
```

```cpp
GaussianFactorGraph clone() const
```

```cpp
GaussianFactorGraph::shared_ptr cloneToPtr() const
```

```cpp
GaussianFactorGraph negate() const
```

## 类型别名

```cpp
using This = GaussianFactorGraph
```
```cpp
using Base = FactorGraph< GaussianFactor >
```
```cpp
using BaseEliminateable = EliminateableFactorGraph< This >
```
```cpp
using shared_ptr = std::shared_ptr< This >
```
```cpp
using Keys = KeySet
```

## 详细说明

A Linear Factor Graph is a factor graph where all factors are Gaussian, i.e. Factor == GaussianFactor VectorValues = A values structure of vectors Most of the time, linear factor graphs arise by linearizing a non-linear factor graph.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`GaussianFactorGraph` 用于 GTSAM factor graph 优化流程中。

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
