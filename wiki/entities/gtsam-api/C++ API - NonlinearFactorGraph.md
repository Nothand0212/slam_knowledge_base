---
type: entity
tags: [GTSAM, C++ API, FactorGraph, NonlinearFactorGraph]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::NonlinearFactorGraph

> **类** | 头文件: `NonlinearFactorGraph.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::FactorGraph< NonlinearFactor >`

## 构造函数

```cpp
NonlinearFactorGraph()
```

```cpp
NonlinearFactorGraph(ITERATOR firstFactor, ITERATOR lastFactor)
```

```cpp
NonlinearFactorGraph(const CONTAINER & factors)
```

```cpp
NonlinearFactorGraph(const FactorGraph< DERIVEDFACTOR > & graph)
```

## 公开方法

### 方法

```cpp
double error(const Values & values) const
```

```cpp
double probPrime(const Values & values) const
```

```cpp
std::shared_ptr< SymbolicFactorGraph > symbolic() const
```

```cpp
Ordering orderingCOLAMD() const
```

```cpp
Ordering orderingCOLAMDConstrained(const FastMap< Key, int > & constraints) const
```

```cpp
std::shared_ptr< GaussianFactorGraph > linearize(const Values & linearizationPoint) const
```
Linearize a nonlinear factor graph.

```cpp
std::shared_ptr< const NonlinearFactorGraph > cloneShared() const
```
Clone into a shared pointer while preserving derived graph behavior.

```cpp
std::shared_ptr< HessianFactor > linearizeToHessianFactor(const Values & values, const Dampen & dampen) const
```

```cpp
std::shared_ptr< HessianFactor > linearizeToHessianFactor(const Values & values, const Ordering & ordering, const Dampen & dampen) const
```

```cpp
Values updateCholesky(const Values & values, const Dampen & dampen) const
```

```cpp
Values updateCholesky(const Values & values, const Ordering & ordering, const Dampen & dampen) const
```

```cpp
NonlinearFactorGraph clone() const
```
Clone() performs a deep-copy of the graph, including all of the factors.

```cpp
NonlinearFactorGraph rekey(const std::map< Key, Key > & rekey_mapping) const
```

```cpp
addExpressionFactor(const SharedNoiseModel & R, const T & z, const Expression< T > & h)
```

```cpp
addPrior(Key key, const T & prior, const SharedNoiseModel & model)
```

```cpp
addPrior(Key key, const T & prior, const Matrix & covariance)
```

```cpp
print(const std::string & str = "NonlinearFactorGraph: ", const KeyFormatter & keyFormatter) const
```

```cpp
printErrors(const Values & values, const std::string & str = "NonlinearFactorGraph: ", const KeyFormatter & keyFormatter, const std::function< bool(const Factor *, double, size_t)> & printCondition = []() const
```

```cpp
bool equals(const NonlinearFactorGraph & other, double tol = 1e-9) const
```

```cpp
dot(std::ostream & os, const Values & values, const KeyFormatter & keyFormatter, const GraphvizFormatting & writer) const
```
Output to graphviz format, stream version, with Values/extra options.

```cpp
std::string dot(const Values & values, const KeyFormatter & keyFormatter, const GraphvizFormatting & writer) const
```
Output to graphviz format string, with Values/extra options.

```cpp
saveGraph(const std::string & filename, const Values & values, const KeyFormatter & keyFormatter, const GraphvizFormatting & writer) const
```
output to file with graphviz format, with Values/extra options.

```cpp
dot(std::ostream & os, const KeyFormatter & keyFormatter, const DotWriter & writer) const
```
Output to graphviz format, stream version.

```cpp
std::string dot(const KeyFormatter & keyFormatter, const DotWriter & writer) const
```
Output to graphviz format string.

```cpp
saveGraph(const std::string & filename, const KeyFormatter & keyFormatter, const DotWriter & writer) const
```
output to file with graphviz format.

## 类型别名

```cpp
using Dampen = std::function< void(const std::shared_ptr< HessianFactor > &hessianFactor)>
```
```cpp
using Base = FactorGraph< NonlinearFactor >
```
```cpp
using This = NonlinearFactorGraph
```
```cpp
using shared_ptr = std::shared_ptr< This >
```

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`NonlinearFactorGraph` 用于 GTSAM factor graph 优化流程中。

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
