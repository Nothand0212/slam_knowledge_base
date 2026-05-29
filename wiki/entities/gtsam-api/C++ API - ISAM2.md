---
type: entity
tags: [GTSAM, C++ API, ISAM2, ISAM2]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::ISAM2

> **类** | 头文件: `ISAM2.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::BayesTree< ISAM2Clique >`

## 构造函数

```cpp
ISAM2(const ISAM2Params & params)
```

```cpp
ISAM2()
```

## 公开方法

### 方法

```cpp
Values calculateBestEstimate() const
```

```cpp
const VectorValues & getDelta() const
```

```cpp
double error(const VectorValues & x) const
```

```cpp
const NonlinearFactorGraph & getFactorsUnsafe() const
```

```cpp
const VariableIndex & getVariableIndex() const
```

```cpp
const KeySet & getFixedVariables() const
```

```cpp
const ISAM2Params & params() const
```

```cpp
size_t treeNnz() const
```

```cpp
printStats() const
```

```cpp
VectorValues gradientAtZero() const
```

```cpp
std::pair< KeySet, bool > predictUpdateInfo(const NonlinearFactorGraph & newFactors, const Values & newTheta, const ISAM2UpdateParams & updateParams) const
```
Predicts the updated variables for a hypothetical update.

### 方法

```cpp
bool equals(const ISAM2 & other, double tol = 1e-9) const
```

```cpp
ISAM2Result update(const NonlinearFactorGraph & newFactors, const Values & newTheta, const FactorIndices & removeFactorIndices, const std::optional< FastMap< Key, int > > & constrainedKeys = {}, const std::optional< FastList< Key > > & noRelinKeys = {}, const std::optional< FastList< Key > > & extraReelimKeys = {}, bool force_relinearize)
```

```cpp
ISAM2Result update(const NonlinearFactorGraph & newFactors, const Values & newTheta, const ISAM2UpdateParams & updateParams)
```

```cpp
marginalizeLeaves(const FastList< Key > & leafKeys, FactorIndices * marginalFactorsIndices, FactorIndices * deletedFactorsIndices)
```

```cpp
marginalizeLeaves(const FastList< Key > & leafKeys, OptArgs &&... optArgs)
```

```cpp
const Values & getLinearizationPoint() const
```
Access the current linearization point.

```cpp
bool valueExists(Key key) const
```
Check whether variable with given key exists in linearization point.

```cpp
Values calculateEstimate() const
```

```cpp
VALUE calculateEstimate(Key key) const
```

```cpp
const Value & calculateEstimate(Key key) const
```

```cpp
Matrix marginalInformation(Key key) const
```
Return the marginal information matrix on any variable.

```cpp
Matrix marginalCovariance(Key key) const
```
Return the marginal covariance matrix on any variable.

```cpp
JointMarginal jointMarginalCovariance(const KeyVector & queryKeys) const
```
Return the joint marginal covariance on a set of variables.

```cpp
JointMarginal jointMarginalInformation(const KeyVector & queryKeys) const
```
Return the joint marginal information on a set of variables.

## 类型别名

```cpp
using This = ISAM2
```
```cpp
using Base = BayesTree< ISAM2Clique >
```
```cpp
using Clique = Base::Clique
```
```cpp
using sharedClique = Base::sharedClique
```
```cpp
using Cliques = Base::Cliques
```

## 详细说明

Implementation of the full ISAM2 algorithm for incremental nonlinear optimization. The typical cycle of using this class to create an instance by providing ISAM2Params to the constructor, then add measurements and variables as they arrive using the update() method. At any time, calculateEstimate() may be called to obtain the current estimate of all variables.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`ISAM2` 用于 GTSAM factor graph 优化流程中。

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
