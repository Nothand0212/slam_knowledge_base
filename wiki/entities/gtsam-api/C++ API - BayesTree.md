---
type: entity
tags: [GTSAM, C++ API, Inference, BayesTree]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::BayesTree

> **类** | 头文件: `BayesTree.h` | [在线文档](https://gtsam.org/doxygen/)

## 公开方法

### 方法

```cpp
Key findParentClique(const CONTAINER & parents) const
```

```cpp
clear()
```

```cpp
deleteCachedShortcuts()
```

```cpp
removePath(sharedClique clique, BayesNetType * bn, Cliques * orphans)
```

```cpp
removeTop(const KeyVector & keys, BayesNetType * bn, Cliques * orphans)
```

```cpp
Cliques removeSubtree(const sharedClique & subtree)
```

```cpp
insertRoot(const sharedClique & subtree)
```

```cpp
addClique(const sharedClique & clique, const sharedClique & parent_clique)
```

```cpp
addFactorsToGraph(FactorGraph< FactorType > * graph) const
```

```cpp
KeySet collectAffectedKeys(const KeyVector & keys) const
```
Returns the set of keys from the tree that are affected by a update to 'keys'.

```cpp
bool equals(const This & other, double tol = 1e-9) const
```

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```

```cpp
size_t size() const
```

```cpp
bool empty() const
```

```cpp
const Nodes & nodes() const
```

```cpp
sharedClique operator[](Key j) const
```

```cpp
const Roots & roots() const
```

```cpp
const sharedClique & clique(Key j) const
```

```cpp
BayesTreeCliqueData getCliqueData() const
```

```cpp
size_t numCachedSeparatorMarginals() const
```

```cpp
sharedConditional marginalFactor(Key j, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
sharedFactorGraph joint(Key j1, Key j2, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
sharedFactorGraph joint(const KeyVector & keys, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
sharedBayesNet jointBayesNet(Key j1, Key j2, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
sharedBayesNet jointBayesNet(const KeyVector & keys, const Eliminate & function = EliminationTraitsType::DefaultEliminate) const
```

```cpp
dot(std::ostream & os, const KeyFormatter & keyFormatter) const
```
Output to graphviz format, stream version.

```cpp
std::string dot(const KeyFormatter & keyFormatter) const
```
Output to graphviz format string.

```cpp
saveGraph(const std::string & filename, const KeyFormatter & keyFormatter) const
```
output to file with graphviz format.

## 类型别名

```cpp
using Clique = CLIQUE
```
```cpp
using sharedClique = std::shared_ptr< Clique >
```
```cpp
using Node = Clique
```
```cpp
using sharedNode = sharedClique
```
```cpp
using ConditionalType = CLIQUE::ConditionalType
```
```cpp
using sharedConditional = std::shared_ptr< ConditionalType >
```
```cpp
using BayesNetType = CLIQUE::BayesNetType
```
```cpp
using sharedBayesNet = std::shared_ptr< BayesNetType >
```
```cpp
using FactorType = CLIQUE::FactorType
```
```cpp
using sharedFactor = std::shared_ptr< FactorType >
```
```cpp
using FactorGraphType = CLIQUE::FactorGraphType
```
```cpp
using sharedFactorGraph = std::shared_ptr< FactorGraphType >
```
```cpp
using Eliminate = FactorGraphType::Eliminate
```
```cpp
using EliminationTraitsType = CLIQUE::EliminationTraitsType
```
```cpp
using Cliques = FastList< sharedClique >
```
```cpp
using Nodes = ConcurrentMap< Key, sharedClique >
```
```cpp
using Roots = FastVector< sharedClique >
```

## 详细说明

Bayes tree 

CONDITIONAL


The type of the conditional densities, i.e. the type of node in the underlying Bayes chain, which could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional. 




CLIQUE


The type of the clique data structure, defaults to BayesTreeClique, normally do not change this as it is only used when developing special versions of BayesTree, e.g. for ISAM2. The type of the conditional densities, i.e. the type of node in the underlying Bayes chain, which could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional. The type of the clique data structure, defaults to BayesTreeClique, normally do not change this as it is only used when developing special versions of BayesTree, e.g. for ISAM2.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`BayesTree` 用于 GTSAM factor graph 优化流程中。

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
