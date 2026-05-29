---
type: entity
tags: [GTSAM, C++ API, Inference, Ordering]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Ordering

> **类** | 头文件: `Ordering.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `KeyVector`

## 构造函数

```cpp
Ordering()
```
Create an empty ordering

```cpp
Ordering(const KEYS & keys)
```
Create from a container.

## 公开方法

### 方法

```cpp
print(const std::string & str = "", const KeyFormatter & keyFormatter) const
```

```cpp
bool equals(const Ordering & other, double tol = 1e-9) const
```

```cpp
static Ordering Colamd(const FACTOR_GRAPH & graph)
```

```cpp
static Ordering Colamd(const VariableIndex & variableIndex)
```
Compute a fill-reducing ordering using COLAMD from a VariableIndex.

```cpp
static Ordering ColamdConstrainedLast(const FACTOR_GRAPH & graph, const KeyVector & constrainLast, bool forceOrder)
```

```cpp
static Ordering ColamdConstrainedLast(const VariableIndex & variableIndex, const KeyVector & constrainLast, bool forceOrder)
```

```cpp
static Ordering ColamdConstrainedFirst(const FACTOR_GRAPH & graph, const KeyVector & constrainFirst, bool forceOrder)
```

```cpp
static Ordering ColamdConstrainedFirst(const VariableIndex & variableIndex, const KeyVector & constrainFirst, bool forceOrder)
```

```cpp
static Ordering ColamdConstrained(const FACTOR_GRAPH & graph, const FastMap< Key, int > & groups)
```

```cpp
static Ordering ColamdConstrained(const VariableIndex & variableIndex, const FastMap< Key, int > & groups)
```

```cpp
static Ordering Natural(const FACTOR_GRAPH & fg)
```
Return a natural Ordering. Typically used by iterative solvers.

```cpp
static CSRFormat(std::vector< int > & xadj, std::vector< int > & adj, const FACTOR_GRAPH & graph)
```
METIS Formatting function.

```cpp
static Ordering Metis(const MetisIndex & met)
```
Compute an ordering determined by METIS from a VariableIndex.

```cpp
static Ordering Metis(const FACTOR_GRAPH & graph)
```

```cpp
static Ordering Create(OrderingType orderingType, const FACTOR_GRAPH & graph)
```

### 方法

```cpp
This & operator+=(Key key)
```

```cpp
This & operator,(Key key)
```
Overloading the comma operator allows for chaining appends.

```cpp
This & operator+=(KeyVector & keys)
```
Append new keys to the ordering as `ordering += keys`.

```cpp
bool contains(const Key & key) const
```
Check if key exists in ordering.

```cpp
FastMap< Key, size_t > invert() const
```
Invert (not reverse) the ordering - returns a map from key to order position.

## 类型别名

```cpp
using This = Ordering
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

`Ordering` 用于 GTSAM factor graph 优化流程中。

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
