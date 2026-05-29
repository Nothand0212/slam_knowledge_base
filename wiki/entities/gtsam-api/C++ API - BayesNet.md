---
type: entity
tags: [GTSAM, C++ API, Inference, BayesNet]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::BayesNet

> **类** | 头文件: `BayesNet.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::FactorGraph< CONDITIONAL >`

## 公开方法

### 方法

```cpp
print(const std::string & s = "BayesNet", const KeyFormatter & formatter) const
```

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

```cpp
double logProbability(const HybridValues & x) const
```

```cpp
double evaluate(const HybridValues & c) const
```

## 类型别名

```cpp
using sharedConditional = std::shared_ptr< CONDITIONAL >
```

## 详细说明

A BayesNet is a tree of conditionals, stored in elimination order.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`BayesNet` 用于 GTSAM factor graph 优化流程中。

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
