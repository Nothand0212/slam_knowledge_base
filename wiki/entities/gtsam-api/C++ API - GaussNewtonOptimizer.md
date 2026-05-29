---
type: entity
tags: [GTSAM, C++ API, Optimization, GaussNewtonOptimizer]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::GaussNewtonOptimizer

> **类** | 头文件: `GaussNewtonOptimizer.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NonlinearOptimizer`

## 构造函数

```cpp
GaussNewtonOptimizer(const NonlinearFactorGraph & graph, const Values & initialValues, const GaussNewtonParams & params)
```

```cpp
GaussNewtonOptimizer(const NonlinearFactorGraph & graph, const Values & initialValues, const Ordering & ordering)
```

## 公开方法

### 方法

```cpp
GaussianFactorGraph::shared_ptr iterate()
```

```cpp
const GaussNewtonParams & params() const
```

## 详细说明

This class performs Gauss-Newton nonlinear optimization

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`GaussNewtonOptimizer` 用于 GTSAM factor graph 优化流程中。

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
