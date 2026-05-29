---
type: entity
tags: [GTSAM, C++ API, Optimization, LevenbergMarquardtOptimizer]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::LevenbergMarquardtOptimizer

> **类** | 头文件: `LevenbergMarquardtOptimizer.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NonlinearOptimizer`

## 构造函数

```cpp
LevenbergMarquardtOptimizer(const NonlinearFactorGraph & graph, const Values & initialValues, const LevenbergMarquardtParams & params)
```

```cpp
LevenbergMarquardtOptimizer(const NonlinearFactorGraph & graph, const Values & initialValues, const Ordering & ordering, const LevenbergMarquardtParams & params)
```

## 公开方法

### 方法

```cpp
double lambda() const
```
Access the current damping value.

```cpp
int getInnerIterations() const
```
Access the current number of inner iterations.

```cpp
print(const std::string & str = "") const
```
print

```cpp
GaussianFactorGraph::shared_ptr iterate()
```

```cpp
const LevenbergMarquardtParams & params() const
```

```cpp
writeLogFile(double currentError)
```

```cpp
GaussianFactorGraph::shared_ptr linearize() const
```

```cpp
GaussianFactorGraph buildDampedSystem(const GaussianFactorGraph & linear, const VectorValues & sqrtHessianDiagonal) const
```

```cpp
bool tryLambda(const GaussianFactorGraph & linear, const VectorValues & sqrtHessianDiagonal)
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< LevenbergMarquardtOptimizer >
```

## 详细说明

This class performs Levenberg-Marquardt nonlinear optimization

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`LevenbergMarquardtOptimizer` 用于 GTSAM factor graph 优化流程中。

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
