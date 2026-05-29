---
type: entity
tags: [gtsam_points, C++ API, CUDA, NonlinearFactorSetGPU]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::NonlinearFactorSetGPU

> **类** | 头文件: `nonlinear_factor_set_gpu.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

This class holds a set of GPU-based NonlinearFactors and manages their linearization and cost evaluation tasks.

## 继承关系

- 继承自 `NonlinearFactorSet`

## 构造函数

```cpp
NonlinearFactorSetGPU()
```

## 公开方法

### 方法

```cpp
int size() const
```
Number of GPU factors in this set.

```cpp
clear()
```
Remove all factors.

```cpp
clear_counts()
```
Reset linearization and cost evaluation counts.

```cpp
int linearization_count() const
```
Number of issued linearization tasks.

```cpp
int evaluation_count() const
```
Number of issued cost evaluation tasks.

```cpp
bool add(NonlinearFactor::shared_ptr factor)
```
Add a factor to the GPU factor set if it is a GPU-based one.

```cpp
add(const NonlinearFactorGraph & factors)
```
Add all GPU-based factors in a factor graph to the GPU factor set.

```cpp
linearize(const Values & linearization_point)
```
Compute all GPU-based linearization tasks.

```cpp
error(const Values & values)
```
Compute all GPU-based cost evaluation tasks.

```cpp
std::vector< GaussianFactor::shared_ptr > calc_linear_factors(const Values & linearization_point)
```
Calculate linearized factors.

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`NonlinearFactorSetGPU` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
