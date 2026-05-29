---
type: entity
tags: [gtsam_points, C++ API, Scan Matching Factors, IntegratedMatchingCostFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedMatchingCostFactor

> **类** | 头文件: `integrated_matching_cost_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Abstraction of LSQ-based scan matching constraints between point clouds.

## 继承关系

- 继承自 `gtsam::NonlinearFactor`

## 构造函数

```cpp
IntegratedMatchingCostFactor(Key target_key, Key source_key)
```
Create a binary matching cost factor between target and source poses.

```cpp
IntegratedMatchingCostFactor(const Pose3 & fixed_target_pose, Key source_key)
```
Create a unary matching cost factor between a fixed target pose and an active source pose.

## 公开方法

### 方法

```cpp
size_t dim() const
```

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter = gtsam::DefaultKeyFormatter) const
```
Print the factor information.

```cpp
double error(const Values & values) const
```

```cpp
GaussianFactor::shared_ptr linearize(const Values & values) const
```

```cpp
const Eigen::Isometry3d & get_fixed_target_pose() const
```

```cpp
Eigen::Isometry3d calc_delta(const Values & values) const
```

```cpp
size_t memory_usage() const
```
Calculate the memory usage of this factor.

```cpp
update_correspondences(const Eigen::Isometry3d & delta) const
```
Update point correspondences.

```cpp
double evaluate(const Eigen::Isometry3d & delta, Eigen::Matrix< double, 6, 6 > * H_target = nullptr, Eigen::Matrix< double, 6, 6 > * H_source = nullptr, Eigen::Matrix< double, 6, 6 > * H_target_source = nullptr, Eigen::Matrix< double, 6, 1 > * b_target = nullptr, Eigen::Matrix< double, 6, 1 > * b_source = nullptr) const
```
Evaluate the matching cost.

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedMatchingCostFactor >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedMatchingCostFactor` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
