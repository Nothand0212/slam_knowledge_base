---
type: entity
tags: [gtsam_points, C++ API, Colored & Continuous Factors, IntegratedColorConsistencyFactor_]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedColorConsistencyFactor_

> **类** | 头文件: `integrated_color_consistency_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Photometric consistency factor between point clouds.

## 继承关系

- 继承自 `gtsam_points::IntegratedMatchingCostFactor`

## 构造函数

```cpp
IntegratedColorConsistencyFactor_(Key target_key, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source, const std::shared_ptr< const NearestNeighborSearch > & target_tree, const std::shared_ptr< const IntensityGradients > & target_gradients)
```
Create a binary color consistency factor between target and source poses.

```cpp
IntegratedColorConsistencyFactor_(const Pose3 & fixed_target_pose, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source, const std::shared_ptr< const NearestNeighborSearch > & target_tree, const std::shared_ptr< const IntensityGradients > & target_gradients)
```
Create a unary color consistency factor between a fixed target pose and an active source pose.

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter = gtsam::DefaultKeyFormatter) const
```
Print the factor information.

```cpp
size_t memory_usage() const
```
Calculate the memory usage of this factor.

```cpp
set_num_threads(int n)
```
Set the number of thread used for linearization of this factor.

```cpp
set_max_correspondence_distance(double d)
```
Set the maximum distance between corresponding points. Correspondences with distances larger than this will be rejected (i.e., correspondence trimming).

```cpp
set_photometric_term_weight(double w)
```
Set the weight for the photometric error term.

```cpp
set_correspondence_update_tolerance(double angle, double trans)
```
Correspondences are updated only when the displacement from the last update point is larger than these threshold values.

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedColorConsistencyFactor_< TargetFrame, SourceFrame > >
```

## 详细说明

This factor uses (x, y, z, intensity) to query nearest neighbor search The 4th element (intensity) will be simply ignored if a standard gtsam_points::KdTree is given while it can provide additional distance information between points if gtsam_points::IntensityKdTree is used

While the use of IntensityKdTree significantly improves the convergence speed, it can affect optimization stability in some cases

Park et al., "Colored Point Cloud Registration Revisited", ICCV2017 This factor uses (x, y, z, intensity) to query nearest neighbor search The 4th element (intensity) will be simply ignored if a standard gtsam_points::KdTree is given while it can provide additional distance information between points if gtsam_points::IntensityKdTree is used While the use of IntensityKdTree significantly improves the convergence speed, it can affect optimization stability in some cases

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedColorConsistencyFactor_` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
