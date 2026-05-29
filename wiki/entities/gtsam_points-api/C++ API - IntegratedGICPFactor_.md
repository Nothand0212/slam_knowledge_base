---
type: entity
tags: [gtsam_points, C++ API, Scan Matching Factors, IntegratedGICPFactor_]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedGICPFactor_

> **类** | 头文件: `integrated_gicp_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Generalized ICP matching cost factor Segal et al., "Generalized-ICP", RSS2005.

## 继承关系

- 继承自 `gtsam_points::IntegratedMatchingCostFactor`

## 构造函数

```cpp
IntegratedGICPFactor_(Key target_key, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source, const std::shared_ptr< const NearestNeighborSearch > & target_tree)
```
Create a binary ICP factor between target and source poses.

```cpp
IntegratedGICPFactor_(Key target_key, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source)
```

```cpp
IntegratedGICPFactor_(const Pose3 & fixed_target_pose, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source, const std::shared_ptr< const NearestNeighborSearch > & target_tree)
```
Create a unary GICP factor between a fixed target pose and an active source pose.

```cpp
IntegratedGICPFactor_(const Pose3 & fixed_target_pose, Key source_key, const std::shared_ptr< const TargetFrame > & target, const std::shared_ptr< const SourceFrame > & source)
```

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
set_max_correspondence_distance(double dist)
```
Set the maximum distance between corresponding points. Correspondences with distances larger than this will be rejected (i.e., correspondence trimming).

```cpp
set_correspondence_update_tolerance(double angle, double trans)
```
Correspondences are updated only when the displacement from the last update point is larger than these threshold values.

```cpp
set_fused_cov_cache_mode(FusedCovCacheMode mode)
```
Set the cache mode for fused covariance matrices (i.e., mahalanobis).

```cpp
double inlier_fraction() const
```
Compute the fraction of inlier points that have correspondences with a distance smaller than the trimming threshold.

```cpp
NonlinearFactor::shared_ptr clone() const
```

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedGICPFactor_ >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedGICPFactor_` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
