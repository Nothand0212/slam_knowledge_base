---
type: entity
tags: [gtsam_points, C++ API, Scan Matching Factors, IntegratedVGICPFactor_]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedVGICPFactor_

> **类** | 头文件: `integrated_vgicp_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Voxelized GICP matching cost factor Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA2021 Koide et al., "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors", RA-L2021.

## 继承关系

- 继承自 `gtsam_points::IntegratedMatchingCostFactor`

## 构造函数

```cpp
IntegratedVGICPFactor_(Key target_key, Key source_key, const GaussianVoxelMap::ConstPtr & target_voxels, const std::shared_ptr< const SourceFrame > & source)
```
Create a binary VGICP factor between target and source poses.

```cpp
IntegratedVGICPFactor_(const Pose3 & fixed_target_pose, Key source_key, const GaussianVoxelMap::ConstPtr & target_voxels, const std::shared_ptr< const SourceFrame > & source)
```
Create a unary VGICP factor between a fixed target pose and an active source pose.

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
set_fused_cov_cache_mode(FusedCovCacheMode mode)
```
Set the cache mode for fused covariance matrices (i.e., mahalanobis).

```cpp
int num_inliers() const
```
Get the number of inlier points.

```cpp
double inlier_fraction() const
```
Compute the fraction of inlier points that have correspondences fell in a voxel.

```cpp
const std::shared_ptr< const GaussianVoxelMapCPU > & get_target() const
```
Get the target voxelmap.

```cpp
NonlinearFactor::shared_ptr clone() const
```

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedVGICPFactor_ >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedVGICPFactor_` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
