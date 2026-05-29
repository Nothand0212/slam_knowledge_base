---
type: entity
tags: [gtsam_points, C++ API, Nearest Neighbor, IncrementalCovarianceVoxelMap]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IncrementalCovarianceVoxelMap

> **结构体** | 头文件: `incremental_covariance_voxelmap.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Incremental voxelmap with online covariance and normal estimation.

## 继承关系

- 继承自 `gtsam_points::IncrementalVoxelMap< IncrementalCovarianceContainer >`

## 构造函数

```cpp
IncrementalCovarianceVoxelMap(double voxel_resolution)
```
Constructor.

## 公开方法

### 方法

```cpp
set_num_neighbors(int num_neighbors)
```
Set the number of neighbors for covariance estimation.

```cpp
set_min_num_neighbors(int min_num_neighbors)
```
Set the minimum number of neighbors for covariance estimation.

```cpp
set_warmup_cycles(int warmup_cycles)
```
Set the number of warmup cycles. Covariances of new points in this period are not re-evaluated every frame.

```cpp
set_lowrate_cycles(int lowrate_cycles)
```
Set the number of lowrate update cycles. Covariances of invalid points are re-evaluated every this period.

```cpp
set_remove_invalid_age_thresh(int remove_invalid_age_thresh)
```
Set the age threshold for removing invalid points. Invalid points older than this are removed.

```cpp
set_eig_stddev_thresh_scale(double eig_stddev_thresh_scale)
```
Set the threshold scale for normal validation.

```cpp
set_num_threads(int num_threads)
```
Set the number of threads for normal estimation.

```cpp
clear()
```
Clear the voxelmap.

```cpp
insert(const PointCloud & points)
```
Insert point into the voxelmap.

```cpp
size_t knn_search(const double * pt, size_t k, size_t * k_indices, double * k_sq_dists, double max_sq_dist = std::numeric_limits< double >::max()) const
```
Find k-nearest neighbors. This only finds neighbors with valid covariances.

```cpp
size_t knn_search_force(const double * pt, size_t k, size_t * k_indices, double * k_sq_dists, double max_sq_dist = std::numeric_limits< double >::max()) const
```
Find k-nearest neighbors. This finds neighbors regardless of the validity of covariances.

```cpp
std::vector< size_t > valid_indices(int num_threads = -1) const
```
Get valid point indices. If num_threads is -1, the member variable num_threads is used.

```cpp
std::vector< Eigen::Vector4d > voxel_points(const std::vector< size_t > & indices) const
```
Get points from indices.

```cpp
std::vector< Eigen::Vector4d > voxel_normals(const std::vector< size_t > & indices) const
```
Get normals from indices.

```cpp
std::vector< Eigen::Matrix4d > voxel_covs(const std::vector< size_t > & indices) const
```
Get covariances from indices.

```cpp
std::vector< Eigen::Vector4d > voxel_points() const
```
Get voxel points.

```cpp
std::vector< Eigen::Vector4d > voxel_normals() const
```
Get voxel normals.

```cpp
std::vector< Eigen::Matrix4d > voxel_covs() const
```
Get voxel covariances.

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IncrementalCovarianceVoxelMap` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
