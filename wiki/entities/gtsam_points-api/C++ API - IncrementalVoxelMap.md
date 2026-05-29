---
type: entity
tags: [gtsam_points, C++ API, Nearest Neighbor, IncrementalVoxelMap]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IncrementalVoxelMap

> **结构体** | 头文件: `incremental_voxelmap.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Incremental voxelmap. This class supports incremental point cloud insertion and LRU-based voxel deletion.

## 继承关系

- 继承自 `gtsam_points::NearestNeighborSearch`

## 构造函数

```cpp
IncrementalVoxelMap(double leaf_size)
```
Constructor.

## 公开方法

### 方法

```cpp
set_voxel_resolution(const double leaf_size)
```
Voxel resolution.

```cpp
set_lru_clear_cycle(const int lru_clear_cycle)
```
LRU cache clearing cycle.

```cpp
set_lru_horizon(const int lru_horizon)
```
LRU cache horizon.

```cpp
set_neighbor_voxel_mode(const int mode)
```
Neighboring voxel search mode (1, 7, 19, or 27).

```cpp
VoxelContents::Setting & voxel_insertion_setting()
```
Voxel setting.

```cpp
double leaf_size() const
```
Voxel size.

```cpp
size_t num_voxels() const
```
Number of voxels in the voxelmap.

```cpp
clear()
```
Clear the voxelmap.

```cpp
insert(const PointCloud & points)
```
Insert points to the voxelmap.

```cpp
size_t knn_search(const double * pt, size_t k, size_t * k_indices, double * k_sq_dists, double max_sq_dist = std::numeric_limits< double >::max()) const
```
Find k nearest neighbors.

```cpp
size_t calc_index(const size_t voxel_id, const size_t point_id) const
```
Calculate the global point index from the voxel index and the point index.

```cpp
size_t voxel_id(const size_t i) const
```
Extract the point ID from a global index.

```cpp
size_t point_id(const size_t i) const
```
Extract the voxel ID from a global index.

```cpp
bool has_points() const
```

```cpp
bool has_normals() const
```

```cpp
bool has_covs() const
```

```cpp
bool has_intensities() const
```

```cpp
decltype(auto) point(const size_t i) const
```

```cpp
decltype(auto) normal(const size_t i) const
```

```cpp
decltype(auto) cov(const size_t i) const
```

```cpp
decltype(auto) intensity(const size_t i) const
```

```cpp
std::vector< Eigen::Vector4d > voxel_points() const
```

```cpp
std::vector< Eigen::Vector4d > voxel_normals() const
```

```cpp
std::vector< Eigen::Matrix4d > voxel_covs() const
```

```cpp
std::vector< double > voxel_intensities() const
```

```cpp
PointCloudCPU::Ptr voxel_data() const
```

## 类型别名

```cpp
using Ptr = std::shared_ptr< IncrementalVoxelMap >
```
```cpp
using ConstPtr = std::shared_ptr< const IncrementalVoxelMap >
```

## 详细说明

This class can be used as a point cloud as well as a neighbor search structure. 

For the compatibility with other nearest neighbor search methods, this implementation returns indices that encode the voxel and point IDs. The first `point_id_bits` (e.g., 32) bits of a point index represent the point ID, and the rest `voxel_id_bits` (e.g., 32) bits represent the voxel ID that contains the point. The specified point can be looked up by `voxelmap.point(index)`; This class can be used as a point cloud as well as a neighbor search structure. For the compatibility with other nearest neighbor search methods, this implementation returns indices that encode the voxel and point IDs. The first `point_id_bits` (e.g., 32) bits of a point index represent the point ID, and the rest `voxel_id_bits` (e.g., 32) bits represent the voxel ID that contains the point. The specified point can be looked up by `voxelmap.point(index)`;

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IncrementalVoxelMap` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
