---
type: entity
tags: [gtsam_points, C++ API, Nearest Neighbor, GaussianVoxelMapCPU]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::GaussianVoxelMapCPU

> **类** | 头文件: `gtsam/nearest neighbor/GaussianVoxelMapCPU.h` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

## 继承关系

- 继承自 `gtsam_points::GaussianVoxelMap`
- 继承自 `gtsam_points::IncrementalVoxelMap< GaussianVoxel >`

## 构造函数

```cpp
GaussianVoxelMapCPU(double resolution)
```
Constructor.

## 公开方法

### 方法

```cpp
double voxel_resolution() const
```
Voxel resolution.

```cpp
Eigen::Vector3i voxel_coord(const Eigen::Vector4d & x) const
```
Compute the voxel index corresponding to a point.

```cpp
int lookup_voxel_index(const Eigen::Vector3i & coord) const
```
Look up a voxel index. If the voxel does not exist, return -1.

```cpp
const GaussianVoxel & lookup_voxel(int voxel_id) const
```
Look up a voxel.

```cpp
insert(const PointCloud & frame)
```
Insert a point cloud frame into the voxelmap.

```cpp
save_compact(const std::string & path) const
```
Save the voxelmap.

### 静态方法

```cpp
static GaussianVoxelMapCPU::Ptr load(const std::string & path)
```
Load a voxelmap from a file.

## 类型别名

```cpp
using Ptr = std::shared_ptr< GaussianVoxelMapCPU >
```
```cpp
using ConstPtr = std::shared_ptr< const GaussianVoxelMapCPU >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`GaussianVoxelMapCPU` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
