---
type: entity
tags: [gtsam_points, C++ API, Nearest Neighbor, FastOccupancyGrid]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::FastOccupancyGrid

> **类** | 头文件: `fast_occupancy_grid.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Fast occupancy grid with occupancy blocks and flat hashing for efficient point cloud overlap evaluation.

## 构造函数

```cpp
FastOccupancyGrid(double resolution)
```
Constructor.

## 公开方法

### 方法

```cpp
insert(const PointCloud & points, const Eigen::Isometry3d & pose = Eigen::Isometry3d::Identity())
```
Insert points into the grid.

```cpp
int calc_overlap(const PointCloud & points, const Eigen::Isometry3d & pose = Eigen::Isometry3d::Identity()) const
```
Calculate the number of points overlapping with the grid.

```cpp
double calc_overlap_rate(const PointCloud & points, const Eigen::Isometry3d & pose = Eigen::Isometry3d::Identity()) const
```
Calculate the overlap ratio of the points with the grid.

```cpp
std::vector< unsigned char > get_overlaps(const PointCloud & points, const Eigen::Isometry3d & pose = Eigen::Isometry3d::Identity()) const
```
Get the overlap status of each point in the point cloud.

```cpp
int num_occupied_cells() const
```
Get the number of occupied cells in the grid.

## 类型别名

```cpp
using Ptr = std::shared_ptr< FastOccupancyGrid >
```
```cpp
using ConstPtr = std::shared_ptr< const FastOccupancyGrid >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`FastOccupancyGrid` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
