---
type: entity
tags: [gtsam_points, C++ API, Point Cloud & Trajectory, PointCloud]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::PointCloud

> **结构体** | 头文件: `point_cloud.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Standard point cloud class that holds only pointers to point attributes.

## 构造函数

```cpp
PointCloud()
```

```cpp
PointCloud(const PointCloud & )
```

## 公开方法

### 方法

```cpp
PointCloud & operator=(PointCloud const & )
```

```cpp
size_t size() const
```
Number of points.

```cpp
bool has_times() const
```
Check if the point cloud has per-point timestamps.

```cpp
bool has_points() const
```
Check if the point cloud has points.

```cpp
bool has_normals() const
```
Check if the point cloud has point normals.

```cpp
bool has_covs() const
```
Check if the point cloud has point covariances.

```cpp
bool has_intensities() const
```
Check if the point cloud has point intensities.

```cpp
bool check_times() const
```
Warn if the point cloud doesn't have times.

```cpp
bool check_points() const
```
Warn if the point cloud doesn't have points.

```cpp
bool check_normals() const
```
Warn if the point cloud doesn't have normals.

```cpp
bool check_covs() const
```
Warn if the point cloud doesn't have covs.

```cpp
bool check_intensities() const
```
Warn if the point cloud doesn't have intensities.

```cpp
bool has_times_gpu() const
```
Check if the point cloud has per-point timestamps on GPU.

```cpp
bool has_points_gpu() const
```
Check if the point cloud has points on GPU.

```cpp
bool has_normals_gpu() const
```
Check if the point cloud has point normals on GPU.

```cpp
bool has_covs_gpu() const
```
Check if the point cloud has point covariances on GPU.

```cpp
bool has_intensities_gpu() const
```
Check if the point cloud has point intensities on GPU.

```cpp
bool check_times_gpu() const
```
Warn if the point cloud doesn't have times on GPU.

```cpp
bool check_points_gpu() const
```
Warn if the point cloud doesn't have points on GPU.

```cpp
bool check_normals_gpu() const
```
Warn if the point cloud doesn't have normals on GPU.

```cpp
bool check_covs_gpu() const
```
Warn if the point cloud doesn't have covs on GPU.

```cpp
bool check_intensities_gpu() const
```
Warn if the point cloud doesn't have intensities on GPU.

```cpp
const T * aux_attribute(const std::string & attrib) const
```
Get the pointer to an aux attribute.

```cpp
save(const std::string & path) const
```
Save the point cloud data.

```cpp
save_compact(const std::string & path) const
```
Save the point cloud data with a compact representation without unnecessary fields (e.g., the last element of homogeneous coordinates).

## 类型别名

```cpp
using Ptr = std::shared_ptr< PointCloud >
```
```cpp
using ConstPtr = std::shared_ptr< const PointCloud >
```

## 公开成员变量

```cpp
size_t num_points
```
```cpp
double * times
```
```cpp
Eigen::Vector4d * points
```
```cpp
Eigen::Vector4d * normals
```
```cpp
Eigen::Matrix4d * covs
```
```cpp
double * intensities
```
```cpp
std::unordered_map< std::string, std::pair< size_t, void * > > aux_attributes
```
```cpp
float * times_gpu
```
```cpp
Eigen::Vector3f * points_gpu
```
```cpp
Eigen::Vector3f * normals_gpu
```
```cpp
Eigen::Matrix3f * covs_gpu
```
```cpp
float * intensities_gpu
```

## 详细说明

If you don't want to manage the lifetime of point data by yourself, use gtsam_points::PointCloudCPU. If you don't want to manage the lifetime of point data by yourself, use gtsam_points::PointCloudCPU.

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`PointCloud` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
