---
type: entity
tags: [gtsam_points, C++ API, Point Cloud & Trajectory, PointCloudCPU]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::PointCloudCPU

> **结构体** | 头文件: `point_cloud_cpu.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Point cloud frame on CPU memory.

## 继承关系

- 继承自 `gtsam_points::PointCloud`

## 构造函数

```cpp
PointCloudCPU()
```

```cpp
PointCloudCPU(const Eigen::Matrix< T, D, 1 > * points, int num_points)
```
Constructor.

```cpp
PointCloudCPU(const std::vector< Eigen::Matrix< T, D, 1 >, Alloc > & points)
```
Constructor.

```cpp
PointCloudCPU(const PointCloudCPU & points)
```

## 公开方法

### 方法

```cpp
PointCloudCPU & operator=(PointCloudCPU const & )
```

```cpp
add_times(const T * times, int num_points)
```

```cpp
add_times(const std::vector< T > & times)
```

```cpp
add_points(const Eigen::Matrix< T, D, 1 > * points, int num_points)
```

```cpp
add_points(const std::vector< Eigen::Matrix< T, D, 1 >, Alloc > & points)
```

```cpp
add_normals(const Eigen::Matrix< T, D, 1 > * normals, int num_points)
```

```cpp
add_normals(const std::vector< Eigen::Matrix< T, D, 1 >, Alloc > & normals)
```

```cpp
add_covs(const Eigen::Matrix< T, D, D > * covs, int num_points)
```

```cpp
add_covs(const std::vector< Eigen::Matrix< T, D, D >, Alloc > & covs)
```

```cpp
add_intensities(const T * intensities, int num_points)
```

```cpp
add_intensities(const std::vector< T > & intensities)
```

```cpp
add_aux_attribute(const std::string & attrib_name, const T * values, int num_points)
```

```cpp
add_aux_attribute(const std::string & attrib_name, const std::vector< T, Alloc > & values)
```

```cpp
size_t memory_usage() const
```
Memory usage in bytes.

### 静态方法

```cpp
static PointCloudCPU::Ptr clone(const PointCloud & points)
```
Deep copy.

```cpp
static PointCloudCPU::Ptr load(const std::string & path)
```

## 类型别名

```cpp
using Ptr = std::shared_ptr< PointCloudCPU >
```
```cpp
using ConstPtr = std::shared_ptr< const PointCloudCPU >
```

## 公开成员变量

```cpp
std::vector< double > times_storage
```
```cpp
std::vector< Eigen::Vector4d > points_storage
```
```cpp
std::vector< Eigen::Vector4d > normals_storage
```
```cpp
std::vector< Eigen::Matrix4d > covs_storage
```
```cpp
std::vector< double > intensities_storage
```
```cpp
std::unordered_map< std::string, std::shared_ptr< void > > aux_attributes_storage
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`PointCloudCPU` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
