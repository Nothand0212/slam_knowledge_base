---
type: entity
tags: [gtsam_points, C++ API, Nearest Neighbor, KdTreeX]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::KdTreeX

> **结构体** | 头文件: `kdtreex.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

KdTree with arbitrary dimension.

## 继承关系

- 继承自 `gtsam_points::NearestNeighborSearch`

## 构造函数

```cpp
KdTreeX(const Eigen::Matrix< double, D, 1 > * points, int num_points)
```
Constructor.

## 公开方法

### 方法

```cpp
int dim() const
```

```cpp
size_t kdtree_get_point_count() const
```

```cpp
double kdtree_get_pt(const size_t idx, const size_t dim) const
```

```cpp
bool kdtree_get_bbox(BBox & ) const
```

```cpp
size_t knn_search(const double * pt, size_t k, size_t * k_indices, double * k_sq_dists, double max_sq_dist = std::numeric_limits< double >::max()) const
```
k-nearest neighbor search

```cpp
size_t radius_search(const double * pt, double radius, std::vector< size_t > & indices, std::vector< double > & sq_dists, int max_num_neighbors = std::numeric_limits< int >::max()) const
```
Radius search.

## 类型别名

```cpp
using Index = nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor< double, KdTreeX< D >, double >, KdTreeX< D >, D, size_t >
```

## 公开成员变量

```cpp
const int num_points
```
```cpp
const Eigen::Matrix< double, D, 1 > * points
```
```cpp
double search_eps
```
```cpp
std::unique_ptr< Index > index
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`KdTreeX` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
