---
type: entity
tags: [gtsam_points, C++ API, Scan Matching Factors, IntegratedLOAMFactor_]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedLOAMFactor_

> **类** | 头文件: `integrated_loam_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Scan matching factor based on the combination of point-to-plane and point-to-edge distances.

## 继承关系

- 继承自 `gtsam_points::IntegratedMatchingCostFactor`

## 构造函数

```cpp
IntegratedLOAMFactor_(Key target_key, Key source_key, const std::shared_ptr< const TargetFrame > & target_edges, const std::shared_ptr< const TargetFrame > & target_planes, const std::shared_ptr< const SourceFrame > & source_edges, const std::shared_ptr< const SourceFrame > & source_planes, const std::shared_ptr< const NearestNeighborSearch > & target_edges_tree, const std::shared_ptr< const NearestNeighborSearch > & target_planes_tree)
```

```cpp
IntegratedLOAMFactor_(Key target_key, Key source_key, const std::shared_ptr< const TargetFrame > & target_edges, const std::shared_ptr< const TargetFrame > & target_planes, const std::shared_ptr< const SourceFrame > & source_edges, const std::shared_ptr< const SourceFrame > & source_planes)
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

```cpp
set_max_correspondence_distance(double dist_edge, double dist_plane)
```

```cpp
set_correspondence_update_tolerance(double angle, double trans)
```

```cpp
set_enable_correspondence_validation(bool enable)
```

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedLOAMFactor_< TargetFrame, SourceFrame > >
```

## 详细说明

Zhang and Singh, "Low-drift and real-time lidar odometry and mapping", Autonomous Robots, 2017 Zhang and Singh, "LOAM: LiDAR Odometry and Mapping in Real-time", RSS2014 Tixiao and Brendan, "LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain", IROS2018

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedLOAMFactor_` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
