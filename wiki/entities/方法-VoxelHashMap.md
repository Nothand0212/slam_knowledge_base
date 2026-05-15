---
tags: [LiDAR, ICP, 体素地图]
sources:
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-LiDAR地图表示]]
type: entity
---

> 本页内容已归并至 [[方法-LiDAR地图表示]]。

# VoxelHashMap

> 基于 `tsl::robin_map` 哈希表的 3D 体素局部地图，KISS-ICP 和 GenZ-ICP 的核心数据结构。

## 核心方法

每个体素内维护最多 `max_points_per_voxel` 个点（默认 20），通过 `map_resolution = sqrt(voxel²/max_points)` 做二级去重。支持 **27 邻域最近邻搜索**：在当前体素及其 26 个邻接体素（3×3×3 网格）中遍历查找最近点。通过 `RemovePointsFarFromLocation()` 维护半径 100m 的滑动窗口局部地图。

## 关键设计

- 哈希函数：`vec[0]*73856093 ^ vec[1]*19349663 ^ vec[2]*83492791`，与 iVox 一致
- 地图更新：`AddPoints()` 去重插入 → `RemovePointsFarFromLocation()` 裁剪远点
- 查询 O(K) 而非体素 O(1)，但适应稀疏点云分布

## 与其他地图结构对比

| 结构 | 查询方式 | 优点 | 代价 |
|------|----------|------|------|
| KD-tree | 最近邻树搜索 | 精确、通用 | 动态增删和重建成本高 |
| VoxelHashMap | 哈希体素 + 27 邻域遍历 | 简单、CPU 友好、适合局部滑窗 | 精度受体素大小影响 |
| iKD-Tree | 增量 KD-tree | 支持在线插入删除 | 实现复杂 |
| IVox / 体素地图 | 体素哈希或数组索引 | 适合 LIO 实时 scan-to-map | 参数依赖场景尺度 |

## 工程边界

- 体素大小同时影响地图密度、最近邻精度和运行时间。
- 27 邻域只适用于局部最近邻近似；如果体素太大或点云太稀疏，可能漏掉真实对应。
- 滑动窗口半径能限制内存，但会牺牲长期回访的一致性，因此 KISS-ICP 这类系统通常不做全局回环。

## Agent 实现提示

### 适用场景

用于实时 LiDAR 里程计中的局部地图：需要快速插入当前帧点、裁剪远处点，并为 ICP 提供近似最近邻。适合滑动窗口 scan-to-map，不适合作为长期全局地图或精确全库 KNN。

### 输入输出契约

- 输入：世界系点云、当前传感器或里程计原点、体素大小、最大保留半径、每体素最大点数。
- 输出：可查询的哈希体素地图、近似最近邻 `(point, distance)`、裁剪后的局部地图点云。
- 前置条件：插入点已经变换到地图坐标系；体素大小与 ICP 对应距离、下采样分辨率保持一致量级。

### 实现骨架（伪代码）

```text
function update(points_world, origin):
    for p in points_world:
        voxel <- floor(p / voxel_size)
        bucket <- map[voxel]
        if bucket.size >= max_points_per_voxel:
            continue
        if exists q in bucket where norm(q - p) < map_resolution:
            continue
        bucket.push(p)
    for voxel, bucket in map:
        if norm(bucket.front - origin)^2 > max_distance^2:
            erase voxel

function nearest(query):
    best <- none
    for shift in 27_neighbor_offsets:
        for p in map[floor(query / voxel_size) + shift]:
            update best by Euclidean distance
    return best
```

### 关键源码片段

`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L35-L62`

```cpp
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
}  // namespace

namespace kiss_icp {

std::tuple<Eigen::Vector3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    // Convert the point to voxel coordinates
    const auto &voxel = PointToVoxel(query, voxel_size_);
    // Find the nearest neighbor
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();
    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        const auto &query_voxel = voxel + voxel_shift;
        auto search = map_.find(query_voxel);
        if (search != map_.end()) {
            const auto &points = search.value();
            const Eigen::Vector3d &neighbor = *std::min_element(
                points.cbegin(), points.cend(), [&](const auto &lhs, const auto &rhs) {
                    return (lhs - query).norm() < (rhs - query).norm();
                });
            double distance = (neighbor - query).norm();
```

`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L83-L119`

```cpp
void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const auto voxel = PointToVoxel(point, voxel_size_);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_points = search.value();
            if (voxel_points.size() == max_points_per_voxel_ ||
                std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                            [&](const auto &voxel_point) {
                                return (voxel_point - point).norm() < map_resolution;
                            })) {
                return;
            }
            voxel_points.emplace_back(point);
        } else {
            std::vector<Eigen::Vector3d> voxel_points;
            voxel_points.reserve(max_points_per_voxel_);
            voxel_points.emplace_back(point);
            map_.insert({voxel, std::move(voxel_points)});
        }
    });
}
```

### 实现注意事项

- 体素哈希的查询复杂度取决于每个桶内点数，必须限制 `max_points_per_voxel`。
- `map_resolution` 是二级去重尺度，不能与 `voxel_size` 混为同一个参数。
- 删除远点时用体素内第一个点近似代表该体素，速度快但会有边界误差。
- 多线程 ICP 查询时地图应只读；插入和裁剪最好在配准完成后单线程或加锁执行。

### 源码检索锚点

- `VoxelHashMap::Update`
- `VoxelHashMap::AddPoints`
- `VoxelHashMap::RemovePointsFarFromLocation`
- `voxel_shifts`
- `PointToVoxel`

## 相关页面

- [[方法-体素地图]]
- [[算法-KISS-ICP]]
- [[方法-genz-icp]]
- [[方法-ICP配准方法]]
- [[LiDAR数据管线]]
