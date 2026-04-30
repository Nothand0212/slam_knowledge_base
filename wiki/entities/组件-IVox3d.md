---
tags: [地图表示, 体素, 哈希地图, LIO]
sources:
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# IVox3d

> Lightning-LM 的增量哈希 voxel 地图：`unordered_map<KeyType, grid>`，容量 100 万，LRU 淘汰，支持 6/18/26 邻域搜索。

## 数据结构
- `IVox<3, IVoxNodeType::DEFAULT, PointType>` 模板类
- 每个 IVoxNode 最多存储 10 个点（LRU 插入策略）
- NCLT 默认配置：ivox_grid_resolution=0.5m, nearby_type=18

## 地图更新
`MapIncremental()` 使用并行 `for_each` 更新，仅当新点距离 voxel 中心比现有点更远且未达容量时才添加。相比 iKd-Tree，哈希方案在大规模场景内存效率更高。

## 与 VoxelHashMap 的关系

IVox3d 和 VoxelHashMap 都是哈希体素地图，但侧重点不同：

| 结构 | 代表系统 | 侧重点 |
|------|----------|--------|
| VoxelHashMap | KISS-ICP / GenZ-ICP | 简洁局部地图和 27 邻域最近邻 |
| IVox3d | Lightning-LM / FAST-LIO2 系 | 面向 LIO 的增量点云地图和并行更新 |

IVox3d 更接近 LIO 前端的在线地图组件，通常与点到面残差、在线平面拟合和 IESKF 更新一起使用。

## 工程注意

- `nearby_type` 决定邻域搜索范围，影响速度和配准稳定性。
- 每个 voxel 的容量限制会压缩地图，需避免过度稀疏导致平面拟合失败。
- LRU 淘汰适合局部里程计，不等价于全局一致地图管理。

## 相关页面

- [[组件-lightning-lm]]
- [[方法-在线平面拟合]]
- [[方法-IESKF滤波器]]
- [[方法-VoxelHashMap]]
- [[方法-体素地图]]
