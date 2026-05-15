---
tags: [LiDAR, 地图表示, 八叉树, VoxelMap, FAST-LIVO2]
sources:
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-LiDAR地图表示]]
type: entity
---

> 本页内容已归并至 [[方法-LiDAR地图表示]]。

# VoxelMap八叉树

> FAST-LIVO2 的 LiDAR 地图关联结构：哈希表顶层索引 + 八叉树递归切割，叶节点存储 VoxelPlane（中心、法向量、6×6 协方差、半径）。

## 数据结构

- 顶层：`VOXEL_LOCATION → VoxelOctoTree` 哈希映射
- 叶节点：`VoxelPlane` 含点中心、法向量、平面协方差 `plane_var_`、半径 `radius_`
- 平面协方差由点的不确定性传播：`plane_var_ = Σ J(6×3)·point_var(3×3)·J^T`

## vs iKd-Tree

- VoxelMap：固定网格 + 自适应八叉树，O(logN)，内存效率更高
- iKd-Tree（R3LIVE）：增量 kd 树，支持动态删除插入，适合大规模动态场景

## 在 FAST-LIVO2 中的作用

VoxelMap 八叉树为 LiDAR 点面残差提供局部平面关联。顶层哈希让系统快速定位空间区域，八叉树递归切割让平面单元自适应局部几何复杂度。叶节点保存 `VoxelPlane`，因此前端不只是找最近点，而是找可用于 IESKF 更新的局部平面。

## 工程注意

- 八叉树深度和体素尺寸共同决定地图精度与内存。
- 平面协方差传播依赖点测量噪声模型，不能只看几何拟合残差。
- 对动态场景需要配合滑动窗口、点云剔除或时间衰减，否则旧平面会污染当前匹配。

## 相关页面

- [[算法-FAST-LIVO2]]
- [[方法-体素地图]]
- [[方法-在线平面拟合]]
- [[方法-IESKF滤波器]]
