---
type: entity
tags: [GICP, SO(3), 旋转优化, 流形, 体素化]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
sources:
  - wiki/sources/2026-04-28-rolo-analysis.md
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# RotVGICP

> ROLO-SLAM 的旋转优先 VGICP 变体，把体素化 GICP 和 SO(3) 流形旋转优化结合，用于越野地形中的稳健配准。

## 定义

ROLO-SLAM 自研的体素化 GICP 变体，在 SO(3) 流形上直接做高斯-牛顿旋转优化，避免四元数/Euler 角的参数化奇异性。

## 核心特征

- 基于 `LsqRegistration` → 最小二乘配准框架
- `so3_linearize()` 在 SO(3) 流形上直接线性化旋转
- 体素化：使用 VmfVoxelMap（Voxelized Mean-Free Gaussian Voxel Map）预计算目标分布
- 协方差预计算：`calculate_covariances()` 为源和目标点云计算邻域协方差矩阵
- 支持 OpenMP 多线程
- 与 Fast-GICP 的关系：RotVGICP 侧重于旋转分离优化而非全 6-DoF 同时优化

## 相关页面

- 实现于：[[算法-ROLO-SLAM]] `rot_vgicp.hpp:24-130`
- [[方法-GICP配准方法]]
- [[方法-APDGICP 自适应概率分布 GICP]]
- [[数学-流形优化]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[算法-ROLO-SLAM]]
