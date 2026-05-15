---
type: entity
tags: [ICP, 连续时间, 运动畸变, 因子图]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# 连续时间 ICP 因子

> gtsam_points 中把扫描起止两个位姿同时作为变量的 ICP 因子，使点云配准在因子图内直接处理帧内运动畸变。

## 定义

将激光扫描帧建模为扫描开始位姿到结束位姿之间的连续运动轨迹，通过 slerp 插值获得每点的实时位姿，从而在因子图优化中内建消除运动畸变。

## 核心特征

- 不继承 IntegratedMatchingCostFactor（需要 t0、t1 两个 key）
- 维护 `time_table`（每点归一化时间戳 [0,1]）和 `time_indices`（时间索引）
- linearize() 流程：update_poses() → slerp 插值 → update_correspondences() → 通过 pose_derivatives_t0/t1 传播雅可比到两个 key
- CT-GICP 变体额外使用 D2D 匹配代价，需要目标点协方差和马氏距离缓存
- 相比匀速 deskew 更精确，但计算量更大

## 相关页面

- 实现：gtsam_points `integrated_ct_icp_factor.hpp:21`
- [[概念-连续时间轨迹]]
- [[组件-gtsam_points]]
- [[算法-CT-ICP]]
- [[方法-IntegratedMatchingCostFactor]]
- 参考：CT-ICP 论文 [Bellenbach, 2021]
