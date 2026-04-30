---
tags: [LiDAR, 视觉, 紧耦合, 直接法, IESKF]
sources:
  - raw/docs-deep-dive/fast_livo2_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/fast_livo2_analysis.md
---

# FAST-LIVO2 深度源码分析

> HKU Mars Lab：统一 IESKF 紧耦合，直接法光度误差 + VoxelMap 八叉树，19 维状态多传感器融合

## 摘要
FAST-LIVO2 是所有 LIV 系统中融合最彻底的方案：单一 19 维 IESKF 状态，协方差矩阵自动关联 LiDAR 和视觉不确定性。VIO 用直接法光度误差代替特征点，曝光在线估计。

## 关键概念
- **[[算法-FAST-LIVO2]]**：统一 IESKF 串行 LIO→VIO，7 种 LiDAR 适配，三种运行模式 SLAM_MODE
- **[[方法-VoxelMap八叉树]]**：哈希八叉树递归切割，叶节点 VoxelPlane 含 6×6 协方差和半径
- **[[概念-直接法光度误差]]**：`r=τ_cur·I_cur−τ_ref·I_warped`，粗到精金字塔优化
- **[[方法-曝光在线估计]]**：inv_expo_time 作为 IESKF 状态在线估计，适应相机自动曝光
- **[[方法-统一IESKF融合]]**：LIO 更新后 VIO 继承协方差继续收缩，不分别优化取平均

## 相关页面
- [[算法-R3LIVE]]
- [[算法-LVI-SAM]]
- [[概念-直接法视觉里程计]]
- [[架构-多传感器融合架构]]