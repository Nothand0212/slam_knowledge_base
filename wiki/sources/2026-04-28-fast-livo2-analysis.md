---
tags: [SLAM, LiDAR-视觉-惯性, IESKF, 融合]
sources:
  - raw/docs-deep-dive/fast_livo2_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/fast_livo2_analysis.md
---

# FAST-LIVO2 源码级分析摘要

> 统一IESKF实现LiDAR-Visual-Inertial的真正紧耦合里程计，直接法视觉+曝光在线估计，单协方差矩阵统一管理多模态不确定性

## 核心发现
- 单一IESKF（19维状态）串行执行LIO→VIO更新，VIO以LIO估计为起点、IMU传播为先验，协方差全程连贯传递
- 视觉采用直接法光度误差（非特征点），`inv_expo_time`作为状态在线估计，适应自动曝光变化
- VoxelMap（哈希+八叉树+平面协方差）替代iKd-Tree，支持7种LiDAR即插即用
- LiDAR-视觉三通道互馈：深度图辅助视觉验证、LiDAR平面更新法向量、Shi-Tomasi评分创建视觉地图点

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | LiDAR点到VoxelMap平面残差 + 直接法稀疏光度对齐，OpenMP并行 |
| 后端 | 统一IESKF（19维），IMU前向传播→LIO更新→VIO更新，粗到精金字塔 |
| 独特创新 | 单KF紧耦合+在线曝光估计+VoxelMap八叉树+直接法免特征提取 |

## 关键引用
- 状态向量定义: `common_lib.h:31, 126-223`
- LIO IESKF更新: `voxel_map.cpp:461-498`
- VIO光度残差链式求导: `vio.cpp:1520-1687`
- LIO-VIO交替融合: `LIVMapper.cpp:884-1119`
- VoxelMap平面拟合: `voxel_map.cpp:532-591`
- 视觉地图点生命周期: `visual_point.h:23-47`
- 后向传播去畸变: `IMU_Processing.cpp:494-539`

## 相关页面
- [[2026-04-28-r3live-analysis]]
- [[2026-04-28-lvi-sam-analysis]]
- [[2026-04-28-fast-lio-sam-analysis]]