---
tags: [LiDAR, 视觉, 紧耦合, RGB着色, 3D重建]
sources:
  - raw/docs-deep-dive/r3live_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/r3live_analysis.md
---

# R3LIVE 深度源码分析

> HKU Mars Lab：双 ESIKF 架构 LiDAR-视觉-惯性紧耦合，全球首个 RGB 着色 LiDAR SLAM

## 摘要
R3LIVE 使用 LIO ESIKF + VIO ESIKF 两个独立估计器并行运行，通过共享状态传递先验。核心创新是全球多视图 RGB 点云着色和实时 3D 网格重建（MVS）。

## 关键概念
- **[[算法-R3LIVE]]**：双 ESIKF 紧耦合，RGB 着色 + 离线 MVS 网格重建
- **[[架构-双ESIKF架构]]**：LIO 和 VIO 各自独立 ESIKF，并行线程，LIO 位姿作为 VIO 先验
- **[[方法-RGB着色点云]]**：LiDAR 点附着多视图 RGB 观测，维护 RGB 协方差，实时发布
- **[[架构-LIO-as-Prior设计]]**：VIO 从 LIO 出发 IMU 预积分对齐，视觉失败时优雅降级回纯 LIO

## 相关页面
- [[算法-FAST-LIVO2]]
- [[算法-LVI-SAM]]
- [[架构-多传感器融合架构]]