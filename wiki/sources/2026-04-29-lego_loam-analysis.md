---
tags: [source, 源码分析, SLAM, lego-loam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/lego_loam_analysis.md
sources:
  - raw/docs-deep-dive/lego_loam_analysis.md
---
# LeGO-LOAM 深度分析

**日期**: 2026-04-29 | **语言**: C++/ROS1 | **依赖**: GTSAM, PCL, OpenCV

## 概要
轻量级地面优化 LOAM 改进版（IROS 2018, 2805 stars）。核心创新: [[LiDAR数据管线|地面分割]]引导特征分类（非地面=edge, 地面=planar）+ [[方法-两步解耦优化]]（地面→roll/pitch/ty, 边缘→yaw/tx/tz, 3×3 QR）。4 节点: ImageProjection → FeatureAssociation → mapOptimization（scan-to-map + iSAM2）→ TransformFusion。

## 核心概念
- [[算法-LeGO-LOAM]] — 工程优化 vs 算法完整性经典
- [[LiDAR数据管线|地面分割]] — 相邻线束垂直角度 → 地面点/非地面点
- [[方法-两步解耦优化]] — 分变量 3×3 QR 替代 6-DOF 非线性

## 管线要点
- 地面检测: 下方 7 线, |angle - mount_angle| ≤ 10°
- 6 等分子区域 edge(前2 sharp + 3-20 less) + surf(最多4)
- 距离图像 BFS 聚类: ≥30 点直接有效
- 回环: 7m 半径 + 时间差>30s + PCL ICP fitness<0.3

## 工程局限
- 非结构化环境（空中/斜坡）地面假设失效
- 硬编码 N_SCAN=16 对 VLP-16 优化
- 无重定位/多传感器扩展

## 相关
- [[LiDAR方案对比]]
