---
tags: [LiDAR, 回环, PoseGraph, 工业级]
sources:
  - raw/docs-deep-dive/cartographer_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/cartographer_analysis.md
---

# Cartographer 深度源码分析

> Google 2D/3D LiDAR SLAM 工业级架构：双 SLAM 架构、子地图系统、Branch-and-Bound 回环

## 摘要
Cartographer 采用 Local SLAM（scan-to-submap）+ Global SLAM（回环 + PGO）双层架构，通过子地图冻结后不可变的设计实现了高效的异步回环检测。Lua 配置系统实现场景自适应。

## 关键概念
- **[[算法-Cartographer]]**：2D/3D LiDAR SLAM，Ceres 扫描匹配 + 概率占据网格子地图
- **[[方法-子地图]]**：每个子地图 90 帧扫描，冻结后变为不可变共享指针，支持异步回环检测
- **[[方法-Branch-and-Bound回环检测]]**：多分辨率预计算网格栈 + BnB 搜索，O(n log n) 复杂度全子地图搜索
- **[[方法-概率占据网格]]**：log-odds 表示 + 查找表 O(1) 更新，双三次插值子像素占据概率

## 相关页面
- [[算法-LIO-SAM]]
- [[概念-回环检测方法]]
- [[概念-位姿图优化]]
- [[LiDAR方案对比]]