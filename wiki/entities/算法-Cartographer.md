---
tags: [LiDAR, 激光SLAM, 2D SLAM, 回环检测, 图优化]
sources:
  - wiki/sources/2026-04-28-bev_lslam-analysis.md
  - wiki/sources/2026-04-29-bev_lslam_analysis.md
  - wiki/sources/2026-04-29-cartographer_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# Cartographer

> Google Cartographer是Google开源的工业级2D/3D实时SLAM库，使用子图（Submap）扫描匹配和稀疏位姿调整（SPA）后端，支持多传感器融合和大尺度室内外场景。

## 核心方法
Cartographer将空间划分为局部子图（Submap）：新的LiDAR扫描通过Ceres-based相关扫描匹配（CSM）对齐到当前子图。子图完成后，进入全局回环检测（使用分支定界 Branch-and-Bound 在搜索窗口内查找匹配）。后端使用稀疏位姿调整（SPA）对所有子图-扫描间的约束做位姿图优化，无需全BA。2D模式仅用单线LiDAR，3D模式支持多线LiDAR+IMU。

## 关键设计
- 子图架构：局部地图隔离累积误差，完成后冻结不再变化
- 分支定界回环：O(n)到O(log n)的搜索加速，支持大范围回环
- SPA后端：稀疏位姿图优化比全BA效率高数个数量级
- 多传感器：可选IMU、轮速计和GPS作为先验约束

## 子图思想

子图是 Cartographer 的核心抽象。局部扫描先在活跃子图内累积，子图完成后冻结；全局后端只优化扫描和子图之间的相对约束。这样既避免单个全局栅格持续变形，又让回环约束可以通过子图位姿传播到全局地图。

## 工程边界

Cartographer 的参数非常依赖传感器频率、运动速度和环境尺度。搜索窗口过小会漏回环，过大会拖慢实时性；子图过大增加局部漂移，过小则约束碎片化。它更像一个可配置 SLAM 框架，而不是默认参数即可适配所有机器人。

## 相关页面
- [[2026-04-28-cartographer-analysis]]
- [[2026-04-29-external-primary-source-check]]
- [[LiDAR方案对比]]
- [[架构-滑动窗口优化]]
- [[方法-Branch-and-Bound回环检测]], [[方法-子地图]]
