---
tags: [LiDAR, 激光SLAM, 多传感器融合, 退化检测, ROS2, Ceres, GTSAM, SuperOdom]
sources:
  - raw/docs-deep-dive/superodom_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/superodom_analysis.md
---

# SuperOdom 深度源码分析

> ROS2 Humble 原生 LiDAR-IMU-VIO 松散耦合里程计。IMU-centric 架构，Ceres 解析雅可比 ICP + GTSAM ISAM2 预积分，SuperLoc 退化感知特色，支持 Velodyne/Ouster/Livox 即插即用。

## 摘要

SuperOdom 由三个独立 ROS2 节点管线构成。(1) [[方法-多源位姿预测策略]]实现 VIO-->Neural-->LIO-->IMU-->恒速 五级 fallback；(2) [[方法-解析雅可比 Ceres CostFunction]]手写 point-to-line/plane 解析雅可比比 Autodiff 快 2-3 倍；(3) [[方法-6-DoF 退化检测]]逐自由度可观测性分析与约束注入；(4) [[组件-GTSAM]]接收 LiDAR 里程计作为位姿先验。

## 核心概念

- **松散耦合**：LiDAR（Ceres ICP）--> IMU（GTSAM ISAM2 先验）--> LiDAR（IMU 初始猜测）双向信息流
- **特征策略**：统一均匀采样不区分 edge/planar，在线 PCA 分类，每 100 帧重置 ISAM2
- **局部地图**：21x21x11 三级网格（50m/格），自研 nanoflann Octree，TBB 并行
- **退化处理**：退化时添加 SE3AbsolutatePoseFactor 先验 + 切换预测源

## 相关页面

- [[2026-04-28-superodom-analysis|SuperOdom]]
- [[方法-多源位姿预测策略]]
- [[方法-解析雅可比 Ceres CostFunction]]
- [[方法-6-DoF 退化检测]]
- [[组件-GTSAM]]
- [[方法-体素地图]]