---
tags: [source, 源码分析, SLAM, fast-lio-sam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/fast_lio_sam_analysis.md
sources:
  - raw/docs-deep-dive/fast_lio_sam_analysis.md
---
# FAST-LIO-SAM-SC-QN 深度分析

**日期**: 2026-04-29 | **语言**: C++17/ROS1 | **依赖**: GTSAM, PCL, TEASER++, tbb

## 概要
后端 PGO + 回环检测系统（依赖外部 FAST-LIO2 做 LIO 前端）。[[方法-四阶段回环验证]]流水线: [[方法-ScanContext]] → 欧式距离 → [[方法-Quatro配准]](FPFH+GNC) → [[组件-Nano-GICP]](FastGICP 分布匹配)。后端 [[组件-GTSAM|GTSAM ISAM2]] 增量优化，回环后 3× update 收敛。

## 核心概念
- [[算法-FAST-LIO-SAM-SC-QN]] — LIO+PGO 分离架构
- [[方法-四阶段回环验证]] — 多层验证确保回环质量
- [[方法-Quatro配准]] — FPFH + GNC 全局配准
- [[组件-Nano-GICP]] — NanoFLANN + FastGICP 体素分布匹配
- [[方法-ScanContext]] — 轻量 LiDAR 全局描述子

## 管线要点
- 关键帧: 平移 > 1.5m 欧式距离阈值
- 实时外推: last_corrected × odom_delta → 高频 pose
- 点云存储为 LiDAR frame 坐标（便于回环重投影）

## 工程局限
- 自身无 LIO 前端，依赖多进程通信
- ScanContext 对称场景可能误匹配
- ROS1 only，无 ROS2 支持

## 相关
- [[LiDAR方案对比]]
