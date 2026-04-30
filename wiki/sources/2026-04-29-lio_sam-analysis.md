---
tags: [source, 源码分析, SLAM, lio-sam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/lio_sam_analysis.md
sources:
  - raw/docs-deep-dive/lio_sam_analysis.md
---
# LIO-SAM 深度分析

**日期**: 2026-04-29 | **语言**: C++/ROS1 | **依赖**: GTSAM, PCL, OpenCV

## 概要
LiDAR-IMU 紧耦合 + GPS 因子图 SLAM（3822 stars）。4 个 ROS nodelet: ImageProjection(IMU deskew) → FeatureExtraction(曲率 edge/planar) → imuPreintegration(GTSAM IMU因子) → mapOptmization(scan-to-map LM + ISAM2 全局图)。[[架构-两级优化架构]]：30 次 LM 前端 + ISAM2 增量后端。

## 核心概念
- [[算法-LIO-SAM]] — 工程化 LiDAR-IMU 标杆
- [[架构-两级优化架构]] — LM前端 + ISAM2后端解耦
- [[方法-IMU deskew]] — 角速度积分+逐点旋转校正
- [[方法-退化检测与修复]] — J^T J SVD + 投影修正
- [[概念-IMU预积分|IMU预积分因子]] — GTSAM PreintegratedImuMeasurements

## 因子图设计
- PriorFactor<Pose3>: roll/pitch 1e-2, yaw π², trans 1e8（yaw/trans 弱先验）
- BetweenFactor<Pose3>: rot 1e-6, trans 1e-4（旋转约束远紧于平移）
- GPSFactor: 移动 >5m + 协方差 > 阈值
- 回环: PCL ICP fitness <0.3 → BetweenFactor
- 每100帧重置 ISAM2，保留边缘协方差 prior

## 相关
- [[LiDAR方案对比]]
- [[因子图vs滤波]]
