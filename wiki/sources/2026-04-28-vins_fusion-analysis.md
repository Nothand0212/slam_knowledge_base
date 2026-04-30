---
tags: [VIO, 视觉惯性里程计, 滑动窗口, 因子图, 回环检测]
sources:
  - raw/docs-deep-dive/vins_fusion_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/vins_fusion_analysis.md
---

# VINS-Fusion 源码级分析摘要

> 三进程架构（VIO + 回环 + GPS）的完整 SLAM 流水线，基于 Ceres 因子图 + 滑动窗口 BA，支持多传感器模式。

## 核心发现

- 前端使用 Shi-Tomasi 角点（`goodFeaturesToTrack`）+ KLT 金字塔光流，无描述子匹配
- 视觉预测机制：利用 Estimator 传入的预测 3D 点做光流初始猜测，显著加速跟踪
- 初始化分三种模式：Mono+IMU（SFM + IMU-visual 对齐两步法）、Stereo+IMU（PnP + 陀螺偏置估计）、Stereo only
- 滑动窗口优化使用 Ceres DENSE_SCHUR + DogLeg 策略，残差包括 IMU 预积分 + 视觉重投影 + 边缘化先验 + 立体约束
- IMU 预积分使用**中值积分**（mid-point，RK2），含完整 Jacobian/协方差传播和偏置变化时重传播机制
- Schur 补边缘化保留历史信息，两种策略（MARGIN_OLD / MARGIN_SECOND_NEW）依据视差和跟踪质量选择
- 回环检测使用 DBoW2 + **BRIEF 描述子**词袋，几何验证用 PnP RANSAC，位姿图用 4-DOF（yaw+translation）优化
- GPS 全局融合采用松耦合方式，GeographicLib 做局部 ENU 坐标转换

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | Shi-Tomasi 角点（OpenCV），按跟踪时长排序 + 距离掩码 |
| 跟踪方法 | KLT 金字塔光流 + 反向光流验证 |
| 后端 | Ceres 滑动窗口 BA + Schur 补边缘化 |
| 回环 | DBoW2 + BRIEF 词袋 + 4-DOF 位姿图优化 |
| ROS 耦合 | 深度耦合（ROS 消息 + Topic 通信） |
| 在线标定 | 支持外参旋转 + 时间偏移（td）在线估计 |

## 关键代码引用

- 前端跟踪: `FeatureTracker::trackImage` (feature_tracker.cpp:94-306)
- IMU 预积分: `IntegrationBase::midPointIntegration` (integration_base.h:63-137)
- 滑动窗口优化: `Estimator::optimization` (estimator.cpp:1004-1327)
- Schur 补边缘化: `MarginalizationInfo::marginalize` (marginalization_factor.cpp:183-311)
- 回环检测: `PoseGraph::detectLoop` (pose_graph.cpp:335-417)

## 相关页面

- [[VIO方案对比]]
- [[概念-因子图|因子图优化与滑动窗口]]
- [[概念-回环检测方法]]
- [[GNSS数据管线|GPS视觉惯性融合]]- [[概念-MSCKF]]
- [[概念-位姿图优化]]
