---
tags: [source, 源码分析, SLAM, fusions-slam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/fusions_slam_analysis.md
sources:
  - raw/docs-deep-dive/fusions_slam_analysis.md
---
# fusions_slam 深度分析

**日期**: 2026-04-29 | **语言**: C++/ROS1 | **依赖**: Eigen, PCL, iKD-Tree, Ceres, Glog

## 概要
模块化 IESKF + 多传感器松耦合 LIO 系统。自实现 18 维 [[方法-IESKF滤波器]]（pos/rot/vel/bg/ba/gravity）。[[方法-反向传播去畸变]](FAST-LIO2 经典方法)。多源观测: LiDAR point-to-plane + RTK 位置/姿态/速度。[[方法-Ceres后端融合]]：LIO 相对因子 + GPS 全局因子松耦合后端。

## 核心概念
- [[算法-fusions_slam]] — ROS-算法三层松耦合架构
- [[方法-反向传播去畸变]] — IMU前向积分 + 反向逐点补偿
- [[方法-Ceres后端融合]] — RelativeRTError + TError 因子图
- [[架构-Pipeline 传感器数据调度|MeasureGroupAdd]] — map<uint64_t, DataUnit> 多传感器时间排序
- [[方法-在线平面拟合|planarCheck]] — QR 最小二乘平面拟合

## 松耦合架构亮点
- type 层(纯Eigen/PCL) → 算法核心层(无ROS) → ROS wrapper层
- DataUnit.type 枚举支持 IMU/Lidar/GPS/Camera/Speed 扩展
- 外参在 Wrapper 层处理，算法核心接收统一坐标系

## 工程局限
- 无回环检测（纯 odometry 模式）
- 仅平面特征（非结构化场景退化）
- 硬编码阈值(planarCheck 0.1, 最近邻 5.0)
- 测试覆盖为零，ROS1 only

## 相关
- [[架构-多传感器融合架构]]
