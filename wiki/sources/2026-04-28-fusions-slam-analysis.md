---
tags: [LiDAR, 激光SLAM, fusions_slam, IESKF, 松耦合, RTK]
sources:
  - raw/docs-deep-dive/fusions_slam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/fusions_slam_analysis.md
---

# fusions_slam 源码级分析摘要

> 教科书级 ROS-算法分离架构，自研 18 维 IESKF + iKD-Tree 增量地图，支持 LiDAR/RTK 多源观测融合，含 Cere 后端因子图优化。

## 核心发现

- **三层松耦合架构**：type（纯数据）→ 算法核心（fusion_slam/，零 ROS 依赖）→ ROS 封装（wrapper/ros/）
- 自研 **18 维状态 IESKF**，统一支持 4 种观测：点云（point-to-plane）、位置、姿态、速度
- **MeasureGroupAdd 机制**使用 `std::map<uint64_t, DataUnit>` 统一多传感器时间排序
- 反向传播去畸变（Propagate 类独立实现）借鉴 FAST-LIO2，前向 IMU 积分 + 反向点云补偿
- 后端 **LioGpsOpt** 实现 LIO 相对因子 + GPS 全局因子的 Ceres 松耦合，异步优化线程
- 支持 Velodyne/Ouster 激光雷达及 3 种 RTK 协议，双模式（建图/定位）

## 技术要点

| 维度 | 方案 |
|------|------|
| 前端 | 自研 IESKF（18 维状态，欧拉积分） |
| 特征提取 | 在线平面拟合（planarCheck），无显式特征分类 |
| 配准方法 | iKD-Tree 最近邻 + point-to-plane 残差 + Huber 权重 |
| IMU融合 | IESKF 预测步骤，无预积分 |
| 后端 | Cere 松耦合（LIO 相对 + GPS 全局因子） |
| 回环 | 无回环检测 |
| ROS耦合 | ROS1 (catkin)，wrapper 层极薄 |
| 地图管理 | iKD-Tree 增量维护，FOV 动态窗口平移 |

## 关键代码引用

- IESKF: `IESKF` (include/fusion_slam/modules/ieskf/ieskf.h:50)
- 前向传播: `IESKF::predict()` (src/fusion_slam/modules/ieskf/ieskf.cpp:70)
- 平面拟合: `planarCheck()` (include/fusion_slam/math/math.h:22)
- 去畸变: `Propagate::run()` (src/fusion_slam/modules/sensors/imu/propagate.cpp:33)
- 时间同步: `Front::syncMeasureGroupAdd()` (src/fusion_slam/modules/front/front.cpp:160)
- 后端融合: `LioGpsOpt` (include/fusion_slam/modules/back/lio_gps_opt.h:23)

## 相关页面

- [[LiDAR方案对比]]
- [[方法-IESKF滤波器]]
- [[方法-反向传播去畸变|IMU反向传播去畸变]]
- [[组件-Ceres-Solver|Ceres优化器]]
- [[架构-多传感器融合架构|多源观测融合]]- [[算法-FAST-LIO]]
- [[算法-KISS-ICP]]
