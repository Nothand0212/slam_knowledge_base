---
tags: [算法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-fusions_slam-analysis.md
---
# fusions_slam

> 模块化 LIO/GNSS 融合工程：纯算法核心与 ROS wrapper 分离，自实现 18 维 IESKF，并用 Ceres 后端融合 GPS。

## 架构定位

fusions_slam 是模块化 IESKF + 多传感器松耦合 LIO 工程。代码组织分为三层：type 层提供纯 Eigen/PCL 数据结构，算法核心层实现 IESKF、前端、地图和 IMU 传播，ROS wrapper 层负责消息转换、外参处理和话题接口。这种分层让算法核心不直接依赖 ROS，便于迁移和复用。

## 状态估计

系统自实现 18 维 IESKF，状态包含位置、姿态、速度、陀螺偏置、加速度偏置和重力。LiDAR 前端使用 point-to-plane 约束更新滤波器，IMU 负责高频传播，RTK/GPS 可提供位置、姿态和速度观测。后端再用 Ceres 构造 LIO 相对因子和 GPS 全局因子，实现松耦合轨迹优化。

## 地图与调度

局部地图使用 iKD-Tree 增量维护，并通过 FOV 动态窗口裁剪控制规模。传感器数据通过 `DataUnit` 和时间有序容器调度，理论上可扩展 IMU、LiDAR、GPS、Camera、Speed 等来源。系统同时支持建图模式和已知地图定位模式。

## 工程局限

fusions_slam 更像可学习的融合工程骨架，而不是完整长期 SLAM。它没有回环检测，长期运行会累积漂移；点云前端主要依赖平面特征，在非结构化场景会退化；部分阈值硬编码，且测试覆盖不足。对新项目最值得借鉴的是 ROS/算法分层和多源数据调度，而不是直接照搬阈值。

## 相关
- [[方法-IESKF滤波器]]
- [[方法-反向传播去畸变]]
- [[方法-Ceres后端融合]]
- [[架构-Pipeline 传感器数据调度|MeasureGroupAdd]]
- [[方法-在线平面拟合|planarCheck]]
- [[架构-多传感器融合架构]]
