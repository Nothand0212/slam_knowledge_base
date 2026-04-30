---
tags: [SLAM, robotics, localization, mapping]
created: 2026-04-27
updated: 2026-04-27
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

# SLAM

> Simultaneous Localization and Mapping，同时估计机器人轨迹和环境地图；在 GTSAM 中通常被建模为 factor graph optimization 问题。

## 基本问题

SLAM 的核心不是单独定位，也不是单独建图，而是在位姿未知、地图未知的情况下同时估计二者。机器人用传感器观测约束自身运动和环境结构；随着时间推移，前端提供局部运动与观测关联，后端通过滤波或优化保持轨迹和地图一致。

## 在 GTSAM 中的表达

典型 SLAM 问题会把机器人 pose、landmark、速度、bias 等作为变量，把 odometry、loop closure、视觉重投影、IMU、GPS 等 measurement 写成 factors。

常见 GTSAM 组件：

- `Pose2` / `Pose3`：机器人位姿变量。
- `PriorFactor`：锚定初始 pose 或其他变量。
- `BetweenFactor`：odometry、relative pose、loop closure。
- projection/stereo factors：视觉 landmark measurement。
- `ImuFactor` / `CombinedImuFactor`：VIO/INS 预积分约束。
- `ISAM2`：在线增量式 SLAM。

## 范式差异

滤波式 SLAM 强调在线递推和固定状态规模，适合实时性强的系统；优化式 SLAM 强调批量或增量重线性化，适合高精度后端和回环修正。现代系统通常混合使用：前端滤波或局部优化提供实时里程计，后端因子图处理回环、GNSS 和全局一致性。

## 相关页面

- [[组件-GTSAM]]
- [[概念-因子图]]
- [[GTSAM 因子图工作流]]
- [[GTSAM SLAM 与视觉因子 API]]
