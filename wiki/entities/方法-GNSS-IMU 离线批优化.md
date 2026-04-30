---
type: entity
tags: [GNSS, IMU, 组合导航, 离线批优化, Ceres]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-ob_gins-analysis.md
---

# GNSS/IMU 离线批优化

> 面向组合导航数据集的离线 Ceres 批优化框架，用 GNSS 位置和 IMU 预积分/积分约束输出高精度轨迹。

## 定义

OB_GINS 实现的纯 GNSS+IMU 离线批处理组合导航优化框架，无 ROS 实时依赖，从文本文件读取数据，通过 Ceres 滑动窗口优化输出高精度轨迹。

## 核心特征

- 输入：IMU 文本文件 (dtheta, dvel) + GNSS 文本文件 (BLH, std)，非 ROS 接口
- 输出：优化后的 LLH 位置 + Euler 姿态 + NED 速度 + IMU 零偏
- 窗口大小默认 30 秒整秒节点，满时边缘化最老节点
- 配置驱动初始化（initvel/initatt/initgb/initab 由 user 指定）
- 支持里程计辅助（启用 PreintegrationOdo）
- 代码量 ≈ 15 个源文件，C++17，仅依赖 Ceres + Eigen3 + yaml-cpp

## 设计价值

离线批优化能使用未来数据重线性化历史状态，因此比实时 EKF 更适合做高精度轨迹后处理、传感器标定验证和算法基准。固定窗口边缘化则在精度和内存之间折中，使长数据集不必一次性构建完整全局问题。

## 适用边界

该框架不适合低延迟在线定位，也不直接处理视觉或 LiDAR 原始观测。GNSS 多径、周跳和 IMU 标定误差仍然需要前处理或鲁棒核，否则批优化会把系统误差传播到整段轨迹。

## 相关页面

- 实现：[[算法-OB_GINS]]
- 对比：[[算法-IC-GVINS]]（在线实时 + 视觉）
- [[架构-坐标系管理]]
- [[组件-Ceres-Solver]]
- [[方法-GNSS 位置残差因子]]
- [[GNSS数据管线]]
