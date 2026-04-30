---
tags: [多相机, VIO, ORB-SLAM3, SE2(3), 预积分]
sources:
  - wiki/sources/2026-04-29-openmavis-analysis-analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# OpenMAVIS

> 基于 ORB-SLAM3 的多相机增强视觉惯性 SLAM 系统，支持 4 路相机（前视双目 + 侧视双目）的 IMU_MULTI 模式，引入 SE2(3) 精确 IMU 预积分。

## 概述

OpenMAVIS 在 ORB-SLAM3 基础上扩展了多相机支持（sensor 类型新增 `IMU_MULTI=6`），通过链式 SE3 外参变换从主相机位姿推导所有相机位姿。核心贡献包括：

1. **SE2(3) 精确 IMU 预积分**：使用 J1(dt,θ)/J2(dt,θ) 精确捕捉角速度对位置/速度更新的耦合效应，在高角速度场景精度显著优于 ORB-SLAM3 的零阶近似
2. **统一的 cam_idx 投影框架**：`ImuCamPose` 类自动推导 1/2/4 相机的 Rcw/tcw，视觉边携带相机索引
3. **自适应特征策略**：侧视相机分配 2 倍特征数以补偿单目观测稀疏性

## 关键设计

- 所有相机通过 `Tlr`/`Tlsl`/`Tlsr` 从主相机 cam1 链式推导
- 侧视相机假设与主相机无显著重叠，不做跨相机匹配
- 初始化仅用前视立体，侧视贡献推迟到后续帧通过单目投影匹配
- 继承 ORB-SLAM3 完整回环检测能力（DBoW2 + Sim3 + 位姿图）

## 局限

- 侧视无直接深度，三角化延迟
- 外参假设精确且不变（无在线标定）
- 最多 4 相机硬编码上限
- 特定硬件假设（Hilti 2022 相机布局）

## 相关页面

- [[算法-ORB-SLAM3|ORB-SLAM3]]
- [[概念-IMU预积分]]
- [[架构-多传感器融合架构]]
- [[组件-CameraRig 多相机抽象|多相机链式外参]]