---
tags: [VIO, 视觉惯性, EKF, 滤波, 地标表征]
sources:
  - wiki/sources/2026-04-29-rovio-analysis-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# ROVIO

> ROVIO（Robust Visual Inertial Odometry）是一种基于EKF的紧耦合视觉惯性里程计，使用图像块（patch-based）直接法跟踪地标，每帧更新机器人状态和地标方位向量。

## 核心方法
ROVIO将视觉特征建模为BEAR方位向量（Bearing Vector），而不是3D路标点坐标。每个特征以相机光心出发的单位方向向量和逆深度表示，随相机运动方向向量在EKF中自然旋转。视觉误差定义为特征块在预测位置和观测位置间的光度差异，直接与IMU传播的状态在EKF中紧耦合更新。此设计避免了三角测量过程，状态维度也较低（方位向量2-DoF + 逆深度1-DoF）。

## 关键设计
- BEAR方位参数化：特征表示为方向向量+逆深度，避免3D重构
- 光度误差：直接法在图像块上最小化亮度差异，无需描述符匹配
- EKF紧耦合：IMU和视觉更新共享同一滤波器，非松耦合两步法
- 鲁棒性：直接法+地标方位在纹理稀疏区域仍能工作

## 与 MSCKF/滑窗的区别

ROVIO 把地标直接放进 EKF 状态并用 patch 光度误差更新；MSCKF 通常把特征作为临时变量，通过零空间投影消除特征；滑窗 BA 则在窗口内反复优化位姿和路标。ROVIO 的优势是在线滤波速度快、状态紧凑，代价是单次线性化带来一致性压力。

## 工程边界

Patch 直接法依赖亮度一致性，对曝光突变、运动模糊和动态物体敏感。EKF 框架也要求良好的噪声模型和初始化，若早期 bias 或尺度估计错误，后续很难像 BA 那样全局重线性化修正。

## 相关页面
- [[2026-04-28-rovio-analysis]]
- [[VIO方案对比]]
- [[概念-MSCKF]]
- [[概念-直接法视觉里程计]]
- [[2026-04-29-external-primary-source-check]]
