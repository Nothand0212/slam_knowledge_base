---
tags: [方法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-kimera_vio-analysis.md
---

# OnlineGravityAlignment

> VIO 初始化中的在线重力对齐流程：在视觉尺度、初始速度、陀螺 bias 和重力方向之间建立一致初值。

## 定义

OnlineGravityAlignment 是 Kimera-VIO 的视觉-惯性初始化模块，思路接近 VINS-Mono：先用视觉 SfM/里程计提供相对运动，再用 IMU 预积分约束估计陀螺 bias、速度和重力方向。目标是在后端正式优化前给出物理一致的初始状态。

## 流程

1. `estimateGyroscopeBiasOnly`：先估计陀螺 bias，减少旋转预积分误差。
2. `updateDeltaStates`：用新的 bias 更新 IMU 预积分 delta states。
3. `alignEstimatesLinearly`：线性求解初始速度和重力向量。
4. `refineGravity`：在重力切空间中细化方向，保持重力模长约束。

其中 `createTangentBasis(g0)` 构造重力方向的 2-DoF 切空间基 `b1, b2`，避免把重力当成自由 3D 向量随意优化。

## 工程意义

- VIO 初始化失败通常不是单个参数错，而是尺度、重力、速度和 bias 不一致。
- 重力方向决定 roll/pitch 可观测性，对后续预积分残差影响很大。
- 切空间细化比直接优化 3D 重力向量更符合物理约束。

## 相关页面

- [[算法-Kimera-VIO]]
- [[概念-视觉惯性初始化策略]]
- [[概念-IMU预积分]]
- [[数学-流形优化]]
