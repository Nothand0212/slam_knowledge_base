---
tags: [VIO, 滤波, 视觉惯性, MSCKF]
sources:
  - raw/docs-deep-dive/msckf_vio_analysis.md
created: 2026-04-29
type: source
updated: 2026-05-15
source_path: raw/docs-deep-dive/msckf_vio_analysis.md
---

# MSCKF_VIO 源码级分析摘要

> MSCKF_VIO 深度源码分析

## 核心发现

- 使用 FAST 角点检测 + KLT 金字塔光流，网格特征管理（20格×4点），IMU 旋转预测 + 2点 RANSAC 离群剔除
- 静止初始化需 ≥200 条 IMU：陀螺偏置取均值、重力方向对齐、加速度偏置为零
- RK4 名义状态预测 + 3 阶矩阵指数误差态转移 + OC-MSCKF 可观测性约束修正
- MSCKF 更新用 SVD 零空间投影消去路标变量，SPQR 稀疏 QR 压缩，卡方检验门控
- 最大 20 个相机状态，自适应关键帧删除策略（旋转/平移/跟踪率三阈值）
- 无回环检测、无全局优化，onlineReset 在协方差过大时重置

## 关键代码引用

- predictNewState (msckf_vio.cpp), processModel, measurementJacobian, featureJacobian, pruneCamStateBuffer

## 相关页面

[[概念-MSCKF]] [[概念-MSCKF|OC-MSCKF]] [[方法-视觉特征跟踪|FAST角点检测]] [[概念-MSCKF|零空间投影]] [[概念-IMU预积分|RK4积分]]
