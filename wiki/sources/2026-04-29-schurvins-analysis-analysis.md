---
tags: [VIO,SchurVINS::Solve3 (schur_vins.cpp), Prediction, Backward, StateUpdate]
sources:
  - raw/docs-deep-dive/schurvins_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/schurvins_analysis.md
---

# SchurVINS 源码级分析摘要

> SchurVINS 源码深度分析报告

## 核心发现

- Schur Complement 边缘化替代传统 MSCKF 零空间投影：累积梯度+Hessian 完备残差，3×3 矩阵求逆
- EKF 状态：15 维 IMU + 4×6 维窗口位姿 = 39 DOF，FEJ 保证线性化一致性
- IMU 前向预测用 RK4 直接积分（无预积分框架），后向视觉更新用 Schur Complement + EKF
- 关键帧选择基于共视点跟踪率（<0.70 删最老帧，否则删次新帧）
- 地标独立 EKF（Point::EkfUpdate）利用历史关键帧做轻量更新
- [VIO,Schur Complement,EKF,轻量化]

## 关键代码引用

- [[算法-SchurVINS]] [[算法-SchurVINS|Schur Complement EKF]] [[概念-MSCKF|FEJ]] [[方法-IESKF滤波器|IEKF]]

## 相关页面


