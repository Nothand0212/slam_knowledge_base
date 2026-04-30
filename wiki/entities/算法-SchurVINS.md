---
tags: [VIO, EKF, Schur Complement, 轻量化]
sources:
  - wiki/sources/2026-04-29-schurvins-analysis-analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# SchurVINS

> Schur Complement-Based Lightweight Visual Inertial Navigation System (CVPR 2024)，基于 SVO 2.0 的 Schur Complement EKF 轻量 VIO 系统。

## 概述

SchurVINS 提出了一种介于标准优化和传统滤波之间的混合方案。核心思路是用 **Schur Complement 边缘化**替代 MSCKF 的零空间投影来处理路标变量。在每次视觉更新中，系统先累积所有路标/观测的梯度（一阶）和 Hessian（二阶），然后通过 Schur complement 消去路标变量，得到一个仅含滑动窗口位姿状态的降维残差模型，再执行标准 EKF 更新。

与 MSCKF 的关键区别：MSCKF 通过 SVD 寻找路标雅可比矩阵的左零空间，将观测投影后消去路标；SchurVINS 则直接构建 Hessian 并做 Schur complement，保留完整的梯度信息和 Hessian 结构，在数学上与优化框架中的边缘化完全等价。

## 在分析框架中的应用

- EKF 状态：15 维 IMU（旋转/平移/速度/加速度偏置/陀螺偏置）+ 4×6 维窗口位姿 = 39 DOF
- IMU 前向预测使用 RK4 直接积分（无标准预积分框架）
- FEJ（First-Estimate Jacobian）保证边缘化线性化点一致性
- 地标独立 EKF（Point::EkfUpdate）利用不在窗口内的历史关键帧做轻量更新

## 核心优势

1. **高精度**：完备残差模型（梯度 + Hessian），无信息损失
2. **轻量计算**：每个路标仅 3×3 矩阵求逆，EKF 状态 < 40 DOF
3. **Huber 鲁棒**：在 Schur complement 前加权处理离群点

## 局限

1. 无回环检测，长时间运行有漂移
2. EKF 一次线性化，对非线性场景敏感
3. 窗口固定（window_size=4），历史信息利用有限
4. IMU 非预积分，依赖直接积分

## 相关页面

- [[概念-MSCKF]]
- [[概念-Schur补与边缘化]]
- [[架构-滑动窗口优化]]
- [[算法-SVO-Pro]]