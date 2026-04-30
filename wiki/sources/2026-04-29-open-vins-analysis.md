---
tags: [open_vins, MSCKF, VIO, EKF, 源码分析, IMU预积分, 视觉惯性]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/open_vins_analysis.md
sources:
  - raw/docs-deep-dive/open_vins_analysis.md
---

# OpenVINS 深度源码级分析

> MSCKF 滑动窗口滤波器的完整源码分析（v2.7），覆盖数据接收、特征提取、初始化、状态估计、优缺点全链路

## 核心算法

**MSCKF (Multi-State Constraint Kalman Filter)**：滑动窗口 EKF 滤波器，通过零空间投影消除特征坐标变量

## 特征跟踪

支持两种模式：
- **KLT 光流**（TrackKLT）：快，适合同帧率高场景
- **描述子匹配**（TrackDescriptor）：适应大基线、长间隔

特征处理 100% 依赖 OpenCV：FAST 角点 + KLT 光流 + ORB 描述子

## 状态估计流程

```
IMU 传播（EKF 预测步）
  → 位姿克隆（augment_clone）
    → MSCKF 更新（零空间投影 + Chi² 检验）
      → EKF 更新（K = P*H^T*(H*P*H^T+R)^-1）
        → 边缘化老 clone
```

## 三种 IMU 积分模式
- **离散积分**：快
- **RK4**：平衡
- **ACI² 解析**：最高精度

## 优势与局限

**优势**：ROS-Free 内核、多线程异步初始化、在线标定、模块化协方差系统

**局限**：无内建回环检测、EKF 一次性线性化、纯 CPU

## 相关页面

- [[概念-MSCKF]]
- [[概念-IMU预积分]]
- [[概念-MSCKF]]
- [[2026-04-28-open_vins-analysis]]
- [[概念-视觉惯性初始化策略]]