---
tags: [source, 源码分析, SLAM, nice-slam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/nice_slam_analysis.md
sources:
  - raw/docs-deep-dive/nice_slam_analysis.md
---
# NICE-SLAM 深度分析

**日期**: 2026-04-29 | **语言**: Python+CUDA | **依赖**: PyTorch

## 概要
CVPR 2022 神经隐式 RGB-D SLAM。用 [[方法-层次化特征网格]]（四级 coarse/middle/fine/color 32D 网格） + [[方法-ConvONet|ConvONet 解码器]]表示连续场景。[[概念-可微渲染]] 驱动 Tracking（Adam 优化 7-DOF 位姿）和 Mapping（分阶段 middle→fine→color 联合优化）。

## 核心概念
- [[算法-NICE-SLAM]] — 层次化神经隐式 SLAM
- [[方法-层次化特征网格]] — 多分辨率 3D 特征格，三线性插值
- [[方法-ConvONet]] — 预训练几何/颜色解码器
- [[概念-可微渲染]] — 体积渲染 + 梯度下降替代 ICP
- [[概念-体积渲染]] — alpha 合成累计透射率

## 管线要点
- 每帧随机 1024 射线 → N_samples=16+N_surface=4 → 渲染对比
- 动态物体过滤: 渲染不确定度 z-test
- 多进程: Tracker/Mapper/CoarseMapper 并行，shared_memory 同步

## 工程局限
- GPU 必须，帧率 1-3 FPS，bounding box 预设限制场景规模
- 仅 RGB-D，无语义/回环/IMU

## 相关
- [[概念-深度学习SLAM]]
