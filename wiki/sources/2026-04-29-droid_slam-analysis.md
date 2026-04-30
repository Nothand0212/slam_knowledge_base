---
tags: [source, 源码分析, SLAM, droid-slam]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/droid_slam_analysis.md
sources:
  - raw/docs-deep-dive/droid_slam_analysis.md
---
# DROID-SLAM 深度分析

**日期**: 2026-04-29 | **语言**: Python+CUDA | **依赖**: PyTorch, lietorch, torch_scatter

## 概要
端到端可微深度学习 SLAM（NeurIPS 2021）。CNN 稠密特征 → 全对相关体 → [[方法-ConvGRU]] 迭代预测光流 → [[方法-Dense BA]]（GPU Schur 补 Cholesky 求解）。用光流残差替代传统重投影残差，逐像素优化位姿+逆深度。单目无需三角化，网络学习置信度权重替代 RANSAC 外点剔除。

## 核心概念
- [[算法-DROID-SLAM]] — 端到端可微 Dense BA
- [[方法-Dense BA]] — 逐像素光流残差 + Schur 消元
- [[方法-RAFT光流]] — 全对相关 + GRU 迭代修正
- [[方法-ConvGRU]] — 带全局上下文调制的卷积 GRU
- [[组件-lietorch]] — PyTorch+李群自动微分库

## 管线要点
- twin encoder (fnet 匹配 + cnet 上下文) → H/8 分辨率
- 多尺度相关查找: 4层 × 7×7 = 196 维 → GRU 输入
- Schur 补: 深度 Hessian 对角 → C⁻¹ 逐元素倒数 → 仅 P×6 位姿系统

## 训练亮点
- 三重损失: 测地线(10.0) + 残差(0.01) + 光流(0.05)
- 随机重启(0.2 概率): 课程学习提升优化鲁棒性

## 相关
- [[概念-深度学习SLAM]]
- [[VIO方案对比|VIO对比分析]]
