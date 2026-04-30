---
tags: [视觉SLAM, 深度学习, DenseBA, 光流]
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# DROID-SLAM

> DROID-SLAM是基于深度学习的端到端视觉SLAM系统，使用循环迭代光流更新和Dense Bundle Adjustment，在单目、双目和RGB-D模式下实现高精度稠密跟踪与建图。

## 核心方法

DROID-SLAM 的核心是 RAFT 风格的迭代光流更新网络。图像先经过 twin encoder 得到匹配特征和上下文特征，再构造全对相关体；ConvGRU 反复查询相关体并预测光流残差、置信度和更新方向。所有帧间稠密光流约束随后进入 Dense Bundle Adjustment（DBA）层，通过 Schur 补消元逆深度，只求解较小的位姿系统。

## 关键设计
- DenseBA层：全图像素级BA，网络前向传播即优化
- 迭代光流：ConvGRU逐次修正，避免一次性预测的误差累积
- 多帧信息融合：在特征相关体（correlation volume）中索引远距离帧
- 支持单目/双目/深度：统一框架，仅输入通道不同

## 与传统 SLAM 的差异

DROID-SLAM 不依赖角点、手工描述子、PnP RANSAC 或显式回环模块，而是把稠密对应和优化权重交给网络学习。几何部分并未消失：位姿仍在 SE(3) 上优化，逆深度仍通过 BA 更新，只是残差从传统重投影误差变成网络预测的稠密光流误差。

## 工程边界

系统依赖 GPU、PyTorch/CUDA 扩展和训练数据分布。网络学习的置信度可以替代一部分外点剔除，但在强动态、极端曝光变化和域外场景中仍需要外部质量控制。对机器人落地而言，DROID-SLAM 更适合作为可微 Dense BA 范式参考，而不是直接替代所有传统后端。

## 相关页面
- [[2026-04-28-droid-slam-analysis]]
- [[VIO方案对比]]
- [[概念-深度学习SLAM]]
- [[架构-滑动窗口优化]]
- [[方法-RAFT光流]], [[方法-Dense BA]]
