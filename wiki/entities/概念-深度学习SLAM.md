---
tags: [SLAM, 深度学习, DenseBA, 神经隐式]
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
  - wiki/sources/2026-04-29-monogs-analysis.md
  - wiki/sources/2026-04-29-nice_slam-analysis.md
  - wiki/sources/2026-04-29-paper-notes.md
  - wiki/sources/2026-04-29-pin_slam_analysis.md
  - wiki/sources/2026-04-29-special-sensors-comparison.md
created: 2026-04-28
updated: 2026-04-29
type: entity
---
# 深度学习SLAM

> 用可学习模块替代传统 SLAM 中的手工特征、匹配、优化或地图表示，使前端感知和后端几何之间形成更强的数据驱动耦合。

## 三类路线

深度学习 SLAM 不是单一路线，常见做法可分为三类。第一类是学习前端，用 SuperPoint、SuperGlue、RAFT 等模块替代角点、描述子或光流匹配，但后端仍是几何优化。第二类是可微优化，把 BA、光流更新或位姿跟踪嵌入网络迭代中，例如 [[算法-DROID-SLAM]] 的 RAFT 光流和 Dense BA。第三类是学习式地图表示，用神经隐式场、SDF、3D Gaussian Splatting 或点特征地图表达场景，例如 [[算法-NICE-SLAM]]、[[算法-MonoGS]] 和 [[算法-PIN-SLAM]]。

## 与传统 SLAM 的关系

学习方法并不必然替代几何。效果较好的系统往往仍保留投影模型、SE(3) 位姿、光度/几何残差、Bundle Adjustment 或滤波更新，只是把难以手工建模的部分交给网络学习。例如 DROID-SLAM 用网络估计稠密对应和置信度，但位姿更新仍依赖 Dense BA；MonoGS 使用可微高斯渲染，但跟踪本质仍是最小化渲染误差。

## 工程取舍

优势是低纹理场景中的稠密匹配能力更强，地图可直接支持渲染、补全和语义任务；代价是 GPU 依赖、数据集偏置、在线内存增长和失败可解释性。对于机器人系统，最稳妥的落地方式通常不是端到端替换整条 pipeline，而是把学习模块放在可验证边界内：学习特征、学习匹配、学习动态物体剔除或学习地图先验，再由几何后端做一致性约束。

## 对知识库的组织意义

深度学习 SLAM 页面应和传统 SLAM 页面互相连接：Dense BA 对应 [[概念-Schur补与边缘化]]，可微渲染对应 [[概念-可微渲染]]，3DGS 对应 [[方法-3D Gaussian Splatting]]，隐式地图对应 [[方法-SDF解码器]] 和 [[方法-点基隐式神经表示]]。这样能避免把神经方法孤立成论文清单，而是放回 SLAM 的状态估计和地图表示框架里理解。

## 相关页面
- [[方法-RAFT光流SLAM]]
- [[算法-DROID-SLAM]], [[算法-NICE-SLAM]], [[算法-MonoGS]], [[算法-PIN-SLAM]]
- [[方法-RAFT光流]], [[方法-Dense BA]]
- [[概念-可微渲染]], [[方法-3D Gaussian Splatting]]
