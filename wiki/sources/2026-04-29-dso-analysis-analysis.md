---
tags: [VIO,PointFrameResidual::linearize (Residuals.cpp), EnergyFunctional::solveSystemF, makeKeyFrame (FullSystem.cpp)]
sources:
  - raw/docs-deep-dive/dso_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/dso_analysis.md
---

# DSO 源码级分析摘要

> DSO 稀疏直接法视觉里程计

## 核心发现

- 每个 3D 点仅优化 1 个参数（逆深度），Schur complement 求逆几乎零开销
- 显式建模帧间仿射亮度参数 (a,b) + 在线光度标定，处理相机曝光/增益变化
- 滑动窗口中手动实现 FEJ + 零空间正交化处理 7 DOF 不可观测性
- 粗跟踪用金字塔 LM 优化 8D（6DoF+2 仿射），多初始化策略（20+ 种恒速/扰动模型）
- [直接法,视觉里程计,光度标定,FEJ]

## 关键代码引用

- [[算法-DSO]] [[概念-直接法光度误差|光度误差]] [[概念-三角化与深度估计|逆深度参数化]] [[传感器-传感器标定|光度标定]] [[概念-直接法视觉里程计|粗跟踪]]

## 相关页面


