---
tags: [VIO, 直接法, 视觉里程计]
sources:
  - raw/docs-deep-dive/dso_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/dso_analysis.md
---

# DSO 源码级分析摘要

> 直接法+稀疏的视觉里程计，证明了无需特征点、仅用数千个高梯度像素的光度残差即可超越间接法的精度。

## 核心发现
- 颠覆"直接法=稠密法"的刻板印象：只需 2000 个高梯度点的光度残差即能达到甚至超越间接法精度
- 每个 3D 点仅优化 **1 个参数：逆深度**，Schur complement 求逆开销几乎为零
- 显式建模帧间仿射亮度参数 $(a,b)$ + 在线光度标定，处理相机曝光/增益变化
- 滑动窗口中手动实现 FEJ（First-Estimate Jacobians），保证边缘化理论一致性
- 零空间正交化处理 7 DOF 不可观测性（全局 SE3 + 尺度），防止线性化漂移

## 技术要点
| 维度 | 方案 |
|------|------|
| 范式 | 直接法视觉里程计（纯单目） |
| 前端 | 粗跟踪（coarse tracker）：金字塔 LM 优化 6 DoF 位姿 + 2 仿射参数 |
| 后端 | 滑动窗口光度 BA + 手动 Schur complement 边缘化 |
| 独特创新 | 像素点 1D 逆深度参数化、在线光度标定、FEJ 零空间正交化 |

## 关键引用
- 光度残差计算：`src/FullSystem/Residuals.cpp:78-328`
- 系统矩阵组装与求解：`src/OptimizationBackend/EnergyFunctional.cpp:775-913`
- 粗跟踪：`src/FullSystem/CoarseTracker.cpp:536-718`
- 关键帧管理：`src/FullSystem/FullSystem.cpp:802-903, 1041-1196`
- 帧边缘化策略：`src/FullSystem/FullSystemMarginalize.cpp:58-147`
- 零空间正交化：`src/OptimizationBackend/EnergyFunctional.cpp:718-772`

## 相关页面
- [[2026-04-29-dm-vio-analysis-analysis|DM-VIO方案分析]]
- [[概念-直接法vs间接法]]
- [[VIO方案对比]]