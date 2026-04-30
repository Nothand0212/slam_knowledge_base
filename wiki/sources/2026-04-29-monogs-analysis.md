---
tags: [source, 源码分析, SLAM, monogs]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/monogs_analysis.md
sources:
  - raw/docs-deep-dive/monogs_analysis.md
---
# MonoGS 深度分析

**日期**: 2026-04-29 | **语言**: Python+CUDA | **依赖**: PyTorch, diff_gaussian_rasterization

## 概要
CVPR 2024 [[方法-3D Gaussian Splatting]] 单目 SLAM。百万级各向异性高斯椭球表示场景，[[数学-球谐函数]] 编码视角相关外观，[[方法-微分栅格化]] CUDA alpha-blending 渲染。Tracking: Adam 优化 6-DOF + 曝光参数（光度 L1 损失）。Mapping: 窗口内联合优化 Gaussians + 关键帧位姿（isotropic 正则 + 稠密化剪枝）。

## 核心概念
- [[算法-MonoGS]] — 3DGS + SLAM
- [[方法-3D Gaussian Splatting]] — xyz/rgb/scale/quat/opacity 参数化
- [[数学-球谐函数]] — 视角相关颜色编码
- [[方法-微分栅格化]] — CUDA splatting，对相机位姿增量可微
- [[数学-SE3指数映射]] — 右乘扰动模型更新位姿

## 管线要点
- 关键帧决策: 平移距离 + 共视度 Szymkiewicz-Simpson 系数
- 窗口: 8-10 帧滑动，移除重叠最低帧
- 单目深度"涌现": 随机初始化 → BA 收敛

## 工程局限
- GPU 必须，~3 FPS，无回环/重定位/B-spline 轨迹
- 硬编码 isotropic 正则 10× → 可能约束真实各向异性结构

## 相关
- [[概念-深度学习SLAM]]
