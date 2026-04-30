---
tags: [VIO,SparseImgAlign::run (sparse_img_align.cpp), align1D (feature_alignment.cpp), PoseOptimizer, Estimator]
sources:
  - raw/docs-deep-dive/svo_pro_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/svo_pro_analysis.md
---

# SVO Pro 源码级分析摘要

> SVO Pro 半直接法视觉惯性里程计

## 核心发现

- SparseImgAlign：用上一帧所有3D点通过光度误差直接估计位姿，不需要2D-2D特征匹配
- 1D 极线块对齐（align1D）用逆组合法沿极线精化特征位置，Hessian 只计算一次
- 后端 PoseOptimizer 用重投影误差 BA（三种误差类型）+ Ceres 滑动窗口 VIO BA
- cameraBundle + FrameBundle 原生多相机支持，每个相机独立 Reprojector
- [半直接法,VIO,多相机,深度滤波器]

## 关键代码引用

- [[算法-SVO-Pro]] [[算法-SVO-Pro|半直接法]] [[算法-SVO-Pro|SparseImgAlign]] [[概念-三角化与深度估计|深度滤波器]] [[概念-三角化与深度估计|1D极线对齐]]

## 相关页面

