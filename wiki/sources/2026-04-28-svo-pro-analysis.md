---
tags: [VIO, 半直接法, 多相机]
sources:
  - raw/docs-deep-dive/svo_pro_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/svo_pro_analysis.md
---

# SVO Pro 源码级分析摘要

> 半直接法混合架构：前端用直接法（快不鲁棒）做位姿初始化，后端用间接法（慢但鲁棒）做重投影精化。

## 核心发现
- **三阶段流水线**：Sparse Image Alignment（直接法→初始位姿）→ Feature Alignment（1D 极线块对齐）→ Pose + Structure Optimization（间接法重投影 BA）
- 前端完全不需要特征匹配：用上一帧 3D 点的光度误差直接估计位姿，无描述子计算/暴力搜索
- 1D 极线对齐使用逆组合（inverse compositional）方法：Hessian 只算一次在参考块上
- 原生多相机支持：CameraBundle + FrameBundle 架构天然处理多相机，每个相机独立外参和 Reprojector
- Ceres 后端实现完整 VIO：滑动窗口 BA + IMU 预积分 + 边缘化先验 + Map Alignment

## 技术要点
| 维度 | 方案 |
|------|------|
| 范式 | 半直接法 VIO（直接法前端 + 间接法后端） |
| 前端 | SparseImgAlign 光度误差 + align1D 极线搜索 |
| 后端 | Ceres 滑动窗口 BA + 重投影误差 + IMU 预积分 |
| 独特创新 | 半直接混合流水线、Bayesian 深度滤波器、原生多相机 |

## 关键引用
- Sparse Image Alignment：`svo_img_align/src/sparse_img_align.cpp:34-113`
- 1D 极线对齐：`svo_direct/src/feature_alignment.cpp:31-200`
- Pose-only BA：`svo/src/pose_optimizer.cpp:40-94`
- 深度滤波器：`svo_direct/src/depth_filter.cpp:1-691`
- IMU 预积分：`svo/src/imu_handler.cpp:51-68`
- Ceres 后端：`svo_ceres_backend/include/svo/ceres_backend/estimator.hpp:159-770`

## 相关页面
- [[算法-DSO]]
- [[2026-04-29-orb-slam3-analysis|ORB-SLAM3方案分析]]
- [[VIO方案对比]]