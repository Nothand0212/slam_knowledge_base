---
tags: [VIO,GPS 全局融合松耦合方式，GeographicLib 做局部 ENU 坐标转换]
sources:
  - raw/docs-deep-dive/vins_fusion_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/vins_fusion_analysis.md
---

# VINS-Fusion 源码级分析摘要

> VINS-Fusion 全流水线源码级深度分析

## 核心发现

- 前端使用 Shi-Tomasi 角点（goodFeaturesToTrack）+ KLT 金字塔光流（calcOpticalFlowPyrLK），无描述子匹配；视觉预测机制利用 Estimator 传入预测 3D 点做 LK 初始猜测，显著加速跟踪
- 初始化分三种模式：Mono+IMU（SFM + IMU-visual 对齐两步法）、Stereo+IMU（PnP + 陀螺偏置估计）、Stereo only
- 滑动窗口优化使用 Ceres DENSE_SCHUR + DogLeg 策略，残差包括 IMU 预积分 + 视觉重投影 + 边缘化先验 + 立体约束
- IMU 预积分使用中值积分（mid-point，RK2），含完整 Jacobian/协方差传播和偏置变化时重传播机制
- Schur 补边缘化保留历史信息，MARGIN_OLD / MARGIN_SECOND_NEW 两种策略依据视差和跟踪质量选择
- 回环检测使用 DBoW2 + BRIEF 描述子词袋，几何验证用 PnP RANSAC，位姿图用 4-DOF 优化

## 关键代码引用

- [VIO,视觉惯性里程计,滑动窗口,因子图,回环检测]

## 相关页面

FeatureTracker::trackImage (feature_tracker.cpp), IntegrationBase::midPointIntegration (integration_base.h), Estimator::optimization (estimator.cpp), MarginalizationInfo::marginalize|[[2026-04-29-vins-fusion-analysis-analysis|VINS-Fusion]] [[方法-视觉特征跟踪|KLT光流]] [[方法-Shi-Tomasi角点]] [[组件-Ceres-Solver]] [[概念-IMU预积分]] [[概念-IMU预积分|IMU中值积分]]
