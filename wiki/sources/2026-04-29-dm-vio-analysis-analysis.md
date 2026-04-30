---
tags: [VIO,DelayedGraph::readvanceUntilDelay, computeBAUpdate (BAGTSAMIntegration.cpp), computeDynamicDSOWeight]
sources:
  - raw/docs-deep-dive/dm_vio_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/dm_vio_analysis.md
---

# DM-VIO 源码级分析摘要

> DM-VIO 延迟边缘化视觉惯性里程计

## 核心发现

- 延迟边缘化核心创新：等待 scale/gravity/bias 收敛后再执行 Schur complement，避免错误信息锁定
- BAGTSAMIntegration 桥接 DSO 视觉 Hessian 和 GTSAM IMU 因子，Hessian 直接相加融合
- 动态 DSO 权重：光度 RMSE 上升时降低视觉权重，系统更依赖 IMU
- 多延迟图架构：初始化前用大延迟纯视觉图，成功后升级为含 IMU 因子的主图
- [直接法,VIO,延迟边缘化,GTSAM]

## 关键代码引用

- [[算法-DM-VIO]] [[概念-延迟边缘化]] [[2026-04-29-dm-vio-analysis-analysis|PoseTransformationFactor]] [[2026-04-29-dm-vio-analysis-analysis|动态DSO权重]] [[组件-GTSAM]]

## 相关页面


