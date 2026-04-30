---
tags: [source, 源码分析, SLAM, kimera-vio]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/kimera_vio_analysis.md
sources:
  - raw/docs-deep-dive/kimera_vio_analysis.md
---
# Kimera-VIO 深度分析

**日期**: 2026-04-29 | **语言**: C++17 | **依赖**: GTSAM, OpenCV, OpenGV, DBoW2, KimeraRPGO

## 概要
MIT-SPARK 的 Stereo+IMU 紧耦合 VIO 系统。前端 GFTT 特征 → KLT 光流 → OpenGV RANSAC 三层几何验证；后端 [[组件-GTSAM|GTSAM iSAM2]] 增量 FixedLagSmoother。视觉因子使用 [[方法-SmartStereoFactor]]（GTSAM Schur 消去路标变量）。支持在线 IMU-Camera 时间对齐、[[方法-OnlineGravityAlignment]] 初始化、Delaunay Mesh 重建。

## 核心概念
- [[算法-Kimera-VIO]] — Pipeline Module 多线程架构
- [[方法-SmartStereoFactor]] — GTSAM SmartStereoProjectionPoseFactor
- [[方法-OnlineGravityAlignment]] — 陀螺偏置+重力切空间细化
- [[组件-GTSAM|GTSAM iSAM2]] — Batch/IncrementalFixedLagSmoother

## 管线要点
- 双目矫正 → GFTT(Shi-Tomasi) → ANMS → NCC 稀疏立体匹配
- IMU 预积分 → SmartStereoFactor → BetweenFactor → FixedLagSmoother
- Delaunay 三角化 → 3D Mesh → 平面分割 → PointPlaneFactor 正则

## 工程参考 (phad_fusion)
- PipelineModule SISO/SIMO 纯函数式接口
- SmartStereoFactor 增量添加观测的设计模式
- 避坑: feature_tracks_ 无限增长需窗口裁剪

## 相关
- [[VIO方案对比|VIO对比分析]]
