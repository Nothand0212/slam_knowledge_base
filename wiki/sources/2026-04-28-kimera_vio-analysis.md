---
tags: [VIO, 视觉惯性里程计, 因子图, Mesh重建, GTSAM]
sources:
  - raw/docs-deep-dive/kimera_vio_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/kimera_vio_analysis.md
---

# Kimera-VIO 源码级分析摘要

> 基于 GTSAM iSAM2 的因子图 VIO 框架，独特之处在于集成了在线 Mesh 重建，采用 SmartStereoFactor 做路标高效参数化。

## 核心发现

- 前端使用 GFTT 特征（默认）+ OpenGV RANSAC（5/2/3/1-point）+ KLT 金字塔光流跟踪
- 特征分布使用 ANMS（自适应非极大抑制）替代传统网格策略，来自第三方库 ANMS-Codes
- 立体匹配支持稠密（SGBM/BM）和稀疏（极线 NCC）两种模式
- 初始化假设零速度和 upright vehicle，通过 `InitializationFromImu` 估计重力方向和 IMU bias
- 在线重力对齐模块 `OnlineGravityAlignment` 借鉴 VINS-Mono 方法：偏置估计 → 线性对齐 → 切空间重力细化
- 后端使用 GTSAM `IncrementalFixedLagSmoother`（iSAM2 增量优化），路标用 **SmartStereoProjectionFactor** 实现 Schur Complement 消元
- 回环检测使用 DBoW2 + **ORB 描述子**（不同于前端特征！），几何验证后用 KimeraRPGO 做位姿图优化
- Mesh 重建为独特功能：2D Delaunay 三角化 + VIO 路标填充 + 不良三角形过滤 + 平面分割

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | GFTT 角点（OpenCV），ANMS 均匀分布 |
| 跟踪方法 | KLT 金字塔光流 + IMU 旋转预测 + OpenGV RANSAC |
| 后端 | GTSAM iSAM2 FixedLagSmoother（因子图） |
| 回环 | DBoW2 + ORB 词袋 + KimeraRPGO 位姿图优化 |
| ROS 耦合 | 松耦合，Pipeline Module 模式 |
| 在线标定 | 时间偏移自动对齐（CrossCorrTimeAligner） |

## 关键代码引用

- 前端 Pipeline: `StereoImuPipeline` (pipeline/StereoImuPipeline.h)
- IMU 缓冲: `ThreadsafeImuBuffer` (utils/ThreadsafeImuBuffer.cpp)
- 后端优化: `VioBackend::optimize` (backend/VioBackend.cpp)
- Smart 因子: `SmartStereoProjectionPoseFactor` (backend/VioBackend.cpp:485-512)
- Mesh 重建: `Mesher::createMesh2D` (mesh/Mesher.cpp:130-153)

## 相关页面

- [[VIO方案对比]]
- [[组件-GTSAM|GTSAM因子图框架]]
- [[概念-深度学习SLAM|在线Mesh重建]]
- [[方法-SmartStereoFactor|SmartFactor与SchurComplement]]- [[概念-MSCKF]]
- [[概念-位姿图优化]]
