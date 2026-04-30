---
tags: [VIO, 视觉惯性里程计, SchurComplement, EKF, 轻量化]
sources:
  - raw/docs-deep-dive/schurvins_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/schurvins_analysis.md
---

# SchurVINS 源码级分析摘要

> CVPR 2024 提出的基于 Schur Complement 的轻量级 VIO，将地标 Hessian 边缘化后做 EKF 更新，兼具优化精度与滤波速度。

## 核心发现

- 基于 SVO 2.0 的半直接法前端，使用 FAST 角点检测 + `OccupancyGrid2D` 网格均匀分布（grid_size=25, max_fts=180）
- 通过 `USE_SCHUR_VINS` 宏替换原 SVO 的优化后端，核心流水线为：Forward（IMU 预测）→ 重投影匹配 → Backward（EKF 视觉更新）→ 地标 EKF 优化
- EKF 状态 15 DOF（quat, pos, vel, ba, bg）+ 滑动窗口 4 个 AugState（各 6 DOF）= 总计 39 DOF
- IMU 不使用标准预积分框架，而是直接 RK4 积分 + EKF 传播（连续时间方式），偏差在线修正
- 核心创新 **Solve3()**：构建含地标的完整 Hessian（状态-状态、地标-地标、交叉项），用 Schur Complement 消去地标变量，得等效的降维 EKF 观测模型
- 地标独立 EKF（`Point::EkfUpdate`）利用历史关键帧观测对已边缘化的地标做轻量更新
- 白化残差后 Hessian=观测矩阵（`R = H`），简化 EKF 实现
- 默认关闭回环检测（DBoW2）和全局优化（iSAM2/Ceres），保持轻量化设计

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | FAST 角点（SVO），25px 网格均匀分布 |
| 跟踪方法 | 半直接法（极线搜索匹配） |
| 后端 | Schur Complement EKF（39 DOF 滑动窗口） |
| 回环 | 可选（默认关闭，runlc=false） |
| ROS 耦合 | 深度耦合（SVO ROS 接口，Topic 通信） |
| 在线标定 | 不支持（外参依赖离线标定） |

## 关键代码引用

- EKF 预测: `SchurVINS::Forward` / `Prediction` (svo/src/schur_vins.cpp:665-715, 157-211)
- Schur Complement 核心: `Solve3` (svo/src/schur_vins.cpp:213-422)
- EKF 状态更新: `StateUpdate` (svo/src/schur_vins.cpp:424-450)
- 滑动窗口管理: `Management` / `KeyframeSelect` (svo/src/schur_vins.cpp:485-523, 541-587)
- 地标 EKF: `Point::EkfUpdate` (svo_common/src/point.cpp:335-416)

## 相关页面

- [[VIO方案对比]]
- [[概念-MSCKF]]
- [[VIO方案对比|轻量级VIO设计]]
- [[算法-SVO-Pro|半直接法视觉前端]]- [[概念-MSCKF]]
- [[架构-滑动窗口优化]]
