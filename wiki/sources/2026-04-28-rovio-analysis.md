---
tags: [VIO, EKF, 直接法, 图像块]
sources:
  - raw/docs-deep-dive/rovio_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/rovio_analysis.md
---

# ROVIO 源码级分析摘要

> 以 EKF 滤波框架直接操作原始图像块（patch），完全摒弃特征点提取与匹配流程，证明了在 VIO 场景中描述子匹配不是必需的。

## 核心发现
- **无特征点/无描述子**：直接将图像块作为"特征"，通过光度误差最小化在 EKF 中同时估计位姿和路标
- 路标用 robotcentric 参数化：bearing vector（2D）+ depth（1D），雅可比更稀疏、数值更稳定
- IEKF 迭代重线性化缓解 EKF 线性化误差，候选状态在像素空间沿协方差特征向量方向采样
- IMU 运动学传播 warp 矩阵到大运动下的 patch，使直接法在大运动场景也能收敛
- 支持 4 种深度参数化（常规/逆深度/对数/双曲），以及在线外参标定

## 技术要点
| 维度 | 方案 |
|------|------|
| 范式 | 滤波式直接法 VIO（EKF/IEKF） |
| 前端 | 图像块 warp + 多层级光度对齐方程（降阶雅可比 2x2） |
| 后端 | EKF 更新 + IMU 传播（无全局优化） |
| 独特创新 | Patch-based 观测、robotcentric 路标参数化、IEKF 候选生成 |

## 关键引用
- Patch 数据结构与提取：`include/rovio/Patch.hpp:41-305`
- EKF 状态定义：`include/rovio/FilterStates.hpp:110-444`
- 图像更新（光度误差+IEKF）：`include/rovio/ImgUpdate.hpp:159-1121`
- IMU 传播与 robotcentric 运动学：`include/rovio/ImuPrediction.hpp:39-291`
- 深度参数化：`include/rovio/FeatureDistance.hpp:36-247`

## 相关页面
- [[2026-04-29-msckf-vio-analysis-analysis|MSCKF方案分析]]
- [[VIO方案对比]]
- [[概念-直接法vs间接法]]