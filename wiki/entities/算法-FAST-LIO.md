---
tags: [LiDAR, 激光SLAM, IESKF]
sources:
  - wiki/sources/2026-04-28-fusions-slam-analysis.md
  - wiki/sources/2026-04-28-lightning-lm-analysis.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
created: 2026-04-28
updated: 2026-04-29
type: entity
---

# FAST-LIO

> FAST-LIO 系列是基于 IESKF（Iterated Error State Kalman Filter）和 iKD-Tree 增量地图的轻量级 LiDAR 惯性里程计，以极高效率实现紧耦合 LiDAR-IMU 融合。

## 概述

FAST-LIO 由港大 Mars 实验室提出，经历了 FAST-LIO → FAST-LIO2 的演进。FAST-LIO2 是该系列的代表作，核心设计包括：

- **IESKF 后端**：迭代误差状态卡尔曼滤波，对 LiDAR 点面残差进行多次迭代更新，等价于高斯-牛顿优化，但状态维度保持在 IMU 状态（位姿+速度+偏置+重力），无需维护大量地图点。
- **iKD-Tree**：增量式 KD 树地图，支持实时点插入和删除，使点-面配准在增量地图上直接进行，无需预建全局地图。
- **后向传播（Backward Propagation）**：在一帧扫描期间利用高频 IMU 数据进行反向积分，将每个 LiDAR 点补偿到帧起始时刻，消除运动畸变。

## 扩展方案

- **FAST-LIO-SAM-SC-QN**：在 FAST-LIO2 基础上增加 Scan Context 回环检测和基于 GTSAM 的位姿图优化（PGO），实现全局一致的 LiDAR SLAM。
- **FAST-LIO-LC**：同样增加回环闭合和图优化模块。
- **fusions_slam**：将 FAST-LIO2 封装为模块化 LiDAR 里程计前端，与其他传感器（视觉、GNSS）通过因子图融合。
- **faster-lio**：基于 iVox 体素地图替换 iKD-Tree，进一步提高速度，适合大尺度场景。

## 对 SLAM 算法的意义

FAST-LIO2 是目前最成熟的低延迟 LiDAR 里程计方案之一，可作为多传感器 SLAM 系统的 LiDAR 核心前端。其 IESKF 后端的点面残差公式和 iKD-Tree 数据结构可直接复用。FAST-LIO-SAM-SC-QN 的回环检测和 PGO 管道也是多传感器 SLAM 系统全局一致性模块的重要参考。

## 相关页面

- [[方法-genz-icp]]
- [[架构-滑动窗口优化]]
- [[组件-GTSAM]]
- [[架构-坐标系管理]]