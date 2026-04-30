---
tags: [LiDAR, IMU, 激光SLAM, LOAM, 地面分割]
sources:
  - wiki/sources/2026-04-29-lego_loam-analysis.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# LeGO-LOAM

> LeGO-LOAM（Lightweight and Ground-Optimized LOAM）在LOAM基础上引入地面分割和基于聚类的特征提取，降低计算量，专为无人地面车辆设计。

## 核心方法
LeGO-LOAM对每帧LiDAR点云先做地面分割（使用RANSAC拟合地平面），将点分为地面点和非地面点。对两类点分别提取特征：地面点提取平面特征用于z/roll/pitch估计，非地面点用图像式分割（将LiDAR投影为距离图像做连通域标注）提取边缘特征。先对地面特征做两阶段ICP（z方向先约束，再6-DoF），结果作为非地面边缘特征配准的初值。

## 关键设计
- 地面分割：RANSAC地平面拟合，地面/非地面两点集分开处理
- 两阶段优化：地面特征先求解z/roll/pitch，再做全6-DoF
- 图像式聚类：将LiDAR点云投影为2D图像，用连通域分割剔除离群
- 轻量级：相比LOAM减少约50%特征点

## 与 LOAM 的差异

LOAM 更通用，但对地面车辆场景没有显式利用“地面占主导”这一先验。LeGO-LOAM 用地面分割先稳定高度、roll 和 pitch，再处理非地面边缘，使地面机器人在结构化道路环境中更快、更稳。这个先验也限制了它：坡道、非平整越野地形和多层道路会削弱地面模型。

## 工程边界

LeGO-LOAM 适合低算力无人地面车辆和规则道路场景。若 LiDAR 线数太低、地面不可见或车辆剧烈颠簸，地面分割和图像式聚类都会受影响。现代 LIO 系统通常会用 IMU deskew 和滤波/因子图替代部分 LOAM 风格前端。

## 相关页面
- [[2026-04-28-lego-loam-analysis]]
- [[LiDAR方案对比]]
- [[方法-ICP配准方法]], [[方法-体素地图]]
