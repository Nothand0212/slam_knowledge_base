---
tags: [LiDAR, 神经隐式, 深度学习, 可微]
sources:
  - raw/docs-deep-dive/pin_slam_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/pin_slam_analysis.md
---

# PIN-SLAM 深度源码分析

> IPB Bonn 神经隐式 LiDAR SLAM：Neural Points + MLP 表示连续 SDF 地图，可微 tracking+mapping+回环

## 摘要
PIN-SLAM 用点基隐式神经表示替代传统栅格/点云地图：每个 neural point 携带 8 维特征向量，MLP 解码器输出连续 SDF 值。独创性在于可微框架统一 tracking/mapping/回环，以及基于 neural point 的 Scan Context 回环描述子。

## 关键概念
- **[[算法-PIN-SLAM]]**：纯 LiDAR 神经隐式 SLAM，完整闭环（tracking+mapping+回环+PGO）
- **[[方法-点基隐式神经表示]]**：3D 位置 + 8 维特征 + 四元数方向的 neural points，空间哈希索引
- **[[方法-SDF解码器]]**：极轻量单层 MLP (11→64→1)，从特征解码 SDF + 分析梯度
- **[[方法-Neural Point Map Context]]**：基于 neural points 的 Scan Context 回环描述子，ring key 快速检索
- **[[概念-可微地图]]**：BCE/MAE 损失梯度流回 neural points 特征和 MLP 参数，在线优化

## 相关页面
- [[算法-NICE-SLAM]]
- [[概念-深度学习SLAM]]
- [[方法-体素地图]]
- [[概念-回环检测方法]]