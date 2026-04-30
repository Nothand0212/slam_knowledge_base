---
tags: [LiDAR, 视觉, 松耦合, 因子图, 回环]
sources:
  - raw/docs-deep-dive/lvi_sam_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/lvi_sam_analysis.md
---

# LVI-SAM 深度源码分析

> MIT 多 ROS 节点松耦合：LIO-SAM + VINS-Mono 通过消息桥接，DBoW2 + LiDAR ICP 双重回环

## 摘要
LVI-SAM 由 5 个独立 ROS 节点组成，通过消息通信实现 LiDAR-视觉融合。VIS 给 LIS 提供初始位姿猜测，LIS 给 VIS 提供 bias/gravity 先验。唯一支持回环检测的 LIV 方案。

## 关键概念
- **[[算法-LVI-SAM]]**：5 节点 ROS 松耦合，GTSAM iSAM2 + Ceres 滑窗两个独立优化器
- **[[方法-Shi-Tomasi角点]]**：VINS-Mono 视觉前端特征提取，150 个角点 KLT 光流跟踪
- **[[组件-DBoW2]]**：词袋模型图像检索 + LiDAR ICP 精配验证的双重回环
- **[[架构-多传感器松耦合]]**：ROS 消息桥接，VIS→LIS 位姿先验，LIS→VIS bias/gravity 先验

## 相关页面
- [[算法-FAST-LIVO2]]
- [[算法-R3LIVE]]
- [[组件-DBoW2]]
- [[概念-回环检测方法]]