---
tags: [LiDAR, ICP, 极简]
sources:
  - raw/docs-deep-dive/kiss_icp_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/kiss_icp_analysis.md
---

# KISS-ICP 源码级分析摘要

> 500 行核心 C++ 证明：全点云 Point-to-Point ICP + Geman-McClure 核 + 自适应阈值，可匹敌复杂特征方法

## 核心发现
- **不做特征提取**：全部点参与 ICP，Geman-McClure 核自动实现隐式特征选择（高残差点被软截断）
- **无需 IMU/GPS**：deskew 用纯 LiDAR 恒速模型（`last_pose_ * last_delta_`），校正到帧**末尾**时刻
- `AdaptiveThreshold` 从运动模型预测偏差中自动学习 sigma：`3*sigma` 作对应截断半径，`sigma` 作 GM 核 kernel_scale
- 双层体素降采样范式：dense（0.5× voxel）用于建图，sparse（1.5× voxel）用于 ICP 配准

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | Point-to-Point Gauss-Newton ICP（≤500 迭代），Jacobian 用 Left-perturbative on SE(3)，TBB 并行 data association + parallel_reduce Hessian |
| 后端 | 无（纯前端 odometry） |
| 回环 | 无（设计取舍：目标为非回环 odometry benchmark） |
| 独特创新 | (1) Geman-McClure 鲁棒核：`w = sigma²/(sigma+r²)²`，比 Huber 更激进拒绝大残差 (2) 自适应阈值从模型偏差中自动学习，**不需要调参** (3) VoxelHashMap 27 邻接 voxel 最近邻替代 KD-tree (4) 地图维护半径 100m 滑动窗口，自动剔除远点 |

## 关键引用 (path:line)
- 双层体素降采样: `KissICP.cpp:70-74`
- 恒速模型初始猜测: `KissICP.cpp:47`
- 恒速模型 deskew: `Preprocessing.cpp:68-79`
- 自适应阈值更新: `Threshold.cpp:38-49`
- GM 核权重的 Gauss-Newton 迭代: `Registration.cpp:138-167`
- Left-perturbative Jacobian: `Registration.cpp:84-87`
- GM 权重函数: `Registration.cpp:96-98`
- TBB 并行数据关联: `Registration.cpp:60-78`
- VoxelHashMap 27 邻居搜索: `VoxelHashMap.cpp:46-70`
- 地图远点剔除: `VoxelHashMap.cpp:121-131`

## 相关页面
- [[LiDAR方案对比]]
- [[算法-LIO-SAM]]