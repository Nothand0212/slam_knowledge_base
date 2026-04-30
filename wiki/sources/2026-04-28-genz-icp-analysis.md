---
tags: [LiDAR, 激光SLAM, genz_icp, 纯激光里程计, 自适应权重]
sources:
  - raw/docs-deep-dive/genz_icp_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/genz_icp_analysis.md
---

# GenZ-ICP 源码级分析摘要

> IEEE RA-L 发表的纯激光雷达里程计，核心创新自适应权重 α 在 point-to-plane 与 point-to-point 间自动权衡，无 IMU、无 PCL 依赖，代码极其精简。

## 核心发现

- **纯 LiDAR 里程计**，单一 PointCloud2 订阅，无 IMU、无回环检测
- **自适应权重 α**：根据当前帧 planar/non-planar 点数比自动调节，α→1 偏重点到平面（结构化场景），α→0 偏重点到点（非结构化场景）
- **协方差平面度判定**：体素内邻居特征值分解，λ3/(λ1+λ2+λ3) < threshold 判定平面，法向量为最小特征值对应特征向量
- **自适应阈值 σ**：从运动模型误差 RMSE 动态估计匹配距离（初始 2.0m），运动越大匹配容差越大
- 教科书级 **cpp/ + ros/ 分离架构**：算法核心零 ROS 依赖，ROS1/ROS2 共享同一核心代码
- 全算法约 600 行，使用 `tsl::robin_map` 哈希体素（非 PCL）、TBB 并行化、无 Ceres/GTSAM

## 技术要点

| 维度 | 方案 |
|------|------|
| 前端 | SOPHUS SE(3) 上的加权 ICP（150 次迭代） |
| 特征提取 | 协方差特征分解分类 planar/non-planar |
| 配准方法 | 自适应权重 point-to-plane + point-to-point 混合 ICP |
| IMU融合 | 无 IMU |
| 后端 | 无后端 |
| 回环 | 无回环检测 |
| ROS耦合 | ROS1/ROS2 双版本，thin wrapper 模式 |
| 地图管理 | VoxelHashMap（27 邻域搜索），双重体素化（地图细/配准粗） |

## 关键代码引用

- 核心类: `GenZICP` (cpp/genz_icp/pipeline/GenZICP.hpp:60)
- ICP 求解: `Registration::RegisterFrame()` (cpp/genz_icp/core/Registration.cpp:181)
- 自适应权重: α 计算 (cpp/genz_icp/core/Registration.cpp:202)
- 平面判定: `DeterminePlanarity()` (cpp/genz_icp/core/VoxelHashMap.cpp:105)
- 自适应阈值: `AdaptiveThreshold::ComputeThreshold()` (cpp/genz_icp/core/Threshold.cpp:39)
- 对应查找: `GetCorrespondences()` (cpp/genz_icp/core/VoxelHashMap.cpp:125)

## 相关页面

- [[LiDAR方案对比]]
- [[方法-ICP配准方法]]
- [[方法-GICP配准方法|自适应权重配准]]
- [[方法-退化检测与修复|协方差特征分解]]
- [[架构-cpp+ros分离架构|ROS算法分离设计]]- [[方法-genz-icp]]
- [[方法-GICP配准方法]]
