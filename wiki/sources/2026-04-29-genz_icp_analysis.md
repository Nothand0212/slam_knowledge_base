---
tags: [LiDAR, ICP, 配准, 自适应]
sources:
  - raw/docs-deep-dive/genz_icp_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/genz_icp_analysis.md
---

# GenZ-ICP 深度源码分析

> IEEE RA-L：自适应平面度权重 GICP，协方差判定 planar/non-planar，ROS1/ROS2 双支持

## 摘要
GenZ-ICP 是纯 LiDAR ICP 里程计，核心创新是自适应权重 α：根据场景中 planar/non-planar 点比例自动调节 point-to-plane 和 point-to-point 残差权重。架构上实现 cpp/ 算法核心与 ros/ 薄 wrapper 的教科书级分离。

## 关键概念
- **[[方法-genz-icp]]**：自适应加权 ICP，cpp+ros 分离架构，零 PCL/Ceres/GTSAM 依赖
- **[[方法-自适应平面度权重]]**：`α = planar/(planar+non-planar)`，自动权衡点面和点点配准
- **[[方法-Planar-Non-planar分类]]**：协方差特征值 `λ3/(λ1+λ2+λ3) < threshold` 判定平面度
- **[[架构-cpp+ros分离架构]]**：算法核心 cpp/ 层零 ROS 依赖 + ros/ 薄 wrapper，ROS1/ROS2 条件编译

## 相关页面
- [[算法-KISS-ICP]]
- [[方法-GICP配准方法]]
- [[方法-ICP配准方法]]
- [[LiDAR方案对比]]