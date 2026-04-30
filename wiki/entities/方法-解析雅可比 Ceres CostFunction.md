---
type: entity
tags: [Ceres, 解析雅可比, ICP, 性能优化]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-superodom-analysis.md
---

# 解析雅可比 Ceres CostFunction

> SuperOdom 手写点到线/点到面残差的 Ceres 解析雅可比，用代码复杂度换取 LiDAR 优化速度。

## 定义

SuperOdom 中用手写解析雅可比替代 Ceres 自动微分的 ICP 残差函数实现，通过精确推导点到线和点到面距离的解析雅可比矩阵获得 2-3 倍加速。

## 核心特征

- **EdgeAnalyticCostFunction**：3 维残差，点到线距离 e = |(lp-p_a)×(lp-p_b)| / |p_a-p_b|
- **SurfNormAnalyticCostFunction**：1 维残差，点到面距离 e = plane_norm · p_w + (-OA · n)
- 7 参数块 (xyz + quaternion)，使用 PoseLocalParameterization 做 SE(3) 流形优化
- 配合 TukeyLoss + ScaledLoss 实现基于匹配质量因子的自适应加权
- 优于自动微分：无模板展开开销，代码量可控，调试友好

## 相关页面

- 实现于：[[2026-04-28-superodom-analysis|SuperOdom]] `lidarOptimization.cpp:7-80`
- 对比：gtsam_points 使用 GTSAM 自带解析雅可比，IC-GVINS 使用 Ceres 自动微分
- [[组件-Ceres-Solver]]
- [[方法-ICP配准方法]]
- [[数学-流形优化]]
