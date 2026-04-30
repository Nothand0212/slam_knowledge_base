---
type: entity
tags: [退化检测, LiDAR, 可观测性, 位姿估计, SuperLoc]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-rolo-analysis.md
  - wiki/sources/2026-04-28-superodom-analysis.md
---

# 6-DoF 退化检测

> 分别评估位姿 6 个自由度的可观测性，在几何退化方向上降低更新或注入先验，避免 LiDAR 配准把不可观测运动估计成确定结果。

## 定义

SuperOdom/SuperLoc 中实现的对 LiDAR 扫描匹配的 6 个自由度分别进行可观测性评估和退化判定，在退化方向上注入先验约束防止位姿突变的机制。

## 为什么需要

LiDAR 配准不是所有方向都同样可观。长走廊、平面墙、开阔地或越野地形会让某些平移/旋转方向缺少几何约束。如果仍按完整 6-DoF 更新，优化器可能在退化方向产生看似收敛但实际错误的增量。

## 核心特征

- **SuperOdom**：基于特征可观测性直方图 PlaneFeatureHistogramObs[9] 统计，输出 uncertainty_x/y/z/roll/pitch/yaw 6 维不确定性
- **ROLO**：LM 迭代中海森矩阵特征值 < 100 时启用退化投影 matP = V⁻¹·V'，限制退化方向更新
- 退化响应：注入 SE3AbsolutatePoseFactor（先验约束）限制退化自由度
- 通过 ROS topic 实时发布各方向不确定性，可用作下游规划器的可信度输入
- 比简单拒绝匹配更精细：部分退化时仍可利用非退化方向信息

## 处理策略

| 策略 | 做法 | 适用 |
|------|------|------|
| 投影修正 | 对 Hessian 特征值过小的方向抑制增量 | LIO-SAM/ROLO 类局部优化 |
| 噪声放大 | 退化时增大观测噪声 | 因子图或滤波器后端 |
| 先验注入 | 对退化自由度加入弱先验 | SuperOdom/SuperLoc |
| 多源补充 | 用 IMU、GNSS、视觉或轮速补足退化方向 | 多传感器融合系统 |

## 相关页面

- 实现：[[2026-04-28-superodom-analysis|SuperOdom]] `LidarSlam.cpp:921-992`、[[算法-ROLO-SLAM]] `backMapping.cpp:844-866`
- [[方法-多源位姿预测策略]]
- [[方法-退化检测与修复]]
- [[算法-LIO-SAM]]
- [[LiDAR数据管线]]
