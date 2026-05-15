---
type: entity
tags: [bundle-adjustment, LiDAR, 特征值最小化, 因子图优化]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-GTSAM-Ceres工程因子]]
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
---

> 本页内容已归并至 [[方法-GTSAM-Ceres工程因子]]。

# LiDAR 捆集调整因子

> gtsam_points 将多帧 LiDAR 局部几何约束写成因子图 BA 因子，用特征值最小化表达共面/共线约束。

## 定义

gtsam_points 中针对多帧 LiDAR 点云特征做联合优化的创新因子族，将 LiDAR BA 从启发式平面拟合提升为严格的 GTSAM 因子图优化问题。

## 核心特征

- **PlaneEVMFactor**：最小化协方差矩阵最小特征值 λ₀，所有点共面约束
- **EdgeEVMFactor**：最小化 λ₀ + λ₁，所有点共线约束
- **LsqBundleAdjustmentFactor**：基于 EVM + EF 最优条件，代价仅与帧数有关而与点数无关——解决了传统 BA 计算量随点数线性增长的问题
- 使用 `add(pt, key)` 接口添加特征点
- EVM（Eigenvalue Minimization）避免了启发式的平面拟合阈值

## 与传统点面约束的区别

传统 LiDAR 前端通常先局部拟合平面或边缘，再把当前点到该局部模型的距离作为残差。LiDAR BA 因子则把多帧观测共同放进一个因子中，用整体共面/共线程度约束多个位姿。它更接近视觉 BA 的思想：不是逐帧匹配局部地图，而是让跨帧几何结构共同约束状态。

## 工程边界

这类因子适合关键帧或局部窗口优化，不适合把所有原始点无筛选地加入图中。特征点选择、体素下采样和退化检测仍然重要；否则特征值最小化会被噪声、动态物体或错误关联主导。

## 相关页面

- 实现：gtsam_points `bundle_adjustment_factor.hpp`
- [[组件-gtsam_points]]
- [[概念-位姿图优化]]
- [[概念-因子图]]
- [[方法-IntegratedMatchingCostFactor]]
- 参考：EVM BA [Liu, RA-L2021], LSQ BA [Huang, RA-L2021]
