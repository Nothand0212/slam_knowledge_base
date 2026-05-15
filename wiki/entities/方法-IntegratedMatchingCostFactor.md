---
type: entity
tags: [GTSAM, 因子设计, 扫描配准, 设计模式]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-GTSAM-Ceres工程因子]]
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
---

> 本页内容已归并至 [[方法-GTSAM-Ceres工程因子]]。

# IntegratedMatchingCostFactor

> gtsam_points 的扫描配准因子基类，把数据关联更新、残差/Hessian 计算和 GTSAM 线性化流程统一封装。

## 定义

gtsam_points 中所有扫描配准因子的抽象基类，继承自 `gtsam::NonlinearFactor`。标准化了基于点云匹配的 SE(3) 约束因子的接口设计。

## 核心特征

- 残差维度固定为 6（SE(3) 扰动维度）
- 定义两个纯虚函数：`update_correspondences()` 建立数据关联，`evaluate()` 计算残差和 Hessian
- 支持二元因子（target/source 两个 key）和一元因子（target 固定时）
- error() 和 linearize() 按统一流程：提取位姿 → 计算 Δ → 更新对应 → 计算残差/Hessian
- 新增加因子仅需实现两个纯虚函数 ≈ 200 行代码

## 相关页面

- 被继承：IntegratedICPFactor, IntegratedGICPFactor, IntegratedVGICPFactor, IntegratedLOAMFactor
- [[概念-因子图]]
- [[组件-gtsam_points]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[方法-连续时间 ICP 因子]]
- 参考：gtsam_points `factors/integrated_matching_cost_factor.hpp:19`
