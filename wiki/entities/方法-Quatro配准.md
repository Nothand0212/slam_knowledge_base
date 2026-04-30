---
tags: [方法, 点云配准, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-fast_lio_sam-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
---

# Quatro配准

> 一种面向回环粗对齐的全局点云配准路线，结合 FPFH 特征和 TEASER++/GNC 鲁棒求解，降低对初值的依赖。

## 定义

Quatro 配准在 FAST-LIO-SAM-SC-QN 中用于回环候选的粗几何对齐。它先提取 FPFH 局部几何描述子，再用 TEASER++/GNC 在大量外点下求解刚体变换。和 ICP 不同，它不要求初值已经很接近正确位姿。

## 参数与流程

- 法向估计半径：`fpfh_normal_radius = 0.9`
- FPFH 描述半径：`fpfh_radius = 1.5`
- 噪声界：`noise_bound = 0.3`

流程：

1. 对回环候选点云降采样。
2. 估计法向并计算 FPFH 描述子。
3. 建立描述子匹配。
4. 用 TEASER++/GNC 鲁棒估计 SE(3)。
5. 将结果交给 Nano-GICP 或其他局部 ICP 精配准。

## 工程定位

Quatro 适合做回环验证中的粗对齐，不适合作为每帧实时里程计前端。它比 ICP 更能处理大初始误差，但计算成本和特征匹配复杂度也更高。

## 相关页面

- [[组件-Nano-GICP]]
- [[方法-四阶段回环验证]]
- [[方法-ICP配准方法]]
- [[方法-GICP配准方法]]
- [[概念-回环检测方法]]
- [[2026-04-29-external-primary-source-check]]
