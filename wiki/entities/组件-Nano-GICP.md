---
tags: [组件, 点云配准, SLAM]
type: entity
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
sources:
  - wiki/sources/2026-04-29-fast_lio_sam-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# Nano-GICP

> FAST-LIO-SAM-SC-QN 回环验证中的精配准组件，用 NanoFLANN 和 FastGICP 做快速分布到分布匹配。

## 定义

Nano-GICP 是轻量快速的 GICP 实现，结合 NanoFLANN KD-tree 和 FastGICP 的分布到分布匹配。它会计算 source/target 点云的局部分布协方差，再用 GICP 代价做精配准。

## 参数与定位

- `max_iter = 32`
- `icp_score_threshold = 1.5`
- `correspondences_number = 15`

在 FAST-LIO-SAM-SC-QN 中，它位于四阶段回环验证的第四阶段：Quatro 给出粗对齐后，Nano-GICP 负责精细对齐和 fitness 检查。

## 与 Quatro 的互补

| 阶段 | 方法 | 作用 |
|------|------|------|
| 粗对齐 | [[方法-Quatro配准]] | 处理大初始误差和外点 |
| 精配准 | Nano-GICP | 在较好初值附近优化局部几何一致性 |

这类组合比“单次 ICP 验证回环”更稳，因为粗对齐和精配准承担不同失败模式。

## 相关页面

- [[方法-Quatro配准]]
- [[方法-四阶段回环验证]]
- [[方法-GICP配准方法]]
- [[方法-ICP配准方法]]
- [[概念-回环检测方法]]
- [[2026-04-29-external-primary-source-check]]
