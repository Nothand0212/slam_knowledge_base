---
tags: [特征提取, 协方差, 平面检测, genz-icp]
sources:
  - wiki/sources/2026-04-29-genz_icp_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# Planar/Non-planar分类

> GenZ-ICP 的在线几何分类：通过 27 邻域点协方差特征值分解判断 planar 或 non-planar。

## 核心用途

该分类用于为不同几何区域选择不同 ICP 残差。局部点云近似平面时，点到平面残差能提供更强的法向约束；局部结构不成平面时，强行使用平面法向会引入错误约束，因此退回点到点残差更稳。

## 判定方法
1. 对每源点在 voxel hash map 的 27 邻域中收集邻居点
2. 计算点云协方差矩阵 → 特征值分解
3. `λ3/(λ1+λ2+λ3) < planarity_threshold` → 判定为 planar（λ3 是最小特征值）

## 对应关系类型
- **Planar**：提供 (source, target, normal) 三元组，用于点到平面残差
- **Non-planar**：仅提供 (source, target) 二元组，用于点到点残差
- 最小邻居数 ≥ 5 才能可靠判定平面度

## 风险点

协方差特征值受邻域半径、体素大小和点数影响很大。邻域过小会让噪声主导法向估计，邻域过大又会把边缘、墙角和曲面混成伪平面。工程上应把分类结果当作残差权重或模式选择依据，而不是绝对语义标签。

## 相关页面

- [[方法-genz-icp]]
- [[方法-自适应平面度权重]]
- [[方法-VoxelHashMap]]
- [[方法-ICP配准方法]]
