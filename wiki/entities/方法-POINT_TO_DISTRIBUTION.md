---
tags: [方法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-ct_icp-analysis.md
---

# POINT_TO_DISTRIBUTION

> CT-ICP 中的配准代价类型：把点到局部分布的误差写成马氏距离，用邻域协方差表达局部几何不确定性。

## 定义

`POINT_TO_DISTRIBUTION` 是 CT-ICP 的四种残差类型之一，另外三种是：

- `POINT_TO_POINT`：点到点欧氏距离。
- `POINT_TO_PLANE`：点到局部平面的法向距离。
- `POINT_TO_LINE`：点到局部线结构的距离。
- `POINT_TO_DISTRIBUTION`：点到邻域高斯分布的马氏距离。

它的思想接近 GICP：局部邻域不是被压缩成单个点或单个法向，而是用协方差矩阵描述几何不确定性。

## 代价形式

设变换后的 source 点为 `p`，target 邻域均值为 `μ`，邻域协方差为 `C`。为避免协方差奇异，工程中会加入正则项：

```text
Λ = (C + εI)^(-1)
```

其中 `ε = 0.05`。对应的马氏距离代价为：

```text
cost = w (p - μ)^T Λ (p - μ)
```

这里 `cost` 是标量二次型，不应理解为单一法向方向的一维残差。和 `POINT_TO_PLANE` 相比，它保留完整协方差信息；和普通 `POINT_TO_POINT` 相比，它会沿着局部高不确定方向降低惩罚。

## 工程意义

- 在平面区域，协方差会体现“沿平面方向不确定、法向方向确定”的结构，行为接近点到面。
- 在角点或非结构化区域，完整协方差比单一法向更稳。
- 正则项 `εI` 是必要的，因为小邻域点数不足或几何退化会让 `C` 奇异。

## 相关页面

- [[算法-CT-ICP]]
- [[方法-GICP配准方法]]
- [[方法-ICP配准方法]]
- [[方法-GaussianVoxelMap 体素化配准]]
