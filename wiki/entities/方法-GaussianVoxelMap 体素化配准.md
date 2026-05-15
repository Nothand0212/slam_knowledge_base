---
type: entity
tags: [点云配准, 体素化, GICP, 最近邻搜索]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-LiDAR地图表示]]
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
---

> 本页内容已归并至 [[方法-LiDAR地图表示]]。

# GaussianVoxelMap 体素化配准

> 将目标点云压缩为体素高斯分布，用哈希体素索引替代逐点 KNN，并用马氏距离表达 GICP/VGICP 风格的分布匹配。

## 定义

将目标点云预先体素化为高斯分布集合（GaussianVoxelMap），在每个体素内存储点云的均值 μ 和协方差矩阵 C，实现 O(1) 数据关联的扫描配准方法。

## 残差模型

对 source 点 `p_i` 和 target 体素高斯 `(μ_j, C_j)`，变换后点为：

```text
q_i = T p_i
```

若 source 点自身也带协方差 `C_i`，旋转传播后的联合协方差可近似写成：

```text
Σ_ij = C_j + R C_i R^T
```

对应的马氏距离代价是：

```text
r_ij^2 = (q_i - μ_j)^T Σ_ij^(-1) (q_i - μ_j)
```

这里的 `Σ_ij^(-1)` 是信息矩阵。原来的简写 `||T p_i - μ_j||_{C_j + T C_i T^T}^{-1}` 容易误读；更准确的写法是“用联合协方差的逆来加权平方距离”。

## 核心特征

- 数据关联从 O(logN) KNN 降为 O(1) 体素索引
- 每个 source 点可与多个体素建立多分布对应（增加鲁棒性）
- 残差使用联合协方差的马氏距离，而不是普通欧氏距离
- CPU/GPU 双实现，GPU 版通过 NonlinearFactorGPU 批量线性化
- 支持 `overlap()` 计算帧间重叠率用于关键帧判定
- VmfVoxelMap 变体使用去均值高斯体素，适合 ROLO 的 RotVGICP

## 工程边界

- 体素越大，数据关联越快但几何细节越粗；体素越小，退化为接近 KNN 的稀疏结构。
- 协方差估计需要足够体素内点数，否则应加正则项或回退到点到点/点到面约束。
- GPU 线性化能提升吞吐，但要求帧抽象、体素存储和因子接口都支持批处理。

## 相关页面

- 实现于：[[组件-gtsam_points|IntegratedVGICPFactor]]（gtsam_points）
- [[方法-体素地图]]
- [[方法-RotVGICP]]
- [[算法-ROLO-SLAM]]
- [[方法-GICP配准方法]]
- 参考：VGICP 论文 [Koide, ICRA2021/RA-L2021]
