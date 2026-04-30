---
tags: [LiDAR, IMU, IESKF, 回环]
sources:
  - raw/docs-deep-dive/lightning_lm_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/lightning_lm_analysis.md
---

# Lightning-LM 深度源码分析

> 高翔自研 LIO 系统：23 维 IESKF + IVox3d 哈希地图 + miao 增量 PGO，多雷达类型适配

## 摘要
Lightning-LM 是不使用 IMU 预积分的轻量级 IESKF LiDAR 里程计，创新点在统一在线平面拟合代替 edge/planar 分类，以及基于欧氏距离 + 多分辨率 NDT 的回环检测方案。

## 关键概念
- **[[组件-lightning-lm]]**：自研 IESKF + miao PGO，23 维状态含 LiDAR 外参 + S2 重力
- **[[组件-IVox3d]]**：增量哈希 voxel 地图，容量 100 万 grids，LRU 淘汰，18 邻域搜索
- **[[方法-在线平面拟合]]**：`n^T·p=-1` 约束 QR 求解法向量，验证所有邻点到平面距离 ≤ 0.1
- **[[方法-miao PGO]]**：从 g2o 派生的 LM 增量优化库，回环用 Cauchy 核 + chi² 外点剔除
- **[[方法-多分辨率NDT回环]]**：四级分辨率 {10,5,2,1}m 的 NDT 匹配，不使用 ScanContext

## 相关页面
- [[算法-FAST-LIO]]
- [[方法-IESKF滤波器]]
- [[LiDAR方案对比]]
- [[概念-回环检测方法]]