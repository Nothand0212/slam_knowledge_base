---
tags: [LiDAR, IMU, IESKF, LIO, 高翔]
sources:
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# lightning-lm

> 高翔自研 LiDAR-Inertial Odometry 系统：IESKF + IVox3d 哈希地图 + miao 增量 PGO，多雷达类型适配。

## 系统定位

lightning-lm 是一个工程化 LIO/建图系统，前端负责高频 LiDAR-IMU 里程计，后端负责回环和位姿图修正。它的价值在于把 Faster-LIO 风格的滤波前端、IVox3d 局部地图和自研 PGO 后端组合成可适配多种 LiDAR 的完整方案。

## 核心架构
- **前端 AA-FasterLIO**：23 维 IESKF（含外参 + S2 重力），在线平面拟合代替 edge/planar 分类
- **后端**：miao 库增量 PGO，多分辨率 NDT 回环检测，高度约束防止 Z 轴漂移
- **多雷达支持**：Livox Avia / Velodyne 32 / Ouster 64 / RoboSense 四种类型

## 关键创新
- IVox3d 哈希 voxel 地图（100 万 grids 容量，LRU 淘汰，18 邻域）
- 在线平面拟合：`n^T·p=-1` QR 求解，验证所有点平面性
- 无 IMU 预积分，使用标准递推积分
- Cauchy 鲁棒核 + chi² 外点剔除两步外点处理

## 适用边界

系统仍以 LiDAR 几何为主，对长走廊、开阔地和重复结构需要依赖退化检测与后端回环修正。多雷达适配降低了部署门槛，但不同 LiDAR 的扫描模式、时间戳和畸变模型不同，不能只改 topic 名称就认为等价。

## 相关页面

- [[方法-IESKF滤波器]]
- [[组件-IVox3d]]
- [[方法-在线平面拟合]]
- [[方法-miao PGO]]
- [[LiDAR方案对比]]
