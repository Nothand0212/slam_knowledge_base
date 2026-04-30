---
tags: [LiDAR, 激光SLAM, ICP, 点云, 去畸变, NDT, 地图表示]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-comparison/lidar_pipeline_comparison.md
sources:
  - raw/docs-comparison/lidar_pipeline_comparison.md
---

# LiDAR 数据管线横向对比

> 基于 15 个 LiDAR SLAM 项目的深度分析，横跨经典、现代、多传感器、神经与工程五大类别

## 对比范围

| 类别 | 项目 | 方法 |
|------|------|------|
| 经典 | LeGO-LOAM, LIO-SAM, Cartographer | 特征式 LM / 概率网格 |
| 现代 | FAST-LIO-SAM, fusions_slam, KISS-ICP, CT-ICP, ROLO | IESKF / ICP / CT |
| 多传感器 | R3LIVE, FAST-LIVO2, LVI-SAM | 紧/松耦合融合 |
| 神经 | PIN-SLAM, BEV-LSLAM | SDF / BEV投影 |
| 工程 | Lightning-LM, lt-mapper | 长周期/多会话 |

## 运动畸变矫正 (Deskewing)

这是 LiDAR 管线最重要的步骤：

| 方法 | 代表 | 精度 |
|------|------|------|
| CT 连续时间 | CT-ICP | 最高 |
| IMU 反向传播 | FAST-LIO2, FAST-LIVO2 | 很高 |
| IMU 角速度积分 | LIO-SAM, LVI-SAM | 中高 |
| 恒速模型 | KISS-ICP | 中 |
| IMU 旋转补偿 | LeGO-LOAM | 中 |

## 特征提取趋势

- 传统：LOAM 曲率 + edge/planar 分类
- 现代：在线平面拟合、GM 核自适应权重
- 未来：神经隐式表示（SDF, Neural Points）

## 地图表示演化

```
传统离散 → 结构化稀疏 → 连续/神经
Point Cloud → VoxelHashMap → SDF/Neural Points
KD-Tree → VoxelMap/iKD-Tree
```

## 相关页面

- [[LiDAR方案对比]]
- [[算法-KISS-ICP]]
- [[方法-genz-icp]]
- [[算法-FAST-LIO]]
- [[方法-体素地图]]
- [[概念-连续时间轨迹]]
- [[算法-CT-ICP]]