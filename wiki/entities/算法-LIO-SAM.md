---
tags: [LiDAR, IMU, 因子图, 激光SLAM, SAM]
sources:
  - wiki/sources/2026-04-28-cartographer-analysis.md
  - wiki/sources/2026-04-28-kiss-icp-analysis.md
  - wiki/sources/2026-04-28-lego-loam-analysis.md
  - wiki/sources/2026-04-28-pin-slam-analysis.md
  - wiki/sources/2026-04-29-cartographer_analysis.md
  - wiki/sources/2026-04-29-lio_sam-analysis.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# LIO-SAM

> LIO-SAM（LiDAR-inertial Smoothing and Mapping）是基于因子图的紧耦合LiDAR-惯性SLAM系统，使用平滑与建图范式（SAM），以iSAM2增量推理实现实时高精度位姿估计和全局一致性地图。

## 核心方法
LIO-SAM将SLAM问题构建为因子图，因子包括：IMU预积分因子（帧间运动约束）、LiDAR里程计因子（点到局部地图配准相对位姿）、GPS因子（当GPS可用时的绝对位置约束）和回环因子（基于欧氏距离的关键帧配准）。后端使用GTSAM的iSAM2增量优化器，每次新增因子后高效更新因子图，无需每帧完整BA。前端LiDAR配准使用基于曲率的特征提取（平面点和边缘点）。

## 关键设计
- 因子图架构：所有传感器约束统一为GTSAM因子，iSAM2增量求解
- IMU-LiDAR紧耦合：IMU去畸变+预积分因子，配准初值由IMU提供
- 可选GPS融合：GPS因子注入因子图实现全局定位
- 基于欧氏距离的回环：使用Scan-to-Map配准验证空间邻近的回环候选

## 相关页面
- [[2026-04-28-lio-sam-analysis]]
- [[LiDAR方案对比]]
- [[概念-因子图]]
- [[组件-GTSAM]]
---
## (合并自: LIO-SAM分析摘要.md)
---
---
tags: [LIO-SAM, 分析摘要, LiDAR, IMU, 因子图]
sources: [wiki/sources/2026-04-28-lio-sam-analysis.md]
created: 2026-04-28
type: entity
---

# LIO-SAM分析摘要

> LIO-SAM 将 IMU 预积分 + ISAM2 因子图引入 LiDAR SLAM，建立两级优化的工程标杆。

## 核心方法

LIO-SAM 前端使用曲率特征提取（6子区域，edge/surf 各 20 个。**边缘特征（edge）**=局部曲率大的点，通常对应墙角、树干等物体边界；**平面特征（surf）**=局部曲率小的点，对应地面、墙面等平面区域。），通过帧-地图 LM 迭代 30 次优化位姿。后端使用 GTSAM ISAM2 增量优化因子图，融合 IMU 预积分因子、LiDAR 里程计因子（以 PriorFactor 注入）、GPS 因子和回环因子。

## 关键设计

- 松耦合架构：LiDAR 前端优化结果以 PriorFactor 形式注入 IMU 预积分图
- 退化检测：通过 J^T·J 特征值分解检测退化维度，增大噪声并投影修正
- IMU deskew：角速度积分将点云校正到帧起始时刻
- 双线程预积分器：优化用 + 高频发布用分离
- 回环：PCL ICP 半径搜索 + 时间差 >30s 过滤 + fitnessScore < 0.3 阈值
- ISAM2 重置：每 100 帧重置并保留边缘协方差

## 相关页面

- [[算法-LIO-SAM]]
- [[LiDAR方案对比]]
- [[因子图vs滤波]]