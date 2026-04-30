---
tags: [LiDAR, 激光SLAM, lightning_lm, 紧耦合, IESKF]
sources:
  - raw/docs-deep-dive/lightning_lm_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/lightning_lm_analysis.md
---

# Lightning-LM 源码级分析摘要

> 高性能 LiDAR-IMU 紧耦合 SLAM，自研 IVox3d 增量体素地图 + IESKF + miao PGO，单核 CPU 即可实时运行。

## 核心发现

- 前端 AA-FasterLIO 采用**在线平面拟合**策略，不区分 edge/planar 特征，在 IVox3d 查找最近 5 点后直接拟合平面
- 使用**自写 IESKF**（23 维误差状态），SO3 + S2 切线空间雅可比处理，支持外参在线估计
- **不使用 IMU 预积分**，采用标准前向递推；Anderson Acceleration 可选加速收敛
- 回环检测使用**多分辨率 Grid NDT**（4 层，10/5/2/1m），替代 ScanContext
- PGO 使用自研 **miao 优化库**（g2o 派生），支持增量模式，Cauchy 鲁棒核 + 外点剔除
- 支持 4 种雷达（Livox/Velodyne/Ouster/RoboSense），C++17 / ROS2 Humble

## 技术要点

| 维度 | 方案 |
|------|------|
| 前端 | AA-FasterLIO (scan-to-map) |
| 特征提取 | 在线平面拟合，无 edge/planar 区分 |
| 配准方法 | 自写 point-to-plane 加权最小二乘 + Cauchy 鲁棒核 |
| IMU融合 | 紧耦合 IESKF，前向递推，无预积分 |
| 后端 | 自研 miao (增量 LM + Sparse Eigen) |
| 回环 | 欧式距离候选 + 多分辨率 NDT 验证 |
| ROS耦合 | ROS2 Humble，订阅 PointCloud2 + Imu |
| 地图管理 | IVox3d 哈希体素，LRU 淘汰，容量 1M grids |

## 关键代码引用

- 核心类: `SlamSystem` (src/core/system/slam.cc:241)
- IESKF: `ESKF` (src/core/lio/eskf.hpp)
- 前端匹配: `LaserMapping::Run()` (src/core/lio/laser_mapping.cc:280)
- 平面拟合: `math::esti_plane()` (src/core/lightning_math.hpp:358)
- 点云同步: `LaserMapping::SyncPackages()` (src/core/lio/laser_mapping.cc:422)
- 回环: `LoopClosing::ComputeForCandidate()` (src/core/loop_closing/loop_closing.cc:168)

## 相关页面

- [[LiDAR方案对比]]
- [[方法-IESKF滤波器]]
- [[方法-VoxelHashMap|体素哈希地图]]
- [[方法-ICP配准方法]]
- [[概念-位姿图优化]]- [[算法-FAST-LIO]]
- [[方法-体素地图]]
