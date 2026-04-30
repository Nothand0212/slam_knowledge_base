---
tags: [LiDAR, 对比分析, 激光SLAM, 方案选型]
sources:
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
  - wiki/sources/2026-04-29-framework-comparison.md
  - wiki/sources/2026-04-29-ct_icp-analysis.md
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
  - wiki/sources/2026-04-29-lio_sam-analysis.md
  - wiki/sources/2026-04-29-fast_lio_sam-analysis.md
  - wiki/sources/2026-04-29-fusions_slam-analysis.md
  - wiki/sources/2026-04-29-genz_icp_analysis.md
  - wiki/sources/2026-04-29-lego_loam-analysis.md
  - wiki/sources/2026-04-29-pin_slam_analysis.md
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
  - wiki/sources/2026-04-28-rolo-analysis.md
  - wiki/sources/2026-04-28-cartographer-analysis.md
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
created: 2026-04-28
updated: 2026-04-29
type: comparison
---

# LiDAR SLAM 方案对比

> 对比纯 ICP、LiDAR-IMU 紧耦合、连续时间 ICP、位姿图后端、长期建图和神经隐式 LiDAR SLAM 的工程取舍。

## 方案族总览

| 方案族 | 代表系统 | 前端匹配 | IMU 使用 | 地图表示 | 后端/全局一致性 | 适用重点 |
| ------ | -------- | -------- | -------- | -------- | --------------- | -------- |
| 极简 ICP odometry | [[算法-KISS-ICP]] | 点到点 ICP | 无 | 局部点云地图 | 无全局后端 | 快速、低依赖、纯 LiDAR baseline |
| LOAM 系 | [[算法-LeGO-LOAM]], [[算法-LIO-SAM]] | 边缘/平面特征 | 旋转去畸变或预积分 | 局部/全局点云 | GTSAM iSAM2 / PGO | 工程成熟、易理解 |
| IESKF LIO | [[算法-FAST-LIO]], [[算法-FAST-LIO-SAM-SC-QN]], [[算法-fusions_slam]], [[组件-lightning-lm\|lightning-lm]] | 点到面/体素匹配 | 高频紧耦合 | ikd-Tree、IVox、体素地图 | 可接 PGO / 回环 | 高速实时、去畸变和退化处理 |
| 连续时间 ICP | [[算法-CT-ICP]] | 连续时间轨迹下 ICP | 可选 | 局部地图 | 里程计为主 | 扫描畸变、非同步传感器 |
| 自适应 GICP | [[方法-genz-icp]], [[方法-GICP配准方法]] | 点到分布 | 可选外源 | 最近邻/协方差地图 | 无或外接后端 | 非结构化场景配准 |
| 长期/多会话 | [[算法-Cartographer]], [[算法-lt-mapper]] | scan matching / submap | 可选 | 子图、拓扑地图 | 分支定界、长期 PGO | 多会话、地图维护、回环 |
| 特殊地形 | [[算法-ROLO-SLAM]] | RotVGICP / 运动约束 | 预积分 | 点云子图 | 局部约束为主 | 越野、地面车辆、退化地形 |
| 神经隐式 LiDAR | [[算法-PIN-SLAM]] | neural points / SDF | 可选 | 神经点地图 | 自研 PGO | 稠密几何、地图压缩 |

## 关键维度

### 前端配准

| 配准方式 | 优点 | 代价 |
|----------|------|------|
| 点到点 ICP | 实现简单，依赖少 | 对初值和结构退化敏感 |
| 点到面 / LOAM 特征 | 速度快，适合结构化环境 | 特征提取阈值和地面假设敏感 |
| GICP / 点到分布 | 协方差建模更稳 | 计算量和邻域估计复杂 |
| IESKF 紧耦合 | IMU 预测、去畸变和配准统一 | 状态、噪声和雅可比实现复杂 |
| 连续时间轨迹 | 自然处理扫描畸变 | 优化变量和实现复杂度更高 |
| 神经隐式地图 | 表示连续，利于稠密几何 | GPU、学习率和地图更新策略敏感 |

### IMU 与预处理

LiDAR-IMU 系统通常更关注 IMU 去畸变、状态传播和外参，而不是飞控式的强滤波。高振动平台仍应单独审视 [[传感器-IMU预处理]]，因为预处理延迟会直接影响去畸变时间对齐。

| 系统 | IMU 角色 |
|------|----------|
| KISS-ICP / genz-icp | 可作为纯 LiDAR baseline，IMU 不是主线 |
| LIO-SAM | IMU 预积分和点云去畸变，GTSAM 因子图后端 |
| FAST-LIO / fusions_slam / lightning-lm | IESKF 高频传播，点云更新直接修正状态 |
| CT-ICP | 重点在连续时间轨迹，IMU 可作为外部先验 |
| ROLO-SLAM | 地面车辆运动约束和预积分辅助 |

## 选型建议

| 目标 | 首选参考 | 理由 |
|------|----------|------|
| 纯 LiDAR baseline | [[算法-KISS-ICP]] | 依赖少，逻辑清楚，适合做最小可用里程计 |
| 实时 LIO 前端 | [[算法-FAST-LIO]], [[算法-fusions_slam]] | IESKF 紧耦合范式成熟，速度快 |
| 因子图后端参考 | [[算法-LIO-SAM]], [[算法-FAST-LIO-SAM-SC-QN]] | GTSAM/iSAM2、GPS、回环和 PGO 入口明确 |
| 回环与全局一致性 | [[算法-Cartographer]], [[方法-ScanContext]] | 子图/scan context/分支定界提供成熟思路 |
| 非结构化或退化环境 | [[方法-GICP配准方法]], [[算法-ROLO-SLAM]] | 协方差建模、运动约束和退化检测更关键 |
| 稠密或压缩地图 | [[算法-PIN-SLAM]] | neural points/SDF 对地图表示有启发 |

## 设计结论

多传感器 SLAM 中，LiDAR 前端最好拆成三个独立层次：

- **运动补偿层**：由 IMU、连续时间模型或恒速模型提供点云去畸变。
- **局部配准层**：ICP/GICP/IESKF 输出相对位姿、协方差或残差。
- **全局约束层**：回环、GNSS、视觉或子图约束进入因子图后端。

这样可以避免把 FAST-LIO、LIO-SAM 或 Cartographer 这类完整系统整体搬进新系统，保留可替换的因子接口。

## 相关页面

- [[LiDAR数据管线]]
- [[IMU数据管线]]
- [[传感器-IMU预处理]]
- [[概念-连续时间轨迹]]
- [[方法-GICP配准方法]]
- [[方法-体素地图]]
- [[方法-ScanContext]]
- [[因子图vs滤波]]
- [[VIO方案对比]]
