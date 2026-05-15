---
tags: [GTSAM, 点云, 配准, 因子, Python]
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
created: 2026-04-29
type: entity
updated: 2026-05-15
---

# gtsam_points

> GTSAM 点云配准扩展库，将点云配准（ICP, NDT, GICP）建模为因子图框架下的因子约束，打通 GTSAM 的因子图优化与 LiDAR SLAM 前端。

## 核心定位

gtsam_points 由 Koide Kenji 开发，是面向 LiDAR SLAM 的 GTSAM 因子扩展库。它把 ICP、GICP、VGICP、LOAM、Colored GICP、CT-ICP 等点云配准误差统一封装成 GTSAM factor，使 LiDAR 约束能和 IMU 预积分、GNSS、视觉重投影、先验约束放在同一个因子图里优化。

## 关键组件

- `IntegratedMatchingCostFactor`：配准因子的共同基类，抽象对应关系更新和残差评估。
- `GaussianVoxelMap`：用体素高斯分布加速数据关联，避免每次逐点 KNN。
- 连续时间 ICP 因子：用两个时间端点位姿插值每个点的观测时刻，内建去畸变。
- LiDAR BA 因子：把多帧点云特征约束写成类似 Bundle Adjustment 的因子图问题。
- GPU 批量线性化：通过 GPU factor 扩展提高大规模点云因子的线性化效率。

## 在 SLAM 中的应用

gtsam_points 的意义在于减少“LiDAR 前端一个优化器、全局后端另一个优化器”的割裂。使用它时，scan-to-map、连续时间配准、IMU 预积分和 GNSS 约束可以共享同一套变量、线性化点和稀疏求解器。对需要 GTSAM 生态但又想做高质量点云配准的 LIO/LVI 系统，它比手写桥接层更直接。

## 工程边界

它提供的是因子和优化组件，不是完整 SLAM pipeline。系统仍需要传感器同步、地图管理、关键帧策略、回环检测和异常约束剔除。点云因子数量大时，变量排序、体素分辨率和 GPU/CPU 线性化策略会直接影响实时性。

## 相关页面

- [[组件-GTSAM]], [[概念-因子图]]
- [[方法-GICP配准方法]], [[方法-ICP配准方法]]
- [[方法-GaussianVoxelMap 体素化配准]], [[方法-IntegratedMatchingCostFactor]]
- [[概念-连续时间轨迹]], [[方法-连续时间 ICP 因子]]
- [[方法-gtsam_points因子封装模式]]
- [[方法-gtsam_points体素加速]]
- [[方法-gtsam_points连续时间因子]]
