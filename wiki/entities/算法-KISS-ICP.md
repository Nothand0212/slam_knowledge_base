---
tags: [LiDAR, ICP, 激光SLAM, 里程计]
sources:
  - wiki/sources/2026-04-28-cartographer-analysis.md
  - wiki/sources/2026-04-28-fusions-slam-analysis.md
  - wiki/sources/2026-04-28-lio-sam-analysis.md
  - wiki/sources/2026-04-29-genz_icp_analysis.md
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# kiss-icp

> "Keep It Simple, Stupid"——kiss-icp是一个极简纯LiDAR里程计管线，仅用点对点ICP配准和恒速运动模型预测，无IMU、无特征提取、无回环检测，以不足千行代码实现可靠位姿估计。

## 核心方法
kiss-icp的核心流程极为简洁：用恒速运动模型从上一帧位姿预测当前初值，对去畸变点云执行voxel降采样，然后运行点到点ICP（使用PCL实现或自实现）配准当前扫描到局部地图，输出帧间位姿。局部地图由最近N帧点云累积而成，帧数可配置。没有IMU因子、没有回环检测、没有GTSAM后端，仅靠ICP配准和恒速模型提供连续里程计。

## 关键设计
- 极简设计：零外部依赖（除PCL/Open3D），核心代码<1000行
- 纯ICP：使用最简单的点到点欧氏距离，没有高阶配准模型
- 恒速预测：上一帧速度作为下一帧初值，无IMU
- 局部累积地图：最近N帧点云叠加，提供足够几何约束

## 相关页面
- [[2026-04-28-kiss-icp-analysis]]
- [[LiDAR方案对比]]
---
## (合并自: KISS-ICP分析摘要.md)
---
---
tags: [KISS-ICP, ICP, LiDAR, 里程计]
sources: [wiki/sources/2026-04-28-kiss-icp-analysis.md]
created: 2026-04-28
type: entity
---

# KISS-ICP分析摘要

> 极简 LiDAR ICP 里程计，仅 3 个 C++ 文件，证明 ICP 无需复杂 feature extraction 亦可达到 SOTA 精度。

## 核心方法

KISS-ICP 核心是纯粹的 Point-to-Point ICP 配准，无特征提取、无回环检测、无 IMU 融合。通过自适应关键帧插入、恒定速度运动模型预测和动态点云去畸变，以最简实现达到与复杂方案相当的定位精度。

## 关键设计

- 极度精简：仅 3 个源文件，1500 行 C++
- 自适应关键帧：根据配准重叠率自动选择关键帧
- 速度模型预测：恒定速度模型提供 ICP 初值
- 点云去畸变：利用里程计估计补偿扫描内运动
- 无后端优化：纯里程计模式，无回环/BA

## 相关页面

- [[算法-KISS-ICP]]
- [[LiDAR方案对比]]
- [[算法-CT-ICP]]