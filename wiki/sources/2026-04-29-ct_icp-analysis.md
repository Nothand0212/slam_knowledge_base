---
tags: [source, 源码分析, SLAM, ct-icp]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/ct_icp_analysis.md
sources:
  - raw/docs-deep-dive/ct_icp_analysis.md
---
# CT-ICP 深度分析

**日期**: 2026-04-29 | **语言**: C++/Python | **依赖**: Ceres, SlamCore, PCL

## 概要
连续时间 ICP（886 stars）。将每帧 LiDAR 扫描建模为 12 自由度连续轨迹（begin_pose + end_pose）而非单一位姿。[[方法-连续时间线性插值]]：平移线性 + 旋转四元数 slerp。[[方法-运动先验约束]]正则化（位置一致性+常速度+朝向+小速度）。[[方法-RobustRegistration]] 自适应增强鲁棒性。

## 核心概念
- [[算法-CT-ICP]] — 12-DoF 连续时间帧轨迹
- [[方法-连续时间线性插值]] — 线性+slerp 高效近似
- [[方法-运动先验约束]] — 四种正则化项防跳变
- [[方法-RobustRegistration]] — 失败时缩小体素/扩大邻域自适应
- [[方法-POINT_TO_DISTRIBUTION]] — 邻域协方差马氏距离(GICP等效)

## 管线要点
- 四种残差: POINT_TO_PLANE(默认) / POINT / LINE / DISTRIBUTION(GICP)
- 权重: planarity^power + exp(-dist/threshold)
- 鲁棒核: Cauchy(默认) / Huber / Truncated
- 地图插入: 拒绝大旋转帧 + 连续5次失败强制插入
- Ceres OpenMP 多线程并行计算邻域

## 工程局限
- 纯里程计无回环/全局优化 → 长距离漂移
- 需要每点精确时间戳（部分传感器不支持）
- 无 IMU 深度融合（预留 IMSLIDAR 接口）

## 相关
- [[概念-连续时间轨迹]]
- [[LiDAR方案对比]]
