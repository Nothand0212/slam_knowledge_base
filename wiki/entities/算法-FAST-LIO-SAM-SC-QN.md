---
tags: [算法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-fast_lio_sam-analysis.md
---
# FAST-LIO-SAM-SC-QN

> 外接 FAST-LIO2 前端的 LiDAR SLAM 后端：用 ISAM2 位姿图、ScanContext、Quatro 和 Nano-GICP 组成回环优化管线。

## 架构定位

FAST-LIO-SAM-SC-QN 是一个 LIO + PGO 分离架构的后端系统。它自身不实现 LiDAR-IMU 前端，而是依赖外部 FAST-LIO2 输出高频里程计和去畸变点云；本系统负责关键帧管理、回环检测、位姿图优化和优化后轨迹输出。

## 回环管线

回环验证采用四阶段设计：先用 [[方法-ScanContext]] 检索地点候选，再用欧氏距离门限过滤明显不合理的候选，然后用 [[方法-Quatro配准]] 做全局粗配准，最后用 [[组件-Nano-GICP]] 做局部精配准。只有通过完整链路的约束才进入 GTSAM ISAM2 位姿图。

关键帧通常按平移阈值创建，例如位移超过 1.5 m 才插入，避免后端节点过密。实时输出可用 `last_corrected * odom_delta` 外推，把低频 PGO 修正和高频前端里程计结合起来。

## 工程取舍

这种拆分让系统可以复用 FAST-LIO2 的高质量前端，同时独立替换回环和后端模块。代价是前后端之间要通过 ROS topic/文件接口传递状态，时间同步、点云坐标系和关键帧选择都必须保持一致。ScanContext 在对称场景中仍可能误召回，因此 Quatro 和 Nano-GICP 不是可选增强，而是防止错误回环进入后端的必要保护。

## 相关
- [[方法-四阶段回环验证]]
- [[方法-Quatro配准]]
- [[组件-Nano-GICP]]
- [[方法-ScanContext]]
- [[LiDAR方案对比]]
