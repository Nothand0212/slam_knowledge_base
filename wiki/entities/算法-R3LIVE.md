---
tags: [LiDAR, 视觉惯性, 紧耦合, 稠密建图]
sources:
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
  - wiki/sources/2026-04-29-lvi_sam_analysis.md
  - wiki/sources/2026-04-29-r3live_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# R3LIVE

> R3LIVE是实时RGB-IMU-LiDAR紧耦合融合系统，用两个并行的卡尔曼滤波器（LIO + VIO）联合估计位姿和稠密彩色点云地图，可实时输出带颜色纹理的3D重建。

## 核心方法
R3LIVE包含两个并行子系统：LIO（LiDAR-惯性里程计，基于FAST-LIO2的IESKF）和VIO（视觉-惯性里程计）。LIO用LiDAR点面残差估计IMU频率的位姿，VIO用视觉光度误差在LIO位姿基础上微调。两者异步运行但共享同一IMU状态。建图模块将新扫描的LiDAR点投影到渲染地图上，用视觉颜色更新每个体素的RGB，生成增量稠密彩色点云，无需离线后处理。

## 关键设计
- 双卡尔曼滤波：LIO和VIO两个IESKF共享IMU状态异步更新
- 稠密彩色重建：实时将视觉纹理投影到LiDAR几何表面
- 紧耦合三传感器：IMU做统一纽带，LiDAR提供几何、视觉提供外观
- RGB-D风格输出：建图结果即带颜色的三维点云，可直接可视化

## 架构取舍

R3LIVE 选择双 ESIKF 而不是单一统一滤波器，换来模块并行和更清晰的 LIO/VIO 分工。LIO 提供几何和尺度稳定性，VIO 利用图像光度约束修正细节并给地图着色。代价是两个滤波器之间的信息一致性更难严格保证，需要通过 LIO-as-Prior 和共享状态小心传递先验。

## 工程边界

R3LIVE 的输出地图视觉效果强，但颜色质量依赖相机曝光、外参、时间同步和动态物体处理。若 LiDAR 前端退化，VIO 以 LIO 为先验也可能被带偏。它适合需要实时彩色点云的机器人建图任务，不只是单纯定位后端。

## 相关页面
- [[2026-04-28-r3live-analysis]]
- [[VIO方案对比]]
- [[LiDAR方案对比]]
- [[算法-FAST-LIO]]
- [[架构-双ESIKF架构]], [[方法-RGB着色点云]]
- [[2026-04-29-external-primary-source-check]]
