---
tags: [LiDAR, 视觉惯性, IESKF, 激光SLAM, 紧耦合]
sources:
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
  - wiki/sources/2026-04-29-lvi_sam_analysis.md
  - wiki/sources/2026-04-29-r3live_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# FAST-LIVO2

> FAST-LIVO2是FAST-LIO2的视觉-LiDAR融合扩展，在IESKF框架内紧耦合LiDAR点云和稀疏视觉特征，用一个卡尔曼滤波器同时融合三类传感器，实现快速高精度里程计。

## 核心方法
FAST-LIVO2继承FAST-LIO2的IESKF和iKD-Tree架构，新增视觉特征处理管道。视觉前端提取FAST角点并用KLT光流跟踪，将特征点重投影误差与LiDAR点面残差统一在同一IESKF更新步骤中处理。IMU前向积分做状态预测，后向传播补偿LiDAR和视觉帧内运动畸变。三传感器紧耦合避免了松耦合的信息延迟和精度损失。

## 关键设计
- 单滤波器三融合：LiDAR点面残差、视觉重投影残差、IMU预积分共用一个IESKF
- iKD-Tree复用：增量地图同时服务于LiDAR配准和视觉特征深度初始化
- 后向传播：IMU高频数据同时补偿LiDAR扫描和相机曝光期间的位姿变化
- GPU加速：点云处理和特征跟踪均在GPU上并行

## 与 R3LIVE/LVI-SAM 的区别

LVI-SAM 是子系统松耦合，R3LIVE 使用双 ESIKF，FAST-LIVO2 则强调统一 IESKF。统一滤波器让 LiDAR 和视觉残差共享同一状态和协方差，避免跨模块传递先验时的信息重复或丢失。代价是前端和滤波器耦合更紧，传感器外点会直接影响同一状态。

## 工程边界

FAST-LIVO2 依赖准确外参、时间同步和稳定视觉跟踪。视觉弱纹理时主要靠 LiDAR/IMU，几何退化时则希望视觉提供补充约束。若两个传感器同时退化，统一滤波器也不能凭空恢复可观性，因此仍需要退化检测和残差质量控制。

## 相关页面
- [[2026-04-28-fast-livo2-analysis]]
- [[VIO方案对比]]
- [[LiDAR方案对比]]
- [[算法-FAST-LIO]]
- [[方法-统一IESKF融合]], [[概念-直接法光度误差]]
- [[2026-04-29-external-primary-source-check]]
