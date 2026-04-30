---
tags: [LiDAR, 视觉惯性, 因子图, 多传感器融合, SAM]
sources:
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
  - wiki/sources/2026-04-29-lvi_sam_analysis.md
  - wiki/sources/2026-04-29-r3live_analysis.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# LVI-SAM

> LVI-SAM在LIO-SAM因子图框架上增加视觉惯性子系统（VINS-Mono），两个子系统并行运行并互为备份，通过因子图实现视觉-LiDAR松耦合融合。

## 核心方法
LVI-SAM由两个并行子系统组成：LIO子系统（基于LIO-SAM的LiDAR-惯性里程计）和VIS子系统（基于VINS-Mono的视觉惯性里程计）。两个子系统独立输出6-DoF位姿，当某一传感器退化时另一子系统接管。VIS可利用LiDAR点云初始化深度替代三角测量，LIO用视觉BA结果检测并纠正LiDAR失效。所有因子在GTSAM因子图中由iSAM2统一优化。

## 关键设计
- 双子系统并行：LIO和VIS独立运行，任一退化时互为备份
- LiDAR辅助视觉深度：点云直接为视觉特征提供深度初值
- 视觉辅助LiDAR退化检测：BA结果识别无结构场景中LiDAR失效
- 因子图松耦合：最终约束统一为GTSAM因子，iSAM2求解

## 架构取舍

LVI-SAM 的优点是复用成熟子系统：LIO-SAM 负责几何稳定性，VINS-Mono 负责视觉约束和纹理环境。缺点是两个子系统有各自状态和优化器，信息传递需要通过 ROS 消息和约定接口完成，不如统一滤波/统一因子图那样统计一致。

## 适用边界

它适合工程上快速获得 LiDAR+Vision+IMU 互补能力，尤其是单一传感器偶发退化的场景。若追求严格紧耦合、低延迟和统一协方差传播，FAST-LIVO2 或 R3LIVE 一类架构更直接。

## 相关页面
- [[2026-04-28-lvi-sam-analysis]]
- [[VIO方案对比]]
- [[LiDAR方案对比]]
- [[算法-LIO-SAM]]
- [[概念-因子图]]
- [[架构-多传感器松耦合]]
