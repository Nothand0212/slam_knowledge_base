---
tags: [VIO, GINS, 信息矩阵, 多约束, GNSS]
sources:
  - wiki/sources/2026-04-28-ic_gvins-analysis.md
  - wiki/sources/2026-04-28-ob_gins-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
  - wiki/sources/2026-04-29-gnss-pipeline-comparison.md
  - wiki/sources/2026-04-29-special-sensors-comparison.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# IC-GVINS

> Information-Coupled Global VINS，以信息矩阵耦合机制实现视觉-惯性-GNSS 的多约束全局位姿估计的紧耦合融合框架。

## 核心思想

IC-GVINS 由 Xiao 等人提出（2022），是一种融合视觉、惯性和 GNSS 全局约束的全球参考系位姿估计系统。其核心创新是“信息耦合”机制：将 INS (Inertial Navigation System) 的机械编排解算结果与 VIO 的相对约束通过信息矩阵在全局坐标系中紧耦合，而非直接加权平均。支持多系统 GNSS（GPS/GLONASS/Galileo/BDS）单点定位和 RTK 约束注入。

## 状态与约束

IC-GVINS 的状态通常包含导航系位姿、速度、IMU 偏置，以及视觉/惯性和 GNSS 之间的坐标关系。视觉前端提供局部相对运动和结构约束，IMU 提供高频传播，GNSS 提供全球参考。信息耦合的意义在于把这些约束统一到可比较的信息矩阵里，让系统按不确定性融合，而不是按经验权重混合轨迹。

## 在 SLAM 中的应用

GNSS-Visual-IMU 融合的主要难点是坐标系和统计一致性：VIO 工作在局部坐标系，GNSS/INS 工作在全球导航系，二者尺度、方向和噪声模型不同。IC-GVINS 通过全局参考系下的紧耦合优化减少坐标系切换造成的信息损失，并为城市峡谷、GNSS 断续和多径环境提供更稳的全局约束接入方式。

## 工程边界

IC-GVINS 的收益依赖 GNSS 质量控制。如果 NLOS 或多径观测未经筛选就被赋予过高权重，全球约束会直接污染 VIO/INS 状态。因此需要卫星质量筛选、鲁棒核、协方差膨胀或观测拒绝策略。对纯室内或 GNSS 不可用场景，IC-GVINS 的全局优势会消失，系统退化为 VIO/INS。

## 相关页面

- [[组件-GTSAM]], [[概念-因子图]]
- [[概念-IMU预积分]], [[架构-多传感器融合架构]]
- [[算法-OB_GINS]]
