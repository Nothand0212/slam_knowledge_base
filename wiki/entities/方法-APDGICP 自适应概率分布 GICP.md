---
type: entity
tags: [点云配准, 4D雷达, GICP, 传感器模型, 各向异性]
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
sources:
  - wiki/sources/2026-04-28-4d-radar-slam-analysis.md
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# APDGICP 自适应概率分布 GICP

> 4DRadarSLAM 针对 4D 雷达噪声模型设计的 GICP 变体，把 range/azimuth/elevation 三个方向的各向异性测量不确定性写入配准协方差。

## 定义

4DRadarSLAM 自研的点云配准算法，将 4D 雷达的距离（var ∝ dist²）、方位角（~0.5°）、俯仰角（~1.0°）三维非均匀测量不确定性精确建模入 GICP 协方差矩阵。

## 传感器模型

4D 雷达点的误差不是 LiDAR 式近各向同性噪声。距离方向误差会随距离增长，角度方向由天线阵列分辨率决定：

| 参数 | 4DRadarSLAM 默认值 | 含义 |
|------|--------------------|------|
| `dist_var` | `0.86` | 100m 处距离测量不确定度约 0.86m |
| `azimuth_var` | `0.5°` | 方位角测量精度 |
| `elevation_var` | `1.0°` | 俯仰角测量精度 |

因此每个雷达点的概率分布是沿距离方向拉长、角度方向压缩的椭球。APDGICP 的核心价值就是把这个椭球带入 GICP，而不是假设所有方向同噪声。

## 核心特征

- 每个雷达点的概率分布为严重**各向异性椭球**（距离方向拉长，角度方向压缩）
- 默认参数：dist_var=0.86m @100m, azimuth_var=0.5°, elevation_var=1.0°
- 与标准 GICP（假设各向同性/近各向同性）的本质差异：传感器物理模型驱动
- 配合 Fitness Score 自适应信息矩阵权重
- 支持 5 种配准方法可选（ICP/NDT_OMP/FAST_GICP/APDGICP/FAST_VGICP）

## 在 4DRadarSLAM 中的位置

APDGICP 用于前端 scan-to-scan 配准，输出里程计约束进入后端位姿图。Doppler 信息已经在前端用于自速度估计和动态目标去除，APDGICP 则负责利用雷达点位置的不确定性完成几何配准。两者都是“传感器物理模型驱动”的前端模块，但作用不同。

## 工程启发

- 对低精度或强各向异性传感器，显式建模测量不确定性通常比套用标准 ICP 更可靠。
- 噪声参数应来自传感器规格或标定，而不是后端任意调权。
- Fitness score 阈值仍然需要保留，因为正确的协方差模型不能解决错误初值和动态场景全部问题。

## 相关页面

- 实现于：[[算法-4DRadarSLAM]] `registrations.cpp:38-49`
- [[方法-GICP配准方法]]
- [[传感器-Doppler 自速度估计]]
- [[特殊传感器数据管线|传感器物理模型驱动配准]]
- [[方法-GaussianVoxelMap 体素化配准]]
