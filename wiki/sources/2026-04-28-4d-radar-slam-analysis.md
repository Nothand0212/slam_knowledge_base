---
tags: [4D雷达, 毫米波雷达, SLAM, 多普勒, 传感器模型, 全天候, g2o]
sources:
  - raw/docs-deep-dive/4d_radar_slam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/4d_radar_slam_analysis.md
---

# 4DRadarSLAM 深度源码分析

> ICRA 2023 Zhang, Zhuge et al. 基于 4D 成像雷达（Oculii Eagle）的位姿图优化 SLAM 系统，全天候、抗烟雾，利用多普勒速度 + 各向异性传感器模型驱动配准。

## 摘要

4DRadarSLAM 将 4D 雷达的三项核心能力系统化引入 SLAM。(1) [[传感器-Doppler 自速度估计]]通过 RANSAC+LSQ 鲁棒估计 3D 自运动并区分动静态目标；(2) [[方法-APDGICP 自适应概率分布 GICP]]将雷达距离/方位/俯仰三维各向异性不确定性建模入 GICP；(3) [[方法-Intensity Scan Context]]用 SNR 替代高度值适配 SC；(4) [[方法-五重回环几何验证]]五层递进过滤应对点云稀疏性。

## 核心概念

- **传感器物理模型驱动**：APDGICP 的 dist_var/azimuth_var/elevation_var 直接来自雷达硬件参数
- **多普勒信息链路**：自速度估计 --> 前端 ICP 初值 + 动态目标去除 + ICP 质量验证
- **回环五层验证**：预筛选 5 重 --> ISC --> ICP fitness --> Odometry Check --> Pairwise Consistency
- **g2o 后端**：LM + Cholmod 512 次迭代，GPS/气压计/IMU 预积分多约束因子图
- 不同于 LiDAR 的挑战：稀疏性（约 1000 点/帧）、各向异性噪声、多径鬼影

## 相关页面

- [[算法-4DRadarSLAM]]
- [[方法-APDGICP 自适应概率分布 GICP]]
- [[传感器-Doppler 自速度估计]]
- [[方法-Intensity Scan Context]]
- [[方法-五重回环几何验证]]
- [[方法-GICP配准方法]]
- [[方法-ScanContext]]