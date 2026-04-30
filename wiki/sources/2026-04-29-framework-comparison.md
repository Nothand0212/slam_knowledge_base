---
tags: [框架对比, ROS耦合, SLAM, 定位, 因子图, 滤波]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-top/framework_comparison.md
sources:
  - raw/docs-top/framework_comparison.md
---

# 定位相关子项目框架对比

> slam_ws 仓库中 17 个定位相关子项目的 ROS 耦合度、优化后端、因子扩展对比分析

## 对比维度

表格对比了各项目在 **ROS 耦合**、**优化/滤波后端**、**因子扩展**、**状态与传感器**、**并发模型**、**依赖体量** 六个维度的差异。

## 核心分类

### 后端类型分布
- **GTSAM 因子图系**：IC-GVINS, OB_GINS, FAST-LIO-SAM-SC-QN, lt-mapper, gtsam_points
- **MSCKF/EKF 滤波系**：open_vins, SchurVINS, msckf_vio
- **IESKF 系**：fusions_slam, lightning-lm, FAST-LIO2
- **优化系 (G2O)**：ORB-SLAM3, OpenMAVIS
- **纯前端库**：genz-icp, KISS-ICP

### ROS 耦合度
- **松耦合（核可单独编译）**：fusions_slam, gtsam_points, genz-icp
- **中等**：open_vins, IC-GVINS, OB_GINS
- **紧耦合**：典型 catkin 单节点大仓库

### GNSS 支持
- **主战场**：IC-GVINS, OB_GINS
- 其余多为激光或纯 VIO

## 关键结论

- fusions_slam 的 ROS 与算法分层清晰（wrapper/ros + fusion_slam 核）
- open_vins 库化程度高，CamBase 虚基类和 TrackBase 支持多相机
- GTSAM 系的因子插件化程度高，适合作为 slam_fusion_core 对接目标

## 相关页面

- [[算法-IC-GVINS]]
- [[算法-OB_GINS]]
- [[组件-gtsam_points]]
- [[算法-fusions_slam|fusions_slam分析]]
- [[概念-MSCKF]]
- [[因子图vs滤波]]