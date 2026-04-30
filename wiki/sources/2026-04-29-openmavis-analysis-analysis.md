---
tags: [VIO,ImuCamPose (G2oTypes.cc), IntegrateNewMeasurement (ImuTypes.cc), MultiInitialization (Tracking.cc)]
sources:
  - raw/docs-deep-dive/openmavis_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/openmavis_analysis.md
---

# OpenMAVIS 源码级分析摘要

> 基于 ORB-SLAM3 的多相机增强视觉惯性 SLAM

## 核心发现

- SE2(3) 精确 IMU 预积分使用 J1/J2 捕捉角速度耦合，优于 ORB-SLAM3 零阶近似
- cam_idx 投影框架 + ImuCamPose 统一管理 1/2/4 相机，4 路并行 ORB 提取
- 侧视相机 2 倍特征补偿单目观测稀疏性，前视立体匹配用 Lowe's ratio 0.8 + 极线三角化
- 初始化仅用前视立体，侧视相机未参与；关键假设侧视与主相机无显著重叠
- [多相机,VIO,ORB-SLAM3,SE2(3)]

## 关键代码引用

- [[组件-OpenMAVIS]] [[概念-IMU预积分|SE2(3)IMU预积分]] [[组件-CameraRig 多相机抽象|多相机链式外参]] [[算法-ORB-SLAM3|ORB-SLAM3]]

## 相关页面


