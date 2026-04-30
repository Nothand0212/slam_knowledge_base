---
tags: [VIO, 视觉惯性里程计, 多相机, ORB特征, SE2(3)预积分]
sources:
  - raw/docs-deep-dive/openmavis_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/openmavis_analysis.md
---

# OpenMAVIS 源码级分析摘要

> 基于 ORB-SLAM3 的多相机增强 VIO（IMU_MULTI 模式），核心创新为 SE2(3) 精确 IMU 预积分和多相机 cam_idx 统一投影框架。

## 核心发现

- 新增 `IMU_MULTI`（sensor=6）模式，支持 4 路相机（前视双目 + 侧视双目）+ IMU
- 4 路 ORB 特征提取在独立线程并行执行，侧视相机分配 2 倍特征数补偿观测稀疏性
- 前视左右相机做立体匹配（BFMatcher + Hamming 距离 + Lowe's ratio test=0.8 + 三角化）
- 侧视相机与前视/侧视之间完全不做特征匹配，侧视特征以纯单目方式使用，依赖多帧运动视差
- 初始化仅用前视立体创建地图，侧视相机在后续跟踪中通过单目投影匹配逐步加入
- IMU 预积分为核心创新：使用 **SE2(3) 精确积分**（J1/J2 核函数捕获角速度耦合效应），替代 ORB-SLAM3 的零阶近似
- 多相机统一投影通过 `ImuCamPose` 类的 `cam_idx` 参数 + 链式外参变换实现
- 回环检测继承 ORB-SLAM3 的完整能力（DBoW2 + Sim3 + Essential Graph + Full BA）

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | ORB（4 路并行提取，侧视 2 倍特征） |
| 跟踪方法 | BoW 匹配 / 投影匹配 / IMU 预测 + 多相机网格 |
| 后端 | g2o 滑动窗口 + 边缘化（EdgePriorPoseImu） |
| 回环 | DBoW2 ORB 词袋 + Sim3 + Essential Graph |
| ROS 耦合 | 松耦合，通过配置文件和示例节点接入 |
| 在线标定 | 外参假设精确固定，IMU 偏置在线估计 |

## 关键代码引用

- 多相机输入: `System::TrackMulti` (include/System.h:131), `GrabImageMulti` (src/Tracking.cc:1540)
- 多相机初始化: `MultiInitialization` (src/Tracking.cc:2539-2612)
- SE2(3) 预积分: `Preintegrated::IntegrateNewMeasurement` (src/ImuTypes.cc:182-257)
- 多相机投影: `ImuCamPose` 类 (src/G2oTypes.cc:30-99)
- 前视立体匹配: `ComputeMultiFishEyeMatches` (src/Frame.cc:1327-1381)

## 相关页面

- [[组件-CameraRig 多相机抽象|多相机SLAM系统]]
- [[概念-IMU预积分]]
- [[概念-IMU预积分|SE2(3)精确预积分]]
- [[架构-多传感器融合架构|多传感器外参管理]]- [[架构-多传感器融合架构]]
- [[概念-IMU预积分]]
