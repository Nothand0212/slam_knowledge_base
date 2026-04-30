---
tags: [SLAM, 视觉SLAM, ORB特征, 回环检测, IMU初始化, g2o]
sources:
  - raw/docs-deep-dive/orb_slam3_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/orb_slam3_analysis.md
---

# ORB-SLAM3 源码级分析摘要

> 经典三线程架构（Tracking + LocalMapping + LoopClosing）的完整视觉/视觉-惯性 SLAM 系统，支持 6 种传感器模式和 Atlas 多地图管理。

## 核心发现

- 六种传感器模式：Mono/Stereo/RGBD 及其对应的 IMU 组合（IMU_MONOCULAR/IMU_STEREO/IMU_RGBD）
- 特征前端使用自写 `ORBextractor`：FAST 检测 + 四叉树均匀分布 + IC_Angle 灰度质心方向 + Steered BRIEF 描述子
- 金字塔 8 层，默认 1200 特征/帧，双阈值 FAST（iniThFAST=20, minThFAST=7）确保低纹理也能检测
- 初始化：单目用两帧匹配（H/F 并行计算 + 模型选择 + 三角化），双目直接立体三角化，RGB-D 用深度图
- IMU 初始化为分阶段精化：重力方向估计 → 惯性-only 优化（g2o EdgeInertialGS）→ 尺度/重力应用 → Full Inertial BA
- LocalMapping 线程负责关键帧处理、地图点剔除/创建、Local BA、IMU 初始化和关键帧冗余删除
- 回环检测使用 DBoW2 ORB 词袋 + Sim3 RANSAC + Essential Graph 位姿图优化 + 独立线程 Global BA
- Atlas 多地图系统支持跟踪丢失后创建新地图、跨地图重定位和地图自动合并

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | ORB（FAST + Steered BRIEF），四叉树分布 |
| 跟踪方法 | BoW 匹配 / 投影匹配 / 运动模型 + 恒速预测 |
| 后端 | g2o LM 优化（Motion-only BA → Local BA → Full BA） |
| 回环 | DBoW2 ORB 词袋 + Sim3 求解 + Essential Graph |
| ROS 耦合 | 松耦合（ROS 接口可剥离，内核无 ROS 依赖） |
| 在线标定 | IMU 偏置在线估计，外参依赖离线标定 |

## 关键代码引用

- 系统入口: `System::System` (src/System.cc:41-242)
- ORB 提取: `ORBextractor::operator()` (src/ORBextractor.cc:1086-1197)
- 单目初始化: `TwoViewReconstruction::Reconstruct` (src/TwoViewReconstruction.cc:41-129)
- IMU 初始化: `LocalMapping::InitializeIMU` (src/LocalMapping.cc:1173-1522)
- 跟踪主循环: `Tracking::Track` (src/Tracking.cc:1794-)

## 相关页面

- [[2026-04-29-framework-comparison|SLAM系统架构对比]]
- [[方法-视觉特征跟踪|ORB特征详解]]
- [[概念-视觉惯性初始化策略]]
- [[算法-ORB-SLAM3|多地图与Atlas管理]]- [[算法-DSO]]
- [[概念-回环检测方法]]
