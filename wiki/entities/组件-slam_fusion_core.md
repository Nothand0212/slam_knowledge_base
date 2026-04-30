---
type: entity
tags: [多传感器融合, 基础设施, 骨架库, phad_fusion]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-slam_fusion_core-analysis.md
---

# slam_fusion_core

> phad_fusion 的纯 C++ 多传感器融合骨架库，提供相机外参抽象、数据调度和后端适配器接口。

## 定义

为 phad_fusion 构建的多传感器融合 SLAM 骨架基础设施库。纯 C++、无 ROS 依赖，提供统一多相机表示、传感器类型定义、抽象接口层和 Pipeline 调度骨架。

## 核心特征

- **零 ROS 依赖**：CMake + C++17 + Eigen3，编译门槛极低
- **CameraRig**：用 `vector<Isometry3d>` 统一 N≥1 相机外参管理
- **Pipeline**：传感器数据入队 + 步进调度，设计存在数据流断裂缺陷（onStep 仅传计数非数据）
- **IOptimizationBackend**：后端适配器接口，计划对接 GTSAM 和 Ceres
- 当前状态：骨架阶段，视觉类型链、GTSAM 因子、IMU 预积分、回环检测均未实现
- 代码量 ≈ 375 行，8 个头文件 + 2 个实现文件 + 1 个示例

## 相关页面

- 未来承载：phad_fusion
- 参考：从 gtsam_points、IC-GVINS、OB_GINS、open_vins 学习设计模式
- [[组件-CameraRig 多相机抽象]]
- [[架构-Pipeline 传感器数据调度]]
- [[架构-后端适配器模式]]
- [[架构-多传感器融合架构]]
