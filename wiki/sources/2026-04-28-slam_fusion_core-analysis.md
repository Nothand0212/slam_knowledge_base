---
tags: [多传感器融合, 基础设施, 骨架库, phad_fusion, C++17, GTSAM适配器]
sources:
  - raw/docs-deep-dive/slam_fusion_core_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/slam_fusion_core_analysis.md
---

# slam_fusion_core 深度源码分析

> phad_fusion 的基础设施骨架库。纯 C++、零 ROS 依赖，提供统一多相机表示、传感器类型定义、抽象接口和 Pipeline 调度。

## 摘要

slam_fusion_core 是 375 行代码的极简多传感器融合骨架。(1) [[组件-CameraRig 多相机抽象]]用 vector<Isometry3d> 统一 N>=1 相机；(2) [[架构-Pipeline 传感器数据调度]]管理三队列 IMU/GNSS/Visual；(3) [[架构-后端适配器模式]]通过 IOptimizationBackend 接口封装 GTSAM/Ceres；(4) 明确的设计缺陷：onStep 仅传计数非数据、视觉类型链完全缺失、无线程安全。

## 核心概念

- **纯骨架定位**：类型系统仅定义了基础测量结构体，核心算法（预积分/特征点/GTSAM 因子）全部待实现
- **数据流断裂**：runStep() 传队列大小给后端而非实际数据，phad_fusion 需修复此缺陷
- **CameraRig 设计优秀**：monocular() 工厂方法体现设计经验，需补充内参 + td + 可优化标记
- **正确的架构分层**：接口与实现分离，头文件不暴露优化框架
- **参考来源**：open_vins（视觉类型）、IC-GVINS/OB_GINS（GNSS 因子）、gtsam_points（因子体系）

## 相关页面

- [[组件-slam_fusion_core]]
- [[组件-CameraRig 多相机抽象]]
- [[架构-Pipeline 传感器数据调度]]
- [[架构-后端适配器模式]]
- [[架构-多传感器融合架构]]
- [[概念-因子图]]