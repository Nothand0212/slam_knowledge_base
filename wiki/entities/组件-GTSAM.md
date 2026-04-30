---
tags: [GTSAM, factor-graph, SLAM, library]
created: 2026-04-27
updated: 2026-04-27
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-28-phad-fusion-design.md
  - wiki/sources/2026-04-28-superodom-analysis.md
  - wiki/sources/2026-04-29-dm-vio-analysis-analysis.md
---

# GTSAM

> Georgia Tech Smoothing and Mapping library，用 factor graph 和 Bayes network 建模 robotics/vision 中的 SAM、SLAM、SfM、navigation 和 sensor fusion 问题。

## 定位

GTSAM 是 C++ 库，并提供 Python/MATLAB wrappers。官方文档把 GTSAM 描述为 smoothing and mapping library，底层计算范式是 [[概念-因子图]] 和 Bayes network，而不是直接暴露稀疏矩阵。

面向 ROS 机器人导航工程，GTSAM 常用于：

- pose graph SLAM：`Pose2`/`Pose3` + `BetweenFactor` + `PriorFactor`
- VIO/INS：`NavState` + IMU preintegration + `ImuFactor`/`CombinedImuFactor`
- visual SLAM/SfM：projection factor、stereo factor、smart factor
- localization/sensor fusion：GPS、barometer、magnetometer、自定义 measurement factor
- uncertainty analysis：`Marginals`、`ISAM2` marginal APIs

## 版本注意

本知识库按用户要求以 GTSAM 4.3a1 为准。官方文档首页显示 develop 处于 Pre 4.3 模式，并说明 4.3 附近存在弃用和潜在 API-breaking changes。查 API 时优先使用：

- Python docs: https://borglab.github.io/gtsam/
- C++ reference: https://gtsam.org/doxygen/
- 本地页面：[[GTSAM API 使用索引]]

## 核心抽象

- `Key` / `Symbol`：变量 ID。Python 常用 `from gtsam.symbol_shorthand import X, V, B, L, C`。
- `Values`：变量容器，保存初值或优化结果。
- `NonlinearFactorGraph`：非线性因子图容器。
- `NonlinearFactor`：所有非线性因子的基类。
- `noiseModel`：measurement uncertainty 和 residual whitening 的来源。
- `NonlinearOptimizer`：batch optimizer 基类，派生出 `GaussNewtonOptimizer`、`LevenbergMarquardtOptimizer`、`DoglegOptimizer`。
- `ISAM2`：incremental smoothing and mapping，用于在线/增量 SLAM。

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM 因子图工作流]]
- [[GTSAM Geometry API]]
- [[GTSAM Navigation 与 IMU API]]
- [[2026-04-27-gtsam-4.3a1-docs]]

---
## (合并自: GTSAM因子类型.md)
---
---
tags: [GTSAM, 因子图, 优化]
sources: []
created: 2026-04-28
type: entity
---

# GTSAM 因子类型总结

> 对 GTSAM 和扩展库（gtsam_points）中常用因子类型的综述，覆盖 IMU 预积分、视觉重投影、LiDAR 配准和位姿图约束。

## 概述

GTSAM 的因子图框架通过多种因子类型表达不同传感器的测量约束。这些因子在 `NonlinearFactorGraph` 中组合，并通过 `LevenbergMarquardtOptimizer` 或 `ISAM2` 求解。以下是核心因子类型：

- **PreintegratedImuMeasurements / CombinedImuFactor**：IMU 预积分因子，将连续加速度计和陀螺仪测量值预积分为帧间相对位姿、速度和偏置变化，封装为 `CombinedImuFactor` 时包含偏置雅可比以支持在线偏置估计。
- **SmartStereoFactor**：智能立体视觉因子，内部使用 Schur 补将路标点边缘化，避免将大量地图点显式注入因子图，仅输出位姿间约束，可视为 GTSAM 层面对 MSCKF 零空间投影的对偶实现。
- **IntegratedGICPFactor / IntegratedVGICPFactor（gtsam_points）**：Koide 开发的 LiDAR 点云配准因子，将 GICP/VGICP 损失函数线性化为因子图约束，支持对配准协方差的自动估计。
- **BetweenFactor**：通用相对位姿约束因子，用于里程计边、回环边和先验相对变换，是位姿图优化（PGO）的基本单元。
- **PriorFactor**：一元先验因子，用于锚定全局坐标系（如 GNSS 观测或初始位姿），通常配合大信息矩阵防止漂移。
- **NoiseModelFactor**：所有噪声因子的基类，通过 `noiseModel::Gaussian::Covariance()` 或 `noiseModel::Robust()` 注入测量不确定性模型。

## 在分析框架中的应用

- VINS-Fusion 回环和全局位姿图优化使用 `BetweenFactor` + `PriorFactor`。
- Kimera-VIO 使用 `SmartStereoFactor` 和 `CombinedImuFactor` 进行 VIO。
- fusions_slam 使用 `IntegratedGICPFactor` 将 LiDAR 配准结果作为因子注入。

## 对 SLAM 算法的意义

多传感器因子图融合直接依赖上述因子类型：LiDAR 前端输出 → `IntegratedGICPFactor`；VIO 前端输出 → `CombinedImuFactor` + `SmartStereoFactor`；回环检测 → `BetweenFactor`；GNSS → `PriorFactor`。理解各因子的信息矩阵结构和线性化特性是设计融合策略的前提。

## 相关页面

- [[组件-GTSAM]]
- [[概念-因子图]]
- [[架构-滑动窗口优化]]
- [[方法-genz-icp]]
- [[组件-DBoW2]]
---
## (合并自: GTSAM ISAM2 IMU 预积分.md)
---
---
type: entity
tags: [GTSAM, ISAM2, IMU预积分, 位姿先验]
created: 2026-04-29
---

# GTSAM ISAM2 IMU 预积分

## 定义

SuperOdom 中 IMU 端的 GTSAM ISAM2 增量优化实现，将 LiDAR 里程计结果作为 `PriorFactor<Pose3>` 输入 IMU 图优化，IMU 预积分结果回传给 LiDAR 前端作为初始猜测。

## 核心特征

- 使用 GTSAM `CombinedImuFactor` + `PreintegratedImuMeasurements` 标准预积分
- 状态变量：X(Pose3), V(Velocity3), B(Bias6)
- ISAM2 配置：relinearizeThreshold=0.1, relinearizeSkip=1
- 每帧添加 lidar pose 先验 + IMU 因子 + Bias BetweenFactor
- 支持两种外参路径：LiDAR→IMU 或 LiDAR→Camera→IMU
- 每 100 帧 `reset_graph()` 重置 ISAM2 防止内存和计算量无限增长
- 失败检测：|v|>30m/s 或 |ba|>2.0 或 |bg|>1.0 → 自动 reset

## 关联

- 实现于：[[2026-04-28-superodom-analysis|SuperOdom]] `imuPreintegration.cpp`
- 参考：[[概念-IMU预积分]]、[[组件-GTSAM]]