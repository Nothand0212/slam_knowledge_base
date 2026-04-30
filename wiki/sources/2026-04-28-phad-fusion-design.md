---
tags:
  - SLAM
  - 因子图
  - GTSAM
  - ISAM2
  - 多传感器融合
  - phad_fusion
  - 系统设计
  - 架构设计
  - 增量优化
  - 视觉惯性里程计
  - LiDAR SLAM
  - GNSS 紧耦合
  - 在线标定
  - 模式切换
  - 滑动窗口
  - 回环检测
  - 位姿图优化
sources:
  - raw/notes/phad_fusion_design.md
created: 2026-04-28
updated: 2026-04-29
type: source
source_path: raw/notes/phad_fusion_design.md
---

# phad_fusion 多模态因子图 SLAM 算法设计 摘要

> phad_fusion 是一个以"一切皆是因子"为核心哲学的多传感器因子图 SLAM 系统，基于 GTSAM ISAM2 增量优化引擎，支持纯里程计/局部建图/全局 SLAM 三模式运行时切换，目标在 EuRoC/KITTI 等标准数据集上达到 SOTA 级精度。

## 核心发现

### 设计哲学：Everything is a Factor

phad_fusion 将系统的全部状态估计建模为一个增量式因子图。每种传感器测量、每个运动学约束、每个先验知识都是因子图中的因子。核心循环为：接收数据 → 生成因子 → ISAM2 增量优化 → 输出结果。新增传感器仅需实现一个新的因子类并注册到因子图管理器，核心架构零修改。**这解决了现有开源方案（open_vins/VINS-Fusion/ORB-SLAM3/IC-GVINS 等）对传感器组合硬编码假设的根本性缺陷。**

### 状态变量设计

全状态向量定义在流形上：载体姿态 SO(3) + 位置 R³ + 速度 R³ + 陀螺偏置 R³ + 加速度计偏置 R³，此外包含传感器外参 SE(3)、相机内参 Rⁿ、时间偏移、环境路标 R³（SLAM 模式）、GNSS 相关参数（ENU 原点、接收机钟差）。所有变量均使用 GTSAM 的 Lie group 内置支持做流形优化，ISAM2 在最新线性化点局部坐标中做一阶 Taylor 展开，避免了 EKF 的一次线性化精度损失。

### 三模式架构

系统支持三种运行模式并可运行时无损切换：(1) **纯里程计**（ODOMETRY）— 8-12 帧滑动窗口，路标通过 SmartFactor Schur 补隐式消去，最快响应，适合飞控/自驾输入；(2) **局部建图**（LOCAL_MAPPING）— 10-15 帧窗口 + 显式路标优化，局部精度最高；(3) **全局 SLAM**（GLOBAL_SLAM）— 关键帧 + 路标永久保存，回环检测 + PGO + 全局 BA。模式切换的关键是路标参数化的双向转换（SmartFactor 隐式 ↔ 显式 Factor）。

### 多传感器前端管线

- **视觉前端**：参考 open_vins 的 FAST+KLT 管线 + VINS-Fusion 反向光流验证 + Kimera-VIO 的 ANMS 均匀化和立体匹配，支持可选的 ORB/SuperPoint 描述子用于回环
- **LiDAR 前端**：基于 genz-icp 的自适应 GICP，通过局部协方差特征值比计算平面度权重 α ∈ [0.1, 1.0]，实现退化场景鲁棒的配准，协方差从最终 Hessian 逆近似估计
- **IMU 预积分**：使用 GTSAM 的 PreintegratedCombinedMeasurements（Forster 2015 流形预积分），bias 更新时 Jacobian 一阶修正自动处理
- **GNSS 因子**：Position 因子继承 NoiseModelFactor1<Pose3>，残差 = p_ENU + R*lever - p_GNSS_ENU，未来可扩展双差载波相位 RTK 因子

### 工程架构

零 ROS 强制依赖，CMake 构建，四线程模型（Sensor → Frontend → Backend → Loop），线程间通过 lock-free SPSC queues 通信，内存预算 ODOMETRY 20MB / LOCAL 50MB / GLOBAL 500MB。八阶段实现路线图从骨架编译到 SOTA 优化，当前处于 Phase 0 设计阶段。

## 技术要点

| 维度 | 方案 |
|------|------|
| 优化后端 | GTSAM >= 4.2，ISAM2 增量平滑 + FixedLagSmoother |
| 边缘化 | ISAM2 Bayes Tree 变量消除，严格 Schur 补（非伪逆截断） |
| 视觉特征 | FAST + KLT + 反向光流验证 + RANSAC F 矩阵 |
| LiDAR ICP | genz-icp 自适应 GICP + planar/退化权重调节 |
| 回环检测 | DBoW2/DBoW3 视觉词袋 + ScanContext LiDAR 回环 |
| 在线标定 | 相机内外参 + IMU bias + 时间偏移 + 外参均可作为状态变量 |
| GNSS 集成 | WGS84 ↔ ENU 转换 + 位置因子 + 未来 RTK 双差载波相位 |
| C++ 标准 | C++17 |
| 目标精度 | EuRoC <0.05m ATE, KITTI <1% trans error |

## 关键引用

- 核心循环 runOnce(): `phad_fusion_design.md:146-189`
- 三模式切换机制: `phad_fusion_design.md:484-519`
- GNSS 位置因子设计: `phad_fusion_design.md:357-384`
- 自适应 GICP 代价函数: `phad_fusion_design.md:299-330`
- 初始化三层次策略: `phad_fusion_design.md:192-233`

## 相关页面

- [[2026-04-28-ic_gvins-analysis|IC-GVINS 源码分析]]
- [[2026-04-28-ob_gins-analysis|OB_GINS 源码分析]]
- [[2026-04-28-gtsam_points-analysis|gtsam_points 源码分析]]
- [[组件-slam_fusion_core|slam_fusion_core 骨架库]]
- [[组件-GTSAM|GTSAM ISAM2 增量优化]]
- [[概念-因子图|因子图理论与 MAP 估计]]
- [[数学-流形优化|流形上的优化与 retraction]]
- [[概念-Schur补与边缘化|Schur 补与边缘化理论]]
- [[VIO方案对比|视觉惯性里程计 (VIO)]]
- [[LiDAR数据管线|LiDAR 惯性里程计 (LIO)]]
- [[GNSS数据管线|GNSS 紧耦合定位]]
- [[架构-滑动窗口优化]]
- [[概念-回环检测方法|回环检测与位姿图优化]]
- [[传感器-传感器标定|传感器在线标定]]
- [[架构-多传感器融合架构]]
- [[架构-多传感器融合架构|三模式 SLAM 架构]]
- [[概念-基准测试数据集|EuRoC 基准测试]]
- [[概念-基准测试数据集|KITTI 基准测试]]
- [[组件-GTSAM]]
- [[架构-多传感器融合架构]]
