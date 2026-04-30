---
tags: [VIO, 视觉惯性里程计, MSCKF, EKF, 滤波]
sources:
  - raw/docs-deep-dive/msckf_vio_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/msckf_vio_analysis.md
---

# MSCKF_VIO 源码级分析摘要

> 经典 MSCKF 论文的参考实现，双 Nodelet 架构（image_processor + vio），要求静止初始化，无回环。

## 核心发现

- 双 ROS nodelet 组成流水线：`image_processor` 负责特征提取跟踪，`vio` 负责 EKF 状态估计
- 前端使用 OpenCV FAST 角点 + KLT 金字塔光流，grid 策略（4x5=20 格，每格 3-4 个特征）
- 自写 2 点 RANSAC：利用 IMU 已知旋转做纯平移模型外点剔除（非 OpenCV RANSAC）
- 初始化严格依赖静止状态：取 200 条 IMU 的加速度均值估计重力方向，陀螺仪偏置取角速度均值
- 实现了 **OC-MSCKF**（可观测性约束）修正，保证不可观子空间维度正确
- IMU 传播使用 RK4 积分 + 3 阶矩阵指数离散化，协方差传播含跨状态互协方差
- MSCKF 更新使用 Eigen JacobiSVD 做零空间投影，SuiteSparse SPQR 做大型 H 矩阵 QR 压缩
- 滑动窗口 max_cam_state_size=20，基于旋转/平移/跟踪率三阈值自适应删除冗余相机状态
- 无全局优化和回环检测，仅通过 onlineReset（位置标准差 > 8m 时重置）防止发散

## 技术要点

| 维度 | 方案 |
|------|------|
| 特征提取 | FAST 角点（OpenCV），4×5 网格均匀分布 |
| 跟踪方法 | KLT 金字塔光流 + 自写 2 点 RANSAC |
| 后端 | MSCKF EKF 滤波 + OC 可观测性约束 |
| 回环 | 无 |
| ROS 耦合 | 深度耦合（Nodelet 架构，ROS 消息通信） |
| 在线标定 | 支持 IMU-Camera 6DOF 外参在线估计 |

## 关键代码引用

- 特征跟踪入口: `ImageProcessor::trackFeatures` (src/image_processor.cpp:417-607)
- 2 点 RANSAC: `twoPointRansac` (src/image_processor.cpp:987-1230)
- EKF 状态传播: `predictNewState` (src/msckf_vio.cpp:630-692)
- MSCKF 零空间投影: `featureJacobian` (src/msckf_vio.cpp:862-916)
- OC 转移矩阵修正: `processModel` (src/msckf_vio.cpp:580-597)

## 相关页面

- [[VIO方案对比]]
- [[概念-MSCKF]]
- [[概念-MSCKF|可观测性约束OC-MSCKF]]
- [[概念-三角化与深度估计|双目视觉前端设计]]- [[概念-MSCKF]]
- [[概念-IMU预积分]]
