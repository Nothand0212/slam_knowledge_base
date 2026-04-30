---
tags: [LiDAR, 连续时间, ICP]
sources:
  - raw/docs-deep-dive/ct_icp_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/ct_icp_analysis.md
---

# CT-ICP 源码级分析摘要

> 连续时间轨迹参数化解决Scan内畸变，12自由度（begin/end双pose线性+Slerp插值），Ceres多线程求解，鲁棒配准+运动模型先验实现高速运动下的高精度LiDAR里程计

## 核心发现
- 核心创新：不将一帧LiDAR建模为单时刻6-DOF位姿，而是begin/end双pose（12 DoF），帧内每个点按时间戳插值，原生补偿scan内运动畸变
- 插值方案：平移线性插值 + 旋转球面线性插值（Slerp），比B-Spline高效（仅2个控制点），比GP实用
- 运动模型作为Ceres正则化约束：位置一致性（相邻帧连续）、常速度先验、朝向一致性、小速度惩罚，几何不足时稳定估计
- 鲁棒配准自适应：Registration失败时自动IncreaseRobustnessLevel（缩小体素、扩大邻域），拒绝大旋转帧的地图插入
- 四种几何残差可选（点到平面/点/线/分布），支持Cauchy/Huber/Truncated loss，平面性+邻域距离组合加权

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | 体素下采样+多线程邻域搜索，PCA计算planarity(a2D)和法向量，多种距离度量 |
| 后端 | Ceres求解12-DOF连续时间配准（GN/Ceres/Robust三选一），运动模型正则化，Robust Registration自适应 |
| 独特创新 | 连续时间12-DOF参数化+线性/Slerp插值+抽象运动模型接口+自适应鲁棒配准策略 |

## 关键引用
- TrajectoryFrame定义: `types.h:31-61`
- CTFunctor连续时间Cost Function: `cost_functions.h:186-223`
- 运动补偿模式枚举: `odometry.h:12-17`
- 运动模型约束添加: `motion_model.cpp:12-61`
- Robust Registration: `odometry.cpp:780-852`
- 地图插入策略: `odometry.cpp:855-953`
- ICP_DISTANCE/WEIGHTING枚举: `cost_functions.h:22-28`, `ct_icp.h:49-54`
- POINT_TO_DISTRIBUTION马氏距离: `cost_functions.h:141-183`

## 相关页面
- 纯LiDAR里程计代表，与FAST-LIVO2（LiDAR+视觉+IMU）、ESVO（事件相机）等形成传感器谱系对比