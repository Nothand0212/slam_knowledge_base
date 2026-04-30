---
tags: [open_vins, MSCKF, VIO, 深度分析]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-camera-pipeline-comparison.md
---

# OpenVINS 分析

> MSCKF 类滑动窗口 EKF 滤波器的完整深度分析

## 核心架构

open_vins 以 MSCKF（多状态约束卡尔曼滤波）为核心算法，支持单目/双目/多相机配置，提供三种 IMU 积分模式（离散/RK4/ACI² 解析）。

三种积分模式精度递增：
- **离散积分**：欧拉法，假设加速度/角速度在积分区间内恒定，计算量最小但累积误差最大
- **RK4（四阶龙格-库塔）**：在步长内取 4 个采样点求加权平均，精度显著高于欧拉法，是默认推荐模式
- **ACI²（解析连续时间积分）**：使用闭式解析解，对高动态运动（>1000°/s 角速度）精度最优，但计算量最大

## 特征跟踪

双模式可切换：
- **KLT 光流**：FAST 角点 + 金字塔 KLT 跟踪 + F 矩阵 RANSAC
- **描述子匹配**：ORB 描述子 + KNN 匹配 + Ratio/Symmetry test

100% 依赖 OpenCV 的 FAST + KLT + ORB，自研 Grider_FAST/GRID 网格管理器，CamBase 虚基类支持 pinhole/MEI/equi/KB4 四种相机模型。

- **pinhole**：标准针孔模型 + radtan/equidistant 畸变
- **MEI**：Mei 全向相机模型（通用投影，适合鱼眼 < 180° FOV）
- **equi**：等距投影模型（适合 > 180° FOV 的超广角鱼眼）
- **KB4**：Kannala-Brandt 4 参数鱼眼模型

## 初始化

- **静态初始化**：静止检测 + 加速度均值求重力方向
- **动态初始化**：Ceres 非线性优化（关键帧位姿 + 重力 + bias + 速度 + 特征 3D + 可选内外参）

## 状态估计

```
IMU 传播（EKF 预测步，三模式选一）
  → 位姿克隆 augment_clone
  → MSCKF 更新：零空间投影消除特征坐标 + Chi² 检验
  → EKF 更新：K = P*H^T*(H*P*H^T+R)^-1
  → 边缘化老 clone
  → 高频 fast_state_propagate（IMU 频率里程计）
```

## 关键设计

- **FEJ (First Estimate Jacobian)**：保证线性化点一致性
- **五种特征表示**：XYZ / anchor XYZ / anchor inv depth / MSCKF inv depth / single inv depth

| 表示方式 | 用途 | 参数形式 |
|---------|------|---------|
| XYZ | 标准 3D 特征点 | (x, y, z) |
| Anchor XYZ | 以锚点帧为参考的 3D 坐标 | 锚点帧下的 (x, y, z) |
| Anchor Inv Depth | 以锚点帧为参考的逆深度 | (u, v, 1/d) |
| MSCKF Inv Depth | MSCKF 状态向量中的逆深度 | 直接估计 (1/d) |
| Single Inv Depth | 单帧逆深度（无锚点） | (u, v, 1/d) |
- **ROS-Free 内核**：ov_core 完全独立，ENABLE_ROS=OFF 可剥离
- **支持在线标定**：相机内外参、IMU 内参、g 敏感度、时间偏移

## 适用场景

**适合**：高速无人机 VIO、多相机快速部署、需要在线标定场景

**不适合**：长时间大尺度导航（无回环）、慢速/静止场景、大规模特征地图

## 相关页面

- [[概念-MSCKF]]
- [[概念-MSCKF]]
- [[2026-04-28-open_vins-analysis]]
- [[2026-04-29-open-vins-analysis]]
- [[概念-视觉惯性初始化策略]]
- [[概念-IMU预积分]]
- [[传感器-传感器标定]]