---
tags: [传感器融合, ESIKF, R3LIVE, 紧耦合]
sources:
  - wiki/sources/2026-04-29-r3live_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# 双ESIKF架构

> R3LIVE 核心架构：LIO ESIKF（18 维）和 VIO ESIKF（29+维）两个独立估计器并行运行，通过共享状态传递先验。

## 设计动机

双 ESIKF 架构把 LiDAR-IMU 和 Visual-IMU 估计分成两个滤波器，使每条传感器链路可以独立开发、调试和降级。LIO 负责稳定几何定位，VIO 负责视觉纹理和 RGB 地图更新，二者通过共享状态和先验传递协同。

## 架构特点
- **LIO ESIKF**：点到平面 ICP，iKd-Tree 最近邻，Fast-LIO2 基础
- **VIO ESIKF**：重投影误差 + LK 光流 + 可选光度误差，扩展外参/内参/时间偏移
- **并行线程**：两个 ESIKF 在不同线程中各自运行，互斥锁保护 `g_lio_state`

## vs 统一IESKF
- **优点**：模块化，可独立调试优化，可并行加速
- **缺点**：协方差不一致，信息可能重复或丢失
- R3LIVE 证明了双 KF 架构可行性，FAST-LIVO2 用统一 IESKF 解决了协方差一致性问题

## 风险点

两个滤波器各自维护协方差时，很难严格表达同一 IMU、同一位姿先验被使用了几次。若 LIO 和 VIO 都对同一状态给出强约束，后端可能过度自信。双 ESIKF 因此更适合强调工程解耦的系统；若目标是最优统计一致性，统一滤波或因子图会更自然。

## 相关页面

- [[算法-R3LIVE]]
- [[方法-统一IESKF融合]]
- [[方法-IESKF滤波器]]
- [[架构-多传感器融合架构]]
