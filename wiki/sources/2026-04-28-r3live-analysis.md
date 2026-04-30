---
tags: [SLAM, LiDAR-视觉-惯性, RGB着色]
sources:
  - raw/docs-deep-dive/r3live_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/r3live_analysis.md
---

# R3LIVE 源码级分析摘要

> 双ESIKF架构实现LiDAR-惯性-视觉紧耦合，首创实时RGB着色LiDAR点云+在线标定（内参/外参/时间偏移），支持MVS离线Mesh重建

## 核心发现
- 双ESIKF并行架构：LIO ESIKF（18维，点到平面+iKd-Tree）+ VIO ESIKF（29+维，重投影+LK光流），两者通过`g_lio_state`共享状态
- VIO以LIO估计为先验起点，IMU预积分传播到图像时刻，视觉失败时优雅降级回LIO
- RGB着色链路完整：LIO点云→全局RGB地图（多视图观测+RGB协方差）→实时RGB点云发布→可选MVS离线Mesh
- VIO可同时在线估计相机内参、IMU-相机外参、时间偏移，通过扩展状态维度和Huber loss实现

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | LiDAR: iKd-Tree最近邻+PCA平面拟合；视觉: LK光流跟踪RGB地图点 |
| 后端 | LIO ESIKF（18维）+ VIO ESIKF（29+维，含外参/内参/时间偏移），可选光度误差 |
| 独特创新 | 实时RGB着色重建+在线标定+LIO-as-Prior降级机制+双线程并行 |

## 关键引用
- 多线程架构: `r3live.cpp:92-101`
- LIO IESKF迭代: `r3live_lio.cpp:836-893`
- VIO ESIKF迭代: `r3live_vio.cpp:607-772`
- VIO预积分为先验: `r3live_vio.cpp:549-589`
- RGB点云着色: `r3live_vio.cpp:959-1037`
- 在线标定状态扩展: `r3live_vio.cpp:157-173`
- iKd-Tree搜索: `r3live_lio.cpp:688`

## 相关页面
- [[2026-04-28-fast-livo2-analysis]]
- [[2026-04-28-lvi-sam-analysis]]