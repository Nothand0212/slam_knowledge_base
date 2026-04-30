---
tags: [论文索引, SLAM, VIO, 激光SLAM, 深度学习SLAM]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-top/paper_notes.md
sources:
  - raw/docs-top/paper_notes.md
---

# 论文与资料索引

> 仓库中 25+ 篇 SLAM/VIO 相关论文的创新点提炼与本地 PDF 索引

## 核心论文清单

### VIO / 视觉惯性系
- **OpenVINS** (ICRA 2020)：开源 MSCKF 滑动窗口 VIO 平台，支持在线标定
- **SchurVINS** (CVPR 2024)：Schur 补等价观测 + 轻量 EKF
- **VINS-Mono** (TRO 2018)：紧耦合单目 VIO，滑动窗口 + 回环 + 全局 PGO
- **ORB-SLAM3** (TRO 2021)：ORB 特征多地图系统，Atlas 管理

### 直接法 / 半直接法
- **DSO** (TPAMI 2018)：稀疏直接法光度 BA，逆深度参数化
- **DM-VIO** (RA-L 2022)：延迟边缘化策略
- **SVO Pro** (TRO 2017)：半直接法 VIO
- **ROVIO** (ICRA 2015)：基于直接法 EKF 的 VIO

### LiDAR 系
- **LIO-SAM** (IROS 2020)：紧耦合激光-惯性里程计因子图
- **FAST-LIO2** (TRO 2022)：直接法激光-惯性，ikd-Tree
- **CT-ICP** (ICRA 2022)：连续时间弹性配准
- **KISS-ICP** (RA-L 2023)：极简 ICP 里程计

### 深度学习 SLAM
- **DROID-SLAM** (NeurIPS 2021)：RAFT 光流 + Dense BA
- **NICE-SLAM** (CVPR 2022)：层次化神经隐式
- **MonoGS** (CVPR 2024)：3D Gaussian Splatting SLAM

### 多传感器融合
- **R3LIVE** (2022)：紧耦合 LiDAR-惯性-视觉 + RGB 重建
- **FAST-LIVO2** (TRO 2024)：统一直接法 LIV 里程计
- **LVI-SAM** (ICRA 2021)：LIS + VIS 子系统互补

### GNSS / 组合导航
- **IC-GVINS** (RA-L 2022)：INS 为中心的 GNSS-视觉-惯性紧耦合
- **OB_GINS**：优化型 GNSS/INS/里程计组合导航

## 关键创新模式

1. **从滤波到因子图**：MSCKF → swBA → iSAM2 的演进
2. **从间接法到直接法**：光度误差替代重投影误差
3. **从单传感器到多传感器**：LiDAR + 视觉 + 惯性 + GNSS 紧耦合

## 相关页面

- [[VIO方案对比]]
- [[LiDAR方案对比]]
- [[概念-直接法vs间接法]]
- [[因子图vs滤波]]
- [[概念-深度学习SLAM]]
- [[架构-多传感器融合架构]]