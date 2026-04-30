---
tags: [LiDAR, BEV, ORB, 回环, GMS]
sources:
  - raw/docs-deep-dive/bev_lslam_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/bev_lslam_analysis.md
---

# BEV-LSLAM 深度源码分析

> IEEE RA-L 2025：3D LiDAR→2D BEV 图像→ORB 特征→GMS 匹配，双模态高度+强度通道

## 摘要
BEV-LSLAM 将 LiDAR 点云投影为 200×200 像素鸟瞰图（高度图 + 强度图），然后直接复用 ORB-SLAM 的特征匹配管线。后端用 Fast-VGICP 精配准 + DBoW3 回环 + 4DoF 位姿图优化。

## 关键概念
- **[[算法-BEV-LSLAM]]**：双节点架构，scan2scan 前端 + scan2map 后端，纯 LiDAR 无 IMU
- **[[方法-BEV投影]]**：点云→200×200 鸟瞰图，高度图取最大值 + 地面共面增强，强度图差分增强
- **[[方法-GMS匹配验证]]**：Grid-based Motion Statistics 快速几何验证，利用运动平滑性滤除外点
- **[[方法-Fast-VGICP]]**：体素化 GICP 扫描-子地图稠密配准，14 线程并行

## 相关页面
- [[算法-Cartographer]]
- [[算法-ORB-SLAM3|ORB-SLAM3]]
- [[概念-回环检测方法]]