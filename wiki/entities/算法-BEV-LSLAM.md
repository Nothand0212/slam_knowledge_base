---
tags: [LiDAR, BEV, ORB, 激光SLAM]
sources:
  - wiki/sources/2026-04-29-bev_lslam_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# BEV-LSLAM

> IEEE RA-L 2025：3D LiDAR 点云→2D BEV 图像→ORB 特征→GMS 匹配，后端 DBoW3 回环 + 4DoF PGO。

## 架构
- **双 ROS 节点**：scantoscan_kitti（scan2scan 前端）+ scantomap_kitti（scan2map 后端）
- **BEV 投影**：200×200 像素（80m×80m×0.4m/pixel），高度图+强度图双通道
- **匹配管线**：BFMatcher + GMS 几何验证 + 协方差特征分类（线/面）+ Ceres 优化

## 后端回环
- 两种回环模式：运动距离回环 + DBoW3 词袋回环（**GMS**=基于网格运动统计的特征匹配验证，利用邻域运动一致性过滤误匹配；**findHomography**=OpenCV 的单应矩阵估计，进一步几何验证）
- PoseGraphOptimize4DoF：XYZ+Yaw 优化（地平面假设）
- Fast-VGICP scan-to-submap 精配准

## 设计价值

BEV-LSLAM 的思路是把 LiDAR 地点识别问题转成图像匹配问题，从而复用 ORB、GMS、DBoW3 和单应验证这套成熟视觉工具链。它适合地面车辆场景，因为主要运动可近似为平面上的 4DoF；高度图和强度图则分别提供几何和反射信息。

## 工程边界

BEV 投影会丢失垂直多层结构，4DoF 后端也默认 roll/pitch 变化较小。因此该方案更适合道路或园区地面机器人，不适合楼梯、多层立交、无人机或大坡度越野场景。最终仍需要 Fast-VGICP 做 3D 精配准，不能只依赖 2D 图像匹配。

## 相关页面

- [[方法-BEV投影]]
- [[方法-GMS匹配验证]]
- [[方法-Fast-VGICP]]
- [[LiDAR方案对比]]
