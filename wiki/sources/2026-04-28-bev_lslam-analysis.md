---
tags: [SLAM, LiDAR, BEV, ORB特征, 回环检测, Ceres]
sources:
  - raw/docs-deep-dive/bev_lslam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/bev_lslam_analysis.md
---

# BEV-LSLAM 源码分析 摘要

> 将 3D LiDAR 点云投影为 2D 鸟瞰图，利用 ORB 视觉特征进行帧间匹配的紧凑型 LiDAR SLAM，双节点架构 + DBoW3 回环。

## 核心发现

- **核心创新**：3D 点云 → 高度/强度双通道 BEV 图像（200×200）→ ORB 特征提取 → 帧间匹配，借鉴 ORB-SLAM 管线
- **双节点架构**：scantoscan_kitti (前端 scan2scan) + scantomap_kitti (后端 scan2map + 回环)
- **双模态互补**：高度图捕捉几何结构（建筑/地面），强度图捕捉纹理变化（路标/车道线），GMS 网格运动统计滤除外点
- **BEV 投影**：分辨率 0.4m/px，80m×80m 范围，高度/强度均取 max 值，直方图均衡化 + 高斯模糊
- **特征分类**：通过局部协方差特征值比 (λ₂ > 3·λ₁ → 线特征) 区分线/面特征，使用不同 Ceres 因子
- **局部滑动窗口 BA**：10 帧窗口，匹配链式追踪多帧对应关系，支持 height-only / intensity-only / joint
- **两种回环模式**：(1) 运动距离 KDTree 搜索 + ICP 验证 (2) DBoW3 词袋 + GMS 验证 + P2P RANSAC
- **4DoF 位姿图优化**：仅优化 XYZ + Yaw (地平面假设)，SPARSE_NORMAL_CHOLESKY 求解

## 技术要点

| 维度 | 方案 |
|------|------|
| BEV 尺寸 | 200×200 px (80m / 0.4m) |
| 特征 | ORB (1500 前端 / 2500 后端) + GMS |
| 帧间匹配 | BFMatcher → GMS 滤波 → 特征分类 → Ceres |
| 精配准 | Fast-VGICP (0.5m resolution, 25 iters) |
| 回环 | 运动距离 + DBoW3 词袋，双验证 |
| PGO | 4DoF (XYZ+Yaw), Ceres SPARSE_NORMAL_CHOLESKY |
| 框架 | ROS catkin, PCL, OpenCV, Ceres, fast_gicp, DBoW3 |

## 关键引用

- BEV 投影: `scantoscan_kitti.cpp:261-287`
- ORB 提取: `scantoscan_kitti.cpp:311-316`
- 特征分类: `scantoscan_kitti.cpp:448-491`
- 自定义 Ceres 因子: `lidarFactor.hpp:9-529`
- 位姿图优化: `scantomap_kitti.cpp:2354-2527`

## 相关页面

- [[方法-BEV投影|BEV 表征在 SLAM 中的应用]]
- [[方法-视觉特征跟踪]]
- [[方法-GMS匹配验证|GMS 匹配验证]]
- [[概念-回环检测方法|DBoW3 词袋回环]]
- [[概念-位姿图优化|4DoF 位姿图优化]]
- [[LiDAR数据管线|LiDAR SLAM 分类]]- [[算法-Cartographer]]
- [[概念-回环检测方法]]
