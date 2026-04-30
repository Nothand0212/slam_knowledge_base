---
tags: [LiDAR, LOAM, 地面优化]
sources:
  - raw/docs-deep-dive/lego_loam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/lego_loam_analysis.md
---

# LeGO-LOAM 源码级分析摘要

> 将地面语义先验引入 LiDAR SLAM 前端——地面点用于平面特征，非地面点用于边缘特征，配合两步解耦优化取得极高效率

## 核心发现
- **地面分割**是核心创新：相邻线束垂直角判断地面点（`imageProjection.cpp:260-310`），仅检视下方 7 条线，阈值 10°
- **两步解耦优化**（`featureAssociation.cpp:1666-1695`）：Step1 仅用地面平面特征优化 roll/pitch/ty → Step2 仅用非地面边缘特征优化 yaw/tx/tz
- 每步求解 3×3 矩阵，阻尼因子固定为 0（并非真正的 Levenberg-Marquardt），计算量远低于原始 LOAM 的 6-DOF 全优化
- 特征提取利用地面/非地面标签天然引导分类：边缘特征**严格排除地面点**，平面特征**严格限定地面点**

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | 地面分割 + 角度阈值聚类 + 6子区域曲率特征提取（edge/surf）+ 两步解耦优化（3×3 矩阵求解） |
| 后端 | GTSAM ISAM2：`PriorFactor<`Pose3`>` + `BetweenFactor<`Pose3`>`，噪声旋转 1e-6 / 平移 1e-8 |
| 回环 | PCL ICP（半径 7m 搜索 + 时间差 >30s）+ 融合 ±25 相邻关键帧形成局部子图 + fitnessScore < 0.3 验证 |
| 独特创新 | (1) 地面分割驱动特征语义分类 (2) 两步分离优化变量（地面约束 roll/pitch，边缘约束 yaw/tx/tz） (3) BFS 聚类用预分配数组替代 std::queue 避免动态分配 (4) IMU roll/pitch 指数平滑融合：`0.998*odom + 0.002*imu` |

## 关键引用 (path:line)
- 地面分割算法: `imageProjection.cpp:260-310`
- 点云聚类 BFS 分割: `imageProjection.cpp:312-460`
- 曲率计算: `featureAssociation.cpp:621-641`
- 边缘特征提取（排除地面点）: `featureAssociation.cpp:701-733`
- 平面特征提取（仅地面点）: `featureAssociation.cpp:736-769`
- 两步解耦优化: `featureAssociation.cpp:1666-1695`
- surf 优化（3×3 求解）: `featureAssociation.cpp:1270-1377`
- corner 优化（3×3 求解）: `featureAssociation.cpp:1379-1478`
- 回环检测: `mapOptmization.cpp:814-872`
- iSAM2 因子图: `mapOptmization.cpp:1355-1453`

## 相关页面
- [[LiDAR方案对比]]
- [[算法-LIO-SAM]]
- [[LiDAR数据管线|原始LOAM分析]]