---
tags: [LiDAR, GTSAM, 因子图]
sources:
  - raw/docs-deep-dive/lio_sam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/lio_sam_analysis.md
---

# LIO-SAM 源码级分析摘要

> 将 IMU 预积分 + ISAM2 因子图引入 LiDAR SLAM，建立两级优化（LM前端 + ISAM2后端）的工程标杆

## 核心发现
- LiDAR 里程计以 `PriorFactor` 形式注入 IMU 预积分图，是**松耦合**设计而非紧耦合（`imuPreintegration.cpp:378`）
- 前端 30 次 LM 迭代（帧-地图）与后端 ISAM2 全局图分离，两级解耦：前端的 `PriorFactor` 作为 IMU 预积分图中的 "correction"
- 退化检测通过 `J^T J` 特征值分解实现：退化维度增大噪声（`correctionNoise2`）并通过投影矩阵 `matP` 修正
- 回环闭合后额外 5 次 `ISAM2::update()` 确保全局 BA 收敛

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | 曲率特征提取（6子区域，edge/surf 各 20 个）+ 帧-地图 LM 优化（点到线/点到面 Jacobian） |
| 后端 | GTSAM ISAM2 增量优化：`PriorFactor<`Pose3`>` + `BetweenFactor<`Pose3`>` + `GPSFactor` + 回环 `BetweenFactor` |
| 回环 | PCL ICP 半径搜索（10m） + 时间差 >30s 过滤 + fitnessScore < 0.3 阈值 |
| 独特创新 | (1) IMU 角速度积分 deskew 校正到帧起始时刻 (2) IMU roll/pitch 经 SLERP 与 LiDAR 融合（权重 1%） (3) 双线程预积分器（优化用 + 高频发布用） (4) 每 100 帧重置 ISAM2 保留边缘协方差 |

## 关键引用 (path:line)
- IMU deskew 逐点校正: `imageProjection.cpp:489-519`
- 前端 corner 优化（点-线距离）: `mapOptmization.cpp:974-1063`
- 前端 surf 优化（点-面距离）: `mapOptmization.cpp:1066-1135`
- 退化检测 + 投影修正: `mapOptmization.cpp:1229-1251`
- IMU roll/pitch 约束 Slerp: `mapOptmization.cpp:1312-1335`
- IMU 预积分因子构造: `imuPreintegration.cpp:350-380`
- ISAM2 重置逻辑: `imuPreintegration.cpp:320-347`
- 回环检测距离: `mapOptmization.cpp:610-643`
- 回环后 5× update: `mapOptmization.cpp:1518-1525`
- 关键帧选取阈值（0.2rad, 1.0m）: `mapOptmization.cpp:1354-1379`

## 相关页面
- [[LiDAR方案对比]]
- [[算法-KISS-ICP]]
- [[2026-04-29-lego_loam-analysis|LeGO-LOAM分析摘要]]