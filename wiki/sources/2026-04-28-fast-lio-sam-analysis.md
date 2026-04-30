---
tags: [LiDAR, 激光SLAM, fast_lio_sam, 回环检测, 后端PGO]
sources:
  - raw/docs-deep-dive/fast_lio_sam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/fast_lio_sam_analysis.md
---

# FAST-LIO-SAM-SC-QN 源码级分析摘要

> 模块化后端 PGO + 回环检测系统，依赖外部 FAST-LIO2 前端，四层回环验证（ScanContext → 欧式距离 → Quatro → Nano-GICP）。

## 核心发现

- **非完整 LIO 框架**，仅后端 PGO + 回环检测，LIO 前端依赖外部 FAST-LIO2 节点
- 回环检测实现**四层验证管道**：ScanContext 全局描述子检索 → 欧式距离二次过滤 → Quatro (FPFH+GNC) 粗对齐 → Nano-GICP (分布匹配) 精细配准
- 使用 **GTSAM ISAM2** 增量因子图优化，回环后额外 3 次 update 确保收敛
- 后端与前端通过 `odom_delta_` 机制桥接，实时位姿 = 最近全局优化结果 × 后续里程计增量
- 点云存储为 LiDAR 坐标系（从 FAST-LIO2 输出世界坐标逆变换得到）

## 技术要点

| 维度 | 方案 |
|------|------|
| 前端 | 依赖外部 FAST-LIO2（IESKF + ikd-Tree） |
| 特征提取 | 由 FAST-LIO2 完成 |
| 配准方法 | Quatro 全局 + Nano-GICP 精细（回环验证） |
| IMU融合 | 由 FAST-LIO2 紧耦合完成 |
| 后端 | GTSAM ISAM2 增量因子图 |
| 回环 | ScanContext → 欧式距离 → Quatro → Nano-GICP |
| ROS耦合 | ROS1 (Melodic/Noetic)，ApproximateTime 同步 |
| 地图管理 | 关键帧点云 LiDAR 系存储，按需降采样拼接 |

## 关键代码引用

- 核心类: `FastLioSamScQn` (include/fast_lio_sam_sc_qn.h:53)
- 同步回调: `odomPcdCallback()` (src/fast_lio_sam_sc_qn.cpp:87)
- 回环验证: `coarseToFineAlignment()` (src/loop_closure.cpp:138)
- Quatro: `quatro_handler_->align()` (src/loop_closure.cpp:144)
- Nano-GICP: `icpAlignment()` (src/loop_closure.cpp:110)
- ISAM2 更新: (src/fast_lio_sam_sc_qn.cpp:163-170)

## 相关页面

- [[LiDAR方案对比]]
- [[概念-回环检测方法]]
- [[方法-ScanContext|ScanContext描述子]]
- [[方法-GICP配准方法]]
- [[概念-因子图]]