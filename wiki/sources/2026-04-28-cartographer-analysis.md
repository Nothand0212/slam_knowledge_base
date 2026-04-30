---
tags: [LiDAR, 工业级, 子地图]
sources:
  - raw/docs-deep-dive/cartographer_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/cartographer_analysis.md
---

# Cartographer 源码级分析摘要

> Google 工业级 2D/3D LiDAR SLAM：Local+Global 双层架构 + 子地图冻结不变设计 + Branch-and-Bound 回环检测，架构教科书

## 核心发现
- **双层 SLAM 架构**：Local SLAM（实时 scan-to-submap 匹配）+ Global SLAM（后台 Pose Graph 优化与回环），通过 ThreadPool 异步分离，回环不阻塞实时定位
- **子地图系统**是其灵魂：始终维护 2 个活跃子地图（old + new），冻结后子地图变为不可变 `const` 共享指针，天然线程安全
- **Branch-and-Bound 回环**：预计算多分辨率概率网格栈（金字塔），BnB 在 O(n log n) 内完成全局搜索，远优于暴力匹配
- 概率网格用 log-odds 表示 + 查找表加速更新（O(1)），`CompressedPointCloud` 压缩存储历史节点点云用于回环

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | 两步匹配（RealTimeCorrelativeScanMatcher 暴力搜索窗口 + CeresScanMatcher 非线性精化）+ MotionFilter 剔除静止帧 + 位姿外推器提供初始猜测 |
| 后端 | Ceres 全局优化：子地图内约束 + 回环约束 + 里程计约束 + IMU 约束，每 90 节点触发优化 |
| 回环 | **Branch-and-Bound** 全子地图相关扫描匹配（多分辨率预计算网格栈，depth=7）+ min_score=0.55 阈值过滤 |
| 独特创新 | (1) 子地图冻结后不可变 → 免锁并发 (2) BnB 回环检测（Olson 算法） (3) Lua + Protobuf 配置驱动，参数热加载 (4) 原生多轨迹并行 + 跨轨迹融合 (5) 完整状态序列化/恢复（含网格数据） (6) 内置 metrics 监控系统 |

## 关键引用 (path:line)
- MapBuilder 顶层架构: `map_builder.h:33-96`
- 活跃子地图管理（双地图策略）: `submap_2d.h:74-102`
- 概率网格 log-odds 更新: `submaps.h:37-39`
- 概率值量化存储: `probability_values.h:31-44`
- 两步匹配策略: `local_trajectory_builder_2d.h`
- RealTimeCorrelativeScanMatcher 搜索窗口: Lua config
- FastCorrelativeScanMatcher（BnB 核心）: `fast_correlative_scan_matcher_2d.h:127-165`
- 预计算网格栈: `fast_correlative_scan_matcher_2d.h:95-109`
- ConstraintBuilder 异步约束构建: `constraint_builder_2d.h:60-170`
- 序列化与状态管理: `map_builder.h:55-63`

## 相关页面
- [[LiDAR方案对比]]
- [[算法-LIO-SAM]]
- [[算法-KISS-ICP]]