---
tags: [回环检测, NDT, LiDAR]
sources:
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# 多分辨率NDT回环

> Lightning-LM 回环检测/验证方法：四级分辨率 `{10, 5, 2, 1}m` 的 PCL NDT 匹配，逐层精化。

## 核心思想

多分辨率 NDT 回环把历史子地图转成正态分布网格，并从粗到细逐层配准。粗分辨率先提供大范围捕获能力，细分辨率再提高位姿精度。相比直接在原始点云上 ICP，它对初始误差更宽容，也更适合用作回环候选的几何验证。

## 工作流程
1. 欧氏距离候选检测：`|kf_opt_pose - hist_pose|_xy < max_range`（60m），ID 间隔 > 20
2. 构建 submap：候选帧 ±40 帧，每 4 帧取 1（约 20 帧）
3. 四级 NDT 匹配：每层用 `r*0.1` voxel 降采样后在当前分辨率配准，下层继承上层结果
4. 得分筛选：`getTransformationProbability() > ndt_score_th`（1.0）

## vs ScanContext
- 优势：更简单快速，不需要 3D 描述子
- 劣势：对称/重复结构下可能检测不到回环

## 工程边界

该方法依赖已有轨迹给出的空间邻近候选，因此更像“局部回环验证”而不是全局地点识别。若累计漂移已经超过候选半径，欧氏距离筛选会漏掉真实回环；若重复结构很多，NDT 得分也可能给出错误高分。因此通常需要和描述子检索或图一致性检查组合。

## 相关页面

- [[组件-lightning-lm]]
- [[概念-回环检测方法]]
- [[方法-ScanContext]]
- [[LiDAR数据管线]]
