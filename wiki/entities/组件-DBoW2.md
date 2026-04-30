---
tags: [回环检测, 视觉SLAM, 词袋模型]
sources:
  - wiki/sources/2026-04-29-lvi_sam_analysis.md
  - wiki/sources/2026-04-29-orb-slam3-analysis.md
created: 2026-04-28
updated: 2026-04-29
type: entity
---

# DBoW2/DBoW3

> DBoW2/DBoW3 是基于词袋模型（Bag of Words）和 Brief/ORB 描述符的视觉地点识别库，广泛用于视觉 SLAM 系统的回环检测与重定位。

## 概述

DBoW2（及其后继 DBoW3）由 Dorian Galvez-Lopez 开发，是视觉 SLAM 回环检测的事实标准。其核心数据结构包括：

- **词汇树（Vocabulary Tree）**：对大量 Brief/ORB 描述符进行层次 k-means 聚类构建，叶节点为视觉单词（visual word），用于将图像描述为词频向量。
- **直接索引（Direct Index）**：按词汇树节点组织图像的特征点和描述符，加速特征匹配时仅在共享单词的节点间搜索。
- **反向索引（Inverse Index）**：按词汇单词索引所有包含该单词的图像，加速候选图像的快速筛选。

回环检测流程：将当前帧转为词袋向量 → 在数据库中查询相似图像（通过 TF-IDF 或 L1-score）→ 对候选帧进行几何验证（3D-2D 匹配 + RANSAC）。

## 在分析框架中的应用

- **ORB-SLAM3**：使用 DBoW2 进行关键帧级回环检测和全局重定位，配合 Sim3 变换验证几何一致性。
- **VINS-Fusion**：使用 DBoW2 配合 Brief 描述符检测回环，将回环约束作为 `BetweenFactor` 注入位姿图优化。
- **BEV-LSLAM**：在基于鸟瞰图的 LiDAR 语义 SLAM 中，使用 DBoW2 对 BEV 图像的特征进行回环检测。

## 对 SLAM 算法的意义

回环检测模块需要视觉和 LiDAR 双源验证。DBoW2/DBoW3 的词汇树框架可直接用于视觉分支，结合 LiDAR 的 Scan Context 等地点描述符，实现交叉验证回环检测，降低单传感器误检率。此外，DBoW2 的重定位能力可用于 GPS 不可用时的全局定位恢复。

## 相关页面

- [[方法-视觉特征跟踪]]
- [[架构-滑动窗口优化]]
- [[组件-GTSAM]]
---
## (合并自: DBoW2回环检测.md)
---
---
tags: [回环检测, DBoW2, 词袋模型, LVI-SAM]
sources: [wiki/sources/2026-04-29-lvi_sam_analysis.md]
created: 2026-04-29
type: entity
---

# DBoW2回环检测

> LVI-SAM 使用 DBoW2 词袋模型进行视觉回环候选检索，结合 LiDAR ICP 进行精确验证的双重回环检测。

## 两阶段回环
1. **视觉阶段**：DBoW2 在图像关键帧中检索回环候选，输出候选帧分数
2. **LiDAR 验证**：根据时间戳找到对应 LiDAR 关键帧，提取局部点云，执行 PCL ICP 精配准
3. **约束添加**：若 ICP fitness 分数低于阈值 → 添加回环因子到 GTSAM iSAM2 因子图

## 与单一传感器回环的对比
- 视觉速度快（DBoW2 O(1) 查找），精度不足
- LiDAR 精度高（ICP 精配准），速度慢
- 两者结合实现速度与精度的平衡