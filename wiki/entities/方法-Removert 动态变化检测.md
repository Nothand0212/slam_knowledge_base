---
type: entity
tags: [动态物体, 变化检测, 点云, range-image]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-lt-mapper-analysis.md
---

# Removert 动态变化检测

> 长期 LiDAR 建图中的变化检测方法，通过 range image 比较区分移动物体、消失结构和新增静态结构。

## 定义

lt-mapper/LT-removert 模块中实现的跨会话点云变化检测算法。通过 range image 投影 + 前后帧 occupancy 比较，移除移动物体并识别新增/消失的静态物体。

## 核心特征

- **高动态点移除**：逐帧 range image 投影 → occupancy 状态比较 → 前一帧占据但后一帧消失 → 标记为 dynamic（移动物体）
- **低动态/变化检测**：跨会话比较 → 识别新增/消失区域 → 生成 Delta Map 用于增量更新
- 核心洞察：range image 比 3D 体素更容易判断"视角遮挡"关系
- 输出：静态地图（不含移动物体）+ 变化检测图

## 为什么用 range image

Range image 保留了 LiDAR 的视线顺序，可以判断某个点是被遮挡、消失，还是被新的近处物体替代。纯 3D 体素只看到空间占用差异，很难区分“后方结构被前景遮挡”和“后方结构真的消失”。这对长期地图更新尤其重要。

## 工程边界

变化检测依赖相同区域的重复观测和较好的位姿对齐。若跨会话地图配准误差较大，静态结构也会被误判为变化。动态物体密集、雨雪噪声和稀疏远距离点云也会降低 range image 比较的可靠性。

## 相关页面

- 实现于：[[算法-lt-mapper]] LT-removert `Removerter.cpp`
- [[方法-体素地图]]
- [[算法-lt-mapper]]
- [[概念-回环检测方法]]
- [[LiDAR数据管线]]
