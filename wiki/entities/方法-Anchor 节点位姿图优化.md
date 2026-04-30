---
type: entity
tags: [位姿图优化, 坐标变换, 多会话, GTSAM]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-lt-mapper-analysis.md
---

# Anchor 节点位姿图优化

> 多会话建图中的坐标系对齐模式：为每个会话引入 anchor 位姿，让局部轨迹在统一中心坐标系下联合优化。

## 定义

lt-mapper 的核心创新机制：每个会话引入一个 anchor 节点，通过四变量因子 `BetweenFactorWithAnchoring<Pose3>` 将各会话独立的局部坐标系变换到统一中心坐标系后进行联合位姿图优化。

## 核心特征

- 四变量因子 (key1, key2, anchor_key1, anchor_key2)
- 误差计算：hx1 = anchor_p1 ⊕ p1（局部→中心），hx2 = anchor_p2 ⊕ p2，e = Log(measured⁻¹ · (hx1 ⊖ hx2))
- 对四个变量均解析计算雅可比，使用 GTSAM traits<T>::Compose/Between 自动推导
- 中心会话 anchor 强先验 (priorNoise=1e-12)，查询会话 anchor 弱先验 (largeNoise: π²/1e8)
- 优势：避免逐步 ICP 配准漂移累积，所有会话在统一坐标系联合优化

## 相关页面

- 实现于：[[算法-lt-mapper]] `BetweenFactorWithAnchoring.h:18-127`
- [[概念-位姿图优化]]
- [[架构-坐标系管理]]
- [[概念-因子图]]
- [[组件-GTSAM]]
