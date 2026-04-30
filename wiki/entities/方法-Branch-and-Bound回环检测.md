---
tags: [回环检测, 扫描匹配, Cartographer]
sources:
  - wiki/sources/2026-04-29-cartographer_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# Branch-and-Bound回环检测

> Cartographer 基于多分辨率预计算网格栈 + BnB 搜索的全子地图回环检测。

## 核心思想

Branch-and-Bound 回环检测把 scan-to-submap 匹配看成离散搜索问题。系统在有限的平移和旋转窗口内枚举候选位姿，但不逐一精算所有候选，而是用多分辨率概率网格给每个搜索分支计算“最高可能得分”。如果一个分支的上界已经低于当前最优分数，就直接剪枝。

## 工作流程
1. **预计算网格栈**：对每个子地图预计算多分辨率概率网格金字塔，每层存储滑动窗口最大概率
2. **BnB 搜索**：在最粗分辨率生成候选集合，逐层向下分支和剪枝，用父节点最大可能分数剪枝
3. **约束验证**：搜索窗口 ±7m、±30°，深度 7，最低匹配分数 0.55

## 算法特点
- 复杂度 O(n log n)，远优于暴力搜索的 O(n²)
- 基于 Olson "Real-Time Correlative Scan Matching" 论文
- 全局重定位使用更高阈值（min_score=0.6）

## 工程边界

该方法适合 Cartographer 的概率子地图，因为占用概率天然能给出候选位姿评分。它的代价与搜索窗口、角度分辨率和子地图数量直接相关；窗口过大会变慢，窗口过小会漏掉大漂移回环。最终约束仍需加入后端位姿图，由全局优化验证一致性。

## 相关页面

- [[算法-Cartographer]]
- [[方法-概率占据网格]]
- [[方法-子地图]]
- [[概念-回环检测方法]]
- [[2026-04-29-external-primary-source-check]]
