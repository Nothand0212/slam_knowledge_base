---
tags: [VIO,EventBM::match_an_event (EventBM.cpp), TimeSurface (TimeSurface.cpp), RegProblemLM]
sources:
  - raw/docs-deep-dive/esvo_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/esvo_analysis.md
---

# ESVO 源码级分析摘要

> ESVO 事件相机立体视觉里程计

## 核心发现

- Time Surface：指数衰减时间表面将异步事件流转换为 2D 表示
- 立体匹配用粗+精搜索 ZNCC block matching，25×25 patch，4 线程并行
- Tracking 用逆向组合法（Inverse Compositional）解析雅可比实现 100Hz+ 实时跟踪
- 三线程架构：Time Surface 生成 + Mapping（立体匹配/深度优化/融合）+ Tracking
- [事件相机,立体视觉,直接法,时间表面]

## 关键代码引用

- [[算法-ESVO]] [[传感器-事件相机]] [[方法-Time-Surface|Time Surface]] [[概念-三角化与深度估计|ZNCC立体匹配]] [[概念-直接法视觉里程计|逆向组合法]]

## 相关页面


