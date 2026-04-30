---
tags: [SLAM, 事件相机, 立体]
sources:
  - raw/docs-deep-dive/esvo_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/esvo_analysis.md
---

# ESVO 源码级分析摘要

> 基于事件相机的立体视觉里程计，异步事件流→Time Surface表示→ZNCC块匹配→逆向组合法（Inverse Compositional）100Hz+位姿跟踪

## 核心发现
- Time Surface是核心数据结构：将异步事件流（x,y,t,polarity）转为2D图像，每个像素存最近事件时间戳，支持高斯平滑和Negative翻转
- 立体匹配：事件驱动Block Matching（patch 25×25，ZNCC代价），粗搜（大步长）+精搜（逐像素）两级搜索，4线程并行
- Tracking使用逆向组合法：3D点投影到Time Surface对比模板patch光度差异，解析雅可比大幅加速，实现100Hz+
- Mapping管线：EventBM匹配→DepthProblem非线性逆深度优化（Student-t分布鲁棒）→DepthFusion跨帧融合（卡方/Student-t兼容性检验）

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | Time Surface生成+GaussianBlur/Sobel梯度；事件驱动ZNCC块匹配（patch 25×25, disp 1-40）|
| 后端 | 逆向组合法位姿优化（6-DOF李代数）+ DepthProblem逆深度优化+DepthFusion统计融合 |
| 独特创新 | 事件相机的完整SLAM pipeline+逆向组合法解析雅可比+Student-t鲁棒深度估计+三线程异步架构 |

## 关键引用
- Time Surface定义: `TimeSurfaceObservation.h:27-157`
- 单事件匹配流程: `EventBM.cpp:80-168`
- ZNCC代价函数: `EventBM.cpp:317-333`
- Registration Problem: `RegProblemLM.h:77-150`
- Tracking主循环: `esvo_Tracking.cpp:79-200`
- Mapping管线: `esvo_Mapping.h:43-223`
- DepthFusion跨帧融合: `DepthFusion.h:16-70`
- 多线程匹配: `EventBM.cpp:299-315`

## 相关页面
- 作为事件相机SLAM的唯一代表，与传统帧相机/IMU/LiDAR方案互补