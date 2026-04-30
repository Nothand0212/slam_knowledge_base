---
tags: [事件相机, 视觉SLAM, 立体VO, 异步传感器]
sources:
  - wiki/sources/2026-04-29-esvo-analysis-analysis.md
  - wiki/sources/2026-04-29-special-sensors-comparison.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# ESVO

> ESVO（Event-based Stereo Visual Odometry）是首个基于事件相机的双目视觉里程计系统，利用事件的异步时间表面进行匹配、跟踪和深度估计，实现高速运动下的鲁棒定位。

## 核心方法
ESVO由两个并行的异步管道组成：时间表面构建与立体匹配。事件相机输出异步亮度变化脉冲而非图像帧。ESVO将事件流累积为时间表面（Time Surface）——每个像素存储最近事件的时间戳，表面梯度编码纹理边缘。立体匹配在时间表面间寻找对应，通过互相关法计算视差和深度。里程计跟踪新事件到重建的深度地图，用ICP配准求解位姿。

## 关键设计
- 时间表面：编码事件时序信息为可配准的几何表示
- 异步深度估计：从事件流实时更新深度地图，无需固定帧率
- 高动态范围：事件相机对极暗和极亮区域均有效
- 低延迟运动跟踪：微秒级事件响应，适合高速运动场景

## 工程边界

事件相机只在亮度变化处触发，因此静止场景或低纹理区域事件稀少。ESVO 依赖双目事件相机的时间同步和外参标定，且时间表面的质量受事件噪声、阈值和运动速度影响。它适合作为高速/高动态范围视觉里程计方案，但不是普通帧相机 SLAM 的直接替代品。

## 相关页面
- [[2026-04-28-esvo-analysis]]
- [[VIO方案对比]]
- [[概念-连续时间轨迹]]
- [[方法-Time-Surface]], [[传感器-事件相机]]
- [[2026-04-29-external-primary-source-check]]
