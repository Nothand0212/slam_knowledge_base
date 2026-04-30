---
tags: [VIO,ImgUpdate.hpp (evalInnovation/jacState), ImuPrediction.hpp (evalPrediction), MultilevelPatchAlignment]
sources:
  - raw/docs-deep-dive/rovio_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/rovio_analysis.md
---

# ROVIO 源码级分析摘要

> ROVIO 图像块直接法视觉惯性里程计

## 核心发现

- 路标 robotcentric 参数化（bearing vector + 深度），4 种深度参数化（普通/逆/对数/双曲）
- 光度块对齐使用多层级逆组合法 + IMU 运动学 warp 预测，适应大运动跟踪
- IEKF 候选生成沿协方差特征向量方向采样，extraOutlierCheck 做辨别性检查
- 多相机支持（nCam 模板）+ 跨相机测量（useCrossCameraMeasurements）+ 在线外参标定
- [直接法,EKF,VIO,图像块]

## 关键代码引用

- [[算法-ROVIO]] [[概念-直接法光度误差|图像块直接法]] [[概念-MSCKF|Robotcentric参数化]] [[概念-直接法光度误差|光度块对齐]] [[方法-IESKF滤波器|IEKF]]

## 相关页面


