---
tags: [LiDAR SLAM, 崎岖地形, 旋转优化, GTSAM, ROLO, RotVGICP]
sources:
  - raw/docs-deep-dive/rolo_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/rolo_analysis.md
---

# ROLO-SLAM 深度源码分析

> JFR 2025，地面车辆不平平坦地形 LiDAR-Only SLAM。核心创新为分离式旋转/平移估计 + 连续时间平移约束，四个 ROS1 节点管道完整实现。

## 摘要

ROLO-SLAM 专为越野/山地/矿区等崎岖地形设计。(1) [[方法-RotVGICP]]在 SO(3) 流形上直接优化旋转；(2) [[方法-分离式旋转-平移估计]]先平移预对齐再旋转匹配，显著提升垂直精度；(3) [[方法-地面车辆运动约束]]通过 roll/pitch/z 限幅防止发散；(4) LOAM 风格特征提取 + GTSAM ISAM2 后端 + 距离回环检测。

## 核心概念

- **分离式位姿**：平移插值 --> RotVGICP 旋转 --> 连续时间 CT 平移，三步解耦
- **RotVGICP**：VmfVoxelMap 体素化 + 协方差预计算，SO(3) 高斯-牛顿
- **地面约束**：constraintTransformation 限幅 + 强 roll/pitch 先验 PriorFactor
- **退化检测**：LM 中海森特征值 < 100 --> matP = V^(-1) x V' 投影限制方向
- **纯 LiDAR**：无 IMU 紧耦合，可选里程计仅辅助去畸变
- 缺陷：回环仅距离搜索（无描述子）、前端融合策略注释掉

## 相关页面

- [[算法-ROLO-SLAM]]
- [[方法-RotVGICP]]
- [[方法-分离式旋转-平移估计]]
- [[方法-地面车辆运动约束]]
- [[方法-6-DoF 退化检测]]
- [[概念-连续时间轨迹]]