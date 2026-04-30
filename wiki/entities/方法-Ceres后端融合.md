---
tags: [方法, Ceres, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-fusions_slam-analysis.md
---

# Ceres后端融合

> 用 Ceres 构建低频后端优化，把前端 LIO 相对位姿和 GPS 全局位置约束融合成更稳定的轨迹。

## 定义

Ceres 后端融合是把实时前端输出转化为优化问题的松耦合后端模式。fusions_slam 的 `LioGpsOpt` 不直接处理原始点云或 IMU，而是接收 LIO 里程计结果和 GPS 位置观测，在独立线程中优化轨迹。

## fusions_slam 因子

| 因子 | 约束 | 作用 |
|------|------|------|
| `RelativeRTError` | 相邻关键帧相对位姿 | 保持 LIO 局部运动连续性 |
| `TError` | 轨迹位置接近 GPS 观测 | 抑制长期漂移并锚定全局位置 |

这类后端不是紧耦合 GNSS/INS；GPS 不参与前端 IESKF 的每次状态更新，而是在低频优化层修正轨迹。

## 求解配置

- 线性求解器：`SPARSE_NORMAL_CHOLESKY`
- 鲁棒核：`HuberLoss(1.0)`
- 位姿参数化：`QuaternionParameterization`
- 线程模型：异步后端优化，不阻塞前端 LIO

## 工程边界

- 优点是侵入性低，前端 LIO 可以独立运行。
- 缺点是 GPS 改正存在滞后，不能直接改善前端当前帧的去畸变和配准。
- 如果 GPS 质量波动大，必须配合粗差剔除或动态噪声调节。
- 若需要实时全局一致状态，应考虑把 GNSS 作为 IESKF 观测或因子图在线增量更新。

## 相关页面

- [[算法-fusions_slam]]
- [[架构-多传感器融合架构]]
- [[组件-Ceres-Solver]]
- [[方法-GNSS 位置残差因子]]
- [[因子图vs滤波]]
