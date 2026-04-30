---
tags: [VIO, 直接法, 延迟边缘化, IMU]
sources:
  - raw/docs-deep-dive/dm_vio_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/dm_vio_analysis.md
---

# DM-VIO 源码级分析摘要

> 延迟边缘化（Delayed Marginalization）解决直接法 VIO 的核心矛盾：IMU 参数在边缘化前有时间收敛，避免错误信息被锁定。

## 核心发现
- **延迟边缘化**是核心创新：将帧放入队列暂缓边缘化，等待 scale/gravity/bias 收敛后再执行 Schur complement
- GTSAM + DSO 桥接：通过 `computeBAUpdate` 直接相加视觉 Hessian 和 IMU Hessian，统一优化
- **动态视觉权重**：光度 RMSE 过高时按 $(τ / RMSE)^2$ 降低视觉因子权重，系统更依赖 IMU
- **FEJ 协调机制**：视觉变量用当前值，IMU 变量（vel/bias/scale/gravity）用首次线性化点
- 自动尺度收敛：scaleQueue 监控尺度波动，收敛后自动固定不再优化

## 技术要点
| 维度 | 方案 |
|------|------|
| 范式 | 直接法单目 VIO |
| 前端 | 继承 DSO 粗跟踪 + 轻量级 IMU 因子图辅助 |
| 后端 | GTSAM 因子图 + 延迟边缘化 + 多延迟图架构 |
| 独特创新 | 延迟边缘化、多延迟图切换、动态视觉权重、自动尺度收敛 |

## 关键引用
- 延迟边缘化核心：`src/GTSAMIntegration/DelayedMarginalization.cpp:212-253`
- 多延迟图架构：`src/GTSAMIntegration/DelayedMarginalization.h:114-163`
- GTSAM+DSO Hessian 相加：`src/GTSAMIntegration/BAGTSAMIntegration.cpp:174-175`
- 动态权重计算：`src/BAIMULogic.cpp:801-808`
- 尺度收敛逻辑：`src/BAIMULogic.cpp:457-497`
- Schur complement 实现：`src/GTSAMIntegration/Marginalization.cpp:31-90`
- IMU 预积分因子：`src/IMU/BAIMULogic.cpp:235-323`

## 相关页面
- [[算法-DSO]]
- [[VIO方案对比]]
- [[2026-04-29-vins-fusion-analysis-analysis|VINS-Fusion 边缘化]]