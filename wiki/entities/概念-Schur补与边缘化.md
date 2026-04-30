---
tags: [优化, 边缘化, 因子图]
sources:
  - wiki/sources/2026-04-28-vins_fusion-analysis.md
  - wiki/sources/2026-04-28-schurvins-analysis.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-28-phad-fusion-design.md
created: 2026-04-29
updated: 2026-04-29
type: entity
---

# Schur补与边缘化

> 通过 Schur 补将优化问题中待淘汰变量的信息压缩为剩余变量上的先验约束，是滑动窗口 SLAM 和因子图增量推理的数学基础。

## 数学形式

边缘化通常发生在线性化后的正规方程中。把增量变量分为需要保留的 `x_r` 和准备淘汰的 `x_m`：

```text
[ H_rr  H_rm ] [ dx_r ] = [ b_r ]
[ H_mr  H_mm ] [ dx_m ]   [ b_m ]
```

若 `H_mm` 可逆，消去 `dx_m` 后，保留变量上的等效信息矩阵和右端项为：

```text
H_prior = H_rr - H_rm H_mm^{-1} H_mr
b_prior = b_r  - H_rm H_mm^{-1} b_m
```

这组 `H_prior, b_prior` 就是边缘化先验。它不是原始观测，而是过去被移出窗口的变量对当前变量的压缩信息。

## 在 SLAM 中的应用

滑动窗口 VIO 会在窗口满时边缘化旧关键帧、速度、偏置或路标点，以限制状态维度。MSCKF 通过零空间投影/Schur 消去特征点，只保留对相机状态的约束；VINS-Fusion 会把被移出窗口的状态压缩成 prior factor；GTSAM smart factor 在内部三角化并消去路标，使多帧观测直接约束位姿。

关键工程问题是线性化点。一旦边缘化先验生成，它对应的线性化点必须固定；后续若把相关变量重新线性化但不更新先验，会破坏一致性。FEJ（First-Estimate Jacobian）就是为抑制这种不一致而使用第一次估计处的雅可比。

## 常见风险

- `H_mm` 病态或不可逆时，说明被边缘化变量观测不足，需要阻尼、先验或剔除退化变量。
- 先验因子过密，会破坏稀疏结构，导致后端求解变慢。
- 在多传感器异步融合中，错误边缘化仍有未来观测的状态，会让时间对齐和延迟补偿变得困难。

## 相关页面

- [[架构-滑动窗口优化]], [[组件-GTSAM]]
- [[数学-流形优化]], [[概念-因子图]]
- [[概念-MSCKF]], [[组件-Ceres-Solver]]
- [[方法-SmartStereoFactor]]
