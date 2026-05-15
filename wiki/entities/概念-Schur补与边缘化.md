---
tags: [优化, 边缘化, 因子图]
sources:
  - wiki/sources/2026-04-28-vins_fusion-analysis.md
  - wiki/sources/2026-04-28-schurvins-analysis.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-28-phad-fusion-design.md
created: 2026-04-29
updated: 2026-05-15
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
- [[方法-SmartStereoFactor]] [[方法-滑动窗口边缘化]] [[方法-关键帧选择策略]] [[方法-ISAM2增量固定滞后平滑]]

## VINS-Fusion 源码实现

**源码锚点**: `raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.cpp:L183-L311`

### Schur 补边缘化核心

```
marginalize():
    // 1. 参数块编排：被边缘化的变量在前（索引 0..m-1），保留的在后（索引 m..m+n-1）
    m = sum(dim_of_dropped_blocks)   // 被边缘化变量总维度
    n = sum(dim_of_kept_blocks)      // 保留变量总维度

    // 2. 多线程构建 H 和 b（所有残差因子 Evaluate 后）
    A = zero_matrix(m+n, m+n);  b = zero_vector(m+n)
    for each residual block (4-thread parallel):
        J_i = leftCols(localSize) of jacobian_i
        A.block(idx_i, idx_j) += J_i^T * J_j
        b.segment(idx_i)     += J_i^T * residual

    // 3. Schur 补
    Amm = 0.5 * (A(0:m, 0:m) + A(0:m, 0:m)^T)  // 对称化
    Amm_inv = V * diag(1/eig if eig>eps else 0) * V^T  // 正则化逆

    A_reduced = A(m:, m:) - A(m:, 0:m) * Amm_inv * A(0:m, m:)
    b_reduced = b(m:)     - A(m:, 0:m) * Amm_inv * b(0:m)

    // 4. 分解为线性 Jacobian + 残差（便于后续作为 Ceres CostFunction）
    eigendecompose(A_reduced) → V_eig * S * V_eig^T
    linearized_jacobians = sqrt(S) * V_eig^T
    linearized_residuals = 1/sqrt(S) * V_eig^T * b_reduced
```

**关键行号**: `marginalization_factor.cpp:L281-L306`（Schur 补 + 特征分解）

### SE(3) 位姿的切空间处理

位姿在优化中用 7 维表示（平移 + 四元数），但 Schur 补在线性化后的**切空间（6 维李代数）**上运算：

```
localSize(7) = 6     // 从 7 维参数块映射到 6 维切空间
globalSize(6) = 7    // 反向映射

Jacobian 截取: jacobian_i.leftCols(6)  // 只取切空间分量
```

### 先验因子接入优化

边缘化后产生的 `MarginalizationFactor` 在下一轮 Ceres 优化中作为 CostFunction：

```
Evaluate(parameters, residuals, jacobians):
    dx = current_params ⊖ linearization_point  // 流形上的差
    residuals = linearized_residuals + linearized_jacobians * dx
    jacobians = linearized_jacobians
```

先验因子**不允许重新线性化**，保证了 first-estimate Jacobian (FEJ) 一致性。

### KF 与边缘化策略联动

```
addFeatureCheckParallax() → KF = true
    → marginalization_flag = MARGIN_OLD
    → 边缘化最老帧（Pose[0], SpeedBias[0]）
    → 将其 IMU/视觉/先验约束通过 Schur 补压缩

addFeatureCheckParallax() → KF = false
    → marginalization_flag = MARGIN_SECOND_NEW
    → 丢弃次新帧视觉观测，只合并 IMU 预积分
    → 不执行 Schur 补（避免引入不必要的先验）
```

**源码锚点**: `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L411-L427`
