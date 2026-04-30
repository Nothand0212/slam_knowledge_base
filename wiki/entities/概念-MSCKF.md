---
tags: [滤波, VIO, 视觉惯性]
sources:
  - wiki/sources/2026-04-28-kimera_vio-analysis.md
  - wiki/sources/2026-04-28-msckf_vio-analysis.md
  - wiki/sources/2026-04-28-open_vins-analysis.md
  - wiki/sources/2026-04-28-schurvins-analysis.md
  - wiki/sources/2026-04-28-vins_fusion-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-29-msckf-vio-analysis-analysis.md
  - wiki/sources/2026-04-29-open-vins-analysis.md
created: 2026-04-28
type: entity
updated: 2026-04-30
---

# MSCKF

> Multi-State Constraint Kalman Filter，一种在滑动窗口内维护多帧相机位姿的 EKF 框架，利用零空间投影消除路标点状态，避免 VIO 中特征点数量膨胀。

## 概述

MSCKF（Multi-State Constraint Kalman Filter）由 Mourikis 和 Roumeliotis 于 2007 年提出，是视觉惯性里程计中经典的高效滤波方案。其核心思路是维护一个滑动窗口（例如最近 10 帧相机位姿），但不将三维路标点增广到状态向量中。当一个特征被某帧边缘化出窗口时，利用该特征在所有观测帧上的重投影约束一次性更新滤波器，通过将约束投影到路标点的零空间来消除未知深度带来的依赖。

这一设计的优势在于滤波器状态维度仅随窗口大小线性增长（而非随路标数量增长），EKF 更新复杂度为 O(n²)（n 为窗口帧数）。配合 FEJ（First-Estimate Jacobian）可缓解 EKF 单次线性化带来的不一致性问题。其局限是没有全局优化能力，长期漂移仅能依靠回环检测配合位姿图优化来修正。

## 在分析框架中的应用

- **open_vins**：MSCKF 的标准现代实现，支持单目/双目 VIO，内建 FEJ 和在线标定，同时提供可选的全局 SLAM 图优化。
- **msckf_vio**：Kumar Robotics 的经典原生实现，使用 ETH 的 VI-sensor，在无人机上广泛使用。
- VINS-Fusion 的前端也参考了 MSCKF 的单帧到多帧约束结构。

## 对 SLAM 算法的意义

MSCKF 提供了 VIO 滤波方案的性能基准。若选择以滤波为主的前端方案，MSCKF 的零空间投影原理可用于在 LiDAR-惯性-视觉融合中将视觉特征约束压缩为帧间相对约束，避免显式估计路标深度，降低状态维度和计算量。

## Agent 实现提示

### 适用场景

当 VIO/视觉融合前端需要利用多帧特征轨迹约束相机克隆位姿，但又不希望把每个三维路标长期加入滤波状态时，使用 MSCKF 更新。它特别适合实时性优先、窗口较短、视觉特征数量大、后端不做全局 BA 的系统。

### 输入输出契约

- **输入**：滤波器状态 `state`、待边缘化或跟踪结束的特征轨迹集合、每帧归一化观测、相机克隆位姿、像素噪声和卡方门限。
- **输出**：经零空间投影后的残差 `res`、状态雅可比 `H_x`、参与更新的状态块顺序 `Hx_order`，以及通过 EKF 更新后的状态和协方差。
- **约束**：特征必须先三角化或转换到可线性化的锚点表示；观测帧数不足或卡方检验失败的特征应丢弃。

### 实现骨架（伪代码）

```pseudo
function msckfUpdate(state, feature_tracks):
    H_all, r_all, order_all = empty
    for track in feature_tracks:
        feat = convertTrackToUpdaterFeature(track, state.feature_representation)
        H_f, H_x, r, order = buildFeatureJacobian(state, feat)
        H_x, r = leftNullspaceProject(H_f, H_x, r)
        S = H_x * marginalCov(state, order) * H_x.T + sigma_px^2 * I
        if r.T * inv(S) * r < chi2Threshold(len(r)):
            append(H_all, H_x); append(r_all, r); merge(order_all, order)
    compressed = compressMeasurements(H_all, r_all)
    kalmanUpdate(state, compressed.H, compressed.r, order_all)
```

### 关键源码片段

`raw/codes/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp:L196-L220`

```cpp
    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // Nullspace project
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

    /// Chi2 distance check
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is more)
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
```

`raw/codes/open_vins/ov_msckf/src/update/UpdaterHelper.cpp:L426-L454`

```cpp
void UpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Apply the left nullspace of H_f to all variables
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = (int)H_f.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
  // NOTE: need to eigen3 eval here since this experiences aliasing!
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // Sanity check
  assert(H_x.rows() == res.rows());
}
```

### 实现注意事项

- 零空间投影的目标是消去 `H_f` 对特征位置的依赖，保留只关于状态误差的约束。
- FEJ 场景下，雅可比应使用首次估计的克隆位姿或特征锚点，避免可观测性被后续重线性化破坏。
- 多相机、多克隆状态更新时，`Hx_order` 必须与协方差子块顺序一致。
- 卡方门限应在零空间投影后计算；投影前的残差维度不是最终观测维度。

### 源码检索锚点

- `UpdaterMSCKF::update`
- `get_feature_jacobian_full`
- `nullspace_project_inplace`
- `chi_squared_table`
- `StateHelper::EKFUpdate`

## 相关页面

- [[架构-滑动窗口优化]]
- [[方法-视觉特征跟踪]]
- [[组件-GTSAM]]
---
## (合并自: MSCKF与SchurComplement对比.md)
---
---
tags: [MSCKF, Schur补, 边缘化, VIO, 滤波]
sources: []
created: 2026-04-29
type: entity
---

# MSCKF与SchurComplement对比

> 两种主流 VIO 滤波方案：经典 MSCKF 维护滑动窗口状态直接估计，SchurVINS/SchurComplement 利用 Schur 消元分离位姿和特征点状态。

## 核心区别

| | MSCKF | SchurComplement |
|---|---|---|
| 状态向量 | 滑动窗口内所有位姿 + 特征点 | 位姿仅保留，特征点 Schur 边缘化 |
| 特征点处理 | 三角化后加入状态向量，边缘化时移除 | 永远不进状态向量，每帧消元 |
| 计算复杂度 | O(N³) 随状态增长 | 特征点部分 O(N) 消元，位姿部分较小 |
| 代表实现 | msckf_vio, open_vins | SchurVINS |
| 线性化 | 每帧重新线性化 | Schur 消元后再线性化 |

## 关键设计

- MSCKF：维护 20-30 帧滑动窗口，特征点观测满一定帧数后三角化加入状态，边缘化最老帧
- SchurComplement：利用特征点只连接有限帧的稀疏性，消元后位姿 Hessian 矩阵保持稠密
- 融合方案：SchurVINS 将 Schur 消元与 MSCKF EKF 结合，RK4 IMU 积分提升精度

## 相关页面

- [[概念-MSCKF]]
- [[概念-Schur补与边缘化]]
- [[架构-滑动窗口优化]]
