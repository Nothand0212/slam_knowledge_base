---
tags: [LiDAR, GICP, ICP]
sources:
  - wiki/sources/2026-04-28-4d-radar-slam-analysis.md
  - wiki/sources/2026-04-28-fast-lio-sam-analysis.md
  - wiki/sources/2026-04-28-genz-icp-analysis.md
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-29-genz_icp_analysis.md
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
created: 2026-04-29
updated: 2026-04-30
type: entity
---

# GICP配准方法

> 广义 ICP 将点云配准建模为概率框架下的最大似然估计，用协方差矩阵建模点的局部表面不确定性，统一了点到点、点到面和面到面 ICP。

## 概述

GICP（Generalized ICP）由 Segal 等人于 2009 年提出，将 ICP 视为两个高斯分布点集的概率匹配问题。标准 ICP 是 GICP 在协方差取单位矩阵时的特例，点到面 ICP 是协方差取表面法向退化矩阵时的特例。GICP 通过局部点分布估计协方差，在平面区域退化为点到面 ICP，在曲面/角点区域保持面到面 ICP 的精度。

## 残差形式

设 source 点 `p_i` 变换到 target 坐标系后为 `T p_i`，target 对应点为 `q_i`。若两侧局部协方差分别为 `C_i` 和 `C_j`，则联合协方差可写成：

```text
Σ_i = C_j + R C_i R^T
```

GICP 最小化的是马氏距离：

```text
e_i = (T p_i - q_i)^T Σ_i^(-1) (T p_i - q_i)
```

当 `Σ_i` 接近单位阵时，它退化为点到点 ICP；当平面切向方差很大、法向方差很小时，它行为接近点到面 ICP。

## 在SLAM中的应用

GICP 被广泛用于 LiDAR SLAM 的点云配准和回环验证：Fast GICP 提供多线程加速实现，FAST-LIO-SAM 使用 GICP 作为回环约束的几何验证步骤，RT-KLIO 在其 LiDAR 里程计中使用 GICP 替代传统 ICP。GICP 对结构化环境的平面区域配准精度优于标准 ICP，适合城市道路和室内走廊场景。

## 变体

| 变体 | 核心变化 | 适用点 |
|------|----------|--------|
| Fast GICP / Nano-GICP | 并行化和 KD-tree/近邻加速 | 工程实时性 |
| VGICP / GaussianVoxelMap | 用体素高斯分布替代逐点 KNN | 大规模地图、GPU 批处理 |
| GenZ-ICP | 根据局部平面度自适应选择点到面/点到点权重 | 结构化与非结构化混合环境 |
| RotVGICP | 分离旋转和平移估计，结合体素高斯地图 | 地面车辆和退化场景 |

## 工程注意

- 协方差估计质量决定 GICP 的收益，近邻数过少会不稳定。
- 初值仍然重要；GICP 改善局部模型，不解决全局配准搜索。
- 回环验证中应配合 ScanContext、NDT 或位姿一致性检查，避免错误闭环。
- 用作因子图约束时，应把配准协方差或 Hessian 质量传播到噪声模型中。

## Agent 实现提示

### 适用场景

用于需要比点到点 ICP 更稳的局部配准：城市道路、室内平面、回环约束验证、因子图中的 scan matching 因子。它依赖局部协方差表达几何不确定性，适合结构化环境，但仍需要外部初值或候选生成器。

### 输入输出契约

- 输入：带点协方差的 target/source 点云，target 最近邻索引，当前相对位姿 `delta`，最大对应距离，协方差缓存策略。
- 输出：马氏距离误差、Hessian/梯度块、有效对应索引、内点比例。
- 前置条件：每个点的协方差已估计并正则化；坐标系约定明确；source 到 target 的初值足以找到正确近邻。

### 实现骨架（伪代码）

```text
for each source point i:
    p <- transform(source.mean[i], delta)
    j <- target_tree.knn(p, k=1, max_distance)
    if no match:
        mark_outlier(i)
        continue
    Sigma <- target.cov[j] + delta.matrix * source.cov[i] * delta.matrix^T
    M <- inverse(top_left_3x3(Sigma))
    cache M for point i

for each valid correspondence i -> j:
    r <- target.mean[j] - transform(source.mean[i], delta)
    e <- r^T M r
    J_target, J_source <- se3_jacobians(r, source.mean[i], delta)
    accumulate H_target, H_source, H_target_source, b_target, b_source
```

### 关键源码片段

`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L162-L190`

```cpp
  const auto perpoint_task = [&](int i) {
    if (do_update) {
      Eigen::Vector4d pt = delta * frame::point(*source, i);

      size_t k_index = -1;
      double k_sq_dist = -1;
      size_t num_found = target_tree->knn_search(pt.data(), 1, &k_index, &k_sq_dist, max_correspondence_distance_sq);
      correspondences[i] = (num_found && k_sq_dist < max_correspondence_distance_sq) ? k_index : -1;
    }

    switch (mahalanobis_cache_mode) {
      case FusedCovCacheMode::FULL:
        if (correspondences[i] < 0) {
          mahalanobis_full[i].setZero();
        } else {
          const auto& target_cov = frame::cov(*target, correspondences[i]);
          const Eigen::Matrix4d RCR = (target_cov + delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose());
          mahalanobis_full[i].setZero();
          mahalanobis_full[i].template topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
        }
        break;
      case FusedCovCacheMode::COMPACT:
        if (correspondences[i] < 0) {
          mahalanobis_compact[i].setZero();
        } else {
          const auto& target_cov = frame::cov(*target, correspondences[i]);
          const Eigen::Matrix4d RCR = (target_cov + delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose());
          const Eigen::Matrix3d maha = RCR.topLeftCorner<3, 3>().inverse();
          mahalanobis_compact[i] = compact_cov(maha);
```

`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L242-L280`

```cpp
    const auto& mean_A = frame::point(*source, i);
    const auto& cov_A = frame::cov(*source, i);
    const auto& mean_B = frame::point(*target, target_index);
    const auto& cov_B = frame::cov(*target, target_index);

    const Eigen::Vector4d transed_mean_A = delta * mean_A;
    const Eigen::Vector4d residual = mean_B - transed_mean_A;

    Eigen::Matrix4d mahalanobis;
    switch (mahalanobis_cache_mode) {
      case FusedCovCacheMode::FULL:
        mahalanobis = mahalanobis_full[i];
        break;
      case FusedCovCacheMode::COMPACT:
        mahalanobis = uncompact_cov(mahalanobis_compact[i]);
        break;
      case FusedCovCacheMode::NONE: {
        const auto& delta_l = linearization_point;  // Delta at the linearization point
        const Eigen::Matrix4d RCR = (cov_B + delta_l.matrix() * cov_A * delta_l.matrix().transpose());
        mahalanobis.setZero();
        mahalanobis.topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
      } break;
    }

    const double error = residual.transpose() * mahalanobis * residual;
    if (H_target == nullptr) {
      return error;
    }

    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    Eigen::Matrix<double, 6, 4> J_target_mahalanobis = J_target.transpose() * mahalanobis;
    Eigen::Matrix<double, 6, 4> J_source_mahalanobis = J_source.transpose() * mahalanobis;
```

### 实现注意事项

- 协方差矩阵求逆前必须正则化，否则平面或线结构会产生病态矩阵。
- 最近邻更新可以按位姿变化阈值延迟，但缓存的马氏矩阵必须与线性化点一致。
- GICP 的误差单位依赖协方差尺度，不能直接与普通 ICP fitness 混用阈值。
- 多线程累加 Hessian 时要使用线程局部块或安全 reduction，避免非确定性写冲突。

### 源码检索锚点

- `IntegratedGICPFactor_`
- `update_correspondences`
- `FusedCovCacheMode`
- `max_correspondence_distance_sq`
- `mahalanobis_full`

## 相关页面

- [[方法-ICP配准方法]], [[方法-genz-icp]], [[算法-KISS-ICP]]
- [[算法-FAST-LIO]], [[概念-回环检测方法]]
- [[方法-体素地图]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[方法-POINT_TO_DISTRIBUTION]]
