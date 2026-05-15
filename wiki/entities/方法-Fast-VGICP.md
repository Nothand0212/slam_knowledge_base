---
tags: [ICP, 配准, 体素, BEV-LSLAM]
sources:
  - wiki/sources/2026-04-29-bev_lslam_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-29
type: entity
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# Fast-VGICP

> BEV-LSLAM 后端精配准算法：体素化 GICP，在 0.5m 分辨率下将 scan 点云与局部子图进行稠密配准。

## 定义

Fast-VGICP 是 GICP 的体素化加速实现。它把目标点云划分成固定分辨率体素，在每个体素内维护局部高斯分布，用体素索引和并行计算替代标准 GICP 中昂贵的逐点近邻搜索。

## 使用场景

- 作为 scan-to-map 后端精化前端 ORB 匹配给出的初值
- 25 次迭代，14 线程并行
- 将 height 和 intensity 点云合并后送入 VGICP

## 与标准 ICP 的关系

| 方法 | 对应关系 | 代价 | 特点 |
|------|----------|------|------|
| ICP | 点到点最近邻 | 欧氏距离 | 快但对噪声和退化敏感 |
| GICP | 点/邻域协方差 | 马氏距离 | 几何建模更稳 |
| VGICP | 体素高斯分布 | 体素级马氏距离 | 更适合大规模 scan-to-map |

BEV-LSLAM 中，Fast-VGICP 不是负责全局搜索，而是在 ORB/GMS 等前端已经给出候选初值后做局部精配准。

## 工程注意

- 体素分辨率 `0.5m` 是精度和速度折中，不能无脑套到所有传感器。
- 初值错误时，VGICP 仍可能收敛到错误局部极小值。
- height 和 intensity 点云融合后应注意尺度和噪声差异，否则某一通道会主导配准。

## Agent 实现提示

### 适用场景

用于大规模 scan-to-map 精配准，尤其是 target 地图较大、逐点 KNN 成本过高时。Fast-VGICP 把 target 聚合为体素高斯，source 点只需落格查询目标体素，适合作为回环候选或 BEV 前端初值之后的局部几何验证。

### 输入输出契约

- 输入：source 点云及协方差、target GaussianVoxelMap、初始位姿、体素分辨率、邻域/半径策略、最大迭代次数。
- 输出：优化位姿、收敛标志、fitness 或马氏误差、有效体素对应比例。
- 前置条件：target 体素已完成均值/协方差统计；source 点云密度与体素分辨率匹配；初值不应跨越过多体素。

### 实现骨架（伪代码）

```text
target_voxels <- build_gaussian_voxelmap(target_points, voxel_resolution)
source_covs <- estimate_covariances(source_points)
T <- initial_guess
for iter in 1..max_iter:
    for each source point i:
        p <- transform(source[i], T)
        voxel <- target_voxels.lookup(voxel_coord(p))
        if voxel missing:
            skip i
        Sigma <- voxel.cov + T.matrix * source_cov[i] * T.matrix^T
        r <- voxel.mean - p
        accumulate r^T Sigma^-1 r and Jacobians
    dx <- solve(H, -b)
    T <- exp(dx) * T
return T
```

### 关键源码片段

`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:L114-L146`

```cpp
  const auto perpoint_task = [&](int i) {
    Eigen::Vector4d pt = delta * frame::point(*source, i);
    Eigen::Vector3i coord = target_voxels->voxel_coord(pt);
    const auto voxel_id = target_voxels->lookup_voxel_index(coord);

    if (voxel_id < 0) {
      correspondences[i] = nullptr;

      switch (mahalanobis_cache_mode) {
        case FusedCovCacheMode::FULL:
          mahalanobis_full[i].setZero();
          break;
        case FusedCovCacheMode::COMPACT:
          mahalanobis_compact[i].setZero();
          break;
        case FusedCovCacheMode::NONE:
          break;
      }
    } else {
      const auto voxel = &target_voxels->lookup_voxel(voxel_id);
      correspondences[i] = voxel;

      switch (mahalanobis_cache_mode) {
        case FusedCovCacheMode::FULL: {
          const Eigen::Matrix4d RCR = (voxel->cov + delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose());
          mahalanobis_full[i].setZero();
          mahalanobis_full[i].template topLeftCorner<3, 3>() = RCR.topLeftCorner<3, 3>().inverse();
          break;
        }
        case FusedCovCacheMode::COMPACT: {
          const Eigen::Matrix4d RCR = (voxel->cov + delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose());
          const Eigen::Matrix3d maha = RCR.topLeftCorner<3, 3>().inverse();
          mahalanobis_compact[i] = compact_cov(maha);
```

`raw/codes/gtsam_points/include/gtsam_points/cuda/kernels/vgicp_derivatives.cuh:L39-L72`

```cpp
    const Eigen::Vector3f& mean_A = source_means_ptr[source_idx];
    const Eigen::Matrix3f& cov_A = source_covs_ptr[source_idx];
    const Eigen::Vector3f transed_mean_A = R * mean_A + t;

    const Eigen::Vector3f& mean_B = voxel_means_ptr[target_idx];
    const Eigen::Matrix3f& cov_B = voxel_covs_ptr[target_idx];

    const int num_points = voxel_num_points_ptr[target_idx];

    const Eigen::Matrix3f RCR = (R * cov_A * R.transpose());
    const Eigen::Matrix3f RCR_inv = (cov_B + RCR).inverse();
    Eigen::Vector3f error = mean_B - transed_mean_A;

    Eigen::Matrix<float, 3, 6> J_target;
    J_target.block<3, 3>(0, 0) = -skew_symmetric(transed_mean_A);
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();

    Eigen::Matrix<float, 3, 6> J_source;
    J_source.block<3, 3>(0, 0) = R * skew_symmetric(mean_A);
    J_source.block<3, 3>(0, 3) = -R;

    Eigen::Matrix<float, 6, 3> J_target_RCR_inv = J_target.transpose() * RCR_inv;
    Eigen::Matrix<float, 6, 3> J_source_RCR_inv = J_source.transpose() * RCR_inv;

    LinearizedSystem6 linearized;
    linearized.num_inliers = 1;
    linearized.error = error.transpose() * RCR_inv * error;
    linearized.H_target = J_target_RCR_inv * J_target;
    linearized.H_source = J_source_RCR_inv * J_source;
    linearized.H_target_source = J_target_RCR_inv * J_source;
    linearized.b_target = J_target_RCR_inv * error;
    linearized.b_source = J_source_RCR_inv * error;
```

### 实现注意事项

- 体素分辨率过大时会把不同平面混成一个高斯，过小时有效对应不足。
- 体素查询通常是近似对应，必要时可扩展到 7/27 邻域或半径邻域。
- GPU 实现要把 correspondence、voxel mean/cov、source mean/cov 放成连续数组，减少 kernel 内随机访问。
- 对比 GICP 指标时要记录体素内点数和落格比例，低落格比例通常表示初值或地图覆盖有问题。

### 源码检索锚点

- `IntegratedVGICPFactor_`
- `target_voxels->lookup_voxel_index`
- `vgicp_derivatives_kernel`
- `GaussianVoxelMapGPU`
- `LinearizedSystem6`

## 相关页面

- [[方法-GICP配准方法]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[算法-BEV-LSLAM]]
- [[方法-GMS匹配验证]]
- [[2026-04-29-external-primary-source-check]]
