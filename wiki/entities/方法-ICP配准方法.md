---
tags: [LiDAR, ICP, 点云配准]
sources:
  - wiki/sources/2026-04-28-genz-icp-analysis.md
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-28-lightning-lm-analysis.md
  - wiki/sources/2026-04-29-genz_icp_analysis.md
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-ICP变体族]]
type: entity
---
> 本页内容已归并至 [[方法-ICP变体族]]。

# ICP配准方法

> 迭代最近点算法，通过最小化两组点云间对应点距离迭代求解刚体变换，是 LiDAR SLAM 中最基础的点云配准方法。

## 概述

ICP（Iterative Closest Point）在两帧点云间交替执行最近邻对应搜索和 SVD 求解刚体变换，收敛到局部最优。标准点到点 ICP 对初始位姿敏感，衍生出点到线（Point-to-Line）、点到面（Point-to-Plane）和对称 ICP 等变体以提升鲁棒性和收敛速度。kiss-icp 使用恒定速度模型提供初值，genz-icp 利用 GICP 协方差建模。

## 基本循环

1. 用当前位姿初值把 source 点云变换到 target 坐标系。
2. 为每个 source 点查找最近邻或局部几何结构。
3. 构造残差，例如点到点、点到面或马氏距离。
4. 求解位姿增量并更新 `T`。
5. 重复直到增量、残差下降或迭代次数满足停止条件。

ICP 的核心假设是初值已经落在正确匹配盆地内。它擅长局部精配准，不擅长全局重定位。

## 常见残差

| 残差 | 形式 | 特点 |
|------|------|------|
| 点到点 | `r = T p - q` | 实现简单，但对结构退化和初值敏感 |
| 点到面 | `r = n^T(T p - q)` | 结构化环境收敛快，是 LIO 常见选择 |
| GICP | `(T p - q)^T Σ^(-1)(T p - q)` | 用局部协方差表达几何不确定性 |
| NDT | 点落入目标分布网格的似然 | 对初值更稳，但地图离散参数敏感 |

## 在SLAM中的应用

LiDAR SLAM 前端的核心组件：FAST-LIO 使用点到面 ICP 进行 scan-to-map 配准，LeGO-LOAM 使用两步 LM 优化分步估计地面和边缘特征配准，CT-ICP 在连续时间轨迹框架中进行 scan-to-sweep 弹性配准。ICP 还与 IMU 预积分结合，在去畸变后提供更精确的点云初值。

## 工程陷阱

- 最近邻不是数据关联真值，动态点和重复结构会制造错误对应。
- 只看最终 fitness score 不够，还应检查 Hessian 退化、内点比例和位姿增量。
- 高速运动场景必须先做 deskew，否则 ICP 会拟合畸变后的点云。
- 回环场景应先用 ScanContext、NDT grid search 或分支定界缩小候选，再用 ICP 精配准。

## Agent 实现提示

### 适用场景

用于实现局部 scan-to-map 或 scan-to-scan 精配准：已有较可靠初值，需要用最近邻对应和 SE(3) 增量优化把当前帧贴合局部地图。适合里程计前端、回环几何验证和粗配准后的精化；不适合直接做全局重定位。

### 输入输出契约

- 输入：source 点云、target 局部地图或点云、初始位姿 `T_init`、最大对应距离、鲁棒核尺度、最大迭代次数和收敛阈值。
- 输出：优化后位姿 `T_target_source`，以及可选诊断量：有效对应数、均方残差、最后一次位姿增量范数、Hessian 条件数。
- 前置条件：source 已去畸变并下采样；target 查询结构已构建；初值误差应落在 ICP 收敛盆地内。

### 实现骨架（伪代码）

```text
T <- T_init
source_w <- transform(source, T)
for iter in 1..max_iter:
    correspondences <- []
    for p in source_w:
        q, dist <- target.nearest_neighbor(p)
        if dist < max_corr_dist:
            correspondences.append((p, q))
    H, b <- build_point_to_point_or_point_to_plane_system(correspondences)
    dx <- solve(H, -b)
    dT <- se3_exp(dx)
    source_w <- transform(source_w, dT)
    T <- dT * T
    if norm(dx) < convergence_threshold:
        break
return T
```

### 关键源码片段

`raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L60-L78`

```cpp
Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });
    return correspondences;
}
```

`raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L96-L121`

```cpp
    auto GM_weight = [&](const double &residual2) {
        return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    const double w = GM_weight(residual.squaredNorm());
                    return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
                                        J_r.transpose() * w * residual);  // JTr
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return {JTJ, JTr};
```

### 实现注意事项

- 对应搜索和线性系统构建要分离，方便替换 KD-tree、体素哈希或 GPU 查询。
- 先用距离门限裁剪离群对应，再用鲁棒核压低大残差；两者用途不同。
- `dx` 很小时不代表配准正确，还要检查有效对应数量和残差是否合理。
- 点到点版本在平面走廊等退化环境中约束弱，工程上常切换为点到面、GICP 或融合 IMU 先验。

### 源码检索锚点

- `DataAssociation`
- `BuildLinearSystem`
- `Registration::AlignPointsToMap`
- `VoxelHashMap::GetClosestNeighbor`

## 相关页面

- [[方法-genz-icp]], [[算法-KISS-ICP]], [[算法-CT-ICP]]
- [[算法-FAST-LIO]], [[算法-LeGO-LOAM]]
- [[方法-GICP配准方法]]
- [[方法-IMU deskew]]
- [[方法-退化检测与修复]]
