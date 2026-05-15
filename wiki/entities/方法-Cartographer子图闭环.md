---
tags: [Cartographer, 子图, 闭环检测, Branch-and-Bound, 扫描匹配, 位姿图优化]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/cartographer
---

# Cartographer 子图闭环

> Cartographer 采用子图（submap）架构管理局部 SLAM 结果，并通过 Branch-and-Bound 多分辨率扫描匹配在子图间检测回环，最终以 Ceres 位姿图优化全局一致性。

## 系统架构

Cartographer 将 SLAM 分两层：

1. **局部 SLAM（Local SLAM）**：以固定数量雷达扫描构建短期子图（默认 90 帧）。每帧通过 Ceres 非线性优化对齐到当前子图网格；
2. **全局 SLAM（Global SLAM）**：后台线程周期性在所有已完成的子图间搜索回环，构建子图-节点位姿约束，求解全局位姿图；

```
传感器输入 → 体素滤波 → Local SLAM
                            │
                    ┌───────▼────────┐
                    │  子图 (Submap) │  ← 每90帧生成一个新子图
                    │  概率占据网格  │     旧子图标记为finished
                    └───────┬────────┘
                            │
                ┌───────────▼───────────┐
                │   Global SLAM (后台)   │
                │  ┌───────────────────┐ │
                │  │ BnB 回环检测      │ │
                │  │ 约束构建 + 优化   │ │
                │  │ 子图裁剪          │ │
                │  └───────────────────┘ │
                └───────────────────────┘
```

## 子图管理

### Submap2D 结构

`raw/codes/cartographer/cartographer/mapping/2d/submap_2d.cc:L70-L76` — 子图持有概率网格（`ProbabilityGrid`）或 TSDF 网格（`TSDF2D`），以及局部坐标系位姿：

```cpp
Submap2D::Submap2D(const Eigen::Vector2f& origin,
                   std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}
```

子图有两个关键状态：
- **活跃子图（active）**：当前正在插入激光帧的子图，前几帧同时插入两个相邻子图以保证过渡平滑；
- **已完成子图（finished）**：不再插入新数据，但其位姿仍可在全局优化中被调整；

子图数目由 `num_range_data` 参数控制（典型值 90）。

### ActiveSubmaps 管理

`ActiveSubmaps2D` 类维护活跃子图的轮换：当当前子图的插入帧数达到 `num_range_data` 时，标记为 finished，并创建新的活跃子图。Cartographer 始终维护 1~2 个活跃子图，以此在局部连续性和全局可优化性之间取得平衡。

## Ceres 扫描匹配

### 前端的 Ceres 精匹配

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc:L63-L107` — 局部 SLAM 中每帧 LiDAR 扫描与当前子图网格的匹配通过 Ceres 求解：

```cpp
void CeresScanMatcher2D::Match(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const Grid2D& grid,
    transform::Rigid2d* const pose_estimate,
    ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {x, y, θ};
  ceres::Problem problem;
  problem.AddResidualBlock(
      CreateOccupiedSpaceCostFunction2D(weight, point_cloud, grid),
      nullptr, ceres_pose_estimate);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          translation_weight, target_translation),
      nullptr, ceres_pose_estimate);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          rotation_weight, ceres_pose_estimate[2]),
      nullptr, ceres_pose_estimate);
  ceres::Solve(ceres_solver_options_, &problem, summary);
  *pose_estimate = Rigid2d({ceres_pose_estimate[0], ceres_pose_estimate[1]},
                            ceres_pose_estimate[2]);
}
```

三个残差块的含义：
- **OccupiedSpaceCostFunction2D**：惩罚扫描点落在网格空闲区域的代价；
- **TranslationDeltaCostFunctor2D**：惩罚偏离初始平移估计；
- **RotationDeltaCostFunctor2D**：惩罚偏离初始旋转估计；

### 代价函数

对于概率网格，每个扫描点的代价为：
$$
\mathcal{C}(\mathbf{p}_i) = 1 - P_{\text{occ}}(\mathbf{p}_i)
$$

其中 $P_{\text{occ}}$ 是网格单元的被占据概率。对于 TSDF 网格，代价为该点 TSDF 值的平滑插值。

## Branch-and-Bound 回环检测

### 多分辨率预计算网格栈

在回环搜索之前，系统为目标子图预计算一个多分辨率概率网格金字塔 `PrecomputationGridStack2D`。每层存储一个滑动窗口内的最大概率值，使得任何候选位姿的得分上界可以 O(1) 计算。

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L91-L161`：

```cpp
PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid) {
  // 第一步：计算每行滑动窗口宽度width的最大概率
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    // ... 行方向滑动窗口最大值 ...
    for (int x = -width+1; x != 0; ++x) {
      intermediate[x+width-1+y*stride] = current_values.GetMaximum();
    }
  }
  // 第二步：在第一步结果上再取列方向滑动窗口最大值
  // 最终得到 width×width 窗口内的最大概率
}
```

网格栈的深度由 `branch_and_bound_depth` 参数决定（典型值 7），对应的宽度依次为 1, 2, 4, ..., 64。

### BnB 递归搜索

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L335-L378`：

```cpp
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates,
    const int candidate_depth, float min_score) const {
  if (candidate_depth == 0) {
    return *candidates.begin();  // 叶子节点，返回最高分
  }
  Candidate2D best_high_resolution_candidate(0,0,0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) {
      break;  // 剪枝：上界低于当前最优
    }
    // 将候选分解为4个高分辨率子候选
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      for (int y_offset : {0, half_width}) {
        higher_resolution_candidates.emplace_back(
            candidate.scan_index,
            candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    // 在更高分辨率网格上评分
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth-1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(..., higher_resolution_candidates,
                       candidate_depth-1, best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}
```

搜索窗口由 `linear_search_window`（默认 ±7m）和 `angular_search_window`（默认 ±30°）定义。最低匹配分数 `min_score`（默认 0.55）作为剪枝阈值。

算法复杂度：窗口 $w \times w$、角度分辨率 $n_\theta$、深度 $d$ 时约为 $O(\frac{w^2 \cdot n_\theta}{2^d} \cdot \log n_{\text{scan}})$，远优于暴力搜索。

## 约束构建与全局优化

### 约束构建

回环检测产生的 `Constraint` 包含约束类型（INTRA_SUBMAP 或 INTER_SUBMAP）和相对位姿：

$$
\mathbf{z}_{ij} = \{^j\mathbf{p}_i \in SE(3), \text{tag} \in \{\text{INTRA}, \text{INTER}\}\}
$$

其中 INTRA_SUBMAP 约束来自局部 SLAM（scan-to-submap 匹配），INTER_SUBMAP 约束来自回环检测。

`raw/codes/cartographer/cartographer/mapping/pose_graph.cc:L55-L70` 约束的反序列化定义。

### 全局位姿图优化

`pose_graph_2d.cc` 中的 `RunOptimization()` 构建子图和节点的位姿图，最小化全局代价：

$$
\min_{\mathbf{p}_i, \mathbf{p}_j} \sum_{ij} \|\mathbf{z}_{ij} - (\mathbf{p}_j^{-1} \mathbf{p}_i)\|_{\Sigma_{ij}}^2
$$

其中 $\mathbf{p}_i, \mathbf{p}_j$ 分别为子图和节点的全局位姿，$\mathbf{z}_{ij}$ 为约束测量的相对位姿，$\Sigma_{ij}$ 为约束的协方差。优化采用 Ceres 的 SPARSE_NORMAL_CHOLESKY 求解器。

优化每 `optimize_every_n_nodes` 帧触发一次。完成后修正所有子图和节点的全局位姿，使全局地图保持一致性。

## Agent 实现提示

### 适用场景

当系统需要 2D 或 3D 激光雷达在大场景中进行实时 SLAM，要求子图级管理的低时延局部构图和高效的全局回环检测时，采用 Cartographer 的子图 + BnB 架构。适合室内移动机器人、物流仓储等场景，不适用于纯视觉 SLAM 或无结构化环境的纯 LiDAR 里程计。

### 输入输出契约

- **输入**：
  - 2D 激光扫描（`sensor::PointCloud`，含 range 信息和时间戳）；
  - IMU 数据（可选，用于姿态外推和重力对齐）；
  - 里程计数据（可选，作为初始位姿先验）；
- **输出**：
  - 子图网格列表（`ProbabilityGrid` 或 `TSDF2D`，每个子图含局部位姿和网格数据）；
  - 全局优化后的子图位姿 `global_submap_poses_2d`；
  - 回环约束 `std::vector<Constraint>`；
  - 全局一致的点云地图（通过合并各子图网格得到）；
- **坐标系**：sensor frame → tracking frame → local map frame → global frame。子图网格分辨率典型值 0.05 m，BnB 搜索窗口 ±7m/±30°；

### 实现骨架（伪代码）

```pseudo
function buildLocalSlam(submap, scan):
    // 1. 运动学/IMU 外推初始位姿
    initial_pose = pose_extrapolator.extrapolate(scan.timestamp)
    // 2. 体素降采样
    filtered_scan = voxel_filter(scan, resolution=0.025)
    // 3. 初匹配（相关扫描匹配）
    real_time_csm.match(filtered_scan, submap.grid, initial_pose) → pose
    // 4. Ceres 精匹配
    ceres_matcher.match(filtered_scan, submap.grid, pose) → refined_pose
    // 5. 插入子图
    submap.insert_range_data(filtered_scan, refined_pose)
    // 6. 子图轮换检查
    if submap.num_range_data >= max_per_submap:
        finish_current_submap()
        create_new_submap()

function detectLoopClosure(constraint_builder, finished_submaps):
    for each finished_submap as target:
        // 1. 采样当前scan
        sampled_scan = sample(current_scan, ratio=0.3)
        // 2. 构建多分辨率预计算网格栈
        grid_stack = PrecomputationGridStack2D(target.grid, depth=7)
        // 3. BnB搜索
        candidates = generate_lowest_resolution_candidates(
            discrete_scans, search_params)
        best = BranchAndBound(discrete_scans, search_params,
                              candidates, depth=7, min_score=0.55)
        // 4. 阈值通过后构建约束
        if best.score > min_score:
            constraint = {submap_id, node_id, relative_pose, INTER_SUBMAP}
            pending_constraints.push(constraint)

function runGlobalOptimization(constraints, submaps, nodes):
    problem = ceres.Problem()
    for each submap, node:
        problem.addParameterBlock(pose_submap, pose_node)
    for each constraint in constraints:
        residual = log(relative_pose^{-1} * (pose_node^{-1} * pose_submap))
        problem.addResidualBlock(residual, loss_function, pose_submap, pose_node)
    ceres.solve(problem)
    update_all_submap_and_node_poses()
```

### 关键源码片段

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc:L63-L107` — Ceres 扫描匹配的三种残差组合：

```cpp
ceres::Problem problem;
problem.AddResidualBlock(
    CreateOccupiedSpaceCostFunction2D(
        weight / sqrt(static_cast<double>(point_cloud.size())),
        point_cloud, grid),
    nullptr, ceres_pose_estimate);         // 占据代价
problem.AddResidualBlock(
    TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.translation_weight(), target_translation),
    nullptr, ceres_pose_estimate);         // 平移惩罚
problem.AddResidualBlock(
    RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.rotation_weight(), ceres_pose_estimate[2]),
    nullptr, ceres_pose_estimate);         // 旋转惩罚
ceres::Solve(ceres_solver_options_, &problem, summary);
```

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L171-L186` — 多分辨率预计算网格栈构造：

```cpp
PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve(
      (limits.num_x_cells + max_width - 1) * limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}
```

`raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L335-L378` — Branch-and-Bound 递归搜索（伪代码见上文，此处为原文）：

```cpp
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates,
    const int candidate_depth, float min_score) const {
  if (candidate_depth == 0) return *candidates.begin();
  Candidate2D best_high_resolution_candidate(0,0,0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) break;  // 剪枝
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width})
      for (int y_offset : {0, half_width})
        higher_resolution_candidates.emplace_back(
            candidate.scan_index,
            candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth-1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(..., higher_resolution_candidates,
                       candidate_depth-1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}
```

`raw/codes/cartographer/cartographer/mapping/2d/probability_grid.cc` — 概率网格的占据/空闲概率更新公式（hit/miss 更新），使用 `ValueConversionTables` 的预计算查找表将概率映射到 `[0, 255]` 的整数值，实现 O(1) 的网格更新。

### 实现注意事项

- **子图边界限制**：活跃子图一旦标记为 finished 就不可再插入新数据，必须在创建新子图前确认旧子图的数据完整性；
- **预计算网格的滑动窗口最大值**：`SlidingWindowMaximum` 使用单调双端队列保证 O(1) 均摊操作——推送时弹出队尾 ≤ 新值的元素，删除时仅当被删值等于队首才弹出。需要严格保持队列的非增序不变量；
- **BnB 剪枝效率**：搜索窗口大小（`linear_search_window`, `angular_search_window`）和深度（`branch_and_bound_depth`）直接影响 BnB 性能。窗口增大会使候选数指数增长，深度增大增加链式递归开销；
- **约束去重与时间窗**：回环约束在构建后不是立即发送到优化器，而是通过 `ConstraintBuilder` 的异步线程池处理。两个约束如果共享子图和节点且有足够的时差，可能都需要保留；
- **子图裁剪**：`OverlappingSubmapsTrimmer2D` 根据子图覆盖区域和创建时序裁剪冗余子图，释放内存并减少优化规模；
- **全局优化频率**：`optimize_every_n_nodes` 控制触发频率，典型值 90（每完成一个子图触发一次）。全局优化使用 SPARSE_SCHUR 或 ITERATIVE_SCHUR，在大规模问题中需要注意内存消耗；
- **坐标系一致性**：`local_pose()` 返回子图原点在全局坐标系中的位姿（`Rigid3d`），与 `global_submap_poses_2d` 中存储的位姿在 2D 投影下保持一致；

### 源码检索锚点

- `raw/codes/cartographer/cartographer/mapping/2d/submap_2d.h` — Submap2D 类定义
- `raw/codes/cartographer/cartographer/mapping/2d/submap_2d.cc:L70-L76` — 子图构造
- `raw/codes/cartographer/cartographer/mapping/2d/probability_grid.h` — 概率占据网格
- `raw/codes/cartographer/cartographer/mapping/2d/grid_2d.h` — 2D 网格基类
- `raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc:L63-L107` — Ceres 扫描匹配
- `raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.cc` — 占据代价函数
- `raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L36-L74` — 滑动窗口最大值
- `raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L91-L186` — 预计算网格
- `raw/codes/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc:L335-L378` — BnB 递归
- `raw/codes/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc:L861` — `RunOptimization` 全局优化
- `raw/codes/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc:L52-L67` — PoseGraph2D 构造
- `raw/codes/cartographer/cartographer/mapping/pose_graph.cc` — 约束定义和序列化

## 相关页面

- [[算法-Cartographer]]
- [[方法-Branch-and-Bound回环检测]]
- [[方法-子地图]]
- [[方法-概率占据网格]]
- [[概念-回环检测方法]]
- [[概念-位姿图优化]]
- [[组件-Ceres-Solver]]
- [[方法-Atlas多地图管理]]
