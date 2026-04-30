---
tags: [LiDAR, 回环检测, 描述子]
sources:
  - wiki/sources/2026-04-28-4d-radar-slam-analysis.md
  - wiki/sources/2026-04-29-fast_lio_sam-analysis.md
created: 2026-04-29
updated: 2026-04-30
type: entity
---

# ScanContext

> 一种基于俯视图的 LiDAR 地点描述子，将 3D 点云压缩为 2D 矩阵并通过旋转不变性匹配实现高效回环检测。

## 核心思想

ScanContext 将一帧 3D LiDAR 扫描投影到极坐标鸟瞰网格。通常把半径方向划分为 ring，把方位角划分为 sector，每个格子存储该区域点的最大高度或强度统计量，得到一个 `N_ring x N_sector` 的矩阵。矩阵列方向的循环平移对应车辆 yaw 角变化，因此可以通过列移位搜索同时完成地点相似度计算和粗 yaw 估计。

检索一般分两步：

1. 用 ring key 或 sector key 做近邻检索，快速筛出少量候选历史帧。
2. 对候选矩阵做 column-wise 距离，枚举列移位，取最小距离作为地点相似度，并把最优列移位作为初始 yaw。

## 在 SLAM 中的应用

ScanContext 常作为 LiDAR 回环检测的候选生成器，而不是最终回环判定器。LIO-SAM、FAST-LIO-SAM-SC-QN 等系统通常先用 ScanContext 召回候选，再用距离门限、Quatro、GICP/Nano-GICP 或 ICP fitness 做几何验证，最后才把回环边加入位姿图。

这种分层设计的原因是：描述子检索擅长大范围召回，但在对称道路、长走廊、重复结构和动态车流场景中容易误匹配；几何配准能给出相对位姿和协方差，但直接全库 ICP 成本过高。ScanContext 位于两者之间，负责把全局搜索问题压缩成少数候选。

## 设计边界

- 对 roll/pitch 变化不大的车载 LiDAR 很有效；对剧烈姿态变化或非水平安装，需要先做姿态归一化。
- 高度版 ScanContext 依赖垂直结构；4D 雷达或低线束 LiDAR 可改用强度/SNR，形成 [[方法-Intensity Scan Context]]。
- 输出的 yaw 只能作为粗初值，不能替代后续的 6-DoF 配准和回环一致性检查。

## Agent 实现提示

### 适用场景

用于 LiDAR SLAM 回环候选生成：在全历史关键帧中快速召回少量可能闭环，再交给 Quatro、GICP、ICP 或位姿图一致性检查验证。适合车载、地面机器人等 roll/pitch 较稳定场景。

### 输入输出契约

- 输入：当前关键帧点云、历史 ScanContext 描述子库、ring key KD-tree、最近帧排除窗口、候选数量、距离阈值。
- 输出：候选历史帧 id、ScanContext 距离、粗 yaw 偏移；若无候选则返回 `-1` 或空结果。
- 前置条件：关键帧点云已下采样；描述子参数 `num_ring/num_sector/max_radius` 与历史库一致；最近若干帧应排除，避免把连续帧当回环。

### 实现骨架（伪代码）

```text
function make_scan_context(scan):
    desc <- zeros(num_ring, num_sector)
    for point in scan:
        range <- sqrt(x^2 + y^2)
        angle <- atan2(x, y)
        if outside_roi(range, angle): continue
        ring <- discretize_range(range)
        sector <- discretize_angle(angle)
        desc[ring, sector] <- max(desc[ring, sector], height_or_intensity(point))
    return desc

function detect_loop(query_scan):
    curr_desc <- make_scan_context(query_scan)
    curr_key <- ring_mean(curr_desc)
    candidates <- kd_tree.knn(curr_key, k)
    for candidate in candidates:
        dist, shift <- column_shift_distance(curr_desc, history[candidate])
        keep minimum dist
    if minimum dist < threshold:
        return candidate_id, shift * sector_angle
    return no_loop
```

### 关键源码片段

`raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp:L162-L190`

```cpp
MatrixXd SCManager::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
{
    TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).
        pt.intensity = _scan_down.points[pt_idx].intensity;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        // azim_angle = xy2theta(pt.x, pt.y);
        azim_angle = (atan2f(pt.x, pt.y) - M_PI_2) * 180/M_PI;

        if( abs(azim_angle) > PC_AZIMUTH_ANGLE_MAX)
            continue;
        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;
```

`raw/codes/FAST-LIO-SAM-SC-QN/fast_lio_sam_sc_qn/src/loop_closure.cpp:L34-L55`

```cpp
void LoopClosure::updateScancontext(pcl::PointCloud<PointType> cloud)
{
    sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}

int LoopClosure::fetchCandidateKeyframeIdx(const PosePcd &query_keyframe,
                                           const std::vector<PosePcd> &keyframes)
{
    // from ScanContext, get the loop candidate
    std::pair<int, float> sc_detected_ = sc_manager_.detectLoopClosureIDGivenScan(query_keyframe.pcd_); // int: nearest node index,
                                                                                                        // float: relative yaw
    int candidate_keyframe_idx = sc_detected_.first;
    if (candidate_keyframe_idx >= 0) // if exists
    {
        // if close enough
        if ((keyframes[candidate_keyframe_idx].pose_corrected_eig_.block<3, 1>(0, 3) - query_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
                .norm() < config_.scancontext_max_correspondence_distance_)
        {
            return candidate_keyframe_idx;
        }
    }
    return -1;
}
```

`raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp:L316-L348`

```cpp
    // knn search
    std::vector<size_t> knn_candidate_indexes( NUM_CANDIDATES_FROM_TREE );
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &knn_candidate_indexes[0], &out_dists_sqr[0] );
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) );
    t_tree_search.toc("Tree search");

    /*
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    TicToc t_calc_dist;
    for ( int iter_idx = 0; iter_idx < NUM_CANDIDATES_FROM_TREE; iter_idx++ )
    {
        if (knn_candidate_indexes[iter_idx] > candidate_keyframe_indices.size()-1){  // To avoid out-of-range knn indice
            ROS_INFO("To skip out-of-range knn indice: %ld",knn_candidate_indexes[iter_idx]);
            continue;
        }
        MatrixXd polarcontext_candidate = polarcontexts_[candidate_keyframe_indices[knn_candidate_indexes[iter_idx]]];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate );
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_keyframe_indices[knn_candidate_indexes[iter_idx]];
        }
```

### 实现注意事项

- 描述子矩阵的列循环平移只提供 yaw 粗估计，不能直接作为闭环约束。
- ring key KD-tree 只做召回；最终排序要重新计算完整 ScanContext 距离。
- 高度统计、强度统计和雷达 SNR 统计不可混用同一阈值，应随传感器重标定。
- 必须排除最近关键帧，并在候选后加入距离门限或 GICP/ICP 验证，降低重复结构误检。

### 源码检索锚点

- `SCManager::makeScancontext`
- `makeRingkeyFromScancontext`
- `detectLoopClosureIDGivenScan`
- `distanceBtnScanContext`
- `LoopClosure::fetchCandidateKeyframeIdx`

## 相关页面

- [[概念-回环检测方法]], [[组件-DBoW2]]
- [[算法-LIO-SAM]], [[算法-FAST-LIO-SAM-SC-QN]]
- [[方法-Intensity Scan Context]], [[方法-四阶段回环验证]]
- [[概念-位姿图优化]]
