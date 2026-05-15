---
tags: [方法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-GTSAM-Ceres工程因子]]
sources:
  - wiki/sources/2026-04-29-kimera_vio-analysis.md
---

> 本页内容已归并至 [[方法-GTSAM-Ceres工程因子]]。

# SmartStereoFactor

> GTSAM smart factor 在双目 VIO 中的应用：内部三角化并消去路标，只把多帧双目观测转化为位姿约束。

## 工作方式

GTSAM `SmartStereoProjectionPoseFactor` 收集同一个路标在多个双目关键帧中的观测 `(uL, uR, v)`，在因子内部完成三角化、重投影误差计算和路标消元。外部因子图只暴露相机位姿变量，不显式加入 landmark 变量，因此能减少变量数量并保持后端稀疏。

和普通 BA 的区别在于：普通 BA 把位姿和路标都作为优化变量；smart factor 把路标作为可被三角化的中间量，通过 [[概念-Schur补与边缘化]] 或等价的消元过程把多帧视觉观测压缩成位姿之间的约束。

## 关键参数

- `rank_tolerance`：判断三角化约束是否退化。
- `landmark_distance_threshold`：过滤过远或数值不稳定的路标。
- `retriangulation_threshold`：控制何时重新三角化。
- `outlier_rejection`：在重投影误差过大时拒绝观测。

## 工程价值与边界

SmartStereoFactor 适合 Kimera-VIO 这类双目惯性系统：双目给出更稳定的初始深度，多帧观测提供几何约束，GTSAM iSAM2 可增量更新位姿图。它的边界是对特征跟踪质量敏感，低视差、远点、重复纹理和错误匹配都会导致三角化退化；因此前端仍需要 KLT/NCC、RANSAC 和关键帧筛选。

## Agent 实现提示

### 适用场景

当 Agent 需要在双目 VIO 后端把一条 landmark track 转成只约束位姿的视觉因子时使用本提示。SmartStereoFactor 适合轨迹长度至少 2、双目观测质量稳定、希望避免显式 landmark 变量膨胀的增量优化。

### 输入输出契约

- **输入**：`FeatureTrack`（`LandmarkId` + 多帧 `StereoPoint2(uL, uR, v)`）、对应帧 ID、双目标定 `stereo_cal_`、智能因子噪声模型和参数、左相机到机体系外参。
- **输出**：新建或更新后的 `SmartStereoFactor`、因子图 slot、landmark 是否仍为 smart 的状态。
- **失败契约**：观测数不足、内部三角化点无效、退化、远点、外点或点在相机后方时，不转换为可信 projection factor；应保留为 smart、剔除或等待更多观测。

### 实现骨架（伪代码）

```python
def add_or_update_smart_stereo_factor(track, graph_state):
    if track.landmark_id not in graph_state.old_smart_factors:
        factor = SmartStereoProjectionPoseFactor(noise, params, body_P_left_rect)
        for frame_id, stereo_px in track.observations:
            factor.add(stereo_px, Symbol("x", frame_id), stereo_cal)
        graph_state.new_smart_factors[track.landmark_id] = factor
        return factor

    old_factor = graph_state.old_smart_factors[track.landmark_id]
    factor = clone(old_factor)
    factor.add(track.latest_stereo_px, Symbol("x", track.latest_frame_id), stereo_cal)

    if not is_smart_factor_point_good(factor, min_num_observations):
        return Pending("smart factor triangulation not reliable")
    return factor
```

### 关键源码片段

`raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L478-L504`

```cpp
void RegularVioBackend::addLandmarkToGraph(const LandmarkId& lmk_id,
                                           const FeatureTrack& ft,
                                           LmkIdIsSmart* lmk_id_is_smart) {
  CHECK_NOTNULL(lmk_id_is_smart);
  // All landmarks should be smart the first time we add them to the graph.
  // Add as a smart factor.
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor(new SmartStereoFactor(
      smart_noise_, smart_factors_params_, B_Pose_leftCamRect_));

  VLOG(20) << "Adding landmark with id: " << lmk_id
           << " for the first time to graph. \n"
           << "Nr of observations of the lmk: " << ft.obs_.size()
           << " observations.\n";
  if (VLOG_IS_ON(30)) {
    new_factor->print();
  }

  // Add observations to smart factor.
  VLOG(20) << "Creating smart factor involving lmk with id: " << lmk_id;
  for (const std::pair<FrameId, StereoPoint2>& obs : ft.obs_) {
    VLOG(20) << "SmartFactor: adding observation of lmk with id: " << lmk_id
             << " from frame with id: " << obs.first;
    gtsam::Symbol pose_symbol('x', obs.first);
    new_factor->add(obs.second, pose_symbol, stereo_cal_);
  }
```

`raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L578-L604`

```cpp
void RegularVioBackend::updateExistingSmartFactor(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_obs,
    LandmarkIdSmartFactorMap* new_smart_factors,
    SmartFactorMap* old_smart_factors) {
  CHECK_NOTNULL(new_smart_factors);
  CHECK_NOTNULL(old_smart_factors);
  CHECK_NE(lmk_id, -1) << "When calling update existing smart factor, the slot "
                          "should already be != -1! \n";

  // Update existing smart-factor.
  const SmartFactorMap::iterator& old_smart_factors_it =
      old_smart_factors->find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors->end())
      << "Landmark with id: " << lmk_id << " not found in old_smart_factors_\n";

  // Get old factor.
  SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
  CHECK(old_factor);

  // Clone old factor as a new factor.
  SmartStereoFactor::shared_ptr new_factor(new SmartStereoFactor(*old_factor));

  // Add observation to new factor.
  VLOG(20) << "Added observation for smart factor of lmk with id: " << lmk_id;
  new_factor->add(
      new_obs.second, gtsam::Symbol('x', new_obs.first), stereo_cal_);
```

`raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L963-L976`

```cpp
bool RegularVioBackend::isSmartFactor3dPointGood(
    SmartStereoFactor::shared_ptr factor,
    const size_t& min_num_of_observations) {
  CHECK(factor);
  if (!(factor->point().valid())) {
    // The point is not valid.
    VLOG(20) << "Smart factor is NOT valid.";
    return false;
  } else {
    CHECK(factor->point());
    CHECK(!factor->isDegenerate());
    CHECK(!factor->isFarPoint());
    CHECK(!factor->isOutlier());
    CHECK(!factor->isPointBehindCamera());
```

`raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L992-L1005`

```cpp
    if (factor->measured().size() >= min_num_of_observations) {
      VLOG(20) << "Smart factor is valid with: " << factor->measured().size()
               << " observations (wo the one we are going to add now).";
      return true;
    } else {
      // Should not be a warning this, but just in case.
      LOG(WARNING) << "Smart factor has not enough"
                   << " observations: " << factor->measured().size()
                   << ", but should be more or equal to "
                   << min_num_of_observations;
      return false;
    }
  }
}
```

### 实现注意事项

- Smart factor 的观测绑定的是 pose symbol，不显式创建 landmark 变量；不要在同一条 track 上同时创建普通 landmark BA 变量，除非有明确转换逻辑。
- 更新旧 smart factor 时采用 clone-and-add，避免直接修改已在线性化或已插入图中的因子。
- `factor->point().valid()` 只是第一层检查，还要处理退化、远点、外点和 behind-camera。
- `min_num_of_observations` 应大于仅用于创建 smart factor 的最短 track 长度，否则会过早相信弱几何约束。

### 源码检索锚点

- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp`：`addLandmarkToGraph`、`updateExistingSmartFactor`、`isSmartFactor3dPointGood`
- `raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h`：`SmartStereoFactor`、`FeatureTrack`、`LandmarkIdSmartFactorMap`
- `raw/codes/Kimera-VIO/include/kimera-vio/backend/VioBackend.h`：`smart_factors_params_`、`setSmartStereoFactorsParams`
- `raw/codes/Kimera-VIO/tests/testVioBackend.cpp`：smart factor 数量和图结构断言

## 相关页面
- [[算法-Kimera-VIO]]
- [[概念-Schur补与边缘化]]
- [[组件-GTSAM]]
- [[概念-三角化与深度估计]]
- [[GTSAM SLAM 与视觉因子 API]]
