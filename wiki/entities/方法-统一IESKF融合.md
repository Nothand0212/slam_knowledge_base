---
tags: [传感器融合, 紧耦合, IESKF, FAST-LIVO2]
sources:
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-30
---

# 统一IESKF融合

> FAST-LIVO2 核心融合机制：单一 19 维 IESKF，LIO 更新后 VIO 以更新后的状态为起点继承协方差继续收缩。

## 核心思想

统一 IESKF 把 LiDAR、视觉和 IMU 都放进同一个误差状态和协方差矩阵中。LiDAR 更新后，视觉更新不是另起一个滤波器，而是在同一状态上继续线性化和收缩协方差。这样传感器之间的相关性由协方差自然传播，不需要手工把一个估计器的结果再转成另一个估计器的先验。

## 与分离 KF 的对比
- **分离 KF**（如 R3LIVE 双 ESIKF）：各自维护协方差，信息可能重复或丢失
- **统一 IESKF**：协方差矩阵自动关联 LiDAR 和视觉不确定性，不需要外部约束对齐

## 串行链式更新
1. LIO 更新 `_state` 和 `_state.cov`
2. VIO 以更新后的 `_state` 为起点，`state_propagat`（IMU 传播）作为先验
3. 先验传递：`vec = x_propagate - state` 携带 IMU 传播信息
4. VIO 雅可比相对于完整 19 维状态，不是只约束视觉子空间

## 工程边界

统一状态的优势是统计一致性，代价是模块耦合更强。任何一个传感器的错误残差都会直接影响全局协方差，因此需要严格的外点剔除、残差门限和退化检测。若视觉更新和 LiDAR 更新频率差异很大，还要明确串行更新顺序，避免旧视觉帧在新 LiDAR 状态上重复使用。

## Agent 实现提示

### 适用场景

当系统需要 LiDAR、视觉和 IMU 共享同一个名义状态与协方差，而不是维护两个独立滤波器再手工对齐时，使用统一 IESKF 融合。FAST-LIVO2 的工程模式是：IMU 传播产生 `state_propagat`，LIO 或 VIO 根据当前事件串行更新同一个 `_state`，视觉模块直接持有共享状态指针。

### 输入输出契约

- **输入**：统一状态 `_state`、IMU 传播状态 `state_propagat`、事件标志 `lio_vio_flg`、LiDAR 点云/视觉图像、局部体素地图、视觉点云缓存。
- **输出**：同一个 `_state` 的串行后验、用于下一次 IMU 传播的 `latest_ekf_state/latest_ekf_time`、共享视觉/雷达中间量。
- **融合顺序**：事件调度必须保证每次 VIO 更新看到的是最近一次 LIO/IMU 后的状态；更新后要刷新共享状态和时间戳，避免重复消费旧观测。

### 实现骨架（伪代码）

```pseudo
function initializeFusion():
    vio_manager.state = &_state
    vio_manager.state_propagat = &state_propagat
    configure lidar_to_camera and imu_to_lidar extrinsics

function stateEstimationAndMapping(meas):
    state_propagat = propagateImu(_state, meas.imu)
    if meas.flag == LIO or meas.flag == LO:
        voxelmap.state = _state
        voxelmap.StateEstimation(state_propagat)
        _state = voxelmap.state
        publish_lio_outputs()
    else if meas.flag == VIO:
        vio_manager.processFrame(meas.image, shared_points, voxel_map, last_lio_time)
    latest_ekf_state = _state
    latest_ekf_time = meas.update_time
```

### 关键源码片段

`raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L128-L147`

```cpp
  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();
```

`raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L267-L305`

```cpp
void LIVMapper::stateEstimationAndMapping()
{
  switch (LidarMeasures.lio_vio_flg)
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO()
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }

  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1))
  {
    vio_manager->plot_flag = true;
  }
  else
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);
```

`raw/codes/FAST-LIVO2/src/vio.cpp:L1488-L1506`

```cpp
    if (error <= last_error)
    {
      old_state = (*state);
      last_error = error;

      auto &&H_sub_T = H_sub.transpose();
      H_T_H.setZero();
      G.setZero();
      H_T_H.block<6, 6>(0, 0) = H_sub_T * H_sub;
      MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
      auto &&HTz = H_sub_T * z;
      auto vec = (*state_propagat) - (*state);
      G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
      auto solution = -K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
      (*state) += solution;
      auto &&rot_add = solution.block<3, 1>(0, 0);
      auto &&t_add = solution.block<3, 1>(3, 0);

      if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f)) { EKF_end = true; }
```

### 实现注意事项

- VIO 管理器持有 `_state` 和 `state_propagat` 的指针，意味着 LIO 与 VIO 更新同一份状态；实现时要避免复制后忘记写回。
- 统一 IESKF 不等于并行融合：同一时刻多个观测应有明确顺序，通常先 IMU 传播，再按事件串行执行 LIO/VIO。
- 视觉更新中的 `vec = (*state_propagat) - (*state)` 保留了 IMU 传播先验，与 LiDAR IESKF 的结构一致。
- 外点剔除必须在每个传感器内部完成；统一协方差会放大坏残差的跨传感器影响。

### 源码检索锚点

- `vio_manager->state = &_state`
- `stateEstimationAndMapping`
- `handleLIO`
- `handleVIO`
- `VIOManager::updateState`
- `(*state_propagat) - (*state)`

## 相关页面

- [[算法-FAST-LIVO2]]
- [[方法-IESKF滤波器]]
- [[架构-双ESIKF架构]]
- [[架构-多传感器融合架构]]
