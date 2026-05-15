---
tags: [IESKF, 滤波, LiDAR, IMU, 状态估计]
sources:
  - wiki/sources/2026-04-28-fusions-slam-analysis.md
  - wiki/sources/2026-04-28-lightning-lm-analysis.md
  - wiki/sources/2026-04-29-fusions_slam-analysis.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
  - wiki/sources/2026-05-02-p2v-slam.md
created: 2026-04-29
updated: 2026-05-02
type: entity
---

# IESKF滤波器

> Iterated Error-State Kalman Filter: 在误差状态上进行迭代卡尔曼滤波，是 FAST-LIO 系列和 R3LIVE 等现代 LiDAR SLAM 系统的核心估计器。

## 核心原理

IESKF 将状态分解为名义状态（nominal state）和误差状态（error state）。名义状态通过 IMU 运动模型预测，误差状态在每个滤波周期内通过迭代线性化观测模型进行更新。相比标准 EKF，IESKF 在高度非线性系统中收敛性更好。

典型状态包含：

```text
x = {R, p, v, b_g, b_a, g}
```

其中 `R,p,v` 是姿态、位置和速度，`b_g,b_a` 是陀螺仪/加速度计 bias，`g` 是重力。名义状态保留在流形上，误差状态 `δx` 在线性空间中求解。

## 关键设计

- **误差状态参数化**：名义状态在流形上，误差状态在切空间（Lie algebra）上，线性化误差更小
- **迭代更新**：每次观测后用更新后的名义状态重新线性化观测模型，重复直至收敛
- **零空间保持**：可观测性约束确保不可观维度不引入虚假信息
- **iKD-Tree 加速**：FAST-LIO 使用增量 KD-Tree 实现高效点云配准

## 更新流程

1. IMU 高频传播名义状态和协方差。
2. 当前 LiDAR 点云 deskew 后，与局部地图建立点到面对应。
3. 构造观测残差 `r(x)` 和雅可比 `H`。
4. 在当前线性化点求解误差状态更新。
5. 将 `δx` 注入名义状态，并重新线性化，直到收敛或达到迭代上限。

IESKF 与滑动窗口 BA 的差异在于：它不保留大量历史关键帧变量，而是把历史信息压缩进当前状态协方差，因此延迟低、前端实时性强。

## 代表实现

- FAST-LIO / FAST-LIO2：IESKF + iKD-Tree 体素地图
- R3LIVE：IESKF + 辐射图重建
- lightning-lm：IESKF + IVox3d 哈希体素地图
- fusions_slam：自实现 18 维 IESKF，支持 LiDAR、位置、姿态和速度观测
- P2V-SLAM：IESKF + 隐式点-体素残差，IR-Net 输出残差和不确定度，替代显式点到面测量模型

## Agent 实现提示

### 适用场景

当 LiDAR 或视觉观测模型非线性强、残差需要在当前状态附近反复重线性化，并且系统希望保持低延迟单状态滤波时，使用 IESKF。典型场景是 FAST-LIO/FAST-LIVO2 风格的点到面残差更新：IMU 先传播状态和协方差，LiDAR 匹配局部地图后迭代更新位姿。

### 输入输出契约

- **输入**：传播状态 `state_propagat`、当前迭代状态 `state_`、点到面残差列表 `ptpl_list_`、点协方差/平面协方差、状态协方差 `cov`、最大迭代次数和收敛阈值。
- **输出**：更新后的名义状态 `state_`、残差统计、有效特征数、可选的后验协方差或用于后续传感器的共享状态。
- **残差形式**：每个 LiDAR 点产生一个标量点到面残差 `z = -distance_to_plane`，雅可比至少覆盖姿态误差和平移误差。

### 实现骨架（伪代码）

```pseudo
function ieskfPointToPlaneUpdate(state_propagat, state, residuals):
    for iter in 0..max_iterations:
        ptpl_list = buildPointToPlaneResiduals(state, local_map)
        H, R_inv, z = allocate(len(ptpl_list))
        for each point_plane in ptpl_list:
            point_body = extrinsic.R * point + extrinsic.t
            point_world = state.rot * point_body + state.pos
            R_inv[i] = inverseResidualVariance(point_plane, point_world, state.cov)
            H[i] = [skew(point_body) * state.rot.T * normal, normal]
            z[i] = -signedDistance(point_world, plane)
        K1 = inverse(H.T * R_inv * H + inv(state.cov))
        vec = state_propagat minus state
        solution = K1 * H.T * R_inv * z + vec - gainCorrection(vec)
        injectError(state, solution)
        if small(solution.rotation, solution.translation): break
    return state
```

### 关键源码片段

`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L407-L428`

```cpp
    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    MatrixXd Hsub(effct_feat_num_, 6);
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_);
    VectorXd meas_vec(effct_feat_num_);
    meas_vec.setZero();
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;
```

`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L452-L477`

```cpp
      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    auto vec = state_propagat - state_;
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    int minRow, minCol;
    state_ += solution;
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
```

### 实现注意事项

- 残差构造和状态注入必须使用同一种扰动定义；左扰动/右扰动混用会让姿态雅可比符号错误。
- `state_propagat - state_` 是迭代更新中保留 IMU 先验的关键项，不能简单丢弃。
- 有效点过少或平均残差异常时应跳过更新，避免错误地图面直接污染协方差。
- 点到面测量协方差应合并点自身协方差、平面拟合协方差和法向投影，而不是固定常数噪声。

### 源码检索锚点

- `VoxelMapManager::StateEstimation`
- `BuildResidualListOMP`
- `ptpl_list_`
- `Hsub_T_R_inv`
- `state_propagat - state_`

## 相关页面

- [[算法-FAST-LIO]]
- [[算法-R3LIVE]]
- [[架构-多传感器融合架构]]
- [[因子图vs滤波]]
- [[方法-IMU deskew]]
- [[方法-在线平面拟合]]
- [[算法-P2V-SLAM]]
- [[方法-隐式点-体素观测模型]]
