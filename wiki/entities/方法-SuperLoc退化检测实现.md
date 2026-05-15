---
type: entity
tags: [退化检测, SuperLoc, SuperOdom, 协方差分解, 特征值, LiDAR, 特征可观测性, 先验注入]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/SuperOdom
---

# SuperLoc 退化检测实现

> SuperOdom/SuperLoc 的双层退化检测体系：第一层对每个平面特征做 9 维可观测性分析，聚合为 6-DoF 不确定性直方图；第二层对 Ceres 协方差矩阵做特征值分解，按 DoF 注入加权先验约束，并在退化时切换预测源。

## 问题

LiDAR scan-to-map 优化在几何特征不足的方向上是病态的。典型的退化场景：

- **长走廊**：走廊轴线（通常 x 方向）的平移不可观测，墙面只在垂直于轴线的 y/z 方向提供约束
- **开阔地**：地面平面约束 roll/pitch 和 z，但 x/y 平移和 yaw 旋转完全不可观测
- **单一平面**（如贴墙飞行）：无法估计沿墙面法向的平移

若不做退化检测，Ceres/GTSAM 优化器会在无约束方向产生"看起来收敛但实际任意"的位姿增量。

## 双层退化检测架构

SuperLoc 实现两层互补的退化感知：

```
特征级可观测性分析 (每帧)
    ↓
  PlaneFeatureHistogramObs[9] 直方图
    ↓
  EstimateLidarUncertainty() → 6 维不确定性 (0~1)
    ↓
优化级协方差分解 (Ceres Covariance)
    ↓
  EstimateRegistrationError() → 6×6 Hessian 特征值 + 方向
    ↓
退化响应:
  ├── isDegenerate 标志 → 切换预测源 (VIO/Neural-IMU)
  └── 按 (1-uncertainty) 注入加权先验约束
```

## 第一层：特征可观测性分析

### 原理

对每个平面特征点，评估它对 6 个旋转方向和 3 个平移方向的可观测性贡献：

- **旋转可观测性**：$\text{cross} = \mathbf{p} \times \mathbf{n}$（点 p 与法向量 n 的叉积），分解到世界坐标系的 3 个轴及其反方向，得到 6 维旋转可观测性指标
- **平移可观测性**：$\text{planar}^2 \cdot |\mathbf{n} \cdot \mathbf{axis}|$，平面法向在轴方向的分量越大，该轴向平移越可观测

每个特征贡献其质量最高的 top-2 旋转方向和 top-2 平移方向，计入 `PlaneFeatureHistogramObs[9]` 直方图。

### 源码

```cpp
// LidarSlam.cpp:L660-L685 — 对每个特征排序并选出 top observability
void analyzeFeatureObservability(pcaFeature &feature) {
    std::vector<QualityPair> rotation_quality = {
        {feature.rx_cross, rx_cross}, {feature.neg_rx_cross, neg_rx_cross},
        {feature.ry_cross, ry_cross}, {feature.neg_ry_cross, neg_ry_cross},
        {feature.rz_cross, rz_cross}, {feature.neg_rz_cross, neg_rz_cross}};
    std::vector<QualityPair> trans_quality = {
        {feature.tx_dot, tx_dot}, {feature.ty_dot, ty_dot}, {feature.tz_dot, tz_dot}};
    std::sort(rotation_quality.begin(), rotation_quality.end(), ...);
    std::sort(trans_quality.begin(), trans_quality.end(), ...);
    // 取 top-2 rotation + top-2 translation
    feature.observability.at(0) = rotation_quality.at(0).second;
    feature.observability.at(1) = rotation_quality.at(1).second;
    feature.observability.at(2) = trans_quality.at(0).second;
    feature.observability.at(3) = trans_quality.at(1).second;
}
```

每个匹配特征成功后累加直方图：

```cpp
// LidarSlam.cpp:L342-L345
const auto &obs = constraint.feature.observability;
PlaneFeatureHistogramObs[obs[0]]++;
PlaneFeatureHistogramObs[obs[1]]++;
PlaneFeatureHistogramObs[obs[2]]++;
```

### 不确定性估计

直方图归一化为 0~1 的不确定性值：

```cpp
// LidarSlam.cpp:L921-L959 — EstimateLidarUncertainty()
// 平移不确定性
uncertaintyX = (PlaneFeatureHistogramObs[6] / TotalTransFeature) * 3;
uncertaintyX = min(uncertaintyX, 1.0);  // 钳位到 [0,1]
// 旋转不确定性
uncertaintyRoll = ((PlaneFeatureHistogramObs[0]+PlaneFeatureHistogramObs[1]) / TotalRotationFeature) * 3;
uncertaintyRoll = min(uncertaintyRoll, 1.0);
```

不确定性越低（接近 0），该方向越可靠；越高（接近 1），越退化。

## 第二层：优化级协方差特征值分解

### 数学推导

Ceres 求解结束后，计算参数块的协方差矩阵 $C \in \mathbb{R}^{6 \times 6}$（在切空间上），对应 6-DoF 位姿。

协方差的逆近似 Hessian：
$$H \approx C^{-1} = J^T J$$

对 $C$ 的平移子块（3×3）和旋转子块（3×3）分别做特征值分解：

$$\Sigma_{t} = V_t \Lambda_t V_t^T, \quad \Lambda_t = \text{diag}(\lambda_{t1}, \lambda_{t2}, \lambda_{t3})$$
$$\Sigma_{r} = V_r \Lambda_r V_r^T, \quad \Lambda_r = \text{diag}(\lambda_{r1}, \lambda_{r2}, \lambda_{r3})$$

- **最大不确定度方向**：最大特征值 $\lambda_{\max}$ 对应的特征向量
- **条件数**：$\kappa = \sqrt{\lambda_{\max}} / \sqrt{\lambda_{\min}}$，越大越病态

### 源码

```cpp
// LidarSlam.cpp:L860-L894 — EstimateRegistrationError()
// 1. 通过 Ceres Covariance API 获取 6×6 协方差
ceres::Covariance covarianceSolver(covOptions);
covarianceSolver.Compute(covarianceBlocks, &problem);
covarianceSolver.GetCovarianceBlockInTangentSpace(paramBlock, paramBlock, err.Covariance.data());

// 2. 平移子块特征值分解
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3,3>());
err.PositionError = sqrt(eigPosition.eigenvalues()(2));             // 最大平移标准差
err.PositionErrorDirection = eigPosition.eigenvectors().col(2);     // 最大误差方向
err.PosInverseConditionNum = sqrt(λ0) / sqrt(λ2);                   // 条件数倒数

// 3. 旋转子块特征值分解
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3,3>());
err.OrientationError = Rad2Deg(sqrt(eigOrientation.eigenvalues()(2)));
err.OriInverseConditionNum = sqrt(λ0) / sqrt(λ2);
```

## 退化响应策略

### 1. 预测源切换

退化时，IMU 姿态预测不可靠，切换为更鲁棒的 VIO 或 Neural-IMU 预测：

```cpp
// laserMapping.cpp:L384-L411 — determinePredictionSource()
if (slam.isDegenerate) {
    if (vio_prediction_status)  return VIO_ODOM;
    if (nio_prediction_status)  return NEURAL_IMU_ODOM;
} else {
    if (lio_prediction_status)  return LIO_ODOM;
    if (imu_orientation_status) return IMU_ORIENTATION;
}
return CONSTANT_VELOCITY;  // 全部不可用时的兜底
```

### 2. 加权先验注入

退化方向加绝对位姿约束，权重与可观测性成正比：

```cpp
// LidarSlam.cpp:L287-L303 — addAbsolutePoseConstraints()
// 仅在 VIO 可用且 isDegenerate 时注入
information(0,0) = (1 - uncertainty_x) * max(50, good_feature_num*0.1) * Visual_confidence_factor;
information(1,1) = (1 - uncertainty_y) * max(50, good_feature_num*0.1) * Visual_confidence_factor;
information(2,2) = (1 - uncertainty_z) * max(50, good_feature_num*0.1) * Visual_confidence_factor;
// yaw 方向信息为 0（不约束），由 LiDAR 配准完全决定
information(5,5) = max(5, good_feature_num*0.001) * 0;
SE3AbsolutatePoseFactor *factor = new SE3AbsolutatePoseFactor(position, information);
problem.AddResidualBlock(factor, nullptr, pose_parameters);
```

关键设计：特征越少的方向先验越强（$1-\text{uncertainty}$ 越大），且 yaw 方向信息矩阵置 0，让 LiDAR 配准自由估计。

### 3. 退化信号传递

```cpp
// laserMapping.cpp:L563-L567
if (slam.isDegenerate) {
    odomAftMapped.pose.covariance[0] = 1;  // 标记退化状态
} else {
    odomAftMapped.pose.covariance[0] = 0;
}
```

## 伪代码

```pseudo
function SuperLocDegeneracyHandling(edge_pts, plane_pts, initial_pose):
    // === 第一层：特征可观测性分析 ===
    histogram[9] = zeros()
    for each plane_point in plane_pts:
        feature = computePCA(plane_point, local_map)
        obs = analyzeFeatureObservability(feature, current_pose)
        histogram[obs[0]]++  // top-2 rotation
        histogram[obs[1]]++
        histogram[obs[2]]++  // top-2 translation

    // 归一化为不确定性
    for dof in [x,y,z,roll,pitch,yaw]:
        uncertainty[dof] = clamp(histogram_base[dof] / total_base[dof] * 3, 0, 1)

    // === 第二层：Ceres 协方差分解 ===
    ceres_solve(problem)
    covariance_6x6 = ceres.Covariance(problem)
    eig_t = eigen(covariance_6x6[0:3, 0:3])  // 平移
    eig_r = eigen(covariance_6x6[3:6, 3:6])  // 旋转

    // === 退化响应 ===
    if any eigenvalue < threshold:
        isDegenerate = true
        prediction_source = prefer(VIO_ODOM > NEURAL_IMU_ODOM)
        // 按 DoF 注入加权先验
        info[i,i] = (1 - uncertainty[i]) * feature_weight * visual_confidence
        add_prior_factor(initial_pose, info)

    return optimized_pose, isDegenerate, uncertainty
```

## Agent 实现提示

### 适用场景

- LiDAR 在几何退化环境（走廊/隧道/开阔地）中的在线鲁棒状态估计
- 需要 6-DoF 级别的不确定性量化时（如规划器需要知道哪些方向可靠）
- 多传感器融合中当 LiDAR 退化时主动切换预测源

### 输入输出契约

- **输入**：当前帧平面特征点云、地图、初始位姿、VIO/IMU/LIO 预测状态
- **输出**：优化后位姿、6 维不确定性 [0,1]、退化标志 `isDegenerate`、预测源枚举
- **坐标系**：特征在世界系做 PCA；叉积和点积在旋转后的世界系计算

### 实现注意事项

- `PlaneFeatureHistogramObs` 每帧必须清零（`LidarSlam.cpp:L857`），否则直方图累积跨帧污染
- Ceres 协方差计算需要 `apply_loss_function = true`，否则鲁棒核的效果未反映在协方差中
- `addAbsolutePoseConstraints` 只在 VIO 预测可用时调用，防止退化时注入错误先验
- yaw 方向信息矩阵置零是关键设计决策——走廊场景 yaw 变化由 LiDAR 自身约束

### 源码检索锚点

- 特征可观测性分析：`raw/codes/SuperOdom/super_odometry/src/LidarProcess/LidarSlam.cpp:L580-L685`
- 不确定性估计：`raw/codes/SuperOdom/super_odometry/src/LidarProcess/LidarSlam.cpp:L921-L992`
- Ceres 协方差分解：`raw/codes/SuperOdom/super_odometry/src/LidarProcess/LidarSlam.cpp:L860-L894`
- 加权先验注入：`raw/codes/SuperOdom/super_odometry/src/LidarProcess/LidarSlam.cpp:L287-L303`
- 预测源切换：`raw/codes/SuperOdom/super_odometry/src/LaserMapping/laserMapping.cpp:L384-L411`
- 退化标志发布：`raw/codes/SuperOdom/super_odometry/src/LaserMapping/laserMapping.cpp:L563-L567`
- 数据结构定义：`raw/codes/SuperOdom/super_odometry/include/super_odometry/LidarProcess/LidarSlam.h:L97-L108`（`Feature_observability`）、`:L230-L265`（成员变量）

## 相关页面

- [[方法-6-DoF 退化检测]]
- [[方法-LIO-SAM退化处理]]
- [[方法-ICP配准方法]]
- [[方法-多源位姿预测策略]]
- [[算法-LIO-SAM]]
