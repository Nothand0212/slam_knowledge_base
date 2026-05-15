---
tags: [4D雷达, SLAM, 多传感器, 回环检测, ISC, Doppler速度]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/4DRadarSLAM
---

# 4DRadarSLAM 工程

> 基于 4D 毫米波雷达的图化 SLAM 系统：从雷达点云预处理、Doppler 自速度估计、ISC 回环检测到 g2o 位姿图优化的完整工程管线。

## 4D 雷达测量模型

4D 雷达每帧输出稀疏点云，每个点携带五个观测量：

| 物理量 | 符号 | 单位 | 精度特性 |
|--------|------|------|----------|
| 径向距离 | $r$ | m | 高精度，0.1m 级 |
| 方位角 | $\theta$ | rad/deg | 角分辨率 ~1° |
| 俯仰角 | $\phi$ | rad/deg | 精度较弱，~1° |
| Doppler 速度 | $v_d$ | m/s | 径向分量，高精度 |
| SNR / 强度 | $I$ | dB | 与表面反射特性相关 |

点的 3D 坐标由球坐标转换：

$$x = r \cos\phi \cos\theta, \quad y = r \cos\phi \sin\theta, \quad z = r \sin\phi$$

Doppler 观测方程（静态目标假设）：

$$v_d = -\mathbf{n}^T \mathbf{v}_r, \quad \mathbf{n} = [x/r, y/r, z/r]^T$$

其中 $\mathbf{v}_r$ 是雷达在 body 系下的 3D 速度，$\mathbf{n}$ 是点到雷达的单位方向向量。

## 预处理管线

`raw/codes/4DRadarSLAM/apps/preprocessing_nodelet.cpp` 实现了完整的雷达点云预处理链：

```
原始雷达点云 (raw/codes/4DRadarSLAM/apps/preprocessing_nodelet.cpp:L267-L301)
├── Power 阈值过滤 (power_threshold)
├── 坐标变换 Radar→LiDAR (外参矩阵 Radar_to_livox)
├── Ego Velocity 估计 (Doppler RANSAC LSQ)
│   ├── 有效目标筛选 (min_dist, max_dist, azimuth/elevation FOV, min SNR)
│   ├── 静止检测 (Doppler 绝对值中位数 < thresh_zero_velocity)
│   ├── RANSAC 3D 最小二乘 (solve3DFullRansac)
│   └── 协方差估计 (solve3DFull, sigma_v_r)
├── 动态物体去除 (可选, enable_dynamic_object_removal)
├── Deskew 去畸变 (IMU 角速度, scan_period)
├── 距离滤波 (distance_filter: near/far/z_low/z_high)
├── 体素降采样 (VoxelGrid 0.1m) 或 ApproxVoxelGrid
└── 离群点剔除 (StatisticalOutlierRemoval 或 RadiusOutlierRemoval)
```

### Doppler 自速度估计

`raw/codes/4DRadarSLAM/src/radar_ego_velocity_estimator.cpp:L252-L303` — `solve3DFull`：

将 Doppler 观测构造成线性系统：

$$\begin{bmatrix} \mathbf{n}_1^T \\ \vdots \\ \mathbf{n}_N^T \end{bmatrix} \mathbf{v}_r = \begin{bmatrix} v_{d,1} \\ \vdots \\ v_{d,N} \end{bmatrix}$$

其中 $H \in \mathbb{R}^{N \times 3}$，$y \in \mathbb{R}^{N}$。最小二乘解：

$$\mathbf{v}_r = (H^T H)^{-1} H^T y$$

使用 `bdcSvd` 或 Cholesky 分解求解。RANSAC 变体 (`solve3DFullRansac`, L172-L250) 通过随机采样 $N_{\text{ransac}}$ 个点迭代求解，选取内点最多的模型。协方差估计：

$$\sigma_{v_r} = \sqrt{\text{diag}\left(\frac{\mathbf{e}^T\mathbf{e}}{N-3} (H^T H)^{-1}\right)} + \sigma_{\text{offset}}$$

关键阈值：
- `min_dist`/`max_dist`: 距离范围 (1.0/100.0 m)
- `azimuth_thresh_deg`/`elevation_thresh_deg`: 角度 FOV (~60°/~10°)
- `min_db`: SNR 最低阈值
- `thresh_zero_velocity`: 静止判断中位数阈值
- `N_ransac_points`: RANSAC 采样点数 (3)

## ISC (Intensity Scan Context) 回环检测

`raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp:L162-L214` — `makeScancontext`：

描述子维度：$N_{\text{ring}} = 20$（径向环）× $N_{\text{sector}} = 60$（方位扇区）

**构建流程**：
1. 对每个点计算极坐标 `(azim_range, azim_angle)`
2. 分别映射到 ring_idx 和 sector_idx
3. 每个 bin 取最大 SNR 值（而不是最大高度，因为雷达 z 精度弱）
4. 无点 bin 填充 0

**检索流程** (`detectLoopClosureID`, L272-L374)：
1. **Ring Key KD 树**：每行均值组成 RingKey (20D 向量)，构建 nanoflann KD 树
2. **KNN 搜索**：查询最近 $N_{\text{candidates}}$ 个候选
3. **V-Key 快速对齐**：扇区均值向量 `fastAlignUsingVkey` 估计旋转偏移
4. **精细余弦距离**：在 $\pm \text{SEARCH\_RADIUS}$ 扇区偏移范围内计算 `distDirectSC`，取最小距离
5. **阈值判决**：`min_dist < SC_DIST_THRES`（默认 0.3-0.5）

`distDirectSC` (L80-L101)：对每一列计算余弦相似度，排除全零列后取均值：

$$\text{dist} = 1 - \frac{1}{N_{\text{eff}}} \sum_{i} \frac{\mathbf{c}_{1,i} \cdot \mathbf{c}_{2,i}}{|\mathbf{c}_{1,i}| \, |\mathbf{c}_{2,i}|}$$

## 五层回环几何验证

`raw/codes/4DRadarSLAM/src/radar_graph_slam/loop_detector.cpp:L100-L332`：

```
Layer 1: 预筛选 (find_candidates)
  ├── 距上次回环累计距离 > min_loop_interval_dist (5m)
  ├── 两点间累计行驶距离 > accum_distance_thresh (8m)
  ├── 气压计高度差 < max_baro_difference (3m)
  ├── 偏航角差 < max_yaw_difference (45°)
  └── 椭圆距离模型 (考虑里程计漂移)

Layer 2: ISC 描述子匹配 (performScanContextLoopClosure)
  └── detectLoopClosureID → dist < SC_DIST_THRES

Layer 3: ICP Fitness 检查
  └── registration->getFitnessScore() < historyKeyframeFitnessScore (6)

Layer 4: Odometry Check (单边一致性)
  ├── T_err = T_lc · T_odom
  └── |平移|/num_between < 0.1m/frame ∧ |旋转|/num_between < 0.05rad/frame

Layer 5: Pairwise Consistency (多边一致性)
  ├── T_err = T_lc_ij · T_odom_li · T_lc_kl · T_odom_jk
  └── |平移| < 0.1m ∧ |旋转| < 0.05rad
```

## g2o 位姿图后端

`raw/codes/4DRadarSLAM/src/radar_graph_slam/graph_slam.cpp:L58-L100`：

- 基于 g2o 的 `SparseOptimizer`，使用 pcg/cholmod/csparse 求解器
- 自定义边类型：`EdgeSE3Plane`、`EdgeSE3PriorXY`、`EdgeSE3PriorXYZ`、`EdgeSE3PriorZ`、`EdgeSE3SE3`、`EdgeSE3PriorVec`、`EdgeSE3PriorQuat` 等
- 里程计边不使用鲁棒核，回环边使用 Huber 核（`kernel_size = 1.0`）
- GNSS 因子：`EdgeSE3GtUTM` 支持 UTM 坐标系的先验位姿约束

`raw/codes/4DRadarSLAM/src/radar_graph_slam/information_matrix_calculator.cpp` — 信息矩阵计算器，基于配准点云的密度和分布计算回环边的信息矩阵。

## 配准方法

`raw/codes/4DRadarSLAM/src/radar_graph_slam/registrations.cpp:L23-L140` — `select_registration_method`：

支持 ICP、GICP、GICP_OMP、NDT_OMP、FAST_GICP、FAST_VGICP、FAST_VGICP_CUDA、FAST_APDGICP 八种配准方法。FAST_APDGICP 专为雷达适配：通过 `distVar`(0.86)、`azimuthVar`(0.5)、`elevationVar`(1.0) 参数建模雷达各向异性噪声，其中俯仰方差最大以反映 4D 雷达 z 方向精度弱的特点。

## 工程挑战

| 挑战 | 表现 | 应对 |
|------|------|------|
| 点云稀疏 | 单帧仅数百点，多径/杂波 | 回环用 ISC + 多层几何验证 |
| z 轴噪声大 | 俯仰角精度 ~1° | ISC 用 SNR 替代高度；椭圆距离模型忽略 z |
| 动态物体干扰 | 车辆/行人产生虚假 Dopppler | RANSAC 自速度估计；动态点剔除 |
| 描述子混淆 | 稀疏导致 ISC 相似度高 | 五层验证链严格过滤 |
| 里程计漂移 | 稀疏点云配准退化 | 椭圆搜索范围随距离自适应扩大 |

## Agent 实现提示

### 适用场景

当需要使用 4D 毫米波雷达（如 TI AWR2243、Arbe Phoenix）构建 SLAM 系统时，采用此工程范式。适用于 GNSS 拒止环境（隧道、地下停车场）、恶劣天气（雨雾雪）下 LiDAR/相机退化场景。不适合需要稠密建图或高精度几何重建的任务。

### 输入输出契约

- **输入**：
  - 雷达点云：`sensor_msgs::PointCloud`（含 x,y,z,doppler,intensity 通道）
  - IMU 数据：`sensor_msgs::Imu`（角速度 + 加速度）
  - 可选：气压计高度、GNSS 位姿
  - 外参：`Radar_to_livox`、`T_imu_lidar`
  - 配置参数：预处理阈值、ISC 维度、回环验证阈值、配准方法
- **输出**：
  - 优化后位姿图（g2o）
  - 关键帧位姿序列
  - 回环边约束集合
  - Doppler 自速度估计（含协方差）
- **坐标系**：雷达帧→LiDAR 帧→IMU 帧→世界帧，时间戳为 Unix 微秒

### 实现骨架（伪代码）

```pseudo
function processRadarPointCloud(raw_cloud, imu_queue):
    // 1. 坐标变换 (Radar -> LiDAR frame)
    cloud_lidar = transform(raw_cloud, T_radar_lidar)

    // 2. Doppler 自速度估计
    valid_targets = filterTargets(cloud_lidar, config)  // distance, SNR, FOV
    if medianAbsDoppler(valid_targets) < zero_vel_thresh:
        v_r = [0, 0, 0]
    else:
        v_r, sigma_v = solve3DFullRansac(valid_targets)  // H×v_r = y

    // 3. 动态点剔除
    if enable_dynamic_removal:
        cloud = selectInliers(cloud_lidar, valid_targets, inlier_indices)
    else:
        cloud = cloud_lidar

    // 4. Deskew (IMU 角速度)
    for each point in cloud:
        delta_t = scan_period * point_index / cloud_size
        point = rotate(imu_angvel, delta_t, point)

    // 5. 滤波
    cloud = distanceFilter(cloud, near, far, z_low, z_high)
    cloud = voxelDownsample(cloud, resolution=0.1)
    cloud = statisticalOutlierRemoval(cloud, meanK=20, stddev=1.0)

    return cloud, v_r, sigma_v

function detectLoopClosure(cur_keyframe, history_keyframes, sc_manager):
    // 1. 预筛选
    candidates = []
    for each kf in history_keyframes:
        if kf.accum_distance < accum_dist_thresh: continue
        if abs(baro_diff) > max_baro_diff: continue
        if abs(yaw_diff) > max_yaw_diff: continue
        if !ellipseCheck(cur_kf, kf, odom_drift): continue
        candidates.add(kf)

    // 2. ISC 匹配
    loop_id, yaw_diff = sc_manager.detectLoopClosureID(candidates, cur_keyframe)

    // 3. ICP 配准
    T_lc = icpAlign(cur_keyframe.cloud, history_keyframes[loop_id].cloud)
    if fitness_score > 6: return nullptr

    // 4. Odometry check (单边)
    T_err = T_lc * T_odom
    if |T_err.trans| / n_frames > 0.1 or |T_err.rot| / n_frames > 0.05:
        return nullptr

    // 5. Pairwise check (多边)
    for each prev_loop in loop_history:
        T_err = T_lc * T_odom1 * T_prev_lc * T_odom2
        if |T_err.trans| > 0.1 or |T_err.rot| > 0.05:
            return nullptr

    return Loop(cur_keyframe, history_keyframes[loop_id], T_lc)
```

### 关键源码片段

**Doppler 最小二乘速度估计**

`raw/codes/4DRadarSLAM/src/radar_ego_velocity_estimator.cpp:L252-L281`

```cpp
bool RadarEgoVelocityEstimator::solve3DFull(const Matrix& radar_data,
                                            Vector3& v_r, Vector3& sigma_v_r,
                                            bool estimate_sigma) {
  Matrix H(radar_data.rows(), 3);
  H.col(0) = radar_data.col(0);  // nx
  H.col(1) = radar_data.col(1);  // ny
  H.col(2) = radar_data.col(2);  // nz
  const Matrix HTH = H.transpose() * H;
  const Vector y = radar_data.col(3);  // Doppler

  v_r = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);

  if (estimate_sigma) {
    const Vector e = H * v_r - y;
    const Matrix C = (e.transpose() * e).x() * HTH.inverse() / (H.rows() - 3);
    sigma_v_r = Vector3(C(0,0), C(1,1), C(2,2));
    sigma_v_r = sigma_v_r.array().sqrt();
    sigma_v_r += Vector3(config_.sigma_offset_radar_x, ...);
  }
  return true;
}
```

**ISC 描述子构建**

`raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp:L162-L214`

```cpp
MatrixXd SCManager::makeScancontext(pcl::PointCloud<SCPointType> &_scan_down) {
  const int NO_POINT = -1000;
  MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

  for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++) {
    pt = _scan_down.points[pt_idx];
    azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
    azim_angle = (atan2f(pt.x, pt.y) - M_PI_2) * 180/M_PI;

    if (abs(azim_angle) > PC_AZIMUTH_ANGLE_MAX) continue;
    if (azim_range > PC_MAX_RADIUS) continue;

    ring_idx = max(min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
    sctor_idx = max(min(PC_NUM_SECTOR, int(ceil(((azim_angle - PC_AZIMUTH_ANGLE_MIN)
        / (PC_AZIMUTH_ANGLE_MAX - PC_AZIMUTH_ANGLE_MIN)) * PC_NUM_SECTOR))), 1);

    if (desc(ring_idx-1, sctor_idx-1) < pt.intensity)
      desc(ring_idx-1, sctor_idx-1) = pt.intensity;  // max SNR per bin
  }
  // reset no points to zero
  desc = (desc.array() == NO_POINT).select(0, desc);
  return desc;
}
```

### 实现注意事项

1. **Doppler 符号约定**：`raw/codes/4DRadarSLAM/src/radar_ego_velocity_estimator.cpp:L47` 中 `doppler = -item[idx.doppler]`，雷达原始 Doppler 是径向远离为正，而速度估计假设单位方向向量指向目标，因此取反使方程一致。
2. **ISC 极坐标方位角偏移**：`azim_angle = (atan2f(pt.x, pt.y) - M_PI_2) * 180/M_PI`，与标准 ScanContext 不同（原版使用 `xy2theta`），此偏移使得 0° 方向指向雷达前方。
3. **椭圆距离模型**：里程计漂移随距离增加而放大，因此搜索椭圆的半轴 $\text{rad} = 10 + \text{drift} \times \text{distance} \times \text{scale}$ 自适应扩大，防止真回环因漂移被排除。
4. **鲁棒核策略**：回环边使用 Huber 核（size=1.0），里程计边不使用核——因为里程计边本身已通过配准验证，异常较少；而回环边存在误匹配风险。
5. **APDGICP 各向异性协方差**：`distVar=0.86`、`azimuthVar=0.5`、`elevationVar=1.0` 反映了雷达在三个方向上的测量噪声差异，直接影响 GICP 中每个点的协方差矩阵，从而影响配准残差的权重分配。
6. **外参链**：`Radar_to_livox = RGB_to_livox * Thermal_to_RGB * Radar_to_Thermal * Change_Radarframe`，其中 `Change_Radarframe` 将雷达坐标系旋转 90° 以对齐 LiDAR 坐标约定（雷达 x 指向前方，LiDAR z 指向上方）。

### 源码检索锚点

- 预处理：`raw/codes/4DRadarSLAM/apps/preprocessing_nodelet.cpp`
- 自速度估计：`raw/codes/4DRadarSLAM/src/radar_ego_velocity_estimator.cpp`
- ISC 描述子：`raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp`
- 回环检测：`raw/codes/4DRadarSLAM/src/radar_graph_slam/loop_detector.cpp`
- 配准方法选择：`raw/codes/4DRadarSLAM/src/radar_graph_slam/registrations.cpp`
- 位姿图：`raw/codes/4DRadarSLAM/src/radar_graph_slam/graph_slam.cpp`
- 关键帧管理：`raw/codes/4DRadarSLAM/src/radar_graph_slam/keyframe.cpp`
- 信息矩阵计算：`raw/codes/4DRadarSLAM/src/radar_graph_slam/information_matrix_calculator.cpp`
- 点云地图生成：`raw/codes/4DRadarSLAM/src/radar_graph_slam/map_cloud_generator.cpp`

## 相关页面

- [[方法-Intensity Scan Context]]
- [[方法-五重回环几何验证]]
- [[方法-回环验证方法族]]
- [[方法-APDGICP 自适应概率分布 GICP]]
- [[方法-ICP配准方法]]
- [[特殊传感器数据管线]]
