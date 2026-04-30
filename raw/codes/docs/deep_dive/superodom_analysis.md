# SuperOdom 深度源码分析报告

> 分析日期：2026-04-28
> 代码版本：ROS2 Humble 分支，Slim 版本
> 源码路径：`/home/lin/Projects/lin_ws/slam_ws/SuperOdom/super_odometry/`

---

## 1. 数据接收与预处理

### 1.1 传感器数据入口与 ROS 话题

SuperOdom 是三个独立 ROS 2 节点的松散耦合管道：

| 节点 | 源文件 | 订阅 TOPIC | 发布 TOPIC |
|------|--------|-----------|-----------|
| `feature_extraction_node` | `src/featureExtraction_node.cpp:1` | 激光点云、IMU、VIO里程计 | 特征点云、去畸变点云 |
| `imu_preintegration_node` | `src/imuPreintegration_node.cpp:1` | IMU原始数据、激光里程计 | 融合状态估计 |
| `laser_mapping_node` | `src/laserMapping_node.cpp:1` | 激光特征信息 | 最终激光里程计、地图 |

传感器订阅实现在 `featureExtraction::initInterface()` (`src/FeatureExtraction/featureExtraction.cpp:68-110`)：

- **激光点云**：根据传感器类型 (Velodyne/Ouster) 订阅 `sensor_msgs::PointCloud2`，Livox 订阅 `livox_ros_driver2::CustomMsg`。QoS 设为 `BEST_EFFORT + keep_last(2/10)`。
- **IMU**：订阅 `sensor_msgs::Imu`，`BEST_EFFORT` 模式。
- **VIO 里程计**：订阅 `nav_msgs::Odometry`，用于视觉惯性里程计的辅助去畸变。

### 1.2 点云预处理与去畸变

去畸变（deskewing）在 `featureExtraction::undistortionAndFeatureExtraction()` (`src/FeatureExtraction/featureExtraction.cpp:440-501`) 中实现：

1. **时间同步**：`synchronize_measurements<T>()` (`:172-217`) 使用 `MapRingBuffer` 确保 IMU/VIO 数据覆盖激光帧的时间窗口 (`meas_start_time < lidar_start_time && meas_end_time >= lidar_end_time`)。

2. **去畸变执行**：`removePointDistortion<T>()` (`:222-314`) 对每个激光点 (含逐点时间戳 `point.time`) 进行运动补偿：
   - 查找该点时刻前后最近的 IMU/VIO 位姿
   - 使用 **slerp** 插值得到该时刻的旋转、线性插值得到位移
   - 将点从当前 LiDAR 位姿变换到扫描起始位姿（IMU 模式下经过 `T_i_l` / `T_l_i` 外参变换）
   - 关键代码：`transformPoint()` (`:353-368`) 计算 `T_original_current = T_w_original.inverse() * T_w_current`

3. **LiDAR-Only 模式**：如果无 IMU 数据 (`imuBuf.empty()`)，跳过去畸变，直接用 `default_quaternion` 处理 (`:483-496`)。

4. **Buffer 管理**：`manageLidarBuffer()` (`:825-842`) 限制 LiDAR buffer 大小不超过 50 帧。

### 1.3 点云数据结构

自定义点云类型定义在 `include/super_odometry/sensor_data/pointcloud/point_os.h` 中，包含 `PointcloudXYZITR`（XYZ + intensity + time + ring），Ouster 到传感器坐标系的转换通过 `utils::transformOusterPoints()` 完成，使用 `T_ouster_sensor` 外参 (`featureExtraction.cpp:743`)。

---

## 2. 特征提取

### 2.1 特征类型与提取方法

特征提取在 `featureExtraction::extractFeatures()` (`src/FeatureExtraction/featureExtraction.cpp:422-437`) 中完成。**与 LOAM 不同，SuperOdom 仅使用均勻采样后的平面特征**：

```cpp
uniformFeatureExtraction(lidar_msg, plannerPoints, config_.filter_point_size, config_.min_range);
```

等间隔均勻采样实现 (`:504-525`)：
- 每隔 `skip_num` 个点采样一个
- 检查相邻点是否有实际位移（排除 NaN 重複点）
- 检查点到原点距离 > `block_range`（盲区过滤）
- 所有采样点标记为 `PlannerPoints`（`edgePoints` 和 `depthPoints` 在此版本为空）

### 2.2 特征数量与选取策略

- `skip_num` 由配置参数 `filter_point_size` 确定（默认 3）
- `min_range` 默认 0.2m（近距离盲区过滤）
- `max_range` 默认 130m
- **无角点/平面点区分**：所有特征点统一作为 surf 点进入后端优化，由 PCA 在线判断点属于线特征还是面特征
- 不进行基于曲率的排序和选点（与 LOAM 的核心区别）

### 2.3 特征发布

`publishTopic()` (`:389-420`) 发布 `super_odometry_msgs::LaserFeature` 消息，包含：
- `cloud_nodistortion`：去畸变后完整点云
- `cloud_corner`：角点（此版本为空）
- `cloud_surface`：均勻采样后的平面特征点
- `initial_quaternion/initial_pose`：帧起始位姿

---

## 3. 位姿计算

### 3.1 位姿估计方法

位姿优化流程在 `LidarSLAM::Localization()` (`src/LidarProcess/LidarSlam.cpp:30-51`) 中：

```cpp
// ICP 迭代最多 LocalizationICPMaxIter 次（默认4次）
for (size_t icp_iter = 0; icp_iter < LocalizationICPMaxIter; ++icp_iter) {
    extractFeaturesConstraints(feature_corres, edge_num, planner_num);
    auto problem = setupOptimizationProblem(feature_corres, predictodom, T_w_initial_guess);
    auto summary = solveOptimizationProblem(problem);
}
```

**核心算法：基于 Ceres Solver 的非线性最小二乘优化，本质是 scan-to-map 的解析 ICP。**

### 3.2 优化细节

#### 3.2.1 特征匹配与约束构建

- **线特征（Edge）**：`ComputeLineDistanceParameters()` (`LidarSlam.cpp:408-441`)
  - 在局部地图中搜索 `LocalizationLineDistanceNbrNeighbors=10` 个最近邻
  - 使用 RANSAC 风格选点：找到最多 inlier 的直线模型（`nearestKSearchSpecificEdgePoint`, `LocalMap.h:377-474`）
  - PCA 验证：最大特征值 > 4 倍次大特征值
  - 构建投影矩阵 `P = I - v*v^T`（v 为线方向），计算点到线距离

- **面特征（Plane）**：`ComputePlaneDistanceParameters()` (`LidarSlam.cpp:520-578`)
  - 5 个最近邻拟合平面
  - QR 分解求解平面法向量
  - 检查平面质量（点到平面距离 < planeRes/2）
  - **特征可观测性分析**：`FeatureObservabilityAnalysis()` 分析 6-DoF 退化方向

#### 3.2.2 Ceres 问题构建

`setupOptimizationProblem()` (`LidarSlam.cpp:219-234`)：

- 7 参数块 (xyz + quaternion)，使用 `PoseLocalParameterization` (SE(3) 上的局部参数化)
- 线特征残差：`EdgeAnalyticCostFunction` (`src/LaserMapping/lidarOptimization.cpp:7-47`)
  - 残差 = `|(lp - last_point_a) × (lp - last_point_b)| / |last_point_a - last_point_b|`
  - 解析雅可比，3 维残差
- 面特征残差：`SurfNormAnalyticCostFunction` (`lidarOptimization.cpp:50-80`)
  - 残差 = `plane_norm · point_w + negative_OA_dot_norm`
  - 解析雅可比，1 维残差
- 损失函数：`TukeyLoss` + `ScaledLoss`（基于匹配质量因子的自适应加权）
- 退化时：添加 `SE3AbsolutatePoseFactor`（先验约束）(`LidarSlam.cpp:291-304`)

#### 3.2.3 Ceres 求解器配置

- `linear_solver_type = DENSE_QR`
- `max_num_iterations = 4`
- 协方差估计：`Covariance::Options` 使用 `DENSE_SVD` 算法 (`LidarSlam.cpp:861-895`)

### 3.3 IMU 积分

IMU 预积分在 `imuPreintegration` 类中实现 (`src/ImuPreintegration/imuPreintegration.cpp`)：

- **库**：GTSAM 的 `CombinedImuFactor` 和 `PreintegratedImuMeasurements`
- **优化器**：GTSAM `ISAM2` (`relinearizeThreshold=0.1, relinearizeSkip=1`)
- **状态**：Pose3(X), Velocity(V), Bias(B)，每帧添加 lidar pose 先验因子 + IMU 因子 + bias between 因子
- **外参链**：LiDAR→IMU 或 LiDAR→Camera→IMU（支持两种外参路径）
- **失败检测**：`failureDetection()` 检查速度 > 30m/s，加速度偏置 > 2.0，角速度偏置 > 1.0
- **Livox 特殊处理**：重力对齐矩阵 `imu_laser_R_Gravity`，加速度归一化

### 3.4 位姿预测策略

`laserMapping::determinePredictionSource()` (`src/LaserMapping/laserMapping.cpp:384-412`) 实现了多策略混合的初始位姿预测：

| 优先级 | 预测源 | 条件 |
|--------|--------|------|
| 1 | VIO_ODOM | 系统退化时可用 |
| 2 | NEURAL_IMU_ODOM | 退化时 VIO 不可用 |
| 3 | LIO_ODOM | 非退化时可用 |
| 4 | IMU_ORIENTATION | IMU 可用 |
| 5 | CONSTANT_VELOCITY | 兜底 |

---

## 4. 初始化

### 4.1 初始位姿

`laserMapping::initializeFirstFrame()` (`src/LaserMapping/laserMapping.cpp:281-314`)：

- 如果有 IMU：提取 roll/pitch，将 yaw 置零，通过外参变换得到 LiDAR 初始旋转
- 无 IMU：使用单位四元数
- 初始化位置为原点（(0,0,0)）
- **定位模式**：从配置文件读取 `init_x/y/z/roll/pitch/yaw` 覆盖初始位姿，允许加载先验地图

### 4.2 IMU 初始化

`featureExtraction::imuInitialization()` (`src/FeatureExtraction/featureExtraction.cpp:603-618`)：

- 收集至少 1 秒的 IMU 数据
- 调用 `Imu::imuInit()` 估计重力方向和 IMU 偏置
- 计算 `imu_laser_R_Gravity` 重力对齐矩阵（对 Livox 特别重要）

### 4.3 地图初始化

`LidarSLAM::initializeMapping()` (`src/LidarProcess/LidarSlam.cpp:89-100`)：

- 设置局部地图原点为当前 LiDAR 位置
- 将第一帧的特征点变换到世界坐标系并加入局部地图
- **定位模式**：`laserMapping::initializationParam()` 从 PCD/PLY 文件加载先验全局地图 (`laserMapping.cpp:163-176`)

---

## 5. 多传感器融合

### 5.1 传感器列表

- **LiDAR**：Velodyne (VLP-16/32/64)、Ouster (OS1-128)、Livox (Mid-360)
- **IMU**：板载 IMU（可選，通过空 `imu_topic` 禁用）
- **VIO**（可選）：视觉惯性里程计

### 5.2 融合方式：松散耦合 (Loose Coupling)

SuperOdom 采用 **IMU-centric 的松散耦合架构**：

1. **LiDAR Odometry → IMU**：LiDAR 里程计结果作为 `PriorFactor<Pose3>` 输入 IMU 预积分图优化。激光位姿纠正 IMU 的偏置估计。

2. **IMU → LiDAR Odometry**：IMU 预积分结果作为 LiDAR ICP 的初始位姿猜测（`T_w_initial_guess`）。IMU 提供的 roll/pitch 可直接使用（`use_imu_roll_pitch` 配置）。

3. **退化处理**：当 LiDAR 特征可观测性不足时，切换为 VIO 或 IMU 预测。

关键数据流：`激光特征消息` → `laser_mapping_node` (ICP 优化) → `laser_odometry` → `imu_preintegration_node` (ISAM2) → `state_estimation`

### 5.3 融合架构图

```
[LiDAR] → [featureExtraction] → [laserMapping (Ceres ICP)]
                                       ↓ 位姿先验
[IMU]   → [imuPreintegration (GTSAM ISAM2)] → 高频状态估计 (200Hz+)
                                        ↓ 位姿预测
[VIO]   ──────────────→ 退化时备用
```

---

## 6. 后端优化

### 6.1 回环检测

**此 Slim 版本不包含回环检测模块。** 全版本 SuperOdometry 的完整 pipeline 包含回环检测，但此仓库仅发布 LiDAR Odometry + IMU Odometry 组件。

### 6.2 全局优化

- **IMU 图优化**：GTSAM `ISAM2`，状态变量 X(pose3), V(velocity3), B(bias6)
- **因子**：`PriorFactor<Pose3>` (激光位姿) + `ImuFactor` + `BetweenFactor<ConstantBias>` (bias 随机游走)
- **关键帧策略**：每 100 帧重置一次图以控制计算量（`reset_graph()`, `imuPreintegration.cpp:162-205`）
- **LiDAR 端**：纯前端扫描匹配，无全局 BA/PGO。Ceres 协方差估计用于 `RegistrationError` 评估和退化检测。

### 6.3 局部地图管理

局部地图使用 **三级网格结构**：

```
LocalMap (21×21×11 栅格, 每格 50m)
├── MapBlock[] (4851 个)
│   ├── edge_pc (线特征点云 + Octree KD-tree)
│   └── surf_pc (面特征点云 + Octree KD-tree)
└── shiftMap() 滑动窗口机制
```

- `LocalMap.h` (`include/super_odometry/LidarProcess/LocalMap.h`) 和 `LocalMap.h` (`include/super_odometry/LocalMap.h`) 两个版本，新版本使用自研 `nanoflann::Octree`，老版本使用 `pcl::KdTreeFLANN`
- `addEdgePointCloud()` / `addSurfPointCloud()` 对每个被更新的 block 进行体素滤波 + KD-tree 重建，使用 **TBB parallel_for** 并行化
- `shiftMap()` 实现环形缓冲区式的滑动窗口，当 LiDAR 移动超过阈值时移动栅格数据
- 点云搜索范围：当前块周围的 5×5×3 邻域（XY 范围 ±2，Z 范围 ±1）

### 6.4 退化检测与不确定度

`LidarSLAM::EstimateLidarUncertainty()` (`LidarSlam.cpp:921-992`)：

- 基于特征可观测性直方图统计 `PlaneFeatureHistogramObs[9]`
- 6-DoF 各自的退化程度：`uncertainty_x/y/z/roll/pitch/yaw`
- 通过 ROS topic `uncertainty_*` 实时发布
- `EstimateRegistrationError()` 使用 Ceres `Covariance` 计算完整的 6×6 协方差矩阵和逆条件数

---

## 7. 优缺点分析

### 7.1 算法优缺点

**优点：**
- 解析雅可比（非自动微分），计算效率高
- 在线 PCA 特征分类（区分线/面），无需依赖预定义特征类型
- 自适应体素大小（根据场景尺度调整分辨率）
- 6-DoF 退化检测完善（SuperLoc 的 alignment risk prediction）
- 多源位姿预测策略，鲁棒性强

**缺点：**
- 无回环检测，长距离漂移无法消除
- 松散耦合，非最优融合（相比紧耦合如 LIO-SAM）
- 无视觉特征，退化场景（长隧道）可能失效
- 特征提取仅用均勻采样，信息利用率较低

### 7.2 工程优缺点

**优点：**
- ROS2 Humble 原生支持，现代化架构
- 模块化节点设计，支持 LiDAR-Only 和 LiDAR-IMU 双模式
- 支持多种 LiDAR（Velodyne/Ouster/Livox）即插即用
- Docker 部署支持完善
- 代码规范，C++17，注释清晰

**缺点：**
- ISAM2 增量优化在 IMU 端可能成为性能瓶颈
- 滑动窗口地图无边界清理机制（地图无限增长）
- 配置文件分散（多级 YAML 嵌套），调试不便

### 7.3 适用场景

**适合：**
- 室内外结构化环境（有丰富平面特征）
- 多旋翼/UAV 快速机动场景（IMU-centric 设计）
- 需要退化感知的定位任务（SuperLoc 特色）
- 多传感器平台（LiDAR + IMU + 相机/VIO）

**不适合：**
- 大规模室外长距离建图（无回环）
- 极度退化环境（空旷场地、浓雾）
- 需要紧耦合最优估计的应用

---

## 8. 对 phad_fusion 参考

### 8.1 值得借鉴的设计

1. **多源位姿预测策略** (`laserMapping.cpp:384-412`)：
   - 实现灵活的多级 fallback 机制（IMU→VIO→恒速），phad_fusion 可采用类似的多源融合预测优先级

2. **解析雅可比 + Ceres 优化** (`lidarOptimization.cpp:7-80`)：
   - 手写的 `EdgeAnalyticCostFunction` 和 `SurfNormAnalyticCostFunction` 比自动微分快 2-3 倍
   - 配合 `TukeyLoss` + `ScaledLoss` 实现自适应加权

3. **特征可观测性分析** (`LidarSlam.cpp:580-685`)：
   - 6-DoF 退化检测（尤其是 roll/pitch/yaw 六个方向的分别评估）
   - 退化时的先验约束注入 (`SE3AbsolutatePoseFactor`)

4. **自适应体素大小** (`laserMapping.cpp:600-651`)：
   - 根据场景特征密度动态调整体素分辨率，在开放/封闭场景间自动切换

5. **Octree-based KD-tree** (`include/super_odometry/flann/octree.h`)：
   - 自研的 nanoflann Octree 比 pcl::KdTreeFLANN 在大规模点云下更高效

### 8.2 应避免的设计

1. **松散耦合的非最优性**：LiDAR 和 IMU 各自独立优化（而非联合状态估计），信息丢失。phad_fusion 应考虑紧耦合。

2. **无回环检测**：此版本适合作为前端的参考，但完整系统必须加入回环模块。

3. **地图无限增长**：`LocalMap` 的 `shiftMap()` 只移动数据但不删除远距离 block，需增加地图裁剪机制。

4. **IMU 初始化对 Livox 的依赖**：重力对齐逻辑硬编码了 Livox 特殊处理，通用性不足。


## 9. 数据管线

### 9.1 传感器输入总览

| 传感器 | 硬件规格 | ROS2 Topic | 消息类型 | 频率 | 处理节点 |
|--------|---------|-----------|---------|------|---------|
| **LiDAR** | Velodyne/Ouster/Livox | `/points_raw` | `PointCloud2` / `CustomMsg` | 10-20 Hz | `feature_extraction_node` |
| **IMU** | 6轴 MEMS | `/imu_raw` | `sensor_msgs::Imu` | 200 Hz | `feature_extraction_node` + `imu_preintegration_node` |
| **VIO** | 外源视觉里程计 | `/vio_odom` | `nav_msgs::Odometry` | 20-30 Hz | `feature_extraction_node` (退化解耦时) |

### 9.2 LiDAR 数据管线

```
LiDAR 硬件 (Velodyne/Ouster/Livox)
  ↓ (featureExtraction.cpp:68-76)
ROS2 订阅: BEST_EFFORT QoS, keep_last(2)
  ↓ 传感器类型适配 (featureExtraction.cpp:743):
    Ouster: transformOusterPoints() (T_ouster_sensor 外参变换)
    Livox: CustomMsg → PointcloudXYZITR 自定义格式 (x,y,z, intensity, time, ring)
  ↓ (featureExtraction.cpp:440-501, undistortionAndFeatureExtraction)
[去畸变] removePointDistortion():
  ↓ synchronize_measurements() (:172-217):
    MapRingBuffer: 确保 IMU/VIO 数据时间覆盖激光帧窗口
    条件: meas_start < lidar_start && meas_end >= lidar_end
  ↓ 逐点去畸变 (:222-314):
    point.time (逐点时间戳) → 查找最近 IMU/VIO 位姿
    slerp 插值旋转: q(t) = slerp(q_prev, q_curr, (t - t_prev)/(t_curr - t_prev))
    线性插值位移: p(t) = lerp(p_prev, p_curr, ...)
    T_w_original^(-1) x T_w_current -> 点变换到帧起始位姿
    transformPoint() (:353-368) -> T_i_l / T_l_i 外参变换
  ↓ LiDAR-Only fallback: 无 IMU -> 跳过去畸变 (:483-496)
[特征提取] uniformFeatureExtraction() (:504-525):
  ↓ skip_num = filter_point_size (默认 3), min_range=0.2m, max_range=130m
  ↓ 等间隔采样: 每隔 skip_num 点取一个 -> PlannerPoints (全为面特征)
  ↓ 无角点提取: edgePoints/depthPoints 为空 -> 后端 PCA 在线区分线/面
  ↓ manageLidarBuffer() (:825-842): 限制 buffer 50 帧 -> 删除最旧帧
  ↓ 发布 LaserFeature 消息:
    cloud_nodistortion (去畸变点云), cloud_corner (空),
    cloud_surface (均匀采样面特征), initial_quaternion/initial_pose
```

### 9.3 IMU 预积分管线 (GTSAM)

```
IMU 硬件 -> ROS2 Imu 消息 (200 Hz)
  ↓ (imuPreintegration.cpp:42-45) 订阅 + BEST_EFFORT QoS
imuHandler() (:150-340):
  ↓ imuIntegratorOpt_->integrateMeasurement(
      accel (body frame, m/s^2), gyro (body frame, rad/s), dt)
  ↓ GTSAM PreintegratedImuMeasurements (CombinedImuFactor):
    p_{k+1|k} = p_k + v_k*dt + 0.5*(R_k*(a_k - ba) + g)*dt^2
    v_{k+1|k} = v_k + (R_k*(a_k - ba) + g)*dt
    R_{k+1|k} = R_k * exp((w_k - bg)*dt)
  ↓ 状态变量: X(Pose3), V(Velocity3), B(bias6)
  ↓ ISAM2 优化器 (relinearizeThreshold=0.1, relinearizeSkip=1):
    每帧: addPriorFactor(Pose3, from laser odom) + ImuFactor + BetweenFactor(bias)
  ↓ 外参链: LiDAR->IMU 或 LiDAR->Camera->IMU
  ↓ 失败检测 (imuPreintegration.cpp:620-640):
    |v| > 30 m/s -> reset; ba_norm > 2.0 -> reset; bg_norm > 1.0 -> reset
  ↓ 图重置: 每 100 帧 reset_graph() (重置 ISAM2, 保留最新状态)
```

### 9.4 多源位姿预测策略

```
(laserMapping.cpp:384-412, determinePredictionSource)

优先级  预测源          条件                    机制
  1    VIO_ODOM        系统退化 && VIO 可用     T_pred = T_VIO
  2    NEURAL_IMU_ODOM 退化 && VIO 不可用      T_pred = T_NN_IMU
  3    LIO_ODOM        非退化                   T_pred = prev_LIO * delta
  4    IMU_ORIENTATION IMU 可用                  R_pred = R_IMU, t_pred = const_vel
  5    CONSTANT_VELOCITY 兜底                    T_pred = prev * prev_delta
```

### 9.5 LiDAR ICP 优化管线 (Ceres)

```
激光特征消息 -> laser_mapping_node
  ↓ (LaserSlam.cpp:30-51, Localization)
确定预测位姿 -> T_w_initial_guess
  ↓ ICP 迭代 (最多 4 次):
for iter in range(LocalizationICPMaxIter):
  [特征匹配] extractFeaturesConstraints() (LidarSlam.cpp:330-400):
    线特征: KD树搜索 10 最近邻 -> RANSAC 选直线 -> PCA 验证 (lambda_max > 4*lambda_2nd)
            投影矩阵: P = I - v*v^T (v 线方向)
    面特征: KD树搜索 5 最近邻 -> QR 分解求法向量 -> 平面质量检查
            特征可观测性: FeatureObservabilityAnalysis() 6-DoF 退化方向
  [问题构建] setupOptimizationProblem() (LidarSlam.cpp:219-234):
    线残差 EdgeAnalyticCostFunction (3维):
      e = |(lp - p_a) x (lp - p_b)| / |p_a - p_b|    (点到线距离)
    面残差 SurfNormAnalyticCostFunction (1维):
      e = plane_norm * p_w + (-OA * n)
    7参数块 [xyz + quaternion], PoseLocalParameterization (SE(3) 流形)
    损失函数: TukeyLoss + ScaledLoss (基于匹配质量自适应加权)
    退化时: 添加 SE3AbsolutatePoseFactor (先验约束)
  [求解] Ceres LM, DENSE_QR, 4 次迭代
  ↓ 协方差估计 (LidarSlam.cpp:861-895):
    Covariance::DENSE_SVD -> 6x6 协方差 -> inverse condition number
```

### 9.6 跨传感器协同

| 机制 | 实现 | 参数 |
|------|------|------|
| **时间同步** | MapRingBuffer 时间窗口对齐 | `meas_start < lidar_start && meas_end >= lidar_end` |
| **IMU-LiDAR 外参** | T_i_l / T_l_i (配置读取) | 静态外参 |
| **LiDAR 降级检测** | 6-DoF 可观测性直方图 | `PlaneFeatureHistogramObs[9]` |
| **降级 Fallback** | LiDAR->VIO->NN_IMU->恒速 | `determinePredictionSource()` |
| **IMU 失败恢复** | 速度/偏置过大 -> 重置图 | `failureDetection()` |
| **图重置** | 每 100 帧重新初始化 ISAM2 | `reset_graph()` |
| **GNSS 集成** | 不直接使用 (该 slim 版本) | -- |

### 9.7 数据流架构图

```
[LiDAR]     [IMU]      [VIO]
   ↓          ↓  ↓          ↓
feature_extraction     imu_preintegration
   ↓ 去畸变+采样            ↓ GTSAM预积分+ISAM2
   ↓                       ↑ prior(Pose3)
laser_mapping               |
   ↓ Ceres ICP              |
   ↓ laser_odometry --------+
   ↓ state_estimation (200Hz+)
```