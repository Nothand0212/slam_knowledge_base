# LVI-SAM 深度源码分析

> 论文: LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping
> 开发者: Tixiao Shan, et al. (MIT)
> GitHub: 1822 stars


## 1. 数据接收与预处理 (Velodyne/Ouster LiDAR + 全局快门相机 + IMU)

### 1.1 系统架构：多ROS节点松耦合

LVI-SAM由**5个独立ROS节点**组成，区别于FAST-LIVO2和R3LIVE的单进程架构：

```
imageProjection     →  LiDAR投影+去畸变
featureExtraction   →  LiDAR特征提取（边缘+平面）
imuPreintegration   →  IMU预积分(GTSAM)
mapOptimization     →  因子图优化 (LIS核心)
visual_odometry     →  视觉里程计 (VIS核心, 含loop)
```

### 1.2 LiDAR预处理 (`imageProjection.cpp`)

`ImageProjection::cloudHandler()` 接收原始点云：
1. **运动畸变校正**: 使用IMU数据插值去畸变
2. **投影**: 将3D点投影到2D range image（按ring和水平角度）
3. 时间同步: 使用点云的 `header.stamp`，所有传感器以LiDAR时间为准

### 1.3 特征提取 (`featureExtraction.cpp`)

基于LeGO-LOAM/LIO-SAM的特征提取：
- **平面特征**: `cloud_surface`，基于局部曲率
- **边缘特征**: `cloud_corner`，高曲率点
- 区分地面点和非地面点（通过IMU pitch角判断）
- 输出 `lvi_sam::cloud_info` 自定义消息，包含：
  - `cloud_corner`: 边缘特征点云
  - `cloud_surface`: 平面特征点云
  - `imuRollInit, imuPitchInit, imuYawInit`: IMU初始姿态
  - `odomAvailable, odomX/Y/Z/Roll/Pitch/Yaw`: VINS里程计信息

### 1.4 IMU预积分 (`imuPreintegration.cpp:337-421`)

单独ROS节点，使用GTSAM的 `PreintegratedImuMeasurements`:
```cpp
imuIntegratorImu_->integrateMeasurement(
    gtsam::Vector3(accel.x, accel.y, accel.z),
    gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);
```
每帧预测位姿: `gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom)`

### 1.5 视觉预处理 (`feature_tracker.cpp`)

**特征提取与跟踪**:
1. 使用 `cv::goodFeaturesToTrack()` 提取Shi-Tomasi角点
2. `cv::calcOpticalFlowPyrLK()` KLT光流金字塔跟踪
3. 通过基础矩阵（`cv::findFundamentalMat()`）RANSAC剔除outliers
4. 新特征均匀分布 (`setMask()` 保证网格覆盖)

跟踪结果输出频率由 `FREQ` 参数控制（默认10Hz）。


## 2. 前端

### 2.1 LiDAR前端: Corner+Surface双特征

`featureExtraction.cpp` 提取两类特征:
- **Corner (边缘)**: 根据曲率排序，取 `edgeThreshold` 个最高曲率点
- **Surface (平面)**: 取 `surfThreshold` 个最低曲率点

在 `mapOptmization.cpp` 中使用:
- `cornerOptimization()` (`mapOptmization.cpp:966-1056`):
  - 对边缘点，在历史边缘地图中找5个最近点
  - PCA分解判定线特征: `if (matD1[0] > 3*matD1[1])` → 计算点到线距离
- `surfOptimization()` (`mapOptmization.cpp:1058-1127`):
  - 对平面点，找5个最近点
  - QR分解拟合平面 → 计算点到平面距离

### 2.2 视觉前端: Shi-Tomasi + KLT

`feature_tracker.cpp`:
- 提取 `MAX_CNT`（默认150）个Shi-Tomasi角点
- KLT金字塔跟踪（`cv::Size(21,21)` 搜索窗口）
- 外点剔除: `cv::findFundamentalMat()` + `status` 掩码

### 2.3 LiDAR-视觉协同

LVI-SAM的前端协同是**消息驱动的松耦合**:

**VIS→LIS方向** (`mapOptmization.cpp:818-848`, `updateInitialGuess()`):
```cpp
if (cloudInfo.odomAvailable && cloudInfo.odomResetId == odomResetId) {
    // 使用VINS位姿增量更新LiDAR初始猜测
    Eigen::Affine3f transIncre = lastVinsTransformation.inverse() * transBack;
    Eigen::Affine3f transFinal = transTobe * transIncre;
}
```

**LIS→VIS方向**: LiDAR点云通过 `imuPreintegration` 节点发布的 `odometry/imu` topic被VINS系统消费，用于初始化深度和尺度。


## 3. 初始化

### 3.1 LiDAR-Inertial系统(LIS)初始化

**因子图初始化** (`imuPreintegration.cpp:152-195`):
```cpp
if (!systemInitialized) {
    prevPose_ = lidarPose.compose(lidar2Imu);
    // 先验位姿因子
    graphFactors.add(PriorFactor<Pose3>(X(0), prevPose_, priorPoseNoise));
    // 先验速度因子
    graphFactors.add(PriorFactor<Vector3>(V(0), prevVel_, priorVelNoise));
    // 先验bias因子
    graphFactors.add(PriorFactor<imuBias::ConstantBias>(B(0), prevBias_, priorBiasNoise));
    optimizer.update(graphFactors, graphValues);
}
```

**地图初始化**: `ikdtree.Build(feats_down->points)` 使用首帧点云建树。

### 3.2 Visual-Inertial系统(VIS)初始化

基于VINS-Mono的初始化流程 (`estimator.cpp`, `initial/`):

1. **相对位姿估计** (`relativePose()`): 五点法求解本质矩阵
2. **初始结构** (`initialStructure()` → `initial_sfm.cpp`):
   - 三角化特征点
   - 全局SFM（PnP + 三角化迭代）
3. **视觉-惯性对齐** (`initial_aligment.cpp`, `visualInitialAlign()`):
   - 陀螺仪bias标定（旋转约束最小二乘）
   - 速度、重力、尺度初始化（线性最小二乘）

### 3.3 双系统对齐

LIS和VIS各自独立初始化:
1. LIS初始化后发布 `odometry/imu` (带bias、gravity信息)
2. VINS使用LIS提供的imu bias和gravity作为先验
3. 两系统共享重力方向（对齐到 `"odom"` 坐标系）


## 4. 逐帧估计

### 4.1 LIS: GTSAM因子图优化 (`mapOptmization.cpp:49-`)

**因子图结构**:
```
    X0  ──►  X1  ──►  X2  ──► ... ──► Xk
    │         │         │                │
    V0  ──►  V1  ──►  V2  ──► ... ──► Vk
    │         │         │                │
    B0  ──►  B1  ──►  B2  ──► ... ──► Bk
```

因子类型:
- **IMU因子**: `gtsam::ImuFactor(X(k-1), V(k-1), X(k), V(k), B(k-1), preint_imu)`
- **Bias随机游走**: `BetweenFactor<imuBias>(B(k-1), B(k), ...)`
- **先验位姿因子** (来自LiDAR odometry): `PriorFactor<Pose3>(X(k), lidarPose, correctionNoise)`

**ISAM2增量优化** (`imuPreintegration.cpp:265-266`):
```cpp
optimizer.update(graphFactors, graphValues);
optimizer.update();  // 增量式ISAM2优化
```

每100帧重置图以防止内存和计算量膨胀 (`imuPreintegration.cpp:199-226`)。

**scan-to-map优化** (`mapOptmization.cpp:1150-`):
```cpp
LMOptimization(iterCount)  // Levenberg-Marquardt求解6-DOF位姿
    → cornerOptimization() + surfOptimization()
    → 构建 H^T·H 系统
    → LM迭代求解 (退化检测 matP 矩阵)
```

### 4.2 VIS: VINS-Mono滑动窗口优化

基于 `ceres::Problem` 的滑动窗口bundle adjustment (`estimator.cpp`):

**状态** (`estimator.h:76-93`):
```cpp
Vector3d Ps[WINDOW_SIZE+1];  // 位置 (10个关键帧)
Vector3d Vs[WINDOW_SIZE+1];  // 速度
Matrix3d Rs[WINDOW_SIZE+1];  // 旋转
Vector3d Bas[WINDOW_SIZE+1]; // accel bias
Vector3d Bgs[WINDOW_SIZE+1]; // gyro bias
double td;                    // IMU-相机时间偏移
```

**因子类型**:
- **IMU预积分因子** (`imu_factor.h`): `evaluate()` 计算IMU残差
- **视觉重投影因子** (`projection_factor.cpp`): `evaluate()` 计算特征点到图像平面的重投影误差
- **边缘化先验因子** (`marginalization_factor.cpp`): Schur补保留旧帧的约束

**边缘化** (`marginalization_factor.cpp`):
- `MARGIN_OLD`: 最老帧边缘化 → `MarginalizationInfo::marginalize()`
- `MARGIN_SECOND_NEW`: 倒数第二新帧边缘化

### 4.3 LIS-VIS数据流

**VIS→LIS**: VINS位姿通过 `cloudInfo.odomX/Y/Z/Roll/Pitch/Yaw` 发送给LiDAR作为初始猜测：
```cpp
// mapOptmization.cpp:818-848
Eigen::Affine3f transIncre = lastVinsTransformation.inverse() * transBack;
Eigen::Affine3f transFinal = transTobe * transIncre;
```

如果VINS失败: 降级到纯IMU旋转积分 (`cloudInfo.imuAvailable`)。

**LIS→VIS**: LiDAR优化的bias/gravity信息嵌入 `odometry/imu` 消息中，使用 `pose.covariance[n]` 字段编码：
```cpp
odometry.pose.covariance[0] = imuPreintegrationResetId;
odometry.pose.covariance[1-3] = accel_bias;
odometry.pose.covariance[4-6] = gyro_bias;
odometry.pose.covariance[7] = imuGravity;
```

### 4.4 回环检测

**视觉回环** (`loop_detection.cpp`):
- 使用DBoW2词袋模型进行图像检索
- `Detector::detectLoop()` 返回回环候选帧的分数
- 发送 `loop_time_cur, loop_time_pre` 给LiDAR系统

**LiDAR回环** (`mapOptmization.cpp:482-648`, `performLoopClosure()`):
1. 根据视觉回环的时间戳找到对应LiDAR关键帧ID
2. 提取当前帧和候选帧附近的LiDAR点云
3. ICP精配准 (`pcl::IterativeClosestPoint`, `mapOptmization.cpp:552-568`)
4. 如果ICP分数 < `historyKeyframeFitnessScore` → 添加回环约束因子到GTSAM图

**回环因子**: `gtsam::Pose3 poseFrom.between(poseTo)` 约束两个关键帧之间的相对位姿。


## 5. 关键创新点

### 5.1 LIO-SAM + VINS-Mono 的工程组合

LVI-SAM = **LIO-SAM** (LiDAR因子图) + **VINS-Mono** (视觉滑窗) + 消息桥接:

创新不在于新算法，而在于系统工程:
- 两个成熟系统通过ROS消息通信
- VIS给LIS提供初始位姿猜测
- LIS给VIS提供bias/gravity先验
- 视觉回环触发LiDAR精配准

### 5.2 多传感器回环检测

独特的混合回环:
1. **视觉DBoW** → 快速候选检索
2. **LiDAR ICP** → 精确约束验证
两者结合: 视觉速度 + LiDAR精度

### 5.3 鲁棒初始化

独立初始化 + 相互参考:
- 视觉初始化: VINS-Mono的五点法+SFM+对齐
- LiDAR初始化: iKd-Tree+IMU传播+L-M配准
- 任一系统失败时不阻塞另一系统

### 5.4 退化检测

L-M优化中的 `isDegenerate` 标志 (`matP` 矩阵检测):
```cpp
cv::eigen(matAtA, matE, matV);
// 检查最小特征值是否过小 → 退化
// 对退化方向限制更新幅度
```


## 6. 优缺点 + 意义

### 6.1 优势

1. **模块化**: 5个独立节点，单个节点崩溃不影响全局
2. **回环**: 支持DBoW+ICP双重验证的回环
3. **成熟系统**: 继承LIO-SAM和VINS-Mono的全部功能
4. **鲁棒性**: 任一传感器失败时降级运行

### 6.2 局限性

1. **松耦合**: LiDAR和视觉的约束没有统一的协方差结构
2. **5节点复杂度**: 部署调试困难，ROS通信开销
3. **视觉依赖KLT**: 快速运动和大视角变化下跟踪易丢失
4. **无RGB着色**: 纯里程计系统
5. **因子图计算开销**: ISAM2 + Ceres两个优化器的内存和CPU占用高

### 6.3 意义

1. 验证了松耦合LiDAR-Visual-Inertial的工程可行性
2. 松耦合vs紧耦合的参考基准
3. 多传感器回环检测的最佳实践


## 7. 三大框架技术路线差异

### 7.1 核心差异对比

| 维度 | FAST-LIVO2 | R3LIVE | LVI-SAM |
|------|-----------|--------|---------|
| **耦合方式** | 统一IESKF 紧耦合 | 双ESIKF 紧耦合 | GTSAM+Ceres 松耦合 |
| **系统结构** | 单进程 | 多线程单进程 | 多ROS节点 |
| **视觉方法** | 直接法(光度) | 重投影+LK | 特征点+KLT |
| **LiDAR地图** | VoxelMap | iKd-Tree | 体素+kd-tree |
| **回环** | 无 | 无 | DBoW+ICP |
| **RGB着色** | 简单投影 | 全局优化 | 无 |
| **3D重建** | 无 | MVS | 无 |
| **部署难度** | 低 | 中 | 高(5个launch文件) |
| **计算效率** | 最高 | 中 | 较低 |

### 7.2 技术路线本质分歧

三条路线的核心选择:

1. **FAST-LIVO2**: "用数学一致性换精度" — 统一IESKF保证协方差一致性，直接法利用全图像信息
2. **R3LIVE**: "用模块化换功能" — 双ESIKF实现RGB着色和在线标定等特色功能
3. **LVI-SAM**: "用系统工程换可靠性" — 松耦合可独立调试各子系统，支持回环

FAST-LIVO2 在精度上最优（统一协方差），R3LIVE 在视觉输出上最优（RGB着色+重建），LVI-SAM 在可靠性上最优（回环+降级运行）。


## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 原始格式 | ROS Topic | 消费节点 |
|--------|------|----------|-----------|----------|
| Velodyne/Ouster LiDAR | 10Hz | `sensor_msgs::PointCloud2` | `points_raw` | imageProjection → featureExtraction |
| IMU (9-axis) | 200-400Hz | `sensor_msgs::Imu` | `imu_raw` | imuPreintegration |
| 全局快门相机 | 10-30Hz | `sensor_msgs::Image` (mono) | `image_raw` | visual_odometry (feature_tracker) |

**特殊**: 五个ROS节点通过topic松耦合通信, 无共享内存。核心桥接消息 `lvi_sam::cloud_info` 承载角点/平面点/IMU姿态/VINS里程计。

### 8.2 LiDAR管线 (LIS子系统)

```
原始PointCloud2
  → imageProjection::cloudHandler(): 点云反序列化
  → deskewInfo(): 利用IMU旋转插值去畸变(无逐点时间戳时用方位角推算相对时间)
  → projectPointCloud(): 投影到range image (N_SCAN × Horizon_SCAN), 距离滤波
  → cloudExtraction(): 提取有效点, 记录startRingIndex/endRingIndex
  → 发布 lvi_sam::cloud_info (含原始点+ring信息)
  → featureExtraction::laserCloudInfoHandler():
      calculateSmoothness(): diffR = Σrange[i±5] - 10*range[i]; curvature = diffR²
      markOccludedPoints(): 遮挡/平行点标记, neighborPicked排除
      extractFeatures(): 每线6扇区取前20最大曲率→角点, 低曲率→平面点
  → 发布 lvi_sam::cloud_info (含cloud_corner + cloud_surface)
  → mapOptimization::laserCloudInfoHandler():
      cornerOptimization(): kd-tree 5近邻→PCA判断线特征(matD1[0]>3*matD1[1])→点到线距离
      surfOptimization(): kd-tree 5近邻→QR分解平面拟合→点到面距离
      LMOptimization(): LM迭代30次解6DOF, 含海森退化检测
```

**标定**: LiDAR-IMU外参预标定(lidar2Imu), 不变  
**预处理**: 运动畸变校正(IMU旋转插值) → range image投影 → 距离滤波  
**特征**: 角点(高曲率, `edgeThreshold`个) + 平面点(低曲率, `surfThreshold`个), 区分地面/非地面  
**匹配**: pcl::KdTreeFLANN 5近邻 → PCA线判定 / QR平面拟合  
**因子构建**: L-M `H^T*H*Δx = H^T*b`, 角点残差=点到线距离, 平面残差=点到面距离

### 8.3 视觉管线 (VIS子系统)

```
原始单目图像
  → feature_tracker::readImage():
      cv::goodFeaturesToTrack(): 提取Shi-Tomasi角点 (MAX_CNT=150)
      cv::calcOpticalFlowPyrLK(): KLT光流金字塔跟踪 (搜索窗口21×21)
      cv::findFundamentalMat(): RANSAC基础矩阵外点剔除
      setMask(): 均匀分布新特征(网格覆盖)
  → 发布 feature_points (含归一化平面坐标+像素速度+id)
  → estimator::processImage():
      relativePose(): 五点法求解本质矩阵 → 恢复R,t
      initialStructure(): 三角化+全局SFM (initial_sfm.cpp)
      visualInitialAlign(): 陀螺仪bias标定+速度/重力/尺度初始化
      solveOdometry(): 滑动窗口BA (Ceres, WINDOW_SIZE=10)
        IMU因子(imu_factor.h) + 视觉重投影因子(projection_factor.cpp) + 边缘化先验
```

**标定**: 相机内参预标定, IMU-相机外参预标定, 无在线标定  
**预处理**: 转灰度, 无畸变校正(假设已校准)  
**特征**: Shi-Tomasi角点 (150个), 无描述子, 依赖KLT跟踪  
**匹配**: KLT光流金字塔帧间跟踪 → 基础矩阵RANSAC外点剔除  
**因子构建**: `projection_factor.cpp: evaluate()` 重投影误差(∂proj/∂pose + ∂proj/∂point_position); `imu_factor.h: evaluate()` IMU预积分残差; 边缘化因子Schur补保留旧帧约束

### 8.4 IMU管线

```
sensor_msgs::Imu
  → imuPreintegration::imuHandler():
      integrateMeasurement(accel, gyro, dt): GTSAM PreintegratedImuMeasurements
      predict(prevState, prevBias): 预测当前NavState (位姿+速度)
  → 发布 odometry/imu (pose.covariance编码bias/gravity/重置id)
  → LIS消费: 用于去畸变 + 初始位姿猜测
  → VIS消费: IMU预积分因子嵌入Ceres滑动窗口, bias/gravity先验来自LIS
```

**预处理**: 无特殊滤波, 直接积分  
**因子**: LIS侧GTSAM `ImuFactor(Xk-1,Vk-1,Xk,Vk,Bk-1,preint)` + `BetweenFactor(Bk-1,Bk)`, VIS侧Ceres `imu_factor.h`

### 8.5 跨传感器协同

| 协同机制 | 实现位置 | 说明 |
|----------|----------|------|
| 时间同步 | 各节点独立以LiDAR时间戳为基准 | 点云header.stamp统一, IMU/image通过消息时间戳对齐 |
| 缓冲策略 | 各节点独立ROS消息队列 | 无共享缓冲, 依靠ROS消息传递 |
| VIS→LIS位姿先验 | `cloudInfo.odomAvailable` → `updateInitialGuess()` | VINS位姿增量变换到LiDAR系作为scan-to-map初始猜测 |
| LIS→VIS bias先验 | `odometry/imu` 中pose.covariance编码 | LiDAR优化的bias/gravity嵌入IMU消息传给VINS |
| 初始化顺序 | LIS和VIS各自独立初始化 → 相互参考 | LIS发布IMU bias后VINS以此为prior, 任一系统失败不阻塞 |
| 降级策略 | `cloudInfo.odomAvailable=false` → 纯IMU旋转积分 | VIS失败时LIS降级为纯IMU姿态猜测 |
| 回环协同 | VIS DBoW检索 → LIS ICP验证 | 视觉快速候选 + LiDAR精确约束, `performLoopClosure()` |
| 因子图分离 | GTSAM ISAM2 (LIS) + Ceres滑窗 (VIS) | 两个独立优化器, 通过ROS消息交换先验信息 |
