# LIO-SAM 源代码深潜分析

> 基于完整阅读源码 `imageProjection.cpp`, `featureExtraction.cpp`, `imuPreintegration.cpp`, `mapOptmization.cpp`, `utility.h`

---

## 1. 数据接收与预处理 (LiDAR Deskew, IMU Buffer, ROS Topics)

### 1.1 系统架构与 ROS 节点拓扑

LIO-SAM 由 **4 个 ROS nodelet** 串联而成，所有节点继承 `ParamServer` 类（`utility.h:63`），共享参数：

| 节点文件 | 主要功能 | 输入 Topic | 输出 Topic |
|---|---|---|---|
| `imageProjection.cpp` | 点云接收、IMU deskew、投影 | `pointCloudTopic`, `imuTopic` | `lio_sam/deskew/cloud_info` |
| `featureExtraction.cpp` | 曲率特征提取 | `lio_sam/deskew/cloud_info` | `lio_sam/feature/cloud_info` |
| `imuPreintegration.cpp` | IMU 预积分 & 高频里程计 | `imuTopic`, `lio_sam/mapping/odometry_incremental` | `odomTopic+"_incremental"` |
| `mapOptmization.cpp` | 帧-地图配准/GTSAM 因子图优化 | `lio_sam/feature/cloud_info` | `lio_sam/mapping/odometry` |

### 1.2 原始点云接收 (`imageProjection.cpp`)

**点云类型**：支持 Velodyne (XYZIRT)、Ouster (XYZIRT)、Livox 三种传感器（`utility.h:61`）。Velodyne 和 Ouster 的原始点云结构定义为：

- `VelodynePointXYZIRT`（`imageProjection.cpp:4-15`）：字段 `x, y, z, intensity, ring (uint16_t), time (float)`
- `OusterPointXYZIRT`（`imageProjection.cpp:17-31`）：字段 `x, y, z, intensity, t (uint32_t), reflectivity, ring (uint8_t), noise, range`

Ouster 数据会被转换为 Velodyne 格式（`imageProjection.cpp:214-227`），其中 Ouster 的 `t` 字段乘以 `1e-9` 转为秒。Livox 不支持 range image 投影，使用 `columnIdnCountVec` 顺序填充（`imageProjection.cpp:555-556`）。

**缓存策略**：`cloudQueue` 缓冲 2 帧后才开始处理（`imageProjection.cpp:201-202`），确保有足够的历史数据用于 deskew。

**必要的点云字段检查**（`imageProjection.cpp:248-281`）：
- `ring` 字段必须存在，否则退出（line 261-264）
- `time` 或 `t` 字段不存在时，deskew 被禁用，发出警告"system will drift significantly!"（line 280）

### 1.3 IMU Deskew 机制（`imageProjection.cpp:286-357`）

**核心思想**：在单帧扫描时间窗口内，用 IMU 角速度积分补偿每点的运动畸变。

**IMU 队列管理**（`imageProjection.cpp:149-172`）：`imuHandler` 回调将消息压入 `imuQueue`，首先经过 `imuConverter()` 进行坐标系对齐——加速度和角速度左乘外参旋转矩阵 `extRot`，姿态四元数左乘 `extQRPY`（`utility.h:252-282`）。

**积分过程**（`imageProjection.cpp:305-362`）：

```cpp
// imageProjection.cpp:310-313 - 清除过期 IMU 数据
while (!imuQueue.empty()) {
    if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
        imuQueue.pop_front();
}

// imageProjection.cpp:348-351 - 角速度积分
double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
```

**逐点校正**（`imageProjection.cpp:489-519`）：

```cpp
// imageProjection.cpp:510 - 构造相对变换
Eigen::Affine3f transBt = transStartInverse * transFinal;
// imageProjection.cpp:513-516 - 应用到点坐标
newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
```

**关键细节**：
- 旋转内插使用线性插值 (`findRotation`, line 446-471)：在两个 IMU 测量间线性插值 rotation
- 平移 deskew 被注释掉了（`findPosition`, line 473-487），作者注释"walking speed 时平移补偿效果不明显"
- 校正目标将所有点统一到**帧起始时刻**的坐标系（`transStartInverse`, line 504）

### 1.4 里程计 Deskew (`imageProjection.cpp:364-444`)

在 `odomDeskewInfo()` 中，从 `odomQueue`（来自 `imuPreintegration` 节点发布的增量里程计话题 `odomTopic+"_incremental"`）提取帧起始和帧末尾的姿态，计算帧内增量 `odomIncreX/Y/Z`（line 441），并提供 `initialGuessX/Y/Z/Roll/Pitch/Yaw` 作为后续 `mapOptmization` 中 ICP 的初始猜测（line 402-407）。

### 1.5 Range Image 投影 (`imageProjection.cpp:521-572`)

- 分辨率为 `N_SCAN × Horizon_SCAN`（默认 16×1800）（`utility.h:198`）
- Velodyne/Ouster：水平角 `atan2(x, y)` 映射到列索引（line 547-551）
- Livox：顺序填充列（line 555-556）
- 同一单元格保留第一个点（line 562-563），丢弃重复的
- 跳过 `rowIdn % downsampleRate != 0` 的行（降采样，line 541）
- 范围过滤 `[lidarMinRange, lidarMaxRange]`（line 534-535）

---

## 2. 点云特征提取 (Curvature-based Planar/Edge Features)

### 2.1 曲率计算 (`featureExtraction.cpp:81-101`)

```cpp
// featureExtraction.cpp:86-91
float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                + cloudInfo.pointRange[i+5];
cloudCurvature[i] = diffRange*diffRange;   // line 93
```

**曲率 =** `(Σ前后5个点的 range - 10×当前range)²`。这是经典的 LOAM 曲率定义——不是几何曲率，而是 range 值的二阶差分。边缘点曲率高，平面点曲率低。

### 2.2 遮挡与平行光束剔除 (`featureExtraction.cpp:103-139`)

**遮挡点标记**（line 114-131）：
- 相邻列差 < 10 且 `depth1 - depth2 > 0.3`（前景遮挡背景）：标记 [i-5, i] 为不可靠
- `depth2 - depth1 > 0.3`（背景被遮挡）：标记 [i+1, i+6] 为不可靠

**平行光束标记**（line 133-137）：
- 条件：`|range[i-1] - range[i]| > 0.02 * range[i]` 且 `|range[i+1] - range[i]| > 0.02 * range[i]`
- 平行于激光束的平面不可靠

### 2.3 特征选取策略 (`featureExtraction.cpp:141-237`)

**核心思想**：将每条线上的有效点分为 6 个子区域，在每个子区域内独立选取 edge/surface 特征。

```cpp
// featureExtraction.cpp:156-157 - 6 等分子区域
int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;
```

**边缘特征（corner）**（line 165-193）：
1. 按曲率降序排列
2. 选取曲率 `> edgeThreshold`（默认 0.1）且未被标记的
3. **每子区域最多 20 个**边缘特征（line 171）
4. 选中后标记自身 ±5 范围，抑制局部密集（line 178-192）

**平面特征（surface）**（line 196-222）：
1. 从曲率升序开始选取（曲率最小 = 最平面）
2. 选取曲率 `< surfThreshold`（默认 0.1）且未被标记的
3. 同样标记 ±5 范围
4. 所有平面候选和剩余点（`cloudLabel[k] <= 0`，line 226）经体素滤波 `odometrySurfLeafSize`（默认 0.2m）降采样

**最终输出**：
- `cornerCloud`（line 173）：每线最多 6×20=120，实际更少
- `surfaceCloud`（line 236）：经体素降采样后的平面点。注意此处 `*surfaceCloud += *surfaceCloudScanDS`（line 236）**直接累加 DS 而非 scan 原始**，意味着只有 DS 后的平面点（含 `cloudLabel[k] <= 0` 的剩余点）

### 2.4 与原始 LOAM 的区别

- LOAM 中有两类 "less sharp corner" 和 "less flat surface"，LIO-SAM 中只有 corner/surface 各一类
- LOAM 按曲率排序后取全局最大 top-N，LIO-SAM 改为**6 等分子区域 + 每子区域最多 20** 的策略（`featureExtraction.cpp:153-163`）
- 曲率计算相同：10 邻域 diff 的平方

---

## 3. 位姿计算：帧到地图的 Scan-to-Map Optimization

### 3.1 初始猜测来源 (`mapOptmization.cpp:786-845`)

`updateInitialGuess()` 按优先级提供：

1. **第一帧**（`cloudKeyPoses3D->points.empty()`, line 793）：直接用 IMU roll/pitch，yaw 根据 `useImuHeadingInitialization` 决定是否归零（line 799-800）
2. **IMU 预积分里程计可用**（`cloudInfo.odomAvailable`, line 809）：使用完整的 6DoF 增量预积分（`lastImuPreTransformation.inverse() * transBack`），累加到当前 `transformTobeMapped`（line 818-821）
3. **仅 IMU 旋转可用**（`cloudInfo.imuAvailable`, line 832）：只用 roll/pitch/yaw 的旋转增量（line 834-836），不动平移
4. **都没有**：使用上一帧姿态（line 837-838，即 `transTobe * transIncre` 在 transIncre=I 时退化为上一帧）

### 3.2 帧-地图 ICP 优化 (`mapOptmization.cpp:1282-1310`)

**scan2MapOptimization()** 是 LIO-SAM 的核心位姿估计：

```cpp
// mapOptmization.cpp:1292-1303 - 最多 30 次 LM 迭代
for (int iterCount = 0; iterCount < 30; iterCount++) {
    laserCloudOri->clear(); coeffSel->clear();
    cornerOptimization();      // line 1297 - 边缘特征点-线距离
    surfOptimization();        // line 1298 - 平面特征点-面距离
    combineOptimizationCoeffs(); // line 1300
    if (LMOptimization(iterCount) == true) break; // line 1302-1303
}
```

### 3.3 Corner 优化 (`mapOptmization.cpp:974-1063`)

**边缘点对线距离**：对每个 corner 点，在 corner map 中找 5 个最近邻，做 PCA（协方差分解），若最大特征值 > 3× 第二大特征值（line 1020），说明 5 个点分布在一条线上。

**点-线距离（点到直线距离）**（line 1032-1034）：
```cpp
// 直线由 PCA 方向 (matV1) 和均值中心 (cx,cy,cz) 定义
float a012 = sqrt( ... cross((p0-p1),(p0-p2)) ).norm() ... );
// 点到直线距离 = |cross(p-c, v)| / |v|
```

**Jacobian**：系数 `(la, lb, lc)` 对应平移，`s*ld2` 为残差（带自适应权重 `s=1-0.9*|ld2|`）
- 权重 `s` 当距离大时变小：`s = 1 - 0.9 * fabs(ld2)`（line 1049），相当于 Huber 核的软实现
- `s > 0.1` 才保留该关联（line 1056）

### 3.4 Surface 优化 (`mapOptmization.cpp:1066-1135`)

**平面点对面距离**：对每个 surf 点，在 surf map 中找 5 个最近邻，拟合一平面 `ax+by+cz+d=0`（通过最小二乘 `matA0.colPivHouseholderQr().solve(matB0)`，line 1096）

**平面有效性检查**（line 1106-1113）：所有 5 个最近点到拟合平面的距离 < 0.2m

```cpp
// mapOptmization.cpp:1117 - 点到平面距离
float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
// line 1119 - 自适应权重
float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x*pointOri.x + 
         pointOri.y*pointOri.y + pointOri.z*pointOri.z));
```

**Jacobian**：系数 `(pa, pb, pc)` 对应平面法向量

### 3.5 Levenberg-Marquardt 求解 (`mapOptmization.cpp:1158-1280`)

**关键细节**：
- 使用**LOAM 原始的坐标系变换**：lidar(x=forward, y=left, z=up) ↔ camera(x=right, y=front, z=up)，见注释（line 1161-1167）
- 旋转 Jacobian 用链式法则显式推导（line 1202-1213）
- **退化检测**（line 1229-1251）：在第一次迭代对 `J^T J` 做特征值分解，若某特征值 < 100 则标记退化，对退化维度做投影修正（`matP = matV.inv() * matV2`，line 1250）
- 收敛条件：`Δrotation < 0.05°` 且 `Δtranslation < 0.5mm`（line 1276-1277）

### 3.6 IMU Roll/Pitch 约束 (`mapOptmization.cpp:1312-1335`)

`transformUpdate()` 中，如果 IMU pitch 不在 ±1.4 rad（≈80°）范围（非纯旋转），对 roll/pitch 做**球面线性插值 (SLERP)**：

```cpp
// mapOptmization.cpp:1324-1327 - roll Slerp
transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, ...);
transformTobeMapped[0] = rollMid;
```

权重为 `imuRPYWeight`（默认 0.01，即 99% Lidar + 1% IMU）。

---

## 4. IMU 预积分 (`imuPreintegration.cpp`)

### 4.1 GTSAM IMU 预积分参数 (`imuPreintegration.cpp:208-229`)

```cpp
// imuPreintegration.cpp:214-217
boost::shared_ptr<gtsam::PreintegrationParams> p = 
    gtsam::PreintegrationParams::MakeSharedU(imuGravity);
p->accelerometerCovariance = Matrix33::Identity() * pow(imuAccNoise, 2);  // acc noise
p->gyroscopeCovariance     = Matrix33::Identity() * pow(imuGyrNoise, 2);  // gyro noise
p->integrationCovariance   = Matrix33::Identity() * pow(1e-4, 2);         // integration noise
```

### 4.2 双线程预积分器 (`imuPreintegration.cpp:177-178`)

```cpp
gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;  // 用于优化 (因子图构造)
gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;  // 用于高频发布 (>200Hz)
```

- `imuIntegratorOpt_` 在收到 LiDAR 里程计后，集成两帧 LiDAR 间所有 IMU 数据，构造因子
- `imuIntegratorImu_` 在 `imuHandler`（line 458-506）中**每条 IMU 消息**都积分，预测位姿实时发布

### 4.3 IMU 因子构造 (`imuPreintegration.cpp:350-380`)

收到 LiDAR 里程计增量（来自 `mapOptmization`）后：

```cpp
// line 370-372 - IMU 预积分因子
const gtsam::PreintegratedImuMeasurements& preint_imu = 
    dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
gtsam::ImuFactor imu_factor(X(key-1), V(key-1), X(key), V(key), B(key-1), preint_imu);
graphFactors.add(imu_factor);

// line 374-375 - Bias 随机游走因子 (BetweenFactor)
graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    B(key-1), B(key), gtsam::imuBias::ConstantBias(),
    gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

// line 378-379 - LiDAR 里程计因子 (作为 PriorFactor)
gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);  // transform to IMU frame
gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
    degenerate ? correctionNoise2 : correctionNoise);
graphFactors.add(pose_factor);
```

**三种噪声模型**（line 220-225）：
- `priorPoseNoise`: `(1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2)` — 初始姿态（平移/旋转）
- `correctionNoise`: `(0.05, 0.05, 0.05, 0.1, 0.1, 0.1)` — 正常 LiDAR odom
- `correctionNoise2`: `(1, 1, 1, 1, 1, 1)` — 退化情况，增大噪声

### 4.4 ISAM2 配置 (`imuPreintegration.cpp:231-243`)

```cpp
// imuPreintegration.cpp:233-236
gtsam::ISAM2Params optParameters;
optParameters.relinearizeThreshold = 0.1;
optParameters.relinearizeSkip = 1;
optimizer = gtsam::ISAM2(optParameters);
```

**关键**：
- `relinearizeThreshold = 0.1`：当线性化点偏离 0.1 以上触发 relinearize
- `relinearizeSkip = 1`：每次 update 都尝试 relinearize
- **每 100 帧重置 ISAM2**（line 320-347），保留边缘协方差作为 prior：从 `optimizer.marginalCovariance(X(key-1))` 提取（line 323）

### 4.5 故障检测 (`imuPreintegration.cpp:438-456`)

```cpp
// imuPreintegration.cpp:441-445 - 速度 > 30 m/s
if (vel.norm() > 30) { resetParams(); return true; }
// imuPreintegration.cpp:449-453 - bias > 1.0
if (ba.norm() > 1.0 || bg.norm() > 1.0) { resetParams(); return true; }
```

故障后重置预积分和系统初始化标志（line 401-403）。

---

## 5. 因子图设计 (GTSAM Factor Graph)

### 5.1 全局因子图 (`mapOptmization.cpp`)

`gtSAMgraph` 是全局因子图，**不优化速度/偏置**，只优化**每个关键帧的 Pose3**。

使用 Symbol：
```cpp
// mapOptmization.cpp:22-26
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
```

### 5.2 因子类型清单

**① PriorFactor<Pose3>** — 第一帧先验（`mapOptmization.cpp:1385-1386`）：
```cpp
noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
    (Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
```
噪声结构：**roll/pitch (1e-2) + yaw (π²≈10) + translation (1e8)**，即 yaw 和 translation 先验非常弱！

**② BetweenFactor<Pose3>** — 帧间里程计约束（`mapOptmization.cpp:1389-1392`）：
```cpp
noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances(
    (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(),
    poseFrom.between(poseTo), odometryNoise));
```
噪声：rotation=1e-6, translation=1e-4，即旋转约束远比平移紧。

**③ GPSFactor(gtsam::GPSFactor)** — GPS 先验（`mapOptmization.cpp:1465-1469`）：
```cpp
gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), 
    gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
gtSAMgraph.add(gps_factor);
```
添加条件：移动超过 5m（line 1407）且姿态协方差 > 阈值（line 1412）。

**④ BetweenFactor<Pose3>** — 回环约束（`mapOptmization.cpp:1477-1495`）：
```cpp
gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
```
来自 ICP 回环检测，噪声 = `Vector6(fitness_score.repeated<6>())`（line 596-597）。

### 5.3 ISAM2 配置 (`mapOptmization.cpp:159-162`)

```cpp
ISAM2Params parameters;
parameters.relinearizeThreshold = 0.1;
parameters.relinearizeSkip = 1;
isam = new ISAM2(parameters);
```

### 5.4 优化时机 (`mapOptmization.cpp:1497-1527`)

```cpp
void saveKeyFramesAndFactor() {
    if (saveFrame() == false) return;    // 非关键帧跳过
    addOdomFactor();                      // 添加里程计 BetweenFactor
    addGPSFactor();                       // (如果够远/协方差大)
    addLoopFactor();                      // (如果有新回环)
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();                       // 迭代一次 ISAM2
    if (aLoopIsClosed == true) {          // 回环后额外 5 次迭代
        isam->update(); isam->update(); 
        isam->update(); isam->update(); 
        isam->update();
    }
    gtSAMgraph.resize(0); initialEstimate.clear(); // 清空，增量式添加
}
```

**关键**：ISAM2 **增量式**工作，每次只添加新因子，不清空历史状态。回环闭合时额外跑 5 次 `update()` 确保收敛（line 1518-1525）。

### 5.5 关键帧选取策略 (`mapOptmization.cpp:1354-1379`)

```cpp
// mapOptmization.cpp:1372-1375 - 关键帧添加条件
if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&   // 0.2 rad ≈ 11.5°
    abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
    abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
    sqrt(x*x+y*y+z*z) < surroundingkeyframeAddingDistThreshold) // 1.0 m
    return false;  // 不添加到全局因子图
```
**只有当位姿变化超过阈值时才在 GTSAM 因子图中创建新节点**，避免冗余。

### 5.6 姿态校正 (`mapOptmization.cpp:1583-1614`)

回环闭合后，`correctPoses()` 遍历所有关键帧（line 1595-1609），从 `isamCurrentEstimate` 中获取优化后的 Pose3，更新 `cloudKeyPoses3D/6D`，并清除 `laserCloudMapContainer` 缓存（line 1591）。

---

## 6. 回环检测与全局优化 (`mapOptmization.cpp:517-774`)

### 6.1 回环检测策略

**距离回环** (`detectLoopClosureDistance`, line 610-643)：
- 在 `copy_cloudKeyPoses3D` 中 KD-tree 半径搜索 `historyKeyframeSearchRadius`（默认 10m）
- 时间差 > `historyKeyframeSearchTimeDiff`（默认 30s）才接受

```cpp
// mapOptmization.cpp:629
if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > 
    historyKeyframeSearchTimeDiff) { ... }
```

**外部回环** (`detectLoopClosureExternal`, line 645-697)：预留接口，未使用，请忽略。

### 6.2 ICP 回环验证 (`mapOptmization.cpp:558-608`)

```cpp
// mapOptmization.cpp:559-564 - PCL ICP 配置
static pcl::IterativeClosestPoint<PointType, PointType> icp;
icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
icp.setMaximumIterations(100);
icp.setTransformationEpsilon(1e-6);
icp.setEuclideanFitnessEpsilon(1e-6);
icp.setRANSACIterations(0);  // 不使用 RANSAC
```

验证条件（line 572）：
- `icp.hasConverged()` 为 true
- `icp.getFitnessScore() < historyKeyframeFitnessScore`（默认 0.3）

**ICP 输入**：
- source：当前关键帧及其 ±0 邻域（loopFindNearKeyframes(cloud, loopKeyCur, 0)，line 550）
- target：候选关键帧及其 ±historyKeyframeSearchNum 邻域（默认 25，line 551）
- source < 300 点或 target < 1000 点直接放弃（line 552）

### 6.3 回环因子添加 (`mapOptmization.cpp:600-608`)

```cpp
loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
loopPoseQueue.push_back(poseFrom.between(poseTo));
loopNoiseQueue.push_back(constraintNoise);
loopIndexContainer[loopKeyCur] = loopKeyPre;
```

这些在 `addLoopFactor()` 中被统一添加到 `gtSAMgraph`（line 1477-1495）。

---

## 7. 初始化

### 7.1 IMU 预积分初始化 (`imuPreintegration.cpp:252-316`)

首次收到 LiDAR 里程计后执行：

```cpp
// imuPreintegration.cpp:274-316
if (systemInitialized == false) {
    // 步骤 1: 弹出当前时刻前的 IMU 数据 (line 279-288)
    // 步骤 2: 添加 PriorFactor<Pose3> (line 291) - 对齐到 LiDAR
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
    // 步骤 3: 添加 PriorFactor<Vector3> (line 295-296) - 零速度先验
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
    // 步骤 4: 添加 PriorFactor<imuBias::ConstantBias> (line 299-300) - 零偏置先验
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
    // 步骤 5: ISAM2 optimizer.update(graphFactors, graphValues) (line 306)
    // 步骤 6: 重置预积分器 (line 310-311)
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
    systemInitialized = true;
}
```

### 7.2 建图初始化 (`mapOptmization.cpp:793-804`)

```cpp
// mapOptmization.cpp:795-797 - 第一帧用 IMU roll/pitch 初始化
transformTobeMapped[0] = cloudInfo.imuRollInit;
transformTobeMapped[1] = cloudInfo.imuPitchInit;
transformTobeMapped[2] = cloudInfo.imuYawInit;
if (!useImuHeadingInitialization)
    transformTobeMapped[2] = 0;  // yaw 从 0 开始
```

---

## 8. 关键创新点 (为什么是 v2)

### 8.1 LIO-SAM 相比 LeGO-LOAM (v1) 的改进

| 维度 | LeGO-LOAM (v1) | LIO-SAM (v2) |
|---|---|---|
| **传感器融合** | LiDAR-only | LiDAR + IMU + GPS |
| **IMU 处理** | 无 | GTSAM IMU 预积分 (`imuPreintegration.cpp:208`) |
| **优化框架** | 自定义 LM (帧优化) | 2 层：LM (scan2map) + ISAM2 (全局图, `mapOptmization.cpp:162`) |
| **状态估计** | 6DoF Pose | 15DoF State (Pose + Vel + Bias, `imuPreintegration.cpp:20-22`) |
| **因子图** | 无 | Prior + Between + GPS + Loop Between (`mapOptmization.cpp:1377-1495`) |
| **回环** | ICP + 位姿图 | ISAM2 增量 + 回环 5× 迭代 (`mapOptmization.cpp:1518-1525`) |
| **deskew** | 无 IMU deskew | IMU 角速度积分 deskew (`imageProjection.cpp:489-519`) |

### 8.2 关键设计选择

1. **帧-地图 LM + ISAM2 全局图分离**：前端 30 次 LM 迭代获得相对位姿（`mapOptmization.cpp:1292`），后端 ISAM2 做全局优化（`isam->update(graph)` line 1515）— 两级解耦
2. **LiDAR odom 是 IMU 预积分的 "correction"**：IMU 因子用 `PreintegratedImuMeasurements` 预测（line 381），LiDAR 位姿作 `PriorFactor`（line 378），而非传统的 IMU 紧耦合
3. **退化处理**：J^T J 特征值分析 + 投影修正（`matP` line 1250），退化维度增大 noise（`correctionNoise2` line 378）
4. **回环触发 5× ISAM2 更新**：确保全局 BA 收敛（`mapOptmization.cpp:1518-1525`）

---

## 9. 优缺点 + 对 SLAM 算法的意义

### 9.1 优点

1. **精度高**：IMU 预积分 + ISAM2 全局优化 + GPS 因子 + 回环，在 KITTI 等数据集达到 SOTA
2. **鲁棒性强**：IMU roll/pitch 约束（Slerp）、退化检测 + 噪声自适应、故障自恢复（bias/velocity 阈值）
3. **模块化设计**：4 个 ROS nodelet 分离，易于替换模块（换 LiDAR、换 IMU）
4. **实时性**：前端 30 次 LM + 后端 ISAM2 增量优化 + 双线程 IMU 预积分
5. **多传感器兼容**：Velodyne/Ouster/Livox + 9-axis IMU + GPS

### 9.2 缺点

1. **IMU 松耦合**：LiDAR 里程计以 `PriorFactor` 形式而非 `BetweenFactor` 形式注入 IMU 预积分图，并非真正的紧耦合（见 `imuPreintegration.cpp:378`）
2. **回环依赖 PCL ICP**：精度依赖 `historyKeyframeFitnessScore`（0.3）阈值，无描述子验证
3. **特征方法局限性**：在隧道/空旷场景角点平面点不足时退化（退化检测可捕捉但不能完全解决）
4. **初始化依赖 IMU**：重力对齐需要 IMU 静止或慢速运动，纯 LiDAR 模式 yaw 漂移不可避免
5. **不维护全局地图**：`VoxelHashMap` 缓存限制 1000 项（line 936），只维护局部滑动窗口地图

### 9.3 对 SLAM 算法的意义

LIO-SAM 代表了 **LiDAR-Inertial SLAM 工程化部署** 的标杆：
- 证明了 IMU 预积分 + ISAM2 的工程可行性（>200Hz IMU + 10Hz LiDAR 实时）
- 两阶段优化（LM前端 + ISAM2后端）成为后续工作的标准范式（FAST-LIO2, Faster-LIO 等）
- 松耦合 IMU 策略虽然在理论上有损精度，但极大简化实现，适合工程部署
- 退化检测 + 自适应噪声策略被广泛借鉴

---

## 10. 对比：FAST-LIO2 vs R3LIVE vs LVI-SAM 的技术路线差异

| 维度 | LIO-SAM | R3LIVE | LVI-SAM |
|---|---|---|---|
| **融合策略** | LiDAR-IMU 松耦合 | LiDAR-Visual-IMU 紧耦合 | LIO-SAM + VINS-Mono 组合 |
| **视觉** | 无 | RGB 直接法着色 + 重建 | 特征点法 (VINS-Mono) |
| **IMU** | IMU 预积分 (GTSAM) | 滤波 (ESIKF) | VINS 预积分 + LIO-SAM 预积分 |
| **优化后端** | ISAM2 因子图 | ESIKF 迭代卡尔曼 (地图线程也因子图) | 两个独立因子图 (LIO + VIO) |
| **实时地图** | 无纹理点云 | 纹理 3D 高斯/点云 | 点云 (无纹理) |
| **计算负载** | ~9.5ms/frame (4核) | ~42ms/frame (GPU) | ~100ms/frame (4核) |
| **SOTA 场景** | KITTI, 户外驾驶 | 室内外纹理场景 | 户外 GPS 辅助 |

---

**文件索引**：

| 文件 | 功能 | 关键行 |
|---|---|---|
| `include/utility.h` | ParamServer, 点类型, 坐标转换 | 4-347 |
| `src/imageProjection.cpp` | 点云接收, IMU deskew, range 投影 | 1-620 |
| `src/featureExtraction.cpp` | 曲率特征提取, 遮挡剔除 | 1-272 |
| `src/imuPreintegration.cpp` | IMU 预积分, ISAM2, TransformFusion | 1-524 |
| `src/mapOptmization.cpp` | 帧-地图 LM 优化, GTSAM 因子图, 回环 | 1-1779 |