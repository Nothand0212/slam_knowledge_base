# ROLO-SLAM 深度源码分析报告

> 分析日期：2026-04-28
> 代码版本：JFR 2025 发表版本
> 源码路径：`/home/lin/Projects/lin_ws/slam_ws/ROLO/`

---

## 1. 数据接收与预处理

### 1.1 传感器数据入口与 ROS 话题

ROLO-SLAM 是一个 **LiDAR-Only** SLAM 系统，专为地面车辆在不平坦地形设计。由四个 ROS 节点模块组成管道：

| 模块 | 源文件 | 订阅 TOPIC | 发布 TOPIC |
|------|--------|-----------|-----------|
| `imageProjection` | `src/imageProjection.cpp:516` | LiDAR 原始点云、前端里程计 | `rolo/cloud_info`, `rolo/range_image` |
| `featureExtraction` | `src/featureExtraction.cpp:291` | `rolo/cloud_info` | `rolo/feature/cloud_info`, 角点/平面点/地面点 |
| `lidarOdometry` | `src/lidarOdometry.cpp:597` | `rolo/feature/cloud_info`, `rolo/mapping/odometry` | 前端增量里程计、位姿预测 |
| `backMapping` | `src/backMapping.cpp:1617` | 前端 `odomTopic/cloud_info` | `rolo/mapping/odometry`, 全局地图 |

### 1.2 点云预处理与去畸变

#### 1.2.1 点云格式转换

`ImageProjection::cachePointCloud()` (`src/imageProjection.cpp:179-263`)：

- **Velodyne**：直接 `pcl::fromROSMsg` 解析为 `VelodynePointXYZIRT`（含 ring 和 time 字段）
- **Ouster**：解析为 `OusterPointXYZIRT`，转换为 Velodyne 格式，`time = src.t * 1e-9f`（纳秒转秒）
- 验证 `is_dense` 标志和 `ring`/`time` field 存在性

#### 1.2.2 去畸变 (Deskewing)

`ImageProjection::deskewCloudInfo()` (`src/imageProjection.cpp:266-366`)：

- **条件**：`deskewEnabled && odomAvailable`
- **两种模式**：
  1. **无逐点时间戳** (timeFlag == -1)：基于方位角计算每个点的相对时间 `relTime = (ori - startOri) / orientationDiff`
  2. **有逐点时间戳** (timeFlag == 1)：直接用 `laserCloudIn->points[i].time`
- **去畸变算法**：`deskewPoint()` (`:368-396`)
  - 利用前端里程计的帧间变换（`odomIncreX/Y/Z/Roll/Pitch/Yaw`）
  - 匀速模型插值：`transBt = getTransformation(0,0,0, -trans(3), -trans(4), -trans(5))` — **仅补偿旋转**（平移补偿在注释中被禁用）
  - `newPoint = transBt * point`（齐次变换）

#### 1.2.3 Range Image 投影

`ImageProjection::projectPointCloud()` (`src/imageProjection.cpp:399-474`)：

```
rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, FLT_MAX)
```

- 行索引 = 扫描线 ID（通过俯仰角计算 或 ring 字段）
- 列索引 = 水平方位角 `columnIdn = -round((horizonAngle - 90°) / ang_res) + Horizon_SCAN/2`
- 距离滤波：`lidarMinRange < range < lidarMaxRange`
- 降采样：`rowIdn % downsampleRate != 0` 跳过
- 去畸变后的点填充到 `fullCloud->points[index]`
- 发布 range image 作为 `sensor_msgs::Image` 用于调试

#### 1.2.4 点云提取

`ImageProjection::cloudExtraction()` (`src/imageProjection.cpp:477-505`)：

- 遍历 rangeMat 中的有效像素
- 记录每行有效点的起止索引（`startRingIndex[i]`, `endRingIndex[i]`，保留左右各 5 个点的余量用于曲率计算）
- 填充 `extractedCloud`（有效去畸变点）

---

## 2. 特征提取

### 2.1 特征类型

ROLO 提取三类特征：**角点 (Corner)**、**平面点 (Surface)**、**地面点 (Normal/Normal surface)**

### 2.2 平滑度计算（LOAM 风格）

`FeatureExtraction::calculateSmoothness()` (`src/featureExtraction.cpp:87-110`)：

```cpp
float diffRange = range[i-5] + range[i-4] + ... + range[i-1]
                - range[i] * 10
                + range[i+1] + range[i+2] + ... + range[i+5];
cloudCurvature[i] = diffRange * diffRange;
```

与原始 LOAM 完全一致：在 range image 的扫描线方向计算局部平滑度。

### 2.3 异常点标记

`FeatureExtraction::markOccludedPoints()` (`src/featureExtraction.cpp:112-150`)：

- **遮挡点**：相邻点深度突变 > 0.3m（`depth1 - depth2 > 0.3`），标记前后各 5 个点
- **平行点**：前后点深度差都大于 `0.02 * range[i]`，标记该点
- 标记为 `cloudNeighborPicked[i] = 1`，排除在特征提取外

### 2.4 特征选取

`FeatureExtraction::extractFeatures()` (`src/featureExtraction.cpp:153-265`)：

- 每条扫描线分为 **6 个扇区**，每个扇区内：
  1. 按平滑度 **从大到小排序**，取前 20 个最大平滑度点 → 角点
  2. 按平滑度 **从小到大排序**，取低平滑度点 → 平面点（标记为 -1）
  3. 选定点周围 5 个邻点标记为已选
- 角点阈值：`cloudCurvature > edgeThreshold`（默认 0.1）
- 平面点阈值：`cloudCurvature < surfThreshold`（默认 0.1）
- 平面点经过 `odometrySurfLeafSize` 体素滤波（默认 0.2m）

### 2.5 特征数量

- 每线每扇区最多 20 个角点 × 6 扇区 × N_SCAN 线 ≈ 1920 个角点 (16 线)
- 平面点数量取决于体素滤波后的密度

---

## 3. 位姿计算

### 3.1 位姿估计方法

ROLO 前端的核心创新是**分离式位姿估计**：先估计旋转，再估计平移 (`lidarOdometry.cpp`)：

```
scanRegistration() {
    1. Forward Location Prediction (translation interpolation)
    2. Spherical Alignment → RotVGICP → Rotation Estimation
    3. Continuous-Time Translation Estimation
}
```

### 3.2 旋转估计：RotVGICP

`LidarOdometry::scanRegeistration()` (`src/lidarOdometry.cpp:331-383`)：

```cpp
// 步骤 1: 平移插值（前向位置预测）
pcl::transformPointCloud(*featureOld, *feature_propagated, transformation_interpolated);

// 步骤 2: 旋转配准
fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
rot_vgicp.setResolution(1.0);
rot_vgicp.setInputTarget(featureLast);
rot_vgicp.setInputSource(feature_propagated);
rot_vgicp.align(*aligned);
Eigen::Matrix4f trans = rot_vgicp.getFinalTransformation();
transformation_interpolated = transformation_interpolated * transformStep;
Rotation = transformation_interpolated.rotation().cast<double>();
```

**RotVGICP** (`include/rot_gicp/gicp/rot_vgicp.hpp:24-130`) 是自定义的 GICP 变体：

- 基于 `LsqRegistration` → 最小二乘配准框架
- **SO(3) 线性化**：`so3_linearize()` 直接在旋转流形上做高斯-牛顿
- **体素化**：使用 `VmfVoxelMap`（Voxelized Mean-Free Gaussian Voxel Map）预计算目标点云的体素分布
- **协方差计算**：`calculate_covariances()` 为源和目标点云计算邻域协方差矩阵（GICP 风格）
- 支持 OpenMP 多线程（`setNumThreads()`）

### 3.3 平移估计：连续时间

`rot_vgicp.computeTranslation()` (`rot_vgicp.hpp:101-103`)：

```cpp
rot_vgicp.computeTranslation(*aligned, Reg_translation, 
    Translation, TranslationOld, interval_tn, interval_tn_1, CT_lambda);
Translation += Reg_translation;
```

- **连续时间约束**：利用两帧之间的时间间隔 `interval_tn`, `interval_tn_1` 对平移进行平滑约束
- `CT_lambda`（默认 1.0）控制连续时间平滑项权重
- 平移更新量受速度限制和正则化约束

### 3.4 前向位置预测

`LidarOdometry::stateLinearPropagation()` (`src/lidarOdometry.cpp:582-594`)：

- 利用后端优化输出的帧间变换（`lidarMappingAffine`）
- 按时间比例插值：`trans_vec *= propagation_ratio`
- **仅传播平移**：`trans_vec.tail(3) = Eigen::Matrix<float, 3, 1>::Zero()`（旋转置零）
- 作为 RotVGICP 的初始猜测，实现先平移对齐再进行旋转匹配

### 3.5 前端里程计发布

`TransformFusion` 类 (`src/lidarOdometry.cpp:42-206`) 将后端优化位姿与前端增量位姿融合：
- 使用后端位姿 + 前端增量变换 = 高频位姿输出
- 发布 `odom → base_link` TF 变换

---

## 4. 初始化

### 4.1 初始位姿

`LidarOdometry::Init()` (`src/lidarOdometry.cpp:293-319`)：

- `Rotation = Matrix3d::Identity()`
- `Translation = Vector3d::Zero()`
- 第一帧到达时直接作为参考帧存储，不做配准

### 4.2 后端初始化

`backMapping::updateInitialGuess()` (`src/backMapping.cpp:354-393`)：

- 前 `saveKeyFramesAndFactor()` 中第一帧添加强先验因子 `PriorFactor<Pose3>` (`backMapping.cpp:1024-1043`)
  - 噪声：`(1e-2, 1e-2, π², 1e8, 1e8, 1e8)`（偏航和位移弱约束，roll/pitch 强约束，利用地面车辆假设）
- 后续帧添加 `BetweenFactor<Pose3>` 里程计因子

### 4.3 坐标系框架

- `odometryFrame` (odom)
- `mapFrame` (map)
- `lidarFrame` / `baselinkFrame`
- **初始时 map ≡ odom**（静态 TF 发布）

---

## 5. 多传感器融合

### 5.1 传感器

ROLO-SLAM 是 **LiDAR-Only** 系统：
- **LiDAR**：Velodyne (16/32/64 线) 或 Ouster
- **IMU 里程计**：`odomTopic` 可选，用于 de-skewing 和初始位姿猜测
- 无相机、无 GPS（代码中有 GPS 订阅声明但未实现 `backMapping.cpp:76`）

### 5.2 融合方式

ROLO 的 LiDAR-IMU 关系是**辅助性的松散耦合**：

1. **IMU/里程计 → 去畸变**：`ImageProjection::deskewPoint()` 使用里程计帧间变换进行运动补偿
2. **IMU/里程计 → 前端初始化**：`stateLinearPropagation()` 使用后端位姿插值作为前端初始猜测
3. **IMU/里程计 → 后端初始化**：`updateInitialGuess()` 使用里程计增量变换更新后端初始位姿

系统可以在**无 IMU 情况下运行**（纯 LiDAR Odometry），此时：
- 去畸变被跳过（`deskewEnabled=false`）
- 前向预测使用后端优化结果自插值

### 5.3 融合架构图

```
[LiDAR] → [imageProjection (range image + deskew)]
                ↓ cloud_info
         [featureExtraction (LOAM features)]
                ↓ cloud_info + features
         [lidarOdometry (RotVGICP rotation + CT translation)]
                ↓ incremental odometry
         [backMapping (scan2map + GTSAM ISAM2 PGO)]
                ↓ global pose
         [TransformFusion] → 高频里程计输出
```

---

## 6. 后端优化

### 6.1 回环检测

回环检测在 `backMapping::loopClosureThread()` (`src/backMapping.cpp:1314-1329`) 中以独立线程运行。

#### 6.1.1 距离回环检测

`backMapping::detectLoopClosureDistance()` (`:1424-1458`)：

- 对最近关键帧，在 `historyKeyframeSearchRadius` (默认 10m) 半径内搜索历史关键帧
- 要求时间间隔 > `historyKeyframeSearchTimeDiff` (默认 30s)
- 选择距离满足且时间间隔最大的历史帧作为回环候选

#### 6.1.2 ICP 回环验证

`backMapping::performLoopClosure()` (`:1332-1422`)：

- 收集当前帧周围 ±0 帧（即仅当前帧特征）和历史帧周围 ±25 帧的特征点云
- 使用 PCL `IterativeClosestPoint` 进行 ICP：
  - `MaxCorrespondenceDistance = historyKeyframeSearchRadius * 2` (20m)
  - `MaxIterations = 100`
  - `RANSACIterations = 0`
- 验证条件：`hasConverged() && fitnessScore < historyKeyframeFitnessScore` (默认 0.3)
- 回环边的噪声 = `Diagonal::Variances(fitnessScore, ...)` (6 维相同)

### 6.2 全局优化

**库**：GTSAM `ISAM2`

`backMapping::saveKeyFramesAndFactor()` (`src/backMapping.cpp:932-1021`)：

```cpp
// 添加里程计因子
addOdomFactor();   // PriorFactor (首帧) 或 BetweenFactor
// 添加回环因子  
addLoopFactor();   // BetweenFactor (回环边)
// ISAM2 优化
isam->update(gtSAMgraph, initialEstimate);
isam->update();
if (aLoopIsClosed) {
    isam->update();  // 回环时多次优化
    isam->update();
    isam->update();
    isam->update();
    isam->update();
}
```

**优化器参数** (`backMapping.cpp:156-159`)：
- `relinearizeThreshold = 0.1`
- `relinearizeSkip = 1`

### 6.3 局部建图 (Scan-to-Map)

`backMapping::scan2MapOptimization()` (`src/backMapping.cpp:519-549`)：

- 需要足够特征：`edgeFeatureMinValidNum` (默认 10) 个角点 + `surfFeatureMinValidNum` (默认 100) 个平面点
- 提取周围关键帧特征 → `extractSurroundingKeyFrames()` → `extractNearby()`
  - Radius Search 半径 = `surroundingKeyframeSearchRadius` (默认 50m)
  - 额外加入 10 秒内的关键帧（防止原地旋转时参考不足）
- **30 次高斯-牛顿迭代**：
  - `cornerOptimization()`：PCA 线特征匹配 → 点到线距离残差
  - `surfOptimization()`：QR 分解平面拟合 → 点到面距离残差
  - `LMOptimization()`：构造雅可比矩阵 A，求解 `A^T A * Δx = A^T b`
  - 退化检测：海森矩阵特征值 < 100 时启用退化投影 `matP = V⁻¹ * V'`
  - 收敛判断：`ΔR < 0.05° && ΔT < 0.05cm`

### 6.4 姿态约束（地面车辆特性）

`backMapping::transformUpdate()` (`src/backMapping.cpp:898-906`)：

- roll/pitch 角限幅：`constraintTransformation(value, rotation_tollerance)`
- z 坐标限幅：`constraintTransformation(value, z_tollerance)`
- 利用地面车辆运动约束防止发散

### 6.5 回环校正

`backMapping::correctPoses()` (`src/backMapping.cpp:1067-1100`)：

- 回环发生后：遍历 `isamCurrentEstimate` 重新赋值所有历史关键帧位姿
- 清除点云缓存 `laserCloudMapContainer`
- 重新生成全局 Path

### 6.6 地图可视化

`backMapping::visualizeGlobalMapThread()` (`src/backMapping.cpp:1237-1255`) 以 0.2Hz 频率发布全局地图（降采样后）。

---

## 7. 优缺点分析

### 7.1 算法优缺点

**优点：**
- **分离式旋转/平移估计**：先对齐中心再做旋转匹配，降低了耦合度，提高了垂直方向的精度（论文核心贡献）
- **连续时间平移约束**：利用时间信息平滑平移估计，减少帧间跳跃
- **RotVGICP**：在 SO(3) 上做高斯-牛顿，避免了四元数/Euler 角的奇异性
- **地面车辆运动约束**：roll/pitch/z 的限幅有效抑制了垂直漂移
- **完整的前端→后端→回环管道**：功能完善

**缺点：**
- **纯 LiDAR**：退化环境（长隧道、开阔场地）鲁棒性不足
- **RotVGICP 计算量大**：每次配准需要重新计算协方差和体素分布
- **前向预测假设**：假设帧间运动匀速，急转弯/急加减速时可能引入偏差
- **回环检测简单**：仅用距离搜索，无外观描述子，重訪场景可能漏检

### 7.2 工程优缺点

**优点：**
- 代码结构清晰，四个模块职责分明
- 参数集中管理（`ParamLoader` 类）
- OpenMP 多线程并行化（ICP、点云变换、回环检测等）
- 支持 TUM 格式轨迹保存
- 点云类型兼容 Velodyne/Ouster

**缺点：**
- ROS1 (Melodic/Noetic)，未适配 ROS2
- 全局地图容器 `laserCloudMapContainer` 仅简单容量限制（> 1000 清空），无 LRU 策略
- 部分功能未完成（`detectLoopClosureExternal` 标注 "not used yet"）
- 前端里程计融合策略中 `failureDetection` 被注释掉
- 测试代码硬编码文件路径 (`/home/abin/tum_traj/rolo.tum`)

### 7.3 适用场景

**适合：**
- 地面车辆在不平坦地形（越野、山地、矿区）
- 垂直方向精度要求高的应用（层高检测、地形分析）
- LiDAR-only 部署（无 IMU/相机约束时仍可运行）
- 结构化 + 半结构化环境

**不适合：**
- 空中机器人（无地面约束假设）
- 极度动态的城市场景（回环检测简单）
- 实时性要求极高的应用（RotVGICP 计算密集）
- 长隧道/长廊等退化环境

---

## 8. 对 phad_fusion 参考

### 8.1 值得借鉴的设计

1. **分离式旋转/平移估计** (`lidarOdometry.cpp:331-383`)：
   - 对 phad_fusion 的 LiDAR 前端设计有重要参考价值，先平移预对齐再旋转匹配可以显著提高垂直方向精度
   - 适用于任何需要抑制垂直漂移的场景

2. **RotVGICP 的实现** (`include/rot_gicp/gicp/rot_vgicp.hpp`, `src/rot_gicp/gicp/rot_vgicp.cpp`)：
   - SO(3) 流形上的直接优化，避免了参数化问题
   - 体素化 GICP（Voxelized GICP）的思路可以用于大规模点云的快速配准
   - 协方差预计算和体素累积模式可复用

3. **连续时间约束** (`rot_vgicp.hpp:101-103`)：
   - 利用时序信息平滑运动估计，phad_fusion 如果有高频传感器（如 IMU 200Hz）可以在优化中加入类似的正则项

4. **Range Image 预处理管道** (`imageProjection.cpp:399-505`)：
   - 完整的去畸变 → 投影 → 邻域搜索索引构建流程
   - `startRingIndex/endRingIndex/pointColInd/pointRange` 的元数据设计精巧

5. **地面车辆运动约束** (`backMapping.cpp:898-906`)：
   - roll/pitch/z 的限幅是一种简单有效的先验注入方式
   - 可扩展到各类平台的运动约束（如无人机 pitch/roll 约束等）

6. **海森矩阵退化检测** (`backMapping.cpp:844-866`)：
   - 在 GN 迭代中实时检测约束矩阵的退化，并通过投影矩阵 `matP` 进行矫正
   - 比 SuperOdom 的直方图方法更直接

### 8.2 应避免的设计

1. **过度简单的回环检测**：仅距离+时间筛选对复杂环境不可靠，应至少加入 Scan Context 或类似描述子

2. **全局地图容器粗放清理**：> 1000 帧直接清空太粗暴，应实现 LRU 缓存或分区管理

3. **前向预测的匀速假设**：在非平稳运动中会引入误差，phad_fusion 应使用 IMU 预积分提供更准确的预测

4. **去畸变仅补偿旋转**：`deskewPoint` 中平移补偿被注释掉，在不平坦地形的高速运动中可能不够

5. **无 IMU 紧耦合**：作为地面车辆系统，融入 IMU 可以大幅提升极端运动下的鲁棒性


## 9. 数据管线

### 9.1 传感器输入总览

| 传感器 | 频率 | 原始格式 | ROS接口 | 消费模块 |
|--------|------|----------|---------|----------|
| Velodyne LiDAR | 10Hz | `sensor_msgs::PointCloud2` | `imageProjection::cloudHandler()` → `rolo/cloud_info` | imageProjection → featureExtraction → lidarOdometry → backMapping |
| Ouster LiDAR | 10Hz | `sensor_msgs::PointCloud2` (含t字段) | 同Velodyne, `time = src.t * 1e-9f` 转换 | 同上 |
| 前端里程计(可选) | 10Hz | `nav_msgs::Odometry` | 用于去畸变 + 初始猜测 | imageProjection + lidarOdometry + backMapping |

**纯LiDAR系统**: 无IMU紧耦合, 无相机。可选里程计仅用于辅助去畸变和初始化。

### 9.2 LiDAR管线

```
原始PointCloud2 (Velodyne/Ouster)
  → imageProjection::cachePointCloud():
      Velodyne: pcl::fromROSMsg→VelodynePointXYZIRT (含ring+time)
      Ouster: 解析为OusterPointXYZIRT → 格式转换为Velodyne → time=src.t*1e-9
  → deskewCloudInfo():
      条件: deskewEnabled && odomAvailable
      无逐点time时: relTime = (ori-startOri)/orientationDiff (按方位角推算)
      有逐点time时: 直接用laserCloudIn->points[i].time
      deskewPoint(): 匀速模型插值, 仅补偿旋转(newPoint = transBt*point), 平移补偿被注释
  → projectPointCloud():
      rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, FLT_MAX)
      行=ring(俯仰角), 列=-round((horizonAngle-90°)/ang_res)+Horizon_SCAN/2
      距离滤波: lidarMinRange < range < lidarMaxRange
      降采样: rowIdn % downsampleRate != 0 跳过
  → cloudExtraction():
      遍历rangeMat有效像素 → startRingIndex/endRingIndex (左右留5点用于曲率计算)
      → extractedCloud (有效去畸变点)
  → 发布 rolo/cloud_info (含extractedCloud + ring信息)
  → featureExtraction::laserCloudInfoHandler():
      calculateSmoothness(): diffR=Σrange[i±5]-10*range[i], curvature=diffR² (LOAM风格)
      markOccludedPoints(): 相邻深度突变>0.3m标记遮挡, 前后各5点排除
      extractFeatures(): 每线6扇区, 每扇区取curvature前20大→角点, 低→平面点
      体素滤波平面点: odometrySurfLeafSize=0.2m
  → 发布 rolo/feature/cloud_info (角点+平面点+地面点)
  → lidarOdometry::scanRegistration():
      1. stateLinearPropagation(): 后端帧间变换插值 → 仅传播平移(旋转置零)
      2. RotVGICP: SO(3)上高斯-牛顿 → 旋转最优解
      3. computeTranslation(): 连续时间约束平滑平移估计
  → backMapping::scan2MapOptimization():
      cornerOptimization(): PCA线判定 → 点到线距离
      surfOptimization(): QR平面拟合 → 点到面距离
      LMOptimization(): GN 30次迭代, 海森退化检测(matP投影)
```

**标定**: LiDAR-IMU外参无(纯LiDAR), 可选前端里程计坐标系对齐  
**预处理**: 格式统一(Velodyne/Ouster) → 运动畸变校正(仅旋转) → range image投影 → 距离滤波 → 降采样  
**特征**: 角点(高curvature>0.1) + 平面点(低curvature<0.1) + 地面点(法向), LOAM风格6扇区提取  
**匹配**: RotVGICP: 体素化VmfVoxelMap + 邻域协方差 → GICP风格协方差匹配 → SO(3)线性化优化  
**因子构建**: 6-DOF LM → 角点Jacobian(3×6) + 平面点Jacobian(1×6), 海森退化检测特征值<100触发投影

### 9.3 可选里程计管线

```
nav_msgs::Odometry (10Hz)
  → imageProjection: deskewPoint()运动补偿
  → lidarOdometry: stateLinearPropagation()前向预测初始猜测
  → backMapping: updateInitialGuess()后端初始化
  │
  └ 无里程计时: 跳过去畸变, 前向预测用后端自插值
```

### 9.4 跨传感器协同

| 协同机制 | 实现位置 | 说明 |
|----------|----------|------|
| 时间同步 | 以LiDAR `header.stamp` 为准 | 纯LiDAR系统无多传感器时对齐问题 |
| 去畸变辅助 | `deskewEnabled && odomAvailable` | 前端里程计提供帧间变换, 仅补偿旋转(平移注释掉) |
| 初始化 | 首帧Identity位姿 → 后端强先验(偏航/位置弱, roll/pitch强) | 利用地面车辆假设: `PriorFactor(1e-2, 1e-2, π², 1e8, 1e8, 1e8)` |
| 前向预测 | 后端优化帧间变换插值 → 仅传播平移 | `trans_vec *= propagation_ratio`, `trans.tail(3)=Zero()` |
| 退化检测 | LM中Hessian特征值<100 | `matP = V⁻¹*V'` 投影矩阵限制退化方向更新 |
| 地面约束 | roll/pitch/z限幅 | `constraintTransformation(value, rotation_tollerance/z_tollerance)` |
| 降级策略 | 无里程计时纯LiDAR | 去畸变跳过, 预测用后端自插值