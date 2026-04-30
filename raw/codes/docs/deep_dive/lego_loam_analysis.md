# LeGO-LOAM 深度源码分析

> 基于 LeGO-LOAM 仓库 (Tixiao Shan & Brendan Englot, IROS 2018) 的完整源码级分析。
> 核心源文件: `LeGO-LOAM/src/` 下 4 个 .cpp, 1 个头文件 `include/utility.h`。

---

## 1. 数据接收与预处理

### 1.1 系统架构概览

LeGO-LOAM 由四个独立的 ROS 节点组成,运行在单个进程 (`ros::spin()`) 或独立进程(如 `featureAssociation` 和 `mapOptmization` 以 200Hz 循环运行):

| 文件 | 类 | 功能 | 运行方式 |
|------|------|------|---------|
| `imageProjection.cpp:37-523` | `ImageProjection` | 点云投影、地面分割、聚类 | `ros::spin()` |
| `featureAssociation.cpp:37-1886` | `FeatureAssociation` | 去畸变、特征提取、里程计 | 200Hz 循环 |
| `mapOptmization.cpp:49-1551` | `mapOptimization` | scan-to-map 优化、回环、iSAM2 | 200Hz 循环 + 独立线程 |
| `transformFusion.cpp:35-254` | `TransformFusion` | 位姿融合发布 | `ros::spin()` |

### 1.2 点云接收与投影

`imageProjection.cpp:181-197` 的 `cloudHandler()` 是整个系统的入口:

```cpp
// imageProjection.cpp:181-197
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    copyPointCloud(laserCloudMsg);     // 1. ROS -> PCL 转换
    findStartEndAngle();              // 2. 计算起止角度
    projectPointCloud();              // 3. 距离图像投影
    groundRemoval();                  // 4. 地面标记
    cloudSegmentation();              // 5. 点云聚类分割
    publishCloud();                   // 6. 发布结果
    resetParameters();                // 7. 重置状态
}
```

**点云订阅** (`imageProjection.cpp:89`):
```cpp
subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, ...);
```

`utility.h:53` 定义 topic: `/velodyne_points`。

### 1.3 距离图像投影 (`projectPointCloud()`)

`imageProjection.cpp:211-257` 将 3D 点云投影到 2D 距离图像:

- **行索引** (线号): 优先使用 Velodyne 的 `ring` 通道 (`utility.h:60`: `useCloudRing = true`), 直接从 `PointXYZIR` 结构读取 (`imageProjection.cpp:226`)
- **列索引** (水平角): `imageProjection.cpp:237` — `columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2`
- **分辨率**: VLP-16 默认 `N_SCAN=16`, `Horizon_SCAN=1800`, `ang_res_x=0.2°`, `ang_res_y=2.0°` (`utility.h:63-67`)

`imageProjection.cpp:248` 将 range 值存入 `rangeMat`, 并用 `fullCloud` 维护投影后的有序点云:
```cpp
rangeMat.at<float>(rowIdn, columnIdn) = range;
fullCloud->points[index] = thisPoint;
```

---

## 2. 地面分割算法

### 2.1 核心原理

`imageProjection.cpp:260-310` 的 `groundRemoval()` 是 LeGO-LOAM 最关键的创新之一。它基于**相邻线束间的垂直角度**判断地面点。

**算法步骤** (`imageProjection.cpp:267-291`):

```cpp
for (size_t j = 0; j < Horizon_SCAN; ++j){
    for (size_t i = 0; i < groundScanInd; ++i){  // groundScanInd=7 仅检视下方 7 条线
        lowerInd = j + (i) * Horizon_SCAN;
        upperInd = j + (i+1) * Horizon_SCAN;

        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

        angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;

        if (abs(angle - sensorMountAngle) <= 10){  // 角度阈值 10°
            groundMat.at<int8_t>(i,j) = 1;
            groundMat.at<int8_t>(i+1,j) = 1;
        }
    }
}
```

**关键参数** (`utility.h`):
- `groundScanInd = 7`: 只检测 VLP-16 的下面 7 条线束 (0-6), 保证仅地面区域参与判断
- `sensorMountAngle = 0.0°`: 传感器安装俯仰角
- 地面判定阈值: `abs(angle - sensorMountAngle) <= 10°`

**判断逻辑**: 对于同一水平角 (列) 的相邻两条线束 (行 i 和 i+1), 计算两点连线与水平面的夹角。若该夹角接近于 0 (传感器水平安装时), 则判定为地面点。

### 2.2 为什么需要地面分割?

1. **特征提取分离**: 地面点被提取为平面特征 (`segmentedCloudGroundFlag`), 而非地面点提取为边缘特征 — 这使得特征分类天然依赖于物理语义
2. **减少计算量**: `imageProjection.cpp:336-339` 对地面点进行大幅降采样 (每 5 个取 1 个, 且跳过边缘):
   ```cpp
   if (groundMat.at<int8_t>(i,j) == 1){
       if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
           continue;
   }
   ```
3. **两步优化基础**: 地面平面约束提供了 roll/pitch 的强观测, 使得后续边缘匹配只需优化 yaw + tx + ty + tz

### 2.3 点云聚类分割

`imageProjection.cpp:312-460` 使用基于距离图像的 BFS 分割。关键细节:

- **邻域连接条件** (`imageProjection.cpp:416-422`):
  ```cpp
  if ((*iter).first == 0)    alpha = segmentAlphaX; // 水平方向 0.2° = ang_res_x
  else                       alpha = segmentAlphaY; // 垂直方向 2.0° = ang_res_y
  angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));
  if (angle > segmentTheta){  // segmentTheta = 60° (utility.h:113)
      // 两点属于同一聚类
  }
  ```
  这是经典的**角度阈值聚类**: 若两相邻像素的连线与较远点方向的夹角足够大 (表示两点处于同一表面上), 则归为同一聚类。

- **聚类有效性判定** (`imageProjection.cpp:441-460`):
  - ≥30 个点: 直接有效
  - 5-29 个点: 需跨 ≥3 条线束 (`segmentValidLineNum=3`)
  - 不满足条件的标记为 `labelCount=999999` (异常点)

- **队列实现**: 使用预分配数组 `queueIndX/Y` + `allPushedIndX/Y` (`imageProjection.cpp:80-82`) 替代 `std::queue`, 避免动态内存分配

---

## 3. 特征提取: edge/planar 分类

### 3.1 与原始 LOAM 的区别

原始 LOAM (Zhang & Singh, RSS 2014) 的特征提取基于 **曲率 (`c`) 的绝对值**, 在所有方向上选取最大/最小曲率点。LeGO-LOAM 的核心改进是**利用地面分割结果引导特征分类**:

| 方面 | 原始 LOAM | LeGO-LOAM |
|------|-----------|-----------|
| 边缘特征提取 | 从整个扫描的曲率最大值中选取 | 仅从**非地面点**中选取 (`featureAssociation.cpp:706`) |
| 平面特征提取 | 从整个扫描的曲率最小值中选取 | 仅从**地面点**中选取 (`featureAssociation.cpp:741`) |
| 环境适应性 | 对所有方向均匀提取 | 地面和非地面特征在几何上天然分类 |

### 3.2 平滑度计算

`featureAssociation.cpp:621-641`:
```cpp
void calculateSmoothness(){
    for (int i = 5; i < cloudSize - 5; i++) {
        float diffRange = segInfo.segmentedCloudRange[i-5] + ... + [i-1]
                        - segInfo.segmentedCloudRange[i] * 10
                        + [i+1] + ... + [i+5];
        cloudCurvature[i] = diffRange * diffRange;
    }
}
```

使用了与原始 LOAM 相同的公式: $c = (\sum_{j\in[-5,5]\setminus\{0\}} r_j - 10\cdot r_i)^2$

### 3.3 特征提取策略

`featureAssociation.cpp:680-783` 按 6 个子区域分别提取, 每个子区域:

**边缘点** (`featureAssociation.cpp:701-733`):
```cpp
if (cloudNeighborPicked[ind] == 0 &&
    cloudCurvature[ind] > edgeThreshold &&
    segInfo.segmentedCloudGroundFlag[ind] == false) {  // 必须是非地面点！
    // 前 2 个: sharp corner (cloudLabel=2)
    // 第 3-20 个: less sharp corner (cloudLabel=1)
}
```

**平面点** (`featureAssociation.cpp:736-769`):
```cpp
if (cloudNeighborPicked[ind] == 0 &&
    cloudCurvature[ind] < surfThreshold &&
    segInfo.segmentedCloudGroundFlag[ind] == true) {  // 必须是地面点！
    // 最多取 4 个 flat surface 点
}
```

关键常数 (`utility.h:120-124`):
```cpp
extern const int edgeFeatureNum = 2;   // 每子区域 sharp edges
extern const int surfFeatureNum = 4;   // 每子区域 flat surfaces
extern const int sectionsTotal = 6;    // 子区域数量
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
```

### 3.4 遮蔽点剔除

`featureAssociation.cpp:643-678` 检测两类不可靠点:
1. **平行于激光束的平面边缘**: `depth1 - depth2 > 0.3` 时标记前序点为不可靠
2. **被遮挡的边缘**: 两点距离变化 >2% range 时剔除

---

## 4. 两步位姿优化: LeGO-LOAM 的里程计核心

### 4.1 IMU 去畸变 (`adjustDistortion()`)

`featureAssociation.cpp:491-619` 先对每帧点云做运动补偿:

```cpp
float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
// 在 IMU 缓冲区中查找 relTime 对应的位姿
// 将所有点统一变换到 scan-start 时刻
```

### 4.2 两步优化 (`updateTransformation()`)

`featureAssociation.cpp:1666-1695` 是 LeGO-LOAM 最核心的设计:

```cpp
void updateTransformation(){
    // Step 1: 仅使用地面平面特征 (surfFeatures) — 优化 roll, pitch, ty
    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
        findCorrespondingSurfFeatures(iterCount1);   // 对应关系
        if (calculateTransformationSurf(iterCount1) == false) break;  // LM优化
    }
    // Step 2: 仅使用非地面边缘特征 (cornerFeatures) — 优化 yaw, tx, tz
    for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
        findCorrespondingCornerFeatures(iterCount2);
        if (calculateTransformationCorner(iterCount2) == false) break;
    }
}
```

**优化的变量分离**:
- `calculateTransformationSurf()` (`featureAssociation.cpp:1270-1377`): 只更新 `transformCur[0]` (roll), `[2]` (yaw), `[4]` (ty) — 这是因为地面提供的约束主要约束了 roll/pitch 和垂直方向平移
- `calculateTransformationCorner()` (`featureAssociation.cpp:1379-1478`): 只更新 `transformCur[1]` (pitch), `[3]` (tx), `[5]` (tz) — 边缘提供水平方向的约束

**为什么两步分离有效?**

地面点全部是平坦的, 它们 (a) 对 roll/pitch 极其敏感, (b) 在水平方向提供不了足够约束。先用地面的强约束锁定旋转, 再用边缘完成平移, 是一个高效的**解耦优化策略**。这与原始 LOAM 的同步 6-DOF 优化形成对比 — LeGO-LOAM 不需要昂贵的非线性迭代, 每个步骤都是独立的 Gauss-Newton (实际上是 OpenCV `cv::solve` 的 QR 分解)。

### 4.3 特征对应关系

**边缘特征匹配** (`featureAssociation.cpp:1044-1153`):
当前帧的边缘点, 在上一帧的边缘点云 (`laserCloudCornerLast`) 中找**最近的两个点**(需跨不同线束), 将点到直线的距离作为残差。Jacobian 通过链式法则解析求导。

**平面特征匹配** (`featureAssociation.cpp:1155-1268`):
当前帧的平面点 (地面点), 在上一帧平面点云 (`laserCloudSurfLast`) 中找**最近的三个点**(需两个来自同线束), 将点到平面的距离作为残差。

### 4.4 退化检测 (Degeneracy Detection)

`featureAssociation.cpp:1329-1356` 和 `1379-1478`: 在第一次迭代时对 $A^T A$ 做 SVD 分解, 若特征值 < 10 则标记退化, 通过投影矩阵 `matP` 在退化方向上抑制增量。这直接来自 Ji Zhang 的原始工作。

---

## 5. 回环检测与建图

### 5.1 Scan-to-Map 优化

`mapOptmization.cpp:1329-1349` 的 `scan2MapOptimization()`:

```cpp
void scan2MapOptimization(){
    for (int iterCount = 0; iterCount < 10; iterCount++) {
        cornerOptimization(iterCount);  // 边缘 -> 局部地图
        surfOptimization(iterCount);    // 平面 -> 局部地图
        if (LMOptimization(iterCount) == true) break;  // 6-DOF LM 全优化
    }
    transformUpdate();  // IMU 融合
}
```

与里程计不同, Mapping 阶段做的是**全 6-DOF 同时优化** (`LMOptimization()` at `mapOptmization.cpp:1229-1327`), Jacobian 相对于 `transformTobeMapped` 的 6 个变量推导。

**局部地图构建** (`mapOptmization.cpp:956-1065`):
- 回环关闭模式: 用 `deque` 维护最近 50 个关键帧 (`surroundingKeyframeSearchNum=50`)
- 无回环模式: 半径搜索 (`surroundingKeyframeSearchRadius=50.0m`)

### 5.2 GTSAM iSAM2 因子图

`mapOptmization.cpp:36-47` 引入 GTSAM:

```cpp
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
```

**因子图构建** (`mapOptmization.cpp:1355-1453`):
- 首帧: `PriorFactor<Pose3>` (`mapOptmization.cpp:1376-1378`)
- 后续: `BetweenFactor<Pose3>` 连接相邻关键帧 (`mapOptmization.cpp:1384-1391`)

**噪声模型** (`mapOptmization.cpp:348-350`):
```cpp
gtsam::Vector Vector6(6);
Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;  // 旋转噪声 < 平移噪声
priorNoise = noiseModel::Diagonal::Variances(Vector6);
odometryNoise = noiseModel::Diagonal::Variances(Vector6);
```

ISAM2 参数 (`mapOptmization.cpp:229-232`): `relinearizeThreshold=0.01`, `relinearizeSkip=1`。

### 5.3 回环检测

`mapOptmization.cpp:875-945` `performLoopClosure()`:

```cpp
void performLoopClosure(){
    if (potentialLoopFlag == false){
        if (detectLoopClosure() == true){
            potentialLoopFlag = true;
        }
    }
    // ICP 配准: maxCorrespondenceDistance=100, maxIterations=100
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.align(*unused_result);
    // 若 hasConverged && fitnessScore < historyKeyframeFitnessScore(0.3)
    // 则添加 BetweenFactor 约束, 触发 isam->update()
}
```

核心检测逻辑 (`mapOptmization.cpp:814-872` `detectLoopClosure()`):
1. 在 `cloudKeyPoses3D` 中做半径搜索 (7m, `historyKeyframeSearchRadius=7.0`)
2. 找出时间差 >30s 的历史帧
3. 融合 ±25 个相邻关键帧形成局部子图 (`historyKeyframeSearchNum=25`)
4. ICP 配准 (当前帧 vs 历史子图), fitness score < 0.3 则为有效回环

---

## 6. 与 LeGO-LOAM 原始论文的差异

| 论文描述 | 代码实现 (重要差异) |
|----------|-------------------|
| 论文强调 **地面优化** | `featureAssociation.cpp:706` 边缘特征严格排除地面点 (论文也提到, 但代码实现更加硬性约束) |
| 论文描述两步 LM 优化 | 代码 (`featureAssociation.cpp:1270-1478`) 实际使用的是**分变量优化**, 且每步都是小规模 3×3 矩阵求解, 并非真正 Levenberg-Marquardt (阻尼因子固定为 0) |
| 回环检测 "Euclidean-distance-based" | 代码 (`mapOptmization.cpp:830`) 使用**时间差 >30s** (而非仅距离) 作为附加条件, 避免近邻误匹配 |
| 论文描述 6 段均匀分割 | `featureAssociation.cpp:691-694` 使用的是**非均匀加权插值**: `sp = (startRingIndex[i] * (6 - j) + endRingIndex[i] * j) / 6` |
| IMU 仅做初始猜测 | 代码 (`mapOptmization.cpp:488-489`) 还使用了**指数平滑**: `transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast` |
| 论文描述 6-DOF scan-to-map | `LMOptimization()` (`mapOptmization.cpp:1229-1327`) 实际上使用了不同于里程计的 Jacobian 推导 (不包含平移的 `coeff.x/y/z` 依赖关系) |

---

## 7. 优缺点 + 对SLAM算法的意义

### 7.1 优点

1. **计算效率极高**: 地面分割 + 两步解耦优化使得每帧只需 ~20次 迭代 (原始 LOAM 需要 `25+25` + `10` 次 scan-to-map), 且每步求逆的矩阵规模仅 3×3
2. **地面语义引导**: `groundRemoval()` (imageProjection.cpp:260) 将物理语义引入 SLAM 前端, 天然区分了 edge (非地面) 和 planar (地面) 特征
3. **鲁棒初始化**: `checkSystemInitialization()` (featureAssociation.cpp:1605) 直接使用 IMU 提供的 roll/pitch, 无需 LOAM 中的复杂初始对齐
4. **GTSAM iSAM2**: 增量平滑建图框架使回环可以持续加入而不重建全局 BA
5. **轻量级**: 单进程运行, 无 GPU 依赖, 可在嵌入式平台 (如 NVIDIA Jetson) 上实时运行

### 7.2 缺点

1. **严重依赖地面**: 在非结构化环境 (如楼梯、斜坡、空中无人机) 中, `groundScanInd=7` 的地面假设失效, 整个特征提取流程退化
2. **无重定位**: 系统丢失后无法自动恢复, 也没有全局定位能力
3. **LiDAR-only**: 不支持视觉传感器, 在低纹理或光照变化场景无法互补
4. **回环检测简单**: 仅依赖 ICP + 欧氏距离, 缺乏外观验证或词袋模型
5. **硬编码传感器参数** (`utility.h:63-67`): N_SCAN=16, Horizon_SCAN=1800 等对 VLP-16 优化, 换用其他 LiDAR 需手动修改 (虽然头文件已有 32/128 线的注释配置)

### 7.3 对SLAM领域的意义

LeGO-LOAM 是**工程优化 vs 算法完整性**的经典案例:
- 它证明了**环境先验 (地面)** 可以显著简化 SLAM 问题 — 用 ~500 行 C++ 取得了与 LOAM 相当的精度
- **两步优化** 思路启发了后续工作 (FAST-LIO 系列中的运动补偿 + IESKF)
- 它将 iSAM2 引入 LiDAR SLAM, 建立了**因子图 + LiDAR scan-to-map** 的范式
- 其 **2722 stars** 说明了社区对 "轻量高效" LiDAR SLAM 的巨大需求

---

## 参考文献引用 (源码内)

1. `imageProjection.cpp:29-33`: J. Zhang and S. Singh, LOAM, RSS 2014; T. Shan and B. Englot, LeGO-LOAM, IROS 2018
2. `featureAssociation.cpp:29-33`: 同上
3. `mapOptmization.cpp:30-33`: 同上
4. GTSAM 库: F. Dellaert, Georgia Tech

---

## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| LiDAR (VLP-16) | 10 Hz | `sensor_msgs::PointCloud2` (XYZIR) | `imageProjection.cpp:89` | ImageProjection |
| IMU (9-axis, 可选) | 200 Hz | `sensor_msgs::Imu` | `featureAssociation.cpp:52` | FeatureAssociation (deskew + 初始猜测) |

### 8.2 LiDAR 管线

#### 8.2.1 原始数据

- VLP-16：16 线，360° 水平扫描，点间隔 ≈0.2°，约 28,800 点/帧
- **点格式**：`pcl::PointXYZIR`（`utility.h:60`），使用 `ring` 通道（`useCloudRing = true`）获取线号
- **topic**：`/velodyne_points`（`utility.h:53`）
- **标定**：传感器安装角 `sensorMountAngle = 0.0°`（`utility.h`），用于地面分割角度基准

#### 8.2.2 预处理（Range Image 投影）

`imageProjection.cpp:211-257` `projectPointCloud()`：

| 参数 | 值 |
|---|---|
| 行数 `N_SCAN` | 16 |
| 列数 `Horizon_SCAN` | 1800 |
| 水平分辨率 `ang_res_x` | 0.2°（360/1800） |
| 垂直分辨率 `ang_res_y` | 2.0° |

- **行索引**：直接从 `PointXYZIR.ring` 读取
- **列索引**：`columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN/2`
- 存入 `rangeMat`（`float`）和 `fullCloud`（有序点云）

#### 8.2.3 地面分割（Ground Removal）

`imageProjection.cpp:260-310` `groundRemoval()`：

- **核心原理**：基于**相邻线束间垂直角度**判断，仅检测下方 7 条线（`groundScanInd = 7`）
- **判定流程**：
  ```
  for j ∈ [0, Horizon_SCAN), i ∈ [0, groundScanInd):
      diffX = fullCloud(upperInd).x - fullCloud(lowerInd).x
      diffY = fullCloud(upperInd).y - fullCloud(lowerInd).y
      diffZ = fullCloud(upperInd).z - fullCloud(lowerInd).z
      angle = atan2(diffZ, sqrt(diffX² + diffY²)) * 180/π
      if |angle - sensorMountAngle| ≤ 10°:
          groundMat(i,j) = 1; groundMat(i+1,j) = 1
  ```
- **降采样**：地面点每 5 个取 1 个（`j%5 != 0` 跳过）（`imageProjection.cpp:336-339`）

#### 8.2.4 点云聚类（Cloud Segmentation）

`imageProjection.cpp:312-460` `cloudSegmentation()`：

使用**基于距离图像的 BFS 分割**：
- **邻域连接条件**：
  ```
  angle = atan2(d2·sin(alpha), d1 - d2·cos(alpha))
  if angle > segmentTheta (60°):  → 属于同一聚类
  ```
  水平方向 `alpha = segmentAlphaX = 0.2°`，垂直方向 `alpha = segmentAlphaY = 2.0°`
- **聚类有效性**：≥30 点直接有效；5-29 点需跨 ≥3 条线束（`segmentValidLineNum=3`），否则标记 `labelCount=999999`（异常点）
- 使用预分配数组 `queueIndX/Y` + `allPushedIndX/Y` 替代 `std::queue`，避免动态内存分配

#### 8.2.5 特征提取

**IMU 去畸变**（`featureAssociation.cpp:491-619` `adjustDistortion()`）：
- 在 IMU 缓冲区中查找对应时间戳的位姿，将所有点统一变换到 scan-start 时刻
- 仅旋转补偿（`imuShiftFromStartX/Y/Z/Cur` 分别存储）

**曲率计算**（`featureAssociation.cpp:621-641` `calculateSmoothness()`）：
```
c_i = (Σ_{j=i-5}^{i+5,j≠i} r_j - 10·r_i)²
```
与原始 LOAM 相同公式。

**特征分类（关键创新）**（`featureAssociation.cpp:680-783`）：

6 等分子区域，每子区域：
- **边缘（Sharp corner）**：曲率 > `edgeThreshold`（0.1）且**非地面点**（`groundFlag == false`），前 2 个为 sharp（`cloudLabel=2`），第 3-20 个为 less sharp（`cloudLabel=1`）
- **平面（Flat surface）**：曲率 < `surfThreshold`（0.1）且**地面点**（`groundFlag == true`），最多 4 个
- 常数：`edgeFeatureNum=2`, `surfFeatureNum=4`, `sectionsTotal=6`

**遮挡剔除**（`featureAssociation.cpp:643-678`）：
- 相邻列差 > 0.3 标记前后点为不可靠
- 平行于激光束的边缘：两点距离变化 >2% range 时剔除

#### 8.2.6 匹配/配准（两步 LM 解耦优化）

`featureAssociation.cpp:1666-1695` `updateTransformation()`：

**步骤 1：地面平面优化**（仅优化 roll, pitch, ty）：
```
for iter = 0..24:
    findCorrespondingSurfFeatures(iter)   // 在上一帧 surf 点云找 3 个最近邻
    calculateTransformationSurf(iter)      // 3×3 矩阵 QR 分解
```
仅更新：`transformCur[0]` (roll), `[2]` (yaw), `[4]` (ty)

**步骤 2：边缘优化**（仅优化 yaw, tx, tz）：
```
for iter = 0..24:
    findCorrespondingCornerFeatures(iter)  // 在上一帧 corner 点云找 2 个最近邻（需不同线束）
    calculateTransformationCorner(iter)    // 3×3 矩阵 QR 分解
```
仅更新：`transformCur[1]` (pitch), `[3]` (tx), `[5]` (tz)

**残差模型**：
- 边缘：点到直线距离（使用边缘方向向量）
- 平面：点到平面距离（使用平面法向量 + 常数项）

**退化检测**（`featureAssociation.cpp:1329-1356`）：对 `A^T A` 做 SVD，特征值 < 10 则退化，在退化方向上应用投影矩阵 `matP` 抑制增量

#### 8.2.7 算法消费

**Scan-to-Map 全 6-DOF 优化**（`mapOptmization.cpp:1329-1349`）：
- 最多 10 次 LM 迭代
- 同时使用 corner 和 surf 特征，Jacobian 对 `transformTobeMapped` 的 6 个变量推导

**GTSAM iSAM2 因子图**（`mapOptmization.cpp:1355-1453`）：
- **PriorFactor**（首帧）：噪声 `(1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6)`
- **BetweenFactor**（帧间）：同上噪声
- ISAM2：`relinearizeThreshold=0.01`, `relinearizeSkip=1`

**回环检测**（`mapOptmization.cpp:814-945`）：
- 半径搜索 7m，时间差 >30s
- 融合 ±25 帧构建局部子图
- PCL ICP：`maxCorrespondenceDistance=100m`, `maxIterations=100`
- fitnessScore < 0.3 为有效回环

### 8.3 IMU 管线

- 仅用于去畸变和初始姿态猜测
- 指数平滑融合（`mapOptmization.cpp:488-489`）：`transformTobeMapped[0] = 0.998·transformTobeMapped[0] + 0.002·imuPitchLast`

### 8.4 跨传感器协同

**初始化**：`checkSystemInitialization()`（`featureAssociation.cpp:1605`）直接使用 IMU roll/pitch 初始化，无需复杂初始对齐

**时间同步**：IMU 缓冲区 + scan 时间戳查找对应位姿

**降级**：
- 无地面场景（空中、斜坡）：`groundScanInd=7` 假设失效，特征提取退化
- 非结构化场景：聚类分割可能失效