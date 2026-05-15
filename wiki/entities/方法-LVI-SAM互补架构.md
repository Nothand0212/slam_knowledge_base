---
type: entity
tags: [SLAM, LiDAR-视觉融合, LVI-SAM, 因子图, 传感器融合, 紧耦合]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/LVI-SAM
---

# LVI-SAM 互补架构

> LIO（LiDAR-Inertial Odometry）和 VIS（Visual-Inertial System）两个子系统并行运行，通过共享位姿估计实现相互备份和紧耦合融合的 LiDAR-视觉-惯性 SLAM 系统。

## 定义

LVI-SAM（Lidar-Visual-Inertial Smoothing and Mapping）将 LIO-SAM 的 LiDAR-惯性里程计和 VINS-Mono 的视觉-惯性里程计在系统级紧耦合。两个子系统分别维护自己的因子图后端，在位姿初猜、初始化、退化检测和回环检测四个层面交换信息。

## 核心架构

```
          ┌──────────────┐
          │   IMU 数据    │
          └──────┬───────┘
                 │
     ┌───────────┴───────────┐
     │                       │
     ▼                       ▼
┌─────────┐            ┌─────────┐
│  LIO    │◄──位姿初猜──│  VIS    │
│ 子系统  │───LiDAR位姿─►│ 子系统  │
└────┬────┘            └────┬────┘
     │                      │
     ▼                      ▼
┌─────────┐            ┌─────────┐
│ ISAM2   │            │ Ceres   │
│ 因子图  │            │ 滑窗优化│
└────┬────┘            └────┬────┘
     │                      │
     └──────┬───────────────┘
            ▼
     ┌──────────────┐
     │  全局位姿图   │
     │  + 回环检测   │
     └──────────────┘
```

### LIO 子系统（LiDAR-Inertial Odometry）

基于 LIO-SAM，通过 GTSAM 的 ISAM2 增量平滑维护 LiDAR 里程计的因子图：

```cpp
// raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L55-L61
NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;
```

因子图包含三类因子：
- **Odom factor**（`BetweenFactor<Pose3>`）：相邻关键帧间的 LiDAR scan-to-map 配准结果
- **GPS factor**：可选，全局位置约束
- **Loop factor**：闭环检测产生的位姿约束

因子构造代码参见 `raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L1364-L1383`（`addOdomFactor`），iSAM 更新在 `raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L1497-L1502`。

### VIS 子系统（Visual-Inertial System）

继承 VINS-Mono 的滑动窗口优化器，10 帧窗口内通过 Ceres 非线性优化维护位姿、速度、IMU 偏置和特征点深度。窗口管理（滑动 + 边缘化）在 `raw/codes/LVI-SAM/src/visual_odometry/visual_estimator/estimator.cpp:L995-L1116`。

利用 LiDAR 提供的初始位姿加速初始化，通过 `initialStructure()` 直接设置窗口内所有帧的位姿和速度（`raw/codes/LVI-SAM/src/visual_odometry/visual_estimator/estimator.cpp:L215-L273`）。

## 子系统间通信机制

### cloud_info 消息 —— 统一接口

两个子系统通过自定义 ROS 消息 `lvi_sam::cloud_info` 交换信息（`raw/codes/LVI-SAM/msg/cloud_info.msg:L1-L32`）：

```msg
# Cloud Info
Header header

int64 imuAvailable        # IMU 是否可用
int64 odomAvailable       # 里程计是否可用

float32 imuRollInit       # IMU 姿态（fallback用）
float32 imuPitchInit
float32 imuYawInit

float32 odomX             # VINS 产生的里程计位姿
float32 odomY
float32 odomZ
float32 odomRoll
float32 odomPitch
float32 odomYaw

int64 odomResetId         # 里程计重置 ID（检测子系统重启）

sensor_msgs/PointCloud2 cloud_deskewed   # IMU去畸变后的原始点云
sensor_msgs/PointCloud2 cloud_corner     # 角特征点云
sensor_msgs/PointCloud2 cloud_surface    # 面特征点云
```

关键字段：
- `odomAvailable` + 6 自由度位姿：VIS 子系统发布的里程计估计，供 LIO 作为扫描匹配的初始猜测
- `odomResetId`：VIS 重置计数器，LIO 通过比对 `odomResetId` 检测 VIS 是否重启（`raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L821`）
- `imuAvailable`：IMU 数据可用性标志，决定是否启用 IMU 预积分初猜
- `imuRollInit/PitchInit/YawInit`：IMU 直接提供的姿态角，用作纯 IMU fallback

### 位姿初猜共享

LIO 的 `updateInitialGuess()` 中（`raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L801-L868`）：

1. **优先使用 VINS 位姿**：若 `odomAvailable == true` 且 `odomResetId` 一致，计算 VINS 增量变换作为 scan matching 初始猜测
2. **VINS 失效时 fallback 到 IMU**：仅使用 IMU 姿态增量（只有旋转）
3. **双重检测**：`lastVinsTransAvailable` 状态位 + `odomResetId` 对比，确保检测到 VINS 的任何故障

### LiDAR 辅助 VIS 初始化

VIS 子系统在初始化阶段可以直接利用 LiDAR 里程计提供的位姿，跳过视觉 SfM 初始化和 IMU 可观性检查（`raw/codes/LVI-SAM/src/visual_odometry/visual_estimator/estimator.cpp:L216-L271`）：

```cpp
// 检查窗口内 LiDAR 信息是否有效
for (int i = 0; i <= WINDOW_SIZE; i++) {
    if (all_image_frame[Headers[i].stamp.toSec()].reset_id < 0 || ...)
        lidar_info_available = false;
}
if (lidar_info_available == true) {
    // 直接用 LiDAR 位姿初始化所有状态
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        Ps[i] = all_image_frame[Headers[i].stamp.toSec()].T;
        Rs[i] = all_image_frame[Headers[i].stamp.toSec()].R;
        // ...
    }
}
```

### 回环检测协同

VIS 子系统运行基于词袋模型（DBoW）的视觉回环检测（`raw/codes/LVI-SAM/src/visual_odometry/visual_loop/loop_detection.cpp:L56-L139`），检测到回环后通过 `/vins/loop/match_frame` topic 将匹配帧时间戳发给 LIO 子系统。LIO 收到后通过 ICP 验证并添加回环因子（`raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L482-L648`）。

## 因子图模型

LVI-SAM 的 LiDAR 因子图（iSAM2）包含以下误差项：

$$
\arg\min_{\mathbf{X}} \sum_k \underbrace{\| \mathbf{r}_O(\mathbf{x}_{k-1}, \mathbf{x}_k) \|^2_{\Sigma_O}}_{\text{里程计约束}} + \sum_l \underbrace{\| \mathbf{r}_L(\mathbf{x}_a, \mathbf{x}_b) \|^2_{\Sigma_L}}_{\text{回环约束}} + \sum_g \underbrace{\| \mathbf{r}_G(\mathbf{x}_k, \mathbf{p}_{\text{gps}}) \|^2_{\Sigma_G}}_{\text{GPS约束（可选）}}
$$

其中 $\mathbf{x}_k = [\mathbf{R}_k, \mathbf{t}_k] \in SE(3)$ 是关键帧位姿，里程计因子由 scan-to-map 配准的 Hessian 矩阵决定协方差，回环因子由 ICP fitness score 决定噪声权重。

VIS 子系统的滑动窗口优化同时优化位姿、速度、偏置和特征深度的重投影误差 + IMU 预积分误差，采用 VINS-Mono 的标准边缘化策略保留先验信息。

## 并行运行与同步

两个子系统通过 ROS 消息异步通信，不依赖严格的时钟同步：

- IMU 数据同时分发给两个子系统（`raw/codes/LVI-SAM/src/lidar_odometry/imuPreintegration.cpp:L78-L82`）
- 特征提取结果（点云 + 视觉特征）独立处理，由 IMU 完成时间对齐和去畸变
- TF 广播：LIO 子系统发布 `odom → base_link` 变换，VIS 子系统发布 `world → camera` 变换

## Agent 实现提示

### 适用场景
- 需要 LiDAR+视觉互补的室外 SLAM 系统（城市、园区、越野）
- 单个传感器可能退化时（隧道 — 视觉失效、开阔地 — LiDAR 退化）
- 需要有 GPS 融合能力的场合

### 输入输出契约
- **输入**：LiDAR 点云（Velodyne VLP-16 等）、单目/双目图像、IMU（200Hz+）、GPS（可选）
- **输出**：6-DoF 全局位姿、关键帧轨迹、全局点云地图、位姿协方差
- **坐标**：IMU 坐标系为 body frame，LiDAR/camera 通过外参变换（`extRot/extTrans`）

### 实现骨架（伪代码）
```pseudo
function LVI_SAM_pipeline(lidar_scan, image, imu_data):
    // 并行特征提取
    lidar_features = extract_corner_surface(lidar_scan)
    visual_features = KLT_track(image, prev_image)

    // LIO 子系统
    cloud_info = pack(lidar_features, imu_attitude, vis_odom)
    lidar_pose = scan_to_map_optimization(cloud_info)

    // VIS 子系统
    if lidar_initialized:
        vis_pose = VINS_process(image, imu_data, lidar_pose)
    else:
        vis_pose = VINS_process(image, imu_data)

    // 互检：健康监测
    if lidar_degraded:
        lidar_init_guess = vis_pose  // 视觉辅助
    if vis_failed:
        reboot_VINS()

    return fused_pose, diagnostics
```

### 关键源码片段

**位姿共享 — updateInitialGuess（VINS → LIO 初猜）**

`raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L818-L851`

```cpp
// 使用 VINS 位姿作为 LiDAR 扫描匹配的初值
if (cloudInfo.odomAvailable == true && cloudInfo.odomResetId == odomResetId) {
    if (lastVinsTransAvailable == false) {
        lastVinsTransformation = pcl::getTransformation(
            cloudInfo.odomX, cloudInfo.odomY, cloudInfo.odomZ,
            cloudInfo.odomRoll, cloudInfo.odomPitch, cloudInfo.odomYaw);
        lastVinsTransAvailable = true;
    } else {
        Eigen::Affine3f transIncre = lastVinsTransformation.inverse() * transBack;
        Eigen::Affine3f transFinal = transTobe * transIncre;
        // ... 更新 transformTobeMapped
    }
} else {
    // VINS 失效：fallback 到纯 IMU 初猜
    lastVinsTransAvailable = false;
    odomResetId = cloudInfo.odomResetId;
}
```

**IMU 预积分 + 因子图构建（LIO 子系统）**

`raw/codes/LVI-SAM/src/lidar_odometry/imuPreintegration.cpp:L229-L276`

```cpp
// 预积分 IMU 测量值
while (!imuQueOpt.empty()) {
    imuIntegratorOpt_->integrateMeasurement(acc, gyr, dt);
}
// 构造 IMU 因子
gtsam::ImuFactor imu_factor(X(key-1), V(key-1), X(key), V(key), B(key-1), preint_imu);
graphFactors.add(imu_factor);  // 预积分因子
graphFactors.add(BetweenFactor(B(key-1), B(key), ...));  // 偏置演变因子
graphFactors.add(PriorFactor(Pose3(X(key), curPose, ...)));  // 位姿因子（来自 LiDAR）
optimizer.update(graphFactors, graphValues);
```

**iSAM 增量优化（saveKeyFramesAndFactor）**

`raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp:L1483-L1533`

### 实现注意事项
- `odomResetId` 用于检测 VIS 重启；LIO 侧收到新 reset ID 后需要重置 IMU 预积分参数
- LiDAR 退化时 VINS 位姿增量作为初猜可以保持跟踪，但仅当 VINS 本身健康的条件下有效
- 两个子系统的频率不同：LIO 约 5-10Hz（关键帧），VIS 约 10-20Hz（图像帧）；需处理好异步数据的时间戳对齐
- IMU 数据同时分发给两条 `deque`（`imuQueOpt` 和 `imuQueImu`），分别用于优化线程和里程计发布线程
- 特征数量阈值（`edgeFeatureMinValidNum=10`，`surfFeatureMinValidNum=100`）退化检测在 `scan2MapOptimization` 中（`mapOptmization.cpp:L1278`）

### 源码检索锚点
- `raw/codes/LVI-SAM/src/lidar_odometry/mapOptmization.cpp` — LIO 核心（因子图、扫描匹配、回环）
- `raw/codes/LVI-SAM/src/lidar_odometry/imuPreintegration.cpp` — IMU 预积分 + 因子图优化
- `raw/codes/LVI-SAM/src/visual_odometry/visual_estimator/estimator.cpp` — VIS 滑动窗口优化
- `raw/codes/LVI-SAM/src/visual_odometry/visual_loop/loop_detection.cpp` — 视觉回环检测
- `raw/codes/LVI-SAM/src/visual_odometry/visual_feature/feature_tracker.cpp` — KLT 光流跟踪
- `raw/codes/LVI-SAM/msg/cloud_info.msg` — 子系统间通信消息定义

## 相关页面

- [[算法-LIO-SAM]]
- VINS-Mono
- [[方法-多传感器冗余切换]]
- [[方法-6-DoF 退化检测]]
- [[概念-因子图]]
- [[概念-IMU预积分]]
- [[架构-多传感器融合架构]]
