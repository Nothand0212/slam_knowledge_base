# LiDAR 数据管线横向对比

> 基于 15 个 LiDAR SLAM 项目的深度源码分析，横跨经典、现代、多传感器、神经与工程五大类别。
> 生成日期：2026-04-29

---

## 1. 概述

### 1.1 LiDAR 类型与算法分类

| 项目 | 类别 | LiDAR 类型 | 传感器需求 | 运行模式 |
|------|------|-----------|-----------|---------|
| **LeGO-LOAM** | Classic | 机械旋转 (VLP-16) | LiDAR + 可选IMU | 在线 |
| **LIO-SAM** | Classic | 机械旋转 (Velodyne/Ouster) + Livox固态 | LiDAR + IMU + 可选GPS | 在线 |
| **Cartographer** | Classic | 机械旋转 2D/3D | LiDAR + 可选IMU/Odometry/GPS | 在线 |
| **FAST-LIO-SAM** | Modern | 依赖FAST-LIO2前端 | LiDAR + IMU (外部) | 在线 |
| **fusions_slam** | Modern | 机械旋转 (Velodyne/Ouster) | LiDAR + IMU + 可选RTK | 在线 |
| **KISS-ICP** | Modern | 任意带 x/y/z 的LiDAR | **LiDAR-only** | 在线 |
| **CT-ICP** | Modern | 机械旋转式 (需逐点时间戳) | **LiDAR-only** | 在线 |
| **ROLO-SLAM** | Modern | 机械旋转 (Velodyne/Ouster) | LiDAR + 可选里程计 | 在线 |
| **R3LIVE** | Multi-sensor | Livox 固态 | LiDAR + IMU + 全局快门相机 | 在线 |
| **FAST-LIVO2** | Multi-sensor | 7种 LiDAR (Avia/Velo/Ouster/L515/XT32/Pandar/Robosense) | LiDAR + IMU + 相机 | 在线 |
| **LVI-SAM** | Multi-sensor | 机械旋转 (Velodyne/Ouster) | LiDAR + IMU + 单目相机 | 在线 |
| **PIN-SLAM** | Neural | 任意 (Velodyne/Ouster等) | **LiDAR-only + GPU** | 在线 |
| **BEV-LSLAM** | Neural | 机械旋转 (VLP-16) | **LiDAR-only** | 在线 |
| **Lightning-LM** | Engineering | 4种 (Livox/Velodyne/Ouster/RoboSense) | LiDAR + IMU | 在线 + 离线 |
| **lt-mapper** | Engineering | 依赖外部前端 (SC-LIO-SAM等) | 外部前端生成的关键帧+SCD+PGO | **离线** |

### 1.2 算法核心分类

| 前/后端方法 | 代表项目 |
|------------|---------|
| **基于特征 (Feature-based) LM优化** | LeGO-LOAM, LIO-SAM, ROLO-SLAM, LVI-SAM |
| **直接法 (Direct) ICP/NDT** | KISS-ICP, CT-ICP, Cartographer |
| **卡尔曼滤波 (EKF/IESKF/ESKF)** | FAST-LIO2(前端), fusions_slam, R3LIVE, FAST-LIVO2, Lightning-LM |
| **因子图优化 (PGO)** | LIO-SAM, LVI-SAM, FAST-LIO-SAM, lt-mapper, ROLO-SLAM, Lightning-LM |
| **神经隐式表示** | PIN-SLAM (Neural Points + MLP) |
| **BEV投影 + 视觉特征** | BEV-LSLAM (ORB on BEV) |

---

## 2. 原始数据与标定

| 项目 | LiDAR 型号 | 线数 | 扫描频率 | 点格式 | 外参标定 | 时间戳需求 |
|------|-----------|------|---------|--------|---------|-----------|
| **LeGO-LOAM** | Velodyne VLP-16 | 16 | 10 Hz | `PointXYZIR` (ring通道) | `sensorMountAngle=0°` | 方位角推算相对时间 |
| **LIO-SAM** | Velodyne/Ouster/Livox | 16/32/64/128 | 10 Hz | `VelodynePointXYZIRT` / `OusterPointXYZIRT` | `extRot`/`extRPY` (IMU→LiDAR), YAML配置 | 需 `ring` + `time` 字段 |
| **Cartographer** | 任意 2D/3D LiDAR | 任意 | 5-40 Hz | `LaserScan` / `PointCloud2` / `MultiEchoLaserScan` | Lua配置文件 | 标准 ROS header.stamp |
| **FAST-LIO-SAM** | 由FAST-LIO2决定 | 由前端决定 | ~10 Hz | `PointXYZI` (世界系点云) | FAST-LIO2内部配置 | ApproximateTime同步odom+pcd |
| **fusions_slam** | Velodyne/Ouster | 64→32/128→32(降线束) | 10 Hz | `PointXYZIRT` (x,y,z,intensity,ring,offset_time) | `T_imu_lidar` + `T_imu_ant` (YAML) | 纳秒级 `uint64_t nsec` |
| **KISS-ICP** | 任意 (含 x/y/z) | 不限 | 可变 | `vector<Eigen::Vector3d>` | 无需求 | 搜索字段 `t`/`timestamp`/`time`/`time_stamp`，归一化到[0,1] |
| **CT-ICP** | 机械旋转式 | 不限 | 10 Hz | 每点带归一化时间戳 `t∈[0,1]` | 无需求 | 必需逐点时间戳 |
| **ROLO-SLAM** | Velodyne/Ouster | 16/32/64 | 10 Hz | `VelodynePointXYZIRT` (ring+time) | 无 (纯LiDAR) | 支持 ring + time 字段 |
| **R3LIVE** | Livox Avia | 非重复扫描 | 10 Hz | `livox_ros_driver::CustomMsg` | LiDAR-IMU预标定 | `g_camera_lidar_queue` 三传感器同步 |
| **FAST-LIVO2** | 7种LiDAR | Avia/Velo16/Oust64/L515/XT32/Pandar128/Robo | 10 Hz | `CustomMsg` / `PointCloud2` | LiDAR-IMU外参预标定 + 相机内参/外参预标定 | `sync_packages()` 以图像曝光中点为切割轴 |
| **LVI-SAM** | Velodyne/Ouster | 16/32/64 | 10 Hz | `PointCloud2` | LiDAR-IMU外参预标定 (`lidar2Imu`) | 以LiDAR `header.stamp` 为准 |
| **PIN-SLAM** | Velodyne/Ouster等 | 不限 | 10 Hz | ROS `PointCloud2` → `torch.Tensor(N,3)` | 无 (世界系=首帧LiDAR系) | 以 LiDAR 帧时间戳为准 |
| **BEV-LSLAM** | Velodyne VLP-16 (默认) | 16 | 10 Hz | `PointCloud2` | 无 (纯LiDAR)，BEV投影参数: `image_resolution=0.4m` | `systemDelay=10` 帧跳过 |
| **Lightning-LM** | Livox/Velodyne/Ouster/RoboSense | 32/64/Avia | 10 Hz | `PointXYZIT` (x,y,z,intensity,time) | LiDAR-IMU外参预标定 + 可选在线估计 | IMU最新时间戳需 > `lidar_end_time_` |
| **lt-mapper** | 外部前端决定 | 外部决定 | 离线 | PCD `PointXYZI` | 由前端SLAM完成 | 离线文件对齐 (无实时同步) |

---

## 3. 运动畸变矫正 (Deskewing) 对比

这是 LiDAR 管线最重要的步骤。机械旋转式 LiDAR 完成 360° 扫描需约 100ms，期间传感器运动导致首尾点空间位置不一致——在 10 m/s 速度下可达 1m 偏差。

### 3.1 方法总览

| 项目 | 方法 | 是否需要IMU | 具体实现 (源文件:行号) | 校正目标时刻 | 精度评估 |
|------|------|-----------|----------------------|------------|---------|
| **LeGO-LOAM** | IMU 角速度纯旋转去畸变 | 是 (可选) | `featureAssociation.cpp:491-619` `adjustDistortion()`：在IMU缓冲区查找对应时间戳位姿，仅旋转补偿，将所有点统一到 scan-start 时刻 | 帧起始 | 中 (依赖IMU质量，无平移补偿) |
| **LIO-SAM** | IMU 角速度积分去畸变 | 是 | `imageProjection.cpp:286-519`：零阶保持+前向欧拉积分 (`imuRotX = imuRotX + angular_x * dt`)，`findRotation()` 线性插值，`transStartInverse` 将所有点校正到帧起始 | 帧起始 | 高 (角速度积分精度高，但平移补偿被注释) |
| **Cartographer** | 位姿外推器 (匀速/IMU) | 可选 | `pose_extrapolator` (Lua配置): 支持 `constant_velocity` (基于历史位姿线性外推) 或 `imu_based` (IMU高频外推)，配合 `ImuTracker` 估计重力方向 | 当前帧 | 中高 (IMU模式下更优) |
| **FAST-LIO-SAM** | **由FAST-LIO2完成** (反向传播) | 是 (外部) | FAST-LIO2 IESKF反向传播：前向积分记录IMU位姿轨迹，反向遍历将每个点从当前时刻补偿到帧末 | 帧末尾 | 很高 (IESKF紧耦合，18维状态) |
| **fusions_slam** | IMU 反向传播 (Backward Propagation) | 是 | `propagate.cpp:33-123` `Propagate::run()`：步骤1—IMU前向积分 (IESKF predict)；步骤2—反向补偿 `R_i = head.rot * so3Exp(tail.angvel*dt)`, `T_ei = head.pos + head.vel*dt + 0.5*tail.acc*dt²`，用 `state.rotation.conjugate() * (R_i*P_i + T_ei)` 补偿 | 帧末尾 (IMU系) | 很高 (经典FAST-LIO方法) |
| **KISS-ICP** | **恒速模型 (纯LiDAR)** | **否** | `Preprocessing.cpp:55-95`：`omega = relative_motion.log()` (上一帧delta作角速度)，`pose = SE3::exp((stamp-1.0)*omega)`，匀速运动假设 | 帧末尾 | 中 (低速平稳时好，剧烈旋转时退化) |
| **CT-ICP** | **连续时间轨迹 (CT)** / 恒速 / 迭代 / 无补偿 | **否** | `cost_functions.h:68-98` `CTFunctor`：12自由度参数化 (begin_pose + end_pose)，`平移=线性插值`，`旋转=slerp`，直接建模帧内畸变而非预处理补偿。支持4种模式：NONE/CONSTANT_VELOCITY/ITERATIVE/CONTINUOUS | 逐点连续时间 | 最高 (原生建模，高速运动保持精度) |
| **ROLO-SLAM** | 里程计帧间变换去畸变 | 可选 | `imageProjection.cpp:266-396`：无逐点时间戳时 `relTime=(ori-startOri)/orientationDiff`；有逐点时间戳时直接用 `time` 字段。`deskewPoint()` 匀速模型插值，仅补偿旋转，平移被注释 | 帧起始 | 中 (仅旋转补偿) |
| **R3LIVE** | FAST-LIO2 前后向传播 | 是 | `r3live_lio.cpp:493-1081`：`p_imu->Process()` 执行前向+后向去畸变 (继承FAST-LIO2)，补偿到帧末 | 帧末尾 | 很高 |
| **FAST-LIVO2** | IMU 后向传播 | 是 | `IMU_Processing.cpp:237-541` `UndistortPcl()`：前向传播记录位姿到 `IMUpose` 向量；后向传播 `R_i = R_imu*Exp(angvel,dt)`, `P_compensate = extR_Ri*(R_i*(extR*P+extT) + T_ei) - extR_extT` | 帧末尾 | 很高 |
| **LVI-SAM** | IMU 旋转插值去畸变 | 是 | `imageProjection.cpp` `deskewInfo()`：利用IMU旋转插值，无逐点时间戳时方位角推算相对时间 | 帧起始 | 中高 (与LIO-SAM类似) |
| **PIN-SLAM** | **无传统去畸变** (Neural Points建模) | **否** | `tracker.py:367-611`：SDF隐式匹配框架天然处理帧内畸变—每个点的查询在连续SDF空间中独立进行，不需预处理补偿 | 逐点独立 | 中高 (神经表示本身具有连续性) |
| **BEV-LSLAM** | **slerp 插值去畸变** | 否 (ORB特征不需要) | `scantoscan_kitti.cpp:1096-1124` `adjustDistortion()`：通过 slerp 插值校正 scan 内运动畸变 | 帧起始 | 中 (BEV投影丢失垂直信息) |
| **Lightning-LM** | **IMU 前向ESKF + 后向补偿** | 是 | `imu_processing.hpp:173-313` `UndistortPcl()`：前向传播在每个IMU测量处递推ESKF；后向补偿从最后IMU pose往前插值，`p_compensate = offset_R_lidar⁻¹ * (rot⁻¹ * (R_i*(offset_R_lidar*P_i+offset_t_lidar) + T_ei) - offset_t_lidar)` | 帧末尾 | 很高 (23维状态，SO3+S2流形) |
| **lt-mapper** | **无** (输入已完成去畸变) | 外部完成 | 离线处理假设前端SLAM已完成去畸变 | (由前端决定) | 取决于前端 |

### 3.2 Deskewing 方法深度对比

```
                        精度
                         ↑
               CT-ICP (连续时间, 12DoF) ───── 最高
               FAST-LIO2/FAST-LIVO2 (反向传播,18/19维) ─┐
               fusions_slam (反向传播, 18维) ────────────┤ 很高
               Lightning-LM (前向ESKF+后向补偿,24维) ────┘
               R3LIVE (前后向, 18维) ──────────────────
               LIO-SAM (角速度积分, 仅旋转) ──┐
               LVI-SAM (IMU旋转插值) ─────────┤ 中高
               Cartographer (IMU外推) ────────┘
               LeGO-LOAM (IMU旋转补偿) ──┐
               ROLO-SLAM (里程计旋转补偿) ─┤ 中
               KISS-ICP (恒速模型) ────────┘
                                                  → 传感器依赖
               LiDAR-only ←──────────────────→ IMU-baed
```

**核心方法对比：**

| 方法 | 数学本质 | 优势 | 劣势 | 典型精度 @10m/s |
|------|---------|------|------|----------------|
| **IMU纯旋转** | `R(t) = Integrate(gyro, t)` | 简单、快速 | 忽略平移畸变 | ~5-10 cm |
| **IMU反向传播** | 前向积分轨迹 + 反向逐点补偿 `X_i = f(X_end, -dt_i)` | 利用全IMU信息 | 依赖IMU- LiDAR外参 | ~1-3 cm |
| **恒速模型** | `T(t) = T(t₀) * exp(t*ω)` | 零传感器依赖 | 变速时退化 | ~5-15 cm |
| **连续时间 (CT-ICP)** | 建模为12DoF轨迹，直接优化 | 无近似，最优雅 | 2x计算量 | ~2-5 cm |
| **神经隐式 (PIN-SLAM)** | SDF连续空间查询 | 无预处理 | GPU依赖 | ~3-8 cm |

---

## 4. 点云预处理

| 项目 | 降采样方法 | 体素大小 | 距离裁剪 | 地面分割 | 其他滤波 |
|------|-----------|---------|---------|---------|---------|
| **LeGO-LOAM** | 地面点每5取1 | N/A | `lidarMinRange~lidarMaxRange` | **有** `groundRemoval()` 相邻线束垂直角判据 | BFS聚类分割 (角度阈值60°) |
| **LIO-SAM** | 行降采样 `rowIdn % downsampleRate != 0` | `odometrySurfLeafSize=0.2m` (仅平面点体素) | `lidarMinRange~lidarMaxRange` | **无** (不区分地面/非地面) | Range Image去重 (同像素保留首点) |
| **Cartographer** | `voxel_filter_size=0.025m` (2.5cm) + 自适应体素滤波 | 0.025m (2D) | `min_range=0.0`, `max_range=30.0` | **无** | Motion Filter (位移>0.2m 或 旋转>1°才处理) |
| **FAST-LIO-SAM** | `VoxelGrid(0.3m)` 回环匹配前降采样 | 0.3m | 由FAST-LIO2决定 | **无** | 无额外预处理 |
| **fusions_slam** | `VoxelGrid(0.2m)` + 线束降采样(64→32, 128→32) | 0.2m | 车身裁剪 `lidarXYBox` | **无** | NaN点滤除 |
| **KISS-ICP** | **双层体素降采样**：frame_downsample (0.5×voxel) + source (1.5×voxel) | `voxel_size=1.0m` | `min_range=0.0`, `max_range=100.0` | **无** | one-per-voxel 保留首点 |
| **CT-ICP** | `RobustRegistration` 自适应采样体素大小 | 可变 | 地图 `max_distance=100m` | **无** | 大旋转帧拒绝插入，连续失败>5强制插入 |
| **ROLO-SLAM** | `rowIdn % downsampleRate != 0` 行降采样 | `odometrySurfLeafSize=0.2m` (平面点) | `lidarMinRange < range < lidarMaxRange` | **无** (但提取地面点作为Normal特征) | Range Image去重 |
| **R3LIVE** | 外部节点特征下采样 (`feats_down`) | (前端决定) | (前端决定) | **无** | `lasermap_fov_segment()` FOV内外分割 |
| **FAST-LIVO2** | `VoxelMap` 自适应八叉树 | `voxel_size` 可配 | (传感器级过滤) | **无** | LiDAR投影构建 `depth_img` 辅助视觉深度验证 |
| **LVI-SAM** | 行降采样 + 体素滤波 | `odometrySurfLeafSize=0.2m` | `lidarMinRange~lidarMaxRange` | **有** (通过IMU pitch判断分离地面/非地面) | Range Image 投影 |
| **PIN-SLAM** | 体素降采样 `source_vox_down_m=0.8m` | 0.8m | 数据集loader中NaN/范围过滤 | **无** | `data_pool` 滑动窗口累积 |
| **BEV-LSLAM** | BEV投影本身就是降采样 (3D→200×200像素) | `image_resolution=0.4m/pixel` | `MIN=1.0m`, `MAX=150m` | **隐式** (地面共面增强降亮度) | 直方图均衡化 + 高斯模糊 |
| **Lightning-LM** | `VoxelGrid(0.5m)` + 自适应保护(点数<10%则退化到0.1m) | 0.5m | 盲区过滤 `blind_` + Ouster ROI高度过滤 | **无** | `point_filter_num_` 均匀采样 |
| **lt-mapper** | `downSizeFilterICP` VoxelGrid | 由前端配置决定 | 由前端SLAM完成 | 由前端SLAM完成 | 离线假设已完成预处理 |

---

## 5. 特征提取对比

| 项目 | 特征类型 | 提取方法 | 曲率公式 | 网格划分 | 特征数量/帧 |
|------|---------|---------|---------|---------|-----------|
| **LeGO-LOAM** | **Edge + Planar** (按地面语义分类) | LOAM平滑度 + 地面引导：边缘仅来自**非地面**点，平面仅来自**地面**点 | `c = (Σr_{i±5} - 10·r_i)²` | 6等分子区域 | 边缘2+2×6=24 sharp corner + ~120 less sharp；平面4×6=24 |
| **LIO-SAM** | **Corner + Surface** (曲率分类) | LOAM曲率 + 6等分区域独立选取 | `c = (Σr_{i±5} - 10·r_i)²` | 6等分子区域 | Corner: ≤20/子区域 (max 120); Surface: 经0.2m体素后不等 |
| **Cartographer** | **无显式特征** | 占用概率网格 + 双三次插值代价 | N/A | N/A | 全部点参与代价评估 |
| **FAST-LIO-SAM** | **无特征提取** | 由FAST-LIO2前端完成 (点到平面/边原始点匹配) | N/A | N/A | (前端决定) |
| **fusions_slam** | **隐式平面** (无显式分类) | 最近邻5点 `planarCheck()` QR求解平面方程 | N/A | N/A | 通过平面验证的有效点数 |
| **KISS-ICP** | **无特征** | 全点云参与，GM核隐式实现特征选择 | N/A | N/A | source降采样后的全部点 |
| **CT-ICP** | **无显式特征** | 邻域PCA提供 `a2D` (planarity) + `covariance` | N/A | N/A | `num_keypoints` 降采样后全部点 |
| **ROLO-SLAM** | **Corner + Surface + Normal(Ground)** (3类) | LOAM平滑度 + 6扇区法 | `c = (Σr_{i±5} - 10·r_i)²` | 6扇区/线 | Corner: 20×6×16 ≈ 1920; Surface: 体素后不等 |
| **R3LIVE** | **消费外部特征** (`feats_down`) | 外部节点提取，R3LIVE不自己提取 | (外部决定) | (外部决定) | (外部决定) |
| **FAST-LIVO2** | **LiDAR: 六类** `Poss_Plane/Real_Plane/Edge_Jump/Edge_Plane/Wire/ZeroPoint`; **Visual: 直接法patch** | LiDAR: `plane_judge()` + `edge_jump_judge()` 角度变化判据；Visual: Shi-Tomasi 评分的LiDAR点 | N/A | 图像网格 `grid_n_width×grid_n_height` | LiDAR: 降采样后有效点数; Visual: 每网格1个 |
| **LVI-SAM** | **Corner + Surface** | LOAM平滑度 + 区分地面/非地面 | `c = (Σr_{i±5} - 10·r_i)²` | 6扇区/线 | `edgeThreshold` 个角点 + `surfThreshold` 个平面点 |
| **PIN-SLAM** | **SDF隐式特征** | Neural Points (3D位置+8维特征向量+方向四元数+置信度) 编码局部几何 | N/A | N/A | 动态增长，哈希表 `buffer_size=5e7` |
| **BEV-LSLAM** | **ORB 特征** (FAST角点+BRIEF描述子) | BEV投影后对高度图+强度图分别提取ORB | N/A | 无显式网格 | 1500个/图 (双通道各1500) |
| **Lightning-LM** | **统一在线平面拟合** (不使用edge/planar分类) | `esti_plane()` 5近邻QR求解平面方程，检查所有点到平面距离<0.1m | N/A | N/A | 有效特征点数 (NCLT 32线约1000-3000个) |
| **lt-mapper** | **无** (消费前端特征) | 由前端SLAM提取并保存为关键帧点云 | 由前端决定 | 由前端决定 | 由前端决定 |

### 5.1 LOAM 曲率公式

三个项目使用相同的 LOAM 曲率定义：

```
c_i = ( Σ_{j=i-5}^{i+5, j≠i} r_j  -  10 · r_i )²

其中 r_j 是第 j 个点的 range 值
```

这是**不是几何曲率**，而是 range 值的二阶差分——测量扫描线上局部表面的不平坦度。高曲率=边缘特征，低曲率=平面特征。

使用 LOAM 曲率的项目：LeGO-LOAM, LIO-SAM, ROLO-SLAM, LVI-SAM

### 5.2 特征提取策略对比

```
显式特征 (LOAM风格)                 隐式/无特征
    │                                    │
    ├─ LeGO-LOAM: 地面语义引导分类       ├─ fusions_slam: 在线平面拟合
    ├─ LIO-SAM: 纯曲率分类               ├─ KISS-ICP: GM核自动选择
    ├─ ROLO-SLAM: 三类特征               ├─ CT-ICP: PCA平面性加权
    ├─ LVI-SAM: 曲率+地面分离            ├─ Lightning-LM: 统一平面拟合
    └─ BEV-LSLAM: ORB on BEV             ├─ FAST-LIVO2: LiDAR 6类+视觉patch
                                          └─ PIN-SLAM: 神经隐式
```

**关键趋势**：现代方法倾向于放弃显式特征分类，转而使用在线几何上下文（平面拟合、GM核自适应权重、神经表示）。

---

## 6. 配准方法对比

| 项目 | 配准类型 | 求解器 | 迭代次数 | 鲁棒核函数 | 退化检测 | 初始猜测 |
|------|---------|--------|---------|-----------|---------|---------|
| **LeGO-LOAM** | **两步解耦优化**：Step1 地面→平面优化(roll,pitch,ty)，Step2 边缘→(yaw,tx,tz) | `cv::solve` QR分解 (3×3 矩阵) | 25+25 | 无显式核 | SVD `matP` 投影矩阵 | IMU roll/pitch |
| **LIO-SAM** | **Scan-to-Map LM** | LOAM自定义LM + 链式法则解析 Jacobian | 30 | `s=1-0.9*|pd|/sqrt(||p||)` (类Huber) | `J^T J` 特征值<100→投影修正 | IMU预积分/GPS |
| **Cartographer** | **两步匹配**：实时相关匹配(CSM) + Ceres Scan Matcher | Ceres非线性优化 | (Cerer内部) | `huber_scale=1e1` | 无显式 | 匀速模型/IMU外推 |
| **FAST-LIO-SAM** | **Scan-to-Map (FAST-LIO2 IESKF)** | IESKF迭代更新 | (FAST-LIO2内部) | (FAST-LIO2内部) | (FAST-LIO2内部) | FAST-LIO2 odometry |
| **fusions_slam** | **IESKF Iterated Update** (点到平面) 10次迭代 | 自实现IESKF + 卡尔曼增益 | 10 | 无显式 (Huber启发式 `s=1-0.9*|pd|`) | 无显式 | IMU递推 |
| **KISS-ICP** | **Gauss-Newton ICP** (点到点+GM核) | `JTJ.ldlt().solve(-JTr)` | **500** | **Geman-McClure** `w = σ²/(σ²+r²)²` | 无显式 | 恒速模型 `last_pose * last_delta` |
| **CT-ICP** | **Ceres CT-ICP** (点到平面/点/线/分布 12DoF参数化) | Ceres (GN/CERES/ROBUST三种) + OpenMP多线程 | (Ceres内部) | **Cauchy/Huber/Truncated** 可选 | 运动模型 `IsValid()` 检查 | `NextFrame()` 预测 |
| **ROLO-SLAM** | **分离式**：先平移插值对齐→**RotVGICP**旋转估计→连续时间平移估计 | RotVGICP SO(3)高斯-牛顿 + Ceres | RotVGICP迭代 + 后端30次LM | GICP协方差加权 | Hessian特征值<100→`matP` 投影 | 后端位姿插值 (仅平移) |
| **R3LIVE** | **LIO IESKF** (点到平面) + **VIO ESIKF** (视觉重投影) | 自实现IESKF/ESIKF | `NUM_MAX_ITERATIONS` | `HuberLossScale` (VIO) | 无显式 | LIO传播→VIO继承 |
| **FAST-LIVO2** | **统一IESKF** (串行LIO→VIO) ：LIO VoxelMap点-平面→VIO直接法光度误差 | 自实现IESKF + OpenMP并行 | LIO: max_iter; VIO: max_iter × pyramid_levels | LIO: 协方差 `R_inv=1/(0.001+σ_l)`; VIO: 光度一致性 | 无显式 | IMU传播 `state_propagat` |
| **LVI-SAM** | **Scan-to-Map LM** + **VINS-Mono Ceres滑窗** (两个独立优化器) | LIS: GTSAM ISAM2 + 自定义LM；VIS: Ceres滑窗BA | LIS: 30次LM; VIS: Ceres内部 | LIS: `s=1-0.9*|pd|`; VIS: Huber | LIS: `matP` 投影矩阵 | VINS位姿→LIS初始猜测 |
| **PIN-SLAM** | **Scan-to-Implicit-Map LM** (点-SDF残差) | 自实现LM + 自动微分 | 50 | **多层Geman-McClure** `w_res*w_grad*w_normal*w_color` | Hessian最小特征值检查 | 匀速模型/上一帧 |
| **BEV-LSLAM** | **前端**: ORB匹配+Ceres (LidarICPFactor/EdgeFactor); **后端**: Fast-VGICP | Ceres (DENSE_QR/SOLVER) | 前端: 10; 后端VGICP: 25 | GMS几何验证 + 距离筛选 | 无显式 | 前端匹配→后端初值 |
| **Lightning-LM** | **IESKF + Cauchy核** (点到平面12维雅可比) | 自实现IESKF (+可选AA) | max_iter=4 | **Cauchy** `ρ=δ²*ln(1+x/δ²)`, δ=2.0 | 退化解检测 (收敛条件) | IMU前向递推 |
| **lt-mapper** | **PGO-only** (无扫描匹配) | GTSAM ISAM2 | 5x `update()` 确保收敛 | **Cauchy M-estimator** (跨会话回环) | 信息增益筛选 | 外部前端位姿 |

### 6.1 鲁棒核函数横向对比

| 核函数 | 公式 | 大残差行为 | 使用项目 |
|--------|------|-----------|---------|
| **Geman-McClure** | `w = σ²/(σ²+r²)²` | 激进截断 (r²>>σ²时 w→0) | KISS-ICP, PIN-SLAM |
| **Cauchy** | `ρ = δ²·ln(1+r²/δ²)` | 软截断 | CT-ICP, Lightning-LM, lt-mapper |
| **Huber-like (自适应)** | `s = 1 - 0.9·|pd|` | 软截止 | LIO-SAM, LeGO-LOAM, ROLO-SLAM, LVI-SAM |
| **Truncated L2** | `ρ = min(σ², r²)` | 硬截断 | CT-ICP |
| **Point-to-Distribution** | 马氏距离 | 协方差加权 | CT-ICP, ROLO-SLAM (GICP风格) |

---

## 7. 地图表示对比

| 项目 | 地图类型 | 数据结构 | 更新策略 | 窗口/范围管理 | 分辨率 |
|------|---------|---------|---------|-------------|--------|
| **LeGO-LOAM** | 边缘地图 + 平面地图 (分离存储) | `laserCloudCornerLast` / `laserCloudSurfLast` 点云 | 帧间替换 (前→后) | 固定上一帧 | N/A |
| **LIO-SAM** | 局部地图 (角点+平面点分离) | KD-Tree (`pcl::KdTreeFLANN`) | 半径搜索 + 体素缓存 (1000项限制) | 半径搜索 `surroundingKeyframeSearchRadius=50m` | N/A |
| **Cartographer** | **概率占据网格 (Occupancy Grid)** 2D; **混合网格 (Hybrid Grid)** 3D | 2D: `ProbabilityGrid` (log-odds, 16-bit存储); 3D: 高低分辨率混合网格 | 子地图内增量插入 (命中/穿透概率更新) | 子地图 90帧冻结; 全局多个子地图 | 0.05m (2D) |
| **FAST-LIO-SAM** | 世界坐标系点云 (关键帧存储) | `vector<Keyframe>` (LiDAR frame坐标) | VoxelGrid 0.3m 拼接降采样 | 全局地图按需重建 | 0.3m |
| **fusions_slam** | **iKD-Tree 增量地图** | iKD-Tree (FAST-LIO2增量kd树) + 体素滤波中点判断 | `mapIncremental()`: 检查体素中点距离决定是否添加 | FOV动态窗口 `MOV_THRESHOLD * DET_RANGE` 触发平移 | N/A |
| **KISS-ICP** | **VoxelHashMap** (哈希体素地图) | `tsl::robin_map<Voxel, vector<Vec3d>>` | `AddPoints()` 去重 (每voxel≤20点, 间距≥0.224m) | 滑动窗口 `max_distance=100m` 自动删除远点 | `voxel_size=1.0m`; `max_points_per_voxel=20` |
| **CT-ICP** | **VoxelHashMap** (类KISS-ICP) | 带邻域描述子的哈希体素 | 增量插入 + 大旋转帧拒绝 + 强制插入保护 | 远距离体素删除 | 可变体素 |
| **ROLO-SLAM** | 全局点云地图 (角点+平面点) | `laserCloudMapContainer` (简单容量限制>1000清空) | 关键帧拼接→降采样 | 周围关键帧半径搜索50m + 10s窗口 | N/A |
| **R3LIVE** | iKd-Tree + **RGB着色全局地图** | iKd-Tree增量kd树 + `Rgbmap_tracker` 多视图RGB观测 | 去畸变点云 `append_points_to_global_map()` | `lasermap_fov_segment()` FOV分割 | N/A |
| **FAST-LIVO2** | **VoxelMap** + 视觉稀疏地图 | 哈希→`VoxelOctoTree`八叉树 (叶节点`VoxelPlane`含6×6协方差) + `SubSparseMap` | 八叉树递归切割 PCA判定平面 | `mapSliding()` 局部窗口 | 八叉树自适应分辨率 |
| **LVI-SAM** | iKd-Tree + **VINS特征点地图** | LIS: iKd-Tree; VIS: 滑动窗口特征点 | 增量插入+降采样 | 周围关键帧半径搜索 | N/A |
| **PIN-SLAM** | **Neural Points + MLP** (点基隐式神经表示) | `NeuralPoints` (空间哈希 `buffer_size=5e7`) + `Decoder` (1层MLP, 64 hidden) | `mapper.process_frame()` 在线梯度优化 + `adjust_map()` 刚体变换 | 全局地图 (连续SDF) | **连续** (MLP插值, 无固定分辨率) |
| **BEV-LSLAM** | 高度点云 + 强度点云 (局部子图) | 关键帧按类型分离存储 | KDTree半径搜索 15m 拼接→降采样 | 半径 15m 局部子图 | `image_resolution=0.4m/pixel` |
| **Lightning-LM** | **IVox3d** (哈希voxel增量地图) | `std::unordered_map<KeyType, IVoxNode>` 容量1e6网格 LRU淘汰 | `MapIncremental()` 并行更新，voxel中心距离判据 | `capacity_=1,000,000` grids LRU | `ivox_grid_resolution=0.5m` |
| **lt-mapper** | 多会话融合点云 + Delta Map | Removert Range Image投影 | 离线批处理 + 变化检测 | 跨会话全局对齐 | 前端决定 |

### 7.1 地图表示演化趋势

```
传统离散            →        结构化稀疏            →        连续/神经
  │                             │                              │
  ├─ Point Cloud                 ├─ VoxelHashMap               ├─ PIN-SLAM (SDF)
  ├─ KD-Tree                     ├─ VoxelMap (八叉树)           └─ (未来方向)
  ├─ Occupancy Grid              ├─ iKD-Tree / IVox3d
  └─ 子地图                      └─ RGB着色地图
```

---

## 8. LiDAR 因子/观测构建

| 项目 | 因子类型 | 残差维度 | 后端求解器 | 噪声模型 | 其他因子 |
|------|---------|---------|-----------|---------|---------|
| **LeGO-LOAM** | **BetweenFactor<Pose3>** (里程计) + **PriorFactor<Pose3>** (首帧) | 6 | GTSAM ISAM2 | `(1e-6,1e-6,1e-6,1e-8,1e-8,1e-6)` | 回环 BetweenFactor (ICP验证) |
| **LIO-SAM** | **ImuFactor** + **PriorFactor<Pose3>** (LiDAR odom) + **BetweenFactor<ConstantBias>** (bias游走) | 15 | GTSAM ISAM2 | normal: `(0.05,0.05,0.05,0.1,0.1,0.1)`; degenerate: `(1,1,1,1,1,1)` | GPSFactor, 回环 BetweenFactor |
| **Cartographer** | **子地图内扫描匹配约束** + **回环约束** + **里程计约束** + **IMU约束** + **固定帧约束** | 3/6 | Ceres PGO | Rotation: 1.6e4, Translation: 1e5 (子地图) | 加速度(1.1e2)、旋转(1.6e4) |
| **FAST-LIO-SAM** | **PriorFactor<Pose3>** + **BetweenFactor<Pose3>** (odom + loop) | 6 | GTSAM ISAM2 | Odom: `(1e-4,1e-4,1e-4,1e-2,1e-2,1e-2)`; Loop: `ICP_score.repeated<6>()` | 回环后3x `update()` |
| **fusions_slam** | **IESKF 4种观测**：`lidarObserve` (点到平面) + `rotationObserve` + `positionObserve` + `velocityObserve` | 1/3 (按观测类型) | 自实现IESKF + Cere后端 `LioGpsOpt` | 各观测独立噪声 | Cere: LIO相对约束+GPS全局约束 |
| **KISS-ICP** | **无因子图** (纯前端odometry) | 点到点3D残差 | N/A (仅Gauss-Newton) | `pose.covariance = diag(0.1×6)` 固定 | 无后端 |
| **CT-ICP** | **无因子图** (纯前端odometry) | 点到平面1D/点到点3D/点到线1D/分布1D | N/A (仅Ceres ICP) | 运动模型约束权重 | 位置一致性+常速度+朝向一致性+小速度 |
| **ROLO-SLAM** | **PriorFactor<Pose3>** + **BetweenFactor<Pose3>** (odo+loop) | 6 | GTSAM ISAM2 | Prior: `(1e-2,1e-2,π²,1e8,1e8,1e8)` (地面假设) | 回环后5x `update()` |
| **R3LIVE** | **LIO ESIKF**: 点到平面雅可比 `Hsub = [point×R^n, n]`; **VIO ESIKF**: 重投影误差链式求导 (含外参/内参/时间偏移) | LIO: 6; VIO: 29+ | LIO IESKF + VIO ESIKF (双滤波器) | 初始协方差 `I*1e-5` (R,T) | LIO→VIO先验传递 |
| **FAST-LIVO2** | **统一IESKF (19维)**：LIO观测 `Hsub = [point_crossmat*R^T*n, n^T]`; VIO观测光度残差 `∂r/∂δθ = J_img*∂π*[Rci-Rci*skew(Pic)]` | LIO: 6; VIO: 6/7 | 自实现IESKF | `R_inv=1/(0.001+σ_l+n^T*var*n)` (LIO); `img_point_cov` (VIO) | 同一协方差矩阵串联更新 |
| **LVI-SAM** | **LIS**: ImuFactor + PriorFactor<Pose3>(LiDAR odom) + BetweenFactor(bias); **VIS**: IMU预积分 + 视觉重投影 + 边缘化先验 | LIS: 15; VIS: 滑动窗口 | GTSAM ISAM2 (LIS) + Ceres滑窗 (VIS) | LIS: normal/degenerate; VIS: Ceres自动 | 回环 BetweenFactor (VIS DBoW→LIS ICP验证) |
| **PIN-SLAM** | **自研PGO** (iSAM2或LM batch) + 里程计边 + 回环边 (协方差加权) | 6 | 自研PGO (g2o风格) | 配准协方差作为边的信息矩阵 | `adjust_map(pose_diff)` 刚体变换校正 Neural Points |
| **BEV-LSLAM** | **LidarICPFactor(3D)** + **LidarEdgeFactor(3D)** + **local_BA_Factor(3D)** + **FourDOFError(4D)** | 2/3/4 | Ceres AutoDiff | `keyframeAddingDistance=0.5m`, `keyframeAddingAngle=0.3rad` | 4DoF PGO (XY+yaw) |
| **Lightning-LM** | **IESKF CustomObservationModel** (点到平面12维雅可比) + **miao PGO** | 12 (位置3+旋转3+外参6) | 自实现IESKF + miao PGO (增量LM) | Motion: 0.1m/3°; Loop: 0.2m/3° | 高度先验EdgeHeightPrior(z=0, 0.1m) |
| **lt-mapper** | **BetweenFactorWithAnchoring<Pose3>** (4变量因子) + PriorFactor (anchor) + BetweenFactor (odo/loop) | 6 + anchor变换 | GTSAM ISAM2 | `priorNoise=1e-12` (center); `largeNoise=(π²,1e8)` (query) | Cauchy M-estimator (跨会话回环); 信息增益筛选 |

### 8.1 后端架构分类

```
滤波方法                              因子图/优化方法
    │                                      │
    ├─ IESKF (fusions_slam)                ├─ GTSAM ISAM2
    ├─ ESKF (Lightning-LM)                 │   ├─ 15DoF: LIO-SAM, LVI-SAM
    ├─ 双ESIKF (R3LIVE)                    │   ├─ 6DoF: LeGO-LOAM, FAST-LIO-SAM, ROLO-SLAM
    └─ 统一IESKF (FAST-LIVO2)              │   └─ Anchor PGO: lt-mapper
                                           ├─ Ceres PGO: Cartographer, BEV-LSLAM
                                           ├─ miao PGO: Lightning-LM
                                           └─ 自研PGO: PIN-SLAM
```

---

## 9. 设计模式总结

### 9.1 数据管线拓扑对比

```
┌──────────────────────────────────────────────────────────────────────┐
│                         LiDAR 数据管线全景                             │
│                                                                      │
│  [原始点云]                                                            │
│      │                                                               │
│      ├─→ [格式转换]  ROS→PCL / Eigen / Tensor                          │
│      │                                                               │
│      ├─→ [去畸变]                                                     │
│      │    ├── IMU反向传播:  fusions_slam, Lightning-LM, FAST-LIO2, R3LIVE, FAST-LIVO2
│      │    ├── IMU角速度积分: LIO-SAM, LeGO-LOAM, LVI-SAM
│      │    ├── 恒速模型:      KISS-ICP
│      │    ├── 连续时间:      CT-ICP
│      │    └── 无处理:        PIN-SLAM (神经隐式), lt-mapper (离线)
│      │                                                               │
│      ├─→ [降采样]                                                     │
│      │    ├── VoxelGrid:      Lightning-LM (0.5m), fusions_slam (0.2m)
│      │    ├── 双层Voxel:      KISS-ICP (0.5x+1.5x)
│      │    ├── BEV投影:        BEV-LSLAM (200×200)
│      │    └── Range Image:    LeGO-LOAM, LIO-SAM, ROLO-SLAM
│      │                                                               │
│      ├─→ [地面分割] (可选)                                             │
│      │    ├── 有: LeGO-LOAM (相邻线束角度), LVI-SAM (IMU pitch)
│      │    └── 无: 其余项目
│      │                                                               │
│      ├─→ [特征提取]                                                    │
│      │    ├── LOAM曲率+分类:   LeGO-LOAM, LIO-SAM, ROLO-SLAM, LVI-SAM
│      │    ├── 在线平面拟合:    fusions_slam, Lightning-LM
│      │    ├── 无显式特征:      KISS-ICP, CT-ICP, Cartographer
│      │    ├── 神经表示:        PIN-SLAM (SDF)
│      │    └── 视觉特征:        BEV-LSLAM (ORB), FAST-LIVO2 (直接法)
│      │                                                               │
│      ├─→ [配准/位姿估计]                                               │
│      │    ├── LM/GN (Scan-to-Map): LIO-SAM, LeGO-LOAM, ROLO-SLAM, LVI-SAM
│      │    ├── ICP+GM核:        KISS-ICP (点到点, 500次)
│      │    ├── CT-ICP (Ceres):  CT-ICP (点到面/点/线/分布, 12DoF)
│      │    ├── IESKF/ESKF:      FAST-LIVO2, R3LIVE, fusions_slam, Lightning-LM
│      │    ├── LM+AutoDiff:     PIN-SLAM (SDF残差)
│      │    └── 概率网格CSM:      Cartographer
│      │                                                               │
│      └─→ [算法消费]                                                    │
│           ├── 纯前端odometry:    KISS-ICP, CT-ICP (无后端)
│           ├── GTSAM ISAM2:      LIO-SAM, LeGO-LOAM, FAST-LIO-SAM, ROLO-SLAM, LVI-SAM, lt-mapper
│           ├── Ceres PGO:        Cartographer, BEV-LSLAM
│           ├── 滤波+自定义PGO:    fusions_slam, Lightning-LM (miao)
│           ├── 双滤波器:         R3LIVE
│           ├── 统一IESKF:        FAST-LIVO2
│           └── 神经表示+PGO:     PIN-SLAM
└──────────────────────────────────────────────────────────────────────┘
```

### 9.2 关键设计模式分类

| 设计模式 | 代表项目 | 核心思想 | 适用场景 |
|---------|---------|---------|---------|
| **两步解耦优化** | LeGO-LOAM | 地面约束 roll/pitch，边缘约束 yaw/translation，分离求解 | 地面车辆、结构化环境 |
| **分离式旋转/平移估计** | ROLO-SLAM | 先平移对齐再纯旋转匹配，连续时间约束平滑平移 | 不平坦地形、越野 |
| **反向传播去畸变** | fusions_slam, Lightning-LM, FAST-LIVO2 | 前向记录轨迹 + 反向逐点补偿，利用全IMU信息 | 高动态场景、需要高精度 |
| **自适应阈值** | KISS-ICP | `AdaptiveThreshold` 用模型预测RMS误差自动调参 | LiDAR-only、快速部署 |
| **连续时间轨迹** | CT-ICP | 将帧建模为轨迹而非单点，原生处理畸变 | 高速运动、高精度需求 |
| **统一IESKF紧耦合** | FAST-LIVO2 | 单一KF串联LIO→VIO，协方差自动关联不确定性 | 多传感器融合里程计 |
| **Anchor节点PGO** | lt-mapper | 用anchor节点将多会话对齐建模为PGO | 多机器人、多会话建图 |
| **GTSAM增量优化** | LIO-SAM, FAST-LIO-SAM, lt-mapper | ISAM2增量式因子图，支持在线回环 | 长时间运行、回环场景 |
| **松耦合消息驱动** | LVI-SAM | 多ROS节点通过topic通信，独立优化器相互参考 | 模块化、可维护性优先 |
| **神经隐式地图** | PIN-SLAM | 连续SDF + 可微地图 = 统一tracking/mapping/回环框架 | GPU可用、需要连续表示 |
| **BEV降维+视觉特征** | BEV-LSLAM | 3D→2D投影后用成熟视觉SLAM管线 | CPU-only LiDAR、室外场景 |

### 9.3 技术路线选择建议

| 需求 | 推荐技术路线 | 推荐项目参考 |
|------|------------|------------|
| **极简部署 (LiDAR-only, 无IMU)** | 双层体素 + GM核ICP + 自适应阈值 | KISS-ICP |
| **追求精度 (LiDAR-only)** | 连续时间CT-ICP (12DoF) | CT-ICP |
| **LIO紧耦合 (高动态)** | IESKF + 反向传播去畸变 | fusions_slam, Lightning-LM |
| **多传感器融合 (LIV)** | 统一IESKF + 直接法视觉 | FAST-LIVO2 |
| **模块化系统 (含回环)** | GTSAM ISAM2 + 松耦合消息桥接 | LVI-SAM |
| **连续/可微地图** | Neural Points + MLP + 可微LM | PIN-SLAM |
| **CPU-only 高效系统** | VoxelMap/VoxelHashMap + 在线平面拟合 | FAST-LIVO2 (LO模式), Lightning-LM |
| **多会话/多机器人** | Anchor节点PGO + SC+RS回环 | lt-mapper |
| **RGB着色+重建** | 双ESIKF + 多视图RGB观测 + MVS | R3LIVE |
| **BEV降维+视觉特征匹配** | 3D→BEV投影 + ORB + Ceres优化 | BEV-LSLAM |

### 9.4 关键文件索引

| 项目 | 去畸变文件 | 特征提取文件 | 配准/匹配文件 | 后端/PGO文件 |
|------|-----------|------------|------------|-------------|
| **LeGO-LOAM** | `featureAssociation.cpp:491-619` | `featureAssociation.cpp:621-783` | `featureAssociation.cpp:1270-1478` | `mapOptmization.cpp:1355-1527` |
| **LIO-SAM** | `imageProjection.cpp:286-519` | `featureExtraction.cpp:81-237` | `mapOptmization.cpp:974-1310` | `imuPreintegration.cpp:350-380` + `mapOptmization.cpp:1354-1614` |
| **Cartographer** | `pose_extrapolator` (Lua) | N/A | `real_time_correlative_scan_matcher_2d.h` + `ceres_scan_matcher_2d.h` | `pose_graph_2d.cc:126-150` + `optimization_problem_2d.h` |
| **FAST-LIO-SAM** | (FAST-LIO2, 外部) | (FAST-LIO2, 外部) | (FAST-LIO2, 外部) | `fast_lio_sam_sc_qn.cpp:113-238` |
| **fusions_slam** | `propagate.cpp:33-123` | `ieskf.cpp:207-271` (隐式平面拟合) | `ieskf.cpp:107-205` | `lio_gps_opt.cpp:51-160` |
| **KISS-ICP** | `Preprocessing.cpp:55-95` | N/A (GM核隐式) | `Registration.cpp:138-167` | N/A (纯前端) |
| **CT-ICP** | `cost_functions.h:68-98` (CTFunctor) | N/A (PCA邻域) | `ct_icp.cpp:559-603` (Ceres多线程) | N/A (纯前端) |
| **ROLO-SLAM** | `imageProjection.cpp:266-396` | `featureExtraction.cpp:87-265` | `lidarOdometry.cpp:331-383` (RotVGICP) | `backMapping.cpp:932-1021` (GTSAM ISAM2) |
| **R3LIVE** | `r3live_lio.cpp:493-1081` (p_imu->Process) | 外部节点 | `r3live_lio.cpp:661-898` (LIO ESIKF) | `r3live_vio.cpp:607-772` (VIO ESIKF) |
| **FAST-LIVO2** | `IMU_Processing.cpp:237-541` | `preprocess.h:24-34` (7种LiDAR特征) | `voxel_map.cpp:414-498` + `vio.cpp:1520-1687` | 无独立后端 (统一IESKF) |
| **LVI-SAM** | `imageProjection.cpp` `deskewInfo()` | `featureExtraction.cpp` | `mapOptmization.cpp:966-1127` (LIS) + `estimator.cpp` (VIS) | GTSAM ISAM2 (LIS) + Ceres滑窗 (VIS) |
| **PIN-SLAM** | N/A (SDF隐式处理) | `model/neural_points.py:29` (NeuralPoints) | `utils/tracker.py:367-611` (点-SDF LM) | `utils/pgo.py` + `pin_slam.py:315-348` |
| **BEV-LSLAM** | `scantoscan_kitti.cpp:1096-1124` (slerp) | `scantoscan_kitti.cpp:261-316` (ORB on BEV) | `scantoscan_kitti.cpp:333-506` + `scantomap_kitti.cpp:804-854` (VGICP) | `scantomap_kitti.cpp:2354-2527` (4DoF Ceres PGO) |
| **Lightning-LM** | `imu_processing.hpp:173-313` | `lightning_math.hpp:358-414` (esti_plane) | `laser_mapping.cc:629-684` (IESKF 12维雅可比) | `loop_closing.cc:253-350` (miao PGO) |
| **lt-mapper** | (前端完成) | (前端完成) | N/A (PGO-only) | `LTslam.cpp:157-184` (GTSAM ISAM2) + `BetweenFactorWithAnchoring.h:18-127` |

---

*本文档基于以下 15 个项目的深度源码分析生成：*
- */home/lin/Projects/lin_ws/slam_ws/docs/deep_dive/\*analysis.md*