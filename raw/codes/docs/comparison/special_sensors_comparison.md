# 特殊传感器数据管线横向对比

> 覆盖：4D 毫米波雷达、事件相机、轮速/里程计、深度相机、磁力计/气压计、CAN/车辆传感器
> 数据来源：`docs/deep_dive/*.md` + `docs/framework_comparison.md`

---

## 1. 4D 毫米波雷达

### 1.1 传感器物理基础

4D 成像雷达（如 Oculii Eagle, 76-81 GHz FMCW）输出每个点的 **3D 空间位置 + 多普勒径向速度**，即 (x, y, z, v_doppler, SNR)。

| 特性 | 4D 成像雷达 | 机械/固态 LiDAR | 视觉相机 |
|------|-----------|---------------|---------|
| 点云密度 | 低（~10³点/帧） | 高（~10⁴-10⁵点/帧） | N/A（像素级） |
| 距离精度 | 中等（~0.1-1m） | 高（~0.02m） | 差（深度估计不准） |
| 角度分辨率 | 差（~1-2°） | 好（~0.1-0.2°） | 好（像素级） |
| Doppler 速度 | 每点自带 | 无 | 需要光流估计 |
| 恶劣天气 | 毫米波穿透 | 烟雾/雨水散射 | 完全失效 |
| FOV | 宽（>100°） | 30-360° | 通常<90° |
| 测距范围 | >300m | ~100-200m | <50m 可靠 |

### 1.2 信号链路（物理层 → 点云，4DRadarSLAM 使用 Oculii Eagle）

```
FMCW chirp 发射: 77-81 GHz 线性扫频, Tc ~40 μs, B ~4 GHz
    s_TX(t) = A_TX * exp(j*2π*(f_c*t + 0.5*α*t²)), α = B/Tc
                  ↓
接收混频 (dechirp):
    s_RX(t) 延迟 τ = 2R/c, s_IF = s_TX * conj(s_RX)
    f_IF = α*τ = 2αR/c  →  R = c * f_IF / (2α)
                  ↓
Range FFT (1D): ADC → per-chirp N_FFT 点 (256/512)
    距离分辨率: ΔR = c/(2B) ≈ 0.038 m (B=4 GHz)
                  ↓
Doppler FFT (2D): 同一 range bin 跨 M chirp → M_FFT (128/256)
    径向速度: v_r = λ * f_d / 2  (λ ≈ 3.8 mm @79 GHz)
    速度分辨率: Δv = λ/(2*Tc*M) ≈ 0.05 m/s
                  ↓
MIMO DBF (3D): 虚拟阵列波束形成 → 方位角 α, 俯仰角 β
    角度分辨率: Δα ≈ λ/(d*N_TX*N_RX) ≈ 1-2°
                  ↓
CFAR 检测: CA-CFAR / OS-CFAR 自适应阈值
    T = β * P_noise  (虚警率 P_fa = 1e-6)
                  ↓
4D 点云输出: (x, y, z, doppler, SNR), 每帧 10²-10³ 点
```

### 1.3 使用项目

| 项目 | 传感器 | 数据格式 | 频率 |
|------|--------|---------|------|
| **4DRadarSLAM** | Oculii Eagle (76-81 GHz) | `sensor_msgs::PointCloud` (ch[0]=doppler, ch[1]=range, ch[2]=SNR) | ~10 Hz |

### 1.4 预处理管线（4DRadarSLAM, `preprocessing_nodelet.cpp`）

```
[SNR 功率滤波] channels[2].values[i] > power_threshold (默认 0.0)
    ↓ NaN/Inf 检查
[坐标系链式变换] Radar → Change_Radarframe → Radar_to_Thermal → Thermal_to_RGB → RGB_to_livox → Livox
    Change_Radarframe = [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
    dst = Radar_to_livox * [x, y, z, 1]^T (4×4 齐次)
    ↓
[Ego-Velocity 估计] (radar_ego_velocity_estimator.cpp):
    原理: v_doppler_i = -v_ego · unit_i (静止目标)
    H_i = [x_i/r_i, y_i/r_i, z_i/r_i], y_i = v_doppler_i
    目标筛选: r∈[1,400]m, SNR>10dB, |azimuth|<56.5°, |elevation|<22.5°
    零速度检测: 排除30%最大|doppler|, median<0.05 m/s → v_r=0
    RANSAC + LSQ: 采样5点, ~115次迭代, solve3DFull() LDLT/SVD
    方差: C = (e^T*e)*(HTH)^(-1)/(N-3), sigma = sqrt(diag(C)) + offset
    ↓
[动态目标去除] enable_dynamic_object_removal → 仅使用 RANSAC 内点(静止目标)
    ↓
[Deskew] 假设匀速旋转, scan_period=0.1s, IMU角速度补偿:
    delta_t = scan_period * i / N; delta_q = exp(0.5 * delta_t * ang_v)
    pt' = delta_q^(-1) * pt
    ↓
[降采样+滤波] 距离滤波 2m<d<100m → VoxelGrid(0.1m) → RadiusOutlier(0.5m,1)
    ↓
输出: filtered_points (PointXYZI, intensity=SNR dB)
```

### 1.5 与 LiDAR 管线的关键差异

| 维度 | 4D 雷达管線 | LiDAR 管線 |
|------|-----------|-----------|
| **点云密度** | 100-1000 点/帧，极度稀疏 | 10⁴-10⁵ 点/帧，密集 |
| **测量噪声模型** | 严重各向异性：距离方向 σ~0.86m@100m, 角度 σ~1° | 近各向同性，σ~2cm |
| **配准方法** | **APDGICP**（传感器模型驱动各向异性协方差） | ICP/GICP/NDT（各向同性或简单各向异性） |
| **速度信息** | 每点自带多普勒径向速度 → ego-velocity 估计 | 无速度信息（需多帧差分/IMU） |
| **动/静态区分** | RANSAC 多普勒一致性 → 自然区分 | 需要语义分割或运动模型 |
| **特征提取** | 不提取几何特征（噪声太大） | 角点/平面点/Harris/曲率等 |
| **描述子** | Intensity Scan Context（SNR 代替代高度） | Scan Context（最大高度） |
| **Deskew** | IMU 角速度 + 匀速旋转假设（简单） | IMU 积分位姿或匀速假设 |
| **恶劣天气** | 物理层不受影响（3.8mm 波长穿透烟雾） | 激光散射失效 |

**APDGICP 核心参数**（`registrations.cpp:46-48`）：
```cpp
apdgicp->setDistVar(0.86);       // 距离测量不确定度 @100m
apdgicp->setAzimuthVar(0.5);     // 方位角精度 (度)
apdgicp->setElevationVar(1.0);   // 俯仰角精度 (度)
```

### 1.6 多普勒速度的利用

多普勒速度是 4D 雷达唯一具有、LiDAR/视觉完全不具备的信息，在 4DRadarSLAM 中四重利用：

| 环节 | 利用方式 | 实现 |
|------|---------|------|
| **自运动估计** | v_doppler = -v_ego · unit → RANSAC LSQ 解 3D 速度 | `radar_ego_velocity_estimator.cpp` |
| **前端初值** | 自速度累积 + IMU 旋转 → 配准初始猜测 | `scan_matching_odometry_nodelet.cpp:291-295` |
| **动态目标去除** | RANSAC outlier（不满足静止模型）→ 标记为动态目标 | `preprocessing_nodelet.cpp:340-344` |
| **配准失败降级** | ICP 失败时 fallback 到 egovel + imu 构造变换 | `scan_matching_odometry_nodelet.cpp:525-526` |

### 1.7 后端融合模式

4DRadarSLAM 基于 **g2o** 位姿图，多普勒信息不作为独立因子进入图优化：前端里程计边（APDGICP 结果）、GPS 约束、气压计约束、回环边。多普勒在**前端预处理阶段**完全消费，后端不直接使用。

---

## 2. 事件相机

### 2.1 传感器物理基础

事件相机（DVS/DAVIS）每个像素独立、异步检测对数亮度变化：
- 数据格式：`(x, y, t, polarity)`，t 为 μs 级时间戳
- 当 `|Δlog(I)| > threshold` 时输出事件（ON=增亮, OFF=变暗）
- 仅输出变化的像素，静止区域无输出

| 特性 | 事件相机 | 传统帧相机 |
|------|---------|-----------|
| 时间分辨率 | μs 级 | ms 级（帧率倒数） |
| 动态范围 | 120dB+ | 60-70dB |
| 数据量 | 稀疏（仅边缘变化处） | 全帧所有像素 |
| 运动模糊 | 天然免疫 | 帧积累导致模糊 |
| 纹理信息 | 仅边缘 | 全图纹理 |
| 噪声 | 热噪声、背景活动（BA） | 读出噪声、暗电流 |

### 2.2 使用项目

| 项目 | 传感器 | 数据接口 | 分辨率 |
|------|--------|---------|--------|
| **ESVO** | 双目事件相机 (DAVIS240/CeleX-V) | ROS topic `events_left/right` → `EventQueue` | 346×260 / 640×480 |

### 2.3 异步事件处理管线（ESVO）

#### 事件缓冲

```
dvs_msgs::Event (x: uint16, y: uint16, ts: ros::Time, polarity: bool)
    ↓ push_back() 实时添加
EventQueue = std::deque<dvs_msgs::Event>  (按时间排序, FIFO)
    maxEventQueueLength = 3,000,000 事件
    EventBuffer_lower_bound(): 二分查找 O(log N) 定位
```

#### Time Surface 构建（ESVO 核心转换）

```
算法: 指数衰减 Time Surface
同步脉冲触发 → createTimeSurfaceAtTime(T):
    对每个像素 (x,y):
        找最近一次事件 e (t_e ≤ T)
        TS(x,y) = exp(-(T - t_e) / τ)    τ = decay_ms (默认 30ms)
    极性处理 (ignore_polarity_=true): 使用绝对值
    FORWARD 模式: 先校正畸变再双线性插值
    ↓
可选高斯平滑: cv::GaussianBlur(TS, kernelSize=15, 0.0)
    ↓
Negative Time Surface: TS_negative = 255 - TS (OFF 极性补充)
    ↓
Sobel 梯度: cv::Sobel(TS, dTS_du, CV_64F, 1, 0) → dTS_du_left_
           cv::Sobel(TS, dTS_dv, CV_64F, 0, 1) → dTS_dv_left_
    ↓
输出: TimeSurfaceObservation { TS_left_, TS_right_, tr_(位姿), id_ }
```

**关键洞察**：Time Surface 将异步稀疏事件流**转换为类帧图像的 2D 表示**，使其可以使用传统图像处理技术（Sobel、高斯模糊）和立体匹配算法。

#### 事件筛选（Mapping 线程）

```
createEdgeMask():      仅保留边缘区域事件
createDenoisingMask(): 空间密度去噪
extractDenoisedEvents(maxNum=20000):
    随机采样 → mask 过滤 → 分配到多线程匹配
```

#### 立体匹配 — ZNCC Block Matching

```
EventBM::match_an_event() (esvo_core/src/core/EventBM.cpp:80-168):
    1. 畸变校正: getRectifiedUndistortedCoordinate(x, y)
    2. 边界检查 + Mask 检查 (UndistortRectify_mask_ > 125)
    3. 提取左图模板 patch (25×25 像素)
    4. 有效性检查: 有效像素 > 5% (95% 0 值 → 无效)
    5. 粗搜索: 步长 step_=5, 沿极线 ZNCC 搜索, 视差范围 [1, 40]
    6. 精搜索: 在粗搜索最优 ±(step-1) 内逐像素 ZNCC
代价函数: ZNCC (零均值归一化互相关), 阈值 0.1 (= ZNCC > 0.9)
多线程: NUM_THREAD_MAPPING=4, 交错索引分配
```

#### 深度估计与融合

```
DepthProblem: 对每个 EventMatchPair 优化逆深度
    warp 方程: x₁_s, x₂_s = warp(x, d, T_left_virtual)
    残差: r = TS_left(x₁_s) - TS_right(x₂_s)  (TS 一致性)
    损失函数: Student-t 分布 (td_nu, td_scale)
    ↓
DepthFusion: 跨帧深度融合
    propagate_one_point(): 前帧深度投影到当前帧
    chiSquareTest(): (invD1-invD2)²/(var1+var2) 统计兼容检验
    兼容 → weighted average 融合
    ↓
DepthRegularization: 空间中值滤波 (radius=5, min_neighbors=3)
```

#### Tracking — 逆向组合法（Inverse Compositional）位姿估计

```
RegProblemLM: 当前 TS 对齐到参考帧的反投影地图
    残差: TS_cur(π(T·X_ref + δx)) - TS_ref(X_ref)
    位姿参数: 7-DOF (quaternion + translation)
    解法:
        数值法: Eigen LM (默认, MAX_ITER=10, BATCH_SIZE=200)
        解析法: ESVO 论文的 Inverse Compositional 解析雅可比
    点选择: up to MAX_REGISTRATION_POINTS=500 个 3D 点
```

### 2.4 与传统相机的差异

| 维度 | 事件相机管线 (ESVO) | 传统帧相机管线 (ORB-SLAM3 等) |
|------|-------------------|---------------------------|
| **数据基础** | 异步事件流 `(x,y,t,p)` | 固定帧率完整图像 |
| **中间表示** | Time Surface（指数衰减，类图像） | 原始灰度/RGB 图像 |
| **特征提取** | 事件密度筛选 + 边缘 mask | FAST 角点 + 四叉树分布 |
| **匹配方法** | ZNCC Block Matching（25×25 patch） | ORB 描述子 Hamming 距离 + BoW |
| **深度估计** | 立体 Block Matching → 非线性逆深度优化 | 三角化（线性 + 非线性） |
| **位姿优化** | Inverse Compositional（解析/数值 LM） | g2o/Ceres 重投影误差 BA |
| **跨帧融合** | Student-t 分布 + χ² 兼容性 DepthFusion | RANSAC + χ² 重投影误差 |
| **时间分辨率** | μs 级（由同步脉冲频率决定） | ms 级（由帧率决定） |
| **动/静态** | 天然输出变化区域，运动越多事件越多 | 需要光流/运动模型估计 |
| **纹理依赖** | 需要边缘变化（静止白墙无事件） | 需要角点/纹理区域 |
| **回环检测** | 无（纯 VO） | 有（BoW + 位姿图） |

### 2.5 三线程并行架构

```
Process 0 (Time Surface): 事件累积 → 同步脉冲 → Time Surface 生成 → 发布
Process 1 (Mapping):      立体匹配 → 深度优化 → 深度融合 → 3D 点云
Process 2 (Tracking):     TS 对齐反投影地图 → 位姿优化 → 输出位姿
```

初始化流程: 累积事件 → 同步脉冲 → 首帧 TS → Mapping 立体匹配 → Tracking identity pose → WORKING 状态

---

## 3. 轮速/里程计

### 3.1 使用项目总览

| 项目 | 里程计类型 | 数据格式 | 采样频率 | 融合方式 |
|------|-----------|---------|---------|---------|
| **IC-GVINS** | 轮速编码器 (单/双轮) | `IMU::odovel` (速度增量, m) | 200 Hz (与 IMU 同帧) | 紧耦合：`PreintegrationOdo` / `PreintegrationEarthOdo` |
| **OB_GINS** | 轮速编码器 (单/双轮) | IMU 文本文件第8/9列 | 100-200 Hz | 紧耦合：预积分中的里程计变体 |
| **Cartographer** | 通用里程计 | `nav_msgs::Odometry` | 可变 | 松耦合：PGO 中的 BetweenFactor |
| **SuperOdom** | VIO 里程计 (外部) | `nav_msgs::Odometry` (VIO 输出) | 20-30 Hz | 松耦合：退化时 fallback 预测、辅助去畸变 |
| **ROLO** | 前端里程计 (可选) | `nav_msgs::Odometry` | 10 Hz | 辅助：去畸变 + 初始猜测 |

### 3.2 数据格式与预处理

#### IC-GVINS / OB_GINS（共享同一套预积分体系）

里程计数据作为 **IMU 数据的一个额外字段**嵌入：

```cpp
// common/types.h
typedef struct IMU {
    double time; double dt;
    Vector3d dtheta;   // 角增量 (rad, FRD)
    Vector3d dvel;     // 速度增量 (m/s)
    double odovel;     // 里程计速度增量 ← 特殊传感器字段
} IMU;
```

- **8 列 IMU 文件**: time + dtheta(3) + dvel(3) + 单轮速 → `odovel = speed × dt`
- **9 列 IMU 文件**: time + dtheta(3) + dvel(3) + 双轮速 → `odovel = mean(speed_L, speed_R) × dt`

**预积分体系的多态可插拔设计**：

```
PreintegrationBase          抽象基类
├── PreintegrationNormal    标准 IMU 预积分 (无里程计)
├── PreintegrationEarth     含地球自转补偿
├── PreintegrationOdo       含里程计
└── PreintegrationEarthOdo  含地球自转+里程计
工厂方法: Preintegration::createPreintegration(parameters, state, options)
```

**里程计比例因子在线估计**（`IntegrationState.sodo`）：作为滑窗优化变量，通过 `ImuErrorFactor` 加入软约束。

#### Cartographer

里程计通过 `SensorCollator` 与 LiDAR 数据按时序排队，约束类型为 `BetweenFactor`（PGO 中 `odometry_translation_weight/rotation_weight = 1e5`）。

#### SuperOdom

无专用轮速计。VIO 里程计作为**多级预测策略**第三级：`LIO_ODOM → VIO_ODOM → NEURAL_IMU_ODOM → IMU_ORIENTATION → CONSTANT_VELOCITY`。VIO 主要用于退化场景（LiDAR 特征不足）时的位姿预测。

### 3.3 标定方式

| 项目 | 标定方法 | 在线估计 |
|------|---------|---------|
| **IC-GVINS** | 里程计比例因子 `sodo` 在线优化，安装角 `abv` 作为状态变量 | sodo (1D), abv (2D) |
| **OB_GINS** | 同上，`IntegrationState` 中含 `sodo` 和 `abv` | sodo (1D), abv (2D) |
| **Cartographer** | 通过 TF 外参 (`odometry_frame → base_link`) | 否（静态 TF） |
| **SuperOdom** | VIO 输出已含自身标定，SuperOdom 仅消费 | 否（外部提供） |

### 3.4 因子/图集成方式

```
IC-GVINS / OB_GINS (Ceres):
    PreintegrationFactor [pose₀(7), mix₀(N), pose₁(7), mix₁(N)]
        mix 含: vel(3) + bg(3) + ba(3) + [sodo(1)] + [abv(2)]
    里程计通过修改预积分残差维度融入 (标准15维 → 含ODO 16+维)

Cartographer (Ceres PGO):
    BetweenFactor<Pose3D> odometry edges
    权重: translation=1e5, rotation=1e5

SuperOdom (GTSAM ISAM2):
    PriorFactor<Pose3> (LiDAR 位姿先验, 非里程计因子)
    VIO 仅用于位姿预测，不作为因子进入优化图
```

### 3.5 设计考虑

| 考虑 | 实践 |
|------|------|
| **滑动/打滑处理** | 无显式处理。依赖 IMU 预积分的协方差传播自然加权，滑移大时误差大则权重低 |
| **零速检测** | IC-GVINS 初始化阶段用陀螺/加速度阈值判定静止以估计零偏 |
| **安装角误差** | IC-GVINS/OB_GINS 在线估计 `abv`（IMU 与车辆前进方向的偏差），防止侧偏误差累积 |
| **比例因子漂移** | 轮径变化（胎压/磨损）→ `sodo` 在线估计，变化>6σ 触发预积分重算 |
| **纯惯性退化** | OB_GINS 无视觉，GNSS 中断后仅里程计+惯性，漂移比有视觉方案快 |

---

## 4. 深度相机 (RGB-D)

### 4.1 使用项目总览

| 项目 | 深度传感器 | 深度使用模式 | 深度分辨率 |
|------|-----------|------------|-----------|
| **NICE-SLAM** | Intel RealSense | 可微渲染监督 + 几何初始化 | 与 RGB 同分辨率 |
| **DROID-SLAM** | 通用 RGB-D | 深度融合 (`DepthVideo.disps_sens`) | 与 RGB 同分辨率 |
| **MonoGS** | 通用 RGB-D | 反投影初始化 + Tracking/Mapping 光度损失 | 与 RGB 同分辨率 |
| **ORB-SLAM3** | 通用 RGB-D | 直接反投影获取 3D 点 | 与 RGB 同分辨率 |

### 4.2 深度数据管线差异

| 维度 | NICE-SLAM (神经隐式) | DROID-SLAM (学习式) | MonoGS (3DGS) | ORB-SLAM3 (传统) |
|------|---------------------|---------------------|---------------|-----------------|
| **地图表示** | 特征网格 + MLP | 位姿+逆深度 state buffer | 3D Gaussian primitives | 稀疏 MapPoint |
| **深度初始化** | 特征网格高斯初始化 (std=0.01) | 传感器深度反算逆深度 | 首帧 RGB-D 反投影 → Gaussian | 直接 UnprojectStereo |
| **深度消费方式** | 可微渲染 → 体积渲染深度 → L1 loss with gt | disp_sens 与 est_disp 比较 → 加权融合 | `L_depth = \|d_render - d_gt\|` (RGB-D 模式) | 直接作为已知深度，匹配时做 3D-2D |
| **深度不确定性** | 渲染不确定度 `Σ w_i*(z_i-d)²` → 动态过滤 | 无显式模型，阈值比较 | opacity > 0.95 像素才计算深度损失 | 通过金字塔层级缩放信息矩阵 |
| **深度融合** | 在线梯度下降优化特征 | 逐帧替换/加权融合 (`disps_sens` 标记) | 在线梯度下降联合优化 | 无融合，已有深度 |
| **空洞处理** | **可插值/外推**（神经先验） | 传感器未覆盖区域不更新 | **可插值/外推**（Gaussian 连续表示） | 深度无效则不创建 MapPoint |
| **GPU 依赖** | 必须 | 必须 | 必须 | 不需要 |
| **实时性** | ~1-3 FPS | 实时 (GPU) | ~3 FPS | 实时 (CPU) |

### 4.3 关键设计差异

```
传统方式 (ORB-SLAM3):
    深度直接用于特征点 3D 恢复: xyz = UnprojectStereo(u, v, depth)
    已知深度 → 双目模式等价于 RGB-D → 跳过三角化
    深度作为"已知量"，不在 BA 中优化

学习方式 (DROID-SLAM):
    传感器深度 → disp_sens = 1/depth
    BA 中既使用传感器深度（融合）又优化网络预测深度
    深度残差不独立定义，通过光流残差间接约束

隐式方式 (NICE-SLAM/MonoGS):
    深度作为可微渲染的训练信号
    RGB-D → 场景表示（特征网格或 Gaussians）→ 渲染深度 → |render - gt|
    深度不单独存储，从场景表示中"查询"获得
```

### 4.4 深度噪声处理

| 项目 | 策略 |
|------|------|
| **NICE-SLAM** | 渲染不确定度异常检测：`\|gt_depth - render\|/\sqrt{uncertainty} < 10×median` |
| **DROID-SLAM** | 阈值比较 + 加权融合，`disps_sens` 标记防重复融合 |
| **MonoGS** | opacity 加权：深度损失仅在高 opacity (>0.95) 像素计算 |
| **ORB-SLAM3** | 无特殊噪聲处理（依赖 RealSense 等深度传感器自身滤波） |

---

## 5. 磁力计/气压计

### 5.1 使用项目

| 项目 | 磁力计 | 气压计 | 使用方式 |
|------|--------|--------|---------|
| **OB_GINS** | 不直接使用（双天线 GNSS 替代航向） | 不直接使用 | -- |
| **IC-GVINS** | 不直接使用（双天线 GNSS + GNSS 位置差分） | 不直接使用 | -- |
| **4DRadarSLAM** | 不使用 | **使用** | g2o 边：`EdgeSE3PriorZ` / `EdgeSE3Z` |
| **R3LIVE** | 不使用 | 不使用 | -- |
| **SuperOdom** | 不使用 | 不使用 | -- |

### 5.2 气压计管线（4DRadarSLAM, `radar_graph_slam_nodelet.cpp:441-517`）

```
气压计 → std_msgs::Float32 (高度, m)
    ↓ 定时器 flush_barometer_queue()
[两种约束模式]:
    Type 1 — 绝对高度先验: EdgeSE3PriorZ (单节点 Z 约束)
    Type 2 — 相对高度约束: EdgeSE3Z (相邻关键帧高度差约束)
        edge = graph_slam->add_se3_prior_z_edge(node, barometer_height, info)
    ↓
g2o 位姿图: odometry + GPS + barometer + loop edges → LM 优化
```

### 5.3 磁力计/航向获取的替代方案

| 方案 | 使用项目 | 原理 |
|------|---------|------|
| **双天线 GNSS 航向** | IC-GVINS, OB_GINS | `isyawvalid=true` → 直接读取 `yaw` |
| **GNSS 位置差分航向** | IC-GVINS (双天线不可用时) | `atan2(dy, dx)` |
| **视觉惯性对齐** | VINS-Mono 体系 | 重力方向 + 视觉 BA 恢复 |
| **先验配置** | OB_GINS | `initatt` 直接从 YAML 读取 |

### 5.4 设计考虑

- **气压计**：绝对气压受天气影响大（同地不同天气可差数米），相对模式（相邻帧差分）更可靠。4DRadarSLAM 的回环预筛选中使用高度差 > 2m 排除候选帧。
- **磁力计**：室内干扰严重（金属结构、电流），SLAM 项目普遍回避。工业级做法使用双天线 GNSS 替代或依赖视觉惯性初始化。
- **融合方式**：气压计约束通常以较低的权重加入位姿图，作为辅助信息而非主要约束。

---

## 6. CAN/车辆传感器

### 6.1 使用项目

| 项目 | 车辆约束类型 | 数据来源 | 融合方式 |
|------|------------|---------|---------|
| **ROLO** | 地面车辆运动约束 | 无外部 CAN，基于几何假设 | 硬约束：roll/pitch/z 限幅 |
| **IC-GVINS** | 轮式机器人里程计 | 轮速编码器（通过 IMU 数据帧） | 紧耦合：预积分里程计变体 |
| **OB_GINS** | 车辆里程计 | 轮速编码器（通过 IMU 文本文件） | 紧耦合：预积分里程计变体 |

### 6.2 车辆运动约束（ROLO 核心设计）

ROLO 利用地面车辆的物理约束防止发散（`backMapping.cpp:898-906`）：

```
constraintTransformation(value, rotation_tollerance):  roll/pitch 角限幅
constraintTransformation(value, z_tollerance):          z 坐标限幅
```

这些不是来自 CAN 总线的传感器读数，而是基于**平台运动学假设**的硬约束：
- 地面车辆 roll/pitch 变化有限（悬挂行程限制）
- z 方向运动通常由地形变化决定
- 偏航（yaw）和 XY 平移不受限制（车辆可自由转向和平移）

**后端初始化先验**也利用了此假设（`backMapping.cpp:1024-1043`）：

```cpp
PriorFactor<Pose3> with noise:
    (1e-2, 1e-2, π², 1e8, 1e8, 1e8)
    // 偏航和位移弱约束 → roll/pitch 强约束
```

### 6.3 CAN 数据集成模式

CAN 总线通常提供：轮速、方向盘转角、油门/制动状态、挡位。在本仓库分析的 SLAM 项目中：

- **无项目直接读取 CAN 总线**。
- **轮速**：通过 IMU 数据帧中间接输入（`odovel` 字段，IC-GVINS/OB_GINS）。
- **方向盘转角**：无项目使用（可用于预测运动曲率，简化车辆运动模型）。
- **挡位/速度方向**：无项目使用（可辅助检测前进/后退切换）。

### 6.4 设计考虑

| 考虑 | 实践 |
|------|------|
| **车辆运动模型** | ROLO 通过 roll/pitch/z 限幅隐式使用自行车模型平面假设 |
| **滑移/侧偏** | 无项目显式建模车辆侧偏角 |
| **速度约束** | IC-GVINS 的 IMU 失败检测：\|v\| > 30 m/s → 重置（车辆速度物理上界） |
| **前进/后退** | 无项目处理方向切换（假设持续前进） |
| **悬架运动** | ROLO 的 roll/pitch/z 限幅在一定程度上补偿悬架引起的小幅姿态变化 |

---

## 7. 多传感器融合模式总结

### 7.1 融合深度分类

```
                        GNSS 观测层次
                             │
    原始伪距/载波 ──────────┼────────── 定位结果 (RTK)
         │                  │                │
    超紧耦合           紧耦合(原始观测)    松耦合(位置级)
    (GNSS直接辅助      (本仓库无项目)    IC-GVINS, OB_GINS,
     信号跟踪)                          4DRadarSLAM, Cartographer
```

```
                       多传感器融合紧度
                             │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
  紧耦合                半紧耦合              松耦合
  IC-GVINS (GNSS+IMU+Camera  SuperOdom (LiDAR↔IMU  LVI-SAM (独立子系统
   同 Ceres 联合优化)      互馈, 各自独立优化)       仅交换位姿约束)
  R3LIVE (LiDAR+IMU          Cartographer
   ESIKF 联合, VIO           (子地图前端↔PGO后端)
   附加先验)
  FAST-LIVO2
   (统一 IESKF)
```

### 7.2 插件式设计模式

以下项目展示了"传感器可插拔"的设计范式：

| 项目 | 插件机制 | 示例 |
|------|---------|------|
| **IC-GVINS/OB_GINS** | 预积分工厂模式多态 | `Preintegration::createPreintegration()`: Normal / Earth / Odo / EarthOdo |
| **4DRadarSLAM** | 配准方法工厂 | ICP / NDT_OMP / FAST_GICP / **FAST_APDGICP** / FAST_VGICP |
| **SuperOdom** | 多源位姿预测优先级 | VIO_ODOM → NEURAL_IMU_ODOM → LIO_ODOM → IMU_ORIENTATION → CONSTANT_VELOCITY |
| **ORB-SLAM3** | 六种传感器模式枚举 | MONOCULAR / STEREO / RGBD / IMU_MONOCULAR / IMU_STEREO / IMU_RGBD |
| **Cartographer** | SensorCollator + 多轨迹 | LiDAR + IMU(可选) + Odometry(可选) + FixedFramePose(可选) |

### 7.3 优先级切换模式

SuperOdom 提供了最完善的多级 fallback 机制（`determinePredictionSource()`, `laserMapping.cpp:384-412`）：

```
系统状态    退化?    VIO可用?    → 预测源
───────────────────────────────────────────
WORKING     是       是         → VIO_ODOM
WORKING     是       否         → NEURAL_IMU_ODOM
WORKING     否       -          → LIO_ODOM
WORKING     是       -          → IMU_ORIENTATION
(兜底)      -        -          → CONSTANT_VELOCITY
```

4DRadarSLAM 的前端配准也有三重质量检查 + fallback：

```
配准质量检查:
    1. 变换增量 > 1.0m / > 3° → 拒绝
    2. 雷达旋转 vs IMU旋转 > 0.8° → 拒绝
    3. 自速度位移 vs ICP位移 > 0.3m → 拒绝
fallback: imu_rotation + egovel_translation
```

### 7.4 健康监控通用模式

| 机制 | 使用项目 | 实现 |
|------|---------|------|
| **协方差传播与检查** | IC-GVINS, OB_GINS | 预积分协方差逆 Cholesky 作为信息矩阵 |
| **GNSS 粗差检测** | IC-GVINS, OB_GINS | χ²(3) 阈值 7.815, HuberLoss → reweight |
| **视觉粗差检测** | IC-GVINS, ORB-SLAM3 | χ²(2) 阈值 5.991, 超标移除残差块 |
| **偏置发散检测** | IC-GVINS, OB_GINS | bg/ba 变化 > 6σ → 触发预积分重算 |
| **IMU 失败检测** | SuperOdom | \|v\| > 30, \|ba\| > 2.0, \|bg\| > 1.0 → reset |
| **退化检测** | SuperOdom, ROLO | 6-DoF 可观测性直方图 / Hessian 特征值 < 100 |
| **配准质量自适应** | 4DRadarSLAM | fitness_score → 自适应信息矩阵权重 |
| **渲染不确定度过滤** | NICE-SLAM, MonoGS | uncertainty-based outlier rejection |

### 7.5 传感器间信息流向

```
                           [IMU 高频 (100-200Hz)]
                           /        |        \
                          /         |         \
          [传播/预测]    [紧耦合]  [预积分]    [去畸变]
          /               |          |           \
    SuperOdom          IC-GVINS   OB_GINS      4DRadarSLAM
    R3LIVE             R3LIVE                  ROLO
    Cartographer       FAST-LIVO2             SuperOdom


        [轮速/里程计]                      [GNSS]
             |                                |
     ┌───────┴────────┐              ┌────────┴─────────┐
  紧耦合            松耦合          位置级松耦合        原始观测紧耦合
  IC-GVINS(Odo)  Cartographer      IC-GVINS, OB_GINS      (本仓库无)
  OB_GINS(Odo)   SuperOdom(VIO)    4DRadarSLAM, Cartographer


      [深度相机 RGB-D]                  [事件相机]
             |                               |
     直接几何深度    可微渲染深度         Time Surface → ZNCC Block Match
     ORB-SLAM3     NICE-SLAM            ESVO
                   MonoGS
                   DROID-SLAM


      [气压计]            [车辆运动约束]        [磁力计]
         |                     |                   |
    相对/绝对Z先验      roll/pitch/z 硬约束     SLAM 项目普遍回避
    4DRadarSLAM         ROLO                  (用双天线GNSS/视觉替代)
```

---

## 8. 关键结论

1. **4D 雷达的核心价值在 Doppler**：从自运动估计到动态去除到前端初值，Doppler 贯穿整个前端。后端融合仍是传统位姿图，Doppler 不进入优化图。

2. **事件相机的核心挑战是表示转换**：从异步稀疏事件流到类图像 Time Surface 是关键一步，之后可复用传统立体匹配和位姿优化技术。

3. **里程计的最佳实践是紧耦合**：IC-GVINS/OB_GINS 的预积分多态设计将里程计无缝融入 IMU 预积分体系，比松耦合独立优化更精确。

4. **RGB-D 在传统和学习范式中角色截然不同**：传统 SLAM 将深度作为"已知几何"，学习式 SLAM 将深度作为"训练信号"。

5. **气压计/磁力计的工程价值有限**：气压计用于辅助回环筛选，磁力计因室内干扰普遍被回避。

6. **车辆约束是低成本提升精度的有效手段**：地面车辆的运动学约束（roll/pitch/z 限幅）在不增加传感器成本的情况下显著抑制垂直漂移。

7. **多传感器融合的核心是可插拔性和降级路径**：传感器可能随时失效，设计时需考虑从最优到最简的多级 fallback 机制。

---

*文档生成日期：2026-04-29*