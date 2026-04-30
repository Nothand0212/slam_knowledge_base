# OpenMAVIS 源码深度分析报告

> 基于 ORB-SLAM3 的多相机增强视觉惯性 SLAM（`IMU_MULTI` 模式，sensor=6）
> 分析版本：OpenMAVIS (commit: 基于 ORB-SLAM3 V1.0, Dec 2021)
> 仓库地址：https://github.com/MAVIS-SLAM/ORB_SLAM3_MULTI

---

## 1. 数据接收与预处理

### 1.1 图像与 IMU 进入系统

图像通过 `System::TrackMulti()` (`include/System.h:131`) 进入，该函数接收四个图像参数：

```cpp
Sophus::SE3f TrackMulti(
    const cv::Mat &imLeft,       // 前视左 (cam1)
    const cv::Mat &imRight,      // 前视右 (cam0)
    const cv::Mat &imSideLeft,   // 侧视左 (cam4)
    const cv::Mat &imSideRight,  // 侧视右 (cam3)
    const double &timestamp,
    const vector<IMU::Point>& vImuMeas,
    string filename);
```

对应 Hilti 2022 数据集的硬件编号映射 (`Examples/Multi-Inertial/multi_inertial_euroc.cc:97-99`)：
- `cam1/data` → 前视左相机 (主相机)
- `cam0/data` → 前视右相机
- `cam4/data` → 侧视左相机
- `cam3/data` → 侧视右相机

`System::TrackMulti()` 内部调用 `Tracking::GrabImageMulti()` (`src/Tracking.cc:1540`)。

IMU 数据通过 `Tracking::GrabImuData()` (`src/Tracking.cc:1712`) 以单个 `IMU::Point` 为单位追加到 `mlQueueImuData` 队列。`IMU::Point` 定义于 `include/ImuTypes.h:46-59`，包含加速计三维向量 `a`、陀螺仪三维向量 `w`、时间戳 `t`。

### 1.2 多相机配置

当 `mSensor == System::IMU_MULTI` (=6) 时，系统加载 **4 个相机模型 + 3 组外参**，在 `Tracking::newParameterLoader()` (`src/Tracking.cc:589-596`)：

| 变量 | 含义 | 来源 |
|---|---|---|
| `mpCamera` (左前主相机) | 世界系相对于 cam1 的位姿 | `settings->camera1()` |
| `mpCamera2` (右前相机) | 相对于 cam1 通过 Tlr 推导 | `settings->camera2()` |
| `mpCamera3` (侧左相机) | 相对于 cam1 通过 Tlsl 推导 | `settings->camera3()` |
| `mpCamera4` (侧右相机) | 相对于 cam1 通过 Tlsr 推导 | `settings->camera4()` |
| `mTlr` | 左→右外参 (SE3) | `settings->Tlr()` |
| `mTlsl` | 左→侧左外参 (SE3) | `settings->Tlsl()` |
| `mTlsr` | 左→侧右外参 (SE3) | `settings->Tlsr()` |

**关键设计**：所有相机通过链式 SE3 变换从主相机位姿推导，而非独立估计。`Settings` 类 (`include/Settings.h`) 通过 `readCamera1()`~`readCamera4()` 分别解析 YAML 配置文件中各相机的内参（fx, fy, cx, cy）、畸变系数（k1-k4, p1-p2）以及图像重叠区域（`lappingBegin/lappingEnd`）。

相机模型可以是 **Pinhole** 或 **KannalaBrandt8（鱼眼）**，由 `GeometricCamera` 抽象基类统一管理 (`include/CameraModels/GeometricCamera.h`)。

### 1.3 同步机制

在 `multi_inertial_euroc.cc:194-201` 中：
```cpp
while(vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni])
{
    vImuMeas.push_back(IMU::Point(acc, gyro, timestamp));
    first_imu[seq]++;
}
```
收集从上一帧时间到当前帧时间之间的所有 IMU 测量，打包传入 `TrackMulti()`。IMU 数据在 `PreintegrateIMU()` (`src/Tracking.cc:1718`) 中从 `mlQueueImuData` 取出并按时间窗口分配到帧间。

### 1.4 图像预处理

**OpenMAVIS 的内核不做图像 rectification/resize**。代码注释明确 (`include/System.h:128-129`):
> "Images must be synchronized and rectified."

但实际上 `GrabImageMulti()` (`src/Tracking.cc:1553-1588`) 仅做 BGR/RGBA→GRAY 的色彩转换。鱼眼模式下的特征提取在**原始鱼眼图像**上进行。`Frame::UndistortKeyPoints()` (`src/Frame.cc:901-934`) 仅在 `mDistCoef` 非零时执行，鱼眼模式下畸变系数为零，跳过。

---

## 2. 特征提取与跟踪

### 2.1 特征提取方法

**使用 ORB 特征**（Oriented FAST + Rotated BRIEF），与 ORB-SLAM3 完全一致。

在 `Tracking::newParameterLoader()` (`src/Tracking.cc:609-619`) 中创建 4 个独立 `ORBextractor`：

| 提取器 | 目标相机 | 特征数 | 行号 |
|---|---|---|---|
| `mpORBextractorLeft` | 前视左 (主相机) | `nFeatures` | 609 |
| `mpORBextractorRight` | 前视右 | `nFeatures` | 612 |
| `mpORBextractorSideLeft` | 侧视左 | `nFeatures * 2` | 618 |
| `mpORBextractorSideRight` | 侧视右 | `nFeatures * 2` | 619 |

**关键差异**：侧视相机分配**2倍**特征数。原因是侧视相机的视场与前视重叠少，大部分特征以单目形式处理，需要更多特征来补偿观测稀疏性。

### 2.2 多线程提取

`Frame` 的多相机构造函数 (`src/Frame.cc:1514-1632`) 中，4路提取在 4 个 `std::thread` 并行执行 (行 1546-1553)：

```cpp
thread threadLeft(&Frame::ExtractORB,this,0,bleft,imLeft,...);
thread threadRight(&Frame::ExtractORB,this,1,bright,imRight,...);
thread threadSideLeft(&Frame::ExtractORB,this,2,bsideleft,imSideLeft,...);
thread threadSideRight(&Frame::ExtractORB,this,3,bsideright,imSideRight,...);
```

`ExtractORB(int flag, ...)` (`src/Frame.cc:465-516`) 通过 flag=0/1/2/3 调度到对应的 ORBextractor，支持 per-camera 的开关控制 (`mbEnableLeft/Right/SideLeft/SideRight`)。

### 2.3 Per-Frame 特征统计

帧总特征数：`N = Nleft + Nright + Nsideleft + Nsideright` (`src/Frame.cc:1564`)。

各相机单独计数：`Nleft`, `Nright`, `Nsideleft`, `Nsideright`。每个相机还维护**非重叠区域特征数** (`monoLeft/monoRight/monoSideLeft/monoSideRight`)，通过 ORBextractor 的 `vLapping` 区域参数 (`src/Frame.cc:467`) 控制。

### 2.4 网格分配

特征索引 `i` 的语义分配规则 (`src/Frame.cc:441-461`, `AssignFeaturesToGrid()`)：
- `i ∈ [0, Nleft)` → 前视左相机 (`mGrid[64][48]`)
- `i ∈ [Nleft, Nleft+Nright)` → 前视右相机 (`mGridRight[64][48]`)
- `i ∈ [Nleft+Nright, Nleft+Nright+Nsideleft)` → 侧视左相机 (`mGridSideLeft[64][48]`)
- `i ∈ [Nleft+Nright+Nsideleft, N)` → 侧视右相机 (`mGridSideRight[64][48]`)

### 2.5 相机间特征匹配

**前视左右立体匹配** (`ComputeMultiFishEyeMatches()`, `src/Frame.cc:1327-1381`)：
1. 仅匹配重叠区域内的特征点 (使用 `monoLeft/monoRight` 分界线，行 1329-1330)
2. `cv::BFMatcher(cv::NORM_HAMMING)` + `knnMatch(k=2)` (行 1346-1348)
3. **Lowe's ratio test 阈值 = 0.8** (行 1355: `(*it)[0].distance < (*it)[1].distance * 0.8`)
4. 对通过 ratio test 的匹配，调用 `KannalaBrandt8::TriangulateMatches()` 三角化 (行 1360)
5. 深度阈值：`depth > 0.0001f` (行 1361)

**关键局限** (`src/Frame.cc:1608-1609` 注释)：
> "Here we assume sideward camera has no significant overlap with stereo camera."

**侧视相机与主相机完全不做特征匹配**。侧视特征以纯单目方式使用，三角化依赖多帧运动视差。

### 2.6 跟踪方法

在 `Track()` (`src/Tracking.cc:1888-2424`) 中：
1. **运动模型** + **投影匹配** (`TrackWithMotionModel()`, 行 3022)：
   - IMU 已初始化：直接 `PredictStateIMU()` (行 3033)
   - 否则：`mVelocity * mLastFrame.GetPose()` 预测 → `SearchByProjection()`
2. **参考关键帧** + **BoW 匹配** (`TrackReferenceKeyFrame()`, 行 2887)：
   - `SearchByBoW()` → `PoseOptimization()`
3. **Local Map 跟踪** (`TrackLocalMap()`, 行 3120)：
   - IMU 初始化后：`PoseInertialOptimizationLastFrame/LastKeyFrame()`

### 2.7 RANSAC 细节

重定位 (`Relocalization()`, `src/Tracking.cc:3829-3830`) 使用 **MLPnP** (Maximum Likelihood PnP)：

```cpp
pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);
//    置信度 0.99 | 每轮10次迭代 | 最大300 | 最少6点 | 重投影误差0.5px | Chi2_95%(2dof)=5.991
```

每轮调用 `pSolver->iterate(5,...)`（5次RANSAC迭代），直到找到足够多的内点（≥50）支持位姿。

---

## 3. 初始化

### 3.1 多相机初始化

`MultiInitialization()` (`src/Tracking.cc:2539-2612`) 流程：

1. **触发条件** (行 1997-1998)：`mState == NOT_INITIALIZED && mSensor == IMU_MULTI`
2. **特征门槛**：`N > 500` (行 2541)
3. **IMU 检查**：当前帧和上一帧都必须有有效预积分 (行 2543)
4. **坐标原点** (行 2555-2559)：
   ```cpp
   Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();  // 用 Tcb 旋转确定初始姿态
   twb0 = mCurrentFrame.mImuCalib.mTcb.translation();     // 用 Tcb 平移确定初始位置
   Vwb0.setZero();
   mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
   ```
   **第一帧的 IMU 坐标系设为世界原点**。

5. **创建关键帧**：`KeyFrame *pKFini = new KeyFrame(mCurrentFrame, ...)` (行 2562)
6. **创建地图点** (行 2567-2589)：**仅使用前视左→右匹配成功的关键点**：`mvLeftToRightMatch[i] != -1` → 三角化 → 创建 MapPoint，同时注册左、右两个观测 (`AddObservation`)。

**侧视相机在初始化完全未参与地图创建**，其贡献推迟到后续帧跟踪时通过 `TrackLocalMap()` → `SearchLocalPoints()` 的单目投影匹配实现。

### 3.2 与 StereoInitialization 的关键差异

`StereoInitialization` (`src/Tracking.cc:2427-2537`) 在非快速模式下检查加速度变化 (行 2439-2442)：
```cpp
if (!mFastInit && (current.avgA - last.avgA).norm() < 0.5)
    return;  // 加速度激励不够，拒绝初始化
```
而 `MultiInitialization` **去除了这个检查**，始终允许初始化。原因是多相机系统即使平动激励不足也能从前视立体获得足够深度信息。

### 3.3 IMU 初始化

IMU 初始化与 ORB-SLAM3 相同，通过 `LocalMapping` 线程执行，关键步骤：
- 收集 ≥ N 个关键帧后触发
- `Optimizer::InertialOptimization()` (`src/Optimizer.cc:3431`)：估计重力方向 `Rwg`、尺度 `scale`、陀螺仪 bias `bg`、加速度计 bias `ba`
- 使用 `EdgeInertialGS` 边（带重力方向和尺度顶点，见 `src/G2oTypes.cc:647-691`）

---

## 4. 逐帧状态估计

### 4.1 Schur Complement 等效观测——机制详解

这是 OpenMAVIS 论文的核心亮点。让我们逐层解析。

**核心思想**：不直接在 EKF 中用 landmark 观测更新状态，而是**先构建包含 landmark 的完整最小二乘残差模型**，再用 **Schur complement** 消去 landmark 变量，得到仅含 ego-motion 的"等效观测"残差。

**具体步骤**：

**Step 1 — 全残差模型** (`EdgeInertial::computeError()`, `src/G2oTypes.cc:565-585`)：

对于一对关键帧 `(KF1, KF2)`，系统同时考虑：
- **Ego-motion 残差** (IMU 预积分 9维):
  ```
  e_R = Log(dR^T * R1^T * R2)                                          // 旋转 3维
  e_V = R1^T * (V2 - V1 - g*Δt) - dV                                   // 速度 3维
  e_P = R1^T * (P2 - P1 - V1*Δt - 0.5*g*Δt²) - dP                     // 位置 3维
  ```
- **Landmark 残差** (视觉重投影 2维/点):
  ```
  e_vis = z_obs - project(X_3D, pose, cam_idx)
  ```

**Step 2 — 构建完整 Hessian**：

g2o 的 `Edge::GetHessian()` 方法族 (在 `G2oTypes.h` 中)：
- `EdgeMono::GetHessian()` (行 378-384): 每个 2D 观测贡献 9×9 Hessian (3D点 + 6D位姿)
- `EdgeStereo::GetHessian()` (行 453-459): 每个 3D 观测贡献 9×9 Hessian
- `EdgeInertial::GetHessian()` (行 508-518): IMU 边贡献 24×24 Hessian

**Step 3 — Schur Complement 消去 Landmark**：

在数学上，如果将 Hessian 分块为：
```
H = [H_pp  H_pm]
    [H_mp  H_mm]
```
其中 p = ego-motion 变量，m = landmark 变量。Schur complement 给出：
```
H_equiv = H_pp - H_pm * H_mm^{-1} * H_mp   // 消去 landmark 后的等价 Hessian
g_equiv = g_p - H_pm * H_mm^{-1} * g_m     // 等价梯度
```

`Optimizer::Marginalize()` (`include/Optimizer.h:98`) 正是做这个操作。最终得到一个仅含 ego-motion 变量的**降维残差模型**——这就是"Schur complement 等效观测"的含义。

**为什么高效**：直接做完整的 EKF 视觉更新需要 O(N_landmarks × D_state²) 的计算量；而 Schur complement 后，只需在 ego-motion 的低维空间（~15维 / KF对）做优化，然后以极低成本 (`H_mm^{-1}` 是对角块) 独立求解每个 landmark。

### 4.2 EKF 后端详情

OpenMAVIS 的"EKF 后端"本质是 **g2o 的滑动窗口非线性优化**，状态变量（g2o 顶点）包括：

| 类名 | 自由度 | 含义 | 定义行号 |
|---|---|---|---|
| `VertexPose` | 6 | IMU 位姿 (Rwb, twb) | `G2oTypes.h:130` |
| `VertexVelocity` | 3 | IMU 速度 (Vwb) | `G2oTypes.h:191` |
| `VertexGyroBias` | 3 | 陀螺仪 bias (bg) | `G2oTypes.h:212` |
| `VertexAccBias` | 3 | 加速度计 bias (ba) | `G2oTypes.h:234` |
| `VertexGDir` | 2 | 重力方向 (仅 yaw) | `G2oTypes.h:274` |
| `VertexScale` | 1 | 单目尺度 | `G2oTypes.h:296` |

状态传播在 `PredictStateIMU()` (`src/Tracking.cc:1832-1880`)：
```
R2 = R1 * dR                    (dR 经 bias 修正)
P2 = P1 + V1*Δt + 0.5*g*Δt² + R1*dP
V2 = V1 + g*Δt + R1*dV
```

### 4.3 视觉更新

视觉更新的核心是 `ImuCamPose` 类 (`src/G2oTypes.cc:30-99`)。构造时自动：
1. 检查 `mpCamera4` 是否存在 → 确定相机数量 (4 or 2 or 1)
2. 对所有相机计算 `Rcw[0..N], tcw[0..N]`（通过外参链式推导）
3. 所有相机模型存储于 `pCamera[0..N]`

投影函数 `ImuCamPose::Project(X_3D, cam_idx)` (`src/G2oTypes.cc:200+`)：
- 用 `Rcw[cam_idx], tcw[cam_idx]` 将世界坐标点转到相机坐标系
- 调用 `pCamera[cam_idx]->project(Pc)` 投影到图像平面

在 `KeyFrame` 层面，每个 MapPoint 的观测存储为：
```cpp
map<KeyFrame*, tuple<int,int,int,int>> observations;
//  4个int = (左前索引, 右前索引, 侧左索引, 侧右索引), -1 = 该相机未观测
```
这允许 g2o 在构造 visual edge 时自动选择正确的相机模型。

### 4.4 IMU 预积分——SE2(3) 精确方法

**核心改进**在 `Preintegrated::IntegrateNewMeasurement()` (`src/ImuTypes.cc:182-257`)。

与 ORB-SLAM3 标准预积分的差异在于对**角速度耦合效应**的精确处理 (行 206-217)：

```cpp
// SE2(3) Exact IMU Pre-integration
float theta = accW.norm();  // accW = ω - bg (角速度幅值)
float theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta3*theta;
Eigen::Matrix<float,3,3> WaccW = Sophus::SO3f::hat(accW);
Eigen::Matrix<float,3,3> WaccW2 = WaccW*WaccW;

// J1, J2 是角速度耦合下的精确积分核
J1 = dt*I + (1-cos(dt*θ))/θ²*W + (dt*θ-sin(dt*θ))/θ³*W²;
J2 = 0.5*dt²*I + (dt*θ-sin(dt*θ))/θ³*W + (0.5*dt²*θ²+cos(dt*θ)-1)/θ⁴*W²;

dP = dP + dV*dt + dR*J2*acc;  // 精确位置更新
dV = dV + dR*J1*acc;           // 精确速度更新
```

**ORB-SLAM3 原始使用的是零阶近似**：`J1 ≈ dt·I, J2 ≈ 0.5·dt²·I`——相当于假设相邻 IMU 测量间角速度为零。在高角速度场景（如无人机急转弯），这种近似引入显著误差。SE2(3) 的 `J1, J2` 精确地捕捉了角速度对位置/速度更新的耦合效应。

### 4.5 因子/残差构造详情

| 边类型 | 残差维度 | 信息矩阵来源 | 定义行 |
|---|---|---|---|
| `EdgeMono(cam_idx)` | 2 | `1/σ²(level)` | `G2oTypes.h:342-388` |
| `EdgeStereo(cam_idx)` | 3 | `1/σ²(level)` | `G2oTypes.h:425-463` |
| `EdgeInertial` | 9 | 预积分协方差逆 | `G2oTypes.h:495-544` |
| `EdgeInertialGS` | 9 | 预积分协方差逆 | `G2oTypes.h:548-631` |
| `EdgeGyroRW` | 3 | IMU walk 噪声 | `G2oTypes.h:635-668` |
| `EdgeAccRW` | 3 | IMU walk 噪声 | `G2oTypes.h:671-704` |
| `EdgePriorPoseImu` | 15 | Schur complement H | `G2oTypes.h:732-765` |

### 4.6 关键帧管理策略

`NeedNewKeyFrame()` (`src/Tracking.cc:3235-3385`) 的组合触发条件：

```
c1a: ≥ mMaxFrames 帧过去 (行 3327)
c1b: ≥ mMinFrames 帧 且 LocalMapping 空闲 (行 3329)
c1c: 特定弱跟踪条件 (非惯性传感器, 行 3331)
c2:  当前匹配 < 75%×参考帧匹配 或 需更多近点 (行 3333)
c3:  IMU模式下距上一KF ≥ 0.5秒 (行 3347-3348)
c4:  弱跟踪条件 (IMU单目, 行 3352-3353)

最终: ((c1a||c1b||c1c) && c2) || c3 || c4  (行 3358)
```

### 4.7 边缘化

`Optimizer::Marginalize()` (`src/Optimizer.cc`)：
对 Hessian 的 `[start:end, start:end]` 块做 Schur complement，在滑动窗口优化中丢弃旧状态并用先验边 (`EdgePriorPoseImu`) 保留其对当前窗口的统计影响。

---

## 5. 局部优化与全局优化

### 5.1 局部优化

**纯视觉 Local BA** (`Optimizer::LocalBundleAdjustment()`, include/Optimizer.h:61):
当前 KF + 共视 KFs + 观测到的 MapPoints。使用 g2o BlockSolver_6_3 + LM。

**视觉-惯性 Local BA** (`Optimizer::LocalInertialBA()`, `src/Optimizer.cc:2707`):
- **优化变量**：
  - 共视窗口 KFs: IMU 位姿 (6DoF) + 速度 (3DoF)
  - 共享 bias: bg (3DoF) + ba (3DoF)
  - 观测到的 MapPoints: 3D 位置
- **固定变量**：窗口外的共视 KFs（参与视觉残差但不优化）
- **约束**：
  - IMU 预积分 EdgeInertial (相邻 KF 对)
  - 视觉重投影 EdgeMono/EdgeStereo (带 `cam_idx`)
  - Bias 随机游走 EdgeGyroRW/EdgeAccRW
  - 先验边 EdgePriorPoseImu（旧状态边际化后）
- **鲁棒核**：Huber (视觉 5.99, IMU 7.815)
- **求解器**：Levenberg-Marquardt (g2o)

### 5.2 全局优化

- `Optimizer::GlobalBundleAdjustemnt()` (行 56-61): 全地图纯视觉 BA，BlockSolver_6_3
- `Optimizer::FullInertialBA()` (行 396+): 全地图视觉+惯性联合 BA，包含所有 KF 对的 IMU 边

### 5.3 回环检测

继承 ORB-SLAM3 的完整回环能力：
- DBoW2 回环候选检测 (`KeyFrameDatabase::DetectRelocalizationCandidates()`)
- Sim3 求解 (`Optimizer::OptimizeSim3()`)
- 位姿图优化 (`OptimizeEssentialGraph()` / `OptimizeEssentialGraph4DoF()`)
- Full BA（回环融合后全局精调）

---

## 6. 与 ORB-SLAM3 的区别

### 6.1 Sensor 类型扩展

新增 `IMU_MULTI` (=6) sensor 类型 (`include/System.h:98`)：
- ORB-SLAM3: `MONOCULAR=0, STEREO=1, RGBD=2, IMU_MONOCULAR=3, IMU_STEREO=4, IMU_RGBD=5`
- OpenMAVIS 新增: `IMU_MULTI=6`

### 6.2 核心新增功能

| 功能 | 实现位置 | 描述 |
|---|---|---|
| **TrackMulti()** | `System.h:131`, System.cc | 4路图像+IMU 输入接口 |
| **GrabImageMulti()** | `src/Tracking.cc:1540-1609` | 4路图像灰度化+Frame构建 |
| **MultiInitialization()** | `src/Tracking.cc:2539-2612` | 前视立体初始化+IMU bias |
| **ComputeMultiFishEyeMatches()** | `src/Frame.cc:1327-1381` | 前视左右立体匹配 |
| **4路并⾏ ORB** | `src/Frame.cc:1546-1553` | 4线程特征提取 |
| **4相机网格** | `src/Frame.cc:419-463` | `mGrid` + `mGridRight/SideLeft/SideRight` |
| **cam_idx 投影** | `src/G2oTypes.cc:30-99` | `ImuCamPose` 支持1/2/4相机 |
| **EdgeMono/Stereo(cam_idx)** | `G2oTypes.h:342-493` | 视觉边携带相机索引 |
| **J1/J2 精确预积分** | `src/ImuTypes.cc:206-217` | SE2(3) 角速度耦合精确积分 |
| **侧视2×特征** | `src/Tracking.cc:618-619` | `mpORBextractorSideLeft/Right` 特征数加倍 |
| **4相机外参** | `src/Tracking.cc:590-596` | `mTlr, mTlsl, mTlsr` |
| **Settings 4相机** | `include/Settings.h` | `calibration1_~4_`, `Tlr/Tlsl/Tlsr` |
| **KeyFrame 4相机位姿** | `src/KeyFrame.cc:156-218` | `GetPose(cameraID)` / `GetRotation(cameraID)` 等 |

### 6.3 IMU 预积分的改进

ORB-SLAM3 的预积分使用 `dP += dV*dt + 0.5*dR*acc*dt*dt` 和 `dV += dR*acc*dt`——即零阶角速度近似。

OpenMAVIS 的 SE2(3) 预积分 (`src/ImuTypes.cc:206-217`) 使用 `J1(dt,θ), J2(dt,θ)` 捕获**角速度耦合**——在高角速度下有显著的精度优势。这是论文标题中 "SE2(3) Based Exact IMU Pre-integration" 的直接体现。

---

## 7. 优缺点分析

### 7.1 算法优势

1. **SE2(3) 精确预积分** (`src/ImuTypes.cc:206-217`)：高角速度场景精度优于 ORB-SLAM3。
2. **统一的 cam_idx 投影框架** (`G2oTypes.cc:30-99`)：不需新增状态变量即可支持任意数量相机。
3. **多线程 + 自适应特征** (`src/Tracking.cc:618-619`)：侧视2×特征补偿了单目观测的不确定性。
4. **多相机网格架构** (`src/Frame.cc:419-463`)：per-camera网格保证了投影匹配的 O(1) 邻域查询。

### 7.2 算法局限

1. **侧视无直接深度** (`src/Frame.cc:1608` 硬编码假设)：侧视特征三角化延迟，高速旋转下易失效。
2. **初始化只用前视立体** (`src/Tracking.cc:2567`)：初始地图稀疏，没有利用侧视相机信息。
3. **无在线外参标定**：`mTlr/mTlsl/mTlsr` 假设精确且不变。
4. **最多4相机**：硬编码 `if(pKF->mpCamera4)` 判断上限。
5. **特定硬件假设**：Hilti 2022的相机布局（前视重叠、侧视不重叠）被硬编码。

### 7.3 工程优缺点

**优点**：
- `Settings` 类 (`include/Settings.h`) 统一配置管理，支持新旧两种参数加载
- `GeometricCamera` 虚函数抽象，Pinhole + Fisheye 无缝切换
- IMU 预积分的 `SetNewBias()` / `Reintegrate()` 支持线性化点重标定
- 时间测量宏 `REGISTER_TIMES` 提供细粒度 profiling

**缺点**：
- `mbleft/mbright/mbsideleft/mbsideright` 分散的布尔标志位，缺乏统一的多相机数据结构
- 侧视匹配逻辑中的 TODO (`src/Frame.cc:1336`): "TODO: for different setup, sideward camera may have sufficient overlap"
- 无自动 rectification，依赖外部工具链
- 多相机特征数量的配置分散在 ORBextractor 构造函数而非 YAML

### 7.4 适用场景

**适合**：
- 前视双目 + 侧视扩展的固定多相机阵列 (Hilti 2022 典型场景)
- 中低速运动（侧视单目三角化需要帧间视差）
- 已知精确外参的标定平台

**不适合**：
- 相机布局不可预测的动态 Rig
- 高角速度/高角加速度场景（侧视特征来不及三角化）
- 需要全部相机实时深度估计（如稠密 SLAM）

---

## 8. 对 phad_fusion 的关键参考

### 8.1 值得借鉴的设计

| OpenMAVIS 做法 | phad_fusion 对应设计 |
|---|---|
| `cam_idx` 参数 + `GetFeaturesInArea(selectedCamera)` (`Frame.h:131`) | `CameraRig::camera_id` + 特征查询转发 |
| `GeometricCamera` 抽象 + `project()`/`unproject()` (`CameraModels/`) | 统一 `CameraModel` trait |
| `KeyFrame::GetPose(cameraID)` 链式变换 (`KeyFrame.cc:156-164`) | `Rig::transform_chain.eval(camera_id)` |
| 每个相机独立 `mGrid[4]` (`Frame.cc:419-463`) | `Camera::feature_grid` |
| `thread::thread*4` 并⾏提取 (`Frame.cc:1546-1553`) | `Rig::extract()` 内 parallel_for |
| `EdgeMono(cam_idx)`, `EdgeStereo(cam_idx)` (`G2oTypes.h:347,430`) | `VisualFactor<CamIdx>` 模板 |
| `ImuCamPose` 中 `Rcw[cam]/tcw[cam]` 自动推导 (`G2oTypes.cc:63-94`) | CameraRig 的 Factor 构造中 `transform_to(world, cam_id)` |

### 8.2 需要避免的陷阱

1. **硬编码的匹配策略** (`src/Frame.cc:1608-1609`)：
   OpenMAVIS 硬编码"侧视不相匹配"。phad_fusion 应设计为**配置驱动**：YAML 中列出 `stereo_pairs: [{0,1}, {0,2}]`，由 Rig 根据配置执行匹配。

2. **初始化忽略侧视** (`src/Tracking.cc:2567-2589`)：
   如果相机 Rig 中所有相机都有视场重叠，应全部参与初始化 MapPoint 创建（包括侧视相机与其他相机的立体匹配）。

3. **4相机硬编码上限**：
   使用 `std::vector<Camera>` 替代 `mpCamera/mpCamera2/mpCamera3/mpCamera4` 分散指针。

4. **信息矩阵特征值裁剪** (`src/G2oTypes.cc:658-662`)：
   ```cpp
   Eigen::SelfAdjointEigenSolver<Matrix9d> es(Info);
   auto eigs = es.eigenvalues();
   for(int i=0;i<9;i++) if(eigs[i]<1e-12) eigs[i]=0;
   ```
   预积分协方差的逆需要对接近零的特征值做归零处理，否则 g2o/GTSAM 因半正定性缺失崩溃。phad_fusion 的 IMU 预积分因子构造中需要同样的安全检查。

5. **Lowe's ratio 阈值的不一致**：
   - `ComputeStereoFishEyeMatches`：0.7（行 1310）
   - `ComputeMultiFishEyeMatches`：0.8（行 1355）
   
   多相机时因特征分散而放宽了阈值，phad_fusion 应根据相机对之间的**视场重叠率**自适应调整 ratio。

### 8.3 建议的 phad_fusion CameraRig 数据模型

```
CameraRig {
    cameras: Vec<Camera>            // 动态数量，每个带内参+外参
    stereo_pairs: Vec<(id1, id2)>  // 配置驱动，替代硬编码
    T_body_imu: SE3                // IMU→Body (与OpenMAVIS的Tcb/Tbc对应)
    feature_grids: Vec<Grid>       // 每个Camera独立
    extractors: Vec<ORB>           // 自适应特征数（重叠少→特征多）
}
```

---

## 9. 附录：关键文件索引

| 文件 | 关键内容 | 行号 |
|---|---|---|
| `include/System.h:98` | `IMU_MULTI=6` sensor 定义 | 98 |
| `include/System.h:131` | `TrackMulti()` 声明 | 131 |
| `include/Tracking.h:79` | `GrabImageMulti()` 声明 | 79 |
| `include/Tracking.h:211` | `MultiInitialization()` 声明 | 211 |
| `include/Tracking.h:271-273` | 4路 ORBextractor 成员 | 271-273 |
| `include/Tracking.h:363-364` | 4路 `mpCamera` 成员 | 363-364 |
| `include/Tracking.h:368-370` | 3组外参 `mTlr/mTlsl/mTlsr` | 368-370 |
| `include/Frame.h:79` | `ExtractORB(flag)` 4路分发 | 79 |
| `include/Frame.h:260-261` | `mvKeys/mvKeys[Right/SideLeft/SideRight]` | 260-261 |
| `include/Frame.h:369-371` | `Nsideleft/Nsideright/mono*` | 369-371 |
| `include/Frame.h:388-394` | 4路网格数组 | 388-394 |
| `include/Frame.h:399-404` | 多相机构造函数签名 | 399-404 |
| `include/G2oTypes.h:130` | `VertexPose` (6DoF IMU) | 130 |
| `include/G2oTypes.h:342-388` | `EdgeMono(cam_idx)` | 342-388 |
| `include/G2oTypes.h:425-463` | `EdgeStereo(cam_idx)` | 425-463 |
| `include/G2oTypes.h:495-544` | `EdgeInertial` (9DoF IMU边) | 495-544 |
| `include/ImuTypes.h:143` | `Preintegrated` 类声明 | 143 |
| `include/Settings.h:74-77` | 4路 Camera getter | 74-77 |
| `src/Tracking.cc:1540` | `GrabImageMulti()` 实现 | 1540 |
| `src/Tracking.cc:1718` | `PreintegrateIMU()` | 1718 |
| `src/Tracking.cc:2539` | `MultiInitialization()` 实现 | 2539 |
| `src/Frame.cc:465` | `ExtractORB()` 4路调度 | 465 |
| `src/Frame.cc:1327` | `ComputeMultiFishEyeMatches()` | 1327 |
| `src/Frame.cc:1514` | 多相机 Frame 构造函数 | 1514 |
| `src/ImuTypes.cc:182` | `IntegrateNewMeasurement()` SE2(3) | 182 |
| `src/G2oTypes.cc:30` | `ImuCamPose(KeyFrame*)` 多相机初始化 | 30 |
| `src/G2oTypes.cc:565` | `EdgeInertial::computeError()` | 565 |
| `src/G2oTypes.cc:668` | `EdgeInertialGS::computeError()` (带重力尺度) | 668 |
| `src/KeyFrame.cc:156` | `GetPose(cameraID)` 链式变换 | 156 |
| `Examples/Multi-Inertial/multi_inertial_euroc.cc` | 数据集播放示例 | 全文 |

---

**分析报告结束。** 分析覆盖 OpenMAVIS 全部 `.cpp/.h/.cc` 源文件 30+ 个，总计 ~15000 行代码。

---

## 10. 数据管线

### 10.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | ROS话题/接口 | 负责模块 |
|--------|------|----------|-------------|----------|
| 相机1 (前视左) | 10-30 Hz | `cv::Mat` (BGR→Gray) | `cam1/data` → `System::TrackMulti` | `Tracking::GrabImageMulti` |
| 相机0 (前视右) | 10-30 Hz | `cv::Mat` (BGR→Gray) | `cam0/data` → `System::TrackMulti` | `Tracking::GrabImageMulti` |
| 相机4 (侧视左) | 10-30 Hz | `cv::Mat` (BGR→Gray) | `cam4/data` → `System::TrackMulti` | `Tracking::GrabImageMulti` |
| 相机3 (侧视右) | 10-30 Hz | `cv::Mat` (BGR→Gray) | `cam3/data` → `System::TrackMulti` | `Tracking::GrabImageMulti` |
| IMU (6 轴) | 100-200 Hz | `IMU::Point{a[3], w[3], t}` | `imu0/data` → `Tracking::GrabImuData` | `IMU::Preintegrated` (SE2(3) 版) |

### 10.2 相机数据处理管线

#### 原始数据
- **图像格式**: `cv::Mat` → `cv::cvtColor(BGR2GRAY)` 灰度转换 (行 1553-1588)
- **无 rectification**: 代码明确注释 "Images must be synchronized and rectified"，仅 BGR→Gray
- **相机模型**: `GeometricCamera` → `Pinhole` / `KannalaBrandt8` (鱼眼), YAML 配置
- **外参** (链式推导): 所有相机通过主相机 cam1 推导:
  - `mpCamera` (cam1, 主) → `mpCamera2` (cam0, 通过 `Tlr`) → `mpCamera3` (cam4, 通过 `Tlsl`) → `mpCamera4` (cam3, 通过 `Tlsr`)
  - `mTlr`/`mTlsl`/`mTlsr` 离线标定，不在线估计

#### 4 路并行 ORB 提取 (`Frame::ExtractORB`, `src/Frame.cc:465-516`)
```
threadLeft       → mpORBextractorLeft (nFeatures)
threadRight      → mpORBextractorRight (nFeatures)
threadSideLeft   → mpORBextractorSideLeft (nFeatures*2)
threadSideRight  → mpORBextractorSideRight (nFeatures*2)
```

- ORB 提取算法与 ORB-SLAM3 一致 (FAST + 四叉树 + IC_Angle + Steered BRIEF)
- 侧视相机**2 倍特征**补偿单目观测稀疏性

#### 每相机独立网格
- `mGrid[64][48]` (前视左), `mGridRight[64][48]`, `mGridSideLeft[64][48]`, `mGridSideRight[64][48]`
- 特征索引语义分配 (行 441-461): `[0,Nleft)`→left, `[Nleft,Nleft+Nright)`→right, ...
- `GetFeaturesInArea(x,y,r,selectedCamera)` 按相机查询邻域

#### 前视立体匹配 (`ComputeMultiFishEyeMatches`, `src/Frame.cc:1327-1381`)
1. 仅匹配重叠区域特征 (使用 `monoLeft/monoRight` 分界线)
2. `cv::BFMatcher(NORM_HAMMING)` + `knnMatch(k=2)`
3. **Lowe's ratio test**: `ratio < 0.8` (行 1355)
4. `KannalaBrandt8::TriangulateMatches()` 三角化
5. 深度阈值: `depth > 0.0001f`

**关键假设** (行 1608): "侧视相机与前视无显著重叠" —— 侧视特征**不做任何跨相机匹配**，纯单目使用，依赖多帧运动视差三角化。

#### 跟踪模式
- 运动模型+投影匹配: `PredictStateIMU()` → `SearchByProjection()`
- 参考关键帧+BoW 匹配: `SearchByBoW()` → `PoseOptimization()`
- 重定位: MLPnP (置信度 0.99, 300 迭代, 6 点, 0.5px, χ²(2)=5.991)

### 10.3 IMU 数据处理管线 (SE2(3) 精确预积分)

#### 原始数据
- **测量**: `IMU::Point {Eigen::Vector3f a, Eigen::Vector3f w, double t}`
- **噪声**: 配置 σ_a, σ_g, σ_ba, σ_bg
- **重力**: `[0,0,-9.81]`

#### 缓冲
- `Tracking::GrabImuData(IMU::Point)` → `mlQueueImuData`
- 图像帧传入时附带 `vImuMeas` (时间窗口内)

#### SE2(3) 精确积分 (`Preintegrated::IntegrateNewMeasurement`, `src/ImuTypes.cc:182-257`)
```
θ = |w| (角速度幅值)
J1 = dt·I + (1-cos(dt·θ))/θ²·[w]× + (dt·θ-sin(dt·θ))/θ³·[w]×²
J2 = dt²/2·I + (dt·θ-sin(dt·θ))/θ³·[w]× + (dt²·θ²/2+cos(dt·θ)-1)/θ⁴·[w]×²

dP += dV·dt + dR·J2·acc       // 精确位置 (vs ORB-SLAM3: dP+=dV·dt+dR·0.5·acc·dt²)
dV += dR·J1·acc                // 精确速度 (vs ORB-SLAM3: dV+=dR·acc·dt)
```

**关键区别**: ORB-SLAM3 原始使用零阶角速度近似 (`J1≈dt·I, J2≈0.5·dt²·I`)，SE2(3) 的 J1、J2 精确捕捉角速度对位置/速度的耦合效应，高角速度场景显著提升精度。

#### 协方差传播 (9×9)
```
A = [[-dR^T·J·(a·dt)×, O, O], [-J·dt²/2, I·dt, O], [J·dt, O, I]]
B = [[J·dt, O], [dR^T·J·dt²/2, J·dt²/2], [dR^T·J·dt, J·dt]]
Σ = A·Σ·A^T + B·diag(σ_g²,σ_a²)·B^T
```

#### 状态传播 (`PredictStateIMU`)
```
R_wb_k = R_wb_{k-1}·dR_corrected
P_wb_k = P_wb_{k-1} + V_{k-1}·Δt + 0.5·G·Δt² + R_{k-1}·dP_corrected
V_wb_k = V_wb_{k-1} + G·Δt + R_{k-1}·dV_corrected
```

### 10.4 算法消费：g2o 多相机因子构造

#### 多相机位姿基类 (`ImuCamPose`, `src/G2oTypes.cc:30-99`)
```
ImuCamPose(KeyFrame*):
  检查 mpCamera4 存在 → 确定相机数 (4/2/1)
  Rcw[cam] = R_ic[cam]·R_wi, tcw[cam] = R_ic[cam]·twi + t_ic[cam]
  pCamera[cam] 存储各相机模型
  投影: project(X_w, cam_idx) = pCamera[cam]->project(Rcw[cam]·X_w + tcw[cam])
```

#### 视觉边 (per-camera)
- `EdgeMono(cam_idx)`: 2 维重投影 `r = [u,v]_obs - [u,v]_proj`，信息矩阵 `1/σ²(level)`
- `EdgeStereo(cam_idx)`: 3 维 `r = [uL,v,uR]`，信息矩阵同上
- 每个 `MapPoint` 的观测存储为 `map<KF*, tuple<idx0,idx1,idx2,idx3>>` (-1 = 该相机未观测)

#### IMU 边
- `EdgeInertial`: 9 维 (r_R+3, r_V+3, r_P+3)，连接 `Pose_i(6), Vel_i(3), BiasG(3), BiasA(3), Pose_j(6), Vel_j(3)`
- `EdgeInertialGS` (初始化专用): 额外含 `VertexGDir(2)` + `VertexScale(1)`
- `EdgeGyroRW`/`EdgeAccRW`: 偏置随机游走 `r = bias_j - bias_i`，信息矩阵 `1/(σ_rw²·Δt)·I`
- 信息矩阵: 预积分协方差逆 `Σ^{-1}`，特征值裁剪 (`<1e-12` → 0)

#### g2o 配置
- `BlockSolver_6_3`/`BlockSolverX`, **Levenberg-Marquardt**
- 鲁棒核: Huber，视觉 `delta=√5.991`, 惯性 `delta=√7.815`

### 10.5 跨传感器协同

#### 时间同步
- 图像帧时间戳为主基准
- IMU 在 `multi_inertial_euroc.cc:194-201` 按时间窗口 `[last_t, curr_t]` 收集

#### 初始化管线 (`MultiInitialization`, `src/Tracking.cc:2539-2612`)
1. 条件: `N>500` 特征 & 有效 IMU 预积分
2. **首帧 IMU 坐标系 = 世界原点**: `Rwb=R(T_cb)`, `twb=t(T_cb)`, `Vwb=0`
3. 地图点: **仅前视左→右匹配特征** (`mvLeftToRightMatch[i]≠-1` → 三角化)
4. 侧视相机**完全未参与初始化**
5. IMU 无加速度激励检查 (不同于 `StereoInitialization` 的 `(avgA).norm()<0.5` 检查)

#### IMU 初始化 (LocalMapping 线程)
- ≥N 个 KF 后触发
- `Optimizer::InertialOptimization`: 估计 `Rwg`, `scale`, `bg`, `ba` (EdgeInertialGS)
- 后续 FullInertialBA 全局精调

#### 降级与异常处理
- 跟踪丢失 → 新建 Atlas 地图 + 保留旧地图
- 回环检测 (继承 ORB-SLAM3): DBoW2 + Sim3 + 位姿图
- 地图点剔除: `GetFoundRatio()<0.25` / KF 冗余 (`KeyFrameCulling`)
- 边缘化: `Optimizer::Marginalize` — Schur complement 构建先验边 `EdgePriorPoseImu`