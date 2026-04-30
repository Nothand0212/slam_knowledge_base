# ORB-SLAM3 源码级深度剖析报告

> 作者：SLAM 算法研究员
> 代码仓库：/home/lin/Projects/lin_ws/slam_ws/ORB_SLAM3/
> 分析日期：2026-04-28

---

## 1. 数据接收与预处理

### 1.1 系统入口点 (`src/System.cc:41-242`)

ORB-SLAM3 的 `System` 类（`include/System.h:83-265`）是整个 SLAM 系统的唯一入口。构造函数 `System::System()`（`src/System.cc:41-242`）接收以下关键参数：

- `strVocFile`: ORB 词袋词汇表文件路径
- `strSettingsFile`: YAML 配置文件路径
- `sensor`: 传感器类型枚举 `eSensor`（`include/System.h:87-94`）

**六种传感器模式**（`include/System.h:87-94`）：
| 枚举值 | 含义 |
|--------|------|
| `MONOCULAR=0` | 纯单目视觉 |
| `STEREO=1` | 纯双目视觉 |
| `RGBD=2` | 纯 RGB-D |
| `IMU_MONOCULAR=3` | 单目+IMU 惯导 |
| `IMU_STEREO=4` | 双目+IMU 惯导 |
| `IMU_RGBD=5` | RGB-D+IMU 惯导 |

**构造函数核心流程**（`src/System.cc:41-242`）：
1. 打开并解析 YAML 配置文件 (`cv::FileStorage`, line 70-98)
2. 加载 ORB 词汇表 (`mpVocabulary->loadFromTextFile(strVocFile)`, line 118)
3. 创建关键帧数据库 `KeyFrameDatabase` (line 128)
4. 创建 Atlas 多地图管理器 (line 132)
5. 若为 IMU 模式，调用 `mpAtlas->SetInertialSensor()` (line 182)
6. 创建可视化绘制器 `FrameDrawer` 和 `MapDrawer` (line 185-186)
7. **创建 Tracking 线程**（主线程运行，line 191-192）
8. **创建并启动 LocalMapping 线程** (`mptLocalMapping = new thread(&LocalMapping::Run, ...)`, line 197)
9. **创建并启动 LoopClosing 线程** (line 214)
10. 设置线程间互指针 (`SetLocalMapper/SetLoopClosing`, line 217-224)
11. **创建并启动 Viewer 线程** (line 233)

**关键设计决策**：Tracking 线程运行在主调用线程，不在独立线程中，而 LocalMapping、LoopClosing、Viewer 分别运行在独立 `std::thread` 中。

### 1.2 图像校正/预处理

**双目模式 TrackStereo** (`src/System.cc:244-326`)：
- 若配置需要校正 (`settings_->needToRectify()`)，使用 `cv::remap()` 配合 M1l/M2l/M1r/M2r 校正矩阵进行在线双目校正（line 254-260）
- 若配置需要缩放 (`needsToResize()`)，使用 `cv::resize()` 调整图像尺寸（line 262-264）
- 否则直接 `clone()` 使用原始图像

**RGB-D 模式 TrackRGBD** (`src/System.cc:328-397`)：
- 支持图像缩放 (`cv::resize()`, line 340-344)
- RGB 图像和深度图同步处理

**单目模式 TrackMonocular** (`src/System.cc:399-474`)：
- 仅进行可选的图像缩放 (`cv::resize()`, line 416-418)

### 1.3 IMU 数据接入

在 VIO 模式下，每帧图像数据传入时附带 IMU 测量向量 `vector<IMU::Point>`：
- 双目 VIO: `src/System.cc:311-313` 调用 `mpTracker->GrabImuData(vImuMeas[i_imu])`
- RGB-D VIO: `src/System.cc:386-388`
- 单目 VIO: `src/System.cc:462-464`

`IMU::Point` 类（`include/ImuTypes.h:46-59`）存储三维加速度 `a`（`Eigen::Vector3f`），三维角速度 `w`（`Eigen::Vector3f`），时间戳 `t`（`double`）。

IMU 数据在 Tracking 中被缓冲到 `mlQueueImuData` 队列（`include/Tracking.h:238`），然后在 `PreintegrateIMU()` 中完成预积分（`src/Tracking.cc:227` 处声明，`Track()` 中 line 1874 调用）。

---

## 2. 特征提取与跟踪

### 2.1 ORBextractor 类详情

ORB 特征提取器定义在 `include/ORBextractor.h:43-111`，实现在 `src/ORBextractor.cc:409-1197`。

#### 2.1.1 构造函数与参数（`src/ORBextractor.cc:409-469`）

```cpp
ORBextractor(int nfeatures, float scaleFactor, int nlevels,
             int iniThFAST, int minThFAST);
```

**默认典型参数**（如 EuRoC 示例配置）：
- `nfeatures = 1200`：每帧提取的最大特征点数
- `scaleFactor = 1.2`：图像金字塔缩放因子
- `nlevels = 8`：金字塔层数
- `iniThFAST = 20`：初始 FAST 角点检测阈值
- `minThFAST = 7`：最小 FAST 检测阈值（兜底）

**金字塔尺度因子预计算**（`src/ORBextractor.cc:414-430`）：
- `mvScaleFactor[0] = 1.0f`，`mvScaleFactor[i] = mvScaleFactor[i-1] * scaleFactor`
- `mvLevelSigma2[i] = mvScaleFactor[i]^2`（用于计算信息矩阵）
- `mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i]`

**每层特征数量分配**（`src/ORBextractor.cc:434-445`）：
```cpp
float factor = 1.0f / scaleFactor;
float nDesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - pow(factor, nlevels));
for (int level = 0; level < nlevels-1; level++) {
    mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
    nDesiredFeaturesPerScale *= factor;  // 逐层递减
}
mnFeaturesPerLevel[nlevels-1] = max(nfeatures - sumFeatures, 0);
```
等比数列分配，越上层（分辨率越低）分配的特征越少。

**BRIEF采样图案预计算**（`src/ORBextractor.cc:447-449`）：
- 从硬编码的 `bit_pattern_31_[256*4]` 数组（line 149-407）加载 512 个采样点对
- 这是一个 31×31 patch 内的 256 对像素坐标，通过贪婪搜索最大化方差和最小化相关性得到

**圆形 patch 边界预计算**（`src/ORBextractor.cc:451-468`）：
- 用于 IC_Angle 方向计算，预计算半 patch 内每行的最大列坐标 `umax[]`

#### 2.1.2 图像金字塔构建（`src/ORBextractor.cc` `ComputePyramid`）

`ComputePyramid()` 函数（`ORBextractor.h:87` 声明）将输入图像逐层缩放存入 `mvImagePyramid`：
- 第 0 层 = 原始图像
- 第 i 层 = 第 i-1 层按 `scaleFactor` 缩放（使用 OpenCV `resize` 或等效方法）

#### 2.1.3 FAST 特征点检测与网格划分（`src/ORBextractor.cc:781-895`）

`ComputeKeyPointsOctTree()`（`src/ORBextractor.cc:781`）是主要实现：

**网格划分策略**（line 797-803）：
```cpp
const float W = 35;  // 每个网格单元宽度 ≈35 像素
const int nCols = width / W;
const int nRows = height / W;
const int wCell = ceil(width / nCols);
const int hCell = ceil(height / nRows);
```
图像被均匀划分为约 35×35 像素的网格单元。

**FAST 检测的双阈值策略**（line 826-859）：
1. 首先使用 `iniThFAST`（高阈值，如 20）在每个网格单元内检测 FAST 角点
2. 若某网格检测不到角点，降级使用 `minThFAST`（低阈值，如 7）重新检测
3. 检测出的角点坐标加上网格偏移后存入 `vToDistributeKeys`

#### 2.1.4 四叉树分布（`src/ORBextractor.cc:555-779`）

`DistributeOctTree()` 将提取的特征点均匀分布到图像上，避免扎堆：

1. **初始节点划分**（line 559-579）：根据图像宽高比确定初始水平分块数，将特征点分配到各初始节点
2. **迭代细分**（line 590-754）：
   - 若节点内只有一个特征点，标记 `bNoMore=true`（不再细分）
   - 否则若节点内特征点数量 ≥ 期望数 或 不再缩小，则停止
   - 否则将该节点用 `DivideNode()` 分为 4 个子节点（四叉树）
   - 重复直到节点数满足目标特征数或无法进一步细分
3. **保留最佳点**（line 758-776）：每个叶子节点中仅保留 Harris 响应值最高的特征点

#### 2.1.5 方向计算（`src/ORBextractor.cc:76-103`）

`IC_Angle()` 函数使用**灰度质心法**（Intensity Centroid）计算方向：

```cpp
static float IC_Angle(const Mat& image, Point2f pt, const vector<int> & u_max)
```

- 在 31×31 的圆形 patch（半径 `HALF_PATCH_SIZE=15`）内
- 计算强度矩：`m_10 = Σ u * I(u,v)`，`m_01 = Σ v * I(u,v)`
- 方向角：`θ = atan2(m_01, m_10)`（使用 `fastAtan2`，line 102）
- 利用预计算的 `umax[]` 避免逐点判断是否在圆内，高效访问

#### 2.1.6 ORB 描述子计算（`src/ORBextractor.cc:107-146`）

`computeOrbDescriptor()` 生成**旋转感知的 steered BRIEF** 32 字节描述子：

```cpp
static void computeOrbDescriptor(const KeyPoint& kpt, const Mat& img,
                                  const Point* pattern, uchar* desc)
```

**旋转补偿**（line 111-112）：
```cpp
float angle = (float)kpt.angle * factorPI;  // 度转弧度
float a = (float)cos(angle), b = (float)sin(angle);
```

**采样坐标旋转变换**（`GET_VALUE` 宏，line 117-119）：
```cpp
#define GET_VALUE(idx) \
    center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
           cvRound(pattern[idx].x*a - pattern[idx].y*b)]
```
将预定义的 256 对采样点按特征点方向旋转后进行像素值比较。

**描述子编码**（line 122-143）：
- 每次处理 16 个采样点对（通过 `pattern += 16` 步进）
- 8 次比较结果打包为 1 字节：`desc[i] = (uchar)val`
- 重复 32 次 → 产生 32 字节（256 位）的描述子
- 描述子存入 `cv::Mat`，每行代表一个特征点的 32 字节描述子

#### 2.1.7 描述子计算调度（`src/ORBextractor.cc:1077-1084`）

`computeDescriptors()` 遍历所有特征点，在每个金字塔层级上分别调用 `computeOrbDescriptor()`，生成 `descriptors`（`Mat::zeros(keypoints.size(), 32, CV_8UC1)`）。

### 2.2 operator() 主入口（`src/ORBextractor.cc:1086-1197`）

```cpp
int ORBextractor::operator()(InputArray _image, InputArray _mask,
    vector<KeyPoint>& _keypoints, OutputArray _descriptors,
    std::vector<int>& vLappingArea)
```

执行顺序：
1. `ComputePyramid(image)` — 建金字塔
2. `ComputeKeyPointsOctTree(allKeypoints)` — 每层 FAST + 四叉树分布
3. 逐层 `computeOrientation(mvImagePyramid[level], ...)` — 计算方向
4. 降采样后重提取（line 1101-1150）处理左右图像重叠区域（鱼眼双目特化）
5. 逐层 `computeDescriptors(...)` — 计算描述子
6. 将所有金字塔层特征点合并到输出 `_keypoints` 和 `_descriptors`

### 2.3 Frame 创建和特征管理

`Frame` 类（`include/Frame.h:53-370`）封装了每一帧的所有数据：

**三种构造函数**（`include/Frame.h:62-68`）：
- 双目：`Frame(imLeft, imRight, timeStamp, ...)` line 62
- RGB-D：`Frame(imGray, imDepth, timeStamp, ...)` line 65
- 单目：`Frame(imGray, timeStamp, ...)` line 68

**关键成员变量**（`include/Frame.h:189-364`）：
- `mvKeys`, `mvKeysRight`：原始特征点坐标（左右目）
- `mvKeysUn`：去畸变后的特征点坐标
- `mvpMapPoints`：每个特征点关联的地图点指针
- `mvuRight`：双目匹配的右目 u 坐标（单目点为负值）
- `mvDepth`：特征点深度
- `mDescriptors`, `mDescriptorsRight`：ORB 描述子
- `mBowVec`, `mFeatVec`：词袋向量和特征向量
- `mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]`：网格索引（`FRAME_GRID_ROWS=48`, `FRAME_GRID_COLS=64`），用于快速邻域搜索

**特征网格化** `AssignFeaturesToGrid()`（`include/Frame.h:317` 声明）：
- 将图像划分为 48×64 的固定网格
- 每个网格单元存储落入其中的特征点索引
- 快速邻域搜索通过 `GetFeaturesInArea(x, y, r)` 实现（line 112），仅查询相关网格单元

**KeyFrame 类**（`include/KeyFrame.h:52-541`）继承自 Frame，额外维护：
- 共视图（Covisibility Graph）：`mConnectedKeyFrameWeights`、`mvpOrderedConnectedKeyFrames`
- 生成树（Spanning Tree）：`mpParent`、`mspChildrens`
- 回环边（Loop Edges）：`mspLoopEdges`
- IMU 预积分：`mpImuPreintegrated`、`mPrevKF`、`mNextKF`

---

## 3. 初始化

### 3.1 单目初始化（`src/Tracking.cc` MonocularInitialization → `src/TwoViewReconstruction.cc`）

单目初始化通过 `Tracking::MonocularInitialization()` 实现，核心依赖 `TwoViewReconstruction` 类（`include/TwoViewReconstruction.h`，实现于 `src/TwoViewReconstruction.cc:32-929`）。

**TwoViewReconstruction::Reconstruct()**（`src/TwoViewReconstruction.cc:41-129`）：

1. **匹配准备**（line 50-64）：整理两帧之间的特征匹配对 `mvMatches12`
2. **RANSAC 采样集生成**（line 69-98）：为每次 RANSAC 迭代预生成 8 个随机采样点索引（8点法）
3. **并行计算 H 和 F**（line 105-110）：
   ```cpp
   thread threadH(&TwoViewReconstruction::FindHomography, ...);
   thread threadF(&TwoViewReconstruction::FindFundamental, ...);
   threadH.join(); threadF.join();
   ```
   两个线程**并行**计算基础矩阵和单应矩阵
4. **模型选择**（line 112-128）：
   - 计算得分比 `RH = SH / (SH + SF)`（line 114）
   - `RH > 0.50` → 选择单应矩阵 H（适用于平面场景或纯旋转）
   - 否则 → 选择基础矩阵 F（适用于一般场景）
   - 分别调用 `ReconstructH()` 或 `ReconstructF()` 进行三角化

**FindHomography**（`src/TwoViewReconstruction.cc:131-179`）：
- 归一化坐标后（`Normalize()`, line 138-139）
- 使用 8 点法 `ComputeH21()` 计算单应矩阵
- 通过 `CheckHomography()` 评估对称转移误差（像素阈值 `mSigma`）
- 取最高得分模型

**FindFundamental**（`src/TwoViewReconstruction.cc:182-226`）：
- 归一化坐标后使用 8 点法 + SVD（秩2约束）计算基础矩阵
- 通过 `CheckFundamental()` 评估极线距离误差
- 取最高得分模型

**三角化**：`ReconstructH()`/`ReconstructF()` 中通过 `GeometricTools::Triangulate()`（`src/GeometricTools.cc`）对每个内点匹配进行线性三角化，检查 parallax、正深度和重投影误差（像素阈值 4.0）。

**CreateInitialMapMonocular**（`src/Tracking.cc`）：将三角化得到的 3D 点创建为 `MapPoint`，将两个初始化帧创建为 `KeyFrame`，构造初始共视图和生成树。

### 3.2 双目初始化（`src/Tracking.cc` `Tracking::StereoInitialization()`）

双目模式初始化**极其简单**：
- 直接从左右目视差（disparity）通过 `UnprojectStereo()` 计算当前帧所有特征点的 3D 坐标
- 将当前帧直接作为第一个关键帧
- 将所有有深度的 3D 点创建为 `MapPoint`
- 无需两帧匹配和三角化

### 3.3 IMU 初始化（`src/LocalMapping.cc:1173-1522`）

`LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)`（`src/LocalMapping.cc:1173`）是 ORB-SLAM3 IMU 初始化的核心：

**前置条件检查**（line 1178-1211）：
- 单目：至少 10 个 KF，时间跨度 ≥ 2.0s
- 双目：至少 10 个 KF，时间跨度 ≥ 1.0s

**步骤 1：重力方向估计**（line 1226-1253）：
- 遍历所有 KF 的 IMU 预积分
- `dirG = Σ (Rwb * ΔV)` 累加加速度方向
- 归一化后，与假设方向 `(0,0,-1)` 计算旋转 `Rwg = SO3::exp(v×g/||v|| * acos(dot))`

**步骤 2：惯性-only 优化**（line 1267）：
```cpp
Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale,
    mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);
```
优化变量：重力方向 `Rwg`（2DoF）、尺度 `mScale`、陀螺仪偏置 `mbg`、加速度计偏置 `mba`、所有 KF 的速度。
这是论文 ICRA 2020 [IMU-Initialization] 的核心。

**步骤 3：尺度/重力应用**（line 1281-1293）：
- 若非单目或尺度偏离 1.0，调用 `Map::ApplyScaledRotation()` 对地图应用 `(Twg, scale)` 变换
- 标记所有 KF 的 `bImu = true`

**步骤 4：视觉-惯性 Full BA**（line 1304-1310）：
- 若 `bFIBA=true`，执行 `Optimizer::FullInertialBA()`（100 次迭代）
- 固定局部 KFs，优化全局位姿、速度、偏置、地图点

**步骤 5：地图更新**（line 1317-1376）：
- 遍历生成树传播 GBA 后的位姿修正（`mTcwGBA`, `mVwbGBA`, `mBiasGBA`）
- 更新地图点世界坐标

**阶段性初始化**（`src/LocalMapping.cc:200-242`）：
- `mTinit > 5.0f`：执行 `SetIniertialBA1()` → `InitializeIMU(1.f, 1e5)`
- `mTinit > 15.0f`：执行 `SetIniertialBA2()` → `InitializeIMU(0.f, 0.f)`
- 尺度精化 `ScaleRefinement()` 在 25s/35s/45s... 时执行

---

## 4. 逐帧状态估计

### 4.1 Tracking 线程详细流程（`src/Tracking.cc:1794-` `Tracking::Track()`）

**Track() 主函数**（`src/Tracking.cc:1794`）：

```
Track():
  1. Check timestamp (line 1820-1856)
  2. PreintegrateIMU() (line 1874)
  3. Check map change (line 1888-1896)
  4. If NOT_INITIALIZED:
     - Stereo/RGBD: StereoInitialization()
     - Mono: MonocularInitialization()
  5. If OK/LOST/RECENTLY_LOST:
     - CheckReplacedInLastFrame()
     - TrackWithMotionModel() or TrackReferenceKeyFrame() or PredictStateIMU()
     - TrackLocalMap() (if OK)
     - NeedNewKeyFrame() → CreateNewKeyFrame()
  6. Update velocity model mVelocity
  7. Store relative pose for trajectory saving
```

**三种跟踪模式切换逻辑**（`src/Tracking.cc:1933-2076`）：

| 状态 | 跟踪策略 | 代码位置 |
|------|----------|----------|
| `OK` | (1) 若速度模型可用且非刚重定位 → `TrackWithMotionModel()` (line 1953); (2) 否则 → `TrackReferenceKeyFrame()` (line 1948); (3) (1) 失败则回退到 (2) (line 1954-1955) |
| `RECENTLY_LOST` | IMU 预测 → `PredictStateIMU()` (line 1989), 超时则进入重定位 `Relocalization()` (line 2015) |
| `LOST` | `Relocalization()` (line 2076) |

**Motion Model 跟踪** `TrackWithMotionModel()`：
- 使用恒定速度模型 `mVelocity` 预测当前帧位姿
- 通过 `ORBmatcher::SearchByProjection()` 将上一帧的地图点投影到当前帧进行匹配
- 调用 `Optimizer::PoseOptimization(Frame*)` 进行**运动-only BA**（仅优化相机位姿，固定地图点）

**Reference KF 跟踪** `TrackReferenceKeyFrame()`：
- 将参考关键帧的地图点通过 `BoW` 加速匹配（`SearchByBoW()`）
- 同样进行运动-only BA 位姿优化

**重定位** `Relocalization()`：
- 通过 `KeyFrameDatabase::DetectRelocalizationCandidates()` 使用 BoW 检测候选 KF
- 对每个候选 KF 使用 `ORBmatcher::SearchByBoW()` 获取 2D-3D 匹配
- 使用 **EPnP (MLPnPsolver)**（`src/MLPnPsolver.cpp`）计算初始位姿
- 执行 `Optimizer::PoseOptimization()` 精细优化

**PoseOptimization（运动-only BA）**（`src/Optimizer.cc` `Optimizer::PoseOptimization(Frame*)`）：
- 残差类型：3D 地图点投影到 2D 像素平面的**重投影误差**（双目还包括右目 u 坐标残差）
- 使用 g2o 的 `OptimizationAlgorithmLevenberg`（**LM 算法**）（`include/Optimizer.h:34`）
- 仅优化 6DoF 相机位姿，地图点固定
- 边缘类型：
  - `EdgeMonoOnlyPose`（`include/G2oTypes.h:390`）：2D 观测 vs 固定 3D 点 → 6DoF 优化
  - `EdgeStereoOnlyPose`（`include/G2oTypes.h:466`）：3D 观测（uL, vL, uR）vs 固定 3D 点 → 6DoF 优化

### 4.2 Local Map Tracking（`src/Tracking.cc` `TrackLocalMap()` + `SearchLocalPoints()`）

1. **UpdateLocalMap()**（line 216-218）：
   - `UpdateLocalKeyFrames()`：获取当前帧的共视 KFs + 邻居 KFs + 子 KFs 作为 Local KFs
   - `UpdateLocalPoints()`：收集所有 Local KFs 观测到的地图点作为 Local MapPoints

2. **SearchLocalPoints()**（line 221）：
   - 将 Local MapPoints 通过 `isInFrustum()` 检查（`include/Frame.h:103`）
   - 使用 `SearchByProjection()` 在局部窗口中匹配更多 2D-3D 对应

3. **再次 Pose Optimization**（`Optimizer::PoseOptimization()`）：
   - 使用更多的 3D-2D 匹配，更精细地优化当前帧位姿
   - 剔除 outlier（卡方分布 χ² 阈值 5.991（2D mono）、7.815（3D stereo））

### 4.3 关键帧决策（`src/Tracking.cc` `Tracking::NeedNewKeyFrame()` line 222）

创建新 KF 的条件（所有条件**必须全部满足**）：
1. 距上次重定位超过 1 帧
2. LocalMapping 线程空闲（`AcceptKeyFrames()`）或距上次 KF 超过 `mMaxFrames` 帧
3. 跟踪质量良好（`mnMatchesInliers > 15` 或 匹配点数量足够）
4. 至少经过了 `mMinFrames` 帧（默认 0）
5. 场景变化足够（至少跟踪到 15 个新地图点）或距上次 KF 超过 `mMaxFrames` 帧

### 4.4 LocalMapping 线程（`src/LocalMapping.cc` `Run()` line 64-282）

`LocalMapping::Run()` 主循环（`src/LocalMapping.cc:64-282`）：

```
while(1):
  1. SetAcceptKeyFrames(false)
  2. CheckNewKeyFrames() - 检查队列
  3. ProcessNewKeyFrame():
     - ComputeBoW()
     - AddObservation / UpdateNormalAndDepth / ComputeDistinctiveDescriptors
     - UpdateConnections() (Covisibility Graph)
     - Atlas->AddKeyFrame()
  4. MapPointCulling() - 剔除低质量地图点
  5. CreateNewMapPoints() - 通过临近 KF 三角化创建新地图点
  6. SearchInNeighbors() - 融合邻近 KF 的重复地图点
  7. Local BA (if no new KF pending):
     - IMU initialized → LocalInertialBA()
     - else → LocalBundleAdjustment()
  8. IMU Initialization (if applicable)
  9. KeyFrameCulling()
  10. -> InsertKeyFrame into LoopClosing queue
  11. SetAcceptKeyFrames(true)
```

**MapPointCulling**（`src/LocalMapping.cc:346-385`）剔除策略：
- `GetFoundRatio() < 0.25` → 标记为 bad
- 创建后 ≥ 2 KF 仍只有 ≤ 2（单目）或 ≤ 3（双目/RGB-D）观测 → 标记为 bad

**CreateNewMapPoints**（`src/LocalMapping.cc:388-711`）：
1. 取当前 KF 的 10~30 个最佳共视 KF（线 391-408，单目取 30，非单目取 10）
2. 对每个邻居 KF，验证基线条件：
   - 双目：`baseline < mb` → 跳过（基线太短）
   - 单目：`ratioBaselineDepth < 0.01` → 跳过（视差不足）
3. 使用 `ORBmatcher::SearchForTriangulation()`（线 466）进行 BoW 加速的匹配，施加极线约束
4. 三角化（`GeometricTools::Triangulate()`, line 585）
5. 检查：正深度、重投影误差（χ² 5.991/7.8）、尺度一致性（`ratioDist` 与 `ratioOctave` 的比值）
6. 创建 `MapPoint` 并记录双向观测

**KeyFrameCulling**（`src/LocalMapping.cc:902`）：
- 遍历当前 KF 的共视 KF
- 若某 KF 的 90% 以上地图点可被 ≥ 3 个其他 KF 在同一或更精细尺度观测到
- 则该 KF 冗余，标记为 bad

### 4.5 LoopClosing 线程

`LoopClosing::Run()`（`src/LoopClosing.cc:90-`）：

**回环检测**（调用 `NewDetectCommonRegions()` line 112）：
1. 通过 BoW 查询 `mpKeyFrameDB` 找到候选回环 KF（`DetectCommonRegionsFromBoW`）
2. 对候选 KF 进行几何验证：
   - `DetectAndReffineSim3FromLastKF()` 计算当前 KF 与候选 KF 间 3D-3D 匹配
   - `FindMatchesByProjection()` 进行投影匹配验证
   - 足够的匹配点 → 回环确认

**Sim3 计算**（`src/Sim3Solver.cc`）：
- RANSAC 计算两帧之间的 Sim3 变换（旋转 + 平移 + 尺度）
- 单目模式尺度可优化，双目/RGB-D 固定 `scale=1.0`

**Loop Fusion**（回环融合）：
- `CorrectLoop()`（`include/LoopClosing.h:141` 声明）
- 将回环匹配的地图点融合（`SearchAndFuse()`, line 137-138）
- 修正当前 KF 的共视 KF 位姿（Sim3 校正）

**Essential Graph Optimization**（`src/Optimizer.cc` `Optimizer::OptimizeEssentialGraph()`）：
- 在回环校正后对 **Essential Graph（生成树 + 共视图 + 回环边）** 执行**位姿图优化**（Pose Graph）
- 单目使用 7DoF Sim3，双目/RGB-D 使用 6DoF SE3
- 边缘约束：帧间相对位姿（由 Sim3 校正后的位姿计算）

**Global BA**（`LoopClosing::RunGlobalBundleAdjustment()`，`src/LoopClosing.cc`）：
- 启动**独立线程** `mpThreadGBA` 执行 `Optimizer::GlobalBundleAdjustemnt()`
- **Full BA**：优化**所有 KF 位姿 + 所有地图点位置**
- 使用 `BundleAdjustment()`（`src/Optimizer.cc:60-310`），g2o BlockSolver_6_3 + LM 算法
- 回环后地图一致性得到最终保证

---

## 5. Atlas 多地图系统

### 5.1 Atlas 创建与管理（`include/Atlas.h:49-166`，`src/Atlas.cc`）

**Atlas 类**（`include/Atlas.h`）管理多个 `Map` 对象：

- `mspMaps`：活跃地图集合（`std::set<Map*>`）
- `mspBadMaps`：标记为 bad 的地图集合
- `mpCurrentMap`：当前活跃地图

**核心方法**：
- `CreateNewMap()`（`src/Atlas.cc:58-77`）：将当前地图设为 Stored，新建 Map 作为当前地图
- `ChangeMap(Map*)`（line 79-89）：切换活跃地图
- `SetMapBad(Map*)`：标记地图为 bad（不删除，保留在 `mspBadMaps`）
- `RemoveBadMaps()`：清理 bad 地图
- `GetAllMaps()`（line 108）：返回所有地图列表

**地图序列化**（`include/Atlas.h:53-69`）：
- 使用 `boost::serialization` 保存/加载整个 Atlas
- 序列化内容包括：所有 Map 的备份向量 `mvpBackupMaps`、相机列表 `mvpCameras`、全局 ID 计数器
- `PreSave()` / `PostLoad()` 方法处理指针到 ID 的转换和恢复

### 5.2 多地图合并（Map Merging）

ORB-SLAM3 的回环检测模块（`LoopClosing`）不仅检测同一地图的回环，还检测**不同地图之间的重访区域**（Common Regions）：

- `mbMergeDetected`（`LoopClosing.h:198`）：标记是否检测到地图间重合
- `NewDetectCommonRegions()`（`LoopClosing.h:125`）：统一处理回环和合并检测
- `MergeLocal()` / `MergeLocal2()`（`LoopClosing.h:143-144`）：执行地图合并
  - 计算 Map1 → Map2 的 Sim3 变换
  - **焊接区域 BA**（Welding BA）：`Optimizer::LocalBundleAdjustment(pMainKF, vpAdjustKF, vpFixedKF)` 融合重叠区域
  - 合并后的 Essential Graph Optimization
  - 旧地图被标记为 bad

---

## 6. 局部优化与全局优化

### 6.1 Local BA（`src/Optimizer.cc` `Optimizer::LocalBundleAdjustment(KeyFrame*, ...)`，`include/Optimizer.h:57`）

**包含关键帧**：
- 当前 KF `pKF`
- `pKF` 的所有共视 KF（covisible，通过 `GetBestCovisibilityKeyFrames(num)` 获取）
- 以上 KF 观测到的所有地图点

**固定关键帧**（不被优化）：
- 其他能观测到这些地图点但不属于共视 KF 的关键帧

**优化变量**：
- 未被固定的 KF 6DoF 位姿
- 所有地图点的 3D 坐标（被 marginalized）

**G2O 配置**：
- `BlockSolver_6_3`：Pose 6维，Landmark 3维
- `LinearSolverEigen`：线性求解器
- `OptimizationAlgorithmLevenberg`：**Levenberg-Marquardt 算法**
- 鲁棒核：Huber Kernel，mono delta sqrt(5.99), stereo delta sqrt(7.815)
- 信息矩阵：按金字塔层级缩放 `invSigma2 = 1.0 / sigma2_level`

**残差**：
- `EdgeSE3ProjectXYZ`：单目 2D 重投影 (u,v)
- `EdgeStereoSE3ProjectXYZ`：双目 3D 重投影 (uL, vL, uR)

### 6.2 Local Inertial BA（`src/Optimizer.cc` `Optimizer::LocalInertialBA(...)`，`include/Optimizer.h:86`）

在纯视觉 Local BA 基础上额外优化：

**新增顶点类型**（`include/G2oTypes.h`）：
- `VertexVelocity`（line 191）：3DoF IMU 速度
- `VertexGyroBias`（line 212）：3DoF 陀螺仪偏置
- `VertexAccBias`（line 234）：3DoF 加速度计偏置
- `VertexPose`（line 130）：6DoF IMU 位姿（替代纯相机位姿，`ImuCamPose` 封装了 Rwb, twb + 相机参数）

**新增边类型**：
- `EdgeInertial`（`include/G2oTypes.h:495`）：9 维 IMU 预积分约束
  - 残差维度：9 = 旋转(3) + 速度(3) + 位置(3)
  - 连接顶点：Pose_i(6) + Vel_i(3) + BiasG_i(3) + BiasA_i(3) + Pose_j(6) + Vel_j(3)
- `EdgeGyroRW`（line 635）：陀螺仪偏置随机游走先验
- `EdgeAccRW`（line 671）：加速度计偏置随机游走先验

### 6.3 Global BA（`src/Optimizer.cc:52-310` `Optimizer::BundleAdjustment()`）

- 优化**所有 KF 位姿**和**所有地图点 3D 坐标**
- 使用与 Local BA 相同的 G2O 框架
- 固定原点 KF（`mnId == pMap->GetInitKFid()`）
- 5 次 LM 迭代（默认，`nIterations=5`，line 51）
- 支持 `pbStopFlag` 中断标记（用于回环后新 KF 到达时中止）

### 6.4 Full Inertial BA（`src/Optimizer.cc` `Optimizer::FullInertialBA()`，`include/Optimizer.h:55`）

- 结合视觉重投影约束 + IMU 预积分约束
- 优化：Pose(6) + Velocity(3) + GyroBias(3) + AccBias(3) + MapPoint(3) + [可选] Gravity(2)
- 使用 `EdgeInertial` + Visual Edges + `EdgeGyroRW`/`EdgeAccRW`（偏置先验）
- 支持 `bFixLocal` 模式（仅优化局部区域）

### 6.5 Essential Graph Optimization（`src/Optimizer.cc` `Optimizer::OptimizeEssentialGraph()`，`include/Optimizer.h:64`）

- 位姿图优化，边约束为**相对 SE3 变换**
- 仅优化位姿，不优化地图点
- 约束来源：生成树边、回环边、强共视边（权重 > 100）
- 信息矩阵：按边类型和观测质量设置

### 6.6 Inertial-Only Optimization（`src/Optimizer.cc` `Optimizer::InertialOptimization()`，`include/Optimizer.h:97-99`）

三个重载版本对应不同初始化阶段：

1. `InertialOptimization(Rwg, scale, bg, ba, bMono, covInertial, ...)`（line 97）：估计重力方向 + 尺度 + 偏置 + 速度
   - 固定视觉位姿（来自纯视觉 SLAM）
   - 仅使用 IMU 预积分约束
   - 使用 `EdgeInertialGS`（含重力、尺度顶点）(`include/G2oTypes.h:548`)
   - 固定所有 KF 位置，优化速度 + 偏置 + 重力方向(2DoF) + 尺度(1DoF)

2. `InertialOptimization(bg, ba, ...)`（line 98）：仅优化陀螺仪 + 加速度计偏置

3. `InertialOptimization(Rwg, scale)`（line 99）：优化重力方向和尺度

### 6.7 与 OpenVINS MSCKF 方法对比

| 对比维度 | ORB-SLAM3 | OpenVINS (MSCKF) |
|----------|-----------|-------------------|
| **优化方法** | Bundle Adjustment（全状态优化） | MSCKF（边缘化过去位姿，投影到当前状态） |
| **状态维度** | 随 KF 数量线性增长（每个 KF 独立优化） | 固定窗口（滑动窗口 + 边缘化），状态维度恒定 |
| **地图复用** | Atlas 多地图，BoW 重定位/Loop 回环 | 无显式地图复用 |
| **尺度可观性** | 视觉-惯性联合 BA 可观测尺度 | 需要足够激励观测尺度收敛 |
| **计算复杂度** | Local BA O(局部KFs·MPs)，Full BA O(N²) | O(窗口大小²) 近似恒定 |
| **全局一致性** | 通过回环 + Full BA 保证 | 通过先验传递，无全局优化 |
| **适用场景** | 室内外大范围长期 SLAM | 小范围 VIO，飞行器/手持设备实时跟踪 |

---

## 7. 优缺点分析

### 7.1 算法优势

1. **三线程架构**（`src/System.cc:191-214`）：
   - Tracking 线程（主线程）、LocalMapping 线程、LoopClosing 线程**异步并行**
   - 实时性好，帧率不受 Local BA 或 Loop Closure 阻塞
   - 线程同步通过关键帧队列 `mlNewKeyFrames`（`include/LocalMapping.h:164`，`std::list<KeyFrame*>`）解耦

2. **Atlas 多地图系统**（`include/Atlas.h:49-166`）：
   - 跟踪丢失后自动创建新地图，不丢弃已有地图数据
   - 通过 Common Regions 检测自动合并（`LoopClosing::NewDetectCommonRegions()`）
   - 大幅提升长期运行鲁棒性

3. **统一的 IMU 集成**（`include/ImuTypes.h`）：
   - `IMU::Preintegrated` 类实现了流形上的 IMU 预积分（`src/ImuTypes.cc:107-421`）
   - 支持偏置在线校正（一阶近似更新 `GetDeltaRotation/GetDeltaVelocity/GetDeltaPosition`）
   - IMU 初始化分为 scale、gravity、bias 阶段逐步精化（`src/LocalMapping.cc:1173-1522`）

4. **多级 BA 优化体系**（`include/Optimizer.h:48-101`）：
   - Motion-only BA → Local BA → Essential Graph → Full BA → Full Inertial BA
   - 计算量精准匹配各模块实时性需求
   - Levenberg-Marquardt 结合 Huber 鲁棒核保证异常值容忍度

5. **ORB 特征的高效性**：
   - FAST 角点检测 O(n)，四叉树分布 O(n log n)，steered BRIEF 描述子匹配用汉明距离（位运算）
   - DBoW2 词袋模型加速匹配/重定位/回环检测（`Thirdparty/DBoW2/`）

6. **支持多种相机模型**（`include/CameraModels/`）：
   - `Pinhole`（针孔模型）和 `KannalaBrandt8`（鱼眼模型）
   - 统一抽象接口 `GeometricCamera`（`include/CameraModels/GeometricCamera.h`）

### 7.2 算法弱点

1. **ORB 特征鲁棒性问题**：
   - FAST 角点在**低纹理**（白墙）、**运动模糊**、**光照剧烈变化**场景下提取数量不足
   - 固定的提取阈值（`iniThFAST/minThFAST`）不适应场景变化
   - Steered BRIEF 旋转补偿后的区分度下降

2. **关键帧选择和管理的保守性**：
   - 基于经验阈值的 KF 插入策略（`src/Tracking.cc` `NeedNewKeyFrame`）在快速旋转/纯旋转场景下可能不插入 KF
   - 无自适应 KF 频率调节

3. **内存占用大**：
   - 所有 KF 保留完整图像特征（`KeyFrame::mvKeys/mvKeysUn/mvKeysRight/mDescriptors`）
   - 所有地图点保留多 KF 观测列表（`MapPoint::mObservations`）
   - Full BA 后旧地图未有效释放（仅标记 `mbBad=true`，`src/Map.cc`）

4. **单目初始化脆弱**：
   - 依赖两帧间足够平移（parallax > 1°）和足够 FAST 匹配
   - 纯旋转/慢速场景无法初始化
   - 基础矩阵 vs 单应矩阵选择在平面场景下可能误判

5. **回环/合并计算开销大**：
   - Sim3 RANSAC 求解（`src/Sim3Solver.cc`）随地图增大耗时增长
   - Full BA 线程与追踪异步，但可能长时间占用 CPU

6. **IMU 初始化对激励敏感**：
   - 需要足够加速度变化才能可观测加速度计偏置和尺度
   - 初始化期间若静止/匀速运动 → 退化

### 7.3 工程优缺点

**优点**：
- C++11 标准，跨平台良好（Ubuntu 16.04/18.04 测试通过，见 `README.md:54`）
- Boost serialization 支持地图保存/加载（`src/System.cc:1403-1506`）
- REGISTER_TIMES 宏可开关性能统计（`include/Config.h:232-233`），输出详细耗时文件
- ROS 接口完善（6 种传感器模式节点，`README.md:154-222`）
- 丰富的示例程序（TUM/EuRoC/KITTI/RealSense，共约 20+ 个示例）

**缺点**：
- 依赖较重：OpenCV（≥4.4）、Eigen3、Pangolin、Boost、DBoW2、g2o（`CMakeLists.txt:33-126`）
- Pangolin 可视化线程在无 GUI 环境需谨慎处理
- `boost 1.58 + Ubuntu 16.04` 存在 set 序列化 bug，需用 vector 替代（`include/Atlas.h:61,147`）
- CMake 使用旧式 `include_directories()`，非 modern CMake target-based 依赖管理（`CMakeLists.txt:45-52`）
- 多处显式使用 `static_cast< ... *>(NULL)`，应使用 `nullptr`
- 硬编码常量分散于多处（EDGE_THRESHOLD=19, PATCH_SIZE=31, FRAME_GRID_ROWS=48 等）

### 7.4 适用与不适用场景

**适用**：
- 手持/轮式机器人室内外长期 SLAM（回环检测 + 全局一致性）
- 无人机带 IMU 的视觉-惯导定位（VIO 模式）
- 多段不连续轨迹拼接（Atlas 多地图）
- 需要地图保存和重载的应用

**不适用**：
- 极低纹理环境（白墙走廊）
- 高动态场景（大量运动物体 → ORB 匹配被外点淹没）
- 实时性要求极高（<5ms/帧）的嵌入式场景（ARM 处理器）
- 纯旋转/静止初始化场景（单目模式）
- 需要稠密重建的场景（ORB-SLAM3 输出稀疏点云）

---

## 8. 对 phad_fusion 的关键参考

### 8.1 三线程架构设计

直接从 ORB-SLAM3 借鉴 `Track-LocalMap-Loop` 模式：
- **Tracking Thread**：运行在主线程（`System::TrackXxx()` → `Tracking::Track()`），负责前端位姿估计
- **LocalMapping Thread**：独立线程（`src/LocalMapping.cc:64` `Run()`），负责后端 BA + 新地图点创建
- **LoopClosing Thread**：独立线程（`src/LoopClosing.cc:90` `Run()`），负责回环检测 + 位姿图优化

**关键参考**（`src/System.cc:191-224`）：
- 线程间的指针设置模式 `SetLocalMapper/SetLoopClosing`
- 关键帧队列 `mlNewKeyFrames`（`std::list<KeyFrame*>`）解耦 Tracking → Mapping
- `mbAbortBA` 标记实现 Local BA 安全中断
- `SetAcceptKeyFrames(true/false)` 控制 KF 插入节奏

### 8.2 ORB 特征管线

**完整可复用的组件**：
- `ORBextractor`（`src/ORBextractor.cc:409-469`）：FAST + 四叉树 + IC_Angle + steered BRIEF
- `ORBmatcher`（`src/ORBmatcher.cc`）：BoW 匹配 / 投影匹配 / 极线搜索匹配三种策略
- `Frame::AssignFeaturesToGrid()`（`include/Frame.h:317`）：48×64 网格加速邻域搜索

**phad_fusion 改进方向**：
- 替换 ORB 为 **SuperPoint** 或 **DISK** 等深度学习特征（鲁棒性更强，但需要 GPU 加速）
- 保留四叉树均匀分布策略（`DistributeOctTree`，`src/ORBextractor.cc:555`），可适配任何角点检测器
- 保留网格空间索引设计（`Frame::mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]`，`include/Frame.h:252`）

### 8.3 Atlas 多地图系统

**直接借鉴**：
- `Atlas::CreateNewMap()`（`src/Atlas.cc:58-77`）：跟踪丢失后创建新地图而保留旧地图
- `Atlas::ChangeMap()`（line 79-89）：支持地图切换
- 地图间 Common Regions 检测和合并（`LoopClosing::NewDetectCommonRegions`，`src/LoopClosing.cc:112`）
- 地图序列化/反序列化（`boost::serialization`，`src/System.cc:1403-1506`）

**phad_fusion 的扩展**：
- 可添加**语义标签**到每个 Map（室内/室外、楼层等）
- 支持**部分地图保存**（仅保留关键帧 + 描述子，丢弃非关键帧）

### 8.4 G2O 优化集成

**完整的最小化示例**（`src/Optimizer.cc:52-310` `BundleAdjustment()`）：
```cpp
// 创建优化器
g2o::SparseOptimizer optimizer;
g2o::BlockSolver_6_3::LinearSolverType* linearSolver =
    new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
g2o::OptimizationAlgorithmLevenberg* solver =
    new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
optimizer.setAlgorithm(solver);

// 添加顶点：g2o::VertexSE3Expmap (Pose), g2o::VertexSBAPointXYZ (Landmark)
// 添加边：EdgeSE3ProjectXYZ (Mono), EdgeStereoSE3ProjectXYZ (Stereo)
// 设置 Huber 鲁棒核：g2o::RobustKernelHuber
// optimizer.initializeOptimization() → optimizer.optimize(nIterations)
```

**IMU 集成的最小化示例**（`src/Optimizer.cc` `FullInertialBA()`）：
- 自定义顶点类：`VertexPose`(6), `VertexVelocity`(3), `VertexGyroBias`(3), `VertexAccBias`(3)（`include/G2oTypes.h:130-253`）
- 自定义 IMU 预积分边：`EdgeInertial`(9)（`include/G2oTypes.h:495`）
- 自定义重力/尺度顶点：`VertexGDir`(2) + `VertexScale`(1)（`include/G2oTypes.h:274,296`）

### 8.5 关键帧管理策略

**关键参考代码**：
- `Tracking::NeedNewKeyFrame()`（`src/Tracking.cc:222`）：多条件联合判断
- `LocalMapping::ProcessNewKeyFrame()`（`src/LocalMapping.cc:298-338`）：KF 插入 → BoW → 观测关联 → 共视图更新
- `LocalMapping::KeyFrameCulling()`（`src/LocalMapping.cc:902`）：90% 冗余 KF 剔除
- `KeyFrame::UpdateConnections()`（`include/KeyFrame.h:224`）：动态共视图维护
- 生成树/共视图/回环边三层拓扑结构（`include/KeyFrame.h:467-477`）

**phad_fusion 的改进方向**：
- 引入**信息熵**度量 KF 的信息增益（替代简单的匹配数阈值）
- 支持**动态调整 KF 频率**（快速旋转时密度加大，匀速时密度减小）
- KF 描述子压缩存储（Product Quantization）

### 8.6 需要避免的坑

1. **线程安全**：ORB-SLAM3 中频繁使用 `std::mutex` + `unique_lock`（如 `KeyFrame::mMutexPose/Connections/Features/Map`）。phad_fusion 必须注意：
   - 地图数据（Map/MapPoint/KeyFrame）的读写锁粒度
   - Tracking → Mapping 之间的 KF 队列必须原子操作

2. **地图点管理**：ORB-SLAM3 的 `MapPoint` 使用 soft delete（`mbBad` 标记）+ `mpReplaced` 替换链（`include/MapPoint.h:236-238`）。这导致了**幽灵引用**（已标记 bad 但仍被某些 KF 引用），需注意：
   - 每次使用前调用 `isBad()` 检查
   - `CheckReplacedInLastFrame()`（`src/Tracking.cc:1943`）在每帧处理替换

3. **G2O 内存管理**：ORB-SLAM3 中 G2O optimizer 析构时自动释放顶点/边，但外部持有的 KeyFrame/MapPoint 指针不变。phad_fusion 中需确保：
   - G2O 优化后的 pose/point 更新通过 `SetPose()/SetWorldPos()` 同步回数据层
   - 不要在 optimizer 对象外持有 solver 内部指针

4. **DBoW2 词汇表兼容性**：ORB-SLAM3 的词汇表文件和代码紧密耦合 (`src/System.cc:1487-1498` CheckSum验证)。使用不同描述子（非 ORB）时词汇表**完全不可复用**。

5. **Pangolin 线程**：`Viewer` 线程在无 GUI 环境（Docker/SSH headless）会 crash。需要条件编译或提供无 GUI fallback。

---

## 附录：关键文件索引

| 文件 | 行号 | 核心内容 |
|------|------|----------|
| `src/System.cc:41-242` | 41-242 | 系统构造函数，三线程启动 |
| `src/ORBextractor.cc:409-469` | 409-469 | ORBextractor 构造与参数初始化 |
| `src/ORBextractor.cc:781-895` | 781-895 | ComputeKeyPointsOctTree - FAST + 四叉树 |
| `src/ORBextractor.cc:76-103` | 76-103 | IC_Angle - 灰度质心方向 |
| `src/ORBextractor.cc:107-146` | 107-146 | Steered BRIEF 描述子计算 |
| `src/ORBextractor.cc:555-779` | 555-779 | DistributeOctTree - 四叉树特征分布 |
| `src/TwoViewReconstruction.cc:41-129` | 41-129 | 单目初始化 Reconstruct |
| `src/Tracking.cc:1794-` | 1794- | Track() 主函数 |
| `src/LocalMapping.cc:64-282` | 64-282 | Run() 主循环 |
| `src/LocalMapping.cc:388-711` | 388-711 | CreateNewMapPoints - 三角化建点 |
| `src/LocalMapping.cc:1173-1522` | 1173-1522 | InitializeIMU - IMU 初始化 |
| `src/Optimizer.cc:60-310` | 60-310 | BundleAdjustment - Full BA |
| `include/G2oTypes.h:130-253` | 130-253 | G2O 自定义顶点 (Pose/Velocity/Bias) |
| `include/G2oTypes.h:342-847` | 342-847 | G2O 自定义边 (Mono/Stereo/Inertial) |
| `include/ImuTypes.h:143-251` | 143-251 | Preintegrated - IMU 预积分 |
| `src/Atlas.cc:58-89` | 58-89 | CreateNewMap / ChangeMap |
| `include/Atlas.h:53-69` | 53-69 | Atlas Boost 序列化 |

---

> 报告总长：~650 行
> 覆盖源文件：System.cc, Tracking.cc, LocalMapping.cc, LoopClosing.cc, ORBextractor.cc, Optimizer.cc, TwoViewReconstruction.cc, Atlas.cc, ImuTypes.cc, G2oTypes.h, ImuTypes.h, Frame.h, KeyFrame.h, Map.h, MapPoint.h, Optimizer.h, Atlas.h, System.h, ORBextractor.h, CMakeLists.txt, README.md

---

## 9. 数据管线

### 9.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | ROS话题/接口 | 负责模块 |
|--------|------|----------|-------------|----------|
| 相机 (mono/stereo/RGB-D) | 10-60 Hz | `cv::Mat` (BGR/RGB + 可选 Depth) | `IMAGE_TOPIC` → `System::TrackMonocular/Stereo/RGBD` | `Tracking::Track`, `ORBextractor` |
| IMU (6 轴) | 100-200 Hz | `IMU::Point {a[3], w[3], t}` | `IMU_TOPIC` → `System::TrackXxx(vImuMeas)` | `Tracking::PreintegrateIMU`, `IMU::Preintegrated` |

### 9.2 相机数据处理管线

#### 原始数据
- **图像格式**: `cv::Mat` (BGR → GRAY 色彩转换, `cv::cvtColor`)
- **分辨率**: 配置定义, 支持缩放 `cv::resize()` (`Frame::needsToResize()`)
- **双目校正**: `cv::remap(imLeft, imLeftRect, M1l, M2l)` 配合离线标定的校正矩阵
- **相机模型**: `GeometricCamera` 虚基类 → `Pinhole` (针孔) 或 `KannalaBrandt8` (鱼眼)，YAML 配置文件加载
- **畸变模型**: 径向+切向 `k1,k2,p1,p2,k3,k4`，去畸变 `Frame::UndistortKeyPoints()` 调用 `distortPoint()`
- **时间戳**: 双精度 `double` 秒，通过 ROS header 获取

#### 特征提取 (`ORBextractor::operator()`)
`src/ORBextractor.cc:1086-1197`:

**步骤 1: 图像金字塔** (`ComputePyramid`)
- 层数: `nlevels=8`, 缩放因子: `scaleFactor=1.2`
- 第 0 层 = 原图, 第 i 层 = `resize(i-1, 1/scaleFactor)`

**步骤 2: FAST 角点** (`ComputeKeyPointsOctTree` → `src/ORBextractor.cc:781-895`)
- 网格划分: `W=35` 像素/单元 → `nCols=width/W`, `nRows=height/W`
- 双阈值策略:
  - 高阈值: `iniThFAST=20` (第一遍尝试)
  - 低阈值: `minThFAST=7` (高阈值无角点时降级)
- 每 cell 调用 `cv::FAST(img_ROI, pts_new, threshold, true)` (NMS 开启)

**步骤 3: 四叉树分布** (`DistributeOctTree` → `src/ORBextractor.cc:555-779`)
- 初始节点: 由图像宽高决定水平分块数
- 迭代细分: 节点 > 1 点时分为 4 子节点
- 停止: 单点不细分 / 节点数 ≥ 目标数 / 最小尺寸
- 每个叶节点保留 Harris 响应最高点

**步骤 4: 方向计算** (`IC_Angle` → `src/ORBextractor.cc:76-103`)
- 灰度质心法: 31×31 圆形 patch (`HALF_PATCH_SIZE=15`)
- `m_10 = Σ u·I(u,v)`, `m_01 = Σ v·I(u,v)`
- `θ = atan2(m_01, m_10)` (fastAtan2)

**步骤 5: Steered BRIEF 描述子** (`computeOrbDescriptor` → `src/ORBextractor.cc:107-146`)
- 256 对采样点 (来自硬编码 `bit_pattern_31_[256*4]`)
- 旋转补偿: `(x',y') = R(θ)·(x,y)` (采样坐标旋转)
- 编码: 8 次比较 → 1 字节, 32 次 → 32 字节 (256 位)

**步骤 6: 特征分配**
- 等比数列分配: `nFeaturesPerLevel[i] = nFeatures*(1-factor)/(1-factor^nlevels) * factor^i`
- 越上层分辨率越低，特征越少

#### 匹配/关联

**BoW 匹配** (`ORBmatcher::SearchByBoW`):
- 同一 vocabulary tree 节点内匹配
- 计算汉明距离，阈值 50~100

**投影匹配** (`ORBmatcher::SearchByProjection`):
- 3D 点投影到帧 → 在投影点邻域 (`th`) 搜索最佳匹配
- 金字塔尺度一致性检查: `abs(octave_pred - octave_cand) <= 1`
- 旋转直方图投票: `TH_HIGH=100, TH_LOW=50`

**极线匹配** (`ORBmatcher::SearchForTriangulation`):
- BoW 加速 + 极线距离约束
- 视差检查: `parallax > 1°`

**RANSAC:**
- 单目初始化: `TwoViewReconstruction` (H + F 并行 RANSAC, 8 点法)
- 重定位: `MLPnPsolver` (置信度 0.99, 300 次迭代, 6 点最少, 0.5px)
- 回环: `Sim3Solver` (RANSAC ± 20 次迭代)

#### 离群点剔除 (Tracking 层面)
- 卡方检验: `e^T*Σ^{-1}*e < χ²_{0.95}(dof)`，单目 χ²(2)=5.991, 双目 χ²(3)=7.815
- 异常比例: `mnInliers < 0.25*mnMatches` → 跟踪失败

### 9.3 IMU 数据处理管线

#### 原始数据
- **测量**: `IMU::Point {Eigen::Vector3f a, Eigen::Vector3f w, double t}` (`include/ImuTypes.h:46-59`)
- **噪声参数** (YAML): 加速度计噪声密度 σ_a, 陀螺仪噪声密度 σ_g, 偏置随机游走 σ_ba, σ_bg
- **重力**: `G = [0, 0, -9.81]` m/s²

#### 接收与缓冲
`Tracking::GrabImuData(IMU::Point)` (`src/Tracking.cc:1712`):
- 每次加入单点，推入 `mlQueueImuData` (list<IMU::Point>)
- 每个图像帧传入时附带 `vImuMeas` 向量 (时间范围 `[last_frame_t, curr_frame_t]`)

#### IMU 预积分 (`IMU::Preintegrated::IntegrateNewMeasurement`)
`src/ImuTypes.cc:107-240 `:

**步骤 1: 帧间增量** (原始值, 未减偏置):
```
dR(accW*dt)  ← exp(w*dt)
dV(accW*dt)  ← dV + dR * (a*dt)     // 零阶加速度近似
dP(accW*dt)  ← dP + dV*dt + dR*(0.5*a*dt²)
dtSum        ← dtSum + dt
```

**步骤 2: 协方差传播** (9×9 矩阵，δR, δV, δP):
```
A = [[-dR^T*J*(a*dt)×, O, O],
     [-J*dt²/2, I*dt, O],
     [J*dt, O, I]]
B = [[J*dt, O], [dR^T*J*dt²/2, J*dt²/2], [dR^T*J*dt, J*dt]]
noise = diag(σ_g², σ_a²)
Σ = A * Σ * A^T + B * diag(noise, noise) * B^T
```
其中 J = `dR(Δt)⁻¹` = `exp(wΔt)⁻¹` 为右 Jacobian

**步骤 3: 偏置 Jacobian** (对偏置变化的一阶导数):
- `dR_db_g`, `dV_db_g`, `dP_db_g` (9×3 Jacobian to gyro bias)
- `dV_db_a`, `dP_db_a` (6×3 Jacobian to accel bias)

#### 偏置校正 (`Preintegrated::GetDeltaRotation/Velocity/Position`)
```cpp
dR_bias_corrected = dR * exp(J_R_bg * δbg)
dV_bias_corrected = dV + J_V_bg*δbg + J_V_ba*δba
dP_bias_corrected = dP + J_P_bg*δbg + J_P_ba*δba
```

#### 状态传播 (`Tracking::PredictStateIMU` → `src/Tracking.cc:1832-1880`)
```
R_wb_k = R_wb_k-1 * dR_bg_corrected
P_wb_k = P_wb_k-1 + V_wb_k-1*Δt + 0.5*G*Δt² + R_wb_k-1*dP_corrected
V_wb_k = V_wb_k-1 + G*Δt + R_wb_k-1*dV_corrected
```

### 9.4 算法消费：g2o 因子构造

#### 视觉因子 (纯视觉 BA)

**EdgeSE3ProjectXYZ** (`include/G2oTypes.h:342-388`, 单目, 2 维残差):
```
P_c = R_cw*P_w + t_cw           // World→Camera
u_proj = fx*(P_c.x/P_c.z)+cx    // 投影
v_proj = fy*(P_c.y/P_c.z)+cy
r = [u_obs-u_proj, v_obs-v_proj]
```

**EdgeStereoSE3ProjectXYZ** (`include/G2oTypes.h:425-463`, 双目, 3 维残差):
```
r = [uL_obs-uL_proj, v_obs-v_proj, uR_obs-uR_proj]
```
- 信息矩阵: `Σ^-1 = invSigma2_level * I`，`invSigma2 = 1/(1.2^(2*level))` (金字塔层级缩放)

#### IMU 因子 (视觉-惯性 BA)

**EdgeInertial** (`include/G2oTypes.h:495-544`, 9 维残差):
```
r_R = Log(dR^T * R_wb_i^T * R_wb_j)                [3维: 旋转]
r_V = R_wb_i^T*(V_wb_j - V_wb_i - G*Δt) - dV      [3维: 速度]
r_P = R_wb_i^T*(P_wb_j - P_wb_i - V_wb_i*Δt       [3维: 位置]
                - 0.5*G*Δt²) - dP
```
- 连接顶点: `VertexPose(6)i, VertexVelocity(3)i, VertexGyroBias(3), VertexAccBias(3), VertexPose(6)j, VertexVelocity(3)j`
- 信息矩阵: 预积分协方差逆 `Σ_ij^{-1}` (9×9)

**EdgeInertialGS** (`include/G2oTypes.h:548-631`, IMU 初始化专用):
- 额外顶点: `VertexGDir(2)` (重力方向, 2 DoF) + `VertexScale(1)` (单目尺度)
- 9 维残差公式同上，但参数化含 `R_wb = R(Rwg) * R_vision * s`

**EdgeGyroRW** / **EdgeAccRW** (`include/G2oTypes.h:635-704`):
- 偏置随机游走先验: `r = bias_j - bias_i`
- 信息矩阵: `1/(σ_rw² · Δt) * I_{3×3}`

#### g2o 配置
- 求解器: `BlockSolver_6_3` (Pose 6维 + Landmark 3维) / `BlockSolverX` (多类型顶点)
- 算法: **Levenberg-Marquardt** (`OptimizationAlgorithmLevenberg`)
- 线性求解: `LinearSolverEigen` (Dense, 小规模) / `LinearSolverCSparse` (Sparse, 大规模)
- 鲁棒核: `RobustKernelHuber`, 视觉 `delta=√5.991`, 惯性 `delta=√7.815`

### 9.5 跨传感器协同

#### 时间同步
- 图像帧时间戳为主基准
- IMU 按时间窗口分配: `[last_frame_t, curr_frame_t]` 内的 IMU 打包传入 `TrackXxx`
- 无显式时间偏移校准变量

#### 数据缓冲策略
- IMU 队列: `mlQueueImuData` (list)，按需取出分配
- 关键帧队列: `mlNewKeyFrames` → LocalMapping 线程消费 (线程间解耦)
- 三线程异步: Tracking (主线程) → LocalMapping (独立线程) → LoopClosing (独立线程)

#### 初始化管线

**单目**: `TwoViewReconstruction` (H/F 模型选择 + 三角化) → `CreateInitialMapMonocular`

**双目/RGB-D**: `StereoInitialization` → 直接视差计算 3D 点

**单目+IMU**: SfM (relativePose → GlobalSFM → Full BA) → IMU-视觉对齐 (`solveGyroscopeBias` → `LinearAlignment` → `RefineGravity`)

**双目+IMU**: 双目三角化当前帧 → PnP + 三角化滑动窗口 → 陀螺仪偏置 → 滑动窗口优化

**IMU 初始化** (LocalMapping 线程, `LocalMapping::InitializeIMU`):
1. 收集 ≥10 KF, 时间 ≥ 2s(单目)/1s(双目)
2. 重力方向估计: `dirG = Σ(R_wb·ΔV)`, `R_wg = align(dirG, [0,0,-1])`
3. `Optimizer::InertialOptimization`: 优化 `R_wg(2DoF)`, `scale`, `bg`, `ba`, `V_1..V_N`
4. 视觉-惯性 Full BA (可选)
5. 阶段性精化: 5s → `SetInertialBA1`, 15s → `SetInertialBA2`, 25s+ → `ScaleRefinement`

#### 降级与异常处理
- 跟踪丢失 → 创建新 Atlas 地图 (不丢弃旧地图)
- 重定位: BoW 候选检测 → MLPnP → PoseOptimization
- 地图合并: `NewDetectCommonRegions` → `MergeLocal` → Welding BA
- 地图点剔除: `MapPointCulling`: `GetFoundRatio()<0.25` 或观测<2帧(单目)/3帧(双目) → 标记 bad
- 关键帧剔除: `KeyFrameCulling`: 90% 地图点被 ≥3 个其他 KF 观测 → 冗余
- 地图保存/加载: `boost::serialization` 完整 Atlas 序列化