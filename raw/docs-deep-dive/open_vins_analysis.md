# open_vins 深度源码级分析

> **核心算法**：MSCKF (Multi-State Constraint Kalman Filter) 滑动窗口滤波器
> **版本**：v2.7 | **许可证**：GPLv3
> 分析日期：2026-04-28

---

## 1. 数据接收与预处理

### 1.1 图像数据接收

图像和 IMU 通过 `VioManager` 的三个 `feed_measurement_*` 接口进入系统。

图像路径 (`VioManager::feed_measurement_camera` -> `track_image_and_update`)：
1. 收到 `CameraData { timestamp, vector<int> sensor_ids, vector<Mat> images, vector<Mat> masks }`
2. 如果启用降采样 (`params.downsample_cameras = true`)，对每张图像和 mask 做 `cv::pyrDown` 2x 缩小
3. 调用 `trackFEATS->feed_new_camera(message)` 进入特征跟踪器

### 1.2 IMU 数据接收

IMU 路径 (`VioManager::feed_measurement_imu`)：
1. 计算 `oldest_time`：取最老克隆的 timestamp 或初始化窗口起始
2. `propagator->feed_imu(message, oldest_time)`：写入 Propagator 内部 buffer，自动清理 `oldest_time - 0.10` 之前的数据
3. 未初始化时同步写入 `initializer->feed_imu(message, oldest_time)`
4. 已初始化且启用 ZUPT 时同步写入 `updaterZUPT->feed_imu(message, oldest_time)`

### 1.3 时间偏移处理

`state->_calib_dt_CAMtoIMU` 是 IMU 和相机的时钟偏差 (`t_imu = t_cam + t_off`)，可作为在线标定变量。传播时所有 timestamp 都会加上这个偏移。

### 1.4 多相机同步

- 单目模式：`feed_monocular(message, 0)`
- 双目模式 (use_stereo=true)：`feed_stereo(message, 0, 1)`，左右图时间戳相同
- 多相机非双目模式：`parallel_for_` 并行调用每相机的 `feed_monocular`

---

## 2. 特征提取与跟踪

open_vins 支持两种跟踪方式，通过 `params.use_klt` 切换：
- **KLT 光流跟踪** (`TrackKLT`)：速度快，适合帧率高的场景
- **描述子匹配** (`TrackDescriptor`)：适应大基线、长时间间隔

### 2.1 KLT 光流跟踪 (`TrackKLT::feed_monocular`)

#### 步骤 1：预处理

1. **直方图均衡化**：支持 NONE / HISTOGRAM / CLAHE 三种模式，改善光照变化下的鲁棒性
2. **构建图像金字塔**：`cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels)`，用于多层 KLT

#### 步骤 2：特征提取 (`perform_detection_monocular`)

1. **网格 occupancy 管理**：建立两个 grid
   - `grid_2d_close`：精细网格（分辨率 = `min_px_dist`），防止特征点太近
   - `grid_2d_grid`：粗网格（`grid_x * grid_y`），确保均匀分布
2. **清理已有特征**：遍历 `pts_last`，检查每个特征是否在边界内、mask 区外、与邻近点不冲突，冲突的移除
3. **计算缺失特征数**：`num_featsneeded = num_features - pts.size()`，如果 < `max(20, 0.5*num_features)` 则不提取新特征
4. **按网格补充提取**：
   - 找到特征数不足的网格 cell
   - 调用 `Grider_GRID::perform_griding()` 提取新 FAST 角点
   - 新提取的 FAST 角点再做 `grid_2d_close` 近邻过滤
   - 每个新特征赋予唯一自增 ID (`++currid`)

#### 步骤 3：帧间跟踪 (`perform_matching`)

1. `cv::calcOpticalFlowPyrLK` 在金字塔上从 `img_pyramid_last` 跟踪到 `img_pyramid_curr`
   - 参数：`win_size` 窗口大小，`pyr_levels` 金字塔层数，`OPTFLOW_USE_INITIAL_FLOW`
   - 停止条件：COUNT=30 + EPS=0.01
2. **RANSAC 外点剔除**：
   - 先将 UV 点通过相机模型去畸变 (`undistort_cv`) 得到归一化坐标
   - `cv::findFundamentalMat`，RANSAC 阈值 = `2.0 / focal_length`
   - 仅保留 KLT 成功 AND 通过 F 矩阵 RANSAC 的特征

#### 步骤 4：写入特征数据库

对每个存活特征，去畸变后写入 `FeatureDatabase::update_feature(id, timestamp, cam_id, u, v, un, vn)`

### 2.2 描述子匹配 (`TrackDescriptor::feed_monocular`)

#### 步骤 1：特征提取 (`perform_detection_monocular`)

1. `Grider_FAST::perform_griding(img, mask, pts, num_features, grid_x, grid_y, threshold, nonmax)`
   - 将图像分为 `grid_x * grid_y` 个 cell
   - 每 cell 内用 `cv::FAST` 提取角点，按 response 排序取 top-N
   - ROI 坐标加回偏移
   - `cv::cornerSubPix(5x5, 20 iters)` 亚像素细化
2. **ORB 描述子**：`cv::ORB::compute(img, pts, desc)`
3. 同样做 `grid_2d` 近邻过滤

#### 步骤 2：描述子匹配与 RANSAC (`robust_match`)

1. **KNN 匹配**：`cv::BFMatcher::knnMatch(desc0, desc1, k=2)`，双向匹配
2. **Ratio Test**：`match[0].distance / match[1].distance > knn_ratio` -> 剔除
3. **Symmetry Test**：`match1.queryIdx == match2.trainIdx && match2.queryIdx == match1.trainIdx` -> 保留
4. **RANSAC**：去畸变归一化坐标 + `cv::findFundamentalMat(FM_RANSAC, 1/focal, 0.999)`

### 2.3 双目跟踪 (`TrackKLT::feed_stereo`)

除时间跟踪外，新特征提取时额外做：
1. 左图提取新 FAST + 用 KLT 投影到右图 (`cv::calcOpticalFlowPyrLK` + `OPTFLOW_USE_INITIAL_FLOW`)
2. 成功跟踪到右图的特征成为**双目特征**，同时失败的成为**左图单目特征**
3. 右图也独立补充单目特征

### 2.4 FAST 网格提取器 (`Grider_FAST`)

`ov_core/src/track/Grider_FAST.h:46`

- 纯静态工具类，无状态
- 当 `num_features < grid_x * grid_y` 时自动缩减网格，保持宽高比
- 每 cell 单独 `cv::FAST(img_ROI, pts_new, threshold, nonmax)`
- 按 response 排序取 `num_features_grid` 个
- 全部提取后统一做 `cv::cornerSubPix` 亚像素细化

### 2.5 总结：openvins 用了哪些库做特征处理

| 操作 | 实现方式 | 库 |
|------|---------|-----|
| 角点检测 | cv::FAST (OpenCV) | **OpenCV** |
| KLT 光流跟踪 | cv::calcOpticalFlowPyrLK | **OpenCV** |
| 描述子提取 | cv::ORB::compute | **OpenCV** |
| 描述子匹配 | cv::BFMatcher::knnMatch + ratio/symmetry test | **OpenCV** |
| 外点剔除 | cv::findFundamentalMat (FM_RANSAC) | **OpenCV** |
| 亚像素细化 | cv::cornerSubPix | **OpenCV** |
| 网格管理 | Grider_FAST / Grider_GRID / Grider_GRID (自研) | **自研** |
| 去畸变/投影 | CamBase 虚基类 (自研) -> 调用 cv::initUndistortRectifyMap 做去畸变 | **自研 + OpenCV** |

**结论：openvins 没有自己写特征提取算法，100% 依赖 OpenCV 的 FAST + KLT + ORB。**

---

## 3. 初始化

### 3.1 初始化触发

`VioManager::try_to_initialize()` 在每帧跟踪后、未初始化时被调用。
通过 `ov_init::InertialInitializer` 协调，实际执行在两个子类中。
初始化结果通过独立线程异步运行，不阻塞主跟踪线程。

### 3.2 静态初始化 (`StaticInitializer`)

**条件**：imu_data 足够 + 静止检测通过

1. 取初始窗口内的 IMU 加速度计读数求平均
2. 求重力方向：`g = normalize(avg_accel)`，计算初始姿态 `q_GtoI`
3. 位置、速度、bias 初始化为零
4. 协方差矩阵按先验不确定性设置

### 3.3 动态初始化 (`DynamicInitializer::initialize`)

`ov_init/src/dynamic/DynamicInitializer.cpp:44`

当静止条件不满足（存在运动激励时），走更复杂的动态初始化：

**前置检查**：
1. 取最近 `init_window_time` 秒内的数据窗口
2. 要求有效特征数 >= `0.75 * init_max_features`
3. IMU 测量数 >= 2
4. 特征深拷贝一份（不阻塞主跟踪线程）

**优化问题**：构建 Ceres 非线性最小二乘问题

- **优化变量**：
  - 每 `init_window_time/init_dyn_num_pose` 秒一个关键帧的姿态 (JPLQuatLocal)
  - 重力向量
  - 加速度计/陀螺仪 bias
  - 速度（每关键帧）
  - 特征 3D 坐标
  - 相机内外参（可选的在线标定）
- **残差块**：
  - `Factor_GenericPrior`：先验约束，防止优化漂移
  - `Factor_ImageReprojCalib`：重投影误差（支持在线标定）
  - `Factor_ImuCPIv1`：IMU 连续预积分约束（CpiV1 测量模型）
- **求解器**：Ceres Solver（自动求导）

**初始化完成后的对齐**：
- 从优化结果中计算 IMU 到世界系的初始变换
- 构建初始 `_imu` 状态 (`q_GtoI, p_IinG, v_IinG, bg, ba`)
- 将初始窗口内的所有相机时间戳插入为 `_clones_IMU`

### 3.4 第一帧位姿是如何计算的？

1. **静态初始化**：第一帧的位姿由加速度计确定的重力方向决定（z 轴与重力方向对齐），位置和偏航角无约束
2. **动态初始化**：第一帧位姿是 Ceres 非线性优化结果的一部分
3. **初始化后**：后续帧不再用初始化时的优化，而是走 EKF 滤波路径

---

## 4. 逐帧状态估计（初始化后）

`VioManager::do_feature_propagate_update()` 是初始化后每帧的核心入口。

### 4.1 预测步骤：IMU 预积分与位姿克隆

`Propagator::propagate_and_clone(state, timestamp)`

**步骤**：
1. `select_imu_readings(imu_data, time0, time1)` 选取 [上次更新时间, 当前帧时间] 间的 IMU 测量
2. 如果需要精确切边，用 `interpolate_data` 线性插值在起点和终点生成虚拟 IMU 测量
3. 对每段相邻 IMU 测量：
   - `predict_mean_*` 传播位姿/速度（三种模式选一种）
   - `predict_and_compute` 计算状态转移矩阵 F 和噪声协方差 Qd
   - `StateHelper::EKFPropagation` 更新全状态协方差
4. `StateHelper::augment_clone(state, timestamp)` 插入当前 IMU 位姿为新克隆

**三种积分模式**：
- **离散积分** (`predict_mean_discrete`)：`v_k+1 = v_k - g*dt + R_k^T*(a - ba)*dt`
- **RK4** (`predict_mean_rk4`)：四阶龙格库塔法，精度高但开销大
- **解析积分 ACI^2** (`predict_mean_analytic`)：闭式表达，最高精度

### 4.2 更新步骤

#### 等待足够 clone

当 `_clones_IMU.size() >= min(max_clone_size, 5)` 时才执行更新，确保可以三角化。

#### UpdaterMSCKF 更新 (`UpdaterMSCKF::update`)

`ov_msckf/src/update/UpdaterMSCKF.cpp:58`

**算法流程**：

1. **清理测量**：每个特征只保留 clone timestamp 上的测量，去掉太老和不在 clone 上的
2. **构造相机位姿**：从 IMU clone + 外参计算每个相机在每帧的绝对位姿
3. **三角化**：`FeatureInitializer::single_triangulation(feat, clones_cam)` 三角化 3D 位置
4. **Gauss-Newton 精化**（可选）：`single_gaussnewton(feat, clones_cam)` 多视角 BA 精化
5. **计算 Jacobian** (`UpdaterHelper::get_feature_jacobian_full`)：
   - 对每个特征，计算测量 Jacobian
   - H_f：对位姿状态（IMU 状态、clone 位姿、外参、时间偏移）的 Jacobian
   - H_x：对特征 3D 位置的 Jacobian
6. **零空间投影**（MSCKF 核心）：
   ```
   nullspace = H_x.transpose().fullPivLu().kernel()
   H_proj = nullspace.transpose() * H_f
   res_proj = nullspace.transpose() * residual
   ```
   目的：消除特征 3D 坐标变量，使得测量仅与滤波状态相关
7. **堆叠 + Chi^2 检验**：
   - 所有特征压缩后 H_proj 堆叠为大型线性系统
   - 计算 r^T * S^-1 * r，与 `chi_squared_table[residual_dimension]` 比较
   - 超出阈值则剔除该特征
8. **EKF 更新** (`StateHelper::EKFUpdate`)：
   - `K = P * H^T * (H*P*H^T + R)^-1`
   - `x_new = x + K*(z - h(x))`
   - `P_new = (I - K*H) * P`

### 4.3 后续几帧与第一帧的区别

| 阶段 | 第一帧 | 后续帧 |
|------|-------|--------|
| 位姿来源 | 静态=重力对齐 / 动态=Ceres优化 | IMU预积分 + MSCKF更新 |
| 不确定度 | 先验协方差 | 传播+更新后的滤波协方差 |
| 特征处理 | 提取新特征 | KLT跟踪已有 + 补充新特征 |
| clone管理 | 首次插入 | 维护滑动窗口大小 |

### 4.4 SLAM 特征更新 (`UpdaterSLAM::update`)

与 MSCKF 不同，SLAM 特征将 3D 坐标**加入状态**，不通过零空间投影消除。
适用于长期跟踪的标志物（Aruco 标签）或需要持久化地图的场景。

### 4.5 零速更新 ZUPT (`UpdaterZeroVelocity::try_update`)

检测条件：基于加速度计和陀螺仪的方差窗口分析，满足即视为静止。
更新时构造虚拟测量：速度 = 0，角速度 = 0，直接 EKF 更新。

### 4.6 边缘化

`StateHelper::marginalize_old_clone(state)`：
- 检查 clone 数是否超过 `max_clone_size`
- 对最老的 clone 相关的 SLAM 特征做一次性 EKF 更新
- 从协方差中删除最老 clone 的行列

### 4.7 高频输出（`fast_state_propagate`）

Propagator 维护一个缓存，以 IMU 频率调用 `fast_state_propagate`：
- 仅传播 IMU 状态的 13 维均值 + 12 维协方差
- 输出平滑的 IMU 频率里程计，供下游 ROS 话题使用

---

## 5. 局部优化与全局优化

### 5.1 局部优化

open_vins 严格来说**没有**通常意义上的局部 BA。其局部优化体现在：
- **MSCKF 更新本身就是局部的**：仅使用滑动窗口内 clone 上的测量，相当于一次批量线性化
- **Gauss-Newton 精化** (`single_gaussnewton`)：在每次 MSCKF 更新前对特征做多视角 BA 精化
- **动态初始化中的 Ceres BA**：这是唯一一次真正的优化（迭代非线性优化），但只在初始化时使用
- **FEJ (First Estimate Jacobian)**：保证线性化点一致性，防止同一状态被多次线性化造成不一致

### 5.2 全局优化

open_vins 本身**没有内建全局优化或回环检测**。文档明确指出可以通过外部工具扩展：
- `ov_secondary`：基于 VINS-Fusion 的位姿图优化
- `ov_maplab`：对接 maplab 的回环检测和全局优化
- 输出 marginalized feature track 和 3D 位置，供外部 PGO 使用

### 5.3 关键对比：open_vins vs ORB-SLAM3 的优化策略

| 维度 | open_vins | ORB-SLAM3 |
|------|----------|-----------|
| 局部优化 | MSCKF 更新 (EKF，一次线性化) | 局部 BA (迭代非线性优化) |
| 全局优化 | 无（依赖外部） | 全局 BA + 位姿图优化 |
| 回环检测 | 无（依赖外部） | 有（DBoW2 词袋 + Sim3 对齐） |
| 边缘化 | 滑动窗口老 clone 直接丢弃 | Schur complement 保留信息 |
| 初始化 | 静态(重力对齐) or 动态(Ceres BA) | 两阶段并行 (单目+IMU) |

---

## 6. 优缺点分析

### 6.1 算法层面优势

- **模块化协方差系统**：`ov_type::Type` 继承体系让增删状态变量不需要手改协方差索引
- **五种特征表示**：XYZ / anchor XYZ / anchor inv depth / MSCKF inv depth / single inv depth
- **三种积分模式**：离散(快)、RK4(平衡)、ACI^2(精确)，按计算资源选择
- **支持在线标定**：相机内外参、IMU 内参、g 敏感度、时间偏移均可估计
- **零空间投影保证一致性**：MSCKF 不会因特征坐标误差引入偏置
- **比因子图快**：EKF 更新 O(n^2) vs 迭代优化的 O(kn^3)，适合高速无人机

### 6.2 算法层面局限

- **EKF 线性化一次性的局限**：滑动窗口内只线性化一次，非线性强时可能发散
- **无内建回环检测**：不适合需要全局一致地图的长时间导航
- **MSCKF 假设特征在 clone 上可三角化**：纯旋转或 depth ratio 不够时失败
- **信息稀疏的代价**：零空间投影丢弃了特征间的约束信息
- **纯 CPU**：无 GPU 加速，大规模特征处理受限

### 6.3 工程层面优势

- **ROS-Free 内核**：ov_core 完全独立，ov_msckf 核可通过 ENABLE_ROS=OFF 剥离
- **ROS1/ROS2 双版本**：同一代码库兼容两套 ROS 系统
- **多线程异步初始化**：初始化在线程中，跟踪不中断
- **完善的仿真系统**：SE(3) B-spline 生成 ground truth，支持蒙特卡洛
- **丰富的推导文档**：docs.openvins.com 包含所有方程的推导

### 6.4 工程层面局限

- **可视化强依赖 ROS/Rviz**：只能通过 ROS 可视化
- **配置 YAML 复杂度高**：约 200+ 参数，调参门槛不低
- **Catkin/ament 双构建维护成本**：两个分支需保持一致
- **特征提取 100% 依赖 OpenCV**：没有自研算法，灵活性受限

### 6.5 适用与不适用场景

**适合**：
- 高速无人机 VIO（MSCKF 比因子图在高速场景更鲁棒）
- 多相机快速部署（配置式多相机支持）
- 需要在线标定场景
- SLAM 系统的前端里程计模块

**不适合**：
- 长时间大尺度导航（缺回环检测）
- 慢速/静止场景（MSCKF 依赖视差三角化）
- 大规模特征地图存储（不管理全局地图）
- 密集重建（仅稀疏特征）

---

## 7. 对 phad_fusion 的关键参考

| open_vins 设计精华 | phad_fusion 如何借鉴 |
|-------------------|---------------------|
| CamBase 虚基类：统一 undistort/distort 接口 | CameraRig 设计参考 |
| TrackBase 虚基类：KLT/Descriptor 可插拔 | 特征跟踪器抽象层 |
| Propagator 三模式：离散/RK4/ACI^2 | IMU 预积分因子多种精度 |
| MSCKF 零空间投影：特征变量不增广 | 视觉因子的边缘化策略 |
| ENABLE_ROS 条件编译：ROS/NON-ROS 双版本 | wrapper 层松耦合设计模式 |
| StateHelper 友元：外部操作协方差 | 因子后端的状态管理隔离 |
| Ceres 动态初始化框架 | 初始化模块的因子图构建 |
| 多线程异步初始化 + 高频 fast_propagate | 并发架构设计 |

**关键避坑**：
- 不要像 open_vins 一样无内置回环检测，phad_fusion 需要预留回环接口
- EKF 的 FEJ 策略在因子图后端里等价于在最新线性化点做 Taylor 展开
- open_vins 的 YAML 参数过多（200+），phad_fusion 应做配置分层

---

## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | ROS话题/接口 | 负责模块 |
|--------|------|----------|-------------|----------|
| 相机 (mono/stereo/multi) | 10-60 Hz | `cv::Mat` (BGR/Gray) | `CAMERA_TOPIC` → `VioManager::feed_measurement_camera` | `TrackKLT` / `TrackDescriptor` |
| IMU (加速度计+陀螺仪) | 100-1000 Hz | `ImuData {timestamp, a[3], w[3]}` | `IMU_TOPIC` → `VioManager::feed_measurement_imu` | `Propagator`, `InertialInitializer` |

### 8.2 相机数据处理管线

#### 原始数据
- **图像格式**: ROS `sensor_msgs::Image` → `cv_bridge::toCvCopy` → `cv::Mat` (BGR/Gray)
- **分辨率**: 配置驱动，支持降采样 (`params.downsample_cameras=true` 时 `cv::pyrDown` 2x 缩小)
- **标定来源**: `Kalibr` 格式 YAML，通过 `CamBase` 虚基类加载 (`ov_core/src/cam/cam_*.cpp`)，支持 pinhole / MEI / equi / KB4 模型
- **外参**: `T_CtoI` (IMU→Camera)，存储在 `state->_calib_IMUtoCAM[i]`，支持在线标定
- **时间偏移**: `_calib_dt_CAMtoIMU` 存储 `t_imu = t_cam + t_off`，支持在线估计

#### 预处理
1. **降采样** (`VioManager::feed_measurement_camera` → `VioManager.cpp:81`): 若 `params.downsample_cameras=true`，对图像和 mask 做 `cv::pyrDown` 2x 缩小
2. **直方图均衡化** (`TrackKLT::feed_monocular`): 支持 NONE / HISTOGRAM / CLAHE 三种模式，改善光照鲁棒性
3. **构建图像金字塔** (`TrackKLT`): `cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels)`

#### 特征提取 (`TrackKLT::perform_detection_monocular` + `Grider_FAST`)
1. **网格划分**: `grid_x` × `grid_y` 均匀网格，加 `grid_2d_close` (分辨率=`min_px_dist`) 精细网格防止特征点太近
2. **FAST 检测** (`Grider_FAST::perform_griding`): 每 cell 调用 `cv::FAST(img_ROI, pts_new, threshold, nonmax)`，按 response 排序取 top-N
3. **亚像素细化**: `cv::cornerSubPix(5x5, 20 iters)` 对所有提取点
4. **网格 occupancy 清理**: 遍历 `pts_last`，检查边界/mask/近邻冲突，冲突移除
5. **补充提取**: `num_featsneeded = num_features - pts.size()`，按不足的 grid cell 补充新 FAST 角点
6. **唯一 ID**: 每个新特征赋予自增 ID `++currid`

#### 匹配/关联
**KLT 模式** (`TrackKLT::perform_matching`):
1. `cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts, ..., win_size, pyr_levels, OPTFLOW_USE_INITIAL_FLOW)`
   - 停止条件: `COUNT=30 + EPS=0.01`
2. **RANSAC 外点剔除**: 去畸变得到归一化坐标 → `cv::findFundamentalMat(FM_RANSAC)`, 阈值=`2.0/focal_length`

**描述子模式** (`TrackDescriptor::robust_match`):
1. `Grider_FAST` 提取 ORB 特征 (`cv::ORB::compute`)
2. `cv::BFMatcher::knnMatch(k=2)` 双向 KNN
3. Ratio test: `match[0].distance / match[1].distance > knn_ratio` → 剔除
4. Symmetry test: 双向匹配一致性
5. RANSAC: `cv::findFundamentalMat(FM_RANSAC, 1/focal, 0.999)`

**双目匹配** (`TrackKLT::feed_stereo`): 左图 FAST → KLT 投影到右图 → 成功为 stereo feature，失败为单目 feature；右图也独立补充单目特征

#### 特征数据库写入
`FeatureDatabase::update_feature(id, timestamp, cam_id, u, v, un, vn)`: 对每个存活特征，去畸变后写入归一化坐标 `(un, vn)`

### 8.3 IMU 数据处理管线

#### 原始数据
- **规格**: 加速度计 `a ∈ R³` [m/s²]，陀螺仪 `w ∈ R³` [rad/s]，时间戳 `t` [s]
- **标定参数**: `_calib_imu_dw` (陀螺仪 scale/bias), `_calib_imu_da` (加速度计 scale/bias), `_calib_imu_tg` (g 敏感度), `_calib_imu_GYROtoIMU` (陀螺仪→IMU 旋转), `_calib_imu_ACCtoIMU` (加速计→IMU 旋转)
- **噪声配置**: `params.imu_noises` (accel_noise_density, gyro_noise_density, accel_random_walk, gyro_random_walk)

#### 接收与缓冲
`VioManager::feed_measurement_imu` → `VioManager.cpp:75`:
1. 计算 `oldest_time` = 最老 clone 时间戳或初始化窗口起始
2. `propagator->feed_imu(message, oldest_time)`: 写入 `imu_data` 向量，自动清理 `oldest_time - 0.10s` 之前数据
3. 未初始化时同步写入 `initializer->feed_imu(message, oldest_time)`
4. 已初始化+ZUPT 启用时同步写入 `updaterZUPT->feed_imu(message, oldest_time)`

#### IMU 传播 (`Propagator::propagate_and_clone`)
`ov_msckf/src/state/Propagator.cpp:33`:
1. **IMU 选取**: `select_imu_readings(imu_data, time0, time1)` 取区间内 IMU 测量，`time0=state→_timestamp+last_t_off`, `time1=timestamp+t_off_new`
2. **精确切边插值**: `interpolate_data` 在首尾生成虚拟 IMU 测量 (线性插值)
3. **三种积分模式**:
   - **离散积分** `predict_mean_discrete`: `v_{k+1} = v_k - g*dt + R_k^T*(a-ba)*dt`, `p_{k+1} = p_k + 0.5*(v_k+v_{k+1})*dt`
   - **RK4** `predict_mean_rk4`: 四阶龙格库塔
   - **解析** `predict_mean_analytic` (ACI²): 分段加速度闭式积分
4. **协方差传播** (`predict_and_compute` → `StateHelper::EKFPropagation`):
   - 状态转移矩阵: `F = Φ(Δt, a, w, R)` (15×15 误差态)
   - 噪声协方差: `Qd = G*Q*G^T*Δt` (15×15)
   - 全状态协方差: `P = Φ*P*Φ^T + Qd` (含 IMU 内参 + clone 交叉协方差)
5. **状态增广** (`StateHelper::augment_clone`): 当前 IMU 位姿插入为 `_clones_IMU`

#### 高频输出 (`Propagator::fast_state_propagate`)
Propagator 内部缓存维护，以 IMU 频率调用: 仅传播 13 维 IMU 均值 + 12 维协方差，输出 IMU 频率里程计

### 8.4 算法消费：MSCKF 观测因子

#### 特征测量准备 (`UpdaterMSCKF::update`)
`ov_msckf/src/update/UpdaterMSCKF.cpp:58`:
1. **测量清理**: 只保留 clone timestamp 上的观测
2. **相机位姿构造**: `T_CW = T_CI * T_IW(i)`，从 IMU clone + 外参链式计算
3. **三角化** `FeatureInitializer::single_triangulation(feat, clones_cam)`: 多视角线性三角化
4. **GN 精化** (可选) `single_gaussnewton(feat, clones_cam)`: 多视角 BA 精化 3D 坐标

#### 残差与雅可比 (`UpdaterHelper::get_feature_jacobian_full`)
```
观测模型: z = π(T_CW * P_W),  r = z_obs - z_pred (2维)
H_f = ∂r/∂P_W  (2×3, 对特征 3D 位置的 Jacobian)
H_x = ∂r/∂x    (2×[15+6N+...], 对全状态: IMU + clones + 外参 + 时间偏移)
```

#### 零空间投影 (MSCKF 核心)
```
nullspace = H_f^T.fullPivLu().kernel()        // 特征左零空间
H_proj = nullspace^T * H_x                     // 投影到零空间，消去特征坐标
res_proj = nullspace^T * residual
```

#### 堆叠 + Chi² 检验
- 所有特征 H_proj 堆叠为大型线性系统
- `r^T * S^{-1} * r` 与 `chi_squared_table[dof]` 比较 → 超出阈值剔除
- EKF 更新: `K = P*H^T*(H*P*H^T+R)^{-1}` → `x = x + K*(z-h(x))` → `P = (I-K*H)*P`

#### 信息矩阵
- 视觉噪声: `σ²=1.0` 像素² (归一化平面)，信息矩阵 `R^{-1} = I_{2×2}/σ²`
- IMU 噪声: 来自 `params.imu_noises` 配置，通过协方差传播建立

### 8.5 跨传感器协同

#### 时间同步
- `_calib_dt_CAMtoIMU`: `t_imu = t_cam + t_off`，作为在线标定变量
- 传播时所有时间戳加上偏移，`select_imu_readings` 根据偏移选取 IMU

#### 数据缓冲策略
- IMU: Propagator 内部 `imu_data` 向量 + mutex 保护，自动清理过期数据
- Camera: 帧级处理，无帧缓冲队列（同步处理）
- 初始化数据: `initializer` 独立线程持有深拷贝，不阻塞主跟踪

#### 初始化管线
1. **静态初始化** (`StaticInitializer`): 静止检测 → 加速度均值求重力方向 → `q_GtoI` → vel/pos/bias=0
2. **动态初始化** (`DynamicInitializer::initialize`): Ceres 非线性优化: 关键帧位姿 + 重力 + bias + 速度 + 特征 3D + 可选内外参，残差: `Factor_GenericPrior` + `Factor_ImageReprojCalib` + `Factor_ImuCPIv1`，对齐后构建初始 `_imu` 状态 + `_clones_IMU`

#### 降级与异常处理
- FEJ (First Estimate Jacobian): 所有状态变量在首次线性化点固定，保证 EKF 一致性
- Chi² 门控: 每个特征在 EKF 更新前卡方检验
- ZUPT: 零速检测 (`UpdaterZeroVelocity::try_update`)，构造虚拟零速约束
- Clone 管理: 超 `max_clone_size` 时边缘化最老 clone，相关 SLAM 特征做一次性 EKF 更新
