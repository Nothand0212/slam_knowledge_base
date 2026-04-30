# 相机数据管线横向对比

> 基于各项目源码级深度分析，覆盖 15 个 SLAM/VIO 算法的相机数据管线全貌
> 分析日期：2026-04-29

---

## 1. 概述

### 被比较的算法清单

| 类别 | 算法 | 核心方法 | 传感器 | 论文/年份 |
|------|------|---------|--------|-----------|
| **直接法** | DSO | 稀疏直接法光度 BA | 单目 | Engel, IROS 2018 |
| | DM-VIO | 直接法 VIO + 延迟边缘化 | 单目+IMU | von Stumberg, IROS 2022 |
| | ESVO | 事件相机立体直接法 | 双目事件 | Zhou, TRO 2021 |
| **间接法** | ORB-SLAM3 | ORB 特征 + 多地图 BA | 单/双/RGB-D/IMU | Campos, TRO 2021 |
| | OpenVINS | MSCKF EKF 滤波 | 单/双/多目+IMU | Geneva, IROS 2020 |
| | VINS-Fusion | KLT 光流 + 滑动窗口 BA | 单/双目+IMU | Qin, TRO 2019 |
| | SVO Pro | 半直接法 VIO | 单/多目+IMU | Forster, TRO 2017 |
| | Kimera-VIO | ISAM2 因子图 VIO + Mesh | 双目+IMU | Rosinol, TRO 2020 |
| **深度学习** | DROID-SLAM | CNN 光流 + Dense BA | 单目/RGB-D | Teed, NeurIPS 2021 |
| | MonoGS | 3D Gaussian Splatting SLAM | 单目/RGB-D | Matsuki, CVPR 2024 |
| | NICE-SLAM | 神经隐式 RGB-D SLAM | RGB-D | Zhu, CVPR 2022 |
| **滤波** | MSCKF_VIO | OC-MSCKF EKF | 双目+IMU | Sun, ICRA 2017 |
| | SchurVINS | Schur Complement EKF VIO | 双目+IMU | Xu, CVPR 2024 |
| | ROVIO | Patch-based 直接法 EKF | 单/双目+IMU | Bloesch, IROS 2015 |
| | OpenMAVIS | 多相机 ORB + SE2(3) 预积分 | 4 相机+IMU | Liu, IROS 2023 |

---

## 2. 原始数据规格对比

| 算法 | 相机类型 | 典型分辨率 | 频率 | 快门假设 | 原始图像格式 |
|------|---------|-----------|------|----------|-------------|
| **DSO** | 单目灰度 | 640x480 (EuRoC) | 20-30 Hz | 全局快门 | MinimalImageB3, 8bit 3ch BGR 转灰度 |
| **DM-VIO** | 单目灰度 | 640x480 (EuRoC) | 10-30 Hz | 全局快门 | cv::Mat CV_8UC1 Gray，经光度标定 |
| **ESVO** | 双目事件相机 | 346x260 (DAVIS240) / 640x480 (CeleX-V) | 异步，百万 events/s | N/A (异步传感器) | dvs_msgs::Event (x, y, t, polarity) |
| **ORB-SLAM3** | 单/双/RGB-D | 配置可变，典型 752x480 | 10-60 Hz | 全局/卷帘支持 | cv::Mat BGR 转 Gray + 可选 Depth |
| **OpenVINS** | 单/双/多目 | 配置驱动 | 10-60 Hz | 全局快门 | cv::Mat BGR/Gray，可降采样 pyrDown 2x |
| **VINS-Fusion** | 单/双目 | 配置可变 | 10-30 Hz | 全局快门 | cv::Mat CV_8UC1 灰度 |
| **SVO Pro** | 单/多目 (N cam) | 640x480 / 2x752x480 | 20-60 Hz | 全局快门 | cv::Mat 8/16bit 灰度 |
| **Kimera-VIO** | 双目灰度 | 752x480 (EuRoC MH) | 20 Hz | 全局快门 | Frame (cv::Mat + timestamp) 8bit 灰度 |
| **DROID-SLAM** | 单目/RGB-D | 自适应，240x320 / 370x480 | 15-30 Hz | 无严格要求 | torch tensor [B,N,3,H,W] uint8 |
| **MonoGS** | 单目/RGB-D | HxW 自适应 | 15-30 Hz | 无严格要求 | numpy uint8 -> torch tensor [C,H,W] CUDA |
| **NICE-SLAM** | RGB-D | HxW 自适应 | 15-30 Hz | 无严格要求 | numpy uint8 -> torch tensor, depth mm->m |
| **MSCKF_VIO** | 双目灰度 | 752x480 (EuRoC) | 10-30 Hz | 全局快门 | cv::Mat Gray (cv_bridge) |
| **SchurVINS** | 双目灰度 | 752x480 (EuRoC) | 10-30 Hz | 全局快门 | cv::Mat Gray CV_8UC1 (cv_bridge::toCvShare) |
| **ROVIO** | 单/双目灰度 | 752x480 (EuRoC) | 20-60 Hz | 全局快门 | cv::Mat 8bit gray -> ImgMeasurement |
| **OpenMAVIS** | 4 相机 (前视+侧视) | 752x480 (Hilti) | 10-30 Hz | 全局快门 | cv::Mat BGR -> Gray (cvtColor) |

**关键观察**: DSO/DM-VIO/ROVIO 对图像格式要求最严格(需要原始线性响应用于光度误差); 深度学习方法对分辨率和快门无特殊要求; 所有间接法都需要高纹理图像来提取足够的角点。

---

## 3. 标定管线对比

### 3.1 内参标定

| 算法 | 相机模型 | 内参来源 | 在线优化 | 畸变模型 |
|------|---------|---------|---------|---------|
| DSO | 自研 PinholeRadTan8 / PinholeEqui7 | camera.txt | **在线** (4 内参 -> BA 变量) | 径向+切向 |
| DM-VIO | 同 DSO | YAML / 数据集文件 | **在线** (继承 DSO) | 同 DSO |
| ESVO | CameraSystem 双独立 | yaml -> readFromYaml() | 否 | getRectifiedUndistortedCoordinate() |
| ORB-SLAM3 | Pinhole / KannalaBrandt8 (鱼眼) | YAML | 否 | k1-k4, p1-p2; UndistortKeyPoints() |
| OpenVINS | Pinhole / MEI / Equi / KB4 | Kalibr 格式 YAML, CamBase 虚基类 | **支持在线** | initUndistortRectifyMap |
| VINS-Fusion | camodocal (Pinhole/MEI/Equidistant/CataCamera) | YAML | 否 | liftProjective() |
| SVO Pro | PinholeProjection / EquidistantProjection | calib/camera_rig.yaml | 否 | 无传统去畸变 (直接法用原图) |
| Kimera-VIO | StereoCamera (左右独立) | YAML -> CameraParams::parseYAML() | 否 | initUndistortRectifyMap + remap |
| DROID-SLAM | 针孔模型 (4 floats) | 每帧传入 track(..., intrinsics=K) | 否 | N/A (网络隐式处理) |
| MonoGS | 针孔 -> Camera 对象 | YAML config | 否 | N/A (微分渲染) |
| NICE-SLAM | 针孔 | YAML config | 否 | N/A (射线可微渲染) |
| MSCKF_VIO | radtan / equidistant | Kalibr camchain-imucam-*.yaml | 否 | cv::undistortPoints / cv::fisheye |
| SchurVINS | PinholeProjection / EquidistantProjection | calib/camera_rig.yaml | 否 | 无传统去畸变 |
| ROVIO | Pinhole / Equidistant | yaml -> Camera (ROSCamera::readFromROS) | 否 | 相机模型内部处理 |
| OpenMAVIS | Pinhole / KannalaBrandt8 | YAML (per-camera) | 否 | 鱼眼畸变为零，跳过 |

**关键差异**: 只有直接法(DSO/DM-VIO)在线优化相机内参，所有间接法假设标定完美。滤波方法(OpenVINS/MSCKF_VIO/ROVIO)支持外参在线估计。

### 3.2 外参标定 (T_cam_imu, T_cam_cam)

| 算法 | 外参来源 | 在线估计 | 特点 |
|------|---------|---------|------|
| DSO | 纯视觉，无 IMU | N/A | N/A |
| DM-VIO | YAML T_cam_imu (Camera->IMU) | 否 | PoseTransformationFactor 桥接 |
| ESVO | yaml -> CameraSystem (K, R, T 双独立) | 否 | 用于立体匹配极线搜索 |
| ORB-SLAM3 | YAML Tcb = T_cam_imu | 否 | 链式推导 |
| OpenVINS | T_CtoI 存于 state->_calib_IMUtoCAM[i] | **6 DoF 在线** | 时间偏移也在线 |
| VINS-Fusion | T_IC = [RIC, TIC] | **仅旋转在线** (ESTIMATE_EXTRINSIC=2) | 时间偏移 td 也在线 |
| SVO Pro | T_cam_imu 每相机独立 | 否 | T_cam_imu * T_world_imu.inverse() |
| Kimera-VIO | StereoCamera 共享 (R_rect, baseline) | 否 | 左右 R1,R2 整流矩阵 |
| MSCKF_VIO | T_imu_cam0, T_imu_cam1 链式 (Kalibr) | **6 DoF 在线** (在状态向量中) | 滤波状态变量 |
| SchurVINS | T_cam0_imu + T_cam1_cam0 | 不支持 | 外参雅可比被注释掉 |
| ROVIO | IMU->Camera (_vep/_vea) | **在线** (doVECalibration_) | 外参-路标耦合雅可比自动传播 |
| OpenMAVIS | mTlr/mTlsl/mTlsr + Tcb (4 组) | 否 | 链式 SE3 推导，不支持在线 |

### 3.3 光度标定 (直接法特有)

| 算法 | 光度标定方式 | 内容 | 在线/离线 |
|------|------------|------|----------|
| DSO | CalibHessian 类 | 响应函数 B[256]/Binv[256] (256-bin LUT)，渐晕 V(x,y) | **在线** |
| DM-VIO | 继承 DSO + responseCalibration | vignette + response + exposure | **在线** |
| ROVIO | 仿射光照 | alpha/beta (useIntensityOffset_/useIntensitySqew_) | 在线 (在亮度误差中) |
| SVO Pro | Sparse Image Alignment 仿射 | alpha/beta | 在线 (前端优化变量) |
| MonoGS | 每帧 exposure_a, exposure_b | Adam lr=0.01 学习 | 在线 |
| 其余算法 | 无光度标定 | 特征鲁棒性 或 直方图均衡化 | N/A |

### 3.4 在线/离线策略总结

| 标定策略 | 代表算法 |
|----------|---------|
| **完全离线** | ORB-SLAM3, Kimera-VIO, NICE-SLAM, VINS-Fusion(内参), SchurVINS, ESVO |
| **内参在线优化** | DSO, DM-VIO |
| **外参在线全估计** | OpenVINS, MSCKF_VIO, ROVIO |
| **外参部分在线** | VINS-Fusion (仅旋转) |
| **时间偏移在线** | OpenVINS, VINS-Fusion, DM-VIO |
| **曝光在线学习** | DSO, DM-VIO, SVO Pro, MonoGS |

---

## 4. 图像预处理对比

### 4.1 去畸变与立体校正

| 算法 | 去畸变方式 | 立体校正 | 时机 |
|------|----------|---------|------|
| DSO | 畸变参数在投影/反投影中显式处理 | N/A (单目) | 实时 (在雅可比中体现) |
| ORB-SLAM3 | cv::remap(imLeft, imLeftRect, M1l, M2l) | **在线立体校正** cv::remap() | 每帧 |
| OpenVINS | CamBase::undistort() | 无显式校正 | 进入跟踪器前 |
| VINS-Fusion | liftProjective(pt) (camodocal) | 无显式校正 | 角点检测后 |
| SVO Pro | **无传统去畸变** (直接法用原图) | 无 | N/A |
| Kimera-VIO | initUndistortRectifyMap + remap | **立体整流** (行对齐) | 实时 |
| MSCKF_VIO | cv::undistortPoints / cv::fisheye | 发布前去畸变 | publish() 中 |
| SchurVINS | **无传统去畸变** (保留原图) | 无 | N/A |
| ROVIO | 相机模型内投影/反投影 | N/A | 实时 |
| DROID-SLAM/MonoGS/NICE-SLAM | 无显式去畸变 | N/A | 网络/渲染隐式处理 |

### 4.2 金字塔构建参数

| 算法 | 最大层数 | 缩放因子 | 构建方式 | 用途 |
|------|---------|---------|---------|------|
| DSO | 6 层 (L0-L5) | 2.0 | cv::pyrDown() 降采样 | 直接法粗到细跟踪+选点 |
| DM-VIO | 同 DSO 6 层 | 2.0 | 同 DSO | 同 DSO |
| ORB-SLAM3 | 8 层 | 1.2 | ComputePyramid() 自写 | ORB 尺度不变性 |
| OpenVINS | pyr_levels 层 | 2.0 | cv::buildOpticalFlowPyramid | KLT 光流 |
| VINS-Fusion | 1~3 层 (动态) | 2.0 | calcOpticalFlowPyrLK 内部 | KLT 光流 |
| SVO Pro | img_align_max_level=4 | 2.0 | 图像金字塔 (自写) | 稀疏图像对齐 |
| Kimera-VIO | klt_max_level_=3 | 2.0 | KLT 内部金字塔 | 时间追踪 |
| MSCKF_VIO | 3 层 | 2.0 | cv::buildOpticalFlowPyramid | KLT 光流 |
| SchurVINS | n_pyr_levels (3-4) | 2.0 | addImageBundle 内构建 | 直接法对齐 |
| ROVIO | nLevels=4 | 2.0 | ImagePyramid (自写) | 多层块对齐 |
| DROID-SLAM | N/A | CNN stride=2 逐步降采样 | BasicEncoder | 特征编码+相关体 |
| MonoGS/NICE-SLAM | N/A | 层次化网格 / 微分栅格化 | 特征网格/微分渲染 | 场景表示 |

**关键观察**: 直接法使用 2.0 大步长金字塔 (粗到细收敛快)，ORB-SLAM3 使用 1.2 小步长 (精细尺度不变性)，KLT 光流都使用 2.0 缩放因子和 3 层金字塔。

### 4.3 直方图均衡与光照预处理

| 算法 | 均衡化方式 |
|------|----------|
| OpenVINS | NONE / HISTOGRAM / CLAHE 三模式可配，改善 KLT 光照鲁棒性 |
| MonoGS | 曝光补偿: exp(exposure_a)*image + exposure_b，Adam 在线学习 |
| DROID-SLAM | ImageNet 标准化 (img-mean)/std |
| NICE-SLAM | 无 (直接使用 GPU tensor) |
| DSO/DM-VIO | 光度标定 (响应函数+渐晕+曝光归一化)，远强于直方图均衡 |
| 其余 | 无，直接使用原始灰度图像 |

---

## 5. 特征/表征提取对比

### 5.1 间接法 -- 特征点提取参数

| 算法 | 检测器 | 阈值 | 网格策略 | 亚像素精化 | 描述子 |
|------|--------|------|---------|-----------|--------|
| ORB-SLAM3 | cv::FAST | iniThFAST=20 / minThFAST=7 双阈值降级 | W=35px 网格 + **四叉树** DistributeOctTree() -> 递归分 4 子节点 | 叶节点保留 Harris 响应最高 | **Steered BRIEF** 32字节(256位), IC_Angle 灰度质心方向, 旋转补偿 R(theta)*pattern |
| OpenVINS KLT | cv::FAST | grid_threshold | grid_x * grid_y 均匀网格 + grid_2d_close 精细网格(min_px_dist) | cv::cornerSubPix (5x5, 20 iters) | 无 |
| OpenVINS Desc | Grider_FAST (cv::FAST) | threshold | 同 KLT, 每 cell 按 response top-N | cv::cornerSubPix | ORB (cv::ORB::compute) |
| VINS-Fusion | **Shi-Tomasi** (cv::goodFeaturesToTrack) | quality=0.01 | setMask() 按 track_cnt 排序, MIN_DIST 半径冲突丢弃 | cornerMinEigenVal 内部 | **无** (纯 KLT 光流) |
| SVO Pro | FAST(默认)/Shi-Tomasi | FAST=20 | cell_size=30px + closeness_check_grid | 无 | **无** (半直接法) |
| Kimera-VIO | **GFTT**(默认)/FAST/ORB | quality=0.01, minDist=10 | **ANMS** (Adaptive NMS) 非 bucketing | GFTTDetector 内部 | 无 (KLT 光流) |
| Kimera 回环 | ORB (独立于前端) | cv::ORB::create | 独立检测器 | 无 | ORB (用于 DBoW2) |
| MSCKF_VIO | cv::FastFeatureDetector | 10 (EuRoC) | 4x5=20 格, 每格 3-4 特征 | 无 | 无 (KLT + 2point RANSAC) |
| SchurVINS | FastDetector | 10 | OccupancyGrid2D 25px, 每格 1 点 | 无 | 无 (半直接法) |
| OpenMAVIS | ORB (同 ORB-SLAM3) | 同 ORB-SLAM3 | 4 相机独立 mGrid[4][64][48] | 四叉树同 ORB-SLAM3 | Steered BRIEF |

**核心发现**:
- FAST 角点检测是主流 (9/10 间接法使用)，阈值集中在 10-20
- 网格策略是普遍选择，但从简单网格(OpenVINS)到四叉树(ORB-SLAM3)复杂度不同
- VINS-Fusion 是唯一使用 Shi-Tomasi 而非 FAST 的主流 VIO
- Kimera-VIO 用 ANMS 而不用网格 bucketing
- SVO Pro/SchurVINS 只用角点检测做 seeding，不依赖描述子匹配

### 5.2 直接法 -- 光度模式选点策略

| 算法 | 选点策略 | Pattern | 梯度阈值 | 点数量 |
|------|---------|--------|---------|--------|
| DSO | PixelSelector::makeMaps() 网格采样+自适应密度(sparsityFactor) | 8-point 圆形 pattern (patternP[8]) | minUseGrad_pixsel=10, 3级: type=1/2/3 | 2000 (desiredImmatureDensity) |
| DM-VIO | 继承 DSO PixelSelector | 同 DSO 8-point | 同 DSO | 2000 |
| ROVIO | **无选点**，直接 Patch 处理 | 整块 patchSize^2 (默认 8x8) | Shi-Tomasi 分值 s=lambda1+lambda2 | 20-30 路标 (nMax) |
| SVO Pro | 关键帧 FAST/Shi-Tomasi 检测 | 8x8 patch | FAST=20 | max_fts=180 |
| DROID-SLAM | **无选点** (全像素) | 全对相关体 (H/8 x W/8 所有像素) | 网络学习 weight | 全部像素 (Dense BA) |
| MonoGS | **无选点**，Gaussians 渲染 | 3D Gaussian primitives | 无硬阈值 | 百万级 Gaussians |
| NICE-SLAM | 随机射线采样 | 沿射线采样 16+4 点 | 无硬阈值 | tracking_pixels=1024 条射线 |

**关键对比**: DSO 类直接法选点策略最复杂(梯度+方向分散+自适应网格)，ROVIO 最简化(直接 Patch)，DROID-SLAM 最激进(全像素 Dense BA)。

### 5.3 事件相机 -- ESVO 的 Time Surface 方法

| 特性 | 参数/方法 | 说明 |
|------|----------|------|
| 事件累积 | EventQueue = deque (time-sorted, max 3M events) | FIFO 按时间排序队列 |
| Time Surface | TS(x,y) = exp(-(T - t_e) / tau), tau=30ms | 指数衰减时间表面 |
| 平滑 | cv::GaussianBlur(TS, kernelSize=15) | 减少事件噪声 |
| 极性处理 | ignore_polarity_=true 忽略/OFF 用 Negative TS (255-TS) | 两种极性模式 |
| 梯度 | cv::Sobel(TS, dTS_du, CV_64F, 1, 0) | 用于 Tracking 解析雅可比 |
| 特征抽取 | createEdgeMask() + createDenoisingMask() + 随机采样 20000 events | 边缘保留+空间密度去噪 |
| 立体匹配 | ZNCC patch_size=25x25, disparity=[1,40], threshold=0.1 | 粗+精搜索(step=5) |

### 5.4 深度学习 -- 特征网络与表示

| 算法 | 网络结构 | 特征分辨率 | 输出维度 | 预训练 |
|------|---------|-----------|---------|--------|
| DROID-SLAM | BasicEncoder (ResNet-style, 3层下采样) + ConvGRU | H/8 x W/8 | fmap=128D, cnet=256D | TartanAir 合成数据集 |
| MonoGS | 无网络 (Gaussian 场景表示) | 全分辨率 | xyz(3)+rgb(3)+scale(3)+rot(4)+opacity(1)=59 参数/Gaussian | 可选 ply checkpoint |
| NICE-SLAM | 四级特征网格 + 4 个 MLP 解码器 (共享) | grid_len: 2m/0.16m/0.16m/0.16m | c_dim=32 每格 | ConvONet 大规模 3D 预训练 |

**DROID-SLAM 特征提取流程**:
- `DroidNet.extract_features()`: images[:,:,[2,1,0]]/255.0 -> ImageNet 标准化
- `fnet(images)` -> [B,N,128,H/8,W/8] 匹配特征
- `cnet(images)` -> [B,N,256,H/8,W/8] 上下文特征 (net tanh + inp relu 分组)
- 全对相关: `C(i,j) = <f_i/4, f_j/4>` (4D 张量)

---

## 6. 匹配与跟踪对比

### 6.1 KLT 光流参数对比

| 算法 | 窗口大小 | 金字塔层数 | 最大迭代 | 停止精度 | 初始猜测 | 反向检查 |
|------|---------|-----------|---------|---------|---------|---------|
| DSO | N/A (直接法 8D LM) | 6 层 | LM stepsize/maxIters | 步长收敛 | 恒速+20 变体 | N/A |
| VINS-Fusion | 21x21 | 1 (有预测) / 3 (无预测) | 30 (COUNT) | 0.01 (EPS) | OPTFLOW_USE_INITIAL_FLOW (IMU 预测) | FLOW_BACK (距离<0.5px) |
| OpenVINS | win_size (可配) | pyr_levels | 30 | 0.01 | OPTFLOW_USE_INITIAL_FLOW | RANSAC (F 矩阵) |
| Kimera-VIO | klt_win_size_=21 | klt_max_level_=3 | klt_max_iter_=30 | klt_eps_=0.001 | OpticalFlowPredictor (IMU 旋转) | RANSAC 5-point/2-point |
| MSCKF_VIO | 15x15 | 3 | 30 | 0.01 | IMU 旋转预测 H=K*R*K^-1 | 2-point RANSAC (custom) |
| SVO Pro | 8x8 patch | img_align_max_level=4 | 30 (1D align) | 块对齐收敛 | 恒速模型 | MAD + Tukey biweight |
| SchurVINS | N/A (直接 EKF) | n_pyr_levels (3-4) | 1 (single EKF) | EKF 更新 | IMU RK4 预测 | 4.0px/3.0px 阈值 |

**典型 KLT 参数配置**: 窗口 15x15-21x21, 3 层金字塔, 30 次迭代, 0.01 精度, IMU 旋转预测, 再加 RANSAC 精筛。

### 6.2 描述子匹配策略

| 算法 | 匹配器 | Ratio Test | KNN | 双向 | RANSAC 参数 |
|------|--------|-----------|-----|------|------------|
| ORB-SLAM3 | BoW (SearchByBoW) / 投影 (SearchByProjection) | 汉明距离阈值 50-100 | - | - | 单目: TwoViewReconstruction (H+F 并行, 8 点法), 重定位: MLPnP (置信度 0.99, 300 iters, 6 点 min, 0.5px) |
| OpenVINS Desc | cv::BFMatcher::knnMatch(k=2) | distance[0]/distance[1] > knn_ratio | k=2 | symmetry test | cv::findFundamentalMat(FM_RANSAC, 1/focal, 0.999) |
| OpenMAVIS | cv::BFMatcher(NORM_HAMMING) + knnMatch(k=2) | 0.8 (多相机) / 0.7 (鱼眼) | k=2 | 否 | MLPnP (0.99, 10/300, 6 点, 0.5px, chi2=5.991) |
| Kimera 回环 | DBoW2 (OrbVocabulary + OrbDatabase) | NSS 评分 + 时间一致性 | - | - | 2D-2D: 3-point RANSAC |
| VINS-Fusion 回环 | BRIEF (searchByBRIEFDes) | HammingDist < 80 | 4 best | - | PnPRANSAC (CV, 10px/460 thresh) |

### 6.3 直接法对齐策略

| 算法 | 对齐方法 | 优化变量 | 优化器 | 特殊技术 |
|------|---------|---------|--------|---------|
| DSO | 全帧光度 BA (CoarseTracker) | 6 DoF + 2 仿射 (8D) | LM (lambda=0.1 -> 自适应) | 多初始化策略 (20+), 6 层金字塔逐层下降 |
| DM-VIO | 继承 DSO + CoarseIMULogic | 同 DSO + IMU bias 先验 | LM + GTSAM 因子图 | 动态 DSO 权重 (RMSE 高->依赖 IMU) |
| SVO Pro | 1D 极线对齐 (Inverse Compositional) | 仅 1D (沿极线) | Gauss-Newton (Hessian 预计算在参考 patch) | 固定线性化点 (IC 方法) |
| ROVIO | 直接 Patch 光度误差 EKF | IMU(15D) + 路标(3D*N) + 外参(6D*M) | IEKF (迭代 EKF 候选生成) | MultilevelPatchAlignment, warp 预测 |
| SchurVINS | Schur Complement EKF | IMU(15D) + 窗口态(6D*4) | EKF + Schur + LDLT | FEJ 线性化点, 白化残差 (H=R) |

**逆组合 (Inverse Compositional)**: SVO Pro 和 ROVIO 使用此方法 -- Hessian 仅在参考 patch 计算一次，通过 warp 更新，极大节省计算量。DSO 不使用 IC，其 CoarseTracker 是全 LM 迭代。

---

## 7. 异常值剔除对比

| 算法 | 方法 | 模型 | 阈值 | 备注 |
|------|------|------|------|------|
| **DSO** | Huber 鲁棒核 + 在线 outlier 剔除 | 光度残差分布 | setting_huberTH=9, setting_outlierTH=12 | flagPointsForRemoval(): 残差>TH / OOB / idepth Hessian 不足 -> 删除 |
| **DM-VIO** | 继承 DSO + 动态权重 | 同 DSO + IMU 权重 | rmseThresh ~25 | 光度 RMSE 上升 -> 降低视觉权重 (tau/RMSE)^2 |
| **ESVO** | Student-t 分布 | TS 残差分布 | td_nu 自由度, td_scale | 残差>3*td_stdvar -> outlier |
| **ORB-SLAM3** | 卡方检验 + RANSAC (多层) | 重投影误差 | chi2(2)=5.991(单目), chi2(3)=7.815(双目) | Tracking: e^T*Sigma^{-1}*e < chi2; mnInliers < 0.25*mnMatches -> 失败 |
| **OpenVINS** | RANSAC + card方检验 | F 矩阵 + MSCKF 残差 | 2.0/focal_length (RANSAC), chi_squared_table[dof] | 去畸变 -> F 矩阵 RANSAC -> MSCKF card方门控 |
| **VINS-Fusion** | RANSAC(禁用) + 重投影阈值 | F 矩阵 (被 #if 1 禁用) | ave_err*FOCAL_LENGTH > 3 | rejectWithF 被禁，仅 outliersRejection 后处理 |
| **SVO Pro** | Tukey biweight + MAD scale | 重投影误差 | poseoptim_thresh=2.0px | w = (1-(r/(c*sigma))^2)^2, c=4.685 (95%效率) |
| **Kimera-VIO** | 三层 RANSAC (Mono/Stereo/PnP) | 5pt/2pt/3pt/1pt | mono=1.5px, stereo=0.01m | OpenGV 实现，IMU 辅助 2pt/1pt RANSAC |
| **DROID-SLAM** | **学习式置信度** | 网络 weight 输出 | 无硬阈值 | weight [B,N,2,H,W] 逐像素, GRU 隐式学习 |
| **MonoGS** | Opacity-aware weighting | 光度误差 | 无硬阈值 | Tracking: L=mean(opacity*|I_render-I_gt|), 低 opacity 区自动降权 |
| **NICE-SLAM** | 统计异常检测 | 渲染不确定度 | tmp < 10*median(tmp) | tmp=|gt_depth-render_depth|/sqrt(uncert+1e-10) |
| **MSCKF_VIO** | 自写 2-point RANSAC + card方 | 纯平移模型 + EKF 残差 | pixel_unit 阈值, chi2 95% | 退化检测: 平均位移<1px -> 纯旋转 |
| **SchurVINS** | Huber + 像素阈值 | EKF 重投影 | 4.0px(当前帧), 3.0px(平均) | RemoveOutliers + RemovePointOutliers |
| **ROVIO** | 多级检查 (4 级) | 光度+辨别性 | patchRejectionTh_ ~30 | IEKF 收敛+边界+光度+辨别性 (4 方向 >=2 通过) |
| **OpenMAVIS** | ORB-SLAM3 继承 + MLPnP | 卡方 + RANSAC | 同 ORB-SLAM3 | 重定位: MLPnP (0.99, 10/300, 6 点, 0.5px) |

**关键观察**:
- 卡方检验 (95% 置信度) 是所有间接法标配
- 直接法用 Huber/Tukey 代替卡方
- 深度学习方法**完全不用硬阈值**，改由网络学习的置信度/不确定度取代
- ROVIO 的异常值筛选最严格 (4 级检查)
- DROID-SLAM 和 MonoGS 的置信度学习代表了未来方向

---

## 8. 观测模型对比

| 算法 | 残差类型 | 维度 | 雅可比计算 | 信息矩阵 |
|------|---------|------|-----------|---------|
| **DSO** | 光度残差: r = I_target(pi(p')) - e^(a_j-a_i)*I_source(p) - (b_j-b_i) | 1D / pattern 点 (8 个残差 / 点) | 解析: dr/dxi_target, dr/dxi_source, dr/d_idepth, dr/da, dr/db (Residuals.cpp:78-274) | pixel_weight * gradient_weight, Huber 白化 |
| **DM-VIO** | 同 DSO + IMU 预积分 (9D) | 1D (光度) + 9D (IMU) | 同 DSO + GTSAM ImuFactor + PoseTransformationFactor | DSO Hessian + IMU 协方差逆, 动态权重 |
| **ESVO** | TS 一致性: r = TS_left(x1) - TS_right(x2) | patchSize^2 / 点 | 解析 (链式法则) / 数值 (Eigen LM) | Student-t 权重 (td_nu, td_scale) |
| **ORB-SLAM3** | 重投影误差: r = [u_obs-u_proj, v_obs-v_proj] | 2D (单目) / 3D (双目) | G2O 自动 (EdgeSE3ProjectXYZ / EdgeStereoSE3ProjectXYZ) | 1/sigma^2(level)*I, sigma^2=1.2^(2*level) |
| **OpenVINS** | 重投影 + IMU (MSCKF 零空间投影后) | 2D 经零空间投影后变 2D*N-3 | 解析 (UpdaterHelper::get_feature_jacobian_full) | R=I/sigma^2 (sigma=1.0 pixel on normalized plane) |
| **VINS-Fusion** | 重投影误差 (归一化平面) + IMU 预积分 (15D) | 2D (vision) + 15D (IMU) | Ceres 自动求导 (ProjectionTwoFrameOneCamFactor) | sqrt_info=focal_length/1.5*I, IMU: integrated cov inverse |
| **SVO Pro** | 前端: 光度 r=I_cur-T[I_ref]; 后端: 重投影 r=pi(T*P)-p_obs; IMU: 预积分(15D) | 8D (光度) / 2D (重投影) / 15D (IMU) | 前端: 解析+缓存; 后端: Ceres 自动 | 前端: 对角像素噪声; 后端: Cauchy/Huber |
| **Kimera-VIO** | Smart Stereo Projection (Schur 消去 landmark) + IMU 预积分 | 2D 经 SmartFactor 压缩后 | GTSAM SmartStereoProjectionPoseFactor + ImuFactor | GTSAM 自动 (因子图信息矩阵) |
| **MSCKF_VIO** | 重投影 + IMU (MSCKF 零空间投影后) | 4D (双目) -> 零空间投影 | 自写解析 (measurementJacobian, featureJacobian) | sigma_obs^2 (固定像素噪声) |
| **SchurVINS** | 重投影 (白化后) + IMU 直接积分 | 2D (白化后) + 15D (IMU 误差态) | 自写解析: dr/dPc -> dPc/dPos -> dPc/dPw | R=H (白化残差使 Hessian=观测协方差) |
| **ROVIO** | 直接 Patch 光度 + IMU 直接积分 | patchSize^2 (光度) + 15D (IMU) | 自写解析: A_red (降阶 2x2) -> c_J -> featureOutputJac | 从对齐 Hessian 推导 (噪声 = noiseGain * A_red^{-1}) |
| **DROID-SLAM** | 光流残差: r = target - coords (GRU 预测 vs 投影) | 2D / 像素 | 解析: Ji=dpi/dGi(2x6), Jj=dpi/dGj(2x6), Jz=dpi/ddi(2x1) | w_i^2 * I (pixel-wise weight * valid * 0.001) |
| **MonoGS** | 光度渲染: r = I_render(u,v) - I_obs(u,v) | 3D (RGB) / 像素 | Autograd through differentiable rasterization | 隐式单位阵 (MSE loss, opacity-weighted) |
| **NICE-SLAM** | 可微渲染: r_depth + r_color | 1D (深度) + 3D (颜色) | Autograd through volume rendering | 隐式单位阵 (MSE loss) |
| **OpenMAVIS** | 同 ORB-SLAM3 重投影 + SE2(3) IMU 预积分 (9D) | 2D/3D (vision) + 9D (IMU) | G2O 自动 (EdgeMono(cam_idx) / EdgeInertial) | 同 ORB-SLAM3 + 预积分协方差逆 (特征值裁剪 <1e-12 -> 0) |

**核心差异总结**:
- **残差维度**: 光度残差是 1D (标量), 重投影是 2D/3D, IMU 预积分是 9D/15D
- **雅可比方式**: DSO/SchurVINS/ROVIO 手写解析雅可比, VINS-Fusion/SVO Pro/Kimera 用 Ceres/GTSAM 自动微分, DROID-SLAM 用 lietorch 显式构造, MonoGS/NICE-SLAM 用 PyTorch autograd
- **信息矩阵**: 直接法用 Huber/Tukey 鲁棒权重, 间接法用 pyramid-level 缩放逆方差, 深度法用网络 confidence

---

## 9. 设计模式总结

### 9.1 间接法 vs 直接法的取舍矩阵

| 决策维度 | 间接法 (ORB-SLAM3/VINS/OpenVINS) | 直接法 (DSO/DM-VIO/ROVIO) | 半直接法 (SVO Pro) |
|----------|----------------------------------|---------------------------|---------------------|
| **图像信息利用** | 几百个特征点坐标 (丢弃 99%+ 像素) | 数千个高梯度点 + 整张图像梯度 | 前两步用光度，第三步用几何 |
| **纹理要求** | 必须有强角点 (白墙失效) | 任何有梯度的区域均可 (更宽容) | 介于两者之间 |
| **光照鲁棒性** | 高 (描述子有一定不变性) | 低 (光度恒定假设严格) | 中 (后端几何约束兜底) |
| **计算开销** | 中 (描述子+匹配) | 高 (逐像素梯度+光度 Jacobian) | 低 (无描述子，前端快速) |
| **几何参数化** | 3D 点 (x,y,z) 优化 3 参数 | 逆深度 d 优化 1 参数 (更稀疏) | Seed -> 逆深度 (深度滤波器) |
| **初始化复杂度** | 高 (单目需 H/F 矩阵+三角化) | 中 (纯光度最小化) | 中 (深度滤波器) |
| **尺度可观性** | 需双目/IMU 提供尺度 | 单目不可观 (任意单位) | 需 IMU 提供尺度 |
| **回环支持** | 强 (描述子+词袋) | 弱 (无描述子) | 弱 (依赖外部) |

**关键权衡**: 
- 直接法用更多像素信息换取光照敏感性
- 间接法用特征稀疏性换取光照鲁棒性和回环能力
- SVO Pro 的"半直接"设计用前端直接法加速 + 后端间接法兜底，是最务实的工程方案

### 9.2 特征生命周期管理策略

```
特征状态机类型对比:
────────────────────────────────────────────────

DSO 三点状态模型:
  PixelSelector 选点 -> ImmaturePoint (极线搜索) -> PointHessian (进入 BA)
                                                        |
                                                        v
                                                  flagForRemoval -> marginalize

ORB-SLAM3 特征管理:
  FAST 检测 -> mvKeys (Frame) -> MapPoint -> LocalMapping (MapPointCulling)
                                   |
                                   +-> KeyFrame->mConnectedKeyFrameWeights (共视图)
                                   +-> LoopClosing (回环融合)
                                   +-> mObservations 多 KF 观测列表

VINS-Fusion 光流跟踪:
  goodFeaturesToTrack -> track_cnt=1 -> KLT 跟踪 -> track_cnt++ -> 丢失则丢弃
  生命周期: 仅存在于滑动窗口 (10+1 帧) 内，无长期特征存储

OpenVINS MSCKF:
  FAST 检测 -> Klf/desc tracking -> FeatureDatabase.update_feature()
           -> MSCKF update (用完即丢) OR SLAM feature (持久化)
  MSCKF 特征不存状态，SLAM 特征增入滤波器

SVO Pro Depth Filter:
  FAST 检测 -> Seed (mu, sigma^2) -> 极线 Bayesian 更新
           -> sigma^2 收敛 -> 提升为 3D Landmark -> 进入 BA

ROVIO Patch 管理:
  ShiTomasi 添加 -> EKF 状态中估计 bearing+depth
                 -> block update 条件满足时更新 reference patch
                 -> 状态维度限制 (nMax) 决定存活数量
```

**设计要点**:
- DSO 的 ImmaturePoint -> PointHessian 两级是最精细的点质量管控
- ORB-SLAM3 的特征管理最完整 (长期共视图+回环)
- MSCKF 的"用完即丢"是最激进的简化，VINS-Fusion 类似
- SVO Pro 的 Bayesian 深度滤波器是最优雅的不确定性建模
- ROVIO 的 Patch 直接状态存储是最特别的设计

### 9.3 多相机处理模式

| 算法 | 多相机策略 | 相机间匹配 | 位姿推导 | 特点 |
|------|-----------|-----------|---------|------|
| **ORB-SLAM3** | 单/双目独立通道 | 左右立体匹配 (BFMatcher + ratio test) | 独立 KF 位姿 | 6 种传感器模式 (mono/stereo/rgbd x IMU) |
| **OpenVINS** | CameraBundle 多相机 | 左右 KLT 跟踪 (双目模式), 右目独立补充单目 | IMU clone + 外参链式 | 支持任意数量相机并行 |
| **SVO Pro** | CameraBundle + FrameBundle | 多相机 sparse image alignment 联合优化 | T_cur_ref -> cur_cam * T_imu_world * ref_cam.inv() | 每相机独立 Reprojector |
| **Kimera-VIO** | 双目专用 (StereoImuPipeline) | 稀疏: NCC 沿极线; 稠密: cv::StereoSGBM/BM | 左右分别 fillQueue | 左右独立特征+立体匹配 |
| **MSCKF_VIO** | 双目 (message_filters::TimeSynchronizer) | KLT + 极线校验 (E=[t]x*R) | IMU clone + 外参链式 | 4x5 网格独立 |
| **SchurVINS** | 双目 (FrameHandlerStereo) | 立体三角化 (findEpipolarMatchDirect) | 同 SVO Pro | 4 窗口态 |
| **ROVIO** | 单/双/多目 (nCam 模板) | 跨相机测量 (useCrossCameraMeasurements_) | robotcentric 自动 | noiseGainForOffCamera_ 缩放 |
| **OpenMAVIS** | 4 相机 (前视左右+侧视左右) | **仅前视左右匹配** (ratio=0.8), 侧视无跨相机匹配 | 链式 SE3 (mTlr/mTlsl/mTlsr) 从主相机推导 | 侧视 2x 特征补偿 |

**设计哲学**:
- OpenVINS/ROVIO 的模板参数 nCam 是最优雅的多相机抽象
- OpenMAVIS 硬编码了特定硬件布局 (侧视不匹配)
- SVO Pro 的 FrameBundle 联合 Sparse Image Alignment 是最高效的多相机直接法
- 所有算法都通过外参链式推导相机位姿，不独立估计

### 9.4 技术路线全景图

```
方法纯度谱 (从几何到学习):
────────────────────────────────────────────────────────────
间接法(纯几何)                         直接法(光度)                 深度学习(数据驱动)
ORB-SLAM3  VINS  OpenVINS    SVO_Pro   DSO  ROVIO  DM-VIO    DROID  MonoGS  NICE-SLAM
    |         |       |          |         |     |      |        |       |       |
  ORB      Shi-     FAST      FAST+    高梯度  Patch  高梯度    CNN   3DGS   神经隐式
 特征      Tomasi   KLT      Direct   采样+   直接    采样+   特征+   渲染    网格
 匹配      光流     EKF      Align    光度    EKF    光度    Dense          +MLP
                                           BA           BA

特征密度谱 (从稀疏到稠密):
──────────────────────────────────────
稀疏(百级)                   半稠密(千级)                  全稠密(万/百万级)
ORB-SLAM3 OpenVINS MSCKF    DSO SVO DM-VIO Kimera    DROID MonoGS NICE-SLAM
  ~1000      80      80      2000   180   2000  ~500   全像素  百万Gaussian  全像素

优化方法谱 (从滤波到优化到学习):
─────────────────────────────────────────
EKF 滤波                  因子图优化              梯度下降学习
MSCKF ROVIO SchurVINS    ORB-SLAM3 VINS Kimera    DROID MonoGS NICE-SLAM
 (单次线性化)            (迭代 LM/ISAM2)          (Adam/autograd)
```

### 9.5 关键工程参考

| 参考来源 | 最佳实践 | 适用场景 |
|---------|---------|---------|
| DSO: 8-point pattern + 逆深度 1D 优化 | Schur 补消去点只需要标量求逆，O(1) 代价 | 任何因子图后端 |
| ORB-SLAM3: 三级 BA 体系 (Motion-only -> Local -> Full) | 计算量精准匹配实时性需求 | 需要多级优化的系统 |
| DM-VIO: 延迟边缘化 (DelayedGraph) | 等尺度收敛再边缘化，避免信息锁定 | 含 IMU 初始化的 VIO |
| OpenVINS: CamBase 虚基类 + TrackBase 可插拔 | 相机模型和跟踪器抽象层 | 需要多相机支持 |
| SVO Pro: Inverse Compositional 1D 对齐 | Hessian 只算一次，极线约束大幅降维 | 需要极低延迟的跟踪 |
| Kimera-VIO: Smart Stereo Projection Factor | 路标 Schur 消去 + 增量添加观测 | GTSAM 因子图系统 |
| MSCKF: OC-MSCKF 可观测性约束 | 修正 F 矩阵保持零空间维度一致 | 任何 EKF 系统 |
| SchurVINS: 白化残差使 Hessian=R | 简化 EKF 实现，数学等价 | 轻量 EKF 后端 |
| ROVIO: Patch 直接入 EKF 状态 | 完全不用特征提取/匹配 | 恒光、短程嵌入式 |
| DROID-SLAM: Dense BA 的 GPU Schur 补 | 深度 C 是对角阵 -> 逐元素求逆 -> O(P*M) | GPU 后端 |
| MonoGS: 微分栅格化 + Adam 位姿优化 | 端到端可微，渲染=匹配 | 需要高质量场景渲染 |
| NICE-SLAM: 层次化特征网格 + Frustum Selection | 粗到细特征+视野限制优化范围 | 可扩展的神经隐式表示 |

---
*文档结束。交叉引用 15 个 deep_dive 源文件，覆盖约 12 万行源码分析。*
