# SVO Pro 深度源码分析

> **SVO Pro: Semi-direct Visual Odometry**  
> University of Zurich, Robotics and Perception Group  
> 半直接法 VIO，结合直接法+特征法优势  
> 原始 SVO 开源 2014，Pro 版本 2019+
> GitHub: 1568 stars

---

## 1. 半直接法 (semi-direct): 如何结合直接法+间接法

SVO 的 "semi-direct"（半直接法）是其最独特的设计哲学。它并非两种方法的简单堆砌，而是在流水线的**不同阶段使用不同方法**，各取其长。

### 1.1 概念框架

```
SVO 流水线:
┌─────────────────────────────────────────────────────────┐
│  Step 1: Sparse Image Alignment (直接法)                 │
│  通过最小化整幅图像的光度误差估计帧间运动                │
│  输入: 参考帧的 3D 点 + 当前帧图像                       │
│  输出: 当前帧的初始位姿                                  │
├─────────────────────────────────────────────────────────┤
│  Step 2: Feature Alignment (直接法)                      │
│  对每个特征点在其极线上进行 1D 块对齐                    │
│  输入: 2D 特征位置 + 当前帧图像                          │
│  输出: 精确的 2D 匹配位置                                │
├─────────────────────────────────────────────────────────┤
│  Step 3: Pose + Structure Optimization (间接法)          │
│  通过最小化重投影误差优化位姿和 3D 点                    │
│  输入: 2D-3D 对应关系                                   │
│  输出: 精化的位姿和路标                                  │
└─────────────────────────────────────────────────────────┘
```

### 1.2 为什么这样设计？

**直接法的优势**（Step 1 & 2）：
- 不需要特征匹配，省去了描述子计算和暴力搜索
- 使用图像中所有可用信息（不只是角点）
- 对重复纹理鲁棒

**间接法的优势**（Step 3）：
- 重投影误差对光照变化不敏感
- BA（Bundle Adjustment）框架成熟，可以联合优化位姿和结构
- 支持鲁棒的异常值剔除（通过重投影误差阈值）

**关键洞察**：用直接法做（快但不够鲁棒的）前端初始化，用间接法做（慢但鲁棒的）后端精化。这种分工避免了 DSO 全直接法在光照变化时的脆弱性，也避免了 ORB-SLAM 全特征法在特征不足时的失败。

### 1.3 源码架构体现

SVO Pro 的模块划分直接反映了半直接法哲学：

| 模块 | 方法 | 关键类 | 位置 |
|------|------|--------|------|
| `svo_img_align` | 直接法 | `SparseImgAlign` | `svo_img_align/include/` |
| `svo_direct` | 直接法 | `feature_alignment`, `DepthFilter` | `svo_direct/include/` |
| `svo_ceres_backend` | 间接法 | `Estimator` | `svo_ceres_backend/include/` |
| `svo` | 间接法 | `PoseOptimizer`, `Reprojector` | `svo/include/` |

---

## 2. 前端: Sparse Model-based Image Alignment (不用特征匹配!)

### 2.1 核心思想

`SparseImgAlign`（`svo_img_align/include/svo/img_align/sparse_img_align.h:30-78`）实现了前端最关键的位姿估计。它的输入是：
- 参考帧（上一帧）的带有**已知 3D 位置**的特征点
- 当前帧的图像

它通过最小化**每个特征点 patch 的光度误差**来估计当前帧相对于参考帧的位姿，**完全不需要 2D-2D 特征匹配**。

### 2.2 算法流程

在 `sparse_img_align.cpp:34-113` 中：

**Step 1: 特征选择**（行 42-51）
```cpp
void SparseImgAlign::run(const FrameBundle::Ptr& ref_frames, const FrameBundle::Ptr& cur_frames){
    fts_vec_.clear();
    for(auto frame : ref_frames->frames_){
        std::vector<size_t> fts;
        sparse_img_align_utils::extractFeaturesSubset(
              *frame, options_.max_level, patch_size_with_border_, fts);
        fts_vec_.push_back(fts);
    }
```

这里选择所有可见的特征（3D 位置已知），对每个特征检查其 patch 是否完全在图像内。

**Step 2: 预计算缓存**（行 65-91）

三个关键缓存：
- `uv_cache_`（2×N）：特征在参考图像中的像素坐标
- `xyz_ref_cache_`（3×N）：特征在 IMU 系中的 3D 位置
- `jacobian_proj_cache_`（6×N）：投影的雅可比（d proj(X)/d xi）

```cpp
sparse_img_align_utils::precomputeBaseCaches(
      *ref_frames_->at(i), fts_vec_.at(i),
      options_.use_distortion_jacobian,
      feature_counter, uv_cache_,
      xyz_ref_cache_, jacobian_proj_cache_);
```

**Step 3: 金字塔迭代优化**（行 93-99）
```cpp
for(level_=options_.max_level; level_>=options_.min_level; --level_){
    mu_ = 0.1;
    have_cache_ = false;
    optimize(state);
}
```

从最高层开始逐层优化，每层内使用 Levenberg-Marquardt 迭代。状态变量包含：
- `T_icur_iref`：当前帧 IMU 相对于参考帧 IMU 的变换（6-DoF）
- `alpha`：仿射光照增益（可选）
- `beta`：仿射光照偏置（可选）

### 2.3 关键数学：Photometric Error 的雅可比

在 `evaluateError`（`sparse_img_align.cpp:115-156`）中：

```cpp
double SparseImgAlign::evaluateError(const SparseImgAlignState& state, HessianMatrix* H, GradientVector* g){
    // 对每个参考帧
    for(size_t i=0; i<ref_frames_->size(); ++i){
        const Transformation T_cur_ref = cur_frames_->at(i)->T_cam_imu() * state.T_icur_iref 
            * ref_frames_->at(i)->T_imu_cam();
        sparse_img_align_utils::computeResidualsOfFrame(
              cur_frames_->at(i), level_, patch_size_, fts_vec_.at(i).size(), T_cur_ref,
              state.alpha, state.beta, ref_patch_cache_, xyz_ref_cache_,
              feature_counter, match_pxs, residual_cache_, visibility_mask_);
    }
    float chi2 = sparse_img_align_utils::computeHessianAndGradient(
          jacobian_cache_, residual_cache_, visibility_mask_, weight_scale_, weight_function_, H, g);
    return chi2;
}
```

雅可比构建的链式法则：
```
d(intensity) / d(pose) = d(intensity) / d(pixel) × d(pixel) / d(point_3D) × d(point_3D) / d(pose)
```

这个预计算缓存在 `jacobian_cache_` 中（8×N：6 维位姿 + 2 维仿射光照）。

### 2.4 特征对齐：1D Epipolar Search

在 `svo_direct/src/feature_alignment.cpp:31-200` 中，`align1D` 函数实现了沿极线方向的 1D 块对齐（对齐 Step 2）：

```cpp
bool align1D(const cv::Mat& cur_img, const Eigen::Ref<GradientVector>& dir,
             uint8_t* ref_patch_with_border, uint8_t* ref_patch, const int n_iter,
             bool affine_est_offset, bool affine_est_gain,
             Keypoint* cur_px_estimate, double* h_inv)
```

**核心数学**（`feature_alignment.cpp:54-65`）：
```cpp
const float dx = static_cast<float>(it[1]) - static_cast<float>(it[-1]);
const float dy = static_cast<float>(it[ref_step]) - static_cast<float>(it[-ref_step]);
J[0] = 0.5f * (dir(0)*dx + dir(1)*dy);  // 梯度沿极线方向的投影
J[1] = affine_est_offset? 1.0f : 0.0f;  // 偏置项
J[2] = affine_est_gain? -1.0f*it[0] : 0.0f;  // 增益项
H += J*J.transpose();
```

这使用了**逆组合 (inverse compositional)** 方法：
- Hessian 只计算一次（在参考块上）
- 每次迭代只需要计算残差和 Jres
- 更新量：`update = Hinv * Jres`，其中 `u += update[0]*dir[0]`, `v += update[0]*dir[1]`

约束：位移仅在 `dir`（极线方向）上，即 1D 搜索。

### 2.5 不使用特征匹配的全流程

1. **初始化**：第一帧上检测 FAST/Shi-Tomasi 角点（仅此一次需要角点检测）
2. **Sparse Image Alignment**：用上一帧的所有 3D 点直接估计位姿（不需要匹配）
3. **Reprojection**：将局部地图中的 3D 点投影到当前帧，通过 1D 极线对齐找到精确 2D 位置
4. **Pose Optimization**：用重投影误差优化位姿
5. **新的路标初始化**：深度滤波器在关键帧上初始化新的种子点

整个过程中没有描述子计算，没有暴力匹配搜索。

---

## 3. 后端: 重投影优化 (BA with reprojection error)

### 3.1 PoseOptimizer

`PoseOptimizer`（`svo/include/svo/pose_optimizer.h:20-101`）是一个轻量级的 pose-only BA：

```cpp
class PoseOptimizer : public vk::solver::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer>{
    enum class ErrorType {kUnitPlane, kBearingVectorDiff, kImagePlane};
};
```

关键参数（`frame_handler_base.h:157-167`）：
- `poseoptim_thresh = 2.0`：重投影误差阈值（像素）
- `poseoptim_using_unit_sphere = false`：使用 unit plane 还是 unit sphere
- `poseoptim_prior_lambda = 0.0`：旋转先验权重

**三种误差类型**（`pose_optimizer.h:33`）：
1. **kUnitPlane**：在单位平面上的 2D 误差（适用于小 FoV 针孔相机）
2. **kBearingVectorDiff**：bearing vector 间的角度差异（适用于鱼眼/全向相机）
3. **kImagePlane**：直接的图像像素重投影误差

优化方法（`pose_optimizer.cpp:40-94`）：
```cpp
size_t PoseOptimizer::run(const FrameBundle::Ptr& frame_bundle, double reproj_thresh_px){
    // 使用 Gauss-Newton + MAD scale estimator + Tukey robust weight
    Transformation T_imu_world = frame_bundle->at(0)->T_imu_world();
    measurement_sigma_ = scale_estimator_.compute(start_errors);
    optimize(T_imu_world);
    removeOutliers(reproj_thresh_px, f.get(), ...);
}
```

### 3.2 Ceres Backend: 完整的 Visual-Inertial BA

`Estimator`（`svo_ceres_backend/include/svo/ceres_backend/estimator.hpp:159-770`）实现了完整的后端优化。

**核心结构**：
```cpp
class Estimator{
    void addCameraBundle(...);
    int addImu(const svo::ImuParameters& imu_parameters);
    void addStates(...);
    void addLandmark(...);
    void addObservation(...);
};
```

后端维护一个**滑动窗口**，包含：
- **States**（`estimator.hpp:74-115`）：时间有序的帧状态列表，每个状态包含：
  - `BackendId`（帧 ID）
  - `is_keyframe`（是否为关键帧）
  - `timestamp`
- **路标**：3D 点，用 homogeneous 坐标表示
- **观测**：每个路标在多帧中的观测

**优化因子**：

1. **Reprojection Error**（`reprojection_error.hpp`）：
```cpp
// 标准的重投影误差: e = π(T_cw * P_w) - p_obs
```

2. **IMU Error**（`imu_error.hpp`）：预积分误差
```cpp
// e_imu = [e_R, e_v, e_p] 即旋转、速度、位置的预积分误差
```

3. **Relative Pose Error**（`relative_pose_error.hpp`）：帧间相对位姿约束

4. **Marginalization Error**：滑动窗口边缘化先验

5. **Map Alignment Error**（`map_alignment.hpp`）：与全局地图对齐（如果可用）

**边缘化策略**：
- 使用 Schur Complement 将最老帧的状态边缘化
- 边缘化先验以 `MarginalizationError` 的形式保留在优化中
- 线性化点固定在边缘化时的估计值（FEJ 风格）

### 3.3 IMU 预积分

在 `svo/include/svo/imu_handler.h:23-47` 中：

```cpp
class PreintegratedImuMeasurement{
    Eigen::Vector3d delta_t_ij_;  // 位置预积分
    Eigen::Vector3d delta_v_ij_;  // 速度预积分
    Quaternion delta_R_ij_;        // 旋转预积分
    double dt_sum_;
};
```

预积分在 `imu_handler.cpp:51-68` 中实现，使用二阶积分（中点法）：
```cpp
void PreintegratedImuMeasurement::addMeasurement(const ImuMeasurement &m){
    const double dt = m.timestamp_ - last_imu_measurement.timestamp_;
    // 二阶积分
    delta_t_ij_ += delta_v_ij_*dt + (delta_R_ij_.rotate(a))*dt*dt*0.5;
    delta_v_ij_ += delta_R_ij_.rotate(a*dt);
    delta_R_ij_ = delta_R_ij_ * Quaternion::exp(w*dt);
    dt_sum_ += dt;
}
```

偏置变化时的更新通过一阶近似实现，与 Forster et al. (TRO 2017) 的方法一致。

---

## 4. 多相机支持

### 4.1 CameraBundle 架构

SVO Pro 通过 `CameraBundle` 类（`svo_common` 中）原生支持多相机：

```
CameraBundle:
├── Camera 0 (主相机, 640x480)
├── Camera 1 (辅助相机, 640x480)
└── ...
```

每个相机有独立的：
- 内参 (`Camera` 对象)
- 外参 (`T_cam_imu_`，即相机到 IMU 的变换)

关键配置（`frame_handler_base.cpp:122-126`）：
```cpp
reprojectors_.reserve(cams_->getNumCameras());
for(size_t camera_idx=0; camera_idx<cams_->getNumCameras(); ++camera_idx){
    reprojectors_.emplace_back(new Reprojector(reprojector_options, camera_idx));
}
```

每个相机有自己的 Reprojector 实例。

### 4.2 FrameBundle

`FrameBundle`（`svo/include/svo/frame_handler_base.h:24` 中引用）封装了同一时刻的多相机帧：

```cpp
class FrameBundle {
    std::vector<FramePtr> frames_;  // 每个相机一帧
    // 时间戳相同，但图像和位姿不同
};
```

在 `addImageBundle`（`frame_handler_base.cpp:158-197`）中：
```cpp
bool FrameHandlerBase::addImageBundle(const std::vector<cv::Mat>& imgs, const uint64_t timestamp){
    CHECK_EQ(imgs.size(), cams_->getNumCameras());
    for(size_t i=0; i<imgs.size(); ++i){
        frames.push_back(std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(), timestamp, ...));
        frames.back()->set_T_cam_imu(cams_->get_T_C_B(i));
    }
    FrameBundlePtr frame_bundle(new FrameBundle(frames));
    return addFrameBundle(frame_bundle);
}
```

### 4.3 多相机 Sparse Image Alignment

在 `sparse_img_align.cpp:41-113` 中，对齐过程同时处理所有相机：

```cpp
for(size_t i=0; i<ref_frames_->size(); ++i){
    sparse_img_align_utils::precomputeBaseCaches(*ref_frames_->at(i), fts_vec_.at(i), ...);
}
```

优化变量是 `T_icur_iref`（当前帧 IMU 相对于参考帧 IMU），通过各自的外参映射到每个相机：
```cpp
const Transformation T_cur_ref = cur_frames_->at(i)->T_cam_imu() * state.T_icur_iref 
    * ref_frames_->at(i)->T_imu_cam();
```

### 4.4 多相机深度滤波

`DepthFilter`（`svo_direct/src/depth_filter.cpp:26-44`）为每个相机构建独立的检测器：
```cpp
feature_detector_ = feature_detection_utils::makeDetector(detector_options, cams->getCameraShared(0));
```

---

## 5. 与 DSO / ORB-SLAM3 的对比

### 5.1 与 DSO 对比

| 方面 | SVO Pro | DSO |
|------|---------|-----|
| 方法 | 半直接法 | 全直接法 |
| 前端 | Sparse image alignment (光度误差) + 1D 极线对齐 | 全图像光度BA |
| 后端 | 重投影误差 BA + IMU 预积分 | 光度误差 BA（滑动窗口） |
| 特征 | FAST/Shi-Tomasi 角点种子 | 高梯度像素采样 |
| 光照鲁棒性 | 较高（后端用几何误差） | 较低（全靠光度误差）|
| 计算效率 | 较高（稀疏，选择性优化）| 中等（稠密采样点）|
| IMU 融合 | 完整 VIO（预积分 + 后端BA）| 有限支持 |

DSO 的纯粹性使其在纹理丰富、恒光场景中精度很高，但 SVO Pro 的半直接法通过后端几何约束获得了更好的光照鲁棒性。

### 5.2 与 ORB-SLAM3 对比

| 方面 | SVO Pro | ORB-SLAM3 |
|------|---------|-----------|
| 方法 | 半直接法 | 全特征法 (ORB) |
| 前端 | 直接法对齐 + 1D 极线搜索 | ORB 提取 + 描述子匹配 |
| 后端 | 滑动窗口 BA + 边缘化 | 全局/局部 BA + 共视图 |
| 重定位 | 基于关键帧的对齐 | 基于 ORB 词袋 (DBoW2) |
| 回环检测 | 基于 PGO 的回环 | DBoW2 回环 + 全局 BA |
| 初始化 | 深度滤波器 (Bayesian) | 单目: 对极几何 + 三角化 |
| 地图管理 | 滑动窗口 + 全局地图模块 | 共视图 + Atlas 多地图 |
| 精度 | 中高 | 高（公认 SOTA） |
| 速度 | 快（无描述子开销）| 中等 |

**关键差异**：

1. **不需要描述子**：SVO 的最大计算优势，但也导致其在回环检测和重定位时只能依赖直接法对齐，精度不如 DBoW2。

2. **深度估计**：SVO 使用 Bayesian 深度滤波器（`DepthFilter`，基于 "Video-based Real-Time Multi View Stereo" by Vogiatzis & Hernández），而 ORB-SLAM3 使用直接三角化 + 重投影误差 BA。深度滤波器在少视图情况下更鲁棒但收敛慢。

3. **地图表示**：ORB-SLAM3 维护完整的全局共视图，SVO 使用滑动窗口 + 可选的全局地图。

4. **Scale 处理**：SVO Pro 的 IMU 预积分与后端 BA 紧密集成，尺度估计更准确。ORB-SLAM3 的 IMU 初始化需要专门的 15 秒初始化阶段。

### 5.3 系统定位对比

```
方法纯度谱:
DSO (100% 直接) ─────── SVO (50% 直接) ─────── ORB-SLAM3 (0% 直接)
         ↑                      ↑                        ↑
    全光度误差            半直接混合            全几何/描述子
```

SVO 占据了一个独特的位置：它既不是纯粹的直接法也不是纯粹的特征法，而是一个经过精心设计的混合体。

---

## 6. 数据管线

### 6.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| 全局快门相机 (N×cam) | 20-60 Hz | `cv::Mat` (8/16bit 灰度) | `FrameHandlerBase::addImageBundle()` | `svo::FrameHandlerBase` |
| IMU (加速度计+陀螺仪) | 100-200 Hz | `ImuMeasurement` (acc(3), gyr(3), ts) | `ImuHandler::addImuMeasurement()` | `ImuHandler` |
| CameraBundle 标定 | 一次性 | YAML → `CameraBundle` (每个相机: fx,fy,cx,cy + dist + T_cam_imu) | `camera_calibration.cpp` | `CameraBundle` |

### 6.2 相机管线

#### 原始数据
```
规格: 单目 640×480 (EuRoC) 或多相机 (如 2×752×480)
标定: CameraBundle存储每个相机的 Camera 对象 (pinhole / atan / equidistant)
       + T_cam_imu 外参 (每相机独立)
```

#### 预处理
**Step 1 — 帧创建** (`FrameHandlerBase::addImageBundle()`, `frame_handler_base.cpp:158`):
```
接收 N 张同步图像 → 创建 N 个 Frame 对象
每个 Frame 存储: cv::Mat img_, T_cam_imu_, T_imu_world_
Frame 内部不做预处理, 保留原始图像供后续直接法使用
```

**Step 2 — 曝光设置** (可选, `Camera::create()`):
```
曝光时间 / gain 存储在 Frame 对象中, 影响深度滤波器的光度不确定性建模
```

#### 特征提取
**SVO 不使用传统特征匹配**。特征提取仅在关键帧创建时进行:

**特征检测** (`DepthFilter::addKeyframe()` → `FeatureDetector::detect()`, `depth_filter.cpp:89`):
```
算法: FAST 角点检测 (默认) or Shi-Tomasi (GFTT)
参数: DetectorOptions
  - detector_type: kFast / kShiTomasi / kFastGrad
  - cell_size: 图像网格分块大小 (默认 30px)
  - max_level: 金字塔最高层
  - threshold_primary: FAST 阈值 (默认 20)
  - threshold_secondary: Shi-Tomasi 阈值
  - border: 图像边界留白 (默认 8px)
  - max_number_of_features_per_frame: 每帧最多特征数

采样策略: grid-based
  图像被分为 ceil(w/cell_size) × ceil(h/cell_size) 的规则网格
  每个格子使用 closeness_check_grid 确保空间均匀分布
  默认 cell_size=30 → 640/30≈21 × 480/30=16 ≈ 340 个格子
```

**Secondary 特征检测** (`options_.extra_map_points=true`):
```
额外在每个格子用 Shi-Tomasi 检测, 增加低纹理区域的覆盖
```

#### 匹配与异常值剔除
**1D 极线对齐** (`feature_alignment.cpp:align1D()`, line 31):
```
输入: 当前帧图像 + 参考帧 patch + 极线方向 dir
输出: 精化的 2D 匹配位置 cur_px_estimate
算法: Inverse Compositional 块对齐
  - Hessian 计算一次 (在参考 patch 上): H = Σ(J·J^T), J=[∂I/∂u沿dir, 1, -I]
  - 迭代更新: Δu = H⁻¹·(Σ J·res), 沿极线方向更新
  - 约束: 仅在极线上做 1D 搜索
参数:
  - n_iter: 最大迭代次数 (默认 30)
  - affine_est_offset: 是否估计亮度偏置
  - affine_est_gain: 是否估计亮度增益
```

**几何异常值剔除** (`Reprojector` + `PoseOptimizer`):
```
1. 重投影后 MAD scale estimator 计算 2D 误差标准差
2. Tukey biweight (默认) 鲁棒权重:
   w_i = (1 - (r_i/(c·σ))²)²  if |r_i|<c·σ,  else 0
   其中 c = 4.685 (Tukey 95% 效率)
3. 阈值: poseoptim_thresh = 2.0 px
   重投影误差 > 2.0 px → 外点, 从优化中移除
```

#### 算法消费 — 因子构建
**前端: Sparse Image Alignment** (`sparse_img_align.cpp:115-156`):
```
光度误差:
  r = I_cur(π(T_cur_ref · X_ref)) - (α·I_ref + β)
雅可比 (缓存在 jacobian_cache_, 8×N):
  ∂r/∂ξ (6D) = ∇I_cur · ∂π/∂x · ∂x/∂ξ
  ∂r/∂α, ∂r/∂β (2D)
信息矩阵: 对角, 权重 = 1/σ² (σ 来自像素噪声模型)
```

**后端: Ceres Reprojection Error** (`reprojection_error.hpp`):
```
重投影误差:
  r = π(T_cw · P_w) - p_obs  (2D)
雅可比 (自动微分 by Ceres):
  ∂r/∂T_cw (2×6), ∂r/∂P_w (2×3)
损失函数: Cauchy loss or Huber
```

**IMU Preintegrated Factor** (`imu_error.hpp`):
```
IMU 预积分误差 (15D):
  r_R = Log(ΔR_ij^T · R_i^T · R_j)
  r_v = R_i^T · (v_j - v_i - g·Δt) - Δv_ij
  r_p = R_i^T · (p_j - p_i - v_i·Δt - 0.5g·Δt²) - Δp_ij
雅可比: Ceres 自动微分 + 预积分偏置雅可比 (一阶近似)
协方差: 预积分过程中累积传播
```

**Marginalization Error** (`MarginalizationError`):
```
H_marg · Δx - b_marg (先验因子)
使用 FEJ 线性化点 (fixed linearization point)
```

### 6.3 深度滤波器管线

#### Seed 初始化
```
Seed 状态: (μ, σ²) 使用 Gaussian + Uniform 混合分布
初始范围: [depth_min, depth_max] (从场景先验设定)
每帧更新: 沿极线匹配 → Bayesian 更新 μ, σ²
收敛判断: σ² < seed_convergence_sigma2_thresh (默认 0.5)
```

#### 深度估计
```
算法: "Video-based Real-Time Multi-View Stereo" (Vogiatzis & Hernández 2011)
过程:
  1. 对每个 seed, 在 target 帧的极线上进行 ZNCC 块匹配
  2. 构造深度-代价函数 → Gaussian 近似 → 更新 μ, σ²
  3. 使用 Student-t 分布做 outlier 鲁棒性
  4. σ² 小到一定阈值 → 收敛为 3D 路标
```

### 6.4 跨传感器协同

#### 时间同步
```
相机-IMU 时间对齐: IMU 时间戳与图像帧时间戳对齐
  imu_handler.cpp: getClosestImuMeasurements() 按时间戳查找最近 IMU 数据
  视觉帧 timestamp → 查找 [t-frame, t+frame] 范围内的 IMU 测量
```

#### 数据缓冲
```
IMU 缓冲: ThreadsafeQueue<ImuMeasurement> (imu_buffer_)
  生产者: ImuHandler::imuCallback() (ROS 回调)
  消费者: ImuHandler::waitForImuMeasurementsUntil()

图像缓冲: 无 (流水线处理, 立即消费)
  每帧 addImageBundle() 直接触发 pipeline
```

#### 初始化管线
```
1. 首帧: 检测 FAST/Shi-Tomasi 特征 → 创建 Seeds (深度滤波器)
2. 后续帧: Sparse Image Alignment (恒速模型) → 1D 极点对齐 → Pose Optimization
3. 收敛 Seeds → 三角化 3D 点 → 地图构建
4. IMU 初始化 (如果有): 预积分累积 + 在线重力对齐
```

#### 降级策略
```
tracking 丢失:
  ↓ 恒速模型失败 → 恒静止模型
  ↓ 恒静止模型失败 → 多假设尝试 (旋转扰动)
  ↓ 全部失败 → 返回 kTrackingLost → 系统状态机重置
```

---

## 7. 优缺点 + 对SLAM算法的意义

### 6.1 优点

1. **计算效率极高**：无描述子计算，前端仅需光度误差和极线搜索
2. **光照鲁棒性平衡**：前端直接法快，后端间接法鲁棒，两者互补
3. **IMU 集成深度**：预积分 + 滑动窗口 BA，支持偏置在线估计
4. **原生多相机**：CameraBundle + FrameBundle 架构天然支持多相机
5. **深度滤波器**：Bayesian 深度估计在少视图场景下提供更好的不确定性建模

6. **模块化设计**：
   - `svo_common`：核心数据类型
   - `svo_img_align`：稀疏图像对齐（直接法前端）
   - `svo_direct`：特征对齐 + 深度滤波（直接法中层）
   - `svo`：位姿优化 + 路标管理（间接法模块）
   - `svo_ceres_backend`：滑动窗口 VIO BA（间接法后端）
   - `svo_online_loopclosing`：在线回环闭合
   - `svo_global_map`：全局地图构建

### 6.2 缺点

1. **重定位能力弱**：无描述子词袋，在跟踪失败后只能靠直接法对齐重定位
2. **深度滤波器收敛慢**：在纹理少或视差小的区域可能需要多帧才能收敛
3. **滑动窗口限制**：边缘化后的老帧信息被压缩为先验，丢失了细节关联
4. **初始化依赖**：深度滤波器需要合理的初始深度范围（`depth_min`, `depth_max`）
5. **大场景回环**：回环检测能力弱于 ORB-SLAM3 的 DBoW2 + 全局 BA 方案
6. **参数敏感**：`max_n_kfs`, `img_align_max_level`, `seed_convergence_sigma2_thresh` 等大量参数需针对场景调优

### 6.3 对 SLAM 算法的意义

SVO Pro 的最重要贡献是证明了**半直接混合架构**在实际机器人导航中的有效性：

1. **模块化设计的价值**：SVO Pro 将直接法和间接法清晰地划分到不同模块，使其可以独立优化和替换。例如，`PoseOptimizer` 可以替换为 GPU 加速版本，`DepthFilter` 的参数可以针对不同传感器调整。

2. **"够用就好"的设计哲学**：SVO Pro 不做完整的 SLAM（无全局优化），而是在 VIO 层面做到又快又准。这适合无人机、移动机器人等对延迟敏感的场景。

3. **IMU 深度集成的范例**：SVO Pro 的 IMU 处理（预积分 + 后端 BA）是 VIO 系统设计的典范，从传感器时间对齐到偏置估计都有完整实现。

4. **开源生态**：从原始的 `svo`（2014）到 `svo_ros`（ROS 接口）、`svo_benchmarking`（评估工具）、`svo_test_utils`（测试工具），形成了完整的开发和评估工作流。

### 6.4 与 ROVIO 的互补定位

SVO Pro 和 ROVIO 代表了两种不同类型的 "anti-feature" VIO 设计：

| | ROVIO | SVO Pro |
|------|-------|---------|
| 框架 | EKF（滤波） | 滑动窗口 BA（优化） |
| 观测 | 全光度误差 | 前端光度 + 后端几何 |
| 路标管理 | 在 EKF 状态中 | 深度滤波器 + 后端优化 |
| 适用场景 | 短程、低延迟、嵌入式 | 中程、平衡、桌面/嵌入 |

---

## 附录：关键源文件索引

| 文件 | 行数范围 | 核心内容 |
|------|----------|----------|
| `svo_img_align/include/svo/img_align/sparse_img_align.h` | 1-159 | 稀疏图像对齐类与工具函数声明 |
| `svo_img_align/src/sparse_img_align.cpp` | 1-655 | 前端位姿估计核心实现（光度误差最小化） |
| `svo_img_align/include/svo/img_align/sparse_img_align_base.h` | - | 对齐器基类（LM 优化、状态定义） |
| `svo_direct/src/feature_alignment.cpp` | 1-976 | 1D 极线块对齐（align1D, align2D） |
| `svo_direct/src/depth_filter.cpp` | 1-691 | Bayesian 深度滤波器实现 |
| `svo_direct/include/svo/direct/depth_filter.h` | 1-237 | 深度滤波器与匹配器配置 |
| `svo_direct/include/svo/direct/matcher.h` | - | 极线搜索匹配器 |
| `svo_direct/include/svo/direct/feature_detection.h` | - | 特征检测器接口 |
| `svo/include/svo/frame_handler_base.h` | 1-522 | 帧处理器基类与前端配置 |
| `svo/src/frame_handler_base.cpp` | 1-1258 | 前端主循环完整实现 |
| `svo/include/svo/pose_optimizer.h` | 1-187 | 位姿优化器（pose-only BA + 三类误差） |
| `svo/src/pose_optimizer.cpp` | 1-629 | 位姿优化的 Gauss-Newton 实现 |
| `svo/include/svo/imu_handler.h` | 1-193 | IMU 处理器（预积分+偏置管理） |
| `svo/src/imu_handler.cpp` | 1-609 | IMU 测量管理与时序对齐 |
| `svo_ceres_backend/include/svo/ceres_backend/estimator.hpp` | 1-770 | 滑动窗口 VIO BA 估计器 |
| `svo_ceres_backend/src/estimator.cpp` | - | 后端优化实现（Ceres problem 构建） |
| `svo_ceres_backend/include/svo/ceres_backend/imu_error.hpp` | - | IMU 预积分误差因子 |
| `svo_ceres_backend/include/svo/ceres_backend/reprojection_error.hpp` | - | 重投影误差因子 |
| `svo_ceres_backend/include/svo/ceres_backend/map.hpp` | - | Ceres 后端地图管理 |
| `svo/src/reprojector.cpp` | - | 路标重投影到当前帧 |
| `svo/src/initialization.cpp` | - | 系统初始化模块 |
| `svo/include/svo/map.h` | - | 前端地图（关键帧+路标管理） |
| `svo_common/include/svo/common/frame.h` | - | Frame 数据结构 |
| `svo_common/include/svo/common/point.h` | - | Point (3D 路标) 数据结构 |
| `svo_common/include/svo/common/seed.h` | - | 深度滤波器种子数据结构 |