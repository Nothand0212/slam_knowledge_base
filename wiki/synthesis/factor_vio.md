---
created: 2026-06-01
updated: 2026-06-01
type: synthesis
tags: [stereo-vio, factor-graph, GTSAM, iSAM2, SmartFactor, 方案设计, factor-vio]
sources:
  - wiki/synthesis/stereo-vio-integrated-architecture.md
  - wiki/synthesis/landmark-pipeline-design.md
  - wiki/entities/设计-立体VIO前端管线.md
  - wiki/entities/架构-GTSAM iSAM2 双目VIO后端设计.md
  - wiki/entities/设计-双目VIO初始化子系统.md
  - wiki/entities/设计-双目VIO回环子系统.md
  - raw/codes/Kimera-VIO/
  - raw/codes/VINS-Fusion/
  - raw/codes/ORB_SLAM3/
  - raw/codes/open_vins/
  - raw/codes/dm-vio/
---

# Factor-VIO：基于因子图的立体视觉惯性里程计

> 综合 Kimera-VIO、VINS-Fusion、ORB-SLAM3、OpenVINS、DM-VIO、SVO Pro 六系统源码分析，
> 产出完整可工程落地的立体 VIO 方案设计。

---

## 一、系统总览

### 1.1 架构图

```
┌──────────────────────────────────────────────────────────────────┐
│                         Factor-VIO                               │
├────────────────┬──────────────────────┬──────────────────────────┤
│  Tracking      │  Local Mapping       │  Loop Closing            │
│  Thread        │  Thread              │  Thread                  │
│  (每帧,~15ms)  │  (每关键帧,~40ms)     │  (后台, 1-5 Hz)          │
├────────────────┼──────────────────────┼──────────────────────────┤
│                │                      │                          │
│ 前端            │ 路标管线              │ 回环检测                  │
│ ┌───────────┐  │ ┌────────────────┐   │ ┌──────────────────┐    │
│ │KLT跟踪     │  │ │深度滤波器       │   │ │DBoW3 ORB查询     │    │
│ │IMU预测     │  │ │(Gaussian+      │   │ │时间/空间过滤     │    │
│ │NCC立体匹配 │  │ │ Uniform混合)   │   │ │分组一致性确认    │    │
│ │双轮RANSAC  │  │ ├────────────────┤   │ └────────┬─────────┘    │
│ │关键帧决策  │  │ │SmartFactor试用  │   │          │              │
│ │特征老化    │  │ │(隐式路标)      │   │ 几何验证: 3D-2D PnP    │
│ └───────────┘  │ │┌──────────────┐│   │          │              │
│                │ ││clone-and-add ││   │          ▼              │
│ FrontendOutput │ ││slot管理      ││   │ 回环注入(BetweenFactor) │
│       │        │ │└──────────────┘│   │          │              │
│       ▼        │ ├────────────────┤   │          ▼              │
│ FeatureDatabase│ │ │晋升门控        │   │ Post-loop              │
│ (共享读写)     │ │ │(10条件检查)   │   │ SmartFactor提升         │
│       │        │ ├────────────────┤   └──────────────────────────┘
│       ▼        │ │ │显式路标        │
│ PendingLandmark│ │ │GenericStereo   │
│       │        │ │ │Factor+Point3   │
│       ▼        │ ├────────────────┤
│                │ │ │Post-update     │
│                │ │ │chi²异常值剔除  │
│                │ ├────────────────┤
│                │ │ │边缘化管理       │
│                │ │ │延迟边缘化      │
│                │ └────────────────┘
│                │          │
│                │  LandmarkInjection
│                │          │
│                │          ▼
│                │ ┌────────────────┐
│                │ │因子图后端       │
│                │ │iSAM2.update()  │
│                │ │updateSmoother  │
│                │ │异常恢复        │
│                │ └────────────────┘
└────────────────┴──────────────────────┴──────────────────────────┘

共享数据结构:
  FeatureDatabase      (前端写 → 路标管线读)
  KeyframeDatabase     (路标管线写 → 回环读)
  ExplicitLandmarkMap  (路标管线写 → 回环读)
```

### 1.2 全局状态机

```
UNINITIALIZED → STATIC_CHECK → STATIC_INIT ──成功──→ INITIALIZED
                    │ 失败                            │
                    ▼                                 │
              DYNAMIC_CHECK → DYNAMIC_INIT ──成功─────┘
                    │ 失败
                    ▼
               RETRY (最多3次) → 降级为弱先验初始化

INITIALIZED:
  ├── NOMINAL (正常运营: SmartFactor试用期启用)
  ├── LOOP_CORRECT (回环注入 + SmartFactor提升)
  └── GLOBAL_BA (累计3次回环或漂移>0.5m触发)
```

### 1.3 路标状态机

```
                    ┌─────────┐
      新特征观测 →  │CANDIDATE│  (仅KLT跟踪,无3D)
                    └────┬────┘
                         │ 立体匹配成功 + 深度有效
                         ▼
                    ┌────────────┐
                    │DEPTH_FILTER│  (贝叶斯深度滤波)
                    └─────┬──────┘
                          │ 深度方差收敛
                          ▼
                    ┌────────────┐
                    │SMART_TRIAL │  (SmartStereoFactor, 隐式路标)
                    └─────┬──────┘
                          │ 晋升门控通过 (≥4观测, 视差>3°, 重投影<2px, SVD条件数<1e6)
                          ▼
                    ┌────────────┐
                    │ PROMOTING  │  (创建Point3 + GenericStereoFactor)
                    └─────┬──────┘
                          │ chi²后验通过
                          ▼
                    ┌────────────┐
                    │  EXPLICIT  │  (显式路标, 参与iSAM2迭代优化)
                    └─────┬──────┘
                          │ 连续N帧chi²通过 + 观测数充足
                          ▼
                    ┌────────────┐      ┌─────────────┐
                    │   STABLE   │ ───→ │MARGINALIZED │ (被边缘化出窗口)
                    └─────┬──────┘      └─────────────┘
                          │ 连续异常
                          ▼
                    ┌────────────┐
                    │REMEDIATING │ ──恢复──→ STABLE
                    └─────┬──────┘
                          │ 恢复失败
                          ▼
                    ┌────────────┐
                    │  CULLED    │ (永久删除)
                    └────────────┘
```

---

## 二、前端设计

### 2.1 逐帧处理流程

```
processFrame(stereo_image, imu_buffer, prev_state):

  PHASE 0: 时间戳与IMU窗口
    t_cur = stereo_image.timestamp
    imu_window = selectImuBetween(imu_buffer, prev_state.timestamp, t_cur)
    delta_t_segments = extractTimeSegments(imu_window)  // 真实时间戳差,非常数dt

  PHASE 1: IMU预积分
    pim = new PreintegratedCombinedMeasurements(params, prev_state.bias)
    for each (dt, acc, gyr) in imu_window:
        pim.integrateMeasurement(acc, gyr, dt)

  PHASE 2: IMU预测旋转 → KLT初值
    predicted_rotation = pim.deltaRij()
    H = K * predicted_rotation^T * K^(-1)        // Kimera OpticalFlowPredictor
    for each track in tracks_active:
        track.predicted_pt = project(H, track.pixel_pt_prev)

  PHASE 3: KLT跟踪 (前帧→当前左目)
    cur_pts, status, err = calcOpticalFlowPyrLK(
        prev_left_gray, cur_left_gray,
        prev_pts, predicted_pts,
        winSize=(21,21), maxLevel=3,
        criteria=(COUNT|EPS, 30, 0.01),
        OPTFLOW_USE_INITIAL_FLOW)
    // 降级: 成功<10时回退到无预测3层
    if countSuccess(status) < 10:
        cur_pts = calcOpticalFlowPyrLK(..., maxLevel=3)  // 无USE_INITIAL_FLOW

  PHASE 3b: 特征老化淘汰
    for each track where track.age > max_feature_track_age (25):
        status[track_idx] = false
        prev_landmarks[track_idx] = -1

  PHASE 4: 双向光流验证
    if config.forward_backward_check:
        back_pts, back_status = calcOpticalFlowPyrLK(
            cur_left_gray, prev_left_gray, cur_pts, initial=prev_pts)
        for i: if norm(back_pts[i]-prev_pts[i]) > 0.5: status[i]=false

  PHASE 5: 2D-2D 单目RANSAC
    status = status AND inBorder(cur_pts)
    cur_un_pts = undistort(cur_pts)
    status = status AND monocularRANSAC(
        prev_un_pts, cur_un_pts,
        threshold=1.0e-6,        // bearing vector dot-product space
        minInliers=10,           // Kimera minNrMonoInliers
        confidence=0.995,
        givenRotation=imu_rotation_available ? predicted_R : null)
    // 2D-2D外点: 永久标记landmark=-1

  PHASE 5b: 3D-3D 立体RANSAC (仅对2D-2D幸存者+有立体深度的特征)
    if config.stereo and |stereo_pts| >= 5:
        inliers, _ = onePointStereoRANSAC(
            stereo_pts_ref, stereo_pts_cur, imu_rotation,
            mahalanobisSqThreshold=1.0,  // 3-DOF, ~80% confidence
            minInliers=5)
        for t where t.has_stereo and not inlier:
            t.stereo_3d3d_outlier = true   // 降级标记,不删除!
            // → getSmartStereoMeasurements中uR=NaN → SmartFactor退化为单目

  PHASE 6: 压缩存活点
    tracks_alive = compactByStatus(tracks_active, status)

  PHASE 7: 关键帧决策
    is_keyframe = shouldBeKeyframe(tracks_alive, prev_keyframe, pim)
    // 条件: disparity>0.5px+Δt>0.2s | Δt>5s | Δtranslation>0.5m | Δrotation>15° | features<20 | dropout>50%

  PHASE 7b: 补新特征 (仅关键帧)
    if is_keyframe:
        n_raw = min((max_features-n_alive)*6, 2000)
        new_pts = goodFeaturesToTrack(left_gray, n_raw, qualityLevel=0.001)
        new_pts = anmsBinning(new_pts, scores, n_need, grid=(7,5))
        cornerSubPix(left_gray, new_pts, (5,5), (-1,-1), (COUNT|EPS,20,0.001))
        for p in new_pts: tracks_alive.append(new FeatureTrack(p, next_id++))

  PHASE 8: 立体匹配
    if config.stereo:
        for t in tracks_alive:
            right_pt, ncc_score = nccStereoMatch(
                left_gray, right_gray, t.pixel_pt,
                template=(101,11), threshold=0.15)
            if ncc_score < 0.15 and disparity > 1.0:
                depth = fx * baseline / disparity
                if 0.3 < depth < 15.0:
                    t.has_stereo = true; t.stereo_depth = depth

  return FrontendFrame(timestamp, tracks_alive, pim, raw_imu, is_keyframe)
```

**RANSAC 执行时机**：Kimera-VIO 中 RANSAC 仅发生在关键帧内部（`processStereoFrame` L342+ 块）。Factor-VIO 设计选择在每帧执行 2D-2D RANSAC，关键帧额外执行 3D-3D RANSAC。这是有意偏离 Kimera 的架构选择——目的是在非关键帧上也剔除明显的 2D 外点。

### 2.2 NCC立体匹配详细规格

```
function nccStereoMatch(left_img, right_img, left_pt, baseline, K):
    // 已立体校正 → 极线水平对齐, 搜索降为1D
    
    // 1. 提取左图模板 (窄长条,沿极线方向)
    template = left_img(ROI: left_pt ± (50px, 5px))   // 101×11 模板
    
    // 2. 定义右图搜索stripe (基于深度范围)
    max_disp = fx * baseline / min_depth    // 0.3m处视差
    min_disp = fx * baseline / max_depth    // 15m处视差
    stripe = right_img(ROI: [left_pt.x-max_disp, left_pt.x-min_disp], y±5)
    
    // 3. NCC模板匹配
    cv::matchTemplate(stripe, template, result, CV_TM_SQDIFF)
    normalize(result)  // → [0,1], 0=完美匹配
    min_val, min_loc = minMaxLoc(result)
    
    // 4. 亚像素精化 (可选)
    if config.subpixel_stereo:
        cornerSubPix(right_img, right_pt, (10,10), criteria=(COUNT|EPS,40,0.001))
    
    return right_pt, min_val   // min_val=0.0完美, 1.0完全不相关
```

### 2.3 前端参数表

> ⚠️ 标注: 🔵 = 与 Kimera-Euroc 实际值不同（Factor-VIO 设计选择）, 🟢 = 与 Kimera 一致

| 分类 | 参数 | 值 | 来源 | 备注 |
|------|------|-----|------|------|
| **KLT** | 窗口大小 | 🔵 21×21 px | VINS-Fusion | Kimera-Euroc 实际用 24×24 |
| | 金字塔层数 | 🔵 3 (有IMU预测时1) | VINS-Fusion | Kimera-Euroc 实际用 4 层 |
| | 最大迭代 | 🟢 30 | 三者一致 | |
| | 收敛ε | 🔵 0.01 | VINS-Fusion/OpenVINS | Kimera-Euroc 实际用 **0.1** (10× 差) |
| | 双向光流阈值 | 0.5 px | VINS-Fusion (Kimera 默认关) | |
| **特征检测** | 检测器 | 🟢 Shi-Tomasi (GFTT) | Kimera | |
| | qualityLevel | 🟢 0.001 | Kimera | VINS-Fusion 用 0.01 |
| | 目标特征数 | 🟢 300/帧 | Kimera Euroc | |
| | 原始候选数 | 🟢 2000 (ANMS前) | Kimera | |
| | 空间分布 | 🟢 Binning 7×5 | Kimera Euroc | |
| | 子像素精化 | 🔵 (5,5)窗口, 20次 | OpenVINS | Kimera 用 (10,10)窗口/40次 |
| **特征管理** | 最大年龄 | 🟢 25关键帧 | Kimera | |
| | 仅在KF检测 | 🟢 true | Kimera | |
| **立体匹配** | 模板尺寸 | 🟢 101×11 px | Kimera | |
| | 匹配方法 | 🟢 CV_TM_SQDIFF (非NCC!) | Kimera | 文档称"NCC"但实际是平方差 |
| | 匹配阈值 | 🟢 0.15 (归一化[0,1]) | Kimera | |
| | 深度范围 | 🔵 [0.3, 15.0] m | 设计选择 | Kimera Euroc 用 [0.5, 10.0]m |
| | 子像素精化 | 可选(Kimera默认关) | Kimera | |
| **2D-2D RANSAC** | 算法 | 🟢 2-point(IMU)/5-point(Nistér) | Kimera | |
| | 阈值 | 🟢 1.0e-6 (bearing cos空间) | Kimera | |
| | 最少内点 | 🟢 10 | Kimera | |
| | 置信度 | 🟢 0.995 | Kimera | |
| **3D-3D RANSAC** | 算法 | 🟢 1-point voting(IMU)/3-point(Arun) | Kimera | |
| | 阈值 | 🟢 1.0 (马氏距离²) | Kimera | |
| | 最少内点 | 🟢 5 | Kimera | |
| | 外点处理 | 🟢 **降级(FAILED_ARUN),不删除** | Kimera | |
| **关键帧** | 最小时间 | 🟢 0.2 s | Kimera | |
| | 最大时间 | 🟢 5.0 s | Kimera | |
| | 最小视差 | 🟢 0.5 px | Kimera | |
| | 🔵 最大平移 | **0.5 m** | **Factor-VIO 新增** | 不存在于 Kimera/VINS/ORB-SLAM3 源码 |
| | 🔵 最大旋转 | **15°** | **Factor-VIO 新增** | 不存在于 Kimera/VINS/ORB-SLAM3 源码 |
| | 🔵 特征丢失率 | **50%** | **Factor-VIO 新增** | 不存在于 Kimera/VINS/ORB-SLAM3 源码 |

### 2.4 前端输出结构 (FrontendOutput → LandmarkPipeline)

```cpp
struct FrontendOutput {
    double timestamp;
    std::vector<FeatureTrack> tracks;
    int n_stereo_tracked;
    int n_total_tracked;
    std::shared_ptr<PreintegratedCombinedMeasurements> pim;
    std::vector<std::tuple<double, Vector3, Vector3>> imu_window_raw;
    bool is_keyframe;
    uint64_t keyframe_id;
    std::vector<PendingLandmark> pending_landmarks;  // → LandmarkPipeline
    FrontendDiagnostics diag;
};

struct FeatureTrack {
    uint64_t id;                    // 全局唯一track ID
    Vector2 pixel_pt;              // 当前帧左目像素
    Vector3 normalized_pt;         // 去畸变归一化坐标 (z=1)
    Vector3 prev_normalized_pt;    // 上一帧归一化坐标
    int track_length;              // 连续跟踪帧数
    int track_age_kfs;             // 连续跟踪关键帧数
    bool has_stereo;               // 当前帧是否有有效立体
    bool stereo_3d3d_outlier;      // 3D-3D RANSAC外点标记 (降级不删)
    double stereo_depth;           // 立体深度 (m)
    double stereo_ncc_score;       // NCC匹配质量 (低=好)
    Vector3 prev_kf_normalized_pt; // 上一关键帧的归一化坐标
    Vector3 prev_kf_3d;           // 上一关键帧的3D点 (相机系)
};
```

---

## 三、路标管线设计

### 3.1 深度滤波器 (DEPTH_FILTER状态)

参考 SVO Pro 的贝叶斯深度滤波器 (Gaussian+Uniform混合模型)。

```
function depthFilterUpdate(seed, frame_obs):
    // 模型: p(d|obs) = ρ * N(d|μ,σ²) + (1-ρ) * U(d|d_min,d_max)
    // 其中 ρ = inlier ratio估计
    
    // 1. 从立体视差计算当前帧深度观测
    z = fx * baseline / disparity_frame
    
    // 2. 计算观测不确定性 (深度对1px视差误差的敏感度)
    tau² = (z² / (fx * baseline))²  // σ_d ≈ z²/(f·b) · σ_disparity
    
    // 3. 贝叶斯更新 (假设N(z, tau²)观测模型)
    s² = 1/(1/σ² + 1/τ²)
    μ_new = s² * (μ/σ² + z/τ²)
    
    // 4. 更新内点率估计 (Vogiatzis模型)
    // 计算观测似然: L = N(z|μ,σ²+τ²) / U(z)
    // 更新beta分布参数: a += inlier_evidence, b += outlier_evidence
    
    // 5. 收敛检查
    if σ² < (μ_range / 200)²:  // SVO Pro: threshold=200
        return CONVERGED, μ_new, σ²
    return TRACKING, μ_new, σ²
```

**深度滤波器参数**：

| 参数 | 值 | 来源 |
|------|-----|------|
| 收敛阈值 | σ² < (μ_range/200)² | SVO Pro |
| Beta先验 a | 10 | SVO Pro |
| Beta先验 b | 10 | SVO Pro |
| 最小深度 | 0.3 m | — |
| 最大深度 | 15.0 m | — |
| 视差噪声 σ_disparity | 1.0 px | — |

### 3.2 SmartFactor试用期 (SMART_TRIAL状态)

路标在试用期内以 `SmartStereoProjectionPoseFactor` 形式存在于因子图中。

**因子参数**：

```cpp
auto params = gtsam::SmartStereoProjectionParams(
    gtsam::HESSIAN,                    // 线性化模式
    gtsam::ZERO_ON_DEGENERACY,         // 退化时Jacobian归零
    false,                             // throwCheirality
    false);                            // verboseCheirality
params.setRankTolerance(1.0);
params.setLandmarkDistanceThreshold(20.0);     // 20m外标记退化
params.setRetriangulationThreshold(1e-3);
params.setDynamicOutlierRejectionThreshold(3.0);  // 3σ重投影拒绝
params.setEnableEPI(false);

auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 3.0);  // 3px立体噪声
```

**SmartFactor管理 (Clone-and-add + Slot追踪)**：

```cpp
struct SmartFactorSlot {
    gtsam::FactorIndex slot;              // -1=未入图, >=0=图中的槽位
    SmartStereoFactor::shared_ptr factor;
};

// 每关键帧处理:
delete_slots = [];
for each active_smart_landmark:
    old = old_smart_factors[lmk_id];
    new_sf = clone(old.factor);           // 拷贝已有观测
    new_sf->add(stereo_meas, pose_key, K); // 追加新观测
    
    if old.slot != -1:
        delete_slots.push(old.slot);      // 删除旧槽位
    new_factors.push(new_sf);             // SMART FACTORS MUST BE FIRST

new_factors.push(imu_and_prior_factors...); // IMU因子排在SmartFactor后面

result = smoother->update(new_factors, new_values, timestamps, delete_slots);

// 从result恢复slot映射 (1:1对应关系)
for i, lmk_id in enumerate(smart_factor_order):
    old_smart_factors[lmk_id].slot = result.newFactorsIndices[i];
```

### 3.3 晋升门控 (SMART_TRIAL → PROMOTING)

```
function shouldPromote(lmk_id, smart_factor, current_estimate):
    // G1: 最少观测数
    if smart_factor.numObservations() < min_obs_for_promotion (4):
        return false
    
    // G2: 三角化有效性
    result = smart_factor.triangulate(current_estimate)
    if not result.valid(): return false
    
    // G3: Cheirality检查 (所有观测相机前方为正)
    if result.behindCamera(): return false
    
    // G4: 退化检查
    if result.degenerate(): return false
    
    // G5: 远点检查
    if result.farPoint(): return false
    
    // G6: 异常值检查
    if result.outlier(): return false
    
    // G7: 视差角充足 (>3°, 防止共线退化)
    max_parallax = max_pairwise_parallax(smart_factor.connectedPoses())
    if max_parallax < 3.0°: return false
    
    // G8: 重投影误差可控 (<2.0px mean)
    mean_reproj = mean_reprojection_error(smart_factor, current_estimate)
    if mean_reproj > 2.0: return false
    
    // G9: SVD条件数 (数值稳定性)
    if triangulation_condition_number > 1e6: return false
    
    // G10: 最小深度 (>0.1m)
    point3d = result.get()
    if point3d.norm() < 0.1: return false
    
    return true
```

| 门控条件 | 阈值 | 依据 |
|----------|------|------|
| 最少观测数 | 4帧 | Kimera `min_num_obs_for_proj_factor` |
| 视差角 | >3° | 物理: 低于3°时深度反演条件数劣化 |
| 重投影误差 | <2.0 px mean | 经验: 2px对应~0.5° bearing误差 @460focal |
| SVD条件数 | <1e6 | 数值稳定性 |
| 最小深度 | >0.1m | 防止近场野值 |

### 3.4 晋升操作 (PROMOTING → EXPLICIT)

```cpp
function promoteLandmark(lmk_id, smart_factor, current_estimate):
    // 1. 从当前估计三角化路标3D位置
    point3d = smart_factor.triangulate(current_estimate).get()
    
    // 2. 分配GTSAM变量key
    lmk_key = L(next_lmk_id++)
    
    // 3. 创建显式因子 (替换旧SmartFactor)
    for each obs in smart_factor.observations():
        K_pose = X(obs.kf_id)
        stereo_meas = StereoPoint2(obs.uL, obs.uR, obs.v)
        factor = GenericStereoFactor<Pose3,Point3>(
            stereo_meas,
            noiseModel::Isotropic::Sigma(3, 1.5),  // 比SmartFactor紧的噪声
            K_pose, lmk_key, stereo_cal)
        new_factors.push(factor)
    
    // 4. 从old_smart_factors中移除 (slot标记删除)
    old_slot = old_smart_factors[lmk_id].slot
    delete_slots.push(old_slot)
    old_smart_factors.erase(lmk_id)
    
    // 5. 注册显式路标
    explicit_landmarks[lmk_id] = {lmk_key, point3d, EXPLICIT, 0}
    
    // 6. 插入初始值
    new_values.insert(lmk_key, point3d)
```

### 3.5 Post-update异常值剔除 (EXPLICIT → STABLE/REMEDIATING/CULLED)

```cpp
function postUpdateOutlierCheck(isam2_result, explicit_landmarks):
    for each lmk in explicit_landmarks:
        // 计算chi²统计量
        residuals = computeStereoResiduals(lmk, isam2_result)
        chi2 = residuals^T * Σ^{-1} * residuals  // Σ = Isotropic(3,1.5)
        
        if chi2 > chi2_threshold (7.815):  // χ²₃, p=0.05
            lmk.consecutive_outlier_count++
            if lmk.consecutive_outlier_count >= 3:  // ORB-SLAM3 nThObs
                cullLandmark(lmk)                    // → CULLED
        else:
            lmk.consecutive_outlier_count = 0
            if lmk.total_observations >= min_stable_obs (10):
                lmk.state = STABLE                   // → STABLE
```

### 3.6 路标参数表

| 参数 | 值 | 来源 |
|------|-----|------|
| min_obs_for_promotion | 4 | Kimera `min_num_obs_for_proj_factor` |
| min_parallax_promotion | 3.0° | 推导 |
| max_reproj_promotion | 2.0 px | 经验 |
| svd_condition_max | 1e6 | 数值稳定性 |
| chi2_outlier_threshold | 7.815 | χ²₃(p=0.05), ORB-SLAM3 |
| consecutive_outlier_cull | 3帧 | ORB-SLAM3 `nThObs` |
| depth_filter_convergence | σ² < (μ_range/200)² | SVO Pro |
| min_stable_obs | 10 | 经验 |
| smart_factor_pixel_sigma | 3.0 px | Kimera `smartNoiseSigma_` |
| explicit_pixel_sigma | 1.5 px | 晋升后收紧 |
| outlier_rejection_sigma | 3.0 | Kimera `outlierRejection_` |
| landmark_distance_threshold | 20.0 m (Euroc YAML: 10.0) | Kimera header默认(20.0), Euroc覆盖为10.0 |
| retriangulation_threshold | 1e-3 | Kimera |
| max_feature_track_age | 25 KFs | Kimera |

---

## 四、初始化设计

### 4.1 静态初始化

```
function staticInitialize(imu_data, wait_duration=1.0s):
    // 收集两窗口IMU数据
    window1 = imu_data[-0.5s:]
    window2 = imu_data[-1.0s:-0.5s]
    
    // 静止检测
    a_var = std(window1.accel)
    if a_var > imu_static_threshold (1.0 m/s²):
        return NEED_MOTION  // 设备在运动,切换动态初始化
    
    // 重力方向估计
    a_mean = mean(window2.accel)
    z_axis = a_mean / |a_mean|
    R_GtoI = gram_schmidt(z_axis)     // z轴对齐重力方向
    q_GtoI = rot2quat(R_GtoI)
    
    // 偏置估计
    bg = mean(window2.gyro)           // 陀螺偏置=静止时角速度均值
    ba = a_mean - R(q_GtoI)·g          // 加计偏置=实测均值-理论重力
    
    return StaticResult{Pose3(q_GtoI, 0), 0, ConstantBias(ba, bg)}
```

### 4.2 动态初始化 (双目简化版)

```
function dynamicInitialize(kf_queue, imu_pims):
    // 阶段1: 陀螺偏置估计 (线性系统)
    // 构造 A * bg = b, A和b由视觉旋转和IMU旋转的差异组成
    A = []; b = []
    for each consecutive KF pair (i, i+1):
        R_visual = kf_queue[i+1].R * kf_queue[i].R^T
        R_imu = pim.deltaRij().matrix()
        dR = Log_SO3(R_visual^T * R_imu)    // 残差
        J = computeGyroJacobian(pim)          // IMU对bg的Jacobian
        A.push(J); b.push(dR)
    bg = solve(A^T*A * x = A^T*b)            // LDLT
    
    // 阶段2: 速度+重力方向优化 (GTSAM因子图, 双目无尺度)
    graph = NonlinearFactorGraph()
    values = Values()
    for each KF i:
        values.insert(X(i), kf_queue[i].pose)   // 来自前端PnP,固定
        values.insert(V(i), estimateVel(kf_queue))
    values.insert(B_shared, ConstantBias(Vector3::Zero(), bg))
    values.insert(GravityDir, estimateRwg())       // SO(3), 2-DOF
    
    for each consecutive KF pair:
        graph.add(CombinedImuFactor(X(i),V(i),B_shared,X(i+1),V(i+1),B_shared,pim))
    
    optimize(graph, values, GaussNewton, maxIter=50)
    
    // 验证
    g_world = Rwg_opt * (0,0,-9.81)
    if ||g_world|-9.81| > 0.3: return FAIL
    if |bg_opt| > 0.1 rad/s: return FAIL
    
    return DynamicResult{Rwg_opt, bg_opt, velocities}
```

### 4.3 初始先验 (anchorInitialState)

```cpp
void anchorInitialState(LocalStateEstimate init):
    Values v;
    v.insert(X(0), init.T_w_b);
    v.insert(V(0), init.v_w);
    v.insert(B(0), init.bias);
    
    NonlinearFactorGraph g;
    
    // 姿态先验: roll/pitch由重力可观=紧(但Kimera用0.1745rad初始), yaw不可观=适中, 位置=极紧
    auto pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1745, 0.1745, 1.75e-3, 1e-5, 1e-5, 1e-5).finished());
    //                    roll    pitch    yaw      x     y     z
    g.addPrior(X(0), init.T_w_b, pose_noise);
    
    auto vel_noise = noiseModel::Isotropic::Sigma(3, 1e-3);
    g.addPrior(V(0), init.v_w, vel_noise);
    
    auto bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
    //                  acc_bias(x3)     gyr_bias(x3)
    g.addPrior(B(0), init.bias, bias_noise);
    
    smoother->update(g, v, timestamps);
```

**先验参数对比**:

| 参数 | Kimera-VIO | 本方案 | 依据 |
|------|-----------|--------|------|
| 位置σ | 1e-5 m | 1e-5 m | 首帧固定,防止整体漂移 |
| roll/pitchσ | 🔵 0.1745 rad (=10°) | 🔵 0.1745 rad | Kimera源码默认值(`VioBackendParams.h:L111`: `10.0/180.0*M_PI`), 非1e-5 |
| yawσ | 1.75e-3 rad | 1.75e-3 rad | yaw不可观,适度松 |
| 速度σ | 1e-3 m/s | 1e-3 m/s | 静止初始化可信 |
| acc biasσ | 0.1 m/s² | 0.1 m/s² | 中等先验 |
| gyr biasσ | 0.01 rad/s | 0.01 rad/s | 中等先验 |

### 4.4 SmartFactor启用策略

Kimera-VIO 从 KF=1 立即启用 SmartFactor，无需"前N帧禁用"阶段。前提是首帧先验极紧（位置σ=1e-5m），给SmartFactor提供稳定参考系进行三角化。

**本方案推荐相同策略**：初始化质量保证（紧先验）→ 信任SmartFactor从第一帧工作。如果初始化质量不确定，可保留前10 KF禁用SmartFactor的降级选项。

---

## 五、因子图后端设计

### 5.1 状态变量

| 符号 | GTSAM类型 | 维度 | 每关键帧 | 含义 |
|------|----------|------|---------|------|
| `X(k)` | `Pose3` | 6 | ✓ | IMU体在世界系的位姿 |
| `V(k)` | `Vector3` | 3 | ✓ | IMU体在世界系的速度 |
| `B(k)` | `imuBias::ConstantBias` | 6 | ✓ | IMU偏置(acc3+gyr3) |
| `L(id)` | `Point3` | 3 | 仅显式路标 | 世界系3D点坐标 |

### 5.2 因子类型

> ⚠️ 标注: 🏷️ = Kimera-VIO `VioBackend.cpp` 源码中有, ✨ = Factor-VIO 新增, ❌ = Kimera-VIO 有但本方案不需要

#### 5.2.1 视觉因子

| # | 来源 | GTSAM 类 | Kimera 创建位置 | 连接 | 噪声 | 何时使用 | Factor-VIO? |
|---|------|----------|---------------|------|------|---------|------------|
| V1 | 🏷️ | `SmartStereoProjectionPoseFactor` | `L489`: `new SmartStereoFactor(noise, params, T_b_cam)` | `X(t₁)...X(tₙ)` (多帧位姿) | `Isotropic(3, 3.0)` | **每 KF**,所有路标 | ✅ SmartFactor 试用期 |
| V2 | ✨ | `GenericStereoFactor<Pose3,Point3>` | 不存在于默认 VioBackend; RegularVioBackend `L836` | `X(k), L(id)` | `Isotropic(3, 1.5)` | SmartFactor 晋升后 | ✅ 显式路标 |

#### 5.2.2 IMU 因子

| # | 来源 | GTSAM 类 | Kimera 创建位置 | 连接 | 噪声 | 何时使用 | Factor-VIO? |
|---|------|----------|---------------|------|------|---------|------------|
| I1 | 🏷️ | `ImuFactor` | `L926`: `emplace_shared<ImuFactor>` | `X(i-1),V(i-1),X(i),V(i),B(i-1)` | Cholesky(PIM_cov⁻¹) | Euroc 默认 (`imu_type=1`) | ✅ 推荐 |
| I2 | 🏷️ | `CombinedImuFactor` | `L915`: `emplace_shared<CombinedImuFactor>` | `X(i-1),V(i-1),B(i-1),X(i),V(i),B(i)` | Cholesky(PIM_cov⁻¹) | `imu_type=0` 时 | 备选 |
| I3 | 🏷️ | `BetweenFactor<ConstantBias>` | `L953`: bias 随机游走 | `B(i-1),B(i)` (零增量) | `sqrt(dt)*diag(acc_w², gyr_w²)` | **仅 ImuFactor 模式** | ✅ 配合 I1 |

#### 5.2.3 先验因子

| # | 来源 | GTSAM 类 | Kimera 创建位置 | 连接 | 噪声 | 何时使用 | Factor-VIO? |
|---|------|----------|---------------|------|------|---------|------------|
| P1 | 🏷️ | `PriorFactor<Pose3>` | `L1285`: 首帧; `L1448`: 异常恢复 | `X(0)` / `X(nearby)` | `Gaussian::Covariance` (经B_Rot_W旋转,非对角); roll/pitch=0.1745rad, yaw=0.00175rad, pos=1e-5m | 初始化时 + IndeterminantLinearSystem 恢复时 | ✅ |
| P2 | 🏷️ | `PriorFactor<Vector3>` | `L1296`: 首帧; `L1012`: 零速; `L1470`: 恢复 | `V(0)` / `V(k)` | `Isotropic(3,1e-3)` (首帧) / `Isotropic(3,0.032)` (零速) | 初始化时 + LOW_DISPARITY 时 + 恢复时 | ✅ |
| P3 | 🏷️ | `PriorFactor<ConstantBias>` | `L1313`: 首帧; `L1460`: 恢复 | `B(0)` / `B(nearby)` | `Diag(0.1³,0.01³)` | 初始化时 + 恢复时 | ✅ |
| P4 | 🏷️ | `LinearContainerFactor` | iSAM2 边缘化自动生成 | 被边缘化的变量群 | Schur 补自动计算 | 每次边缘化老变量后 | ✅ (自动) |
| P5 | ❌ | `PriorFactor<Point3>` | VioBackend.cpp L2032(打印检测) | `L(id)` | — | VioBackend图中不应存在(L2226 CHECK断言) | ❌ |

#### 5.2.4 相对约束因子

| # | 来源 | GTSAM 类 | Kimera 创建位置 | 连接 | 噪声 | 何时使用 | Factor-VIO? |
|---|------|----------|---------------|------|------|---------|------------|
| R1 | 🏷️ | `BetweenFactor<Pose3>` | `L984`: 立体 RANSAC | `X(i-1),X(i)` | `Diag(0,0,0, 100,100,100)` (精密) | `addBetweenStereoFactors_=true` 且立体跟踪 VALID 时 | 可选 (默认关) |
| R2 | 🏷️ | `BetweenFactor<Pose3>` | `L997`: 静止约束 | `X(i-1),X(i)` (Identity) | `Diag(10000,10000,10000, 1000,1000,1000)` (精密) | LOW_DISPARITY 时 (`kfTrackingStatus_mono_==LOW_DISPARITY`) | ✅ |
| R3 | 🏷️ | `BetweenFactor<Vector3>` | `L1326`: 匀速先验 | `V(i-1),V(i)` (零增量) | `Diag(100,100,100)` (精密) | **已被注释掉,从未调用** | ❌ |
| R4 | 🏷️ | `BetweenFactor<Pose3>` | `L407`: 外部里程计 | `X(i-1),X(i)` | 来自 `OdometryParams` (精密) | 有外部里程计且精度>0 时 | 可选 |
| R5 | ✨ | `BetweenFactor<Pose3>` | 回环线程注入 | `X(i), X(j)` | 从 PnP 内点分布估计 | 回环检测成功后 | ✅ |

#### 5.2.5 速度因子

| # | 来源 | GTSAM 类 | Kimera 创建位置 | 连接 | 噪声 | 何时使用 | Factor-VIO? |
|---|------|----------|---------------|------|------|---------|------------|
| S1 | 🏷️ | `PriorFactor<Vector3>` | `L1012` (零速) | `V(k)` (零) | `Diag(1000,1000,1000)` (精密≈0.032m/s) | LOW_DISPARITY 且 `zero_velocity_precision_>0` | ✅ |
| S2 | 🏷️ | `PriorFactor<Vector3>` | `L1027` (外部里程计速度) | `V(k)` | 来自 `OdometryParams.velocityPrecision_` | 有外部里程计速度时 | 可选 |

#### 5.2.6 RegularVioBackend 专属 (Factor-VIO 暂不需要)

| # | GTSAM 类 | 用途 |
|---|---------|------|
| — | `PointPlaneFactor` | 路标到平面的距离约束 (结构规律) |
| — | `PriorFactor<OrientedPlane3>` | 平面先验 (如地面法向) |

### 5.3 每关键帧完整后端处理流程 (optimize)

参考 Kimera-VIO `VioBackend::optimize()` (`VioBackend.cpp:L1036-L1220`) 的精确实现。

```
function optimize(timestamp_kf_nsec, cur_id, max_extra_iterations, extra_delete_slots):
    // ===== 阶段 0: 构建新因子图 =====
    delete_slots = extra_delete_slots          // 子类可注入额外删除槽位
    
    // 遍历所有更新后的 SmartFactor (new_smart_factors_)
    for each (lmk_id, new_sf) in new_smart_factors_:
        old_entry = old_smart_factors_[lmk_id]       // 查找旧记录
        old_slot = old_entry.second                   // -1 = 首次入图
        
        if old_slot != -1:                            // 已在图中
            DCHECK(smoother->getFactors().exists(old_slot))
            delete_slots.push_back(old_slot)           // 标记旧槽位删除
            new_factors_tmp.push_back(new_sf)          // 加入新SmartFactor
            lmk_ids_tmp.push_back(lmk_id)
        elif not smoother->getFactors().exists(old_slot):
            // 超出时间窗口 → 清理
            old_smart_factors_.erase(lmk_id)
            deleteLmkFromFeatureTracks(lmk_id)
        else:                                         // 首次入图
            new_factors_tmp.push_back(new_sf)
            lmk_ids_tmp.push_back(lmk_id)
    
    // ⚠️ SmartFactor 必须排第一, IMU/Prior 因子排后面
    // 理由: iSAM2 返回的 newFactorsIndices 与输入顺序 1:1 对应
    new_factors_tmp.push_back(new_imu_prior_and_other_factors_)
    
    // ===== 阶段 1: 时间戳映射 =====
    key_frame_count = {}                              // Key → Double
    for each (key, value) in new_values_:
        key_frame_count[key] = cur_id                 // 用当前KF ID作为时间戳
    // IncrementalFixedLagSmoother 基于时间戳决定边缘化:
    //   时间戳 < max_timestamp - smootherLag → 被边缘化
    //   即: cur_id - key_timestamp > lag_states → 老变量被边缘化
    
    // ===== 阶段 1.5: Motion-only BA (精化当前帧位姿, 固定路标) =====
    // 目的: 在坏观测进入 iSAM2 之前拦截——iSAM2 的 FEJ 让错误不可逆
    // 范围: 仅对显式路标 (SmartFactor 试用期路标依赖内部的 3σ 拒绝)
    // 成本: ~2ms, 4 次 GN 迭代, 6-DOF 变量, 零新依赖 (复用 GenericStereoFactor)
    // 对标: ORB-SLAM3 PoseOptimization (Tracking.cc motion-only BA)
    if explicit_obs_this_kf.size() >= 10:
        graph_mo = NonlinearFactorGraph()
        values_mo = Values()
        values_mo.insert(X(cur_id), pose_imu_predicted)  // IMU 预积分初值
        
        auto noise = Isotropic::Sigma(3, 1.5)             // 与显式因子一致
        auto fix = Isotropic::Sigma(3, 1e-6)              // 极紧先验 ≈ 固定路标
        
        for each obs in explicit_obs_this_kf:
            graph_mo.emplace_shared<GenericStereoFactor<Pose3,Point3>>(
                StereoPoint2(obs.uL, obs.uR, obs.v), noise,
                X(cur_id), L(obs.lmk_id), stereo_cal)
            values_mo.insert(L(obs.lmk_id), obs.point3d)
            graph_mo.addPrior(L(obs.lmk_id), obs.point3d, fix)
        
        result_mo = GaussNewtonOptimizer(graph_mo, values_mo,
            GaussNewtonParams{maxIter=4, errorTol=0}).optimize()
        refined_pose = result_mo.at<Pose3>(X(cur_id))
        
        // 用精化位姿覆盖 IMU 初值 → iSAM2 获得更好的线性化点
        new_values_.update(X(cur_id), refined_pose)
        
        // 标记 suspect 路标: 重投影 > χ² 阈值 → 留给 post-update 二次裁决
        for each obs in explicit_obs_this_kf:
            err = computeStereoReprojError(refined_pose, obs)
            if err.chi2 > 5.991: obs.suspect = true        // χ²_2, p=0.05
    
    // ===== 阶段 2: iSAM2 增量 BA 优化 =====
    // ⚠️ isam2.update() 本身就是 Bundle Adjustment!
    // 它在 Bayes Tree 中做增量非线性优化:
    //   1. 将 new_factors 在当前线性化点线性化 → 高斯因子
    //   2. 对受影响的 clique 做消元 (elimination) → 更新 Bayes Tree
    //   3. 对受影响变量做回代 (back-substitution) → 更新估计值
    //   4. 检查是否需要重线性化 (relinearizeThreshold)
    //
    // 被优化的变量:
    //   - X(k): 窗口内所有 KF 位姿 (受 SmartFactor+IMU+Prior 约束)
    //   - V(k): 窗口内所有 KF 速度 (受 IMU+Prior 约束)
    //   - B(k): 窗口内所有 KF IMU 偏置 (受 IMU+bias_walk+Prior 约束)
    //   - L(id): 显式路标 3D 位置 (受 GenericStereoFactor 约束)
    //
    // 注意: SmartFactor 的隐式路标在 SmartFactor 内部通过 Schur 补消去，
    //       不进入 iSAM2 状态向量。这是 SmartFactor "试用期"的本质。
    //
    // 与 ORB-SLAM3 的对比:
    //   ORB-SLAM3: 显式调用 Optimizer::LocalBundleAdjustment() 或 LocalInertialBA()
    //              → g2o LM 优化器, 批量迭代, 优化窗口内 KF+MapPoint
    //   Factor-VIO: isam2.update() 自动完成等价的增量平滑
    //              → 只重线性化受影响变量, 不需要显式调用 "BA 函数"
    result = updateSmoother(&result, new_factors_tmp, new_values_,
                            key_frame_count, delete_slots)
    // updateSmoother 内部:
    //   backup = shallow_copy(smoother)
    //   try: smoother->update(new_factors, new_values, timestamps, delete_slots)
    //   catch IndeterminantLinearSystem: 注入priors + 回滚 + 重试
    //   catch CheiralityException: cleanCheiralityLmk + 回滚 + 递归(最多5次)
    
    if not result: return false
    
    // ===== 阶段 3: 恢复 SmartFactor 槽位映射 =====
    // result.newFactorsIndices: 每个新因子的槽位索引 (与new_factors_tmp 1:1)
    updateNewSmartFactorsSlots(lmk_ids_tmp, old_smart_factors_, result)
    // 内部: old_smart_factors_[lmk_id].second = result.newFactorsIndices[i]
    
    // ===== 阶段 4: 更新状态 + 清理 =====
    state_ = smoother->calculateEstimate()
    updateStates(cur_id)                             // 从state_提取 X/V/B
    new_smart_factors_.clear()                       // 消耗完毕
    new_imu_prior_and_other_factors_.resize(0)       // 消耗完毕
    
    // ===== 阶段 5: 重复迭代 (如果 max_extra_iterations > 1) =====
    for n_iter = 1..max_extra_iterations:
        // 用更新后的状态重新线性化
        result = updateSmoother(&result)
        // Kimera: numOptimize=1 (Euroc YAML), 即不做额外迭代
    
    // ===== 阶段 6: 后处理统计 =====
    postDebug(total_start_time)                       // 计算优化前后误差对比
    computeSmartFactorStatistics()                    // 统计退化/远点/异常值
    
    return true
```

**关键数据结构**：

```cpp
// SmartFactor 槽位追踪 (VioBackend.h)
using Slot = long int;                              // -1=未入图, >=0=槽位
using SmartFactorMap = FastMap<LandmarkId,
    pair<SmartStereoFactor::shared_ptr, Slot>>;

// 成员变量
SmartFactorMap old_smart_factors_;                  // 已在图中的SmartFactor
LandmarkIdSmartFactorMap new_smart_factors_;        // 本轮新/更新的SmartFactor
NonlinearFactorGraph new_imu_prior_and_other_factors_; // IMU/Prior/其他
Values new_values_;                                 // 新变量初值
Values state_;                                      // 当前优化后的状态
```

### 5.3a 后端处理流程与 ORB-SLAM3 LocalMapping 的对照

ORB-SLAM3 没有 iSAM2 增量优化，它的 LocalMapping 线程是**批量处理**模式。两者架构差异如下：

| 维度 | Factor-VIO (iSAM2 增量) | ORB-SLAM3 LocalMapping (批量) |
|------|------------------------|------------------------------|
| 优化触发 | 每个 KF 触发一次 `isam2.update()` | 空闲时处理队列中所有 KF |
| 路标管理 | SmartFactor 隐式 + GenericStereoFactor 显式 | MapPoint 显式对象 + 引用计数 + BA |
| 路标剔除 | chi² 后验 + SmartFactor 内部拒绝 | `MapPointCulling()`: found_ratio<0.25 或 创建2KF后观测≤3 |
| 三角化 | SmartFactor 隐式 / 晋升时显式 | `CreateNewMapPoints()`: KF 间 ORB 匹配 + 视差检查 |
| BA | iSAM2 增量自动处理 | `LocalBundleAdjustment()` / `LocalInertialBA()` 显式调用 |
| 冗余 KF 剔除 | 边缘化自动处理 | `KeyFrameCulling()`: 90%(视觉)/50%(双目惯性) 观测被其他 KF 覆盖则剔除 |
| IMU 初始化 | 初始化阶段完成后开始 | `InitializeIMU()`: 在 LocalMapping 线程中渐进初始化 |

**ORB-SLAM3 LocalMapping::Run() 的精确循环** (`LocalMapping.cc:L75-L195`):

```
while true:
    if CheckNewKeyFrames():                    // 有新KF在队列中
        ProcessNewKeyFrame()                    // BoW转换 + 插入Map + 更新共视图
        MapPointCulling()                       // 剔除近期添加的坏MapPoint
        CreateNewMapPoints()                    // 与邻居KF三角化新MapPoint
        if not CheckNewKeyFrames():            // 队列空了 → 可以做BA
            SearchInNeighbors()                 // 在邻居KF中融合重复MapPoint
            
            if KeyFramesInMap > 2:
                if Inertial and IMU_initialized:
                    dist = 最近两KF的相机位移
                    if dist > 0.05: mTinit += dt
                    if not IniertialBA2 and mTinit<10s and dist<0.02:
                        ResetActiveMap()        // 运动不足 → 重置
                    LocalInertialBA(currentKF)  // 20+ KF + 100+路标
                else:
                    LocalBundleAdjustment(currentKF)
            
            if not IMU_initialized and Inertial:
                InitializeIMU(1e2, 1e5, true)   // 双目: prior_bias=1e2, prior_scale=1e5
            
            KeyFrameCulling()                    // 剔除冗余KF
    else:
        usleep(3000)                            // 3ms 休眠
```

**ORB-SLAM3 MapPointCulling() 的精确条件** (`LocalMapping.cc:L346-L385`):

```
function MapPointCulling():
    for each pMP in mlpRecentAddedMapPoints:
        if pMP->isBad():                                    // 已标记坏点 → 删除
        elif pMP->GetFoundRatio() < 0.25:                   // 预测可见中实际找到<25%
            pMP->SetBadFlag()                                // → 标记坏点(不立即删,等BA)
        elif (nCurrentKFid - pMP->mnFirstKFid) >= 2         // 创建≥2KF后
             and pMP->Observations() <= nThObs:             // 且观测≤3(双目)/2(单目)
            pMP->SetBadFlag()
        // 创建≥3KF后移出检查列表 (已充分验证或已被剔除)
```

**关键差异总结**：

1. **ORB-SLAM3 的 MapPoint 是显式的**——有 `SetBadFlag()` / `Replace()` / `isBad()` 生命周期，BA 优化其 3D 位置。Factor-VIO 中对应的是显式路标 (STABLE/EXPLICIT 状态)。

2. **ORB-SLAM3 的三角化是 KF 间批量操作** (`CreateNewMapPoints`)——从共视图中选前 10-30 个邻居 KF，做 ORB 匹配 + 视差检查 + 三角化。Factor-VIO 中对应的是 SmartFactor 的隐式三角化 + SVO Pro 深度滤波器。

3. **ORB-SLAM3 的 BA 是显式调用**——`LocalBundleAdjustment` (仅视觉) / `LocalInertialBA` (视觉+IMU)。Factor-VIO 中 iSAM2 自动完成等价的增量优化。

4. **ORB-SLAM3 的 `KeyFrameCulling`** 在 Factor-VIO 中没有直接对应——边缘化自动处理。但如果需要手动清理，可参考 ORB-SLAM3 的冗余观测阈值 (视觉90%/双目惯性50%)。

### 5.3b 后端与 ORB-SLAM3 Tracking 线程的对照

ORB-SLAM3 的 Tracking 线程是**逐帧**处理，与 LocalMapping **异步**。Factor-VIO 中 Tracking 和 LocalMapping 也在不同线程。

**ORB-SLAM3 Tracking::Track() 精确流程** (`Tracking.cc:L1794-L2332`):

```
function Track():
    // ===== 0: 预处理 =====
    if LocalMapper has mbBadImu flag:                       // LocalMapping 检测到IMU异常
        ResetActiveMap(); return
    
    // 时间戳异常检测
    if last_timestamp > current_timestamp:                  // 时间戳倒流
        CreateMapInAtlas(); return                           // 创建新地图
    if dt > 1.0s:                                           // 时间戳跳跃>1s
        if IMU_initialized: CreateMapInAtlas()              // 创建新地图
        else: ResetActiveMap()                               // 重置
    
    // ===== 1: IMU 预积分 (每帧, KF间累积) =====
    if Inertial and not first_frame:
        PreintegrateIMU()
        // 双目模式: mpImuPreintegratedFromLastKF 在 KF 间累积
        //            mCurrentFrame.mpImuPreintegratedFrame 帧间IMU
    
    // ===== 2: 状态机驱动的跟踪策略 =====
    switch mState:
        case NOT_INITIALIZED:
            StereoInitialization()                           // 双目: ≥500特征→直接初始化
            // 双目初始化: 立体匹配→3D点→创建初始Map和KF→mState=OK
        
        case OK:
            CheckReplacedInLastFrame()                       // LocalMapping可能替换了MapPoint
            if mbVelocity or IMU_initialized:                // ⚠️ OR逻辑(源码), 非AND
                bOK = TrackWithMotionModel()                 // 用mVelocity预测+投影匹配
            else:
                bOK = TrackReferenceKeyFrame()               // BoW加速的参考KF匹配
            
            if not bOK:
                if KFs_in_map > 10:
                    mState = RECENTLY_LOST                   // 短期丢失
                else:
                    mState = LOST                            // 长期丢失
        
        case RECENTLY_LOST:
            if IMU_initialized:
                PredictStateIMU()                            // IMU传播预测当前位姿
            bOK = Relocalization()                           // DBoW2全局重定位
            if bOK: mState = OK
        
        case LOST:
            if KFs_in_map <= 10: ResetActiveMap()            // 早期丢失→重置
            else: CreateMapInAtlas()                          // 创建新地图(保留旧地图)
    
    // ===== 3: 局部地图跟踪 (位姿精化) =====
    if bOK:
        bOK = TrackLocalMap()                                // 投影局部地图点到当前帧
        //   → 更多2D-3D匹配 → motion-only BA (仅优化当前帧位姿,固定MapPoint)
    
    // ===== 4: 速度模型更新 =====
    if bOK:
        mVelocity = T_cur_wc * T_last_cw                     // SE3速度
        mbVelocity = true
    
    // ===== 5: 关键帧决策 =====
    if bOK:
        if NeedNewKeyFrame():
            CreateNewKeyFrame()
            // → 插入 LocalMapping 队列
    
    // ===== 6: 异常值处理 =====
    if bOK:
        for each MapPoint in current_frame:
            if Observations < 1:
                mvbOutlier[i] = false; mvpMapPoints[i]=NULL  // 清理孤立观测
        DeleteTemporalPoints()                               // 清理临时三角化点
    
    // ===== 7: 丢失恢复 =====
    if mState == LOST:
        if KFs_in_map <= 10: ResetActiveMap()
        elif not IMU_initialized: ResetActiveMap()
        else: CreateMapInAtlas()                             // Atlas 多地图机制
    
    mLastFrame = Frame(mCurrentFrame)                        // 深拷贝当前帧为下一帧的上帧
```

**IMU 预测位姿** (`Tracking.cc:L1746-L1786`):

```cpp
// 从上一关键帧预测当前位姿 (IMU预积分)
Rwb2 = NormalizeRotation(Rwb1 * pim->GetDeltaRotation(bias_lastKF));
twb2 = twb1 + Vwb1*dt + 0.5*dt²*Gravity + Rwb1*pim->GetDeltaPosition(bias_lastKF);
Vwb2 = Vwb1 + dt*Gravity + Rwb1*pim->GetDeltaVelocity(bias_lastKF);
// Gravity = (0, 0, -9.81) 在 World 系中

// 或从上一帧预测 (帧间IMU)
Rwb2 = NormalizeRotation(Rwb1 * pim_frame->GetDeltaRotation(bias_lastFrame));
twb2 = twb1 + Vwb1*dt + 0.5*dt²*Gravity + Rwb1*pim_frame->GetDeltaPosition(bias_lastFrame);
Vwb2 = Vwb1 + dt*Gravity + Rwb1*pim_frame->GetDeltaVelocity(bias_lastFrame);
```

**ORB-SLAM3 状态机与 Factor-VIO 对照**：

| ORB-SLAM3 Tracking 状态 | Factor-VIO 对应 |
|------------------------|-----------------|
| `NO_IMAGES_YET` | `UNINITIALIZED` |
| `NOT_INITIALIZED` | `STATIC_CHECK → STATIC_INIT` / `DYNAMIC_CHECK → DYNAMIC_INIT` |
| `OK` | `NOMINAL` (正常跟踪) |
| `RECENTLY_LOST` | 跟踪质量下降 → 增大特征检测 + KLT降级策略 |
| `LOST` | 重定位 (全量ORB + DBoW3 + PnP) 或 重新初始化 |
| Atlas 多地图 | 单地图模式(简化); 或 RF2-Atlas 模式 |

### 5.4 BA 优化的本质：iSAM2 增量平滑 vs g2o 批量 BA

**Factor-VIO 中没有显式的"BA 函数调用"——`isam2.update()` 本身就是 BA。**

```
ORB-SLAM3 的 BA (显式 g2o 调用):
  Optimizer::LocalBundleAdjustment(kf, map)        ← 批量 LM 迭代
  Optimizer::LocalInertialBA(kf, map, ...)          ← 批量 LM 迭代 (含 IMU)
  优化变量: 窗口内 KF 位姿 + 路标 3D 位置 (+ 速度/偏置/重力)
  
Factor-VIO 的 BA (隐式 iSAM2 增量):
  isam2.update(new_factors, new_values, timestamps, delete_slots)
  内部流程:
    ① 新因子在当前线性化点线性化 → 高斯因子
    ② 对受影响 clique 消元 (elimination) → 更新 Bayes Tree
    ③ 回代 (back-substitution) → 更新估计值
    ④ 检查是否需要重线性化 (relinearizeThreshold=0.01)
  
  ⚠️ "增量"不是"只优化新变量"——受影响 clique 中的旧变量也被重线性化+重优化
```

**BA 层级对照**：

| ORB-SLAM3 | g2o 调用 | 优化变量 | Factor-VIO 等价物 | 缺失风险 |
|-----------|---------|---------|------------------|---------|
| Motion-only BA | `Optimizer::PoseOptimization` | 仅当前帧位姿, 固定 MapPoint | `isam2.update()` 加新因子——新位姿自然被单独优化 (旧 clique 未受影响) | 无 |
| Local BA | `Optimizer::LocalBundleAdjustment` | 当前 KF + 共视 KF + 路标 | `isam2.update()` 加新 SmartFactor——共视 KF 所在的 clique 被重线性化 | 无 |
| Local Inertial BA | `Optimizer::LocalInertialBA` | Local BA + 速度/偏置/重力 | `isam2.update()` 加新 IMU 因子 + SmartFactor——IMU 连接的变量所在 clique 被触发 | 无 |
| **Essential Graph** | `Optimizer::OptimizeEssentialGraph` | 仅 KF 位姿 (不含路标) | `isam2.update()` 加 BetweenFactor——Bayes Tree 增量传播回环校正 | ⚠️ 回环后**不**显式优化路标 |
| **Full BA** | `Optimizer::GlobalBundleAdjustment` | 所有 KF + 所有 MapPoint | ❌ **无增量等价物**——需从零构建因子图 + `LevenbergMarquardtOptimizer` | 🔴 |
| **Full Inertial BA** | `Optimizer::FullInertialBA` | Full BA + 速度/偏置/重力/尺度 | ❌ 同上 | 🔴 |

### 5.4a iSAM2 增量 vs g2o 多层 BA 的根本差异

ORB-SLAM3 在 **三个不同层次** 显式调用优化器：

```
Tracking 线程:    Motion-only BA (固定路标, 仅优化当前帧6-DOF)
                  └→ g2o, 4次 Gauss-Newton 迭代, ~3ms
                  
LocalMapping 线程: Local BA / LocalInertial BA (窗口KF+路标联合优化)
                  └→ g2o, 10次 LM 迭代, ~50-100ms
                  
LoopClosing 线程: Essential Graph (仅位姿) → Full BA (全变量)
                  └→ g2o, 20/100次 LM 迭代, 后台线程
```

Factor-VIO 只有 **一个 iSAM2**，所有优化通过增量 `isam2.update()` 隐式完成。这不是缺陷——是范式差异：

| 维度 | ORB-SLAM3 多层 BA | Factor-VIO 单 iSAM2 |
|------|------------------|-------------------|
| Motion-only BA | **显式**调用, 固定路标只优化位姿 | **隐式**——新 KF 进入 Bayes Tree 时，旧 clique 不受影响，新位姿被自然优化 |
| Local BA | **显式**批量 LM 迭代窗口内所有变量 | **隐式**——`isam2.update()` 的重线性化等价于对受影响 clique 做 BA |
| 回环后 Essential Graph | **显式**优化所有 KF 位姿 (不含路标) | **隐式**——BetweenFactor 注入后 Bayes Tree 增量传播校正 |
| 回环后路标优化 | **显式** Full BA 重新优化所有 MapPoint | **缺失**——iSAM2 做 incremental update 但不会像 Full BA 那样从头重建所有路标位置 |
| 边缘化 | 手动管理滑动窗口 + 先验 | iSAM2 自动按时间戳边缘化 |

**iSAM2 缺失 Full BA 的后果**：

1. **回环后路标不全局更新**：ORB-SLAM3 回环后做 Full BA，所有 MapPoint 的 3D 位置从头重优化。iSAM2 做增量更新——只有受回环因子影响的 clique 被重线性化，边缘化的旧路标**不会被重新优化**。

2. **边缘化先验不可逆**：被边缘化的旧 KF 变成 LinearContainerFactor（先验），其线性化点被 FEJ 固定。如果后续回环发现这些 KF 的位置有误，**先验无法修正**——错误被永久锁定在图中。

3. **长期误差累积**：连续边缘化使误差逐步固化，没有周期性的 Full BA 来全局矫正。这是 iSAM2 的固有问题——DM-VIO 的**延迟边缘化**正是为此设计。

**Factor-VIO 的缓解策略**：

1. **回环后触发 Global BA**：在回环累计 ≥3 次或漂移 >0.5m 时，从零构建因子图 + `LevenbergMarquardtOptimizer` 做批量优化。结果作为 iSAM2 的新初值（warm-start）。

2. **显式路标参与全局优化**：晋升后的显式 `Point3` 变量在 Global BA 中被重新优化，弥补 iSAM2 增量模式无法全局修正路标位置的短板。

3. **紧首帧先验 + 强 IMU 约束**：减少漂移累积，降低回环时的校正量，使得 iSAM2 增量传播足够覆盖校正范围。

### 5.4c ORB-SLAM3 的因子体系 (g2o 边) 与 Factor-VIO (GTSAM 因子) 对照

ORB-SLAM3 **不使用 SmartStereo**——它基于 g2o，所有路标都是显式的 `VertexSBAPointXYZ`。

**ORB-SLAM3 g2o 边类型** (`OptimizableTypes.h`):

| g2o 边 | 连接 | 残差维度 | 用途 | Factor-VIO 等价 |
|---------|------|---------|------|----------------|
| `EdgeSE3ProjectXYZ` | `VertexSE3Expmap`(pose) + `VertexSBAPointXYZ`(point) | 2 (单目重投影) | Local BA / Full BA 的视觉约束 | `GenericStereoFactor<Pose3,Point3>` |
| `EdgeStereoSE3ProjectXYZ` | `VertexSE3Expmap`(pose) + `VertexSBAPointXYZ`(point) | 3 (uL,uR,v) | 双目 Local BA 视觉约束 | `GenericStereoFactor<Pose3,Point3>` |
| `EdgeSE3ProjectXYZOnlyPose` | `VertexSE3Expmap`(pose) 单边 | 2 | Motion-only BA (固定路标,只优化位姿) | iSAM2 增量自动等价——新因子不影响路标所在 clique |
| `EdgeInertial` | `VertexSE3`(pose_i+j) + `VertexVelocity`(v_i+j) + `VertexGyroBias` + `VertexAccBias` | 9 (er,ev,ep) | **6顶点**(2pose+2vel+2bias) | `ImuFactor` + `BetweenFactor<ConstantBias>` |
| `EdgeInertialGS` | 上面 4 种 + `VertexGDir` + `VertexScale` | 9 | IMU 初始化优化 (含重力方向+尺度) | 初始化阶段专用因子图 |
| `EdgeSim3ProjectXYZ` | `VertexSim3` + `VertexSBAPointXYZ` | 2 | 回环 Sim3 约束 | `BetweenFactor<Pose3>` (回环) |

**ORB-SLAM3 g2o 顶点类型**:

| g2o 顶点 | 维度 | Factor-VIO 等价 |
|----------|------|----------------|
| `VertexSE3Expmap` | 6 | `X(k) = Pose3` |
| `VertexSBAPointXYZ` | 3 | `L(id) = Point3` |
| `VertexVelocity` | 3 | `V(k) = Vector3` |
| `VertexGyroBias` | 3 | `B(k).gyroscope()` |
| `VertexAccBias` | 3 | `B(k).accelerometer()` |
| `VertexGDir` | 2 (SO(3) 切空间) | 不显式建模; IMU 预积分隐式约束 |
| `VertexScale` | 1 | 双目已知基线, 不需要 |

**关键差异**:

| | ORB-SLAM3 (g2o) | Factor-VIO (GTSAM) |
|---|---|---|
| 路标存在形式 | **始终显式** `VertexSBAPointXYZ` | SmartFactor 隐式 (试用期) → `Point3` 显式 (晋升后) |
| 路标初始值 | 关键帧间 ORB 匹配+三角化 | 深度滤波器 (SVO Pro) → SmartFactor 隐式三角化 → 晋升时显式 |
| 路标剔除 | `MapPointCulling`: found_ratio<0.25 或 2KF 后观测≤3 | chi² 后验 + SmartFactor 内部拒绝 + 连续 3 帧异常值 |
| BA 中的路标角色 | **参与优化**——和位姿一起被 g2o LM 迭代 | 试用期: **不参与** (Schur 补消去); 晋升后: **参与** (iSAM2 状态变量) |
| 新增路标的 BA 成本 | O(n³) 随路标数增长 (全局批量) | 近似 O(1) 增量 (在固定窗口+稀疏连接前提下, clique 有界) |
| 路标总数上限 | 受限于批量 BA 计算时间 (通常 1000-3000) | 受限于 iSAM2 窗口大小 (固定, ~25-30 KFs 内的路标) |

### 5.4b 为什么 Kimera 用隐式路标，而 VINS/ORB-SLAM3 用显式路标？

这不是偶然的技术选择——是后端架构决定的必然结果。

#### Kimera-VIO 选择 SmartFactor 的原因

1. **GTSAM 血缘**。SmartFactor 是 MIT Luca Carlone 组贡献给 GTSAM 的核心创新（Carlone et al., ICRA 2014）。Kimera-VIO 由同组开发——用自己的因子是自然的。

2. **iSAM2 增量性能的关键保障**。SmartFactor 通过 Schur 补将路标从状态中消去——Bayes Tree 的规模只取决于位姿数，与路标数无关。这是 iSAM2 实现 近似常数时间增量更新的前提。

3. **双目天然有深度**。单目需要用深度滤波器、逆深度参数化等方式初始化路标深度。双目的 SmartFactor 从第一帧观测就能三角化——不需要额外的初始化系统。

4. **结构无关（structureless）范式的简洁性**。不管理路标变量生命周期、不维护路标地图、不处理路标合并/替换/跨图转移。代码量远小于 ORB-SLAM3 的 MapPoint 系统。

#### VINS-Fusion 选择显式路标的原因

1. **Ceres 没有 SmartFactor**。VINS-Fusion 的后端是 Ceres Solver，支持的是显式参数块。路标以逆深度（1 参数）参数化，在滑动窗口中与位姿、速度、偏置一起优化。

2. **滑动窗口的 Schur 补要求显式变量**。VINS 的边缘化需要显式的路标变量才能做 Schur 消元→先验压缩。没有显式路标变量，边缘化无法进行。

3. **工程惯性**。VINS-Fusion 从 VINS-Mono 演进而来，VINS-Mono 用显式路标达到了 SOTA。没有切换的动力。

#### ORB-SLAM3 选择显式路标的原因

1. **g2o 没有 SmartFactor 等价物**。g2o 的所有优化都基于显式顶点（VertexSE3Expmap、VertexSBAPointXYZ）。

2. **MapPoint 不只是 3D 坐标**。ORB-SLAM3 的 MapPoint 承载了大量比 3D 位置更重要的信息：
   - **ORB 描述子**：回环检测的基础。没有描述子就没有 DBoW2。
   - **观测历史**：哪些 KF 的第几个关键点观测了它——用于 Local BA 选边、MapPointCulling、重定位。
   - **统计信息**：`found_ratio`、`mnVisible`、`mnFound`——决定路标质量的依据。
   - **替换/合并**：`Replace()` 将一个 MapPoint 的观测转移给另一个——Atlas 多地图合并的基础。

3. **Full BA 需要显式路标**。ORB-SLAM3 的 Global Bundle Adjustment 同时优化所有 KF 和所有 MapPoint。SmartFactor 的隐式路标无法参与全局优化。

4. **重定位需要描述子**。跟踪丢失后，ORB-SLAM3 用当前帧的 ORB 描述子匹配 MapPoint 的描述子做 PnP 重定位。SmartFactor 没有描述子。

#### 三种选择的核心约束

```
后端是 g2o (ORB-SLAM3)
  → 必须显式路标（VertexSBAPointXYZ）
  → 附带获得：描述子、回环、重定位、Full BA

后端是 Ceres (VINS-Fusion)
  → 必须显式路标（参数块）
  → 附带获得：逆深度参数化、滑动窗口边缘化

后端是 GTSAM iSAM2 (Kimera-VIO)
  → 可选 SmartFactor 或显式路标
  → 选择 SmartFactor：近似常数时间增量、无路标管理负担
  → 代价：无描述子、无回环集成、无 Full BA
```

#### Factor-VIO 的混合路线

**Kimera 选 SmartFactor 是因为 iSAM2 让它成为可能，不是因为显式路标不好。**

Factor-VIO 的"试用期 SmartFactor → 晋升显式路标"正是要同时获得两者的优势：
- 日常运营：SmartFactor 的 近似常数时间增量和低管理负担
- 回环/全局 BA/重定位：显式路标的描述子和全局优化能力

这和 OKVIS2 的设计理念一致——OKVIS2 在正常运营时将路标边缘化为位姿图边（类似 SmartFactor），回环时再逆向恢复为显式路标和观测。

#### 一、架构哲学

```
ORB-SLAM3: "先信任, 后验证"
  路标创建 → 立即可用 (VertexSBAPointXYZ)
  → 参与 BA 优化 3D 位置
  → MapPointCulling 事后剔除不靠谱的
  
Factor-VIO: "先验证, 后信任"  
  路标创建 → 试用期 (SmartFactor, 隐式)
  → 不参与 BA，只约束位姿
  → 10 条件晋升门控通过后 → 显式 Point3 → 参与 BA
```

#### 二、路标生命周期

| 阶段 | ORB-SLAM3 | Factor-VIO |
|------|----------|-----------|
| 创建 | KF 间 ORB 匹配 + 三角化 → 立即 `new MapPoint` → `VertexSBAPointXYZ` | 立体匹配成功 → `DEPTH_FILTER` → 深度收敛 → `SMART_TRIAL` |
| 初值 | 三角化结果直接作为 g2o 顶点初值 | 深度滤波器收敛值 → SmartFactor 隐式三角化 |
| 早期优化 | BA 中参与迭代（位姿+路标联合优化） | SmartFactor 内部 Schur 补——路标不进入状态，只约束位姿 |
| 中期稳定 | 连续观测 → `mnFound/mnVisible` 统计 | 4+ 帧观测 + 10 条件通过 → 晋升为显式 `Point3` |
| 剔除 | `MapPointCulling()`: found_ratio<0.25 或 2KF 后观测≤3 | chi² 后验连续 3 帧 >7.815 → `CULLED` |
| 长期 | 参与 Full BA → 全局一致性优化 | iSAM2 边缘化 → `MARGINALIZED` (作为先验保留) |

#### 三、BA 中的行为差异

**ORB-SLAM3 的 BA** (g2o 批处理):
```
每次 Local BA 调用:
  optimizer.initializeOptimization()
  for 每个局部 KF:
      addVertex(VertexSE3Expmap)           ← 位姿顶点
      addVertex(VertexVelocity)            ← 速度顶点 (Inertial BA)
      addVertex(VertexGyroBias)            ← 偏置顶点 (Inertial BA)
  for 每个局部 MapPoint:
      addVertex(VertexSBAPointXYZ)         ← 路标顶点 (始终存在!)
  for 每个观测:
      addEdge(EdgeStereoSE3ProjectXYZ)     ← stereo 边
      addEdge(EdgeSE3ProjectXYZ)           ← mono 边
      addEdge(EdgeInertial)                ← IMU 边
  optimizer.optimize(steps)                ← g2o LM 批量迭代
  → 位姿 + 路标 + 速度 + 偏置 + 重力同时优化
```

**Factor-VIO 的 BA** (iSAM2 增量):
```
每个 KF 的 isam2.update():
  ① 新因子线性化 → 高斯因子
  ② 对受影响 clique 消元 → Bayes Tree 更新
  ③ 回代 → 变量更新
  
  参与优化的变量:
    试用期路标: 不在状态中 (Schur 补消去)
    晋升后路标: L(id) = Point3 ← 在状态中, 参与 clique 消元+回代
  
  → 增量平滑: 不是每次对所有变量 LM 迭代
  → 但受影响 clique 中的旧变量 (含显式路标) 会被重线性化+重优化
```

#### 四、计算成本

| | ORB-SLAM3 (g2o 批处理) | Factor-VIO (iSAM2 增量) |
|---|---|---|
| 每次 Local BA 时间 | O(n³), n = KF+路标数 (通常 50-200 变量) | O(1) 增量 (约 5-40ms, 取决于受影响 clique 大小) |
| 路标数对 BA 的影响 | **线性增长** — 每个路标增加 3 维状态 + 多条边 | 试用期: 无影响 (Schur 补消去); 晋升后: 增加 clique 大小 |
| Full BA 时间 | O(n³), n 可达数千 | 无增量等价物; 需从零构建因子图 + 批量优化 |
| 内存 | 所有 KF + 所有 MapPoint 保留在内存中 | iSAM2 窗口内变量 (~25-30 KFs + 显式路标) + 边缘化先验 |

#### 五、鲁棒性

| 场景 | ORB-SLAM3 | Factor-VIO |
|------|----------|-----------|
| 三角化初值差 | **BA 可修正**——路标是变量, 多帧观测在批量迭代中拉回正确位置 | **试用期危险**——SmartFactor 隐式三角化依赖位姿精度, 位姿偏则三角化偏, 无迭代修正 |
| 路标误匹配 (outlier) | **事前剔除弱**——三角化直接进 BA; **事后剔除强**——MapPointCulling + BA 中 Huber 核 | **事前过滤强**——双轮 RANSAC + 深度滤波 + 10 条件门控; 晋升后 chi² 监测 |
| 路标被动态物体污染 | 进入 BA → 可能污染位姿估计 → 后续 MapPointCulling 剔除 | 难以通过双轮 RANSAC + 深度滤波 → 难以进入 SmartFactor 试用期 |
| 弱纹理场景 (MH) | ORB 特征不足 → 三角化困难 → 路标数量少 → BA 约束弱 | KLT 在弱纹理下跟踪困难 → SmartFactor 退化率高 → 晋升率低 |
| 快速旋转 | ORB 匹配困难 → 路标创建少 | IMU 预测 KLT → 跟踪成功率高于 ORB 匹配 |

#### 六、事后补救机制对比

**ORB-SLAM3 的事后补救**:

| 机制 | 位置 | 触发条件 | 动作 |
|------|------|---------|------|
| `MapPointCulling()` | `LocalMapping.cc:L346` | found_ratio<0.25 或 创建≥2KF后观测≤3 | `SetBadFlag()` — 标记坏点, BA 不再使用 |
| `CheckReplacedInLastFrame()` | `Tracking.cc:L1943` | LocalMapping 的 BA 替换了 MapPoint | 用新 MapPoint 替换 Tracking 中持有的旧指针 |
| BA 内的 Huber 核 | `Optimizer.cc` 中 g2o edge 的 `setRobustKernel` | 重投影残差异常大 | Huber 降权异常观测, 在 BA 迭代中自然收敛或削弱 |
| `SearchInNeighbors()` | `LocalMapping.cc:L108` | 新 KF 插入后, 与邻居 KF 融合重复 MapPoint | 合并同一物理点的多个 MapPoint |
| `KeyFrameCulling()` | `LocalMapping.cc:L191` | 90%(视觉)/50%(双目惯性) MapPoint 被其他 ≥3 KF 观测 | 剔除冗余 KF, 释放内存 |

**Factor-VIO 的事后补救**:

| 机制 | 触发条件 | 动作 |
|------|---------|------|
| **chi² 后验异常值剔除** (§3.5) | 显式路标 `chi² > 7.815` (χ²₃, p=0.05) | 标记异常; 连续 3 帧 → `CULLED` |
| **`REMEDIATING` 状态** (§1.3 路标状态机) | 显式路标偶发异常 | 允许恢复: 若后续 chi² 连续通过 → 回到 `STABLE`; 若持续异常 → `CULLED` |
| **SmartFactor 内部异常值拒绝** | `DynamicOutlierRejectionThreshold=3.0` | 3σ 外重投影观测被 SmartFactor 内部拒绝, 不参与隐式三角化 |
| **`prunePostUpdateOutlierObservations`** | chi² 后验发现异常观测 | 重建 SmartFactor (移除异常观测) 或降级为 CULLED |
| **退化检测 (SmartFactor)** | `isDegenerate()==true` | ZERO_ON_DEGENERACY 模式: 退化路标的 Jacobian 归零, 不影响位姿优化 |
| **updateSmoother Cheirality 恢复** (§5.6) | 路标三角化到相机后方 | 递归删除该路标全部因子 + 回滚 iSAM2 + 重试 (最多 5 次) |

**两者的关键差异**:

- ORB-SLAM3 的事后补救**发生在 BA 之内** (Huber 核在优化迭代中起作用) 和**BA 之外** (MapPointCulling 事后剔除)。路标始终参与 BA。
- Factor-VIO 的事后补救**发生在 iSAM2 更新之后** (chi² 检验, REMEDIATING)。试用期路标不参与 BA 但 SmartFactor 内部有统计拒绝。晋升后的显式路标有完整的 chi² 监测+恢复路径。
- ORB-SLAM3 在路标质量差时**降权**（Huber 减小残差权重），Factor-VIO 在路标质量差时**先降级再考察**（REMEDIATING → 恢复或剔除）。

**ORB-SLAM3 显式路标的优势**:
1. **BA 可修正初值错误**——路标 3D 位置在批量 LM 迭代中不断精化
2. **全局一致性更好**——Full BA 同时优化所有变量, 误差分布更均匀
3. **实现更简单**——路标始终是显式顶点, 没有"试用期→晋升"的转换逻辑
4. **回环后路标自动更新**——Full BA 重新优化所有路标位置

**ORB-SLAM3 显式路标的劣势**:
1. **坏路标污染 BA**——错误三角化的路标会拉偏位姿, 依赖 Huber 核 + MapPointCulling 事后补救
2. **计算随路标数增长**——不能无限增加路标, 需 KeyFrameCulling 控制规模
3. **路标初始三角化无质量门控**——ORB 匹配 + 视差检查是仅有的过滤
4. **内存占用大**——所有 KF 和 MapPoint 长期驻留

**Factor-VIO 隐式→显式的优势**:
1. **试用期保护**——坏路标在 SmartFactor 阶段自然退化, 不污染 BA 状态
2. **O(1) 增量**——计算量与路标总数无关 (仅受影响 clique)
3. **多级质量门控**——双轮 RANSAC + 深度滤波 + 10 条件晋升 + chi² 后验
4. **降级不删除**——3D-3D RANSAC 外点保留 2D bearing 约束

**Factor-VIO 隐式→显式的劣势**:
1. **试用期无法迭代修正**——如果初值差且通过了门控, 晋升后的初值仍然差
2. **晋升时机敏感**——晋升太早 (信息不足) 或太晚 (被边缘化前) 都会降低效果
3. **复杂度高**——需要维护两套路标管理代码 (SmartFactor 槽位 + 显式路标 Key)
4. **回环后需手动提升**——受影响 SmartFactor 要重新三角化并晋升

#### 七、适用场景推荐

| 场景 | 推荐 |
|------|------|
| 纹理丰富 (V1_01, V2_02) | 两者均可; Factor-VIO 的计算效率更优 |
| 弱纹理 (MH_01-MH_05) | ORB-SLAM3 的显式路标更可靠 (KLT 在弱纹理下退化严重) |
| 动态环境 | Factor-VIO 的多级过滤更有优势 |
| 长距离/大场景 | ORB-SLAM3 (Full BA + Atlas 多地图) vs Factor-VIO (仅窗口内, 需回环) |
| 嵌入端/低算力 | Factor-VIO (近似常数时间增量) |
| 需要稠密路标 | ORB-SLAM3 (显式路标可直接用于重定位) |

| ORB-SLAM3 | g2o 调用 | 优化变量 | Factor-VIO 等价物 |
|-----------|---------|---------|------------------|
| Motion-only BA | `Optimizer::PoseOptimization` | 仅当前帧位姿, 固定 MapPoint | `isam2.update()` 加新因子——新位姿自然被单独优化 (旧 clique 未受影响) |
| Local BA | `Optimizer::LocalBundleAdjustment` | 当前 KF + 共视 KF + 路标 | `isam2.update()` 加新 SmartFactor——共视 KF 所在的 clique 被重线性化 |
| Local Inertial BA | `Optimizer::LocalInertialBA` | Local BA + 速度/偏置/重力 | `isam2.update()` 加新 IMU 因子 + SmartFactor——IMU 连接的变量所在 clique 被触发 |
| Full BA | `Optimizer::GlobalBundleAdjustment` | 所有 KF + 所有 MapPoint | 非增量: 从零构建因子图 + `LevenbergMarquardtOptimizer` |
| Full Inertial BA | `Optimizer::FullInertialBA` | Full BA + 速度/偏置/重力/尺度 | 非增量: 同上, 含 IMU 变量 |
| PGO (回环后) | `Optimizer::OptimizeEssentialGraph` | 仅 KF 位姿 (不含路标) | `isam2.update()` 加 BetweenFactor——Bayes Tree 增量传播回环校正 |

**iSAM2 比 g2o 批量 BA 的关键优势**：

1. **选择性重线性化**：不是每帧对整个窗口重新 LM 迭代——只重新线性化 Bayes Tree 中受新因子影响的 clique。这使增量平滑的时间接近常数时间而非 O(n³)。

2. **自动边缘化**：`IncrementalFixedLagSmoother` 按时间戳自动边缘化旧变量。ORB-SLAM3 需要手动管理滑动窗口。

3. **因子移除与重线性化**：iSAM2 支持通过 `delete_slots` 移除因子，受影响 clique 可被重线性化。但**已边缘化的旧变量信息变为线性化先验（LinearContainerFactor），通常不可无损恢复**。若需要回环后重建旧观测，应借鉴 OKVIS2 的 pose-graph edge revival 或 Basalt 的 non-linear factor recovery 机制。

**iSAM2 的局限**：

1. **FEJ (First-Estimates Jacobian)**：边缘化先验的线性化点一旦固定就不能变。如果边缘化时 scale/gravity/bias 尚未收敛, 错误会固化。参见 [[概念-Schur补与边缘化]] 和 DM-VIO 的延迟边缘化。

2. **需要好的初值**：增量平滑对初值敏感——差的初值导致线性化点偏, 后续增量修正幅度有限。ORB-SLAM3 的批量 BA 在初值差时可以通过多次 LM 迭代修正。

### 5.5 iSAM2参数配置

```cpp
// 参考 GTSAM VisualISAM2Example + ISAM2Example_SmartFactor + Kimera-VIO
gtsam::ISAM2Params isam_param;
isam_param.optimizationParams = gtsam::ISAM2GaussNewtonParams();
isam_param.optimizationParams.wildfireThreshold = 0.001;
isam_param.cacheLinearizedFactors = true;        // 性能优化
isam_param.relinearizeThreshold = 0.01;          // 激进重线性化 (GTSAM 示例一致)
isam_param.relinearizeSkip = 1;                  // 每步检查 (示例一致)
isam_param.findUnusedFactorSlots = true;         // 防止槽位泄漏
isam_param.enableDetailedResults = true;         // ★ 启用 ISAM2Result.detail (GTSAM示例)
isam_param.evaluateNonlinearError = true;        // ★ 启用 errorBefore/errorAfter
isam_param.factorization = gtsam::ISAM2Params::CHOLESKY;

auto smoother = IncrementalFixedLagSmoother(
    lag_seconds = 5.0,           // 约25-30关键帧
    isam_param);
```

**GTSAM 示例中的关键模式**：

1. **增量更新后必须清空输入** (`VisualISAM2Example:L138-139`):
```cpp
isam.update(graph, initialEstimate);   // 只传 NEW 因子和初值
graph.resize(0);                        // ← 每次更新后清空!
initialEstimate.clear();                // ← 每次更新后清空!
```

2. **额外迭代** (`VisualISAM2Example:L131`): `isam.update()` 不带参数调用一次（额外重线性化），Factor-VIO 通过 `max_extra_iterations` 支持此模式。

3. **首帧固定**: GTSAM 示例用 `NonlinearEquality<Pose3>(1, Pose3())` 而非 `PriorFactor` 来固定首帧（StereoVOExample:L41），彻底消除 gauge freedom。Factor-VIO 用极紧 `PriorFactor` (σ=1e-5) 达到近似效果。

4. **SmartFactor 单例复用 vs clone-and-add**: GTSAM 示例中同一个 SmartFactor 对象跨多次 `isam.update()` 复用，直接 `smartFactor->add(meas, key)` 追加观测（ISAM2Example_SmartFactor:L83）。Kimera-VIO 和 Factor-VIO 用 clone-and-add 避免直接修改已线性化的因子——两者均可，clone-and-add 更安全。

5. **GTSAM 官方立体 VO 示例用显式路标而非 SmartFactor**: `StereoVOExample.cpp` 直接使用 `GenericStereoFactor<Pose3, Point3>` + 显式 `Point3` 变量（L50-67），不用 SmartFactor。这验证了 Factor-VIO 的核心设计决策——显式路标是 GTSAM 的推荐模式，SmartFactor 是服务于"试用期"的优化手段。

### 5.6 updateSmoother异常恢复 (Kimera-VIO模式)

```
procedure updateSmoother(new_factors, new_values, timestamps, delete_slots):
    backup = shallow_copy(smoother)    // 浅拷贝:共享factor ptr,拷贝Bayes Tree
    
    try:
        return smoother->update(new_factors, new_values, timestamps, delete_slots)
    
    catch IndeterminantLinearSystemException:
        // 秩亏 → 在first_key和nearbyVariable添加6个PriorFactor
        //   pose(0.01rad, 0.1m), vel(0.1m/s), bias(从参数)
        smoother = backup
        nfg = new_factors + 6_prior_factors
        return smoother->update(nfg, new_values, timestamps, delete_slots)
        // 若仍失败: return false
    
    catch CheiralityException | StereoCheiralityException:
        // 路标在相机后方 → 删除该路标的全部因子,递归重试(最多5次)
        counter++
        if counter > 5: return false
        smoother = backup
        cleaned = cleanCheiralityLmk(lmk_key, new_factors, new_values, timestamps, delete_slots)
        return updateSmoother(cleaned)  // 递归
    
    catch (InvalidNoiseModel | InvalidMatrixBlock | InvalidDenseElimination |
           InvalidArgumentThreadsafe | ValuesKeyDoesNotExist | CholeskyFailed |
           RuntimeErrorThreadsafe | OutOfRangeThreadsafe | ...共12种):
        printSmootherInfo(...)    // 打印完整诊断
        return false
```

### 5.7 诊断监控

| 指标 | 正常范围 | 告警阈值 |
|------|---------|---------|
| isam2.error/n_factors | <1.0 | >2.0 |
| nDegenerate/nTotal (SmartFactor) | <0.3 | >0.5 |
| nOutlierRejected/nTotal | <0.1 | >0.3 |
| relinearizedCount | <5 | >20 |
| imuBiasDrift/frame | <1e-6 | >1e-4 |

---

## 六、回环设计

### 6.1 检测管线

```
function detectLoop(query_kf):
    // 1. DBoW3查询
    bow_vec = dbow3.transform(query_kf.orb_descriptors)
    results = dbow3.query(bow_vec, max_results=5)
    
    // 2. 分值过滤
    candidates = [r for r in results if r.score > 0.015]
    
    // 3. 时间过滤 (排除最近50帧)
    candidates = [r for r in candidates if query_kf.id - r.kf_id > 50]
    
    // 4. 分组一致性 (连续3帧确认)
    for c in candidates:
        if loop_groups[c.kf_id].count >= 3:
            loop_candidates.push(c)
    
    return best_candidate
```

### 6.2 几何验证

```cpp
function geometricVerify(query_kf, loop_kf):
    // 主路径: 3D-2D PnP RANSAC (使用显式路标的3D位置)
    pts3d = explicit_landmarks.at(loop_kf).get3DPoints()
    pts2d = query_kf.getObservations(loop_kf.landmark_ids)
    
    if pts3d.size() >= 15:
        success, T_rel, inliers = solvePnPRansac(
            pts3d, pts2d, K,
            minInliers=15, confidence=0.99,
            maxIter=300, reprojThreshold=3.0)
        if success:
            // 额外验证: 共视邻居确认(≥3个)
            covisible = countCovisibleNeighbors(query_kf, loop_kf)
            if covisible < 3: return REJECTED
            return ACCEPTED, T_rel
    
    // 回退: 2D-2D对极几何
    F, inliers = findFundamentalMat(pts2d_query, pts2d_loop)
    if inliers > 30:
        return ACCEPTED_2D, decomposeE(K.inv()*F*K)
    
    return REJECTED
```

### 6.3 回环注入与Post-loop处理

```
procedure onLoopAccepted(query_kf, loop_kf, T_rel, covariance):
    // 1. 注入回环因子
    loop_factor = BetweenFactor<Pose3>(
        X(query_kf.id), X(loop_kf.id), T_rel,
        noiseModel::Gaussian::Covariance(covariance))  // 从PnP内点估计,非Identity!
    loop_factor_with_huber = Robust(Huber(1.345), loop_factor)
    
    // 2. iSAM2更新 → 传播校正到位姿
    smoother->update({loop_factor}, {}, {}, {})
    
    // 3. 检测受影响的SmartFactor (loop_kf + 1-hop邻居)
    affected_kfs = {loop_kf.id} ∪ loop_kf.covisible_neighbors
    for each smart_lmk in old_smart_factors:
        if smart_lmk.connected_poses ∩ affected_kfs ≠ ∅:
            // 4. 用校正后位姿重三角化 → 提升为显式
            if canPromote(smart_lmk, corrected_estimate):
                promoteLandmark(smart_lmk)
    
    // 5. 再次iSAM2更新 → 纳入提升后的显式因子
    smoother->update(promoted_factors, promoted_values, {}, promoted_delete_slots)
```

---

## 七、坐标系约定

### 7.1 位姿变换语义

```
T_a_b = 将点从坐标系b变换到坐标系a的刚体变换

世界系 (W): ENU, Z-up, 重力方向 [0,0,-9.81]
IMU体 (B): IMU芯片坐标系, X-前 Y-右 Z-下 (常见)
左相机 (C_L): 相机坐标系, Z-前
右相机 (C_R): 与左相机差基线baseline

T_w_b: body→world (iSAM2优化变量)
T_b_cam: camera→body (外参, body_P_sensor)
```

### 7.2 GTSAM因子中的坐标语义

| 因子 | body_P_sensor 含义 | 验证 |
|------|-------------------|------|
| `SmartStereoProjectionPoseFactor` | **T_body_camera** (相机在IMU系) | EuRoC: `(0.05, 0, 0)` |
| `GenericStereoFactor<Pose3,Point3>` | 内嵌在 Cal3_S2Stereo 基线中 | 同上 |
| `ImuFactor` / `CombinedImuFactor` | IMU 外参通常为 identity (body=IMU); 预积分测量已在 IMU frame | **不要**混用 camera extrinsic |

**⚠️ 陷阱**: 如果传入T_camera_body (反了), 相机位置差一个符号, 所有三角化点跑到相机后方。

### 7.3 Cal3_S2Stereo构造

```cpp
// 六参数: (fx, fy, skew, u0, v0, baseline)
// 四参数便捷版: (fx, u0, v0, baseline) — 假设fx=fy, skew=0

// 正确:
auto K = Cal3_S2Stereo(rect_cam.K(0,0), rect_cam.K(0,2), rect_cam.K(1,2), baseline);
//                      fx              u0=cx            v0=cy            b

// ⚠️ 验证: 构造后打印K.fy()应≈K.fx(), K.px()应≈cx
```

### 7.4 重力方向统一

```
本方案: ENU (Z-up), gravity = [0, 0, -9.81]
GTSAM MakeSharedU = ENU (本方案选择)
GTSAM MakeSharedD = NED (不兼容, 会导致优化发散)
```

---

## 八、参数速查表

### 8.1 前端参数

| 参数 | 值 |
|------|-----|
| KLT窗口 | 21×21 |
| KLT金字塔 | 3层 |
| KLT迭代/ε | 30/0.01 |
| 双向光流阈值 | 0.5px |
| 目标特征数 | 300/帧 |
| qualityLevel | 0.001 |
| NCC模板 | 101×11 |
| NCC阈值 | 0.15 |
| 深度范围 | [0.3, 15.0]m |
| 2D-2D RANSAC阈值 | 1.0e-6 |
| 2D-2D最少内点 | 10 |
| 3D-3D RANSAC阈值 | 1.0 |
| 3D-3D最少内点 | 5 |
| 特征年龄上限 | 25 KFs |
| KF最小间隔 | 0.2s |
| KF最小视差 | 0.5px |

### 8.2 IMU参数 (ADIS16448, EuRoC)

| 参数 | 值 |
|------|-----|
| gyro_noise | 1.92e-4 rad/s/√Hz |
| accel_noise | 1.83e-3 m/s²/√Hz |
| gyro_walk | 4.0e-6 rad/s²/√Hz |
| accel_walk | 2.0e-4 m/s³/√Hz |
| gravity | 9.81007 m/s² |
| IMU频率 | 200 Hz |

### 8.3 路标参数

| 参数 | 值 |
|------|-----|
| 深度滤波收敛 | σ² < (μ_range/200)² |
| 晋升最少观测 | 4帧 |
| 晋升视差角 | >3° |
| 晋升重投影 | <2.0px |
| chi²阈值 | 7.815 (χ²₃, p=0.05) |
| 连续异常剔除 | 3帧 |
| SmartFactor pixelσ | 3.0px |
| Explicit pixelσ | 1.5px |

### 8.4 iSAM2参数

| 参数 | 值 |
|------|-----|
| 优化器 | GaussNewton |
| 分解 | CHOLESKY |
| relinearizeThreshold | 0.01 |
| relinearizeSkip | 1 |
| wildfireThreshold | 0.001 |
| cacheLinearized | true |
| findUnusedSlots | true |
| 窗口大小 | 25-30 KFs |

---

## 九、探针系统设计

> 参考 Kimera-VIO `DebugVioInfo` + `DebugTrackerInfo` + ORB-SLAM3 `REGISTER_TIMES` 模式。
> 每个模块独立维护探针结构体，在关键节点收集数据。支持 compile-time 宏控制 + 运行时 JSONL 输出。

### 9.1 全局配置与宏控制

```cpp
#define FACTOR_VIO_ENABLE_PROBES    1    // 0=编译排除(嵌入式), 1=启用

struct ProbeConfig {
    // 逐模块控制
    bool enable_frontend = true;
    bool enable_landmarks = true;
    bool enable_backend = true;
    bool enable_init = true;
    bool enable_loop = true;

    // 输出控制
    bool jsonl_per_kf = true;          // 每KF输出一行JSONL
    bool jsonl_on_abnormal_only = false; // 仅异常时输出
    std::string jsonl_path = "/tmp/factor_vio_diag.jsonl";

    // 图片输出 (仅前端)
    bool save_kf_images = false;       // 关键帧可视化
    bool save_stereo_images = false;   // 立体匹配可视化
    std::string image_dir = "/tmp/factor_vio_img";

    // 异常触发全量 dump
    bool dump_on_factor_failure = true;
    bool dump_on_cheirality = true;
    bool dump_on_tracking_lost = true;
    std::string dump_dir = "/tmp/factor_vio_dump";

    // 直方图收集 (可周期性 reset)
    int histogram_reset_interval_kfs = 100;
};
```

### 9.2 前端探针

#### 数据结构

```cpp
struct FrontendProbe {

    // ======== PHASE 3: KLT 跟踪 ========
    int klt_attempted = 0;            // calcOpticalFlowPyrLK 输入的 prev_pts 数
    int klt_success = 0;              // status[i]==1 的特征数
    int klt_fb_failed = 0;            // 双向光流验证失败数 (||back-prev|| > 0.5px)
    int klt_border_failed = 0;        // 边界外剔除数 (x<0 || x>=W || y<0 || y>=H)
    int klt_aged_out = 0;             // track_age > max_feature_track_age(25) 强制淘汰
    double klt_mean_error_px = 0;     // calcOpticalFlowPyrLK 返回 error 的均值

    // ======== PHASE 5: 2D-2D RANSAC ========
    int r2d2d_putatives = 0;          // RANSAC 输入: 有效 feature 匹配对总数
    int r2d2d_inliers = 0;            // RANSAC 内点数 (epipolar 距离 < 1e-6)
    int r2d2d_iters = 0;             // RANSAC 实际迭代次数
    double r2d2d_inlier_ratio = 0;   // r2d2d_inliers / r2d2d_putatives

    // ======== PHASE 5b: 3D-3D RANSAC ========
    int r3d3d_putatives = 0;          // RANSAC 输入: 有 3D 深度的匹配对总数
    int r3d3d_inliers = 0;            // RANSAC 内点数 (Mahalanobis² < 1.0)
    int r3d3d_downgraded = 0;        // 外点但保留为 mono-only (降级不删)
    int r3d3d_iters = 0;             // RANSAC 实际迭代次数
    double r3d3d_inlier_ratio = 0;

    // ======== PHASE 8: 立体匹配 ========
    int stereo_matched = 0;           // NCC < 0.15 && disparity > 0
    int stereo_failed_ncc = 0;       // NCC >= 0.15 (匹配质量不足)
    int stereo_failed_disp = 0;      // disparity <= 0 (无效视差)
    int stereo_depth_oob = 0;         // depth < 0.3m 或 > 15m
    double stereo_ncc_mean = 0;       // 成功匹配的 NCC 均值
    double stereo_ncc_min = 0;        // 成功匹配中最好的 NCC 分数
    double stereo_ncc_max = 0;        // 成功匹配中最差的 NCC 分数

    // ======== 右目关键点状态分布 ========
    int rkp_valid = 0;               // KeypointStatus::VALID
    int rkp_no_left = 0;             // NO_LEFT_RECT
    int rkp_no_right = 0;            // NO_RIGHT_RECT
    int rkp_no_depth = 0;            // NO_DEPTH
    int rkp_failed_arun = 0;         // FAILED_ARUN (3D-3D 外点)

    // ======== 特征空间分布 ========
    // 将图像划分为 grid_rows × grid_cols 网格, 统计每格特征数
    static constexpr int grid_rows = 5, grid_cols = 7;
    int grid_counts[grid_rows][grid_cols] = {};
    double grid_mean = 0;            // 每格平均特征数
    double grid_stddev = 0;          // 标准差
    double grid_uniformity = 0;      // = 1.0 - stddev/mean (1.0=完全均匀)
    int grid_empty_cells = 0;        // 特征数为 0 的网格数

    // ======== PHASE 7b: 特征检测 ========
    int new_features_detected = 0;   // goodFeaturesToTrack 返回的角点数
    int raw_corners_before_anms = 0; // ANMS 前的原始候选数 (~2000)
    double feature_survival_rate = 0;// 当前帧活跃特征 / 上帧活跃特征

    // ======== 耗时 (ms) ========
    double time_klt_ms = 0;
    double time_stereo_ms = 0;
    double time_ransac_ms = 0;       // 2D-2D + 3D-3D 合计
    double time_detect_ms = 0;       // GFTT + ANMS
    double time_total_ms = 0;
};
```

#### 收集点

| 收集时机 | 填充的字段 |
|---------|-----------|
| PHASE 3 完成后 | `klt_attempted`, `klt_success`, `klt_mean_error_px` |
| PHASE 3b 完成后 | `klt_aged_out` |
| PHASE 4 完成后 | `klt_fb_failed`, `klt_border_failed` |
| PHASE 5 完成后 | `r2d2d_*` |
| PHASE 5b 完成后 | `r3d3d_*`, `rkp_*` |
| PHASE 8 完成后 | `stereo_*`, `grid_*` |
| PHASE 7b 完成后 | `new_features_*`, `raw_corners_*` |
| 帧结束时 | `feature_survival_rate`, `time_*` |

#### 健康检查

```cpp
struct FrontendAlerts {
    bool klt_quality_warning = false;   // klt_success / klt_attempted < 0.5
    bool klt_crisis = false;            // klt_success < 10 (即将丢失)
    bool grid_nonuniform = false;       // grid_uniformity < 0.3
    bool stereo_degraded = false;       // stereo_matched / (matched+failed_ncc) < 0.3
    bool r3d3d_high_reject = false;     // r3d3d_inlier_ratio < 0.3 (深度质量差)
    bool r2d2d_high_reject = false;     // r2d2d_inlier_ratio < 0.3 (跟踪质量差)
    bool too_many_aged_out = false;     // klt_aged_out > 20
    bool insufficient_features = false; // grid_counts 总计 < 50
    bool time_budget_exceeded = false;  // time_total_ms > 20
};

FrontendAlerts checkFrontendHealth(const FrontendProbe& p) {
    FrontendAlerts a;
    double track_rate = (p.klt_attempted > 0) ?
        (double)p.klt_success / p.klt_attempted : 1.0;
    a.klt_quality_warning = (track_rate < 0.5);
    a.klt_crisis = (p.klt_success < 10);
    a.grid_nonuniform = (p.grid_uniformity < 0.3);
    a.stereo_degraded = (p.stereo_matched + p.stereo_failed_ncc > 0) &&
        ((double)p.stereo_matched / (p.stereo_matched + p.stereo_failed_ncc) < 0.3);
    a.r3d3d_high_reject = (p.r3d3d_putatives > 0) &&
        (p.r3d3d_inlier_ratio < 0.3);
    a.r2d2d_high_reject = (p.r2d2d_putatives > 0) &&
        (p.r2d2d_inlier_ratio < 0.3);
    a.too_many_aged_out = (p.klt_aged_out > 20);
    a.insufficient_features = (p.klt_success < 50);
    a.time_budget_exceeded = (p.time_total_ms > 20);
    return a;
}
```

#### 可视化输出

```cpp
// 在 processFrame 末尾, 当 save_kf_images=true 时调用
cv::Mat visualizeFeatureTracks(const cv::Mat& left_gray,
    const std::vector<FeatureTrack>& tracks,
    const FrontendProbe& p) {

    cv::Mat vis; cv::cvtColor(left_gray, vis, cv::COLOR_GRAY2BGR);

    // 特征点颜色编码
    for (auto& t : tracks) {
        cv::Scalar c;
        if      (!t.has_stereo)                    c = CV_RED;    // 无深度
        else if (t.stereo_3d3d_outlier)             c = CV_YELLOW; // 3D-3D 外点(降级)
        else if (t.track_length < 3)                c = CV_BLUE;   // 新track
        else if (t.track_length > 20)               c = CV_WHITE;  // 长寿命
        else                                        c = CV_GREEN;  // 成熟track
        cv::circle(vis, cv::Point2i(t.pixel_pt.x(), t.pixel_pt.y()),
                   2, c, -1);
    }

    // 覆盖 5×7 网格线 + 每格计数
    for (int r = 0; r < p.grid_rows; r++) {
        for (int c = 0; c < p.grid_cols; c++) {
            int x0 = c * left_gray.cols / p.grid_cols;
            int y0 = r * left_gray.rows / p.grid_rows;
            int x1 = (c+1) * left_gray.cols / p.grid_cols;
            int y1 = (r+1) * left_gray.rows / p.grid_rows;
            cv::rectangle(vis, cv::Point(x0,y0), cv::Point(x1,y1),
                         CV_GRAY, 1);
            cv::putText(vis, std::to_string(p.grid_counts[r][c]),
                       cv::Point(x0+5, y0+20), 0, 0.5, CV_GRAY);
        }
    }

    // 右上角统计面板
    int y = 20;
    auto stat = [&](const std::string& s) {
        cv::putText(vis, s, cv::Point(10, y), 0, 0.4, CV_WHITE); y += 12;
    };
    stat(format("KLT: %d/%d (%.1f%%)  FBfail:%d  Age:%d  err:%.2f",
        p.klt_success, p.klt_attempted,
        100.0*p.klt_success/std::max(1,p.klt_attempted),
        p.klt_fb_failed, p.klt_aged_out, p.klt_mean_error_px));
    stat(format("2D2D RANSAC: %d/%d inl (%.1f%%, %d it)",
        p.r2d2d_inliers, p.r2d2d_putatives,
        100.0*p.r2d2d_inlier_ratio, p.r2d2d_iters));
    stat(format("3D3D RANSAC: %d/%d inl (%.1f%%, %d it)  dg:%d",
        p.r3d3d_inliers, p.r3d3d_putatives,
        100.0*p.r3d3d_inlier_ratio, p.r3d3d_iters, p.r3d3d_downgraded));
    stat(format("Stereo: %d ok  NCC:%.3f~%.3f avg:%.3f",
        p.stereo_matched, p.stereo_ncc_min, p.stereo_ncc_max, p.stereo_ncc_mean));
    stat(format("Grid: %.2f uniform  %d empty  new:%d",
        p.grid_uniformity, p.grid_empty_cells, p.new_features_detected));
    stat(format("Time: KLT:%.1f Stereo:%.1f RANSAC:%.1f Detect:%.1f Tot:%.1f",
        p.time_klt_ms, p.time_stereo_ms, p.time_ransac_ms,
        p.time_detect_ms, p.time_total_ms));

    return vis;
}

// 立体匹配可视化: 左右目连线, 红色=极线偏差>1px
cv::Mat visualizeStereoMatches(const cv::Mat& left, const cv::Mat& right,
    const std::vector<std::pair<cv::Point2f,cv::Point2f>>& matches) {

    cv::Mat vis(left.rows, left.cols*2, CV_8UC3);
    cv::cvtColor(left,  vis(cv::Rect(0,0,left.cols,left.rows)), cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, vis(cv::Rect(left.cols,0,right.cols,right.rows)), cv::COLOR_GRAY2BGR);

    for (auto& [lp, rp] : matches) {
        cv::Scalar c = (abs(lp.y - rp.y) > 1.0) ? CV_RED : CV_GREEN;
        cv::line(vis, lp, cv::Point2f(rp.x+left.cols, rp.y), c, 1);
        cv::circle(vis, lp, 2, CV_BLUE, -1);
        cv::circle(vis, cv::Point2f(rp.x+left.cols, rp.y), 2, CV_BLUE, -1);
    }
    return vis;
}
```

---

### 9.3 路标管线探针

#### 数据结构

```cpp
struct LandmarkProbe {

    // ======== 状态分布 ========
    int n_total = 0;                // 总路标数 (所有状态之和)
    int n_candidate = 0;            // CANDIDATE: 仅跟踪, 无 3D
    int n_depth_filter = 0;         // DEPTH_FILTER: 贝叶斯滤波中
    int n_smart_trial = 0;          // SMART_TRIAL: SmartFactor 隐式
    int n_promoting = 0;            // PROMOTING: 正在晋升
    int n_explicit = 0;             // EXPLICIT+STABLE: 显式 Point3
    int n_remediating = 0;          // REMEDIATING: 恢复中
    int n_marginalized = 0;         // MARGINALIZED: 已边缘化
    int n_culled = 0;               // CULLED: 已删除

    // ======== 深度滤波器 ========
    int df_converged_this_kf = 0;   // 本轮收敛数 (进入 SMART_TRIAL)
    int df_diverged_this_kf = 0;    // 本轮发散数 (→ CULLED)
    double df_mean_converge_kfs = 0;// 平均收敛所需关键帧数
    double df_median_depth_m = 0;   // 收敛路标的深度中位数

    // ======== SmartFactor 健康 (遍历所有 SmartFactor) ========
    int sf_total = 0;               // SmartFactor 总数
    int sf_valid = 0;               // point().valid() == true
    int sf_degenerate = 0;          // isDegenerate() — 视差不足
    int sf_far_point = 0;           // isFarPoint() — 超出距离阈值
    int sf_outlier = 0;             // isOutlier() — 重投影异常
    int sf_behind_camera = 0;       // isPointBehindCamera() — 🔴 严重问题!
    int sf_non_initialized = 0;     // 尚未完成首次三角化
    double sf_mean_obs = 0;         // SmartFactor 平均观测数
    int sf_max_obs = 0;             // SmartFactor 最大观测数
    double sf_mean_pixel_error = 0; // 有效 SmartFactor 的平均重投影误差 (px)
    double sf_max_pixel_error = 0;  // 有效 SmartFactor 的最大重投影误差

    // ======== 晋升统计 ========
    int promoted_this_kf = 0;       // 本轮 SMART→PROMOTING→EXPLICIT
    int promoted_total = 0;         // 历史累计晋升数
    // 10 个门控的拒绝计数 (G1~G10):
    int reject_G1_obs = 0;          // G1: 观测数 < 4
    int reject_G2_triang = 0;       // G2: triangulate() 失败
    int reject_G3_cheir = 0;        // G3: Cheirality 异常
    int reject_G4_degen = 0;        // G4: isDegenerate
    int reject_G5_far = 0;          // G5: isFarPoint
    int reject_G6_outlier = 0;      // G6: isOutlier
    int reject_G7_parallax = 0;     // G7: 视差角 < 3°
    int reject_G8_reproj = 0;       // G8: 重投影误差 > 2px
    int reject_G9_svd = 0;          // G9: SVD 条件数 > 1e6
    int reject_G10_depth = 0;       // G10: 深度 < 0.1m

    // ======== chi² 后验 ========
    int chi2_checked = 0;           // 本轮检验的显式路标数
    int chi2_passed = 0;            // chi² ≤ 7.815
    int chi2_warning = 0;           // 单次 > 7.815
    int chi2_culled = 0;            // 连续 3 次 > 7.815 → CULLED
    double chi2_mean = 0;           // 平均值
    double chi2_max = 0;            // 最大值
    int chi2_remediating = 0;       // REMEDIATING 状态路标数
};
```

#### 收集点

| 收集时机 | 填充的字段 |
|---------|-----------|
| 每 KF 的路标管线处理后 | `n_*` (状态分布), `df_*`, `promoted_*`, `reject_*`, `chi2_*` |
| 每 KF 的 SmartFactor 统计 | `sf_*` (遍历 `old_smart_factors_` 中所有 SmartFactor) |

#### SmartFactor 观测验证

```cpp
// verbosity≥2 时, 对每个活跃 SmartFactor 执行:
void verifySmartFactorObservations(
    LandmarkId lmk_id,
    const SmartStereoProjectionPoseFactor::shared_ptr& sf,
    const std::vector<FeatureTrack>& expected_tracks) {

    // 检查 1: 观测数量
    size_t actual_n = sf->keys().size();
    size_t expected_n = expected_tracks.size();
    if (actual_n != expected_n) {
        LOG(ERROR) << "[PROBE] SF lmk=" << lmk_id
                   << " obs mismatch: expected=" << expected_n
                   << " actual=" << actual_n;
        return;
    }

    // 检查 2: 每个 key 与期望的 KF ID 对应
    for (size_t i = 0; i < actual_n; i++) {
        auto sym = gtsam::Symbol(sf->keys().at(i));
        if (sym.chr() != 'x') {
            LOG(ERROR) << "[PROBE] SF lmk=" << lmk_id
                       << " key[" << i << "] not a pose: chr=" << sym.chr();
        }
        KfId actual_kf = sym.index();
        KfId expected_kf = expected_tracks[i].kf_id;
        if (actual_kf != expected_kf) {
            LOG(ERROR) << "[PROBE] SF lmk=" << lmk_id
                       << " obs[" << i << "] KF mismatch: "
                       << "expected=" << expected_kf << " actual=" << actual_kf;
        }
    }

    // 检查 3: 当前三角化有效性
    auto result = sf->point();
    if (!result) {
        LOG(WARNING) << "[PROBE] SF lmk=" << lmk_id
                     << " triangulation invalid";
    } else if (result->z() < 0) {
        LOG(ERROR) << "[PROBE] SF lmk=" << lmk_id
                   << " BEHIND CAMERA! z=" << result->z();
    }
}

// 晋升质量验证: 晋升后立即对所有历史观测帧重投影
void verifyPromotionQuality(
    LandmarkId lmk_id,
    const gtsam::Point3& promoted_pt,
    const SmartStereoProjectionPoseFactor::shared_ptr& old_sf,
    const gtsam::Values& current_estimate) {

    auto keys = old_sf->keys();
    auto meas = old_sf->measured();
    for (size_t i = 0; i < keys.size(); i++) {
        KfId kid = gtsam::Symbol(keys[i]).index();
        auto pose = current_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', kid));
        auto p_cam = T_b_cam.inverse() * pose.inverse() * promoted_pt;

        if (p_cam.z() <= 0) {
            LOG(ERROR) << "[PROBE] Promotion lmk=" << lmk_id
                       << " KF=" << kid << " BEHIND CAMERA!";
            continue;
        }

        double reproj_uL = K.fx() * p_cam.x() / p_cam.z() + K.px();
        double reproj_v  = K.fy() * p_cam.y() / p_cam.z() + K.py();
        double reproj_uR = K.fx() * (p_cam.x()-K.baseline()) / p_cam.z() + K.px();

        double err = sqrt(pow(meas[i].uL - reproj_uL, 2) +
                          pow(meas[i].v  - reproj_v,  2) +
                          pow(meas[i].uR - reproj_uR, 2));

        if (err > 5.0) {
            LOG(WARNING) << "[PROBE] Promotion lmk=" << lmk_id
                         << " KF=" << kid << " reproj=" << err << "px";
        }
    }
}
```

#### 健康检查

```cpp
struct LandmarkAlerts {
    bool sf_behind_camera = false;    // 🔴 ANY sf_behind > 0
    bool sf_high_degenerate = false;  // 🔴 (degenerate+non_init)/total > 0.5
    bool sf_high_outlier = false;     // 🟡 sf_outlier/total > 0.3
    bool promotion_stalled = false;   // 🟡 promoted_this_kf==0 for >20 KFs
    bool chi2_high_culling = false;   // 🟡 chi2_culled > 5 per KF
    bool df_high_divergence = false;  // 🟡 df_diverged > df_converged
    bool explicit_starving = false;   // 🟡 n_explicit < 20 && n_total > 200
};

LandmarkAlerts checkLandmarkHealth(const LandmarkProbe& p) {
    LandmarkAlerts a;
    a.sf_behind_camera = (p.sf_behind_camera > 0);
    a.sf_high_degenerate = (p.sf_total > 0 &&
        (double)(p.sf_degenerate + p.sf_non_initialized) / p.sf_total > 0.5);
    a.sf_high_outlier = (p.sf_total > 0 &&
        (double)p.sf_outlier / p.sf_total > 0.3);
    a.chi2_high_culling = (p.chi2_culled > 5);
    a.df_high_divergence = (p.df_diverged_this_kf > p.df_converged_this_kf);
    a.explicit_starving = (p.n_explicit < 20 && p.n_total > 200);
    return a;
}
```

---

### 9.4 后端探针

#### 数据结构

```cpp
struct BackendProbe {

    // ======== 因子注入 (每 KF) ========
    int n_sf_added = 0;              // 新 SmartFactor (首次入图, slot==-1)
    int n_sf_replaced = 0;           // 替换的 SmartFactor (clone+delete_slot+add)
    int n_sf_promoted = 0;           // SmartFactor→GenericStereoFactor 转换
    int n_imu = 0;                   // ImuFactor 数 (通常 1)
    int n_explicit_new = 0;          // 新增 GenericStereoFactor (晋升产生)
    int n_explicit_existing = 0;     // 已有显式路标的新观测
    int n_between_stereo = 0;        // BetweenFactor (立体 RANSAC, 默认关)
    int n_between_static = 0;        // BetweenFactor (静止约束, LOW_DISPARITY)
    int n_prior_pose = 0;            // PriorFactor<Pose3>
    int n_prior_vel = 0;             // PriorFactor<Vector3>
    int n_prior_bias = 0;            // PriorFactor<ConstantBias>
    int n_zupt = 0;                  // 零速先验
    int n_deleted_slots = 0;         // delete_slots 中的槽位数

    // ======== iSAM2 更新 ========
    int update_ok = 0;               // update 成功
    int update_fail = 0;             // update 失败 (任意异常)
    int exc_indeterminant = 0;       // IndeterminantLinearSystemException
    int exc_cheirality = 0;          // CheiralityException | StereoCheirality
    int exc_other = 0;               // 其他 12 种异常
    int exc_cheirality_recovers = 0; // Cheirality 异常中成功递归修复次数
    int exc_cheirality_fails = 0;    // 递归 5 次仍失败

    // ======== Motion-only BA ========
    int moba_executed = 0;           // 本轮执行次数 (有 ≥10 显式路标时 =1)
    int moba_obs_used = 0;           // 使用的显式路标观测数
    int moba_iterations = 0;         // GN 迭代次数 (固定 4)
    int moba_suspects_marked = 0;    // chi²>5.991 标记的 suspect 路标数

    // ======== 因子图大小 ========
    int graph_factors = 0;           // smoother_->getFactors().size()
    int graph_variables = 0;         // state_.size()
    int graph_explicit_lmks = 0;     // L(*) 变量数
    int graph_smart_factors = 0;     // SmartFactor 数 (遍历 dynamic_cast)

    // ======== Hessian 稀疏度 ========
    int hessian_elements = 0;        // 线性化后 Hessian 总元素数
    int hessian_zeros = 0;           // 零元素数 (abs < 1e-15)
    double hessian_sparsity = 0;     // = 1.0 - zeros/elements

    // ======== 优化前后误差 ========
    double error_before = 0;         // graph.error(state) before update
    double error_after = 0;          // graph.error(state) after update
    double error_ratio = 0;          // = error_after / error_before

    // ======== 耗时 (ms) ========
    double time_factor_build_ms = 0; // 因子组装 (SmartFactor 遍历 + 排序)
    double time_isam2_update_ms = 0; // smoother_->update()
    double time_slot_recovery_ms = 0;// updateNewSmartFactorsSlots
    double time_post_update_ms = 0;  // chi² 检查 + 状态更新
    double time_moba_ms = 0;         // Motion-only BA
    double time_total_ms = 0;
};
```

#### 收集点

| 收集时机 | 填充的字段 |
|---------|-----------|
| optimize() 阶段 1.5 后 | `moba_*`, `time_moba_ms` |
| optimize() 阶段 0 后 | `n_sf_*`, `n_imu`, `n_explicit_*`, `n_between_*`, `n_prior_*`, `n_zupt`, `n_deleted_slots` |
| optimize() 阶段 2 后 | `update_ok/fail`, `exc_*`, `graph_*`, `hessian_*`, `error_*` |
| optimize() 结束后 | `time_*` |

#### 因子注入验证

```cpp
void verifyFactorInjection(const NonlinearFactorGraph& graph_before,
    const NonlinearFactorGraph& graph_after,
    const BackendProbe& p,
    const FactorIndices& delete_slots,
    const FactorIndices& new_indices) {

    // 检查 1: 新因子总数 = 各类型之和
    int expected_new = p.n_sf_added + p.n_sf_replaced + p.n_sf_promoted
                     + p.n_imu + p.n_explicit_new + p.n_explicit_existing
                     + p.n_between_stereo + p.n_between_static
                     + p.n_prior_pose + p.n_prior_vel + p.n_prior_bias + p.n_zupt;
    int actual_new = graph_after.size() - graph_before.size() + delete_slots.size();

    if (expected_new != actual_new) {
        LOG(ERROR) << "[PROBE] Factor count mismatch: expected="
                   << expected_new << " actual=" << actual_new;
        LOG(ERROR) << "  Breakdown: sf_add=" << p.n_sf_added
                   << " sf_rep=" << p.n_sf_replaced
                   << " sf_prom=" << p.n_sf_promoted
                   << " imu=" << p.n_imu
                   << " exp_new=" << p.n_explicit_new
                   << " exp_ex=" << p.n_explicit_existing
                   << " btw_stereo=" << p.n_between_stereo
                   << " btw_static=" << p.n_between_static
                   << " prior_p=" << p.n_prior_pose
                   << " prior_v=" << p.n_prior_vel
                   << " prior_b=" << p.n_prior_bias
                   << " zupt=" << p.n_zupt;
    }

    // 检查 2: 被删的 slot 确实不在 after 图中
    for (auto& slot : delete_slots) {
        if (graph_after.exists(slot)) {
            LOG(ERROR) << "[PROBE] Deleted slot " << slot
                       << " still in graph!";
        }
    }

    // 检查 3: SmartFactor 的 slot 一致性
    // new_indices 的前 N 个必须对应 SmartFactor (SmartFactor 排在最前面)
    int sf_count = p.n_sf_added + p.n_sf_replaced;
    for (int i = 0; i < sf_count; i++) {
        auto g = graph_after.at(new_indices[i]);
        if (!dynamic_cast<const SmartStereoProjectionPoseFactor*>(g.get())) {
            LOG(ERROR) << "[PROBE] Slot " << new_indices[i]
                       << " (index " << i << ") is NOT a SmartFactor!";
        }
    }
}
```

#### Smoother 状态快照 (异常触发)

```cpp
void dumpSmootherState(const std::string& reason,
    const IncrementalFixedLagSmoother& smoother,
    const NonlinearFactorGraph& new_factors,
    const Values& new_values,
    const FactorIndices& delete_slots) {

    std::string dir = probe_config.dump_dir + "/" +
        std::to_string(timestamp_ns) + "_" + reason + "/";
    std::filesystem::create_directories(dir);

    // 1. 因子图 → DOT 格式 (可用 Graphviz 打开)
    std::ofstream dot(dir + "factor_graph.dot");
    smoother.getFactors().saveGraph(dot);

    // 2. 当前状态值 → 文本
    std::ofstream val(dir + "values.txt");
    smoother.calculateEstimate().print("Current estimate:", val);

    // 3. 新因子 + 待删除槽位
    std::ofstream nf(dir + "new_factors.txt");
    nf << "New factors: " << new_factors.size() << "\n";
    new_factors.print("", nf);
    nf << "Delete slots: ";
    for (auto& s : delete_slots) nf << s << " ";
    nf << "\n";

    // 4. 优化统计
    auto factors = smoother.getFactors();
    auto estimate = smoother.calculateEstimate();
    double error = factors.error(estimate);
    std::ofstream stats(dir + "stats.txt");
    stats << "Factors: " << factors.size() << "\n"
          << "Variables: " << estimate.size() << "\n"
          << "Error: " << error << "\n"
          << "Error/factor: " << error / std::max(1UL, factors.size()) << "\n";

    // 5. SmartFactor 详情
    int sf_count = 0;
    std::ofstream sf_f(dir + "smart_factors.txt");
    for (size_t i = 0; i < factors.size(); i++) {
        auto sf = dynamic_cast<const SmartStereoProjectionPoseFactor*>(
            factors.at(i).get());
        if (!sf) continue;
        sf_count++;
        sf_f << "SF[" << i << "]: obs=" << sf->keys().size()
             << " valid=" << sf->point().valid()
             << " degenerate=" << sf->isDegenerate()
             << " behind=" << sf->isPointBehindCamera()
             << " outlier=" << sf->isOutlier()
             << " far=" << sf->isFarPoint() << "\n";
    }
    sf_f << "Total SmartFactors: " << sf_count << "\n";

    LOG(ERROR) << "[PROBE] Smoother state dumped to " << dir
               << " (reason: " << reason << ")";
}
```

#### 健康检查

```cpp
struct BackendAlerts {
    bool isam2_failure = false;          // 🔴 update_fail > 0
    bool cheirality_recovery_fail = false; // 🔴 exc_cheirality_fails > 0
    bool indeterminant_system = false;   // 🔴 exc_indeterminant > 0
    bool high_relinearization = false;   // 🟡 (待补充)
    bool error_ratio_spike = false;      // 🟡 error_ratio > 2.0
    bool hessian_dense = false;          // 🟡 sparsity < 0.5
    bool time_budget_exceeded = false;   // 🟡 time_total_ms > 50
    bool sf_count_mismatch = false;      // 🟡 graph_smart_factors != n_sf_total
};

BackendAlerts checkBackendHealth(const BackendProbe& p) {
    BackendAlerts a;
    a.isam2_failure = (p.update_fail > 0);
    a.cheirality_recovery_fail = (p.exc_cheirality_fails > 0);
    a.indeterminant_system = (p.exc_indeterminant > 0);
    a.error_ratio_spike = (p.error_ratio > 2.0);
    a.hessian_dense = (p.hessian_sparsity < 0.5);
    a.time_budget_exceeded = (p.time_total_ms > 50);
    return a;
}
```

---

### 9.5 初始化 + 回环探针

```cpp
struct InitProbe {
    // === 状态 ===
    bool static_attempted = false;
    bool static_succeeded = false;      // 静止条件满足 + 重力估计通过
    bool dynamic_attempted = false;
    bool dynamic_succeeded = false;
    int retry_count = 0;

    // === 静止检测 (静态初始化) ===
    double accel_variance = 0;          // 加速度计方差 (m²/s⁴)
    double static_duration_s = 0;       // 已持续静止时长 (s)

    // === 重力估计 ===
    gtsam::Vector3 estimated_gravity;
    double gravity_magnitude = 0;       // |g_est|
    double gravity_magnitude_error = 0; // ||g_est| - 9.81|

    // === 偏置估计 ===
    gtsam::Vector3 estimated_bg;        // 陀螺偏置
    gtsam::Vector3 estimated_ba;        // 加计偏置
    bool bg_valid = false;              // |bg| < 0.1 rad/s
    bool ba_valid = false;              // |ba| < 0.5 m/s²

    // === 动态初始化 ===
    int kfs_collected = 0;              // 累积关键帧数
    double motion_duration_s = 0;       // 运动持续时长
    double avg_chi2_per_factor = 0;     // 初始化优化收敛指标
};

struct LoopProbe {
    // === 检测管道 ===
    int dbow_queries = 0;              // DBoW3 查询总次数
    int dbow_candidates = 0;           // 评分 > 0.015 的候选帧数
    int temporal_filtered = 0;         // 时间过滤(< 50 KF) 排除数
    int group_confirmed = 0;           // 分组一致性(≥3 连续 KF) 通过数

    // === 几何验证 ===
    int geom_verified = 0;             // PnP 验证通过数
    int geom_rejected = 0;             // PnP 验证失败数
    int pnp_matches = 0;               // 当前 PnP 匹配特征数
    int pnp_inliers = 0;               // 当前 PnP 内点数
    double pnp_mean_reproj = 0;        // PnP 平均重投影误差 (px)

    // === 回环接受/拒绝原因 ===
    int rejected_score = 0;            // BoW 评分不足
    int rejected_time = 0;             // 时间过滤
    int rejected_geometry = 0;         // PnP 验证失败
    int rejected_covisible = 0;        // 共视邻居 < 3

    // === 回环后处理 ===
    int loop_accepted = 0;             // 累计接受的回环数
    int sf_promoted_post_loop = 0;     // Post-loop SmartFactor 提升数
    int sf_failed_post_loop = 0;       // Post-loop 提升失败数
    double drift_corrected_m = 0;      // 最近回环消除的漂移量 (m)

    // === PGO 统计 ===
    int pgo_edges_total = 0;           // 位姿图中总边数
    int pgo_loop_edges = 0;            // 回环边数
};
```

---

### 9.6 JSONL 输出格式

每关键帧输出一行完整 JSON (无换行), 各模块为嵌套对象:

```json
{
  "ts": 1403636579763555584, "kf_id": 42, "is_kf": true,
  "frontend": {
    "klt": {"att": 210, "ok": 187, "fb": 3, "age": 0, "err": 0.12},
    "ransac_2d": {"put": 187, "inl": 153, "its": 15, "rate": 0.82},
    "ransac_3d": {"put": 98, "inl": 76, "dg": 22, "its": 8, "rate": 0.78},
    "stereo": {"ok": 142, "ncc_f": 12, "disp_f": 5, "oob": 3,
               "ncc_avg": 0.08, "ncc_min": 0.02, "ncc_max": 0.14},
    "rkp": {"v": 142, "nl": 0, "nr": 12, "nd": 5, "fa": 22},
    "grid": {"uni": 0.73, "empty": 2, "mean": 5.2, "std": 3.1},
    "detect": {"new": 48, "raw": 287},
    "survival": 0.89,
    "time": {"klt": 4.2, "stereo": 3.1, "ransac": 2.8, "detect": 1.5, "tot": 12.3}
  },
  "landmarks": {
    "states": {"tot": 423, "cand": 51, "df": 89, "smart": 203, "expl": 80},
    "sf": {"tot": 203, "ok": 189, "deg": 8, "far": 0, "out": 6, "behind": 0,
           "noninit": 0, "mean_obs": 5.3, "max_obs": 18,
           "mean_px": 0.8, "max_px": 2.3},
    "promote": {"kf": 3, "tot": 156,
                "rej": {"G7": 5, "G2": 2, "G8": 1}},
    "chi2": {"chk": 80, "ok": 78, "warn": 2, "cull": 0, "mean": 2.1, "max": 12.4},
    "df": {"conv": 4, "div": 1, "med_d": 3.2}
  },
  "backend": {
    "factors": {"sf_add": 180, "sf_rep": 23, "sf_prom": 3, "imu": 1,
                "exp_n": 3, "exp_x": 8, "btw_s": 0, "btw_st": 0,
                "p_p": 0, "p_v": 0, "p_b": 0, "zupt": 0, "del": 5},
    "isam2": {"ok": 1, "fail": 0, "indet": 0, "cheir": 0, "other": 0},
    "moba": {"exe": 1, "obs": 80, "iter": 4, "susp": 2},
    "graph": {"fac": 1347, "var": 352, "expl": 80, "sf": 203},
    "hessian": {"ele": 4512, "zero": 3200, "sp": 0.71},
    "error": {"bef": 521.3, "aft": 478.9, "rat": 0.92},
    "time": {"build": 5.2, "isam2": 18.3, "slot": 0.8, "post": 3.1,
             "moba": 2.1, "tot": 32.4}
  },
  "init": {"state": "INITIALIZED", "g_err": 0.02, "bg_norm": 0.003},
  "loop": {"acc": 0, "cand": 0}
}
```

---

### 9.7 使用示例与运行时集成

```cpp
// 在 processKeyframe() 末尾统一收集
void processKeyframe(const LocalGraphKeyframeInput& in) {
    // ... 正常处理 ...

    // ===== 探针收集 =====
    auto& fp = frontend_.probe();
    auto& lp = landmark_pipeline_.probe();
    auto& bp = backend_.probe();
    auto& ip = init_.probe();

    // 每 KF 输出 JSONL
    if (cfg_.probe.jsonl_per_kf) {
        writeDiagnosticsJsonl(cfg_.probe.jsonl_path,
                              timestamp_ns, kf_id, fp, lp, bp, ip,
                              loop_.probe());
    }

    // 健康检查
    auto fa = checkFrontendHealth(fp);
    auto la = checkLandmarkHealth(lp);
    auto ba = checkBackendHealth(bp);

    bool abnormal = fa.klt_crisis || la.sf_behind_camera ||
                    ba.isam2_failure || ba.cheirality_recovery_fail;

    // 异常 → 全量 dump
    if (abnormal && cfg_.probe.dump_on_factor_failure) {
        dumpSmootherState("abnormal", smoother_, new_factors,
                          new_values_, delete_slots);
        // 同时输出上一帧正常的 JSONL 作为对比
        writeDiagnosticsJsonl(cfg_.probe.jsonl_path + ".prev",
                              timestamp_ns, kf_id, fp, lp, bp, ip,
                              loop_.probe());
    }

    // 前端可视化
    if (cfg_.probe.save_kf_images && is_keyframe) {
        auto vis_features = visualizeFeatureTracks(left_img, tracks, fp);
        auto vis_stereo   = visualizeStereoMatches(left_img, right_img, matches);
        cv::imwrite(cfg_.probe.image_dir + "/features_" +
                    std::to_string(kf_id) + ".png", vis_features);
        cv::imwrite(cfg_.probe.image_dir + "/stereo_" +
                    std::to_string(kf_id) + ".png", vis_stereo);
    }

    // 周期性清空直方图避免溢出
    if (kf_id % cfg_.probe.histogram_reset_interval_kfs == 0) {
        fp.resetHistograms();
        lp.resetHistograms();
    }
}
```

---

## 十、外部参考文献

1. Dellaert & Kaess, *Factor Graphs for Robot Perception*, 2017 — https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf
2. Kaess et al., *iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree*, IJRR 2012
3. Forster et al., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, TRO 2017
4. GTSAM Official Documentation — https://borglab.github.io/gtsam/
5. Rosinol et al., *Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping*, ICRA 2020 — https://arxiv.org/abs/1910.02490
6. Qin et al., *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*, TRO 2018 — https://arxiv.org/abs/1708.03852
7. Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*, TRO 2021 — https://arxiv.org/abs/2007.11898
8. von Stumberg & Cremers, *DM-VIO: Delayed Marginalization Visual-Inertial Odometry*, RA-L 2022 — https://arxiv.org/abs/2201.04114
9. Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA 2020
10. Leutenegger et al., *OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop Closure*, 2022 — https://arxiv.org/abs/2202.09199

## 十一、相关页面

- [[stereo-vio-integrated-architecture]]
- [[设计-立体VIO前端管线]]
- [[架构-GTSAM iSAM2 双目VIO后端设计]]
- [[设计-双目VIO初始化子系统]]
- [[设计-双目VIO回环子系统]]
- [[landmark-pipeline-design]]
- [[VIO方案全景对比]]
- [[因子图vs滤波]]
- [[概念-IMU预积分]]
- [[概念-Schur补与边缘化]]
