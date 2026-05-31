---
tags: [PHAD, VIO, frontend, stereo, KLT, IMU-preintegration, keyframe, quality-gate, design]
created: 2026-05-18
updated: 2026-06-01
type: entity
sources:
  - raw/codes/VINS-Fusion/
  - raw/codes/Kimera-VIO/
  - raw/codes/open_vins/
  - raw/codes/ORB_SLAM3/
  - raw/codes/rpg_svo_pro_open/
  - raw/docs-deep-dive/vins_fusion_analysis.md
  - raw/docs-deep-dive/kimera_vio_analysis.md
  - raw/docs-deep-dive/open_vins_analysis.md
  - wiki/pitfalls/2026-05-18-phad-frontend-pitfalls.md
  - wiki/sources/2026-05-15-imu-preintegration.md
---

# 立体 VIO 前端管线设计

> PHAD SLAM 立体+IMU 前端的完整设计规格：特征检测/跟踪、立体匹配、IMU 预积分传播、
> 关键帧选择准则、与路标管线和后端的接口契约。基于 VINS-Fusion、Kimera-VIO、OpenVINS、
> ORB-SLAM3、SVO Pro 五个参考系统的实测分析。

---

## 1. 设计决策总览

### 1.1 特征策略决策：Pure KLT

**选择：纯 KLT 光流 + Shi-Tomasi 补点，关键帧才提取 ORB 描述子（用于回环）。**

**理由：**

| 维度 | KLT-only | KLT+ORB hybrid | 结论 |
|------|----------|---------------|------|
| 双目简化 | 立体提供瞬时深度，无需宽基线 ORB 匹配 | ORB 描述子对帧间跟踪冗余 | 立体消解了 ORB 的主要价值 |
| 计算量 | ~5ms/frame (KLT), ~2ms (GoodFeaturesToTrack) | ~15ms+ (全量 ORB 提取每帧) | KLT 至少快 3x |
| 回环支持 | 无描述子，无法做 Bag-of-Words | ORB 描述子天然支持 DBoW3 | 仅在关键帧提取 ORB |
| 参考先例 | VINS-Fusion (L330 行) 纯 KLT 达到 SOTA | ORB-SLAM3 用 ORB 但不做帧间 KLT | VINS-Fusion stereo+IMU ATE 0.08m |

**混合策略**：正常帧用 KLT 跟踪；关键帧提取 FAST 角点 + 计算 ORB 描述子，用于 DBoW3 回环检测（参考 Kimera-VIO `LoopClosureDetector` 在回环线程独立提取 ORB 的模式 —— `raw/codes/Kimera-VIO/include/kimera-vio/loopclosure/LoopClosureDetector.h:L419`）。

ORB 描述子提取时机：仅在 `is_keyframe == true` 时执行（参考 `raw/codes/Kimera-VIO/include/kimera-vio/loopclosure/LoopClosureDetector.h:L377-L379`）。

### 1.2 立体匹配策略：NCC 沿极线

**选择：NCC 模板匹配沿水平极线搜索（Kimera-VIO 模式）。**

| 方法 | 优点 | 缺点 | 适合 PHAD? |
|------|------|------|-----------|
| KLT 左→右跟踪 (VINS-Fusion) | 快速（复用 KLT） | 没有亚像素 NCC 精度 | ⚠ 可能 |
| NCC 沿极线 (Kimera-VIO) | 亚像素精度，明确分数门限 | 略慢，需要 stripe 搜索 | ✅ 推荐 |
| SGBM 稠密 (Kimera-VIO dense mode) | 稠密深度图 | GPU 依赖，开销大 | ❌ |

**选择 NCC 的理由**：
- 双目极线已知（rectified images）→ 搜索维度降为 1D
- NCC 相关分数提供天然的匹配质量度量（可设为 `admission_quality` 的一项输入）
- 参考 `raw/codes/Kimera-VIO/src/frontend/StereoMatcher.cpp:L283-L423`
- 模板尺寸：11×101 像素（窄高沿极线方向）

### 1.3 IMU 预积分：GTSAM PreintegratedCombinedMeasurements

**选择：GTSAM 原生 `PreintegratedCombinedMeasurements`（Forster 2015 流形预积分）。**

- 与 PHAD 的后端 GTSAM iSAM2 原生兼容
- 自动处理 bias 一阶修正（无需自写 Jacobian 传播）
- 噪声模型直接对接 IMU 数据手册参数
- 参考 `raw/codes/Kimera-VIO/src/imu-frontend/ImuFrontend.cpp` 和 [[概念-IMU预积分]]

### 1.4 关键帧选择：多条件复合决策树

**选择：参考 VINS-Fusion 的多条件复合策略（`addFeatureCheckParallax`），增强为 6 条件版。**

VINS-Fusion 原生有 5 个条件（`raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L52-L119`），我们扩展为 6 条件，增加了 **特征存活率低** 和 **最大时间间隔**：

---

## 2. 逐帧特征跟踪伪代码

### 2.1 主循环（per-frame processing loop）

```pseudo
function processFrame(stereo_image, imu_buffer, prev_state):
    // ===== PHASE 0: 时间戳与IMU窗口 =====
    t_cur = stereo_image.timestamp
    imu_window = selectImuBetween(imu_buffer, prev_state.timestamp, t_cur)
    delta_t_segments = extractTimeSegments(imu_window)  // 真实时间戳差，非常数dt

    // ===== PHASE 1: IMU预积分 =====
    pim = new PreintegratedCombinedMeasurements(params, prev_state.bias)
    for each (dt, acc, gyr) in imu_window:
        pim.integrateMeasurement(acc, gyr, dt)
    // 保留原始IMU数据用于bias更新时重传播
    imu_buf_raw.push(imu_window)

    // ===== PHASE 2: IMU预测旋转 → 用于KLT初值 =====
    predicted_rotation = pim.deltaRij()
    homography_H = K * predicted_rotation^T * K^(-1)    // 参考 Kimera OpticalFlowPredictor
    for each track in tracks_active:
        predicted_pt = project(homography_H, track.pixel_pt_prev)
        track.predicted_pt = predicted_pt

    // ===== PHASE 3: KLT跟踪（前帧→当前左目） =====
    cur_pts, status, err = calcOpticalFlowPyrLK(
        prev_left_gray, cur_left_gray,
        prev_pts,      // 上帧所有活跃特征的像素坐标
        predicted_pts, // IMU旋转预测初值
        winSize=(21,21), maxLevel=3,   // 3层金字塔保守值
        criteria=(COUNT|EPS, 30, 0.01),
        OPTFLOW_USE_INITIAL_FLOW
    )
    // 降级：如果成功跟踪 < 10个，回退到无预测3层（VINS模式）
    if countSuccess(status) < 10:
        cur_pts = calcOpticalFlowPyrLK(prev_left_gray, cur_left_gray,
                    prev_pts, winSize=(21,21), maxLevel=3, ...)  // 无USE_INITIAL_FLOW

    // ===== PHASE 4: 双向光流验证（VINS-Fusion模式） =====
    if config.forward_backward_check:
        back_pts, back_status = calcOpticalFlowPyrLK(
            cur_left_gray, prev_left_gray, cur_pts,
            initial=prev_pts, ...  // prev_pts作为初值
        )
        for i in all tracks:
            if not (status[i] and back_status[i] and
                    norm(back_pts[i] - prev_pts[i]) < 0.5):
                status[i] = false

    // ===== PHASE 5: 边界外剔除 + F矩阵RANSAC外点剔除 =====
    status = status AND inBorder(cur_pts, img_size)
    cur_un_pts = undistort(cur_pts, camera)  // 去畸变到归一化平面
    status = status AND fundamentalMatrixRANSAC(
        prev_un_pts, cur_un_pts,
        threshold=2.0/focal, confidence=0.999
    )  // 参考 OpenVINS TrackKLT: F矩阵RANSAC, 2/f阈值

    // ===== PHASE 6: 压缩存活点 =====
    tracks_alive = compactByStatus(tracks_active, status)
    for t in tracks_alive:
        t.track_length++

    // ===== PHASE 7: 补新特征 (Shi-Tomasi) =====
    n_need = max_features - |tracks_alive|
    mask = buildSpatialMask(tracks_alive, min_dist_px)
    // 优先保留长跟踪点，半径MIN_DIST内禁止新特征（VINS setMask模式）
    if n_need > 0:
        new_pts = goodFeaturesToTrack(cur_left_gray,
                    n_need, qualityLevel=0.01, minDistance=min_dist_px,
                    mask=mask)
        // 亚像素精化 (cornerSubPix, 参考 open_vins Grider_FAST)
        cornerSubPix(cur_left_gray, new_pts, winSize=(5,5), zeroZone=(-1,-1),
                     criteria=(COUNT|EPS, 20, 0.001))
        for p in new_pts:
            tracks_alive.append(FeatureTrack(
                pixel_pt=p,
                normalized_pt=undistort(p, camera),
                id=next_id++,
                track_length=1,
                has_stereo=false,
                has_3d=false
            ))

    // ===== PHASE 8: 立体匹配 =====
    if config.stereo:
        for t in tracks_alive:
            right_pt, match_score = nccStereoMatch(
                cur_left_gray, cur_right_gray, t.pixel_pt,
                camera.baseline, camera.K
            )
            if match_score < ncc_threshold:  // 0.15 (Kimera-VIO)
                t.right_pixel_pt = right_pt
                t.right_normalized_pt = undistort(right_pt, camera)
                disparity = t.pixel_pt.x - right_pt.x
                if disparity > min_disparity:  // 正视差
                    depth = camera.fx * camera.baseline / disparity
                    if min_depth < depth < max_depth:
                        t.stereo_depth = depth
                        t.has_stereo = true
                        t.stereo_match_quality = match_score

    // ===== PHASE 9: 特征速度计算（用于时间偏移因子） =====
    dt = t_cur - prev_state.timestamp
    for t in tracks_alive:
        t.velocity = (t.normalized_pt - t.prev_normalized_pt) / dt

    // ===== PHASE 10: 构建Frame输出 =====
    return FrontendFrame(
        timestamp=t_cur,
        tracks=tracks_alive,
        pim=pim,
        raw_imu=imu_buf_raw,
        n_tracked=count_if(has_stereo),
        n_total=|tracks_alive|
    )
```

### 2.2 立体匹配算法详细规格

```pseudo
function nccStereoMatch(left_img, right_img, left_pt, baseline, K):
    // 极线假设：rectified images, 同名点在同一行
    // 搜索范围：从左点左侧开始（右图点应在左边）
    
    // 1. 提取左图模板（窄长条，沿极线方向）
    tmpl_half_w = 50   // 半宽（模板总宽101px, 参考 Kimera templ_cols_=101）
    tmpl_half_h = 5    // 半高（模板总高11px,  参考 Kimera templ_rows_=11）
    template = left_img(ROI: left_pt ± (50, 5))
    
    // 2. 定义右图搜索stripe
    max_disparity_px = fx * baseline / min_depth    // e.g. 0.1m → very wide
    min_disparity_px = fx * baseline / max_depth    // e.g. 15.0m → narrow
    stripe_x_start = left_pt.x - max_disparity_px
    stripe_x_width = max_disparity_px - min_disparity_px + 2*tmpl_half_w
    stripe = right_img(ROI: [stripe_x_start, left_pt.y ± tmpl_half_h],
                        width=stripe_x_width)
    
    // 3. NCC模板匹配 (Zero-mean Normalized Cross-Correlation)
    //    使用 cv::matchTemplate(stripe, template, result, CV_TM_CCOEFF_NORMED)
    //    或 CV_TM_SQDIFF + normalize (Kimera方式)
    cv::matchTemplate(stripe, template, result, CV_TM_CCOEFF_NORMED)
    double min_val, max_val; Point min_loc, max_loc
    cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc)
    
    match_score = 1.0 - max_val                      // 0=完美匹配, 1=不相关
    right_pt = (stripe_x_start + max_loc.x + tmpl_half_w, left_pt.y)
    
    // 4. 亚像素精化（可选）
    if config.subpixel_stereo:
        cv::cornerSubPix(right_img, right_pt, (10,10), (-1,-1),
                         TermCriteria(COUNT|EPS, 40, 0.001))
    
    return right_pt, match_score

function checkStereoQuality(left_pt, right_pt, match_score, baseline, K):
    // 质量门控 (admission_quality 子项)
    checks = QualityCheck[]
    
    // Q1: NCC匹配分数
    checks.append(ncc_score >= ncc_min_score)          // ≥ 0.85 (=1-0.15)
    
    // Q2: 正视差
    disparity = left_pt.x - right_pt.x
    checks.append(disparity > min_disparity_px)         // ≥ 1.0 px
    
    // Q3: 深度范围
    depth = K.fx * baseline / disparity
    checks.append(min_depth <= depth <= max_depth)      // [0.1, 15.0] m
    
    // Q4: 深度不确定性 (近似)
    depth_uncertainty = (depth * depth) / (K.fx * baseline) * pixel_noise
    checks.append(depth_uncertainty < max_depth_uncertainty)
    
    return all(checks), checks
```

---

## 3. IMU 预积分配置

### 3.1 EuRoC ADIS16448 噪声参数

| 参数 | 符号 | 数据手册值 | 连续时间 (GTSAM) | 来源 |
|------|------|-----------|-----------------|------|
| 陀螺白噪声密度 | $\sigma_g$ | 0.0135 °/s/√Hz | **1.92 × 10⁻⁴** rad/s/√Hz | ARW: 0.66°/√hr ÷ √3600 × π/180 |
| 加速度计白噪声密度 | $\sigma_a$ | 0.23 mg/√Hz | **1.83 × 10⁻³** m/s²/√Hz | VRW: 0.11 m/s/√hr ÷ √3600 |
| 陀螺随机游走 | $\sigma_{bg}$ | 14.5 °/hr (in-run stability) | **4.0 × 10⁻⁶** rad/s²/√Hz | Kalibr 推荐值 |
| 加速度计随机游走 | $\sigma_{ba}$ | — | **2.0 × 10⁻⁴** m/s³/√Hz | Kalibr 推荐值 |
| 积分不确定性 | $\sigma_{int}$ | — | **0.0** | GTSAM 默认关闭 |

**数据手册到 GTSAM 的转换**（参考 `raw/codes/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml` 和 Kalibr IMU 噪声模型讨论 — [GitHub ethz-asl/kalibr#63](https://github.com/ethz-asl/kalibr/issues/63)）：

```python
# 连续时间噪声密度 (GTSAM PreintegrationCombinedParams)
gyro_noise_density  = 0.66 * pi/180 / sqrt(3600)  # = 1.92e-4 rad/s/√Hz
accel_noise_density = 0.11 / sqrt(3600)            # = 1.83e-3 m/s²/√Hz
gyro_random_walk    = 4.0e-6                       # rad/s²/√Hz
accel_random_walk   = 2.0e-4                       # m/s³/√Hz
```

### 3.2 GTSAM 预积分参数配置

```cpp
// 创建预积分参数
auto params = PreintegratedCombinedParams::MakeSharedU(gravity_mag);
// gravity_mag = 9.81007 (苏黎世重力，EuRoC 数据集)

// 设置IMU噪声模型
params->setAccelerometerCovariance(
    I_3x3 * pow(accel_noise_density, 2));       // σ_a² × I₃
params->setGyroscopeCovariance(
    I_3x3 * pow(gyro_noise_density, 2));         // σ_g² × I₃
params->setIntegrationCovariance(
    I_3x3 * pow(integration_uncertainty, 2));

// bias随机游走协方差
params->setBiasAccCovariance(
    I_3x3 * pow(accel_random_walk, 2));          // σ_ba² × I₃
params->setBiasOmegaCovariance(
    I_3x3 * pow(gyro_random_walk, 2));           // σ_bg² × I₃

// 使用欧拉积分 (body-frame)
params->setBodyPSensor(Pose3());                  // identity: IMU frame = body frame
```

### 3.3 Bias 更新与重传播

```pseudo
// GTSAM预积分自动处理一阶bias修正
// 使用方式:
pim.deltaXij()         → [ΔR, Δv, Δp] 当前预积分量
pim.biasHat()          → 预积分时的bias线性化点

// 当后端估计新bias后:
old_bias = pim.biasHat()
new_bias = backend_latest_bias
if norm(new_bias - old_bias).norm() > bias_reset_threshold:  // e.g. acc 0.1, gyr 0.01
    // 完全重传播 (GTSAM resetIntegrationAndSetBias)
    pim.resetIntegrationAndSetBias(new_bias)
    for each (dt, acc, gyr) in imu_window_raw:
        pim.integrateMeasurement(acc, gyr, dt)
else:
    // 一阶Taylor修正 (GTSAM自动, 调用updateBias)
    // δx_corrected = δx + J_bias * (new_bias - old_bias)
    // 协方差不变 (GTSAM内部不更新协方差的bias Jacobian → 但 CombinedImuFactor 会处理)
```

### 3.4 IMU 时间戳处理（非常数 dt）

```pseudo
// CRITICAL: 时间戳差必须从实际传感器时间戳计算，不使用常数 dt
function selectImuBetween(imu_buf, t_start, t_end):
    // imu_buf: [(timestamp_ns, acc3, gyr3), ...] 按时间戳排序
    result = []
    for (ts, acc, gyr) in imu_buf:
        if ts >= t_start and ts < t_end:
            if result is not empty:
                dt = (ts - prev_ts) * 1e-9  // 纳秒→秒
            else:
                dt = (ts - t_start) * 1e-9  // 首段: 上次图像时间到第一个IMU
            result.append((dt, acc, gyr))
            prev_ts = ts
    
    // 末尾补一段到 t_end (如果需要精确切边)
    // 参考 OpenVINS Propagator::interpolate_data
    if need_precise_boundary:
        acc_end = linearInterpolate(acc_last, acc_next, t_end)
        dt_end = (t_end - prev_ts) * 1e-9
        result.append((dt_end, acc_end, gyr_last))
    
    return result
```

---

## 4. 关键帧选择决策树

### 4.1 决策伪代码

```pseudo
function shouldInsertKeyframe(tracks, prev_keyframe, pim, config):
    // ===== 6条件决策树 =====
    
    // C1: 距上一关键帧时间过长 (强制)
    dt_since_kf = current_time - prev_keyframe.timestamp
    if dt_since_kf > config.kf_max_time_interval:          // 1.0 s
        return true, "MAX_TIME"
    
    // C2: 累积运动过大 (强制)
    delta_p = pim.deltaPij().norm()
    delta_angle = 2 * acos(min(1.0, pim.deltaRij().toQuaternion().w()))
    if delta_p > config.kf_max_translation:                // 0.5 m
        return true, "MAX_TRANSLATION"
    if delta_angle > config.kf_max_rotation:                // 15° = 0.262 rad
        return true, "MAX_ROTATION"
    
    // C3: 特征存活率过低 (强制)
    track_survival_rate = |tracks.track_length >= 3| / |tracks_total|
    if |tracks_total| < config.kf_min_features:             // 20
        return true, "LOW_FEATURES"
    if |tracks_alive| < 0.5 * prev_keyframe.n_features:     // 跟踪丢失>50%
        return true, "FEATURE_DROPOUT"
    
    // C4: 新特征比例过高 (特征退化信号)
    new_features = count_if(tracks.track_length == 1)
    if new_features > 0.5 * |tracks_total|:
        return true, "HIGH_NEW_RATIO"
    
    // C5: 平均视差超过阈值 (VINS-Fusion 主条件)
    if dt_since_kf < config.kf_min_time_interval:           // 0.2 s (防止过密)
        return false, "TOO_SOON"
    
    // 计算相对上一关键帧的归一化平面平均视差
    avg_parallax = computeMeanParallax(tracks, prev_keyframe)
    if avg_parallax >= config.kf_min_parallax:              // 10/460 ≈ 0.022 rad
        return true, "PARALLAX"
    
    // C6: 长期跟踪点不足 (VINS-Fusion条件)
    long_tracks = count_if(tracks.track_length >= 5)
    if long_tracks < config.kf_min_long_tracks:             // 30
        return true, "LOW_LONG_TRACKS"
    
    return false, "NORMAL"

function computeMeanParallax(tracks, prev_keyframe):
    // 只统计在pref_keyframe也有观测的特征
    // 参考 VINS-Fusion compensatedParallax2:
    // raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L530-L563
    parallax_sum = 0
    count = 0
    for t in tracks:
        if t.hasObservation(prev_keyframe):
            un_cur = t.normalized_pt_cur
            un_kf = t.normalized_pt_at(prev_keyframe)
            parallax = norm(un_cur - un_kf)
            parallax_sum += parallax
            count++
    if count == 0: return 0
    return parallax_sum / count
```

### 4.2 关键帧选择参数表

| 参数 | 符号 | 推荐值 | 来源参考 |
|------|------|--------|---------|
| 最大时间间隔 | `kf_max_time_interval` | **1.0 s** | Kimera-VIO 10s过于宽松; VINS 无此条件 |
| 最小时间间隔 | `kf_min_time_interval` | **0.2 s** | Kimera-VIO `min_intra_keyframe_time_ns_=0.2s` |
| 最大平移 | `kf_max_translation` | **0.5 m** | VINS-Fusion `WINDOW_SIZE`相关; Kimera 无此条件 |
| 最大旋转 | `kf_max_rotation` | **15° (0.262 rad)** | VINS-Fusion 经验值 |
| 最小特征数 | `kf_min_features` | **20** | VINS-Fusion `last_track_num < 20` |
| 最小长期跟踪数 | `kf_min_long_tracks` | **30** | VINS-Fusion `long_track_num < 40` (保守) |
| 最小视差 | `kf_min_parallax` | **10 / focal ≈ 0.022 rad** | VINS-Fusion `keyframe_parallax=10/FOCAL_LENGTH` |
| 新特征比例阈值 | — | **0.5** | VINS-Fusion `new_feature_num > 0.5*last_track_num` |
| 特征丢失比率 | — | **0.5** | PHAD 新增 (参考 pitfalls: 存活率是隐藏瓶颈) |

---

## 5. 接口契约

### 5.1 Frontend → Landmark Pipeline 接口

每个特征 track 在关键帧时打包为 `PendingLandmark` 结构，传递给路标管线（深度滤波器 → SmartFactor 试验 → GenericStereoFactor）：

```cpp
struct PendingLandmark {
    // === 标识 ===
    uint64_t track_id;               // 全局唯一特征ID（前端自增）
    uint64_t anchor_keyframe_id;     // 此路标的anchor关键帧

    // === 观测 (归一化平面坐标) ===
    Vector3 normalized_pt_left;      // 左目归一化坐标 (x, y, 1)
    Vector3 normalized_pt_right;     // 右目归一化坐标 (x, y, 1)
    // 若 has_stereo=false, right为(0,0,0)

    // === 立体质量指标 (→ admission_quality) ===
    bool has_stereo;                 // 是否有有效立体匹配
    double stereo_ncc_score;         // NCC相关分数 [0, 1] (低=好)
    double stereo_disparity;         // 像素视差
    double stereo_depth_initial;     // 初始深度估计 (m)
    double stereo_depth_uncertainty; // 深度不确定性 (m) — 从视差1px误差计算

    // === 跟踪质量指标 (→ admission_quality) ===
    int track_length;                // 连续跟踪帧数
    double last_klt_error;           // KLT跟踪误差
    double parallax_since_create;    // 创建以来的归一化平面总位移
    double ransac_inlier_ratio;      // 最近RANSAC的内点率
    bool passed_fb_check;            // 是否通过双向光流检查

    // === 3D初始化候选项 (传给深度滤波器) ===
    Vector3 pinhole_3d_camera;       // 从立体视差直接三角化的3D点（相机系）
    Pose3 T_world_camera;            // anchor关键帧的相机会员位姿

    // === 像素坐标（用于可视化/调试） ===
    Vector2 pixel_left;
    Vector2 pixel_right;
};
```

**接口语义**：
- Frontend 在关键帧时，对所有满足 `admission_quality` 最低门槛的活跃 track，打包 `PendingLandmark` 列表
- Landmark Pipeline 接收此列表，启动深度滤波器（参考 SVO Pro 的 DepthFilter → `raw/codes/rpg_svo_pro_open/svo/direct/depth_filter.cpp`）
- 深度滤波器 `updateSeed(px, depth, depth_var)` → 收敛后 → SmartFactor 试验 → 通过后 → 转为显式 GenericStereoFactor
- 未通过 admission_quality 的 track 不进入 Landmark Pipeline（节省计算）

### 5.2 Frontend → Backend 接口

每帧（不限于关键帧）的输出结构：

```cpp
struct FrontendOutput {
    // === 时间戳 ===
    double timestamp;

    // === 特征数据 ===
    vector<FeatureTrack> tracks;     // 所有活跃track的当前帧观测
    int n_stereo_tracked;            // 有立体匹配的track数
    int n_total_tracked;             // 总跟踪特征数

    // === IMU预积分 ===
    shared_ptr<PreintegratedCombinedMeasurements> pim;
    // GTSAM CombinedImuFactor可直接使用此对象
    // 包含: deltaRij(), deltaPij(), deltaVij(), deltaTij(),
    //       preintMeasCov() (9x9), biasHat()
    
    // === 原始IMU数据（用于bias变化时重传播） ===
    vector<tuple<double, Vector3, Vector3>> imu_window_raw;
    // [(dt, acc3, gyr3), ...]

    // === 关键帧标识 ===
    bool is_keyframe;
    uint64_t keyframe_id;            // 若是关键帧, 自增ID

    // === 关键帧时才有的数据 ===
    vector<PendingLandmark> pending_landmarks;  // → Landmark Pipeline
    vector<ORBDescriptor> keyframe_descriptors; // → Loop Closure (ORB on kf)
    cv::Mat keyframe_image;                     // → 回环线程

    // === 诊断信息 ===
    FrontendDiagnostics diag;
};

struct FrontendDiagnostics {
    int klt_total_attempted;      // KLT总尝试跟踪数
    int klt_success;              // KLT成功数
    int klt_fb_failed;            // 双向光流失败数
    int klt_border_failed;        // 边界外剔除数
    int ransac_inliers;           // RANSAC内点数
    int ransac_outliers;          // RANSAC外点数
    int stereo_matched;           // 立体匹配成功数
    int stereo_failed_ncc;        // NCC分数不够
    int stereo_failed_disparity;  // 视差不合法
    int new_features_detected;    // 新检测特征数
    double track_survival_rate;   // 特征存活率
    double avg_parallax;          // 平均视差
    double processing_time_ms;    // 前端处理耗时
};
```

**后端组装因子图**：
```cpp
// 后端接收 FrontendOutput后:
auto output = frontend.processFrame(...);

// 1. IMU因子
auto imu_factor = CombinedImuFactor(
    output.pim,                          // 预积分对象
    X(prev_keyframe.pose_key),           // GTSAM key: 上一关键帧位姿
    X(prev_keyframe.vel_key),            // GTSAM key: 上一关键帧速度
    X(prev_keyframe.bias_key),           // GTSAM key: 上一关键帧IMU bias
    X(cur_keyframe.pose_key),
    X(cur_keyframe.vel_key),
    X(cur_keyframe.bias_key)
);
graph.add(imu_factor);

// 2. Bias随机游走因子 (GTSAM BetweenFactor)
graph.add(BetweenFactor<imuBias::ConstantBias>(
    X(prev_keyframe.bias_key),
    X(cur_keyframe.bias_key),
    imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()),  // zero delta
    bias_noise_model                                                  // 从IMU参数构造
));

// 3. → Landmark Pipeline 处理 pending_landmarks
landmark_pipeline.process(output.pending_landmarks);
```

---

## 6. 特征质量门规格（admission_quality 替代）

### 6.1 门控流水线

```
Track创建 → 逐帧质量评分 → 关键帧时 admission_quality判定 → 进入Landmark Pipeline
               ↓ 不通过
            保持跟踪但不建图（仍是活跃track，用于位姿估计）
```

### 6.2 质量评分函数

每个 track 在关键帧时刻计算综合质量得分 `Q ∈ [0, 1]`：

```
Q(track) = Q_stereo * Q_track * Q_parallax * Q_geometric

其中每个子项 ∈ [0, 1]:
```

| 质量维度 | 计算方式 | 不合格阈值 | 来源 |
|----------|---------|-----------|------|
| **Q_stereo** | `1.0 - ncc_score` (归一化NCC，低=好) | > 0.15 (即ncc_score过高) | Kimera-VIO `tolerance_template_matching_=0.15` |
| | or 0.0 (无立体观测时为0) | | |
| **Q_track** | `min(1.0, track_length / min_track_length)` | track_length < 3 | VINS-Fusion 三角化前至少2帧；保守取3 |
| **Q_parallax** | `min(1.0, parallax_accum / min_parallax_landmark)` | < 0.005 rad (≈2.3 px @460 focal) | 防止退化三角化 |
| **Q_geometric** | `inlier_ratio` (RANSAC内点率) | < 0.7 | 一致性检查 |

### 6.3 分层准入策略

| 层级 | 名称 | 条件 | 进入的管线阶段 |
|------|------|------|--------------|
| **L0: Raw Track** | 刚检测的新特征 | track_length == 1, 无立体 | 仅位姿估计 (KLT跟踪) |
| **L1: Stereo Candidate** | 获得有效立体匹配 | has_stereo=true, disparity_valid=true | 深度滤波器初始化 (1个种子) |
| **L2: Depth Converged** | 深度滤波器收敛 | depth_variance < threshold | SmartFactor 试验 |
| **L3: Established Landmark** | SmartFactor通过 | 卡方检验 + cheirality check | GenericStereoFactor (BA) |
| **DISCARD** | 质量太低 | 任何门控不通过 | 继续跟踪但不进入后续管线 |

### 6.4 设计要点（来自踩坑经验）

**关键教训**（参考 [[2026-05-18-phad-frontend-pitfalls|phad前端踩坑]]）:

1. **chi2预过滤单独开启是灾难** (Pitfall 6)
   → 必须在 admission_quality 和后期因子剔除之间配对使用
   → 预过滤只拒绝"明显不可用"的观测，不做严格卡方筛选

2. **全局过滤总是帮V伤害MH** (Pitfall 3)
   → MH低纹理场景不能用全局激进阈值
   → 使用 per-sequence 或 per-frame 自适应阈值

3. **特征存活率是隐藏瓶颈** (Pitfall 7)
   → 核心不是 target 太高，是 track 存活率太低
   → 需要改善KLT参数或预处理, 而非降低 admission 门槛

4. **3D投影预测在路标质量差时反效** (Pitfall 4)
   → L0 级track不应该做3D投影预测, 只用IMU旋转预测 (homography)
   → 仅L2+级路标可以用3D投影优化 KLT 初值

---

## 7. 参数推荐表

### 7.1 KLT / 特征检测参数

| 参数 | 推荐值 | VINS-Fusion | Kimera-VIO | OpenVINS | 理由 |
|------|--------|-------------|------------|----------|------|
| 窗口大小 | **21×21** | 21 | 24 | 可配 | 21是广泛使用的平衡值 |
| 金字塔层数 | **3** | 1(有预测)/3(无) | 3 | 可配 | 保守选择，大运动鲁棒 |
| 最大迭代 | **30** | 30 | 30 | 30 | 三者一致 |
| 终止 EPS | **0.01** | 0.01 | 0.01 | 0.01 | 三者一致 |
| 双向光流阈值 | **0.5 px** | 0.5 | 无 | 无 | VINS独有，效果好 |
| 最大特征数 | **200** per cam | 150 | 400 | 可配 | 双目的200足够(400个等效观测) |
| Shi-Tomasi qualityLevel | **0.01** | 0.01 | 0.001(GFTT) | 自适应 | VINS的0.01已验证可用 |
| 最小间距 minDist | **15 px** | 30 | 10 | 可配 | 取中间值，兼顾覆盖和均匀 |
| RANSAC类型 | **F矩阵** | F(禁用) | E/3D-3D | F矩阵 | OpenVINS F矩阵验证简单有效 |
| RANSAC阈值(归一化面) | **2.0/focal** | — | 1e-6(内积) | 2.0/focal | OpenVINS值更直观 |

### 7.2 立体匹配参数

| 参数 | 推荐值 | Kimera-VIO | 理由 |
|------|--------|-----------|------|
| 模板宽度 | **101 px** | 101 | 沿极线方向长条 |
| 模板高度 | **11 px** | 11 | 窄高，仅极线行 |
| NCC 分数阈值 | **0.15** | 0.15 | SQDIFF normalized上限 |
| 最小深度 | **0.3 m** | 0.1 | 略保守（防近场噪声） |
| 最大深度 | **15.0 m** | 15.0 | 室内/无人机合理 |
| 亚像素精化 | **开启** | 默认关 | 提升三角化精度 |

### 7.3 IMU 预积分参数

| 参数 | 推荐值 | VINS-Fusion | 理由 |
|------|--------|-------------|------|
| 积分方法 | **GTSAM PreintegratedCombinedMeasurements** | 自写中点法 | PHAD 用 GTSAM |
| 重传播阈值 (acc bias) | **0.05 m/s²** | 0.1 (禁用) | 保守值，确保精度 |
| 重传播阈值 (gyr bias) | **0.005 rad/s** | 禁用 | 保守值 |
| 重力幅值 | **9.81007** | 9.81007 | 苏黎世值(EuRoC) |

### 7.4 关键帧参数

| 参数 | 推荐值 | 来源 |
|------|--------|------|
| 最大时间间隔 | **1.0 s** | 新增 (Kimera 10s太宽) |
| 最小时间间隔 | **0.2 s** | Kimera-VIO |
| 最大平移 | **0.5 m** | VINS-Fusion经验 |
| 最大旋转 | **15°** | VINS-Fusion经验 |
| 最小特征数 | **20** | VINS-Fusion |
| 最小长期跟踪数 | **30** | VINS-Fusion (保守) |
| 最小视差 | **10 pixel / focal** ≈ 0.022 rad | VINS-Fusion keyframe_parallax |
| 新特征比例 | **0.5** | VINS-Fusion |
| 特征丢失比例 | **0.5** | 新增 (PHAD) |

---

## 8. 失败模式与恢复策略

### 8.1 失败模式矩阵

| 模式 | 症状 | 根因 | 检测方法 | 恢复策略 |
|------|------|------|---------|---------|
| **F1: KLT跟踪崩溃** | status 成功数 < 5 | 大运动/模糊/低纹理 | 每帧检查 `countSuccess(status)` | 回退到无预测 3 层金字塔重试 → 若仍失败: 全量 Shi-Tomasi 重新检测 → 标记帧为 LOST |
| **F2: 特征存活率崩塌** | track存活率 < 0.3 | 低纹理墙面 (MH场景) | `track_survival_rate` | 降低 Shi-Tomasi qualityLevel 到 0.005; 开启 CLAHE 预处理; 减小 minDist |
| **F3: 立体匹配退化** | stereo_matched < 20 | 重复纹理/低纹理/遮挡 | `n_stereo_tracked` | 接受更多单目 track; 增加 NCC 搜索范围; ~若持续退化: 切到单目模式~ |
| **F4: RANSAC 内点不足** | inliers < 8 | 误匹配过多/小视差 | `ransac_inliers` | 收紧双向光流阈值到 0.3px; 提高 RANSAC 置信度到 0.9999 |
| **F5: IMU 数据缺失** | imu_window.empty() | 传感器断开/丢帧 | `imu_window.size() == 0` | 跳过本帧 IMU 预积分; 仅用视觉; 增大关键帧频率补偿 |
| **F6: 全部 track 丢失** | tracks_alive == 0 | 剧烈运动/遮挡/进入新场景 | `n_total_tracked == 0` | **重定位模式**: 全量 Shi-Tomasi (qualityLevel=0.005) + 降级到 5 层金字塔; 尝试 PnP match 最后的 3D 路标 |
| **F7: 时间戳异常** | dt_segments中有负值或超大值 | 传感器时钟错误 | 检查 `dt < 0 or dt > 2.0` | 丢弃异常 IMU 测量; 标记 dt_warning; 通知运维 |
| **F8: 深度滤波器不收敛** | admission_quality L2 永远达不到 | 特征运动不足/纯旋转 | L2计数百分比 < 10% | 等待更多关键帧 (至少3帧视差); 降低深度方差阈值 |

### 8.2 降级策略

```
当前端检测到失败模式时，按以下优先级降级：

Level 0 (正常): KLT + IMU预测 + 双向光流 + NCC立体
Level 1 (KLT弱): KLT + 无预测3层 + 双向光流 + NCC立体
Level 2 (跟踪丢失): 全量Shi-Tomasi重检测 + NCC立体 (保留旧track ID)
Level 3 (立体退化): KLT + 单目模式 (接受没有深度的track, 延迟三角化)
Level 4 (IMU失效): KLT + NCC立体 + 跳过预积分(纯视觉)
Level 5 (重定位): 全量ORB提取 + PnP匹配旧3D点 + DBoW3全局检索
```

### 8.3 重启逻辑

```pseudo
function handleTrackingFailure(frontend_state):
    if frontend_state.failure_mode >= LEVEL_3:
        // 连续3帧降级 → 触发重定位
        if consecutive_degraded_frames >= 3:
            // 1. 保存当前状态快照
            saveStateSnapshot()
            
            // 2. 全量ORB提取+描述子计算（本帧）
            keypoints, descriptors = ORB_detect_and_compute(current_image)
            
            // 3. 查询DBoW3数据库找候选回环帧
            candidates = bow_database.query(descriptors)
            
            // 4. 对每个候选帧做PnP+几何验证
            for candidate in candidates:
                matches = matchDescriptors(current, candidate)
                if matches < 30: continue
                success, T_wc, inliers = solvePnPRansac(
                    candidate.landmarks_3d, current_keypoints,
                    camera_matrix, threshold=10.0, confidence=0.99)
                if success and inliers > 15:
                    // 5. 重定位成功 → 更新位姿 → SE3: track_relocalized
                    setPose(T_wc)
                    // 6. 用新位姿重新做立体匹配获得深度
                    // 7. 清除旧track, 建立新track与新位姿的关联
                    frontend_state.reset()
                    return RELOCALIZED
            
            // 8. 重定位失败 → 标记系统需要重新初始化
            return NEED_REINITIALIZATION
    
    // 正常降级恢复
    frontend_state.degradation_level = min(
        frontend_state.degradation_level + 1, MAX_LEVEL)
    frontend_state.consecutive_degraded_frames++
    return DEGRADED
```

---

## 9. Agent 实现提示

### 适用场景

当 agent 需要实现 PHAD SLAM 的立体+IMU 前端时，按此规格实现：KLT 跟踪 → IMU 预积分 → 关键帧判断 → 质量门控 → 输出接口。

### 输入输出契约

- **输入**：立体灰度图对 (left, right, cv::Mat_<uint8_t>)、IMU 缓冲区 (deque<ImuMeas>)、上帧状态 (tracks_active)、相机标定 (StereoCamera)、配置文件。
- **输出**：`FrontendOutput` 结构（含 tracks、PIM、pending_landmarks、诊断信息）。
- **坐标约定**：KLT/角点使用 rectified pixel coordinates；`undistort` 后输出 normalized plane coordinates (z=1)；RANSAC 在 normalized coordinates 上运行（阈值 `2.0 / focal_length`）。

### 实现骨架（伪代码）

见 §2.1 完整伪代码。

### 关键源码片段

**KLT 入口（VINS-Fusion 参考）**:
`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L117-L140` — 有预测时用 1 层金字塔 + OPTFLOW_USE_INITIAL_FLOW；失败<10时回退 3 层。

**双向光流验证（VINS-Fusion 独有）**:
`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L135-L151` — 正向+反向 KLT + 距离 < 0.5px 筛选。

**IMU 旋转预测光流（Kimera-VIO 参考）**:
`raw/codes/Kimera-VIO/src/frontend/optical-flow/OpticalFlowPredictor.cpp:L70-L126` — H = K×R^T×K^{-1} homography 投影。

**NCC 立体匹配（Kimera-VIO 参考）**:
`raw/codes/Kimera-VIO/src/frontend/StereoMatcher.cpp:L283-L423` — searchRightKeypointEpipolar, 模板匹配 CV_TM_SQDIFF。

**关键帧决策（VINS-Fusion 参考）**:
`raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L52-L119` — addFeatureCheckParallax, 5条件复合判定。

**F矩阵 RANSAC（OpenVINS 参考）**:
`raw/codes/open_vins/ov_core/src/track/TrackKLT.cpp` — findFundamentalMat, 阈值 2.0/focal。

### 实现注意事项

1. **时间戳务必用实际传感器时间戳**，不能假设常数 dt。`selectImuBetween` 从 `[(ts_ns, acc, gyr), ...]` 中计算每段 `dt = (ts_next - ts_cur) × 1e-9`。
2. **KLT 失败点的清理时机**：在 RANSAC 之前清理边界外和 mask 冲突点；在 RANSAC 之后清理几何外点；在双向光流之后清理不一致点。三个阶段不可混序。
3. **new track 不与旧 track 共享 ID** — 前端 `next_id` 全局自增，确保每个 track 有唯一标识。
4. **PHAD 的立体简化**：已知基线 + epipolar，立体匹配只需沿水平极线搜索。不要实现通用 2D 搜索。
5. **参考 pitfalls**：admission_quality 只拒收"明显不可用"的观测（track_length<3, 无立体, RANSAC外点, 极小视差），不做严格的 chi2 预过滤 — 留给后端的 SmartFactor 和 GenericStereoFactor 做统计检验。

### 源码检索锚点

- `raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp` — KLT、反向检查、补点、去畸变、setMask
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L52-L119` — keyframe 决策 addFeatureCheckParallax
- `raw/codes/Kimera-VIO/src/frontend/StereoMatcher.cpp:L283-L483` — NCC 极线搜索 + 深度验证
- `raw/codes/Kimera-VIO/src/frontend/Tracker.cpp:L92-L211` — KLT + IMU预测 + RANSAC
- `raw/codes/Kimera-VIO/src/frontend/VisionImuFrontend.cpp:L175-L232` — 关键帧判断 shouldBeKeyframe
- `raw/codes/open_vins/ov_core/src/track/TrackKLT.cpp` — 金字塔KLT + F矩阵RANSAC
- `raw/codes/open_vins/ov_msckf/src/state/Propagator.cpp` — IMU传播（离散/RK4/ACI²）

---

## 10. 相关页面

- [[概念-SLAM]]
- [[概念-IMU预积分]]
- [[方法-视觉特征跟踪]]
- [[图像预处理与观测模型]]
- [[概念-三角化与深度估计]]
- [[概念-PnP 运动估计]]
- [[组件-GTSAM]]
- [[2026-05-18-phad-frontend-pitfalls]]
- [[VIO方案对比]]
- [[方法-层次化特征网格]]
