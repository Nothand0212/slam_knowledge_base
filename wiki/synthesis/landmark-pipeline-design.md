---
tags: [landmark, 路标, 因子图, GTSAM, VIO, PHAD, 双目, SmartFactor, PFB, 深度滤波, 边缘化]
type: synthesis
created: 2026-06-01
updated: 2026-06-01
sources:
  - raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp
  - raw/codes/rpg_svo_pro_open/svo_direct/src/depth_filter.cpp
  - raw/codes/ORB_SLAM3/src/LocalMapping.cc
  - raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp
---

# 双目 VIO 路标管线设计规范

> 从首次观测 → 深度滤波初始化 → SmartFactor 试用期 → 显式因子晋升 → 长期持久化 → 最终边缘化的完整路标生命周期设计。
> 针对 PHAD VIO 系统的 20× RMSE 退化（0.09m → 1.81m on EuRoC V1_01）进行根治性设计。

## 1. 问题根因分析

### 1.1 SmartFactor-Only 后端的致命缺陷

当 PHAD VIO 的 PFB（PriorFactor-Based）显式路标后端被删除并替换为 SmartFactor-only 后，RMSE 从 0.09m 退化至 1.81m。根因有三个层面：

| 层面 | 问题 | 后果 |
|------|------|------|
| **变量缺失** | SmartFactor 的路标 3D 位置是隐式的——每次从连接位姿重新三角化，不作为优化变量 | 位姿估计错误时，三角化退化；位姿残差不反映路标质量的下降 |
| **边缘化寄生** | 当旧帧被 ISAM2 边缘化，SmartFactor 失去连向该帧的观测，剩余约束变弱 | 三角化方程欠约束，内部 SVD 退化 |
| **无质量门控** | SmartFactor 没有"不可信"概念——即使只有 2 个退化观测也照常输出约束 | 坏路标污染优化，产生系统性偏差 |

### 1.2 设计原则

1. **路标必须作为显式变量参与优化**（当质量达标后）
2. **质量门控在转换处硬性检查**，不允许半信半疑的路标进入因子图
3. **双目基线提供绝对尺度的初始深度**，避免单目三角化的尺度漂移
4. **初始化期间（前 10 KF）所有路标保持显式**，不给 SmartFactor 退化窗口
5. **借鉴 DM-VIO 延迟边缘化思想**——路标在被边缘化前应有充分收敛机会

---

## 2. 路标状态机

```
                    ┌───────────────────────────────────────┐
                    │                                       │
                    ▼                                       │
              ┌──────────┐    首次观测       ┌──────────────┐
    新特征───▶│ CANDIDATE │──────────────▶│ DEPTH_FILTER │
              │ 仅记录   │  ≥2 帧双目观测   │ 贝叶斯深度估计 │
              └──────────┘                  └──────┬───────┘
                                                   │ depth variance 收敛
                                                   │ (σ₂ < σ₂_converged)
                                                   ▼
                                          ┌──────────────┐
                                          │  SMART_TRIAL │  ← GTSAM SmartStereoProjectionPoseFactor
                                          │  试用期      │     3-8 观测，内部三角化验证
                                          └──────┬───────┘
                                                 │
                                    ┌────────────┼────────────┐
                                    │ 通过晋升门控？            │ 未通过
                                    │ ≥N_obs && parallax > θ   │ 但仍可重试
                                    │ && reproj_err < ε        │
                                    ▼                         ▼
                           ┌──────────────┐         ┌────────────────┐
                           │  PROMOTING   │         │ SMART_RETRY   │
                           │ Smart→Explicit│         │ 等待更多观测  │
                           │ 转换中        │         └────────────────┘
                           └──────┬───────┘                 │
                                  │ 转换成功                │ 累积≥N_obs
                                  ▼                         │ 重新评估
                           ┌──────────────┐                │
                           │   EXPLICIT   │◀───────────────┘
                           │ 显式因子     │  GTSAM GenericStereoFactor<Pose3,Point3>
                           │ + Point3变量  │  + Huber loss + chi2 test
                           └──────┬───────┘
                                  │
                    ┌─────────────┼─────────────┐
                    │ chi2 异常值检测             │
                    ▼                           ▼
           ┌──────────────┐            ┌──────────────┐
           │   STABLE     │            │  REMEDIATING │
           │ 稳定持久化    │            │ 重新评估中    │
           └──────┬───────┘            └──────────────┘
                  │                           │
                  │ 被边缘化                   │ 连续K帧 chi2异常
                  ▼                           ▼
           ┌──────────────┐            ┌──────────────┐
           │ MARGINALIZED │            │   CULLED     │
           │ Schur补消除   │            │ 永久移除      │
           └──────────────┘            └──────────────┘
```

### 2.1 状态定义

| 状态 | 因子类型 | 路标变量 | 说明 |
|------|---------|---------|------|
| `CANDIDATE` | 无 | 无 | 特征跟踪中但观测不足，仅累积双目观测对 |
| `DEPTH_FILTER` | 无 | 内部 μ, σ², a, b | 贝叶斯深度滤波器运行中，每帧更新 |
| `SMART_TRIAL` | `SmartStereoProjectionPoseFactor` | 隐式（内部三角化） | 被加入因子图，但不创建 Point3 变量 |
| `PROMOTING` | 转换中 | 过渡态 | SmartFactor → Explicit，仅持续一帧 |
| `EXPLICIT` | `GenericStereoFactor<Pose3, Point3>` | `Point3(lmk_id)` | 显式变量参与优化 |
| `STABLE` | `GenericStereoFactor<Pose3, Point3>` | `Point3(lmk_id)` | 通过 chi2 验证，长期稳定 |
| `REMEDIATING` | `GenericStereoFactor<Pose3, Point3>` | `Point3(lmk_id)` | chi2 超标，重新评估 |
| `MARGINALIZED` | `LinearContainerFactor`（先验） | 被 Schur 补消除 | 从图中移除，信息压缩为先验 |
| `CULLED` | 无 | 无 | 永久删除 |

### 2.2 状态转换与门控

```
CANDIDATE → DEPTH_FILTER:
  - 触发: track.obs.size() >= 2 帧双目观测
  - 动作: 初始化贝叶斯深度滤波器 (μ=1/depth_mean, σ²=depth_range²/36, a=b=10)

DEPTH_FILTER → SMART_TRIAL:
  - 触发: isConverged() → σ² < (mu_range / 200)²
  - 动作: 创建 SmartStereoProjectionPoseFactor，加入因子图
  - 并行: 深度滤波器继续更新，提供备用深度值

SMART_TRIAL → PROMOTING:
  - 触发: ALL of:
    (1) factor->measured().size() >= min_obs_for_promotion (4-6)
    (2) 最大视差角 > min_parallax_deg (3°)
    (3) factor->point().valid() && !isDegenerate() && !isFarPoint() && !isOutlier()
    (4) 重投影误差均值 < max_mean_reproj_px (2.0 px)
    (5) 三角化方差 < max_triangulation_var (0.01 m², 仅逆深度参数化)
  - 来源: Kimera-VIO RegularVioBackend::updateLmkIdIsSmart (L870-L957)

SMART_TRIAL → SMART_RETRY:
  - 触发: promotion 门控未通过，但观测数 < max_retry_obs (10)
  - 动作: 保持 SmartFactor，等待更多观测后重新评估

SMART_RETRY → PROMOTING:
  - 同 SMART_TRIAL → PROMOTING 条件

SMART_TRIAL / SMART_RETRY → CULLED:
  - 触发: 连续 missed > max_consecutive_missed (5)，或 track 断裂 > max_track_gap_frames

PROMOTING → EXPLICIT:
  - 触发: convertSmartToProjectionFactor() 成功
  - 动作: (1) 从 SmartFactor 提取 Point3 初值 (2) 创建 Point3 变量加入 Values
         (3) 为所有历史观测创建 GenericStereoFactor (4) 删除 SmartFactor
  - 来源: Kimera-VIO RegularVioBackend::convertSmartToProjectionFactor (L635-L730)

PROMOTING → SMART_RETRY:
  - 触发: 转换失败（Point3 无效、观测不足等）
  - 回退: 保留 SmartFactor

EXPLICIT → STABLE:
  - 触发: 连续 K（3）次优化后 chi2 < chi2_threshold (5.991, 95% quantile, dof=2)
  - 动作: 标记为 stable，降低后续 chi2 检查频率

EXPLICIT → REMEDIATING:
  - 触发: 连续 K（2）次优化后 chi2 > chi2_threshold
  - 动作: 大幅增加 Huber loss 的 k 值（等同于降低权重），等待重新验证

REMEDIATING → STABLE:
  - 触发: 连续 K（2）次优化后 chi2 < chi2_threshold
  - 动作: 恢复正常权重

REMEDIATING → CULLED:
  - 触发: 连续 K（5）次优化后仍 chi2 > chi2_threshold
  - 动作: 永久移除路标及所有关联因子

STABLE → MARGINALIZED:
  - 触发: 路标的 host KF 即将被 ISAM2 边缘化
  - 动作: Schur 补消除 Point3 变量，信息压缩为 LinearContainerFactor 先验

任何状态 → CULLED（初始化期间例外）:
  - 触发: track 完全丢失（所有观测帧被边缘化，无再捕获可能）
```

---

## 3. 深度滤波器设计（借鉴 SVO Pro）

### 3.1 贝叶斯模型

采用 Vogiatzis-Hernández (2011) 的高斯+均匀混合模型，每路标维护 4 维状态：

| 参数 | 符号 | 含义 | 初始值 |
|------|------|------|--------|
| 均值 | μ | 逆深度均值 | 1/depth_mean（来自双目匹配） |
| 方差 | σ² | 逆深度方差 | (mu_range)²/36 |
| Beta α | a | inlier 计数 | 10 |
| Beta β | b | outlier 计数 | 10 |

### 3.2 双目初始深度确定

```
对每一对立体匹配 (uL, uR, v):
    disparity = |uL - uR|          (像素)
    如果 disparity < min_disparity (1.0 px):
        标记为"深度不确定"，使用 scene_depth_mean 回退
    否则:
        depth = f * b / disparity  (米)
        depth_uncertainty = Z² / (f * b) * σ_disparity

场景统计:
    depth_mean = median(all_valid_depths)
    depth_min  = 0.1 * depth_mean  (clip at 0.3m)
    depth_max  = 10.0 * depth_mean (clip at 50m)
    mu_range   = 1/depth_min       (逆深度参数化)
```

### 3.3 更新方程

每帧对每个路标在极线上搜索最佳匹配后：

```
Step 1 - 极线搜索得到测量 z（深度，米）
Step 2 - 计算测量不确定性 τ² = computeTau(pose, bearing, z, px_error_angle)
Step 3 - Vogiatzis 更新:
    s² = 1 / (1/σ² + 1/τ²)        # 高斯乘积方差
    m  = s² * (μ/σ² + z⁻¹/τ²)     # 注意：z⁻¹ 为逆深度
    C1 = (a/(a+b)) * normPdf(μ, σ²+τ²)
    C2 = (b/(a+b)) * (1/mu_range)
    归一化 C1, C2 为后验概率
    μ_new = C1*m + C2*μ
    σ²_new = C1*(s²+m²) + C2*(σ²+μ²) - μ_new²
    a_new = ..., b_new = ...  # Beta-Bernoulli 矩匹配更新

Step 4 - 异常值处理:
    如果极线搜索失败 → b += 1（增加 outlier 权重）
    如果 μ < 0 → 标记为 CULLED
```

### 3.4 收敛判定

```python
def is_converged(state):
    # 逆深度方差小于阈值 → 深度估计稳定
    sigma2_thresh = (mu_range / CONVERGENCE_SIGMA2_THRESH)**2
    return state.sigma2 < sigma2_thresh

# 双级阈值（借鉴 SVO Pro）:
# 普通种子: CONVERGENCE_SIGMA2_THRESH = 200
# 高质量（用于持久化）: CONVERGENCE_SIGMA2_THRESH = 500
```

---

## 4. SmartFactor 试用期 → 显式因子晋升

### 4.1 晋升门控条件表

| # | 条件 | 阈值 | 依据 |
|---|------|------|------|
| C1 | 最小观测数 | ≥ 4 (min_obs_for_promotion) | Kimera-VIO Default (L26) |
| C2 | 最大视差角 | > 3° (min_parallax_deg) | VINS-Fusion triangulation threshold (0.5° for 2-view, 3° conservative) |
| C3 | SmartFactor Point3 有效 | CHECK(!isDegenerate, !isFarPoint, !isOutlier, !isPointBehindCamera) | Kimera-VIO isSmartFactor3dPointGood (L963-L976) |
| C4 | 平均重投影误差 | < 2.0 px (max_mean_reproj_px) | ORB-SLAM3 标准：pixel noise σ=1px → 3σ≈3px; 2px conservative |
| C5 | 三角化数值稳定性 | SVD 条件数 < 1e6 | 物理：条件数过大表示近乎退化 |
| C6 | 深度 > 最小深度 | > 0.3 m (min_valid_depth) | 物理：相机最小对焦距离 |
| C7 | 深度 < 最大深度 | < 50 m (max_valid_depth_smart) | 物理：b=12cm, f=400px → disparity=1px → Z≈50m |

### 4.2 晋升伪代码

```python
# 严格遵循 Kimera-VIO RegularVioBackend 的 convertSmartToProjectionFactor 模式
# raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L635-L730

def promote_landmark(lmk_id, smart_factor, graph_state):
    """SmartFactor → Explicit GenericStereoFactor + Point3 variable"""
    
    # 1. 提取 SmartFactor 当前的 3D 估计
    point3 = smart_factor.point()
    if not point3 or not point3.valid():
        return Failure("SmartFactor point3 invalid")
    
    # 2. 门控检查
    if not check_promotion_gates(smart_factor):
        return Failure("Promotion gates not met")
    
    # 3. 创建 Point3 优化变量
    lmk_key = Symbol('l', lmk_id)
    graph_state.new_values.insert(lmk_key, point3)
    
    # 4. 为每个历史观测创建 GenericStereoFactor
    for i, (pose_key, stereo_point2) in enumerate(zip(smart_factor.keys(), smart_factor.measured())):
        if is_valid_stereo(stereo_point2):  # uR != NaN, parallax > 0, parallax < max_parallax
            factor = GenericStereoFactor_Pose3_Point3(
                stereo_point2,
                stereo_noise,         # Isotropic | 2x2 noise model, σ=1.5px
                pose_key,
                lmk_key,
                stereo_calibration,
                true,                 # throwCheirality
                true,                 # verboseCheirality  
                body_Pose_leftCamRect
            )
            graph_state.new_factors.push_back(factor)
    
    # 5. 使用 Cauchy loss (robust) — 初始晋升阶段需要鲁棒损失
    for factor in factors_for_this_lmk:
        factor.set_loss(CauchyLoss(k=3.0))
    
    # 6. 标记 SmartFactor 待删除
    if smart_factor.slot_in_graph != -1:
        graph_state.delete_slots.push_back(smart_factor.slot_in_graph)
    graph_state.old_smart_factors.erase(lmk_id)
    
    return Success(point3, len(factors_for_this_lmk))
```

### 4.3 晋升后的本地 BA（恢复 PFB 的 local_ba 模式）

晋升后立即在新路标周围运行一次 mini-BA（借鉴 ORB-SLAM3 Local BA 和原 PFB 的 local_ba_pattern）：

```python
def local_ba_after_promotion(lmk_id, graph_state, max_connected_poses=10):
    """在晋升后对新路标+其共视关键帧运行 mini-BA，剔除 outlier"""
    
    lmk_key = Symbol('l', lmk_id)
    connected_poses = get_connected_poses(lmk_key, max_connected_poses)
    
    # 构造局部因子图
    local_graph = NonlinearFactorGraph()
    for pose in connected_poses:
        local_graph.push_back(get_prior_on_pose(pose, graph_state.current_values))
        local_graph.push_back(get_imu_factors(pose, graph_state))
        for factor in get_stereo_factors(pose, lmk_key):
            local_graph.push_back(factor)
    
    # 固定位姿，只优化路标位置（仅第一轮）
    local_values = Values()
    local_values.insert(lmk_key, graph_state.current_values.at(lmk_key))
    for pose in connected_poses:
        local_values.insert(pose, graph_state.current_values.at(pose))
    
    # Levenberg-Marquardt 优化
    params = LevenbergMarquardtParams()
    params.setMaxIterations(10)
    result = LevenbergMarquardtOptimizer(local_graph, local_values, params).optimize()
    
    # Chi2 验证
    chi2 = compute_chi2(local_graph, result, lmk_key)
    if chi2 > CHI2_95_DOF2:  # 5.991
        # 标记 outlier observations
        mark_outlier_observations(lmk_id, local_graph, result)
    
    # 更新全局状态
    graph_state.current_values.update(lmk_key, result.at(lmk_key))
    
    return chi2
```

---

## 5. 后优化异常值剔除（Chi2-Based）

### 5.1 单路标 Chi2 测试

每次 ISAM2 optimize 完成后，对所有 EXPLICIT 路标运行：

```python
def post_update_outlier_check(graph, result, landmark_states):
    """借鉴 PFB 的 local_ba_pattern 和 ORB-SLAM3 MapPointCulling"""
    
    for lmk_id, state in landmark_states.items():
        if state.status not in [EXPLICIT, STABLE, REMEDIATING]:
            continue
        
        lmk_key = Symbol('l', lmk_id)
        point3 = result.at(lmk_key)
        
        # 1. 检查深度合理性
        if point3.z() < MIN_VALID_DEPTH or point3.z() > MAX_VALID_DEPTH:
            mark_for_remediation(lmk_id, "depth out of range")
            continue
        
        # 2. 计算该路标所有 stereo 因子的 Chi2 总和
        total_chi2 = 0.0
        total_dof = 0
        for factor in get_stereo_factors_for_landmark(lmk_id):
            error = factor.unwhitenedError(result)
            noise = factor.noiseModel()
            whitened = noise.whiten(error)
            chi2_i = whitened.squaredNorm()  # dof=2 for stereo
            total_chi2 += chi2_i
            total_dof += 2
        
        # 3. 每观测平均 chi2
        avg_chi2_per_obs = total_chi2 / (total_dof / 2) if total_dof > 0 else 999
        
        # 4. 状态机转换
        if avg_chi2_per_obs > CHI2_95_DOF2:  # 5.991
            state.consecutive_bad_chi2 += 1
            state.consecutive_good_chi2 = 0
        else:
            state.consecutive_good_chi2 += 1
            state.consecutive_bad_chi2 = 0
        
        # 状态转换
        if state.status == EXPLICIT:
            if state.consecutive_good_chi2 >= 3:
                state.status = STABLE
                reduce_huber_loss(lmk_id)  # 降低 Huber k，增加权重
            elif state.consecutive_bad_chi2 >= 2:
                state.status = REMEDIATING
                increase_huber_loss(lmk_id)  # 提高 Huber k，降低权重
        
        elif state.status == STABLE:
            if state.consecutive_bad_chi2 >= 2:
                state.status = REMEDIATING
                increase_huber_loss(lmk_id)
        
        elif state.status == REMEDIATING:
            if state.consecutive_good_chi2 >= 2:
                state.status = STABLE
                reduce_huber_loss(lmk_id)
            elif state.consecutive_bad_chi2 >= 5:
                state.status = CULLED
                cull_landmark(lmk_id)
        
        # 5. found_ratio 检查（借鉴 ORB-SLAM3）
        found_ratio = state.num_found / max(state.num_visible, 1)
        if found_ratio < 0.25:  # ORB-SLAM3 MapPointCulling L367
            state.status = CULLED
            cull_landmark(lmk_id)
```

### 5.2 Huber Loss 自适应策略

| 阶段 | Huber k 值 | 效果 |
|------|-----------|------|
| EXPLICIT（刚晋升） | k=3.0 | 强鲁棒，给新路标收敛空间 |
| STABLE（已验证） | k=1.345（标准 95% 效率） | 正常权重，最大化信息 |
| REMEDIATING（重新评估） | k=0.5 | 极低权重，对优化几乎无影响 |
| 初始化期间（前 10 KF） | k=3.0 | 初始化阶段状态不确定性高 |

---

## 6. 因子类型表

| 阶段 | GTSAM 因子类 | 噪声模型 | 损失函数 | 路标变量 |
|------|-------------|---------|---------|---------|
| CANDIDATE | 无 | — | — | 无 |
| DEPTH_FILTER | 无（纯滤波器） | — | — | 内部 μ, σ², a, b |
| SMART_TRIAL | `SmartStereoProjectionPoseFactor` | `Isotropic::Sigma(2, 1.5)` | 无（内部 outlier rejection） | 隐式（内部三角化） |
| PROMOTING | 过渡态 | — | — | 临时 Point3 |
| EXPLICIT | `GenericStereoFactor<Pose3, Point3>` | `Isotropic::Sigma(2, 1.5)` | `Huber(k=3.0)` | `Symbol('l', lmk_id)` → `Point3` |
| STABLE | `GenericStereoFactor<Pose3, Point3>` | `Isotropic::Sigma(2, 1.5)` | `Huber(k=1.345)` | `Symbol('l', lmk_id)` → `Point3` |
| REMEDIATING | `GenericStereoFactor<Pose3, Point3>` | `Isotropic::Sigma(2, 1.5)` | `Huber(k=0.5)` | `Symbol('l', lmk_id)` → `Point3` |
| MARGINALIZED | `LinearContainerFactor` (HessianFactor) | N/A（已线性化） | 无 | 被 Schur 补消除 |
| CULLED | 无 | — | — | 无 |

### 6.1 噪声模型详述

```
stereo_noise = noiseModel::Isotropic::Sigma(2, 1.5)
# 2 维（水平 uL-uR 视差 + 垂直 v 坐标）
# σ = 1.5 px ⟹ 约 95% 的观测落在 ±3 px 内
# 对应到深度空间：σ_Δz ≈ Z²/(f·b) · σ_disparity
#   Z=5m, f=400px, b=0.12m → σ_Δz ≈ 25²/(400·0.12)·1.5 ≈ 2.0 px → σ_Δz ≈ 0.16m
#   Z=20m → σ_Δz ≈ 1.25m
```

### 6.2 初始化期间特殊处理

```python
# 前 10 个 KF 期间
if num_keyframes <= INITIALIZATION_KF_COUNT:  # 10
    # 所有路标直接晋升为 EXPLICIT（跳过 SMART_TRIAL）
    # 使用较大的 Huber k=3.0 为初始化不确定性留余地
    # 同一次优化中同时优化位姿和路标
    bypass_smart_factor = True
    default_huber_k = 3.0
```

理由：初始化期间位姿不确定性大，SmartFactor 的隐式三角化对位姿误差敏感。直接作为显式变量参与优化，让 ISAM2 同时优化位姿和路标，而非依赖不稳定的内部三角化。

---

## 7. 参数推荐表

| 参数 | 建议值 | 单位 | 依据来源 | 可调范围 |
|------|--------|------|----------|---------|
| **深度滤波** | | | | |
| `CONVERGENCE_SIGMA2_THRESH` | 200 | — | SVO Pro ✓ | 100-500 |
| `INIT_BETA_A` | 10 | — | SVO Pro ✓ | 5-20 |
| `INIT_BETA_B` | 10 | — | SVO Pro ✓ | 5-20 |
| `PX_ERROR_ANGLE` | 1.0 | px | SVO Pro ✓ | 0.5-2.0 |
| `MIN_DISPARITY` | 1.0 | px | 物理：子像素精度 | 0.5-2.0 |
| **SmartFactor 试用** | | | | |
| `MIN_OBS_FOR_SMART` | 2 | obs | Kimera-VIO ✓ | 2-3 |
| `MIN_OBS_FOR_PROMOTION` | 4 | obs | Kimera-VIO ✓ | 3-6 |
| `MIN_PARALLAX_DEG` | 3.0 | ° | VINS-Fusion (0.5°) + conservative margin | 1.5-5.0 |
| `MAX_MEAN_REPROJ_PX` | 2.0 | px | 物理：1.5σ noise + 0.5px margin | 1.5-3.0 |
| `MAX_TRIANGULATION_VAR` | 0.01 | m² | 经验初始值，对应 σ≈10cm at 5m | 0.001-0.05 |
| `MIN_VALID_DEPTH` | 0.3 | m | 物理：相机最小对焦距离 | 0.2-0.5 |
| `MAX_VALID_DEPTH_SMART` | 50.0 | m | 物理：b=12cm, f=400px → d=1px → Z≈50m | 30-80 |
| `MAX_PARALLAX_PX` | 150 | px | Kimera-VIO ✓ | 100-200 |
| `MAX_RETRY_OBS` | 10 | obs | 经验：超过此数不再等 | 8-15 |
| **显式因子** | | | | |
| `STEREO_NOISE_SIGMA` | 1.5 | px | PHYSICAL: pixel accuracy | 1.0-2.0 |
| `HUBER_K_EXPLICIT` | 3.0 | — | 经验：初始晋升弱鲁棒 | 2.0-5.0 |
| `HUBER_K_STABLE` | 1.345 | — | 统计：95% efficiency of normal | 1.0-2.0 |
| `HUBER_K_REMEDIATING` | 0.5 | — | 经验：极低影响 | 0.3-0.8 |
| **Chi2 监测** | | | | |
| `CHI2_95_DOF2` | 5.991 | — | 统计：χ²(2, 0.95) | — |
| `CONSECUTIVE_GOOD_FOR_STABLE` | 3 | frames | 经验：3 帧连续良好 | 2-5 |
| `CONSECUTIVE_BAD_FOR_REMEDIATE` | 2 | frames | 经验：2 帧连续异常 | 1-3 |
| `FOUND_RATIO_THRESH` | 0.25 | — | ORB-SLAM3 ✓ | 0.15-0.35 |
| **边缘化** | | | | |
| `MIN_OBS_BEFORE_MARG` | 5 | obs | 经验：路标被边缘化前的最少观测数 | 5-10 |
| `DELAY_MARG_FRAMES` | 3 | frames | DM-VIO思想：给新路标额外收敛时间 | 2-5 |
| **初始化** | | | | |
| `INITIALIZATION_KF_COUNT` | 10 | KFs | 经验：初始化阶段长度 | 5-15 |

---

## 8. 接口契约

### 8.1 Frontend → Landmark Pipeline

```
输入合约:
  FeatureTrack {
    LandmarkId track_id;                    # 持久 ID
    vector<pair<FrameId, StereoPoint2>> obs; # 每帧的双目观测 (uL, uR, v)
    FrameId first_frame;                    # 首次观测帧
    FrameId last_frame;                      # 最后观测帧
    int consecutive_missed;                  # 连续丢失帧数
    bool is_stereo;                          # 是否双目观测（uR != NaN）
  }

前置条件:
  - obs.size() >= 1
  - 每个 StereoPoint2 的 uL, uR, v 均在图像范围内
  - 左右相机外参已标定（baseline, rectification matrices）
  - 每帧的 Pose3 在 ISAM2 Values 中可用（Symbol('x', frame_id)）

输出合约（给 Backend）:
  LandmarkInjection {
    LandmarkId id;
    LandmarkStatus status;  # CANDIDATE | SMART_TRIAL | EXPLICIT | ...
    oneof factor_config:
      SmartFactorConfig;     # 用于 SmartStereoProjectionPoseFactor
      ExplicitFactorConfig;  # 用于 GenericStereoFactor + Point3
      None;                  # CANDIDATE / CULLED
  }
```

### 8.2 Backend → Landmark Pipeline

```
输入合约（每轮优化后）:
  OptimizationResult {
    Values optimized_values;              # ISAM2 优化后的所有变量值
    NonlinearFactorGraph current_graph;    # 当前因子图
    set<Key> marginalized_keys;           # 本轮被边缘化的 Key
    double total_error;                    # 优化总误差
  }

查询合约:
  get_pose(frame_id) → Pose3             # 获取当前位姿估计
  get_pose_covariance(frame_id) → Matrix6 # 获取位姿协方差
  is_frame_marginalized(frame_id) → bool  # 查询帧是否已被边缘化

输出合约（修改因子图）:
  LandmarkGraphMutation {
    vector<FactorInsertion> factors_to_add;     # 新因子
    vector<Key> variables_to_add;               # 新变量 (Point3)
    vector<Values> initial_values;               # 变量初值
    vector<FactorIndex> factors_to_remove;      # 待删除因子的 slot
    vector<Key> variables_to_remove;            # 待删除变量
    vector<Key> variables_to_marginalize;       # 待边缘化变量
  }
```

### 8.3 与其他模块的交互

| 模块 | 交互方向 | 交互内容 |
|------|---------|---------|
| **Frontend (FeatureTracker)** | → Landmark | 提供 FeatureTrack（观测序列） |
| **Backend (ISAM2)** | ← Landmark | 接收因子图修改指令 |
| **Backend (ISAM2)** | → Landmark | 优化后的 Values 和边缘化事件 |
| **KeyframeSelector** | → Landmark | 通知新关键帧到达 + KF 参数 |
| **StereoMatcher** | → Landmark | 提供每帧的双目视差和匹配质量 |
| **Marginalization module** | ↔ Landmark | 协调路标边缘化时机 |

---

## 9. 明确文档化的失败模式与恢复策略

### 9.1 失败模式清单

| # | 失败模式 | 症状 | 发生条件 | 恢复策略 |
|---|---------|------|---------|---------|
| F1 | **深度滤波器不收敛** | σ² 长期不缩小 | 特征纹理弱、极线搜索持续失败 | b 递增 → outlier → 最终 CULLED；如果场景整体不收敛，降低 `CONVERGENCE_SIGMA2_THRESH` |
| F2 | **SmartFactor 退化三角化** | `isDegenerate() == true` | 视差角太小（< 1°），或位姿估计误差大 | 保持 SMART_RETRY，等待更多观测增加视差角 |
| F3 | **晋升后立即成为异常值** | 晋升后第一轮 chi2 超标 | SmartFactor 三角化初值不准，或被噪声观测误导 | 触发 REMEDIATING：Huber k=0.5，允许几帧重新收敛 |
| F4 | **稳定路标突然退化** | STABLE → chi2 连续超标 | 场景变化、光照变化、动态物体遮挡 | REMEDIATING → CULLED；如果是全局现象，检查前端特征跟踪 |
| F5 | **边缘化删除有效路标** | 边缘化后优化残差增大 | 路标被过早边缘化，信息压缩不完整 | 增加 `MIN_OBS_BEFORE_MARG` 和 `DELAY_MARG_FRAMES` |
| F6 | **双目视差异常（误匹配）** | 视差值为负或极小/极大 | 重复纹理、镜面反射、遮挡 | 晋升门控 C8（parallax > 0 && < max_parallax_px）兜底 |
| F7 | **深度滤波器数值溢出** | μ 变为 NaN 或 inf | 逆深度参数化下远点数值不稳定 | 深度 > MAX_VALID_DEPTH 时设为固定远距离值，标记为 outlier |
| F8 | **初始化期路标爆炸** | 前 10 KF 路标数失控 | 初始化期所有路标 bypass SmartFactor | 初始化期仍然运行深度滤波器收敛检查，只晋升收敛的种子 |
| F9 | **iSAM2 变量泄漏** | 被 CULLED 的 Point3 仍留在 Values 中 | 未正确调用 `graph_state.delete_variables` | 实现 CULLED 状态机的析构清理逻辑 |
| F10 | **SmartFactor 内部离群点累积** | 内部 outlier 标志反复切换 | 边缘化帧导致观测被移除，剩余观测含噪声 | 限制 SmartFactor 最小观测数 ≥ 3，低于此数时转为 SMART_RETRY |

### 9.2 全局健康监控

后台线程周期性检查：

```python
def global_health_monitor(landmark_states, optimization_history):
    stats = {
        'total_landmarks': len(landmark_states),
        'by_status': Counter(state.status for state in landmark_states.values()),
        'promotion_rate': num_promoted_per_kf,
        'cull_rate': num_culled_per_kf,
        'mean_depth_uncertainty': mean(state.sigma2 for state in depth_filter_states),
        'mean_chi2_stable': mean(chi2 for lmk in stable_landmarks),
    }
    
    # 告警条件
    if stats['by_status'].get(CULLED, 0) / stats['total_landmarks'] > 0.5:
        warn("High culling rate - check frontend or depth filter params")
    if stats['promotion_rate'] < 2.0:  # per KF
        warn("Low promotion rate - depth filter may be too strict")
    if len(stable_landmarks) < 20:
        warn("Too few stable landmarks - risk of optimization degeneration")
```

---

## 10. 实现顺序与集成检查点

| 阶段 | 功能 | 验证方式 |
|------|------|---------|
| 1 | 深度滤波器（贝叶斯更新+收敛判定） | 单元测试：已知深度真值的模拟极线匹配 |
| 2 | SmartFactor 创建 + 更新（复用现有代码） | 已有 Kimera-VIO 测试覆盖 |
| 3 | 晋升门控 + `convertSmartToProjectionFactor` | 含 ground truth 的数据集上对比晋升前后的路标深度误差 |
| 4 | Chi2 监测 + EXPLICIT/STABLE/REMEDIATING 状态机 | 单序列上验证状态转换逻辑 |
| 5 | 边缘化协调（延迟边缘化 + Schur 补） | 验证边缘化后剩余路标的 chi2 不恶化 |
| 6 | 初始化特殊处理（前 10 KF 全显式） | EuRoC V1_01 对比 SmartFactor-only vs 新管线 |
| 7 | 全局健康监控 + 参数自适应 | 16 序列全量回归测试 |

---

## 11. 参考文献与源码锚点

| 参考系统 | 关键文件 | 借鉴的机制 |
|---------|---------|-----------|
| Kimera-VIO | `RegularVioBackend.cpp:L478-L504, L635-L730, L825-L865, L870-L957, L963-L976` | SmartFactor→Explicit 晋升全流程 |
| SVO Pro | `depth_filter.cpp:L255-L365, L367-L499, L501-L552, L580-L596` | 贝叶斯深度滤波器 |
| ORB-SLAM3 | `LocalMapping.cc:L346-L385`, `MapPoint.cc:L216-L239` | MapPointCulling, found_ratio, mbBad |
| DM-VIO | `DelayedMarginalization.h:L35-L167`, `Marginalization.cpp:L30-L180` | 延迟边缘化, Schur 补实现 |
| VINS-Fusion | `feature_manager.cpp:L389-L428` | DLT 三角化, 视差角阈值 |
| OKVIS2 | arXiv:2202.09199 | 可逆边缘化：Schur 补将路标压缩为位姿图边（H* 信息矩阵），回环时逆向复活为完整观测 |

---

## 附录 A: OKVIS2 可逆边缘化补充（待集成）

OKVIS2 (Leutenegger, 2022) 提出了一种**可逆边缘化**机制：当路标需要降级时，通过 Schur 补将多个观测压缩为带精确信息矩阵 H* 的位姿-位姿边（而非直接丢弃），在回环时可逆向恢复为完整重投影观测。

**对当前设计的启示**：
- MARGINALIZED 状态可以升级为 **COMPRESSED（可逆压缩）** 中间状态
- 压缩边保留 H* 信息矩阵（含完整观测几何），而非仅保留对角权重
- MST（最大生成树）边选择策略：以共视路标数量为权重，确保最重要协视关系可恢复
- 复活（revival）条件：回环检测成功时，或路标重新被 ≥ K 帧观测到时

当前 MVP 仍采用不可逆 MARGINALIZED → CULLED 路径，OKVIS2 的可逆边缘化作为 Phase 2 增强。

---

## 相关页面

- [[方法-SmartStereoFactor]]
- [[概念-三角化与深度估计]]
- [[方法-延迟边缘化VIO]]
- [[2026-05-18-landmark-lifecycle]]
- [[算法-Kimera-VIO]]
- [[算法-SVO-Pro]]
- [[算法-ORB-SLAM3]]
- [[算法-DM-VIO]]
- [[GTSAM SLAM 与视觉因子 API]]
