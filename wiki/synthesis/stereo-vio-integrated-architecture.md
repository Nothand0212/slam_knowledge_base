---
created: 2026-06-01
updated: 2026-06-01
type: synthesis
tags: [stereo-vio, 架构设计, 多模块集成, GTSAM, iSAM2]
sources:
  - wiki/synthesis/landmark-pipeline-design.md
  - wiki/entities/设计-立体VIO前端管线.md
  - wiki/entities/架构-GTSAM iSAM2 双目VIO后端设计.md
  - wiki/entities/设计-双目VIO初始化子系统.md
  - raw/notes/loop_closure_design.md
---

# 双目 VIO 集成架构设计

> 综合路标管线、前端、因子图后端、初始化、回环五个模块的详细设计，产出统一的系统架构、模块接口契约、数据流和全局参数表。

## 一、设计来源与约束

### 1.1 从失败中提炼的设计约束

| # | 约束 | 来源 |
|---|------|------|
| C1 | **禁止 SmartFactor-only 后端**——路标必须经过试用期后晋升为显式变量 | PHAD 20× RMSE 退化 (0.09m→1.81m) |
| C2 | **因子注入顺序不可颠倒**——SmartFactor 必须先于 IMU 因子加入 | Kimera-VIO 源码注释 + Ghost landmark 为空的 bug |
| C3 | **初始化期间禁止 SmartFactor**——前 10 KF 全部显式路标 | 初始位姿不确定性导致隐式三角化不可靠 |
| C4 | **回环后 SmartFactor 必须重新三角化**——用校正后位姿重算 | Loop→Landmark 接口时序修正 |
| C5 | **边缘化不可逆**——参数收敛前延迟边缘化，收敛后常规处理 | DM-VIO 延迟边缘化范式 |
| C6 | **双目基线提供绝对尺度**——初始化跳过单目的尺度恢复 | 立体 VIO 核心优势 |

### 1.2 五个模块的设计文档

| 模块 | 文档 | 子代理轮次 |
|------|------|-----------|
| 路标管线 | `wiki/synthesis/landmark-pipeline-design.md` | 6 参考系统并行研究 |
| 前端 | `wiki/entities/设计-立体VIO前端管线.md` | 5 参考系统，820 行 |
| 因子图后端 | `wiki/entities/架构-GTSAM iSAM2 双目VIO后端设计.md` | 4 参考系统源码分析 |
| 初始化 | `wiki/entities/设计-双目VIO初始化子系统.md` | 4 参考系统，932 行 |
| 回环 | `raw/notes/loop_closure_design.md` | 3 参考系统，750 行 |

---

## 二、系统线

### 2.1 线程模型

```
┌─────────────────────────────────────────────────────────────┐
│                        主进程                               │
├───────────────┬───────────────────┬─────────────────────────┤
│  Tracking     │   Local Mapping   │   Loop Closing          │
│  Thread       │   Thread          │   Thread                │
│  (每帧)       │   (每关键帧)       │   (后台, 1-5 Hz)        │
├───────────────┼───────────────────┼─────────────────────────┤
│ 前端:         │ 路标管线:          │ 回环检测:               │
│  KLT 跟踪     │  深度滤波          │  DBoW3 ORB 查询          │
│  立体 NCC 匹配 │  SmartFactor 试用  │  时间/空间过滤           │
│  IMU 预积分    │  晋升门控          │  分组一致性确认          │
│  关键帧决策    │  显式路标管理      │                          │
│               │  异常值监测        │ 几何验证:               │
│               │                    │  3D-2D PnP RANSAC       │
│               │ 因子图后端:         │  共视邻居确认            │
│               │  因子组装          │                          │
│               │  iSAM2.update()   │ 回环注入:               │
│               │  Post-update chi2  │  Backend 线程内执行      │
│               │  边缘化管理        │  BetweenFactor + 提升    │
└───────────────┴───────────────────┴─────────────────────────┘

共享数据结构:
  FeatureDatabase  (前端写, 路标管线读)
  KeyframeDatabase (路标管线写, 回环读)
  ExplicitLandmarkMap (路标管线写, 回环读)
  iSAM2 + Smoother (路标管线写, 回环线程只读)
```

### 2.2 全局状态机

```
┌──────────────┐
│ UNINITIALIZED │
└──────┬───────┘
       │
       ▼
┌──────────────┐   静止条件满足    ┌──────────────┐
│ STATIC_CHECK │ ───────────────→ │ STATIC_INIT   │
└──────┬───────┘                  └──────┬───────┘
       │ 静止失败                        │ 成功
       ▼                                 ▼
┌──────────────┐   运动条件满足    ┌──────────────┐
│ DYNAMIC_CHECK│ ───────────────→ │ DYNAMIC_INIT  │
└──────┬───────┘                  └──────┬───────┘
       │ 运动不足                        │ 成功
       ▼                                 ▼
   ┌──────────┐              ┌──────────────────┐
   │ 重试/等待 │              │   INITIALIZED     │
   └──────────┘              │ (SmartFactor 启用) │
                             └────────┬─────────┘
                                      │
                        ┌─────────────┼─────────────┐
                        ▼             ▼             ▼
                   ┌─────────┐  ┌─────────┐  ┌─────────┐
                   │ NOMINAL │  │ LOOP_   │  │ GLOBAL  │
                   │ (正常运营)│  │ CORRECT │  │ BA      │
                   └─────────┘  └─────────┘  └─────────┘
```

### 2.3 路标状态机（来自路标管线模块）

```
CANDIDATE → DEPTH_FILTER → SMART_TRIAL → PROMOTING → EXPLICIT → STABLE
                ↓ 失败         ↓ 退化        ↓ 失败      ↓ 异常值
              CULLED         CULLED       CULLED    REMEDIATING
                                                       ↓ 恢复失败
                                                     CULLED
                                                       
STABLE → MARGINALIZED (被边缘化)
```

**SmartFactor 仅在 SMART_TRIAL 和 PROMOTING 状态使用。其余状态均为显式路标。**

---

## 三、模块接口契约

### 3.1 Frontend → Landmark Pipeline

```
接口: PendingLandmark + FeatureDatabase 查询

struct PendingLandmark {
    uint64_t track_id;                     // 路标唯一标识
    AdmissionLevel admission_level;        // L0-L4 质量评分
    StereoQuality stereo_quality;          // NCC 分数、视差、深度不确定性
    TrackQuality track_quality;            // 跟踪长度、光流幅度、RANSAC 比例
    Initial3D initial_3d;                  // 锚定关键帧下的 3D 位置
};

// 历史观测通过 FeatureDatabase 按 track_id 查询
// 前端每帧写入 FeatureDatabase::update(track_id, frame_id, kp_L, kp_R, ...)
// 路标管线在关键帧时刻查询 FeatureDatabase::get_observations(track_id)

enum class AdmissionLevel {
    L0_RAW = 0,      // 仅单帧立体观测，未经验证
    L1_STEREO = 1,   // 左右一致性通过 + NCC > 阈值
    L2_DEPTH = 2,    // 深度不确定性 < 阈值 + 深度在有效范围
    L3_ESTABLISHED = 3, // 多帧跟踪 + 视差 > 阈值
    DISCARD = 99     // 拒绝进入路标管线
};
```

### 3.2 Landmark Pipeline → Backend

```
接口: LandmarkInjection (每关键帧)

struct LandmarkInjection {
    // 试用期路标 (SMART_TRIAL 状态)
    std::vector<SmartStereoFactor> smart_factors;
    
    // 晋升路标 (PROMOTING → EXPLICIT 状态)
    std::vector<ExplicitLandmark> promoted_landmarks;
    
    // 现有显式路标的新观测
    std::vector<LandmarkObservation> existing_observations;
    
    // Post-update chi2 异常值（需从图中移除）
    std::vector<LandmarkId> outlier_ids;
};

// 注入顺序（强制约束，违反会导致退化）:
// 1. SmartFactors FIRST
// 2. IMU factors + prior factors
// 3. Existing explicit landmark factors
// 4. New promoted landmark factors (含 Point3 Values insert)
// 5. isam2.update()
// 6. Post-update chi2 outlier check
```

### 3.3 Initialization → Backend + Landmark Pipeline

```
接口: InitializationResult

struct InitializationResult {
    bool success;
    gtsam::Values initial_values;        // X(0), V(0), B(0)
    gtsam::NonlinearFactorGraph priors;  // PriorFactor<Pose3>, PriorFactor<Vector3>, PriorFactor<ConstantBias>
    gtsam::Vector3 gravity_direction;    // 世界坐标系中的重力方向
    gtsam::imuBias::ConstantBias initial_bias;
};

// 传递给 Backend:
//   将 initial_values 和 priors 作为首帧变量和因子
//   后续关键帧从 ID=1 开始递增

// 传递给 Landmark Pipeline:
//   设置 phase = INITIALIZATION (0-9 KFs)
//   强制 SmartFactor 禁用，全部显式路标
//   第 10 KF 触发 transition_to_normal()
```

### 3.4 Loop Closure → Backend + Landmark Pipeline

```
接口: LoopResult

struct LoopResult {
    bool accepted;
    FrameId loop_kf_id;                  // 匹配到的历史关键帧
    FrameId query_kf_id;                 // 当前查询关键帧
    gtsam::Pose3 T_relative;             // loop_kf → query_kf 相对位姿
    gtsam::Matrix6 covariance;           // 从 PnP 内点分布估计
    std::vector<LandmarkId> affected_smarts; // 受影响的 SmartFactor ID 列表
};

// 注入时序 (关键修正):
//  1. 暂停前端输入
//  2. inject Loop BetweenFactor 到 iSAM2
//  3. isam2.update() → 传播校正到位姿
//  4. detectAffectedSmartFactors() → 用校正后的位姿检测
//  5. promoteSmartFactors() → 重三角化 + 创建显式因子
//  6. isam2.update() → 纳入提升后的显式因子
//  7. 恢复前端输入
```

---

## 四、单关键帧端到端数据流

```
第 N 关键帧处理流程:

Frontend Thread:
  1. 帧到达 → KLT 跟踪 + 立体 NCC 匹配
  2. IMU 预积分 (KF_{N-1} → KF_N)
  3. 关键帧决策 (视差/时间/特征数)
  4. → 输出: FrontendOutput {特征观测, IMU PIM, 新 PendingLandmarks}

Landmark Pipeline Thread (接收 FrontendOutput):
  5. 对于每个 PendingLandmark:
     - 查询 FeatureDatabase → 历史观测列表
     - AdmissionLevel < L2 → 跳过
     - 新路标: 初始化 DEPTH_FILTER 状态
     - 现有 SMART_TRIAL 路标: 更新 SmartFactor, 检查晋升条件
     - EXPLICIT 路标: 生成新 GenericStereoFactor 观测
  6. 检查晋升门控 (≥4 观测, 视差角>3°, 重投影<2.0px, SVD 条件数<1e6)
  7. 通过 → 创建 ExplicitLandmark (Point3 + GenericStereoFactor)
  8. → 输出: LandmarkInjection

Backend (同线程, Landmark Pipeline 内):
  9. 组装新因子图:
     new_factors: [SmartFactors..., IMU, priors, explicit_factors, promoted_factors]
     new_values:  [X(N), V(N), B(N)] ∪ [L(id) for promoted landmarks]
  10. isam2.update(new_factors, new_values, timestamps)
  11. Post-update 异常值检测:
      - 对每个显式路标, 计算 chi2 = r^T * Σ^{-1} * r
      - chi2 > 7.815 → 标记为异常值
      - 连续 3 帧异常值 → 从图中移除, 路标 CULLED
  12. 边缘化过期变量 (nr_states KFs 之外)
  13. → 输出: 更新后的位姿估计, IMU bias

Loop Closure Thread (异步, 回环检测成功后):
  14. 通过 shared_mutex 读取 KeyframeDatabase
  15. 回环检测 → 几何验证 → 回环接受
  16. 插入 Backend 队列: LoopResult
  17. Backend 线程在下一个关键帧处理前消费 LoopResult:
      注入 BetweenFactor → isam2.update() → 提升受影响的 SmartFactors → isam2.update()
```

---

## 五、跨模块冲突解决记录

### 5.1 Frontend→Landmark: 历史观测传递方式

**冲突**: 路标管线期望 `std::vector<FrameObservation>` 作为 PendingLandmark 的一部分，但前端按帧输出，不维护历史。

**解决**: 采用 **FeatureDatabase 共享模式**（参考 OpenVINS）。前端每帧写入观测到 FeatureDatabase，路标管线在关键帧时按 `track_id` 查询。`PendingLandmark` 只传 `track_id` 和 anchor KF 的当前观测。

### 5.2 Backend→Landmark: SmartFactor 在 iSAM2 中的替换

**冲突**: SmartFactor 晋升为 GenericStereoFactor 后，旧的 SmartFactor 仍保留在 iSAM2 的 Bayes Tree 中，造成冗余约束。

**解决**: 两种策略：
- **v1 方案（简单）**: 保留旧 SmartFactor——它会在下次重线性化时用校正后位姿重新三角化，收敛到与显式因子一致的结果。冗余但数学一致。
- **v2 方案（精确）**: 使用 `isam2.update(graph, initial, factor_indices_to_remove)` 移除旧 SmartFactor。需维护 `FactorIndex` 映射表。

### 5.3 Loop→Landmark: 回环后 SmartFactor 提升时序

**冲突**: 原始设计在回环注入**之前**提升 SmartFactor，但此时 iSAM2 尚未传播校正，`calculateEstimate()` 返回的是旧位姿。

**解决（关键修正）**: 
```
正确顺序: inject Loop → iSAM2.update() → promote after correction → iSAM2.update()
错误顺序: promote before injection → inject Loop → iSAM2.update()
```
提升时机从"回环检测成功后立即执行"改为"回环注入 + iSAM2 传播校正后执行"。

### 5.4 Initialization→All: SmartFactor 启用时机

**冲突**: 初始化期间启用了来自 frontend 的 SmartFactor？

**解决**: 统一规定：在 `InitializationResult.success == true` 且 `current_kf_id >= 10` 之前，Landmark Pipeline 的 `phase = INITIALIZATION`，所有新路标直接进入 EXPLICIT 状态（跳过 SMART_TRIAL）。第 10 KF 后 `transition_to_normal()` 启用 SmartFactor 试用期。

---

## 六、全局参数速查表

### 6.1 前端参数

| 参数 | 推荐值 | 单位 | 来源 |
|------|--------|------|------|
| max_feature_count | 200-400 | 个 | VINS-Fusion stereo |
| min_feature_dist | 15 | px | VINS-Fusion |
| KLT window size | 21×21 | px | OpenVINS |
| KLT pyramid levels | 3 (1 with IMU) | - | OpenVINS |
| KLT max iterations | 30 | - | VINS-Fusion |
| KLT epsilon | 0.01 | px | VINS-Fusion |
| FB threshold | 1.5 | px | VINS-Fusion |
| NCC window size | 11×11 | px | Kimera-VIO |
| min disparity | 1.0 | px | stereo basic |
| max disparity | 128 | px | stereo basic |
| left-right consistency | 1.0 | px | standard |
| min KF parallax | 10 (10px/460归一化) | px | VINS-Fusion |
| max KF interval | 1.0 | s | empirical |
| min KF features | 50 | 个 | empirical |
| feature survival ratio | 0.7 | - | PHAD |

### 6.2 IMU 参数 (ADIS16448, EuRoC)

| 参数 | 推荐值 | 单位 | 来源 |
|------|--------|------|------|
| accel_noise_density | 2.0e-3 | m/s²/√Hz | Kalibr empirical |
| gyro_noise_density | 1.9e-4 | rad/s/√Hz | Kalibr empirical |
| accel_random_walk | 3.0e-4 | m/s³/√Hz | datasheet |
| gyro_random_walk | 1.0e-5 | rad/s²/√Hz | datasheet |
| gravity_magnitude | 9.81 | m/s² | standard |
| IMU rate | 200 | Hz | ADIS16448 |

### 6.3 后端参数

| 参数 | 推荐值 | 来源 |
|------|--------|------|
| smoother_lag (nr_states) | 30 | Kimera-VIO |
| relinearizeThreshold | 0.01 | Kimera-VIO |
| relinearizeSkip | 1 | Kimera-VIO |
| wildfireThreshold | 0.001 | Kimera-VIO |
| optimizer | GaussNewton | Kimera-VIO |
| factorization | CHOLESKY | Kimera-VIO |
| cacheLinearizedFactors | true | Kimera-VIO |
| delay_marg_frames | 100 | DM-VIO (初始化阶段) |

### 6.4 路标管线参数

| 参数 | 推荐值 | 单位 | 来源 |
|------|--------|------|------|
| min_obs_for_promotion | 4 | 帧 | Kimera-VIO |
| min_parallax_promotion | 3.0 | ° | derivation |
| max_reproj_promotion | 2.0 | px | empirical |
| svd_condition_max | 1e6 | - | numerical stability |
| chi2_outlier_stereo | 7.815 | - | χ²₃(p=0.05), ORB-SLAM3 |
| consecutive_outlier_cull | 3 | 帧 | ORB-SLAM3 (nThObs) |
| depth_filter_convergence | σ² < (μ_range/200)² | - | SVO Pro |
| depth_filter_a_init | 10 | - | SVO Pro |
| depth_filter_b_init | 10 | - | SVO Pro |
| smart_factor_rank_tolerance | 1.0 | - | Kimera-VIO |
| smart_factor_retriangulation | 1e-5 | - | GTSAM default |
| smart_factor_outlier_rejection | 3.0 | - | Kimera-VIO |

### 6.5 初始化参数

| 参数 | 推荐值 | 单位 | 来源 |
|------|--------|------|------|
| static_imu_threshold | 1.0-1.5 | m/s² | OpenVINS |
| static_window_size | 1.0 | s | OpenVINS |
| min_kfs_dynamic_init | 10 | 个 | ORB-SLAM3 stereo |
| min_duration_dynamic_init | 1.0 | s | ORB-SLAM3 |
| gravity_magnitude_check | 9.81 ± 2.0 | m/s² | sanity |
| gyro_bias_max | 0.1 | rad/s | sanity |
| gravity_refinement_iter | 4 | - | VINS-Fusion |
| dynamic_init_GN_iter | 50 | - | ORB-SLAM3 |

### 6.6 回环参数

| 参数 | 推荐值 | 单位 | 来源 |
|------|--------|------|------|
| DBoW3 vocab tree k | 10 | - | ORB-SLAM3 |
| DBoW3 vocab tree L | 6 | - | ORB-SLAM3 |
| DBoW3 score threshold | 0.015 | - | VINS-Fusion |
| temporal_exclusion_window | 50 | 帧 | standard |
| group_consistency_min | 3 | 连续关键帧 | ORB-SLAM3 |
| PnP min inliers | 15 | 个 | ORB-SLAM3 |
| PnP confidence | 0.99 | - | standard |
| PnP max iterations | 300 | - | standard |
| PnP reprojection threshold | 3.0 | px | VINS-Fusion |
| covariance_from_inlier_distribution | true | - | 禁止 Identity |
| huber_k_pgo | 1.345 | - | standard robust |
| min_loops_for_GBA | 3 | 次 | empirical |
| GBA LM iterations | 50 | - | empirical |

---

## 七、已知跨模块失效模式

| # | 模式 | 涉及模块 | 症状 | 缓解 |
|---|------|---------|------|------|
| FM1 | 初始化期间 SmartFactor 被意外启用 | Init→Landmark | 初始路标三角化错误，后续无法纠正 | LandmarkPipeline.phase 检查 |
| FM2 | SmartFactor 晋升时序错误 | Landmark→Backend | 新 Point3 变量未在 new_values 中 → iSAM2 异常 | LandmarkInjection 验证 |
| FM3 | 回环后 SmartFactor 用旧位姿提升 | Loop→Landmark | 已提升的路标位置错误 → 全局偏差 | 必须等 iSAM2 传播校正后再提升 |
| FM4 | chi2 异常值在 SmartFactor 试用期累积 | Backend→Landmark | 大量路标被标记异常值 → 特征不足 | 区分 SmartFactor 内部拒绝 vs 显式异常值 |
| FM5 | FeatureDatabase 与 iSAM2 状态不同步 | Frontend→Backend | 路标管线查询到的历史帧已被后端边缘化 | 检查 frame_id 是否在 smoother lag 内 |
| FM6 | 延迟边缘化队列溢出 | Backend | 100 帧延迟 + 高帧率关键帧 → 内存爆炸 | 监控 delayed_frames.size() |
| FM7 | DBoW3 误匹配 + 几何验证漏检 | Loop→Backend | 假正回环注入 → 全局图扭曲 | PnP + 共视邻居 + 里程计一致性三重验证 |
| FM8 | 初始化失败后无限重试 | Init | 系统永远不进入正常模式 | max_retries=3, 最终降级为弱先验初始化 |

---

## 八、相关页面

- [[从零搭建VIO系统]]
- [[VIO方案全景对比]]
- [[因子图vs滤波]]
- [[phad_fusion设计总结]]
