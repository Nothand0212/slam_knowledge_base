# 回环检测与全局一致性设计：Stereo VIO 因子图后端

> 设计目标：为基于 GTSAM iSAM2 的双目 VIO 系统设计完整的回环闭合子系统 —— 检测、几何验证、位姿图优化、SmartFactor 后处理、可选全局 BA。
> 设计日期：2026-06-01

---

## 1. 概述

本设计描述回环闭合子系统在 Stereo VIO 后端的全链路行为。后端使用 GTSAM iSAM2（`IncrementalFixedLagSmoother`）作为增量优化引擎，路标管线为三阶段：
**深度滤波器 → SmartStereoFactor 试跟踪 → 显式 GenericStereoFactor 提升**。

回环闭合与这个路标管线存在根本性冲突：当回环约束调整位姿时，SmartFactor 基于调整前位姿的三角化结果变为过时（stale），必须妥善处理。

本设计的核心原则：
- 回环检测和验证在**独立线程**中异步执行，不阻塞 VIO 线程
- 回环因子（`BetweenFactor<Pose3>`）**紧耦合**注入 iSAM2，iSAM2 的贝叶斯树天然支持增量位姿图优化
- SmartFactor 失效问题通过**检测-提升**策略解决：检测受回环影响的 SmartFactor → 提升为显式因子 → 重新三角化
- 全局 BA 作为可选的后处理步骤，在独立线程中运行

---

## 2. 系统架构

### 2.1 数据流

```
┌────────────────┐    FeatureTrack      ┌────────────────┐
│  Frontend       │────────────────────→│  Backend        │
│  (FAST+KLT)     │    keyframe_signal   │  (iSAM2)        │
│                 │←─── ORB desc req ───│                 │
└────────────────┘                     └───────┬─────────┘
                                               │ LoopCandidate
                                        ┌──────▼─────────┐
                                        │  LoopDetector   │
                                        │  (独立线程 1-5Hz) │
                                        │  DBoW3+ORB+PnP  │
                                        └──────┬─────────┘
                                               │ LoopResult
                                        ┌──────▼─────────┐
                                        │  LoopInjector   │
                                        │  (Backend线程内) │
                                        │  SmartFactor升   │
                                        │  级+因子注入+GBA │
                                        └────────────────┘
```

### 2.2 线程模型

| 线程 | 频率 | 职责 |
|------|------|------|
| Frontend | 20-50 Hz | 特征提取、KLT跟踪、三角化、关键帧判定。应答后端请求：对关键帧提取 ORB 描述子并写入 KF 对象 |
| Backend | 10-20 Hz | iSAM2 增量优化、滑动窗口管理、接收回环注入指令、SmartFactor 提升、触发 GBA |
| LoopDetector | 1-5 Hz | DBoW3 查询、PnP RANSAC 验证、噪声模型估计。**不直接修改因子图**，只产出 `LoopResult` 投递给 Backend |

**线程安全契约**：
- LoopDetector 只读：关键帧数据库、显式路标3D坐标、iSAM2当前位姿估计
- LoopDetector 只写：`LoopResult` 消息（通过 lock-free SPSC 队列发给 Backend）
- Backend 在 `iSAM2::update()` 的间隙处理回环注入，保证 iSAM2 内部状态一致性

---

## 3. 回环检测管线

### 3.1 特征提取策略

回环检测使用**独立的 ORB 特征提取器**（与前端 FAST+KLT 分离），理由：
- 前端 KLT 跟踪的特征不需要描述子，提取 ORB 会浪费计算
- 回环需要旋转不变描述子，ORB 的 IC_Angle + rBRIEF 天然满足
- 参考 [[算法-Kimera-VIO]] 的做法：`LoopClosureDetector` 使用独立 `cv::ORB` 提取器
- 参考 [[方法-视觉回环检测管线]]：ORB-SLAM3 全链路依赖 ORB 描述子

**提取时机**：Backend 在插入新关键帧后，向 Frontend 请求该帧的 ORB 描述子。Frontend 异步计算后写入 `KeyFrame` 对象。关键帧需缓存原始/校正左图用于此目的。

**ORB 参数**（初始推荐，需根据数据集标定）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `nfeatures` | 1000 | 每帧 ORB 特征数 |
| `scaleFactor` | 1.2 | 金字塔缩放因子 |
| `nlevels` | 8 | 金字塔层数 |
| `iniThFAST` | 20 | 初始 FAST 阈值 |
| `minThFAST` | 7 | 最小 FAST 阈值（兜底） |
| `patchSize` | 31 | IC_Angle 灰度质心 patch |
| `WTA_K` | 2 | BRIEF 描述子配对方式 |

### 3.2 DBoW3 词汇表

使用 **DBoW3**（C++11 重写版本，比 DBoW2 API 更现代）搭配 ORB 描述子：

**训练方案**：
1. **方案 A（推荐）**：使用预训练的 ORB 词汇表（如 ORB-SLAM3 附带的 `ORBvoc.txt.tar.gz`），加载后可直接用于 DBoW3（需格式转换）
2. **方案 B**：在目标场景数据集上训练自定义词汇表（DBoW3 提供 `demo/train_voc.cpp` 工具）
   - 分支因子 `k = 10`，层数 `L = 6`（与 VINS-Fusion 的 `brief_k10L6` 一致）
   - 训练集：目标场景代表性图像 5000-10000 张

**数据库管理**：
- 每个关键帧插入时调用 `database.add(descriptors)` 加入 DBoW3 数据库
- 不区分地图（简化单地图模式，放弃 ORB-SLAM3 Atlas 的多地图管理）

### 3.3 回环检测伪代码

```
function detectLoop(current_kf):
    // Step 1: DBoW3 查询
    results = database.query(current_kf.bow_vector, max_results=4)
    
    // Step 2: 时间过滤（排除最近 50 帧）
    results = filter(results, lambda r: current_kf.id - r.id > 50)
    
    // Step 3: 评分过滤（DBoW3 归一化相似度）
    min_score = 0.015  // 参考 VINS-Fusion 的 0.015 阈值
    candidates = [r for r in results if r.score >= min_score]
    if empty(candidates): return NO_LOOP
    
    // Step 4: 分组检测（一致性检测，避免单帧误检）
    //   将时间接近的候选分为一组，组内成员数 >= 3 才进入几何验证
    //   参考 ORB-SLAM3 的 mnCovisibilityConsistencyTh = 3
    return group_candidates(candidates, min_group_size=3)
```

### 3.4 时间与空间过滤

| 过滤条件 | 值 | 理由 |
|----------|-----|------|
| 最小帧间隔 | 50 帧 | 排除最近历史（约 5-10s @ 5-10Hz），避免自匹配 |
| 最小空间距离 | 3m（基于里程计） | 排除空间上太近的候选 |
| 描述子相似度阈值 | 0.015（DBoW3 L1-score） | 参考 VINS-Fusion，低于此值的候选大概率是误匹配 |

---

## 4. 几何验证

### 4.1 验证策略选择

回环候选经过描述子匹配筛选后，必须通过几何验证才能确认。本设计采用 **3D-2D PnP RANSAC** 作为主验证方案：

**为什么 3D-2D PnP（而非 2D-2D 对极几何）**：
- Stereo VIO 有大量已提升的显式路标（`GenericStereoFactor`），它们的 3D 坐标在局部窗口内精度高
- PnP 直接恢复 SE(3)，无需 2D-2D→三角化→尺度恢复的两步流程
- PnP 内点数/重投影误差直接反映回环的几何一致性

**为什么不用 Sim3**：
- 双目系统尺度可观测，无尺度漂移。`s=1` 固定。
- SE(3) 比 Sim(3) 少 1 个自由度，RANSAC 收敛更快（3 点 vs 4 点）

### 4.2 显式路标不足时的回退策略

```
function geometricVerify(current_kf, candidate_kf):
    // Step 1: 收集候选帧的显式路标（已提升为 GenericStereoFactor 的）
    explicit_lms = candidate_kf.getExplicitLandmarks()
    
    // Step 2: ORB 描述子匹配：当前帧 2D → 候选帧 3D
    matches = matchByBoW(current_kf.descriptors, candidate_kf.descriptors)
    p3d = [matches[i].landmark_3d for i where matches[i].landmark_3d exists]
    p2d = [matches[i].keypoint_2d for i where matches[i].landmark_3d exists]
    
    if count(p3d) >= MIN_PNP_MATCHES (20):
        // 主路径：3D-2D PnP RANSAC
        return verifyByPnPRANSAC(p3d, p2d, camera_model, params)
    
    elif count(matches) >= MIN_2D2D_MATCHES (30):
        // 回退路径 1：2D-2D 对极几何 + 三角化恢复尺度
        // （候选帧路标太少时使用）
        E, inlier_mask = findEssentialMat(p2d_cur, p2d_cand, focal, principal_point)
        R, t = recoverPose(E, p2d_cur, p2d_cand, ...)
        // 三角化内点并重投影验证
        if verifyTriangulatedPoints(...) >= MIN_INLIERS:
            return SE3Result(R, t, inlier_info)
    
    else:
        // 回退路径 2：如果连 2D-2D 匹配都不够
        return VERIFY_FAILED_INSUFFICIENT_MATCHES
```

### 4.3 PnP RANSAC 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 最小内点数 | 15 | 参考 ORB-SLAM3 `nBoWInliers=15` |
| 置信度 | 0.99 | 标准 RANSAC 置信度 |
| 最大迭代次数 | 300 | `maxIters=300`，与 ORB-SLAM3 Sim3Solver 一致 |
| 重投影误差阈值（像素） | 3.0 px | 双目系统特征定位精度 ~1px，取 3σ |
| 额外验证: 相对 yaw | < 30° | 排除明显方向不一致 |
| 额外验证: 相对平移 | < 20m | 排除不合理的远距离回环（参考 VINS-Fusion） |

### 4.4 额外一致性检查

参考 [[方法-回环验证方法族]] 的 N 阶段验证框架，PnP 通过后增加：

```
// Step 3a: 共视邻域一致性（参考 ORB-SLAM3 §2d）
consistent_neighbors = 0
for neighbor_kf in current_kf.covisibles(top=5):
    T_rel = neighbor_kf.T_world_body * T_body_world_corrected
    proj_inliers = searchByProjection(neighbor_kf, candidate_kf, T_rel)
    if proj_inliers >= MIN_NEIGHBOR_INLIERS (10):
        consistent_neighbors += 1

if consistent_neighbors < 3:
    return VERIFY_FAILED_CONSISTENCY  // 至少 3 个邻居确认

// Step 3b: 里程计一致性检查
T_odom = composeOdometryFromKFs(current_kf, candidate_kf)
T_err = T_loop * T_odom.inverse()
trans_per_frame = T_err.translation().norm() / num_frames_between
if trans_per_frame > MAX_ODOM_DRIFT_PER_FRAME:
    return VERIFY_FAILED_ODOM  // 回环位姿与里程计严重不一致
```

---

## 5. 回环因子注入

### 5.1 因子类型与图结构

回环约束被建模为 `gtsam::BetweenFactor<gtsam::Pose3>`，连接两个关键帧位姿节点：

```
BetweenFactor<Pose3>:
  key1 = X(current_kf_id)   // 当前关键帧的位姿变量
  key2 = X(candidate_kf_id) // 回环候选关键帧的位姿变量
  measured = T_ij           // PnP RANSAC 恢复的相对位姿: T_j ← T_i^{-1} * T_j
  noise_model = Σ           // 从 PnP 内点分布估计的协方差
```

**为什么是 BetweenFactor（而非其他约束类型）**：
- `BetweenFactor<Pose3>` 约束的是相对位姿，天然适合回环边
- iSAM2 对 BetweenFactor 的增量处理已高度优化
- 与里程计因子（IMU 预积分、视觉/LiDAR 相对位姿）使用相同因子类型，便于统一管理

### 5.2 噪声模型构建

**禁止使用固定 `Identity()` 协方差**（用户的诊断文档显示这是已知问题）。噪声模型必须从 PnP 几何验证中**数据驱动估计**：

```
function computeNoiseModel(pnp_result):
    // 方案 A（推荐）：从 PnP 内点的重投影误差分布估计信息矩阵
    // 注：重投影误差在像素空间，需映射到位姿空间
    
    // Step 1: 计算 PnP 最终内点的平均重投影误差
    mean_reproj_error = mean(pnp_result.inlier_reprojection_errors)
    
    // Step 2: 将像素误差转换为归一化平面误差
    mean_normalized_error = mean_reproj_error / focal_length
    
    // Step 3: 从内点数量估计平移不确定性
    // 三角形定位精度 ≈ (baseline * error) / (disparity * sqrt(N))
    // 简化估计：σ_trans = mean_normalized_error * avg_depth / sqrt(n_inliers)
    avg_depth = mean(pnp_result.inlier_3d_points.depth())
    sigma_trans = mean_normalized_error * avg_depth / sqrt(n_inliers)
    
    // Step 4: 旋转不确定性：近似为平移不确定性与平均深度的比值
    sigma_rot = sigma_trans / avg_depth
    
    // Step 5: 构建信息矩阵（对角近似）
    info = Diagonal(6)
    info[0:3, 0:3] = 1.0 / (sigma_rot^2)     // 旋转部分
    info[3:6, 3:6] = 1.0 / (sigma_trans^2)   // 平移部分
    
    return GaussianNoiseModel(information=info)
```

**方案 B（简化备选）**：如果特征分布各向异性不严重，可用标量噪声模型近似：

```
// 平移噪声 ≈ 0.05-0.15m，旋转噪声 ≈ 0.01-0.03 rad
// 具体值需在实际数据集上标定
sigma_trans = clamp(0.3 / sqrt(n_inliers), 0.03, 0.2)  // 米
sigma_rot   = clamp(0.1 / sqrt(n_inliers), 0.005, 0.05)  // 弧度
noise = IsotropicNoiseModel(sigma=sigma_trans, dim=6)
```

### 5.3 鲁棒核函数

所有回环因子应包裹 **Huber 鲁棒核**（参考 [[方法-回环验证方法族]] 中对 `Huber(size=1.0)` 的推荐）：

```cpp
auto huber = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),  // 标准 Huber k
    noise_model
);
auto loop_factor = gtsam::BetweenFactor<gtsam::Pose3>(
    X(current_kf_id), X(candidate_kf_id), T_ij, huber
);
```

Huber 核的目的：即使经过几何验证的单个回环边仍可能有小概率误匹配。鲁棒核确保该边不会压倒里程计和 IMU 约束。

### 5.4 注入时序

```
Backend 线程内伪代码：

function onLoopResult(loop_result):
    // 阶段 1: 暂停接收新的前端输入（或使用双缓冲）
    pauseFrontendInput()
    
    // ═══════════════════════════════════════════
    // 关键时序约束：SmartFactor 提升必须在回环因子注入
    // 且 iSAM2 传播校正之后执行，否则重三角化使用的
    // 位姿仍是校正前的旧值。
    // ═══════════════════════════════════════════
    
    // 阶段 2: 注入回环因子（先注入，后传播）
    isam2_graph.push_back(loop_factor)
    isam2_initial.insert(X(current_kf), corrected_pose)
    
    // 阶段 3: iSAM2 第一次增量更新（传播回环校正到受影响位姿）
    isam2.update(isam2_graph, isam2_initial)
    isam2_graph.resize(0)
    isam2_initial.clear()
    
    // 阶段 4: 检测提升受影响的 SmartFactor
    //   ← 此时 isam2.calculateEstimate() 返回的是校正后的位姿
    //   ← SmartFactor 重三角化使用校正后的几何，而非旧位姿
    affected_smart_factors = detectAffectedSmartFactors(
        loop_result.current_kf_id,
        loop_result.candidate_kf_id
    )
    promoteSmartFactors(affected_smart_factors)
    // promoteSmartFactors() 内部：
    //   - 从 iSAM2 estimate 读取校正后位姿
    //   - 重三角化 → 创建 GenericStereoFactor + Point3 变量
    //   - 将新因子和新变量插入 new_factors/new_initial
    
    // 阶段 5: iSAM2 第二次增量更新（纳入提升后的显式因子）
    //   注意：iSAM2 会基于新线性化点（校正后位姿）重线性化
    if not new_factors.empty():
        isam2.update(new_factors, new_initial)
    
    // 阶段 6: （可选）检查漂移，触发全局 BA
    if estimateDrift() > DRIFT_THRESHOLD:
        scheduleGlobalBA()
    
    // 阶段 7: 恢复前端输入（在下一次 KF 因子注入之前）
    resumeFrontendInput()
    
    // 阶段 8: 发布校正后的轨迹供下游使用
    publishCorrectedTrajectory()
```

**关键点**：
- 回环因子注入在 Backend 线程内部执行（与 iSAM2 更新同一线程），保证 iSAM2 状态一致性
- iSAM2 的贝叶斯树会自动重线性化受回环边影响的团，天然实现增量 PGO
- **SmartFactor 提升必须在回环注入+传播之后**（阶段 4），否则重三角化使用旧位姿，提升无意义
- 回环处理产生**两次 iSAM2 更新**：第一次传播回环校正，第二次纳入提升后的显式因子
- **不需要**像 ORB-SLAM3 那样手动停止 LocalMapping 或中止 Global BA——iSAM2 处理增量更新天然支持并发安全性（前提是单线程内注入）

---

## 6. Post-Loop SmartFactor 提升策略

### 6.1 问题定义

当回环约束调整关键帧位姿后，**所有以回环区域位姿为 anchor 的 SmartFactor 的三角化结果都变为过时**。GTSAM 的 SmartFactor 内部缓存三角化结果，如果位姿发生显著变化，缓存的 3D 点将不再是正确线性化点，导致后续优化收敛到局部最小值或发散。

**受影响范围**：
- 直接受影响：回环连接的两个关键帧（current_kf, candidate_kf）上观测到的所有 SmartFactor
- 间接影响：回环帧共视邻居上的 SmartFactor（因为 iSAM2 重线性化也会调整这些位姿）
- 传播影响：共视邻居的邻居（共视图 2-hop 内）—— 通常可忽略，因为 iSAM2 逐步重线性化会自行修正

### 6.2 检测受影响的 SmartFactor

```
function detectAffectedSmartFactors(loop_kf1_id, loop_kf2_id):
    affected = set()
    
    // 直接受回环边影响的位姿
    affected_poses = set([loop_kf1_id, loop_kf2_id])
    // + 各自共视邻居（1-hop）
    for kf_id in [loop_kf1_id, loop_kf2_id]:
        affected_poses.union(getCovisibilityNeighbors(kf_id, max_covisibles=10))
    
    // 遍历所有 SmartFactor，检查其观测中是否包含受影响的位姿
    for (lm_id, smart_factor) in all_smart_factors:
        for pose_key in smart_factor.keys():
            frame_id = symbolIndex(pose_key)
            if frame_id in affected_poses:
                affected.add(lm_id)
                break
    
    return affected
```

### 6.3 提升流程

**前置条件**：此函数在回环因子已注入且 `isam2.update()` 传播校正后调用。`isam2.calculateEstimate()` 返回的是校正后的位姿。

```
function promoteSmartFactors(affected_lm_ids):
    // 注：此函数内部不调用 isam2.update()——
    // 新因子和新变量被累积到 new_factors/new_initial 中，
    // 由上层 onLoopResult() 在阶段 5 统一执行第二次 isam2.update()。
    
    for lm_id in affected_lm_ids:
        old_smart = smart_factors[lm_id]
        
        // Step 1: 从当前 iSAM2 estimate 中提取相关位姿
        poses = [isam2.calculateEstimate(X(kf_id)) for kf_id in old_smart.frame_ids()]
        
        // Step 2: 用最新位姿重新三角化
        cameras = composeCameras(poses, body_P_cam)
        new_point_3d = triangulateSafe(old_smart.measurements(), cameras)
        
        if not new_point_3d.valid():
            // 三角化失败：观测几何退化（可能是远点或 view-angle 太小）
            // → 保留为 SmartFactor 等待更多观测，或者丢弃该路标
            continue
        
        // Step 3: 创建显式 GenericStereoFactor 替换 SmartFactor
        landmark_key = L(lm_id)
        
        // 插入路标变量初值
        new_initial.insert(landmark_key, new_point_3d)
        
        // 对每帧观测创建显式重投影因子（注意：使用克隆的原始测量）
        for (kf_id, stereo_px) in old_smart.measurements():
            factor = GenericStereoFactor(
                stereo_px, 
                X(kf_id), landmark_key, 
                stereo_cal, body_P_cam, 
                visual_noise
            )
            new_factors.push_back(factor)
        
        // Step 4: 标记旧 SmartFactor 为已提升
        //  GTSAM ISAM2 不直接支持从 Bayes Tree 中删除因子。
        //  策略选项：
        //  A)（简单）保留旧 SmartFactor 在图中：SmartFactor 在下一次
        //     线性化时会自动重三角化（使用新位姿），与新显式因子
        //     形成冗余约束。冗余虽保守但不错误。
        //  B)（精确）使用 ISAM2::update(graph, init, remove_indices)
        //     的三参数版本，通过 FactorIndex 显式移除旧 SmartFactor。
        //     需要维护 SmartFactor → FactorIndex 的映射。
        //  v1 推荐方案 A（简单+正确）；v2 迁移到方案 B（节省优化开销）。
        smart_factors.erase(lm_id)
        promoted_landmarks.insert(lm_id)
    
    // Step 5: 返回累积的新因子和新变量（由上层 onLoopResult 统一执行 iSAM2 更新）
    return (new_factors, new_initial)
```

**为什么需要两次 iSAM2 更新**：
1. 第一次 `isam2.update(loop_factor)` 传播回环校正到位姿
2. SmartFactor 使用校正后位姿重三角化 → 创建显式因子
3. 第二次 `isam2.update(promoted_factors)` 将显式因子纳入优化

两次更新是必要的：如果先提升再注入回环，则提升使用的位姿是旧值；如果提升和回环在一次 update 中合并注入，则 SmartFactor 重三角化时位姿尚未被回环校正。

### 6.4 为什么 1-hop 邻居覆盖是充分的

回环校正通过 iSAM2 贝叶斯树传播时，位姿调整幅度随图距离衰减：

| 距离 | 典型位姿调整 | SmartFactor 影响 | 处理方式 |
|------|------------|-----------------|----------|
| loop_kf 自身 | 最大（可达 0.5-2m） | 严重过时 | 显式提升 |
| 1-hop 共视邻居 | 中等（~0.1-0.5m） | 三角化偏差 | 显式提升 |
| 2-hop | 较小（< 0.05m） | 在 SmartFactor 容忍度内 | GTSAM 自动重三角化 |
| 3-hop+ | 微小（< 0.01m） | 可忽略 | GTSAM 自动处理 |

1-hop 邻居选择在"覆盖所有显著影响的 SmartFactor"和"避免过度扩大因子图规模"之间取得平衡。

### 6.5 提升策略的边界条件

| 场景 | 处理 |
|------|------|
| SmartFactor 观测数 < 3 | 三角化不可靠 → 丢弃（或保留为 SmartFactor 等待更多观测） |
| 三角化点在相机后方 | `isPointBehindCamera()` → 丢弃 |
| 三角化点太远 | `isFarPoint(distance > 50m)` → 保留为 SmartFactor（远点用逆深度参数化更好） |
| 三角化退化 | `isDegenerate(condition_number > 1e5)` → 保留为 SmartFactor |
| SmartFactor 被多次提升 | 已提升的路标不重复处理 |
| 回环区域外但通过 iSAM2 传播间接影响的 SmartFactor | iSAM2 重线性化自动处理，不需要显式提升 |

### 6.6 为什么不自动提升所有 SmartFactor

1. **性能**：回环可能涉及 100+ SmartFactor，每个都做显式提升会显著增加因子图规模
2. **必要性**：只有回环核心区域（loop_kf 及其直接邻居）的位姿有较大调整。2-hop 以外，iSAM2 的重线性化足以处理
3. **过度提升的代价**：显式路标变量增加了 iSAM2 的状态维度，减慢后续优化

---

## 7. 全局 BA 规范

### 7.1 触发条件

全局 BA 在以下条件之一满足时触发（均在独立线程中运行）：

| 触发条件 | 阈值 | 说明 |
|----------|------|------|
| 累计回环次数 | >= 3 次 | 多次回环后做一次全局精化 |
| 漂移超限 | 当前位姿 vs 最新回环校正位姿差异 > 0.5m | 每个回环后检查 |
| 主动请求 | 用户调用 | 离线模式或地图保存前 |
| 时间间隔 | 距上次 GBA > 30s | 防止 GBA 过于频繁 |

### 7.2 优化变量

```python
# 被优化的变量
optimize:
    all_keyframe_poses[6 DOF]       # 所有关键帧的 SE(3) 位姿
    all_explicit_landmarks[3 DOF]   # 所有已提升为显式因子的路标 3D 坐标

# 不优化的变量
fixed:
    first_keyframe_pose             # 固定第一帧作为坐标系原点
    imu_biases                      # GBA 不优化 bias（bias 由滑动窗口管理）
    velocities                      # GBA 不优化速度
    smart_factor_implicit_points    # SmartFactor 内部参数（不在 GBA 变量中）

# 路径约束（可选，防止 GBA 发散）
optional_prior:
    origin_pose_prior               # 第一帧的弱先验（大协方差）
```

### 7.3 优化配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 求解器 | GTSAM `LevenbergMarquardtOptimizer` | LM 算法 |
| 最大迭代次数 | 50 | 一般 20-30 次收敛 |
| 收敛阈值（相对误差降低） | 1e-5 | 两次迭代间 cost 变化 < 1e-5 * cost |
| 收敛阈值（绝对误差） | 1e-3 | 梯度无穷范数 |
| 鲁棒核 | Huber（k=1.345） | 对所有重投影因子 |
| 重投影噪声模型 | 各向同性 1.0 px | 双目系统 pixel noise |
| 是否线性求解器 | `MULTIFRONTAL_CHOLESKY` | GTSAM 默认，适合稀疏问题 |

### 7.4 GBA 与 iSAM2 的关系

GBA 运行时：
- VIO 的 iSAM2 仍在运行（继续处理新帧）
- GBA 完成后，将优化后的位姿作为 **warm-start** 写回 iSAM2
- iSAM2 下一次 `update()` 使用新的线性化点，后续优化会从 GBA 结果开始

```
function onGlobalBAComplete(gba_result):
    // 将 GBA 结果写回 iSAM2 作为新的初始估计
    for kf_id, optimized_pose in gba_result.poses:
        isam2_new_initial.insert(X(kf_id), optimized_pose)
    
    for lm_id, optimized_point in gba_result.landmarks:
        if lm in promoted_landmarks:
            isam2_new_initial.insert(L(lm_id), optimized_point)
    
    // iSAM2 下次 update 时使用新初值，自动从 GBA 结果开始
    // 注意：不修改已注入的因子，只修改初值
```

### 7.5 GBA 期间的地图规模限制

| 地图规模 | 策略 |
|----------|------|
| < 200 KF | 全量 BA（参考 ORB-SLAM3，GBA 在线程中运行） |
| 200-500 KF | 全量 BA（可能需要 5-15s，仍可在线运行） |
| > 500 KF | 限制为最近 500 KF + 受回环影响的早期 KF（分段 BA） |
| > 2000 KF | 只做位姿图优化（PGO），不做 GBA |

---

## 8. 参数推荐汇总表

### 8.1 DBoW3 相关

| 参数 | 推荐值 | 参考系统 |
|------|--------|----------|
| 特征数 (nfeatures) | 1000 | ORB-SLAM3 |
| 词汇树分支 (k) | 10 | DBoW3 默认, VINS-Fusion brief_k10L6 |
| 词汇树层数 (L) | 6 | 同上 |
| 倒排索引查询数 | 4 | ORB-SLAM3, VINS-Fusion (top-4) |
| 相似度阈值 (minScore) | 0.015 | VINS-Fusion (BRIEF 0.015; ORB 需标定) |
| 最近帧排除 | 50 帧 | VINS-Fusion |
| 一致性组最小成员 | 3 | ORB-SLAM3 `mnCovisibilityConsistencyTh=3` |

### 8.2 PnP RANSAC 相关

| 参数 | 推荐值 | 参考系统 |
|------|--------|----------|
| 最小匹配点数 | 20 (3D-2D), 30 (2D-2D fallback) | ORB-SLAM3 20 for BoW |
| 最小内点数 | 15 | ORB-SLAM3 `nBoWInliers=15` |
| RANSAC 置信度 | 0.99 | ORB-SLAM3 |
| 最大迭代次数 | 300 | ORB-SLAM3 |
| 重投影误差阈值 | 3.0 px | 双目像素噪声 ~1σ, 取 3σ |
| 相对 yaw 上限 | 30° | VINS-Fusion |
| 相对平移上限 | 20 m | VINS-Fusion |
| 邻居一致性最小数 | 3 | ORB-SLAM3 共视邻居确认 |
| 描述子匹配方式 | BoW 直接索引 + Hamming 距离 | ORB-SLAM3 `SearchByBoW` |

### 8.3 PGO 相关

| 参数 | 推荐值 | 参考系统 |
|------|--------|----------|
| 噪声模型类型 | 对角高斯（从 PnP 内点分布估计） | **禁止 Identity** |
| 平移 σ（估计值） | 0.03 - 0.20 m | 标定值，取决于场景/特征分布 |
| 旋转 σ（估计值） | 0.005 - 0.05 rad | 同上 |
| Huber 核 (k) | 1.345 | 标准 Huber，参考 4DRadarSLAM `size=1.0` |
| PGO 自由度 | 6-DOF (SE(3)) | 双目系统尺度可观测 |
| 优化求解器 | iSAM2（增量）/ LM（GBA） | GTSAM |

### 8.4 SmartFactor 提升相关

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 受影响范围 | 回环 KF + 共视邻居（max 10） | 1-hop 提升，2-hop 由 iSAM2 自动处理 |
| 提升最小观测数 | 3 | 少于 3 次观测无法可靠三角化 |
| 最远距离阈值 | 50 m | 超过该距离保留为 SmartFactor |
| 三角化退化阈值 | condition_number > 1e5 | 接近奇异矩阵 |

### 8.5 GBA 相关

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 触发回环数 | >= 3 | 多次回环后全局精化 |
| 漂移触发阈值 | 0.5 m | 回环校正后位姿差异 |
| 最小间隔 | 30 s | 防止 GBA 过于频繁 |
| KF 上限 | 500 | 超过则分段或只做 PGO |
| LM 最大迭代 | 50 | 一般 20-30 收敛 |

---

## 9. 失败模式与恢复策略

| 失败模式 | 概率 | 影响 | 检测方法 | 恢复策略 |
|----------|------|------|----------|----------|
| **假正回环（perceptual aliasing）** | 低（~5%） | 严重：错误约束拉歪全局地图 | 多层验证门限（描述子→PnP→一致性→里程计），任一失败则拒绝 | 错误的回环边不注入；如已注入，通过 Huber 核降低权重 |
| **假负回环（missed loop）** | 中（~15%） | 中等：累积漂移未能消除 | 监视漂移率；如漂移 > 阈值且长时间无回环，降低 BoW 阈值 | 降低 minScore；增加候选数；考虑 ScanContext LiDAR 回环作为补充 |
| **回环区域显式路标不足** | 中（~10%） | 中等：PnP 失败，需回退 2D-2D | `n_explicit_lms < MIN_PNP_MATCHES` | 自动回退 2D-2D 对极几何验证（§4.2 回退路径） |
| **SmartFactor 提升后三角化退化** | 低（~5%） | 低：个别路标丢失 | `triangulateSafe()` 返回失败 | 保留为 SmartFactor；或丢弃（如果观测太少） |
| **iSAM2 回环注入后发散** | 极低（<1%） | 严重：后续优化全部错误 | `iSAM2::update()` 后检查 cost 的增量是否异常 | 移除最近注入的回环边；需要用 GTSAM 的 `removeFactors()` 或重建 ISAM2 |
| **GBA 与 VIO 窗口冲突** | 低（~3%） | 低：GBA 结果过时 | GBA 完成时检查是否有新回环被检测 | 以 GBA 结果为初值，iSAM2 后续更新自动修正 |
| **回环计算阻塞 VIO** | 低 | 高：VIO 频率下降 | LoopDetector 运行时间超过 500ms | LoopDetector 只在独立线程中运行；Backend 不做阻塞等待 |
| **词汇表不匹配（场景迁移）** | 中 | 高：回环检测几乎失效 | 长时间无回环且漂移累积 | 在线更新词汇表；或使用 NetVLAD/SuperPoint 等学习型描述子 |

---

## 10. 线程安全规范

### 10.1 共享数据结构与锁策略

| 数据结构 | 写者 | 读者 | 保护机制 |
|----------|------|------|----------|
| KeyFrame 数据库 | Backend（插入）, LoopDetector（插入 BoW） | LoopDetector | `std::shared_mutex`（读共享，写独占） |
| DBoW3 Database | LoopDetector（每 KF 插入） | LoopDetector | 单线程访问（LoopDetector 内部） |
| ORB 词汇表 | 初始化加载 | LoopDetector | 只读，无需锁 |
| 显式路标 3D 坐标 | Backend（iSAM2 更新后） | LoopDetector（PnP 使用时） | `shared_mutex` 保护路标 map 的读写 |
| ISAM2 内部状态 | Backend（update/回环注入/GBA 写回） | 无外部读者（LoopDetector 只通过 estimate 接口读取） | 单线程（Backend 内），无锁 |
| LoopResult 队列 | LoopDetector（写入） | Backend（读取） | lock-free SPSC 队列 |
| SmartFactor Map | Backend | Backend | 单线程（Backend 内），无锁 |

### 10.2 LoopDetector 只读契约

LoopDetector 在独立线程中运行，对 Backend 状态的访问**全部只读**：

```cpp
class LoopDetector {
    std::shared_mutex& kf_db_mutex_;       // 读锁保护 KF 数据库
    std::shared_mutex& landmarks_mutex_;   // 读锁保护显式路标
    // 注意：不持有任何 Backend 内部状态的写权限
    
    LoopResult detect(KeyFrame* current_kf) {
        // 只读操作：
        shared_lock lk1(kf_db_mutex_);      // 读锁 KF 数据库
        auto candidates = dbow3_database_.query(current_kf->bow_vec);
        
        shared_lock lk2(landmarks_mutex_);  // 读锁路标
        auto pnp_result = geometricVerify(current_kf, candidates);
        
        // 只读操作结束，释放锁
        // 构建 LoopResult（纯数据，不包含指针/引用）
        return LoopResult{pnp_result.T_ij, pnp_result.info, ...};
    }
};
```

### 10.3 Backend 回环注入时序

```
Backend 主循环：

while (running) {
    // ... 正常 VIO 处理 ...
    
    // 在两次 iSAM2::update() 之间处理回环
    if (loop_queue_.try_dequeue(loop_result)) {
        // 1. 读取当前 iSAM2 estimate（只读）
        auto current_estimate = isam2_.calculateEstimate();
        
        // 2. SmartFactor 提升（读写 iSAM2 图，但在单线程内）
        auto affected = detectAffectedSmartFactors(loop_result);
        promoteSmartFactors(affected);
        
        // 3. 注入回环因子
        new_factors_.push_back(buildLoopFactor(loop_result));
        
        // 4. iSAM2 更新（单线程内，原子性保证）
        isam2_.update(new_factors_, new_initial_);
        new_factors_.resize(0);
        new_initial_.clear();
        
        // 5. 发布校正后的估计
        publishCorrectedEstimate(isam2_.calculateEstimate());
    }
}
```

---

## 11. 实现路线图（Phase 6 拆分）

### 11.1 子阶段 6a：ORB 特征提取 + DBoW3 集成

- [ ] 在 Frontend 中实现 `extractORBForLoopDetection()`：独立 ORB 提取器，在关键帧判定后异步计算
- [ ] 在 KeyFrame 中增加 ORB 描述子缓存
- [ ] 引入 DBoW3 库（CMake 可选依赖 `ENABLE_LOOP=ON`）
- [ ] 加载/训练 ORB 词汇表
- [ ] 实现 `LoopDatabase`：DBoW3 数据库管理（insert, query）
- [ ] 单元测试：ORB 特征提取、BoW 向量生成、数据库查询

### 11.2 子阶段 6b：几何验证

- [ ] 实现 `PnPGeometricVerifier::verify()`：3D-2D PnP RANSAC
- [ ] 实现 `EpipolarGeometricVerifier::verify()`：2D-2D 回退路径
- [ ] 实现噪声模型估计器 `computeLoopNoiseModel()`
- [ ] 实现共视邻居一致性检查
- [ ] 实现里程计一致性检查
- [ ] 单元测试：PnP 验证、噪声模型估计、一致性检查

### 11.3 子阶段 6c：LoopDetector 线程

- [ ] 实现 `LoopDetector` 类：独立线程 + lock-free 队列
- [ ] 实现 KF 数据库的 shared_mutex 保护
- [ ] 实现显式路标的 shared_mutex 保护
- [ ] 实现 `LoopResult` 消息结构
- [ ] 集成测试：LoopDetector 持续运行不阻塞 VIO

### 11.4 子阶段 6d：SmartFactor 提升

- [ ] 实现 `detectAffectedSmartFactors()`：基于共视图的影响范围分析
- [ ] 实现 `promoteSmartFactors()`：SmartFactor → GenericStereoFactor 转换
- [ ] 处理边界条件：退化三角化、远点、后方点
- [ ] 单元测试：回环前后 SmartFactor 是否正确提升

### 11.5 子阶段 6e：回环注入 + iSAM2 集成

- [ ] 实现 `injectLoopFactor()`：BetweenFactor 构建与 iSAM2 注入
- [ ] 实现注入前后的 SmartFactor 提升协调
- [ ] 实现异常检测（cost 异常增加 → 拒绝回环）
- [ ] 集成测试：EuRoC 序列上回环后 ATE 降低

### 11.6 子阶段 6f：全局 BA

- [ ] 实现 `GlobalBA::run()`：独立线程 LM 优化
- [ ] 实现 GBA 结果写回 iSAM2
- [ ] 实现 GBA 触发逻辑（累计回环数/漂移超限/定时器）
- [ ] 实现地图规模限制（>500 KF 分段策略）
- [ ] 集成测试：EuRoC 闭合轨迹 GBA 后精度改善

---

## 附录 A：参考系统关键源码锚点

| 功能 | 参考锚点 |
|------|----------|
| ORB-SLAM3 LoopClosing 主循环 | `raw/codes/ORB_SLAM3/src/LoopClosing.cc:L90-L309` |
| ORB-SLAM3 BoW 候选检测 | `raw/codes/ORB_SLAM3/src/KeyFrameDatabase.cc:L604-L730` |
| ORB-SLAM3 Sim3 RANSAC | `raw/codes/ORB_SLAM3/src/Sim3Solver.cc:L1-L489` |
| ORB-SLAM3 回环校正 CorrectLoop | `raw/codes/ORB_SLAM3/src/LoopClosing.cc:L969-L1213` |
| ORB-SLAM3 本质图优化 | `raw/codes/ORB_SLAM3/src/Optimizer.cc` (§6.5) |
| VINS-Fusion DBoW2 词袋初始化 | `raw/codes/VINS-Fusion/loop_fusion/src/pose_graph.cpp:L62-L66` |
| VINS-Fusion detectLoop | `raw/codes/VINS-Fusion/loop_fusion/src/pose_graph.cpp:L335-L417` |
| VINS-Fusion findConnection 几何验证 | `raw/codes/VINS-Fusion/loop_fusion/src/keyframe.cpp:L270-L506` |
| VINS-Fusion optimize4DoF PGO | `raw/codes/VINS-Fusion/loop_fusion/src/pose_graph.cpp:L434-L611` |
| Kimera-VIO LoopClosureDetector | `raw/codes/Kimera-VIO/src/loopclosure/LoopClosureDetector.cpp` |
| Kimera-VIO SmartStereoFactor 管理 | `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L478-L512` |
| GTSAM BetweenFactor | `raw/codes/gtsam/gtsam/slam/BetweenFactor.h` |
| GTSAM ISAM2 | `raw/codes/gtsam/gtsam/nonlinear/ISAM2.h` |

## 附录 B：与 phad_fusion 设计文档的关系

本设计文档详细化了 `phad_fusion_design.md` 中 Phase 6 的以下内容：

| phad_fusion_design.md 规格 | 本文档实现细节 |
|---------------------------|--------------|
| DBoW2/DBoW3 视觉回环 | §3: DBoW3 + ORB 描述子 + 训练方案 |
| 几何验证 | §4: 3D-2D PnP RANSAC + 2D-2D 回退 + 一致性检查 |
| BetweenFactor<Pose3> PGO | §5: 因子构建 + 噪声模型估计 + Huber 核 + ISAM2 注入 |
| 全局 BA 独立线程 | §7: 触发条件 + 优化配置 + iSAM2 写回 |
| Loop Thread 1-5Hz | §2.2: 线程模型 + §10: 线程安全规范 |
| 路标管线的 SmartFactor 交互 | §6: 受影响的 SmartFactor 检测 + 提升策略 |

## 相关页面

- [[方法-视觉回环检测管线]] — ORB-SLAM3 三阶段回环参考
- [[方法-回环验证方法族]] — 广义 N 阶段验证框架
- [[方法-SmartStereoFactor]] — SmartFactor 工作方式与边界
- [[方法-ISAM2增量固定滞后平滑]] — iSAM2 与回环的增量交互
- [[概念-位姿图优化]] — PGO 的理论基础
- [[概念-回环检测方法]] — 回环检测通用概念
- [[组件-DBoW2]] — DBoW2/DBoW3 词袋模型
- [[算法-Kimera-VIO]] — GTSAM + SmartFactor 后端参考
