---
created: 2026-06-01
updated: 2026-06-01
type: synthesis
tags: [stereo-vio, factor-graph, GTSAM, iSAM2, SmartFactor, 方案设计]
sources:
  - wiki/synthesis/stereo-vio-integrated-architecture.md
  - wiki/entities/设计-立体VIO前端管线.md
  - wiki/entities/架构-GTSAM iSAM2 双目VIO后端设计.md
  - wiki/entities/设计-双目VIO初始化子系统.md
  - wiki/entities/设计-双目VIO回环子系统.md
  - raw/codes/Kimera-VIO/src/backend/VioBackend.cpp
  - raw/codes/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp
  - raw/codes/Kimera-VIO/src/frontend/Tracker.cpp
  - wiki/synthesis/landmark-pipeline-design.md
---

# Factor-VIO：基于因子图的立体视觉惯性里程计方案

> 设计动机：PHAD SLAM 从 PFB 后端迁移到 SmartFactor-only 后，EuRoC V1_01 RMSE 从 0.09m 退化到 1.81m（20×）。通过对 Kimera-VIO 源码的逐层审计，本文提炼出完整的立体 VIO 设计方案及工程陷阱清单。

---

## 一、设计动机：一个失败案例的解剖

### 1.1 退化数据

```
e0d9cca (PFB 后端):        V1_01 = 0.092m, V2_03 = 0.320m
0545116 (删除 PFB):        SmartFactor 成为唯一视觉后端
111e759 (已退化):          V1_01 = 1.784m
HEAD:                      V1_01 = 1.811m
```

### 1.2 根因

SmartStereoProjectionPoseFactor 的路标是隐式变量——3D 位置不进入优化状态，每次从位姿瞬时重算。当系统缺少以下任何一项时，隐式三角化发散：

1. **极紧的首帧先验**（位置 σ=1e-5 m，Kimera 级别）——给 SmartFactor 提供稳定参考系
2. **双轮 RANSAC 前端过滤**——2D-2D + 3D-3D 累积剔除，阻止错误观测进入 SmartFactor
3. **IMU 预积分强约束**——正确的噪声参数 + bias random walk 因子，位姿不会漂太远
4. **updateSmoother 异常恢复**——14 种异常捕获 + 备份回滚 + Cheirality 递归修复

### 1.3 设计原则

1. **SmartFactor 是可靠的，但前提是它的输入是干净的、参考系是稳定的。**
2. **每个模块的保护机制不是可选的——删掉任何一层，系统都会在某个场景下崩溃。**
3. **参数值不能拍脑袋。Kimera-VIO 的每个阈值都有其物理或统计依据。**

---

## 二、系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        三线程架构                            │
├───────────────┬───────────────────┬─────────────────────────┤
│  Tracking     │   Local Mapping   │   Loop Closing          │
│  (每帧,~15ms) │   (每关键帧,~40ms)│   (后台,~100ms)         │
├───────────────┼───────────────────┼─────────────────────────┤
│ KLT + IMU预测  │ SmartFactor 试用期 │ DBoW3 ORB 检测          │
│ NCC 立体匹配   │ iSAM2.update()    │ PnP 几何验证            │
│ 双轮 RANSAC    │ Post-update chi2  │ BetweenFactor 注入      │
│ 关键帧决策     │ Cheirality 恢复   │ Post-loop 路标提升      │
│ 特征老化淘汰   │ 延迟边缘化        │                         │
└───────────────┴───────────────────┴─────────────────────────┘
```

**后端范式**：GTSAM IncrementalFixedLagSmoother (iSAM2)，因子图统一表达所有约束。

---

## 三、逐层设计规范

### 3.1 前端

#### 特征跟踪

| 参数 | 值 | 来源 |
|------|-----|------|
| 检测器 | Shi-Tomasi (GFTT) | Kimera-VIO `FeatureDetector.cpp` |
| qualityLevel | 0.001（先大量提取） | Kimera GFTT |
| 目标特征数 | 300/帧 | Kimera Euroc |
| 原始候选数 | 2000（ANMS 前） | Kimera `max_nr_keypoints_before_anms_` |
| KLT 窗口 | 21×21 px | VINS-Fusion |
| KLT 金字塔 | 3 层（有 IMU 预测时 1 层） | OpenVINS |
| KLT 最大迭代 | 30 | 三者一致 |
| KLT ε | 0.01 | 三者一致 |
| 双向光流验证 | 开启，阈值 0.5px | VINS-Fusion 独有 |
| **特征年龄上限** | **25 关键帧** | Kimera `max_feature_track_age_` |

#### 立体匹配

| 参数 | 值 | 来源 |
|------|-----|------|
| 模板尺寸 | 101×11 px（沿极线） | Kimera `StereoMatcher.cpp` |
| NCC 方法 | CV_TM_SQDIFF + normalize | Kimera |
| NCC 阈值 | 0.15 | Kimera `tolerance_template_matching_` |
| 深度范围 | [0.3, 15.0] m | Kimera（保守化） |
| 双向匹配 | 可选（Kimera 默认关） | — |

#### 双轮 RANSAC（关键设计）

```
流程：KLT 跟踪 → 2D-2D RANSAC → 立体匹配 → 3D-3D RANSAC → 后端

第一轮：2D-2D 单目 RANSAC
  算法：2-point (IMU 旋转已知) 或 5-point Nistér
  阈值：ransac_threshold_mono = 1.0e-6（bearing vector 余弦空间）
  最少内点：10
  外点处理：永久标记 landmark = -1

第二轮：3D-3D 立体 RANSAC（仅对第一轮幸存者执行）
  算法：1-point voting (IMU 旋转已知) 或 3-point Arun
  阈值：ransac_threshold_stereo = 1.0（马氏距离平方，3 DOF）
  最少内点：5
  外点处理：⚠️ 降级为单目（uR=NaN），不删除！
  → SmartFactor 收到 StereoPoint2(uL, NaN, v) 退化为单目观测
```

**为什么降级不删除**：在 MH 等低纹理场景中，NCC 立体匹配错误率显著升高。如果直接删除立体外点，可能丢失 30-50% 的观测。降级保留了方向约束（2D bearing）。

#### 关键帧决策

| 条件 | 参数 | 值 |
|------|------|-----|
| 最小时间间隔 | `min_intra_keyframe_time` | 0.2 s |
| 最大时间间隔 | `max_intra_keyframe_time` | 5.0 s |
| 最小视差 | `disparity_threshold` | 0.5 px |
| 最大平移 | `max_translation` | 0.5 m |
| 最大旋转 | `max_rotation` | 15° |
| 最小特征数 | `min_features` | 20 |
| 特征丢失率 | `feature_dropout_ratio` | 0.5 |

#### IMU 预测

光学流预测器：`H = K · R^T · K^{-1}`（纯旋转 homography），将 bearing vector 从上一帧旋转后投影到当前帧作为 KLT 初始值。

#### 特征检测时机

**仅在关键帧时检测新特征**。非关键帧仅做 KLT 跟踪。

#### ANMS/Binning 空间分布

Kimera Euroc 使用 Binning（7×5 grid），先检测 2000 个候选再按 response 排序选取。优于简单 grid mask + minDistance。

---

### 3.2 初始化

#### Kimera-VIO 做法（推荐）

静态 IMU → 估计重力方向 + 初始 bias → 设置极紧先验 → **从 KF=1 立即启用 SmartFactor**。

```cpp
// 首帧先验（Kimera-VIO 精确值，VioBackendParams.h:L110-L115）
double initialPositionSigma_   = 1e-5;      // ← 关键：极紧！
double initialRollPitchSigma_  = 1e-5;      // (YAML 覆盖为 0.1745)
double initialYawSigma_        = 1.75e-3;
double initialVelocitySigma_   = 1e-3;
double initialAccBiasSigma_    = 0.1;
double initialGyroBiasSigma_   = 0.01;
```

**先验松紧的症状**：

| 参数 | 过松 | 过紧 |
|------|------|------|
| 位置 σ > 0.01m | SmartFactor 三角化跑到相机后方 | 初始化后无法修正 |
| 速度 σ > 0.1m/s | IMU 传播偏，bias 收敛慢 | 静止初始化后速度不更新 |

#### 双路径初始化

| | 静态初始化 | 动态初始化 |
|---|----------|----------|
| 条件 | IMU 方差 < 1.0 m/s²，持续 1s | 静态失败，视觉跟踪正常 |
| 输出 | 重力方向 + bg + ba + v=0 | 重力方向 + bg + ba + velocities |
| 参考 | OpenVINS StaticInitializer | ORB-SLAM3 InertialOptimization（简化版） |
| 双目优势 | 基线提供绝对尺度，跳过 SfM | 同左 |

---

### 3.3 因子图后端

#### 状态变量（每关键帧）

| 符号 | 类型 | 维度 |
|------|------|------|
| `X(k)` | `Pose3` | 6 DOF |
| `V(k)` | `Vector3` | 3 DOF |
| `B(k)` | `imuBias::ConstantBias` | 6 DOF |
| `L(id)` | `Point3` | 3 DOF（仅显式路标） |

#### 因子注入顺序（强制约束）

```
1. SmartFactor 列表     ← 必须先于 IMU！slot 恢复依赖 1:1 对应
2. IMU 预积分因子
3. Bias random walk 因子（如果用 ImuFactor 而非 CombinedImuFactor）
4. 显式视觉因子（GenericStereoFactor + Point3）
5. 回环因子（可选）
6. 其他先验因子
```

#### SmartFactor 槽位管理

```cpp
// Kimera-VIO optimize() 模式 (VioBackend.cpp:L1069-L1112)
delete_slots = [];
for each new_smart_factor:
    old_slot = old_smart_factor_slots[lmk_id];
    if (old_slot != -1):              // 已在图中 → 删除旧 + 添加新
        delete_slots.push(old_slot);
        new_factors.push(new_sf);
    else:                              // 首次入图
        new_factors.push(new_sf);
new_factors.push(imu_and_priors...);   // SmartFactor 已全在前面
result = smoother->update(new_factors, new_values, timestamps, delete_slots);
// 从 result.newFactorsIndices 恢复 slot 映射（1:1 对应前 N 个 SmartFactor）
```

#### updateSmoother 异常恢复

```
updateSmoother(new_factors, new_values, timestamps, delete_slots):
    backup = shallow_copy(smoother)    // 浅拷贝备份 iSAM2 状态
    try:
        return smoother->update(...)
    catch IndeterminantLinearSystem:
        smoother = backup
        注入 6 个 PriorFactor(first_key, nearby_var 的 pose/vel/bias)
        重试
    catch CheiralityException:
        counter++ (< 5)
        smoother = backup
        cleanCheiralityLmk(lmk)  // 删除该路标的所有因子
        递归调用 updateSmoother()
    catch (其他 12 种):
        打印诊断 → return false
```

#### IMU 因子类型选择

| | ImuFactor | CombinedImuFactor |
|---|----------|------------------|
| 连接变量 | `X(i-1),V(i-1),X(i),V(i),B(i-1)` | `X(i-1),V(i-1),B(i-1),X(i),V(i),B(i)` |
| bias walk | 需手动加 `BetweenFactor<ConstantBias>` | 内嵌 |
| Kimera Euroc | ✅ 默认 | — |

**如果使用 ImuFactor 但遗漏 `BetweenFactor<ConstantBias>`，bias 会飘。**

#### iSAM2 参数

| 参数 | 值 |
|------|-----|
| optimizer | GaussNewton |
| factorization | CHOLESKY |
| relinearizeThreshold | 0.01 |
| relinearizeSkip | 1 |
| wildfireThreshold | 0.001 |
| cacheLinearizedFactors | true |
| findUnusedFactorSlots | true |
| smoother lag (nr_states) | 25-30 |

---

### 3.4 回环

| 组件 | 选择 |
|------|------|
| 检测 | DBoW3 + ORB（仅关键帧提取） |
| 验证 | 3D-2D PnP RANSAC（min 15 inliers） |
| 约束 | `BetweenFactor<Pose3>` 注入 iSAM2 |
| 噪声模型 | 从 PnP 内点分布估计（禁止 Identity） |
| 时序 | 回环注入 → isam2.update() → 提升受影响的 SmartFactor → isam2.update() |

---

## 四、坐标系约定与参数命名陷阱

### 4.1 `T_a_b` 的含义

```
T_a_b = 将点从坐标系 b 变换到坐标系 a 的刚体变换
T_w_b: body → world
T_b_cam: camera → body（相机在 body 系中的位姿）
```

**GTSAM `SmartStereoProjectionPoseFactor` 的 `body_P_sensor` = `T_body_camera`**。

如果传成 `T_camera_body`（反了），相机位置差一个符号，所有三角化点跑到相机后方。

**EuRoC 典型值**：`T_b_cam = Pose3(Rot3(), Point3(0.05, 0.0, 0.0))`（左相机在 IMU 前方 5cm）。

### 4.2 `Cal3_S2Stereo` 构造

```cpp
// 六参数: (fx, fy, skew, u0, v0, baseline)
// 四参数便捷版: (fx, u0, v0, baseline) — 假设 fx=fy, skew=0

// ❌ 易错：把 cx 当成 fy
Cal3_S2Stereo(K(0,0), K(0,2), K(1,2), baseline)
//             fx      cx      cy      b
// 四参数版本解释：fx    u0      v0      b  ← 正确
```

**验证**：构造后打印 `fy()` 和 `px()`，确保 `fy ≈ fx`。

### 4.3 先验 σ 量级

| 参数 | Kimera 值 | 常见错误值 | 症状 |
|------|----------|-----------|------|
| 位置 σ | 1e-5 m | 0.05 m | SmartFactor 三角化跑到相机后方 |
| 速度 σ | 1e-3 m/s | 0.1 m/s | 初期 IMU 传播偏 |
| roll/pitch σ | 1e-5 rad | 随意 | 重力方向漂移 |

### 4.4 重力方向

所有系统必须在 **ENU (Z-up)** 和 **NED (Z-down)** 之间保持统一：
- VINS-Fusion: ENU
- Kimera-VIO: `n_gravity = [0, 0, -9.81]`（Z-up 世界系中重力向下）
- GTSAM `MakeSharedU`: ENU
- GTSAM `MakeSharedD`: NED

不统一 → 初始化计算的重力方向、尺度、偏置全错。

---

## 五、与参考系统的逐模块对应

| 模块 | 首选参考 | 关键教训 |
|------|---------|---------|
| 前端特征跟踪 | Kimera-VIO `Tracker.cpp` | 双轮 RANSAC（2D+3D），特征年龄上限 25，仅 KF 检测 |
| 立体匹配 | Kimera-VIO `StereoMatcher.cpp` | NCC 101×11 模板，阈值 0.15，深度 [0.3,15]m |
| 关键帧决策 | Kimera-VIO `VisionImuFrontend.cpp` | 视差 >0.5px + 时间 >0.2s |
| 初始化 | OpenVINS + Kimera | 首帧先验极紧（1e-5），从 KF=1 用 SmartFactor |
| IMU 预积分 | Kimera `ImuFrontend.cpp` | ImuFactor + 独立 BetweenFactor<ConstantBias> |
| 因子图后端 | Kimera `VioBackend.cpp` | updateSmoother 14 异常捕获 + 备份回滚 |
| SmartFactor 管理 | Kimera `VioBackend.cpp` L1069-L1112 | Clone-and-add + slot 管理 |
| 异常值剔除 | ORB-SLAM3 + Kimera | chi² 硬阈值 7.815（χ²₃, p=0.05） |
| 回环检测 | ORB-SLAM3 `LoopClosing` | DBoW3+ORB+PnP 验证+共视邻居确认 |

---

## 六、实现检查清单

### 前端

- [ ] 双轮 RANSAC：2D-2D 后接 3D-3D
- [ ] 3D-3D 外点降级为单目，不删除
- [ ] 特征年龄上限 `max_feature_track_age = 25`
- [ ] 仅关键帧检测新特征
- [ ] ANMS/Binning 空间分布
- [ ] IMU 旋转预测 KLT 初始值
- [ ] 双向光流验证

### 初始化

- [ ] 首帧位置先验 σ ≤ 1e-4 m
- [ ] 首帧速度先验 σ ≤ 1e-2 m/s
- [ ] `T_b_cam` 方向正确（`T_body_camera`，非 `T_camera_body`）
- [ ] `Cal3_S2Stereo` 构造参数正确（验证 `fy ≈ fx`）
- [ ] 重力方向统一为 ENU

### 后端

- [ ] SmartFactor 先于 IMU 因子注入
- [ ] Clone-and-add + slot 管理
- [ ] updateSmoother 异常恢复（至少 Indeterminant + Cheirality）
- [ ] 如果用 ImuFactor，需配 BetweenFactor<ConstantBias>
- [ ] iSAM2 `cacheLinearizedFactors=true` + `findUnusedFactorSlots=true`
- [ ] Post-update chi² 异常值剔除

---

## 七、相关页面

- [[stereo-vio-integrated-architecture]]
- [[设计-立体VIO前端管线]]
- [[架构-GTSAM iSAM2 双目VIO后端设计]]
- [[设计-双目VIO初始化子系统]]
- [[设计-双目VIO回环子系统]]
- [[landmark-pipeline-design]]
- [[VIO方案全景对比]]
- [[因子图vs滤波]]
- [[概念-Schur补与边缘化]]
- [[概念-IMU预积分]]
- [[phad_fusion设计总结]]
