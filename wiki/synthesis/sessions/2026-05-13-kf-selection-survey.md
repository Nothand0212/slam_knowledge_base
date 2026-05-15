---
tags: [keyframe, KF选择, SLAM, VIO, 调研, phad_slam]
created: 2026-05-13
updated: 2026-05-15
type: synthesis
related: [phad_slam, VINS-Fusion, ORB-SLAM3, OpenVINS, Kimera-VIO]
---

# 关键帧选择策略调研

## 调研动机

phad_slam 实验发现默认 KF 率 72-74% 是精度主导瓶颈，降低到 ~13% 后 13/16 序列改善，但 corridor3/5 回归（需要高 KF 率）。需要设计自适应 KF 策略。本文调研四个代表性 SLAM/VIO 项目的 KF 选择逻辑，提炼可借鉴的设计模式。

## 调研项目

| 项目 | 后端 | KF 方式 | 类比度 |
|---|---|---|---|
| **VINS-Fusion** | Ceres 滑窗 | 视差 + 跟踪守卫 | 高（前端相同 GFTT+LK） |
| **ORB-SLAM3** | g2o 局部/全局 BA | 多条件 OR + 剔除冗余 | 高（phad_slam 参考架构） |
| **Kimera-VIO** | GTSAM iSAM2 固定滞后 | 视差 + 时间 + 特征数 | **最高（同后端）** |
| OpenVINS | MSCKF EKF | 固定 FIFO 滑窗 | 低（滤波架构） |

## 各项目详细分析

### VINS-Fusion

**文件**: `vins_estimator/src/estimator/feature_manager.cpp:52-119`

**决策函数**: `addFeatureCheckParallax()` 每帧调用，返回 bool

**触发条件**（任一满足即为 KF）：

| 条件 | 阈值 | 含义 |
|---|---|---|
| `frame_count < 2` | 2 | 窗口初期强制 KF |
| `last_track_num < 20` | 20 | 跟踪质量下降 |
| `long_track_num < 40` | 40 | 长跟踪特征不足 |
| `new_feature_num > 0.5 * last_track_num` | 50% | 场景突变 |
| `parallax_num == 0` | 0 | 无跨帧特征 |
| `avg_parallax >= MIN_PARALLAX` | **10px** (归一化后 0.022) | 视差足够 |

**关键设计**:
1. 视差在**倒数第二帧与倒数第三帧**之间计算（非当前帧），因为当前帧位姿未优化
2. KF 标志决定边际化策略：KF→删最老帧，非 KF→删次新帧
3. 参数固定，无在线自适应

**KF 率**: ≈ 13-20%（与 phad_slam 降低后的值接近）

### ORB-SLAM3

**文件**: `src/Tracking.cc:3064-3214`

**决策函数**: `NeedNewKeyFrame()` 返回 bool

**触发逻辑**（最终决策表达式）:
```
if (((c1a || c1b || c1c) && c2) || c3 || c4)  →  need KF
```

| 条件 | 含义 | 典型阈值 |
|---|---|---|
| c1a | 帧数 >= mMaxFrames | **fps 值** (10-30) |
| c1b | 帧数 >= mMinFrames && LM 空闲 | **0 帧**（几乎立即） |
| c1c | 跟踪极弱或需要近距离点 | inlier < 25% refMatches |
| c2 | inlier 比例 < thRefRatio | **0.75-0.90**（按传感器类型） |
| c3 (IMU) | 时间 >= 0.5s | 0.5s |
| c4 (IMU mono) | inlier < 75 && > 15 | 75/15 |

**KeyFrameCulling**（`src/LocalMapping.cc:902`）:
- 剔除冗余 KF：若 90%（或 50% for IMU stereo）以上的地图点已被 >=3 个其他 KF 充分覆盖
- IMU 安全守卫：不删除末尾 2 个 KF，检查时间间隔 (<3s) 保证 IMU 预积分安全合并

**关键设计**:

1. **多条件复合决策** — 与 VINS 的单一视差阈值不同，ORB 用 4 组条件覆盖不同场景
2. **参考关键帧概念** — inlier 比例是相对于参考 KF 的地图点数，而非绝对特征数
3. **冗余剔除** — 后处理删除不再提供新信息的 KF，保持地图紧凑
4. **IMU 特定逻辑** — 针对 IMU 初始化前后的不同行为有专门条件
5. **不完全固定** — `thRefRatio` 根据传感器类型和 inlier 数量动态变化（0.40-0.90）

### Kimera-VIO

**文件**: `src/frontend/VisionImuFrontend.cpp:175-232`

**决策函数**: `shouldBeKeyframe()` 每帧调用

**后端**: `gtsam::IncrementalFixedLagSmoother`，窗口 25-30 状态（**与 phad_slam 完全相同**）

**触发条件**（任一满足即为 KF）:

| 条件 | 变量 | 默认值（EuRoC） |
|---|---|---|
| 超过最大时间 | `kf_diff_ns >= max_intra_keyframe_time_ns_` | **5.0s** |
| 超过最大视差 | `disparity > max_disparity_since_lkf_` | 1000px（实质禁用） |
| 视差足够 + 最小时间 | `disparity > 0.5px && kf_diff >= 0.2s` | **0.5px / 0.2s** |
| 视差翻转（刚变低） | `disparity < 0.5px && prev != LOW_DISPARITY` | — |
| 特征太少 | `nr_valid_features <= min_number_features_` | 0（禁用） |
| 第一帧 | `frame.isKeyframe_` | 强制 true |

**关键设计**:

1. **与 phad_slam 同架构** — 都使用 ISAM2 IncrementalFixedLagSmoother，直接可比
2. **视差为主要触发** — 0.5px 阈值 + 0.2s 最小间隔 → 约 5Hz 最大 KF 率
3. **最大时间保护** — 5.0s 确保不丢关键信息（phad_slam 只有 0.5s）
4. **视差翻转检测** — 运动停止时立即插入 KF
5. **未做冗余剔除** — 与 ORB 不同，Kimera 不需要主动剔除，固定滞后自动完成

### OpenVINS

**架构差异**: 使用 MSCKF 滤波而非优化，每帧 clone 到状态向量，FIFO 11 帧滑窗。**没有 KF 选择**——所有帧都成为 clone。SLAM feature 通过跟踪长度 (>11 帧) 选择，而非几何准则。不直接适用于 phad_slam 设计。

## 设计模式总结

### 模式 1：视差驱动（VINS-Fusion, Kimera-VIO）

```
if avg_parallax > threshold && time_since_last_kf > min_interval → KF
```

- 优点：简洁，视差与信息增益直接相关
- 缺点：低纹理/低视差场景 KF 不足，旋转为主时视差小
- 适用于：前端稳定的系统

### 模式 2：质量守卫（VINS-Fusion, ORB-SLAM3）

```
if tracking_quality < guard_threshold → force KF
```

- 质量度量：跟踪特征数（VINS < 20）、inlier 比例（ORB < 0.75）
- 作用：防止跟踪质量低时丢失关键帧，保护关键信息

### 模式 3：时间保护（全部项目）

```
if time_since_last_kf > max_interval → force KF
```

- Kimera: 5.0s，VINS: 无显式上限，phad_slam: **0.5s（太激进）**

### 模式 4：冗余剔除（ORB-SLAM3）

```
if most_points_covered_by_other_kfs → cull
```

- 后处理：BA 完成后检查该 KF 是否提供新信息
- 固定滞后平滑器的边际化自动实现类似效果，但 ORB 有额外安全守卫

## 对 phad_slam 自适应 KF 设计的建议

### 关键差异诊断

| 参数 | phad_slam 原值 | Kimera-VIO | VINS-Fusion | phad_slam 新值 |
|---|---|---|---|---|
| parallax_threshold_px | 10 | 0.5px disparity | 10px | 40 |
| tracking_threshold | 20-30 | 0 (禁用) | 20 | 100 |
| time_threshold_s | 0.5 | 0.2 (min) / 5.0 (max) | 无 | 0.5 |
| KF 率 | 72-74% | ~25% | ~15% | ~13% |

### 推荐设计：三阶段触发 + IMU 自适应

借鉴 Kimera-VIO 和 VINS-Fusion 的组合模式：

**阶段 1 — 质量守卫**（防止信息丢失）
```
if active_tracks < 30 || track_quality_dropped > 40% → force KF
```

**阶段 2 — 视差驱动**（主要触发，类似当前逻辑）
```
if parallax > 40 && time > 0.5s → KF
```

**阶段 3 — 时间保护**（防止过长间隔）
```
if time > max_time → force KF
# max_time 根据 IMU 激励自适应：
#   acc_std < 0.1g → max_time = 2.0s (低动态，走廊等)
#   acc_std >= 0.1g → max_time = 5.0s (高动态，EuRoC V 序列)
```

其中 IMU 加速度标准差作为动态性指标：
- 低动态（走廊慢速运动）：需要更密集的 KF 来约束 IMU 漂移
- 高动态（快速旋转/加速）：IMU 预积分准确，可用稀疏 KF

**关键文件位置**（phad_slam 中）:
- KF 选择: `core/keyframe/keyframe_selector.cpp`
- 前端 tracks: `core/frontend/frontend.cpp`
- IMU 数据: `core/imu/imu_buffer.cpp`

## 参考源码位置

| 项目 | KF 决策核心 |
|---|---|
| VINS-Fusion | `vins_estimator/src/estimator/feature_manager.cpp:52-119` |
| ORB-SLAM3 | `src/Tracking.cc:3064-3214` (选择) + `src/LocalMapping.cc:902` (剔除) |
| Kimera-VIO | `src/frontend/VisionImuFrontend.cpp:175-232` |
| OpenVINS | `ov_msckf/src/core/VioManager.cpp:340-342, 585-596` (FIFO clone) |
