---
tags: [ISAM2, GTSAM, 固定滞后, 增量平滑, 贝叶斯树, iSAM2, Kimera-VIO, LIO-SAM, SLAM]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/Kimera-VIO/src/frontend/VisionImuFrontend.cpp
  - raw/codes/LIO-SAM/src/mapOptmization.cpp
---

# ISAM2 增量固定滞后平滑

> iSAM2 的 `IncrementalFixedLagSmoother` 是 Kimera-VIO、LIO-SAM 等系统的在线优化引擎。它将全量因子图优化与滑动窗口管理统一在一个框架内：新因子增量加入、旧因子自动边际化、贝叶斯树避免全局重线性化。

## 核心概念

### 与普通滑动窗口的区别

| 特性 | 普通滑窗 (VINS-Fusion) | ISAM2 固定滞后 |
|------|----------------------|----------------|
| 优化引擎 | Ceres LM（每轮全量求解） | GTSAM iSAM2（增量更新） |
| 边缘化 | 手动 Schur 补 + 先验因子 | 贝叶斯树自动边际化 |
| 线性化 | 从头开始（或 warm-start） | 贝叶斯树缓存，只重线性化受影响的枝 |
| 状态管理 | 手动 swap/delete 数组 | `Smoother::update()` 自动管理 |
| 新因子接入 | 重建整个 Problem | `ISAM2::update()` 增量插入 |

### 贝叶斯树基础

ISAM2 将因子图转换为贝叶斯树（Bayes Tree），每个团（clique）对应一组变量的条件概率：

```
因子图:  f1(x0,x1) f2(x1,x2) f3(x2,x3) ...
         │          │          │
贝叶斯树: [x0|x1] → [x1|x2] → [x2|x3] → ...
          团0         团1        团2
```

新因子加入时，ISAM2 只重线性化受影响的团，不触发全局优化。这是其增量特性的核心。

---

## 固定滞后平滑的工作流

```
新帧到达 ──→ 添加因子到图 (ISAM2::update)
                  │
                  ▼
            ISAM2 增量优化（贝叶斯树局部更新）
                  │
                  ▼
            Smoother::update() ──→ 检查时间窗口
                  │
                  ├─ 超出窗口的老状态 → 自动边际化
                  ├─ 新状态加入（状态膨胀）
                  └─ 当前估计返回
```

### 关键 API（GTSAM 4.x）

```cpp
// 创建固定滞后平滑器
gtsam::IncrementalFixedLagSmoother smoother(lag_seconds, isam2_params);

// 每帧调用
gtsam::NonlinearFactorGraph new_factors;
gtsam::Values new_initial;
// ... 添加 IMU 预积分因子、视觉因子、先验因子 ...
smoother.update(new_factors, new_initial, timestamp);

// 获取当前估计
gtsam::Values current_estimate = smoother.calculateEstimate();
```

---

## Kimera-VIO 中的实际使用

**锚点**: `raw/codes/Kimera-VIO/src/backend/VioBackend.cpp`

Kimera-VIO 使用 `IncrementalFixedLagSmoother`，典型参数：
- 窗口大小：25-30 个状态（约 5-10 秒）
- `lag_seconds`: 根据序列调整，Euroc 约 5-10s
- 每帧添加的因子：
  - SmartStereoFactor（双目视觉约束）
  - ImuFactor（IMU 预积分）
  - BetweenFactor（里程计约束）
  - PriorFactor（第一帧先验）

### 与 KF 选择的配合

Kimera-VIO 的 `shouldBeKeyframe()` 中 `max_intra_keyframe_time_ns_` 默认 5.0s：
- 即使视差很小，最大 5s 也必须插入一帧
- 固定滞后平滑器会自动清理超过窗口的老状态
- 因此不需要像 ORB-SLAM3 那样的冗余剔除

---

## LIO-SAM 中的使用

**锚点**: `raw/codes/LIO-SAM/src/mapOptmization.cpp`

LIO-SAM 使用 ISAM2（非固定滞后模式），而是两阶段优化：
1. 前端：scan-to-map LM（高频，~10Hz）
2. 后端：ISAM2 因子图（低频，~1Hz）

因子图包含：
- LiDAR 里程计因子（帧间约束，前端产出）
- IMU 预积分因子
- GPS 因子（可选）
- 回环因子

由于使用全量 ISAM2 而非固定滞后，LIO-SAM 的因子数量会持续增长，但增量特性保证回环后只需局部重线性化。

---

## 与 VINS-Fusion Schur 补边缘化的对比

| 维度 | VINS-Fusion Schur 补 | ISAM2 固定滞后 |
|------|---------------------|---------------|
| 数学本质 | 显式 Schur 消元 | 贝叶斯树条件化 |
| 实现复杂度 | 手动构建 H、求逆、正则化 | GTSAM 内部处理 |
| 线性化一致性 | FEJ（手动保证） | ISAM2 自动保证 |
| 灵活性 | 完全控制边缘化策略 | 按时间戳自动管理 |
| 新因子类型 | 需要定义 drop_set | 自动关联变量 |
| 回环支持 | 困难（先验已固化） | 天然支持（贝叶斯树局部更新） |
| 计算效率 | 轻量（只对一小块矩阵） | 增量高效，但第一次全局优化较慢 |

---

## Agent 实现提示

### 适用场景
当需要为在线 SLAM/VIO 系统选择优化后端时，参考本页帮助决策 ISAM2 vs 手动滑窗边缘化。

### 输入输出契约
- **输入**：新因子（IMU、视觉、先验等）、新变量初值、时间戳
- **输出**：当前状态估计（增量更新后的 Values）

### 实现骨架（伪代码）

```
class FixedLagSmoother:
    smoother = IncrementalFixedLagSmoother(lag_s, isam2_params)
    current_timestamp = 0

    function onNewFrame(factors, initial_values, timestamp):
        // 1. 将新因子和变量注入 ISAM2
        smoother.update(factors, initial_values, timestamp)

        // 2. 内部自动执行：
        //    a) ISAM2::update() — 增量优化，贝叶斯树局部重线性化
        //    b) 边际化超出 lag_s 窗口的旧变量
        //    c) 返回最新 estimate

        // 3. 获取结果
        current_estimate = smoother.calculateEstimate()
        current_timestamp = timestamp
        return current_estimate
```

### 实现注意事项
- `IncrementalFixedLagSmoother` 的 `lag_seconds` 是平滑窗口长度，注意与 KF 选择的 `max_intra_keyframe_time` 区分
- ISAM2 每轮优化前会重线性化受新因子影响的团；如果发现精度退化，可以调大 `relinearizeThreshold` 或 `relinearizeSkip`
- 与手动滑窗不同，ISAM2 边际化老状态不需要手动构建 Schur 补，GTSAM 内部通过贝叶斯树条件化完成
- LIO-SAM 使用全量 ISAM2（不设时间窗），因子积累后回环时 ISAM2 增量更新可以高效重线性化
- 对比 VINS 手动边缘化，ISAM2 的边际化更"黑盒"，但对于回环、重线性化等高级特性的支持更完善

### 源码检索锚点
- Kimera-VIO 后端: `raw/codes/Kimera-VIO/src/backend/VioBackend.cpp` (ISAM2 使用)
- LIO-SAM 后端: `raw/codes/LIO-SAM/src/mapOptmization.cpp` (ISAM2 使用)
- GTSAM ISAM2: `GTSAM::ISAM2` (GTSAM 库)

## 相关页面

[[概念-因子图]] [[组件-GTSAM]] [[方法-滑动窗口边缘化]] [[方法-关键帧选择策略]] [[架构-滑动窗口优化]] [[概念-Schur补与边缘化]] [[算法-Kimera-VIO]] [[算法-LIO-SAM]]
