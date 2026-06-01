---
tags: [pitfalls, Factor-VIO, GTSAM, iSAM2, SmartFactor, OpenVINS, ORB-SLAM3, VINS-Fusion]
type: synthesis
created: 2026-06-02
updated: 2026-06-02
sources:
  - wiki/synthesis/factor_vio.md
  - wiki/synthesis/stereo-vio-integrated-architecture.md
  - wiki/synthesis/landmark-pipeline-design.md
  - raw/codes/gtsam/
  - raw/codes/Kimera-VIO/
  - raw/codes/open_vins/
  - raw/codes/ORB_SLAM3/
  - raw/codes/VINS-Fusion/
---

# Factor-VIO 实现踩坑清单

> 汇总官方文档、源码锚点、AnySearch 网络检索与已关闭/已讨论 GitHub issues，对 Factor-VIO 的实现约束进行反向加固。

## 一、GTSAM SmartFactor 踩坑

### 1. SmartFactor 不是 Robust factor 容器

**证据**：`SmartFactorBase` 构造函数要求传入的噪声模型能 `dynamic_pointer_cast` 为 `noiseModel::Isotropic`，否则抛出 `SmartFactorBase: needs isotropic`。

- `raw/codes/gtsam/gtsam/slam/SmartFactorBase.h:L66-L73` — `SharedIsotropic noiseModel_`
- `raw/codes/gtsam/gtsam/slam/SmartFactorBase.h:L100-L115` — 构造时强制 isotropic
- `raw/codes/gtsam/gtsam/linear/NoiseModel.h:L726-L753` — Robust noise model 是另一套包装机制

**设计结论**：

- `SmartStereoProjectionPoseFactor` 只能作为结构化/隐式几何试用因子。
- 需要 Huber/Cauchy/Tukey、逐观测降权、后验 chi² remediation 时，必须晋升为显式 `GenericStereoFactor<Pose3, Point3>`。
- 不要尝试把 `noiseModel::Robust` 包在 SmartFactor 上，也不要误以为 SmartFactor 的 `dynamicOutlierRejectionThreshold` 等价于鲁棒核。

### 2. SmartFactor 退化时必须 zero-on-degeneracy，而不是让异常传播

**证据**：

- `raw/codes/gtsam/gtsam/slam/SmartFactorParams.h:L34-L37` — `IGNORE_DEGENERACY / ZERO_ON_DEGENERACY / HANDLE_INFINITY`
- `raw/codes/gtsam/gtsam_unstable/slam/SmartStereoProjectionFactor.h:L217-L227` — `ZERO_ON_DEGENERACY` 下返回空 Hessian
- `raw/codes/gtsam/gtsam/geometry/triangulation.h:L644-L676` — `VALID / DEGENERATE / BEHIND_CAMERA / OUTLIER / FAR_POINT`

**设计结论**：SmartFactor shadow 期间必须记录 `valid/degenerate/outlier/farPoint/behindCamera` 状态；退化因子只能不贡献信息，不能污染主图。

### 3. SmartFactor 不能跨边缘化无约束存活

**GitHub issue 线索**：

- GTSAM `#1976` — SmartFactor 与 `IncrementalFixedLagSmoother` 边缘化交互会触发 `VariableIndex` 断言。
- GTSAM `#595` — 空测量 SmartFactor 可能触发 elimination 相关异常。

**设计结论**：

- 被边缘化的 pose key 仍被 SmartFactor 引用时，必须删除/重建 SmartFactor，或提前晋升为显式因子。
- `GraphMutationPreflight` 必须检查 SmartFactor 的 `keys().size() >= 2`、相关 pose key 仍在 active smoother 中、slot 删除与新因子添加同批提交。

## 二、GTSAM iSAM2 图 mutation 踩坑

### 1. `ValuesKeyDoesNotExist` 通常是批量提交次序错误

**GitHub issue 线索**：

- GTSAM `#301` / `#2405` — Dogleg/iSAM2 在新变量加入时可能因 `theta_` 尚未包含新 key 而访问缺失值。
- GTSAM `#1179` / `#1688` — `IndeterminantLinearSystemException` 常见原因是欠约束变量或 missing edge。

**设计结论**：

- 每次 `isam2.update()` 前，`new_factors` 引用的所有 key 必须属于 `current_values ∪ new_values`。
- 新 `L(id)` 必须和引用它的第一批显式因子同批提交。
- 任何 preflight/update 失败只允许整批 rollback 或 shadow-only，禁止发布部分估计。
- 默认优化器使用 Gauss-Newton；Dogleg 只能在专门回归验证后启用。

### 2. 因子槽位不是 LandmarkId

**证据**：Kimera-VIO 的 `convertSmartToProjectionFactor` 明确把旧 SmartFactor 的 `FactorIndex slot` 放入 delete list，再插入新的显式投影因子。

- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L635-L670` — SmartFactor 有效点检查与 `L(id)` 初值插入
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L671-L710` — 为历史观测创建显式 projection factors 并登记旧 slot 删除
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L712-L730` — 从 smart-factor 管理表删除已转换路标，失败时返回 false

**设计结论**：

- `delete_slots` 只能来自 iSAM2 返回的 `FactorIndex`。
- 不允许把 `LandmarkId`、数组下标、`newFactorsIndices` 的局部序号混用。
- slot 找不到时必须 abort mutation，不允许猜测转换。

## 三、OpenVINS 经验：时间戳、特征门控和 chi²

### 1. 相机时间必须先转换到 IMU 时间系

**证据**：OpenVINS 在处理相机消息时使用 `timestamp_imu_inC = message.timestamp - calib_dt_CAMtoIMU`。

- `raw/codes/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp:L474-L477`
- `raw/codes/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:L474-L477`

**设计结论**：Factor-VIO 的 frontend/backend 不能混用 camera timestamp 和 IMU timestamp。进入 IMU propagation / preintegration 前必须统一到 IMU 时间系。

### 2. 没有足够 IMU 样本时不能硬崩溃

**证据**：

- `raw/codes/open_vins/ov_msckf/src/state/Propagator.cpp:L269-L280` — IMU reading selection 入口与空 IMU 数据告警
- `raw/codes/open_vins/ov_msckf/src/state/Propagator.cpp:L344-L351` — 没有可传播 IMU 样本时返回空
- `raw/codes/open_vins/ov_msckf/src/state/Propagator.cpp:L354-L393` — 缺失/零 dt IMU 样本的补偿、清理与最终样本数检查

**GitHub issue 线索**：OpenVINS `#525`、`#70` 均围绕相机-IMU 时间区间内 IMU 样本不足。

**设计结论**：Factor-VIO 必须提供 `NO_IMU_INTERVAL` 安全路径：本帧不创建 IMU 因子、不 clone 新状态，或降级为视觉-only 预测；绝不能对空 IMU span 强行预积分。

### 3. chi² 是带协方差的后验 gating，不是入图前裸残差阈值

**证据**：OpenVINS 的 MSCKF 更新先 nullspace project，再用 `S = HPHᵀ + R` 计算 Mahalanobis chi²。

- `raw/codes/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp:L202-L225`

**设计结论**：Factor-VIO 只能在 `isam2.update()` 后、拥有后验状态和合理协方差时做硬裁决；入图前最多记录 soft score。

## 四、ORB-SLAM3 stereo-inertial 经验

### 1. 重置循环多半来自 IMU 初始化、外参、相机模型或同步问题

**GitHub issue 线索**：ORB-SLAM3 `#284/#305/#406/#929` 中，Stereo-Inertial 模式反复 `Reset active map`，常见原因是 `Tbc/Tcb` 坐标约定错误、鱼眼相机却配置为 PinHole、IMU packet 与图像帧不同步或激励不足。

**源码锚点**：

- `raw/codes/ORB_SLAM3/src/LocalMapping.cc:L40-L74` — `mbBadImu` 会阻断 LocalMapping 处理新关键帧
- `raw/codes/ORB_SLAM3/src/LocalMapping.cc:L124-L134` — inertial BA 前依赖前序关键帧链和位移

**设计结论**：启用 IMU 初始化前必须做 `ExtrinsicAudit`：

- `T_body_cam` / `T_cam_body` 方向明确。
- 平移范数符合物理基线。
- 相机模型与真实镜头一致（pinhole / fisheye）。
- IMU 频率至少显著高于相机频率，且每帧区间有足够 IMU 样本。

### 2. Local BA / VI BA 需要图连通性下限

**GitHub issue 线索**：ORB-SLAM3 `#779` 报告 `LocalInertialBA` 中视觉边不足导致断言失败。

**设计结论**：Factor-VIO 在触发 Global BA / post-loop promotion / local refinement 前必须检查：

- 每个参与 KF 至少有最小视觉约束数。
- loop 引入的新边不会让孤立 key 进入优化。
- 若显式路标不足，保持 `SAFE_DEGRADED` 或视觉-only fallback。

## 五、VINS-Fusion 经验：同步、IMU 单位和 NaN

### 1. 时间偏移是状态/配置的一部分

**证据**：VINS-Fusion 读取 `td` 与 `estimate_td`，可在线估计 camera-IMU 时间偏移。

- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/parameters.cpp:L181-L186`

**GitHub issue 线索**：VINS-Fusion `#41` 明确指出 stereo+IMU 需要毫秒级同步；同步差时 IMU 反而让结果更差。

**设计结论**：Factor-VIO 必须记录并监控 `time_offset_cam_imu_ms`、`timestamp_jitter_ms`、`imu_samples_per_frame`；超阈值时降级为 stereo-only 或强制 shadow。

### 2. IMU 噪声、单位和重力范数必须启动时体检

**证据**：VINS-Fusion 配置读取 `acc_n/acc_w/gyr_n/gyr_w/g_norm`。

- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/parameters.cpp:L92-L103`

**GitHub issue 线索**：VINS-Mono/Fusion 中多次出现 IMU 单位错误（deg/s vs rad/s，g vs m/s²）和噪声参数过小导致 drift。

**设计结论**：启动时必须做 IMU sanity check：

- 静止段 `|acc| ≈ 9.81 ± 1.0 m/s²`。
- 静止段 `|gyro|` 不应明显超过 0.1 rad/s。
- 噪声密度不应直接复用 EuRoC，必须来自 datasheet/Kalibr/Allan variance。

### 3. stereo-only 与 stereo+IMU 初始化路径不同，不能混用 gate

**证据**：VINS-Fusion 分别处理 stereo+IMU 和 stereo-only 初始化。

- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L480-L506` — stereo+IMU：PnP、三角化、陀螺 bias、重传播、优化
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L508-L523` — stereo-only：PnP、三角化、优化

**设计结论**：Factor-VIO 的初始化 state machine 必须区分 stereo-only fallback 和 stereo-inertial nominal。IMU 体检失败时，不要让坏 IMU 因子进入主图。

## 六、Factor-VIO 必须新增/保留的 Guardrails

| Guardrail | 触发 | 动作 |
|-----------|------|------|
| `SmartFactorRobustnessAudit` | SmartFactor 被配置为 robust noise 或需要逐观测 Huber | 阻断；要求显式路标路径 |
| `ISAM2BatchPreflight` | 任意 update 前 | 检查 key/value/slot/delete indices/孤立变量 |
| `TimeSyncAudit` | 每帧图像进入 propagation 前 | 验证 IMU 时间系、样本数、dt 非零 |
| `ExtrinsicAudit` | 初始化前、数据集切换时 | 验证 `T_body_cam`、相机模型、物理基线、重投影误差 |
| `IMUSanityAudit` | 启动和每段静止窗口 | 验证单位、重力范数、gyro bias、噪声配置 |
| `StereoObservationAudit` | 入路标管线前 | 检查 `uL/uR/v` 有限值、视差符号、右目匹配失败不删除左目 track |

## 七、相关页面

- [[factor_vio]]
- [[stereo-vio-integrated-architecture]]
- [[landmark-pipeline-design]]
- [[架构-GTSAM iSAM2 双目VIO后端设计]]
- [[方法-SmartStereoFactor]]
- [[2026-05-18-phad-frontend-pitfalls]]
