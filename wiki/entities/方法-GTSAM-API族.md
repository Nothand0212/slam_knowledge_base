---
type: entity
tags: [GTSAM, API, 因子图, SLAM, 优化, 预积分, 视觉, Lie群, 自定义因子]
created: 2026-05-15
updated: 2026-05-15
sources:
  - https://borglab.github.io/gtsam/
  - https://gtsam.org/doxygen/
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

# GTSAM API 族

> GTSAM 所有 Python/C++ API 的统一入口：Geomtry 类型、Factor Graph 构建、Optimizer/ISAM2、视觉/Navigation factor 与自定义因子。本文是 [[GTSAM API 使用索引]]、[[GTSAM Geometry API]]、[[GTSAM Nonlinear 优化 API]]、[[GTSAM SLAM 与视觉因子 API]]、[[GTSAM Navigation 与 IMU API]]、[[GTSAM 自定义因子与 Jacobian]]、[[GTSAM 因子图工作流]] 七篇的归并。

## 概述

GTSAM（Georgia Tech Smoothing And Mapping）是一个基于因子图的非线性优化库，专为 SLAM/SfM/传感器融合设计。其架构哲学围绕四个核心抽象：

| 抽象 | 角色 | Python 类型 |
|------|------|-------------|
| **Factor Graph** | 容纳所有因子（约束、先验），定义优化问题 | `NonlinearFactorGraph` |
| **Values** | 所有变量的当前估计，key → manifold value | `Values` |
| **Noise Model** | 定义因子残差的协方差/权重 | `noiseModel.Diagonal`、`Isotropic` |
| **Optimizer** | 反复线性化→求解→retract，逼近 MAP 估计 | `LM`、`GN`、`Dogleg`、`ISAM2` |

数学上，优化问题写为：

$$X^* = \arg\min_X \sum_i \| h_i(X) - z_i \|_{\Sigma_i}^2$$

GTSAM 将每个 measurement 建模为一个 `NonlinearFactor`，残差在 tangent space 上线性化，优化器对 `Values` 反复 retract 增量直至收敛。

**关键设计决策**：
- 所有 Lie group 变量使用**右扰动**（right perturbation）约定：$x_{new} = x \cdot \text{Exp}(\delta)$
- Python pybind11 wrapper 暴露 C++ API，自定义因子通过 `CustomFactor` callback 实现
- ISAM2 将因子图转换为贝叶斯树实现增量平滑，避免每次全局重线性化

## API 模块矩阵

| 模块 | 核心类 | 典型用法 | SLAM 角色 |
|------|--------|----------|-----------|
| **Geometry** | `Pose2`, `Pose3`, `Rot3`, `Cal3_S2`, `StereoCamera` | 定义状态变量类型，manifold 操作 | 优化变量（位姿、路标、内参） |
| **Factor Graph Core** | `NonlinearFactorGraph`, `Values`, `Key`, `noiseModel` | 建图、插入初值、绑定噪声 | 问题表达层 |
| **Optimization** | `LevenbergMarquardtOptimizer`, `GaussNewtonOptimizer`, `DoglegOptimizer` | batch 离线优化 | 一次性求解 |
| **Incremental** | `ISAM2`, `Marginals` | 增量在线优化、covariance 查询 | 实时 SLAM |
| **SLAM/Vision** | `PriorFactor`, `BetweenFactor`, `GenericProjectionFactor`, `SmartProjectionFactor`, `GenericStereoFactor3D` | 先验、里程计、重投影、双目 | 视觉/激光前端约束 |
| **Navigation/IMU** | `NavState`, `PreintegrationParams`, `PreintegratedImuMeasurements`, `CombinedImuFactor`, `GPSFactor` | IMU 预积分、GNSS 融合 | VIO/INS 后端 |
| **Custom** | `CustomFactor` | 自定义残差与雅可比 | 任意传感器/约束 |

## 各模块详解

### 1. Geometry — Lie Group / Manifold 类型

Geometry 模块提供优化变量类型，均实现 `Manifold`、`LieGroup` concept：

- `retract(delta)`：tangent vector → manifold
- `localCoordinates(other)`：manifold 差 → tangent vector
- `compose` / `inverse` / `between`：群运算

#### Pose2（2D 位姿）

```python
from gtsam import Pose2, Rot2, Point2
p = Pose2(1.0, 2.0, 0.3)           # x, y, theta
p = Pose2(Rot2.fromDegrees(90), Point2(1, 2))
p.x(); p.y(); p.theta()
p.transformTo(point) ; p.bearing(point)
p.localCoordinates(q); p.retract(delta)
Pose2.Expmap(xi); Pose2.Logmap(pose)
```

#### Pose3（3D 位姿, SE(3)）

```python
from gtsam import Pose3, Rot3, Point3; import numpy as np
R = Rot3.Yaw(np.deg2rad(30))
P = Pose3(R, Point3(3, 4, 0))
P.rotation(); P.translation(); P.matrix()
P.inverse(); P.compose(Q); P.between(Q)
P.localCoordinates(Q); P.retract(delta)
```

**工程注意**：Jacobian/tangent vector 维度为 6。Pose3 同时作为 pose manifold 和 rigid transform，但 pose 概念上不等同于 transform。GTSAM 使用右扰动约定。

#### Rot3（SO(3) 姿态）

```python
R = Rot3(); R = Rot3.Yaw(0.1); R = Rot3.Ypr(yaw, pitch, roll)
R = Rot3.Expmap(np.array([0.01, 0.02, 0.03]))
R.matrix(); R.quaternion(); R.rpy()
R.compose(Q); R.inverse(); R.between(Q)
Rot3.Logmap(R)
```

#### Cal3_S2（针孔相机内参）

参数 `[fx, fy, s, u0, v0]`：

```python
K = Cal3_S2(500.0, 500.0, 0.0, 320.0, 240.0)
K.fx(); K.fy(); K.principalPoint()
normalized = K.calibrate(Point2(330, 250))    # pixel → normalized
pixel = K.uncalibrate(normalized)              # normalized → pixel
```

#### StereoCamera / Cal3_S2Stereo

处理 `(uL, uR, v)` stereo measurement：

```python
K = Cal3_S2Stereo(fx, fy, s, u0, v0, baseline)
camera = StereoCamera(left_camera_pose, K)
z = camera.project(point_world)
p = camera.backproject(stereo_point)
```

### 2. Factor Graph Core — 问题表达

#### NonlinearFactorGraph

```python
graph = NonlinearFactorGraph()
graph.add(factor)
graph.addPriorPose2(key, pose, noise)
graph.addPriorPose3(key, pose, noise)
graph.size(); graph.empty(); graph.error(values)
graph.linearize(values)  # → GaussianFactorGraph
```

#### Values（变量容器）

```python
values = Values()
values.insert(X(0), Pose3())
values.insert(L(1), Point3(1, -2, 5))
pose = values.atPose3(X(0))
point = values.atPoint3(L(1))
vector = values.atVector(K(0))  # 标量/向量变量
```

#### Key（变量 ID）

```python
from gtsam.symbol_shorthand import X, V, B, L, C
X(0)   # 位姿 x0 → 8646911284551352320
V(0)   # 速度 v0
B(0)   # bias b0
L(1)   # landmark l1
C(0)   # camera c0
```

#### noiseModel（噪声模型）

```python
noise = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))  # 各向异性
noise = noiseModel.Isotropic.Sigma(3, 0.2)                      # 同方差
noise = noiseModel.Unit.Create(3)                                # 单位噪声
# 鲁棒核:
noise = noiseModel.Robust.Create(noiseModel.mEstimator.Huber(1.345), base_noise)
```

**经验规则**：residual 维度必须匹配 noise model 维度；sigma 越小约束越强；不要用极小 sigma 掩盖建模错误。

### 3. Optimizer — Batch 优化器

| Optimizer | 适用场景 | 注意 |
|-----------|----------|------|
| `LevenbergMarquardtOptimizer` | 默认 batch 选择，非线性强或初值一般 | damping 稳定但收敛慢 |
| `GaussNewtonOptimizer` | 初值好、问题近似二次 | 对 bad initial guess 更敏感 |
| `DoglegOptimizer` | trust-region 风格 | 结合 steepest descent 与 GN |
| `GncOptimizer` | 非凸 robust optimization / outlier 问题 | 调度型加权 |

```python
params = LevenbergMarquardtParams()
params.setVerbosityLM("SUMMARY")
params.setMaxIterations(100)
params.setRelativeErrorTol(1e-5)
params.setAbsoluteErrorTol(1e-5)
optimizer = LevenbergMarquardtOptimizer(graph, initial, params)
result = optimizer.optimize()
print(graph.error(initial), "→", graph.error(result))
```

常用 LM 参数：`lambdaInitial`、`lambdaFactor`、`lambdaUpperBound`、`lambdaLowerBound`、`minModelFidelity`、`diagonalDamping`、`orderingType`、`linearSolverType`。

### 4. ISAM2 & Marginals — 增量优化与协方差

#### ISAM2（增量平滑与建图）

将因子图转换为贝叶斯树，新因子加入时只重线性化受影响的团，避免全局优化。

```python
isam = ISAM2(ISAM2Params())

# 每新增一帧:
new_factors = NonlinearFactorGraph()
new_values = Values()
new_factors.add(odom_factor)
new_values.insert(X(k), initial_pose)
isam.update(new_factors, new_values)

# 查询结果:
estimate = isam.calculateEstimate()
x = isam.calculateEstimatePose3(X(k))
cov = isam.marginalCovariance(X(k))
info = isam.marginalInformation(X(k))
joint_cov = isam.jointMarginalCovariance([X(k-1), X(k)])
```

ISAM2 的固定滞后模式（`IncrementalFixedLagSmoother`）通过贝叶斯树自动边际化旧变量，是 Kimera-VIO、LIO-SAM 等系统的核心在线引擎。

#### Marginals（Batch 结果协方差）

```python
marginals = Marginals(graph, result)
cov = marginals.marginalCovariance(X(0))
joint = marginals.jointMarginalCovariance([X(0), L(1)])
joint.at(key_i, key_j)        # 按 key 查询 block
joint.fullMatrix()             # 完整稀疏矩阵
```

**注意**：查询 block 时优先用 `at(key_i, key_j)` 而非依赖 full matrix 的内部排列。

### 5. SLAM / Vision Factors

#### PriorFactor

锚定第一帧或给 landmark/参数提供初始约束：

```python
prior_noise = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
graph.addPriorPose2(X(0), Pose2(0, 0, 0), prior_noise)
```

注意：非常强的 prior 可能恶化 condition number；需真正固定变量时用 `NonlinearEquality`。

#### BetweenFactor

相对位姿测量（odometry、loop closure）：

```python
odom_noise = noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
graph.add(BetweenFactorPose2(X(0), X(1), Pose2(1, 0, 0), odom_noise))

# 3D版:
pose3_noise = noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1]))
graph.add(BetweenFactorPose3(X(1), X(2), measured_pose3, pose3_noise))
```

误差直觉：预测相对位姿 `X1.between(X2)` 与 measurement `Z` 的差，映射到 tangent space。

#### GenericProjectionFactor（单目重投影）

连接 camera pose 和 3D landmark，calibration 固定：

```python
pixel_noise = noiseModel.Isotropic.Sigma(2, 1.0)
factor = GenericProjectionFactorCal3_S2(Point2(330, 250), pixel_noise, X(0), L(1), K)
graph.add(factor)
```

可选参数：`body_P_sensor`（body→camera 外参）、cheirality handling flags。

#### SmartProjectionFactor（隐式路标）

SfM/visual SLAM 中把路标作为隐式变量：
- factor 内部按当前 camera 位姿三角化
- 线性化时通过 Schur 补消去 landmark，只约束相机位姿
- 减少图规模，避免显式 landmark 变量

```python
smart_noise = noiseModel.Isotropic.Sigma(2, 1.0)
factor = SmartProjectionFactorPinholeCameraCal3_S2(smart_noise, SmartProjectionParams())
factor.add(Point2(150, 505), C(0))
factor.add(Point2(470, 495), C(1))
factor.add(Point2(480, 150), C(2))
graph.add(factor)
```

**SmartProjectionPoseFactor**（calibration 已知固定，只优化 poses）通常更高效。

**工程模式**（Kimera-VIO）：clone-and-add — 更新已有 smart factor 时 clone 旧因子后再添加新观测，不直接修改已线性化的因子。

#### Stereo Factor

```python
K = Cal3_S2Stereo(fx, fy, s, u0, v0, baseline)
stereo_noise = noiseModel.Isotropic.Sigma(3, 1.0)
factor = GenericStereoFactor3D(StereoPoint2(uL, uR, v), stereo_noise, X(0), L(1), K)
graph.add(factor)
```

### 6. Navigation / IMU Factors

#### NavState

表示 `(R, p, v)` 即 attitude、position、velocity。Tangent increment 顺序 `[dR, dP, dV]`，维度 9：

```python
X = NavState(Rot3.Yaw(0.1), Point3(1, 2, 3), np.array([0.5, 0, 0]))
X.attitude(); X.position(); X.velocity(); X.pose(); X.bodyVelocity()
X.retract(delta); X.localCoordinates(Y)
NavState.Expmap(xi); NavState.Logmap(X)
```

#### PreintegrationParams（IMU 预积分参数）

```python
params = PreintegrationParams.MakeSharedU(9.81)    # ENU, Z-up
params = PreintegrationParams.MakeSharedD(9.81)    # NED, Z-down
params.setAccelerometerCovariance(np.eye(3) * accel_sigma**2)
params.setGyroscopeCovariance(np.eye(3) * gyro_sigma**2)
params.setIntegrationCovariance(np.eye(3) * 1e-8)
params.setBodyPSensor(body_P_sensor)  # IMU→body 外参
```

注意：IMU 噪声单位匹配连续时间 white noise 约定。

#### ImuFactor（5-way factor）

连接 `Pose_i, Vel_i, Pose_j, Vel_j, Bias_i`：

```python
pim = PreintegratedImuMeasurements(params, bias_hat)
for acc, gyro, dt in imu_packets:
    pim.integrateMeasurement(acc, gyro, dt)
graph.add(ImuFactor(X(i), V(i), X(j), V(j), B(i), pim))
```

`ImuFactor2` 使用 `NavState`，连接 `NavState_i, NavState_j, Bias_i`，结构更紧凑。

#### CombinedImuFactor（6-way factor）

连接 `Pose_i, Vel_i, Bias_i, Pose_j, Vel_j, Bias_j`，在 15D covariance 中包含 bias random walk。

**官方建议**：默认不推荐一开始就用 CombinedIMU——bias 通常变化慢，低频 Markov chain 往往更合适。短时实验建议先用单个 constant bias 把主 pipeline 跑通。CombinedIMU 适合确实需要每个 interval 高精度 bias 估计的场景。

预积分实现类型受 `GTSAM_TANGENT_PREINTEGRATION` 编译 flag 控制（默认 true → tangent-space preintegration）。

#### GPSFactor family

GPS 测量必须先转换到 ENU/NED/ECEF。变体：

| 因子 | 说明 |
|------|------|
| `GPSFactor` | Pose3 + zero lever arm |
| `GPSFactorArm` | Pose3 + 已知 body-frame lever arm |
| `GPSFactorArmCalib` | Pose3 + 变量 lever arm（在线估计） |
| `GPSFactor2` / `GPSFactor2Arm` / `GPSFactor2ArmCalib` | 对应 NavState 版本 |

```python
gps_noise = noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5, 1.0]))
graph.add(GPSFactor(X(k), Point3(10.5, 20.2, 5.1), gps_noise))
graph.add(GPSFactorArm(X(k), gps_meas, Point3(-0.1, 0.0, 0.05), gps_noise))
```

### 7. CustomFactor — 自定义因子与 Jacobian

#### Python 函数签名

```python
def error_func(this: CustomFactor, values: Values, H: list[np.ndarray]) -> np.ndarray:
    # this.keys()  → 变量 key 列表
    # H is None    → 当前调用不需要 Jacobian
    # 返回值: 1D np.ndarray residual
    pass

factor = CustomFactor(noise, [key0, key1], error_func)
```

#### Jacobian 约定

GTSAM 使用**右侧 exponential map** 更新——$x_{new} = x \cdot \text{Exp}(\delta)$——因此 Jacobian 相对于右扰动线性化。不要直接套用左扰动或欧式加法推导。

#### Column-major (Fortran order) 要求

```python
H[0] = np.zeros((residual_dim, variable_dim), order="F")
```

保证与 C++ interop 兼容。

#### Numerical Jacobian 验证

```python
from gtsam.utils.numerical_derivative import numericalDerivative21, numericalDerivative22

def f(T1, T2):
    v = Values()
    v.insert(key0, T1)
    v.insert(key1, T2)
    return error_func(factor, v)

num_H0 = numericalDerivative21(f, values.atPose3(key0), values.atPose3(key1))
num_H1 = numericalDerivative22(f, values.atPose3(key0), values.atPose3(key1))
np.testing.assert_allclose(H[0], num_H0, rtol=1e-5, atol=1e-8)
```

#### 性能注意

- Python callback 每次 linearization 从 C++ 调 Python，有额外开销
- pybind11 获取 Python GIL，Python callback factor 不能并行 evaluation
- 大量 measurement 时优先 batch 一个 factor 处理多条测量
- 性能关键路径建议最终用 C++ factor

## 因子图工作流

### Batch / Incremental 模板

```python
# === Batch ===
graph = NonlinearFactorGraph()
initial = Values()
graph.addPriorPose2(X(0), Pose2(0, 0, 0), prior_noise)
graph.add(BetweenFactorPose2(X(0), X(1), Pose2(1, 0, 0), odom_noise))
initial.insert(X(0), Pose2(0, 0, 0))
initial.insert(X(1), Pose2(1.1, 0.1, 0.02))
result = LevenbergMarquardtOptimizer(graph, initial).optimize()
cov = Marginals(graph, result).marginalCovariance(X(1))

# === Incremental (ISAM2) ===
isam = ISAM2(ISAM2Params())
new_factors = NonlinearFactorGraph(); new_values = Values()
new_factors.addPriorPose2(X(0), Pose2(0, 0, 0), prior_noise)
new_values.insert(X(0), Pose2(0, 0, 0))
isam.update(new_factors, new_values)
pose = isam.calculateEstimatePose2(X(0))
cov = isam.marginalCovariance(X(0))
```

### Gauge Freedom 处理

SLAM 因子图常有 gauge freedom（整张图可整体平移/旋转而 error 不变）。工程上用 `PriorFactor` 或 `NonlinearEquality` 锚定：

- `PriorFactor`：软约束，受噪声模型权重影响
- `NonlinearEquality`：强约束；但可能恶化条件数

### 常见调试顺序

1. 打印 `graph.size()` 和 `initial.size()`，确认 key 没漏
2. 对单个 factor 调 `error(values)` 或 `unwhitenedError(values)`
3. 对自定义 factor 单元测试 numerical Jacobian
4. 优化前后调用 `graph.error(initial)` 和 `graph.error(result)`
5. 对 3D/VIO 问题检查 frame convention、gravity direction、IMU body_P_sensor

## Agent 实现提示

### 适用场景

1. **Batch SLAM 后端**：已知全部观测，需一次性优化位姿图时，用 LM/GN + PriorFactor/BetweenFactor 建图
2. **在线 VIO/VSLAM**：帧序贯到达，用 ISAM2 + IMU preintegration + SmartProjectionFactor 实现增量平滑
3. **GNSS/INS 融合**：IMU 预积分 + GPSFactor arm variant + NavState 变量建模
4. **视觉 SfM**：多帧重投影 BA，SmartProjectionFactor 消去 landmark 减少图规模
5. **自定义传感器**：CustomFactor 接入任意 measurement model，Jacobian 采用数值导数验证

### 输入输出契约

| 组件 | 输入 | 输出 |
|------|------|------|
| `NonlinearFactorGraph` | `add(factor)` | `.error(values)`, `.linearize(values)` |
| `Values` | `insert(key, value)` | `.atPose3(key)`, `.atPoint3(key)` |
| `LM/GN/Dogleg` | `(graph, initial, params)` | `.optimize()` → 优化后 `Values` |
| `ISAM2` | `update(new_factors, new_values)` | `calculateEstimate()` — 贝叶斯树增量更新 |
| `Marginals` | `(graph, result)` | `marginalCovariance(key)`, `jointMarginalCovariance(keys)` |
| `CustomFactor` | callback `(factor, values, H) → residual` | 残差 `np.ndarray` + Jac `H[i]` 按 column-major 写入 |
| `ImuFactor` | `(Xi, Vi, Xj, Vj, Bi, pim)` | 9 维残差；`CombinedImuFactor` → 15D |

### 实现骨架（伪代码）

```python
# === Batch Pose Graph SLAM ===
def solve_pose_graph_slam(odometry, loop_closures, prior_pose):
    graph = NonlinearFactorGraph()
    initial = Values()
    prior_noise = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
    odom_noise = noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
    lc_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))

    graph.addPriorPose3(X(0), prior_pose, prior_noise)
    initial.insert(X(0), prior_pose)
    for (i, j, dpose) in odometry:
        key_i, key_j = X(i), X(j)
        graph.add(BetweenFactorPose3(key_i, key_j, dpose, odom_noise))
        if not initial.exists(key_j):
            initial.insert(key_j, initial.atPose3(key_i).compose(dpose))
    for (i, j, rpose) in loop_closures:
        graph.add(BetweenFactorPose3(X(i), X(j), rpose, lc_noise))

    result = LevenbergMarquardtOptimizer(graph, initial).optimize()
    cov = Marginals(graph, result).marginalCovariance(X(-1))
    return result, cov

# === VIO with ISAM2 + preintegration ===
def vio_pipeline(imu_buffer, cam_frames, calib):
    params = PreintegrationParams.MakeSharedU(9.81)
    params.setAccelerometerCovariance(np.eye(3) * 0.01)
    params.setGyroscopeCovariance(np.eye(3) * 0.001)
    isam = ISAM2(ISAM2Params())
    result = Values()

    for k, frame in enumerate(cam_frames):
        new_factors = NonlinearFactorGraph()
        new_values = Values()
        if k == 0:
            new_factors.addPriorPose3(X(0), Pose3(), prior_noise)
            new_values.insert(X(0), Pose3())
            new_values.insert(V(0), np.zeros(3))
            new_values.insert(B(0), ConstantBias())

        # IMU preintegration
        pim = PreintegratedImuMeasurements(params, result.atConstantBias(B(k-1)))
        for acc, gyro, dt in imu_buffer:
            pim.integrateMeasurement(acc, gyro, dt)
        new_factors.add(ImuFactor(X(k-1), V(k-1), X(k), V(k), B(k-1), pim))

        # Visual factors
        for track in frame.tracks:
            if track.is_initialized():
                factor = SmartProjectionPoseFactor(vis_noise, SmartProjectionParams())
                for obs in track.observations:
                    factor.add(obs.point2, X(obs.frame_id), K)
                new_factors.add(factor)

        new_values.insert(X(k), predict_pose)
        new_values.insert(V(k), predict_vel)
        new_values.insert(B(k), ConstantBias())
        isam.update(new_factors, new_values)
        result = isam.calculateEstimate()

    return result

# === CustomFactor 模板 ===
def make_custom_factor(key0, key1, noise):
    def error_func(this, values, H):
        x = values.atPose3(key0)
        y = values.atPose3(key1)
        residual = compute_residual(x, y)
        if H is not None:
            H[0] = compute_jac_wrt_x(x, y, order='F')  # (dim_res, 6) column-major
            H[1] = compute_jac_wrt_y(x, y, order='F')
        return residual
    return CustomFactor(noise, [key0, key1], error_func)
```

### 关键源码片段

标准优化流水线见上方 [因子图工作流](#因子图工作流)，同 GTSAM 官方 Python 示例模式。工程因子实现参考 [[方法-GTSAM-Ceres工程因子]] 中 `error()`/`linearize()` 流水线。ENU/右扰动/CustomFactor 约定见各模块详解对应小节。

### 实现注意事项

1. **右扰动约定**：自写 CustomFactor 时 Jacobian 必须是相对于 $x \cdot \text{Exp}(\delta)$ 的线性化，不可套用左扰动或欧式加法微分雅可比。
2. **Column-major H 矩阵**：Python callback 中 H[i] 必须用 `order='F'` 或 `np.asfortranarray()` 创建，否则 C++ interop 读取错误。
3. **Python GIL 限制**：CustomFactor 每次 evaluation 获取 Python GIL，不能并行。大量 measurement 时用单 factor 批处理多条观测。
4. **CombinedIMU vs ImuFactor**：除每个 interval 都需高精度 bias 估计外，首选 ImuFactor + 低频 bias Markov chain。
5. **Gauge freedom**：对首帧加 PriorFactor 锚定；非常强的 prior 可能恶化条件数，必要时用 NonlinearEquality。
6. **Smart factor clone-and-add**：更新已有 smart factor 时必须 clone 再 add，不可直接修改已线性化的旧因子。
7. **GPS 坐标系**：所有 GNSS 测量先统一到同一 local ENU/NED 原点，杆臂必须显式补偿。
8. **ISAM2 vs LM**：在线场景用 ISAM2（贝叶斯树增量更新），一次性批处理用 LM（稳定默认）；ISAM2 的 `IncrementalFixedLagSmoother` 自带滑动窗口管理。

### 源码检索锚点

- GTSAM Python 官方文档根：https://borglab.github.io/gtsam/
- C++ Doxygen：https://gtsam.org/doxygen/
- Concepts：https://borglab.github.io/gtsam/gtsam-concepts
- 工程因子实现参考：[[方法-GTSAM-Ceres工程因子]] — IntegratedMatchingCostFactor, SmartStereoFactor, GNSS residual
- ISAM2 增量模式参考：[[方法-ISAM2增量固定滞后平滑]]
- `raw/codes/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp` — IMCF 基类 error/linearize 流水线
- `raw/codes/Kimera-VIO/src/backend/RegularVioBackend.cpp:L478-L504` — addLandmarkToGraph (SmartStereoFactor)

## 相关页面

### 专题（已归并本页）
- [[GTSAM API 使用索引]] — superseded
- [[GTSAM Geometry API]] — superseded
- [[GTSAM Nonlinear 优化 API]] — superseded
- [[GTSAM SLAM 与视觉因子 API]] — superseded
- [[GTSAM Navigation 与 IMU API]] — superseded
- [[GTSAM 自定义因子与 Jacobian]] — superseded
- [[GTSAM 因子图工作流]] — superseded

### 工程实现
- [[方法-GTSAM-Ceres工程因子]]
- [[方法-ISAM2增量固定滞后平滑]]
- [[方法-滑动窗口边缘化]]
- [[方法-Ceres解析雅可比CostFunction]]
- [[方法-Anchor 节点位姿图优化]]

### 框架与系统
- [[组件-GTSAM]]
- [[组件-Ceres-Solver]]
- [[算法-Kimera-VIO]]
- [[算法-IC-GVINS]]
- [[算法-LIO-SAM]]

### 理论基础
- [[概念-因子图]]
- [[概念-位姿图优化]]
- [[概念-Schur补与边缘化]]
- [[概念-三角化与深度估计]]
