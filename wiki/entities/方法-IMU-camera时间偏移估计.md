---
tags: [时间同步, IMU-camera标定, td估计, VINS-Fusion]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp
  - raw/codes/VINS-Fusion/vins_estimator/src/factor/projection_factor.cpp
---

# IMU-Camera 时间偏移估计

> VINS-Fusion 中 $t_d$ (time offset) 的在线估计方法：将时间偏移作为 Ceres 优化变量嵌入视觉重投影因子，利用光流速度进行线性插值补偿。

## 问题动机

在多传感器融合系统中，相机和 IMU 通常由不同时钟源驱动，且数据传输存在延迟。实际的时间关系为：

$$
t_{\text{camera}} = t_{\text{imu}} + t_d
$$

其中 $t_d$ 是 IMU 相对于相机的时间偏移（IMU 的测量时间戳滞后于相机快门时刻）。若 $t_d$ 不准确，视觉观测量会错误地关联到错误的 IMU 状态，导致：

- **IMU 预积分误差**：预积分起始/终止时刻的 IMU 状态与视觉观测的实际时刻错位
- **重投影残差系统偏差**：投影所用的世界点坐标对应的 IMU 姿态与实际成像时的姿态不一致
- **快速运动时影响放大**：静止时 $t_d$ 误差几乎不可见，运动越剧烈偏差越显著

## VINS-Fusion 方法

VINS-Fusion 将 $t_d$ 作为 Ceres 优化问题的一个 1 维参数块，与位姿、速度、bias、深度和外参**联合优化**。其核心思想是利用特征点在图像上的**光流速度**（像素速度），通过线性插值将特征坐标从观测时刻纠正到优化变量 $t_d$ 所对应的时刻。

### 数学模型

对于特征点在第 $i$ 帧和第 $j$ 帧的观测 $\mathbf{p}_i$ 和 $\mathbf{p}_j$，实际的重投影因子使用时间补偿后的坐标：

$$
\mathbf{p}_i^{t_d} = \mathbf{p}_i - (t_d - t_{d,i}) \cdot \mathbf{v}_i
$$

$$
\mathbf{p}_j^{t_d} = \mathbf{p}_j - (t_d - t_{d,j}) \cdot \mathbf{v}_j
$$

其中：
- $t_{d,i}$、$t_{d,j}$ 是特征点被观测时记录的系统当前 $t_d$ 估计值（存储在 `FeaturePerFrame::cur_td` 中）
- $\mathbf{v}_i$、$\mathbf{v}_j$ 是特征点在归一化平面上的光流速度（来自特征跟踪器的像素速度除以焦距）

补偿后的坐标进入标准重投影链：

$$
\mathbf{p}_{\text{imu},i}^{t_d} = \mathbf{R}_{bc} \cdot \left( \frac{\mathbf{p}_i^{t_d}}{d} \right) + \mathbf{t}_{bc}
$$

$$
\mathbf{p}_{w}^{t_d} = \mathbf{R}_{b_i,w} \cdot \mathbf{p}_{\text{imu},i}^{t_d} + \mathbf{t}_{b_i,w}
$$

两帧重投影误差定义为：

$$
\mathbf{e} = \pi\left( \mathbf{R}_{bc}^{-1} \left[ \mathbf{R}_{b_j,w}^{-1} (\mathbf{p}_w^{t_d} - \mathbf{t}_{b_j,w}) - \mathbf{t}_{bc} \right] \right) - \begin{bmatrix} \mathbf{p}_j^{t_d[0]} \\ \mathbf{p}_j^{t_d[1]} \end{bmatrix}
$$

### Jacobian 对 $t_d$

时间偏移对残差的 Jacobian 由两部分组成：(1) 由 $\mathbf{p}_i^{t_d}$ 通过重投影链传播；(2) 直接通过 $\mathbf{p}_j^{t_d}$ 的测量残差项。

在 `ProjectionTwoFrameOneCamFactor::Evaluate` 中实现（核心逻辑）：

```cpp
// 时间补偿
pts_i_td = pts_i - (td - td_i) * velocity_i;
pts_j_td = pts_j - (td - td_j) * velocity_j;

// 标准重投影链（使用补偿后的坐标）
pts_camera_i = pts_i_td / inv_dep_i;
pts_imu_i = qic * pts_camera_i + tic;
pts_w = Qi * pts_imu_i + Pi;
pts_imu_j = Qj.inverse() * (pts_w - Pj);
pts_camera_j = qic.inverse() * (pts_imu_j - tic);

// 残差中对 pts_j_td 的观察
residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();

// Jacobian w.r.t td
jacobian_td = reduce * ric^T * Rj^T * Ri * ric * velocity_i / inv_dep_i * (-1.0)
            + sqrt_info * velocity_j.head<2>;
```

## 数值稳定性守卫

VINS-Fusion 在系统**静止或低速**时固定 $t_d$，因为此时时间偏移几乎不可观测：

```cpp
// raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1040-L1043
problem.AddParameterBlock(para_Td[0], 1);

if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
    problem.SetParameterBlockConstant(para_Td[0]);
```

**守卫条件**的行为：
- `!ESTIMATE_TD`：配置文件中禁止 $t_d$ 估计 — 始终保持参数块为常量
- `Vs[0].norm() < 0.2`：滑动窗口首帧（通常是最旧帧）IMU 速度范数小于 0.2 m/s — 系统接近静止，不具备足够的运动激励来观测时间偏移

**为什么速度是关键条件**：$t_d$ 的可观测性依赖于图像中特征位置的时间变化率与 IMU 状态变化率的耦合。当系统静止时，IMU 状态不变，重投影残差对 $t_d$ 的导数趋近于零，此时 $t_d$ 会收敛到伪值或将运动误差吸收到自身。

外参估计也有相同的守卫条件（`ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2`），两者共享相同的可观测性逻辑。

## 伪代码：td 估计完整管线

```python
def estimate_time_offset():
    # ---- 初始化 ----
    td = config.initial_td          # 从配置文件或出厂标定获取初值
    ESTIMATE_TD = 1                 # 在线估计开关

    for each image_frame:
        # ---- 1. 特征跟踪 (feature_tracker) ----
        features, velocities = track_features(image_frame)
        # velocity 是像素速度，除以焦距得到归一化平面速度

        # ---- 2. 记录特征帧（附带当前的 td） ----
        for each feature_point:
            feature.per_frame.append(FeaturePerFrame(
                point=xyz, uv=uv, velocity=velocity, cur_td=td
            ))

        # ---- 3. IMU 预积分 (processMeasurements) ----
        imu_time = image_time + td           # td 修正后的 IMU 对齐时间
        acc_data, gyr_data = get_IMU_between(prev_time, imu_time)
        pre_integrations.push(IntegrateIMU(acc_data, gyr_data))

        # ---- 4. 管理滑动窗口 ----
        if parallax_sufficient:
            slide_window_oldest()
        else:
            slide_window_second_newest()

        # ---- 5. 滑动窗口优化 ----
        problem = CeresProblem()

        # 添加参数块
        problem.add_parameter_block(para_Td, dim=1)
        if not ESTIMATE_TD or velocity_norm < 0.2:
            problem.set_parameter_block_constant(para_Td)  # 可观测性守卫

        # 添加 IMU 因子
        for each consecutive frame pair (i, i+1):
            problem.add_residual(IMUFactor(pre_integrations[i+1]))

        # 添加视觉因子（投影因子带 td）
        for each feature observed in frames i, j:
            pts_i_compensated = pts_i - (para_Td - td_i) * velocity_i
            pts_j_compensated = pts_j - (para_Td - td_j) * velocity_j
            problem.add_residual(
                ProjectionTdFactor(
                    pts_i_compensated, pts_j_compensated,
                    pose_i, pose_j, extrinsic, depth,
                    para_Td
                )
            )

        problem.solve()

        # ---- 6. 提取优化结果 ----
        td = para_Td[0]   # 仅当 td 未被固定时才更新
```

## 与其他方法的对比

| 方法 | 原理 | 适用场景 |
|------|------|----------|
| **VINS 在线估计** | td 作为 Ceres 变量联合优化 | 运行时自动修正，对运动激励有要求 |
| **硬件触发同步** | 外部信号线或 PTP（IEEE 1588）精密时钟同步 | 高精度需求场景，需要硬件支持，延迟固定且已知 |
| **Kalibr 离线标定** | 连续时间轨迹拟合估计时间偏移 | 出厂标定，可控环境，需标定板 |
| **互相关检测** | IMU 和相机信号的互相关峰值 | 粗略估计，用于验证或初值 |

### FAST-LIO2 / LIO-SAM 的时间处理

FAST-LIO2 和 LIO-SAM **不显式估计 LiDAR-IMU 时间偏移**。两者假设时间戳已通过硬件同步或预处理对齐：

- **LIO-SAM** (`raw/codes/LIO-SAM/src/imageProjection.cpp:L237-L238`)：使用 LiDAR 点云中逐点的 `time` 字段（每个点相对扫描起始时间）进行运动畸变校正（deskew），不做时间偏移优化
- **FAST-LIO2**：在状态传播阶段使用 IMU 数据与 LiDAR 扫描结束时刻对齐，依赖点云内部的 `curvature` 或 `timestamp` 字段

### 工程建议

- **初始化**：使用硬件 PTP 同步或 Kalibr 离线标定提供 $t_d$ 初值
- **在线校验**：开启 `ESTIMATE_TD=1`，但仅在 `Vs > 0.2 m/s` 时允许 td 更新
- **收敛判定**：观察 $t_d$ 的收敛曲线，静止时不应漂移
- **外参与 td 的耦合**：在相机-IMU 外参同时估计时，td 和外参平动分量存在耦合（两者都影响重投影的位置），建议在足够旋转激励下先收敛外参旋转，再进行全量估计
- **多相机同步**：VINS-Fusion 支持双相机，每个相机与 IMU 的 $t_d$ 不同。实际实现中是单一 td 变量校正两个相机的观测

## Agent 实现提示

### 适用场景
多传感器融合系统（VIO、LIO、VILO）中 IMU-相机时间延迟的在线估计，以及离线数据处理中的时间同步校验。

### 输入输出契约
- **输入**：IMU 测量序列（加速度、角速度、时间戳），相机图像帧序列（时间戳），特征跟踪结果（归一化坐标 + 像素速度）
- **输出**：估计的时间偏移 $t_d$（标量，单位秒）
- **前提**：系统处于运动状态（速度 > 阈值），IMU 预积分和视觉特征匹配已经工作

### 实现骨架（伪代码）
```python
# 时间补偿因子核心逻辑
def td_compensate_point(point_raw, velocity, td_current, td_observed):
    """将特征点坐标从 td_observed 时刻线性插值到 td_current 时刻"""
    return point_raw - (td_current - td_observed) * velocity

# 重投影残差计算
def compute_residual_and_jacobian(pts_i, pts_j, vel_i, vel_j,
                                   td_i, td_j, td_param,
                                   pose_i, pose_j, extrinsic, inv_depth):
    # 时间补偿
    pts_i_td = pts_i - (td_param - td_i) * vel_i
    pts_j_td = pts_j - (td_param - td_j) * vel_j

    # 标准重投影
    pts_cam_i = pts_i_td / inv_depth
    pts_imu_i = R_bc * pts_cam_i + t_bc
    pts_w = R_i * pts_imu_i + t_i
    pts_imu_j = R_j.inverse() * (pts_w - t_j)
    pts_cam_j = R_bc.inverse() * (pts_imu_j - t_bc)

    # 残差（归一化平面）
    residual = pts_cam_j.head<2>() / pts_cam_j.z() - pts_j_td.head<2>()

    # Jacobian: 链式法则，td 通过 pts_i_td 和 pts_j_td 各自影响残差
    dr_dptsi = rhs_jacobian * R_bc.T * R_j.T * R_i * R_bc / inv_depth
    jac_td = dr_dptsi * (-vel_i) + sqrt_info * vel_j.head<2>()
    return residual, jac_td
```

### 关键源码片段
- `raw/codes/VINS-Fusion/vins_estimator/src/factor/projectionTwoFrameOneCamFactor.cpp:L57-L61` — td 补偿特征点坐标
- `raw/codes/VINS-Fusion/vins_estimator/src/factor/projectionTwoFrameOneCamFactor.cpp:L140-L145` — td 的解析 Jacobian
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1040-L1043` — 数值稳定性守卫（速度阈值）
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1084-L1086` — 视觉因子注册，td 作为参数块
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.h:L31-L42` — FeaturePerFrame 存储 cur_td 和 velocity
- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L270-L283` — processMeasurements 中 td 修正 IMU 时间戳

### 实现注意事项
1. **velocity 的量纲**：特征跟踪器输出的 velocity 是**像素/秒**（图像平面），投影因子将其视为归一化平面速度使用——实际使用时需除以焦距
2. **td_i / td_j 冻结**：`FeaturePerFrame::cur_td` 在特征被观测时**拷贝当前 td 值**，优化过程中不随 `para_Td[0]` 变化。这是增量式优化中的增量固定技巧（incremental fixing），避免 Jacobian 中出现二阶耦合
3. **滑动窗口中的 td 更新**：窗口滑动（marginalization）时 td 通过优先级信息传递，被边缘化的帧的 td 相关约束转移到先验因子
4. **收敛条件**：td 通常在 10-30 次优化迭代内收敛（如果运动激励充分），收敛后波动在 ±0.5ms 以内

### 源码检索锚点
- `ESTIMATE_TD` 参数定义：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/parameters.h:L48`
- td 初始赋值：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L108`
- td 参数块注册与常量化：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1040-L1043`
- td 更新到成员变量：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L951`
- td 用于 IMU 时间戳修正：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L280`
- 视觉因子创建（单目双帧）：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1084-L1086`
- 视觉因子创建（双目双帧）：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1094-L1096`
- 视觉因子创建（双目单帧）：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1100-L1102`

## 相关页面

- [[方法-多传感器外参在线标定]]
- [[传感器-传感器标定]]
- [[概念-IMU预积分]]
- [[数学-流形优化]]
- [[架构-多传感器融合架构]]
- [[组件-Ceres-Solver]]

