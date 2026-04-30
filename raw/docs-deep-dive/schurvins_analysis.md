# SchurVINS 源码深度分析报告

> **分析日期**: 2026-04-28
> **代码版本**: CVPR 2024 (基于 SVO 2.0 / rpg_svo_pro_open)
> **论文**: SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System

---

## 1. 数据接收与预处理

### 1.1 图像/IMU 数据入口

数据接收通过 ROS 接口实现，核心类为 `svo::SvoInterface` (`svo_ros/include/svo_ros/svo_interface.h:32`)。

**图像订阅**:
- 双目模式下订阅 `/cam0/image_raw` 和 `/cam1/image_raw` 两个 topic
- 回调函数 `stereoCallback()` (`svo_ros/include/svo_ros/svo_interface.h:82`)
- 图像到 Frame 转换在 `FrameHandlerBase::addImageBundle()` (`svo/src/frame_handler_base.cpp:189`)
  - 创建 `Frame` 对象，同时构建图像金字塔（`n_pyr_levels` 层，默认值 3）

**IMU 订阅**:
- 订阅 `/imu0` topic，回调 `imuCallback()` (`svo_ros/include/svo_ros/svo_interface.h:85`)
- IMU 数据存储在 `ImuHandler::measurements_` 双端队列，最新数据在队列头部 (`svo/include/svo/imu_handler.h:187`)

### 1.2 时间戳对齐策略

实现在 `addFrameBundle()` (`svo/src/frame_handler_base.cpp:345-363`):

1. **IMU 等待**: 调用 `imu_handler_->waitTill(curr_frame_bundle_stamp)`，等待 IMU 数据到达 (`svo/include/svo/imu_handler.h:181`)
2. **边缘 IMU 提取**: `getMeasurementsContainingEdges()` (`svo/src/imu_handler.cpp:105`)
   - 查找时间戳小于 `frame_timestamp - imu_calib_.delay_imu_cam` 的测量值
   - 回退一个元素以确保插值所需的边界值（行 132-136）
3. **反序存储**: 获取的 IMU 数据按时间反序存储在 `frame_bundle->imu_datas_` 中
4. **Forward 中的插值**: `SchurVINS::Forward()` (`svo/src/schur_vins.cpp:665`)
   - 如果 IMU 时间 < 图像时间：直接使用实际 IMU 数据
   - 如果 IMU 时间 > 图像时间：线性插值到图像时间戳（行 700-713）

### 1.3 图像预处理

- 不进行传统图像去畸变，保持原始图像用于直接法
- 构建图像金字塔，最大层数 `img_align_max_level: 4` (vio_stereo.yaml:84)
- 使用 `FastDetector` 在金字塔图像上检测 FAST 角点

---

## 2. 特征提取与跟踪

### 2.1 特征提取方法

SchurVINS 继承 SVO 2.0 的半直接法（semi-direct），使用 FAST 角点检测器：

- `FastDetector` 类 (`svo_direct/include/svo/direct/feature_detection.h:66`)
- FAST 阈值 `detector_threshold_primary: 10` (vio_stereo.yaml:79)
- 同时支持 Edgelet (关闭) 和 Shi-Tomasi 角点

特征类型枚举 (`svo_common/include/svo/common/types.h:94-107`):
```
kEdgelet=6, kCorner=7, kMapPoint=8, kFixedLandmark=9, kOutlier=10
```
SchurVINS 主要使用 `kCorner` 类型 (`schur_vins.cpp:598`)。

### 2.2 每帧特征数量与网格策略

- **最大特征数**: `max_fts: 180` (vio_stereo.yaml:18)
- **网格大小**: `grid_size: 25` (vio_stereo.yaml:77)
- **网格策略**: `OccupandyGrid2D` (`svo_common/include/svo/common/occupancy_grid_2d.h`)
  - 每网格单元最多一个特征，确保均匀分布
  - 重投影阶段已占用网格被标记，新特征不落其中

### 2.3 RANSAC 离群点剔除

- **双目初始化**: `StereoInit` 直接三角化 (`svo/src/stereo_triangulation.cpp:23`)
- **单目初始化**: Homography / 5-Point / 2-Point RANSAC
- **EKF 后剔除**: (`svo/src/schur_vins.cpp:801-915`)
  - `RemoveOutliers()`: 以 4.0px 阈值移除当前帧外点 (行 868-869)
  - `RemovePointOutliers()`: 以 3.0px 平均重投影误差标记 `local_status_` (行 801-854)

---

## 3. 初始化

### 3.1 第一帧位姿确定

`setInitialPose()` (`svo/src/frame_handler_base.cpp:628-674`)，三种策略：

1. **IMU 旋转先验**: 用提供的 `R_imu_world_` 初始化
2. **IMU 内置重力**: 加速度计→重力方向→Roll/Pitch
3. **默认值**: identity 变换 (`vio_stereo.yaml:30-33`)

### 3.2 双目初始化流程

`processFirstFrame()` (`svo/src/frame_handler_stereo.cpp:69-101`):

1. `schurvinsForward()`: IMU 预测 (行 71)
2. `StereoInit::addFrameBundle()`: 立体匹配+三角化 (行 72)
   - `StereoTriangulation::compute()` (`svo/src/stereo_triangulation.cpp:23`)
   - 左图 FAST 角点 (行 41)
   - 沿极线在右图搜索 (`matcher.findEpipolarMatchDirect()`, 行 101-102)
   - 三角化创建 Point 对象 (行 108-125)
3. 关键帧加入地图 (行 78-83)
4. 深度滤波器初始化 (行 86)
5. `schurvinsBackward()`: EKF 视觉更新 (行 87)

### 3.3 SchurVINS 状态初始化

`InitState()` (`svo/src/schur_vins.cpp:618-648`):

使用 IMU 加速度（归一化）与世界重力 `(0,0,1)` 计算初始旋转:
```cpp
quat = Eigen::Quaterniond::FromTwoVectors(acc_norm, grav_norm);
```
位置、速度、偏差均为零。

---

## 4. 逐帧状态估计 — Schur Complement EKF 核心

### 4.1 系统架构

SchurVINS 在 `USE_SCHUR_VINS` 宏下替代原 SVO 的 `sparseImageAlignment()` 和 `optimizePose()`
(`frame_handler_base.cpp:689-881`)。

核心管道 (`processFrame()`, `svo/src/frame_handler_stereo.cpp:103-147`):
```

STEP 1: schurvinsForward()  →  IMU 预测 + 姿态写入帧
STEP 2: projectMapInFrame() →  地图点重投影 + 特征匹配
STEP 3: schurvinsBackward() →  EKF 视觉更新 + 外点剔除
STEP 4: optimizeStructure() →  地标点 EKF 优化
```

### 4.2 EKF 状态变量

`ImuState` 类 (`svo/include/svo/schur_vins.h:47-65`):

| 分量 | 维度 | 说明 |
|------|------|------|
| quat | 4 (3 DOF 误差) | IMU 在世界系的旋转 |
| pos | 3 | IMU 在世界系的位置 |
| vel | 3 | IMU 在世界系的速度 |
| ba | 3 | 加速度计偏差 |
| bg | 3 | 陀螺仪偏差 |
| **总计** | **15 DOF (误差态)** | |

`AugState` 类 (`svo_common/include/svo/common/local_feature.h:57-84`):
- 窗口内每个相机位姿 6 DOF（姿态 3 + 平移 3）
- 存储在 `states_map` 中

**窗口大小**: `window_size: 4` (vio_stereo.yaml:198)
**总误差态维度**: 15 (IMU) + 4×6 (相机) = **39 DOF**

协方差矩阵初始化 (`InitCov`, `svo/src/schur_vins.cpp:32-44`):
- 姿态: `1e-4`, 位置: `1e-3`, 速度: `1e-3`
- 加偏: `2e-3`, 陀偏: `1e-6`

### 4.3 IMU 预测 (Forward)

`Forward()` (`svo/src/schur_vins.cpp:665-715`):

对每个 IMU 数据:
1. 跳过已处理数据 (行 686)
2. 正常数据：调用 `Prediction(deltaT, ...)` (行 695)
3. 跨图像边界的 IMU：线性插值 (行 700-713)
4. 结果写入帧: `frame->T_f_w_ = T_cam_imu * T_world_imu.inverse()` (行 718-720)

#### Prediction() — EKF 预测

(`svo/src/schur_vins.cpp:157-211`):

**F 矩阵** (15×15, 误差状态转移):
- `F(0,12)` = -rot (d(theta)/d(bg))
- `F(3,6)` = I (d(p)/d(v))
- `F(6,0)` = -Skew(rot*acc) (d(v)/d(theta))
- `F(6,9)` = -rot (d(v)/d(ba))

**G 矩阵** (15×12, 噪声驱动):
- `G(0,0)` = -rot (陀螺噪声→姿态)
- `G(6,3)` = -rot (加速噪声→速度)
- `G(9,6)`, `G(12,9)` = I (随机游走)

**Phi 矩阵** (三阶近似):
```
Phi = I + F*dt + 0.5*(F*dt)^2 + (1/6)*(F*dt)^3
```

**协方差传播** (行 193-199):
```
P = Phi * P * Phi^T + Phi * G * Q * G^T * Phi^T * dt
```

同时传播交叉协方差（窗口位姿与 IMU 状态之间）。

#### PredictionState() — 状态名义值

(`svo/src/schur_vins.cpp:120-155`):

使用 **4 阶 Runge-Kutta 积分**:
```
k1: dv/dt = q*acc + g, dp/dt = v
k2: dv/dt = q_half*acc + g, dp/dt = v + k1_dv*dt/2
k3: dv/dt = q_half*acc + g, dp/dt = v + k2_dv*dt/2
k4: dv/dt = q_full*acc + g, dp/dt = v + k3_dv*dt
→ 最终: p += dt/6*(k1+2k2+2k3+k4)
```

姿态: `q_new = q * Quaternion::exp(w*dt)`

### 4.4 视觉更新 (Backward)

`Backward()` (`svo/src/schur_vins.cpp:740-783`):

1. `AugmentState()` (行 97-118): 新 AugState 插入窗口，协方差扩维 6×6
2. `RegisterPoints()` (行 589-616): 帧中地图点注册到 `local_pts`
3. `Solve3()` (行 213-422): **核心 Schur Complement 更新**
4. 结果写回帧 (行 750-757)
5. 外点剔除 (行 763-764)

### 4.5 Schur Complement 机制详解 — 核心创新

`Solve3()` (`svo/src/schur_vins.cpp:213-422`)，下面逐步解释。

#### 4.5.1 观测残差定义

每个地图点 Pw 在每个相机帧的投影残差 (`schur_vins.cpp:287-288`):
```
r = (obs.xy - Pc.xy / Pc.z) * (focal_length / level_scale)
```
其中:
- `Pc` = 地图点在相机系的坐标
- `Pi = state.quat.inverse() * (Pw - state.pos)` (IMU 系中的点)
- `Pc = q_i_c.inverse() * (Pi - t_i_c)` (相机系中的点)

#### 4.5.2 Jacobian 链式求导

每个观测的 Jacobian 分解为 (`schur_vins.cpp:304-334`):

**a) 重投影对相机系中点的 Jacobian** `dr/dPc` (2×3, 行 313-319):
```
dr_dpc(0,0) = 1/Pc.z;  dr_dpc(1,1) = 1/Pc.z
dr_dpc(0,2) = -Pc.x/Pc.z^2;  dr_dpc(1,2) = -Pc.y/Pc.z^2
```

**b) 相机系中点对照相机位姿的 Jacobian** `dPc/dPos` (3×6, 行 322-324):
```
dpc_dpos.leftCols(3)  = -R_i_c^T * Skew(Pi) * R_w_i
dpc_dpos.rightCols(3) = -R_c_w
```

**c) 相机系中点对地图点位置的 Jacobian** `dPc/dPw` (行 324):
```
dpc_dPw = R_c_w  // = (R_c_i * R_i_w) 的转置
```

**组合 Jacobian** (行 330-333):
```
jx = dr_dpc * dpc_dpos  // 残差对相机状态 (2×6)
jf = dr_dpc * R_c_w     // 残差对地图点 (2×3)
```

#### 4.5.3 Hessian/梯度累积

对每点每观测累加 (行 348-353):

| 符号 | 代码变量 | 维度 | 含义 |
|------|----------|------|------|
| H_ss | Amtx | 6N×6N | 状态-状态 Hessian, = Σ Jx^T Jx |
| b_s | Bvct | 6N×1 | 状态梯度, = Σ Jx^T r |
| H_ll | pt->V | 3×3 | 地标点 Hessian, = Σ Jf^T Jf |
| b_l | pt->gv | 3×1 | 地标点梯度, = Σ Jf^T r |
| H_sl | feature->W | 6×3 | 交叉项, = Jx^T Jf |

完整正规方程:
```
[ H_ss   H_sl  ] [ dx_s ]   [ b_s ]
[ H_sl^T H_ll  ] [ dx_l ] = [ b_l ]
```

#### 4.5.4 Schur Complement 边缘化

(`schur_vins.cpp:375-415`):

对每个地图点执行消元:
```
H_marg = H_ss - H_sl * H_ll^{-1} * H_sl^T
b_marg = b_s  - H_sl * H_ll^{-1} * b_l
```

代码实现 (行 397-413):
```cpp
const Matrix6o3d WVinv = W_i * pt->Vinv;       // = H_sl_i * H_ll^{-1}
Matrix6d p2p_schur = WVinv * W_j.transpose();  // = H_sl_i * H_ll^{-1} * H_sl_j^T
Amtx.block -= p2p_schur;   // H_ss -= H_sl * H_ll^{-1} * H_sl^T
Bvct.segment -= WVinv * gv; // b_s -= H_sl * H_ll^{-1} * b_l
```

**关键**: Schur Complement 将超大规模的联合优化**精确等价地压缩**为仅含滑动窗口位姿状态的小规模问题 (N×6 维)。地标贡献被完全吸收到 H_marg 和 b_marg 中，无信息丢失。

#### 4.5.5 EKF 状态更新

`StateUpdate()` (`svo/src/schur_vins.cpp:424-450`):

标准 EKF 公式:
```
S = H * P * H^T + R         // 创新协方差
K = P * H^T * S^{-1}        // 卡尔曼增益
dx = K * gradient            // 状态修正量
P = (I - K*H) * P            // 协方差更新
```

**重要设计选择**: Hessian 矩阵同时充当观测矩阵和观测噪声 (`R = H`)。这是因为残差已通过 `obs_invdev` 白化:
```cpp
r *= obs_invdev;  // schur_vins.cpp:302
```
使得 `E[r r^T] = I`，此时 `H = J^T J` 自然等价于 `J^T * I^{-1} * J = J^T R^{-1} J`。

#### 4.5.6 状态修正

`StateCorrection()` (`svo/src/schur_vins.cpp:452-483`):

```cpp
// IMU 状态
curr_state->quat = dq * curr_state->quat;  // 左乘更新
curr_state->pos += dx.segment<3>(3);
curr_state->vel += dx.segment<3>(6);
curr_state->ba  += dx.segment<3>(9);
curr_state->bg  += dx.segment<3>(12);
// 窗口状态
item.second->quat = dq * item.second->quat;
item.second->pos  += dAX.tail<3>();
```

协方差: `P = (I - K*H) * P`，并对称化以保证数值稳定 (行 479-482)。

### 4.6 IMU 处理方法

SchurVINS **不使用**标准的预积分框架 (如 Forster et al., IJRR 2015):

- **连续时间 EKF 预测**: 直接用原始 IMU 数据进行 RK4 积分 + EKF 传播
- **无 delta 量**: 不使用 dR_ij, dv_ij, dp_ij 等预积分量
- **偏差在线修正**: ba, bg 实时估计，每次 Prediction 中: `acc = raw - ba; gyr = raw - bg`

优点: 避免偏差变化时的预积分 Jacobian 更新
缺点: 每次 IMU 数据需直接积分（但 IMU 频高，计算可忽略）

### 4.7 Schur Complement vs 常规 EKF/MSCKF

| 维度 | 标准 EKF | MSCKF | SchurVINS |
|------|----------|-------|-----------|
| 地标处理 | 增入状态向量 | 零空间投影消除 | Schur Complement 边缘化 |
| 观测残差 | 逐观测独立 | 所有观 零空间投影 | 累积梯度+Hessian 完备残差 |
| 数学模型 | 单观测线性化 | 线性化投影 | 精确等价的边缘化 |
| 信息利用 | 逐点更新 | 一次性批处理 | 一次性批处理 |
| 协方差 | 显式扩展 | 标准 EKF | 标准 EKF |

**核心优势**:
1. **完备信息**: 梯度（一阶）+ Hessian（二阶）显式建模，不丢失信息
2. **观测协方差显式**: `obs_dev` 参数 + Huber 加权
3. **与优化等价**: Schur complement 边缘化在数学上与滑动窗口 BA 中的边缘化完全等价
4. **计算轻量**: 每个点的 V (3×3) 求逆极快 (`schur_vins.cpp:377`)

---

## 5. 局部优化与全局优化

### 5.1 EKF 滑动窗口

有滑动窗口，但并非传统 BA 框架，而是 **EKF 中的相机状态窗口**：
- 窗口大小: `window_size=4` 个 AugState
- `Management()` (`schur_vins.cpp:485-523`): 移除状态并收缩协方差
- `KeyframeSelect()` (`schur_vins.cpp:541-587`):
  - 最新两帧共视点轨迹率 `common_cnt / curr_fts_cnt`
  - 轨迹率 < 0.70: 删最老帧 L1
  - 轨迹率 ≥ 0.70: 删次新帧 R2

### 5.2 地标 EKF 优化

`StructureOptimize()` (`svo/src/schur_vins.cpp:785-798`):
- 仅对已被 Schur complement 处理过的点 (`schur_pts_`)
- 调用 `Point::EkfUpdate()` (`svo_common/src/point.cpp:335-416`)
  - 利用**不在滑动窗口内的历史关键帧观测**计算残差 + Hessian
  - 结合 dsw_ (状态修正量) 修正点位置
  - EKF 更新: `pos_new = pos + K * residual`

### 5.3 全局优化 / 回环检测

保留 SVO 2.0 可选后端，但**默认全部关闭**:
- **Ceres 后端**: `use_ceres_backend: false` (vio_stereo.yaml:129)
- **回环检测** (DBoW2): `runlc: false` (vio_stereo.yaml:165)
- **全局地图** (iSAM2): `CATKIN_IGNORE` 文件禁用

默认配置是**纯 VIO 系统**，与轻量化设计一致。

---

## 6. 与 open_vins MSCKF 的对比

### 6.1 open_vins MSCKF 零空间投影

核心步骤:
1. 构造联合观测矩阵 H = [H_x | H_f]
2. SVD/Givens 旋转找 H_f 的**左零空间**矩阵 N:  N * H_f = 0
3. 投影观测: H' = N * H_x,  r' = N * r
4. 用 H', r' 进行标准 EKF 更新

### 6.2 SchurVINS 的 Schur Complement

不构造零空间，而是构造海森并执行 Schur complement:

| | open_vins | SchurVINS |
|--|-----------|-----------|
| 数学原理 | 零空间 N^T H_f = 0 | Schur complement H - H_sl H_ll^{-1} H_sl^T |
| 计算方式 | Givens QR | 3×3 逆 + 矩阵乘法 |
| 信息等价性 | N^T 的 span 与消元等价 | 精确边缘化 |
| 数值 | SVD 稳定 | 小矩阵逆稳定 |

### 6.3 理论区别

- MSCKF: **零空间投影** → 等价于**条件化**
- SchurVINS: **Schur Complement** → 等价于**边缘化**
- 在线性高斯下: **两者完全等价**（都得到 H_x - H_xf H_ff^{-1} H_xf^T）
- 实现差异:
  - MSCKF: 找零空间基，Givens 优化计算量
  - SchurVINS: 每个地标独立 3×3 逆，天然可并行

**区别总结**:
1. **信息形式**: SchurVINS 显式保留 Hessian/Gradient，便于后续 Point EkfUpdate
2. **工程实现**: 3×3 逆 vs Givens QR，各有优势
3. **扩展性**: SchurVINS 天然支持地标 EKF (Point::EkfUpdate)，MSCKF 需单独设计

---

## 7. 优缺点分析

### 7.1 算法优势

1. **高精度**: Schur complement 构建**完备残差模型** (梯度 + Hessian + 观测协方差)，无信息损失
2. **轻量计算**:
   - 地标 3×3 逆 (`pt->Vinv = pt->V.inverse()`, schur_vins.cpp:377)
   - EKF 状态 < 40 DOF
   - 无 BA 迭代，每帧一次 EKF
3. **Huber 鲁棒**: 在 Schur complement 前已加权 (schur_vins.cpp:294-301)
4. **FEJ**: First-Estimate Jacobian 避免不一致性 (`quat_fej`, `pos_fej`)

### 7.2 算法局限

1. **无回环**: 默认配置无回环检测，长时间运行有漂移
2. **线性化误差**: EKF 一次线性化，对非线性场景敏感（相比迭代优化）
3. **窗口固定**: window_size=4，固定小窗口限制了历史信息利用
4. **IMU 非预积分**: 依赖直接积分，偏差更新后需重新线性化
5. **特征依赖**: 依赖 SVO 的 FAST 角点 + 极线匹配，低纹理/运动模糊场景脆弱

### 7.3 工程优势

- 继承 SVO 2.0 成熟前端 (相机模型、曝光补偿、可视化管理)
- 多线程重投影 (svo/src/frame_handler_base.cpp:761-773)
- ROS 生态兼容性好
- 配置文件清晰分离 (vio_stereo.yaml)

### 7.4 工程局限

- SVO 2.0 代码量庞大，改造成本高
- 大量条件编译宏 (`#ifdef USE_SCHUR_VINS`)
- 依赖 OpenGV / GTSAM (若不使用某些功能可裁剪)
- GPLv3 许可限制了商用场景

### 7.5 适用场景

- **计算资源受限**平台 (无人机、嵌入式): 轻量 EKF
- **双目+IMU** 配置: 精度高，无需尺度估计
- **需要高频输出**的场景: 无 BA 迭代延迟
- **室内/小场景**: 无回环漂移可控

### 7.6 不适用场景

- **大场景长时间**: 无回环漂移累积
- **低纹理/低光照**: FAST 角点不足
- **高速运动**: 极线搜索范围受限
- **纯单目+IMU**: 初始化依赖复杂（5pt/Homography），需已知尺度

---

## 8. 对 phad_fusion 的关键参考

### 8.1 值得引入的设计模式

1. **Schur Complement 等价 EKF 更新** (核心)
   - schur_vins.cpp:213-422 (Solve3)
   - 将特征点 Hessian 边缘化后做 EKF，兼具优化精度与滤波速度
   - 建议: phad_fusion 可复刻此机制，状态 = (IMU + 相机外参 + 窗口位姿)

2. **地标独立 EKF** (point.cpp:335-416)
   - 每个地标维护自己的 V, Vinv, gv, W
   - 利用历史关键帧 + 最新状态修正量进行轻量更新
   - 建议: phad_fusion 可对 GPS/UWB 锚点使用类似模式

3. **白化残差 + Hessian=观测矩阵** (schur_vins.cpp:302, StateUpdate:438)
   - R = H (Hessian 作为观测协方差)
   - 简化了 EKF 实现，数学等价于信息矩阵形式
   - 建议: 可借鉴此简化

4. **滑动窗口 FEJ** (schur_vins.h:58-60, schur_vins.cpp:204-206)
   - 状态扩增时使用 First-Estimate 线性化点
   - 避免多观测的线性化不一致

5. **KeyframeSelection() 策略** (schur_vins.cpp:541-587)
   - 基于共视点轨迹率决定删除哪帧
   - 简单有效的关键帧选择

### 8.2 应避免的陷阱

1. **EKF 一次线性化局限性**
   - Solve3() 中 Jacobian 只在当前名义值计算 (schur_vins.cpp:322-333)
   - 非线性严重时 (鱼眼、大畸变) 可能发散
   - 建议: phad_fusion 对强非线性传感器考虑迭代 EKF (IEKF)

2. **协方差矩阵维护复杂** (schur_vins.cpp:485-523)
   - Management() 中手动平移协方差矩阵块，易出错
   - 建议: 使用 Eigen block 操作时充分单元测试

3. **IMU 非预积分的低效** (schur_vins.cpp:120-155)
   - PredictionState() 用 RK4 直接积分原始 IMU → 精度可，但无法批量积分
   - 建议: phad_fusion 可考虑使用现代 `ImuFactor` (GTSAM 风格) 提高效率

4. **外参未在线估计**
   - schur_vins.cpp:326-328 中的 `jext` (外参 Jacobian) 被注释掉
   - 相机-IMU 外参依赖离线标定精度
   - 建议: 若 phad_fusion 有多传感器，考虑在线外参估计

5. **线程安全问题** (schur_vins.cpp:87)
   - `msg_queue_mtx` + `condition_variable` 仅在头文件声明，使用未覆盖所有关键路径
   - 建议: 对所有共享状态 (cov, states_map, local_pts) 加入 mutex 保护

### 8.3 直接映射建议表

| SchurVINS 模块 | phad_fusion 映射位置 | 复用程度 |
|----------------|---------------------|---------|
| SchurVINS::Solve3() | CameraRig::SchurUpdate() | 核心算法直接借用 |
| Point::EkfUpdate() | Anchor::EkfUpdate() | 对 GPS/UWB 锚点 EKF |
| SchurVINS::Prediction() | IMUPropagator::Predict() | 可复刻误差状态 F/G |
| FrameHandlerBase pipeline | FusionPipeline::Process() | 流程结构借鉴 |
| OccupandyGrid2D | 可选 (非必须) | 若用视觉特征就借 |

---

## 9. 数据管线

### 9.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | ROS话题/接口 | 负责模块 |
|--------|------|----------|-------------|----------|
| 相机 (stereo) | 10-30 Hz | `sensor_msgs::Image` → `cv::Mat` (Gray) | `/cam0/image_raw`, `/cam1/image_raw` → `SvoInterface::stereoCallback` | `FrameHandlerStereo` |
| IMU (6 轴) | 100-200 Hz | `acc[3]` [m/s²], `gyr[3]` [rad/s] | `/imu0` → `SvoInterface::imuCallback` | `ImuHandler`, `SchurVINS` |

### 9.2 相机数据处理管线

#### 原始数据
- **图像格式**: `cv::Mat` (Gray, CV_8UC1), ROS `cv_bridge::toCvShare`
- **相机模型**: `PinholeProjection` / `EquidistantProjection` (继承 `CameraBase`)，从 `calib/camera_rig.yaml` 加载
- **外参**: `T_cam0_imu` (离线标定), `T_cam1_cam0` (双目外参), 不支持在线估计
- **时间偏移**: `imu_calib_.delay_imu_cam` (IMU→Camera 延迟)，YAML 配置

#### 图像到 Frame 转换
`FrameHandlerBase::addImageBundle` (`svo/src/frame_handler_base.cpp:189`):
1. 创建 `Frame` 对象，传入左右灰度图
2. 构建图像金字塔: `n_pyr_levels` 层 (默认 3 层, 配置 `img_align_max_level: 4`)
3. 无传统图像去畸变 —— 保留原始图像用于直接法

#### 特征提取 (`FastDetector`)
`shurvins_estimator/src/fast_detector.cpp`:
- 算法: **FAST 角点** (OpenCV, 阈值 `detector_threshold_primary: 10`)
- 类型: `kCorner` (主要使用), 也支持 `kEdgelet` (关闭), Shi-Tomasi
- 最大特征数: `max_fts: 180`
- 网格策略: `OccupandyGrid2D`, 单元大小 `grid_size: 25`，每格最多 1 点
- 重投影阶段已占用网格被标记，新特征不入已占格

#### 双目立体匹配 (`StereoTriangulation::compute`)
`svo/src/stereo_triangulaion.cpp:23`:
1. 左图 FAST 角点提取 (行 41)
2. 沿极线在右图搜索: `matcher.findEpipolarMatchDirect()` (行 101-102, 直接法极线匹配)
3. 立体三角化: 计算 3D 坐标 (行 108-125)
4. 创建 `Point` 对象

#### 地图点重投影匹配 (`projectMapInFrame`)
- 将已有地图点投影到当前帧
- 在投影位置做直接法对齐
- 成功匹配的特征用于 EKF 视觉更新

#### 外点剔除 (EKF 后)
`svo/src/schur_vins.cpp:801-915`:
- `RemoveOutliers()`: 4.0px 阈值移除当前帧外点 (行 868-869)
- `RemovePointOutliers()`: 3.0px 平均重投影误差标记 `local_status_` (行 801-854)

### 9.3 IMU 数据处理管线

#### 原始数据
- **测量**: `ImuMeasurement {acc[3], gyr[3], timestamp}` (`svo_common/include/svo/common/imu.h`)
- **噪声参数**: 陀螺噪声 `gyr_noise`, 加速噪声 `acc_noise`, 随机游走 `gyr_rw`, `acc_rw` (YAML)
- **重力**: 由加速度计静止测量或配置提供

#### 接收与缓冲
`imuCallback` → `ImuHandler::addImuMeasurement` (`svo/include/svo/imu_handler.h:187`):
- 存入 `measurements_` (deque，最新数据在头部)
- `waitTill(frame_stamp)`: 等待 IMU 数据到达

#### 边缘 IMU 提取 (`ImuHandler::getMeasurementsContainingEdges`)
`svo/src/imu_handler.cpp:105`:
- 查找时间戳 < `frame_timestamp - delay_imu_cam` 的测量
- 回退一个元素保证插值边界值 (行 132-136)
- 反序存储 (最新在前) 到 `frame_bundle→imu_datas_`

#### Forward 插值
`SchurVINS::Forward()` (`svo/src/schur_vins.cpp:665-715`):
- 若 IMU 时间 < 图像时间: 直接使用
- 若 IMU 时间 > 图像时间: 线性插值到图像时间戳 (行 700-713)

#### IMU 状态预测 (`SchurVINS::Prediction`)
`svo/src/schur_vins.cpp:157-211`:

**状态名义值 (RK4 积分)** (`PredictionState`, `svo/src/schur_vins.cpp:120-155`):
```
k1: dv/dt = q*acc_world + g,  dp/dt = v
k2: dv/dt = q_half*acc + g,    dp/dt = v + k1_dv*dt/2
k3: dv/dt = q_half*acc + g,    dp/dt = v + k2_dv*dt/2
k4: dv/dt = q_full*acc + g,    dp/dt = v + k3_dv*dt
p += dt/6*(k1+2k2+2k3+k4)
```
- 姿态: `q_new = q * Quaternion::exp(w_raw*dt)`，**无预积分**，直接积分原始 IMU

**误差状态转移 (三阶近似):**
- F 矩阵 (15×15): `F(0,12)=-rot`, `F(3,6)=I`, `F(6,0)=-Skew(rot*acc)`, `F(6,9)=-rot` 等
- G 矩阵 (15×12): `G(0,0)=-rot`, `G(6,3)=-rot`, `G(9,6)=I`, `G(12,9)=I`
- Phi 矩阵: `Phi = I + F*dt + 0.5*(F*dt)² + (1/6)*(F*dt)³`
- 协方差: `P = Phi*P*Phi^T + Phi*G*Q*G^T*Phi^T*dt`
- 同时传播交叉协方差

#### 结果写入帧
`frame->T_f_w_ = T_cam_imu * T_world_imu.inverse()` (行 718-720)

### 9.4 算法消费：Schur Complement EKF 观测因子

#### 观测残差 (Backward / Solve3)
`svo/src/schur_vins.cpp:213-422`:

**重投影残差** (2 维, 白化后):
```
r = (obs.xy - Pc.xy/Pc.z) * (focal_length / level_scale) * obs_invdev
```
其中 `obs_invdev` 为观测噪声标准差倒数（白化: `E[r r^T]=I`）:
```
P_i = q_iw⁻¹ * (P_w - p_iw)         // World→IMU
P_c = q_ic⁻¹ * (P_i - t_ic)          // IMU→Camera
```

**Jacobian 链式求导:**
```
dr/dPc:  1/Pc.z,  -Pc.x/Pc.z²,  -Pc.y/Pc.z²        (2×3)
dPc/dPose: [-R_ic^T * Skew(Pi)*R_wi, -R_cw]         (3×6)
dPc/dPw: R_cw                                         (3×3)
→ Jx = dr_dpc * dPc_dPose (2×6), Jf = dr_dpc * R_cw (2×3)
```

**Hessian 累积** (每点每观测):
- `H_ss += Jx^T*Jx` (6N×6N, 状态-状态)
- `b_s += Jx^T*r` (6N×1, 状态梯度)
- `H_ll += Jf^T*Jf` (3×3, 每地图点独立)
- `b_l += Jf^T*r` (3×1)
- `H_sl += Jx^T*Jf` (6×3, 交叉项)

**Schur Complement 消去地标:**
```
H_marg_i,j = H_ss_i,j - H_sl_i * H_ll⁻¹ * H_sl_j^T   (6×6 每对)
b_marg_i = b_s_i - H_sl_i * H_ll⁻¹ * b_l              (6×1)
```
代码: `p2p_schur = WVinv * W_j.transpose()`, `Bvct -= WVinv * gv`

**EKF 更新** (`StateUpdate`, `svo/src/schur_vins.cpp:424-450`):
```
S = H_marg * P * H_marg^T + R     // R = H_marg (因残差已白化)
K = P * H_marg^T * S⁻¹
dx = K * gradient
P = (I - K*H_marg) * P
```

**状态修正** (`StateCorrection`, `svo/src/schur_vins.cpp:452-483`):
```cpp
curr_state->quat = dq * curr_state->quat;  // 左乘
curr_state->pos += dx[3:6];                // 位置
curr_state->vel += dx[6:9];                // 速度
curr_state->ba += dx[9:12];                // 加速偏置
curr_state->bg += dx[12:15];               // 陀螺偏置
```
- 协方差对称化: `P = 0.5*(P + P^T)`

**地标 EKF** (`Point::EkfUpdate`, `svo_common/src/point.cpp:335-416`):
- 利用不在窗口内的历史 KF 观测
- 结合 `dsw_` (状态修正量) + 各帧 Jacobian 做独立 EKF 更新
- `pos_new = pos + K * residual`

#### 信息矩阵
- 视觉噪声: 白化后 `R = H_marg` (残差方差=1)，观测标准差 `obs_invdev` 由配置
- IMU 噪声: `Q = diag(gyr_noise², acc_noise², gyr_rw², acc_rw²)` (12×12)
- Huber 核: 在 Schur complement 前加权 (行 294-301)

### 9.5 跨传感器协同

#### 时间同步
- 主时钟: 图像时间戳
- IMU-图像对齐: `getMeasurementsContainingEdges` 基于 `delay_imu_cam` 偏移
- Forward 插值: IMU 线性插值到图像时刻

#### 数据缓冲策略
- IMU: `ImuHandler::measurements_` (deque, mutex + condition_variable)
- 窗口管理: `window_size=4` AugState, `Management()` 移除状态并收缩协方差
- `KeyframeSelect()`: 共视点跟踪率 < 0.70 → 删最老帧; 否则 → 删次新帧

#### 初始化管线
1. **状态初始化** (`InitState`): IMU 加速度归一化 → `Quaternion::FromTwoVectors(acc_norm, grav_norm)` → pos/vel/bias=0
2. **初始位姿** (`setInitialPose`): IMU 旋转先验 / 加速度计重力方向 / identity
3. **双目初始化** (`processFirstFrame`): `schurvinsForward` → `StereoInit::addFrameBundle` (立体匹配+三角化) → 关键帧加入地图 → 深度滤波器初始化 → `schurvinsBackward`
4. **协方差初始化** (`InitCov`): diag(att=1e-4, pos=1e-3, vel=1e-3, ba=2e-3, bg=1e-6)

#### 降级与异常处理
- FEJ (First-Estimate Jacobian): `quat_fej`, `pos_fej` 状态增广时使用首次线性化点
- EKF 外点剔除: 4.0px (当前帧) / 3.0px (平均)
- 无回环检测 (默认 `runlc=false`)
- 无全局优化 (`use_ceres_backend=false`, iSAM2 禁用)
