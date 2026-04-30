# IC-GVINS 源码深度分析

> 全称：A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
> 团队：武汉大学 i2Nav 课题组
> 论文：IEEE Robotics and Automation Letters (RA-L), 2022
> 主页：https://github.com/i2Nav-WHU/IC-GVINS

---

## 1. 数据接收与预处理

### 1.1 数据入口：ROS Topic 订阅

IC-GVINS 通过 ROS 接口接收传感器数据。入口文件为 `ROS/fusion_ros.cc`，订阅三个 topic：

| 传感器 | ROS消息类型 | 默认Topic | 频率 |
|--------|------------|-----------|------|
| 相机 | sensor_msgs/Image | /cam0 | 20 Hz |
| IMU | sensor_msgs/Imu | /imu0 | 200 Hz |
| GNSS-RTK | sensor_msgs/NavSatFix | /gnss0 | 1 Hz |

三个回调函数 `imuCallback()`, `gnssCallback()`, `imageCallback()` 分别处理（`fusion_ros.cc:123-233`）。

### 1.2 GNSS 原始测量类型

IC-GVINS **不处理原始伪距/载波相位观测值**。它使用的 GNSS 测量是 RTK 解算后的绝对位置：

```
gnss_.blh[0] = gnssmsg->latitude * D2R;    // 纬度 (rad)
gnss_.blh[1] = gnssmsg->longitude * D2R;   // 经度 (rad)
gnss_.blh[2] = gnssmsg->altitude;          // 高程 (m)
gnss_.std[0] = sqrt(position_covariance[4]); // 北方向标准差 (m)
gnss_.std[1] = sqrt(position_covariance[0]); // 东方向标准差 (m)
gnss_.std[2] = sqrt(position_covariance[8]); // 地方向标准差 (m)
```

GNSS 数据结构定义在 `common/types.h:32-40`：
```cpp
typedef struct GNSS {
    double time;
    Vector3d blh;       // 纬度/经度/高程 (rad, rad, m)
    Vector3d std;       // NE方向位置标准差
    bool isyawvalid;    // 双天线航向是否有效
    double yaw;         // 双天线航向角 (rad)
} GNSS;
```

**结论：IC-GVINS 使用的是 GNSS 位置级测量（定位结果），不是原始观测值。属于松耦合中的位置融合。**

### 1.3 IMU 数据格式

IMU 数据以增量形式存储，定义于 `common/types.h:50-58`：
```cpp
typedef struct IMU {
    double time;
    double dt;
    Vector3d dtheta;    // 角增量 (rad), 前右下方向
    Vector3d dvel;      // 速度增量 (m/s)
    double odovel;      // 里程计速度增量
} IMU;
```

强调：**IMU 必须是前-右-下 (Front-Right-Down) 坐标系**。ROS 回调中直接将 `angular_velocity * dt` 和 `linear_acceleration * dt` 写入 dtheta/dvel（`fusion_ros.cc:137-142`）。

### 1.4 坐标框架约定

IC-GVINS 使用 **WGS84 经纬高 → 局部切平面 (Local Tangent Plane)** 的转换链：

**ECEF → BLH**：`Earth::ecef2blh()` (`common/earth.h:132-150`)
**BLH → ECEF**：`Earth::blh2ecef()` (`common/earth.h:117-130`)
**全局 LLH → 局部切平面**：`Earth::global2local()` (`common/earth.h:185-192`)

核心转换公式（`common/earth.h:185-191`）：
```cpp
static Vector3d global2local(const Vector3d &origin, const Vector3d &global) {
    Vector3d ecef0 = blh2ecef(origin);        // 站心原点→ECEF
    Matrix3d cn0e  = cne(origin);             // NED→ECEF 旋转矩阵
    Vector3d ecef1 = blh2ecef(global);        // 目标点→ECEF
    return cn0e.transpose() * (ecef1 - ecef0); // 转为局部 NED 坐标
}
```

**关键约定**：
- **世界坐标系原点**：第一个有效的 GNSS 定位位置（`ic_gvins.cc:207`）
- **局部坐标系**：NED（北-东-地），用 `blh` 的 [N, E, D] 三元素表示
- **IMU 坐标系**：前-右-下 (FRD)，与 NED 对齐
- **相机坐标系**：通过外参 `pose_b_c_` (body到camera的变换) 转换

GNSS 到达时直接转换为局部坐标（`ic_gvins.cc:216`）：
```cpp
gnss_.blh = Earth::global2local(integration_config_.origin, gnss_.blh);
```

### 1.5 时间同步

IC-GVINS **不是硬件同步**，而是通过以下机制处理时间对齐：

1. **时间戳统一为 GPS 周秒**：ROS UNIX 时间通过 `GpsTime::unix2gps()` 转换为 GPS week + weeksec（`fusion_ros.cc:128-130`）

2. **相机-IMU 时间偏差在线估计**：外参中包含 `td_b_c`（IMU到相机的时间延迟），作为优化变量：
   ```cpp
   // ic_gvins.cc:176-179
   td_b_c_ = config["cam0"]["td_b_c"].as<double>();
   // t_i = t_c + td
   ```
   视觉跟踪时图像时间戳会被补偿：`frame->setStamp(frame->stamp() + td)`（`ic_gvins.cc:529`）

3. **滑窗内 GNSS-IMU 时间对齐**：GNSS 时间戳插入到滑窗内最近的时间节点。允许的最小同步间隔为 0.025 秒（`MINMUM_SYNC_INTERVAL`）。若 GNSS 时间戳接近现有节点（< 0.025s），则通过 INS 速度对位置进行时间补偿后对齐（`ic_gvins.cc:829-886`）。

4. **IMU 丢失检测**：若 IMU 数据间隔超过 1.5 倍采样周期，自动补插虚拟 IMU 数据（`ic_gvins.cc:171-183`）。

---

## 2. 传感器融合架构

### 2.1 紧耦合 vs 松耦合

IC-GVINS 采用的是 **位置级紧耦合 (Tightly-Coupled at Position Level)**：

- GNSS、视觉、IMU 三个传感器在 **同一个 Ceres 非线性最小二乘问题** 中联合优化
- 不是先独立计算 VIO 再与 GNSS 融合
- 但 GNSS 不是用原始伪距/载波，而是用定位结果（位置残差）

**所以：对 GNSS 原始观测量而言是松耦合（只用位置），对整个系统而言是紧耦合（三传感器联合优化）。**

### 2.2 GNSS 观测如何融入优化

GNSS 作为一个 **绝对位置观测因子** 加入到滑窗优化中。因子结构：

- 连接到单个 IMU 状态节点的 pose 参数块（7 维：t[3] + q[4]）
- 残差为 3 维位置误差
- 通过杆臂补偿将 GNSS 天线位置关联到 IMU 位置

### 2.3 状态向量

IC-GVINS 估计的状态变量，定义在 `preintegration/integration_state.h:35-51`：

```cpp
typedef struct IntegrationState {
    double time;
    Vector3d p{0,0,0};          // IMU位置 (NED局部坐标, m)
    Quaterniond q{0,0,0,0};     // IMU姿态四元数 (IMU→NED)
    Vector3d v{0,0,0};          // IMU速度 (NED, m/s)
    Vector3d bg{0,0,0};         // 陀螺零偏 (rad/s)
    Vector3d ba{0,0,0};         // 加速度计零偏 (m/s^2)
    Vector3d s{0,0,0};          // IMU比例因子 (PPM)
    double sodo{0};             // 里程计比例因子
    Vector2d abv{0,0};          // 安装角
    Vector3d sg{0,0,0};         // 陀螺比例因子
    Vector3d sa{0,0,0};         // 加速度计比例因子
} IntegrationState;
```

**实际优化变量在滑窗内**（`ic_gvins.cc:144-150`）：
- `statedatalist_`：每个时间节点的 pose [7] + mix [9/10/12]
- `extrinsic_[8]`：相机-IMU外参 [t(3) + q(4) + td(1)]
- `invdepthlist_`：每个地图点的逆深度

Pose 参数块为 7 维 (t[3] + q[4])，使用自定义 `PoseParameterization`（继承 `ceres::LocalParameterization`）做流形优化，局部参数化为 6 维。

Mix 参数块包含速度+IMU误差参数，维度根据配置为 9, 10, 12 或 18。

地图点采用**逆深度参数化**（1 维参数块）。

### 2.4 IMU 预积分方法

IC-GVINS 实现了一套**可插拔的 IMU 预积分体系**：

1. **PreintegrationBase** (`preintegration_base.h`): 抽象基类
2. **PreintegrationNormal** (`preintegration_normal.h`): 标准预积分，不考虑地球自转
3. **PreintegrationEarth** (`preintegration_earth.h`): 考虑地球自转补偿的预积分
4. **PreintegrationOdo** / **PreintegrationEarthOdo**: 带里程计的变体

工厂方法 `Preintegration::createPreintegration()` 根据配置选择具体实现（`preintegration.h:57-73`）。

**地球自转补偿**：当 `iswithearth=true` 时，预积分补偿 `iewn`（地球自转角速度在导航系下的投影），存储位置序列 `pn_` 用于修正（`preintegration_earth.h:74-80`）。

预积分残差使用协方差逆的 Cholesky 分解作为信息矩阵：
```cpp
// preintegration_normal.cc:40-41
sqrt_information_ = Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(
    covariance_.inverse()).matrixL().transpose();
```

零偏变化通过一阶泰勒展开进行在线校正（jacobian-based correction），而非每次重新积分（`preintegration_normal.cc:55-58`）。

**关键设计**：`IMU_GRY_BIAS_STD = 7200 deg/hr` 和 `IMU_ACC_BIAS_STD = 20000 mGal` 作为硬编码的先验约束。当优化后的零偏变化超过 6 倍标准差时，触发重积分 `doReintegration()`（`ic_gvins.cc:1680-1695`）。

**注意：该框架不使用 GTSAM！** IC-GVINS 使用 **Ceres Solver** 作为非线性优化后端，所有因子（factor）均为自定义 Ceres CostFunction。本文后续所述的"因子"均指 Ceres AutoDiff/Manual CostFunction，概念上与 GTSAM factor 等价但 API 不同。

---

## 3. Ceres 因子设计（核心）

> **注意**：IC-GVINS 使用 Ceres Solver 而非 GTSAM。以下分析的"因子"均为 Ceres CostFunction 类。因子图通过 Ceres Problem + AddResidualBlock 构建，非线性优化通过 Ceres Solver 求解。

### 3.1 GNSS 位置因子

**文件**：`factors/gnss_factor.h:31-76`

**继承关系**：`GnssFactor : public ceres::SizedCostFunction<3, 7>`
- 残差维度：3（位置 NED）
- 参数块：1 个 7 维 pose 参数块 [tx, ty, tz, qx, qy, qz, qw]

**完整类定义**：
```cpp
class GnssFactor : public ceres::SizedCostFunction<3, 7> {
public:
    explicit GnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3],
                      parameters[0][4], parameters[0][5]};

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);

        // 残差 = IMU位置 + 旋转 * 杆臂 - GNSS测量位置
        error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

        // 信息矩阵 = diag(1/std_N, 1/std_E, 1/std_D)
        Matrix3d sqrt_info_ = Matrix3d::Zero();
        sqrt_info_(0, 0)    = 1.0 / gnss_.std[0];
        sqrt_info_(1, 1)    = 1.0 / gnss_.std[1];
        sqrt_info_(2, 2)    = 1.0 / gnss_.std[2];

        error = sqrt_info_ * error;  // 加权残差

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>
                    jacobian_pose(jacobians[0]);
                jacobian_pose.setZero();

                // ∂e/∂p = I
                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
                // ∂e/∂θ = -R * [lever]×
                jacobian_pose.block<3, 3>(0, 3) =
                    -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);

                jacobian_pose = sqrt_info_ * jacobian_pose;
            }
        }
        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};
```

**残差数学模型**：

```
e_GNSS = W * (p_IMU + R(q) * l_ant - p_GNSS_measured)

其中:
  p_IMU ∈ R^3    : IMU在局部NED坐标系中的位置
  R(q)  ∈ SO(3)  : IMU到NED的旋转矩阵
  l_ant ∈ R^3    : GNSS天线在IMU坐标系中的杆臂
  p_GNSS ∈ R^3   : GNSS测量的局部NED位置
  W = diag(1/σ_N, 1/σ_E, 1/σ_D) : 测量噪声加权矩阵
```

**雅可比**：
- 对位置的雅可比：`∂e/∂p = I`
- 对姿态的雅可比：`∂e/∂θ = -R(q) * [l_ant]_×`（杆臂的反对称矩阵）

**连接到优化问题**（`ic_gvins.cc:1891-1909`）：
```cpp
auto factor = new GnssFactor(data, antlever_);
auto id = problem.AddResidualBlock(factor, loss_function,
                                   statedatalist_[index].pose);
```

第一次优化使用 HuberLoss(1.0) 核函数，粗差剔除后第二次不使用核函数。

### 3.2 视觉重投影因子

**文件**：`factors/reprojection_factor.h:36-158`

**继承关系**：`ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>`
- 残差维度：2（归一化相机坐标下的投影误差）
- 参数块：5 个
  - Pose_i [7]：参考帧 IMU 位姿
  - Pose_j [7]：观测帧 IMU 位姿
  - Extrinsic [7]：相机-IMU外参
  - InvDepth [1]：地图点逆深度
  - Td [1]：时间延时

**重投影链路**（归一化相机坐标下计算）：

```
1. 归一化坐标：pts_c_0 = pts0_td / invdepth  (逆深度恢复3D点)
2. IMU坐标系： pts_b_0 = R_bc * pts_c_0 + t_bc
3. 世界坐标系： pts_n   = R_0 * pts_b_0 + p_0
4. 目标帧IMU： pts_b_1 = R_1^T * (pts_n - p_1)
5. 目标帧相机：pts_c_1 = R_cb * (pts_b_1 - t_bc)
6. 归一化残差：residual = (pts_c_1 / pts_c_1.z).head(2) - pts1_td.head(2)
```

其中时间延时的补偿（`reprojection_factor.h:73-74`）：
```cpp
pts_0_td = pts0_ - (td - td0_) * vel0_;  // 参考帧特征点速度补偿
pts_1_td = pts1_ - (td - td1_) * vel1_;  // 观测帧特征点速度补偿
```

### 3.3 IMU 预积分因子

**文件**：`preintegration/preintegration_factor.h:30-74`

**继承关系**：`PreintegrationFactor : public ceres::CostFunction`
- 残差维度：由 `preintegration_->numResiduals()` 决定（标准 15 维）
- 参数块：4 个 — Pose_0 [7], Mix_0 [N], Pose_1 [7], Mix_1 [N]

该因子是一个**通用包装器**（wrapper），将 `PreintegrationBase` 的方法映射到 Ceres CostFunction 接口（`preintegration_factor.h:45-68`）：
```cpp
bool Evaluate(...) const override {
    IntegrationState state0, state1;
    preintegration_->constructState(parameters, state0, state1);
    preintegration_->evaluate(state0, state1, residuals);
    // ... 计算雅可比 ...
}
```

**预积分残差公式**（`preintegration_normal.cc:61-67`）：
```
e_p  = R_0^T * (p_1 - p_0 - v_0*Δt - 0.5*g*Δt^2) - p_corrected
e_v  = R_0^T * (v_1 - v_0 - g*Δt) - v_corrected
e_q  = 2 * (q_corrected^{-1} * R_0^T * q_1).vec()
e_bg = bg_1 - bg_0
e_ba = ba_1 - ba_0
```

其中 `p_corrected`, `v_corrected`, `q_corrected` 是经过零偏一阶校正的预积分量。

### 3.4 IMU 误差先验因子

**文件**：`preintegration/imu_error_factor.h:30-94`

**继承关系**：`ImuErrorFactor : public ceres::CostFunction`
- 残差维度：6（无里程计）或 7（有里程计）
- 参数块：1 个 Mix 参数块

```cpp
// bg, ba 的正则化
for (size_t k = 0; k < 3; k++) {
    residuals[k+0] = parameters[0][k+3] / IMU_GRY_BIAS_STD;  // 7200 deg/hr
    residuals[k+3] = parameters[0][k+6] / IMU_ACC_BIAS_STD;  // 20000 mGal
}
```

这是对 IMU 零偏的**软约束**，防止零偏估计漂移过大。硬编码的标准差为 7200 deg/hr 和 20000 mGal。

### 3.5 边缘化因子

**文件**：`factors/marginalization_factor.h:31-105`

**继承关系**：`MarginalizationFactor : public ceres::CostFunction`
- 封装了边缘化先验信息（线性化残差 + 线性化雅可比）
- 使用舒尔补（Schur complement）将移除的状态量边缘化为保留状态量的先验

### 3.6 位姿局部参数化

**文件**：`factors/pose_parameterization.h:30-66`

**继承关系**：`PoseParameterization : public ceres::LocalParameterization`
- Global size: 7 (t[3] + q[4])
- Local size: 6 (dt[3] + dθ[3])
- Plus 操作：`p' = p + dp`, `q' = (q * dq).normalized()`，其中 dq = exp(dθ)

### 3.7 因子图结构

每个优化周期构建的 Ceres Problem 包含以下残差块：

```
状态节点:
  TimeNode_k: [pose(7), mix(N)]   (k = 0, 1, ..., K)
  Extrinsic: [7] (或 constant)
  Td: [1] (或 constant)
  InvDepth_id: [1] × M 个地图点

因子 (残差块):
  ┌── PreintegrationFactor(0) ─── [pose_0, mix_0, pose_1, mix_1]
  │   PreintegrationFactor(1) ─── [pose_1, mix_1, pose_2, mix_2]
  │   ...
  │   PreintegrationFactor(K-1) ─ [pose_K-1, mix_K-1, pose_K, mix_K]
  │
  ├── GnssFactor(t_k) ─────────── [pose_k]    (当 GNSS 对齐时间节点 k)
  │
  ├── ReprojectionFactor ──────── [pose_ref, pose_obs, extrinsic, invdepth, td]
  │   (每个地图点的每个观测)
  │
  ├── ImuErrorFactor ──────────── [mix_K]     (IMU零偏软约束)
  │
  └── MarginalizationFactor ───── [保留的参数块]  (边缘化先验)
```

### 3.8 优化策略

**Ceres Solver 两层优化**（`ic_gvins.cc:1130-1239`）：

1. **第一轮**（1/4 总迭代次数）：
   - 使用 HuberLoss(1.0) 核函数
   - 线性求解器：DENSE_SCHUR
   - 信任域：LEVENBERG_MARQUARDT
   - 4 线程并行

2. **粗差剔除**：
   - GNSS：Chi-squared 检验阈值 7.815（3 自由度, p=0.05），超标则 reweight
   - 视觉：Chi-squared 阈值 5.991（2 自由度），超标则移除残差块

3. **第二轮**（3/4 总迭代次数）：
   - 移除 GNSS 核函数（无 loss function）
   - 视觉粗差已被移除

**注意：IC-GVINS 不使用 ISAM2 或批量优化，而是使用基于 Ceres 的滑动窗口优化 + 边缘化。**

---

## 4. 位姿计算

### 4.1 初始化流程

IC-GVINS 的初始化分阶段进行，状态机定义于 `ic_gvins.h:48-55`：

```
GVINS_ERROR → GVINS_INITIALIZING → GVINS_INITIALIZING_INS
                                  → GVINS_INITIALIZING_VIO
                                  → GVINS_TRACKING_INITIALIZING
                                  → GVINS_TRACKING_NORMAL
```

**第一阶段：GNSS/INS 初始化** (`gvinsInitialization()`, `ic_gvins.cc:584-691`)：

1. **零速检测**：收集两个 GNSS 时刻之间的 IMU 数据，通过陀螺/加速度计阈值判断是否静止
2. **陀螺零偏估计**（静止时）：取陀螺数据的均值作为初始零偏
3. **重力调平**（静止时）：通过加速度计均值计算初始横滚/俯仰角：
   ```cpp
   initatt[0] = -asin(fb[1] / gravity);  // roll
   initatt[1] = asin(fb[0] / gravity);   // pitch
   ```
4. **航向初始化**：
   - 优先使用双天线 GNSS 航向（若 `isyawvalid == true`）
   - 否则用 GNSS 位置差分计算航向：`atan2(dy, dx)`
5. **初始状态构造**：位置 = GNSS位置 - 杆臂；速度 = 0（或给定初始速度）

**第二阶段：GINS 滑窗初始化** (`gvinsInitializationOptimization()`, `ic_gvins.cc:694-722`)：
- 用 GNSS+IMU 预积分做 50 次 LM 迭代优化
- 线性求解器：SPARSE_NORMAL_CHOLESKY
- 收敛判断后进入视觉初始化

**第三阶段：视觉系统初始化**：利用 IMU 提供先验位姿辅助特征跟踪和三角化。

### 4.2 状态传播（预测）

通过 **INS 机械编排** (`MISC::insMechanization()`) 进行状态传播。每次 IMU 到来时，从前一个 INS 状态积分得到新状态（`ic_gvins.cc:284`）。

优化完成后，INS 窗口用新的状态重新积分：`MISC::redoInsMechanization()`（`ic_gvins.cc:277`）。

### 4.3 多传感器更新

更新通过 Ceres 非线性优化完成。优化完成后：

1. **更新外参** (`updateParametersFromOptimizer()`, `ic_gvins.cc:1299-1389`)：
   - 外参更新需通过合理性检查（平移 < 1m, 旋转 < 5deg）
   
2. **更新关键帧位姿**：从优化后的 `statedatalist_` 恢复状态，转换为相机位姿

3. **更新地图点位置**：从优化后的逆深度恢复 3D 位置

---

## 5. 优缺点分析

### 5.1 算法优势

1. **INS-centric 设计**：以 INS 为核心，不依赖视觉特征的质量，对光照变化、弱纹理等环境鲁棒
2. **GNSS 辅助初始化**：利用 GNSS+IMU 做重力对齐和航向初始化，避免了纯 VIO 的对极几何初始化失败问题
3. **地球自转补偿**：对高精度 IMU 有实际收益
4. **多线程架构**：融合、跟踪、优化三线程分离，实时性好
5. **INS 辅助视觉**：利用 IMU 提供先验位姿辅助特征跟踪和三角化，在动态场景下更稳定
6. **两阶段粗差剔除**：Chi-squared 检验 + reweighting 相结合
7. **统一世界框架**：所有传感器在同一个世界坐标系中融合，避免了坐标系对齐误差

### 5.2 算法局限

1. **GNSS 测量层次浅**：只用 RTK 定位结果，不支持原始伪距/载波相位。在城市峡谷等 GNSS 多径环境中，RTK 固定解本身可能不可靠
2. **对 RTK 依赖强**：需要高精度 GNSS 定位（通常厘米级），SPP 精度下融合效果有限
3. **无 GNSS 中断鲁棒性**：GNSS 缺失时退化为纯 VIO，长期漂移
4. **滑动窗口而非全局优化**：历史信息通过边缘化近似保留，存在线性化误差累积
5. **无回环检测**：纯里程计模式，无全局一致性
6. **零偏硬编码**：IMU 零偏先验标准差硬编码，对不同 IMU 需要修改源码

### 5.3 工程特性

| 方面 | 评价 |
|------|------|
| 构建系统 | CMake + catkin (ROS) |
| 依赖 | Ceres 2.x, Eigen 3.3+, OpenCV 3.2+, ROS, yaml-cpp |
| C++ 标准 | C++14 |
| 代码组织 | 清晰的分层架构：common, factors, fileio, preintegration, tracking, ROS |
| 可扩展性 | 预积分通过多态可插拔；新增传感器因子只需继承 ceres::CostFunction |
| 文档 | README 完善，有中文注释 |
| 数据格式 | ROS bag (标准消息格式) |

### 5.4 适用场景

- **开阔天空**：最佳性能，GNSS RTK 稳定
- **城市峡谷**：部分可用，但当 RTK 固定解失败时性能下降。HuberLoss + Chi-squared 拒绝可部分缓解
- **室内**：GNSS 不可用，自动退化为 VIO
- **高动态**：INS-centric 设计使其在高动态场景下优于纯 VIO

---

## 6. 对 phad_fusion 的关键参考

### 6.1 GNSS 因子设计范式

IC-GVINS 的 GNSS 因子提供了一个清晰的**位置级 GNSS 因子模板**：

**继承模式**：
```cpp
class GnssFactor : public ceres::SizedCostFunction<3, 7> {
    // 残差维度：3 (位置)
    // 参数块：[pose(7)]  -- 单个7维状态参数块
};
```

对 GTSAM 实现的对等映射：
```cpp
// GTSAM 对应实现思路
class GnssFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    // 使用 Pose3 类型变量
    // 残差：p_IMU + R*lever - p_GNSS
};
```

**关键设计点**：
- 杆臂 (lever arm) 必须在因子中显式补偿
- 信息矩阵由 GNSS 标准差构造：`Ω = diag(1/σN², 1/σE², 1/σD²)`
- 残差使用局部 NED 坐标系而非 ECEF

### 6.2 坐标框架管理模式

IC-GVINS 的坐标框架管理是一个清晰的参考模式：

1. **原点固定于第一个 GNSS 测量**
2. **使用 WGS84 椭球参数** → ECEF → NED 的转换链
3. **所有内部状态在局部 NED 中运算**
4. **输出时通过 `local2global()` 转回 LLH**

对 phad_fusion 的关键启示：
- 必须建立 **全局经纬高 ↔ 局部切平面** 的双向映射
- 需要 `Earth::gravity()` 根据纬度计算当地重力
- 如果考虑地球自转，需要 `iewn()` 计算导航系下自转角速度

### 6.3 因子图节点/边管理

滑窗内的因子图管理模式：

1. **时间节点作为状态节点**：每个时间节点包含一个 pose(7) + mix(N) 双参数块
2. **预积分因子连接相邻时间节点**：二元因子，连接 (pose_k, mix_k) ↔ (pose_k+1, mix_k+1)
3. **GNSS 因子连接单时间节点**：一元因子
4. **视觉因子连接两帧+外参+深度**：五元因子
5. **边缘化因子连接保留的时间节点**：先验信息

对 phad_fusion 的借鉴：
- 如果使用 GTSAM，每个时间节点可以是一个 `Pose3` + `imuBias::ConstantBias` 的 `NavState`
- 预积分因子使用 `gtsam::CombinedImuFactor`
- GNSS 因子可以设计为 `NoiseModelFactor1<Pose3>`

### 6.4 GNSS 集成中的陷阱规避

从 IC-GVINS 的源码中可以总结以下注意事项：

1. **杆臂补偿必须正确**：`p_IMU = p_GNSS - R(q) * lever`，顺序不能搞反。IC-GVINS 在初始化时就先减去杆臂（`ic_gvins.cc:659`）

2. **GNSS 时间戳必须与 IMU 时间戳对齐**：IC-GVINS 使用 INS 速度做时间补偿来对齐，对齐容差为 0.025s

3. **GNSS 标准差必须合理**：IC-GVINS 从 NavSatFix 的 position_covariance 中提取对角线元素开根号，且做了零值检查

4. **GTSAM 与 Ceres 的关键差异**：
   - Ceres 的 `SizedCostFunction<N,M>` 直接指定残差/参数维度
   - GTSAM 使用类型系统 + NoiseModel 自动管理
   - GTSAM 的 factor graph 有显式的拓扑结构，Ceres 的 Problem 是隐式的
   - GTSAM 天然支持 `ISAM2` 增量优化，Ceres 需要手动管理滑动窗口

5. **初始化顺序**：先用 GNSS+IMU 做粗对齐，再引入视觉。这比直接从视觉初始化更鲁棒

6. **IMU 噪声参数的单位转换**：注意 deg/hr → rad/s, mGal → m/s², PPM → 无量纲的转换因子

---

## 附录：关键文件索引

| 文件 | 路径 | 用途 |
|------|------|------|
| 主系统框架 | `ic_gvins/ic_gvins/ic_gvins.{h,cc}` | 状态机、优化、边缘化 |
| GNSS 因子 | `ic_gvins/ic_gvins/factors/gnss_factor.h` | GNSS 位置观测因子 |
| 视觉因子 | `ic_gvins/ic_gvins/factors/reprojection_factor.h` | 重投影误差因子 |
| 预积分基类 | `ic_gvins/ic_gvins/preintegration/preintegration_base.h` | 预积分抽象接口 |
| 预积分工厂 | `ic_gvins/ic_gvins/preintegration/preintegration.h` | 工厂方法 |
| 标准预积分 | `ic_gvins/ic_gvins/preintegration/preintegration_normal.{h,cc}` | 标准 IMU 预积分 |
| 地球自转预积分 | `ic_gvins/ic_gvins/preintegration/preintegration_earth.{h,cc}` | 含地球自转补偿 |
| 预积分因子 | `ic_gvins/ic_gvins/preintegration/preintegration_factor.h` | 预积分 CostFunction |
| IMU 误差因子 | `ic_gvins/ic_gvins/preintegration/imu_error_factor.h` | IMU 零偏约束 |
| 位姿参数化 | `ic_gvins/ic_gvins/factors/pose_parameterization.h` | 流形上的位姿参数化 |
| 边缘化因子 | `ic_gvins/ic_gvins/factors/marginalization_factor.h` | 边缘化先验 |
| 状态定义 | `ic_gvins/ic_gvins/preintegration/integration_state.h` | 所有状态和参数结构体 |
| 坐标转换 | `ic_gvins/ic_gvins/common/earth.h` | WGS84 椭球和坐标转换 |
| 旋转工具 | `ic_gvins/ic_gvins/common/rotation.h` | 旋转矩阵/四元数/Euler 转换 |
| 数据类型 | `ic_gvins/ic_gvins/common/types.h` | GNSS/IMU/Pose/PVA 结构体 |
| 辅助函数 | `ic_gvins/ic_gvins/misc.h` | INS 机械编排、插值、时间对齐 |
| ROS 接口 | `ic_gvins/ROS/fusion_ros.cc` | ROS Topic 订阅和数据转发 |
| 配置文件 | `config/gvins.yaml` | IMU 噪声、窗口大小、外参等 |

---

## 7. 数据管线

### 7.1 传感器输入总览

| 传感器 | 硬件规格 | ROS Topic | 消息类型 | 频率 | 核心参数 |
|--------|---------|-----------|---------|------|---------|
| **IMU** | 6轴 MEMS (FRD) | `/imu0` | `sensor_msgs::Imu` | 200 Hz | ARW, VRW, gbstd, abstd (config) |
| **相机** | 单目全局快门 | `/cam0` | `sensor_msgs::Image` | 20 Hz | 内参 (fx,fy,cx,cy), 畸变 (k1,k2,p1,p2) |
| **GNSS-RTK** | 多频 RTK 接收机 | `/gnss0` | `sensor_msgs::NavSatFix` | 1 Hz | 定位标准差 (从 position_covariance 提取) |

### 7.2 IMU 数据管线

```
传感器硬件 → ROS Imu 消息
  ↓ (fusion_ros.cc:123-161, imuCallback)
angular_velocity (rad/s) × dt → imu_.dtheta[3]   (角增量, FRD)
linear_acceleration (m/s²) × dt → imu_.dvel[3]    (速度增量, FRD)
  ↓ UNIX时间 → GpsTime::unix2gps() → GPS 周秒
  ↓ 丢失检测: dt > 1.5×imudatadt → 插补虚拟 IMU (ic_gvins.cc:171-183)
  ↓ 入队 imu_buffer_ (条件变量通知融合线程)
[融合线程] runFusion() (ic_gvins.cc:237-393)
  ↓ MISC::insMechanization() (ic_gvins.cc:284): 单步 INS 机械编排
    p_k+1 = p_k + v_k·dt + 0.5·(R_k·fb + g)·dt²
    v_k+1 = v_k + (R_k·fb + g)·dt
    q_k+1 = q_k ⊗ exp((w - bg)·dt - iewn·dt)
  其中 fb: 比力, w: 角速度, iewn: 地球自转在导航系投影
  ↓ preintegrationlist_: IMU 数据在时间节点间做预积分
    PreintegrationNormal (标准) 或 PreintegrationEarth (地球自转补偿)
    工厂方法: Preintegration::createPreintegration() (preintegration.h:57)
  ↓ 预积分残差 (preintegration_normal.cc:61-67):
    e_p = R₀ᵀ(p₁ - p₀ - v₀Δt - ½gΔt²) - Δp_corrected
    e_v = R₀ᵀ(v₁ - v₀ - gΔt) - Δv_corrected
    e_q = 2·(Δq_corrected⁻¹ ⊗ R₀ᵀ ⊗ q₁).vec()  [15维: p3+v3+q3+bg3+ba3]
  ↓ PreintegrationFactor (Ceres CostFunction, 4参数块):
    [pose₀(7), mix₀(N), pose₁(7), mix₁(N)]
  ↓ 零偏一阶校正: Δx_corrected = Δx + J_bg·δbg + J_ba·δba
  ↓ 信息矩阵: sqrt_information = Cholesky(cov⁻¹) (preintegration_normal.cc:40)
  ↓ redoInsMechanization() (ic_gvins.cc:277): 优化后用新状态重积分
```

**IMU 噪声参数转换** (config → 内部使用):
```
ARW (deg/√hr)  → rad/√s:   × D2R / 60
VRW (m/s/√hr)  → m/√s:     / 60
gbstd (deg/hr) → rad/s:    × D2R / 3600
abstd (mGal)   → m/s²:     × 1e-5
```

### 7.3 GNSS 数据管线

```
GNSS 接收机 (RTK 解算) → ROS NavSatFix 消息
  ↓ (fusion_ros.cc:163-199, gnssCallback)
latitude (deg)  → gnss_.blh[0] = lat × D2R   (纬度, rad)
longitude (deg) → gnss_.blh[1] = lon × D2R   (经度, rad)
altitude (m)    → gnss_.blh[2] = alt          (高程, m)
  ↓ 标准差提取 (position_covariance 对角线):
    σ_N = √(cov[4]), σ_E = √(cov[0]), σ_D = √(cov[8])
  ↓ UNIX时间 → GpsTime::unix2gps() → GPS 周秒
  ↓ 零值检查: σ=0 → 丢弃; σ > gnssthreshold → 丢弃
  ↓ GNSS 中断模拟 (isusegnssoutage_): time > outage_time → 丢弃
  ↓ 第一次有效: 原点设置 (ic_gvins.cc:207)
    integration_config_.origin = gnss.blh    (WGS84 经纬高)
    gravity = Earth::gravity(gnss.blh)      (当地重力, WGS84 公式)
[坐标转换] addNewGnss() (ic_gvins.cc:199-219):
  ↓ Earth::global2local(origin, gnss.blh) (earth.h:185-192):
    ECEF_gnss = blh2ecef(gnss.blh)          WGS84 椭球 → ECEF
    ECEF_origin = blh2ecef(origin)
    dECEF = ECEF_gnss - ECEF_origin
    gnss.blh = cne(origin)ᵀ · dECEF         ECEF → 局部 NED
  ↓ 其中 cne(origin) = ∂(NED)/∂(ECEF) 旋转矩阵 (earth.h:71-93):
    列0: [-sin(lat)·cos(lon), -sin(lat)·sin(lon), cos(lat)]ᵀ  (北)
    列1: [-sin(lon), cos(lon), 0]ᵀ                          (东)
    列2: [-cos(lat)·cos(lon), -cos(lat)·sin(lon), -sin(lat)]ᵀ (地)
[时间对齐] insertNewGnssTimeNode() (ic_gvins.cc:791-888):
  ↓ 最近前节点: |t_gnss - t_prev| < 25ms → 对齐到前节点 (INS速度补偿)
  ↓ 最近后节点: |t_next - t_gnss| < 25ms → 对齐到后节点 (INS速度补偿)
  ↓ 中间插入: 拆分长预积分段 → 插入 GNSS 节点 → 重积分后续
  ↓ 对齐容差: MINMUM_SYNC_INTERVAL = 0.025s
  ↓ 速度补偿公式:
    gnss.blh[i] = gnss.blh[i] + state.v[i] · dt
    gnss.std *= 1.2  (时间插值增加不确定性)
[因子构建] addGnssFactors() (ic_gvins.cc:1891+):
  ↓ GnssFactor (factors/gnss_factor.h:31-76)
    残差: e = p_IMU + R(q)·l_ant - p_GNSS  (NED, 3维)
    信息矩阵: W = diag(1/σ_N, 1/σ_E, 1/σ_D)
    杆臂: l_ant = antlever_ (config: antlever [x,y,z], FRD)
  ↓ 雅可比: ∂e/∂p = I₃, ∂e/∂θ = -R(q)·[l_ant]×
  ↓ 参数块: [pose(7)] × 1
  ↓ 优化策略: 第一轮 HuberLoss(1.0) → Chi²粗差 (阈值 7.815) → 第二轮无核函数
```

**注: IC-GVINS 不处理 GNSS 原始观测量 (伪距/载波/Doppler/星历)。只使用 RTK 结算定位结果。对 GNSS 原始测量为松耦合。**

### 7.4 视觉数据管线

```
相机 → ROS Image 消息 (MONO8/BGR8/RGB8)
  ↓ (fusion_ros.cc:201-233, imageCallback)
图像数据拷贝 → cv::Mat; UNIX时间 → GPS 周秒
  ↓ Frame::createFrame(stamp, image) (frame 构造)
Frame 入队 frame_buffer_ → 通知跟踪线程
[跟踪线程] runTracking() (ic_gvins.cc:479-552):
  ↓ 时间延时补偿: frame->setStamp(frame->stamp() + td_b_c) (ic_gvins.cc:529)
  ↓ INS 先验位姿: MISC::getCameraPoseFromInsWindow() (ic_gvins.cc:531)
    INS 窗口内插值获取相机时刻 INS 位姿 → 通过外参 pose_b_c_ 转相机位姿
  ↓ tracking_->track(frame) (特征提取/匹配/跟踪):
    特征点 → 关键点 (KeyPoint, 归一化相机坐标) → 速度估计 (vel0/vel1)
  ↓ 关键帧判定: 第一帧/跟踪失败/new keyframe → 入队 keyframes_
  ↓ 通知融合线程: isframeready_ = true
[融合线程] addNewKeyFrameTimeNode() (ic_gvins.cc:724-752):
  ↓ 提取关键帧 → map_->insertKeyFrame(frame) → 地图点 (MapPoint)
  ↓ addNewTimeNode(frametime) → 时间节点 + 预积分
  ↓ 逆深度参数化: invdepthlist_[mappoint.id] = 1/z
[重投影因子] ReprojectionFactor (factors/reprojection_factor.h:36-158):
  ↓ 5参数块: [pose_ref(7), pose_obs(7), extrinsic(7), invdepth(1), td(1)]
  ↓ 残差链路 (归一化相机坐标):
    pts_c₀ = pts0_td / invdepth              (逆深度 → 3D 相机坐标)
    pts_b₀ = R_bc·pts_c₀ + t_bc              (相机 → IMU)
    pts_n  = R₀·pts_b₀ + p₀                  (IMU_0 → NED)
    pts_b₁ = R₁ᵀ·(pts_n - p₁)                 (NED → IMU_1)
    pts_c₁ = R_cb·(pts_b₁ - t_bc)             (IMU_1 → 相机)
    e = (pts_c₁ / pts_c₁.z).head(2) - pts1_td.head(2)  (2维残差)
  ↓ 时间补偿: pts_td = pts - (td - td₀)·vel  (特征点速度修正)
  ↓ 粗差剔除: Chi²(2自由度, 阈值 5.991) (ic_gvins.cc:1269-1297)
```

### 7.5 跨传感器协同

| 机制 | 实现 | 配置 |
|------|------|------|
| **时间基准** | GPS 周秒 (GpsTime::unix2gps) | `fusion_ros.cc:130` |
| **相机-IMU 同步** | 在线 td 估计 (td_b_c_) | `config: td_b_c, optimize_estimate_td_` |
| **GNSS-IMU 同步** | 动态时间对齐 + INS 速度补偿 | `MINMUM_SYNC_INTERVAL=0.025s` |
| **IMU 丢失** | 虚拟 IMU 插补 (dt > 1.5×周期) | `imu_buffer_.push(interpolated)` |
| **初始化** | GNSS/INS 粗对齐 → GINS 滑窗 → 视觉 | `gvinsstate_ 状态机` |
| **三线程架构** | fusion(IMU积分+状态传播) / tracking(视觉) / optimization(Ceres) | 信号量同步 |
| **降级策略** | GNSS 缺失 → 纯 VIO (IMU+视觉); GNSS 粗差 → reweight/reject | HuberLoss + Chi² |
| **边缘化** | 舒尔补 (MarginalizationInfo) | `optimize_windows_size_` (默认 10 关键帧) |

### 7.6 因子图拓扑

```
优化因子图 (Ceres Problem):
  节点: TimeNode_k = [pose(7), mix(N)]  × K个
        Extrinsic[7], Td[1], InvDepth × M个

  边:
  ├── PreintegrationFactor(k,k+1) ── [pose_k, mix_k, pose_k+1, mix_k+1]
  ├── GnssFactor(t) ──────────────── [pose_k]                    (仅GNSS有观测时)
  ├── ReprojectionFactor ─────────── [pose_ref, pose_obs, extrinsic, invdepth, td]
  ├── ImuErrorFactor ─────────────── [mix_K]                     (零偏软约束)
  └── MarginalizationFactor ──────── [保留的参数块]               (边缘化先验)

优化: Ceres LM + DENSE_SCHUR, 两轮 (HuberLoss → outlier culling → 无核函数)
```

---

*分析完成于 2026-04-28，基于 IC-GVINS 开源代码 commit 快照。*