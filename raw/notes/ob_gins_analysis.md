# OB_GINS 源码深度分析

> 全称：Optimization-Based GNSS/INS Integrated Navigation System
> 团队：武汉大学 i2Nav 课题组
> 主页：https://github.com/i2Nav-WHU/OB_GINS

---

## 1. 数据接收与预处理

### 1.1 数据入口：文本文件读取

OB_GINS 是一个**离线批处理程序**，不从 ROS 取数据，而是从文本文件读取 IMU 和 GNSS 数据。入口为 `src/ob_gins.cc:52-621`（单文件 main 函数实现）。

命令行调用：
```bash
./bin/ob_gins ./dataset/ob_gins.yaml
```

配置文件 `config/ob_gins.yaml` 指定数据文件路径：
```yaml
imufile: ""
gnssfile: ""
outputpath: ""
```

### 1.2 GNSS 原始测量类型

OB_GINS **不处理原始伪距/载波相位观测值**。它使用的 GNSS 测量是 RTK/PPP 解算后的绝对位置。

GNSS 数据格式为文本文件，每行 7 列或 13 列：
```
# 7 列格式: time lat lon height std_N std_E std_D
# 13列格式: time lat lon height ... vel_N vel_E vel_D std_N std_E std_D
```

文件读取器 `GnssFileLoader`（`fileio/gnssfileloader.h:30-59`）：
```cpp
const GNSS &next() {
    data_ = load();
    gnss_.time = data_[0];
    memcpy(gnss_.blh.data(), &data_[1], 3 * sizeof(double));
    // 7列: std 在列 4-6; 13列: std 在列 7-9
    if (data_.size() == 7) {
        memcpy(gnss_.std.data(), &data_[4], 3 * sizeof(double));
    } else {
        memcpy(gnss_.std.data(), &data_[7], 3 * sizeof(double));
    }
    gnss_.blh[0] *= D2R;  // 纬度 deg→rad
    gnss_.blh[1] *= D2R;  // 经度 deg→rad
    return gnss_;
}
```

GNSS 数据结构定义在 `common/types.h:32-37`：
```cpp
typedef struct GNSS {
    double time;
    Vector3d blh;    // 纬度(rad), 经度(rad), 高程(m)
    Vector3d std;    // NE方向位置标准差 (m)
} GNSS;
```

**结论：OB_GINS 使用 GNSS 位置级测量，不使用原始伪距/载波/多普勒。与 IC-GVINS 的 GNSS 部分设计几乎一致。**

### 1.3 IMU 数据格式

IMU 数据以增量形式从文本文件读取，每行 7/8/9 列：
```cpp
// imufileloader.h:41-65
imu_.time = data_[0];
memcpy(imu_.dtheta.data(), &data_[1], 3 * sizeof(double));  // 角增量 (rad)
memcpy(imu_.dvel.data(), &data_[4], 3 * sizeof(double));    // 速度增量 (m/s)
// 第8列：单轮速；第9列：双轮速平均值
if (columns_ == 8) {
    imu_.odovel = data_[7] * imu_.dt;
} else if (columns_ == 9) {
    imu_.odovel = 0.5 * (data_[7] + data_[8]) * imu_.dt;
}
```

IMU 采样频率通过配置文件指定 `imudatarate`（默认 200 Hz），dt 由相邻采样时间差确定，若间隔过大则回退到 `1/rate`（`imufileloader.h:51-55`）。

### 1.4 坐标框架约定

OB_GINS 使用与 IC-GVINS **完全相同的坐标框架**（来自同一团队）：

**WGS84 椭球参数**（`common/earth.h:34-40`）：
```cpp
const double WGS84_WIE = 7.2921151467E-5;   // 地球自转角速度
const double WGS84_F   = 0.0033528106647474805; // 扁率
const double WGS84_RA  = 6378137.0;          // 长半轴a
const double WGS84_RB  = 6356752.3142451793; // 短半轴b
const double WGS84_E1  = 0.0066943799901413156; // 第一偏心率平方
```

**坐标转换链**：
1. **全局 BLH → ECEF**：`Earth::blh2ecef()` (`common/earth.h:117-130`)
2. **ECEF → 全局 BLH**：`Earth::ecef2blh()` (`common/earth.h:132-150`)
3. **全局 BLH → 局部 NED**：`Earth::global2local()` (`common/earth.h:185-192`)

核心公式（与 IC-GVINS 一致）：
```cpp
static Vector3d global2local(const Vector3d &origin, const Vector3d &global) {
    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);            // NED→ECEF
    Vector3d ecef1 = blh2ecef(global);
    return cn0e.transpose() * (ecef1 - ecef0);
}
```

**关键约定**：
- **站心坐标系原点**：第一个 GNSS 定位位置，存储在 `parameters->station`（`ob_gins.cc:173-178`）
- **局部坐标系**：NED（北-东-地）
- **IMU 坐标系**：前-右-下 (FRD)

GNSS 到达时直接转换（`ob_gins.cc:175`）：
```cpp
gnss.blh = Earth::global2local(station_origin, gnss.blh);
```

### 1.5 时间同步

OB_GINS 采用**固定整秒间隔的节点采样策略**：

1. **时间戳单位**：GPS 周秒 (seconds of week, SOW)
2. **积分节点间距**：固定 `INTEGRATION_LENGTH = 1.0` 秒（`ob_gins.cc:43`）
3. **IMU-GNSS 时间对齐**：
   - GNSS 数据必须恰好对齐在整秒节点上（容差 0.001s）
   - IMU 数据到整秒时刻若不在采样点上，通过**线性插值**拆分 IMU 增量（`ob_gins.cc:577-594`）
4. **插值判断**（`ob_gins.cc:596-621`）：
   - 前一个 IMU 离整秒 < 0.0001s：直接丢弃（接近）
   - 后一个 IMU 离整秒 < 0.0001s：直接使用
   - 否则：内插拆分 IMU 增量

IMU 插值公式（按时间比例拆分增量）：
```cpp
scale = (imu01.time - time) / imu01.dt;
imu00.dtheta = imu01.dtheta * (1 - scale);
imu11.dtheta = imu01.dtheta * scale;
// dvel, odovel 同理
```

---

## 2. 传感器融合架构

### 2.1 紧耦合 vs 松耦合

OB_GINS 是 **GNSS-IMU 紧耦合**（在优化层面），但使用 **GNSS 位置级观测**（非原始伪距）。准确说是：

- **优化层面**：紧耦合（GNSS 因子和 IMU 预积分因子在同一 Ceres Problem 中联合优化）
- **GNSS 测量层面**：松耦合（只用 RTK 定位结果，不用原始观测值）

与 IC-GVINS 的区别：OB_GINS **没有视觉**，是纯 GNSS+IMU 系统。因此不存在"VIO"部分。

### 2.2 GNSS 观测如何融入优化

GNSS 作为绝对位置观测因子，连接到对应整秒时间节点的 pose 参数块：

1. 滑窗中每个整秒节点有一个 7 维 pose 参数块
2. GNSS 观测通过杆臂补偿后与 IMU 位置建立残差
3. 信息矩阵由 GNSS 定位标准差 diag 构造

### 2.3 状态向量

OB_GINS 估计的状态变量（`preintegration/integration_state.h:35-51`，与 IC-GVINS 完全相同）：

```cpp
typedef struct IntegrationState {
    double time;
    Vector3d p;      // IMU位置 (NED局部坐标, m)
    Quaterniond q;   // IMU姿态 (IMU→NED)
    Vector3d v;      // IMU速度 (NED, m/s)
    Vector3d bg;     // 陀螺零偏 (rad/s)
    Vector3d ba;     // 加速度计零偏 (m/s^2)
    Vector3d s;      // IMU比例因子 (PPM)
    double sodo;     // 里程计比例因子
    Vector2d abv;    // 安装角
    Vector3d sg;     // 陀螺比例因子
    Vector3d sa;     // 加速度计比例因子
} IntegrationState;
```

**滑窗内的优化变量**（`ob_gins.cc:180-183`）：
- `statedatalist[windows+1]`：`IntegrationStateData` 数组，每个元素包含 pose[7] + mix[9/10/12]
- 窗口大小由配置 `windows` 指定（默认 30 秒）

Mix 参数块内容（`integration_state.h:53-66`）：
- 无里程计：vel[3] + bg[3] + ba[3] = 9 维
- 有里程计：vel[3] + bg[3] + ba[3] + sodo[1] = 10 维

### 2.4 IMU 预积分方法

与 IC-GVINS 共享**完全相同的预积分体系**：

1. **PreintegrationBase** (`preintegration_base.h`): 抽象基类
2. **PreintegrationNormal** (`preintegration_normal.h`): 标准预积分
3. **PreintegrationEarth** (`preintegration_earth.h`): 地球自转补偿预积分
4. **PreintegrationOdo** / **PreintegrationEarthOdo**: 里程计变体

工厂方法：
```cpp
PreintegrationOptions options = Preintegration::getOptions(isuseodo, isearth);
auto preintegration = Preintegration::createPreintegration(parameters, imu_pre, state, options);
```

预积分残差计算与 IC-GVINS 一致（15 维残差）：

```
e_p  = R_0^T * (p_1 - p_0 - v_0*Δt - 0.5*g*Δt^2) - p_corrected
e_v  = R_0^T * (v_1 - v_0 - g*Δt) - v_corrected
e_q  = 2 * (q_corrected^{-1} * R_0^T * q_1).vec()
e_bg = bg_1 - bg_0
e_ba = ba_1 - ba_0
```

关键区别：OB_GINS 的 `PreintegrationBase` 比 IC-GVINS 多了一个 `IMU_ACC_Z_SCALE = 100` 常量（`preintegration_base.h:122`），用于加速度计 Z 轴比例因子的额外约束。

**注意：该框架不使用 GTSAM！** OB_GINS 使用 **Ceres Solver**，所有因子均为自定义 Ceres CostFunction。

---

## 3. Ceres 因子设计（核心）

> **注意**：OB_GINS 使用 Ceres Solver 而非 GTSAM。IC-GVINS 的 GNSS 因子和预积分因子直接继承自 OB_GINS（二者源码几乎完全相同）。以下分析基于 OB_GINS 最新版。

### 3.1 GNSS 位置因子（与 IC-GVINS 相同）

**文件**：`factors/gnss_factor.h:32-78`

**继承关系**：`GnssFactor : public ceres::SizedCostFunction<3, 7>`
- 残差维度：3（位置 NED）
- 参数块：1 个 7 维 pose 参数块

**完整类定义**：
```cpp
class GnssFactor : public ceres::SizedCostFunction<3, 7> {
public:
    explicit GnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    void updateGnssState(const GNSS &gnss) {
        gnss_ = gnss;
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3],
                      parameters[0][4], parameters[0][5]};

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);

        // ★ 核心残差公式 ★
        error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

        // 信息矩阵：W = diag(1/σN, 1/σE, 1/σD)
        Matrix3d weight = Matrix3d::Zero();
        weight(0, 0)    = 1.0 / gnss_.std[0];
        weight(1, 1)    = 1.0 / gnss_.std[1];
        weight(2, 2)    = 1.0 / gnss_.std[2];

        error = weight * error;  // 加权残差

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

                jacobian_pose = weight * jacobian_pose;
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
  p_IMU ∈ R^3    : IMU 在局部 NED 中的位置
  R(q)  ∈ SO(3)  : IMU 到 NED 的旋转矩阵
  l_ant ∈ R^3    : 天线杆臂 (FRD, m)
  p_GNSS ∈ R^3   : GNSS 测量的局部 NED 位置
  W = diag(1/σ_N, 1/σ_E, 1/σ_D)
```

**雅可比结构**：
- 位置部分 (3×3)：`∂e/∂p = I`（单位阵）
- 姿态部分 (3×3)：`∂e/∂θ = -R(q) * [l_ant]_×`（杆臂的反对称矩阵乘旋转矩阵）
- 整体雅可比 (3×7)：`[I₃ₓ₃ | -R·[l]× | 0₃ₓ₁]`，最后 1 列为 0（四元数 w 分量的冗余自由度）

### 3.2 IMU 预积分因子

**文件**：`preintegration/preintegration_factor.h:30-74`

**继承关系**：`PreintegrationFactor : public ceres::CostFunction`

与 IC-GVINS 预积分因子**完全相同**。这是一个包装器，将 `PreintegrationBase` 的子类方法映射到 Ceres 接口：

```cpp
class PreintegrationFactor : public ceres::CostFunction {
public:
    PreintegrationFactor() = delete;
    explicit PreintegrationFactor(std::shared_ptr<PreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {
        *mutable_parameter_block_sizes() = preintegration_->numBlocksParameters();
        set_num_residuals(preintegration_->numResiduals());
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        IntegrationState state0, state1;
        preintegration_->constructState(parameters, state0, state1);
        preintegration_->evaluate(state0, state1, residuals);
        if (jacobians) {
            if (jacobians[0])
                preintegration_->residualJacobianPose0(state0, state1, jacobians[0]);
            if (jacobians[1])
                preintegration_->residualJacobianMix0(state0, state1, jacobians[1]);
            if (jacobians[2])
                preintegration_->residualJacobianPose1(state0, state1, jacobians[2]);
            if (jacobians[3])
                preintegration_->residualJacobianMix1(state0, state1, jacobians[3]);
        }
        return true;
    }
private:
    std::shared_ptr<PreintegrationBase> preintegration_;
};
```

参数块结构（`preintegration_normal.cc:148-149`）：
```
{pose_0[7], mix_0[9], pose_1[7], mix_1[9]}  // 4个参数块
```

残差维度：15（p[3] + v[3] + q[3] + bg[3] + ba[3]）

### 3.3 IMU 误差先验因子

**文件**：`preintegration/imu_error_factor.h:30-58`

**继承关系**：`ImuErrorFactor : public ceres::CostFunction`
- 残差维度：6（bg[3] + ba[3]）
- 参数块：1 个 Mix 参数块 [9]

与 IC-GVINS 的区别：OB_GINS 将 `imuErrorEvaluate/imuErrorJacobian` 实现移入了 `PreintegrationBase` 子类中（而非在单独的因子类里），但语义完全一致：

```cpp
// preintegration_normal.cc:264-272
void PreintegrationNormal::imuErrorEvaluate(const double *const *parameters,
                                            double *residuals) {
    residuals[0] = parameters[0][3] / IMU_GRY_BIAS_STD;
    residuals[1] = parameters[0][4] / IMU_GRY_BIAS_STD;
    residuals[2] = parameters[0][5] / IMU_GRY_BIAS_STD;
    residuals[3] = parameters[0][6] / IMU_ACC_BIAS_STD;
    residuals[4] = parameters[0][7] / IMU_ACC_BIAS_STD;
    residuals[5] = parameters[0][8] / IMU_ACC_BIAS_STD;
}
```

硬编码约束值（`preintegration_base.h:119-123`）：
```cpp
static constexpr double IMU_GRY_BIAS_STD = 7200/3600.0 * M_PI/180.0; // 2.0 rad/s
static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;            // 0.2 m/s^2
static constexpr double IMU_ACC_Z_SCALE  = 100;
```

### 3.4 边缘化因子

**文件**：`factors/marginalization_factor.h:31-105`

与 IC-GVINS 完全相同。通过舒尔补将边缘化的状态量转换为保留参数的先验约束。

### 3.5 位姿流形 (Manifold)

**文件**：`factors/pose_manifold.h:28-39`

**继承关系**：`PoseManifold : public ceres::Manifold`

这是 OB_GINS 与 IC-GVINS 的一个**关键 API 差异**。Ceres 2.1 废除了 `LocalParameterization`，改用 `Manifold` 接口。OB_GINS 已适配新 API：

```cpp
class PoseManifold : public ceres::Manifold {
public:
    int AmbientSize() const override;   // = 7
    int TangentSize() const override;   // = 6
    bool Plus(const double *x, const double *delta,
              double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;
    bool Minus(const double *y, const double *x,
               double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;
};
```

Plus 实现（`pose_manifold.cc`）：
- 位置：`p' = p + dp`
- 姿态：`q' = (q * exp(δθ)).normalized()`

而 IC-GVINS 仍使用旧 API `LocalParameterization`。如果 phad_fusion 使用 Ceres 2.1+，应参考 OB_GINS 的 Manifold 实现。

### 3.6 因子图结构

```
时间节点 (整秒, INTEGRATION_LENGTH=1s):
  TimeNode_k = [pose(7), mix(9)]    (k = 0, 1, ..., K)

因子 (Ceres ResidualBlock):
  ┌── PreintegrationFactor(0) ── [pose_0, mix_0, pose_1, mix_1]
  │   PreintegrationFactor(1) ── [pose_1, mix_1, pose_2, mix_2]
  │   ...
  │   PreintegrationFactor(K-1)─ [pose_K-1, mix_K-1, pose_K, mix_K]
  │
  ├── GnssFactor ─────────────── [pose_k]   (当 GNSS 对齐)
  │
  ├── ImuErrorFactor ─────────── [mix_K]    (最新节点零偏约束)
  │
  └── MarginalizationFactor ──── [pose_1...pose_K, mix_1...mix_K]
```

### 3.7 优化策略

**两层迭代优化**（`ob_gins.cc:295-417`）：

1. **第一轮**（1/4 总迭代次数）：
   - GNSS 因子使用 `HuberLoss(1.0)` 核函数
   - 求解器：SPARSE_NORMAL_CHOLESKY
   - 信任域：LEVENBERG_MARQUARDT
   - 可选 GNSS 粗差检测：Chi-squared (3 自由度, 阈值 7.815)

2. **粗差剔除**：
   - 若 `is_outlier_culling=true`：
     - 计算每个 GNSS 残差的 chi2
     - chi2 > 7.815 → reweight GNSS std
     - 移除所有带核函数的 GNSS 残差块
     - 重新添加 GNSS 因子（不带核函数）

3. **第二轮**（3/4 总迭代次数）：
   - 精调

**注意：OB_GINS 不使用 ISAM2。采用 Ceres 滑动窗口 + 边缘化策略。**

---

## 4. 位姿计算

### 4.1 初始化流程

OB_GINS 的初始化比分阶段，在 main 函数中直接完成（`ob_gins.cc:188-214`）：

1. **初始姿态由配置文件指定**（`ob_gins.cc:92-101`）：
   ```yaml
   initvel: [0, 0, 0]      # 初始NED速度 (m/s)
   initatt: [0, 0, 0]      # 初始 RPY 姿态 (deg)
   initgb: [0, 0, 0]       # 初始陀螺零偏 (deg/hr)
   initab: [0, 0, 0]       # 初始加速度计零偏 (mGal)
   ```

2. **初始位置**：取第一个 GNSS 定位值，减去杆臂：
   ```cpp
   IntegrationState state_curr = {
       .time = round(gnss.time),
       .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever,
       .q    = Rotation::euler2quaternion(initatt),
       .v    = initvel,
       .bg   = initbg,
       .ba   = initba,
   };
   ```

**与 IC-GVINS 的初始化差异**：OB_GINS **不做零速检测和重力调平**，姿态/速度/零偏完全依赖配置文件初始值。这对于纯 GNSS+IMU 系统是合理的，因为 GNSS 位置观测可直接约束位置和速度（通过多历元差分）。

### 4.2 状态传播（预测）

通过 IMU 预积分进行状态传播。每个新的整秒到来时：
```cpp
state_curr = preintegrationlist.back()->currentState();
```

非整秒时刻直接用预积分器的 `currentState()` 输出（`ob_gins.cc:518-519`）。

### 4.3 更新与输出

优化完成后直接输出优化后的状态。输出包括（`ob_gins.cc:534-575`）：
- 位置（转回全局 LLH）：`Earth::local2global(origin, state.p)`
- 姿态（四元数 → Euler）：`Rotation::quaternion2euler(state.q) * R2D`
- 速度（NED）
- IMU 零偏

---

## 5. 优缺点分析

### 5.1 算法优势

1. **纯 GNSS+IMU 简洁架构**：没有视觉模块的复杂性，代码量小，易于理解和修改
2. **清晰的代码结构**：common / factors / fileio / preintegration 四层分离
3. **抽象预积分设计**：工厂模式 + 多态，可灵活切换预积分模式
4. **整秒节点同步简单高效**：固定 1 秒积分间隔，IMU 插值保证时间对齐
5. **与 IC-GVINS 共享核心模块**：GNSS 因子、预积分体系完全共享，降低维护成本
6. **支持里程计**：可通过配置文件启用轮速计辅助
7. **Ceres 2.1 Manifold API**：已适配最新 Ceres API
8. **完善的粗差检测**：Chi-squared + reweighting

### 5.2 算法局限

1. **无视觉**：纯 GNSS+IMU 在 GNSS 中断时只有惯性导航，漂移快
2. **GNSS 测量层次浅**：只用 RTK 位置，不引入原始伪距/载波
3. **初始化依赖先验**：姿态和零偏必须由用户给定，没有自动对准
4. **固定整秒采样**：无法灵活适应传感器频率变化
5. **无回环检测**：纯里程计
6. **离线批处理**：非实时在线系统
7. **硬编码约束**：IMU 零偏先验标准差硬编码

### 5.3 工程特性

| 方面 | 评价 |
|------|------|
| 构建系统 | CMake (无 ROS 依赖) |
| 依赖 | Ceres 2.x, Eigen 3.3+, yaml-cpp |
| C++ 标准 | C++17 |
| 代码规模 | 约 15 个源文件，轻量级 |
| 实时性 | 离线批处理 |
| 可扩展性 | 预积分通过多态可插拔；新增因子只需继承 ceres::CostFunction |
| 文档 | README 完善，中文注释 |
| 与 IC-GVINS 关系 | OB_GINS 是 IC-GVINS 的纯 GNSS+IMU 前身，核心模块完全共享 |

### 5.4 适用场景

- **开阔天空 GNSS+IMU 组合导航**：最佳适用
- **车辆里程计辅助**：可通过启用 odo 模式提升精度
- **GNSS 中断场景**：不适用，纯惯性迅速漂移
- **作为 GINS 子模块**：可嵌入到更大系统中作为 GNSS/IMU 子滤波器

---

## 6. 对 phad_fusion 的关键参考

### 6.1 GNSS 因子设计范式

OB_GINS（和 IC-GVINS）的 `GnssFactor` 为位置级 GNSS 因子提供了最简洁的参考实现：

**Ceres 实现模式（C++17）**：
```cpp
class GnssFactor : public ceres::SizedCostFunction<3, 7> {
    // 残差 = IMU位置 + 旋转*杆臂 - GNSS测量位置
    // 残差维度：3
    // 参数块：单个 7 维 pose
};
```

**转 GTSAM 的对等设计**：
```cpp
class GnssFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    Vector3 gnss_pos_;   // GNSS 测量的局部 NED 位置
    Vector3 ant_lever_;  // 天线杆臂

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override {
        // 误差 = pose.translation() + pose.rotation().matrix() * lever - gnss_pos_
    }
};
```

### 6.2 坐标框架管理模式

OB_GINS 的坐标管理比 IC-GVINS 更简洁（无视觉坐标变换）：

1. **原点**：`parameters->station` 存储站心 LLH
2. **所有内部运算在局部 NED**：位置 p 是 NED 偏移
3. **输出时转回 LLH**：`writeNavResult()` 中调用 `Earth::local2global()`
4. **重力**：`Earth::gravity(gnss.blh)` 根据纬度动态计算

对 phad_fusion 的建议架构：
```
数据层:    GNSS LLH → Earth::global2local() → local NED
优化层:    所有状态在 local NED 中
输出层:    local NED → Earth::local2global() → LLH
```

### 6.3 因子图节点/边管理模式

OB_GINS 的滑窗管理模式：

1. **滑窗大小**：`windows` 整秒节点（默认 30）
2. **新节点加入**：每到整秒 push_back 新节点
3. **边缘化触发**：窗口满时边缘化最老节点 + 其关联因子
4. **因子清理**：边缘化后 pop_front 最老节点及其 IMU/GNSS 因子

```
窗口未满：      [0] [1] [2] ... [K]     → 简单优化
窗口满：        [0] [1] ... [W]           → 边缘化 [0], 滑动
边缘化后：      [1] [2] ... [W] [W+1]
```

对 phad_fusion 的建议：
- 如果使用 GTSAM ISAM2，可利用其自带的增量优化能力
- ISAM2 的 `update()` 方法自动处理边缘化
- 若用 Ceres，参考 OB_GINS 的手动边缘化流程

### 6.4 GNSS 集成中的陷阱规避

结合 OB_GINS 和 IC-GVINS 的源码经验：

1. **杆臂补偿方向**：
   ```cpp
   p_IMU = p_GNSS - R(q) * lever  // GNSS→IMU
   // 或
   p_GNSS = p_IMU + R(q) * lever  // IMU→GNSS  (残差中使用)
   ```

2. **时间戳对齐策略选择**：
   - IC-GVINS：动态插入，INS 速度补偿，适合多传感器异步
   - OB_GINS：固定整秒，IMU 插值，适合离线处理

3. **GNSS 标准差处理**：
   - 必须检查零值，否则信息矩阵奇异
   - IC-GVINS 做 `if (std == 0) return;` 检查
   - OB_GINS 通过 `gnssthreshold` 过滤大标准差

4. **Ceres vs GTSAM 的关键 API 差异**：
   | 特性 | Ceres | GTSAM |
   |------|-------|-------|
   | 因子定义 | CostFunction + AddResidualBlock | NoiseModelFactor |
   | 类型系统 | 手动指定残差/参数维度 | 类型推导 |
   | 流形优化 | LocalParameterization/Manifold | Lie group 内置 |
   | 边缘化 | 手动实现 Schur complement | ISAM2 自动 |
   | 增量优化 | 无内置支持 | ISAM2 |
   | 信息矩阵 | 用户手动指定 sqrt_info | NoiseModel 自动管理 |

5. **IMU 噪声参数的单位转换**（配置文件 → 内部使用）：
   ```
   ARW: deg/sqrt(hr) → rad/sqrt(s) : × D2R / 60
   VRW: m/s/sqrt(hr) → m/sqrt(s)   : / 60
   gbstd: deg/hr → rad/s           : × D2R / 3600
   abstd: mGal → m/s^2             : × 1e-5
   ```

---

## 附录：关键文件索引

| 文件 | 路径 | 用途 |
|------|------|------|
| 主程序 | `src/ob_gins.cc` | 主流程、优化循环、边缘化 |
| GNSS 因子 | `src/factors/gnss_factor.h` | GNSS 位置观测因子 |
| 预积分基类 | `src/preintegration/preintegration_base.h` | 预积分抽象接口 |
| 预积分工厂 | `src/preintegration/preintegration.h` | 工厂方法 |
| 标准预积分 | `src/preintegration/preintegration_normal.{h,cc}` | 标准 IMU 预积分（含残差/雅可比详细实现） |
| 地球自转预积分 | `src/preintegration/preintegration_earth.{h,cc}` | 含地球自转补偿 |
| 预积分因子 | `src/preintegration/preintegration_factor.h` | 预积分包装因子 |
| IMU 误差因子 | `src/preintegration/imu_error_factor.h` | IMU 零偏软件约束 |
| 位姿流形 | `src/factors/pose_manifold.{h,cc}` | Ceres Manifold (Ceres 2.1+) |
| 边缘化因子 | `src/factors/marginalization_factor.h` | 边缘化先验 |
| 状态定义 | `src/preintegration/integration_state.h` | 所有状态和参数结构体 |
| 坐标转换 | `src/common/earth.h` | WGS84 椭球、ECEF/BLH/局部坐标转换 |
| 旋转工具 | `src/common/rotation.h` | 旋转矩阵/四元数/Euler/反对称矩阵 |
| 数据类型 | `src/common/types.h` | GNSS/IMU/Pose 结构体 |
| GNSS 文件读取 | `src/fileio/gnssfileloader.h` | GNSS 文本文件解析 |
| IMU 文件读取 | `src/fileio/imufileloader.h` | IMU 文本文件解析 |
| 配置文件 | `config/ob_gins.yaml` | IMU 噪声、窗口大小、初始化参数 |

---

*分析完成于 2026-04-28，基于 OB_GINS 开源代码 commit 快照。*