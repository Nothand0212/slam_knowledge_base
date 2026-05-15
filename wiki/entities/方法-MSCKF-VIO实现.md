---
tags: [MSCKF, VIO, EKF, 零空间投影, 多状态约束, 视觉惯性, 特征三角化]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/msckf_vio/include/msckf_vio/msckf_vio.h
  - raw/codes/msckf_vio/src/msckf_vio.cpp
  - raw/codes/msckf_vio/include/msckf_vio/imu_state.h
  - raw/codes/msckf_vio/include/msckf_vio/cam_state.h
  - raw/codes/msckf_vio/include/msckf_vio/feature.hpp
---

# MSCKF: Multi-State Constraint Kalman Filter VIO

> MSCKF (Multi-State Constraint Kalman Filter) 是 UPenn Kumar Robotics Lab 开发的经典 VIO 算法。核心创新：不将 3D 特征点作为 EKF 状态（避免 EKF-SLAM 的 $O(N^2)$ 协方差膨胀），而是维持一个滑动窗口的相机位姿作为 EKF 状态，通过**零空间投影**消除特征位置变量，将多视角观测转化为相机位姿间的约束。

## 系统架构

```
IMU ──→ [batchImuProcessing] ──→ 状态传播 (EKF predict)
                                      │
图像 ──→ [image_processor] ──→ 特征检测 & KLT 追踪
                                      │
              ┌───────────────────────┘
              │
              ├─ stateAugmentation: 将当前相机位姿加入状态 (最多30个)
              ├─ addFeatureObservations: 累积特征观测
              ├─ 当特征丢失/超限时:
              │    ├─ featureJacobian: 堆积 H_x, H_f, r
              │    ├─ 零空间投影 (SVD of H_f) → H_x', r'
              │    ├─ measurementUpdate: EKF 更新
              │    └─ removeLostFeatures
              └─ pruneCamStateBuffer: 删除冗余相机状态
```

## EKF 状态向量

IMU 状态（21 维）加上最多 N 个克隆的相机状态（每个 6 维）：

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L151-L175`（初始化）

```cpp
struct StateServer {
    IMUState imu_state;          // 21维: q_IG(4), b_g(3), v_I^G(3), b_a(3), p_I^G(3),
                                 //        q_CI(4), p_C^I(3)
    CamStateServer cam_states;   // 每个: q_CG(4), p_C^G(3) → 6 DOF
    MatrixXd state_cov;          // (21+6N)×(21+6N)
};
```

IMU 状态详细：

$$
\mathbf{x}_{\text{IMU}} = \begin{bmatrix}
{}^{I}_{G}\bar{q}^T & \mathbf{b}_g^T & {}^{G}\mathbf{v}_I^T & \mathbf{b}_a^T & {}^{G}\mathbf{p}_I^T & {}^{I}_{C}\bar{q}^T & {}^{C}\mathbf{p}_I^T
\end{bmatrix}^T
$$

维度：$4 + 3 + 3 + 3 + 3 + 4 + 3 = 21$（注：姿态用四元数存储但协方差中误差状态为 3 维）

相机状态（每个克隆帧）：

$$
\mathbf{x}_{C_i} = \begin{bmatrix} {}^{C_i}_{G}\bar{q}^T & {}^{G}\mathbf{p}_{C_i}^T \end{bmatrix}^T
$$

默认最多 30 个相机状态（`max_cam_state_size = 30`），协方差矩阵最多为 $(21+180) \times (21+180) = 201 \times 201$。

## IMU 状态传播

IMU 数据在到达时累积在 `imu_msg_buffer` 中。当收到图像时，执行批量 IMU 处理。

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L750-L800`

传播方程（标准 EKF predict）：

```cpp
void predictNewState(const double& dt,
    const Eigen::Vector3d& gyro,
    const Eigen::Vector3d& acc) {
    // 姿态四元数传播
    Vector4d dq = quaternionFromAngleAxis(
        (gyro - imu_state.gyro_bias) * dt);
    imu_state.orientation = quaternionMultiplication(
        imu_state.orientation, dq);
    // 速度传播 (世界系)
    Matrix3d R_G_I = quaternionToRotation(imu_state.orientation);
    imu_state.velocity += dt * (R_G_I * (acc - imu_state.acc_bias) + gravity);
    // 位置传播
    imu_state.position += dt * imu_state.velocity;
}

// 协方差传播
Matrix<double, 21, 21> F = Matrix<double, 21, 21>::Identity();
Matrix<double, 21, 12> G = Matrix<double, 21, 12>::Zero();
// IMU 连续时间误差状态动力学: δx' = F_c δx + G_c n
// 离散化: F = exp(F_c * dt), Q = ∫ exp(F_c*τ) G_c Q_c G_c^T exp(F_c*τ)^T dτ
```

## 状态增广 (State Augmentation)

每当新图像到达时，将当前 IMU 姿态传播的相机位姿加入 EKF 状态：

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L780-L795`

```cpp
void stateAugmentation(const double& time) {
    // 当前相机位姿 = IMU位姿 * IMU->相机外参
    Matrix3d R_G_I = quaternionToRotation(imu_state.orientation);
    Matrix3d R_G_C = R_G_I * imu_state.R_imu_cam0;
    Vector3d p_C_G = imu_state.position
        + R_G_I * imu_state.t_cam0_imu;

    CAMState new_cam_state;
    new_cam_state.id = time;  // StateID
    new_cam_state.orientation = rotationToQuaternion(R_G_C);
    new_cam_state.position = p_C_G;
    new_cam_state.orientation_null = new_cam_state.orientation;
    new_cam_state.position_null = new_cam_state.position;

    // 协方差增广: P_{new} = J * [P, 0; 0, R_{imu2cam}] * J^T
    cam_states[new_cam_state.id] = new_cam_state;
}
```

## 特征三角化与零空间投影

这是 MSCKF 最核心的机制。当一个特征被多个相机状态观测到，直接堆积所有观测的雅可比：

### 观测模型

一个特征 $j$ 在相机 $i$ 中的 stereo 观测 $\mathbf{z}_i^j$（4 维：`[u_L, v_L, u_R, v_R]`）：

$$
\mathbf{z}_i^j = \begin{bmatrix}
\frac{X_{C_i}}{Z_{C_i}} \\ \frac{Y_{C_i}}{Z_{C_i}} \\ \frac{X_{C_i} - b}{Z_{C_i}} \\ \frac{Y_{C_i}}{Z_{C_i}}
\end{bmatrix} = \mathbf{h}(\mathbf{x}_{C_i}, \mathbf{p}_f^G)
$$

### Jacobian 堆积

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L862-L915`

```cpp
void featureJacobian(const FeatureIDType& feature_id,
    const std::vector<StateIDType>& cam_state_ids,
    MatrixXd& H_x, VectorXd& r) {

    // 堆积所有相机的观测雅可比
    H_xj = Zero(jacobian_row_size, 21 + 6*N_cam_states);
    H_fj = Zero(jacobian_row_size, 3);
    r_j  = Zero(jacobian_row_size);

    for (const auto& cam_id : valid_cam_state_ids) {
        measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);
        H_xj.block<4,6>(stack_cntr, 21+6*cam_state_cntr) = H_xi;
        H_fj.block<4,3>(stack_cntr, 0) = H_fi;
        r_j.segment<4>(stack_cntr) = r_i;
        stack_cntr += 4;
    }
}
```

### 零空间投影 (Nullspace Projection)

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L906-L914`

特征位置变量 $\mathbf{p}_f$ 不出现在 EKF 状态中。将测量方程线性化后：

$$
\mathbf{r} \simeq \mathbf{H}_x \delta\mathbf{x} + \mathbf{H}_f \delta\mathbf{p}_f + \mathbf{n}
$$

通过 SVD 分解 $\mathbf{H}_f$ 找到其零空间的基：

$$
\mathbf{H}_f = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T
$$

取 $\mathbf{U}$ 的右部列（对应零奇异值方向）$\mathbf{A} = \mathbf{U}_{:,4:}$，左乘消去 $\mathbf{H}_f$：

```cpp
JacobiSVD<MatrixXd> svd_helper(H_fj, ComputeFullU | ComputeThinV);
MatrixXd A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);

H_x = A.transpose() * H_xj;   // 投影后的约束矩阵
r = A.transpose() * r_j;      // 投影后的残差
```

**关键理解**：$\mathbf{A}^T \mathbf{H}_f = \mathbf{0}$，所以投影后的方程 $\mathbf{A}^T \mathbf{r} = \mathbf{A}^T \mathbf{H}_x \delta\mathbf{x}$ 不再包含特征位置变量。一个被 $M$ 个相机观测的特征（$4M$ 个观测方程）经过投影后产生 $4M - 3$ 个有效约束，全部作用在相机位姿之间。

### Measurement Jacobian 细节

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L800-L860`

```cpp
void measurementJacobian(const StateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Matrix<double, 4, 6>& H_x,    // 对相机位姿的雅可比
    Matrix<double, 4, 3>& H_f,    // 对特征位置的雅可比
    Vector4d& r) {                // 残差

    // 投影雅可比 (针孔相机模型)
    // ∂z/∂p_c = [1/Z, 0, -X/Z²; 0, 1/Z, -Y/Z²]
    dz_dpc0(0,0) = 1/p_c0(2);  dz_dpc0(1,1) = 1/p_c0(2);
    dz_dpc0(0,2) = -p_c0(0)/(p_c0(2)*p_c0(2));
    dz_dpc0(1,2) = -p_c0(1)/(p_c0(2)*p_c0(2));

    // 特征在相机系的坐标对相机位姿的雅可比
    dpc0_dxc.leftCols(3) = skewSymmetric(p_c0);  // 对旋转
    dpc0_dxc.rightCols(3) = -R_w_c0;             // 对平移

    // 对特征位置的雅可比
    dpc0_dpg = R_w_c0;

    H_x = dz_dpc0*dpc0_dxc + dz_dpc1*dpc1_dxc;
    H_f = dz_dpc0*dpc0_dpg + dz_dpc1*dpc1_dpg;
}
```

### 可观测性约束 (OC)

MSCKF 施加可观测性约束（observability constraint），确保零空间维度正确。标准 VINS 系统有 4 个不可观方向（绕重力方向的偏航 + 全局位置）。对 $\mathbf{H}_x$ 施加约束：

```cpp
Matrix<double, 4, 6> A = H_x;
Matrix<double, 6, 1> u = Zero();
u.block<3,1>(0,0) = quaternionToRotation(
    cam_state.orientation_null) * IMUState::gravity;
u.block<3,1>(3,0) = skewSymmetric(
    p_w - cam_state.position_null) * IMUState::gravity;
H_x = A - A*u*(u.transpose()*u).inverse()*u.transpose();
H_f = -H_x.block<4,3>(0, 3);
```

强制 $\mathbf{H}_x$ 与不可观方向正交，防止滤波器在不可观方向上产生虚假信息。

## EKF 更新

**源码锚点**: `raw/codes/msckf_vio/src/msckf_vio.cpp:L918-L999`

标准 EKF 更新方程（使用 QR 分解降维提高效率）：

```cpp
void measurementUpdate(const MatrixXd& H, const VectorXd& r) {
    // QR 降维: H 可能很大，先做 QR 分解减少维度
    if (H.rows() > H.cols()) {
        SPQR<SparseMatrix<double>> spqr_helper;
        spqr_helper.compute(H.sparseView());
        H_thin = (Q^T * H).topRows(21+6*N);
        r_thin = (Q^T * r).head(21+6*N);
    }

    // Kalman gain
    S = H_thin*P*H_thin^T + R;
    K = P*H_thin^T * S^{-1};

    // 状态更新 (误差状态)
    delta_x = K * r_thin;

    // 注入名义状态
    imu_state.orientation += dq_imu;
    imu_state.gyro_bias += delta_x_imu[3:6];
    imu_state.velocity += delta_x_imu[6:9];
    imu_state.acc_bias += delta_x_imu[9:12];
    imu_state.position += delta_x_imu[12:15];

    // 更新相机位姿 (误差状态)
    for each cam_state:
        cam_state.orientation += dq_cam;
        cam_state.position += delta_x_cam;

    // 协方差更新
    P = (I - K*H_thin) * P;
}
```

## PRUNE：相机状态管理

当相机状态超过 `max_cam_state_size`（默认 30）时，需要删除一些。选择标准：使删除后的协方差变化最小（即保留信息量最大的状态）。

通常会保留间隔均匀的相机状态（keyframe selection），保证足够的视差以支持特征三角化。

## Agent 实现提示

### 适用场景

中等规模场景的实时 VIO，计算资源受限但需要比纯 EKF-SLAM 更好的 consistency。特别适合无人机/手持设备的视觉惯性导航。相比 ROVIO（基于 patch 的直接法），MSCKF 使用传统特征（KLT tracker），对光照变化更鲁棒。

### 输入输出契约

- **输入**：IMU（200 Hz）、双目图像（20-40 Hz）、相机内参/外参
- **输出**：6-DOF 位姿、线速度、IMU bias、特征点云（用于可视化）
- **状态维度**：IMU 21 维 + 相机位姿 $6 \times N$（N ≤ 30）

### 实现骨架（伪代码）

```pseudo
function MSCKF.run():
    state = init_imu_state()
    cov = init_covariance()
    imu_buffer = []
    cam_states = {}
    feature_map = {}

    for each IMU measurement:
        imu_buffer.append(imu)
        if imu_buffer.size > threshold:
            batch_imu_processing(time_bound)

    for each image pair:
        // 1. 批量处理缓冲的IMU数据直到当前图像时间
        batch_imu_processing(image_time)
        // 2. 状态增广
        state_augmentation(image_time)
        // 3. 特征追踪 & 添加观测
        add_feature_observations(feature_msg)
        // 4. 对丢失/超限特征执行更新
        for each lost_feature:
            H_x, r = feature_jacobian(feature, cam_state_ids)
            // 零空间投影
            SVD(H_f) -> A = U.rightCols(N-3)
            H_x_proj = A^T * H_x
            r_proj = A^T * r
            // 卡方检验
            if gating_test(H_x_proj, r_proj, dof):
                measurement_update(H_x_proj, r_proj)
            remove_feature(feature)
        // 5. 删除冗余相机状态
        prune_cam_state_buffer()
```

### 关键源码片段

**零空间投影**（`msckf_vio.cpp:L906-L914`）：
```cpp
JacobiSVD<MatrixXd> svd_helper(H_fj, ComputeFullU | ComputeThinV);
MatrixXd A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);
H_x = A.transpose() * H_xj;
r = A.transpose() * r_j;
```

**可观测性约束**（`msckf_vio.cpp:L844-L853`）：
```cpp
Matrix<double, 4, 6> A = H_x;
Matrix<double, 6, 1> u = Matrix<double, 6, 1>::Zero();
u.block<3, 1>(0, 0) = quaternionToRotation(
    cam_state.orientation_null) * IMUState::gravity;
u.block<3, 1>(3, 0) = skewSymmetric(
    p_w - cam_state.position_null) * IMUState::gravity;
H_x = A - A*u*(u.transpose()*u).inverse()*u.transpose();
```

**观测雅可比**（`msckf_vio.cpp:L800-L858`）：
```cpp
// stereo projection jacobian
dz_dpc0(0,0) = 1/p_c0(2);  dz_dpc0(1,1) = 1/p_c0(2);
dz_dpc0(0,2) = -p_c0(0)/(p_c0(2)*p_c0(2));
dz_dpc0(1,2) = -p_c0(1)/(p_c0(2)*p_c0(2));
// pose jacobian
dpc0_dxc.leftCols(3) = skewSymmetric(p_c0);
dpc0_dxc.rightCols(3) = -R_w_c0;
```

### 实现注意事项

1. **零空间投影的正确实现**：`rightCols(jacobian_row_size - 3)` 的原因是 $\mathbf{H}_f$ 的列数为 3（特征位置的 3 维），左零空间维度 = 行数 - rank = 4M - 3
2. **可观测性约束必须施加**：不施加会导致滤波器在偏航方向虚假收敛
3. **QR 降维**：使用 SPQR 而非标准 QR，因为 $\mathbf{H}$ 是稀疏矩阵
4. **特征处理时序**：只有在特征丢失/跟踪失败时才触发更新（延迟更新策略），利用所有已积累的观测
5. **相机状态数量**：30 是经验值，太少影响精度，太多增加计算负担
6. **卡方检验**：`gating_test` 用于外点剔除，避免错误观测破坏滤波器

### 源码检索锚点

| 模块 | 文件 | 行号 |
|------|------|------|
| StateServer 定义 | `msckf_vio.h` | L70-L77 |
| 类成员函数声明 | `msckf_vio.h` | L131-L164 |
| 参数加载 | `msckf_vio.cpp` | L60-L150 |
| 预测新状态 | `msckf_vio.cpp` | L750-L780 |
| 状态增广 | `msckf_vio.cpp` | L780-L800 |
| 观测雅可比 | `msckf_vio.cpp` | L800-L860 |
| 特征雅可比 (零空间投影) | `msckf_vio.cpp` | L862-L915 |
| EKF 更新 | `msckf_vio.cpp` | L918-L999 |
| IMU 状态定义 | `imu_state.h` | - |
| 相机状态定义 | `cam_state.h` | - |

## 相关页面

- [[方法-ROVIO-EKF-VIO]]
- [[方法-IESKF滤波器]]
- [[方法-VIO初始化方法]]
- [[方法-滑动窗口边缘化]]
