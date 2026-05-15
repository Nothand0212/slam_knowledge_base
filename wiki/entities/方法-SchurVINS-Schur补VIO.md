---
type: entity
tags: [VIO, Schur补, 边缘化, EKF, SVO, 视觉惯性里程计, Ceres]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/SchurVINS
---

# SchurVINS Schur 补 VIO

> 基于 Schur 补将完整 VIO 残差模型分解为自运动（ego-motion）残差和路标残差两部分，两阶段 EKF 更新实现高效率且不损失精度的视觉-惯性导航系统。

## 定义

SchurVINS 提出了一种基于滤波器的轻量级 VIO 框架。核心创新在于：
1. 为高维观测构建等效残差模型（梯度 + Hessian + 观测协方差）
2. 用 **Schur 补** 将完整残差模型分解为 ego-motion 和 landmark 两个独立的残差模型
3. 在两个模型上分别执行 EKF 更新，避免直接处理超高维观测

由 ByteDance 提出，基于 SVO2.0/rpg_svo_pro_open 开发，发表于 CVPR 2024。

## 与传统边缘化的区别

| 维度 | 传统滑窗边缘化（VINS-Mono 等） | SchurVINS Schur 补方法 |
|------|-------------------------------|------------------------|
| 问题维度 | 直接在全部状态（位姿+路标）上优化 | 分解为 ego-motion（低维）+ landmark（逐点） |
| 边缘化策略 | 边缘化最老帧时保留 Schur 先验 | **每次新观测到达时** 用 Schur 补分离路标 |
| 信息保留 | 所有边缘化项耦合在一个先验因子中 | ego-motion 和 landmark 完全解耦 |
| 路标处理 | 所有路标在优化中联合求解 | 每点独立 EKF 更新，轻量高效 |
| 后端类型 | 全批量非线性优化（Ceres/g2o） | EKF 滤波 + Ceres 边缘化 |

核心差异：SchurVINS 不只是在滑动窗口边缘化时用 Schur 补 —— 它把每次观测的 Hessian 都做 Schur 补分解，使路标变量始终与位姿变量解耦。

## 数学公式

### 完整残差模型

对于视觉重投影观测 $\mathbf{z}$，在 EKF 线性化后，残差的二阶近似为：

$$
e(\delta \mathbf{x}) \approx \frac{1}{2} \delta \mathbf{x}^T \mathbf{H} \delta \mathbf{x} + \mathbf{b}^T \delta \mathbf{x} + c
$$

其中 $\mathbf{H} \in \mathbb{R}^{N \times N}$ 是 Hessian 矩阵，$\mathbf{b}$ 是梯度项，$\delta \mathbf{x}$ 是状态增量。

### Schur 补分解

将状态分解为 ego-motion（位姿/速度/偏置）$\mathbf{x}_e$ 和路标 $\mathbf{x}_l$：

$$
\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{x}_e \\ \delta \mathbf{x}_l \end{bmatrix}, \quad
\mathbf{H} = \begin{bmatrix} \mathbf{H}_{ee} & \mathbf{H}_{el} \\ \mathbf{H}_{el}^T & \mathbf{H}_{ll} \end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix} \mathbf{b}_e \\ \mathbf{b}_l \end{bmatrix}
$$

对路标部分做 Schur 补消元：

$$
\mathbf{H}_{\text{schur}} = \mathbf{H}_{ee} - \mathbf{H}_{el} \mathbf{H}_{ll}^{-1} \mathbf{H}_{el}^T
$$

$$
\mathbf{b}_{\text{schur}} = \mathbf{b}_e - \mathbf{H}_{el} \mathbf{H}_{ll}^{-1} \mathbf{b}_l
$$

ego-motion 残差模型变为 $e_e(\delta \mathbf{x}_e) = \frac{1}{2} \delta \mathbf{x}_e^T \mathbf{H}_{\text{schur}} \delta \mathbf{x}_e + \mathbf{b}_{\text{schur}}^T \delta \mathbf{x}_e + c'$。

路标残差可以通过分离的模型独立更新：

$$
e_l(\delta \mathbf{x}_l) = \frac{1}{2} \delta \mathbf{x}_l^T \mathbf{H}_{ll} \delta \mathbf{x}_l + (\mathbf{b}_l + \mathbf{H}_{el}^T \delta \hat{\mathbf{x}}_e)^T \delta \mathbf{x}_l
$$

这样，ego-motion EKF 更新的状态维度从 $O(N_{\text{pose}} + N_{\text{landmark}})$ 降低到 $O(N_{\text{pose}})$，路标更新降为 $O(N_{\text{landmark}})$ 个独立的 1 维/3 维滤波。

## 实现细节

### 边缘化误差类（MarginalizationError）

SchurVINS 的关键数据结构和算法集中在 `MarginalizationError` 类（`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp`），它继承自 `ceres::CostFunction`，复用 Ceres 的参数块管理但不走 Ceres 优化：

```cpp
// raw/codes/SchurVINS/svo_ceres_backend/include/svo/ceres_backend/marginalization_error.hpp:L67-L68
class MarginalizationError : public ceres::CostFunction, public ErrorInterface
```

内部维护：
- **H_** (`Eigen::MatrixXd`)：全系统 Hessian 矩阵
- **b0_** (`Eigen::VectorXd`)：全系统梯度向量
- **dense_indices_**：区分 dense 参数块（位姿/速度/偏置）和 sparse 参数块（路标点）
- **J_** / **e0_**：Schur 补后的等效雅可比和等效残差
- **parameter_block_infos_**：各参数块的排序位置和维度信息

### 两步 Schur 补流程

`marginalizeOut()` 函数（`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L555-L895`）实现两步消元：

**第一步：路标（稀疏）部分消元**

对 `HomogeneousPointParameterBlock`（路标，4 维齐次坐标 → 3 维最小表示），逐块执行 Schur 补。

```cpp
// raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L683-L766
// Landmark 部分 (如果存在)
if (marginalization_start_idx_and_length_pairs_landmarks.size() > 0) {
    // 预条件
    Eigen::VectorXd p = H_.diagonal().cwiseSqrt();
    H_ = p_inv.asDiagonal() * H_ * p_inv.asDiagonal();

    // 分块: U (保留), V (被边缘化 = 路标), W (交叉)
    splitSymmetricMatrix(..., H_, U, W, V);

    // 逐路标伪逆 + Schur 消元
    for (size_t i = 0; i < V.cols(); i += sdim) {
        Eigen::MatrixXd V_inv_sqrt;
        pseudoInverseSymmSqrt(V.block(i,i,sdim,sdim), V_inv_sqrt);
        Eigen::MatrixXd M = W.block(0,i,W.rows(),sdim) * V_inv_sqrt;
        delta_H += M * M.transpose();  // 累积 Schur 项
        delta_b += M1 * b_b.segment(i);
    }
    H_ -= delta_H;   // Schur 消元后的 Hessian
    b0_ -= delta_b;  // Schur 消元后的梯度
}
```

关键点：
- 每个路标块（3×3）独立伪逆，通过 `pseudoInverseSymmSqrt` 计算 $V^{-1/2}$（基于特征值分解，截断负特征值）
- 逐点累积 Schur 补项 `M * M^T`，避免构建超大矩阵
- 预条件（Jacobi scaling）改善数值稳定性

**第二步：稠密（位姿/速度/偏置）部分消元**

对需要边缘化掉的稠密参数块（如滑出窗口的旧帧位姿），执行标准的稠密 Schur 补：

```cpp
// raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L768-L821
if (marginalization_start_idx_and_length_pairs_dense.size() > 0) {
    // 分解: U (保留), V (被消去), W (交叉)
    splitSymmetricMatrix(..., H_, U, W, V);

    // Schur 补
    Eigen::MatrixXd V_inverse_sqrt;
    pseudoInverseSymmSqrt(0.5*(V+V.transpose()), V_inverse_sqrt);
    Eigen::MatrixXd M = W * V_inverse_sqrt;

    b0_ = b_a - M * V_inverse_sqrt.transpose() * b_b;  // 更新梯度
    H_ = U - M * M.transpose();                         // 更新 Hessian
}
```

### updateErrorComputation：从 Hessian 到残差因子

边缘化完成后，调用 `updateErrorComputation()`（`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L897-L947`）：

1. **Eigenvalue thresholding**：对 `0.5 * (H + H^T)` 做 SVD/特征值分解，将小于 `ε * max_eigenvalue` 的特征值置零，保证正半定性
2. **构造雅可比 J_**：$J = \sqrt{S} \cdot U^T \cdot P^{-1}$，其中 $P = \text{diag}(p)$ 是预条件矩阵
3. **构造等效残差 e0_**：$e_0 = -S^{-1/2} U^T P^{-1} b_0$
4. 残差计算式：$e = e_0 + J \cdot \Delta \chi$

这样边缘化先验成为一个标准的 Ceres 残差因子，可以在后续优化中计算雅可比和残差。

### 边缘化策略

`Estimator::applyMarginalizationStrategy()`（`raw/codes/SchurVINS/svo_ceres_backend/src/estimator.cpp:L632-L1065`）：

- **保留策略**：最新 `num_keyframes` 个关键帧 + `num_imu_frames` 个帧（不分是否是关键帧）
- **两类边缘化**：
  - `marginalize_all_but_pose_frames`：只保留位姿、消去速度和偏置
  - `marginalize_pose_frames`：完全消去帧的所有相关参数块（包括外参、路标）
- **路标处理**：如果在被消去的帧中最后一次可见且不在新窗口内可见 → 边缘化该路标；如果仍可见 → 保留
- **步骤流水线**（5 步计时）：收集非位姿项 → 收集位姿项 → 执行实际边缘化 → 更新误差计算 → 读入新图

### 高维观测的降维

传统 VIO 若直接对每帧所有特征点构建 EKF 观测，状态维度可达数千维。SchurVINS 的策略：

1. 对每帧图像，先通过前端（SVO 直接法 + 特征点跟踪）检测特征
2. 构建当前帧所有观测的联合残差模型（梯度 g + Hessian H + 协方差 Σ）
3. 通过 Schur 补将路标维度消去，得到压缩后的 ego-motion 残差（维度 = 位姿维度，如 6-9 维）
4. EKF 用压缩后的残差更新 ego-motion
5. 用更新后的 ego-motion 独立更新各个路标

## Agent 实现提示

### 适用场景
- 计算资源受限的平台（无人机嵌入式端、移动设备）
- 需要实时 VIO 但无法承受完整 BA 的计算量
- 特征点数量较多（100+）但需要保持 EKF 轻量的场景
- 作为 SVO2.0 后端的升级替换

### 输入输出契约
- **输入**：图像帧（单目/双目）、IMU 测量（加速度计 + 陀螺仪）、相机标定、IMU 噪声参数
- **输出**：6-DoF 位姿（T_WS）、速度、IMU 偏置、特征点 3D 位置
- **坐标**：IMU frame（S frame）为 body frame，相机 frame（C frame）通过外参 $T_{SC}$ 标定

### 实现骨架（伪代码）
```pseudo
class SchurVINS_EKF:
    def process_frame(image, imu_meas):
        # 1. IMU 传播到当前帧时刻
        T_WS_pred, cov_pred = imu_propagation(prev_state, imu_meas)

        # 2. 构建完整残差模型
        H_ee, H_el, H_ll, b_e, b_l = build_full_residual_model(
            T_WS_pred, image_features, feature_depths)

        # 3. Schur 补消去路标
        H_schur = H_ee - H_el * inv(H_ll) * H_el^T
        b_schur = b_e - H_el * inv(H_ll) * b_l

        # 4. Ego-motion EKF 更新
        K = cov_pred * H_schur^T / (H_schur * cov_pred * H_schur^T + R)
        T_WS_new = T_WS_pred ⊕ (K * b_schur)

        # 5. 路标独立 EKF 更新
        for each landmark l:
            b_l_cond = b_l + H_el^T * (T_WS_new ⊖ T_WS_pred)
            landmark_l.update(b_l_cond, H_ll[l])

        # 6. 滑动窗口边缘化（消去旧帧状态）
        if need_marginalization:
            MarginalizationError.marginalizeOut(old_frame_ids)
```

### 关键源码片段

**Schur 补核心实现（稠密部分）**

`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L808-L816`

```cpp
// Schur
Eigen::MatrixXd M = W * V_inverse_sqrt;
// rhs
b0_.resize(b_a.rows());
b0_ = (b_a - M * V_inverse_sqrt.transpose() * b_b);
// lhs
H_.resize(U.rows(), U.cols());
H_ = (U - M * M.transpose());
```

**路标伪逆（特征值截断）**

`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L739-L741`

```cpp
Eigen::Matrix<double, sdim, sdim> V_inv_sqrt;
Eigen::Matrix<double, sdim, sdim> V1 = V.block(i, i, sdim, sdim);
MarginalizationError::pseudoInverseSymmSqrt(V1, V_inv_sqrt);
```

**updateErrorComputation — Hessian → J/e0 重构**

`raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp:L918-L941`

```cpp
// SVD 分解并截断小特征值
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(
    0.5 * p_inv.asDiagonal() * (H_ + H_.transpose()) * p_inv.asDiagonal());
double tolerance = epsilon * H_.cols() * saes.eigenvalues().array().maxCoeff();
S_ = Eigen::VectorXd(
    (saes.eigenvalues().array() > tolerance).select(
        saes.eigenvalues().array(), 0));
// 构造雅可比 J 和等效残差 e0
J_ = (p.asDiagonal() * saes.eigenvectors() * (S_sqrt_.asDiagonal())).transpose();
e0_ = (-J_pinv_T * b0_);
```

**边缘化策略入口**

`raw/codes/SchurVINS/svo_ceres_backend/src/estimator.cpp:L632-L639`

```cpp
bool Estimator::applyMarginalizationStrategy(
    size_t num_keyframes, size_t num_imu_frames, MarginalizationTiming* timing)
{
    // 保留最新 num_imu_frames 帧
    for (size_t k = 0; k < num_imu_frames; ++k) { ++rit_id; ++rit_keyframe; }
    // 滑动窗口外的帧：keyframe 全消 / 非 keyframe 消位姿
    while (rit_id != states_.ids.rend()) {
        if (!(*rit_keyframe) || counted_keyframes >= num_keyframes)
            marginalize_pose_frames.push_back(*rit_id);
        marginalize_all_but_pose_frames.push_back(*rit_id);
    }
```

### 实现注意事项
- 齐次坐标路标参数块使用 `HomogeneousPointParameterBlock`（4 维）和 `HomogeneousPointLocalParameterization`（最小表示 3 维），在添加残差块时需要正确区分 dense/sparse 索引
- `dense_indices_` 标记 "稠密" 参数块的数量（位姿/速度/偏置/外参），路标被视为 "稀疏"，分别执行不同策略的 Schur 补
- 预条件（对角 Jacobi scaling）对数值稳定性至关重要：`H_ = p_inv * H_ * p_inv`；Schur 消元后要恢复缩放 `H_ = p_a * H_ * p_a`
- 负特征值截断：路标 Hessian 的伪逆中 `pseudoInverseSymmSqrt` 将负特征值置零，保证半正定性
- `MarginalizationError` 既是 `ceres::CostFunction`（可以被 Ceres 优化器读取雅可比/残差），也维护线性系统的内部状态
- 边缘化后 `residual_block` 从 Map 中移除，线性化点固定在边缘化时刻的估计值上

### 源码检索锚点
- `raw/codes/SchurVINS/svo_ceres_backend/src/marginalization_error.cpp` — 边缘化误差类的完整实现（Schur 补核心）
- `raw/codes/SchurVINS/svo_ceres_backend/include/svo/ceres_backend/marginalization_error.hpp` — 边缘化误差类 API
- `raw/codes/SchurVINS/svo_ceres_backend/src/estimator.cpp` — Estimator 主类，`applyMarginalizationStrategy`
- `raw/codes/SchurVINS/svo_ceres_backend/include/svo/ceres_backend/estimator.hpp` — Estimator API + States 数据结构
- `raw/codes/SchurVINS/svo_ceres_backend/src/homogeneous_point_parameter_block.cpp` — 齐次坐标路标参数块
- `raw/codes/SchurVINS/svo_ceres_backend/src/imu_error.cpp` — IMU 误差模型
- `raw/codes/SchurVINS/svo_ceres_backend/include/svo/ceres_backend/map.hpp` — 参数/残差 Map 管理

## 相关页面

- VINS-Mono
- [[概念-Schur补与边缘化]]
- [[概念-因子图]]
- [[概念-Schur补与边缘化]]
- [[概念-MSCKF]]
- [[概念-IMU预积分]]
- [[算法-SVO-Pro]]
