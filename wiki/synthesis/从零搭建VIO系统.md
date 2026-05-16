---
tags: [VIO, 教程, 视觉惯性里程计, 系统搭建, Ceres, IMU预积分, 滑动窗口, 回环检测]
type: synthesis
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/entities/方法-视觉特征跟踪.md
  - wiki/entities/概念-IMU预积分.md
  - wiki/entities/方法-VIO初始化方法.md
  - wiki/entities/方法-滑动窗口边缘化.md
  - wiki/entities/方法-关键帧选择策略.md
  - wiki/entities/方法-视觉回环检测管线.md
  - wiki/entities/方法-IMU-camera时间偏移估计.md
  - wiki/entities/方法-多传感器外参在线标定.md
  - wiki/synthesis/VIO方案全景对比.md
  - wiki/synthesis/优化后端选型指南.md
---

# 从零搭建 VIO 系统

> 一份可执行的工程指南：自底向上实现一个完整的单目视觉-惯性里程计（VIO）系统。以 VINS-Fusion 的设计为骨干，串联视觉前端、IMU 预积分、初始化、滑动窗口优化和回环检测五大模块。每个模块给出决策依据、伪代码、关键参数和常见坑。

---

## 0. 前置知识

### 0.1 数学基础

你需要熟悉以下数学工具，否则看公式会很吃力：

| 领域 | 具体内容 | 推荐材料 |
|------|---------|---------|
| 李群李代数 | SO(3)/SE(3) 定义, exp/log 映射, 左右雅可比, 扰动模型 | 《视觉SLAM十四讲》第 4 讲 |
| 非线性优化 | Gauss-Newton, Levenberg-Marquardt, Dogleg, 鲁棒核函数, Huber/Cauchy loss | [[概念-因子图]], [[优化后端选型指南]] |
| IMU 运动学 | 连续/离散 IMU 积分, 预积分理论, 误差状态传递 | [[概念-IMU预积分]] |
| 多视图几何 | 本质矩阵/基础矩阵/单应矩阵, PnP, 三角化, RANSAC | [[概念-PnP 运动估计]], [[概念-三角化与深度估计]] |
| 概率与估计 | 高斯噪声模型, 最大后验估计 (MAP), 协方差传播, 边缘化 | — |

### 0.2 编程基础

| 技能 | 说明 |
|------|------|
| C++ 14/17 | VIO 系统的主语言, 推荐用 `Eigen`, `Ceres`, `OpenCV` |
| Eigen 3.x | 矩阵/向量运算, SVD/LDLT/QR 分解, 几何变换 |
| Ceres Solver | 非线性优化后端, AutoDiff, 自定义 CostFunction | [[组件-Ceres-Solver\|参考]] |
| OpenCV | 图像预处理, 特征检测, 光流跟踪 |
| ROS/ROS2 | 传感器数据接入 (非必须, 但工程上常用) |

### 0.3 必读论文

| 论文 | 关联主题 | 阅读重点 |
|------|---------|---------|
| Forster et al., *IMU Preintegration on Manifold*, RSS 2015 / TRO 2017 | IMU 预积分 | 预积分的递推公式、协方差传播、bias 更新的 Jacbobian |
| Qin et al., *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*, TRO 2018 | 系统架构 | 初始化管线、滑动窗口边缘化策略 |
| Mur-Artal et al., *ORB-SLAM3*, TRO 2021 | 多地图、回环 | Atlas 多地图、IMU 初始化、位姿图优化 | [[算法-ORB-SLAM3\|参考]] |
| Lupton & Sukkarieh, *Visual-Inertial-Aided Navigation for High-Dynamic Motion*, TRO 2012 | IMU 预积分基础 | 相对运动增量概念，提出了预积分视角 |
| Lucas & Kanade, *An Iterative Image Registration Technique*, IJCAI 1981 | KLT 光流 | 局部窗口亮度恒定假设与最小二乘求解 |

### 0.4 开发环境建议

```bash
# 核心依赖
sudo apt install libeigen3-dev libceres-dev libopencv-dev
# 可选: ROS 传感器接入
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
# 调试工具: 实时可视化
sudo apt install ros-noetic-rviz ros-noetic-rqt-plot
# 可选: DBoW2 回环检测
git clone https://github.com/dorian3d/DBoW2.git
```

**建议数据集**：先跑 Euroc MAV 数据集（室内, 双目+IMU, 有 ground truth），再挑战户外 KITTI 或你自己的硬件。

---

## 1. 视觉前端

> 从原始图像中提取并跟踪稀疏特征点，为后端提供帧间 2D 观测约束。详细见 [[方法-视觉特征跟踪]]。

### 1.1 Shi-Tomasi vs FAST vs SuperPoint：你该选哪个

| 维度 | Shi-Tomasi | FAST | SuperPoint |
|------|-----------|------|-----------|
| 角点质量 | 高（双方向梯度保证，连续得分可排序） | 中（启发式二值判定） | 高（学习到的模式，含描述子） |
| 计算开销 | 中（需梯度计算） | 低（仅亮度比较） | 高（需要 GPU） |
| 跟踪稳定性 | 高（适合 KLT） | 中（可能选到噪声） | 高（对光照/模糊鲁棒） |
| 回环/重定位 | ❌ 无描述子 | ✅ 配合 ORB+BRIEF | ✅ 自带描述子 |
| 代表项目 | VINS-Fusion | ORB-SLAM3 | DROID-SLAM |

**选型决策树**：
```
是否需要回环/重定位？
├── 是 → ORB 流水线 (FAST + Steered BRIEF + DBoW2)
└── 否 → 是否有 IMU？
    ├── 是 → KLT 光流 + Shi-Tomasi 补点（推荐！VINS-Fusion 风格）
    └── 否 → SuperPoint（鲁棒）/ ORB（兼容性好）
```

本指南采用 **KLT 光流 + Shi-Tomasi** 方案，这是 VIO 入门的最佳平衡点——实现简单、不需要 GPU、跟踪稳定、200+ Hz 可跑。

### 1.2 KLT 金字塔光流

**光流约束方程**：对局部窗口内 n×n 个像素，假设亮度恒定 $I_x u + I_y v + I_t = 0$，构造超定方程：

$$
A d = b,\quad d = (A^T A)^{-1} A^T b
$$

**金字塔 KLT**：从最低分辨率开始跟踪，逐层精化，可以处理更大的帧间运动。常用参数：

| 参数 | VINS-Fusion 默认 | 含义 |
|------|-----------------|------|
| 窗口大小 | 21×21 px | 局部窗口，太小欠约束，太大假设不成立 |
| 金字塔层数 | 3 (失败时退到 3) / 1 (有预测时) | 层数太多会导致底层特征点失真 |
| 最大迭代 | 30 | 每层最多迭代次数 |
| 停止 EPS | 0.01 | 两层间最小光流变化 |
| 使用初始流 | 是（IMU 预测位姿→投影） | 有 IMU 时必须打开 |

**常见坑**：
- **窗口太大**（>31px）：假设局部窗口内像素运动一致的假设会失效，尤其在大旋转时
- **金字塔层数过多**（>5 层）：底层分辨率太低，小特征点（<3px）无法被可靠跟踪
- **忘记 forward-backward check**：正向跟踪后反向验证，丢掉的点远少于投入的计算开销

### 1.3 IMU-driven 特征预测

这是 VIO 前端的核心技巧——用 IMU 预积分预测当前帧的位姿，将上一帧的特征点投影到当前帧，作为 KLT 光流的初始值。具体到像素坐标：

```
1. 上帧已去畸变归一化坐标 p_n_prev → 相机坐标 p_c_prev = [p_n_prev, 1]^T
2. 由 IMU 预测从 prev 到 curr 时刻的位姿变换 {R_prev2curr, t_prev2curr}
3. p_c_curr = R_prev2curr * (p_c_prev - t_prev2curr)
4. 投影回像素：p_px_curr = K * (p_c_curr / p_c_curr.z)
5. 以 p_px_curr 作为 KLT 的 initial_guess（启用 OPTFLOW_USE_INITIAL_FLOW）
```

**效果**：如果 IMU 高频（200+ Hz），预测非常准。KLT 只需做 1-3 次迭代即收敛，大幅节约计算。更重要的是，在快速旋转场景下，光流窗口不会漂丢。

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L117-L133` — 有预测时只用 1 层金字塔，失败点 < 10 时才退回 3 层。

### 1.4 特征生命周期管理

```pseudo
function track_sparse_features(prev_frame, curr_frame, tracks, config):
    // Step 1: KLT 跟踪上一帧的特征点
    curr_pts, status = pyr_lk(prev_frame.gray, curr_frame.gray,
                               tracks.prev_pts, config=config.klt)
    // Step 2: 双向验证（防止误跟踪）
    if config.forward_backward_check:
        back_pts, _ = pyr_lk(curr_frame.gray, prev_frame.gray, curr_pts)
        status = status AND norm(back_pts - tracks.prev_pts) < config.fb_threshold_px

    // Step 3: 剔除失败点、越界点
    curr_pts, ids, track_cnt = compact_and_reject_border(curr_pts, ids, track_cnt, status, img_size)

    // Step 4: 补新点 — mask 优先保留长跟踪点
    mask = build_spatial_mask(img_size, curr_pts, config.min_dist_px)
    n_need = config.max_features - len(curr_pts)
    if n_need > 0:
        new_pts = goodFeaturesToTrack(curr_frame.gray, n_need,
                                       quality=0.01, min_dist=config.min_dist_px, mask=mask)
        append(ids, track_cnt, new_pts)

    // Step 5: 去畸变，输出归一化坐标 + 像素速度
    un_pts = undistortedPts(curr_pts, camera)
    velocity = compute_velocity(ids, un_pts, prev_un_pts, dt)
    return FeatureFrame(curr_pts, un_pts, ids, track_cnt, velocity)
```

**补点策略精髓**（VINS-Fusion 模式）：
1. 先通过 KLT 跟踪上一帧的所有特征点（不做全量检测）
2. **只对跟踪丢失的区域补点**，而不是每帧从头检测
3. 用 `setMask()` 优先保留 `track_cnt` 高的长跟踪点，新角点必须与已有角点保持 `MIN_DIST` 距离
4. `qualityLevel=0.01` 是一个非常低的阈值——宁可多选后在 mask 阶段淘汰，也不要因阈值过高导致低纹理区域无点

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L176-L200`

### 1.5 你需要的参数参考

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `MAX_CNT` | 150 (VGA), 200-400 (HD) | 每帧最多跟踪的特征数 |
| `MIN_DIST` | 10 px (VGA), 12-20 px (HD) | 角点间最小距离，太大覆盖不足，太小簇集 |
| `qualityLevel` | 0.01 | Shi-Tomasi 阈值 = qualityLevel × max(eigenvalue) |
| `fb_threshold_px` | 1.5 px | 双向光流一致性阈值 |
| `ransac_threshold` | 2.0 / fx（归一化平面） | 基础矩阵 RANSAC 阈值，注意是归一化坐标不是像素 |
| `klt_win_size` | 21×21 | KLT 局部窗口 |
| `klt_pyr_levels` | 3 | 金字塔层数（有预测时退到 1） |
| `klt_max_iter` | 30 | 每层最大迭代次数 |

---

## 2. IMU 预积分

> 将两关键帧间的高频 IMU 测量预先积分为一个相对运动增量，使优化时无需重复积分。详见 [[概念-IMU预积分]]。

### 2.1 中值积分 vs RK4：你该选哪个

| 方法 | 精度 | 复杂度 | 代表项目 | 适用场景 |
|------|------|--------|---------|---------|
| 欧拉离散 | 低 | 极简 | fusions_slam | 快速原型 |
| 零阶预积分 | 低 | 简 | ORB-SLAM3 | 已有视觉约束强时的补充 |
| **中值积分 (RK2)** | **中** | **中** | **VINS-Fusion** | **入门首选** |
| RK4 + 三阶 Phi | 高 | 较高 | msckf_vio | 高速无人机 |
| SE2(3) 精确积分 | 最高 | 最高 | OpenMAVIS | 高动态+高精度 |

**推荐**：入门用中值积分。它比欧拉精度高一个数量级，比 RK4 实现简单很多，且 VINS-Fusion 有完整的开源参考实现。

### 2.2 中值预积分的递推公式

设在关键帧 i 和 j 之间有 N 个 IMU 测量 $\{a_0, \omega_0, \dots, a_{N-1}, \omega_{N-1}\}$，步长为 $\Delta t$。预积分量 $\boldsymbol{\alpha}_{b_k}^{b_{k+1}}, \boldsymbol{\beta}_{b_k}^{b_{k+1}}, \boldsymbol{\gamma}_{b_k}^{b_{k+1}}$ 分别表示位置增量、速度增量和旋转增量（用四元数或 SO(3) 表示），递推公式：

$$
\begin{aligned}
\bar{\omega}_k &= \frac{1}{2}(\omega_k + \omega_{k+1}) - b_g \\
\bar{a}_k &= \frac{1}{2}\left[ \boldsymbol{\gamma}_{b_k}^{b_k}(a_k - b_a) + \boldsymbol{\gamma}_{b_k}^{b_{k+1}}(a_{k+1} - b_a) \right] \\
\boldsymbol{\gamma}_{b_k}^{b_{k+1}} &= \boldsymbol{\gamma}_{b_k}^{b_k} \otimes \text{Exp}(\bar{\omega}_k \cdot \Delta t) \\
\boldsymbol{\beta}_{b_k}^{b_{k+1}} &= \boldsymbol{\beta}_{b_k}^{b_k} + \bar{a}_k \cdot \Delta t \\
\boldsymbol{\alpha}_{b_k}^{b_{k+1}} &= \boldsymbol{\alpha}_{b_k}^{b_k} + \boldsymbol{\beta}_{b_k}^{b_k} \cdot \Delta t + \frac{1}{2}\bar{a}_k \cdot \Delta t^2
\end{aligned}
$$

### 2.3 协方差传播与偏置更新

**协方差递推**：设误差状态 $\delta \mathbf{x} = [\delta \boldsymbol{\alpha}, \delta \boldsymbol{\beta}, \delta \boldsymbol{\theta}, \delta b_a, \delta b_g]^T$，误差递推矩阵为 15×15 的 $F_k$，噪声 Jacobian 为 15×18 的 $V_k$：

$$
\Sigma_{k+1} = F_k \cdot \Sigma_k \cdot F_k^T + V_k \cdot Q \cdot V_k^T
$$

**Bias 更新的一阶近似**：这是预积分的核心技术。若 bias 在优化中从 $\bar{b}$ 变为 $\hat{b}$，通过偏置 Jacobian 做一阶修正，**无需重积分**：

$$
\begin{aligned}
\boldsymbol{\alpha}_{ij}^{\text{new}} &\approx \boldsymbol{\alpha}_{ij} + J_{b_a}^{\boldsymbol{\alpha}} \cdot \delta b_a + J_{b_g}^{\boldsymbol{\alpha}} \cdot \delta b_g \\
\boldsymbol{\beta}_{ij}^{\text{new}} &\approx \boldsymbol{\beta}_{ij} + J_{b_a}^{\boldsymbol{\beta}} \cdot \delta b_a + J_{b_g}^{\boldsymbol{\beta}} \cdot \delta b_g \\
\boldsymbol{\gamma}_{ij}^{\text{new}} &\approx \boldsymbol{\gamma}_{ij} \otimes \text{Exp}(J_{b_g}^{\boldsymbol{\gamma}} \cdot \delta b_g)
\end{aligned}
$$

其中偏置 Jacobian 在递推过程中一并计算。

### 2.4 实现骨架

```pseudo
class Preintegration:
    delta_p, delta_v: Vector3  // 位置、速度增量
    delta_q: Quaternion        // 旋转增量
    cov: Matrix15              // 15×15 协方差矩阵
    jacobian: Matrix15_6       // 偏置 Jacobian (15 行 6 列)
    sum_dt: double             // 总积分时间

    function integrate(acc0, gyr0, acc1, gyr1, dt):
        // 中值积分一步
        w_mid = 0.5 * (gyr0 + gyr1) - bg
        R_0 = delta_q.toMatrix()
        R_1 = delta_q.toMatrix() * exp_so3(w_mid * dt)
        a_mid = 0.5 * (R_0 * (acc0 - ba) + R_1 * (acc1 - ba))

        delta_p += delta_v * dt + 0.5 * a_mid * dt^2
        delta_v += a_mid * dt
        delta_q = delta_q * quat_from_rotvec(w_mid * dt)

        // 传播协方差和 Jacobian
        F, V = compute_error_jacobians(acc0, acc1, gyr0, gyr1, dt, bg, ba)
        cov = F * cov * F^T + V * noise_cov * V^T
        jacobian = F * jacobian
        sum_dt += dt

    function predict(pose_i, vel_i):
        // 用预积分预测 j 时刻的状态初值
        R_wj = pose_i.R * delta_q
        p_wj = pose_i.p + vel_i * sum_dt + 0.5 * g_w * sum_dt^2 + pose_i.R * delta_p
        v_wj = vel_i + g_w * sum_dt + pose_i.R * delta_v
        return Pose(R_wj, p_wj), v_wj
```

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/factor/integration_base.h`（`IntegrationBase::midPointIntegration`）

### 2.5 常见坑

1. **dt 必须来自相邻 IMU 时间戳**：首帧可用标称周期兜底（`1/200`），但不能长期依赖常数周期
2. **预积分完整性**：积分区间不能跨越尚未确认的未来校正时间
3. **bias 更新后必须重传播**：每次后端优化更新 bias 后，要调用 `repropagate()` 用新 bias + 一阶 Jacobian 修正预积分量
4. **速度变量不可省**：预积分约束连接 `pose, velocity, bias`，缺少速度会让惯性约束不可正确表达
5. **重力必须一致**：预测时 $g_w$ 必须与实际重力方向一致，一般在初始化后对齐到世界系

---

## 3. 初始化

> 从单目视觉（无尺度）和 IMU（无全局位置）中恢复尺度、重力方向、偏置和各帧速度。详见 [[方法-VIO初始化方法]]。

### 3.1 初始化问题的本质

待恢复的未知量：

| 未知量 | 符号 | 自由度 | 困难 |
|--------|------|--------|------|
| 尺度因子 | $s$ | 1 | 单目相机无法观测绝对尺度 |
| 重力方向 | $\mathbf{g}$ | 2（模长已知 ~9.81） | 加速度计测量含重力和本体加速度 |
| 陀螺仪偏置 | $\mathbf{b}_g$ | 3 | 旋转预积分误差随时间累积 |
| 加速度计偏置 | $\mathbf{b}_a$ | 3 | 与重力耦合，短时窗口不可观 |
| 各帧速度 | $\mathbf{v}_k$ | $3N$ | 纯视觉不提供速度信息 |

### 3.2 VINS-Fusion 初始化管线（推荐入门方案）

VINS-Fusion 采用 **视觉 SfM → IMU 预积分对齐** 的两阶段管线：

```pseudo
function VIO_Initialize(all_image_frames, imu_data):
    // Phase 0: IMU 激励检查
    accel_var = compute_acceleration_variance(imu_data)
    if accel_var < 0.25:
        warn "IMU excitation not enough — keep accumulating frames"

    // Phase 1: 视觉 SfM（获取无尺度位姿）
    select reference frame l with enough parallax to latest frame
    rel_R, rel_T = compute_5pt_essential_matrix(l, latest)
    sfm.construct(num_frames, poses_Q[], trans_T[], l, rel_R, rel_T, sfm_features)
    // 内部: 三角化 → PnP → 全局 BA (Ceres)

    // Phase 2: PnP 恢复非关键帧位姿
    for each non-kf frame: solvePnP(3D-2D correspondences from SfM)

    // Phase 3: 陀螺偏置估计（先旋转，后平移）
    Bgs = solveGyroscopeBias(all_image_frames)
    // 构造线性系统, min sum || R_bk^bk+1 * J_bg^γ * δbg - γ_corrected ||
    // 用 LDLT 求解，然后 repropagate_all_preintegrations(Bgs)

    // Phase 4: 速度、重力、尺度 闭式解
    g, x = LinearAlignment(all_image_frames)
    // x = [v_0, v_1, ..., v_n, g]  (3*(n+1) + 3 维)
    // min || [H_s H_b] * x - b_s ||^2  (线性最小二乘, LDLT)

    // Phase 5: 切空间重力精细化（4 次迭代）
    RefineGravity(all_image_frames, g, x)
    // 重力在切空间 2-DoF 参数化: g = g_mag * (b1*sinθ*cosφ + b2*sinθ*sinφ + ĝ*cosθ)
    // LDLT 优化 w1, w2, 4 次迭代

    // Phase 6: 恢复到真实尺度 + 重力对齐
    scale = x.tail<1>() / 100.0  // 从 x 中提取尺度
    for each frame:
        P_w = scale * P_sfm - R_latest * T_IC  // 有尺度世界位姿
        V_w = R * speed_vector_from_x          // 世界速度
    R_w0 = g2R(g)  // 重力方向 → 旋转矩阵，对齐世界系
    R_yaw_only = extract_yaw_alignment(R_w0)
    rotate_all_states(R_yaw_only)
    re_triangulate()  // 用有尺度的位姿重新三角化

    return {Ps, Rs, Vs, Bgs, scale, gravity}
```

**代码锚点**：
- `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L209-L217` — 顶层调用
- `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L14-L47` — 陀螺偏置求解
- `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L135-L207` — 线性对齐
- `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L65-L133` — 重力精细化
- `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_sfm.cpp:L128-L323` — 视觉 SfM

### 3.3 ORB-SLAM3 初始化管线（进阶参考）

ORB-SLAM3 在其已有的 ORB 视觉地图上直接做 IMU 初始化（`LocalMapping::InitializeIMU`），通过 g2o 非线性优化联合估计所有量：

```pseudo
function ORB_SLAM3_InitializeIMU():
    require: ≥ 10 KFs, ≥ 2s duration (mono)
    // 粗估计重力方向: sum of -R * Δv across adjacent KFs
    // g2o LM 优化 (200 iterations):
    //   变量: V_i, b_g, b_a, gravity_dir(SO(3)), scale
    //   边: EdgeInertialGS (IMU 预积分残差)
    //   检查 scale < 0.1 → 失败
```

**对比**：VINS-Fusion 分阶段解耦（线性+迭代），代码量小；ORB-SLAM3 直接用 g2o 全量优化，更通用但更重。

### 3.4 初始化关键参数

| 参数 | 建议值 | 说明 |
|------|--------|------|
| 最小帧数 | 30-50 帧 | 确保有足够的视差和运动激励 |
| IMU 激励方差阈值 | 0.25 | VINS 风格：小于此值拒绝初始化 |
| 参考帧选择视差 | 20 px（像素） | 选一个有足够视差的帧作为 SfM 参考 |
| 重力模长 | 9.81 m/s² | 已知先验，减少 1 DoF |
| 切空间迭代 | 4 次 | 重力精细化的轮数 |
| RANSAC 阈值 (SfM) | 归一化平面 2.0 / fx | 本质矩阵内点阈值 |

### 3.5 常见坑

1. **顺序不可颠倒**：必须先解陀螺偏置，后解重力/速度/尺度。陀螺偏置误差通过时间累积放大平移误差
2. **预积分重传播不能忘**：每次偏置更新后立即 repropagate，否则后续求解用旧偏置
3. **加速度计偏置短窗口不可观**：匀速直线运动下 $\mathbf{b}_a$ 与重力耦合。VINS 在线性阶段不优化 $\mathbf{b}_a$
4. **视觉 SfM 质量决定一切**：特征少、视差不足会导致整个初始化失败
5. **重力对齐要单保留 yaw**：对齐后只修正 roll/pitch，保留视觉 SfM 估计的 yaw

---

## 4. 滑动窗口优化

> 将滑动窗口内的状态（位姿、速度、偏置、特征深度）通过 Ceres 构建为非线性最小二乘问题，每次优化后边缘化最老帧。详见 [[方法-滑动窗口边缘化]], [[方法-关键帧选择策略]]。

### 4.1 滑动窗口 vs 全局 BA vs 滤波

| 维度 | 滑动窗口优化 | 全局 BA | 滤波 |
|------|------------|---------|------|
| 计算代价 | 固定上限 O(N_max) | 随时间增长 O(t) | 固定（协方差更新） |
| 精度 | 高（窗口内联合优化） | 最高（全历史） | 中（线性化一次） |
| 线性化一致性 | 一次固定（FEJ） | 持续重线性化 | 一次固定 |
| 代表 | VINS-Fusion | ORB-SLAM3 Global BA | ROVIO, MSCKF |
| 适合 | **实时 VIO（推荐入门）** | 离线/懒加载精化 | 硬实时嵌入式 |

### 4.2 Ceres Problem 构建

滑动窗口优化器需要管理三类因子：

```pseudo
function build_optimization_problem():
    problem = Ceres::Problem()
    options = ceres::Solver::Options()
    options.linear_solver_type = ceres::DENSE_SCHUR  // 滑窗适合稠密 Schur
    options.trust_region_strategy_type = ceres::DOGLEG

    // === 1. 边缘化先验因子（上一轮边缘化的遗留信息）===
    if marginalization_info.valid():
        problem.AddResidualBlock(
            MarginalizationFactor(marginalization_info),
            NULL,  // loss function = NULL (FEJ, 不能加鲁棒核!)
            param_pose[0], param_speed_bias[0], param_pose[1], ...
        )

    // === 2. IMU 预积分因子 ===
    for i in [0, window_size - 2]:
        preint = pre_integrations[i + 1]  // 帧 i → i+1 的预积分
        problem.AddResidualBlock(
            IMUFactor(preint),
            loss_function_imu,
            param_pose[i], param_speed_bias[i],
            param_pose[i+1], param_speed_bias[i+1]
        )

    // === 3. 视觉重投影因子 ===
    for each feature tracked in window:
        for each observation (frame_id, uv) of this feature:
            if frame_id == start_frame:
                problem.AddParameterBlock(param_inv_depth[feature_id], 1)
            problem.AddResidualBlock(
                ProjectionFactor(pts_i_px, pts_j_px, camera_intrinsics),
                loss_function_visual,
                param_pose[frame_id], param_pose[obs_frame_id],
                param_extrinsic, param_inv_depth[feature_id]
            )
```

**Ceres 求解器配置建议**：

```cpp
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_SCHUR;          // 滑窗特征多，Schur 最优
options.trust_region_strategy_type = ceres::DOGLEG;       // 比 LM 更鲁棒于坏初值
options.minimizer_progress_to_stdout = false;
options.max_solver_time_in_seconds = 0.04;                // 40ms 硬时限
options.max_num_iterations = 8;                            // 最多 8 轮
options.num_threads = 4;                                  // 多线程 Jacobian 求值
```

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L820-L1070`

### 4.3 关键帧选择策略

**VINS 风格（推荐）**：基于"质量守卫 + 视差"的简单规则（`raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L52-L119`）：

```pseudo
function should_be_keyframe():
    // 1. 质量守卫（立即触发 KF）
    if frame_count < 2: return true
    if last_track_num < 20: return true       // 跟踪特征不足
    if new_feature > 0.5 * last_track_num: return true  // 场景突变

    // 2. 视差检查（倒数第二帧 vs 倒数第三帧的补偿视差）
    avg_parallax = compute_compensated_parallax(frame[-3], frame[-2])
    return avg_parallax >= MIN_PARALLAX  // 默认 ~10 / 460 ≈ 0.022 归一化
```

**KF 与边缘化联动**（核心机制）：
- **KF** → `MARGIN_OLD`：Schur 补删除最老帧，将其信息保留为先验
- **非 KF** → `MARGIN_SECOND_NEW`：直接丢弃次新帧的视觉观测（保留最老帧维持视差基线），IMU 合并到前一帧

### 4.4 Schur 补边缘化

边缘化是滑窗优化中最复杂的工程实现。核心思想：将待删除变量的信息通过 Schur 补压缩到保留变量上：

$$
\begin{bmatrix} H_{mm} & H_{mr} \\ H_{rm} & H_{rr} \end{bmatrix} \begin{bmatrix} \delta x_m \\ \delta x_r \end{bmatrix} = \begin{bmatrix} b_m \\ b_r \end{bmatrix}
$$

Schur 补后：

$$
(H_{rr} - H_{rm} H_{mm}^{-1} H_{mr}) \cdot \delta x_r = b_r - H_{rm} H_{mm}^{-1} b_m
$$

```pseudo
function marginalize(residual_blocks, keep_vars, drop_vars):
    // 1. 对所有残差块 Evaluate → J_i, r_i
    A = zero_matrix(m+n, m+n); b = zero_vector(m+n)

    // 2. 多线程构建 Hessian A 和梯度 b
    for each residual_block:
        for each param_block p in residual_block.params:
            J_p = jacobian w.r.t p (use localSize for SE(3))
            // A.block(idx_p, idx_q) += J_p^T * J_q
            // b.segment(idx_p) += J_p^T * residual

    // 3. Schur 补 (被边缘化变量在前)
    Amm_inv = regularized_pseudo_inverse(A[0:m, 0:m])
    Amm_inv = 修复半正定性 (特征值 < eps → 0)
    A_schur = A[m:, m:] - A[m:, 0:m] * Amm_inv * A[0:m, m:]
    b_schur = b[m:] - A[m:, 0:m] * Amm_inv * b[0:m]

    // 4. 分解为先验 Jacobian 和残差
    eigen_decompose(A_schur) → V * S * V^T
    linearized_jacobian = sqrt(S) * V^T
    linearized_residual = 1/sqrt(S) * V^T * b_schur

    // 5. 输出 MarginalizationFactor
    return MarginalizationFactor(keep_vars, linearized_jacobian, linearized_residual,
                                  linearization_point)
```

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.cpp:L183-L311`

### 4.5 常见坑

1. **FEJ (First-Estimate Jacobian)**：边缘化先验因子的线性化点一旦固定就不能再变，后续优化中不能重新线性化。否则会违反可观测性约束，引入虚假信息
2. **SE(3) 用 6 维李代数而不是 7 维四元数**：构建 Hessian 时必须用 `localSize()` 映射为 6 维切空间
3. **H_mm 逆的正则化**：信息矩阵可能半正定（秩亏），需要用特征值分解 + 截断来做伪逆
4. **多线程时注意 Hessian 对称性**：`A.block(i,j)` 更新后必须同步更新 `A.block(j,i)`
5. **不要对边缘化先验加鲁棒核**：先验是线性高斯的形式，鲁棒核会破坏一致性

### 4.6 滑动窗口数据结构建议

```cpp
struct VIOState {
    int window_size = 10;  // 滑动窗口大小

    // 优化变量
    std::vector<Pose>    Rs[WINDOW_SIZE];      // 旋转 (四元数)
    std::vector<Vector3> Ps[WINDOW_SIZE];      // 位置
    std::vector<Vector3> Vs[WINDOW_SIZE];      // 速度
    std::vector<Bias>    Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE];  // IMU bias

    // 存储
    std::vector<Preintegration*> pre_integrations;  // 帧间预积分对象
    std::vector<FeatureManager> feature_manager;    // 特征跟踪数据
    MarginalizationInfo* last_marginalization_info; // 上一轮的边缘化先验
};
```

---

## 5. 回环检测

> 检测相机是否回到之前访问过的场景，闭合累积漂移。详见 [[方法-视觉回环检测管线]], [[概念-回环检测方法]]。

### 5.1 DBoW2 词袋模型集成

**推荐方案**：使用 DBoW2 + ORB 描述子（离线预训练词袋，推荐 BRIEF_GIST 100k 词表）。

```pseudo
function init_loop_detection():
    // 1. 加载预训练词袋 (离线生成，推荐 100k 视觉单词)
    vocabulary = DBoW2::Vocabulary("ORBvoc.txt")

    // 2. 创建回环数据库
    database = DBoW2::Database(vocabulary, true, 3)  // true = 内部使用 DI, 3 = 树级

    // 3. 每个关键帧插入时
    for each keyframe:
        ORB_descriptors = extract_orb_keyframe(image)
        bow_vector = vocabulary.transform(ORB_descriptors)
        feature_vector = vocabulary.transform(ORB_descriptors, true)
        entry_id = database.add(bow_vector, feature_vector)
        keyframe.dbow_id = entry_id
```

**代码锚点**：VINS-Fusion 使用 DBoW2 做回环检测，位于 `raw/codes/VINS-Fusion/loop_fusion/src/KeyFrameDatabase.cpp`

### 5.2 三阶段回环管线

```pseudo
function detect_loop(current_keyframe):
    // === 阶段 1: 倒排索引候选（DBoW2）===
    results = database.query(current_kf.bow_vector, top_n=10)
    // 候选过滤:
    //   - 排除时间上太近的 KF (< 30 frames)
    //   - 排除空间上太近的 KF (共视关系)
    //   - 要求共同单词数 >= 0.8 * maxCommonWords
    candidates = filter_by_time_and_geometry(results)
    if candidates.empty(): return false

    // === 阶段 2: 几何验证 ===
    for pKF in candidates:
        // 2a: BoW 描述子匹配 (threshold=0.9)
        matches = match_orb_by_bow(current_kf, pKF, 0.9)
        if matches.size() < 20: continue

        // 2b: PnP RANSAC (双目/IMU 模式) 或 基础矩阵 (纯单目)
        inliers, rel_pose = ransac_pnp(matches, 0.99, 15, 300)
        if inliers.size() < 15: continue

        // 2c: 位姿图验证（共视邻域一致性）
        //  至少 3 个相邻关键帧也确认同一回环区域
        consistent_count = 0
        for neighbor in current_kf.covisibles(10):
            if verify_loop_with(neighbor, candidate, rel_pose):
                consistent_count++
        if consistent_count < 3: continue

        // === 阶段 3: 回环校正 ===
        detected_loop(current_kf, pKF, rel_pose)
        return true

    return false
```

### 5.3 Sim3 与位姿图优化

单目情况下必须用 Sim(3)（含尺度 s），因为长时间单目估计存在尺度漂移。双目/RGB-D/IMU（尺度已可观）可用 SE(3)。

```pseudo
function correct_loop(current_kf, loop_kf, relative_sim3):
    // 1. 停止局部建图线程（避免冲突）
    local_mapper.request_stop()

    // 2. 传播 Sim3 校正到当前 KF 的共视邻居
    for KF_i in current_kf.covisibles():
        T_iw_corrected = T_ic * T_cw_corrected    // 通过相对位姿传播
        KF_i.set_pose(T_iw_corrected)

    // 3. 校正地图点：Pw_corrected = T_wi_corrected * T_iw_original * Pw_original
    for each corrected KF:
        for each map_point:
            Pw_corrected = corrected_Swi.map(original_Siw.map(Pw_original))

    // 4. 融合重复地图点
    for (loop_mp, cur_mp) in matched_mp_pairs: cur_mp.Replace(loop_mp)

    // 5. 位姿图优化（本质图 + 回环边）
    OptimizeEssentialGraph(map, current_kf, loop_kf, LoopConnections)
    //   IMU 模式: 4-DoF 优化 (固定 roll/pitch, 优化 yaw + xyz + scale)

    // 6. 可选: 全局 BA
    if map.kf_count < 200: RunGlobalBundleAdjustment(map)

    local_mapper.release()
```

### 5.4 VINS-Fusion 回环配置参数

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `vocabulary_file` | `support_files/brief_k10L6.bin` | 预训练 BRIEF 词袋 |
| `min_loop_score` | 0.015 | DBoW2 score 最小阈值 |
| `min_inliers_loop` | 20 | 最小内点数 |
| `loop_fusion_time_threshold` | 5.0s | 同一区域距上次回环的时间间隔 |
| `pose_graph_optimize_num` | 50 | 位姿图中累积多少个节点做一次优化 |

### 5.5 常见坑

1. **词袋离线训练**：必须用与运行时相同的描述子类型（BRIEF/ORB），同类型的 100k 词汇文件
2. **Sim3 vs SE3**：单目 VIO 初始化后尺度可观，但长期运行仍可能有尺度漂移。建议用 4-DoF 位姿图（固定 roll/pitch，只优化 yaw + xyz）来处理
3. **时间过滤很重要**：不要把时间上相邻的 KF 当回环
4. **共视邻域一致性**：至少 3 个连续关键帧都确认同一回环区域才触发校正，防止单帧误检
5. **描述子匹配阈值**：BoW 匹配用 0.9（严格），重投影匹配用 5-8 px（宽松）

---

## 6. 完整管线集成

### 6.1 代码模块划分建议

```
vio_system/
├── include/
│   ├── frontend/
│   │   ├── feature_tracker.hpp      # Shi-Tomasi + KLT 前端
│   │   └── feature_manager.hpp      # 特征生命周期
│   ├── imu/
│   │   ├── preintegration.hpp       # 中值预积分
│   │   └── imu_state.hpp            # IMU 状态 (pose, vel, bias)
│   ├── initialization/
│   │   ├── visual_sfm.hpp           # 5-pt 本质矩阵 + Global SfM
│   │   ├── imu_alignment.hpp        # 陀螺偏置 + 线性对齐 + 重力精细化
│   │   └── initializer.hpp          # 初始化顶层调度
│   ├── estimator/
│   │   ├── sliding_window.hpp       # 滑窗管理 + Ceres 构建
│   │   ├── marginalization.hpp      # Schur 补边缘化
│   │   └── imu_factor.hpp           # IMU 预积分因子
│   ├── loop/
│   │   ├── loop_detector.hpp        # DBoW2 回环检测
│   │   └── pose_graph.hpp           # 位姿图优化
│   └── utility/
│       ├── camera_model.hpp         # 相机模型 (pinhole + 去畸变)
│       ├── parameters.hpp           # 全局参数管理
│       └── visualization.hpp        # 可视化输出
├── src/
│   └── ... (对应 .cpp)
├── config/
│   └── euroc_config.yaml            # 按数据集配置
├── support_files/
│   └── brief_k10L6.bin              # DBoW2 词袋
├── CMakeLists.txt
└── README.md
```

### 6.2 配置参数指南

建议用 YAML 管理参数，按场景分类：

```yaml
# ======================== 视觉前端 ========================
feature_tracker:
  max_cnt: 150                   # 最大特征数 (VGA)
  min_dist: 10                   # 角点间最小距离 px
  quality_level: 0.01            # Shi-Tomasi 质量阈值
  forward_backward_threshold: 1.5 # 双向光流一致性 px
  ransac_threshold: 2.0          # 归一化平面 RANSAC (除以 fx)

# ======================== IMU 预积分 ========================
imu:
  acc_n: 0.08                    # 加速度计噪声密度 m/s²/√Hz
  gyr_n: 0.004                   # 陀螺仪噪声密度 rad/s/√Hz
  acc_w: 0.00004                 # 加速度计随机游走 m/s³/√Hz
  gyr_w: 2.0e-6                  # 陀螺仪随机游走 rad/s²/√Hz
  gravity_magnitude: 9.81        # 本地重力模长 m/s²

# ======================== 初始化 ========================
initialization:
  min_features: 30               # 最少特征数
  min_parallax: 20               # 参考帧选择的最小视差 px
  imu_excitation_threshold: 0.25 # IMU 激励方差阈值
  min_init_frames: 30            # 最小初始化帧数

# ======================== 滑动窗口 ========================
sliding_window:
  window_size: 10                # 窗口大小（帧数）
  min_parallax: 10               # KF 触发视差 (归一化 ×460)
  max_solver_time: 0.04          # 最大求解时间 s
  max_iterations: 8              # 最大迭代次数

# ======================== 外参 ========================
extrinsics:
  estimate_td: true              # 是否在线估计时间偏移
  estimate_extrinsic: true       # 是否在线标定外参
  initial_T_IC: [0, 0, 0, 1, 0, 0, 0]  # IMU→Camera 外参初值 (x,y,z,qx,qy,qz,qw)

# ======================== 回环检测 ========================
loop_closure:
  enable: true
  vocabulary_file: support_files/brief_k10L6.bin
  min_loop_score: 0.015
  min_inliers: 20
  pose_graph_criterion: 50
```

### 6.3 调试 Checklist

按从前往后的顺序排查问题：

**前端排查**：
- [ ] KLT 跟踪成功率 > 80%？（过低 → 检查相机曝光、运动速度）
- [ ] per-frame 跟踪特征数 = MAX_CNT 附近？（不是 → 调低 qualityLevel 或增大 min_dist 覆盖）
- [ ] 特征空间分布是否均匀？（否 → 启用 mask 或四叉树分布）
- [ ] forward-backward check 通过率 > 90%？（否 → 增大 fb_threshold 或减少金字塔层数）

**IMU 排查**：
- [ ] IMU 噪声参数是否正确？（查阅传感器 datasheet，100-200Hz 的 IMU 噪声比静止时看到的要大）
- [ ] 预积分 dt 是否来自真实时间戳？（用常数 dt → 积分发散）
- [ ] bias 更新后是否 repropagate？（没做 → 所有后续因子都带旧偏置误差）

**初始化排查**：
- [ ] IMU 激励方差 > 0.25？（不够 → 拿着设备走 8 字形）
- [ ] SfM 后 3D 点数量 > 100？（太少 → 检查视觉前端是否有足够特征）
- [ ] 陀螺偏置收敛到物理合理范围（< 0.1 rad/s）？（太大 → 视觉旋转估计有问题）
- [ ] 尺度估计为正且合理（0.5~3.0 倍的物理尺度）？（负的 → 视觉 SfM 左右手系反了）

**优化排查**：
- [ ] 每轮优化后 cost 是否下降？（不降 → 检查因子残差 Jacobian 符号）
- [ ] 边缘化后约束数不会爆炸？（是的 → 检查 Schur 补是否秩亏）
- [ ] Ceres solver 超时了？（是 → 增大 max_solver_time 或减少特征数）

**回环排查**：
- [ ] 回环 score > 0.015 的帧存在？（没有 → 检查词袋是否用对描述子类型）
- [ ] 回环校正后位姿跳变 < 0.5m？（很大 → 可能是错误回环，收紧一致性检查）

---

## 7. 进阶优化

### 7.1 时间偏移在线估计

相机和 IMU 通常由不同时钟驱动。实际时间关系为 $t_{\text{camera}} = t_{\text{imu}} + t_d$。详见 [[方法-IMU-camera时间偏移估计]]。

VINS-Fusion 将 $t_d$ 作为一个 1 维 Ceres 变量联合优化。核心思想是利用特征点的光流速度，通过线性插值将特征坐标从观测时刻纠正到优化变量 $t_d$ 对应的时刻：

$$
\mathbf{p}_i^{t_d} = \mathbf{p}_i - (t_d - t_{d,i}) \cdot \mathbf{v}_i
$$

其中 $\mathbf{v}_i$ 是归一化平面上的光流速度（像素速度 / 焦距）。

**数值稳定性守卫**：系统静止或低速时固定 $t_d$，因为时间偏移几乎不可观：

```cpp
if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
    problem.SetParameterBlockConstant(para_Td[0]);
```

**代码锚点**：`raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp:L1040-L1043`

**建议参数**：初值置 0，运动展开后在线估计 5-10 秒即可收敛到 ~1ms 精度。

### 7.2 外参在线标定

将相机-IMU 外参 $T_{IC} = [R_{IC} \mid t_{IC}]$ 作为 Ceres 参数块，在滑动窗口优化中与状态变量联合优化。详见 [[方法-多传感器外参在线标定]]。

**建议策略**：
1. 系统启动后前 5-10 秒**固定外参**（用 CAD 或标定值），让初始化完成
2. 初始化成功后**开放平移外参估计**（平移激励需要足够的平移+旋转运动）
3. 系统稳定后**再开放旋转外参估计**（旋转激励需要足够的旋转变化）
4. 外参收敛后（方差 < 阈值）**固定回去**，避免在线漂移

**常见坑**：
- 只做直线运动时外参平移不可观（需要至少 2 个方向的加速度变化）
- 只做匀速旋转时外参旋转不可观（需要角加速度变化）
- 外参与时间偏移耦合——建议先固定 td 标外参，再固定外参标 td，交替 2-3 轮

### 7.3 重力在线精细化

在滑动窗口运行过程中，初始化时估计的重力方向可能有 0.5°-1° 的剩余误差。可以在后端优化中周期性地细化重力方向（滑动窗口中保留一个全局重力方向变量，与位姿联合优化）。

### 7.4 其他进阶方向

| 方向 | 参考资料 | 工作量 |
|------|---------|--------|
| 多相机支持 (双目/立体) | VINS-Fusion, ORB-SLAM3 stereo 模式 | 中 |
| GPS 融合 | VINS-Fusion global_fusion 模块 | 中 |
| LiDAR 紧耦合 (LVI-SAM) | [[算法-LVI-SAM\|LVI-SAM]], [[方法-LVI-SAM互补架构]] | 高 |
| 深度学习特征 | SuperPoint + NetVLAD 替代 ORB + DBoW2 | 高 |
| 事件相机 VIO | ESVO, [[方法-事件相机前端管线]] | 极高 |

---

## 8. 你需要的完整文件清单

### 8.1 核心头文件（.hpp）

| 文件 | 内容 | 参考 |
|------|------|------|
| `feature_tracker.hpp` | KLT 金字塔 + Shi-Tomasi + 双向验证 + mask 补点 | `raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/` |
| `preintegration.hpp` | 中值积分 + 协方差传播 + bias Jacobian + predict | `raw/codes/VINS-Fusion/vins_estimator/src/factor/integration_base.h` |
| `initializer.hpp` | SfM + 陀螺偏置 + 线性对齐 + 重力精细化 | `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.h` |
| `sliding_window.hpp` | Ceres 构建 + KF 选择 + 边缘化调度 | `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.h` |
| `marginalization.hpp` | Schur 补 + 先验因子 | `raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.h` |
| `imu_factor.hpp` | Ceres::CostFunction: IMU 预积分残差 + Jacobian | `raw/codes/VINS-Fusion/vins_estimator/src/factor/imu_factor.h` |
| `projection_factor.hpp` | Ceres::CostFunction: 视觉重投影残差 + td/ex Jacobian | `raw/codes/VINS-Fusion/vins_estimator/src/factor/projection_factor.h` |
| `loop_detector.hpp` | DBoW2 词袋 + 数据库 + 候选搜索 | `raw/codes/ORB_SLAM3/src/KeyFrameDatabase.cc` |

### 8.2 配置文件（YAML）

建议每个数据集一个配置文件，覆盖：
- 相机内参 + 畸变系数
- IMU 噪声参数（查阅传感器 datasheet）
- 相机-IMU 外参初值（CAD 或 Kalibr 标定）
- 滑动窗口参数
- 回环检测阈值

---

## 附录：参考实现速查表

| 模块 | 代表实现 | 文件路径 |
|------|---------|---------|
| 视觉前端 (KLT) | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp` |
| 视觉前端 (KLT) | open_vins | `raw/codes/open_vins/ov_core/src/track/TrackKLT.cpp` |
| 视觉前端 (ORB) | ORB-SLAM3 | `raw/codes/ORB_SLAM3/src/ORBextractor.cc` |
| IMU 预积分 | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/factor/integration_base.h` |
| IMU 预积分 | GTSAM | `gtsam/navigation/PreintegratedImuMeasurements.h` |
| IMU 预积分 | LIO-SAM | `raw/codes/LIO-SAM/src/imuPreintegration.cpp` |
| VIO 初始化 | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp` |
| VIO 初始化 | ORB-SLAM3 | `raw/codes/ORB_SLAM3/src/LocalMapping.cc` (InitializeIMU) |
| VIO 初始化 | open_vins | `raw/codes/open_vins/ov_init/src/dynamic/DynamicInitializer.cpp` |
| 滑动窗口+边缘化 | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp` |
| 边缘化 (Schur 补) | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/factor/marginalization_factor.cpp` |
| 固定滞后平滑 | Kimera-VIO | `raw/codes/Kimera-VIO/src/backend/VioBackEnd.cpp` |
| 回环检测 | ORB-SLAM3 | `raw/codes/ORB_SLAM3/src/LoopClosing.cc` |
| DBoW2 词袋 | DBoW2 | `thirdparty/DBoW2/DBoW2` (在 ORB-SLAM3/VINS-Fusion 等项目中) |
| 位姿图优化 | ORB-SLAM3 | `raw/codes/ORB_SLAM3/src/Optimizer.cc` (OptimizeEssentialGraph) |
| 时间偏移估计 | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/factor/projection_factor.cpp` |
| 外参在线标定 | VINS-Fusion | `raw/codes/VINS-Fusion/vins_estimator/src/estimator/estimator.cpp` (ESTIMATE_EXTRINSIC) |
| 完整系统 | VINS-Fusion | [GitHub: HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) |
| 完整系统 | open_vins | [GitHub: rpng/open_vins](https://github.com/rpng/open_vins) |
| 完整系统 | ORB-SLAM3 | [GitHub: UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) |

---

## 相关页面

- [[VIO方案全景对比]] — 全系统级别的架构、前端、后端、初始化、回环的全面对比
- [[优化后端选型指南]] — Ceres/GTSAM/g2o 的 API 对比和选型建议（本文第 4 章的补充）
- [[方法-视觉特征跟踪]] — 所有视觉前端方法的深入分析（第 1 章详细版）
- [[概念-IMU预积分]] — IMU 预积分的完整数学推导和方法谱（第 2 章详细版）
- [[方法-VIO初始化方法]] — VINS-Fusion/ORB-SLAM3/open_vins 初始化对比（第 3 章详细版）
- [[方法-滑动窗口边缘化]] — Schur 补边缘化 C++ 实现细节（第 4 章详细版）
- [[方法-关键帧选择策略]] — KF 选帧策略的三种范式对比
- [[方法-视觉回环检测管线]] — ORB-SLAM3 三阶段回环管线详解（第 5 章详细版）
- [[方法-IMU-camera时间偏移估计]] — td 在线估计的数学和代码（第 7.1 节详细版）
- [[方法-多传感器外参在线标定]] — 外参在线标定的策略和注意事项（第 7.2 节详细版）
- [[概念-视觉惯性初始化策略]] — 初始化策略的概念级综述
- [[概念-MSCKF]] — 滤波范式的可观测性约束 VIO（与本文的优化范式对比）
- [[概念-Schur补与边缘化]] — 边缘化的线性代数基础
- [[概念-位姿图优化]] — 回环约束如何接入后端位姿图
- [[组件-DBoW2]] — 词袋模型视觉地点识别
- [[组件-Ceres-Solver]] — Ceres 非线性优化库
- [[组件-GTSAM]] — GTSAM 因子图优化库
