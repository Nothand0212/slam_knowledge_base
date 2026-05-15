---
tags: [VIO, 初始化, IMU, 陀螺偏置, ORB-SLAM3, VINS-Fusion]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp
  - raw/codes/ORB_SLAM3/src/Optimizer.cc
---

# IMU 陀螺偏置初始化

> 对比 VINS-Fusion 和 ORB-SLAM3 两种系统中陀螺仪偏置（gyroscope bias）$\mathbf{b}_g$ 的初始化方法。核心思路都是利用 IMU 预积分旋转与视觉估计旋转之间的一致性，但求解方式不同：VINS 用线性最小二乘闭式解，ORB-SLAM3 用 g2o 非线性图优化联合求解。

## 物理问题

给定 N 个连续关键帧，视觉 SfM/ORB-SLAM 已估计出帧间相对旋转 $\mathbf{R}_{ij}^{\text{vision}} = \mathbf{R}_i^{\top} \mathbf{R}_j$。IMU 预积分给出的旋转为：

$$\Delta \tilde{\mathbf{R}}_{ij} = \prod_{k=i}^{j-1} \text{Exp}\left( (\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g) \Delta t \right)$$

若陀螺偏置 $\mathbf{b}_g$ 不是真实值，$\Delta \tilde{\mathbf{R}}_{ij}$ 会与 $\mathbf{R}_{ij}^{\text{vision}}$ 产生系统偏差。陀螺偏置初始化的目标是通过最小化这种偏差来估计 $\mathbf{b}_g$。

一阶线性化形式为：

$$\Delta \tilde{\mathbf{R}}_{ij}(\mathbf{b}_g + \delta \mathbf{b}_g) \approx \Delta \tilde{\mathbf{R}}_{ij}(\mathbf{b}_g) \cdot \text{Exp}\left( \frac{\partial \Delta \mathbf{R}_{ij}}{\partial \mathbf{b}_g} \delta \mathbf{b}_g \right)$$

## VINS-Fusion：SolveGyroscopeBias

线性最小二乘，仅用旋转残差。源码位于 `raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L14–L47`。

### 数学推导

对任意相邻图像帧对 $(i, j)$，定义旋转残差：

$$\mathbf{r}_{ij} = 2 \cdot \text{Vec}\left( \Delta \tilde{\mathbf{q}}_{ij}^{-1} \otimes (\mathbf{q}_i^{-1} \otimes \mathbf{q}_j) \right)$$

其中 $\mathbf{q}_i$ 是视觉估计的第 $i$ 帧旋转四元数，$\Delta \tilde{\mathbf{q}}_{ij}$ 是 IMU 预积分旋转，$\otimes$ 表示四元数乘法。

一阶展开下 Jacobian 为：

$$\mathbf{J}_{ij} = \frac{\partial \mathbf{r}_{ij}}{\partial \delta \mathbf{b}_g} = \mathbf{J}_b^g$$

即 IMU 预积分中旋转关于陀螺偏置的 Jacobian（`pre_integration->jacobian.block<3,3>(O_R, O_BG)`）。

### 伪代码

```pseudo
function SolveGyroscopeBias(all_image_frame, Bgs[]):
    A = 0₃ₓ₃, b = 0₃

    for each adjacent frame pair (i, j):
        q_ij = Rᵢᵀ * Rⱼ                        // 视觉相对旋转
        tmp_A = jacobian.block<3,3>(O_R, O_BG)  // 预积分 J_b^g
        tmp_b = 2 * (Δq_ij⁻¹ ⊗ q_ij).vec()     // 旋转残差

        A += tmp_Aᵀ * tmp_A
        b += tmp_Aᵀ * tmp_b

    δb_g = A.ldlt().solve(b)                    // 更新量 = dst(b)

    for i = 0 to WINDOW_SIZE: Bgs[i] += δb_g

    // 重传播：用新偏置更新所有预积分值
    for each adjacent frame pair (i, j):
        pre_integration.repropagate(Vector3d::Zero(), Bgs[0])
```

### 关键点

- 仅用旋转约束（陀螺偏置与平移预积分理论上正交）。
- 所有相邻帧对的残差被等权累加，形成 $3 \times 3$ 正规方程。
- 求解的 $\delta \mathbf{b}_g$ 是全局增量，因为 VINS 假设滑窗内偏置为常数。
- 求解后立即 `repropagate()` 更新预积分——这是必须的，否则后续 LinearAlignment 会用到带旧偏置的预积分。

## ORB-SLAM3：InertialOptimization 中的陀螺偏置

ORB-SLAM3 将陀螺偏置作为 g2o 图优化中的一个顶点，与速度、加速度计偏置、重力方向、尺度联合优化。源码入口位于 `raw/codes/ORB_SLAM3/src/Optimizer.cc:L3042` 和 `L3227`。

### 图结构

优化图中陀螺偏置相关的顶点和边：

| 顶点类型 | 变量 | 维度 | 是否固定 |
|----------|------|------|----------|
| `VertexPose` | 关键帧 $\text{SE}(3)$ 位姿 | 6 | 固定（来自视觉地图） |
| `VertexVelocity` | 关键帧速度 $\mathbf{v}_i^w$ | 3 | 优化 |
| `VertexGyroBias` | 陀螺偏置 $\mathbf{b}_g$ | 3 | 优化 |
| `VertexAccBias` | 加速度计偏置 $\mathbf{b}_a$ | 3 | 优化（带先验） |
| `VertexGDir` | 重力方向 $\mathbf{R}_{wg}$ | $\text{SO}(3)$ | 优化 |
| `VertexScale` | 单目尺度 $s$ | 1 | 优化（双目固定） |

每条 `EdgeInertialGS` 边连接 **8 个顶点**（`Optimizer.cc:L3160–L3168`）：

$$\text{Edge}(VP1, VV1, VG, VA, VP2, VV2, VGDir, VS)$$

残差为全预积分约束：旋转、位置、速度三个维度的误差同时作用在陀螺偏置、加计偏置、重力、尺度和速度上。

### 数学形式

对于关键帧对 $(i, j)$，IMU 预积分残差（旋转部分）：

$$\mathbf{e}_R = \text{Log}\left( \Delta \mathbf{R}_{ij}^{\top} \cdot \mathbf{R}_{i}^{w^{\top}} \cdot \mathbf{R}_{j}^{w} \right)$$

其中 $\Delta \mathbf{R}_{ij}$ 是关于 $\mathbf{b}_g$ 的函数（通过一阶 Jacobian 线性化）。g2o 的 LM 求解器自动计算 Jacobian 并迭代求解。

### 两种模式

ORB-SLAM3 有两个 InertialOptimization 重载：

1. **全变量模式**（`Optimizer.cc:L3042`）：首次 IMU 初始化，优化 $\mathbf{b}_g, \mathbf{b}_a, \mathbf{v}, \mathbf{g}, s$（5 类变量），200 次 LM 迭代。
2. **仅偏置模式**（`Optimizer.cc:L3227`）：地图已有 IMU 初始化，仅优化 $\mathbf{b}_g, \mathbf{b}_a, \mathbf{v}$，重力和尺度固定，200 次 LM 迭代。

### 偏置更新后的处理

优化后若偏置变化超过阈值（`Optimizer.cc:L3378`，`||bg_new - bg_old|| > 0.01`），触发 `Reintegrate()` 重积分——这与 VINS 的 `repropagate()` 作用相同。

## 两种方法对比

| 特性 | VINS-Fusion | ORB-SLAM3 |
|------|-------------|-----------|
| **求解方法** | 线性 LDLT（闭式解） | g2o LM（非线性迭代，200 次） |
| **约束维度** | 仅旋转 | 旋转 + 位置 + 速度（全预积分） |
| **参数化** | 全局增量 $\delta \mathbf{b}_g$ | 单变量 $\mathbf{b}_g$ |
| **与其他变量耦合** | 解耦（先求 $\mathbf{b}_g$，再求重力/速度/尺度） | 联合优化（$\mathbf{b}_g$ 与 $\mathbf{b}_a, \mathbf{g}, s, \mathbf{v}$ 同时求解） |
| **迭代次数** | 1 次（线性解） | 200 次（非线性优化） |
| **数值稳定性** | 高（3×3 矩阵 LDLT） | 依赖初始值（需 LM 阻尼调整） |
| **偏置先验** | 无 | 有（`EdgePriorGyro`，可配置信息矩阵） |
| **窗口大小** | 滑窗（WINDOW_SIZE，典型 10） | 所有关键帧 |
| **更新方式** | 全局加法（所有帧加相同 $\delta$） | 优化顶点值 |

### 为什么 VINS 的线性方法也能工作

VINS 的 `solveGyroscopeBias` 之所以只用旋转残差就足够，是因为：

1. 陀螺偏置在短时间内主要影响旋转预积分，对平移的影响通过 $\Delta t^2$ 和 $\Delta t^3$ 项才显现，相对较小。
2. 视觉旋转估计（5 点法 + SfM BA）的定向精度通常高于平移精度，因为平移和尺度在单目中是耦合的。
3. $3 \times 3$ 的正规方程条件数好，LDLT 直接求解稳定。

但 VINS 的方法有一个隐含假设：**视觉旋转估计的尺度已正确**。这对单目 SfM 来说通常是满足的（旋转无尺度问题），但要求 SfM 阶段不失败。

## 实现注意事项

1. **Jacobian 索引**：VINS 中 `jacobian.block<3,3>(O_R, O_BG)` 使用的是预积分类内部的 Jacobian 布局。不同预积分实现（VINS vs ORB vs GTSAM）的 Jacobian 排序可能不同，移植代码时需检查。
2. **重传播代价**：偏置更新后的 `repropagate()` / `Reintegrate()` 在窗口大时计算代价高（O(N) 遍历所有 IMU 测量），但对初始化阶段影响不大。
3. **退化检测**：若相邻帧间旋转变化太小（平面运动或纯平移），$A$ 矩阵条件数恶化，$\mathbf{b}_g$ 的某些分量可能不可观。应向用户提示或拒绝初始化。

## Agent 实现提示

### 适用场景

- VIO/SLAM 系统初始化阶段，需要从 IMU 预积分旋转和视觉旋转的差异中估计陀螺仪偏置
- 实现线性最小二乘闭式解（VINS 风格，快速稳定）或 g2o 非线性联合优化（ORB-SLAM3 风格，精度更高）
- 需要理解偏置更新后预积分重传播的必要性和时机

### 输入输出契约

**VINS-Fusion solveGyroscopeBias**
- 输入：`all_image_frame`（含每帧的视觉旋转 `R` 和 IMU 预积分 `pre_integration`）
- 输出：`Bgs[0..WINDOW_SIZE]`（滑窗内各帧陀螺偏置，所有帧加相同增量 `delta_bg`）
- 内部变量：`A`（3×3 正规方程矩阵累加器）、`b`（3×1 残差累加器）、`delta_bg = A.ldlt().solve(b)`
- 副作用：所有 `pre_integration->repropagate(Vector3d::Zero(), Bgs[0])` 重新传播预积分

**ORB-SLAM3 InertialOptimization（陀螺偏置部分）**
- 输入：地图关键帧序列 `vpKF`（含视觉位姿 `Tcw` 和 IMU 预积分）、初始偏置/重力/尺度估计
- 输出：优化后的 `bg`（单变量，所有 KF 共享）、速度 `v`、加计偏置 `ba`、重力方向 `Rwg`、尺度 `s`
- 迭代：g2o LM 200 次，`EdgeInertialGS` 同时约束旋转、位置、速度三维度
- 收敛检查：若 `||bg_new - bg_old|| > 0.01`，触发 `Reintegrate()` 重积分

### 实现骨架（伪代码）

```pseudo
# === VINS 线性最小二乘风格 ===

def SolveGyroscopeBiasLinear(all_frames):
    A = zeros(3, 3)
    b = zeros(3)
    for each (frame_i, frame_j) in adjacent_pairs:
        q_ij_vision = frame_i.R.T * frame_j.R            # 视觉相对旋转
        J_bg = frame_j.pre_integration.jacobian[O_R : O_R+3, O_BG : O_BG+3]  # 偏置 Jacobian (3×3)
        dq_imu = frame_j.pre_integration.delta_q         # IMU 预积分旋转
        residual = 2 * (dq_imu.inverse() * q_ij_vision).vec()  # 旋转残差 (3×1)
        A += J_bg.T * J_bg
        b += J_bg.T * residual

    delta_bg = A.ldlt().solve(b)                         # 全局增量
    for i in range(WINDOW_SIZE + 1):
        Bgs[i] += delta_bg

    for each (frame_i, frame_j) in adjacent_pairs:
        frame_j.pre_integration.repropagate(zeros(3), Bgs[0])  # 重传播，必须

# === ORB-SLAM3 g2o 图优化风格 ===

def SolveGyroscopeBiasGraphOpt(map_kfs):
    # 添加顶点
    for each kf in map_kfs:
        add VertexGyroBias(bg)             # 共享单变量
        add VertexVelocity(v_i)            # 每帧一个
        add VertexAccBias(ba)
    add VertexGDir(Rwg)                    # 重力方向 (SO(3))
    if mono:
        add VertexScale(s)                 # 单目尺度

    # 添加边 (EdgeInertialGS, 8 顶点)
    for each (kf_i, kf_j) in adjacent_pairs:
        edge = EdgeInertialGS()
        edge.setVertices(VP1, VV1, VG, VA, VP2, VV2, VGDir, VS)
        edge.setMeasurement(pre_integration_ij)
        edge.setInformation(IMU 预积分信息矩阵)
        optimizer.addEdge(edge)

    optimizer.optimize(200)                # LM 迭代

    if ||bg_new - bg_old|| > 0.01:
        Reintegrate(bg)                    # 偏置变化超过阈值，重积分
```

### 关键源码片段

`raw/codes/VINS-Fusion/vins_estimator/src/initial/initial_aligment.cpp:L14-L47` — solveGyroscopeBias 完整实现：
```cpp
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A; Vector3d b; Vector3d delta_bg;
    A.setZero(); b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3); tmp_A.setZero();
        VectorXd tmp_b(3); tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}
```

### 实现注意事项

1. **Jacobian 索引依赖实现**：VINS 的 `jacobian.block<3,3>(O_R, O_BG)` 依赖预积分类内部的 Jacobian 布局常量。不同的预积分库（VINS vs ORB-SLAM3 vs GTSAM）的索引定义可能不同，移植时必须检查。VINS 的布局是 15×15 Jacobian，`O_R=0, O_BG=12`。
2. **所有帧加相同增量是假设**：VINS 假设滑窗内陀螺偏置为常数，因此所有帧加相同的 `delta_bg`。如果窗口较大且偏置漂移明显，应考虑分段求解。
3. **重传播不可跳过**：偏置更新后若不 `repropagate()`，后续 `LinearAlignment` 会使用带旧偏置的 IMU 预积分值（位置/速度），导致重力、尺度、速度估计全部偏差。ORB-SLAM3 同理，偏置变化 >0.01 时触发 `Reintegrate()`。
4. **退化检测**：若所有相邻帧间旋转变化都很小（如纯平移、平面运动），正规方程矩阵 `A` 的某些特征值会接近零，对应的偏置分量不可观。应向用户发出警告 `ROS_WARN` 或拒绝初始化。
5. **仅用旋转约束足够**：陀螺偏置主要影响旋转预积分，对平移预积分的影响通过 `Δt²` 和 `Δt³` 项才显现，因此在短窗口中仅用旋转残差即可精确估计 `δbg`。这也是"先旋转后平移"分阶段策略的物理基础。

### 源码检索锚点

| 系统 | 文件 | 函数 | 行号 |
|------|------|------|------|
| VINS-Fusion | `initial_aligment.cpp` | `solveGyroscopeBias()` | L14–L47 |
| VINS-Fusion | `initial_aligment.cpp` | `VisualIMUAlignment()`（入口） | L209–L217 |
| ORB-SLAM3 | `Optimizer.cc` | `InertialOptimization`（全变量，含陀螺偏置） | L3042–L3224 |
| ORB-SLAM3 | `Optimizer.cc` | `InertialOptimization`（仅偏置模式） | L3227–L3387 |
| ORB-SLAM3 | `LocalMapping.cc` | `InitializeIMU()`（调用入口） | L1173–L1319 |
| VINS-Fusion | `estimator.cpp` | `visualInitialAlign()`（顶层调用） | L726–L784 |
| ORB-SLAM3 | `Optimizer.cc` | 偏置变化阈值 + Reintegrate | L3378 |

## 相关页面

- [[方法-VIO初始化方法]]
- [[方法-重力精细化]]
- [[方法-OnlineGravityAlignment]]
- [[概念-视觉惯性初始化策略]]
- [[概念-IMU预积分]]
