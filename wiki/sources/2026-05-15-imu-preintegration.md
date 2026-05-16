---
tags: [IMU, 预积分, 因子图, VIO, 流形优化]
type: source
sources:
  - "https://arxiv.org/abs/1512.02363"
created: 2026-05-15
updated: 2026-05-15
paper_title: "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
paper_authors: "Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza"
paper_venue: "IEEE Transactions on Robotics (TRO)"
paper_year: 2016
paper_arxiv: "1512.02363"
paper_url: "https://arxiv.org/abs/1512.02363"
---

# On-Manifold IMU 预积分（Forster 2016 TRO）

> IMU 预积分理论的奠基之作：在流形上严格推导 IMU 测量的相对运动增量模型，包括噪声传播协方差、偏置一阶修正的闭式雅可比，以及最大后验估计形式，并与因子图无缝对接。

## 核心贡献

1. **流形上的预积分理论**：将两帧之间所有 IMU 测量积分成一个相对运动增量 $\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij}$，该增量独立于初始状态，当优化中关键帧状态被调整时无需重新积分。在 SO(3) 流形上正确处理旋转噪声，而非简单地将旋转视为向量
2. **闭式偏置修正**：预积分量对 IMU bias 的雅可比 $J_{b_a}, J_{b_\omega}$ 在预积分过程中递推累积，bias 变化时通过一阶 Taylor 展开 $ \hat{\Delta} \leftarrow \hat{\Delta} + J_b \cdot \delta b $ 更新预积分值，避免昂贵的重积分
3. **噪声协方差递推**：推导误差状态动力学，在预积分过程中递推传播 9×9（仅预积分量）或 15×15（含 bias）的噪声协方差矩阵 $\Sigma_{ij}$，为后续因子图优化提供正确的信息矩阵
4. **因子图统一框架**：将预积分 IMU 模型表达为因子图中的二元因子（连接帧 $i,j$），可与视觉的 structureless 模型（避免优化 3D 点）无缝结合，使得增量平滑算法（iSAM2）的应用成为可能

## 核心公式

### 预积分定义

从帧 $i$ 到帧 $j$ 的 IMU 预积分量在 $B_i$ 坐标系下表示：

$$
\begin{aligned}
\Delta R_{ij} &\doteq R_i^T R_j = \prod_{k=i}^{j-1} \text{Exp}\left((\tilde{\omega}_k - b_i^g - \eta_k^{gd}) \Delta t\right) \\
\Delta v_{ij} &\doteq R_i^T (v_j - v_i - g \Delta t_{ij}) = \sum_{k=i}^{j-1} \Delta R_{ik} (\tilde{a}_k - b_i^a - \eta_k^{ad}) \Delta t \\
\Delta p_{ij} &\doteq R_i^T \left(p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2}g \Delta t_{ij}^2\right) = \sum_{k=i}^{j-1} \left[\Delta v_{ik} \Delta t + \frac{1}{2} \Delta R_{ik} (\tilde{a}_k - b_i^a) \Delta t^2\right]
\end{aligned}
$$

其中 $\tilde{\omega}_k, \tilde{a}_k$ 为陀螺/加速度计测量值，$b_i^g, b_i^a$ 为时刻 $i$ 的 bias，$\eta_k^{gd}, \eta_k^{ad}$ 为离散化白噪声。

### 预积分残差（用于因子图优化）

将预测值（由预积分 $\Delta \tilde{R}_{ij}, \Delta \tilde{v}_{ij}, \Delta \tilde{p}_{ij}$ 计算）与状态变量之差在切空间表达：

$$
\begin{aligned}
r_{\Delta R_{ij}} &\doteq \text{Log}\left((\Delta \tilde{R}_{ij})^T R_i^T R_j\right) \\
r_{\Delta v_{ij}} &\doteq R_i^T (v_j - v_i - g \Delta t_{ij}) - \Delta \tilde{v}_{ij} \\
r_{\Delta p_{ij}} &\doteq R_i^T (p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2}g \Delta t_{ij}^2) - \Delta \tilde{p}_{ij}
\end{aligned}
$$

残差分布在 SO(3) × ℝ³ × ℝ³ 上，信息矩阵为预积分协方差矩阵的逆 $\Sigma_{ij}^{-1}$。

### 偏置雅可比递推（一阶）

记 $\bar{J}_{k+1}^b = \frac{\partial (\Delta R_{i,k+1}, \Delta v_{i,k+1}, \Delta p_{i,k+1})}{\partial (b_i^g, b_i^a)}$，递推公式为：

$$
\bar{J}_{k+1}^b = A_{k} \bar{J}_k^b + B_k
$$

其中 $A_k, B_k$ 依赖于当前 IMU 测量和累积预积分值，由连续时间误差动力学导出。

### 噪声协方差递推

预积分协方差 $\Sigma_{i,k+1}$ 的递推形式：

$$
\Sigma_{i,k+1} = A_k \Sigma_{i,k} A_k^T + B_k \Sigma_{\eta} B_k^T
$$

$\Sigma_{\eta}$ 为 IMU 连续噪声协方差（对角矩阵，含陀螺 $g_d$ 和加速度计 $a_d$ 噪声）。

### 最大后验估计（MAP）形式

预积分模型下的 MAP 问题为：

$$
\theta^* = \arg\min_{\theta} \sum_{(i,j)\in\mathcal{K}} \|r_{\mathcal{I}_{ij}}\|^2_{\Sigma_{ij}} + \sum_{(i,j)\in\mathcal{C}} \|r_{\mathcal{C}_{ij}}\|^2
$$

其中 $\mathcal{K}$ 为含预积分因子的关键帧对集合，$\mathcal{C}$ 为视觉约束集合（如 structureless vision factor）。

## 工程要点

| 维度 | 方案 |
|------|------|
| 旋转处理 | SO(3) 流形，使用 $\text{Exp}/\text{Log}$ 映射 |
| 噪声模型 | IMU 连续白噪声 + bias 随机游走，协方差异步递推 |
| 偏置修正 | 一阶 Taylor 展开，$J_b$ 在预积分过程中累积 |
| 因子表达 | `ImuFactor`（连接 pose, vel, bias）+ `BetweenFactor`（bias 随机游走） |
| 视觉约束 | structureless model（不估计 3D 点），Schur 消元后注入 |
| 优化后端 | GTSAM iSAM2 增量平滑 |
| 重力方向 | 优化中作为已知常量 $g$，不参与估计（需初始化对齐） |

## 方法定位

该论文是 GTSAM `PreintegratedImuMeasurements` 的理论基础。几乎所有现代 VIO/VISLAM/LIO-SLAM 系统的 IMU 处理都建立在预积分思想之上。LIO-SAM 的 `CombinedImuFactor`、VINS-Fusion 的 Ceres 预积分、ORB-SLAM3 的 IMU 初始化均源于此框架。后续 SE2(3) 预积分（OpenMAVIS）、地球自转补偿预积分（IC-GVINS）均是对此框架的扩展。

## 相关页面

- [[概念-IMU预积分]] — 预积分的概念概览和应用场景
- [[概念-因子图]] — 因子图理论框架
- [[组件-GTSAM]] — GTSAM 中 `ImuFactor` / `PreintegratedImuMeasurements` 的实现
- [[数学-流形优化]] — 流形上的优化理论（SO(3) 上的 Log/Exp 映射）
- [[IMU数据管线]] — 本知识库中所有 IMU 预积分实现对比
- [[架构-滑动窗口优化]] — 预积分在滑动窗口中的角色
- [[概念-视觉惯性初始化策略]] — VIO 初始化中重力对齐与预积分的关系
