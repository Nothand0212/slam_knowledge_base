---
tags: [ICP, 点云配准, LiDAR, SLAM]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/kiss-icp
  - raw/codes/genz-icp
  - raw/codes/ROLO
  - raw/codes/ct_icp
  - raw/codes/FAST-LIO2
---

# ICP 变体族

> 迭代最近点（ICP）及其衍生变体的统一知识页。涵盖从基线点到点 ICP 到连续时间因子图 ICP 的 10 个变体，包括各自残差形式、协方差建模策略、退化处理方法和工程实践。

## 概述

ICP 是 LiDAR SLAM 中最基础的点云配准算法。给定 source 点云 $\mathcal{P} = \{p_i\}$ 和 target 点云 $\mathcal{Q} = \{q_j\}$，ICP 通过交替执行两步迭代求解刚体变换 $T \in SE(3)$：

$$
\begin{aligned}
\text{(1) 对应搜索:}&\quad q_i = \arg\min_{q \in \mathcal{Q}} \|T p_i - q\| \\
\text{(2) 位姿更新:}&\quad T \leftarrow \exp(\hat\xi) \cdot T,\quad \xi = -H^{-1}b
\end{aligned}
$$

核心假设是初值 $T_0$ 已落在正确匹配盆地内。ICP 擅长局部精配准（fine registration），不擅长全局重定位（global localization）。

**为什么有这么多变体？** 标准点到点 ICP 有三个根本弱点：(a) 残差只有欧氏距离，对平面走廊等退化结构约束不足；(b) 最近邻对应不可靠（动态物体、重复结构、遮挡）；(c) 帧内运动畸变破坏刚体假设。变体族从四个维度改进：

| 维度 | 问题 | 代表性变体 |
|------|------|-----------|
| 残差类型 | 点到点约束太弱 | 点到面、GICP、点到分布 |
| 协方差建模 | 忽略局部几何不确定性 | GICP、APDGICP、POINT_TO_DISTRIBUTION |
| 计算加速 | 逐点 KNN 太慢 | VGICP、Nano-GICP、CUDA 并行 |
| 运动建模 | 帧内点不在同一刚体 | 连续时间 ICP 因子、slerp 插值 |

这四个维度的正交组合形成了以下变体族。

### 统一数学框架

所有 ICP 变体可以统一为加权非线性最小二乘问题。定义 source-to-target 残差函数 $e_i(T) = f(T p_i - q_i)$，其中 $f$ 根据变体不同取值：

$$
T^* = \arg\min_{T \in SE(3)} \sum_i w_i \cdot \rho\big(e_i(T)^T \Omega_i \, e_i(T)\big)
$$

其中 $\rho$ 是鲁棒核函数（GM/Huber）、$\Omega_i$ 是信息矩阵（取决于协方差建模策略）、$w_i$ 是自适应权重（如 GenZ-ICP 的 $\alpha$ 或 GM 核的自动降权）。

位姿更新使用 SE(3) 上的 left-perturbation：

$$
T \leftarrow \exp(\delta\xi^\wedge) \cdot T,\quad
\delta\xi = -\big(\sum_i J_i^T \Omega_i' J_i\big)^{-1} \big(\sum_i J_i^T \Omega_i' e_i\big)
$$

Jacobian 通用形式（对 $s = T p$）：

$$
J_i = \frac{\partial e_i}{\partial \delta\xi}\Big|_{\delta\xi=0} =
\begin{cases}
[n^T,\; (p \times n)^T] & \text{点到面 (1×6)} \\
[I,\; -[s]_\times] & \text{点到点 (3×6)} \\
M^{1/2} \cdot [I,\; -[s]_\times] & \text{GICP 马氏加权 (3×6)}
\end{cases}
$$

### Hessian 退化与可观测性

配准质量不仅看残差下降，还要检查 Hessian 矩阵 $H$ 的条件数。当 $\text{cond}(H) > 10^6$ 时，某些方向（如平面走廊中的切向分量）严重欠约束。工程对策包括：(a) 注入 IMU 先验；(b) 使用退化检测（条件数 + 最小特征值阈值）后仅信任可观方向；(c) 切换到更丰富的残差类型（点到点 → GICP → 连续时间 ICP）。

## 变体对比表

| 变体 | 残差类型 | 协方差建模 | 速度 | 退化处理 | 代表框架 |
|------|----------|-----------|------|---------|---------|
| 点到点 ICP | 3D 欧氏距离 | 无（或单位阵） | 慢（需多迭代） | GM 核降权 | KISS-ICP |
| 点到面 ICP | 1D 法向投影 | 隐含在法向中 | 快（30 iter） | 面退化需切换 | FAST-LIO, LeGO-LOAM |
| GICP | 马氏距离 | 邻域协方差矩阵 | 中（KNN 开销） | 协方差正则化 | FAST-LIO-SAM, RT-KLIO |
| POINT_TO_DISTRIBUTION | 马氏距离 | 邻域协方差 + εI | 中 | 正则项 ε=0.05 | CT-ICP |
| GenZ-ICP | 自适应平面/点 | 平面度自适应权重 | 中 | 自动阈值 + 权重 α | genz-icp, hdl_localization |
| Fast-VGICP | 马氏距离 | 体素高斯分布 | 快（14 线程） | 体素落格校验 | BEV-LSLAM |
| Nano-GICP | 马氏距离 | 邻域协方差 | 快（NanoFLANN） | fitness score 阈值 | FAST-LIO-SAM-SC-QN |
| RotVGICP | 马氏距离 | VmfVoxelMap 体素 | 中（多线程） | SO(3) 流形分离 | ROLO-SLAM |
| APDGICP | 马氏距离 | 传感器各向异性 | 中 | fitness score 自适应权重 | 4DRadarSLAM |
| 连续时间 ICP | 点到面 | 无（或继承 GICP） | 慢（slerp 开销） | 两关键帧约束 | gtsam_points |

## 各变体详解

### 1. 点到点 ICP（Point-to-Point）

最基础的 ICP 形式。每个 source 点 $s$ 到 target 最近点 $t$ 构成 3 维残差（使用 left-perturbation on SE(3)）：

$$
r = s - t \in \mathbb{R}^3,\qquad J = \begin{bmatrix} I_3 & -[s]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6}
$$

每对对应关系贡献 3 个方程，信息量是点到面的 3 倍，但缺少结构先验。收敛需要 500 次迭代（vs 点到面的 30 次），KISS-ICP 的洞察是：**GM 核 + 大量迭代的组合等价于自动特征选择器**——大残差的 outlier 被核函数压制，平面点因残差小自然获得高权重。

KISS-ICP 证明了点到点 ICP 的一个关键洞察：**GM 核函数 `w = σ² / (σ² + r²)` 配合 500 次迭代，等价于自动特征选择器**。大残差的离群值被核函数压制到权重接近零，平面点因残差小自然获得高权重（`w → 1`）。这使得点到点 ICP 虽然理论约束弱，但在实践中通过迭代 + 核函数自动找出了最可靠的点对应。

实现参考 `raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L60-L78` 的 TBB 并行数据关联和 `L96-L121` 的加权线性系统构建。

### 2. 点到面 ICP（Point-to-Plane）

将残差投影到 target 点的局部法向 $n$ 上，降为 1 维：

$$
r = n^T(Tp - q) \in \mathbb{R},\qquad J = \begin{bmatrix} n^T & (p \times n)^T \end{bmatrix} \in \mathbb{R}^{1 \times 6}
$$

结构化环境（室内走廊、城市道路）收敛极快，是 LIO 前端的标准选择。但仅约束法向方向的 1 个 DOF，切向方向完全自由，在平面走廊场景中 Hessian 退化严重——这正是 KISS-ICP 回归点到点 ICP 的主要原因之一。

**工程注意**：法向 estimation 质量直接影响收敛，半径选择（约 0.2-0.5m）需与传感器噪声匹配。与 IMU 预积分结合可补充退化方向约束。

### 3. GICP（广义 ICP）

将点云配准建模为两个高斯分布点集的概率匹配问题。设 source 点协方差为 $C_s$，target 协方差为 $C_t$，联合协方差：

$$
\Sigma_i = C_t + R\, C_s\, R^T
$$

最小化马氏距离：

$$
e_i = r_i^T \Sigma_i^{-1} r_i,\quad r_i = Tp_i - q_i
$$

当 $\Sigma_i = I$ 时退化为点到点 ICP；当 target 协方差在法向方向极小（平面先验）时退化为点到面 ICP。GICP 通过局部点分布估计协方差，在平面区域自动获得点到面加速，在曲面/角点保持面到面精度。

关键源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L162-L190` 展示了马氏距离缓存的完整流程，`L242-L280` 展示了 Jacobian 传播：

$$
J_{\text{target}} = \begin{bmatrix} -[Tp]_\times & I_3 \end{bmatrix},\quad
J_{\text{source}} = \begin{bmatrix} R[p]_\times & -R \end{bmatrix}
$$

**工程注意**：协方差必须正则化再求逆；近邻数过少不稳定；误差单位依赖协方差尺度，不能与普通 ICP fitness 混用阈值。

### 4. POINT_TO_DISTRIBUTION（点到分布 ICP）

CT-ICP 四种残差类型之一，把点到局部分布的误差写成带正则化的马氏距离：

$$
\Lambda = (C + \varepsilon I)^{-1},\quad \text{cost} = w \cdot (p - \mu)^T \Lambda (p - \mu)
$$

其中 $\varepsilon = 0.05$ 防止小邻域退化导致的协方差奇异。与 POINT_TO_PLANE 的区别：保留完整协方差信息而非仅用法向方向；与 POINT_TO_POINT 的区别：沿局部高不确定方向自动降低惩罚。

在平面区域，协方差体现"沿平面方向不确定、法向方向确定"的结构，行为接近点到面；在角点或非结构化区域，完整协方差比单一法向更稳健。**常被用于 GICP 轻量化实现中，因为邻域协方差的计算量远小于 GICP 的逐点协方差传播。**

### 5. GenZ-ICP（自适应 GICP）

核心创新是**平面度自适应权重**机制。算法分析每个点的局部邻域平面度（planarity），将点分为两类分别处理：

- **平面点**：使用点到面约束，赋予高权重 `α`（默认 0.9）
- **非平面点**：使用点到点约束，赋予低权重 `1-α`

线性系统为两类点的加权叠加，源码 `raw/codes/genz-icp/cpp/genz_icp/core/Registration.cpp:L67-L140`：

```cpp
// 平面点：1D 残差 × 高权重
JTJ += α · J_planar^T · w_planar · J_planar
// 非平面点：3D 残差 × 低权重
JTJ += (1-α) · J_non_planar^T · w_non_planar · J_non_planar
```

平面度判定阈值**自动估计**，无需手动调节。配准完成后使用 $H^{-1}$ 作为位姿协方差，可直接用作因子图约束的信息矩阵。GenZ-ICP 在结构化（走廊、室内）和非结构化（开阔地、树林）环境均能产生可靠约束，是 `genz-icp` 和 `hdl_localization` 的核心配准模块。

### 6. Fast-VGICP（体素化 GICP）

把 target 点云划分成固定分辨率（通常 0.5m）的体素，每个体素维护均值和高斯分布，用体素索引查询替代标准 GICP 的逐点 KNN 搜索：

$$
\text{target\_voxel} = \text{GaussianVoxelMap.lookup}(\text{coord}(Tp_i))
$$

联合协方差和残差与 GICP 完全相同：$\Sigma_i = C_{\text{voxel}} + R\, C_s\, R^T$，$r_i = \mu_{\text{voxel}} - Tp_i$。

CUDA 实现见 `raw/codes/gtsam_points/include/gtsam_points/cuda/kernels/vgicp_derivatives.cuh:L39-L72`，每对对应关系输出 6×6 Hessian 块 `H_target`、`H_source` 和互协方差 `H_target_source`，适合大规模因子图融合。

**工程注意**：体素分辨率过大时混合不同平面成单个高斯；过小时有效对应不足；可选扩展到 7/27 邻域增强鲁棒性。用于 BEV-LSLAM 中 scan-to-map 后端精化，25 次迭代，14 线程并行。身高和强度点云融合时需注意尺度差异。

### 7. Nano-GICP（轻量 GICP）

结合 **NanoFLANN KD-tree**（近邻搜索）和 **FastGICP**（分布到分布优化）的轻量快速实现。参数为 `max_iter=32`、`icp_score_threshold=1.5`、`correspondences_number=15`，在 FAST-LIO-SAM-SC-QN 中作为四阶段回环验证的精配准模块。

四阶段回环验证流程：ScanContext 粗检测 → 位姿一致性验证 → Quatro 粗对齐 → **Nano-GICP 精配准**。Nano-GICP 在最后一阶段负责：根据 Quatro 给出的粗变换进一步精细化对齐，计算 fitness score，最终判断回环是否成立。

与粗对齐方法 Quatro 互补：Quatro 处理大初始误差和外点（全局搜索盆地），Nano-GICP 在较好初值附近优化局部几何一致性。这种"粗对齐 + 精配准"组合比单次 ICP 验证回环更稳健，因为两步承担不同的失败模式。

### 8. RotVGICP（旋转优先 VGICP）

ROLO-SLAM 的旋转优先变体，关键特征：在 SO(3) 流形上直接做旋转高斯-牛顿优化（`so3_linearize()`），避免四元数/Euler 角奇异性。使用 **VmfVoxelMap**（Voxelized Mean-Free Gaussian Voxel Map）预计算目标分布，协方差通过 `calculate_covariances()` 预计算。

与 Fast-GICP 的核心区别：RotVGICP 分离旋转和平移估计——先优化旋转 $R$（在 SO(3) 流形上），再固定 $R$ 优化平移 $t$，而非全 6-DOF 同时优化。这种解耦策略对越野地形中的退化场景（如平坦地面导致 z/roll/pitch 不可观）更稳健，因为旋转自由度获得独立收敛通道。迭代完成后，使用 $H^{-1}$ 输出位姿协方差，作为后续因子图约束的信息矩阵。实现位于 `rot_vgicp.hpp:24-130`，支持 OpenMP 多线程。

### 9. APDGICP（自适应概率分布 GICP）

4DRadarSLAM 针对 4D 毫米波雷达设计的变体，将传感器物理噪声模型直接编码进 GICP 协方差：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `dist_var` | 0.86 | 100m 处距离测量不确定度 |
| `azimuth_var` | 0.5° | 方位角精度 |
| `elevation_var` | 1.0° | 俯仰角精度 |

每个雷达点的概率分布是沿距离方向拉长、角度方向压缩的**严重各向异性椭球**。这与标准 GICP 的"各向同性/近各向同性"假设有本质差异：标准 GICP 用相邻点统计协方差（几何驱动），APDGICP 用传感器规格推导协方差（物理驱动）。配准时还配合 fitness score 自适应调整信息矩阵权重。

**核心价值**：对低精度或强各向异性传感器，显式建模测量不确定性通常比套用标准 ICP 更可靠。这种"传感器物理模型驱动配准"范式也适用于其他非 LiDAR 传感器（如 RGB-D、热成像）。

### 10. 连续时间 ICP 因子

gtsam_points 中把扫描起止两个位姿 $T_{t_0}$、$T_{t_1}$ 同时作为因子的 ICP 变体，使点云配准在因子图内直接处理帧内运动畸变。

关键特征：不继承 `IntegratedMatchingCostFactor`（需要两个 key）；维护 `time_table`（每点归一化时间戳 $\in [0,1]$，从实际扫描时间戳除以最大时间戳得到）和 `time_indices`（每点的预计算时间索引）。`time_table` 构建时合并重复时间戳（阈值 `1e-3`）以减少 slerp 计算量。

源码 `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L140-L179` 展示了 `linearize()` 流程：`update_poses(values)` 计算每时刻的 slerp 插值位姿和 `pose_derivatives_t0/t1` → `update_correspondences()` 使用插值位姿做最近邻对应 → 逐点计算雅可比并传播到两个关键帧 key：

$$
\begin{aligned}
H_{00} = \sum_i J_{0,i}^T J_{0,i},\quad J_{0,i} &= J_{\text{error}} \cdot J_{\text{pose}} \cdot J_{t_0} \\
H_{11} = \sum_i J_{1,i}^T J_{1,i},\quad J_{1,i} &= J_{\text{error}} \cdot J_{\text{pose}} \cdot J_{t_1} \\
H_{01} = \sum_i J_{0,i}^T J_{1,i},\quad b_k &= -\sum_i J_{k,i}^T \cdot e_i
\end{aligned}
$$

相比匀速 deskew 更精确（不需要假设恒定速度），但计算量更大——每点都需要 slerp 并计算两个雅可比链式传播。存在 CT-GICP 变体，额外使用 D2D（分布到分布）代价并需要目标点协方差和马氏距离缓存。对于旋转为主的运动，slerp 精度优势明显；对于平移为主的运动，匀速 deskew 通常足够。

### 对应策略补充

各变体在对应（correspondence）策略上的差异与残差类型同样重要：

| 策略 | 代表变体 | 查询数据结构 | 适用场景 |
|------|---------|-------------|---------|
| 逐点最近邻 KNN | 点到点、点到面、GICP | KD-tree (Flann/NanoFLANN) | 中小规模 scan-to-scan |
| 体素格查询 | VGICP、RotVGICP | GaussianVoxelMap | 大规模 scan-to-map |
| 体素哈希 Nearest | Fast 变体 | VoxelHashMap | 实时里程计 |
| 多尺度 KNN + 鲁棒统计 | GenZ-ICP | KD-tree + 自动平面度 | 混合结构 |

对应的质量直接决定配准上限：错误对应（outlier correspondence）即使 GICP 的马氏距离也无法纠正，因为错误的数据关联会将残差拉到错误盆地。因此**距离门限先裁剪，鲁棒核再压制**是各变体的共同工程实践。

## 选型指南

### 按场景推荐

| 场景 | 推荐变体 | 理由 |
|------|---------|------|
| 室内/城市平面结构，LiDAR 前端 | 点到面 ICP 或 GenZ-ICP | 收敛快，平面主导 |
| 非结构化户外（树林、开阔地） | GICP 或 GenZ-ICP | 协方差建模保全面信息 |
| 大规模 scan-to-map，地图存储为主 | Fast-VGICP | 体素化替代 KNN，内存可控 |
| 回环验证（有粗对齐后） | Nano-GICP 或 GICP | 精配准 + fitness 检查 |
| 4D 雷达 / 低精度传感器 | APDGICP | 传感器物理模型驱动 |
| 越野/崎岖地形，退化风险高 | RotVGICP | SO(3) 流形解耦，旋转优先 |
| 高速运动 / 帧内畸变严重 | 连续时间 ICP 因子 | 内建 slerp 去除畸变 |
| 实现极简 / 任何结构都需工作 | 点到点 ICP + GM 核 | 零依赖，无退化偏好 |
| 通用且需协方差输出 | GenZ-ICP | 自适应 + H⁻¹ 协方差 |

### 选型决策树

```text
需要协方差输出？
├── 是 → 传感器有各向异性噪声? → APDGICP
│          └── 否 → 需要自适应混合? → GenZ-ICP → GICP
└── 否 → 帧内畸变严重? → 连续时间 ICP 因子
         └── 否 → target 是大规模地图? → Fast-VGICP
                  └── 否 → 有可靠平面结构? → 点到面 ICP
                           └── 否 → 点到点 ICP + GM 核
```

### 常见工程陷阱速查

| 陷阱 | 表现 | 对策 |
|------|------|------|
| 初值偏差 > 收敛半径 | 残差不降甚至增大 | 前端提供更好初值 / 多分辨率粗配准 |
| 体素分辨率为传感器 10× | 有效对应不足 30% | 降至 2-3× 传感器噪声水平 |
| 协方差未正则化就求逆 | 位姿跳变或 NaN | GICP 加 `εI`；POINT_TO_DISTRIBUTION 用 `0.05I` |
| Hessian 退化时不报告 | 里程计在退化方向漂移 | 检查 `cond(H)` 和最小特征值 |
| `dx` 收敛但 fitness 差 | 误将错误对应当正确配准 | 同时检查内点比例和均方残差 |
| 回环仅用 ICP 验证 | 误闭环率 > 5% | 粗对齐 + ICP 精配准两步验证 |

## Agent 实现提示

### 适用场景

实现局部 scan-to-map 或 scan-to-scan 精配准：源点云与目标地图之间的 SE(3) 增量优化。适用于里程计前端、回环几何验证和粗配准后的精化。不适合全局重定位 / 位姿初始化（ICP 假设初值在收敛盆地内）。需要外部提供可靠初值、KD-tree/体素查询结构、鲁棒核尺度和对应阈值。

### 输入输出契约

- **输入**：source 点云（含可选协方差/法向/时间戳）、target 点云或体素地图、初始位姿 $T_{\text{init}}$、最大对应距离、鲁棒核尺度、最大迭代次数、收敛阈值、协方差缓存模式
- **输出**：优化后位姿 $T_{\text{source}}^{\text{target}}$，诊断量：有效对应数、均方残差、最终位姿增量范数、Hessian 条件数、内点比例
- **前置条件**：source 已去畸变并降采样；target 查询结构已构建；初值误差在收敛盆地内

### 实现骨架（伪代码）

```text
T ← T_init
for iter in 1..max_iter:
    -- 步骤 1: 对应搜索（KD-tree 或体素查询）
    correspondences ← []
    for each source point s:
        S ← T · s
        (q, dist) ← target_lookup(S)
        if dist < max_corr_dist:
            correspondences.append((s, q))

    -- 步骤 2: 构建线性系统（按变体选择残差类型）
    H, b ← 0
    for each (s, q) in correspondences:
        if variant == POINT_TO_PLANE:
            r ← n_q^T (T·s - q)              -- 1D 标量残差
            J ← [n_q^T, (s × n_q)^T]         -- 1×6
        elif variant == GICP:
            r ← T·s - q                       -- 3D 向量残差
            Σ ← cov_q + R·cov_s·R^T
            M ← Σ^(-1)
            J ← compute_se3_jacobian(T·s, R) -- 3×6
            H += J^T M J
            b -= J^T M r
            continue
        elif variant == POINT_TO_POINT:
            r ← T·s - q                       -- 3D 向量残差
            J ← [I, -[T·s]_×]                -- 3×6
        w ← robust_kernel(r)                  -- GM / Huber
        H += w · J^T J
        b -= w · J^T r

    -- 步骤 3: 求解与更新
    dx ← solve(H, -b)
    T ← exp(dx) · T
    if |dx| < convergence_threshold:
        break

return T
```

### 关键源码片段

**点到点 ICP + GM 核** — `raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L96-L121`

```cpp
auto GM_weight = [&](const double &residual2) {
    return square(kernel_scale) / square(kernel_scale + residual2);
};
// TBB parallel_reduce: 3×6 Jacobian × GM weight
// H += J_r^T · w · J_r,  b -= J_r^T · w · residual
```

**GICP 马氏距离缓存** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L162-L190`

```cpp
const Eigen::Matrix4d RCR = (target_cov + delta.matrix() *
    frame::cov(*source, i) * delta.matrix().transpose());
mahalanobis_full[i].topLeftCorner<3, 3>() =
    RCR.topLeftCorner<3, 3>().inverse();
```

**GICP Jacobian 传播** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:L242-L280`

```cpp
J_target.block<3,3>(0,0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
J_target.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
J_source.block<3,3>(0,0) = delta.linear() *
    gtsam::SO3::Hat(mean_A.head<3>());
J_source.block<3,3>(0,3) = -delta.linear();
```

**VGICP 体素查询** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:L114-L146`

```cpp
Eigen::Vector3i coord = target_voxels->voxel_coord(pt);
const auto voxel_id = target_voxels->lookup_voxel_index(coord);
if (voxel_id < 0) { /* skip */ }
const auto voxel = &target_voxels->lookup_voxel(voxel_id);
// 联合协方差: voxel->cov + R · cov_source · R^T
```

**GenZ-ICP 自适应权重** — `raw/codes/genz-icp/cpp/genz_icp/core/Registration.cpp:L67-L140`

```cpp
// 平面点: point-to-plane, 高权重 α
J_planar = [normals[i]^T | (src_planar[i] × normals[i])^T]
JTJ += α · J_planar^T · w_planar · J_planar
// 非平面点: point-to-point, 低权重 1-α
J_non_planar = [I | -[src_non_planar[i]]×]
JTJ += (1-α) · J_non_planar^T · w_non_planar · J_non_planar
```

**CT-ICP slerp + Jacobian 链式传播** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:L140-L179`

```cpp
const int time_index = time_indices[i];
const auto& pose = source_poses[time_index];          // slerp 插值位姿
const auto& H_pose_0 = pose_derivatives_t0[time_index];
const auto& H_pose_1 = pose_derivatives_t1[time_index];
// 链式传播: J_error → J_pose → J_t0/J_t1
H_0 = H_error_pose * H_pose_0;  // 6×1
H_00 += H_0^T * H_0;            // 6×6
```

### 实现注意事项

- **对应搜索和线性系统构建要分离**，方便替换 KD-tree、体素哈希或 GPU 查询后端
- **先用距离门限裁剪离群对应，再用鲁棒核压低大残差**；两者用途不同，不可互相替代
- `dx` 很小时不代表配准正确——还要检查有效对应数量、均方残差和 Hessian 条件数
- **协方差矩阵求逆前必须正则化**（GICP/VGICP/APDGICP），否则平面或线结构产生病态 $\Sigma$
- 最近邻更新可按位姿变化阈值延迟，但**缓存的马氏矩阵必须与线性化点一致**
- 多线程累加 Hessian 时使用线程局部块或安全 reduction，**避免非确定性写冲突**
- 体素分辨率选择需与传感器噪声和场景几何匹配：过大混平面，过小缺对应
- GICP 误差单位依赖协方差尺度，**不能与普通 ICP fitness 混用阈值**

### 源码检索锚点

- `DataAssociation` / `BuildLinearSystem` / `Registration::AlignPointsToMap` — 通用 ICP
- `IntegratedGICPFactor_` / `FusedCovCacheMode` / `mahalanobis_full` — GICP
- `IntegratedVGICPFactor_` / `target_voxels->lookup_voxel_index` — VGICP
- `IntegratedCT_ICPFactor_` / `time_indices` / `pose_derivatives_t0` — 连续时间 ICP
- `BuildLinearSystem` (genz-icp) / `src_planar` / `compute_jacobian_and_residual_planar` — GenZ-ICP
- `vgicp_derivatives_kernel` / `LinearizedSystem6` — VGICP CUDA

## 相关页面

- [[方法-点到点ICP]], [[方法-GICP配准方法]], [[方法-Fast-VGICP]], [[方法-APDGICP 自适应概率分布 GICP]]
- [[组件-Nano-GICP]], [[方法-genz-icp]], [[方法-RotVGICP]], [[方法-POINT_TO_DISTRIBUTION]], [[方法-连续时间 ICP 因子]]
- [[方法-ICP配准方法]] — 原通用 ICP 页
- [[算法-KISS-ICP]], [[算法-CT-ICP]], [[算法-FAST-LIO]], [[算法-ROLO-SLAM]]
- [[算法-4DRadarSLAM]], [[算法-BEV-LSLAM]], [[算法-LeGO-LOAM]]
- [[方法-Geman-McClure鲁棒核]], [[方法-Quatro配准]], [[方法-GaussianVoxelMap 体素化配准]]
- [[概念-因子图]], [[概念-回环检测方法]], [[概念-连续时间轨迹]]
- [[传感器-Doppler 自速度估计]], [[方法-退化检测与修复]], [[方法-体素地图]]
- [[数学-流形优化]], [[组件-gtsam_points]]
