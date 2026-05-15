---
tags: [直接法, 光度误差, SLAM, DSO, SVO, VIO, 稀疏直接法, 半直接法]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-dso-analysis.md
  - wiki/sources/2026-04-28-rovio-analysis.md
  - wiki/sources/2026-04-29-camera-pipeline-comparison.md
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
  - wiki/sources/2026-04-29-orb-slam3-analysis.md
  - wiki/sources/2026-04-29-paper-notes.md
  - wiki/sources/2026-04-30-image-preprocessing-comparison.md
supersedes:
  - 概念-直接法光度误差
  - 概念-直接法视觉里程计
  - 概念-直接法vs间接法
---

# 直接法 SLAM 族

> 直接法（Direct Methods）不使用特征提取和描述子匹配，而是直接比较图像像素灰度值构造光度误差（photometric error），并在此基础上最小化位姿和场景结构。本章统一收录稀疏直接法、半直接法、滤波直接法和混合方案的核心共性与差异。

## 概述

间接法（Indirect / Feature-based）的前端先提取关键点、计算描述子、做显式匹配，后端最小化几何重投影误差。直接法则跳过特征提取步骤，在图像梯度较好的像素或 patch 上直接定义光度残差：

$$
r = I_{\text{ref}}(u) - I_{\text{cur}}\big(\pi(T_{\text{cur},\text{ref}} \cdot \pi^{-1}(u, d))\big)
$$

其中 $T_{\text{cur},\text{ref}} \in SE(3)$ 是参考帧到当前帧的刚体变换，$\pi$ / $\pi^{-1}$ 为相机投影与反投影。完整的光度误差模型还包含曝光/辐射项：

$$
r = \tau_{\text{cur}} \cdot I_{\text{cur}}\big(\pi(T_{\text{cw}} \cdot p_w)\big) \;-\; \tau_{\text{ref}} \cdot I_{\text{ref}}(\text{patch})
$$

其中 $\tau$ 为曝光时间或逆曝光因子，用于吸收自动曝光、增益或亮度变化带来的系统性偏差。

**为什么需要一族方法？** 直接法的核心假设（亮度恒定）在真实场景中经常失效，不同系统从四个维度缓解这一问题：

| 维度 | 问题 | 代表性方案 |
|------|------|-----------|
| 光度建模 | 亮度变化破坏一致性假设 | 曝光建模 (DSO/DM-VIO/FAST-LIVO2)、仿射亮度 (SVO/ROVIO) |
| 鲁棒估计 | 运动模糊、遮挡、动态物体 | Huber/GM 核降权、patch NCC 筛选、马氏门限 |
| 初始化 | 单目无深度且初值误差大 | 粗到精金字塔、IMU/LiDAR 先验主导初始化 |
| 梯度选择 | 平坦区像素提供零约束 | 高梯度阈值、网格化每格保留一个最优候选 |

这四个维度的正交组合形成了以下变体族。

### 统一残差框架

所有直接法变体可统一为加权非线性最小二乘问题。定义 patch-wise 或 pixel-wise 的光度残差函数 $e_i(T, d_i, \tau)$：

$$
T^*, d^*, \tau^* = \arg\min \sum_i w_i \cdot \rho\big(e_i(T, d_i, \tau)^2\big)
$$

其中 $\rho$ 为鲁棒核函数（Huber / Cauchy / GM），$w_i$ 为梯度权重（抑制低纹理区域），$d_i$ 为共视点逆深度。位姿更新使用 SE(3) 上的 left-perturbation：

$$
T \leftarrow \exp(\delta\xi^\wedge) \cdot T,\quad
\delta\xi = -\Big(\sum_i J_i^T \, w_i' \, J_i\Big)^{-1} \Big(\sum_i J_i^T \, w_i' \, e_i\Big)
$$

光度雅可比通过链式法则传播至位姿、深度、曝光参数和仿射亮度参数，详细形式见下文"链式雅可比"节。

## 对比表

### 直接法 vs 间接法

| 维度 | 间接法 (Feature-based) | 直接法 (Direct) |
|------|----------------------|-----------------|
| 误差函数 | 几何重投影误差 $\|u - \pi(TX)\|$ | 光度误差 $I_{\text{ref}} - I_{\text{cur}}(\pi(TX))$ |
| 前端 | 特征提取 + 描述子匹配 | 像素/patch 灰度比较 |
| 数据关联 | 显式特征匹配 | 隐式光度一致性 |
| 描述子 | ORB / SIFT / SuperPoint | 无（或图像梯度方向直方图） |
| 光度标定 | 不需要 | 推荐（response / vignette / exposure） |
| 对光照鲁棒性 | 强，描述子具有光照不变性 | 弱，依赖光度标定与曝光建模 |
| 对大基线鲁棒性 | 强，描述子抗视角变化 | 弱，灰度一致性假设在大基线失效 |
| 对弱纹理鲁棒性 | 弱，特征提取失败 | 强，可利用所有梯度像素 |
| 密度 | 稀疏（典型 200-2000 点） | 稀疏/半稠密/稠密可选 |
| 速度 | 中（特征提取 + 描述子计算） | 快（跳过特征步骤，但金字塔采样有开销） |
| 初始化 | 可通过对极几何 + 三角化 | 需要较好初值或先验（IMU/LiDAR） |
| 回环检测 | 强，描述子 + DBoW 成熟 | 弱，通常依赖几何验证或独立回环模块 |
| 代表框架 | ORB-SLAM3, VINS-Fusion | DSO, ROVIO, SVO, DM-VIO |

### 各变体特征对比

| 变体 | 方法类别 | 稀疏度 | 后端类型 | 光度建模 | IMU 融合 | 代表框架 |
|------|---------|--------|---------|---------|---------|---------|
| 稀疏直接法 | 纯直接 | 稀疏 (~2000 pts) | 滑动窗口 BA | response + vignette + exposure | 无（纯视觉） | DSO |
| 延迟边缘化直接法 | 纯直接 | 稀疏 | 因子图 + delayed marginalization | response + vignette + exposure | IMU 预积分 | DM-VIO |
| 滤波直接法 | 纯直接 | 稀疏 (~25 patches/层) | EKF (IEKF) | 仿射亮度 + warping | IMU 传播 | ROVIO |
| 半直接法 | 混合 | 稀疏 | 滑动窗口 BA (重投影) | 仿射亮度补偿 | 可选 | SVO / SVO Pro |
| LIO 混合直接法 | 混合 | 稀疏 patches | IESKF | 逆曝光 + patch warping | LiDAR + IMU | FAST-LIVO2 |

## 各变体详解

### 1. 光度误差模型

直接法的核心残差。对参考帧 patch $\mathcal{P}$ 中的每个像素偏移 $(x, y)$：

$$
r = \tau_{\text{cur}} \cdot I_{\text{cur}}\big(\pi(T_{\text{cw}} \cdot p_w(u))\big) \;-\; \tau_{\text{ref}} \cdot I_{\text{ref}}^{\text{patch}}[x, y]
$$

其中 $p_w(u) = T_{\text{rw}} \cdot \pi^{-1}(u, d)$ 是参考帧像素对应的 3D 点，$T_{\text{cw}}$ 取左乘形式把世界点转到当前帧。

**链式雅可比**（位姿 $\xi \in \mathfrak{se}(3)$ + 曝光 $\tau$ + 仿射亮度 $a, b$）：

$$
\begin{aligned}
\frac{\partial r}{\partial \delta\xi} &= \tau_{\text{cur}} \cdot \nabla I_{\text{cur}}(p) \cdot \frac{\partial \pi}{\partial p_c} \cdot \big[R_{\text{ci}},\; -R_{\text{ci}} \cdot [p_w]_\times \big] \\[4pt]
\frac{\partial r}{\partial \tau} &= I_{\text{cur}}(p) \quad \text{(曝光雅可比)} \\[4pt]
\frac{\partial r}{\partial a} &= I_{\text{ref}},\quad \frac{\partial r}{\partial b} = 1 \quad \text{(仿射亮度雅可比)}
\end{aligned}
$$

**粗到精金字塔**：图像下采样 $k$ 层，从最粗层开始，每层迭代至位姿增量范数 $< \varepsilon$ 后进入下一层。高层提供大收敛盆地，低层精化细节。

### 2. 稀疏直接法（DSO）

DSO（Direct Sparse Odometry, TRO 2017）是目前最成熟的纯视觉稀疏直接法。核心特征：

- **候选点选择**：在每帧图像梯度上做全局阈值 + 局部极大值抑制，保证梯度质量的同时避开平坦区域
- **光度标定流程**：读入相机 response function $G$、vignette 校准图 $V$、曝光时间 $t_e$，将原始像素强度变换为辐照度 $I = G^{-1}(I_{\text{raw}}) / (t_e \cdot V)$
- **滑动窗口 BA**：维护最近 7 个关键帧，对稀疏点（典型 2000 个）做联合优化，窗口满时用 Schur complement 边缘化最旧帧
- **Huber 鲁棒核**：对光度残差应用 Huber weighting，抑制 outlier

DSO 在单目纯视觉条件下表现出色，在 EuRoC 等数据集上精度超越同期特征点法。工程要点见下文"关键源码片段"中的 `Undistort.cpp` 片段。

**关键洞察**：DSO 证明了 **2000 个高梯度点 + 完整光度标定 = 超越特征的精度**。这不是"点数更多"的胜利，而是残差类型改变——光度误差天然利用所有可用图像信息而非仅角点邻域。

### 3. 半直接法（SVO / SVO Pro）

SVO（Semi-Direct Visual Odometry, ICRA 2014 / TRO 2017）在前端使用直接法快速估计帧间运动，后端回退到重投影误差做 BA 精化。这是实用工程中最重要的混合路线：

**SVO 前端流程**（sparse image alignment）：
1. 从参考帧提取 FAST 角点（数量 ≈ 200）
2. 在 4 层图像金字塔上做逆 compositional 亮度对齐（不是 ICP 型 forward additive）
3. 对齐方式：最小化 `I_ref(warped(T, u)) − I_cur(u)`，更新 $T$ 后重投影
4. 收敛后调用 `reprojector_->reprojectPoints()` 将关键点投影到当前帧
5. 重新提取角点补充丢失的 track

**SVO 后端**：用重投影误差做局部 BA（不是光度 BA），因此称为"半直接"。帧间运动由直接法获得（快速、零描述子），结构的全局一致性由几何重投影保证（可靠）。

SVO Pro 进一步增加了 edgelet（线特征）和四轴相机校准支持，适用于大 FOV 无人机场景。SVO 在 100+ fps 下运行，远快于纯特征点法。

**关键源码**：`raw/codes/rpg_svo_pro_open/svo/src/frame_handler_mono.cpp` 的 `processFrame()` 流水线，以及 `raw/codes/rpg_svo_pro_open/svo_img_align/src/sparse_img_align.cpp` 的逆 compositional 实现。

### 4. 滤波直接法（ROVIO）

ROVIO（Robust Visual-Inertial Odometry, ICRA 2016）将光度误差直接嵌入迭代扩展卡尔曼滤波器：

- **状态**：IMU 位姿 + 速度 + IMU bias + 每个共视路标点的 bearing vector + 逆深度
- **观测**：对每个路标点的多级 patch，使用 `MultilevelPatchAlignment` 最小化 photometric cost
- **更新**：用 IEKF 光度更新取代特征重投影更新，把光度残差 Jacobian 传播到 IMU 状态
- **光度建模**：仿射亮度修正（$I_{\text{ref}} = a \cdot I_{\text{cur}} + b$），缓解曝光变化

ROVIO 的优势在于低纹理场景的鲁棒性——当 ORB-SLAM 因为缺角点而丢弃帧时，ROVIO 的 25 个 patch 依然可提供测量。缺点是需要较好的初始深度估计（通常假设正前方平面初始值），且 EKF 的线性化误差需要定期 FEJ 修正。

**关键源码**：`raw/codes/rovio/include/rovio/MultilevelPatchAlignment.hpp` — 多级 patch 对齐的光度 cost 和 Jacobian。

### 5. 延迟边缘化直接法（DM-VIO）

DM-VIO（IROS 2022）将直接法光度 BA 的稀疏优势与因子图边缘化的数值鲁棒性结合：

- **延迟边缘化队列**：在视觉初始化期间，旧帧先放入边缘化队列而不立即执行 Schur complement，等尺度/重力方向充分收敛后再边缘化
- **多延迟图架构**：初始化前用大延迟纯视觉图（CoarseGraph）、初始化成功后切换到主图（MainGraph）
- **COD 伪逆**：对 Hessian 块使用 COD 分解求伪逆，预处理保证数值稳定

DM-VIO 有效解决了传统 DSO 开环漂移和 VINS-Fusion IMU 初始化慢的问题。它在视觉退化场景中因为有 IMU 融合，精度显著高于纯 DSO。

### 6. LIO 混合直接法（FAST-LIVO2）

FAST-LIVO2 将直接法光度 patch 残差作为 IESKF 的量测更新之一：

- **patch 残差**：$r = \text{state}\_\text{inv\_expo} \cdot I_{\text{cur}}(\text{warped}) - \text{inv\_ref\_expo} \cdot I_{\text{ref}}[\text{patch}]$
- **多观测类型融合**：在同一个 IESKF 量测步骤中同时处理 LiDAR 点到面残差、视觉 patch 光度残差和曝光残差
- **曝光在线估计**：将 `inv_expo_time` 作为 IESKF 状态量实时估计，光度残差 Jacobian 的第 7 列为曝光分量
- **处理策略**：相机图像到达时 `processCameraFrame()` → 构建视觉量测 → 由 IESKF 更新到 IMU 传播状态；视觉失败时不丢弃，继续用 LiDAR-IMU 维持

LASER/IMU 先验对 FAST-LIVO2 非常关键——粗到精金字塔优化需要较好初值，LiDAR 恰好提供这一先验，使视觉 patch 残差只做局部精化而不会被大位移带偏。

### 7. 预处理决策树

直接法中的图像预处理是残差模型的一部分，不是独立于算法的增强步骤：

```text
是否需要对每个像素做响应函数修正？
├── 是 → 是否也有 vignette 和 exposure 数据？
│         ├── 是 → DSO/DM-VIO 全光度标定路径
│         └── 否 → 仅响应函数 (R3LIVE)
└── 否 → 是否需要对帧间亮度变化做修正？
          ├── 是 → 是否在线估计 exposure？
          │         ├── 是 → FAST-LIVO2 逆曝光状态
          │         └── 否 → SVO/ROVIO 仿射亮度
          └── 否 → 原始灰度 (只在恒定光照有效)
```

**关键原则**：CLAHE / 直方图均衡等非线性局部映射会破坏像素强度的单调性，除非显式建模这种非线性变换，否则不应作为直接法的默认预处理。如果必须使用，应作为"破坏光度假设"的对照实验记录。

### 常见工程陷阱速查

| 陷阱 | 表现 | 对策 |
|------|------|------|
| 初值偏差 > 金字塔收敛半径 | photometric cost 不降甚至发散 | 提供 IMU/LiDAR 先验；增大最粗层下采样倍数 |
| 未做光度标定 | 室内/弱光精度显著下降 | 按 Offline Calibration Toolkit 标定 response/vignette |
| patch 大小与运动速度不匹配 | 大运动时 patch 跨场景边界 | 减小 patch（DSO 常用 8 像素 pattern）或使用仿射 warping |
| CLAHE 直接接直接在法前置 | 残差分布偏斜、曝光参数不收敛 | 去掉 CLAHE，只用 response + vignette + exposure |
| 金字塔每层梯度未同步 | 低层投影偏差逐层放大 | 每层独立重构梯度图，投影阈值与 pyramid scale 一致 |
| 曝光参数方向与状态定义冲突 | Jacobian 符号反、残差增大 | 确认使用 inverse_exposure 还是 exposure_time，Jacobian 列顺序匹配 |

## 选型指南

| 场景 | 推荐变体 | 理由 |
|------|---------|------|
| 纯视觉、弱纹理、单目 | DSO | 完整光度 BA，单目表现最优 |
| 纯视觉 + IMU、需要鲁棒性 | DM-VIO | 延迟边缘化，融合 IMU 预积分 |
| 高速飞行、计算受限 | SVO / SVO Pro | 100+ fps，半直接法均衡速度与精度 |
| 嵌入式平台、滤波框架 | ROVIO | EKF 低功耗，patch 小时变换鲁棒 |
| LiDAR + IMU + 相机多源融合 | FAST-LIVO2 | LiDAR/IMU 提供先验，视觉做局部光度精化 |
| 研究目的、需要对比间接法 | DSO + ORB-SLAM3 双开 | 直接法光度观察，间接法做回环和全局约束 |

## Agent 实现提示

### 适用场景

当 agent 需要实现直接法视觉前端、构造 photometric residual 并接入优化器/滤波器，或为 LIVO/VIO 系统添加直接法视觉量测时使用。核心不是"提取特征点"，而是为参考 patch 和当前投影 patch 构造可解释的强度差，并将位姿、深度、曝光/仿射亮度和鲁棒核的 Jacobian 一起接到后端。

不适用场景：(a) 需要长期重定位和回环检测的系统——应优先选间接法或独立 DBoW 回环模块；(b) 大基线跨视角匹配——光度一致性假设完全失效，应使用描述子；(c) 曝光极不稳定的场景且无法做光度标定——残差将不可信。

### 输入输出契约

- **输入**：参考帧图像（含 pyramid）、当前帧图像、参考 patch 坐标和强度、3D 点坐标或逆深度、当前位姿初值、相机模型（pinhole/atan/spline）、曝光/仿射亮度参数、图像梯度图（每层）。
- **输出**：光度残差向量 $r \in \mathbb{R}^M$、位姿 Jacobian $J_{\text{pose}} \in \mathbb{R}^{M \times 6}$、可选曝光 Jacobian $J_{\text{expo}} \in \mathbb{R}^{M \times 1}$（和/或 $J_{\text{affine}} \in \mathbb{R}^{M \times 2}$）、patch-wise error sum、有效观测数、平均 NCC。
- **坐标/单位**：投影点落在 processed pixel 坐标；图像强度可以是 raw intensity、response/vignette 校正后的辐照度、或乘以逆曝光后的归一化强度——三者不可混用，一旦选定必须在整个 pipeline 中保持一致。时间单位统一为秒；translation in world/body frame (m)。

### 实现骨架（伪代码）

```pseudo
function direct_visual_frontend(ref_frame, cur_image, state, camera, config):
    validated_inputs = validate_contract(ref_frame, cur_image, state, camera, config)

    residuals = []   // M 个残差
    H_pose = []      // M × 6
    H_expo = []      // M × 1 (可选)
    H_affine = []    // M × 2 (可选, SVO/ROVIO)
    diagnostics = Diagnostic()
    n_valid = 0

    for level from coarsest to finest:
        T_level = state.T_cur_ref

        for point in ref_frame.points_at_level[level]:
            // 投影当前帧
            p_cur = project(camera, transform(T_level, point.p_world, point.patch_offset))
            if not in_bounds(p_cur, cur_image[level]):
                continue

            // 双线性插值 + 梯度
            I_cur, grad_x, grad_y = bilinear_sample_with_gradient(cur_image[level], p_cur)
            I_ref = ref_frame.patch_intensity[level][point.id][point.patch_offset]

            // 光度残差: 按系统选择曝光 / 仿射建模
            if config.estimate_exposure:
                r = state.tau_cur * I_cur - ref_frame.tau_ref * I_ref
            elif config.estimate_affine:
                r = I_cur - (state.a * I_ref + state.b)
            else:
                r = I_cur - I_ref

            // Jacobian 链式传播
            grad_pixel = [grad_x, grad_y]
            dpi_dpc = project_jacobian(camera, p_cur)                 // 2×3
            dpc_dpose = camera_frame_jacobian(point.p_world)          // 3×6
            J_pose = state.tau_cur * grad_pixel * dpi_dpc * dpc_dpose  // 1×6

            // 鲁棒核降权
            w = robust_weight(r, config.loss_scale, config.loss_type)
            r_weighted = w * r

            // 累加
            residuals.append(r_weighted)
            H_pose.append(J_pose)
            if config.estimate_exposure:
                H_expo.append(I_cur)
            if config.estimate_affine:
                H_affine.append([I_ref, 1.0])

            diagnostics.patch_error_sum += r * r
            n_valid += 1

        // 求解 + 位姿更新
        H_aug = build_augmented_hessian(H_pose, H_expo, H_affine)
        b = collect_weighted_residuals()

        dx = solve(H_aug, -b)
        T_level = exp(dx[:6]) * T_level

        if norm(dx) < config.pyramid_convergence_thresholds[level]:
            break  // 进入下一层

    state.T_cur_ref = T_level
    diagnostics.n_valid = n_valid

    return state, residuals, diagnostics
```

### 关键源码片段

**DSO 光度预处理** — `raw/codes/dso/src/util/Undistort.cpp:L222-L250`

```cpp
if(!valid || exposure_time <= 0 || setting_photometricCalibration==0) {
    for(int i=0; i<wh;i++)
        data[i] = factor*image_in[i];
    output->exposure_time = exposure_time;
} else {
    for(int i=0; i<wh;i++)
        data[i] = G[image_in[i]];

    if(setting_photometricCalibration==2)
        for(int i=0; i<wh;i++)
            data[i] *= vignetteMapInv[i];

    output->exposure_time = exposure_time;
}
```

三种 photometric mode：(0) 仅乘 factor，(1) 仅 response，(2) response + vignette。

**FAST-LIVO2 patch 残差与曝光 Jacobian** — `raw/codes/FAST-LIVO2/src/vio.cpp:L1619-L1628`

```cpp
double cur_value =
    w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
    w_ref_bl * img_ptr[scale * width] + w_ref_br * img_ptr[scale * width + scale];
double res = state->inv_expo_time * cur_value - inv_ref_expo * P[patch_size_total * level + x * patch_size + y];
z(i * patch_size_total + x * patch_size + y) = res;
patch_error += res * res;
n_meas += 1;

if (exposure_estimate_en) {
    H_sub.block<1, 7>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt, cur_value;
} else {
    H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt;
}
```

第 7 列为曝光雅可比 `cur_value`，与状态量中的 `inv_expo_time` 对应。

**SVO sparse image alignment** — `raw/codes/rpg_svo_pro_open/svo_img_align/src/sparse_img_align.cpp`（逆 compositional 路径）

SVO 的 `compute_log_pyr_mapping_ir()` 计算多层金字塔上 image 梯度和 Hessian 预计算（constant-time per iteration 关键），`optimize_gauss_newton()` 使用逆 compositional 规则，Hessian 在每个 image pyramid 开始前仅计算一次，无需每迭代重新计算图像梯度。

**ROVIO multilevel patch alignment** — `raw/codes/rovio/include/rovio/MultilevelPatchAlignment.hpp`

仿射亮度 patch 对齐，update 时更新 bearing vector 和仿射参数 $(a, b)$，`linearizePatchAlignment()` 生成 IMU 状态对 patch 残差的 Jacobian。

### 实现注意事项

- **图像预处理必须可逆或显式建模**：response/vignette/exposure 是光度模型的一部分，不是图像增强。CLAHE / 直方图均衡等非线性变换会改变局部亮度映射，通常不适合作为直接法默认预处理。
- **patch residual 的符号约定、曝光参数方向（exposure_time vs inverse_exposure）必须和状态定义一致**，Jacobian 列顺序在所有量测类型中必须对齐。
- **图像金字塔每层必须同步梯度、尺度和投影阈值**：低层用于捕获大位移（使用大 patch 和宽松投影门限），高层用于精化（使用小 patch 和严格门限）。
- **鲁棒核尺度和异常值门限用途不同**：先用距离/motion 门限裁剪明显不可行的对应，再用 Huber/GM/Cauchy 核压制剩余大残差。
- **FEJ (First-Estimate Jacobians)** 对于带边缘化或 IEKF 的直接法非常重要——线性化点必须与原始估计一致，错误的 Jacobian 线性化点会在多次状态更新后导致不一致。
- **patch 大小选择**：DSO 使用 8 像素 sparse pattern（非 dense patch），ROVIO 使用 4 级 patch (8/16/32/64)，SVO 使用 4×4 patch——过大 patch 在大运动时跨场景边界，过小 patch 对噪声敏感。
- **直接法输出不宜直接用于全局回环**：光度一致性假设仅在相邻帧间成立，全局回环应使用几何验证或描述子匹配。

### 源码检索锚点

- `raw/codes/dso/src/util/Undistort.cpp` — response/vignette/exposure 光度预处理
- `raw/codes/dso/src/FullSystem/HessianBlocks.cpp` — 图像金字塔、梯度权重和 pattern 选择
- `raw/codes/dm-vio/src/util/MainSettings.cpp` — different photometric mode 工程开关
- `raw/codes/rovio/include/rovio/MultilevelPatchAlignment.hpp` — affine brightness patch residual
- `raw/codes/rovio/include/rovio/RovioFilter.hpp` — IEKF 光度更新流程
- `raw/codes/FAST-LIVO2/src/vio.cpp` — LIVO patch residual、inverse exposure 和 IESKF 量测 Jacobian
- `raw/codes/rpg_svo_pro_open/svo_img_align/src/sparse_img_align.cpp` — 逆 compositional 稀疏图像对齐
- `raw/codes/rpg_svo_pro_open/svo/src/reprojector.cpp` — 点投影与深度图管理
- `raw/codes/rpg_svo_pro_open/svo/src/frame_handler_mono.cpp` — 单目 SVO 主循环

## 相关页面

- [[概念-直接法光度误差]]（已 superseded，详见本页第 1-2 节）
- [[概念-直接法视觉里程计]]（已 superseded，详见本页对比表和第 2-4 节）
- [[概念-直接法vs间接法]]（已 superseded，详见本页对比表）
- [[算法-DSO]], [[算法-DM-VIO]], [[算法-ROVIO]], [[算法-SVO-Pro]], [[算法-FAST-LIVO2]]
- [[方法-视觉特征跟踪]] — 间接法前端
- [[方法-曝光在线估计]]
- [[方法-图像预处理]]
- [[概念-延迟边缘化]]
- [[概念-因子图]]
- [[概念-Schur补与边缘化]]
- [[数学-流形优化]]
- [[方法-关键帧选择策略]]
