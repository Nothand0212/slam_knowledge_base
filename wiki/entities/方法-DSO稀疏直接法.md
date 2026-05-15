---
tags: [DSO, 直接法, 光度标定, 滑动窗口, 稀疏直接法, Schur补, FEJ]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/dso/src/FullSystem/FullSystemOptimize.cpp
  - raw/codes/dso/src/FullSystem/FullSystem.h
  - raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp
  - raw/codes/dso/src/FullSystem/Residuals.cpp
  - raw/codes/dso/src/FullSystem/Residuals.h
  - raw/codes/dso/src/FullSystem/PixelSelector2.cpp
  - raw/codes/dso/src/util/globalCalib.cpp
  - raw/codes/dso/src/util/globalCalib.h
  - raw/codes/dso/src/OptimizationBackend/EnergyFunctional.h
  - raw/codes/dso/src/OptimizationBackend/AccumulatedSCHessian.cpp
---

# DSO 稀疏直接法

> DSO（Direct Sparse Odometry）是稀疏直接法的代表框架，在滑动窗口内联合优化所有关键帧的位姿、相机内参、仿射亮度参数以及稀疏点的逆深度。使用完整的光度标定模型（vignetting + response function + exposure time）补偿图像亮度变化，采用 Schur 补 + First-Estimate Jacobian（FEJ）实现窗口边缘化。

## 能量函数

DSO 定义滑动窗口内的总能量为所有活动残差的加权和：

$$
E = \sum_{i \in \mathcal{F}} \sum_{p \in \mathcal{P}_i} \sum_{j \in \text{obs}(p)} w_{ij} \cdot \rho\big( r_{ij}^2 \big)
$$

其中：
- $\mathcal{F}$：滑动窗口内的关键帧集合（通常 5-7 帧）
- $\mathcal{P}_i$：关键帧 $i$ 上被激活的稀疏点
- $\text{obs}(p)$：点 $p$ 被观测到的其他关键帧
- $w_{ij}$：基于梯度的权重（抑制低纹理区域残差）
- $\rho$：Huber 鲁棒核函数

单个光度残差定义为：

$$
r_{ij} = I_j\big(\pi(T_{ji} \cdot \pi^{-1}(p, d_p))\big) - I_i(p)
$$

其中 $T_{ji} \in SE(3)$ 是帧 $i$ 到帧 $j$ 的刚体变换，$d_p$ 是点在 host 帧的逆深度。

**源码锚点**: `raw/codes/dso/src/FullSystem/Residuals.cpp:L78-L274`

### 完整光度残差（含光度标定）

在实际实现中，残差包含完整的光度校正项：

$$
r_{ij} = \tau_j \cdot V_j(u_j) \cdot G^{-1}\big(I_j(u_j)\big) \;-\; \tau_i \cdot V_i(u_i) \cdot G^{-1}\big(I_i(u_i)\big)
$$

其中：
- $G^{-1}(\cdot)$：逆响应函数（radiometric response），将 0-255 灰度值映射到线性辐照度
- $V(\cdot)$：vignetting（渐晕）校正，补偿镜头边缘变暗
- $\tau$：曝光时间因子

程序中进一步简化：将 $G^{-1}$ 和 $V$ 合并为单一的查找表 `G[256]`（存储在 `FrameHessian` 中），并通过 `CalibHessian` 在线优化。

**源码锚点**: `raw/codes/dso/src/util/globalCalib.h:L30-L44` (全局标定参数声明)

### 仿射亮度模型

DSO 为每帧维护一对仿射亮度参数 $(a, b)$，在残差计算中补偿帧间亮度变化：

$$
I_j^{\text{corrected}} = e^{-a_j} \cdot (I_j - b_j)
$$

等价于：$I_{\text{affine}} = \frac{I - b}{e^{a}}$。这两个参数在优化中作为变量参与求解。

**源码锚点**: `raw/codes/dso/src/FullSystem/Residuals.cpp:L96-L97`, `L196-L198`

```cpp
Vec2f affLL = precalc->PRE_aff_mode;
float b0 = precalc->PRE_b0_mode;
// ...
float residual = hitColor[0] - (float)(affLL[0] * color[idx] + affLL[1]);
```

## 光度雅可比矩阵（链式法则）

从像素灰度到优化变量的完整雅可比传播链：

$$
\frac{\partial r}{\partial \xi} = \underbrace{\nabla I_j(u_j)}_{\text{图像梯度}} \cdot \underbrace{\frac{\partial \pi}{\partial p_c}}_{\text{投影雅可比}} \cdot \underbrace{\frac{\partial (R \cdot p_w + t)}{\partial \xi}}_{\text{SE(3) 雅可比}}
$$

**源码中位姿雅可比** (`raw/codes/dso/src/FullSystem/Residuals.cpp:L144-L157`)：

```cpp
d_xi_x[0] = new_idepth*HCalib->fxl();   // translation x
d_xi_x[1] = 0;
d_xi_x[2] = -new_idepth*u*HCalib->fxl(); // translation z
d_xi_x[3] = -u*v*HCalib->fxl();          // rotation roll
d_xi_x[4] = (1+u*u)*HCalib->fxl();       // rotation pitch
d_xi_x[5] = -v*HCalib->fxl();            // rotation yaw
```

**逆深度雅可比** (`raw/codes/dso/src/FullSystem/Residuals.cpp:L116-L117`)：

```cpp
d_d_x = drescale * (PRE_tTll_0[0]-PRE_tTll_0[2]*u)*SCALE_IDEPTH*HCalib->fxl();
d_d_y = drescale * (PRE_tTll_0[1]-PRE_tTll_0[2]*v)*SCALE_IDEPTH*HCalib->fyl();
```

**光度参数雅可比**：对仿射参数 $a, b$ 的雅可比通过 $J_{ab}$ 矩阵累积 (`raw/codes/dso/src/FullSystem/Residuals.cpp:L220-L243`)。

## 滑动窗口边缘化与 FEJ

### 边缘化策略

DSO 在每帧到达后调用 `flagFramesForMarginalization` 判定哪些帧应被边缘化 (`raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp:L58-L147`)：

1. **点数不足条件**：如果帧的剩余活跃点数 / 初始点数 < `setting_minPointsRemaining`（默认 0.05），标记为边缘化
2. **滑窗满条件**：如果滑窗内帧数 ≥ `setting_maxFrames`（默认 7），选择距离最新帧"信息量最少"的帧边缘化

"信息量"定义为：

```cpp
distScore = 0;
for each target frame:
    distScore += 1/(1e-5 + distance_to_target);
distScore *= -sqrt(distance_to_latest_frame);
```

距离最近、与邻居帧高度重叠的帧优先被边缘化。

**源码锚点**: `raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp:L107-L140`

### 边缘化执行过程

边缘化由 `EnergyFunctional::marginalizeFrame` 执行（通过 `AccumulatedSCHessian`），内部使用 Schur 补：

1. 收集与被边缘化帧相关的所有残差
2. 构建增广 Hessian $\begin{bmatrix} \mathbf{H}_{mm} & \mathbf{H}_{ma} \\ \mathbf{H}_{am} & \mathbf{H}_{aa} \end{bmatrix}$
3. Schur 补：$\mathbf{H}_{aa}^{\text{new}} = \mathbf{H}_{aa} - \mathbf{H}_{am} \mathbf{H}_{mm}^{-1} \mathbf{H}_{ma}$
4. 将结果作为先验信息（用 Hessian 形式）保留在 EnergyFunctional 中

**源码锚点**: `raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp:L152-L215` (marginalizeFrame)，`raw/codes/dso/src/OptimizationBackend/AccumulatedSCHessian.cpp:L34-L77` (addPoint Schur 累积)

### First-Estimate Jacobian（FEJ）

为防止线性化点漂移导致的系统不一致，DSO 对所有边缘化相关的残差使用 FEJ：
- 首次边缘化时记录变量的线性化点
- 后续优化中，相关残差的雅可比始终在 FEJ 点计算，而非当前线性化点
- 由 `EnergyFunctional::setAdjointsF` / `setDeltaF` 管理 FEJ 状态

## 点选择策略

### 梯度直方图法

DSO 的 `PixelSelector` 基于图像梯度自适应选择候选点 (`raw/codes/dso/src/FullSystem/PixelSelector2.cpp:L41-L80`)：

1. 对第一帧计算梯度幅值直方图（32×32 网格）
2. 设定阈值：保留梯度处于上分位（例如 top 30%）的像素
3. 在每个 32×32 块中仅保留 1 个最优候选点（避免聚集）

```pseudo
for each 32x32 grid cell:
    threshold = grad_histogram.quantile(1 - setting_desiredDensity)
    select pixel with max(gradient) above threshold in this cell
```

### 激活候选点（Immature Points）

被选中的候选点初始为 ImmaturePoint，通过多帧极线搜索追踪：

1. 沿极线计算 NCC（归一化互相关）匹配代价
2. 收敛后逆深度不确定性足够小 → 激活为 PointHessian（加入优化）
3. 不收敛或超出边界 → 丢弃

### Pattern 模式（8 点采样）

每个点的光度残差不是基于单个像素，而是基于以该点为中心的 **8 点固定 pattern**（`patternNum = 8`）：

```cpp
const float patternP[8][2] = {
    // 围绕中心点的 8 个偏移采样
};
```

每个 pattern 点独立计算残差和雅可比，通过 `projectPoint` 投影到目标帧后做双线性插值。这一设计提高了光度约束的鲁棒性，特别是对运动模糊。

**源码锚点**: `raw/codes/dso/src/FullSystem/Residuals.cpp:L183-L245`

## 优化流程

核心优化函数 `FullSystem::optimize(int mnumOptIts)` 执行 Levenberg-Marquardt 迭代 (`raw/codes/dso/src/FullSystem/FullSystemOptimize.cpp:L408-L601`)：

```pseudo
function optimize(num_iterations):
    activeResiduals = collect_all_active_residuals()

    // 线性化所有残差（不 fix）
    E_old = linearizeAll(fixLinearization=false)

    // 应用残差 -> 填充 Hessian
    applyResiduals()

    lambda = 0.1
    for i = 1 to num_iterations:
        backupState()                        // 备份线性化点
        solveSystem(iteration, lambda)       // 求解 Hx = -b
        doStepFromBackup(...)                // 更新变量
        E_new = linearizeAll(fixLinearization=false)
        if E_new < E_old:
            accept update
            applyResiduals()
            lambda *= 0.25                   // 收缩阻尼
        else:
            loadStateBackup()
            lambda *= 100                    // 增大阻尼
        if canBreak() and i >= min_iterations:
            break

    // 最终线性化（fix=true，确定 OUTLIER/IN 状态）
    linearizeAll(fixLinearization=true)
```

每次迭代中的收敛判定 (`raw/codes/dso/src/FullSystem/FullSystemOptimize.cpp:L289-L305`)：

```cpp
return sqrtf(sumA) < 0.0005*setting_thOptIterations &&
       sqrtf(sumB) < 0.00005*setting_thOptIterations &&
       sqrtf(sumR) < 0.00005*setting_thOptIterations &&
       sqrtf(sumT)*sumNID < 0.00005*setting_thOptIterations;
```

## 关键数据结构

| 结构 | 作用 | 文件 |
|------|------|------|
| `FrameHessian` | 维护帧的位姿、仿射亮度、内参状态 | `FullSystem/HessianBlocks.h` |
| `PointHessian` | 维护点的逆深度、host 帧、残差列表 | `FullSystem/HessianBlocks.h` |
| `PointFrameResidual` | 单点-帧对的光度残差和雅可比 | `FullSystem/Residuals.h` |
| `CalibHessian` | 在线优化的相机内参和光度标定参数 | `util/NumType.h` |
| `EnergyFunctional` | 管理 H/b 组装、Schur 补、先验 | `OptimizationBackend/EnergyFunctional.h` |
| `EFFrame / EFPoint / EFResidual` | EnergyFunctional 的内部表示 | `OptimizationBackend/EnergyFunctionalStructs.h` |
| `AccumulatedSCHessian` | 逐点 Schur 补累积器 | `OptimizationBackend/AccumulatedSCHessian.h` |
| `PixelSelector` | 基于梯度直方图的点选择器 | `FullSystem/PixelSelector2.h` |

## 光度标定管线

DSO 需要离线的光度标定文件 `pcalib.txt`：

```
# pcalib.txt 格式
G[0] G[1] ... G[255]     # 256 值逆响应函数（辐照度 = G[pixel_value]）
vignette[0] vignette[1] ...   # 逐像素 vignetting 校正因子
```

在线优化中，`CalibHessian` 还会进一步优化 vignetting 模型参数 (global camera intrinsics)。

**源码锚点**: `raw/codes/dso/src/util/globalCalib.cpp:L45-L65`（金字塔内参构建，与光度标定共享框架）

---

## Agent 实现提示

### 适用场景

- 弱纹理场景（白墙、地板），间接法特征提取失败
- 需要利用所有梯度像素做约束的场景
- 计算资源有限但需要高精度（稀疏点 + 滑动窗口，比稠密法快 10x+）
- 不适用：强光照变化（未标定相机）、大基线运动（>30° 旋转）、快速纯旋转

### 输入输出契约

- **输入**：光度标定文件（pcalib.txt）、相机内参、图像帧序列（需 raw 格式，不得有 JPEG 压缩 / 自动白平衡 / 自动曝光）
- **输出**：每帧 world-to-camera 6-DoF 位姿、稀疏 3D 点云（PC_***.ply）、每帧光度仿射参数
- **坐标约定**：DSO 使用 `worldToCam`（左乘）表示位姿：$p_{\text{cam}} = T_{\text{cw}} \cdot p_{\text{world}}$
- **深度表示**：逆深度 $d = 1/z$，idepth = $d$，idepth_scaled = $d \cdot$ SCALE_IDEPTH

### 实现骨架（伪代码）

```pseudo
function DSO_pipeline():
    // 1. 加载光度标定：G[256], vignette_map
    calib = loadPhotometricCalib("pcalib.txt")
    // 2. 为每帧计算 photometrically corrected image:
    //    I_corrected[u,v] = G[I_raw[u,v]] / vignette_map[u,v] / exposure_time
    K, fx, fy, cx, cy = loadIntrinsics()

    pixel_selector = PixelSelector(w, h)

    for each frame:
        frame = FrameHessian(image_corrected, exposure, K)
        // 3. 粗跟踪
        if not initialized:
            FrameHessian ref = select first frame with enough gradient
            coarseInitializer.trackFrame(frame)
        else:
            coarseTracker.trackNewestCoarse(frame)

        // 4. 点选择
        if frame is KF:
            candidates = pixel_selector.select(frame, desired_density=2000)
            for each candidate:
                ImmaturePoint ip = traceCandidate(candidate, prev_frames)
                if ip.converged:
                    activatePoint(ip)  // 加入 PointHessian

            // 5. 滑动窗口优化
            optimize(min_iter=4, max_iter=6)

            // 6. 边缘化
            flagFramesForMarginalization(newFrame)
            for each flagged frame:
                marginalizeFrame(frame)  // Schur 补 → 先验

        // 7. 移除 Outlier
        removeOutliers()

    return allKF_poses, all_active_points
```

### 关键源码片段

**光度残差 + 雅可比线性化** (`raw/codes/dso/src/FullSystem/Residuals.cpp:L78-L274`)：
```cpp
double PointFrameResidual::linearize(CalibHessian* HCalib) {
    state_NewEnergyWithOutlier=-1;
    if(state_state == ResState::OOB)
        { state_NewState = ResState::OOB; return state_energy; }

    FrameFramePrecalc* precalc = &(host->targetPrecalc[target->idx]);
    // ... projectPoint 计算 u,v,Ku,Kv
    // ... 计算 d_xi_x, d_xi_y（位姿雅可比）
    // ... 计算 d_C_x, d_C_y（内参雅可比）
    // ... 计算 d_d_x, d_d_y（逆深度雅可比）

    // pattern-based 残差计算
    for(int idx=0;idx<patternNum;idx++) {
        projectPoint(point->u+patternP[idx][0], ...);
        Vec3f hitColor = getInterpolatedElement33(dIl, Ku, Kv, wG[0]);
        float residual = hitColor[0] - (affLL[0]*color[idx] + affLL[1]);
        // Huber weight + gradient weight
        float hw = fabsf(residual) < setting_huberTH ? 1 : setting_huberTH/fabsf(residual);
        energyLeft += w*w*hw*residual*residual*(2-hw);
    }
    // Outlier 判定
    if(energyLeft > max(host->frameEnergyTH, target->frameEnergyTH))
        state_NewState = ResState::OUTLIER;
    else
        state_NewState = ResState::IN;
}
```

**边缘化策略** (`raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp:L107-L140`)：
```cpp
// 选择距离最近的信息量最少的帧边缘化
for(FrameHessian* fh : frameHessians) {
    if(fh->frameID > latest->frameID-setting_minFrameAge || fh->frameID == 0) continue;
    double distScore = 0;
    for(FrameFramePrecalc &ffh : fh->targetPrecalc) {
        if(ffh.target->frameID > latest->frameID-setting_minFrameAge+1) continue;
        distScore += 1/(1e-5+ffh.distanceLL);
    }
    distScore *= -sqrtf(fh->targetPrecalc.back().distanceLL);
    if(distScore < smallestScore) {
        smallestScore = distScore;
        toMarginalize = fh;
    }
}
```

### 实现注意事项

- **光度标定是强依赖**：未标定的相机（自动曝光 / 自动白平衡 / JPEG 压缩）会导致光度一致性假设彻底失效，DSO 将快速丢失
- **Huber 核的两段式公式**：`energy = w * hw * r^2 * (2 - hw)`——当 $|r| \leq \text{th}$ 时 $hw=1$ 即为二次项；当 $|r| > \text{th}$ 时 $hw = \text{th}/|r|$ 退化为 $2\text{th}|r| - \text{th}^2$
- **pattern 点投影时要检查 OOB**：每个 pattern 点独立验证是否在图像内，一个 OOB 即导致整个残差标记 OOB（保守处理）
- **frameEnergyTH 自适应阈值**：基于前 N% 残差的中位数自动计算 Outlier 阈值（`setNewFrameEnergyTH`, L93-L141）
- **SCALE_IDEPTH / SCALE_F / SCALE_C 缩放常量**：使不同维度的雅可比和 Hessian 在同一数量级，避免数值奇异性
- **Schur 补的点深度消元**：DSO 对每个 PointHessian 的逆深度做 Schur 消元（AccumulatedSCHessian），将 H 矩阵压缩为仅含帧位姿的较小矩阵

### 源码检索锚点

- `raw/codes/dso/src/FullSystem/FullSystemOptimize.cpp:L408-L601` — optimize 主循环
- `raw/codes/dso/src/FullSystem/Residuals.cpp:L78-L274` — 光度残差 linearize + Jacobian
- `raw/codes/dso/src/FullSystem/Residuals.h` — PointFrameResidual 和 RawResidualJacobian 定义
- `raw/codes/dso/src/FullSystem/FullSystemMarginalize.cpp` — 边缘化判定和执行
- `raw/codes/dso/src/FullSystem/PixelSelector2.cpp` — 梯度直方图点选择
- `raw/codes/dso/src/OptimizationBackend/AccumulatedSCHessian.cpp` — 逐点 Schur 消元
- `raw/codes/dso/src/OptimizationBackend/EnergyFunctional.h` — EnergyFunctional 接口
- `raw/codes/dso/src/FullSystem/HessianBlocks.h` — FrameHessian / PointHessian
- `raw/codes/dso/src/util/globalCalib.cpp` — 全局标定参数和金字塔内参

## 相关页面

- [[方法-直接法SLAM族]]
- [[方法-滑动窗口边缘化]]
- [[方法-关键帧选择策略]]
- [[方法-延迟边缘化VIO]]
- [[方法-图像预处理]]
- [[方法-曝光在线估计]]
