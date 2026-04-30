# DSO (Direct Sparse Odometry) 源码深度分析

> 分析基于 DSO 开源代码 (Jakob Engel et al., IROS 2018)
> 文件路径: `/home/lin/Projects/lin_ws/slam_ws/dso/`
> 核心代码量: ~15个 .cpp/.h 文件, 总计约 5000 行

---

## 1. 直接法 vs 间接法的根本区别: 为什么不用特征点?

### 1.1 间接法 (特征点法) 的核心思想

间接法的代表是 ORB-SLAM3。它遵循以下流程:
1. 提取图像特征点 (FAST, Harris角点等)
2. 计算特征描述子 (ORB, BRIEF等)
3. 在帧间匹配描述子 → 得到 2D-2D 匹配
4. 通过匹配+三角化估计 3D 点坐标
5. 最小化**重投影误差**: 将 3D 点投影到相机平面, 计算与观测(匹配)位置的像素差

重投影误差公式: `r_i = p_i - π(T * P_i)`
其中 `p_i` 是特征点的像素坐标, `P_i` 是三维世界坐标点, `π` 是相机投影函数, `T` 是相机位姿。

**本质问题**: 间接法只利用了 *匹配到的特征点的像素坐标* (每帧几百个点), 丢弃了图像中 99% 以上的像素信息。

### 1.2 DSO的光度误差 (Photometric Error) — 直接法

在 DSO 的核心残差文件中 (`src/FullSystem/Residuals.cpp:78-328`), 光照误差定义为:

```
r = I_target(π(K * R * K^{-1} * p_source + K * t * d)) - (exp(a_target - a_source) * I_source(p_source) + b_target - b_source)
```

**完整展开** (`src/FullSystem/Residuals.cpp:193-194`):

```cpp
// 帧间投影: 把 reference 帧中的 pattern 点投影到 target 帧
// Residuals.cpp, lines 108-191: 计算投影变换
float drescale, u, v, new_idepth; 
float Ku, Kv;
projectPoint(point->u, point->v, point->idepth_zero_scaled, 0, 0, HCalib,
        PRE_RTll_0, PRE_tTll_0, drescale, u, v, Ku, Kv, KliP, new_idepth);

// 然后在每个 pattern 点处计算残差:
for(int idx=0; idx<patternNum; idx++) {
    // 用实际逆深度投影 pattern 点
    projectPoint(point->u+patternP[idx][0], point->v+patternP[idx][1],
         point->idepth_scaled, PRE_KRKiTll, PRE_KtTll, Ku, Kv);
    
    // 读取 target 帧的插值颜色 (双线性插值3x3)
    Vec3f hitColor = getInterpolatedElement33(dIl, Ku, Kv, wG[0]);
    
    // 光度残差 = target帧插值亮度 - (仿射变换后的 source亮度)
    float residual = hitColor[0] - (float)(affLL[0] * color[idx] + affLL[1]);
}
```

**公式表示:**
$$r = \sum_{p \in N_p} w_p \left\| I_j[p'] - e^{a_{ji}} \cdot I_i[p] - b_{ji} \right\|_\gamma$$

其中:
- $p$ 是 source 帧中的 pattern 点 (8个点的pattern)
- $p'$ 是 $p$ 在 target 帧中的投影位置, 通过相机投影函数 + 逆深度 $d$ 得到
- $a_{ji}, b_{ji}$ 是帧间仿射亮度参数 (处理曝光差异)
- $\|\cdot\|_\gamma$ 是 Huber 损失函数

**为什么这样更好?**

| 对比维度 | 间接法 (ORB-SLAM3) | 直接法 (DSO) |
|---------|-------------------|-------------|
| 信息利用 | 几百个特征点坐标 | 整张图像的梯度信息 (5000-10000点) |
| 纹理要求 | 必须有强角点 | 任何有梯度的区域均可 |
| 几何表达 | 3D点参数化 (x,y,z) | 只优化1个参数: 逆深度 d |
| 光照变化 | 依赖描述子不变性 | 显式建模仿射亮度 $a,b$ |
| 相机内参 | 假定标定完美 | 在线优化内参与光度参数 |
| 初始化 | 需要计算本质矩阵/H矩阵 | 纯光度最小化 (不需要计算几何) |

### 1.3 光度标定 (Photometric Calibration)

DSO 使用光度响应函数 (response function) 来去除相机 JPEG 压缩的非线性。由 `CalibHessian` 类管理 (`src/FullSystem/HessianBlocks.h:299-399`):

```cpp
// HessianBlocks.h:380-398
float B[256];     // 响应函数 G (原始→图像亮度)
float Binv[256];  // 逆响应函数 G^{-1} (图像亮度→原始)

// 计算响应函数梯度 (用于像素选择中的权重)
EIGEN_STRONG_INLINE float getBGradOnly(float color) {
    int c = color+0.5f;
    if(c<5) c=5; if(c>250) c=250;
    return B[c+1]-B[c];
}
```

在 `makeImages()` 中 (`src/FullSystem/HessianBlocks.cpp:128-191`), 计算梯度时使用了响应函数修正:

```cpp
// HessianBlocks.cpp:184-188
if(setting_gammaWeightsPixelSelect==1 && HCalib!=0) {
    float gw = HCalib->getBGradOnly((float)(dI_l[idx][0]));
    dabs_l[idx] *= gw*gw;  // 转换到原始光照空间梯度
}
```

---

## 2. 数据接收与预处理

### 2.1 图像接收与金字塔构建

入口函数 `addActiveFrame()` (`src/FullSystem/FullSystem.cpp:802-903`):

```cpp
// FullSystem.cpp:810-817
FrameHessian* fh = new FrameHessian();
FrameShell* shell = new FrameShell();
shell->camToWorld = SE3();
shell->aff_g2l = AffLight(0,0);
shell->marginalizedAt = shell->id = allFrameHistory.size();
shell->timestamp = image->timestamp;

// 构建图像金字塔 + 梯度
fh->makeImages(image->image, &Hcalib);
```

`FrameHessian::makeImages()` 构建图像金字塔 (`src/FullSystem/HessianBlocks.cpp:128-191`):
- 对每层金字塔 (0-5层): 存储 `(I, dI/dx, dI/dy)` (亮度+梯度)
- `dI` 是 `Eigen::Vector3f[N]` 数组, 分别存储 (亮度, x梯度, y梯度)
- `absSquaredGrad` 存储梯度平方和, 用于像素选择

```cpp
// HessianBlocks.cpp:138-143
int w=wG[0], h=hG[0];
for(int i=0;i<w*h;i++)
    dI[i][0] = color[i];  // 亮度

// 计算图像梯度:
for(int idx=wl; idx < wl*(hl-1); idx++) {
    float dx = 0.5f*(dI_l[idx+1][0] - dI_l[idx-1][0]);
    float dy = 0.5f*(dI_l[idx+wl][0] - dI_l[idx-wl][0]);
    dI_l[idx][1] = dx;    // x梯度
    dI_l[idx][2] = dy;    // y梯度
    dabs_l[idx] = dx*dx + dy*dy;
}
```

### 2.2 像素选择 (Pixel Selector)

`PixelSelector` (`src/FullSystem/PixelSelector.h` + `.cpp`) 从每帧图像中选择高梯度点:

- 选择 `desiredImmatureDensity` 个像素点 (默认 ~2000)
- 根据梯度选择, 同时保证空间分布的均匀性
- 返回 `selectionMap` (float数组, 0=不选, >0=选择且标记类型)

在 `makeNewTraces()` (`src/FullSystem/FullSystem.cpp:1286-1311`) 中:

```cpp
int numPointsTotal = pixelSelector->makeMaps(newFrame, selectionMap, setting_desiredImmatureDensity);

// 对每个选中像素创建 ImmaturePoint
for(int y=patternPadding+1; y<hG[0]-patternPadding-2; y++)
for(int x=patternPadding+1; x<wG[0]-patternPadding-2; x++) {
    int i = x+y*wG[0];
    if(selectionMap[i]==0) continue;
    ImmaturePoint* impt = new ImmaturePoint(x, y, newFrame, selectionMap[i], &Hcalib);
}
```

### 2.3 未成熟点 (Immature Point) 的创建

`ImmaturePoint` 构造函数 (`src/FullSystem/ImmaturePoint.cpp:32-61`):

```cpp
ImmaturePoint::ImmaturePoint(int u_, int v_, FrameHessian* host_, float type, CalibHessian* HCalib)
    : u(u_), v(v_), host(host_), my_type(type), 
      idepth_min(0), idepth_max(NAN), lastTraceStatus(IPS_UNINITIALIZED) {

    for(int idx=0; idx<patternNum; idx++) {
        int dx = patternP[idx][0];  // 8-point pattern
        int dy = patternP[idx][1];
        Vec3f ptc = getInterpolatedElement33BiLin(host->dI, u+dx, v+dy, wG[0]);
        color[idx] = ptc[0];       // 保存 8 个 pattern 点的亮度
        
        // 累积梯度 Hessian: 用于后续极线搜索的可信度评估
        gradH += ptc.tail<2>() * ptc.tail<2>().transpose();
        
        // 计算权重: 梯度越大的像素, weight 越大
        weights[idx] = sqrtf(setting_outlierTHSumComponent / 
            (setting_outlierTHSumComponent + ptc.tail<2>().squaredNorm()));
    }
    
    energyTH = patternNum * setting_outlierTH;
    quality = 10000;  // 初始质量值
}
```

---

## 3. 滑动窗口光度误差优化 (Photometric Bundle Adjustment)

### 3.1 优化变量

DSO 的滑动窗口中优化的变量有 (以 `CPARS=4` + `8*frames` 维度表示):

| 变量 | 每帧维度 | 总计维度 | 源码位置 |
|------|---------|---------|---------|
| 相机内参 ($f_x, f_y, c_x, c_y$) | 4 (全局) | $CPARS=4$ | `HessianBlocks.h:300-305` |
| 帧位姿 (SE(3) manifold) | 6 | $6 \times N_f$ | `HessianBlocks.h:148-156` |
| 帧仿射亮度参数 $(a,b)$ | 2 | $2 \times N_f$ | `HessianBlocks.h:153` |
| **总计** | | $4 + 8N_f$ | `EnergyFunctional.cpp:438-443` |

### 3.2 点的参数化: 只优化逆深度

与传统 SLAM 优化 xyz 坐标不同, DSO 每个 3D 点只优化 **1 个参数: 逆深度 $d = 1/z$** (`HessianBlocks.h:402-459`):

```cpp
struct PointHessian {
    float u, v;               // host帧中的像素坐标 (固定,不优化)
    float idepth;             // 逆深度 (唯一需要优化的几何参数!)
    float idepth_scaled;      // SCALE_IDEPTH * idepth (数值缩放)
    float idepth_zero;        // 线性化点的逆深度 (FEJ)
    float idepth_hessian;     // 逆深度 Hessian (用于判定是否足够约束)
    
    float color[MAX_RES_PER_POINT];   // host帧中的pattern亮度 (固定)
    float weights[MAX_RES_PER_POINT]; // host帧像素权重 (固定)
    
    std::vector<PointFrameResidual*> residuals; // 该点的所有光度残差
};
```

**为什么只用逆深度?**
- 1 参数 vs 3 参数 (xyz) ⇒ 系统稀疏性更好, 边缘化更高效
- 在图像坐标中参数化更自然: 点的像素坐标 $(u,v)$ 来自 host 关键帧, 已知且固定
- 只需要估计逆深度 $d$ 即可唯一确定 3D 位置

### 3.3 能量函数结构 (Energy Functional)

DSO 使用因子图形式组织优化 (`EnergyFunctional.h:60-161`):

**三类残差:**
1. **Active residuals (A)**: 在最新线性化点被线性化且激活的残差
2. **Linearized residuals (L)**: 在旧的线性化点上被线性化 (即被 '固定' 在某 FEJ 点)
3. **Marginalization residuals (M)**: 之前被边缘化掉的信息

**系统矩阵组装** (`EnergyFunctional.cpp:775-913`):

```cpp
void EnergyFunctional::solveSystemF(int iteration, double lambda, CalibHessian* HCalib) {
    // 1. 累加 Active residuals → HA, bA
    accumulateAF_MT(HA_top, bA_top, multiThreading);
    
    // 2. 累加 Linearized residuals → HL, bL
    accumulateLF_MT(HL_top, bL_top, multiThreading);
    
    // 3. 累加 Schur complement (点的 Hessian) → H_sc, b_sc
    accumulateSCF_MT(H_sc, b_sc, multiThreading);
    
    // 4. 加入边缘化先验
    bM_top = bM + HM * getStitchedDeltaF();
    
    // 5. 构建最终系统:
    HFinal_top = HL_top + HM + HA_top - H_sc;
    bFinal_top = bL_top + bM_top + bA_top - b_sc;
    
    // 6. LDLT 求解 (带 preconditioning)
    x = SVecI.asDiagonal() * HFinalScaled.ldlt().solve(...);
    
    // 7. 回代: 更新帧参数 → 更新点逆深度
    resubstituteF_MT(x, HCalib, multiThreading);
}
```

### 3.4 点优化: 独立的 GN 优化

在 `FullSystem::optimizeImmaturePoint()` (`src/FullSystem/FullSystemOptPoint.cpp:51-205`):

```cpp
// 对每个帧 (除了 host) 计算残差
for(FrameHessian* fh : frameHessians) {
    float energy = point->linearizeResidual(HCalib, 1, residuals[i], Hdd, bd, currentIdepth);
}

// Gauss-Newton: H * step = -b, step = b/H, 直接闭环
for(int iteration=0; iteration<setting_GNItsOnPointActivation; iteration++) {
    float H = lastHdd * (1+lambda);
    float step = (1.0/H) * lastbd;
    float newIdepth = currentIdepth - step;
    
    // 在新点处线性化
    float newEnergy = linearizeAt(newIdepth, ...);
    
    if(newEnergy < lastEnergy) {
        currentIdepth = newIdepth;
        lambda *= 0.5;    // LM damping: 接近最优, 减小damping
    } else {
        lambda *= 5;      // 远离最优, 增大damping
    }
}
```

### 3.5 Schur Complement (边缘化点)

DSO 利用每个点只连接 host+target 帧的特性, 通过 Schur complement 消去点变量 (`AccumulatedSCHessian.cpp`):

$$\begin{bmatrix} H_{ff} & H_{fp} \\ H_{pf} & H_{pp} \end{bmatrix} \begin{bmatrix} \Delta x_f \\ \Delta x_p \end{bmatrix} = \begin{bmatrix} b_f \\ b_p \end{bmatrix}$$

消去点后: $(H_{ff} - H_{fp} H_{pp}^{-1} H_{pf}) \Delta x_f = b_f - H_{fp} H_{pp}^{-1} b_p$

其中 $H_{pp}$ 是1×1 矩阵 (因为每个点只有逆深度一个变量!), 这个求逆几乎为零开销!

### 3.6 残差线性化 (Jacobian 计算)

在 `src/FullSystem/Residuals.cpp:78-274`, `PointFrameResidual::linearize()`:

**核心 Jacobian 计算 (lines 100-157):**

```cpp
// 1. 投影点: p' = KRK^{-1}p + Kt*d
// 2. 计算 ∂u/∂ξ (2×6 Jacobian for pose):
d_xi_x[0] = new_idepth*HCalib->fxl();      // ∂u/∂tx
d_xi_x[1] = 0;
d_xi_x[2] = -new_idepth*u*HCalib->fxl();   // ∂u/∂tz
d_xi_x[3] = -u*v*HCalib->fxl();            // ∂u/∂rx
d_xi_x[4] = (1+u*u)*HCalib->fxl();         // ∂u/∂ry
d_xi_x[5] = -v*HCalib->fxl();              // ∂u/∂rz

// 3. 计算 ∂u/∂d (逆深度导数):
d_d_x = drescale * (PRE_tTll_0[0]-PRE_tTll_0[2]*u)*SCALE_IDEPTH*HCalib->fxl();
d_d_y = drescale * (PRE_tTll_0[1]-PRE_tTll_0[2]*v)*SCALE_IDEPTH*HCalib->fyl();

// 4. 光度 Jacobian (链式法则):
//    ∂r/∂ξ = (∂I/∂u * ∂u/∂ξ + ∂I/∂v * ∂v/∂ξ)
//    ∂r/∂d = (∂I/∂u * ∂u/∂d + ∂I/∂v * ∂v/∂d)  + affine
```

**鲁棒权重 (Huber)** (`Residuals.cpp:208-209`):
```cpp
float w = sqrtf(setting_outlierTHSumComponent / ...);
w = 0.5f*(w + weights[idx]);  // 组合梯度权重 + 光度权重

float hw = fabsf(residual) < setting_huberTH ? 1 : setting_huberTH / fabsf(residual);
energyLeft += w*w*hw * residual*residual*(2-hw);
```

### 3.7 LM 优化 (Levenberg-Marquardt)

在 `FullSystem::optimize()` (`src/FullSystem/FullSystemOptimize.cpp:408-602`):

```cpp
float optimize(int mnumOptIts) {
    double lambda = 1e-1;
    float stepsize = 1;
    
    for(int iteration=0; iteration<mnumOptIts; iteration++) {
        backupState(iteration!=0);
        solveSystem(iteration, lambda);
        Vec3 newEnergy = linearizeAll(false);
        
        // 判断接受/拒绝
        if(newTotalEnergy < lastTotalEnergy) {
            // 接受: 减小 lambda, 更像 GN
            acceptUpdate();
            lambda *= 0.25;
            stepsize *= 1.5;
        } else {
            // 拒绝: 增大 lambda, 更像 GD
            loadStateBackup();
            lambda *= 100;
            stepsize *= 0.5;
        }
        
        // 收敛判断: 步长足够小
        if(canbreak && iteration >= setting_minOptIterations) break;
    }
}
```

**步长收敛标准** (`FullSystemOptimize.cpp:216-309`):

```cpp
bool doStepFromBackup(float stepfacC, ...) {
    // 归一化步长:
    float meanStepT = sqrtf(sumT_step) / numFrames;
    float meanStepR = sqrtf(sumR_step) / numFrames;
    
    // 转换到实际物理量 (乘以标定因子)
    return meanStepT < threshold_T && meanStepR < threshold_R;
}
```

### 3.8 Nullspace 处理 (零空间正交化)

DSO 识别出光度 BA 的零空间维度 (`FullSystemOptimize.cpp:658-712`):

- **7 DOF**: 6 DoF 位姿 (全局SE3变换) + 1 DoF 尺度 (尺度模糊)
  - DSO (单目) ⇒ 7 个零空间向量
- **通过 `orthogonalize()`** (`EnergyFunctional.cpp:718-772`) 将 Hessian 投影到零空间的正交补空间, 防止数值漂移

---

## 4. 关键帧管理与边缘化

### 4.1 前端跟踪 (Coarse Tracking)

`CoarseTracker::trackNewestCoarse()` (`src/FullSystem/CoarseTracker.cpp:536-718`):

**算法流程:**
1. 从最粗金字塔层开始 (L=4或5)
2. 每个层: LM优化 6 DoF位姿 + 2个仿射参数 (共8D)
3. 逐层下传导位姿估计
4. 如果最细层残差 > `minResForAbort[lvl]*1.5`, 则返回失败

**残差计算** (`CoarseTracker.cpp:358-514`):
```cpp
Vec6 calcRes(int lvl, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH) {
    // 逐点投影: p' = RKi * Vec3f(x,y,1) + t*id
    for(int i=0; i<nl; i++) {
        Vec3f pt = RKi * Vec3f(x, y, 1) + t*id;
        u = pt[0] / pt[2];  v = pt[1] / pt[2];
        
        // 计算残差
        Vec3f hitColor = getInterpolatedElement33(dINewl, Ku, Kv, wl);
        float residual = hitColor[0] - (affLL[0] * refColor + affLL[1]);
        float hw = fabs(residual) < huberTH ? 1 : huberTH / fabs(residual);
        E += hw*residual*residual*(2-hw);
    }
}
```

**多种初始化尝试** (`FullSystem.cpp:286-343`): 恒速模型、半速、恒零、旋转扰动等 20+种初始化策略。

### 4.2 新关键帧的判定

在 `addActiveFrame()` (`FullSystem.cpp:869-888`):

```cpp
needToMakeKF = allFrameHistory.size()==1 ||
    // 平移 + 旋转 + 仿射亮度的加权组合
    setting_kfGlobalWeight*setting_maxShiftWeightT * sqrtf(tres[1])/(wG[0]+hG[0]) +
    setting_kfGlobalWeight*setting_maxShiftWeightR * sqrtf(tres[2])/(wG[0]+hG[0]) +
    setting_kfGlobalWeight*setting_maxShiftWeightRT* sqrtf(tres[3])/(wG[0]+hG[0]) +
    // 仿射亮度变化 (曝光/增益变化)
    setting_kfGlobalWeight*setting_maxAffineWeight * fabs(logf(refToFh[0])) > 1 ||
    // 或 残差增大到初值的2倍
    2*coarseTracker->firstCoarseRMSE < tres[0];
```

### 4.3 关键帧添加的完整流程

`makeKeyFrame()` (`src/FullSystem/FullSystem.cpp:1041-1196`):

```
1. traceNewCoarse()     → 所有未成熟点在新帧中做极线搜索
2. flagForMarginalization() → 选择哪些旧帧需要被边缘化
3. insertFrame()        → 添加到滑动窗口, 更新 Hessian/HM/bM
4. insertResidual()     → 为所有活跃点创建到新帧的残差
5. activatePointsMT()   → 选择足够约束的未成熟点→转化为点 Hessian
6. EF->makeIDX()        → 建立索引
7. optimize()           → GN/LM 优化 (5-15 次迭代)
8. removeOutliers()     → 移除 outlier 残差
9. flagPointsForRemoval() → 标记需要边缘化的点 (OOB/约束太弱)
10. ef->marginalizePointsF() → 边缘化被标记的点
11. makeNewTraces()     → 在新关键帧选新像素, 创建新的 ImmaturePoint
12. marginalizeFrame()  → 边缘化被标记的旧关键帧
```

### 4.4 帧边缘化的帧选择策略

`flagFramesForMarginalization()` (`FullSystemMarginalize.cpp:58-147`):

```cpp
// 策略1: 点不够的帧 → 标记边缘化
for(FrameHessian* fh : frameHessians) {
    int in = fh->pointHessians.size() + fh->immaturePoints.size();
    int out = fh->pointHessiansMarginalized.size() + fh->pointHessiansOut.size();
    
    if(in < setting_minPointsRemaining * (in+out) && 
       framesLeft > setting_minFrames) {
        fh->flaggedForMarginalization = true;
    }
}

// 策略2: 帧数超过 maxFrames 时, 选择"最接近其他帧"的帧
if(framesLeft >= setting_maxFrames) {
    double smallestScore = 1;
    for(FrameHessian* fh : frameHessians) {
        double distScore = 0;
        for(FrameFramePrecalc &ffh : fh->targetPrecalc) {
            distScore += 1/(1e-5 + ffh.distanceLL);  // 距离越小, score越大
        }
        distScore *= -sqrtf(fh->targetPrecalc.back().distanceLL);
        if(distScore < smallestScore) { // 选择"最近"的帧
            toMarginalize = fh;
        }
    }
}
```

**关键洞察**: 与 ORB-SLAM3 等间接法不同, DSO 不是只保留"最远"的帧, 而是边缘化"最近"的帧。这是因为直接法需要足够的视差来约束逆深度, 而非常接近的帧视差小, 提供的深度信息少。

### 4.5 边缘化的数学实现

`EnergyFunctional::marginalizeFrame()` (`src/OptimizationBackend/EnergyFunctional.cpp:498-610`):

```cpp
// 1. 将待边缘化帧移到 Hessian 的最后 8 行/列
swapRowsAndCols(fh->idx, last);

// 2. 先验加到待边缘化帧
HM.bottomRightCorner<8,8>().diagonal() += fh->prior;
bM.tail<8>() += fh->prior.cwiseProduct(fh->delta_prior);

// 3. Preconditioning: S = sqrt(|H_ii| + 10)
VecX SVec = (HM.diagonal().abs()+10).csqrt();

// 4. Schur Complement:
//    H̃ = H_aa - H_am * H_mm^{-1} * H_ma
//    (aa = kept, mm = marginalized)
Mat88 hpi = HMScaled.bottomRightCorner<8,8>().inverse();
HMScaled.topLeft -= bli * HMScaled.bottomLeft;
bMScaled.head -= bli * bMScaled.tail;

// 5. 对称化 (数值稳定性):
HM = 0.5*(HMScaled.topLeft + HMScaled.topLeft.transpose());
```

---

## 5. 与 LeGO-LOAM 原始论文的差异

DSO 本身**不是** LeGO-LOAM 的一部分。这里纠正一个常见误解:
- **DSO** = Direct Sparse Odometry (纯视觉直接法 VO)
- **LeGO-LOAM** = Lightweight Ground-Optimized LOAM (LiDAR-only 间接法)

它们是两个完全不同的算法。DSO 的作者是 Jakob Engel, LeGO-LOAM 的作者是 Tixiao Shan。

DSO 代码与论文的主要差异:
1. **论文中提到的 stereo 模式在代码中未完全公开** - 单目模式是唯一开源的
2. **代码中的像素选择比论文更复杂** - 有 3 个层次的梯度阈值 (type=1,2,3)
3. **论文中的全局 pose graph 优化在代码中简化** - 只有局部窗口优化
4. **光度标定的在线优化** - 论文提到但不强调; 代码中 `CalibHessian` 的优化是个核心特性

---

## 6. 优缺点 + 对 SLAM 算法的意义

### 6.1 优点

| 优点 | 技术原理 | 代码证据 |
|------|---------|---------|
| 高精度 | 稠密光度信息利用, 点数量可达 2000/帧 | `activatePointsMT()` 中选择 `desiredPointDensity` 个点 |
| 纯视觉 | 不需要 LiDAR/IMU, 相机即可 | 整个 DSO 只有视觉输入 |
| 光度鲁棒 | 显式仿射模型 ($a,b$) + 光度标定 | `AffLight`, `CalibHessian::B/Binv` |
| 高效率 | Schur complement 只需 1D 求逆 | 每个点只有逆深度变量, $H_{pp}$ 是标量 |
| 抗遮挡 | 滑动窗口 + 在线 outlier 剔除 | `flagPointsForRemoval()` 持续监测残差 |
| 动态曝光 | 在线估计每一帧的曝光/增益 | `AffLight::fromToVecExposure()` |

### 6.2 缺点

| 缺点 | 原因 | 后果 |
|------|------|------|
| 光照变化敏感 | 虽然建模仿射参数, 但光源变化仍影响 | 室内开灯/关灯场景失效 |
| 运动模糊敏感 | 需每帧都清晰可跟踪 | 快速旋转/手抖动时丢失 |
| 初始化困难 | 纯光度初始化需要足够视差+纹理 | `trackFrame()` 可能需要多帧才 SNAP |
| 无回环检测 | 滑动窗口无长期约束 | 长时间运行漂移累积不可逆 |
| 几何退化 | 均匀纹理/白墙无梯度 | 只有 0 个有效点可选 |
| 尺度不可观 | 单目 VO 尺度模糊 | 轨迹单位为任意单位 |

### 6.3 对 SLAM 算法的意义

**DSO 的核心贡献是证明了"直接法+稀疏"的可行性:**

1. **颠覆了"直接法=稠密法"的刻板印象** - DSO 证明了只需要几千个高梯度点的光度残差就能达到甚至超过间接法的精度
2. **开辟了直接法 VIO 方向** - 后来的 DM-VIO、DSO with IMU、Basalt 等都是基于 DSO 的光度误差框架
3. **First-Estimate Jacobians (FEJ) 在视觉 SLAM 的实践** - `idepth_zero` 固定线性化点, 保证了边缘化的理论一致性
4. **高效的 Schur complement 实现** - 1D 点参数化使得消去点变量极其高效

---

## 7. 数据管线

### 7.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| 灰度相机 (单目) | 20-30 Hz (数据集驱动) | `MinimalImageB3` (8bit, 3ch) 或 `ImageAndExposure` | `ImageFolderReader::getImage()` | `FullSystem` |
| 光度标定文件 | 一次性加载 | 256-bin LUT (`float calib[256]`) | `IOWrap::readPhotometricCalibration()` | `CalibHessian` |
| 曝光时间 | 每帧 (可选, Euroc) | `float exposure_time` | `ImageAndExposure` 结构体 | `FrameHessian` |

### 7.2 灰度相机完整管线

#### 原始数据
- **规格**: 640×480 (Euroc), 8bit Bayer→灰度, 由 `ImageRW_OpenCV::readImage()` 读取
- **标定来源**: `camera.txt` (fx, fy, cx, cy, k1, k2, p1, p2) → `CalibHessian` 构造时初始化, 并在线优化
- **光度标定**: `pcalib.txt` 提供响应函数 `G: [0,255] → [0,255]` 的 256-bin 查询表

#### 预处理
**Step 1 — 光度校正** (`HessianBlocks.cpp:184-188`):
```
每个像素 I_raw 通过响应函数 G 映射: I_rad = G^{-1}(I_raw)
实际代码: float gw = HCalib->getBGradOnly(color); dabs_l[idx] *= gw*gw
```
只在 `setting_gammaWeightsPixelSelect==1` 时启用, 影响梯度权重而非像素值本身。

**Step 2 — 图像金字塔构建** (`FrameHessian::makeImages()`, `HessianBlocks.cpp:128-191`):
| 层级 | 分辨率 | 操作 |
|------|--------|------|
| Level 0 | 640×480 | 原始分辨率, 存储 `(I, ∇I_x, ∇I_y)` |
| Level 1-5 | 逐层 1/2 | `cv::pyrDown()` 降采样, 重新计算梯度 |
- 每层存储 `Eigen::Vector3f dI[N]` 数组: `[0]=亮度, [1]=∂I/∂x, [2]=∂I/∂y`
- 梯度用中心差分: `dx = 0.5*(I[idx+1] - I[idx-1])`, `dy = 0.5*(I[idx+wl] - I[idx-wl])`
- `absSquaredGrad[N]` 存储 `dx²+dy²` 用于像素选择

**Step 3 — 像素选择** (`PixelSelector::makeMaps()`, `FullSystem.cpp:1286`):
```
算法: 网格采样 (gridMaxSelection) + 自适应密度调节
参数: minUseGrad_pixsel=10 (绝对梯度阈值)
       desiredImmatureDensity ≈ 2000 (目标未成熟点数)
       sparsityFactor: 自适应网格大小 (初始1, 动态调整)
```
- 将图像划分为 `pot × pot` 网格 (pot=sparsityFactor)
- 每格选4个方向梯度最强的像素 (x方向、y方向、x+y方向、x-y方向)
- 自适应调节网格大小, 使选中点数接近 `desiredImmatureDensity`
- **选点类型分级** (selectionMap值):
  - `type=0`: 未选中
  - `type=1/2/3`: 不同梯度级别 (代码中 `my_type` 字段, 对应 `makeMaps` 中的 `THFac` 调节)

#### 特征提取
**DSO 无传统特征提取**。像素选择器选出高梯度点后, 直接构造 `ImmaturePoint`:

**ImmaturePoint 初始化** (`ImmaturePoint::ImmaturePoint()`, `ImmaturePoint.cpp:32-61`):
```
对每个选中像素 (u,v):
  1. 提取 8-point pattern 亮度和梯度 (固定圆形 pattern, patternP[8])
  2. 计算 pattern Hessian: gradH = Σ(∇I_i · ∇I_i^T) (2x2 矩阵)
  3. 计算逐点权重: weights[idx] = sqrt(TH/(TH + ||∇I||²))
     其中 TH = setting_outlierTHSumComponent (默认 50)
  4. 能量阈值: energyTH = 8 × setting_outlierTH (默认 12)
  5. 初始逆深度: idepth_min=0, idepth_max=NAN (待极线搜索确定)
```

#### 匹配与异常值剔除
**未成熟点 → 成熟点的极线搜索** (`ImmaturePoint::traceOn()`, `ImmaturePoint.cpp`):
```
1. 在 target 帧沿极线搜索 (基于逆深度范围 [idepth_min, idepth_max])
2. 对每个候选逆深度 d_k:
   投影 pattern 点: p'_k = π(K·(R·K⁻¹·p + t·d_k))
   计算光度残差: r_k = Σ w_i·||I(p'_k + dp_i) - (e^a·I_i + b)||
3. 选择最佳逆深度 + 不确定性估计 (基于 Hessian 二阶近似)
4. 若 uncertainty < threshold → 转为 PointHessian (成熟点)
```

**异常值剔除 — 两级机制**:
1. **在线 outlier 剔除** (`FullSystem.cpp:flagPointsForRemoval()`):
   - 残差 > `setting_outlierTH * setting_outlierTHSumComponent` → 标记删除
   - 点在图像边界外 (OOB) → 标记删除
   - 逆深度 Hessian < `setting_minIdepthH_marg` → 约束不足, 标记删除

2. **Huber 鲁棒核** (`Residuals.cpp:208-209`):
   ```cpp
   float hw = fabs(residual) < setting_huberTH ? 1 : setting_huberTH / fabs(residual);
   // 默认 setting_huberTH = 9
   ```

#### 算法消费 — 因子构建
**PointFrameResidual** 是 DSO 的最小残差单元 (`Residuals.cpp:78-274`):
```
残差公式: r = I_target(π(p')) - exp(a_target - a_source)·I_source(p) - (b_target - b_source)

雅可比 (链式法则):
  ∂r/∂ξ_target  = ∇I_target · ∂π/∂X' · ∂X'/∂ξ_target      (2×6)
  ∂r/∂ξ_source  = -∇I_target · ∂π/∂X' · Adj_G(source→target)·∂X_cam/∂ξ_source  (2×6)
  ∂r/∂idepth    = ∇I_target · ∂π/∂X' · ∂X'/∂d · SCALE_IDEPTH  (2×1)
  ∂r/∂a, ∂r/∂b  = [I_source, 1]  (2×2 仿射雅可比)

信息矩阵: Σ⁻¹ = w²·hw·I (对角), w = pixel_weight · gradient_weight
```

**EnergyFunctional 三类残差组合** (`EnergyFunctional.cpp:775-913`):
```
H_final = H_Active + H_Linearized + H_Marginalization - H_Schur
b_final = b_Active + b_Linearized + b_Marginalization - b_Schur
```

### 7.3 跨帧协同

#### 初始化管线
```
非关键帧 (tracking): 恒速模型 → CoarseTracker (金字塔 LM, 8D) → 位姿
关键帧 (activate): makeKeyFrame() 12 步流程 (见 4.3)
```
**CoarseTracker 层间传递** (`CoarseTracker.cpp:536-718`):
- L=5→4→3→2→1→0 逐层优化, 每层将最优位姿传给下一层
- 若最细层残差 > `minResForAbort[lvl]×1.5` → tracking 失败

**降级策略** (`FullSystem.cpp:286-343`):
- 主尝试: 恒速模型 (20+ 种变体: 正/反向, 旋转扰动等)
- 兜底: 直接用上一帧位姿 (恒静止模型)
- tracking 失败超过阈值 → 系统丢失, 重新初始化

#### 数据缓冲
- 帧间无缓冲, 流水线处理 (每帧 `addActiveFrame()` 阻塞等待结果)
- 历史帧 `allFrameHistory` 维护完整轨迹, 但仅滑动窗口 (最多7帧) 参与优化
- IMU 数据不支持 (DSO 为纯视觉)

---


## 8. 技术路线对比 (与其他算法)

| 特性 | DSO | ORB-SLAM3 | DM-VIO |
|------|-----|-----------|--------|
| 前端类型 | 直接法 (光度残差) | 间接法 (重投影) | 直接法 (光度残差) |
| 传感器 | 单目 | 单目/双目/IMU | 单目+IMU |
| 点参数化 | 逆深度 (1D) | xyz (3D) | 逆深度 (1D) |
| 地图 | 稀疏 (2000点/帧) | 稀疏 (几百点/帧) | 稀疏 (2000点/帧) |
| 边缘化 | Schur complement (手动) | G2O | GTSAM (延迟边缘化) |
| 回环检测 | ✗ | ✓ (DBoW2) | ✗ |
| 光度标定 | ✓ (在线优化) | ✗ | ✓ (在线优化) |
| 初始化 | 直接法初始化 | 两帧H矩阵 | 直接法初始化 + IMU初始化 |

**DSO 的历史转折意义**: 它出现在 2016-2018 年间, 当时 SLAM 领域主流是间接法。DSO 用源代码证明了直接法不仅可行, 而且在很多场景下比间接法更精确。这种"范式转移"影响了后续一大批 VIO 系统 (DM-VIO, VINS-Direct) 的设计理念。