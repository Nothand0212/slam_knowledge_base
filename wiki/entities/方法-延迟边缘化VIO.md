---
tags: [VIO, 延迟边缘化, DM-VIO, 滑动窗口, 因子图, GTSAM, FEJ, DSO]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp
  - raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.h
  - raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp
  - raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.h
  - raw/codes/dm-vio/src/GTSAMIntegration/FEJValues.h
  - raw/codes/dm-vio/src/IMU/BAIMULogic.cpp
  - raw/codes/dm-vio/src/IMU/CoarseIMULogic.h
  - raw/codes/dm-vio/src/GTSAMIntegration/BAGTSAMIntegration.h
---

# 延迟边缘化 VIO（DM-VIO）

> DM-VIO 是一种基于 DSO 直接法前端 + GTSAM 因子图后端的视觉惯性里程计。核心创新是**延迟边缘化（Delayed Marginalization）**：在滑动窗口中延迟旧关键帧的边缘化时机，允许 IMU 初始化有机会获得更多约束后再执行 Schur 补，从而避免"信息永久锁入"导致的精度损失。

## 动机：标准边缘化的问题

在传统滑动窗口 VIO（如 VINS-Fusion）中，当新 KF 加入时立即边缘化最老帧，通过 Schur 补将老帧的信息压缩为先验因子：

```pseudo
新KF到达 → 即时边缘化最老帧 → Schur补 → 线性先验因子接入
```

**关键缺陷**：边缘化是**不可逆信息压缩**。如果在 IMU 初始化尚未完成（scale 未知）时就边缘化了老帧，该帧的视觉信息会以错误的 scale 被永久注入先验，后续即使 IMU 完成初始化也无法纠正。

## 核心思想：滞后队列

DM-VIO 维护多个 `DelayedGraph`，每个 graph 有一个 `delayN` 参数表示当前保留但未真正执行 Schur 补的帧数量。基本图（Main Graph，delay=0）立即执行标准边缘化；延迟图（delay>0）暂时积累 KF 和边缘化命令，直到 IMU 初始化成功后再一次性"readvance"，以正确的 metric scale 重新执行 Schur 补。

```
┌─────────────────────────────────────────────────────┐
│  DelayedMarginalizationGraphs                       │
│  ┌──────────────┐  ┌──────────────┐                 │
│  │ Main Graph   │  │ Delayed Graph │  (delay=8)     │
│  │ (delay=0)    │  │ delayN = 8    │                 │
│  │ 即时边缘化   │  │ 暂缓边缘化   │                 │
│  └──────────────┘  └──────────────┘                 │
│  添加到 DSO BA 优化      等待 IMU 初始化后 readvance│
└─────────────────────────────────────────────────────┘
```

**源码锚点**: `raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.h:L114-L163`

### 数学形式

设原始因子图有 $n$ 个关键帧，每个 KF 产生一组待边缘化变量集 $K_i$。在即时边缘化方案中：

$$
p(\mathcal{X}) \propto \prod_{i=1}^n \text{margOut}\big(F_i, K_i\big)
$$

在延迟方案中，延迟 $d$ 帧执行边缘化：

$$
p(\mathcal{X}) \propto \prod_{i=1}^{n-d} F_i \cdot \text{margOut}\big(\prod_{j=n-d+1}^{n} F_j, \bigcup_{j} K_j\big)
$$

其中 $\text{margOut}$ 为 Schur 补操作。延迟的关键在于：在最终执行 Schur 补时，变量 $\mathcal{X}$ 已经包含了从 IMU 初始化得到的 metric scale 和重力方向，使得线性化点和边缘化信息都是正确的。

## DM-VIO 系统架构

### 两阶段优化（Two-Stage）

DM-VIO 继承 DSO 的两阶段设计并融合 IMU：

1. **Coarse Tracking（粗跟踪）**：每帧执行，估计当前帧与参考 KF 之间的相对位姿。CoarseIMULogic 通过 GTSAM 因子图将 IMU 预积分因子注入粗跟踪优化 (`raw/codes/dm-vio/src/IMU/CoarseIMULogic.h:L40-L90`)

2. **Refined BA（精 BA）**：BAIMULogic 将 IMU 因子（velocity、bias、scale、gravity direction）加入 DSO 后端联合优化 (`raw/codes/dm-vio/src/IMU/BAIMULogic.h:L66-L195`)

```
新帧 → CoarseTracker（DSO光度 + IMU预积分因子）
         ↓ 判断是否为KF
         ↓ KF=true
    Refined BA（DSO光度 + GTSAM IMU因子 + scale/gravity）
         ↓
    边缘化（BAGraphs→DelayedMarginalization）
         ↓
    老KF信息作为先验返回BA + CoarseTracker
```

**源码锚点**: `raw/codes/dm-vio/src/IMU/BAIMULogic.cpp:L235-L324` (addKeyframe 添加 IMU 因子)

### 因子分组（Factor Groups）

BAIMULogic 使用三层因子分组 (`raw/codes/dm-vio/src/IMU/BAIMULogic.h:L68-L75`)：

| Group | 名称 | 内容 |
|-------|------|------|
| 0 | `NO_IMU_GROUP` | DSO 纯视觉光度因子 |
| 1 | `BIAS_AND_PRIOR_GROUP` | bias 随机游走因子 + 先验因子 |
| 2 | `METRIC_GROUP` | IMU 预积分因子（PoseTransformationFactor 包装） |

### FEJ（First-Estimates Jacobian）

边缘化操作需要在正确的线性化点执行。DM-VIO 通过 `FEJValues` 类管理所有与边缘化因子相连的变量的 FEJ 值：

- **DSO 变量**（位姿 `p*`、仿射亮度 `a*`）：使用 DSO 提供的 evalValues（FEJ 点）
- **IMU 变量**（速度 `v*`、偏置 `b*`、scale `s0`、重力 `g0`）：首次与边缘化因子连接时的当前值

**源码锚点**: `raw/codes/dm-vio/src/GTSAMIntegration/FEJValues.h:L38-L91`

## Schur 补边缘化实现

核心实现在 `marginalizeOut()` (`raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp:L30-L90`)：

1. `extractKeysToMarginalize`：将因子图拆分为待边缘化因子（含被边缘化 key）和保留因子
2. 对保留图上被边缘化变量的的 connected keys 调用回调（用于插入 FEJ 值）
3. 将待边缘化因子在当前值 $\mathbf{x}_0$ 处线性化：$\mathbf{H}\delta\mathbf{x} = \mathbf{b}$
4. 构造增广 Hessian，按 ordering（先被边缘化变量，后保留变量）分块：

$$
\begin{bmatrix} \mathbf{H}_{mm} & \mathbf{H}_{ma} \\ \mathbf{H}_{am} & \mathbf{H}_{aa} \end{bmatrix}
\begin{bmatrix} \delta\mathbf{x}_m \\ \delta\mathbf{x}_a \end{bmatrix} =
\begin{bmatrix} \mathbf{b}_m \\ \mathbf{b}_a \end{bmatrix}
$$

5. Schur 补 + preconditioning (`raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp:L140-L180`)：

```cpp
// Preconditioning like in DSO
gtsam::Vector SVec = (pair.first.diagonal().cwiseAbs() +
    gtsam::Vector::Constant(pair.first.cols(), 10)).cwiseSqrt();
gtsam::Vector SVecI = SVec.cwiseInverse();

gtsam::Matrix hessianScaled = SVecI.asDiagonal() * pair.first * SVecI.asDiagonal();
gtsam::Vector bScaled = SVecI.asDiagonal() * pair.second;

gtsam::Matrix Hmm = hessianScaled.block(0, 0, mSize, mSize);
gtsam::Matrix Hma = hessianScaled.block(0, mSize, mSize, aSize);
gtsam::Matrix Haa = hessianScaled.block(mSize, mSize, aSize, aSize);

// Schur complement
gtsam::Matrix HmmInv = Hmm.completeOrthogonalDecomposition().pseudoInverse();
gtsam::Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
gtsam::Vector baNew = ba - Hma.transpose() * HmmInv * bm;
```

6. 将结果封装为 `LinearContainerFactor`（即 `HessianFactor`），作为线性化先验加入新图

### 延迟图的 readvanceUntilDelay

当 `marginalizationOrder.size() > delayN` 时，从队列头部弹出最早的一批 key 执行真正的 Schur 补 (`raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp:L212-L253`)：

```cpp
void DelayedGraph::readvanceUntilDelay() {
    if(marginalizationPaused) return;
    while(marginalizationOrder.size() > delayN) {
        // 在边缘化前插入 FEJ 值
        fejValues->insertConnectedKeys(keysToMarg, delayedValues);
        // 执行真实 Schur 补
        auto connectedKeyCallback = [this](const gtsam::FastSet<gtsam::Key>& connectedKeys) {
            fejValues->insertConnectedKeys(connectedKeys, delayedValues);
        };
        graph = marginalizeOut(*graph, fejValues->fejValues, keysToMarg, connectedKeyCallback);
        // 清理被边缘化的 key
        marginalizationOrder.pop_front();
    }
}
```

### GTSAM 集成：两个 Values 的概念

DM-VIO 维护两组 Values：`baValues`（当前优化值）和 `baEvalValues`（FEJ 值）。BAGTSAMIntegration 拦截 DSO 的 Hessian 和梯度向量，用 GTSAM 因子产生增量 (`raw/codes/dm-vio/src/GTSAMIntegration/BAGTSAMIntegration.h:L126-L243`)。

## 与 VINS-Fusion 即时边缘化的对比

| 维度 | VINS-Fusion | DM-VIO |
|------|------------|--------|
| 边缘化时机 | 新 KF 到达时立即执行 | 可延迟 d 帧后执行 |
| IMU 初始化 | 独立松耦合初始化，完成后切换 | 滞后边缘化保护初始化期间的线性化点 |
| 滑窗大小 | 固定 10 帧 | 固定 7 帧（默认） |
| 边缘化实现 | 手写 Ceres 残差块 + Eigen Schur 补 | GTSAM 因子 + LinearContainerFactor |
| FEJ 处理 | 固定线性化点后不再更新 | FEJValues 精确控制每变量的线性化点 |
| Scale 处理 | 初始化后 fix scale | DSO→IMU 通过 PoseTransformationFactor 在线优化 scale |
| 后端框架 | Ceres Solver | GTSAM + DSO 原生 Hessian 拼接 |
| 先验传递 | 边缘化因子直接加入下一轮 Ceres | 先验可用于 Coarse 粗跟踪和 BA 精优化 |

## 动态 DSO 权重

DM-VIO 支持根据 RMSE 动态调节 DSO 光度因子的权重 (`raw/codes/dm-vio/src/IMU/BAIMULogic.cpp:L801-L807`)：

$$
w_{\text{DSO}} = \begin{cases}
1.0, & \text{if RMSE} < \text{thresh} \\
(\text{thresh}/\text{RMSE})^2, & \text{otherwise}
\end{cases}
$$

这相当于对高噪声帧自动降权，防止一个坏帧拖垮整个优化。

## 关键配置项

| 参数 | 含义 | 典型值 |
|------|------|--------|
| `delayN` | 主图的延迟帧数 | 0（即时） |
| `maxGroupInGraph` | 子图中最大因子组 | 2（全部 IMU 因子） |
| `setting_optScaleBA` | BA 中是否优化 scale | true |
| `setting_optGravity` | BA 中是否优化重力方向 | true |
| `setting_scaleFixTH` | scale 收敛阈值 | 0.01 |
| `weightDSOToGTSAM` | DSO 光度 vs GTSAM IMU 的权重比 | 1.0 |

---

## Agent 实现提示

### 适用场景

- 需要视觉+IMU 紧耦合的单目/双目 VIO，且 IMU 初始化在系统早期阶段完成
- 对滑窗线性化点精度敏感的系统（长走廊、纯旋转、低激励场景）
- 因子图后端 + 直接法前端组合的系统设计

### 输入输出契约

- **输入**：图像帧序列（需光度标定）、IMU 测量（陀螺仪+加速度计）、相机-IMU 外参、GTSAM 预积分参数
- **输出**：6-DoF 位姿（world-to-camera）、速度（world frame, m/s）、IMU bias、metric scale、重力方向
- **坐标约定**：DSO 坐标系下 worldToCam 位姿（左乘），IMU 因子使用 imuToWorld 位姿（右乘），通过 `TransformDSOToIMU` 转换

### 实现骨架（伪代码）

```pseudo
class DelayedMarginalizationVIO:
    def __init__(self, delay_n, max_frames):
        self.main_graph = DelayedGraph(delay=0, max_group=ALL_FACTORS)
        self.delayed_graph = DelayedGraph(delay=delay_n, max_group=VISUAL_ONLY)
        self.fej_values = FEJValues()
        self.ba_values = Values()       # 当前值
        self.ba_eval_values = Values()  # FEJ 值

    def add_keyframe(self, kf_id, pose, imu_meas):
        # 1. 向两个图添加 IMU 因子
        imu_factor = PoseTransformationFactor(
            ImuFactor(prev_pose, prev_vel, curr_pose, curr_vel, bias, imu_meas),
            transform_dso_to_imu)
        self.main_graph.addFactor(imu_factor, group=METRIC_GROUP)
        self.delayed_graph.addFactor(imu_factor, group=METRIC_GROUP)
        # 2. 添加 bias random walk 因子
        bias_factor = BetweenFactor(prev_bias, curr_bias, zero, noise_model)
        self.main_graph.addFactor(bias_factor, group=BIAS_GROUP)
        # 3. 边缘化最老 KF
        keys_to_marg = [vel_oldest, bias_oldest, pose_oldest]
        self.main_graph.marginalize(keys_to_marg, self.ba_values)
        self.delayed_graph.marginalize(keys_to_marg, self.ba_values)
        # 4. 延迟图 readvance（如果 delayN 被满足）
        self.delayed_graph.readvanceUntilDelay()

    def on_imu_init(self, scale, gravity, velocity, bias):
        # IMU 初始化成功 → 立即 readvance 全部延迟图
        self.delayed_graph.readvanceGraph(new_delay=0)
        # 将延迟图结果替换主图
        self.replaceMainGraph(self.delayed_graph)

    def optimize(self):
        # 联合优化：DSO 光度 Hessian + GTSAM IMU 因子 Hessian
        H_dso, b_dso = DSO.computeHessian()
        H_imu, b_imu = self.main_graph.getHAndB(self.ba_eval_values)
        H_full = H_dso + H_imu
        b_full = b_dso + b_imu
        delta = solve(H_full, b_full)
        self.ba_values.applyUpdate(delta)
```

### 关键源码片段

**延迟边缘化核心循环** (`raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp:L212-L253`)：
```cpp
void DelayedGraph::readvanceUntilDelay() {
    if(marginalizationPaused) return;
    while(marginalizationOrder.size() > delayN) {
        auto&& keysToMarg = marginalizationOrder.front();
        fejValues->insertConnectedKeys(keysToMarg, delayedValues);
        auto connectedKeyCallback = [this](const auto& connectedKeys) {
            fejValues->insertConnectedKeys(connectedKeys, delayedValues);
        };
        if(!keysToMarg.empty()) {
            graph = marginalizeOut(*graph, fejValues->fejValues, keysToMarg,
                                   connectedKeyCallback);
            for(auto&& key : keysToMarg) {
                delayedCurrValues.erase(key);
                delayedValues.erase(key);
            }
            fejValues->keysRemoved(keysToMarg);
        }
        marginalizationOrder.pop_front();
    }
}
```

**Schur 补实现** (`raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp:L140-L180`)：
```cpp
gtsam::Matrix computeSchurComplement(const gtsam::Matrix& augmentedHessian,
                                      int mSize, int aSize) {
    auto pair = pairFromAugmentedHessian(augmentedHessian);
    gtsam::Vector SVec = (pair.first.diagonal().cwiseAbs() +
        gtsam::Vector::Constant(pair.first.cols(), 10)).cwiseSqrt();
    gtsam::Vector SVecI = SVec.cwiseInverse();
    gtsam::Matrix hessianScaled = SVecI.asDiagonal() * pair.first * SVecI.asDiagonal();
    gtsam::Vector bScaled = SVecI.asDiagonal() * pair.second;
    gtsam::Matrix Hmm = hessianScaled.block(0, 0, mSize, mSize);
    gtsam::Matrix Hma = hessianScaled.block(0, mSize, mSize, aSize);
    gtsam::Matrix Haa = hessianScaled.block(mSize, mSize, aSize, aSize);
    // ... Schur: HaaNew = Haa - Hma^T * Hmm^{-1} * Hma
}
```

### 实现注意事项

- delayN 须 ≤ 滑窗内的总 KF 数，否则永远不会真正边际化导致图无限增长
- FEJ 值在边缘化变量被连接到先验因子时必须已经固定，否则产生不一致；DSO 变量使用 DSO 提供的 evalValues，IMU 变量使用首次连接时的值
- `PoseTransformationFactor` 封装了 DSO→IMU 坐标转换，其雅可比通过 `JACOBIAN_FACTOR` 模式传播到 scale 和重力方向
- 延迟图与主图的 marginalizationOrder 必须同步（同一时刻 push 相同的 keysToMarginalize）
- scale 固定后不再优化（`scaleFixed=true`），此时 scale key 从 ordering 中移除

### 源码检索锚点

- `raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.h` — DelayedGraph 和 DelayedMarginalizationGraphs 类定义
- `raw/codes/dm-vio/src/GTSAMIntegration/DelayedMarginalization.cpp` — readvanceUntilDelay 和 marginalize 全流程
- `raw/codes/dm-vio/src/GTSAMIntegration/Marginalization.cpp` — Schur 补 + preconditioning
- `raw/codes/dm-vio/src/GTSAMIntegration/FEJValues.h` — FEJ 值管理
- `raw/codes/dm-vio/src/GTSAMIntegration/BAGTSAMIntegration.h` — GTSAM-DSO BA 集成接口
- `raw/codes/dm-vio/src/IMU/BAIMULogic.cpp` — IMU 因子构造 + 动态权重 + scale 固定逻辑
- `raw/codes/dm-vio/src/IMU/CoarseIMULogic.h` — Coarse 阶段 IMU 集成

## 相关页面

- [[方法-滑动窗口边缘化]]
- [[方法-直接法SLAM族]]
- [[方法-DSO稀疏直接法]]
- [[方法-VIO初始化方法]]
- [[方法-关键帧选择策略]]
