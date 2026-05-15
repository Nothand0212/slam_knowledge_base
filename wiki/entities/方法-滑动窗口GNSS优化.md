---
type: entity
tags: [GNSS, 滑动窗口, 批优化, OB_GINS, 边缘化, 固定整秒, IMU插值, Ceres, Schur补]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/OB_GINS/
  - raw/codes/IC-GVINS/
---

# 滑动窗口 GNSS 优化

> OB_GINS 的固定整秒节点滑动窗口批优化方法：以 1 秒为间隔构建优化节点，在窗口中联合优化 GNSS 位置因子和 IMU 预积分因子，窗口满时通过 Schur 边缘化移除最老节点。

## 一、方法概览

滑动窗口优化是离线 GNSS-INS 组合导航的核心计算策略。与全局批优化（一次性构建所有时间的全问题）相比，滑动窗口将优化规模限制在固定数量节点内，通过边缘化（marginalization）将历史信息浓缩为先验因子，从而在精度和计算效率之间取得平衡。

OB_GINS 的滑动窗口具有两个关键设计决策：
1. **固定整秒节点**：所有优化状态在整数 GPS 秒上，而非按传感器时间动态创建
2. **Schur 边缘化**：移除最老节点时，将其相关的 GNSS/IMU 因子通过 Schur 补压缩为线性先验

## 二、固定整秒节点采样

### 2.1 采样策略

OB_GINS 的核心时间组织方式：每 1.0 秒创建一个优化节点（`INTEGRATION_LENGTH = 1.0`），GNSS 观测必须对齐在整秒时刻（容差 ±1ms）。

```
时间轴:   0s      1s      2s      3s      4s      ...
          │───────│───────│───────│───────│
IMU:      0 ──┬── 1 ──┬── 2 ──┬── 3 ──┬── 4
             │       │       │       │
GNSS:        ●       ●       ●       ●       （整秒对齐）
```

当 IMU 数据的时间戳跨越整秒边界时，通过线性拆分将 IMU 增量按时间比例分割：

```pseudo
function imuInterpolation(imu01, imu00, imu11, mid):
    scale = (imu01.time - mid) / imu01.dt

    imu00.time   = mid
    imu00.dt     = imu01.dt - (imu01.time - mid)
    imu00.dtheta = imu01.dtheta * (1 - scale)
    imu00.dvel   = imu01.dvel   * (1 - scale)

    imu11.time   = imu01.time
    imu11.dt     = imu01.time - mid
    imu11.dtheta = imu01.dtheta * scale
    imu11.dvel   = imu01.dvel   * scale
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L577-L594`

### 2.2 临界情况处理

当 IMU 时间与整秒边界非常接近（< 0.1ms）时，不做插值，直接判断归属：

```pseudo
function isNeedInterpolation(imu0, imu1, mid):
    if imu0.time < mid and imu1.time > mid:
        if mid - imu0.time < 0.0001:   return -1  // 前IMU对齐，丢弃
        if imu1.time - mid < 0.0001:   return  1  // 后IMU对齐，跳过
        return 2  // 需要插值
    return 0
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L596-L621`

### 2.3 设计权衡

| 维度 | 固定整秒节点（OB_GINS） | 动态节点（IC-GVINS） |
|------|----------------------|---------------------|
| 实现复杂度 | 低（仅需线性插值） | 高（需要插入/删除/合并节点） |
| 时间分辨率 | 固定 1s，不可变 | 可变，按传感器到达时间 |
| 多传感器兼容 | GNSS+IMU 专配 | GNSS+相机+IMU 自然支持 |
| 适用场景 | 离线后处理 | 在线多传感器融合 |
| GNSS 观测利用率 | 丢失在非整秒时刻的观测 | 可对齐到最近节点 |

## 三、滑动窗口 Ceres 优化

### 3.1 窗口数据结构

OB_GINS 使用三个 deque 同步维护窗口内的数据：

```pseudo
preintegrationlist:  deque<PreintegrationBase>  // N 个预积分对象（= N+1 个节点间的 N 段）
statedatalist:       deque<IntegrationStateData> // N+1 个状态节点
gnsslist:            deque<GNSS>                 // N+1 个 GNSS 观测（与整秒对齐）
timelist:            deque<double>               // N+1 个整秒时刻
```

索引关系：`preintegrationlist[k]` 连接 `statedatalist[k]` 和 `statedatalist[k+1]`；`gnsslist[k]` 按时间对应 `timelist[k]` 时刻的 `statedatalist[k]`。

### 3.2 优化问题构建

每次新增一个整秒节点后，触发一次完整窗口优化：

```pseudo
function buildOptimization(window):
    problem = CeresProblem(fast_removal=true)

    // 1. 注册参数块 (N+1 个节点)
    for k in [0..N]:
        problem.addParameter(pose[k], PoseManifold)  // 7 dim → 6 dof
        problem.addParameter(mix[k], 9 dim)           // v[3] + bg[3] + ba[3]

    // 2. GNSS 位置因子 (N+1 个)
    for k, gnss in enumerate(gnsslist):
        factor = GnssFactor(gnss, lever_arm)
        problem.addResidual(factor, HuberLoss(1.0), pose[k])

    // 3. IMU 预积分因子 (N 个)
    for k in [0..N-1]:
        factor = PreintegrationFactor(preint_list[k])
        // 参数块: pose[k], mix[k], pose[k+1], mix[k+1]
        problem.addResidual(factor, nullptr, 4 param blocks)

    // 4. IMU 零偏先验因子 (1 个，仅最新节点)
    factor = ImuErrorFactor(preint_list[-1])
    problem.addResidual(factor, nullptr, mix[-1])

    // 5. 边缘化先验因子 (来自上一窗口)
    if prior_info.isValid():
        factor = MarginalizationFactor(prior_info)
        problem.addResidual(factor, nullptr, prior_param_blocks)

    return problem
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L295-L355`

### 3.3 两轮优化 + 粗差剔除

OB_GINS 采用与 IC-GVINS 相同的两轮优化策略：

**第一轮**：
- LM 求解，`max_num_iterations = 总迭代次数的 1/4`
- GNSS 因子使用 `HuberLoss(1.0)` 鲁棒核降权粗差
- 求解后对每个 GNSS 残差做 Chi² 检验：$\chi^2 = 2 \cdot cost$
- 当 $\chi^2 > 7.815$（$\chi^2(3)$ 95% 置信度），判定为粗差，重加权 `std *= sqrt(chi2/7.815)`
- 移除所有带核函数的 GNSS 因子

**第二轮**：
- LM 求解，`max_num_iterations = 总迭代次数的 3/4`
- GNSS 因子不带鲁棒核（粗差已被降权或剔除）
- 此轮负责精化状态

```pseudo
function twoStepOptimization(problem, gnss_ids, num_iter):
    // Round 1: Huber + cull
    options.max_num_iterations = num_iter / 4
    solver.Solve(problem, options)

    for k, (time, id) in enumerate(gnss_ids):
        chi2 = 2 * problem.EvaluateResidualBlock(id).cost
        if chi2 > 7.815:
            gnsslist[k].std *= sqrt(chi2 / 7.815)

    for id in gnss_ids:
        problem.RemoveResidualBlock(id)  // Remove with loss

    // Re-add without loss
    for k, gnss in enumerate(gnsslist):
        factor = GnssFactor(gnss, lever)
        problem.addResidual(factor, nullptr, pose[idx(gnss.time)])

    // Round 2: refine
    options.max_num_iterations = num_iter * 3 / 4
    solver.Solve(problem, options)
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L357-L417`

### 3.4 Ceres 配置参数

```yaml
# OB_GINS YAML 配置示例
windows: 30              # 窗口节点数 (30 秒)
starttime: 0             # 起始时间 (s)
endtime: 3600            # 结束时间 (s)
num_iterations: 20       # 总迭代次数
is_outlier_culling: true # 是否做GNSS粗差检测
isearth: true            # 是否考虑地球自转
```

优化器配置：
```cpp
options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
options.num_threads                = 4;
```

## 四、Schur 边缘化

### 4.1 边缘化触发时机

当窗口内节点数达到配置的 `windows` 值（默认 30）时，对最老的状态节点执行边缘化：

```pseudo
function marginalizeOldest(window, prior_info):
    margin_info = MarginalizationInfo()

    // 1. 添加上一轮边缘化的先验因子
    if prior_info.isValid():
        for k, block in enumerate(prior_blocks):
            if block == state[0].pose or block == state[0].mix:
                margin_idx.append(k)  // 标记待边缘化参数
        margin_info.addResidual(prior_factor, margin_idx)

    // 2. 添加第一个预积分因子 (连接 state[0] 和 state[1])
    factor = PreintegrationFactor(preint_list[0])
    margin_info.addResidual(factor, [0, 1])  // 边缘化 state[0] 的参数

    // 3. 添加第一个 GNSS 因子 (如果时间对齐)
    if |timelist[0] - gnsslist[0].time| < 1ms:
        factor = GnssFactor(gnsslist[0], lever)
        margin_info.addResidual(factor, [])

    // 4. 执行 Schur 边缘化
    margin_info.marginalization()
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L430-L487`

### 4.2 Schur 补数学原理

设状态向量划分为待边缘化部分 $\mathbf{x}_m$ 和保留部分 $\mathbf{x}_r$。线性化最小二乘系统的法方程为：

$$\begin{bmatrix} \mathbf{H}_{mm} & \mathbf{H}_{mr} \\ \mathbf{H}_{rm} & \mathbf{H}_{rr} \end{bmatrix} \begin{bmatrix} \Delta\mathbf{x}_m \\ \Delta\mathbf{x}_r \end{bmatrix} = \begin{bmatrix} \mathbf{b}_m \\ \mathbf{b}_r \end{bmatrix}$$

通过 Schur 补消去 $\mathbf{x}_m$：

$$\begin{aligned}
\mathbf{H}_p &= \mathbf{H}_{rr} - \mathbf{H}_{rm} \mathbf{H}_{mm}^{-1} \mathbf{H}_{mr} \\
\mathbf{b}_p &= \mathbf{b}_r - \mathbf{H}_{rm} \mathbf{H}_{mm}^{-1} \mathbf{b}_m
\end{aligned}$$

然后对 $\mathbf{H}_p$ 做特征值分解以获取线性化雅可比 $\mathbf{J}_0$ 和残差 $\mathbf{e}_0$：

$$\mathbf{H}_p = \mathbf{V} \mathbf{S} \mathbf{V}^T, \quad \mathbf{J}_0 = \mathbf{S}^{1/2} \mathbf{V}^T, \quad \mathbf{e}_0 = -\mathbf{S}^{-1/2} \mathbf{V}^T \mathbf{b}_p$$

其中 $\mathbf{S}$ 为特征值对角矩阵，小于 `EPS = 1e-8` 的特征值截断为零（对应不可观方向的正则化）。

源：`raw/codes/OB_GINS/src/factors/marginalization_info.h:L148-L188`

### 4.3 Pose 的局部参数化处理

边缘化中 Pose（全局 7 维，局部 6 维）需要特殊处理：

```cpp
static int localSize(int size) {
    return size == POSE_GLOBAL_SIZE ? POSE_LOCAL_SIZE : size;
}
static int globalSize(int size) {
    return size == POSE_LOCAL_SIZE ? POSE_GLOBAL_SIZE : size;
}
```

在构建法方程时，Jacobian 按 `localSize` 截断（取前 6 列对应 3D 平移 + 3D 旋转正切空间），保证法方程的维度与自由度一致。

源：`raw/codes/OB_GINS/src/factors/marginalization_info.h:L45-L51`

### 4.4 窗口滑出

边缘化完成后执行滑出操作，移除最老的节点和对应 GNSS/预积分：

```pseudo
function slideWindow(window):
    if |timelist[0]| == |gnsslist[0].time|:
        gnsslist.pop_front()      // 移除最老 GNSS
    timelist.pop_front()          // 移除最老时间戳
    preintlist.pop_front()        // 移除第一个预积分

    // 状态指针前移 (N+1 → N)
    for k in [0..windows-1]:
        statedatalist[k] = statedatalist[k+1]
    state[windows] = stateFromData(statedatalist[windows])
```

源：`raw/codes/OB_GINS/src/ob_gins.cc:L490-L504`

## 五、与 IC-GVINS 滑动窗口对比

| 维度 | OB_GINS | IC-GVINS |
|------|---------|----------|
| 节点创建 | 固定 1s 间隔 | 按 GNSS/关键帧动态插入 |
| 节点移除 | 严格 FIFO | 基于关键帧时间对齐 |
| 非关键帧处理 | 无概念 | 次新帧 (KEYFRAME_REMOVE_SECOND_NEW) 合并后移除 |
| IMU 插值 | 线性拆分增量 | 不插值，直接按时间截取 IMU 窗口 |
| GNSS 对齐 | 对齐容差 ±1ms，不匹配则丢弃 | ±25ms 对齐 + 速度补偿，中间则插入新节点 |
| 最多相同 GNSS 使用 | 一次（对齐后即移除） | 一次（不重复） |
| 重积分触发 | 无（离线，每次重优化的状态固定） | 初始化阶段，因零偏重新估计后触发 |
| 融合线程 | 单线程 | 三线程（Fusion/Tracking/Optimization） |

IC-GVINS 的动态节点策略更复杂但更灵活。当 GNSS 在两个节点之间到达时：
- 距前一节点 < 25ms → 对齐到前一节点，用当前速度补偿位置
- 距后一节点 < 25ms → 对齐到后一节点，用当前速度补偿位置
- 在两个节点中间 → 删除后续节点，插入新的 GNSS 节点，再重新添加后续节点

当两个视觉关键帧之间的时间节点（非关键帧）不再需要时，IC-GVINS 通过 `removeUnusedTimeNode` 合并回相邻的预积分，消除冗余状态。

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L791-L888`（动态 GNSS 插入）和 `L754-L789`（冗余节点移除）

## Agent 实现提示

### 适用场景

- 离线 GNSS/INS 数据集后处理，目标是高精度轨迹（亚米级定位）
- 已知初始状态（位置、姿态、速度、零偏），不需要自动初始化
- IMU 频率稳定（100-200Hz），GNSS 频率固定（1Hz SPP/RTK）
- 不适用：在线实时定位、多传感器异步融合（LiDAR/相机）、传感器频率不稳定

### 输入输出契约

- **输入**:
  - IMU 文件：每行 `time dt dtheta[x,y,z] dvel[x,y,z]`，200Hz
  - GNSS 文件：每行 `time BLH[lat,lon,h] std[N,E,D]`，1Hz
  - YAML 配置：窗口大小、迭代次数、粗差阈值、IMU 噪声参数、是否地球自转
  - 初始状态：`initvel`, `initatt`, `initgb`, `initab`
  - 杆臂：`antlever` (body 系, m)
- **输出**:
  - 导航文件：`time, lat(deg), lon(deg), h(m), vn(m/s), ve(m/s), vd(m/s), roll(deg), pitch(deg), yaw(deg)`
  - IMU 误差文件：`time, bg[3](deg/hr), ba[3](mGal), sodo`
- **坐标**: body 系 = 前右下，导航系 = 站心 NED，重力 = NED Z 正向

### 实现骨架（伪代码）

```pseudo
function sliding_window_gnss_optimization(imu_stream, gnss_stream, config):
    windows = config.windows  // e.g. 30
    int_len = 1.0  // INTEGRATION_LENGTH

    // 数据对齐到 starttime
    imu, gnss = align_data(imu_stream, gnss_stream, config.starttime)

    // 初始化
    origin = gnss.blh
    gravity = Earth.gravity(origin)
    gnss_local = Earth.global2local(origin, gnss.blh)
    state = create_init_state(gnss_local, config.initatt, config.initvel,
                              config.initbg, config.initba)

    // 初始化容器 (大小 windows+1)
    states       = [state] * (windows + 1)
    statedatas   = [stateToData(state)] * (windows + 1)
    gnsslist     = [gnss_local]
    timelist     = [round(gnss.time)]
    preintlist   = [createPreintegration(imu, state, config)]

    prior_info = None
    prior_blocks = None

    // 预读下一个GNSS
    gnss_next = gnss_stream.next()
    gnss_next_local = Earth.global2local(origin, gnss_next.blh)

    sow = timelist[0] + int_len  // next integer second

    while imu.time <= config.endtime:
        preintlist[-1].addImu(imu)
        imu = imu_stream.next()

        if imu.time > sow:
            // GNSS对齐检查: |gnss.time - sow| < 0.001s
            if abs(gnss_next.time - sow) < 0.001:
                gnsslist.append(gnss_next_local)
                gnss_next = gnss_stream.next()
                gnss_next_local = global2local(origin, gnss_next.blh)

            // IMU插值使预积分恰好到整秒
            interp = isNeedInterpolation(imu_prev, imu, sow)
            if interp == NEED_INTERP:
                split_imu = imuInterpolation(imu, imu_prev, imu, sow)
                preintlist[-1].addImu(split_imu)

            // 当前状态
            timelist.append(sow)
            state = preintlist[-1].currentState()
            states[len(preintlist)] = state
            statedatas[len(preintlist)] = stateToData(state)

            // ---- 构建 Ceres 优化 ----
            problem = CeresProblem()
            for k in range(len(preintlist) + 1):
                problem.addParam(pose[k], PoseManifold)
                problem.addParam(mix[k])

            for k in range(len(preintlist)):
                problem.addResidual(PreintFactor(preintlist[k]), null,
                                    pose[k], mix[k], pose[k+1], mix[k+1])

            // GNSS因子: 按时间对齐
            for gnss in gnsslist:
                for k in range(len(preintlist) + 1):
                    if abs(gnss.time - timelist[k]) < 0.001:
                        problem.addResidual(GnssFactor(gnss, lever), HuberLoss(1.0), pose[k])
                        break

            problem.addResidual(ImuErrorFactor(preintlist[-1]), null, mix[-1])

            // 边缘化先验
            if prior_info and prior_info.isValid():
                problem.addResidual(MarginalizationFactor(prior_info), null, prior_blocks)

            // 两轮优化
            options.max_iter = config.num_iter / 4
            solver.Solve(problem, options)
            // Chi² 粗差检测 + 重加权
            if config.outlier_culling:
                for gnss_id in gnss_ids:
                    chi2 = 2 * problem.EvaluateResidualBlock(gnss_id).cost
                    if chi2 > 7.815:
                        reweight_gnss(gnss, sqrt(chi2/7.815))
                remove_all_gnss(problem)
                readd_gnss_without_loss(problem)

            options.max_iter = config.num_iter * 3 / 4
            solver.Solve(problem, options)

            // ---- 窗口管理 ----
            if len(preintlist) == windows:
                // 边缘化最老节点
                margin_info = MarginalizationInfo()
                if prior_info:
                    add_prior_to_margin(margin_info, prior_info, prior_blocks,
                                       states[0].pose, states[0].mix)
                add_first_preint_to_margin(margin_info, preintlist[0])
                if abs(timelist[0] - gnsslist[0].time) < 0.001:
                    add_first_gnss_to_margin(margin_info, gnsslist[0])

                margin_info.marginalization()
                prior_info = margin_info
                prior_blocks = margin_info.getParamterBlocks(address_map)

                // 滑出
                if round(timelist[0]) == round(gnsslist[0].time):
                    gnsslist.pop_front()
                timelist.pop_front()
                preintlist.pop_front()
                for k in range(windows):
                    statedatas[k] = statedatas[k+1]

            // 输出优化后轨迹
            write_result(timelist[-1], origin, states[-1])

            // 新建下一段预积分
            preintlist.append(createPreintegration(imu, states[-1], config))
            sow += int_len

    return trajectory
```

### 关键源码片段

**a) OB_GINS 主循环（优化 + 粗差 + 边缘化）**

`raw/codes/OB_GINS/src/ob_gins.cc:L226-L521` — 完整展示了从 IMU 预积分 → 节点采样 → Ceres 优化构建 → 两轮求解 → Chi² 粗差 → 窗口滑动/边缘化的全流程。关键段：L295-355（优化问题构建）、L364-394（粗差剔除逻辑）、L430-487（边缘化）、L490-504（滑出）。

**b) Schur 边缘化核心实现**

`raw/codes/OB_GINS/src/factors/marginalization_info.h` — 完整的边缘化实现（310 行 header-only）。核心流程：`L69-L97` marginalization() 入口 → `L225-L245` updateParameterBlocksIndex() 索引分配 → `L249-L266` preMarginalization() 参数拷贝 → `L191-L223` constructEquation() 构建 H/b → `L166-L188` schurElimination() Schur 补 → `L149-L163` linearization() 特征值分解获取 J0/e0。

**c) IMU 插值拆分**

`raw/codes/OB_GINS/src/ob_gins.cc:L577-L594` — `imuInterpolation`: 将 IMU 增量按时间比线性拆分，`scale = (t1 - t_mid) / dt`，前段比例 `(1-scale)`，后段比例 `scale`。注：该拆分假设角速率和加速度在 dt 内为常数（零阶保持近似），对低动态场景精度足够。

### 实现注意事项

1. **边缘化数据结构一致性**：边缘化使用 `reinterpret_cast<long>(pointer)` 作为 unordered_map 的 key，依赖指针地址唯一性。Ceres 优化后指针地址不变，滑出时需要重新建立 `address_map` 将新指针映射到旧数据。若地址映射错误，边缘化先验会约束到错误参数上导致系统发散。
2. **Pose 局部参数化**：在边缘化的 `localSize()/globalSize()` 中，pose 块全局 7 维、局部 6 维。Jacobian 截断时取前 6 列，后 4 列对应四元数正切空间。这一维度假定与 Ceres `PoseManifold` 一致，如用其他参数化（如 `EigenQuaternionManifold`），局部尺寸可能不同，需同步修改。
3. **Chi² 阈值选择**：$\chi^2(3) = 7.815$ 对应 95% 置信度。如果 GNSS 噪声模型正确且无粗差，约 5% 的正常观测会被误判为重加权，这是可接受的概率。若期望更低误判率，可提高阈值到 $\chi^2(3, 0.99) = 11.345$。
4. **特征值截断**：`EPS = 1e-8`，小于该值的特征值视为零（对应不可观方向）。过大的 EPS 会丢失信息，过小则数值不稳定。对于 6-DoF 全局位姿（无 GNSS 时），roll+pitch 由重力可观，yaw+position 不可观，特征值谱中应出现 4 个接近零的值。
5. **窗口大小的选择**：窗口越大精度越高但计算量越大。默认 30 秒平衡了局部线性化误差和全局一致性。对于 GNSS 中断场景，窗口至少应能覆盖中断长度，否则中断期间的 IMU 预报无 GNSS 约束矫正。
6. **边缘化的线性化点固定**：Schur 边缘化在第一次线性化的点处执行 Jacobian 计算并固定（FEJ - First Estimate Jacobian）。后续优化中，边缘化先验因子的 Jacobian 不再重新线性化，以保证零空间一致性。违反 FEJ 会导致 `linearized_jacobians_` 与实际 Jacobian 不一致，引入虚假信息（spurious information）。

### 源码检索锚点

- `raw/codes/OB_GINS/src/ob_gins.cc:L226-L521` — 主循环（完整优化流程）
- `raw/codes/OB_GINS/src/ob_gins.cc:L577-L621` — IMU 插值相关函数
- `raw/codes/OB_GINS/src/ob_gins.cc:L364-L417` — Chi² 粗差剔除
- `raw/codes/OB_GINS/src/ob_gins.cc:L430-L504` — 边缘化 + 窗口滑出
- `raw/codes/OB_GINS/src/factors/marginalization_info.h` — Schur 边缘化核心
- `raw/codes/OB_GINS/src/factors/gnss_factor.h:L31-L77` — GNSS 位置因子
- `raw/codes/OB_GINS/src/factors/marginalization_factor.h` — 边缘化因子适配器
- `raw/codes/OB_GINS/src/preintegration/preintegration_factor.h` — 预积分因子适配器
- `raw/codes/OB_GINS/src/preintegration/preintegration_earth.h` — 地球自转补偿预积分
- `raw/codes/OB_GINS/src/preintegration/imu_error_factor.h` — IMU 零偏先验因子
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L791-L888` — 动态 GNSS 节点插入（对比）
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1130-L1238` — IC-GVINS 完整优化流程（对比）

## 相关页面

- 实现于：[[算法-OB_GINS]]
- 对比系统：[[算法-IC-GVINS]]
- 时间采样：[[方法-固定整秒节点采样]]
- 边缘化理论：[[概念-Schur补与边缘化]]、[[概念-延迟边缘化]]
- 坐标系统：[[架构-坐标系管理]]
- 预积分：[[概念-IMU预积分]]、[[方法-地球自转补偿预积分]]
- GNSS 因子：[[方法-GNSS 位置残差因子]]
- 粗差处理：[[方法-Ceres两轮优化粗差剔除]]
- 后端框架：[[组件-Ceres-Solver]]
- 紧耦合总体：[[方法-GNSS-INS紧耦合]]
