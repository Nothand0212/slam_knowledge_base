---
type: entity
tags: [GNSS, INS, 紧耦合, 组合导航, i2Nav, 因子图, IMU预积分, 伪距, 载波相位, Ceres]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/IC-GVINS/
  - raw/codes/OB_GINS/
---

# GNSS-INS 紧耦合架构

> 以 INS 机械编排为核心状态传播，GNSS 伪距/多普勒/载波相位及定位结果作为因子接入 Ceres 批优化进行全局约束的紧耦合组合导航方法。

## 一、方法概述

GNSS-INS 紧耦合（tightly-coupled）将 GNSS 原始观测（伪距 $\rho$、多普勒 $D$、载波相位 $\phi$）或定位结果直接作为非线性最小二乘因子，与 IMU 预积分因子在统一 Ceres 优化问题中联合求解。与松耦合（loosely-coupled，将 GNSS 定位结果作为独立位姿注入）相比，紧耦合在 GNSS 可见星数不足（<4 颗）时仍能贡献部分约束，显著提升城市峡谷、高架桥下等退化场景的鲁棒性。

i2Nav 团队（武汉大学）的 IC-GVINS 和 OB_GINS 是紧耦合 GNSS-INS 的两个代表性开源实现，共享预积分框架和 Ceres 因子体系，但面向不同场景：

| 特性 | IC-GVINS | OB_GINS |
|------|----------|---------|
| 传感器 | GNSS + IMU + 单目相机 | GNSS + IMU（+ 里程计） |
| 观测类型 | GNSS 定位（RTK/SPP） | GNSS 定位（RTK/SPP） |
| 优化后端 | Ceres + Schur 边缘化 | Ceres + Schur 边缘化 |
| 时间组织 | 动态节点（按 GNSS/关键帧插入） | 固定整秒节点 |
| 执行模式 | 多线程在线实时 | 单线程离线批处理 |
| 地球自转补偿 | 支持 | 支持 |

## 二、系统架构

```
┌─────────────────────────────────────────────────────┐
│                  Ceres 非线性优化                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │GNSS 因子 │  │IMU 预积分│  │ 边缘化先验因子    │   │
│  │(位置/伪距)│  │   因子   │  │(Marginalization) │   │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘   │
│       │             │                 │              │
│  ┌────┴─────────────┴─────────────────┴──────────┐   │
│  │         滑动窗口状态 (Pose + Mix)              │   │
│  │  statedatalist = [{p,q}, {v,bg,ba,sodo}] × N  │   │
│  └──────────────────────┬────────────────────────┘   │
└─────────────────────────┼────────────────────────────┘
                          │
     ┌────────────────────┼────────────────────┐
     ▼                    ▼                     ▼
┌─────────┐        ┌──────────┐          ┌──────────┐
│IMU 积分 │        │GNSS 数据  │          │ INS 编排 │
│(dtheta, │        │(BLH, std) │          │(速度/位置│
│ dvel)   │        │           │          │ 传播)    │
└─────────┘        └──────────┘          └──────────┘
```

核心数据流：IMU 增量 → 预积分对象（补偿零偏和地球自转）→ 构建相邻状态节点间的相对运动约束；GNSS 定位 → 全局坐标转换（BLH→站心 NED）→ 构建绝对位置约束。两者在 Ceres Problem 中以因子图形式联合优化，Schur 边缘化维护滑动窗口的历史信息。

## 三、GNSS 观测模型

### 3.1 伪距观测方程

GNSS 接收机对卫星 $s$ 的伪距观测：

$$\rho_r^s = r_r^s + c(\delta t_r - \delta t^s) + I_r^s + T_r^s + \epsilon_\rho$$

其中 $r_r^s = \|\mathbf{p}^s - \mathbf{p}_r\|$ 为星地几何距离，$\delta t_r$、$\delta t^s$ 分别为接收机和卫星钟差，$I_r^s$、$T_r^s$ 为电离层和对流层延迟，$\epsilon_\rho$ 为伪距噪声。

在 IC-GVINS/OB_GINS 中，处理后的 GNSS 定位结果 (BLH) 转化为站心 NED 坐标后接入：

$$e_{pos} = \mathbf{W} \cdot \left(\mathbf{p}_{IMU}^n + \mathbf{R}_b^n(\mathbf{q}) \cdot \mathbf{l}_{ant} - \mathbf{p}_{GNSS}^n\right)$$

其中 $\mathbf{W} = \text{diag}(1/\sigma_N, 1/\sigma_E, 1/\sigma_D)$ 为信息矩阵的 Cholesky 因子，$\mathbf{l}_{ant}$ 为 IMU 到 GNSS 天线的杆臂向量。

### 3.2 载波相位观测方程

$$\phi_r^s = \lambda^{-1} \left(r_r^s + c(\delta t_r - \delta t^s) - I_r^s + T_r^s\right) + N_r^s + \epsilon_\phi$$

$N_r^s$ 为整周模糊度。载波相位精度可达毫米级，但需要正确固定模糊度才能提供绝对约束。OB_GINS 未直接使用原始载波相位，而是依赖 RTK 处理后的定位结果。

### 3.3 多普勒观测方程

$$D_r^s = -\frac{1}{\lambda} \cdot \frac{(\mathbf{v}^s - \mathbf{v}_r) \cdot (\mathbf{p}^s - \mathbf{p}_r)}{r_r^s} + c\dot{\delta t}_r + \epsilon_D$$

多普勒观测提供速度级约束，可直接约束 INS 速度状态。

## 四、因子图建模

### 4.1 状态定义

每个优化节点的状态由两部分组成：

```pseudo
struct StateNode:
    pose[7]   ;  // 位置 (x, y, z) + 姿态四元数 (qx, qy, qz, qw)
    mix[9~18] ;  // 速度 (3) + 陀螺零偏 (3) + 加表零偏 (3) [+ 里程计比例因子] [+ 安装角偏差]
```

Pose 块为 7 维过参数化，使用 PoseManifold 将优化自由度限制为 6；Mix 块为欧氏空间，维度随是否启用里程计和比例因子估计而变化。

### 4.2 因子类型

优化问题中包含三类因子：

**a) GNSS 位置因子** — 约束单个状态节点的位姿：

$$e_{GNSS} = \sqrt{\boldsymbol{\Sigma}^{-1}} \left(\mathbf{p} + \mathbf{R}(\mathbf{q}) \cdot \mathbf{l}_{ant} - \mathbf{p}_{GNSS}\right) \in \mathbb{R}^3$$

参数块：`{pose[7]}`，残差维度 3。雅可比：

$$\frac{\partial e}{\partial \mathbf{p}} = \mathbf{I}_3, \quad \frac{\partial e}{\partial \boldsymbol{\theta}} = -\mathbf{R}(\mathbf{q}) \cdot [\mathbf{l}_{ant}]_\times$$

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/gnss_factor.h:L31-L76`

**b) IMU 预积分因子** — 约束相邻两状态节点的相对运动：

$$e_{IMU} = \mathbf{z}_{ij} - \mathbf{h}(\mathbf{x}_i, \mathbf{x}_j)$$

参数块：`{pose_i, mix_i, pose_j, mix_j}`，残差维度 15（含地球自转补偿）。

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/preintegration_factor.h:L30-L73`

**c) IMU 零偏先验因子** — 约束 IMU 误差参数：

$$e_{bias} = \left[\frac{b_g}{\sigma_{bg}}, \frac{b_a}{\sigma_{ba}}\right]^T \in \mathbb{R}^6$$

参数块：`{mix}`，残差维度 6。防止零偏在 GNSS 中断期间发散。

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/imu_error_factor.h:L30-L94`

### 4.3 GNSS 粗差检测（Chi² 检验）

IC-GVINS 采用两轮优化 + Chi² 粗差策略。第一轮使用 HuberLoss 优化，然后对每个 GNSS 残差块：

$$\chi^2 = 2 \cdot \text{cost} \sim \chi^2(3)$$

当 $\chi^2 > 7.815$（95% 置信度，3 自由度），判定为粗差，进行重加权（std *= $\sqrt{\chi^2/7.815}$），第二轮无核函数优化。源：`raw/codes/OB_GINS/src/ob_gins.cc:L364-L394`

## 五、坐标系统与杆臂补偿

IC-GVINS/OB_GINS 使用站心 NED 坐标系作为导航系（n-frame）。数据流转：

1. GNSS 输出：WGS84 BLH（纬度、经度、高程）
2. `Earth::global2local(origin, blh)` 转为站心 NED，原点为该段首个 GNSS 定位
3. 杆臂补偿：天线相位中心 → IMU 测量中心
4. 地球自转 $\boldsymbol{\omega}_{ie}^n = [\omega_e \cos\varphi, 0, -\omega_e \sin\varphi]^T$ 在 NED 下的投影补偿

源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L185-L191`

## 六、两种实现的关键差异

### 6.1 时间组织

- **OB_GINS**: 固定整秒节点 `INTEGRATION_LENGTH = 1.0s`，GNSS 对齐容差 1ms。实现简单，适合离线后处理。
- **IC-GVINS**: 动态节点，GNSS 到达时按与相邻节点的时间差决定对齐或插入新节点。当 GNSS 距前/后节点 < 25ms 时对齐并补偿速度；在中间时插入新节点以避免超长预积分（>10s）。源：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L791-L888`

### 6.2 预积分策略

两个项目共享相同的 `PreintegrationBase` → `PreintegrationEarth` 继承体系，差别在于 OB_GINS 的 `ImuErrorFactor` 寄生于 `PreintegrationBase` 的多态方法（`imuErrorEvaluate`、`imuErrorJacobian`），而 IC-GVINS 单独实现为独立 CostFunction。IC-GVINS 额外支持 IMU 先验因子（`ImuPosePriorFactor`）用于初始阶段约束。

### 6.3 在线 vs 离线

IC-GVINS 三线程并行（Fusion/ Tracking/ Optimization），支持实时 ROS 输入和可视化；OB_GINS 单线程顺序读取文本文件，每次新增节点触发一次完整滑动窗口优化（包含边缘化）。

## Agent 实现提示

### 适用场景

- 户外大尺度组合导航，GNSS 频繁可用但可能存在短时中断
- 需要全球参考系一致性（坐标系对齐 WGS84）
- 不适用于：纯室内无 GNSS、消费级 MEMS IMU（陀螺噪声淹没地球自转信号）

### 输入输出契约

- **输入**:
  - IMU: 增量角速度 dtheta(rad)、增量速度 dvel(m/s)、时间戳 dt(s)，body 系
  - GNSS: BLH(rad, rad, m)、各向异性标准差 std(NED, m)、时间戳(s)
  - 杆臂：antenna lever arm (body 系，m)
  - 配置：零偏标准差、ARW/VRW、是否地球自转补偿、窗口大小、迭代次数
- **输出**:
  - 优化后状态：每节点的 p(NED, m)、q(b→n 四元数)、v(NED, m/s)、bg(rad/s)、ba(m/s²)
  - 全局定位：LLH(WGS84)、Euler 姿态(RPY, deg)
- **坐标/单位**: body 系为前右下（FRD），导航系为站心 NED，时间同步到 GPS 秒

### 实现骨架（伪代码）

```pseudo
function gnss_ins_tightly_coupled(imu_stream, gnss_stream, config):
    // 1. 初始化
    origin, gravity = init_from_first_gnss(config.imu)
    state_0 = {p: gnss.blh-lever, q: init_att, v: 0, bg: 0, ba: 0}
    preint = create_preintegration(imu_0, state_0, config)

    // 2. 主循环
    state_list = [state_0]
    gnss_list = [gnss_0]
    preint_list = [preint]

    while has_data(imu_stream):
        imu = next(imu_stream)
        preint.add_imu(imu)

        if at_integration_node(imu.time):
            // 固定整秒或 GNSS 到达
            if gnss_available():
                gnss_list.append(gnss)

            state = preint.currentState()
            state_list.append(state)

            // 3. 构建 Ceres 优化问题
            problem = CeresProblem()
            for each node i in [0..N]:
                problem.add_parameter(pose[i], PoseManifold)
                problem.add_parameter(mix[i])

            for each gnss in gnss_list:
                i = time_index(gnss.time)
                factor = GnssFactor(gnss, lever_arm)
                problem.add_residual(factor, HuberLoss, pose[i])

            for k in [0..N-1]:
                factor = PreintegrationFactor(preint_list[k])
                problem.add_residual(factor, null, pose[k], mix[k], pose[k+1], mix[k+1])

            factor = ImuErrorFactor(preint_list[-1])
            problem.add_residual(factor, null, mix[-1])

            if prior_valid():
                problem.add_residual(MarginalizationFactor(prior))

            // 4. 两轮优化 + 粗差剔除
            Solve(problem, options_round1)  // HuberLoss
            for each gnss_factor:
                chi2 = 2 * cost
                if chi2 > 7.815:
                    reweight(gnss.std *= sqrt(chi2/7.815))
            remove_and_readd_gnss(problem, without_loss)
            Solve(problem, options_round2)

            // 5. 窗口管理
            if window_full():
                marginalize_oldest(state_list, preint_list, gnss_list)

            // 6. 新建预积分
            preint_list.append(new_preintegration(imu, state_new))

    return state_list, stats
```

### 关键源码片段

**a) GNSS 位置因子完整实现**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/gnss_factor.h:L31-L76` — 展示 3 维位置残差 + 杆臂补偿 + 信息矩阵加权 + 解析雅可比，是紧耦合 GNSS 因子的标准 Ceres CostFunction 模式。

```cpp
// 残差: e = W * (p + R(q)*lever - p_gnss)
Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

// 信息矩阵的 Cholesky 分解 (对角)
Matrix3d sqrt_info_ = Matrix3d::Zero();
sqrt_info_(0,0) = 1.0 / gnss_.std[0];
sqrt_info_(1,1) = 1.0 / gnss_.std[1];
sqrt_info_(2,2) = 1.0 / gnss_.std[2];
error = sqrt_info_ * error;

// 解析雅可比: de/dp = I, de/dθ = -R*skew(lever)
jacobian_pose.block<3,3>(0,0) = Matrix3d::Identity();
jacobian_pose.block<3,3>(0,3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);
```

**b) 两轮优化 + GNSS 粗差剔除**

`raw/codes/OB_GINS/src/ob_gins.cc:L296-L417` — OB_GINS 的完整优化循环，展示参数块注册 → 因子添加 → LM 求解 → Chi² 粗差 → 移除重加因子 → 再求解的流程。注意 HuberLoss(1.0) 用于第一轮，第二轮无核函数。

**c) 地球自转补偿的预积分张量**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h:L229-L236` — 在地球系下计算投影：地球自转角速度 `WGS84_WIE = 7.2921151467E-5 rad/s`，NED 下 $[ω_e·cos(φ), 0, -ω_e·sin(φ)]^T$。该值在预积分时加入陀螺测量补偿。

### 实现注意事项

1. **杆臂方向**：`lever_` 定义在 body 系，表示从 IMU 中心指向天线相位中心。符号错误会导致米级系统偏差。
2. **坐标转换链路**：BLH → ECEF → NED(local) → body。`global2local` 内部通过 `cne` 矩阵旋转，原点一旦设定不应更改（否则轨迹跳变）。
3. **信息矩阵构造**：使用对角线近似（忽略 GNSS 定位的 N/E/D 相关性），对于 RTK 这在统计上合理；对 SPP（单点定位），cov 非对角不可忽略。
4. **零偏先验标准差**：IMU 零偏标准差的值直接影响优化收敛行为，过大则零偏漂移，过小则约束太强导致 GNSS 残差不能有效吸收误差。参考 IC-GVINS 默认值：陀螺 7200 deg/hr、加表 20000 mGal。
5. **预积分时间长度**：IC-GVINS 设定 `MAXIMUM_PREINTEGRATION_LENGTH = 10.0s`，超长预积分会因线性化误差放大导致精度下降。
6. **多线程安全**：IC-GVINS 中 `state_mutex_` 保护优化状态不被他线程并发读写。Fusion 线程检测 `isoptimized_` 标志以决定是否重新编排 INS 积分。

### 源码检索锚点

- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/gnss_factor.h` — GNSS 位置因子 CostFunction
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/preintegration_factor.h` — 预积分因子适配器
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/preintegration_earth.h` — 地球自转补偿预积分
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/preintegration/integration_state.h` — 状态与参数定义
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/earth.h` — 地球模型与坐标变换
- `raw/codes/IC-GVINS/ic_gvins/ic_gvins/common/rotation.h` — 旋转工具（反对称矩阵、Euler 转换）
- `raw/codes/OB_GINS/src/ob_gins.cc` — OB_GINS 离线批处理主循环
- `raw/codes/OB_GINS/src/factors/gnss_factor.h` — OB_GINS 版 GNSS 因子（与 IC-GVINS 同构）
- `raw/codes/OB_GINS/src/factors/marginalization_info.h` — Schur 边缘化实现
- `raw/codes/OB_GINS/src/preintegration/preintegration_earth.h` — OB_GINS 地球自转补偿

## 相关页面

- 实现于：[[算法-IC-GVINS]]、[[算法-OB_GINS]]
- 核心因子：[[方法-GNSS 位置残差因子]]
- 预积分基础：[[概念-IMU预积分]]、[[方法-地球自转补偿预积分]]
- 坐标系：[[架构-坐标系管理]]
- 粗差剔除：[[方法-Ceres两轮优化粗差剔除]]
- 对比架构：[[架构-多传感器融合架构]]
- 后端工具：[[组件-Ceres-Solver]]、[[概念-因子图]]
- 相关初始化：[[方法-INS-centric初始化策略]]
