# phad_fusion：多模态因子图 SLAM 算法设计

> 设计目标：一套传感器无关的因子图 SLAM 系统，支持纯定位/局部建图/全局 SLAM 三模式切换，目标精度 SOTA

---


## 1. 问题定义

我们要解决的核心问题：给定一组异构传感器（相机、IMU、LiDAR、GNSS、轮速计等，
数量和类型随部署场景变化），实时估计载体的**全状态**（位姿、速度、IMU bias、传感器标定参数），
并在需要时维护**环境地图**（稀疏路标点云 或 稠密体素地图）。

这不是一个"把几个开源项目拼起来"的集成问题，而是一个需要从数学建模、因子设计、
优化策略、边缘化理论到并发架构全链路思考的算法设计问题。


### 1.1 为什么现有方案不够好

| 方案 | 优点 | 致命缺陷 |
|---|---|---|
| open_vins MSCKF | 快，工程规范 | EKF 一次线性化，非线性误差累积；无回环；扩展新传感器需手改协方差 |
| VINS-Fusion | 滑动窗口 BA 精度好，有回环 | Ceres 非增量式，每帧重做窗口优化；GPS 仅松耦附加；传感器类型固定 |
| ORB-SLAM3 | 三线程架构，Atlas 多地图 | ORB 特征光照不变性差；G2O 不如 GTSAM 的增量推理；特征与后端紧耦合换成其他传感器困难 |
| Kimera-VIO | GTSAM iSAM2 + SmartStereoFactor | 仅视觉惯性，不支持 LiDAR/GNSS；稠密重建与定位不耦合 |
| IC-GVINS | GNSS-VIO 紧耦合 | Ceres 非增量；无回环；不支持 LiDAR |
| lightning-lm | IESKF + voxel 地图，性能极高 | 自研 miao 后端生态封闭；视觉能力弱；无 GNSS |

共同缺陷：**每个系统对传感器组合做了硬编码假设**。增加一种传感器 = 大量代码修改。
没有一个系统支持"运行时切换模式"——比如从 VIO 纯定位变为 LiDAR-SLAM 建图。


### 1.2 phad_fusion 要达成的目标

| 维度 | 目标 | 度量 |
|---|---|---|
| 精度 | SOTA 级别，挑战 ORB-SLAM3/VINS-Fusion 在标准数据集上的 ATE/RPE | EuRoC <0.05m RMSE ATE; KITTI <1% translation error |
| 传感器灵活性 | 任意传感器组合可工作，增减传感器不需改核心代码 | 支持 Mono/Stereo/Multi-Cam + IMU + LiDAR + GNSS + Wheel 任意组合 |
| 运行模式 | 3 模式可切换：纯里程计 / 局部建图 / 全局 SLAM | 模式切换无重启，<100ms 切换延迟 |
| 实时性 | 30Hz 以上稳定输出，后端不堆积 | Frontend <15ms, Backend <30ms per frame |
| 跨平台 | ARM/X86/Linux/嵌入式 | 无 GPU 依赖的核心路径; 可选 GPU 加速前端 |
| 工程质量 | 单元测试覆盖 >70%, 文档完整, 可单独编译为 .so | CMake only 构建, ROS 完全可选 |


### 1.3 核心设计哲学

**"一切皆是因子"（Everything is a Factor）**

整个系统的状态估计被建模为一个增量式因子图。每一种传感器测量、每一个运动学约束、
每一个先验知识，都在因子图中体现为一个 **因子（Factor）**。
系统的核心循环是：接收数据 → 生成因子 → ISAM2 增量优化 → 输出结果。

这意味着：
- 新增传感器 = 实现一个新的因子类 + 注册到因子图管理器。核心架构零修改。
- 切换运行模式 = 改变哪些类型的因子被激活。VIO 模式 = 仅视觉 + 惯性因子；
  全 SLAM 模式 = 视觉 + 惯性 + LiDAR + 回环因子。
- 传感器失效 = 对应因子停止加入，ISAM2 自然地从剩余约束中推断。


## 2. 理论基础


### 2.1 因子图建模

系统的全部状态变量 $\mathcal{X}$ 和全部测量 $\mathcal{Z}$ 被建模为因子图 $G = (\mathcal{X}, \mathcal{F})$，其中每个因子 $f_k \in \mathcal{F}$ 是一个势函数：

```
$$p(\mathcal{X} \mid \mathcal{Z}) \propto \prod_{k} f_k(\mathcal{X}_k)$$
```

最大后验估计 (MAP) 等价于最小化所有因子的负对数：

```
$$\mathcal{X}^* = \arg\min_{\mathcal{X}} \sum_{k} \| \mathbf{e}_k(\mathcal{X}_k) \|^2_{\Sigma_k}$$
```

其中 $\mathbf{e}_k(\mathcal{X}_k) = h_k(\mathcal{X}_k) \boxminus z_k$ 是**局部坐标下的残差**，
$h_k$ 是测量函数，$\boxminus$ 是流形上的减法算子，$\Sigma_k$ 是信息矩阵。


### 2.2 状态变量定义

状态向量 $\mathcal{X}$ 按时间组织为一系列节点。每个时间步 $t_i$ 的状态节点包含：

```
$$\mathbf{x}_i = \begin{bmatrix}
{}^{W}\mathbf{R}_{B_i} \in SO(3) & \text{载体姿态 (World ← Body)} \\
{}^{W}\mathbf{p}_{B_i} \in \mathbb{R}^3 & \text{载体位置 (World frame)} \\
{}^{W}\mathbf{v}_{B_i} \in \mathbb{R}^3 & \text{载体速度 (World frame)} \\
\mathbf{b}_{g_i} \in \mathbb{R}^3 & \text{陀螺仪 bias} \\
\mathbf{b}_{a_i} \in \mathbb{R}^3 & \text{加速度计 bias}
\end{bmatrix}$$
```

此外，状态集 $\mathcal{X}$ 还包含：
- 传感器外参 $\mathcal{C}_{ext}$：每个传感器相对 Body 系的固定变换 $\mathbf{T}^{B}_{S_j} \in SE(3)$
- 相机内参 $\mathcal{C}_{int}$：每相机的投影参数向量 $\boldsymbol{\theta}_{cam_j} \in \mathbb{R}^n$
- 时间偏移 $\mathcal{T}_{off}$：各传感器相对 IMU 的时钟偏差 $\Delta t_j$
- 环境路标 $\mathcal{L}$：持续跟踪的特征 3D 坐标 $\mathbf{p}^W_{l_j} \in \mathbb{R}^3$ （仅 SLAM 模式）
- GNSS 相关 $\mathcal{G}$：ENU 原点偏移、接收机钟差等


### 2.3 流形上的优化与局部坐标

所有状态变量都定义在适当的流形上（SO(3), SE(3), $\mathbb{R}^n$）。
GTSAM 的 ISAM2 在每次迭代中，在**最新线性化点** $\mathcal{X}^0$ 的局部坐标中对因子做一阶 Taylor 展开：

```
$$\mathbf{e}_k(\mathcal{X}_k^0 \boxplus \boldsymbol{\delta}_k) \approx \mathbf{e}_k(\mathcal{X}_k^0) + \mathbf{J}_k \boldsymbol{\delta}_k, \quad
\mathbf{J}_k = \left. \frac{\partial \mathbf{e}_k}{\partial \boldsymbol{\delta}_k} \right|_{\mathcal{X}_k^0}$$
```

其中 $\boxplus$ 是 retraction 算子：对 SO(3) 为指数映射，对 $\mathbb{R}^n$ 为普通加法。
这一步与 EKF（open_vins）中 FEJ 的一次线性化不同——ISAM2 在每次迭代中重新选择是否需要重线性化，
避免了 EKF 的精度损失。


### 2.4 增量平滑与边缘化（ISAM2 原理）

ISAM2 维护因子图的 **Bayes Tree** 分解，当新因子加入时只更新受影响的 clique，复杂度为
$O(|\mathcal{X}_{reelim}| \cdot d^2)$，其中 $d$ 是变量维度，而非全图大小。
这使得增量运算比每帧重做 batch BA 快 10-100 倍。

边缘化由 ISAM2 的 **固定滞后平滑器**（FixedLagSmoother）管理。
当滑动窗口外移出状态变量时，ISAM2 通过 Bayes Tree 的变量消除（Variable Elimination）
做正确的 Schur 补，等价于将边缘化变量的信息投影到剩余变量上：

```
$$\Lambda_{rem} = \Lambda_{rr} - \Lambda_{rm} \Lambda_{mm}^{-1} \Lambda_{mr}$$
```

这与 Kimera-VIO 的 IncrementalFixedLagSmoother 完全等价。
注意：open_vins/VINS-Fusion 的边缘化实现都有近似（EKF 的丢弃 / Ceres 的伪逆截断），
而 ISAM2 做的是理论严格的 Schur 补。


## 3. 算法设计

本节描述系统的完整数据处理流程。核心循环为：
接收传感器数据 → 预处理 → 生成因子 → ISAM2 优化 → 输出。


### 3.1 主循环：runOnce()

```cpp
bool Pipeline::runOnce() {
    // 1. 从各传感器队列取最新数据包
    auto [t_min, packets] = alignAndCollect(time_window_us);

    // 2. 若为第一帧，执行初始化
    if (!_initialized) {
        _initialized = initialize(packets);
        if (!_initialized) return false;
    }

    // 3. 传感器前端：特征提取 / ICP 匹配 / GNSS 坐标转换
    for (auto& p : packets) {
        _frontends[p.sensor_id]->process(p);
    }

    // 4. 预积分：IMU 数据在连续帧间累积
    _preint_mgr->integrate(packets);

    // 5. 判断是否插入新状态节点
    if (shouldInsertNewNode(packets)) {
        auto new_state = createStateNode(packets);

        // 6. 添加因子
        addImuFactor(cur_state, new_state);
        for (auto& p : visual_packets)  addVisualFactor(new_state, p.features);
        for (auto& p : lidar_packets)   addLidarFactor(cur_state, new_state, p.icp_result);
        for (auto& p : gnss_packets)    addGnssFactor(new_state, p.position_enu);
        for (auto& p : wheel_packets)   addWheelFactor(cur_state, new_state, p.odom);
        if (zeroVelocityDetected)       addZUPTFactor(new_state);

        // 7. ISAM2 增量更新
        _isam->update();

        // 8. 边缘化出窗口的老状态
        if (windowSizeExceeded()) {
            _isam->marginalizeOut(oldest_state_keys);
        }
    }

    // 9. 输出当前最优估计
    _latest_state = _isam->calculateEstimate();
    return true;
}
```


### 3.2 初始化子系统

初始化的目标：利用前几秒的传感器数据，确定初始状态（重力方向、初始位姿、速度、IMU bias、
尺度）。不同于 open_vins/ORB-SLAM3 的"等静止才初始化"策略，
phad_fusion 采用**多层级递进初始化**：


#### 层级 1：GNSS 辅助初始化（最快，<1s）

如果 GNSS 可用，直接确定 ENU 原点、初始位置和航向（通过连续两帧 GNSS 的位置差）。
速度 = 0（或低通滤波），姿态从加速度确定 roll/pitch，yaw 从 GNSS 航向确定。
IMU bias 初始化为 0，协方差较大。


#### 层级 2：视觉惯性初始化（1-3s 数据窗口）

使用 Ceres 构建小规模优化问题，参考 open_vins 的 DynamicInitializer 思路：

```
变量:
  - N 个关键帧姿态 (SO(3) × R³, 间隔 ~0.2s)
  - 重力向量 g ∈ R³
  - 速度 v_i (每关键帧)
  - 加速度计/陀螺仪 bias
  - M 个特征 3D 坐标 (从三角化初始化)

残差:
  - 视觉重投影误差: e = π(R_wc*(P_w - t_c)) - uv
  - IMU 连续预积分: e_r, e_v, e_p (CpiV1 测量模型)
  - 先验约束: 防止优化发散

求解器: Ceres Solver, LM 算法

完成后: 计算 IMU → World 变换, 填充初始 x_0
```


#### 层级 3：LiDAR 辅助初始化

如果 LiDAR 可用且视觉初始化条件不满足，使用连续两帧 LiDAR 的 ICP 结果（genz-icp）
得到相对运动，结合 IMU 预积分做松耦对齐。适用于视觉退化场景（暗光、无纹理）。


### 3.3 视觉前端：特征跟踪管道

参考 open_vins TrackKLT + VINS-Fusion 反向光流验证 + Kimera-VIO ANMS 均匀化。


#### 3.3.1 特征提取（per frame）

```
1. 直方图均衡化（CLAHE, 参考 Kimera-VIO）
2. 构建图像金字塔（cv::buildOpticalFlowPyramid, levels=3）
3. 自适应网格 FAST 提取（参考 open_vins Grider_GRID）
   - 目标特征数: 150-200 per camera
   - 网格: 根据当前成功跟踪数动态调整 grid 密度
   - cv::cornerSubPix 亚像素精化
4. 对新提取的特征计算描述子（可选，用于回环检测）
   - 默认: ORB 描述子（cv::ORB::compute）
   - 可选: SuperPoint（GPU 加速，未来）
```


#### 3.3.2 帧间跟踪

```
1. 上帧特征用 IMU 旋转预测初始位置（参考 Kimera-VIO）
2. cv::calcOpticalFlowPyrLK 金字塔 KLT 跟踪
   - win_size=21, pyr_levels=3, max_iter=30, eps=0.01
3. 反向光流验证（参考 VINS-Fusion）:
   - 将跟踪结果反向 KLT 回 prev frame
   - 如果偏差 > 0.5 pixel → 剔除
4. RANSAC 外点剔除 (open_vins 模式):
   - 先去畸变为归一化坐标
   - cv::findFundamentalMat(FM_RANSAC, 2.0/focal, 0.999)
5. 更新 FeatureDatabase（参考 open_vins FeatureDatabase）
6. 如果存活特征 < max(30, 0.3*target_count)，触发更多 FAST 补充
```


#### 3.3.3 立体匹配（双目模式）

```
如果存在立体像对:
1. 左图特征用 NCC 沿极线搜索右图匹配 (参考 Kimera-VIO StereoMatcher)
2. 视差 < min_disparity 的特征标记为"远点"，用逆深度参数化
3. 视差有效的特征直接三角化到 anchor 坐标系
```


### 3.4 LiDAR 前端

LiDAR 前端的核心是 genz-icp 的自适应 GICP，集成 gtsam_points 的体素化加速。


#### 3.4.1 点云预处理

```
1. IMU 反向传播去畸变（参考 lightning-lm, FAST-LIO）
   - 利用 Propagator 中的 imu_data 队列
   - 对 scan 内每个点，用线性插值的 IMU 位姿补偿
2. PCL VoxelGrid 降采样（分辨率 0.2m 室内 / 0.5m 室外）
3. 可选: 曲率特征提取（c = 1/|S|·||Σ_j∈S (p_j - p_i)|| / ||p_i||）
   用于区分 planar 和 non-planar 区域
```


#### 3.4.2 genz-icp 自适应配准

```
输入: 当前 scan P_src, 目标 scan/local-map P_tgt, 初始位姿 T_init(IMU 传播)

算法:
1. 对每个源点 p_i:
   a. 在目标点云中找 K 近邻（体素哈希加速）
   b. 计算局部分布: μ_i = mean(NN), Σ_i = cov(NN, μ_i)
   c. 计算平面度: planarity(Σ_i) = (λ_2 - λ_3) / λ_1 (λ 为 Σ 的特征值降序)
   d. 自适应权重 α_i:
      α_i = {
         1.0,                          planarity > 0.8  (明确平面)
         planarity,                    planarity ∈ [0.3, 0.8]
         0.1,                          planarity < 0.3  (退化/散乱点)
      }
   其中 α_i 在 0.1~1.0 间连续变化, 非硬阈值

2. 自适应阈值 σ:
   σ = max(σ_min, median_{i}(|p_i - T*p_i_last|))

3. GICP 代价函数:
   E(T) = Σ_i α_i · || μ_tgt(i) - T * p_src(i) ||^2_{Σ_src(i) + T*Σ_tgt(i)*T^T}
   + (1 - α_i) · σ² · log(2π · det(Σ_total))

   planar 区: 退化为点到平面距离 (α=1)
   非 planar 区: 退化为 Mahalanobis 距离, 低权重避免拉扯

4. 用 Gauss-Newton 优化 T (最多 15 次迭代)

输出: T_icp 及其协方差 Σ_icp (从最终 Hessian 的逆近似)
```

协方差估计：$\Sigma_{icp} = (\mathbf{J}^T \mathbf{W} \mathbf{J})^{-1}$，
其中 J 是最终迭代的 Jacobian，W 是权重对角阵。这使 LiDAR 因子可以被因子图正确加权。


### 3.5 IMU 预积分

使用 GTSAM 的 `PreintegratedCombinedMeasurements` (Forster 2015 流形预积分)。
在连续帧间累积 IMU 测量，生成预积分项 $\Delta \mathbf{R}_{ij}, \Delta \mathbf{v}_{ij}, \Delta \mathbf{p}_{ij}$
及其对 bias 的 Jacobian。

```
GTSAM 配置:
- 参数: PreintegrationCombinedParams::MakeSharedU(gravity_mag)
- 积分方式: 内部 100Hz 步长（参考 gtsam_points）
- Noise model: 从 IMU 数据手册查 random walk / noise density

预积分容器生命周期:
- 每个关键帧间隔创建一个新的预积分器
- 当 bias 更新时，用 Jacobian 做一阶修正 (GTSAM 自动处理)
- 预积分结果作为 CombinedImuFactor 加入因子图
```


### 3.6 GNSS 因子

GNSS 因子设计吸取 IC-GVINS 和 OB_GINS 的经验，但改为 GTSAM 因子格式。


#### 3.6.1 位置因子

```
e = p_ENU(x_Body) + R_ENU_Body * lever_arm_gnss - p_gnss_ENU_obs

其中:
- p_ENU(x_Body) = x_Body.translation()  (从状态变量取)
- lever_arm_gnss = 天线在 Body 系下的坐标 (标定值, 或在线估计)
- p_gnss_ENU_obs = WGS84 → ENU 转换后的观测位置

残差维数: 3
Jacobian: d_e/d_pose = [I_3 | -R_Body_ENU * [lever_arm]×]
继承: NoiseModelFactor1<Pose3>
```


#### 3.6.2 可选：双差载波相位因子（Phase 5 以后）

```
使用双差消除接收机/卫星钟差，残差形式:
e = ∇ΔΦ_ij^{kl} - ∇Δρ_ij^{kl} - λ * ∇ΔN_ij^{kl}

维数: 1 per double-difference pair
需要额外估计整周模糊度 ∇ΔN
```


### 3.7 轮速因子

参考 OB_GINS 的轮速处理。轮速计提供 Body 系下的 2D 速度 $\mathbf{v}^{body}_{wheel} = [v_x, v_y, 0]^T$。
因子约束状态速度：

```
e = v_Body(x) - v_wheel_obs

维数: 3 (x, y 分量强约束，z 分量大协方差)
噪声模型: 根据轮速质量设置不同方向的 sigma
```


### 3.8 零速更新 (ZUPT)

```
ZUPT 检测（参考 open_vins UpdaterZeroVelocity）:
- 窗口内陀螺仪方差 < σ²_gyro_thresh (如 0.01 rad²/s²)
- 窗口内加速度计方差 < σ²_accel_thresh (如 0.1 m²/s⁴)
- 窗口长度 = 0.5s

当检测到静止:
- 添加 ZUPT 因子: e_v = v_Body(x) - 0
                     e_omega = b_gyro(x) - wm   (陀螺零偏直接观测)
- 维数: 6
- 高噪声先验权重，不做硬约束
```


## 4. 运行模式


### 4.1 模式一：纯里程计 (Odometer Mode)

**描述**：仅估计当前位姿/速度/IMU bias，不维护环境地图。
等价于 VIO 或 LIO 的里程计功能。
滑动窗口仅保留最近 N 帧，所有路标因子通过 SmartFactor 的 Schur 补隐式消去。

```
激活的因子:
  ✅ IMU 预积分因子
  ✅ 视觉 SmartStereoFactor (Schur 消去路标)
  ✅ LiDAR ICP 因子 (相对位姿约束)
  ✅ GNSS 位置因子 (全局约束)
  ✅ Wheel/ ZUPT 因子
  ❌ 路标显式因子 (不作为状态变量)
  ❌ 回环检测 / PGO

窗口大小: N=8-12 帧
特性: 最快，无漂移累积（有 GNSS 时全局约束），适合飞控/自驾输入
```


### 4.2 模式二：局部建图 (Local Mapping Mode)

**描述**：在里程计基础上，维护一个局部滑动窗口内的稀疏特征地图。
路标 3D 坐标作为状态变量显式优化（不做 Schur 补）。
地图随窗口滑动而更新，不永久保存。

```
激活的因子:
  ✅ 模式一全部因子
  ✅ 视觉重投影因子 (路标作为状态变量, FULL prior)
  ✅ LiDAR 点到体素因子 (gtsam_points IntegratedGICPFactor)
  ❌ 回环检测 / PGO

窗口大小: N=10-15 帧 + 相关路标
特性: 局部精度最高（路标显式优化），无全局约束，漂移随距离线性增长
```


### 4.3 模式三：全局 SLAM (Global SLAM Mode)

**描述**：完整 SLAM 管线。关键帧 + 路标永久保存，回环检测 + PGO + 全局 BA。
参考 ORB_SLAM3 的三线程架构，但后端统一用 GTSAM iSAM2。

```
激活的因子:
  ✅ 模式二全部因子
  ✅ 回环检测: BetweenFactor<Pose3> (回环边)
  ✅ 全局 PGO: 在独立线程中运行 GTSAM LevenbergMarquardt
  ❌ (PGO 和 ISAM2 通过 key 同步，不重复优化)

关键帧选择条件:
  1. 距上一关键帧 > min_interval (0.5s)
  2. 相对运动 > trans_thresh (0.5m) OR rot_thresh (15°)
  3. 当前帧特征与上一关键帧共享 < 60% (视点变化大)

地图管理:
  - 关键帧: vector<KeyFrame> (shared_ptr) 永久保存
  - 路标: map<id, MapPoint> 被观测次数 > 2 的保留
  - 冗余剔除: 被观测次数 < 3 且创建时间 > 5s 的路标删除
  - covisibility graph: 维护关键帧间共视权重矩阵
```


### 4.4 模式切换机制

```cpp
enum class SlamMode { ODOMETRY, LOCAL_MAPPING, GLOBAL_SLAM };

bool switchMode(SlamMode target) {
    if (target == _current_mode) return true;

    // ODOMETRY → LOCAL_MAPPING 或 GLOBAL_SLAM:
    //   将历史视觉 SmartFactor 替换为显式路标因子
    //   增广状态，加入路标变量
    if (_current_mode == ODOMETRY && target >= LOCAL_MAPPING) {
        for (auto& feat : recent_features) {
            addLandmarkVariable(feat.id, feat.triangulated_3d);
            replaceSmartFactor(feat.id);
        }
    }

    // LOCAL_MAPPING → ODOMETRY:
    //   将显式路标因子替换回 SmartFactor (Schur 消去)
    //   从状态中移除路标变量
    if (_current_mode >= LOCAL_MAPPING && target == ODOMETRY) {
        for (auto& lm : active_landmarks) {
            replaceToSmartFactor(lm.id);
            removeLandmarkVariable(lm.id);
        }
    }

    _current_mode = target;
    _isam->update();
    return true;
}
```

模式切换的关键是**路标参数化的双向转换**：SmartFactor (隐式) ↔ 显式 Factor。
这需要正确管理 GTSAM 中的 factor key 和 variable key 的增删。


## 5. 工程架构设计


### 5.1 编译与依赖

```
核心依赖（必需）:
  Eigen3       >= 3.3    线性代数
  GTSAM        >= 4.2    因子图 + iSAM2
  Boost        >= 1.65   filesystem, serialization, thread

可选依赖（按需激活）:
  OpenCV       >= 3.4    视觉前端 (ENABLE_OPENCV=ON)
  PCL          >= 1.10   LiDAR前端 (ENABLE_PCL=ON)
  Ceres        >= 2.1    初始化求解器 (ENABLE_CERES=ON)
  DBoW2/DBoW3            回环检测 (ENABLE_LOOP=ON)
  OpenMP                 特征提取并行化

ROS 依赖（可选）:
  rclcpp / roscpp        仅 wrapper 层需要 (ENABLE_ROS2=ON / ENABLE_ROS1=ON)
  sensor_msgs, nav_msgs, tf2, cv_bridge

构建: CMake 3.16+, 无 catkin/ament 强制要求
  mkdir build && cd build && cmake .. && make -j
```


### 5.2 目录结构

```
phad_fusion/
├── CMakeLists.txt
├── cmake/                   # FindGTSAM.cmake, CompilerFlags.cmake
├── config/                  # 默认 YAML 配置模板
│   ├── system.yaml
│   └── sensors/
├── include/phad_fusion/
│   ├── core/
│   │   ├── pipeline.h       # 主调度器
│   │   ├── config.h         # 配置加载
│   │   └── mode.h           # SlamMode 枚举
│   ├── types/               # 所有数据结构
│   │   ├── state_variable.h
│   │   ├── pose.h
│   │   ├── camera_rig.h
│   │   ├── camera_model.h
│   │   ├── frame.h
│   │   ├── keyframe.h
│   │   ├── map_point.h
│   │   └── sensor_data.h
│   ├── frontend/
│   │   ├── frontend_base.h  # 抽象传感器前端
│   │   ├── visual_frontend.h
│   │   └── lidar_frontend.h
│   ├── preintegration/
│   │   ├── imu_preintegrator.h
│   │   └── preint_manager.h
│   ├── backend/
│   │   ├── factor_graph.h   # GTSAM ISAM2 wrapper
│   │   ├── factor_builder.h # 因子构造工厂
│   │   └── sliding_window.h
│   └── transport/           # ROS Wrapper (可选)
│       ├── ros2_adapter.h
│       └── direct_adapter.h
├── src/                     # 实现
│   └── ... (对应 include/ 的 .cpp 文件)
├── tests/                   # 单元测试 + 集成测试
│   ├── test_imu_preint.cpp
│   ├── test_icp_factor.cpp
│   └── ...
└── examples/                # 使用示例
    ├── simple_vio.cpp
    └── euroc_benchmark.cpp
```


### 5.3 线程模型

```
┌──────────────────┐
│  Sensor Thread    │  ROS callback 或 文件读取回调
│  数据接收 + 缓冲   │  → 写入各传感器 ThreadsafeQueue
└──────┬───────────┘
       │ (SensorPacket)
┌──────▼───────────┐
│  Frontend Thread  │  20-50Hz
│  特征/ICP处理      │  → 输出: FeatureTrack / IcpResult / GnssObs
│  三角化            │  → 写入 FrontendResultQueue
└──────┬───────────┘
       │ (FrontendResult)
┌──────▼───────────┐
│  Backend Thread   │  10-20Hz
│  因子图构建        │  → 插入新状态节点 + 因子
│  ISAM2 增量优化    │  → 边缘化
│  模式管理          │  → 输出: NavState
└──────┬───────────┘
       │ (LoopCandidate)
┌──────▼───────────┐
│  Loop Thread      │  独立频率 ~1-5Hz
│  回环检测+验证     │  → DBoW2/ScanContext
│  PGO + 全局BA     │  → 回环因子写入 Backend Thread
└──────────────────┘

线程间通信: 全部通过 lock-free SPSC queues (moodycamel::ConcurrentQueue)
或 std::mutex + std::condition_variable (低频路径)
```


### 5.4 内存管理

关键帧和路标的生命周期：
- 滑动窗口内：shared_ptr 在 ISAM2 中持有，不可删除
- 窗口外：仅 GLOBAL_SLAM 模式保留。其他模式下 shared_ptr 引用计数归零后自动回收
- 地图序列化：Boost.Serialization 支持 checkpoint/恢复 (离线→在线切换)

内存预算（典型配置）：
- ODOMETRY: ~20MB（12帧 × 200特征 × 2KB）
- LOCAL_MAPPING: ~50MB（15帧 + 500路标）
- GLOBAL_SLAM: ~500MB（1000关键帧 × 2000路标, 可通过 LRU 缓存限制）


## 6. 与 SOTA 方法的定量对比预期

| 指标 | open_vins | VINS-Fusion | ORB-SLAM3 | phad_fusion 目标 |
|---|---|---|---|---|
| EuRoC MH_01 ATE | 0.11m | 0.08m | 0.035m (Stereo-Inertial) | <0.05m |
| EuRoC V1_03 ATE | 0.21m | 0.18m | 0.08m | <0.15m |
| 特征前端速度 | ~5ms (FAST+KLT) | ~12ms (KLT+反验证) | ~20ms (ORB+描述子) | <10ms |
| 后端速度 | ~3ms (EKF udpate) | ~20ms (sw BA) | ~30ms (Local BA) | <20ms |
| 支持 LiDAR | 否 | 否 | 否 | ✅ |
| 支持 GNSS | 否 (外部) | GPS 松耦合 | 否 | ✅ (位置因子 + 未来 RTK) |
| 模式切换 | 无 | 无 | 纯定位模式 | ✅ 3 模式 |
| 在线标定 | ✅ | 固定 | 否 | ✅ (扩展) |
| 增量优化 | ✅ (EKF) | ❌ (重做 sw BA) | ❌ (重做 BA) | ✅ (iSAM2) |
| 多线程架构 | 部分 | 单线程为主 | ✅ 三线程 | ✅ 四线程 |
| 跨平台 | ARM/Ubuntu | Ubuntu | Ubuntu | ARM/X86/Linux |


## 7. 实现路线图


### Phase 1：骨架 + 类型系统（当前）

```
产出:
  - CMake 项目, 纯 C++ 库编译为 libphad_fusion.so
  - include/phad_fusion/types/ 全部头文件
  - include/phad_fusion/core/config.h
  - src/ 空壳实现
  - tests/test_types.cpp (CameraRig 构造、投影/反投影测试)

里程碑: 编译通过 + 类型系统单元测试 100% 通过
```


### Phase 2：IMU 预积分 + GTSAM 后端最小闭环

```
产出:
  - src/preintegration/imu_preintegrator.cpp (GTSAM PreintegratedCombinedMeasurements)
  - src/backend/factor_graph.cpp (ISAM2 初始化/更新/边缘化)
  - src/core/pipeline.cpp 单传感器 (仅 IMU) 跑通
  - tests/test_imu_preint.cpp (仿真数据验证预积分精度)

里程碑: 纯 IMU 积分管道跑通, 输出连续轨迹
```


### Phase 3：视觉前端 + 视觉惯性里程计

```
产出:
  - src/frontend/visual_frontend.cpp (FAST+KLT+RANSAC)
  - SmartStereoFactor wrapper (复用 gtsam_points 或自实现)
  - 双目/单目 VIO 模式完整跑通
  - EuRoC 数据集 Benchmark

里程碑: ODOMETRY 模式 VIO, EuRoC MH_01 ATE < 0.10m
```


### Phase 4：LiDAR 前端 + LIO

```
产出:
  - src/frontend/lidar_frontend.cpp (genz-icp + 去畸变)
  - IntegratedGICPFactor (参考 gtsam_points)
  - LIO 模式: LiDAR + IMU
  - KITTI benchmark

里程碑: ODOMETRY 模式 LIO, KITTI seq 00 translation error < 1.2%
```


### Phase 5：GNSS + 多传感器融合

```
产出:
  - GNSS 位置因子 + WGS84 ↔ ENU 转换
  - 多传感器因子图同时优化 (VIO + LIO + GNSS)
  - 传感器融合权重自动调节 (信息矩阵正确)

里程碑: 多传感器融合精度 > 单独传感器最好精度
```


### Phase 6：回环检测 + PGO + 全局 BA

```
产出:
  - DBoW2/DBoW3 视觉回环
  - ScanContext LiDAR 回环
  - BetweenFactor<Pose3> PGO
  - 全局 BA 独立线程

里程碑: GLOBAL_SLAM 模式, 长轨迹漂移 < 0.3% distance
```


### Phase 7：工程化

```
产出:
  - ROS2 Wrapper (transport/ros2_adapter)
  - 在线标定 (相机内参/外参/IMU bias/时间偏移)
  - 配置文件热加载
  - 测试覆盖率 >70%
  - Docker 镜像

里程碑: 一键部署运行, 全数据集 Benchmark 通过
```


### Phase 8：SOTA 优化

```
产出:
  - 深度学习特征 (SuperPoint/XFeat) 替代 OpenCV
  - GPU 加速前端
  - 在线学习传感器噪声模型
  - 自适应窗口大小和模式切换策略
  - 嵌入式部署优化 (ARM NEON, TFLite)

里程碑: 在 EuRoC/KITTI/TUM-VI 上达到 SOTA
```


## 8. 附录：关键设计对比表

| 设计点 | open_vins | VINS-Fusion | ORB-SLAM3 | phad_fusion 选择 | 理由 |
|---|---|---|---|---|---|
| 特征提取 | FAST+KLT | KLT+反验证 | ORB (自写) | FAST+KLT + 可选 ORB/SP | 最灵活 |
| RANSAC | F矩阵归一化坐标 | F矩阵+反验证 | F/H自适应 | F矩阵归一化坐标 | open_vins 已验证简单有效 |
| 后端 | EKF+FEJ | Ceres swBA | G2O BA | GTSAM iSAM2 | 增量优化 + 严格 Schur 补 |
| 边缘化 | state丢弃 | 伪逆截断 | G2O marginal | ISAM2 auto | 最正确 |
| 路标参数化 | XYZ/anchor/ inv depth | 逆深度 | XYZ (ORB坐标系) | XYZ + inv depth 双模式 | 近=XYZ, 远/uncertain=inv depth |
| IMU 积分 | 离散/RK4/ ACI2 | 中值法 | 中值法 | Preintegrated ImuMeasure- ments | GTSAM 标准实现 |
| GNSS | 无核心支持 | GPS松耦 | 无 | GTSAM位置因子 | 因子图天然支持 |
| LiDAR ICP | 无 | 无 | 无 | genz-icp自适应 GICP | 退化场景鲁棒 |
| ROS 隔离 | ENABLE_ROS 条件编译 | 紧耦合 | 紧耦合 | ENABLE_ROS 条件编译 + transport 层 | 参考 fusions_slam |

