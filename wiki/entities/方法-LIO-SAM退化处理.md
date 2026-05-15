---
type: entity
tags: [退化, LIO-SAM, 因子图, IMU预积分, 噪声矩阵, 特征值分解, 投影修正]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/LIO-SAM
---

# LIO-SAM 退化处理

> LIO-SAM 在两处处理退化：scan-matching 层用 Hessian 特征值分解 + 投影矩阵 `matP` 抑制退化方向增量；因子图层用二元噪声切换 `correctionNoise` / `correctionNoise2` 控制 LiDAR 先验权重。

## 双层退化处理架构

```
┌─ Scan-Matching 层 (mapOptmization.cpp) ─────────────────┐
│  LM 迭代第 0 次：                                         │
│    matAtA = J^T J  (6×6 Hessian 近似)                     │
│    特征值分解: cv::eigen(matAtA, matE, matV)               │
│    退化判定: λ_i < eignThre[100,100,100,100,100,100]      │
│    退化投影: matP = V^{-1} · V'  (置零退化方向行)          │
│    matX = matP · matX2  (仅保留良态方向增量)               │
│    输出: isDegenerate flag + covariance[0]=1              │
└──────────────────────────────────────────────────────────┘
                          │
                          ▼  covariance[0] = 1 (退化)
┌─ 因子图层 (imuPreintegration.cpp) ───────────────────────┐
│    degenerate = (covariance[0] == 1)                      │
│    noise_model = degenerate ? correctionNoise2            │
│                             : correctionNoise              │
│    correctionNoise  = diag(0.05, 0.05, 0.05, 0.1,0.1,0.1)│
│    correctionNoise2 = diag(1,    1,    1,    1,  1,  1  ) │
│    graphFactors.add(PriorFactor<Pose3>(X(key), pose,      │
│                                        noise_model))       │
└──────────────────────────────────────────────────────────┘
```

## 第一层：Scan-Matching 退化投影

### 数学原理（Zhang et al. 2016）

LM 优化的法方程为 $J^T J \Delta x = -J^T r$，其中 $J^T J$ 是 6×6 近似的 Hessian。

特征值分解：
$$J^T J = V \Lambda V^T, \quad \Lambda = \text{diag}(\lambda_1, \ldots, \lambda_6)$$

退化判定：若 $\lambda_i < \theta$（阈值 100），则第 i 个特征方向为退化方向。

构造投影矩阵 $P$，将增量 $\Delta x$ 投影到良态子空间：
$$V' = \text{diag}(v_1, \ldots, v_6), \quad v_i = \mathbb{1}[\lambda_i \ge \theta]$$
$$P = V^{-1} \cdot V'$$
$$\Delta x_{\text{safe}} = P \cdot \Delta x$$

投影后退化方向增量为零，仅非退化方向保留原始解。

### 源码

```cpp
// mapOptmization.cpp:L1229-L1258 — LMOptimization() 中的退化处理
if (iterCount == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);    // 特征值分解
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {    // 从最小特征值开始
        if (matE.at<float>(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
                matV2.at<float>(i, j) = 0;  // 退化方向行置零
            }
            isDegenerate = true;
        } else {
            break;  // 排序后一旦遇到良态特征值即停止
        }
    }
    matP = matV.inv() * matV2;  // 投影矩阵
}

if (isDegenerate) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;  // 投影：退化方向归零
}
```

### 退化信号传递

scan-matching 完成后通过 `covariance[0]` 向下游传递退化标志：

```cpp
// mapOptmization.cpp:L1696-L1699
if (isDegenerate)
    laserOdomIncremental.pose.covariance[0] = 1;
else
    laserOdomIncremental.pose.covariance[0] = 0;
```

## 第二层：因子图噪声调节

### 原理

IMU 预积分因子图中，每个 LiDAR 帧作为 `PriorFactor<Pose3>` 加入。该先验因子的信息矩阵（噪声协方差的逆）决定了 LiDAR 观测的权重：

- **正常状态** (`correctionNoise`)：旋转噪声 $\sigma_r = 0.05$ rad，平移噪声 $\sigma_t = 0.1$ m
  $$\Sigma_{\text{normal}} = \text{diag}(0.05^2, 0.05^2, 0.05^2, 0.1^2, 0.1^2, 0.1^2)$$
- **退化状态** (`correctionNoise2`)：旋转和平移噪声均增大到 $\sigma = 1$
  $$\Sigma_{\text{degen}} = \text{diag}(1^2, 1^2, 1^2, 1^2, 1^2, 1^2)$$

信息矩阵从 $\approx \frac{1}{0.01}$ 降到 $\approx 1$，即权重降低约 100 倍。退化时降低 LiDAR 先验权重，让 IMU 预积分因子主导状态估计。

### 源码

```cpp
// imuPreintegration.cpp:L171-L224 — 噪声模型定义
gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
// 正常噪声：旋转 0.05 rad, 平移 0.1 m
correctionNoise = gtsam::noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
// 退化噪声：旋转 1 rad, 平移 1 m (≈ 降低 100 倍权重)
correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas(
    (Vector(6) << 1, 1, 1, 1, 1, 1).finished());
```

```cpp
// imuPreintegration.cpp:L269-L378 — 退化感知的因子构建
bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
// ...
gtsam::PriorFactor<gtsam::Pose3> pose_factor(
    X(key), curPose,
    degenerate ? correctionNoise2 : correctionNoise  // 二元切换
);
graphFactors.add(pose_factor);
```

## 伪代码

```pseudo
function LIO_SAM_ScanMatching_Degeneracy(source_pts, map, initial_pose):
    // === 第一层: LM 退化投影 ===
    for iter in range(max_iterations):
        J, r = build_jacobian_and_residual(source_pts, map, current_pose)
        H = J^T * J                    // 6×6 近似 Hessian
        delta = solve(H * delta = -J^T * r)

        if iter == 0:
            eigenvalues, V = eigen_decompose(H)
            V_prime = copy(V)
            is_degenerate = false
            threshold = [100, 100, 100, 100, 100, 100]
            for i from 5 down to 0:    // 从最小特征值
                if eigenvalues[i] < threshold[i]:
                    V_prime[i, :] = 0  // 退化方向行置零
                    is_degenerate = true
                else:
                    break
            P = V^{-1} * V_prime       // 投影矩阵

        if is_degenerate:
            delta = P * delta          // 投影到良态子空间

        current_pose += delta
        if converged(delta):
            break

    return current_pose, is_degenerate

function LIO_SAM_FactorGraph_Degeneracy(imu_preint, lidar_pose, is_degenerate):
    // === 第二层: 噪声调节 ===
    graph = NonlinearFactorGraph()
    graph.add(ImuFactor(X(k-1), V(k-1), X(k), V(k), B(k-1), imu_preint))

    if is_degenerate:
        noise = diag(1, 1, 1, 1, 1, 1)    // 大噪声 → IMU 主导
    else:
        noise = diag(0.05, 0.05, 0.05, 0.1, 0.1, 0.1)  // 小噪声 → LiDAR 主导

    graph.add(PriorFactor<Pose3>(X(k), lidar_pose, noise))
    result = optimizer.optimize(graph)
    return result
```

## Agent 实现提示

### 适用场景

- LiDAR-惯性紧耦合系统中 scan-matching 在几何退化环境下的鲁棒性增强
- 需要将退化信号从 scan-matching 层传递到因子图层做联合决策的场景
- 对计算开销敏感、偏好简单二元切换而非连续权重调节的系统

### 输入输出契约

- **Scan-matching 层输入**：当前帧特征点云（corner + surf）、局部地图、初始位姿 `transformTobeMapped[6]`
- **Scan-matching 层输出**：优化位姿、退化标志 `isDegenerate`、`covariance[0]` 退化信号
- **因子图层输入**：IMU 预积分测量、LiDAR 帧位姿、`covariance[0]` 退化信号
- **因子图层输出**：融合后位姿 `prevPose_`、速度 `prevVel_`、偏置 `prevBias_`

### 实现注意事项

- 特征值分解仅在 `iterCount == 0` 时进行（`mapOptmization.cpp:L1229`），后续迭代复用第一次的 `matP`，假定几何结构不变
- 特征值按降序排列，循环从 `i=5`（最小特征值）开始；一旦遇到 $\lambda_i \ge 100$ 即 `break`，因为排序保证了后续特征值只会更大
- `correctionNoise2` 的噪声值（1 rad / 1 m）是经验值，过大可能导致 IMU 漂移主导、过小则退化方向约束不足
- 退化时 `correctionNoise2` 是 6-DoF 统一放大，不区分具体退化方向——比 SuperLoc 的按 DoF 加权粗粒度

### 源码检索锚点

- 退化特征值判定与投影：`raw/codes/LIO-SAM/src/mapOptmization.cpp:L1229-L1258`
- 退化标志定义：`raw/codes/LIO-SAM/src/mapOptmization.cpp:L135`
- 退化信号发布：`raw/codes/LIO-SAM/src/mapOptmization.cpp:L1696-L1699`
- 噪声模型定义：`raw/codes/LIO-SAM/src/imuPreintegration.cpp:L171-L224`
- 退化感知因子构建：`raw/codes/LIO-SAM/src/imuPreintegration.cpp:L269-L378`

## 与 SuperOdom 对比

| 维度 | LIO-SAM | SuperOdom/SuperLoc |
|------|---------|-------------------|
| 退化检测方法 | Hessian $J^T J$ 特征值阈值 | 协方差 $C^{-1}$ 特征值 + 特征可观测性直方图 |
| 退化判定粒度 | 二元（退化/非退化） | 6-DoF 连续不确定性 [0,1] |
| 优化层修正 | 投影矩阵 $P$ 归零退化增量 | SE3 先验约束按 $(1-\text{unc})$ 加权 |
| 因子图层补偿 | 二元噪声切换（100 倍差异） | 预测源切换（VIO/Neural-IMU） |
| yaw 处理 | 统一处理 | yaw 方向信息矩阵置零，自由估计 |
| 实时不确定性发布 | 无 | 6 个 ROS topic 实时发布 |

## 相关页面

- [[方法-6-DoF 退化检测]]
- [[方法-SuperLoc退化检测实现]]
- [[概念-IMU预积分]]
- [[算法-LIO-SAM]]
- [[方法-ICP配准方法]]
