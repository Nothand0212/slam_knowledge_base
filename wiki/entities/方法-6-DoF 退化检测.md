---
type: entity
tags: [退化检测, LiDAR, 可观测性, 位姿估计, SuperLoc]
created: 2026-04-29
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-rolo-analysis.md
  - wiki/sources/2026-04-28-superodom-analysis.md
  - raw/codes/SuperOdom
  - raw/codes/LIO-SAM
---

# 6-DoF 退化检测

> 分别评估位姿 6 个自由度的可观测性，在几何退化方向上降低更新或注入先验，避免 LiDAR 配准把不可观测运动估计成确定结果。

## 定义

SuperOdom/SuperLoc 中实现的对 LiDAR 扫描匹配的 6 个自由度分别进行可观测性评估和退化判定，在退化方向上注入先验约束防止位姿突变的机制。

## 为什么需要

LiDAR 配准不是所有方向都同样可观。长走廊、平面墙、开阔地或越野地形会让某些平移/旋转方向缺少几何约束。如果仍按完整 6-DoF 更新，优化器可能在退化方向产生看似收敛但实际错误的增量。

## 核心特征

- **SuperOdom**：基于特征可观测性直方图 PlaneFeatureHistogramObs[9] 统计，输出 uncertainty_x/y/z/roll/pitch/yaw 6 维不确定性
- **ROLO**：LM 迭代中海森矩阵特征值 < 100 时启用退化投影 matP = V⁻¹·V'，限制退化方向更新
- 退化响应：注入 SE3AbsolutatePoseFactor（先验约束）限制退化自由度
- 通过 ROS topic 实时发布各方向不确定性，可用作下游规划器的可信度输入
- 比简单拒绝匹配更精细：部分退化时仍可利用非退化方向信息

## 处理策略

| 策略 | 做法 | 适用 |
|------|------|------|
| 投影修正 | 对 Hessian 特征值过小的方向抑制增量 | LIO-SAM/ROLO 类局部优化 |
| 噪声放大 | 退化时增大观测噪声 | 因子图或滤波器后端 |
| 先验注入 | 对退化自由度加入弱先验 | SuperOdom/SuperLoc |
| 多源补充 | 用 IMU、GNSS、视觉或轮速补足退化方向 | 多传感器融合系统 |

## 数学基础

### 退化判定的核心思想

对于 scan-to-map 优化问题 $\min_{\Delta x} \|J \Delta x + r\|^2$，法方程为：

$$H \Delta x = -J^T r, \quad H = J^T J$$

其中 $H \in \mathbb{R}^{6 \times 6}$ 是近似 Hessian 矩阵。特征值分解：

$$H = V \Lambda V^T, \quad \Lambda = \text{diag}(\lambda_1, \lambda_2, \ldots, \lambda_6)$$

特征值 $\lambda_i$ 反映了第 $i$ 个特征方向（$V$ 的第 $i$ 列）上的几何约束强度：

- $\lambda_i$ 大 → 该方向约束充足，可正常更新
- $\lambda_i \approx 0$ → 该方向退化，更新量不可信

### 条件数

协方差矩阵 $C = H^{-1}$，对其平移/旋转子块分别做特征值分解可得到：

$$\kappa_{\text{pos}} = \sqrt{\lambda_t^{\max}} / \sqrt{\lambda_t^{\min}}, \quad \kappa_{\text{rot}} = \sqrt{\lambda_r^{\max}} / \sqrt{\lambda_r^{\min}}$$

条件数倒数 $\kappa^{-1}$ 越接近 0，该子空间越病态。

## SuperOdom SuperLoc 实现

SuperLoc 采用**双层退化感知**：特征层 + 优化层。详见 [[方法-SuperLoc退化检测实现]]。

### 第一层：特征可观测性直方图

对每个匹配的平面特征做 9 维可观测性分析（6 个旋转方向 + 3 个平移方向），建立直方图 `PlaneFeatureHistogramObs[9]`，归一化为 6 维不确定性（0~1）。

```cpp
// LidarSlam.cpp:L921-L959 — EstimateLidarUncertainty()
// 平移不确定性：平面法向量在该轴向的分量越大，该轴向越可观测
uncertaintyX = (PlaneFeatureHistogramObs.at(6) / TotalTransFeature) * 3;
// 旋转不确定性：cross(p, n) 在该轴向的分量越大，该旋转方向越可观测
uncertaintyRoll = ((PlaneFeatureHistogramObs.at(0) + PlaneFeatureHistogramObs.at(1)) / TotalRotationFeature) * 3;
```

### 第二层：Ceres 协方差分解

优化结束后通过 Ceres Covariance API 获取参数块协方差，对平移/旋转子块分别做 `SelfAdjointEigenSolver` 分解，提取最大误差方向和条件数。

```cpp
// LidarSlam.cpp:L860-L894 — EstimateRegistrationError()
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3,3>());
err.PositionError = sqrt(eigPosition.eigenvalues()(2));       // 最大平移标准差
err.PosInverseConditionNum = sqrt(λ0) / sqrt(λ2);             // 条件数倒数

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3,3>());
err.OrientationError = Rad2Deg(sqrt(eigOrientation.eigenvalues()(2)));
```

### 退化响应

1. **预测源切换**（`laserMapping.cpp:L384-L411`）：`isDegenerate` 时优先使用 VIO 或 Neural-IMU 预测
2. **加权先验注入**（`LidarSlam.cpp:L287-L303`）：对退化 DoF 按 $(1-\text{uncertainty})$ 注入 SE3 绝对位姿约束，yaw 方向信息矩阵置零

## LIO-SAM 退化补偿

LIO-SAM 在 scan-matching 和因子图两层处理退化。详见 [[方法-LIO-SAM退化处理]]。

### 第一层：Scan-Matching 退化投影

在 LM 优化首轮迭代中，对 $H = J^T J$ 做特征值分解：

```cpp
// mapOptmization.cpp:L1229-L1258 — LMOptimization() 退化投影
cv::eigen(matAtA, matE, matV);           // 特征值分解
// 退化判定：λ_i < 100 → 退化方向
for (int i = 5; i >= 0; i--) {
    if (matE.at<float>(0, i) < eignThre[i]) {
        matV2.at<float>(i, j) = 0;       // 退化方向行置零
        isDegenerate = true;
    } else break;
}
matP = matV.inv() * matV2;               // 投影矩阵
matX = matP * matX2;                     // 投影：退化方向增量为零
```

### 第二层：因子图噪声调节

退化时通过 `covariance[0] = 1` 传递信号到 IMU 预积分因子图，将 LiDAR 先验因子的噪声从 `correctionNoise` 切换到 `correctionNoise2`：

```cpp
// imuPreintegration.cpp:L223-L224 — 噪声定义
correctionNoise  = diag(0.05, 0.05, 0.05, 0.1, 0.1, 0.1);  // 正常 → LiDAR 主导
correctionNoise2 = diag(1,    1,    1,    1,   1,   1);      // 退化 → IMU 主导

// imuPreintegration.cpp:L378 — 二元切换
gtsam::PriorFactor<Pose3> pose_factor(X(key), curPose,
    degenerate ? correctionNoise2 : correctionNoise);
```

噪声放大 ~100 倍使 LiDAR 先验权重降低 ~100 倍，IMU 预积分因子主导位姿估计。

## SuperOdom vs LIO-SAM 对比

| 维度 | SuperOdom/SuperLoc | LIO-SAM |
|------|-------------------|---------|
| **退化检测方法** | 特征可观测性直方图 + Ceres 协方差分解 | $J^T J$ 特征值分解 |
| **判定粒度** | 6-DoF 连续不确定性 [0,1] | 二元（退化/非退化） |
| **检测级数** | 特征层 → 优化层（双层） | 优化层（单层） |
| **优化层修正** | SE3 先验约束按 DoF 加权注入 | 投影矩阵归零退化方向增量 |
| **因子图层补偿** | 预测源切换（VIO/Neural-IMU） | 噪声二元切换（IMU 主导） |
| **yaw 方向** | 信息矩阵置零，自由估计 | 统一特征值阈值处理 |
| **不确定性发布** | 6 个 ROS topic 实时输出 | 无 |
| **ROS 接口** | `Float32` × 6 topics | `covariance[0]` 单值二值信号 |
| **适用场景** | 多传感器融合、需不确定性量化 | LiDAR-惯性紧耦合、简单退化场景 |
| **子空间投影** | 按 (1-unc) 连续权重 | matP = V⁻¹·V' 硬投影 |
| **在线实时性** | 每帧做 Ceres Covariance + 直方图 | 仅首轮迭代做一次特征值分解 |

## Agent 实现提示

### 适用场景

- LiDAR 扫描匹配中某些方向几何约束不足时（长廊、平面墙、开阔地），需要对退化方向抑制更新
- 需要实时输出 6-DoF 不确定性（下游规划器/融合模块的可信度输入）
- 多传感器融合系统中需要根据退化状态动态切换位姿预测源（VIO/Neural-IMU/LiDAR）

### 输入输出契约

**SuperLoc 退化检测（双层）**
- 第一层输入：匹配的平面特征集合 `PlaneFeatureHistogramObs[9]`（6 旋转 + 3 平移可观测性）
- 第一层输出：`uncertainty_x/y/z/roll/pitch/yaw` 6 维归一化不确定性 [0,1]
- 第二层输入：Ceres 优化后参数块协方差矩阵
- 第二层输出：`PositionError`（最大平移标准差）、`OrientationError`（最大旋转标准差，deg）、`PosInverseConditionNum`（条件数倒数）

**LIO-SAM 退化投影**
- 输入：首轮 LM 迭代的法方程矩阵 `matAtA = J^T J`（6×6）
- 输出：投影矩阵 `matP = V⁻¹·V'`（退化方向行置零后的 V 与原 V⁻¹ 乘积），退化标志 `isDegenerate`
- 阈值：`eignThre[6] = {100, 100, 100, 100, 100, 100}`

### 实现骨架（伪代码）

```pseudo
# === SuperLoc 双层退化检测 ===

def DetectDegeneracy_SuperLoc(matched_planes, ceres_covariance):
    # 第一层：特征可观测性直方图
    PlaneFeatureHistogramObs[9] = [0]*9  # 6 rot + 3 trans
    for plane in matched_planes:
        centroid_p = plane.centroid
        normal_n = plane.normal.normalized()
        rot_moment = cross(centroid_p, normal_n)
        # 累积各方向可观测性分量
        PlaneFeatureHistogramObs[0] += abs(rot_moment.x())  # roll
        PlaneFeatureHistogramObs[1] += abs(rot_moment.y())  # pitch
        PlaneFeatureHistogramObs[2] += abs(rot_moment.z())  # yaw
        PlaneFeatureHistogramObs[6] += abs(normal_n.x())    # x-trans
        PlaneFeatureHistogramObs[7] += abs(normal_n.y())    # y-trans
        PlaneFeatureHistogramObs[8] += abs(normal_n.z())    # z-trans
    # 归一化到 [0,1]，0=不可观，1=完全可观
    uncertainty = 1.0 - min(histogram / max_obs_count, 1.0)

    # 第二层：Ceres 协方差分解
    cov = ceres_covariance.topLeftCorner<3,3>()
    eig = SelfAdjointEigenSolver(cov)
    pos_error = sqrt(eig.eigenvalues()(2))       # 最大标准差
    inv_cond = sqrt(λ_min) / sqrt(λ_max)          # 条件数倒数
    return uncertainty, pos_error, inv_cond

# === LIO-SAM 退化投影 ===

def DetectDegeneracy_LIOSAM(J_r, residual):
    H = J_r.T * J_r                          # 6×6 近似 Hessian
    eigenvalues, V = eigen(H)                 # 特征值分解
    V2 = copy(V)
    is_degenerate = False
    for i in range(5, -1, -1):               # 从最小特征值检查
        if eigenvalues[i] < 100:
            V2.row(i).setZero()              # 退化方向行置零
            is_degenerate = True
        else:
            break
    P = V.inv() * V2                         # 投影矩阵
    dx_corrected = P * dx_raw                # 退化方向增量归零
    return dx_corrected, is_degenerate
```

### 关键源码片段

`raw/codes/LIO-SAM/src/mapOptmization.cpp:L1229-L1258` — LIO-SAM 退化投影核心：
```cpp
if (iterCount == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);
    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
        if (matE.at<float>(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
                matV2.at<float>(i, j) = 0;
            }
            isDegenerate = true;
        } else { break; }
    }
    matP = matV.inv() * matV2;
}
if (isDegenerate) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
}
```

`raw/codes/SuperOdom/super_odometry/src/LidarProcess/LidarSlam.cpp:L860-L894` — Ceres 协方差分解（伪代码还原）：
```cpp
// EstimateRegistrationError — 需定位到 LidarSlam.cpp
Covariance::Options cov_options;
Covariance covariance(cov_options);
covariance.Compute(parameter_blocks, &problem);
double cov_mat[36];
covariance.GetCovarianceMatrix(parameter_blocks, cov_mat);
Eigen::Map<Eigen::Matrix<double,6,6>> errCov(cov_mat);

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPos(errCov.topLeftCorner<3,3>());
err.PositionError = sqrt(eigPos.eigenvalues()(2));       // 最大平移标准差
err.PosInverseConditionNum = sqrt(λ0) / sqrt(λ2);        // 条件数倒数

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOri(errCov.bottomRightCorner<3,3>());
err.OrientationError = Rad2Deg(sqrt(eigOri.eigenvalues()(2)));  // 最大旋转标准差
```

### 实现注意事项

1. **退化判定时机**：LIO-SAM 的退化投影仅在首轮 LM 迭代（`iterCount == 0`）做一次，后续迭代复用同一个投影矩阵 `matP`。SuperLoc 则每帧都做双层检测。
2. **从最小特征值遍历**：LIO-SAM 从索引 5（最小特征值）向 0 遍历，遇到第一个满足阈值的就 `break`。这确保只有连续的小特征值方向被抑制，避免过度投影。
3. **噪声调节 vs 投影修正**：LIO-SAM 用投影矩阵归零退化方向增量（硬抑制），而因子图层面通过噪声二元切换（`correctionNoise2` 放大 ~100 倍）使 IMU 主导。两者是互补的。
4. **yaw 方向特殊处理**：SuperLoc 对 yaw 方向信息矩阵置零（自由估计），因为 LiDAR 在平面环境中 yaw 通常不可观。LIO-SAM 用统一阈值处理所有方向。
5. **实时性权衡**：Ceres Covariance 计算代价较高（需要求解法方程逆），SuperLoc 的协方差层可能影响实时性。第一层特征直方图计算开销小，可每帧执行。

### 源码检索锚点

| 系统 | 文件 | 功能 | 行号 |
|------|------|------|------|
| LIO-SAM | `mapOptmization.cpp` | 退化投影（Hessian 特征值分解） | L1229–L1258 |
| LIO-SAM | `imuPreintegration.cpp` | 噪声二元切换（`correctionNoise`/`correctionNoise2`） | L223–L224, L378 |
| SuperOdom | `LidarSlam.cpp` | EstimateLidarUncertainty（特征直方图） | L921–L959 |
| SuperOdom | `LidarSlam.cpp` | EstimateRegistrationError（Ceres 协方差分解） | L860–L894 |
| SuperOdom | `LidarSlam.cpp` | SE3 先验约束注入（退化 DoF 加权） | L287–L303 |
| SuperOdom | `laserMapping.cpp` | 预测源切换（VIO/Neural-IMU 降级） | L384–L411 |
| ROLO | `backMapping.cpp` | 退化投影（Hessian 特征值 <100） | L844–L866 |

## 相关页面

- 实现：[[方法-SuperLoc退化检测实现]] `LidarSlam.cpp:921-992`、[[算法-ROLO-SLAM]] `backMapping.cpp:844-866`
- [[方法-LIO-SAM退化处理]]
- [[方法-多源位姿预测策略]]
- [[方法-退化检测与修复]]
- [[算法-LIO-SAM]]
- [[LiDAR数据管线]]
