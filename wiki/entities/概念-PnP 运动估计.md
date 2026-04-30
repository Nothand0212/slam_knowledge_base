---
tags: [PnP, 3D-2D, 运动估计, 位姿估计, RANSAC, EPnP, 重定位]
sources: [wiki/entities/视觉特征跟踪, wiki/topics/相机数据管线]
created: 2026-04-29
updated: 2026-04-30
type: entity
---

# PnP 运动估计（3D-2D）

> Perspective-n-Point：已知 3D 地图点和它们在当前帧的 2D 投影，求解相机位姿。这是视觉 SLAM 中从"帧间跟踪（2D-2D）"跃迁到"地图定位（3D-2D）"的关键桥梁。

---

## 1. 与 2D-2D 的关系

视觉特征跟踪中，2D-2D（帧间匹配）和 3D-2D（PnP）是互补的两个阶段：

```
首帧初始化 → 2D-2D 匹配 → 三角化得到 3D 点 → PnP 求解后续帧位姿
                                        ↓
                     3D-2D 匹配（投影/极线搜索）
                                        ↓
                             PnP + RANSAC 求解
                                        ↓
                              后端 BA 精化
```

- **2D-2D**：只有像素匹配，需要本质矩阵/基础矩阵分解 → 得到的是相对位姿（无尺度）
- **3D-2D**：已有三角化后的地图点 3D 坐标 → PnP 直接求解绝对位姿，且尺度已确定

---

## 2. PnP 问题定义

给定 $n$ 个 3D 点在世界坐标系下的坐标 $P_i = (X_i, Y_i, Z_i)$ 和它们在图像中的 2D 投影 $p_i = (u_i, v_i)$，求解从世界坐标系到相机坐标系的旋转 $R$ 和平移 $t$：

$$
s_i \begin{bmatrix} u_i \\ v_i \\ 1 \end{bmatrix} = K \begin{bmatrix} R & t \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}
$$

其中 $K$ 为相机内参矩阵（已知），$s_i$ 为未知深度因子。

---

## 3. 最小配置：P3P

理论上 3 个 3D-2D 对应即可求解 $[R|t]$（P3P），但会得到最多 4 个解（需要通过第 4 个点的重投影选择正确解）。P3P 使用三角形的余弦定理构建方程：

$$
\begin{cases}
d_1^2 + d_2^2 - 2d_1d_2\cos\theta_{12} = \|P_1 - P_2\|^2 \\
d_2^2 + d_3^2 - 2d_2d_3\cos\theta_{23} = \|P_2 - P_3\|^2 \\
d_3^2 + d_1^2 - 2d_3d_1\cos\theta_{31} = \|P_3 - P_1\|^2
\end{cases}
$$

其中 $d_i$ 为相机光心到 $P_i$ 的距离，$\theta_{ij}$ 由像素坐标和内参 $K$ 计算得到。

- **使用率**：RANSAC 内层（需要 3 个点）+ 第 4 点验证
- **问题**：只用 3 点，对噪声敏感，后续需要（可选）用更多点做非线性精化

---

## 4. 主流 PnP 求解方法

### 4.1 EPnP (Efficient PnP)

本知识库中**使用最广泛**的 PnP 方法。核心思想：用 4 个虚拟控制点（世界坐标系的原点和三个坐标轴方向点）的加权组合表示任意 3D 点，将问题降维为求解这 4 个控制点在相机坐标系下的坐标。

**算法流程**：
1. 选 4 个控制点 $c_j^w$（世界坐标系质心 + PCA 主方向）
2. 每个 3D 点 $P_i$ 表示为 $P_i = \sum_j \alpha_{ij} c_j^w$（$\alpha$ 由世界坐标唯一确定）
3. 在相机坐标系下同样有 $P_i^c = \sum_j \alpha_{ij} c_j^c$
4. 代入投影方程 → 线性系统求解 $c_j^c$（$12 \times 12$ 小矩阵）
5. Gauss-Newton 非线性精化，最小化重投影误差 $\sum_i \|\pi(RP_i + t) - p_i\|^2$

- **时间复杂度**：$O(n)$，对点数线性增长
- **所需最少点数**：4（非共面）或 6（共面）
- **OpenCV 接口**：`cv::solvePnP(p3d, p2d, K, distCoeffs, rvec, tvec, useExtrinsicGuess=false, SOLVEPNP_EPNP)`

### 4.2 UPnP (Uncalibrated PnP)

EPnP 的扩展，支持**未知焦距**的情况下同时求解相机位姿和焦距。ORB-SLAM3 的重定位模块使用。

### 4.3 DLT (Direct Linear Transform)

最朴素的线性方法：将投影方程重排为 $A p = 0$ 的形式（每个点贡献 2 个方程），用 SVD 求解。精度最低但速度最快。

### 4.4 P3P (Gao's 方法)

ORB-SLAM3 的 RANSAC 内层使用 Gao 等提出的高效 P3P 实现。返回最多 4 个解，用剩余点投票验证。

---

## 5. 各算法的 PnP 使用场景

| 算法 | PnP 方法 | 使用场景 | RANSAC 配置 |
|------|---------|---------|------------|
| **ORB-SLAM3** | EPnP + P3P(RANSAC) | 每帧运动模型引导的跟踪、重定位、回环验证 | P3P 最小子集 + EPnP 精化 |
| **VINS-Fusion** | PnP RANSAC (OpenCV) | 双目/Stereo-only 模式初始化、回环几何验证 | — |
| **open_vins** | （无独立PnP，MSCKF 直接状态增广） | — | — |
| **SVO Pro** | Sparse Image Align + 重投影优化 | 稀疏直接法前端定位 3D 地图点，然后对收敛的点做 PnP 式重投影精化 | — |
| **Kimera-VIO** | GTSAM SmartFactor（内嵌 3D-2D） | 特征跟踪 → triangulation → SmartProjectionPoseFactor | GTSAM RANSAC 内层 |
| **BEV-LSLAM** | ORB + PnP RANSAC | 回环检测几何验证 | — |
| **DROID-SLAM** | DBA（稠密 BA，替代 PnP） | 逐像素深度 + ConvGRU 迭代 → 不显式使用 PnP | — |

---

## 6. PnP + RANSAC 的标准流程

```python
best_inliers = {}
for i in range(max_iter):
    # 1. 随机采样最小子集
    subset = random.sample(matches, min_set_size)  # P3P=3, EPnP=4

    # 2. 求解 PnP
    R, t = solve_pnp(p3d[subset], p2d[subset], K)

    # 3. 计算 all 点的重投影误差
    errors = [\|project(P_i, R, t, K) - p_i\| for i in all_matches]

    # 4. 统计内点数（误差 < 阈值）
    inliers = [i for i, e in enumerate(errors) if e < threshold]

    # 5. 保留内点数最多的模型
    if len(inliers) > len(best_inliers):
        best_inliers = inliers

# 6. 用所有内点精化模型
R_final, t_final = solve_pnp_refine(p3d[best_inliers], p2d[best_inliers], K)
```

**关键参数**：

| 参数 | ORB-SLAM3 | VINS-Fusion | 说明 |
|------|-----------|-------------|------|
| 最小子集大小 | 4 (EPnP) / 3 (P3P) | 4 | P3P 返回多解，需要第 4 点验证 |
| 最大迭代次数 | 300 | 300 | `1 - (1 - 内点率^min_set)^N < 99%` |
| 重投影误差阈值 | $\sigma$ × 卡方对应值 | 3.0 px | 归一化坐标或像素 |

---

## 7. PnP 的退化与对策

### 7.1 共面退化解

当 3D 点近似共面时，EPnP 的线性方程组病态。ORB-SLAM3 的检测：计算点的协方差矩阵，如果最小特征值太小 → 切换为 Homography 分解。

### 7.2 少点退化

$n < 4$（EPnP）或 $n < 3$（P3P）时，PnP 无法求解。对策：
- ORB-SLAM3：先用恒速/运动模型预测位姿，用投影匹配扩大匹配数
- VINS-Fusion：退化到纯 IMU 预积分传播

### 7.3 深度不确定性

远距离 3D 点（深度不确定大 → 3D 坐标误差大）对 PnP 精度影响很小（因为它们在图像中的投影对深度不敏感），可正常参与 RANSAC。真正危险的是**近处 3D 点**的坐标误差。

---

## 8. 与直接法的对比

PnP 属于间接法体系，依赖特征点的显式 3D-2D 对应。直接法（DSO/DM-VIO/DROID-SLAM）无此环节：

| 维度 | PnP (间接法) | 直接法对应 |
|------|-------------|-----------|
| 观测模型 | 几何重投影误差 | 光度误差（像素灰度差） |
| 需要 3D 点吗 | 必须（已三角化） | 不需要显式 3D 坐标 |
| 需要特征吗 | 必须（描述子匹配） | 不需要（梯度场直接对齐） |
| 尺度 | 三角化时确定 | 关键帧对齐间接获得 |

---

## 9. 工程建议

1. **EPnP 是默认选择**：精度和速度的平衡最优，OpenCV 已内置
2. **RANSAC 前先做 IMU/运动模型预测**：将特征从上一帧投影到当前帧（3D→2D 投影），大幅减少匹配搜索空间
3. **卡方检验双重验证**：RANSAC 内点 + 后端 $r^T\Sigma^{-1} r < \chi^2_{0.95}(2) = 5.991$
4. **共面检测不能省**：否则退化解直接导致跟踪丢失

---

## 10. Agent 实现提示

### 适用场景

当 Agent 需要在视觉前端或回环模块中实现 3D-2D 位姿估计时使用本提示：输入是已三角化地图点与当前帧归一化像素观测，输出是当前帧相机/机体系位姿，并将失败原因显式反馈给上层状态机。

### 输入输出契约

- **输入**：`pts3D: List[Point3]`、`pts2D: List[Point2]`、相机内参或归一化相机模型、上一帧/IMU 预测位姿初值、RANSAC/重投影阈值。
- **输出**：`success: bool`、`R_wc`、`t_wc`、内点掩码、重投影误差统计；若匹配数少于 4 或求解失败，必须返回失败而不是写入不可信位姿。
- **坐标约定**：明确区分 `w_T_cam` 与 `cam_T_w`；OpenCV `solvePnP` 返回的是相机到世界变换形式中的 `rvec/tvec` 使用方式，写回状态前要转换回系统内部约定。

### 实现骨架（伪代码）

```python
def estimate_pose_by_pnp(state_pose, observations_2d, landmarks_3d, camera):
    if len(observations_2d) < 4:
        return Failure("not enough 3D-2D correspondences")

    R_cw, t_cw = invert_pose(state_pose.R_wc, state_pose.t_wc)
    rvec, tvec = to_opencv_pose(R_cw, t_cw)

    ok, rvec, tvec, inliers = solve_pnp_ransac(
        landmarks_3d,
        observations_2d,
        camera.K_or_identity_for_normalized_points,
        rvec,
        tvec,
        use_initial_guess=True,
    )
    if not ok or len(inliers) < min_inliers:
        return Failure("pnp rejected")

    R_cw = rodrigues_to_matrix(rvec)
    R_wc, t_wc = invert_pose(R_cw, tvec)
    return Success(R_wc, t_wc, inliers)
```

### 关键源码片段

`raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L215-L244`

```cpp
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
```

`raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp:L245-L256`

```cpp
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
```

### 实现注意事项

- 若输入点是归一化坐标，可像 VINS-Fusion 一样使用单位内参；若是像素坐标，必须传真实 `K` 和畸变参数。
- `solvePnP` 本身不等价于鲁棒估计；外点多时应启用 `solvePnPRansac` 或在前端先做 GMS/基础矩阵过滤。
- 使用初值时要确认 `useExtrinsicGuess=true` 对应的初值坐标系，否则会把上一帧预测反向注入。
- PnP 成功后仍需用重投影误差和后端 pose-only BA 复核，不能只看 OpenCV 返回值。

### 源码检索锚点

- `raw/codes/VINS-Fusion/vins_estimator/src/estimator/feature_manager.cpp`：`solvePoseByPnP`、`initFramePoseByPnP`
- `raw/codes/VINS-Fusion/loop_fusion/src/keyframe.cpp`：`PnPRANSAC`、`solvePnPRansac`
- `raw/codes/ORB_SLAM3/src/Tracking.cc`：`MLPnPsolver`、`SetRansacParameters`
- `raw/codes/ORB_SLAM3/src/Optimizer.cc`：`PoseOptimization`

## 11. 相关页面

- [[方法-视觉特征跟踪]]
- [[概念-三角化与深度估计]]
- [[算法-ORB-SLAM3]]
- [[2026-04-29-vins-fusion-analysis-analysis|VINS-Fusion]]
- [[概念-回环检测方法]]
- [[组件-DBoW2]]
- [[方法-视觉特征跟踪|RANSAC 基础]]
