---
tags: [ROLO, Rotation-LO, LiDAR SLAM, 旋转解耦, RotVGICP, 地面车辆, UGV, GICP, LOAM]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/ROLO/src/lidarOdometry.cpp
  - raw/codes/ROLO/src/imageProjection.cpp
  - raw/codes/ROLO/src/featureExtraction.cpp
  - raw/codes/ROLO/include/rot_gicp/gicp/rot_vgicp.hpp
  - raw/codes/ROLO/include/rot_gicp/gicp/impl/rot_vgicp_impl.hpp
  - raw/codes/ROLO/include/rolo/utility.h
---

# ROLO: Rotation-LO —— 旋转-平移解耦的 LiDAR SLAM

> ROLO (Rotation-LO) 在 LOAM/LeGO-LOAM 的框架基础上，将前端里程计的 6-DOF 非线性优化**解耦为旋转配准和平移配准两步**。旋转部分使用专门设计的 RotVGICP（Rotation-only Voxelized GICP），平移部分使用连续时间约束的平移优化。这一设计专为地面车辆的运动特性优化——旋转和平移的耦合较弱且有不同的观测特性。

## 系统架构

```
点云输入
  │
  ├─ [imageProjection] ─→ 距离图像投影 + 去畸变 (使用IMU里程计)
  │     └─ 速度较快的前端给出去畸变用位姿
  │
  ├─ [featureExtraction] ─→ LOAM风格特征提取
  │     ├─ 平滑度计算 (同LOAM)
  │     ├─ 角点 / 平面点 提取
  │     └─ 遮挡点 / 平行点 剔除
  │
  ├─ [lidarOdometry] ─→ 旋转-平移解耦配准 ⭐核心
  │     ├─ 前向状态插值 (基于后端优化结果)
  │     ├─ RotVGICP: 旋转-only 体素化 GICP
  │     └─ computeTranslation: 连续时间约束平移优化
  │
  ├─ [backMapping] ─→ 后端优化 (ISAM2)
  │     └─ 关键帧 + scan-to-submap + 回环检测
  │
  └─ [transformFusion] ─→ 前端/后端位姿融合 + TF发布
```

## 旋转-平移解耦设计

ROLO 的核心创新：**将 6-DOF 点云配准分解为旋转估计和平移估计两个独立步骤**。

**源码锚点**: `raw/codes/ROLO/src/lidarOdometry.cpp:L331-L383`

```cpp
void scanRegeistration() {
    // 1. 前向插值：用后端优化结果预测当前帧位置
    pcl::transformPointCloud(*featureOld, *feature_propagated,
                              transformation_interpolated);

    // 2. 旋转估计: RotVGICP
    fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
    rot_vgicp.setResolution(1.0);
    rot_vgicp.setInputTarget(featureLast);        // 当前帧
    rot_vgicp.setInputSource(feature_propagated); // 上一帧(平移预对齐)
    rot_vgicp.align(*aligned);
    Eigen::Matrix4f trans = rot_vgicp.getFinalTransformation(); // 旋转估计
    transformation_interpolated = transformation_interpolated * transformStep;
    Rotation = transformation_interpolated.rotation().cast<double>();

    // 3. 平移估计: 连续时间约束
    pcl::transformPointCloud(*featureOld, *feature_rotated,
                              transformation_interpolated);  // 先旋转
    Eigen::Vector3d Reg_translation = Eigen::Vector3d::Zero();
    rot_vgicp.computeTranslation(*aligned, Reg_translation,
        Translation, TranslationOld, 0.1, 0.1, CT_lambda);
    Translation += Reg_translation;
}
```

**解耦的动机**：
1. 地面车辆旋转主要来自转向，变化慢、可预测性强
2. 平移主要沿前进方向，在平面约束下自由度低
3. 6-DOF 联合优化中旋转和平移的雅可比块耦合严重，解耦后各自收敛更快
4. 旋转估计可以用更丰富的点云信息（角点+平面点），平移可以用地面约束 + 连续时间平滑

## RotVGICP: Rotation-Only GICP

ROLO 自研的旋转-only 配准算法，基于 GICP 框架但仅优化旋转（SO(3)）。

**源码锚点**: `raw/codes/ROLO/include/rot_gicp/gicp/rot_vgicp.hpp:L24-L131`

```cpp
template<typename PointSource, typename PointTarget>
class RotVGICP : public LsqRegistration<PointSource, PointTarget> {
    // 旋转-only cost function
    virtual double so3_linearize(const Eigen::Isometry3d& trans,
        Eigen::Matrix<double, 3, 3>* H,
        Eigen::Matrix<double, 3, 1>* b) override;

    // 平移-only cost function
    virtual double t3_linearize(const Eigen::Vector3d& trans,
        const Eigen::Vector3d& init_guess,
        const Eigen::Vector3d& last_t0,
        const double interval_tn, const double interval_tn_1,
        Eigen::Matrix<double, 6, 6>* H,
        Eigen::Matrix<double, 6, 1>* b) override;

    virtual void computeTranslation(PointCloudSource& output,
        Eigen::Vector3d& trans,
        const Eigen::Vector3d& init_guess,
        const Eigen::Vector3d& last_t0,
        const double interval_tn, const double interval_tn_1,
        const float ct_lambda);
};
```

### SO(3) 线性化：旋转-only 雅可比

**源码锚点**: `raw/codes/ROLO/include/rot_gicp/gicp/impl/rot_vgicp_impl.hpp:L282-L370`

旋转-only 的残差线性化在 SO(3) 的切空间上进行：

```cpp
double so3_linearize(const Eigen::Isometry3d& trans,
    Eigen::Matrix<double, 3, 3>* H,
    Eigen::Matrix<double, 3, 1>* b) {

    update_correspondences(trans);

    for i in voxel_correspondences_:
        mean_A = source[i]  (齐次坐标)
        mean_B = target_voxel.mean_dir
        error = mean_B - trans * mean_A

        // 旋转-only 雅可比: ∂(Rp)/∂φ = [Rp]×
        dtdx0 = skewd(transed_mean_A.head<3>())  // 3×3 skew matrix

        w = sqrt(target_voxel->num_points)
        Hi = w * dtdx0^T * M_i * dtdx0     // 3×3 Hessian
        bi = w * dtdx0^T * M_i * error     // 3×1 bias
```

**关键区别**：标准 GICP 的雅可比是 $6 \times 6$（旋转+平移），RotVGICP 的 `so3_linearize` 仅输出 $3 \times 3$ 的 Hessian 和 $3 \times 1$ 的 bias。

李代数扰动模型：
$$
\frac{\partial (\mathbf{R} \mathbf{p})}{\partial \boldsymbol{\phi}} = [\mathbf{R}\mathbf{p}]_{\times}
$$

其中 $\boldsymbol{\phi} \in \mathfrak{so}(3)$ 是旋转的切空间坐标，$[\cdot]_{\times}$ 是反对称矩阵。

### 体素协方差加权

与 GICP 一致，RotVGICP 使用 Mahalanobis 距离加权：

$$
\mathbf{M}_i = (\mathbf{C}_B + \mathbf{T} \mathbf{C}_A \mathbf{T}^T)^{-1}
$$

**源码锚点**: `raw/codes/ROLO/include/rot_gicp/gicp/impl/rot_vgicp_impl.hpp:L205-L211`

```cpp
// 计算马氏矩阵
Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
RCR(3, 3) = 1.0;
voxel_mahalanobis_[i] = RCR.inverse();
voxel_mahalanobis_[i](3, 3) = 0.0;
```

### 权重的点密度加权

每个对应点对的权重视体素内点数的平方根：

```cpp
double w = std::sqrt(target_voxel->num_points);
```

体素内点越多（如平面区域），该约束的权重越大。

## 连续时间约束平移优化

平移估计在旋转之后进行，融入了连续时间轨迹的平滑约束。

**源码锚点**: `raw/codes/ROLO/include/rot_gicp/gicp/impl/rot_vgicp_impl.hpp:L152-L159`

```cpp
void computeTranslation(PointCloudSource& output,
    Eigen::Vector3d& trans,
    const Eigen::Vector3d& init_guess,
    const Eigen::Vector3d& last_t0,
    const double interval_tn, const double interval_tn_1,
    const float ct_lambda) {

    lambda_ = ct_lambda;  // 连续时间约束权重
    LsqRegistration::computeTranslation(output, trans,
        init_guess, last_t0, interval_tn, interval_tn_1, ct_lambda);
}
```

连续时间约束惩罚平移估计与运动平滑先验的偏差：

$$
\min_{\mathbf{t}} \sum_i \|\mathbf{r}_i(\mathbf{t})\|_{\mathbf{M}_i}^2 + \lambda \left\| \frac{\mathbf{t} - \mathbf{t}_{\text{guess}}}{\Delta t_n} - \frac{\mathbf{t}_{\text{guess}} - \mathbf{t}_{n-1}}{\Delta t_{n-1}} \right\|^2
$$

其中 $\mathbf{t}_{\text{guess}}$ 是上一帧平移的后验估计，$\mathbf{t}_{n-1}$ 是上上帧平移。约束项鼓励平移加速度平滑。

参数 `CT_lambda`（默认 1.0）控制平滑约束的强度：
- $\lambda \to 0$：纯点云配准，无平滑约束
- $\lambda \to \infty$：强行约束为匀速运动

## 前向状态插值 (State Linear Propagation)

**源码锚点**: `raw/codes/ROLO/src/lidarOdometry.cpp:L425-L438`

在帧间配准之前，用后端优化结果进行状态前向插值，提供旋转和平移的初值：

```cpp
if(lastOdomTime != -1.0){
    double latestInterval = cloudTimeCur - cloudTimeLast;
    // 基于已知的帧间变换和帧间时间间隔，线性插值当前状态
    stateLinearPropagation(lidarMappingAffine, lastMappingInterval,
                           latestInterval, transformation_interpolated);
    Rotation = transformation_interpolated.rotation().cast<double>();
    Translation = transformation_interpolated.translation().cast<double>();
}
```

这与 KISS-ICP 的匀速运动模型思路一致，但 ROLO 使用了后端优化结果（而非前端自己估计的增量），因此插值更精确。

## 地面车辆运动约束的工程体现

ROLO 针对地面车辆（UGV）的多种工程约束：

| 约束 | 实现位置 | 说明 |
|------|---------|------|
| 平面约束 | `z_tollerance` | 限制 z 轴变化阈值，防止高度漂移 |
| 旋转约束 | `rotation_tollerance` | 限制 pitch/roll 阈值，地面车辆不会翻车 |
| 连续时间平滑 | `CT_lambda` | 约束平移的加速度连续性 |
| 旋转-平移解耦 | `scanRegeistration()` | 旋转用 SO(3) GICP，平移用独立优化 |
| 失效检测 | `failureDetection()` | 检测位姿估计跳变并丢弃 |

**失效检测代码**（`lidarOdometry.cpp:L511-L525`）：

```cpp
bool failureDetection(Affine3f pose_affine, Affine3f pose_affine_transformed,
                      double delt_Time){
    // 计算位姿增量
    delt_t = (t_x-x)^2 + (t_y-y)^2 + (t_z-z)^2
    delt_r = (t_r-r)^2 + (t_p-p)^2 + (t_y-y)^2
    // 加速度超限检测
    if(delt_t/delt_Time^2 >= 5.0 || delt_r/delt_Time^2 >= (0.2)^2){
        return false;  // 当前帧失效
    }
    return true;
}
```

## Agent 实现提示

### 适用场景

地面无人车（UGV）在结构化半结构化环境中的 LiDAR SLAM。旋转-平移解耦特别适合运动模式可预测的地面车辆（转向慢、基本沿平面运动）。多线机械旋转 LiDAR（VLP-16/32、OS1 等）均可使用。

### 输入输出契约

- **输入**：LiDAR 点云（`PointXYZI` 含 ring）、后端优化位姿（用于插值）、IMU 原始数据（用于去畸变）、连续时间权重 `CT_lambda`
- **输出**：6-DOF 增量里程计位姿、旋转和平移的分离估计、配准后的点云
- **残差结构**：旋转-only 使用 SO(3) 切空间残差，平移-only 使用点对体素马氏距离 + 连续时间加速度残差

### 实现骨架（伪代码）

```pseudo
function ROLO LidarOdometry.run():
    for each feature_cloud_info:
        // 1. 解析特征点云
        corner_cloud = msg.extracted_corner
        surf_cloud = msg.extracted_surface
        feature_cloud = corner + surf

        // 2. 前向状态插值
        state_propagate = linear_interpolate(
            mapping_transform, last_interval, current_interval)

        // 3. 旋转配准 (RotVGICP SO3)
        source_propagated = transform(feature_old, state_propagate)
        rot_vgicp.setInputTarget(feature_current)
        rot_vgicp.setInputSource(source_propagated)
        rot_vgicp.align()  // SO3 optimization
        T_rot = rot_vgicp.getFinalTransformation()
        transform_interpolated = transform_interpolated * T_rot

        // 4. 平移配准 (连续时间约束)
        source_rotated = rotate(feature_old, transform_interpolated)
        delta_t = computeTranslation(
            source_rotated, init_t, last_t, dt_n, dt_n-1, CT_lambda)
        transform_interpolated.translation += delta_t

        // 5. 后处理
        if not failure_detection(transform_interpolated):
            publish_odometry(transform_interpolated)
            swap_old_new()
```

### 关键源码片段

**旋转配准调用**（`lidarOdometry.cpp:L342-L357`）：
```cpp
pcl::transformPointCloud(*featureOld, *feature_propagated,
                          transformation_interpolated);
fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
rot_vgicp.setInputTarget(featureLast);
rot_vgicp.setInputSource(feature_propagated);
rot_vgicp.align(*aligned);
transformStep.matrix() = rot_vgicp.getFinalTransformation().cast<float>();
transformation_interpolated = transformation_interpolated * transformStep;
```

**平移配准调用**（`lidarOdometry.cpp:L375-L382`）：
```cpp
pcl::transformPointCloud(*featureOld, *feature_rotated, transformation_interpolated);
rot_vgicp.computeTranslation(*aligned, Reg_translation,
    Translation, TranslationOld, 0.1, 0.1, CT_lambda);
Translation += Reg_translation;
```

**SO(3) 雅可比**（`rot_vgicp_impl.hpp:L337-L339`）：
```cpp
Eigen::Matrix<double, 3, 3> dtdx0 = Eigen::Matrix<double, 3, 3>::Zero();
dtdx0 = skewd(transed_mean_A.head<3>());  // [Rp]×
```

**马氏矩阵构建**（`rot_vgicp_impl.hpp:L205-L209`）：
```cpp
Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
RCR(3, 3) = 1.0;
voxel_mahalanobis_[i] = RCR.inverse();
voxel_mahalanobis_[i](3, 3) = 0.0;
```

### 实现注意事项

1. **旋转配准的初值很重要**：先做平移插值再旋转配准（`transformation_interpolated` 已包含平移），因为旋转中心在原点，平移初值错误会导致旋转估计偏差
2. **CT_lambda 调节**：如果环境特征丰富（城市），lambda 可调小（0.1-0.5）；如果环境退化（隧道），lambda 应调大（2-5）
3. **RotVGICP 内部用体素地图**：`target_` 会被自动体素化（`voxel_resolution_=1.0`），对稠密点云友好
4. **特征点选择**：LOAM 风格的特征提取保留在 `featureExtraction.cpp` 中，角点 + 平面点都用于旋转配准，平移配准更侧重平面点
5. **后端 Isam2**：`backMapping.cpp` 使用 GTSAM ISAM2 做因子图优化，前端提供增量约束作为 odometry factor

### 源码检索锚点

| 模块 | 文件 | 行号 |
|------|------|------|
| 旋转配准调用 | `src/lidarOdometry.cpp` | L331-L370 |
| 平移配准调用 | `src/lidarOdometry.cpp` | L372-L383 |
| 前向插值 | `src/lidarOdometry.cpp` | L425-L438 |
| 状态更新 | `src/lidarOdometry.cpp` | L454-L508 |
| 失效检测 | `src/lidarOdometry.cpp` | L511-L525 |
| RotVGICP 类定义 | `include/rot_gicp/gicp/rot_vgicp.hpp` | L24-L131 |
| SO3 线性化 | `include/.../impl/rot_vgicp_impl.hpp` | L282-L370 |
| 马氏矩阵计算 | `include/.../impl/rot_vgicp_impl.hpp` | L194-L211 |
| 对应关系更新 | `include/.../impl/rot_vgicp_impl.hpp` | L162-L212 |
| 6-DOF 线性化 | `include/.../impl/rot_vgicp_impl.hpp` | L214-L280 |
| translate 优化 | `include/.../impl/rot_vgicp_impl.hpp` | L152-L159 |
| 特征提取 | `src/featureExtraction.cpp` | L87-L150 |
| 参数配置 | `include/rolo/utility.h` | L69-L221 |

## 相关页面

- [[方法-RotVGICP]]
- [[方法-分离式旋转-平移估计]]
- [[方法-地面车辆运动约束]]
- [[方法-LeGO-LOAM地面优化]]
- [[方法-GICP配准方法]]
- [[方法-ISAM2增量固定滞后平滑]]
- [[方法-连续时间线性插值]]
