# CT-ICP (Continuous-Time Iterative Closest Point) 深度源码分析

> 仓库: https://github.com/jedeschaud/ct_icp | 886 stars
> 分析时间: 2026-04-29

---

## 1. 连续时间 ICP 核心思想

### 1.1 问题的提出: Scan内畸变

传统的离散时间 ICP 假设一帧 LiDAR 扫描中所有点都是在同一个时刻采集的。然而，机械旋转式 LiDAR（如 Velodyne、Ouster）内部有旋转机构，一个完整的 360° 扫描需要约 100ms（10Hz 扫描频率）。在这 100ms 内如果传感器以 10m/s 的速度运动，首尾点之间的位移可达 1 米。

**这就是 scan 内畸变（intra-scan distortion）问题**。

### 1.2 连续时间表示

CT-ICP 的核心创新是：**不在单个时刻表示一帧位姿，而是将每帧表示为一条连续的时间轨迹**。

```cpp
// ct_icp/include/ct_icp/types.h:31-61
struct TrajectoryFrame {
    slam::Pose begin_pose, end_pose;  // 帧起始和结束时的位姿

    // 帧内自运动角度
    inline double EgoAngularDistance() const {
        return slam::AngularDistance(begin_pose.pose, end_pose.pose);
    }

    // 与另一帧的姿态距离
    double TranslationDistance(const TrajectoryFrame &other) {
        return (begin_pose.TrConstRef() - other.begin_pose.TrConstRef()).norm() +
               (end_pose.TrConstRef() - other.end_pose.TrConstRef()).norm();
    }

    // 帧中点插值
    [[nodiscard]] inline Eigen::Matrix4d MidPose() const {
        return begin_pose.InterpolatePoseAlpha(end_pose, 0.5).Matrix();
    };
};
```

### 1.3 本质理解

CT-ICP 用一个 12 自由度的参数化来表示一帧的运动（离散 ICP 只用 6 自由度）：

- 平移: `begin_tr (3) + end_tr (3) = 6`
- 旋转: `begin_quat (4) + end_quat (4) = 8`，考虑四元数归一化约束，等效 6 个有效自由度

对于帧内的任意时间戳 `t ∈ [0, 1]`，位姿通过线性插值获得：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:518-524
// 弹性变形：根据每个点的时间戳，在 begin/end pose 之间插值
for (auto i(0); i < num_points; ++i) {
    double timestamp = timestamps[i];
    auto interpolated_pose = frame_to_optimize.begin_pose.InterpolatePose(
        frame_to_optimize.end_pose, timestamp);
    world_point_proxy = interpolated_pose * raw_point;
}
```

CTFunctor 的 Ceres Cost Function 定义：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:186-223
template<typename FunctorT>
struct CTFunctor {
    static constexpr int NumResiduals() { return FunctorT::NumResiduals(); }
    // 4+3+4+3 = 14 个参数块 = 12 自由度
    typedef ceres::AutoDiffCostFunction<CTFunctor<FunctorT>, FunctorT::NumResiduals(), 4, 3, 4, 3> cost_function_t;

    CTFunctor(double timestamp, const Eigen::Vector3d &reference,
              const Eigen::Vector3d &raw_point, ...);

    template<typename T>
    inline bool operator()(const T *const begin_rot_params, const T *begin_trans_params,
                           const T *const end_rot_params, const T *end_trans_params, T *residual) const {
        T alpha_m = T(1.0 - alpha_timestamp_);
        T alpha = T(alpha_timestamp_);

        // 线性插值平移
        Eigen::Matrix<T, 3, 1> tr;
        tr(0, 0) = alpha_m * begin_trans_params[0] + alpha * end_trans_params[0];
        tr(1, 0) = alpha_m * begin_trans_params[1] + alpha * end_trans_params[1];
        tr(2, 0) = alpha_m * begin_trans_params[2] + alpha * end_trans_params[2];

        // 球面线性插值旋转 (slerp)
        Eigen::Quaternion<T> quat_begin(const_cast<T *>(begin_rot_params));
        Eigen::Quaternion<T> quat_end(const_cast<T *>(end_rot_params));
        Eigen::Quaternion<T> quat_inter = quat_begin.normalized().slerp(T(alpha), quat_end.normalized());

        return functor(quat_inter.coeffs().data(), tr.data(), residual);
    }

    FunctorT functor;
    double alpha_timestamp_ = 1.0;  // t ∈ [0, 1], 归一化时间戳
};
```

### 1.4 连续时间 vs 离散时间对比

| 方面 | 离散时间 ICP | 连续时间 CT-ICP |
|------|-------------|-----------------|
| 参数化 | 6 DoF (单pose) | 12 DoF (begin+end) |
| Scan内畸变 | 忽略/预处理补偿 | 原生建模补偿 |
| 运动假设 | 所有点同时采集 | 每个点有独立时间戳 |
| 精度 | 高速运动时下降 | 高速运动时保持 |
| 计算量 | 低 | 约 2x |

运动补偿模式的枚举：

```cpp
// ct_icp/include/ct_icp/odometry.h:12-17
enum MOTION_COMPENSATION {
    NONE = 0,              // 无补偿 (离散时间)
    CONSTANT_VELOCITY = 1, // 匀速假设预处理
    ITERATIVE = 2,         // 每轮 ICP 迭代后重新补偿
    CONTINUOUS = 3         // 连续时间估计 (CT-ICP 核心)
};
```

当使用 `CONTINUOUS` 模式时，内部 CT-ICP 选项会自动配置为连续时间参数化：

```cpp
// ct_icp/src/ct_icp/odometry.cpp:718-723
case MOTION_COMPENSATION::CONTINUOUS:
    options_.ct_icp_options.point_to_plane_with_distortion = true;
    options_.ct_icp_options.parametrization = CONTINUOUS_TIME;
    options_.ct_icp_options.distance = POINT_TO_PLANE;
    break;
```

---

## 2. 轨迹参数化

### 2.1 线性插值 + 球面线性插值 (Slerp)

CT-ICP 使用**线性插值（平移）+ 球面线性插值（旋转）**作为帧内轨迹的连续表示。这是 CT-ICP 论文中对 B-spline 和 Gaussian Process 的一种简化高效近似。

核心实现在 Pose 的 `InterpolatePose` 方法中（定义在 SlamCore 库中）：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:521-525
auto interpolated_pose = frame_to_optimize.begin_pose.InterpolatePose(
    frame_to_optimize.end_pose, timestamp);
```

`GetAlphaTimestamp` 将绝对时间戳归一化到 [0, 1]：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:600-601
builder.SetResidualBlock(options.num_closest_neighbors * k + i, k,
    neighborhood.points[i], neighborhood.description, weight,
    begin_pose.GetAlphaTimestamp(timestamp, end_pose));
```

### 2.2 与 B-Spline / Gaussian Process 的比较

CT-ICP 的线性插值方案可以视为：
- **比 B-Spline 更高效**: B-Spline 需要维护多个控制点，CT-ICP 仅用 2 个（begin/end），计算量大幅降低
- **比 GP 更实用**: Gaussian Process 轨迹在理论上更优雅，但需要维护大量状态点，CT-ICP 的 2 点 + 运动模型先验在实际效果上接近

**未来可能的扩展** — 代码中留下了 B-Spline 的 TODO：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:16-20
enum POSE_PARAMETRIZATION {
    SIMPLE,         //< Simple 6DoF
    CONTINUOUS_TIME //< Continuous time 12DoF
    // B-Spline: TODO // 18DoF
};
```

### 2.3 参数化实现细节

ICPOptimizationBuilder 根据参数化类型添加参数块：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:227-237
switch (options_->parametrization) {
    case CONTINUOUS_TIME:
        problem->AddParameterBlock(begin_quat_, 4, parameterization);  // 带四元数参数化
        problem->AddParameterBlock(end_quat_, 4, parameterization);
        problem->AddParameterBlock(begin_t_, 3);
        problem->AddParameterBlock(end_t_, 3);
        break;
    case SIMPLE:
        problem->AddParameterBlock(end_quat_, 4, parameterization);
        problem->AddParameterBlock(end_t_, 3);
        break;
}
```

对于 `SIMPLE` 模式，distort 帧使得所有点都变换到 `end_pose` 的坐标系下：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:199-213
void DistortFrame(slam::Pose &begin_pose, slam::Pose &end_pose) {
    auto end_pose_I = end_pose.Inverse().pose;
    for (int i(0); i < world_points_.size(); ++i) {
        auto interpolated_pose = begin_pose.InterpolatePose(end_pose, timestamp);
        corrected_raw_points_[i] = end_pose_I * (interpolated_pose * raw_point);
    }
}
```

---

## 3. 运动模型与约束

### 3.1 运动模型抽象

CT-ICP 设计了可插拔的运动模型接口：

```cpp
// ct_icp/include/ct_icp/motion_model.h:11-32
class AMotionModel {
    // 向 Ceres 问题添加运动约束
    virtual void AddConstraintsToCeresProblem(ceres::Problem &problem,
        ct_icp::TrajectoryFrame &frame_to_optimize, int number_of_residuals) const = 0;

    // 预测下一帧
    virtual ct_icp::TrajectoryFrame NextFrame() = 0;

    // 更新模型状态
    virtual void UpdateState(const ct_icp::TrajectoryFrame &optimized_frame, int frame_index) = 0;

    // 检查轨迹有效性
    virtual bool IsValid(const ct_icp::TrajectoryFrame &frame) = 0;
};
```

### 3.2 默认运动模型: PreviousFrameMotionModel

主要约束类型：

```cpp
// ct_icp/include/ct_icp/motion_model.h:35-79
class PreviousFrameMotionModel : public AMotionModel {
    enum MODEL_TYPE {
        CONSTANT_VELOCITY,  // 初始化为匀速运动
        SMALL_VELOCITY      // 初始化为静止
    };

    struct Options {
        double beta_location_consistency = 0.001;  // 位置一致性约束
        double beta_constant_velocity = 0.001;     // 常速度约束
        double beta_small_velocity = 0.0;          // 小速度约束
        double beta_orientation_consistency = 0.0; // 朝向一致性约束
    };
};
```

约束添加到 Ceres 问题中：

```cpp
// ct_icp/src/ct_icp/motion_model.cpp:12-61
void PreviousFrameMotionModel::AddConstraintsToCeresProblem(...) {
    // 1. 位置一致性：当前帧 begin 应该接近上一帧 end
    if (options_.beta_location_consistency > 0.) {
        problem.AddResidualBlock(
            new LocationConsistencyFunctor(previous_frame_.EndTr(),
                sqrt(num_residuals * options_.beta_location_consistency)),
            nullptr, &frame_to_optimize.begin_pose.TrRef().x());
    }

    // 2. 朝向一致性
    if (options_.beta_orientation_consistency > 0.) {
        problem.AddResidualBlock(
            new OrientationConsistencyFunctor(previous_orientation, ...),
            nullptr, &frame_to_optimize.begin_pose.QuatRef().x());
    }

    // 3. 常速度约束
    if (options_.beta_constant_velocity > 0.) {
        problem.AddResidualBlock(
            new ConstantVelocityFunctor(previous_velocity, ...),
            nullptr, &begin_tr, &end_tr);
    }

    // 4. 小速度约束（惩罚大位移）
    if (options_.beta_small_velocity > 0.) {
        problem.AddResidualBlock(
            new SmallVelocityFunctor(...),
            nullptr, &begin_tr, &end_tr);
    }
}
```

**注意**: 约束权重会乘以 `sqrt(num_residuals)` 以保持不同数量残差下的约束有效性。

### 3.3 约束的意义

这些运动约束本质上是 CT-ICP 轨迹的**正则化项**：
- **位置一致性**: 防止相邻帧间出现跳变，确保轨迹光滑
- **常速度**: 提供运动先验，在几何约束不足时稳定估计
- **朝向一致性**: 防止旋转估计发散
- **小速度**: 在低动态场景下抑制异常运动估计

---

## 4. 鲁棒配准与协方差估计

### 4.1 多重距离度量

CT-ICP 支持四种几何残差：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:22-28
enum ICP_DISTANCE {
    POINT_TO_PLANE,        // 点到平面距离（默认）
    POINT_TO_POINT,        // 点到点距离
    POINT_TO_LINE,         // 点到线距离
    POINT_TO_DISTRIBUTION  // 点到分布的马氏距离
};
```

`POINT_TO_DISTRIBUTION` 使用邻域协方差的逆作为信息矩阵，隐式编码了局部几何结构的置信度：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:141-183
struct FunctorPointToDistribution {
    FunctorPointToDistribution(...) {
        neighborhood_information_ = (neighborhood.covariance +
            Eigen::Matrix3d::Identity() * epsilon).inverse();
    }

    template<typename T>
    bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
        Eigen::Matrix<T, 3, 1> diff = transformed - world_reference_.template cast<T>();
        residual[0] = T(weight_) * (diff.transpose() * neighborhood_information_ * diff)(0, 0);
        return true;
    }
};
```

### 4.2 权重方案

三种加权策略：

```cpp
// ct_icp/include/ct_icp/ct_icp.h:49-54
enum WEIGHTING_SCHEME {
    PLANARITY,      // 按平面性系数加权
    NEIGHBORHOOD,   // 按最近邻距离加权
    ALL             // 组合加权（默认）
};
```

权重计算：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:582-587
// 平面性: 高平面性区域残差更可信
double weight = std::pow(neighborhood.description.a2D, options.power_planarity);
// 组合: 平面性 + 距离近
weight = lambda_weight * weight +
         lambda_neighborhood * std::exp(-(neighbor_dist) / (kMaxPointToPlane * kMinNumNeighbors));
```

### 4.3 鲁棒损失函数

```cpp
// ct_icp/include/ct_icp/ct_icp.h:41-48
enum LEAST_SQUARES {
    STANDARD,   // 标准 L2
    CAUCHY,     // Cauchy 损失（默认）
    HUBER,      // Huber 损失
    TOLERANT,   // 宽容损失
    TRUNCATED   // 截断 L2
};
```

自定义截断损失：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:364-374
class TruncatedLoss : public ceres::LossFunction {
    explicit TruncatedLoss(double sigma) : sigma2_(sigma * sigma) {}
    // rho(s) = min(sigma^2, s^2)
};
```

### 4.4 Robust Registration 机制

CT-ICP 实现了自适应的鲁棒配准策略，在配准失败时自动提高保守程度：

```cpp
// ct_icp/src/ct_icp/odometry.cpp:780-852
void Odometry::RobustRegistration(...) {
    RobustRegistrationAttempt attempt(...);
    do {
        TryRegister(frame, frame_info, attempt.registration_options, ...);
        good_enough_registration = AssessRegistration(frame, attempt.summary, log_out);
        if (!good_enough_registration) {
            if (attempt.summary.number_of_attempts < options_.robust_num_attempts) {
                attempt.IncreaseRobustnessLevel();  // 增加采样、扩大邻域
            } else {
                good_enough_registration = true; // 放弃，接受当前结果
            }
        }
    } while (!good_enough_registration);
}
```

`IncreaseRobustnessLevel()` 通过缩小采样体素、扩大邻域搜索等策略逐步增强鲁棒性。

### 4.5 地图点插入策略

```cpp
// ct_icp/src/ct_icp/odometry.cpp:855-953
void Odometry::UpdateMap(...) {
    // 拒绝大旋转帧的地图插入（防止错误帧污染地图）
    if (summary.ego_orientation > options_.robust_threshold_ego_orientation ||
        summary.relative_orientation > options_.robust_threshold_relative_orientation) {
        add_points = false;
    }

    // 连续失败超过5次时强制插入（防止地图漂移）
    if (robust_num_consecutive_failures_ > 5)
        add_points = true;

    // 移除远距离体素
    map_->RemoveElementsFarFromLocation(location, kMaxDistance);

    // 插入新点云
    if (add_points)
        map_->InsertPointCloud(pc_to_add, {begin_pose, end_pose}, indices);
}
```

---

## 5. 协方差估计

CT-ICP 通过多种机制隐式估计配准的不确定性：

### 5.1 平面性 (Planarity) 作为信息度量

`a2D` (planarity) 通过邻域的 PCA 分解计算：`a2D = (λ2 - λ3) / λ1`
- 高 a2D → 平面结构 → 沿法线方向约束强 → 高置信度
- 低 a2D → 线/离散结构 → 约束弱 → 低置信度

### 5.2 POINT_TO_DISTRIBUTION

利用邻域的完整协方差矩阵（而非仅法线）进行马氏距离度量：

```cpp
// ct_icp/include/ct_icp/cost_functions.h:158-159
neighborhood_information_ = (neighborhood.covariance +
    Eigen::Matrix3d::Identity() * epsilon).inverse();
```

epsilon (0.05) 的加入是对奇异协方差矩阵的正则化。

### 5.3 运动模型中的运动评估

```cpp
// ct_icp/src/ct_icp/motion_model.cpp:64-82
bool PreviousFrameMotionModel::IsValid(const TrajectoryFrame &frame) {
    auto prediction = NextFrame();
    // 预测与实际的差异超过阈值 → 配准可能失败
    if (tr_diff > 0.3m || rot_diff > 15°) return false;
}
```

---

## 6. Solver 选择

```cpp
// ct_icp/include/ct_icp/ct_icp.h:35-39
enum CT_ICP_SOLVER {
    GN,     // 手写高斯牛顿（经典版本）
    CERES,  // Ceres Solver（推荐，多线程）
    ROBUST  // 鲁棒求解器（支持线性/平面性分类）
};
```

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:998-1003
#define SELECT_SOLVER
    switch (options_.solver) {
        case CERES: return DoRegisterCeres(voxel_map, ...);
        case GN:    return DoRegisterGaussNewton(voxel_map, ...);
        case ROBUST:return DoRegisterRobust(voxel_map, ...);
    }
```

Ceres 版本使用 OpenMP 并行计算邻域和添加残差，配置 8-16 线程：

```cpp
// ct_icp/src/ct_icp/ct_icp.cpp:559-603
#pragma omp parallel for num_threads(num_threads)
for (int k = 0; k < num_keypoints; ++k) {
    // 并行计算每个点的邻域和残差
}
```

---

## 7. 优缺点与对 SLAM 算法的意义

### 优势
1. **原生处理 scan 内畸变**: 不再需要预处理或近似补偿，在高速运动中精度优势明显
2. **简洁高效**: 仅用 12 自由度（2 个 pose）线性插值，较 B-spline 和 GP 大幅简化，计算开销约为离散时间 ICP 的 2x
3. **运动模型先验**: 通过运动约束正则化，在几何信息不足时（如隧道、走廊）依然稳定
4. **自适应鲁棒性**: Robust Registration 机制自动调整参数，提高各种场景的适用性
5. **多线程并行**: Ceres 版本充分利用多核，实现实时性能
6. **可扩展架构**: 抽象的运动模型、邻域策略、地图类型接口便于扩展

### 劣势
1. **仅里程计**: 纯 LiDAR odometry，无回环检测和全局优化，长距离累积漂移无法消除
2. **计算开销**: 比传统 ICP 多约 2x 计算量，在低算力平台有挑战
3. **依赖时间戳**: 需要每个点精确的时间戳，部分传感器可能不提供
4. **无 IMU 融合**: 虽然预留了接口（IMSLIDAR），但没有原生深度融合
5. **参数敏感**: 运动模型权重等参数需要根据场景调整

### 对 SLAM 算法的意义
- **连续时间范式的成功实践**: CT-ICP 证明了连续时间轨迹参数化在 LiDAR SLAM 中的有效性和实用性，推动了该方向的研究
- **扫描畸变的标准答案**: 将 scan 内畸变作为核心估计问题而非预处理问题，改变了行业对 LiDAR odometry 的认知
- **模块化设计启示**: 运动模型、邻域策略、损失函数等模块化设计为后续工作提供了良好的扩展基础
- **与 IMU 融合前景**: CT-ICP 的连续时间框架天然适合与高频 IMU 数据融合，可能成为 IMU-centric SLAM 的重要组件