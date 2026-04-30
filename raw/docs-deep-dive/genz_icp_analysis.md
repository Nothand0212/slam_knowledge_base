# GenZ-ICP 源码深度分析报告

## 1. 数据接收与预处理

### 1.1 传感器数据入口（ROS 回调链路）

GenZ-ICP 的数据输入极其简洁——仅有**单一激光雷达点云 topic**，无 IMU 订阅。

**ROS1 版本**：
```cpp
// ros/ros1/OdometryServer.cpp:80-81
pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    "pointcloud_topic", queue_size_, &OdometryServer::RegisterFrame, this);
```

**ROS2 版本**：
```cpp
// ros/ros2/OdometryServer.cpp:142-144
pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_topic", rclcpp::SensorDataQoS(), 
    std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
```

回调链路：
```
RegisterFrame (OdometryServer.cpp:117-146)
  → PointCloud2ToEigen(msg)        // ROS PointCloud2 → vector<Eigen::Vector3d>
  → GetTimestamps(msg)             // 提取逐点时间戳（仅 deskew 模式下）
  → odometry_.RegisterFrame(points, timestamps)  // 核心算法入口
  → PublishOdometry(pose, stamp)   // 发布 odom + path + tf
  → PublishClouds(...)             // 发布调试用点云（可选）
```

### 1.2 PointCloud2 → Eigen 转换

`Utils.hpp:199-209` 实现了一个轻量级的 ROS 消息到 Eigen 向量转换：
```cpp
std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstPtr msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}
```
**重要**：不使用 PCL，直接通过 ROS 的 `PointCloud2ConstIterator` 访问 x/y/z 字段，避免了 PCL 依赖。

### 1.3 逐点时间戳提取

`Utils.hpp:90-143` 实现了 `GetTimestamps` 函数，支持 `t`/`timestamp`/`time` 三种字段名，支持 `UINT32`/`FLOAT32`/`FLOAT64` 三种数据类型。

关键步骤（`Utils.hpp:104-115`）：
```cpp
auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;
    // 归一化到 [0.0, 1.0] 区间
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
        [&](const auto &t) { return (t - min_timestamp) / (max_timestamp - min_timestamp); });
}
```

### 1.4 预处理管线

在 `GenZICP::RegisterFrame` 中（`pipeline/GenZICP.cpp:57-68`）：

**步骤一：距离裁剪**
```cpp
const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);
// Preprocessing.cpp:110-119
// 使用 std::copy_if 滤除 max_range 外和 min_range 内的点
```

**步骤二：自适应体素降采样**
```cpp
static double voxel_size = config_.voxel_size;  // 初始 0.25m
const auto source_tmp = VoxelDownsample(cropped_frame, voxel_size);
// 自适应体素大小 = clamp(voxel_size * |source_tmp| / desired_num_voxelized_points, 0.02, 2.0)
double adaptive_voxel_size = Clamp(
    voxel_size * src_tmp.size() / config_.desired_num_voxelized_points, // 默认目标 2000 点
    0.02, 2.0);
```

**自适应机制**（LOCUS 2.0 风格）：如果降采样后点数太多 → 增大体素 → 下一次迭代保留更少点。

**步骤三：双重体素化**
```cpp
// GenZICP.cpp:91-95 (Voxelize)
frame_downsample = VoxelDownsample(frame, max(adaptive_voxel_size * 0.5, 0.02))  // 地图更新用，较小体素
source = VoxelDownsample(frame_downsample, adaptive_voxel_size * 1.0)              // 配准用，标准体素
```
地图保留稍多的点，配准则用稍大的体素，平衡精度与效率。

### 1.5 体素降采样实现

`Preprocessing.cpp:47-66` 使用 `tsl::robin_map`（高性能哈希表）：
```cpp
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame, double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
        const auto voxel = Voxel((point / voxel_size).cast<int>());
        if (grid.contains(voxel)) continue;  // 每个体素只保留第一个点
        grid.insert({voxel, point});
    }
    // 将保留的点收集到输出向量
}
```
哈希函数：`vec[0]*73856093 ^ vec[1]*19349663 ^ vec[2]*83492791`（与 iVox 一致）。

### 1.6 时间同步

GenZ-ICP **没有多传感器时间同步**，仅使用激光雷达自身的时间戳。点云的 `msg->header.stamp` 直接作为当前帧的输出时间戳。

---

## 2. 特征提取与匹配

### 2.1 特征类型：基于协方差的自适应分类

GenZ-ICP 的核心创新在于 **根据局部几何结构自动将点分为 planar / non-planar 两类**，而不是预设固定的特征提取规则。

### 2.2 法向量估计与平面度判断

`VoxelHashMap::GetClosestNeighbor`（`VoxelHashMap.cpp:63-102`）：

```cpp
// 搜素当前体素及其 26 个邻居体素中的所有地图点
for (const auto &voxel_shift : voxel_shifts) {  // 27 个偏移量
    Voxel voxel(kx+shift.x, ky+shift.y, kz+shift.z);
    auto search = map_.find(voxel);
    if (search != map_.end()) {
        for (const auto &neighbor : search->second.points) {
            // 找最近点
            if (dist² < closest_squared_distance) closest_neighbor = neighbor;
            // 累加均值和协方差
            centroid += neighbor;
            covariance += neighbor * neighbor.transpose();
            n_neighbors++;
        }
    }
}
// 协方差归一化（减去均值）
covariance = covariance/n - centroid*centroid.transpose();
```

`VoxelHashMap::DeterminePlanarity`（`VoxelHashMap.cpp:105-123`）：

```cpp
// 特征分解
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
normal = solver.eigenvectors().col(0);  // 最小特征值对应的特征向量 = 法向量

// 平面度判定：λ3 / (λ1+λ2+λ3) < planarity_threshold
double lambda3 = eigenvalues[0];  // 最小
double lambda2 = eigenvalues[1];
double lambda1 = eigenvalues[2];  // 最大
bool is_planar = (lambda3 / (lambda1 + lambda2 + lambda3)) < planarity_threshold_;
```

**物理意义**：如果最小特征值 λ3 相对于总方差很小，说明点大致分布在一个平面上（法向量方向方差极小）。

### 2.3 对应关系查找

`VoxelHashMap::GetCorrespondences`（`VoxelHashMap.cpp:125-206`）使用 TBB 并行：

```cpp
auto compute = [&](const tbb::blocked_range<size_t> &r, ResultTuple result) -> ResultTuple {
    for (size_t i = r.begin(); i != r.end(); ++i) {
        const auto &point = points[i];
        const auto &[closest_neighbor, n_neighbors, covariance, closest_distance] = GetClosestNeighbor(point);
        if (closest_distance > max_correspondance_distance) continue;
        
        if (n_neighbors >= min_neighbors_for_normal_estimation) {  // ≥5 个邻居
            const auto &[is_planar, normal] = DeterminePlanarity(covariance);
            if (is_planar) {
                result.source.push_back(point);     // 源点
                result.target.push_back(closest_neighbor);  // 最近邻点
                result.normals.push_back(normal);   // 法向量
                result.planar_count++;
            } else {
                result.non_planar_source.push_back(point);
                result.non_planar_target.push_back(closest_neighbor);
                result.non_planar_count++;
            }
        } else {
            result.non_planar_source.push_back(point);
            result.non_planar_target.push_back(closest_neighbor);
            result.non_planar_count++;
        }
    }
    return result;
};
return tbb::parallel_reduce(tbb::blocked_range<size_t>(0, points.size()), ...);
```

**匹配策略**：
- 对每个源点，在 voxel hash map 中找到最近体素邻居（27 邻域搜索）
- 区分 planar（有法向量）和 non-planar（仅最近点）两类对应关系
- planar 对应提供 (source, target, normal) 三元组
- non-planar 对应仅提供 (source, target) 二元组

### 2.4 特征类型总结

| 特征类型 | 判定方法 | 提供信息 | 最小邻居数 |
|---------|---------|---------|-----------|
| Planar (平面) | λ3/(λ1+λ2+λ3) < threshold_planarity | (source, target, normal) | ≥5 |
| Non-planar (非平面) | 不满足平面条件或邻居不足 | (source, target) | ≥1 |

---

## 3. 位姿计算

### 3.1 GenZ-ICP 核心：自适应权重 ICP

GenZ-ICP 的位姿计算是一个 **混合 point-to-plane + point-to-point 的加权 ICP**，由 `Registration::RegisterFrame` 实现（`Registration.cpp:181-219`）。

### 3.2 ICP 变量类型

| 组件 | 公式 | 说明 |
|------|------|------|
| Planar 残差 | $r_p = (\mathbf{s}_i - \mathbf{t}_i) \cdot \mathbf{n}_i$ | 标量，点到平面距离 |
| Non-planar 残差 | $\mathbf{r}_{np} = \mathbf{s}_i - \mathbf{t}_i$ | 3D 向量 |
| Planar 雅可比 | $\mathbf{J}_p = [\mathbf{n}_i^T \quad (\mathbf{s}_i \times \mathbf{n}_i)^T]$ | 1×6 |
| Non-planar 雅可比 | $\mathbf{J}_{np} = [\mathbf{I}_{3×3} \quad -[\mathbf{s}_i]_\times]$ | 3×6 |

### 3.3 自适应权重α

```cpp
// Registration.cpp:202
double alpha = static_cast<double>(planar_count) / static_cast<double>(planar_count + non_planar_count);
```

**α 的物理意义**：
- α → 1.0：场景中大多数点是平面 → 倾向于 point-to-plane（如室内建筑）
- α → 0.0：场景中大多数点是非平面 → 倾向于 point-to-point（如植被、街景）
- α ≈ 0.5：两类点大致均衡

这正是 "Adaptive Weighting" 的核心——不预设场景类型，根据当前帧的几何特性自动调整。

### 3.4 线性系统构建

`BuildLinearSystem`（`Registration.cpp:67-144`）：

```cpp
// 加权残差函数
auto Weight = [&](double residual_squared) {
    return kernel_squared / square(kernel + residual_squared);
};
// 相当于 Geman-McClure 鲁棒核（或其变体）

// 对于 planar 点（点面距离）：
JTJ += alpha * J_p^T * w_p * J_p
JTr += alpha * J_p^T * w_p * r_p

// 对于 non-planar 点（点对点距离）：
JTJ += (1 - alpha) * J_np^T * w_np * J_np
JTr += (1 - alpha) * J_np^T * w_np * r_np
```

使用 `tbb::parallel_reduce` 并行化，将 large vector 的累加工作分配到多核。

### 3.5 ICP 迭代

```cpp
for (int j = 0; j < max_num_iterations_; ++j) {  // 默认 150 次
    // 1. 获取对应关系（找最近邻 + 分类 planar/non-planar）
    const auto &[src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, planar_count, non_planar_count] = 
        voxel_map.GetCorrespondences(source, max_correspondence_distance);
    
    // 2. 计算 α
    double alpha = planar_count / (planar_count + non_planar_count);
    
    // 3. 构建并求解最小二乘
    const auto &[JTJ, JTr] = BuildLinearSystem(src_planar, tgt_planar, normals, 
                                                src_non_planar, tgt_non_planar, kernel, alpha);
    const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);  // LDLT 分解求解
    
    // 4. 更新
    const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
    TransformPoints(estimation, source);
    T_icp = estimation * T_icp;
    
    // 5. 收敛判断
    if (dx.norm() < convergence_criterion_) break;
}
return T_icp * initial_guess;
```

**关键参数**：
- `max_correspondence_distance = 3.0 * sigma`（sigma 来自自适应阈值）
- `kernel = sigma / 3.0`
- `convergence_criterion = 0.0001`

### 3.6 自适应阈值（AdaptiveThreshold）

`Threshold.cpp:39-50` 实现：

```cpp
double AdaptiveThreshold::ComputeThreshold() {
    double model_error = ComputeModelError(model_deviation_, max_range_);
    // model_error = |Δtranslation| + 2 * max_range * sin(|Δrotation|/2)
    
    if (model_error > min_motion_th_) {  // 0.1m 最小运动阈值
        model_error_sse2_ += model_error * model_error;
        num_samples_++;
    }
    
    if (num_samples_ < 1) return initial_threshold_;  // 初始 2.0m
    return std::sqrt(model_error_sse2_ / num_samples_);  // RMSE
}
```

σ 从初始 2.0m 逐步下降到运动模型偏差的 RMSE。运动越大 → σ 越大 → max_correspondence_distance 越大 → 允许更远的匹配。

### 3.7 运动预测模型

`GenZICP::GetPredictionModel`（`GenZICP.cpp:104-109`）：
```cpp
Sophus::SE3d prediction = poses_[N-2].inverse() * poses_[N-1];  // 恒速模型
initial_guess = last_pose * prediction;
```

基于前两帧之间的相对位姿做恒速外推，作为 ICP 初始猜测。

### 3.8 去畸变

`Deskew.cpp:36-48` 实现简单的线性插值去畸变：
```cpp
const auto delta_pose = (start_pose.inverse() * finish_pose).log();  // 6D twist
for each point i:
    motion = exp((timestamp[i] - 0.5) * delta_pose)  // 以帧中点 (0.5) 为参考
    corrected_point = motion * original_point
```

与 fusions_slam 不同，GenZ-ICP 的去畸变使用**上两帧的位姿差**估计运动速度（不需要 IMU），所有点补偿到帧中点（时间=0.5）。

### 3.9 协方差估计

GenZ-ICP **不输出协方差**。`nav_msgs::Odometry` 消息中的 `pose.covariance` 字段未填充。ICP 线性系统中 `JTJ` 的逆在理论上可提供位姿不确定度，但代码未计算。

---

## 4. 初始化

### 4.1 首帧处理

GenZ-ICP 的初始化极其简单：

```cpp
// GenZICP.cpp:76
const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
```
首帧时 `poses_` 为空，`last_pose = identity`。

```cpp
// Registration.cpp:193
if (voxel_map.Empty()) return std::make_tuple(initial_guess, ...);
```
地图为空时直接返回 `initial_guess`，首帧位姿 = 单位阵（以第一帧激光雷达坐标系为世界坐标系）。

### 4.2 运动检测

`GenZICP::HasMoved`（`GenZICP.cpp:111-115`）：
```cpp
bool GenZICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * min_motion_th;  // 5 × 0.1m = 0.5m
}
```

运动超过 0.5m 后才启用自适应阈值。

### 4.3 无 IMU 初始化

GenZ-ICP 是纯激光雷达里程计，不包含 IMU 初始化逻辑。

---

## 5. ROS 松耦合设计

### 5.1 目录树分析

```
genz-icp/
├── cpp/                          ← 纯 C++ 算法核心（无 ROS 依赖）
│   └── genz_icp/
│       ├── core/                 ← 核心算法组件
│       │   ├── Deskew.cpp/hpp    ← 点云去畸变
│       │   ├── Preprocessing.cpp/hpp ← 视域裁剪 + 体素降采样
│       │   ├── Registration.cpp/hpp  ← 自适应加权 ICP 求解器
│       │   ├── Threshold.cpp/hpp     ← 自适应匹配阈值估计
│       │   └── VoxelHashMap.cpp/hpp  ← 体素哈希地图 + 法向量估计 + 对应查找
│       ├── metrics/              ← 轨迹评估工具（ATE, KITTI Seq Error）
│       │   └── Metrics.cpp/hpp
│       ├── pipeline/             ← 管线编排
│       │   └── GenZICP.cpp/hpp   ← GenZICP 类：组合所有 core 组件
│       ├── 3rdparty/             ← 第三方库查找脚本
│       └── cmake/                ← 编译选项
├── ros/                          ← ROS 封装层
│   ├── ros1/                     ← ROS1 封装（OdometryServer + Utils）
│   ├── ros2/                     ← ROS2 封装（OdometryServer + Utils）
│   ├── config/                   ← 参数配置文件 + 调参指南
│   ├── launch/                   ← 启动文件
│   ├── rviz/                     ← RViz 配置
│   ├── CMakeLists.txt            ← ROS 包编译（条件编译 ROS1/ROS2）
│   └── package.xml               ← ROS 包描述（条件依赖 ROS1/ROS2）
├── pictures/                     ← README 图片
└── README.md
```

### 5.2 分离机制

GenZ-ICP 的松耦合设计是 **教科书级别的**：

**cpp/genz_icp/ 层**：
- 依赖：`Eigen3`（线性代数）、`Sophus`（李群李代数）、`TBB`（并行化）、`tsl::robin_map`（哈希表）
- 编译为 static library：`genz_icp::core` 和 `genz_icp::pipeline`
- **完全不引用任何 ROS 头文件**
- `GenZICP` 类暴露纯 C++ 接口（`RegisterFrame`、`LocalMap`、`poses()`）
- 可以脱离 ROS 编译和运行（仅需安装 Eigen + Sophus + TBB）

**ros/ 层**：
- 依赖：`sensor_msgs::PointCloud2`、`nav_msgs::Odometry`、`tf2_ros` 等
- 是极薄的一层（< 300 行 per ROS 版本）
- 唯一职责：ROS 消息 ↔ Eigen 向量转换 + 发布 pose/map
- ROS1/ROS2 两个 wrapper 共享同一份 cpp/ 核心代码

**CMakeLists.txt 条件编译**（`ros/CMakeLists.txt:45-105`）：
```cmake
if("$ENV{ROS_VERSION}" STREQUAL "1")
    # 编译 ROS1 odometry_node
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    # 编译 ROS2 odometry_component
else()
    message(FATAL_ERROR "...")
endif()
```
同一 `package.xml` 也使用 `condition="$ROS_VERSION == 1"` 条件声明依赖，支持 ROS1/ROS2 双版本。

### 5.3 值得复用的设计模式

1. **Library + Thin Wrapper 架构**：
   - `genz_icp::core` static library → 纯算法
   - `genz_icp::pipeline` static library → 管线编排
   - ROS node = 单文件 wrapper
   - 这是 phad_fusion 最应该借鉴的结构

2. **header-only Utils.hpp**：
   - ROS 消息转换函数全部 inline 在 header 中
   - `PointCloud2ToEigen`、`EigenToPointCloud2` 等工具函数零运行时开销

3. **Config struct 集中管理**：
   - `GenZConfig`（`pipeline/GenZICP.hpp:36-58`）列出所有可配置参数
   - 类型安全、有默认值、易于序列化

4. **无 PCL 依赖**：
   - 核心算法仅使用 `vector<Eigen::Vector3d>` 表示点云
   - 不使用 PCL 的任何数据结构（预处理除外，但也仅用 `copy_if`）

5. **API 多态支持**：
   - `RegisterFrame(frame)` → 无去畸变
   - `RegisterFrame(frame, timestamps)` → 带去畸变
   - 两个重载，简单清晰

---

## 6. 优缺点分析

### 6.1 算法优点

| 优点 | 说明 |
|------|------|
| 自适应场景分类 | α 自动调节 point-to-plane 与 point-to-point 权重，适应不同场景 |
| 退化鲁棒性 | 非平面场景（如隧道少特征）自动切换到 point-to-point 模式 |
| 自适应阈值 | σ 根据运动模型误差自动调节，无需手工设置匹配距离 |
| 简单可复现 | 全算法 ~600 行，易于理解和修改 |
| 学术验证充分 | 已发表 IEEE RA-L 论文，在 KITTI 等数据集上有基准结果 |

### 6.2 算法缺点

| 缺点 | 说明 |
|------|------|
| 纯激光里程计 | 无 IMU，退化场景（长隧道）缺乏额外约束 |
| 无闭环检测 | 纯 odometry，累计漂移无法消除 |
| 协方差估计缺失 | 无 pose uncertainty 输出，不利于多传感器融合 |
| 恒速运动模型 | 仅用前两帧外推，急转弯时初始猜测偏差大 |
| 27 邻域搜索开销 | 每个源点要查 27 个体素，点数多时开销显著 |

### 6.3 工程优点

| 优点 | 说明 |
|------|------|
| ROS 双版本支持 | 同一代码库同时支持 ROS1 Noetic 和 ROS2 Humble |
| 零额外依赖 | ROS 包不需要 PCL/Ceres/GTSAM |
| 编译即用 | `catkin build` 或 `colcon build` 即可 |
| MIT 许可证 | 商业友好的开源协议 |
| FetchContent 支持 | ROS 包可自动从 GitHub 拉取 cpp/ 核心代码 |
| 完善的调参指南 | `parameter_tuning_guide.md` 详细说明每个参数 |

### 6.4 工程缺点

| 缺点 | 说明 |
|------|------|
| C++20 要求 | 使用 `const auto &[a, b] = func()` 结构化绑定 |
| 调试成本 | TBB 并行 + 哈希表使得调试较为困难 |
| 无 C++ 主程序 | cpp/ 层只有 static library，无独立可执行文件（需通过 ROS 调用或自行编写） |

### 6.5 适用场景
- 激光雷达里程计作为多传感器融合的前端
- KITTI、nuScenes 等自动驾驶数据集评估
- 需要快速部署的机器人 SLAM 系统
- 结构化（建筑）+ 非结构化（植被）混合环境

### 6.6 不适用场景
- 需要绝对定位的任务（无 GPS/RTK 集成）
- 大型长距离场景（无闭环检测）
- 极端退化场景（长隧道、空旷高速路）

---

## 7. 对 phad_fusion 的关键参考

### 7.1 可借鉴的设计

| 设计元素 | 说明 | 来源 |
|---------|------|------|
| cpp/ + ros/ 分离架构 | 算法完全脱离 ROS，通过 thin wrapper 桥接 | 全部目录结构 |
| 自适应权重 α | planar vs non-planar 自动权衡，可扩展为多类特征融合 | `Registration.cpp:202` |
| 协方差平面度判定 | 特征值比例判定平面度，物理意义明确 | `VoxelHashMap.cpp:114-120` |
| AdaptiveThreshold | 运动误差统计 + 自适应匹配距离 | `Threshold.cpp:39-50` |
| 双重体素化 | 地图用细体素 + 配准用粗体素 | `GenZICP.cpp:91-95` |
| Header-only Utils | ROS 转换工具 inline 在 header 中 | `Utils.hpp` |
| GenZConfig 模式 | 参数集中管理，带默认值 | `GenZICP.hpp:36-58` |

### 7.2 应避免的陷阱

| 陷阱 | 说明 | 改进建议 |
|------|------|----------|
| VoxelDownsample 多个变体 | 代码重复（VoxelDownsample/VoxelDownsampleForMap/VoxelDownsampleForScan 内容几乎相同） | 统一为一个函数 + 参数控制日志 |
| 静态体素大小 | `static double voxel_size = config_.voxel_size` 在函数调用间保持状态 | 改为类成员 |
| mid_pose_timestamp 硬编码 | `constexpr double mid_pose_timestamp{0.5}` | 改为可配置参数 |
| 无失败回退 | ICP 收敛失败时仍返回结果 | 增加协方差检查 + fallback to constant velocity |
| No IMU prediction | 恒速模型在急刹车/急转弯时严重偏差 | 可扩展为 IMU 预积分预测 |
| 无时间戳验证 | 未检查 timestamps 有效性（size 必须等于 points size） | 添加 assert |

### 7.3 核心算法集成建议

phad_fusion 可以将 GenZ-ICP 的 **自适应权重机制** 集成到前端里程计中：

1. **扩展现有的特征提取**：在提取平面/边缘特征后，添加协方差平面度判定
2. **自适应 α 权重**：根据 planar/non-planar 点数比动态调整
3. **复用 AdaptiveThreshold**：替代硬编码的匹配距离阈值
4. **参考分离架构**：在 phad_fusion 中也建立 `core/`（ROS 无关）+ `ros/`（wrapper）结构

---

### 附录：关键文件索引

| 模块 | 头文件 | 实现文件 | 代码行数（约） |
|------|-------|---------|-------------|
| GenZICP pipeline | `cpp/genz_icp/pipeline/GenZICP.hpp:60` | `cpp/genz_icp/pipeline/GenZICP.cpp` | 97 + 117 = 214 |
| Registration ICP solver | `cpp/genz_icp/core/Registration.hpp:34` | `cpp/genz_icp/core/Registration.cpp` | 46 + 222 = 268 |
| VoxelHashMap | `cpp/genz_icp/core/VoxelHashMap.hpp:36` | `cpp/genz_icp/core/VoxelHashMap.cpp` | 88 + 256 = 344 |
| Preprocessing | `cpp/genz_icp/core/Preprocessing.hpp:30` | `cpp/genz_icp/core/Preprocessing.cpp` | 51 + 136 = 187 |
| Deskew | `cpp/genz_icp/core/Deskew.hpp:30` | `cpp/genz_icp/core/Deskew.cpp` | 37 + 49 = 86 |
| AdaptiveThreshold | `cpp/genz_icp/core/Threshold.hpp:29` | `cpp/genz_icp/core/Threshold.cpp` | 54 + 52 = 106 |
| Metrics | `cpp/genz_icp/metrics/Metrics.hpp:31` | `cpp/genz_icp/metrics/Metrics.cpp` | 40 + 192 = 232 |
| ROS1 Wrapper | `ros/ros1/OdometryServer.hpp:40` | `ros/ros1/OdometryServer.cpp` | 96 + 228 = 324 |
| ROS2 Wrapper | `ros/ros2/OdometryServer.hpp:41` | `ros/ros2/OdometryServer.cpp` | 96 + 281 = 377 |
| Utils (ROS1) | `ros/ros1/Utils.hpp` (header-only) | - | 236 |
| Utils (ROS2) | `ros/ros2/Utils.hpp` (header-only) | - | 238 |

**外部库依赖总结**：

| 库 | cpp/genz_icp/（算法核心） | ros/（ROS 封装） |
|----|------------------------|-----------------|
| Eigen3 | ✅（所有模块） | ✅ |
| Sophus | ✅（Registration, Deskew, Threshold） | ✅ |
| TBB | ✅（Registration, VoxelHashMap, Preprocessing, Deskew） | 可选 |
| tsl::robin_map | ✅（VoxelHashMap, Preprocessing） | 间接 |
| PCL | ❌ 不使用 | ❌ 不使用 |
| Ceres | ❌ 不使用 | ❌ 不使用 |
| GTSAM | ❌ 不使用 | ❌ 不使用 |
| ROS | ❌ 不使用 | ✅ |
| yaml-cpp | ❌ 不使用 | ✅（ROS2 配置加载） |


## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 原始格式 | ROS接口 | 消费模块 |
|--------|------|----------|---------|----------|
| LiDAR (Velodyne/Ouster/等) | 10Hz | `sensor_msgs::PointCloud2` | `OdometryServer::RegisterFrame()` | GenZICP pipeline |

**纯LiDAR里程计**: 无IMU, 无相机, 无GPS。单一topic输入。`RegisterFrame(frame)` (无去畸变) 或 `RegisterFrame(frame, timestamps)` (带去畸变)。

### 8.2 LiDAR管线

```
原始PointCloud2
  → OdometryServer::RegisterFrame() (ROS wrapper):
      PointCloud2ToEigen(msg): ROS迭代器直接读x/y/z字段 → vector<Eigen::Vector3d> (无PCL依赖)
      GetTimestamps(msg): 提取逐点time (支持t/timestamp/time字段, UINT32/FLOAT32/FLOAT64)
  → GenZICP::RegisterFrame() (pipeline核心):
      step 1: Preprocess(frame, max_range, min_range): std::copy_if范围滤波
      step 2: VoxelDownsample(cropped, voxel_size=0.25m):
              tsl::robin_map哈希表, hash=vec[0]*73856093^vec[1]*19349663^vec[2]*83492791
              自适应体素: voxel_size *= n_points/desired_num(2000), clamp[0.02, 2.0]
      step 3: 双重体素化:
              frame_downsample = VoxelDownsample(frame, max(adaptive_voxel*0.5, 0.02))  // 地图保留
              source = VoxelDownsample(frame_downsample, adaptive_voxel*1.0)              // 配准用
      step 4: Deskew(source, timestamps, start_pose, finish_pose) (可选):
              delta_pose = (start.inv() * finish).log()  // 6D twist
              补偿到帧中点(t=0.5): motion = exp((t_i - 0.5) * delta_pose)
      step 5: GetPredictionModel(): 恒速模型: pred = poses[N-2].inv() * poses[N-1]
              initial_guess = last_pose * pred
  → Registration::RegisterFrame() (ICP求解器):
      for i=0..max_iter(150):
        GetCorrespondences(TBB parallel_reduce):
          27邻域搜索 (当前体素+26邻居)
          GetClosestNeighbor(): 最近点 + 累加centroid + 累加covariance
          DeterminePlanarity(): λ3/(λ1+λ2+λ3) < planarity_threshold → is_planar
          planar → source/target/normal / non-planar → source/target
        BuildLinearSystem():
          α = planar_count / (planar_count + non_planar_count)  // 自适应权重
          Geman-McClure鲁棒核: w = kernel²/(kernel + r²)²
          planar残差: JTJ += α*J_p^T*w*J_p, JTr += α*J_p^T*w*r_p  (r_p标量, J_p(1×6))
          non-planar残差: JTJ += (1-α)*Jnp^T*w*Jnp, JTr += (1-α)*Jnp^T*w*r_np  (r_np(3), Jnp(3×6))
        求解: dx = JTJ.ldlt().solve(-JTr)  (LDLT分解)
        更新: estimation = SE3::exp(dx), T_icp = estimation * T_icp
        收敛: dx.norm() < 0.0001 → break
      return T_icp * initial_guess;
  → VoxelHashMap更新: voxel_map.AddFrame(frame_downsample, pose)
  → AdaptiveThreshold更新: ComputeThreshold() → σ = sqrt(model_error_sse2/num_samples)
                          max_correspondence_distance = 3.0 * σ
```

**标定**: 无外参标定(首帧LiDAR系=世界系), 无内参  
**预处理**: 范围裁剪 → 自适应体素降采样 → 双重体素化(地图细/配准粗) → 可选运动畸变校正(恒速模型)  
**特征**: 在线自适应分类: 协方差Eigen分解 → λ3/(λ1+λ2+λ3)判断planar/non-planar (≥5邻居)  
**匹配**: VoxelHashMap 27邻域搜索 → 最近体素邻居 → planar=(source,target,normal) / non-planar=(source,target)  
**因子构建**: planar: J_p=[n^T, (s×n)^T] (1×6), non-planar: Jnp=[I, -[s]×] (3×6), 自适应α平衡权重, Geman-McClure鲁棒核

### 8.3 自适应阈值管线

```
每帧ICP完成后:
  model_error = |ΔT| + 2*max_range*sin(|ΔR|/2)  // 运动模型偏差
  if model_error > 0.1m:  // min_motion_th
      model_error_sse2 += model_error²
      num_samples++
  输出: σ = sqrt(model_error_sse2 / num_samples)
       max_correspondence_distance = 3.0 * σ
       kernel = σ / 3.0
  初始: σ=2.0m → 逐步收敛到运动误差RMSE
```

### 8.4 跨传感器/模块协同

| 协同机制 | 实现位置 | 说明 |
|----------|----------|------|
| 时间同步 | `msg->header.stamp` 直接作为输出时间戳 | 单传感器无多传感器同步问题 |
| 缓冲策略 | 无缓冲, 逐帧同步处理 | `RegisterFrame` 同步返回位姿 |
| 初始化 | 首帧地图为空 → 直接返回identity | 地图构建后下一帧开始ICP |
| 运动预测 | 恒速模型: 前两帧位姿差外推 | `HasMoved()>0.5m` 后启用自适应阈值 |
| 降级策略 | 无显式降级 | ICP收敛失败仍返回结果, 无fallback机制 |
| 去畸变 | 可选, 基于恒速运动模型 | 不需要IMU, 上帧位姿差估计速度 |
| 协方差 | 不输出 | JTJ的逆理论上可提供不确定度, 代码未计算 |
| ROS分层 | cpp/core(纯算法) + ros/(薄wrapper) | 算法完全脱离ROS, 可通过任何框架调用 |