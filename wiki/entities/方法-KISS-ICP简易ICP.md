---
tags: [KISS-ICP, ICP, LiDAR SLAM, 点云配准, 体素哈希表, 自适应阈值, Geman-McClure]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp
  - raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.hpp
  - raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp
  - raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.hpp
  - raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.hpp
  - raw/codes/kiss-icp/cpp/kiss_icp/core/Preprocessing.hpp
  - raw/codes/kiss-icp/cpp/kiss_icp/core/Threshold.hpp
---

# KISS-ICP: 极简 LiDAR 里程计

> "Keep It Simple, Stupid" —— KISS-ICP 是一个极致简约的 LiDAR 里程计框架：无特征提取、无地面分割、无 IMU 融合，仅用体素降采样 + 点对点 ICP + 自适应阈值 + 匀速运动模型，代码少于 500 行 C++ 核心逻辑。

## 设计哲学

KISS-ICP 的核心主张：**现代 LiDAR 里程计不需要复杂的前处理**。传统 SLAM 方法（LOAM、LeGO-LOAM）的核心计算消耗在特征提取、边缘/平面分类、地面分割等步骤上，KISS-ICP 证明只要配准本身够鲁棒，这些步骤都是可选的。

极简管线：
```
原始点云 → 去畸变 → 体素降采样(两次) → ICP配准 → 更新局部地图 → 发布位姿
```

## 系统架构

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.hpp:L56-L97`

```cpp
class KissICP {
    KISSConfig config_;             // 配置参数
    Preprocessor preprocessor_;     // 距离裁剪 + 去畸变
    Registration registration_;     // 点对点ICP + Geman-McClure核
    VoxelHashMap local_map_;        // 体素哈希表局部地图
    AdaptiveThreshold adaptive_threshold_;  // 自适应对应距离阈值

    Sophus::SE3d last_pose_;        // 上一帧位姿
    Sophus::SE3d last_delta_;       // 帧间增量 (const velocity model)
};
```

## 每帧处理流程

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp:L35-L68`

```cpp
Vector3dVectorTuple RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<double> &timestamps) {
  // 1. 预处理 (距离裁剪 + 去畸变)
  const auto &preprocessed = preprocessor_.Preprocess(frame, timestamps, last_delta_);

  // 2. 体素化 (两层降采样)
  const auto &[source, frame_downsample] = Voxelize(preprocessed);

  // 3. 自适应阈值
  const double sigma = adaptive_threshold_.ComputeThreshold();

  // 4. 初始位姿 (constant velocity model)
  const auto initial_guess = last_pose_ * last_delta_;

  // 5. ICP 配准
  const auto new_pose = registration_.AlignPointsToMap(
      source, local_map_, initial_guess, 3.0 * sigma, sigma);

  // 6. 更新模型偏差 (用于自适应阈值)
  const auto model_deviation = initial_guess.inverse() * new_pose;
  adaptive_threshold_.UpdateModelDeviation(model_deviation);

  // 7. 更新局部地图 + 增量
  local_map_.Update(frame_downsample, new_pose);
  last_delta_ = last_pose_.inverse() * new_pose;
  last_pose_ = new_pose;

  return {preprocessed, source};
}
```

## 匀速运动模型

KISS-ICP 使用最简单的运动先验：上一帧的位姿增量作为当前帧的运动估计。

$$
\mathbf{T}_{\text{guess}} = \mathbf{T}_{k-1} \cdot (\mathbf{T}_{k-1}^{-1} \mathbf{T}_{k-2})
$$

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp:L47`

```cpp
const auto initial_guess = last_pose_ * last_delta_;
```

不需要 IMU，不需要轮速计。对于帧率 ≥ 10 Hz 的 LiDAR，匀速假设在绝大多数场景下足够好。这一设计是 KISS-ICP 保持极简的关键之一。

## 体素降采样与局部地图

### 两层降采样

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp:L70-L75`

```cpp
Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    // 第一层：粗降采样 (voxel_size * 0.5) → 构建地图用
    const auto frame_downsample = VoxelDownsample(frame, voxel_size * 0.5);
    // 第二层：细降采样 (voxel_size * 1.5) → 配准 source 用
    const auto source = VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}
```

设计意图：
- 大分辨率（1.5× vs）source 用于配准：点数少，速度更快
- 小分辨率（0.5× vs）frame_downsample 用于建图：点更密，地图精度更高

### VoxelHashMap 局部地图

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.hpp:L38-L57`

```cpp
struct VoxelHashMap {
    tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_;
    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;

    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;
};
```

使用 `tsl::robin_map`（高性能哈希表）实现体素地图，每个体素内存储最多 `max_points_per_voxel` 个点。邻近查询时返回最近邻点及其距离，用于确定对应关系和距离阈值过滤。

## 点对点 ICP 配准

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L138-L167`

经典的点对点 ICP 实现，加入 Geman-McClure 鲁棒核：

```cpp
Sophus::SE3d AlignPointsToMap(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &initial_guess,
    const double max_distance,
    const double kernel_scale) {

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        // 数据关联
        const auto correspondences = DataAssociation(source, voxel_map, max_distance);

        // 构建线性系统 (带 Geman-McClure 鲁棒加权)
        const auto &[JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);

        // LDLT 求解
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        if (dx.norm() < convergence_criterion_) break;
    }
    return T_icp * initial_guess;
}
```

### Geman-McClure 鲁棒核

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L96-L98`

```cpp
auto GM_weight = [&](const double &residual2) {
    return square(kernel_scale) / square(kernel_scale + residual2);
};
```

权重函数：
$$
w(r^2) = \frac{\sigma^2}{(\sigma^2 + r^2)^2}
$$

其中 $\sigma$ 是自适应阈值（`adaptive_threshold_.ComputeThreshold()`）。残差大的对应点被大幅降权，实现外点鲁棒性。

### 雅可比结构

每条对应关系的残差雅可比（6 维 Lie algebra）：

$$
\mathbf{J}_r = \begin{bmatrix} \mathbf{I}_{3 \times 3} & -[\mathbf{p}_s]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6}
$$

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/core/Registration.cpp:L81-L88`

```cpp
const auto &[source, target] = correspondence;
const Eigen::Vector3d residual = source - target;
Eigen::Matrix3_6d J_r;
J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
```

### 并行化

使用 Intel TBB 实现多线程并行：
- `DataAssociation`：`tbb::parallel_for` 并行查找最近邻
- `BuildLinearSystem`：`tbb::parallel_reduce` 并行构建 `JTJ` 和 `JTr`

## 自适应阈值

`AdaptiveThreshold` 根据运动模型的预测偏差自动调整 ICP 的对应距离阈值：

- 模型预测准（偏差小）→ 阈值收紧 → 更高精度
- 模型预测差（偏差大）→ 阈值放宽 → 更大收敛域

**源码锚点**: `raw/codes/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp:L57-L60`

```cpp
const auto model_deviation = initial_guess.inverse() * new_pose;
adaptive_threshold_.UpdateModelDeviation(model_deviation);
```

## Agent 实现提示

### 适用场景

需要极简、快速部署的 LiDAR 里程计前端。适合多线机械旋转 LiDAR（VLP-16、OS1 等），帧率 ≥ 10 Hz。对特征贫乏场景鲁棒（不依赖特征提取），对动态物体有 GM 核保护。不适合纯旋转或退化场景（6-DOF ICP 本质缺陷）。

### 输入输出契约

- **输入**：LiDAR 点云 `vector<Vector3d>`、时间戳 `vector<double>`（用于去畸变）、配置参数（体素大小、迭代次数等）
- **输出**：当前位姿 `Sophus::SE3d`、预处理后点云、配准用 source 点云
- **残差**：点对点欧氏距离 `r = p_source - p_target`，维度 $3 \times 1$

### 实现骨架（伪代码）

```pseudo
function KissICP.Run():
    T_last = Identity
    delta = Identity
    local_map = VoxelHashMap(voxel_size, max_range, max_points_per_voxel)
    threshold = AdaptiveThreshold(2.0, 0.1, max_range)

    for each LiDAR frame:
        // 1. 距离裁剪 + 去畸变
        frame_clean = filter(frame, min_range, max_range)
        deskew(frame_clean, timestamps, delta)

        // 2. 两层体素降采样
        frame_ds = voxel_downsample(frame_clean, 0.5*vs)
        source   = voxel_downsample(frame_ds, 1.5*vs)

        // 3. 匀速运动先验
        T_guess = T_last * delta
        sigma = threshold.compute()

        // 4. 点对点ICP (带GM核)
        T_new = icp(source, local_map, T_guess, 3*sigma, sigma)

        // 5. 自适应更新
        deviation = T_guess.inv() * T_new
        threshold.update_model_deviation(deviation)
        local_map.update(frame_ds, T_new)
        delta = T_last.inv() * T_new
        T_last = T_new
```

### 关键源码片段

**两层体素化**（`KissICP.cpp:L70-L75`）：
```cpp
const auto frame_downsample = VoxelDownsample(frame, voxel_size * 0.5);
const auto source = VoxelDownsample(frame_downsample, voxel_size * 1.5);
```

**GM 核权重**（`Registration.cpp:L96-L98`）：
```cpp
auto GM_weight = [&](const double &residual2) {
    return square(kernel_scale) / square(kernel_scale + residual2);
};
```

**ICP 迭代求解**（`Registration.cpp:L156-L158`）：
```cpp
const auto &[JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);
const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
```

### 实现注意事项

1. **体素大小是关键参数**：默认 `voxel_size=1.0m`，城市环境 OK，室内需调小到 0.2-0.5m
2. **去畸变需要时间戳**：机械旋转 LiDAR 在运动中会产生点云畸变，`deskew=true` 有效
3. **GM 核的 sigma 来自自适应阈值**：不要手动固定，让 AdaptiveThreshold 自行调节
4. **最大迭代次数**：默认 500 太高，实际 10-20 次即收敛，`convergence_criterion=0.0001` 会提前终止
5. **退化检测**：KISS-ICP 没有内置退化检测，纯旋转或隧道场景可能发散
6. **Python 和 C++ 双版本**：python/ 目录提供相同算法的 Python 实现

### 源码检索锚点

| 模块 | 文件 | 行号 |
|------|------|------|
| RegisterFrame 主流程 | `pipeline/KissICP.cpp` | L35-L68 |
| Voxelize 两层降采样 | `pipeline/KissICP.cpp` | L70-L75 |
| KISSConfig 配置 | `pipeline/KissICP.hpp` | L36-L54 |
| ICP 配准 | `core/Registration.cpp` | L138-L167 |
| GM 核权重 | `core/Registration.cpp` | L96-L98 |
| 雅可比结构 | `core/Registration.cpp` | L81-L88 |
| 数据关联 | `core/Registration.cpp` | L60-L78 |
| VoxelHashMap | `core/VoxelHashMap.hpp` | L38-L57 |
| 预处理 | `core/Preprocessing.hpp` | L32-L45 |

## 相关页面

- [[方法-ICP配准方法]]
- [[方法-点到点ICP]]
- [[方法-Geman-McClure鲁棒核]]
- [[方法-自适应阈值]]
- [[方法-LeGO-LOAM地面优化]]
- [[方法-ROLO-SLAM旋转解耦]]
