---
tags: [LiDAR, 地图表示, 体素, 八叉树, 点云]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-lightning-lm-analysis.md
  - wiki/sources/2026-04-28-superodom-analysis.md
  - wiki/sources/2026-04-29-kiss_icp_analysis.md
  - wiki/sources/2026-04-29-fast_livo2_analysis.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
  - wiki/sources/2026-04-29-pin_slam_analysis.md
  - wiki/sources/2026-05-02-p2v-slam.md
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
---

# LiDAR 地图表示

> LiDAR SLAM 中地图数据结构的设计空间综述：体素哈希、八叉树、高斯压缩体素——覆盖 KISS-ICP、FAST-LIVO2、gtsam_points 和 P2V-SLAM 中的四种核心实现。

## 概述

在 LiDAR SLAM 系统中，地图表示决定了三个关键性能边界：**数据关联速度**（每帧扫描点如何找到目标点云中的对应）、**内存增长**（长期在线运行多大）、以及**配准精度**（残差计算中是否携带不确定性信息）。

不同于视觉 SLAM 依赖特征描述子做稀疏匹配，LiDAR 的前端配准（scan-to-map）通常是稠密的——每一帧数千甚至数万点需要在局部地图中找到最近邻。如果直接用原始点云做全局 KNN，每帧 O(N log N) 的查询成本在实时系统中不可接受。因此，LiDAR 系统普遍用**空间分桶**（spatial bucketing）结构将连续空间离散化，在局部邻域内做近似的最近邻或分布匹配。

本文覆盖四类主流变体：

| 变体 | 核心思想 | 代表框架 |
|------|----------|----------|
| 通用体素地图 | 固定边长立方体 + 哈希表，每个体素存点数/均值/协方差/平面参数 | FAST-LIO, PIN-SLAM |
| VoxelHashMap | `tsl::robin_map` 哈希 + 27 邻域遍历最近邻，每体素限点数 + 二级去重 | KISS-ICP, GenZ-ICP |
| VoxelMap 八叉树 | 顶层哈希索引 + 八叉树递归切割，叶节点存 VoxelPlane（含法向量、6×6 协方差） | FAST-LIVO2 |
| GaussianVoxelMap | 体素内压缩为高斯分布 (μ, Σ)，配准用马氏距离残差，CPU/GPU 双实现 | gtsam_points (VGICP, ROLO) |

它们的共同哲学是：**用体素化牺牲一定程度的最近邻精度，换取可控的内存增长和可并行的数据关联**。体素大小是核心超参数——过小则内存爆炸且体素内点数不足导致协方差退化，过大则抹掉薄结构降低配准精度。

---

## 变体对比表

| 维度 | VoxelHashMap | VoxelMap 八叉树 | GaussianVoxelMap | iKD-Tree（对比基线） |
|------|-------------|---------------|-----------------|-------------------|
| 数据结构 | `tsl::robin_map` 哈希表 | `unordered_map` + 八叉树 | `IncrementalVoxelMap` 哈希 | 增量 kd 树 |
| 近邻查询 | O(K) 27 邻域遍历 | O(log N) 递归查找叶平面 | O(1) 体素索引 → 高斯均值 | O(log N) 树搜索 |
| 内存效率 | 高（每体素限点数） | 中（八叉树节点开销） | 高（每体素压为 1 个分布） | 中（节点+点存储） |
| 增量更新 | 快，体素插入 + 远点裁剪 | 中等，递归分配子节点 | 快，累积均值/协方差 | 支持动态插入删除 |
| 退化处理 | 无（欧氏最近邻，不估平面） | 有（特征值比判断平面度） | 有（协方差正则化或回退） | 无 |
| 不确定性 | 无 | 有（6×6 平面协方差） | 有（联合协方差马氏距离） | 无 |
| 代表框架 | KISS-ICP | FAST-LIVO2 | gtsam_points / ROLO | FAST-LIO2, R3LIVE |

---

## 各变体详解

### 1. 通用体素地图

将 3D 空间切为固定边长 `v` 的立方体，用哈希表 `unordered_map< voxel_id, bucket >` 管理。体素内可存储：

- **点集**：体素内原始点或下采样后的点
- **统计量**：点数 `N`、均值 `μ`、协方差 `C = (1/N) Σ (p_i - μ)(p_i - μ)^T`
- **平面参数**：法向量 `n`、距原点距离 `d`、平面度评分（PCA 特征值比）

体素化的价值在于建立稳定的局部统计单元：近邻查询从全局搜索变局部桶查询；下采样保留空间均匀性；协方差可为 [[方法-GICP配准方法]] 提供权重。

工程上，LiDAR SLAM 的体素地图通常配合**滑动局部地图**使用，而非无限增长的全局网格。体素大小 `v` 是关键参数——需与 LiDAR 线束密度、场景几何复杂度和 ICP 对应距离协调。过小导致体素内点数 `N → 0`、协方差奇异；过大导致薄结构（细杆、边缘）被体素模糊。

P2V-SLAM 的 iVoxMap 将体素从统计单元升级为**可学习的隐式观测单元**：每个体素维护 VE-Net 提取的隐式特征，query 时 IR-Net 用体素特征预测隐式残差和不确定度，送入 IESKF。详见 [[方法-隐式点-体素观测模型]]。

### 2. VoxelHashMap

基于 `tsl::robin_map`（开放寻址哈希表）的体素局部地图，作为 KISS-ICP 和 GenZ-ICP 的核心数据结构。

**哈希函数**使用与 iVox 一致的空间哈希：

$$
h(\mathbf{v}) = v_x \cdot 73856093 \ \oplus \ v_y \cdot 19349663 \ \oplus \ v_z \cdot 83492791
$$

其中 `v_x, v_y, v_z` 是体素整数坐标。

**插入策略**：每个体素维护最多 `max_points_per_voxel` 个点（默认 20）。插入时先按 `voxel_size` 定位桶，再按二级精度的 `map_resolution = sqrt(voxel_size² / max_points_per_voxel)` 去重——只有新点与桶内所有已有点的距离都超过 `map_resolution` 时才加入。这同时实现了体素下采样和空间去重。

**查询策略**：27 邻域遍历——查询点落入体素 `(v_x, v_y, v_z)` 后，遍历当前体素及其 26 个邻接体素（3×3×3 网格）内的所有点，寻最小欧氏距离：

$$
d(\mathbf{q}) = \min_{\delta \in \{-1,0,1\}^3} \ \min_{\mathbf{p} \in \text{voxel}(\lfloor\mathbf{q}/v\rfloor + \delta)} \|\mathbf{q} - \mathbf{p}\|_2
$$

**滑动窗口**：通过 `RemovePointsFarFromLocation(origin)` 按 `max_distance`（默认 100m）裁剪远点。用每个体素内第一个点近似代表该体素位置，速度快但有边界误差。

工程权衡：查询 O(K) 但 K 被 `max_points_per_voxel` 限制，在稀疏点云分布下接近 O(1)；27 邻域是近似最近邻，体素过大会漏掉真实对应；滑动窗口限制内存但牺牲长期回访一致性，因此 KISS-ICP 不做全局回环。

源码锚点：`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L35-L62`（27 邻域偏移数组）、`L83-L119`（`AddPoints` 插入去重逻辑）。

### 3. VoxelMap 八叉树

FAST-LIVO2 中用于 LiDAR 点面残差点面关联的分层结构。两阶段设计：

- **顶层哈希**：`unordered_map< VOXEL_LOCATION, VoxelOctoTree* >`，用大哈希质数 `P=116101` 做二次散列，快速定位空间区域
- **底层八叉树**：`VoxelOctoTree` 在每个顶层桶内做递归切割，叶节点保存 `VoxelPlane`

**VoxelPlane 结构**包含：点中心 `center_`、法向量 `normal_`、6×6 平面协方差 `plane_var_`（3D 位置 + 3D 法向量的联合协方差）、半径 `radius_`、特征值 `{min, mid, max}_eigen_value_`。

**平面协方差传播**：从点测量噪声的 3×3 协方差 `var_i` 传播为平面参数的 6×6 协方差：

$$
\Sigma_{\text{plane}} = \sum_i J_i \cdot \text{var}_i \cdot J_i^T
$$

其中 `J_i` 是 6×3 的雅可比矩阵，编码了点位置分量对平面参数（中心 + 法向量）的影响。这个传播由 `VoxelOctoTree::init_plane()` 中的特征值灵敏度分析（`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L55-L135`）完成。

**递归切割策略**：叶节点累积 `temp_points_` 后调用 `init_plant()`，若 PCA 最小特征值 `λ_min < planner_threshold_`，则该区域被判为平面，停止切割（`octo_state_ = 0`）；否则继续递归分割为 8 个子立方体（`cut_octo_tree()`），直到达到 `max_layer_`。

**退化防御**：每个节点记录中间特征值 `mid_eigen_value_` 和最大特征值 `max_eigen_value_`，`λ_min` 不满足平面度阈值时不生成平面，避免在边角处做出错误的平面假设。详见 `voxel_map.h:L69-L94`。

vs iKD-Tree：VoxelMap 是**固定网格 + 自适应八叉树**，内存效率更高，适合结构化场景；iKD-Tree（R3LIVE/FAST-LIO2 的替代选项）是**增量 kd 树**，支持动态插入删除，更适合大规模动态场景和频繁地图更新。

### 4. GaussianVoxelMap 体素化配准

将目标点云压缩为体素高斯分布，用**马氏距离**代替欧氏距离表达 GICP/VGICP 风格的分布-分布匹配。

**体素压缩**：对体素内所有点计算样本均值与协方差：

$$
\mu_j = \frac{1}{N} \sum_{i} \mathbf{p}_i, \quad
C_j = \frac{1}{N} \sum_{i} (\mathbf{p}_i - \mu_j)(\mathbf{p}_i - \mu_j)^T
$$

存储在 `GaussianVoxel` 结构（`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp:L13-L52`）的 `mean`（齐次 4D）和 `cov`（4×4）中。

**残差模型**：对 source 点 `p_i` 经变换 `T` 后与 target 体素高斯 `(μ_j, C_j)` 的马氏距离：

$$
r_{ij}^2 = (T\mathbf{p}_i - \mu_j)^T \Sigma_{ij}^{-1} (T\mathbf{p}_i - \mu_j)
$$

其中联合协方差为：

$$
\Sigma_{ij} = C_j + R\ C_i\ R^T
$$

`C_i` 是 source 点自身的测量协方差（可选），`R` 是 `T` 的旋转分量。如果 source 点不带协方差，则 `Σ_{ij} = C_j` 退化为 target 分布加权的马氏距离。

**核心优势**：
- 数据关联从 O(log N) KNN 降为 O(1) 体素索引
- 马氏距离在信息矩阵为零特征值方向退化为无穷大，自动实现类似平面约束的效果——分布在平面法向方向收紧，沿平面方向拉长
- CPU/GPU 双实现（`gaussian_voxelmap_cpu.cpp` 和 `gaussian_voxelmap_gpu.cu`），GPU 版通过 `NonlinearFactorGPU` 批量线性化
- 支持 `overlap()` 和 `overlap_gpu()` 计算帧间重叠率，用于关键帧判定
- `VmfVoxelMap` 变体使用去均值高斯体素（只存协方差、不存均值），适合 [[方法-RotVGICP]]

**工程边界**：
- 体素越大数据关联越快但几何细节越粗；体素越小，体素内点数不足导致协方差估计奇异——需加正则项（如 `C_j + εI`）或回退到点到点/点到面约束
- 协方差更新用累积法：`GaussianVoxel::add()` 逐帧累加均值与协方差，`GaussianVoxel::finalize()` 除以点数——支持增量插入，内存可控（`raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp:L23-L47`）

---

## 选型指南

| 条件 | 推荐结构 | 原因 |
|------|---------|------|
| 轻量里程计、无回环需求 | VoxelHashMap | 最小实现复杂度，滑动窗口天然限制内存 |
| 需要点到面约束 + 不确定性 | VoxelMap 八叉树 | 平面协方差传播支持 IESKF 更新 |
| 需要 GICP/分布匹配精度 | GaussianVoxelMap | 马氏距离 + 联合协方差 + GPU 加速 |
| 大规模动态场景 | iKD-Tree | 支持在线插入删除，无需重建 |
| 通用 LIO/LiDAR SLAM 前端 | 通用体素地图 | 可灵活扩展为平面/协方差/隐式特征存储 |
| 学习式 SLAM / 神经隐式 | iVoxMap (P2V) | 体素存特征，解码器生成残差 |

一般性建议：
1. **从 VoxelHashMap 起步**：代码量小且 API 清晰，适合原型验证。如果后续需要更精确的残差，迁移到 GaussianVoxelMap 或 VoxelMap 八叉树。
2. **体素大小应与 LiDAR 线束密度匹配**：Ouster/Velodyne 128 线 → 0.2-0.5m；Livox 非重复扫描 → 0.1-0.3m；固态激光雷达（Realsense L515）→ 0.05-0.1m。
3. **长期大场景**：优先滑动局部地图 + 关键帧子地图 + 回环时按需查询全局地图，不建议维护无限增长的全局体素。
4. **如果不确定是否需要马氏距离或平面协方差**：先在体素内只存点，后续向 `GaussianVoxel` 或 `VoxelPlane` 升级成本可控。

---

## Agent 实现提示

### 适用场景

- **应该用体素地图**：实时 LiDAR 里程计 / SLAM 的 scan-to-map 前端，需要快速局部最近邻或分布匹配，且场景主要是静态或准静态结构
- **不应该用体素地图**：需要精确全局 KNN（如全局回环检测的描述子匹配），或场景极度稀疏（沙漠、开阔水面）——此时体素几乎全是空的，哈希表开销不划算
- **应该用 VoxelHashMap**：轻量 ICP 里程计，代码最小化优先
- **应该用 VoxelMap 八叉树**：IESKF 后端需要点到面残差 + 平面不确定性
- **应该用 GaussianVoxelMap**：需要 GICP/VGICP 的分布匹配精度，或有 GPU 可用时追求批量吞吐

### 输入输出契约

- **输入**：
  - 世界/局部坐标系下的点云（`vector<Vector3d>` 或 `PointCloud`）
  - 传感器当前位置 `origin`（用于滑动窗口裁剪）
  - 体素大小 `voxel_size`（米）
  - 最大保留距离 `max_distance`（米，滑动窗口半径）
  - 每体素最大点数 `max_points_per_voxel`（VoxelHashMap 特有）
- **输出**：
  - 可查询的体素地图对象（`GetClosestNeighbor` / 体素高斯 / 叶平面）
  - 裁剪后的局部地图点云（`Pointcloud()`）
  - 残差项（点面距离 `d`、不确定度、马氏距离平方 `r²`）
- **坐标约定**：所有插入点必须在世界/地图坐标系；传感器本体系点先经位姿变换再插入
- **前置条件**：体素大小与 ICP 对应搜索距离、点云下采样分辨率保持同一量级

### 实现骨架（伪代码）

```text
// ---- VoxelHashMap 风格 ----
function VoxelHashMap_Update(points_world, origin):
    map_resolution = sqrt(voxel_size^2 / max_points_per_voxel)
    for p in points_world:
        v = floor(p / voxel_size)
        bucket = map.get_or_create(v)
        if bucket.size() >= max_points_per_voxel:
            continue
        if any(|q - p| < map_resolution for q in bucket):
            continue
        bucket.push(p)
    for (v, bucket) in map:
        if |bucket[0] - origin| > max_distance:
            map.erase(v)

function VoxelHashMap_Nearest(query):
    v = floor(query / voxel_size)
    best_dist = INF
    for d in { -1, 0, 1 }^3:
        for p in map[v + d]:
            best_dist = min(best_dist, |query - p|)
    return best_dist, best_point

// ---- GaussianVoxelMap 风格 ----
function GaussVoxel_Insert(points):
    for p in points:
        v = floor(p / voxel_size)
        voxel = map.get_or_create(v)
        voxel.count += 1
        voxel.sum_mean += p                      // 累积和
        voxel.sum_cov  += (p * p^T)              // 外积累积

function GaussVoxel_FinalizeAll():
    for voxel in map:
        voxel.mean = voxel.sum_mean / voxel.count
        voxel.cov  = voxel.sum_cov / voxel.count - voxel.mean * voxel.mean^T

function GaussVoxel_Mahalanobis(T, p_source, cov_source):
    q = T * p_source
    v = floor(q / voxel_size)
    (mu_target, cov_target) = map[v]
    Sigma = cov_target + R * cov_source * R^T   // 联合协方差
    r = sqrt((q - mu_target)^T * Sigma^(-1) * (q - mu_target))
    return r

// ---- VoxelMap 八叉树（平面拟合）----
function OctoVoxel_InitPlane(points):
    N = points.size
    center = sum(p) / N
    cov = sum(p * p^T) / N - center * center^T
    eigenvalues, eigenvectors = eigen_decompose(cov)
    if eigenvalues.min < plane_threshold:
        normal      = eigenvectors.col(eigenvalues.min_idx)
        plane_var   = sum(J_i * point_var_i * J_i^T)   // 不确定性传播
        radius      = sqrt(eigenvalues.max)
        return VoxelPlane(center, normal, plane_var, radius)
    else:
        return NULL   // 非平面，需继续分割
```

### 关键源码片段

**VoxelHashMap：27 邻域查询 + 插入**
`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L35-L62, L83-L119`

```cpp
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0,0,0},   Voxel{1,0,0},   Voxel{-1,0,0},  Voxel{0,1,0},   Voxel{0,-1,0},
     Voxel{0,0,1},   Voxel{0,0,-1},  Voxel{1,1,0},   Voxel{1,-1,0},  Voxel{-1,1,0},
     Voxel{-1,-1,0}, Voxel{1,0,1},   Voxel{1,0,-1},  Voxel{-1,0,1},  Voxel{-1,0,-1},
     Voxel{0,1,1},   Voxel{0,1,-1},  Voxel{0,-1,1},  Voxel{0,-1,-1}, Voxel{1,1,1},
     Voxel{1,1,-1},  Voxel{1,-1,1},  Voxel{1,-1,-1}, Voxel{-1,1,1},  Voxel{-1,1,-1},
     Voxel{-1,-1,1}, Voxel{-1,-1,-1}}};

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const auto voxel = PointToVoxel(point, voxel_size_);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_points = search.value();
            if (voxel_points.size() == max_points_per_voxel_ ||
                std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                            [&](const auto &voxel_point) {
                                return (voxel_point - point).norm() < map_resolution;
                            }))
                return;
            voxel_points.emplace_back(point);
        } else {
            map_.insert({voxel, std::vector<Eigen::Vector3d>{point}});
        }
    });
}
```

**GaussianVoxel：增量均值/协方差累积与最终化**
`raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp:L23-L47`

```cpp
void GaussianVoxel::add(const Setting&, const PointCloud& points, size_t i) {
    if (finalized) {
        this->finalized = false;
        this->mean *= num_points;
        this->cov  *= num_points;
    }
    num_points++;
    this->mean += points.points[i];
    this->cov  += points.covs[i];
    if (frame::has_intensities(points))
        this->intensity = std::max(this->intensity, frame::intensity(points, i));
}

void GaussianVoxel::finalize() {
    if (finalized) return;
    mean /= num_points;
    cov  /= num_points;
    finalized = true;
}
```

**VoxelMap 八叉树：叶平面平面度判定与协方差传播**
`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L55-L135`

```cpp
void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane) {
    plane->points_size_ = points.size();
    for (auto pv : points) {
        plane->covariance_ += pv.point_w * pv.point_w.transpose();
        plane->center_     += pv.point_w;
    }
    plane->center_     = plane->center_ / plane->points_size_;
    plane->covariance_ = plane->covariance_ / plane->points_size_
                         - plane->center_ * plane->center_.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    // ... 取最小特征值对应向量为法向
    if (evalsReal(evalsMin) < planer_threshold_) {
        plane->normal_ = evecs.real().col(evalsMin);
        // 不确定性传播：J = ∂(n, center)/∂p_i
        plane->plane_var_ += J * points[i].var * J.transpose();
        plane->is_plane_ = true;
    }
}
```

**VoxelMap 八叉树：递归切割**
`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L163-L199`

```cpp
void VoxelOctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) { octo_state_ = 0; return; }
    for (size_t i = 0; i < temp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point_w[0] > voxel_center_[0]) xyz[0] = 1;
        if (temp_points_[i].point_w[1] > voxel_center_[1]) xyz[1] = 1;
        if (temp_points_[i].point_w[2] > voxel_center_[2]) xyz[2] = 1;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new VoxelOctoTree(...);
            leaves_[leafnum]->voxel_center_[0]
                = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            // ... 同理设置 y, z 子中心
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    }
}
```

### 实现注意事项

1. **体素哈希函数选择**：所有变体的核心。KISS-ICP 和 iVox 用 `v_x*73856093 ^ v_y*19349663 ^ v_z*83492791`（三个大质数异或）；FAST-LIVO2 用 `(z*P+y)*P+x`（嵌套取模），两者效果等价，选习惯的即可。**注意**：不要用 std::pair 做 key——碰撞后性能差；用自定义 hash 对 `Eigen::Vector3i` 或整数三元组。
2. **`map_resolution` 和 `voxel_size` 不能混淆**：`voxel_size` 是哈希桶的边长；`map_resolution` 是二级去重尺度（VoxelHashMap 特有），通常是 `voxel_size / sqrt(max_points)`。前者控制查询复杂度，后者控制局部密度。
3. **滑动窗口裁剪的近似误差**：VoxelHashMap 用体素内第一个点近似代表体素到原点的距离，删除时可能误删或误留边界体素。对精度敏感的配准，考虑用体素中心而非首点做距离计算。
4. **协方差退化处理**：GaussianVoxelMap 中体素内点数不足时协方差矩阵奇异，求逆会爆炸。必须加正则项：`Σ_ij = C_j + R C_i R^T + εI`，`ε` 取体素对角长度的量级（如 `ε = voxel_size²/12`，即均匀分布的方差）。
5. **平面度判定阈值**：VoxelMap 八叉树的 `planner_threshold_` 直接决定哪些区域生成平面——过大则把曲面也当平面；过小则仅极度平坦区域生面（地板、墙壁），在边缘/角落处漏掉约束。FAST-LIVO2 默认 `0.01`（`voxel_map.cpp:L42`），可根据场景调整。
6. **多线程安全**：ICP 查询期间地图应只读。插入和裁剪应在配准完成后的单线程阶段执行，或对哈希表加读写锁（但会损害实时性）。FAST-LIVO2 的 `BuildResidualListOMP` 使用 OMP 多线程并行查找叶平面（`voxel_map.cpp:L238`），前提是八叉树结构在查询期间不变。
7. **坐标系一致**：所有插入点必须是**地图/世界坐标系**。在传感器本体系下做体素化会产生二义性——同一个物理点随传感器运动出现在不同体素中。

### 源码检索锚点

- VoxelHashMap：`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.{hpp,cpp}` — `Update`, `AddPoints`, `RemovePointsFarFromLocation`, `GetClosestNeighbor`, `voxel_shifts`, `PointToVoxel`
- GaussianVoxelMap：`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp` — `GaussianVoxel`, `GaussianVoxelMapCPU`；`raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp` — `GaussianVoxel::add()`, `GaussianVoxel::finalize()`
- GaussianVoxelMap GPU：`raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_gpu.cu` — GPU 因子批量线性化
- GaussianVoxelData 序列化：`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxel_data.hpp` — 紧凑化存储结构
- VoxelMap 八叉树：`raw/codes/FAST-LIVO2/include/voxel_map.h` — `VoxelPlane`, `VoxelOctoTree`, `VoxelMapManager`；`raw/codes/FAST-LIVO2/src/voxel_map.cpp` — `init_plane()`, `cut_octo_tree()`, `BuildResidualListOMP()`
- 体素哈希工具：`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelUtils.hpp` — 体素坐标转换、哈希函数
- iKD-Tree 对比：`raw/codes/FAST-LIO2/` — 增量 kd 树实现（与 VoxelMap 对比参考）

---

## 相关页面

- [[方法-体素地图]]（归并前的通用体素概述）
- [[方法-VoxelHashMap]]（归并前的 KISS-ICP 实现细节）
- [[方法-VoxelMap八叉树]]（归并前的 FAST-LIVO2 八叉树细节）
- [[方法-GaussianVoxelMap 体素化配准]]（归并前的 GICP 风格体素细节）
- [[方法-ICP配准方法]]
- [[方法-GICP配准方法]]
- [[方法-RotVGICP]]
- [[方法-在线平面拟合]]
- [[方法-IESKF滤波器]]
- [[方法-隐式点-体素观测模型]]
- [[算法-KISS-ICP]]
- [[算法-FAST-LIVO2]]
- [[算法-ROLO-SLAM]]
- [[算法-P2V-SLAM]]
- [[算法-FAST-LIO]]
- [[算法-PIN-SLAM]]
- [[组件-gtsam_points]]
- [[组件-IVox3d]]
- [[架构-滑动窗口优化]]
- [[概念-可微地图]]
