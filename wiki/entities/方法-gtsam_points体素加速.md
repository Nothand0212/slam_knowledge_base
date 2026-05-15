---
tags: [gtsam_points, 体素地图, VGICP, iVox, 最近邻搜索, 降采样, 点云配准, GaussianVoxelMap]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-gtsam_points-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
---

# gtsam_points 体素加速

> gtsam_points 提供了一套从简单体素降采样到高斯分布体素地图的加速结构体系，核心是将逐点 KNN 查找替换为 O(1) 哈希体素索引，使 scan-to-map 配准的线性化速度提升一个数量级。

## 体系总览

gtsam_points 的体素加速体系呈三层递进结构：

```text
NearestNeighborSearch       (抽象接口)
    ├── KdTree / KdTree2    (传统 KNN, O(log N))
    └── IncrementalVoxelMap<Container>   (哈希体素, O(1) 查找)
            ├── Container = FlatContainer           → iVox (点级存储)
            │     (include/gtsam_points/ann/ivox.hpp:21)
            ├── Container = IncrementalCovarianceContainer
            │     → IncrementalCovarianceVoxelMap   → iVoxCovarianceEstimation (点级+协方差在线估计)
            │     (include/gtsam_points/ann/ivox_covariance_estimation.hpp:11)
            └── Container = GaussianVoxel            → GaussianVoxelMapCPU (体素级高斯)
                  (include/gtsam_points/types/gaussian_voxelmap_cpu.hpp:73)
```

| 结构 | 粒度 | 每个体素存储 | 适用场景 | 内存特征 |
|------|------|-------------|----------|----------|
| `iVox` (= `IncrementalVoxelMap<FlatContainer>`) | 点级别 | 多个去重后的点 | 快速 scan-to-map ICP | 点密集体素接近 KNN |
| `IncrementalCovarianceVoxelMap` | 点级别 | 点 + 在线估计的协方差+法向量 | 需要逐点协方差的 GICP | LRU 淘汰旧点 |
| `GaussianVoxelMapCPU` | 体素级别 | 单个高斯 (μ, Σ) | 大规模 scan-to-map VGICP | 最省内存 |

## 哈希体素核心：`IncrementalVoxelMap`

`IncrementalVoxelMap<Container>` 是一个模板化的增量体素哈希表，模板参数 `Container` 决定每个体素的内部数据结构。

### 体素索引编码

关键源码 `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp:L84-L86`：

```cpp
// 64-bit 索引编码：高 32 位 = voxel_id，低 32 位 = point_id
inline size_t calc_index(const size_t voxel_id, const size_t point_id) const {
    return (voxel_id << point_id_bits) | point_id;
}
inline size_t voxel_id(const size_t i) const { return i >> point_id_bits; }
inline size_t point_id(const size_t i) const { return i & ((1ull << point_id_bits) - 1); }
```

这种编码让 `knn_search()` 返回的索引同时携带体素 ID 和体素内点 ID，外部可通过 `voxelmap.point(index)` 一次解出实际坐标。

### 体素坐标与哈希

```text
voxel_coord = floor(point / leaf_size)
hash(voxel_coord) = XOR of three int coordinates
```

使用 `XORVector3iHash`（`raw/codes/gtsam_points/include/gtsam_points/util/vector3i_hash.hpp`），对三维整型坐标做按位异或哈希，O(1) 插入和查找。

### LRU 体素淘汰

关键源码 `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp:L124-L126`：

```cpp
size_t lru_horizon;        // LRU 水平线：未访问超过此步数的体素被淘汰
size_t lru_clear_cycle;    // LRU 清理周期：每隔多少帧执行一次淘汰
size_t lru_counter;        // 当前 LRU 计数器
```

每次 `knn_search()` 访问体素时会更新该体素的 `lru` 时间戳。每隔 `lru_clear_cycle` 帧，对所有体素检查 `age = lru_counter - lru`，淘汰超过 `lru_horizon` 的体素。这使动态地图自动遗忘过时区域，无需维护全局 KD-tree。

### 邻接体素搜索模式

`IncrementalVoxelMap` 支持 1/7/19/27 四种邻接模式（对应 6 连通、18 连通、26 连通及包含自身的 27 邻接），由 `set_neighbor_voxel_mode(mode)` 设置。邻接体素偏移量预计算为 `std::vector<Eigen::Vector3i>`。

KNN 查询时先访问查询点所在体素，若点数不足 `k`，再依次访问邻接体素补足 `k` 个近邻。

## FlatContainer：轻量点容器

`FlatContainer`（`raw/codes/gtsam_points/include/gtsam_points/ann/flat_container.hpp:15`）是最简单的体素容器。

```cpp
struct FlatContainer {
    struct Setting {
        double min_sq_dist_in_cell = 0.1 * 0.1;  // 体素内最小点间距
        size_t max_num_points_in_cell = 20;        // 体素内最大点数
    };
    std::vector<Eigen::Vector4d> points;    // 体素内点列表
    std::vector<Eigen::Vector4d> normals;   // 体素内法向量列表
    std::vector<Eigen::Matrix4d> covs;      // 体素内协方差列表

    void add(const Setting& s, const PointCloud& pts, size_t i) {
        // 去重逻辑：体素满 or 最近点距 < min_sq_dist → 拒绝插入
        if (points.size() >= s.max_num_points_in_cell ||
            any(points[j], (pt - pts.points[i]).squaredNorm() < s.min_sq_dist_in_cell))
            return;
        points.emplace_back(pts.points[i]);
    }
};
```

`FlatContainer` 实现了**体素内降采样**——通过最小点间距和最大点数限制，保证每个体素内不会堆积过多冗余点。`iVox = IncrementalVoxelMap<FlatContainer>` 是 Faster-LIO 的原生近邻结构。

## IncrementalCovarianceContainer 与在线协方差估计

继承自 `FlatContainer`，额外维护：

```cpp
std::vector<Eigen::Matrix4d> covs;      // 协方差 (已最终化)
std::vector<size_t> flags;              // 状态标志 (valid / birthday)
```

关键源码 `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_covariance_container.hpp:L82-L83`：

```cpp
static constexpr size_t VALID_BIT = 1ull << 63;       // 最高位：协方差是否已收敛
static constexpr size_t BIRTHDAY_MASK = (VALID_BIT >> 1) - 1;  // 低 62 位：插入时间戳
```

协方差收敛判断：当体素内点数超过 `num_neighbors`（默认 20）且特征值分布满足 `eig_stddev_thresh_scale`（默认 1.0），标记为 `VALID`。依赖 `RunningStatistics<Eigen::Array3d>` 维护特征值统计的滑动窗口（`raw/codes/gtsam_points/include/gtsam_points/util/runnning_statistics.hpp`）。

非 VALID 点在 `knn_search()` 中被跳过（`knn_search_force()` 则不做过滤），防止未收敛协方差污染 GICP 优化。

`iVoxCovarianceEstimation` = `IncrementalCovarianceVoxelMap`（`raw/codes/gtsam_points/include/gtsam_points/ann/ivox_covariance_estimation.hpp:11`），提供完整的增量体素+在线协方差估计能力，是 CT-GICP 和 scan-to-map GICP 因子的推荐 target 结构。

## GaussianVoxelMap：体素级别的分布表示

`GaussianVoxelMapCPU` 将每个体素压缩为单个高斯分布 $(\mu, \Sigma)$，与传统 GICP 逐点协方差不同——体素内所有点被聚合为一个统计量。

### GaussianVoxel 数据布局

关键源码 `raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp:L13-L52`：

```cpp
struct GaussianVoxel {
    bool finalized;            // true: mean/cov 已最终化; false: 仍在累加
    size_t num_points;         // 体素内总点数
    Eigen::Vector4d mean;      // 均值 (x, y, z, 1)
    Eigen::Matrix4d cov;       // 协方差 (4×4, 最后一行/列为 0)
    double intensity;          // 平均强度

    void add(const Setting& s, const PointCloud& points, size_t i);
    void finalize();           // 从累加值计算 mean/num_points 和 cov
};
```

在 `add()` 阶段（`raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`），体素内点通过 Welford 在线算法累加：

$$\mu_k = \mu_{k-1} + \frac{p_k - \mu_{k-1}}{k}, \quad S_k = S_{k-1} + (p_k - \mu_{k-1})(p_k - \mu_k)^T$$

其中 $S_k$ 是累积二次矩，`finalize()` 时计算 $\Sigma = S_n / n$。

### VGICP 中的体素关联

在 `IntegratedVGICPFactor::update_correspondences()` 中（`raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:L99-L153`），数据关联逻辑为：

```text
for each source point i:
    pt_transformed = delta * source.point[i]
    coord = target_voxelmap.voxel_coord(pt_transformed)  // floor(pt / resolution)
    voxel_id = target_voxelmap.lookup_voxel_index(coord)
    if voxel_id < 0:
        correspondences[i] = nullptr  // 无命中
    else:
        correspondences[i] = &target_voxelmap.lookup_voxel(voxel_id)
        C_combined = voxel.cov + R * source.cov[i] * R^T
        mahalanobis = invert(C_combined[0:3, 0:3])
```

注意 VGICP 的数据关联是**无距离阈值的哈希查找**——只要 source 点落入某个体素（基于体素坐标），即建立对应。这意味着体素分辨率 $r$ 隐式地决定了最大允许对应距离（约为 $\sqrt{3} r$）。

### 马氏距离误差的数学形式

对 source 点 $p_i^A$（带协方差 $C_i^A$）和对应体素 $(\mu_j, C_j)$，残差与代价分别为：

$$r_i = R_{BA} p_i^A + t_{BA} - \mu_j$$

$$e_i = r_i^T \left(C_j + R_{BA} C_i^A R_{BA}^T \right)^{-1} r_i$$

其中 $\left(C_j + R_{BA} C_i^A R_{BA}^T \right)^{-1}$ 缓存为 `mahalanobis_full[i]`，每次 `update_correspondences()` 重新计算。GICP 系因子的雅可比结构与 [[方法-gtsam_points因子封装模式]] 中完全相同，仅使用不同的马氏矩阵。

## 体素重叠率检测

gtsam_points 提供 `overlap()` 函数（`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap.hpp:L42`）计算 source 点云落入 target 体素地图的比例：

$$\text{overlap} = \frac{|\{p \in \text{source} : \text{voxel\_exists}(\text{voxel\_coord}(T \cdot p))\}|}{|\text{source}|}$$

这个指标常用于：
- **关键帧选择**：重叠率低于阈值（如 0.5）时插入新关键帧
- **回环候选验证**：重叠率较高时说明候选回环帧与当前帧有显著几何重叠
- **退化检测**：重叠率异常低提示地图覆盖不足或传感器故障

CPU 版和 GPU 版（`overlap_gpu()`）均提供，GPU 版通过 CUDA kernel 批量并行计算。

## 体素分辨率的工程权衡

| 分辨率 | iVox 效果 | GaussianVoxelMap 效果 |
|--------|-----------|----------------------|
| 0.1m | 接近原始点云，计算量大 | 协方差估计稳定，配准精度高 |
| 0.5m | 强降采样，速度快 | 几何细节丢失，平面区域仍有效 |
| 1.0m | 点数过少，配准退化 | 协方差接近均匀，退化为简单 ICP |

生产环境推荐 iVox 分辨率 0.2-0.5m，GaussianVoxelMap 分辨率 0.4-1.0m，具体取决于 LiDAR 线数和扫描密度。

## Agent 实现提示

### 适用场景

- scan-to-map LiDAR 里程计需要 O(1) 数据关联而非 O(log N) KNN
- 大规模地图（10^5+ 点）下的 GICP/VGICP 配准
- 需要在线增量建图（新扫描持续插入体素地图）
- 适合：Faster-LIO 风格的前端、HG-LIO 的体素管理、ROLO-SLAM 的地图维护
- 不适合：纯 scan-to-scan 配准（用 KdTree 即可）、极稀疏点云（<1000 点/帧）

### 输入输出契约

- **输入**：点云帧 + 体素分辨率 + 体素邻接模式(1/7/19/27) + 最大体素点数 + 最小体素内点间距
- **插入 `insert(points)`**：将一帧点云按体素坐标哈希分配到对应体素，自动处理去重和 LRU 淘汰
- **查询 `knn_search(pt, k, indices, sq_dists, max_dist)`**：先查自身体素，不足 k 个时扩展到邻接体素
- **协方差模式额外输入**：近邻数、最小近邻数、warmup 周期数、特征值阈值比例
- **输出**：索引列表（编码 voxel_id|point_id）、平方距离列表、找到的近邻数

### 实现骨架（伪代码）

```text
// ===== 一、构建 iVox 地图 =====
voxelmap = IncrementalVoxelMap<FlatContainer>(resolution=0.25)
voxelmap.set_neighbor_voxel_mode(7)     // 1=自身, 7=面邻接
voxelmap.set_lru_horizon(100)           // 100 帧未访问 → 淘汰
voxelmap.set_lru_clear_cycle(10)        // 每 10 帧清理一次

// 每帧：
voxelmap.insert(current_frame_points)

// ===== 二、查询近邻 =====
function knn_in_voxel(pt, k=10):
    coord = floor(pt / voxelmap.leaf_size)
    voxel_id = voxelmap.lookup_voxel_index(coord)
    candidates = []
    if voxel_id >= 0:
        for each point p in voxelmap.flat_voxels[voxel_id].container.points:
            candidates.push(p)
    if candidates.size() < k:
        // 扩展到邻接体素
        for each neighbor_coord in voxelmap.offsets:
            nbr_id = voxelmap.lookup_voxel_index(coord + neighbor_coord)
            if nbr_id >= 0:
                for each point p in ...:
                    candidates.push(p)
    // 排序取 top-k
    sort by squared distance, return indices

// ===== 三、构建 GaussianVoxelMap =====
gvm = GaussianVoxelMapCPU(resolution=0.5)
for each frame f in keyframes:
    gvm.insert(f.points)   // 在线聚合每个体素的 mean/cov

// ===== 四、VGICP 体素关联 =====
function voxel_correspondence(source_pt, delta, gvm):
    pt_transformed = delta * source_pt
    coord = gvm.voxel_coord(pt_transformed)
    voxel_id = gvm.lookup_voxel_index(coord)
    if voxel_id < 0: return null
    voxel = gvm.lookup_voxel(voxel_id)
    C_combined = voxel.cov + delta.R * source_pt.cov * delta.R^T
    mahalanobis = invert(C_combined[0:3, 0:3])
    residual = pt_transformed - voxel.mean
    error = residual^T * mahalanobis * residual
    return error, mahalanobis
```

### 关键源码片段

1. **iVox 类型定义** — `raw/codes/gtsam_points/include/gtsam_points/ann/ivox.hpp:L21`：`using iVox = IncrementalVoxelMap<FlatContainer>;`
2. **体素索引编码** — `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp:L84-L86`
3. **FlatContainer 去重插入** — `raw/codes/gtsam_points/include/gtsam_points/ann/flat_container.hpp:L33-L53`
4. **协方差 VALID 位标志** — `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_covariance_container.hpp:L82-L83`
5. **GaussianVoxel 均值/协方差线更新** — `raw/codes/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`（Welford 在线算法）
6. **VGICP 体素关联与马氏矩阵计算** — `raw/codes/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:L114-L153`
7. **体素重叠率计算** — `raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap.hpp:L42-L51`
8. **邻接体素偏移预计算** — `raw/codes/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp:L106`

### 实现注意事项

- **体素分辨率 $\neq$ KNN 搜索半径**：体素分辨率决定数据关联的空间粒度，但 `knn_search()` 的 `max_sq_dist` 参数仍独立控制搜索半径。邻接体素模式（如 mode=7 只查面邻接）相当于限制了最大搜索范围。
- **动态体素的协方差滞后问题**：`IncrementalCovarianceVoxelMap` 的协方差估计需要 warmup（默认 10 帧），在新建体素的前几帧，协方差未收敛（VALID 位为 0），`knn_search()` 默认跳过这些点。需要近邻估计协方差的因子应确保 target 体素有足量 warmup 或回退到 `knn_search_force()`。
- **哈希冲突与误匹配**：XORVector3iHash 的碰撞概率极低（64 位空间），但不同体素坐标理论上可能哈希到同一槽。`IncrementalVoxelMap` 使用 `unordered_map`，自动处理碰撞。
- **LRU 淘汰与回环**：如果回环检测触发了历史区域的大范围优化，之前被 LRU 淘汰的体素需要重新插入。应在回环后适当增大 `lru_horizon` 或暂时禁用淘汰。
- **GaussianVoxelMap 的内存增长**：与 iVox 不同，GaussianVoxelMap 没有 LRU 淘汰机制，随地图增长无限扩展。在长期运行中应配合空间分区、八叉树剪枝或按距离的 map 分片策略。
- **GPU 体素重叠率**：使用 `overlap_gpu()` 要求 source 点云和 target 体素地图已上传至 GPU（`points_gpu`、`covs_gpu` 等）。上传开销可能抵消 batch 计算的收益，仅在批量评估多候选回环帧时才有明显优势。

### 源码检索锚点

- 增量体素地图模板：`raw/codes/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- iVox 别名：`raw/codes/gtsam_points/include/gtsam_points/ann/ivox.hpp`
- FlatContainer：`raw/codes/gtsam_points/include/gtsam_points/ann/flat_container.hpp`
- 增量协方差体素地图：`raw/codes/gtsam_points/include/gtsam_points/ann/incremental_covariance_voxelmap.hpp`
- 增量协方差容器：`raw/codes/gtsam_points/include/gtsam_points/ann/incremental_covariance_container.hpp`
- GaussianVoxelMap CPU 实现：`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- GaussianVoxelMap 抽象接口 + overlap 函数：`raw/codes/gtsam_points/include/gtsam_points/types/gaussian_voxelmap.hpp`
- XOR 哈希：`raw/codes/gtsam_points/include/gtsam_points/util/vector3i_hash.hpp`
- 最近邻搜索接口：`raw/codes/gtsam_points/include/gtsam_points/ann/nearest_neighbor_search.hpp`

## 相关页面

- [[组件-gtsam_points]]
- [[方法-gtsam_points因子封装模式]]
- [[方法-gtsam_points连续时间因子]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[方法-ICP变体族|VGICP]]
- [[方法-体素地图]]
- [[方法-RotVGICP]]
- [[方法-Fast-VGICP]]
- [[方法-GICP配准方法]]
- [[方法-LiDAR地图表示]]
- [[方法-VoxelMap八叉树]]
- [[方法-VoxelHashMap]]
