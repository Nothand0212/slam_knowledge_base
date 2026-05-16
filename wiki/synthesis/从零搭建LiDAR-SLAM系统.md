---
type: synthesis
tags: [LiDAR, SLAM, 教程, ICP, 回环检测, 多传感器融合, 点云, 工程实践]
created: 2026-05-15
updated: 2026-05-15
---

# 从零搭建 LiDAR-SLAM 系统

> 面向 C++ 工程师的 LiDAR-SLAM 实战路线：从点云预处理到 ICP 配准、地图管理、退化检测、回环闭合，最终串成完整管线，并接入 IMU 预积分与 IESKF 多传感器融合。每一步都给出可直接照做的伪代码和参考实现锚点。

---

## 0. 前置知识

在动手写第一行位姿估计代码之前，需要理解几个概念层面：

### 0.1 数学基础

- **李群与李代数**：SE(3) 上的 `Exp`/`Log` 映射是位姿增量表达的核心。理解 `T = Exp(dx) * T_prev` 的含义和 Jacobian 推导。详见 [[数学-流形优化]]。
- **非线性最小二乘**：SLAM 后端本质上是在求解 `min Σ ||r_i||^2_Ω`。理解高斯-牛顿 (GN) 与 LM 的区别、正规方程 `H dx = -b` 的构造过程。
- **因子图**：把问题表达为因子（factor）与变量（variable）的二部图，而非手写优化闭环。详见 [[概念-因子图]] 和 [[概念-位姿图优化]]。

### 0.2 传感器认知

| 传感器 | 数据 | 频率 | 在 SLAM 中的角色 |
|--------|------|------|-----------------|
| LiDAR | 点云 (xyz + intensity + timestamp) | 10-20 Hz | 几何约束（配准、建图） |
| IMU | 角速度 + 加速度 | 100-500 Hz | 高频状态传播、去畸变 |
| GNSS/RTK | LLA 位置 (可含速度/姿态) | 1-10 Hz | 全局参考约束 |

### 0.3 总体管线

```
IMU ──→ 状态传播 ──→ 位姿预测 ──→ ┐
                                    ├→ Scan-to-Map 配准 ──→ 优化位姿 ──→ 更新地图
LiDAR ─→ 去畸变 ──→ 下采样 ──→ ──┘                                      │
       ┌─────────────────────────────────────────────────────────────────┘
       │
       ├→ 关键帧判定 ──→ 回环检测 ──→ 位姿图优化 ──→ 全局一致轨迹+地图
       └→ GNSS 因子注入 (可选)
```

### 0.4 开发环境建议

- **构建**：CMake + C++17，依赖体量按需求递增
- **数学库**：Eigen 3.4+（所有现代系统的基础）
- **优化后端**：[[组件-Ceres-Solver]]（灵活但有手写成本）、[[组件-GTSAM]]（因子图内建、iSAM2 增量）
- **点云**：PCL（完整但重）或直接操作 Eigen 向量（KISS-ICP 路线）
- **评测**：KITTI odometry（城市道路）、MulRan（低线束）、Newer College（手持多场景）。详见 [[概念-基准测试数据集]]。

> 最佳学习策略：先写一个纯 C++/Eigen 的最小可用里程计（只含去畸变 + 下采样 + ICP + 局部地图），跑通 KITTI 序列，然后逐步加退化检测、回环、IMU 融合。切忌一开始就想做完整 LIO/LVIO 系统。

Related: [[LiDAR-SLAM路线图对比]], [[LiDAR数据管线]]

---

## 1. 点云预处理

每帧 LiDAR 原始扫描在被送入配准之前需要三步清洗：去畸变、降采样、离群剔除。这部分代码量不大但直接影响配准质量——脏点云进入 ICP 会让 Jacobian 构造和残差统计全部失真。

### 1.1 运动畸变矫正 (Deskew)

LiDAR 扫描不是一个瞬时快照——一帧 360° 扫描通常持续 50-100 ms。如果平台在扫描期间旋转或平移，直接把所有点当作"同一时刻采集"会导致点云被拉伸或扭转。这就是**运动畸变**。

**从简单到复杂的三种方案**：

| 方案 | 实现 | 精度 @10m/s | IMU 依赖 | 参考 |
|------|------|-------------|----------|------|
| 恒速模型 | 假设帧内匀速，用上一帧位姿增量的线性插值补偿每个点 | 5-15 cm | 无 | KISS-ICP |
| IMU 旋转积分 | 用帧内高频 IMU 角速度积分得到每个点的姿态，补偿旋转畸变 | 5-10 cm | 中 | LIO-SAM |
| IMU 反向传播 | IESKF 传播状态序列，从帧末回退计算每个 LiDAR 点的 6-DoF 位姿补偿 | 1-3 cm | 强 | FAST-LIO2, fusions_slam |

参见 [[方法-IMU deskew]] 了解 LIO-SAM 的欧拉积分实现，[[LiDAR数据管线]] 对比多种去畸变精度。

**伪代码：IMU 反向传播去畸变**（FAST-LIO2 / fusions_slam 风格）

```text
function deskewPoints(points, imu_queue, prev_state):
    // Step 1: 在 IMU 队列中向前传播名义状态
    pose_sequence = []
    state = prev_state
    for imu in imu_queue:
        dt = imu.t - state.t
        state = state + integrateImu(imu.gyro, imu.acc, dt)
        pose_sequence.append((imu.t, state.pose))

    // Step 2: 为每个 LiDAR 点反向查找最近的 IMU 位姿
    deskewed = []
    for point in points.reverse():               // 从帧末向前遍历
        T_at_point = interpolate(pose_sequence, point.timestamp)
        T_end = pose_sequence.last.pose
        T_point2end = T_end.inv() * T_at_point    // 补偿到帧末
        point_deskewed = T_point2end * point.xyz
        deskewed.push(point_deskewed)
    return deskewed
```

**核心输入**：每个 LiDAR 点必须带 `offset_time`（相对帧首点的时间偏移，纳秒或秒）。没有逐点时间戳的雷达无法做精细去畸变。

### 1.2 降采样 (Downsampling)

一帧机械式 LiDAR（64-128 线）产生 30k-60k 点。全量送入 ICP 会：
- 最近邻搜索 O(N log N) 成本爆炸
- 近处高密度区域主导优化，远处稀疏区域被忽略
- 同一物体上的稠密点产生冗余约束，增量信息有限

**常用方法**：

| 方法 | 原理 | 优点 | 适用 |
|------|------|------|------|
| Voxel Grid 降采样 | 按固定边长 `v` 划分 3D 网格，每格保留一个点（中心或最近邻） | 空间均匀、参数少 | 通用首选 |
| 体素哈希去重 | 用哈希桶 + 二级分辨率去重，每体素限点数 `N_max` | 控制局部密度 | KISS-ICP VoxelHashMap |
| 随机降采样 | 随机丢弃 90% 点 | 最快 | 点极稠密 (>100k) 时 |

**伪代码：Voxel Grid 降采样**

```text
function voxelDownsample(points, voxel_size):
    // 空间哈希 + 体素去重
    grid = unordered_map<VoxelKey, Point>
    for p in points:
        key = (floor(p.x / voxel_size), floor(p.y / voxel_size), floor(p.z / voxel_size))
        if key not in grid:
            grid[key] = p
    return grid.values()
```

`voxel_size` 建议：Ouster 128 线 → 0.2-0.3m，Livox Avia → 0.1-0.2m，VLP-16 → 0.3-0.5m。太小则点数不降反增（每体素仍有一堆近距点）；太大则抹掉细杆、边缘等薄结构。

### 1.3 离群点剔除

**距离筛选**：近处（0.1m 内）和远处（传感器最大有效距离外）直接丢弃。近处点可能包含安装件反射，远处点误差显著大于信号。

**统计滤波**：对每个点计算 k 近邻（通常 k=10），若该点到邻域均值的距离超过标准差倍数（常用 1.0-2.0σ），标记为离群。适合去除飞点。

**ROI 裁剪**：对车载场景，裁剪车身上的点（已知外参下的自遮挡）。fusions_slam 中通过 `lidarXYBox` 去除矩形 ROI 内的车身回波。

**自适应权重**：在 ICP 迭代中动态压低远点权重——`weight = 1 - 0.9 * |point_dist| / sqrt(||point||)`。fusions_slam 的 IESKF LiDAR 更新中用此公式等效软 Huber 损失（见 [[方法-多传感器融合管线]]）。

> 预处理质量直接决定配准性能。如果跑下来的 ICP 经常收敛到错误位姿，优先检查是否有未去畸变、下采样参数不当或大量车身自遮挡点未过滤。

Related: [[方法-IMU deskew]], [[方法-体素地图]], [[传感器-IMU预处理]], [[LiDAR数据管线]]

---

## 2. ICP 配准

ICP（Iterative Closest Point）是 LiDAR SLAM 前端的心脏。它的核心循环只有三步：

1. 用当前估计把 source 点云变换到 target 坐标系
2. 为每个 source 点找 target 中的对应关系（最近邻或局部几何）
3. 最小化某种残差，求解位姿增量，更新 T

ICP 的**最大前提**是初值落在收敛盆地内。离开这个前提，GN/LM 迭代会收敛到局部极小或直接发散。因此 ICP 永远是"以好初值为起点的精配准工具"，不是全局匹配器。详见 [[方法-ICP配准方法]]。

### 2.1 选型：要点到点、点到面还是 GICP？

| 变体 | 残差形式 | 收敛速度 | 对初值要求 | 退化抵抗 | 计算量 | 代码复杂度 |
|------|----------|----------|-----------|---------|--------|-----------|
| Point-to-Point | `||Tp - q||` | 慢 | 高 | 弱 | 最低 | 最低 |
| Point-to-Plane | `n^T(Tp - q)` | 快 | 中 | 中 | 低 | 中 |
| GICP | `(Tp - q)^T Σ^{-1}(Tp - q)` | 快 | 中 | 强 | 中 | 中 |
| VGICP | GICP + 体素高斯 | 快 | 低 | 强 | 中 | 高 |

**选型路线图**：

- **极简起点**：Point-to-Point + GM 核 → 参考 KISS-ICP（约 2000 行，零特征提取）
- **结构化室内/城市**：Point-to-Plane → 参考 FAST-LIO, LIO-SAM
- **非结构化/越野/稀疏**：GICP/VGICP → 参考 GenZ-ICP, ROLO-SLAM
- **需要 GPU 加速**：GaussianVoxelMap 马氏距离批量 → 参考 gtsam_points

详见 [[方法-ICP变体族]] 对全族的系统性对比。

### 2.2 Jacobian 推导（点到面 ICP，so(3) 扰动）

给定 source 点 `p_i`（LiDAR 本体系），经当前估计 `T = (R, t)` 变换后在 target 坐标系中找到对应平面 `(n, d)`。点到面的有符号距离为：

$$r_i = n^T(R p_i + t) + d = 0$$

对 SE(3) 的左扰动 `ΔT = Exp(δ)`，其中 `δ = [δθ; δt]`，Jacobian（1×6）为：

$$\frac{\partial r_i}{\partial \delta\theta} = -n^T R [p_i]_\times, \quad \frac{\partial r_i}{\partial \delta t} = n^T$$

其中 `[p]_\times` 是叉乘反对称矩阵。这个推导的完整过程在 [[方法-ICP配准方法#Agent 实现提示]] 中。

### 2.3 鲁棒核：防止外点污染梯度

ICP 在每一次迭代中构造 `H` 和 `b` 时，对每个对应关系应用权重 `w_i`：

$$H = \sum w_i J_i^T J_i, \quad b = -\sum w_i J_i^T r_i$$

权重来源有两种，不要混用：

| 类型 | 作用 | 示例 |
|------|------|------|
| 距离门限 | 裁剪明显错误的对应 | `if dist > max_corr_dist: skip` |
| 鲁棒核 | 压低大残差对应的权重 | Geman-McClure, Cauchy, Huber |

KISS-ICP 使用 Geman-McClure 核：`w = k^4 / (k^2 + r^2)^2`，远距离对应权重接近零。LIO-SAM 用 Huber 核。fusions_slam 用自适应权重 `s = 1 - 0.9 |r| / sqrt(||p_i||)` 实现等效软 Cauchy。

详见 [[方法-Geman-McClure鲁棒核]] 和 [[方法-Ceres 两轮优化 + 粗差剔除]]。

### 2.4 最近邻搜索：VoxelHashMap 的 27 邻域

ICP 每次迭代的最近邻搜索是最大计算瓶颈。KISS-ICP 的 VoxelHashMap 比 KD-tree 更快的原因：

- 哈希 O(1) 定位桶 → 在 `floor(p/v)` 的 3x3x3 = 27 个邻域体素内做局部点遍历
- K 被 `max_points_per_voxel=20` 硬限制，最近邻搜索复杂度 ≈ O(27 × 20) = O(1)

伪代码（参考 `raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L35-L62`）：

```text
function findNearest(query_point, voxel_map):
    v = floor(query_point / voxel_size)
    best_dist = INF
    best_point = null
    for dx in {-1, 0, 1}:
        for dy in {-1, 0, 1}:
            for dz in {-1, 0, 1}:
                bucket = voxel_map[(v.x+dx, v.y+dy, v.z+dz)]
                for p in bucket:
                    d = ||query_point - p||
                    if d < best_dist:
                        best_dist = d
                        best_point = p
    return best_point, best_dist
```

### 2.5 收敛判据

不要只看迭代次数。三层检查：

```text
function hasConverged(dx, prev_cost, cost, iter):
    // 1. 位姿增量无穷小 (|dx| < ε_pose, 默认 1e-6)
    if norm(dx) < eps_pose: return true

    // 2. 代价下降趋于平缓 (|cost - prev_cost| / cost  < ε_cost, 默认 1e-6)
    if abs(cost - prev_cost) / max(cost, 1e-9) < eps_cost: return true

    // 3. 硬迭代上限 (iter >= max_iter, 默认 30)
    //   注意：达到 max_iter 不算"收敛"，应标记为"可能未收敛"
    return false
```

此外，建议在配准后检查：
- 有效对应数 / 总点数 < 0.3 → 几何约束不足
- 最终 mean squared residual > 场景特征噪声水平 → 可能存在错对应或未收敛
- Hessian 条件数 > 1e6 → 存在退化方向（见第 4 章）

### 2.6 实现骨架（点到面 ICP，含 Geman-McClure 核）

```text
function ICP_PointToPlane(source_pts, target_map, T_init, max_iter=30):
    T = T_init
    for iter in 1..max_iter:
        // Step 1: 点变换 + 数据关联
        corrs = []
        for p in source_pts:
            pw = T.rot * p + T.trans
            nearest, dist = target_map.findNearestPlane(pw)
            if dist < max_corr_dist:
                n, d = nearest.normal, nearest.distance
                corrs.append((p, pw, n, d))

        // Step 2: 构造线性系统 H dx = -b
        H = zero(6,6); b = zero(6,1)
        total_corr = len(corrs)
        for (p, pw, n, d) in corrs:
            r = n.dot(pw) + d
            J_rot = -n.dot(skew_matrix(R * p))  // 1×3
            J_trans = n                          // 1×3
            J = [J_rot, J_trans]                 // 1×6

            // Geman-McClure 鲁棒核权重
            w = k^2 / (k^2 + r*r)

            H += w * J^T * J
            b -= w * J^T * r

        // Step 3: 求解位姿增量
        if cnt < 10: return FAIL
        dx = solve(H, b)   // LDLT 或 Cholesky

        // Step 4: 更新
        T = se3_exp(dx) * T

        // Step 5: 收敛判断
        if norm(dx) < 1e-6: break
    return T
```

Related: [[方法-ICP配准方法]], [[方法-ICP变体族]], [[方法-GICP配准方法]], [[方法-Fast-VGICP]], [[方法-Geman-McClure鲁棒核]]

---

## 3. 地图表示

地图决定了三个边界：**数据关联速度**、**内存增长**、**配准精度**。如果每帧 1 万点在全局 1000 万点地图中做 KNN，O(N log N) 的代价在实时系统中无法接受。因此 LiDAR SLAM 系统普遍用**空间分桶**结构粗化搜索空间。详见 [[方法-LiDAR地图表示]]。

### 3.1 四种地图结构的对比

| 维度 | [[方法-LiDAR地图表示\|VoxelHashMap]] | [[方法-VoxelMap八叉树\|VoxelMap 八叉树]] | [[方法-GaussianVoxelMap 体素化配准\|GaussianVoxelMap]] | [[算法-FAST-LIO\|iKD-Tree]] |
|------|------|------|------|------|
| 数据结构 | `tsl::robin_map` 哈希表 | 哈希 + 八叉树递归切割 | 哈希体素高斯分布 | 增量 KD 树 |
| 近邻查询 | O(27×K) 27 邻域遍历 | O(log N) 递归找叶平面 | O(1) 体素索引 | O(log N) 树搜索 |
| 内存效率 | 高(每体素限点数) | 中(节点开销) | 高(每体素压为 1 分布) | 中 |
| 不确定性 | 无 | 6×6 平面协方差 | 联合协方差马氏距离 | 无 |
| 代表框架 | KISS-ICP, GenZ-ICP | FAST-LIVO2 | gtsam_points, ROLO | FAST-LIO2, R3LIVE |
| 代码行数 | ~200 | ~600 | ~1500 (CPU/GPU) | ~800 |

### 3.2 VoxelHashMap：最快的起点

核心设计是 `tsl::robin_map`（开放寻址哈希）+ 27 邻域遍历 + 每体素限点数 20 + 二级去重。

**哈希函数**（三个大质数异或）：

$$h(v_x, v_y, v_z) = v_x \cdot 73856093 \oplus v_y \cdot 19349663 \oplus v_z \cdot 83492791$$

**插入策略**：`map_resolution = sqrt(voxel_size² / max_points_per_voxel)` 作为二级去重尺度。只有当新点与同体素内所有已有点的距离都超过 `map_resolution` 时才插入。这同时完成下采样和去重。

**滑动窗口裁剪**：`RemovePointsFarFromLocation(origin, max_distance)` 按传感器当前位置删除远点，天然限制内存增长。代价是牺牲长期回访一致性（这也是 KISS-ICP 不做回环的原因之一）。

源码锚点：`raw/codes/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp:L35-L62, L83-L119`

### 3.3 VoxelMap 八叉树：点到面约束的工业选择

FAST-LIVO2 采用的二层结构：

- **顶层哈希**：`unordered_map<VOXEL_LOCATION, VoxelOctoTree*>`，质数 `P=116101` 二次散列
- **底层八叉树**：递归切割，叶节点存 `VoxelPlane`（包含平面中心、法向量、6×6 平面协方差、半径、特征值三元组）

**平面度判定**：对叶节点累积的点做 PCA。若最小特征值 `λ_min < planner_threshold_`（默认 0.01），该区域判为平面并停止切割；否则递归分割为 8 个子立方体，直至达到 `max_layer_`。

**平面协方差传播**：从点测量噪声的 3×3 协方差 `var_i` 通过 Jacobian `J_i`（6×3）传播为平面参数的 6×6 协方差：

$$\Sigma_{plane} = \sum_i J_i \cdot var_i \cdot J_i^T$$

这使得 IESKF 更新时不仅知道点到面的距离，还知道这个平面的"可信度"。详见 `raw/codes/FAST-LIVO2/src/voxel_map.cpp:L55-L135`。

### 3.4 GaussianVoxelMap：分布-分布匹配的精髓

将目标点云压缩为体素高斯分布 `(μ, Σ)`，用马氏距离代替欧氏距离。残差模型：

$$r^2 = (T p_i - μ_j)^T \Sigma_{ij}^{-1} (T p_i - μ_j), \quad \Sigma_{ij} = C_j + R C_i R^T$$

其中 `C_j` 是 target 体素协方差，`C_i` 是 source 点协方差（可选）。当 `C_j` 在某个方向上有大方差（如沿平面方向），马氏距离在该方向自动放松——等效于从"点到面"升级为"面到面"约束，但保留了协方差建模的数学优美性。

**核心优势**：单个体素只存一个分布 `(μ, Σ)`（2 个 3D 向量 + 1 个 3×3 矩阵），数据量大降。CPU/GPU 双实现，GPU 版通过 `NonlinearFactorGPU` 批量线性化。详见 [[组件-gtsam_points]]。

### 3.5 选型决策树

```
需要连续 SDF 可微地图？ → 是 → PIN-SLAM Neural Points
                     → 否 ↓
需要点到面残差 + 平面不确定性？ → 是 → VoxelMap 八叉树 (FAST-LIVO2)
                              → 否 ↓
需要 GICP/马氏距离精度？ → 是 → GaussianVoxelMap (gtsam_points)
                         → 否 ↓
需要最简实现 / 教学起点？ → VoxelHashMap (KISS-ICP)
```

**一条关键经验**：先从 VoxelHashMap 起步（代码最小，API 最清晰），跑通前端后再按需升级为 GaussianVoxelMap 或 VoxelMap 八叉树。体素大小建议从 LiDAR 两倍水平角分辨率起调（如 64 线 → 0.3m 起点）。

### 3.6 伪代码：VoxelHashMap 更新 + 最近邻查询

```text
function updateVoxelMap(points_world, origin, voxel_size, max_pts_per_voxel, max_dist):
    map_res = sqrt(voxel_size^2 / max_pts_per_voxel)
    for p in points_world:
        v = (floor(p.x/voxel_size), floor(p.y/voxel_size), floor(p.z/voxel_size))
        bucket = map.get(v)
        if not bucket: map[v] = [p]; continue
        if len(bucket) >= max_pts_per_voxel: continue
        if any(||q - p|| < map_res for q in bucket): continue
        bucket.push(p)

    // 滑动窗口裁剪
    for (v, bucket) in map:
        if ||bucket[0] - origin|| > max_dist:
            map.erase(v)
```

Related: [[方法-LiDAR地图表示]], [[方法-体素地图]], [[方法-VoxelMap八叉树]], [[方法-GaussianVoxelMap 体素化配准]], [[方法-在线平面拟合]], [[组件-gtsam_points]]

---

## 4. 退化检测

ICP 的位姿增量由法方程 `H dx = -b`（`H = J^T J`，6×6 的近似 Hessian）求解。如果某些自由度的 `λ` 很小，`dx` 在该方向的分量主要由噪声决定——这就是**退化**。长廊、单平面地面、开阔地是三大典型退化场景。系统必须识别并降低退化方向的更新权重，而不是在不可观方向输出确定值。详见 [[方法-6-DoF 退化检测]] 和 [[方法-退化检测与修复]]。

### 4.1 特征值判定法（LIO-SAM）

LIO-SAM 在 LM 优化首轮迭代中对 `H = J^T J` 做特征值分解：

```text
λ_1 ≥ λ_2 ≥ ... ≥ λ_6    (从大到小)
```

从最小特征值（索引 5）向大遍历：若 `λ_i < eignThre[i]`（默认全部 100），标记该方向为退化，将特征向量矩阵 V 中对应行置零，得到的投影矩阵 `P = V^{-1} · V'` 用于抑制退化方向增量：

```text
dx_corrected = P * dx_raw    // 退化方向增量为零
```

注意：阈值 100 是经验值，依赖点数、残差量纲和环境几何复杂度。在不同数据集上可能需调整。详见 `raw/codes/LIO-SAM/src/mapOptmization.cpp:L1229-L1258`。

### 4.2 条件数判定法（SuperOdom/SuperLoc）

对协方差 `C = H^{-1}` 的平移/旋转子块分别做特征值分解，条件数倒数衡量病态程度：

$$\kappa^{-1}_{pos} = \sqrt{\lambda_{min}} / \sqrt{\lambda_{max}}, \quad \kappa^{-1}_{rot} = \sqrt{\lambda_{min}} / \sqrt{\lambda_{max}}$$

`κ^{-1} → 0` 意味着该子空间极度病态（如长廊中 z 平移被地面约束，x 平移完全退化）。

SuperLoc 采用**双层退化感知**：第一层用平面特征可观测性直方图（9 维：6 旋转 + 3 平移）输出归一化不确定性 [0,1]；第二层通过 Ceres Covariance API 获取参数块协方差并做特征值分解，输出最大平移/旋转标准差和条件数。

### 4.3 退化响应策略

| 策略 | 做法 | 适用 | 代表 |
|------|------|------|------|
| 投影修正 | 退化方向 `dx=0` | LiDAR 局部优化 | LIO-SAM, ROLO |
| 噪声放大 | 退化时 `correctionNoise2` 放大 ~100 倍 | 因子图后端 | LIO-SAM |
| 先验注入 | 退化 DoF 添 SE3 弱先验 | 多传感器融合 | SuperOdom/SuperLoc |
| 多源补充 | 退化方向由 IMU/GNSS/视觉主导 | 多传感器融合 | 各融合方案 |
| 二值切换 | 退化时切换到 IMU/VIO 预测 | 前端降级 | SuperLoc |

### 4.4 伪代码：LIO-SAM 风格退化投影

```text
function detectAndFixDegeneracy(H, raw_dx):
    eigenvalues, V = eigen_decompose(H)
    V2 = copy(V)
    is_degenerate = false
    eignThre = [100, 100, 100, 100, 100, 100]

    // 从最小特征值向大检查
    for i in (5, 4, 3, 2, 1, 0):
        if eigenvalues[i] < eignThre[i]:
            V2.setRow(i, zeros(1,6))
            is_degenerate = true
        else: break

    P = V.inv() * V2
    dx_corrected = P * raw_dx
    return dx_corrected, is_degenerate
```

关键细节：
- 只在首轮迭代做特征值分解，后续迭代复用投影矩阵
- 从最小特征值连续检查并 break 在第一个正常值——确保只有连续的小特征值方向被抑制
- 硬抑制 `dx=0` 不等于"该方向不更新"——可以配合噪声放大让 IMU 主导

### 4.5 退化检测的工程检查清单

- [ ] 每次配准后检查 Hessian 条件数（>1e5 报警）
- [ ] 长廊/隧道环境必须开启退化检测
- [ ] 不要用全局 `eignThre` 常数——不同数据集需校准
- [ ] 退化时输出每个 DoF 的可信度（`covariance[6]`），供下游使用
- [ ] yaw 方向在平面 LiDAR 中天然退化——SuperLoc 将其信息矩阵直接置零

Related: [[方法-6-DoF 退化检测]], [[方法-退化检测与修复]], [[方法-SuperLoc退化检测实现]], [[架构-滑动窗口优化]]

---

## 5. 回环检测

没有回环的 LiDAR 里程计是开环估计——漂移会随时间无界累积（通常 1-3% 的累计误差）。回环检测的目标是识别"我回到来过的地方"，在后端位姿图中添加长距离约束，将漂移消散到全局轨迹中。详见 [[概念-回环检测方法]] 和 [[方法-LiDAR回环检测管线]]。

### 5.1 两层管线：候选召回 + 几何验证

回环永远不能是单步判定——描述子相似不等于位置相同。标准两层管线：

```
当前关键帧 ──→ 描述子提取 ──→ 历史帧检索 ──→ 候选列表
                                              │
                                              ↓
                                         几何配准验证 ──→ 通过 ──→ 加入位姿图
                                              │
                                              ↓ 失败
                                           丢弃候选
```

### 5.2 LiDAR 描述子：ScanContext

ScanContext 是目前 LiDAR SLAM 最广泛使用的回环描述子。它将点云投影到极坐标网格 (`N_r=40` 环 × `N_s=20` 扇区)，每个 bin 取该区域内点的最大高度或最大强度。

**ScanContext vs ISC (Intensity Scan Context)**：

| 描述子 | 编码 | 适用 | 代表 |
|--------|------|------|------|
| ScanContext | 最大高度 | 结构化城市（建筑物高度模式） | LIO-SAM, FAST-LIO-SAM-SC-QN |
| ISC | 最大强度 | 4D 雷达（强度信噪比 > 位置精度） | 4DRadarSLAM |

**旋转不变性**：通过列循环移位搜索实现。先用 SectorKey（列均值向量）做快速对齐估计，再在局部搜索窗口内遍历偏移，寻最小余弦距离。

**距离度量**（余弦距离）：

$$d(SC_1, SC_2) = 1 - \frac{1}{N_{eff}} \sum_{s \in valid} \frac{c_s^{(1)} \cdot c_s^{(2)}}{||c_s^{(1)}|| \cdot ||c_s^{(2)}||}$$

伪代码（参考 `raw/codes/4DRadarSLAM/src/radar_graph_slam/Scancontext.cpp:L272-L374`）：

```text
function detectLoop(currentSC, historySCs, threshold=0.3):
    // Step 1: RingKey KD-tree 快速检索 top-3
    ring_key = mean_rows(currentSC)
    candidates = kd_tree.knn_search(ring_key, k=3)

    // Step 2: 精细化列对齐
    for idx in candidates:
        sector_key_q, sector_key_h = makeSectorKeys(currentSC, historySC[idx])
        rough_align = phaseCorr(sector_key_q, sector_key_h)

        best_dist = INF
        for shift in [rough_align - radius, rough_align + radius]:
            sc_shifted = circshift(historySC[idx], shift)
            dist = columnwiseCosineDistance(currentSC, sc_shifted)
            if dist < best_dist:
                best_dist = dist; best_shift = shift

        if best_dist < threshold:
            yaw_shift = best_shift * sector_angle
            return idx, yaw_shift

    return null
```

### 5.3 几何验证：ICP / NDT / Quatro

一旦 ScanContext 给出候选，需要用点云配准确认：

| 验证方法 | 原理 | 参数 | 代表 |
|----------|------|------|------|
| ICP fitness | PCL ICP 配准 + `fitnessScore < 6.0` | 单阈值 | 4DRadarSLAM |
| 多分辨率 NDT | 4 级分辨率梯度配准 (10m→1m) + `transProb ≥ 1.0` | 分辨率和分数阈值 | Lightning-LM |
| Quatro + GICP | 粗配准 (Quatro GNC) → 精配准 (Nano-GICP) | 两阶段 | FAST-LIO-SAM-SC-QN |

Lightning-LM 的多分辨率 NDT 特别精巧——4 级分辨率（10.0, 5.0, 2.0, 1.0）从粗到细匹配，类似图像金字塔：

```cpp
for (auto& r : {10.0, 5.0, 2.0, 1.0}) {
    ndt.setResolution(r);
    target = VoxelGrid(submap, r * 0.1);
    source = VoxelGrid(curCloud, r * 0.1);
    ndt.align(output, init_pose);
    init_pose = ndt.getFinalTransformation();
}
```

### 5.4 五层回环验证（4DRadarSLAM 的极致做法）

4DRadarSLAM 为应对 4D 雷达的极端噪声，建立了五层递进过滤管线，值得 LiDAR 系统参考：

| 层 | 验证 | 阈值示例 | 作用 |
|----|------|----------|------|
| 0 | 距离/高度/yaw/椭圆预筛选 | 气压差<3m, yaw<45° | 快速排除不可能 |
| 1 | ISC 匹配 | SC_DIST < 0.3 | 描述子召回 |
| 2 | ICP Fitness | fitnessScore < 6.0 | 几何一致性 |
| 3 | Odometry Check | trans_err/nframes < 0.1m | 单边约束一致性 |
| 4 | Pairwise Consistency | pairwise trans < 0.1m | 多边约束一致性 |

每通过一层，候选可信度上升一级，最终加入因子图的边仅通过全部 5 层的候选。

### 5.5 位姿图优化（回环后端）

通过验证的回环边终于可以注入后端。典型的因子图包含：

```text
function optimizePoseGraph(keyframes, odom_edges, loop_edges):
    graph = new FactorGraph()
    values = new Values()

    // 1. 添加位姿节点
    for kf in keyframes:
        graph.add(PriorFactor(kf.id, kf.T, noise_prior))
        values.insert(kf.id, kf.T_init)

    // 2. 添加里程计边（相邻帧）
    for edge in odom_edges:
        graph.add(BetweenFactor(edge.i, edge.j, edge.T_rel, noise_odom))

    // 3. 添加回环边（含鲁棒核）
    for edge in loop_edges:
        robust = CauchyKernel(5.2/5)  // Lightning-LM 默认
        graph.add(BetweenFactor(edge.i, edge.j, edge.T_lc, noise_loop, robust))

    // 4. 增量优化
    isam2.update(graph, values)
    result = isam2.calculateEstimate()
    return refined_poses = result
```

**回环边管理**：
- 离群回环：Lightning-LM 在优化 20 次后检测卡方误差 `χ² > δ` 的回环边，标记 `level=1` 禁用但不删除
- 回环频率控制：距上一次回环 ≥ `loop_kf_gap_` 帧（默认 20），避免回环过频压稠因子图
- 时间间隔门控：同一轨迹内当前帧与候选帧 ID 间隔 ≥ `closest_id_th_`（默认 50），排除时间上过近的帧

Related: [[方法-LiDAR回环检测管线]], [[概念-回环检测方法]], [[概念-位姿图优化]], [[方法-Intensity Scan Context]], [[组件-DBoW2]]

---

## 6. 完整管线集成

把第 1-5 章的模块串成一条可运行的 LiDAR SLAM 管线。以下是 fusions_slam 风格的伪代码架构。

### 6.1 系统架构

```
┌──────────────────────────────────────────────────┐
│                  ROS Wrapper                      │
│  (sensor_msgs → 内部数据结构, 时间同步, publish)  │
├──────────────────────────────────────────────────┤
│                  Front (调度器)                   │
│  syncMeasureGroupAdd → run() → 调度 propagate + update │
├──────┬──────┬──────┬──────┬─────────┬────────────┤
│ IMU  │LiDAR │ RTK  │Map   │ IESKF   │ Back(PGO)  │
│传播  │预处理│观测  │(ikd) │ (预测+  │ (位姿图)   │
│      │      │      │      │  观测)  │            │
└──────┴──────┴──────┴──────┴─────────┴────────────┘
```

核心设计原则：**数据通信与算法实现分离**——ROS 层负责格式转换和业务调度，算法模块不依赖 ROS。详见 [[方法-多传感器融合管线]]。

### 6.2 主循环伪代码

```text
class LiDARSlamPipeline:
    function init(config):
        ieskf = IESKF(config)                    // 状态估计器
        front = FrontBuffer()                    // 传感器缓冲队列
        map = VoxelHashMap(voxel_size=0.3)       // 局部地图
        loop_detector = ScanContext(Nr=40, Ns=20) // 回环检测
        pgo = PoseGraphOptimizer()               // 位姿图后端

    function run():
        while system_running:
            // Step 0: 收集传感器数据
            collectMessages()   // ROS spinOnce 回调入队

            // Step 1: 时间同步 + 数据对齐 (以 LiDAR 帧为基准)
            msg = syncToLidarFrame(front)
            if not msg: continue

            // Step 2: IMU 状态传播 (每个 IMU 采样间隔)
            for imu in msg.imus:
                ieskf.predict(imu.gyro, imu.acc, imu.dt)

            // Step 3: 点云预处理
            cloud_deskewed = deskewPoints(msg.cloud, msg.imus, ieskf.state)
            cloud_filtered = voxelDownsample(cloud_deskewed, 0.3)
            cloud_filtered = removeOutliers(cloud_filtered)

            // Step 4: Scan-to-Map 配准 (IESKF LiDAR 观测更新)
            T_world_lidar = ieskf.lidarObserve(cloud_filtered, map)

            // Step 5: 更新地图
            cloud_world = transformCloud(cloud_filtered, T_world_lidar)
            map.addPoints(cloud_world)
            map.removeFarPoints(T_world_lidar.trans, max_dist=100)

            // Step 6: 关键帧判定
            if shouldInsertKeyframe(T_world_lidar, map):
                kf = Keyframe(T_world_lidar, cloud_filtered)
                keyframes.append(kf)

                // Step 7: 回环检测
                loop_cand = loop_detector.detect(kf, keyframes)
                if loop_cand:
                    T_lc = geometricVerification(kf.cloud, loop_cand.cloud)
                    if T_lc:
                        pgo.addLoopEdge(kf.id, loop_cand.id, T_lc)

                // Step 8: 位姿图优化
                pgo.addPoseNode(kf.id, T_world_lidar)
                pgo.optimize()
                T_world_lidar = pgo.getPose(kf.id)

            // Step 9: 发布输出
            publish(T_world_lidar, cloud_filtered, map)

    function shouldInsertKeyframe(T_cur, map):
        dist = ||T_cur.trans - last_kf.trans||
        angle = ||T_cur.rot - last_kf.rot||
        overlap = map.computeOverlap(cloud_cur, last_kf.cloud)
        return dist > 0.5 or angle > 0.2 or overlap < 0.7
```

### 6.3 模块依赖关系

| 模块 | 输入 | 输出 | 依赖 |
|------|------|------|------|
| Deskew | 原始点云 + IMU 队列 | 去畸变点云 | IMU 内参 |
| Downsample | 去畸变点云 | 降采样点云 | voxel_size |
| OutlierFilter | 降采样点云 | 清洗后点云 | max_range, k_nn |
| ICP/IESKF | 清洗点云 + 局部地图 | 位姿 T | map, max_iter, corr_dist |
| Map | 世界系点云 + 位姿 | 可查询地图 | voxel_size, max_dist |
| LoopDetector | 当前 KF + 历史 KF 库 | 回环候选 | SC_DIST_THRES |
| PGO | 关键帧 + 回环边 | 全局一致位姿 | 信息矩阵, 鲁棒核 |

### 6.4 增量开发路线

不建议一次写完所有模块。推荐按以下次序增量实现：

1. **Day 1**：纯点云读写 + Voxel Grid 降采样可视化
2. **Day 2**：恒速模型 deskew + Point-to-Point ICP（不建地图，只做 scan-to-scan）
3. **Day 3**：VoxelHashMap 局部地图 + scan-to-map ICP
4. **Day 4**：退化检测（Hessian 特征值分解，投影修正）
5. **Day 5**：关键帧自动插入 + ScanContext 回环检测
6. **Day 6**：GTSAM iSAM2 位姿图优化，端到端跑通 KITTI
7. **Day 7+**：加入 IMU 预积分（GTSAM ImuFactor），升级为 LIO

每一步跑通一个数据集序列，可视化轨迹和地图，确认没有累积漂移恶化或崩溃。

Related: [[方法-多传感器融合管线]], [[LiDAR-SLAM路线图对比]], [[架构-多传感器融合架构]], [[phad_fusion设计总结]]

---

## 7. 进阶：多传感器融合

纯 LiDAR SLAM 在两种情况下会出问题：**激烈运动**（ICP 找不到初值）和**几何退化**（走廊、开阔地无足够约束）。IMU 提供高频状态传播解决第一个问题，GNSS/视觉提供全局参考或互补约束解决第二个。本章介绍如何从纯 LiDAR 里程计升级为 LiDAR-惯性-卫星融合系统。

### 7.1 IMU 预积分：从帧间稀疏积分到因式化约束

IMU 在 100-500 Hz 产生测量。如果在后端每次迭代都重积高频 IMU（每帧数千次积分操作），计算成本不可接受。**预积分**（Forster 2015/2017）的核心思想：将两关键帧间的 IMU 测量预积为一个相对运动增量——当关键帧状态在优化中被更新时，预积分量通过一阶泰勒近似修正偏差影响，无需重复积分。

详见 [[概念-IMU预积分]]。预积分的元素包含：

$$\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij}$$

以及它们对偏置 `b_g, b_a` 的 Jacobian，用于一阶偏置更新。

**GTSAM 预积分用法**（LIO-SAM 参考，`raw/codes/LIO-SAM/src/imuPreintegration.cpp:L350-L383`）：

```text
function integrateIMUBetweenKeyframes(imu_queue, kf_i, kf_j, prev_bias):
    preint = new PreintegratedImuMeasurements(params_imu, prev_bias)

    for imu in imu_queue:
        if imu.time < kf_i.time: skip
        if imu.time > kf_j.time: break
        dt = imu.time - last_imu_time  // 必须用相邻 IMU 时间差！
        preint.integrateMeasurement(imu.acc, imu.gyro, dt)
        last_imu_time = imu.time

    graph.add(ImuFactor(
        pose_key(kf_i), vel_key(kf_i),  // X[i-1], V[i-1]
        pose_key(kf_j), vel_key(kf_j),  // X[i], V[i]
        bias_key(kf_i),                  // B[i-1]
        preint
    ))
    graph.add(BetweenFactor(
        bias_key(kf_i), bias_key(kf_j),
        zero_bias,
        noiseModel = Diagonal * sqrt(preint.deltaTij) * noiseBiasRW
    ))

    // 预测初值
    nav_state = preint.predict(prev_state, prev_bias)
    values.insert(pose_key(kf_j), nav_state.pose)
    values.insert(vel_key(kf_j), nav_state.velocity)
```

**偏置更新协议**：每次优化后，用新的偏置估计调用 `preint.resetIntegrationAndSetBias(new_bias)`。不重置会导致旧预积分量与当前偏置估计不一致。

### 7.2 预积分方法选型

| 方法 | 精度 @ 500°/s | 实现复杂度 | 代表 |
|------|---------------|-----------|------|
| 欧拉离散 | 低 | 最低 | fusions_slam, Lightning-LM |
| 中值积分 (RK2) | 中 | 中 | VINS-Fusion |
| GTSAM 标准预积分 | 中 | 中 | LIO-SAM, Kimera-VIO |
| SE2(3) 精确预积分 | 高 | 高 | OpenMAVIS |

对 LiDAR 场景（10 Hz 帧率），IESKF 的迭代更新能补偿欧拉积分的误差，因此欧拉+Fx 协方差传播是工程可行选择。纯 VIO 系统建议至少使用中值积分或 GTSAM 标准。

### 7.3 IESKF：在滤波器中紧耦合 LiDAR 和 IMU

IESKF（Iterated Error State Kalman Filter）将 IMU 状态传播和 LiDAR 点云配准放在同一个误差状态框架中求解。详见 [[方法-统一IESKF融合]] 和 [[方法-多传感器融合管线]]。

**状态向量**（18 维，fusions_slam）：

$$\mathbf{x} = [\delta\mathbf{θ}, \delta\mathbf{p}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a, \delta\mathbf{g}]^T$$

**IMU 传播**（每个 IMU 采样点）：

```text
function predict(state, imu, dt):
    // 1. 名义状态传播（欧拉法）
    R = R * Exp((imu.gyro - bg) * dt)
    p = p + v * dt
    v = v + (R * (imu.acc - ba) + g) * dt

    // 2. 协方差传播
    Fx = [ I - [ω]×dt,  0,     0,   -Jr*dt,  0,     0;   // δθ
           0,            I,    I*dt,  0,      0,     0;   // δp
           -R*[a]×dt,    0,     I,    0,     -R*dt, I*dt; // δv
           0,            0,     0,     I,      0,     0;   // δbg
           0,            0,     0,     0,      I,     0;   // δba
           0,            0,     0,     0,      0,     I ]  // δg
    P = Fx * P * Fx^T + Fw * Q * Fw^T
```

**LiDAR 观测更新**（IESKF 迭代，fusions_slam）：

```text
function lidarObserve(cloud, map):
    for iter in 1..max_iter:
        // 1. 计算残差 Z 和 Jacobian H
        for each point in cloud:
            pw = R * point + t
            n, d = map.fitPlane(pw, k=5)
            if planeValid(n, pw, threshold=0.1):
                r_i = n^T * pw + d          // 点面距离
                s = 1 - 0.9*|r_i| / sqrt(||point||)  // 自适应权重
                if s > 0.9:
                    H.row(i) = [-n^T*R*skew(point), n^T, zeros(1,12)]
                    Z(i) = r_i * s

        // 2. KF 更新
        K = (H^T * H + (P/lambda)^-1)^-1 * H^T
        dx = -K * Z - (I - K*H) * J_inv * error_state
        state = state ⊕ dx

        // 3. 收敛判断
        if |dx.rot| < 1e-4 and |dx.trans| < 1e-4: break

    // 4. Update-and-Reset
    J_reset = I - 0.5 * skew(dx.rot)
    P = J_reset * P * J_reset^T
    dx = zeros(18)
```

### 7.4 GNSS/RTK 观测接入

RTK 提供 ENU 坐标系下的绝对位置、姿态和速度。三种观测类型直接进入 IESKF（fusions_slam 参考）：

```text
function rtkObserve(state, rtk_pose, rtk_vel, config):
    // 位置观测
    z_p = rtk_pose.trans
    H_p = [zeros(3,3), I(3,3), zeros(3,12)]
    update(state, z_p, H_p, noise_pos)

    // 姿态观测（需要双天线 INS）
    z_r = log(Exp(state.rot)^-1 * Exp(rtk_pose.rot))
    H_r = [I(3,3), zeros(3,15)]
    update(state, z_r, H_r, noise_rot)

    // 速度观测
    z_v = rtk_vel
    H_v = [zeros(3,6), I(3,3), zeros(3,9)]
    update(state, z_v, H_v, noise_vel)
```

**策略**：RTK 姿态需要双天线 INS 提供，单点定位系统只能用位置观测（`rtkType=1`）。错误的姿态观测比不观测更有害——会把 IESKF 的航向估计拉偏。

### 7.5 多传感器融合架构选型

| 架构 | 耦合度 | 统计一致性 | 工程复杂度 | 代表 |
|------|--------|-----------|-----------|------|
| 松耦合因子图 (LIO-SAM 式) | 结果层 | 弱 | 低 | LIO-SAM, LVI-SAM |
| 紧耦合 IESKF (fusions_slam 式) | 约束层 | 强 | 中 | fusions_slam, FAST-LIO |
| 统一 IESKF (FAST-LIVO2 式) | 数据层 | 最强 | 高 | FAST-LIVO2, R3LIVE |

**建议路线**：松耦合因子图作为快速入门 → 紧耦合 IESKF 作为产品级方案。统一 IESKF（多传感器共享同协方差）统计最优，但任意传感器故障会污染全局状态，需要严格的在线故障检测和退化处理。

详见 [[架构-多传感器融合架构]] 和 [[优化后端选型指南]]。

### 7.6 IMU 传播 + RTK 交替插入的时序管理

多传感器融合最关键也是最容易出错的模块是**时间同步和时序管理**。fusions_slam 的做法值得学习：

```text
function propagateWithRTK(sorted_msgs, ieskf):
    // mapDatas 按纳秒时间戳排序：IMU1 → RTK1 → IMU2 → RTK2 → IMU3 → ...
    for i in 0..len(sorted_msgs)-1:
        if sorted_msgs[i].type == IMU:
            // 在 IMU 和下一个 IMU 之间可能插入 RTK 观测
            while i+1 < len and sorted_msgs[i+1].type == RTK:
                rtk = sorted_msgs[i+1]
                ieskf.positionObserve(rtk.pos, noise_pos)
                ieskf.rotationObserve(rtk.rot, noise_rot)
                i += 1
            // IMU 传播
            next_imu = sorted_msgs[i+1]
            ieskf.predict(next_imu)
```

要求：所有传感器使用同一时钟源（硬件同步），IMU 和 LiDAR 时间戳以纳秒精度对齐。

Related: [[概念-IMU预积分]], [[方法-统一IESKF融合]], [[方法-多传感器融合管线]], [[架构-多传感器融合架构]], [[方法-IESKF滤波器]], [[组件-GTSAM]]

---

## 附录：参考实现速查表

### A.1 按学习阶段推荐

| 阶段 | 参考系统 | 代码量 | 核心学习点 |
|------|----------|--------|-----------|
| 入门 | [[算法-KISS-ICP]] | ~2000 行 | 纯 ICP + VoxelHashMap + 恒速 deskew |
| 基础 LIO | [[算法-FAST-LIO]] | ~5000 行 | IESKF 紧耦合 + iKD-Tree + 反向传播 |
| 因子图 SLAM | [[算法-LIO-SAM]] | ~8000 行 | GTSAM iSAM2 + IMU 预积分 + GPS 因子 + 退化 |
| 回环闭环 | [[算法-FAST-LIO-SAM-SC-QN]] | 叠加 | ScanContext + Quatro + 四阶段验证 |
| 多传感器融合 | [[方法-多传感器融合管线\|fusions_slam]] | ~15000 行 | 纳秒同步 + RTK 紧耦合 + 自适应权重 |
| 工业级建图 | [[算法-Cartographer]] | ~30000 行 | 子图系统 + BnB 回环 + 概率网格 |
| LVIO | [[算法-FAST-LIVO2]] | ~20000 行 | 统一 IESKF + 直接法光度 + 曝光估计 |

### A.2 按传感器组合

| 传感器组合 | 首推系统 | 备选 |
|-----------|----------|------|
| LiDAR only | KISS-ICP | GenZ-ICP, CT-ICP |
| LiDAR + IMU (LIO) | FAST-LIO2, fusions_slam | LIO-SAM |
| LiDAR + IMU + GPS | fusions_slam | LIO-SAM + GPSFactor |
| LiDAR + IMU + 回环 | FAST-LIO-SAM-SC-QN | LIO-SAM |
| LiDAR + Camera + IMU | FAST-LIVO2 | LVI-SAM, R3LIVE |
| 4D Radar + IMU | 4DRadarSLAM | — |
| 多会话/长期建图 | Cartographer | lt-mapper |

### A.3 核心开源项目入口

| 项目 | 仓库 / 路径 | 关键文件 |
|------|------------|----------|
| KISS-ICP | `raw/codes/kiss-icp/cpp/kiss_icp/core/` | VoxelHashMap.cpp, Registration.cpp |
| FAST-LIO2 | `raw/codes/FAST-LIO2/` | IESKF.hpp, ikdtree.hpp |
| LIO-SAM | `raw/codes/LIO-SAM/src/` | imuPreintegration.cpp, mapOptmization.cpp |
| FAST-LIO-SAM-SC-QN | `raw/codes/FAST-LIO-SAM-SC-QN/` | SCmanager.cpp, loop_closure.cpp |
| fusions_slam | `raw/codes/fusions_slam/src/fusion_slam/modules/` | ieskf.cpp, propagate.cpp, front.cpp |
| FAST-LIVO2 | `raw/codes/FAST-LIVO2/src/` | LIVMapper.cpp, vio.cpp, voxel_map.cpp |
| gtsam_points | `raw/codes/gtsam_points/` | gaussian_voxelmap_cpu.cpp/hpp, registration.cpp |
| 4DRadarSLAM | `raw/codes/4DRadarSLAM/src/radar_graph_slam/` | Scancontext.cpp, loop_detector.cpp |
| Lightning-LM | `raw/codes/lightning-lm/src/core/` | loop_closing.cpp/cc, registration.cpp |

### A.4 关键参数速查

| 参数 | 含义 | 建议起点 | 调参方向 |
|------|------|----------|----------|
| `voxel_size` | 体素边长 (m) | 0.3 (64 线), 0.5 (16 线) | 稠密 LiDAR ↓, 稀疏 ↑ |
| `max_corr_dist` | ICP 最大对应距离 (m) | 2.0-5.0 | 大场景 ↑, 动态环境 ↓ |
| `max_points_per_voxel` | 每体素最多点数 | 20 | 稠密点云 ↑, 稀疏 ↓ |
| `max_iter_icp` | ICP 最大迭代次数 | 30 | 姿态变化大 ↑, 稳定 ↓ |
| `convergence_thresh` | ICP 增量收敛阈值 | 1e-6 | 调小可获更高精度, 但更慢 |
| `SC_DIST_THRES` | ScanContext 距离阈值 | 0.3-0.5 | 召回不够 ↓, 误匹配多 ↑ |
| `loop_kf_gap` | 最小回环间隔(帧) | 20 | 大场景 ↓, 小场景 ↑ |
| `eignThre[6]` | Hessian 退化阈值 | 100 | 退化漏报 ↓, 误报 ↑ |
| `planner_threshold` | 平面度判定 | 0.01 | 复杂几何 ↓, 极平坦 ↑ |
| `GM_kernel_scale` | GM 核尺度 | 1.0 | 外点多 ↓, 结构化好 ↑ |

### A.5 常见错误排查

| 症状 | 可能原因 | 检查点 |
|------|----------|--------|
| ICP 不收敛 / 发散 | 初值差、未去畸变 | 检查 dT 范数、对应数 |
| 轨迹逐渐漂移 | 纯里程计无回环 | 回环检测是否生效 |
| 长廊中位姿跳动 | 退化方向被更新 | Hessian 特征值分布 |
| 地图越来越"厚" | 体素过大或未裁剪 | voxel_size, max_dist |
| RTK 接入后轨迹抖动 | RTK 姿态不准确 | 切换到 position-only 模式 |
| 回环一直不触发 | SC 距离阈值过严 | SC_DIST_THRES, 检查场景特征 |
| 因子图内存爆炸 | 未设 max_age 或重初始化 | ISAM2 图大小, 关键帧频率 |
| 多传感器时间对不齐 | 时钟源不一致 | 检查所有传感器是否硬件同步 |

Related: [[LiDAR-SLAM路线图对比]], [[VIO方案全景对比]], [[优化后端选型指南]], [[phad_fusion设计总结]], [[GTSAM 4.3a1 使用指南]]
