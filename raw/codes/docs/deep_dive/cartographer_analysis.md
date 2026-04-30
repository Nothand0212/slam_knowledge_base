# Cartographer 深度源码分析

> 仓库: https://github.com/cartographer-project/cartographer | 7842 stars
> Google 开源 2D/3D LiDAR SLAM 系统
> 分析时间: 2026-04-29

---

## 1. 工业级架构设计

### 1.1 总体架构

Cartographer 采用经典的**两层 SLAM 架构**：Local SLAM（前端）+ Global SLAM（后端），通过精心设计的线程模型和数据结构确保实时性。

```
┌───────────────────────────────────────────────────┐
│                  MapBuilder                        │
│  ┌──────────────────┐  ┌───────────────────────┐  │
│  │  SensorCollator   │  │      ThreadPool        │  │
│  └──────┬───────────┘  └───────────┬───────────┘  │
│         │                          │              │
│  ┌──────▼──────────────────────────▼───────────┐  │
│  │         TrajectoryBuilders (N条轨迹)          │  │
│  │  ┌──────────────────┐  ┌──────────────────┐  │  │
│  │  │ LocalTrajBuilder  │  │ GlobalTrajBuilder │  │  │
│  │  │  (Local SLAM)    │  │  (Global SLAM)    │  │  │
│  │  └────────┬─────────┘  └────────┬─────────┘  │  │
│  └───────────┼──────────────────────┼────────────┘  │
│              │                      │               │
│  ┌───────────▼──────────────────────▼────────────┐  │
│  │                PoseGraph                       │  │
│  │  ┌──────────────────┐  ┌──────────────────┐   │  │
│  │  │ConstraintBuilder  │  │OptimizationProblem│  │  │
│  │  └──────────────────┘  └──────────────────┘   │  │
│  └───────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

MapBuilder 作为顶层协调器：

```cpp
// cartographer/mapping/map_builder.h:33-96
class MapBuilder : public MapBuilderInterface {
    const proto::MapBuilderOptions options_;
    common::ThreadPool thread_pool_;                          // 后台线程池
    std::unique_ptr<PoseGraph> pose_graph_;                   // 位姿图
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;  // 传感器数据整形器
    std::vector<std::unique_ptr<...>> trajectory_builders_;   // 多条轨迹
};
```

### 1.2 多轨迹支持

Cartographer 原生支持**多条独立轨迹**的并行和后续融合：

```cpp
// cartographer/mapping/pose_graph.h:113
virtual std::vector<std::vector<int>> GetConnectedTrajectories() const = 0;
```

轨迹间可以通过设置初始相对位姿进行关联：

```cpp
// cartographer/mapping/pose_graph.h:133-137
virtual void SetInitialTrajectoryPose(int from_trajectory_id,
    int to_trajectory_id, const transform::Rigid3d& pose, const common::Time time) = 0;
```

### 1.3 Lua 配置系统

Cartographer 使用 Lua 作为配置语言，所有参数均可通过 Lua 文件进行精细控制：

```
configuration_files/
├── map_builder.lua          # 全局 MapBuilder 配置
├── trajectory_builder.lua   # 轨迹构建器全局配置
├── trajectory_builder_2d.lua # 2D 轨迹构建器详细配置
├── trajectory_builder_3d.lua # 3D 轨迹构建器详细配置
├── pose_graph.lua           # 位姿图配置
└── map_builder_server.lua   # 服务端配置
```

典型配置结构（以 2D 为例）：

```lua
-- configuration_files/trajectory_builder_2d.lua:15-115
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0., max_range = 30.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,        -- 体素滤波

  ceres_scan_matcher = {
    occupied_space_weight = 1.,      -- 占据空间权重
    translation_weight = 10.,        -- 平移权重
    rotation_weight = 40.,           -- 旋转权重
  },

  motion_filter = {
    max_distance_meters = 0.2,       -- 运动滤波
    max_angle_radians = math.rad(1.),
  },

  submaps = {
    num_range_data = 90,             -- 每个子地图的扫描帧数
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,             -- 5cm 分辨率
    },
  },
}
```

位姿图配置：

```lua
-- configuration_files/pose_graph.lua:15-95
POSE_GRAPH = {
  optimize_every_n_nodes = 90,       -- 每 90 节点优化一次

  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,   -- 回环最大距离
    fast_correlative_scan_matcher = {
      branch_and_bound_depth = 7,    -- BnB 深度
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
    },
  },

  optimization_problem = {
    huber_scale = 1e1,               -- Ceres 优化参数
    ceres_solver_options = {
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
}
```

---

## 2. 2D/3D LiDAR SLAM 管道

### 2.1 Local SLAM: Scan-to-Submap 匹配

Local SLAM 的核心是在子地图内进行扫描匹配。`LocalTrajectoryBuilder2D` 负责这一过程：

```cpp
// cartographer/mapping/internal/2d/local_trajectory_builder_2d.h
// 流程: SensorData → MotionFilter → ScanMatching → SubmapInsertion
```

**两步匹配策略**：

1. **RealTimeCorrelativeScanMatcher** (实时相关扫描匹配)：在搜索窗口内暴力搜索最优位姿
2. **CeresScanMatcher** (Ceres 扫描匹配)：用 Ceres 非线性优化精化位姿

搜索窗口配置：

```lua
real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,           -- ±10cm 平移搜索
    angular_search_window = math.rad(20.), -- ±20° 旋转搜索
}
```

代价函数将点投影到概率网格中评估：

```cpp
// cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h
// 使用双三次插值获取子像素精度的占据概率
```

运动滤波器在数据进入 Local SLAM 前剔除静止帧：

```cpp
// cartographer/mapping/internal/motion_filter.h
// 只有位移超过 0.2m 或旋转超过 1° 的帧才被处理
```

### 2.2 Global SLAM: 回环检测与 Pose Graph 优化

Global SLAM 通过 `PoseGraph2D` 实现：

```cpp
// cartographer/mapping/internal/2d/pose_graph_2d.cc:52-67
PoseGraph2D::PoseGraph2D(
    const proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options(), thread_pool),
      thread_pool_(thread_pool) {}
```

工作队列机制：所有后台计算通过 `WorkQueue` 管理：

```cpp
// cartographer/mapping/internal/work_queue.h
// 确保约束构建、优化等操作串行化执行
```

节点添加流程：

```cpp
// cartographer/mapping/internal/2d/pose_graph_2d.cc:126-150
NodeId PoseGraph2D::AppendNode(...) {
    const NodeId node_id = data_.trajectory_nodes.Append(
        trajectory_id, TrajectoryNode{constant_data, optimized_pose});

    // 新的子地图出现时自动注册
    if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
        ...) {
        const SubmapId submap_id = data_.submap_data.Append(...);
        data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    }
}
```

### 2.3 优化触发机制

```lua
POSE_GRAPH = {
  optimize_every_n_nodes = 90,  -- 每 90 个节点触发一次全局优化
}
```

优化问题包含多种约束：

```cpp
// cartographer/mapping/internal/optimization/optimization_problem_2d.h:54-56
class OptimizationProblem2D :
    public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D, transform::Rigid2d>
```

约束权重配置：

```lua
optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.1e2,
    rotation_weight = 1.6e4,
    local_slam_pose_translation_weight = 1e5,    -- 局部 SLAM 结果作为硬约束
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,           -- 里程计约束
    odometry_rotation_weight = 1e5,
}
```

---

## 3. 子地图管理 (Submap System)

### 3.1 子地图生命周期

Cartographer 的子地图系统是其架构的灵魂。每个子地图有明确的生命周期：

```cpp
// cartographer/mapping/submaps.h:59-91
class Submap {
    transform::Rigid3d local_pose_;       // 子地图本地坐标系位姿
    int num_range_data_ = 0;              // 已插入的扫描帧数
    bool insertion_finished_ = false;     // 是否已完成插入
};
```

### 3.2 双子地图策略 (Active Submaps)

始终保持两个活跃子地图：

```cpp
// cartographer/mapping/2d/submap_2d.h:74-78
// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
```

活跃子地图管理：

```cpp
// cartographer/mapping/2d/submap_2d.h:79-102
class ActiveSubmaps2D {
    std::vector<std::shared_ptr<Submap2D>> submaps_;  // 总是 2 个
    std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;

    void FinishSubmap();    // 旧子地图冻结
    void AddSubmap(...);    // 新子地图创建
};
```

子地图大小由配置决定：

```lua
submaps = {
    num_range_data = 90,     -- 每个子地图 90 帧
    grid_options_2d = {
        resolution = 0.05,   -- 5cm 分辨率
    },
}
```

### 3.3 概率网格 (Probability Grid)

2D 子地图使用概率占据网格：

```cpp
// cartographer/mapping/2d/probability_grid.h:31-66
class ProbabilityGrid : public Grid2D {
    // 设置初始概率
    void SetProbability(const Eigen::Array2i& cell_index, const float probability);

    // 更新概率（查找表加速）
    bool ApplyLookupTable(const Eigen::Array2i& cell_index,
        const std::vector<uint16>& table);

    float GetProbability(const Eigen::Array2i& cell_index) const;
};
```

### 3.4 概率更新模型

使用 log-odds 表示进行概率更新：

```cpp
// cartographer/mapping/submaps.h:37-39
inline float Logit(float probability) {
    return std::log(probability / (1.f - probability));
}
```

配置中的命中/未命中概率：

```lua
probability_grid_range_data_inserter = {
    insert_free_space = true,      -- 更新射线经过的 free space
    hit_probability = 0.55,        -- 被击中的栅格概率
    miss_probability = 0.49,       -- 被穿透的栅格概率
}
```

概率转换为整型存储（0-32767 表示未知到确定）：

```cpp
// cartographer/mapping/probability_values.h:31-44
inline uint16 BoundedFloatToValue(const float float_value,
    const float lower_bound, const float upper_bound) {
    const int value = common::RoundToInt(
        (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound)
        * (32766.f / (upper_bound - lower_bound))) + 1;
    return value;
}
```

### 3.5 3D 混合网格 (Hybrid Grid)

3D 场景使用混合网格（高低分辨率）：

```cpp
// cartographer/mapping/3d/hybrid_grid.h
// 高分辨率栅格用于精确定位
// 低分辨率栅格用于快速搜索和回环
```

---

## 4. 回环检测 (Loop Closure)

### 4.1 Branch-and-Bound 扫描匹配

Cartographer 的回环检测核心是基于 **Branch-and-Bound** 的相关扫描匹配：

```cpp
// cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h:25-26
// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
```

**预计算网格栈** — 多分辨率金字塔加速搜索：

```cpp
// cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h:95-109
class PrecomputationGridStack2D {
    // 每一层存储该分辨率下的滑动窗口最大概率
    std::vector<PrecomputationGrid2D> precomputation_grids_;

    int max_depth() const { return precomputation_grids_.size() - 1; }
};
```

**BnB 搜索流程**：

```cpp
// cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h:127-165
class FastCorrelativeScanMatcher2D {
    // 局部窗口匹配
    bool Match(const transform::Rigid2d& initial_pose_estimate,
        const sensor::PointCloud& point_cloud, float min_score,
        float* score, transform::Rigid2d* pose_estimate) const;

    // 全子地图匹配（回环检测用）
    bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
        float* score, transform::Rigid2d* pose_estimate) const;

    // 核心 BnB
    Candidate2D BranchAndBound(const std::vector<DiscreteScan2D>& discrete_scans,
        const SearchParameters& search_params,
        const std::vector<Candidate2D>& candidates,
        int candidate_depth, float min_score) const;
};
```

### 4.2 约束构建器 (ConstraintBuilder)

`ConstraintBuilder2D` 异步构建 Pose Graph 约束：

```cpp
// cartographer/mapping/internal/constraints/constraint_builder_2d.h:60-170
class ConstraintBuilder2D {
    // 子地图匹配器缓存
    struct SubmapScanMatcher {
        const Grid2D* grid;  // 子地图网格（不变）
        std::unique_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher;
        std::weak_ptr<common::Task> creation_task_handle;
    };

    // 尝试添加约束（节点→子地图）
    void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
        const NodeId& node_id, const TrajectoryNode::Data* constant_data,
        const transform::Rigid2d& initial_relative_pose);

    // 尝试添加全局约束（全子地图回环检测）
    void MaybeAddGlobalConstraint(const SubmapId& submap_id, const Submap2D* submap,
        const NodeId& node_id, const TrajectoryNode::Data* constant_data);
};
```

**异步计算架构**：约束构建在 ThreadPool 中并行执行，每个子地图的匹配器（预计算网格）独立构建和缓存。

### 4.3 约束评分与过滤

```lua
constraint_builder = {
    min_score = 0.55,                    -- 回环匹配最低分数
    global_localization_min_score = 0.6, -- 全局重定位最低分数
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
}
```

### 4.4 Pose Graph 优化 (Ceres-based)

优化问题以 Ceres Solver 为核心求解器：

```lua
ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 50,
    num_threads = 7,
}
```

优化变量包括每个节点的全局位姿和每个子地图的全局位姿。约束类型：
- **子地图内约束**: 局部 SLAM 的 scan-to-submap 匹配结果
- **回环约束**: BnB 匹配找到的跨子地图匹配
- **里程计约束**: 轮式/视觉里程计（可选）
- **IMU 约束**: 加速度和旋转约束
- **固定帧约束**: GPS 等全局定位

---

## 5. 传感器融合与位姿外推

### 5.1 传感器数据整形器 (SensorCollator)

`SensorCollator` 接收多种传感器数据并按时间排序：

```cpp
// cartographer/sensor/collator_interface.h
// 将异步到达的多传感器数据按时间顺序排队输出
```

### 5.2 位姿外推器

Local SLAM 需要初始位姿估计，由位姿外推器提供：

```lua
pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
        imu_gravity_time_constant = 10.,
        pose_queue_duration = 0.001,
    },
}
```

外推器支持：
- **匀速模型**: 基于历史位姿的线性外推
- **IMU 模型**: 融合惯性数据的高频外推

### 5.3 IMU 追踪器

```cpp
// cartographer/mapping/imu_tracker.h
// 使用 IMU 加速度计和陀螺仪估计重力和传感器朝向
```

---

## 6. 工程经验总结

### 6.1 线程模型

Cartographer 的线程模型精妙之处在于**分离实时性和计算密集性**：

- **Main 线程**: ROS 回调接收传感器数据
- **Local SLAM 线程**: 低延迟的 scan-to-submap 匹配，提供实时位姿
- **ThreadPool**: 后台执行约束构建、子地图预计算、全局优化

这种设计确保回环检测和全局优化不会阻塞实时定位。

### 6.2 内存管理

- **查找表预计算**: 概率更新查找表在初始化时预计算，O(1) 查找
- **子地图冻结后不变**: 冻结后子地图变为不可变的 `const` 共享指针，无需锁保护
- **压缩存储**: `CompressedPointCloud` 压缩存储每个节点的点云数据用于回环

### 6.3 序列化与状态管理

```cpp
// cartographer/mapping/map_builder.h:55-63
void SerializeState(bool include_unfinished_submaps,
    io::ProtoStreamWriterInterface *writer);

std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader, bool load_frozen_state);
```

支持完整的状态序列化和恢复，包括：
- 所有位姿图节点
- 所有子地图（含网格数据）
- 所有约束
- 传感器数据（IMU 等）

### 6.4 可观测性与度量

```cpp
// cartographer/mapping/internal/2d/pose_graph_2d.cc:44-50
static auto* kWorkQueueDelayMetric = metrics::Gauge::Null();
static auto* kActiveSubmapsMetric = metrics::Gauge::Null();
static auto* kFrozenSubmapsMetric = metrics::Gauge::Null();
```

内置 metrics 系统用于监控：
- 工作队列延迟和深度
- 约束数量（同/不同轨迹）
- 活跃/冻结/已删除子地图数量

### 6.5 配置驱动设计

所有模块通过 Protobuf + Lua 配置，参数可热加载，无需重新编译。这种设计使得同一套代码可以适配从室内扫地机器人到室外自动驾驶的多种场景。

---

## 7. 优缺点与对 SLAM 算法的意义

### 优势
1. **真正的工业级实现**: Google 大规模工程实践的产物，代码质量、文档、测试覆盖、可维护性都是一流水平
2. **双 SLAM 架构**: Local + Global 的清晰分离，兼顾实时性和全局一致性
3. **子地图系统**: 子地图冻结后不可变的设计巧妙解决了多线程安全问题，支持高效的异步回环检测
4. **Branch-and-Bound 回环检测**: 通过多分辨率预计算网格栈和 BnB，在 O(n log n) 复杂度内完成全局搜索，远优于暴力搜索
5. **多轨迹支持**: 原生支持多条轨迹的融合与优化，适用于多机器人协作
6. **序列化与恢复**: 完整的状态持久化，支持增量建图和离线优化

### 劣势
1. **计算资源需求高**: 尤其 3D 模式下的混合网格和 BnB 搜索，需要较强的 CPU
2. **参数调优复杂**: Lua 配置虽然灵活，但参数众多，针对新场景调优成本高
3. **无深度学习组件**: 扫描匹配仍基于传统相关匹配和 Ceres 优化，在退化环境中缺乏语义理解
4. **LiDAR 专用**: 主要针对 LiDAR 传感器设计，视觉支持有限
5. **项目维护状态**: Google 开源后社区活跃度有所下降，ROS2 支持不完整

### 对 SLAM 算法的意义
- **架构教科书**: Cartographer 的前端/后端分离、子地图管理、工作队列、异步优化等设计，已成为现代 LiDAR SLAM 系统设计的参考标准
- **BnB 回环检测**: 将 Branch-and-Bound 引入 SLAM 回环检测领域，启发了大量后续工作
- **概率网格模型**: log-odds 表示 + 查找表加速 + 双线性/三次插值代价函数，展示了如何将数学优雅地工程化
- **配置驱动哲学**: 证明了一套完善的配置系统对工业级 SLAM 应用的重要性，启发后续系统（如 FAST-LIO、LIOSAM 等）
- **多传感器融合框架**: 为 IMU、odometry、GPS 等传感器提供了标准化的融合接口

---

## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| LiDAR 2D/3D | 可变 (5-40 Hz) | `sensor_msgs::LaserScan` / `sensor_msgs::PointCloud2` / `sensor_msgs::MultiEchoLaserScan` | `collator_interface.h` | SensorCollator |
| IMU (9-axis, 可选) | ≥100 Hz | `sensor_msgs::Imu` | `collator_interface.h` | SensorCollator → ImuTracker |
| Odometry (可选) | 可变 | `nav_msgs::Odometry` | `collator_interface.h` | SensorCollator |

### 8.2 数据整形（SensorCollator）

**多传感器输入排序**（`sensor/collator_interface.h`）：
所有异步到达的传感器数据通过 `SensorCollator` 按时间顺序排队输出。Collator 接收多种传感器类型（`RangeData`, `ImuData`, `OdometryData`, `FixedFramePoseData`），统一时间戳排序后送入 `TrajectoryBuilder`。

**Motion Filter**（`mapping/internal/motion_filter.h`）：
在数据进入 Local SLAM 前剔除静止帧：
- 仅当 **位移 > 0.2m** 或 **旋转 > 1°** 的帧才被处理
- 配置：`max_distance_meters = 0.2`, `max_angle_radians = 1°`

### 8.3 2D LiDAR 管线

#### 8.3.1 原始数据

- **2D 扫描**：`sensor_msgs::LaserScan`（包含 `ranges[]`, `angle_min`, `angle_increment`）
- **3D 扫描**：`sensor_msgs::PointCloud2`（XYZ）
- 配置范围过滤：`min_range = 0.0`, `max_range = 30.0`

#### 8.3.2 预处理

**体素滤波**（`trajectory_builder_2d.lua`）：
```
voxel_filter_size = 0.025   // 2.5cm 体素降采样
```

**自适应体素滤波**（`voxel_filter.h`）：
- 根据距离自适应调整体素大小
- 远距离点使用更大体素（减少点数）

#### 8.3.3 局部 SLAM：两步扫描匹配

**步骤 1：实时相关扫描匹配**（`scan_matching/real_time_correlative_scan_matcher_2d.h`）：
- 在搜索窗口内暴力搜索最优位姿
- 搜索窗口：`linear_search_window = ±0.1m`, `angular_search_window = ±20°`
- **代价函数**：点投影到概率网格中评分
- 使用双三次插值（bicubic interpolation）获取子像素精度的占据概率

**步骤 2：Ceres 扫描匹配精化**（`scan_matching/ceres_scan_matcher_2d.h`）：
- 以第一步结果作为初始猜测
- 代价函数权重：
  ```
  occupied_space_weight = 1.0     // 占据空间代价权重
  translation_weight = 10.0       // 平移约束权重
  rotation_weight = 40.0          // 旋转约束权重
  ```
- Ceres 求解器：非线性最小二乘优化

#### 8.3.4 子地图构建（Submap）

**概率占据网格**（`mapping/2d/probability_grid.h`）：
```
子地图参数：
  num_range_data = 90             // 每个子地图 90 帧扫描
  resolution = 0.05               // 5cm 分辨率
```
每个子地图 90 帧扫描后冻结，变为不可变对象。

**概率更新模型**（`mapping/submaps.h:37-39`）：
使用 log-odds 表示：
```
odds(p) = p / (1-p)
logit(p) = ln(odds(p))
```

**命中/未命中概率**（`probability_grid_range_data_inserter`）：
```
hit_probability = 0.55    // 被击中栅格的目标概率
miss_probability = 0.49    // 被穿透栅格的目标概率
insert_free_space = true  // 更新射线经过的 free space
```
概率值映射到 16-bit 整型存储（0-32767，[0.1, 0.9] → [1, 32766]）。

**查找表加速**：`ComputeLookupTableToApplyOdds()` 预计算查找表，概率更新 O(1)。

#### 8.3.5 活跃子地图管理（Active Submaps）

`mapping/2d/submap_2d.h:79-102` `ActiveSubmaps2D`：
- 始终维护 **2 个活跃子地图**：旧 submap 用于匹配，新 submap 正在构建
- 旧 submap 达到 90 帧后冻结（`FinishSubmap()`），新 submap 创建（`AddSubmap()`）
- 所有扫描同时插入两个活跃子地图

### 8.4 全局 SLAM：回环检测与 PGO

#### 8.4.1 Branch-and-Bound 回环检测

`scan_matching/fast_correlative_scan_matcher_2d.h:127-165`：

**预计算网格栈（Multi-Resolution Pyramid）**：
```
class PrecomputationGridStack2D:
    precomputation_grids_[]  // 各层存储该分辨率下的滑动窗口最大概率
    最大深度 = grids_.size() - 1
```
每层分辨率递推 2x 降采样，用于加速 BnB 搜索。

**BnB 搜索流程**：
```
MatchFullSubmap(point_cloud, min_score) → (score, pose):
    逐分辨率构建候选集合 → BnB 剪枝 → 返回最优候选
    深度: branch_and_bound_depth = 7
    线性搜索窗口: ±7m
    角度搜索窗口: ±30°
    最低匹配分数: min_score = 0.55
    全局重定位最低分数: 0.6
```

**核心 BnB**（`BranchAndBound()`）：在搜索树中递归分支，用父节点最大可能分数剪枝。

#### 8.4.2 约束构建（异步）

`constraints/constraint_builder_2d.h:60-170` `ConstraintBuilder2D`：
```
每个子地图维护独立的匹配器缓存：
    SubmapScanMatcher { grid, FastCorrelativeScanMatcher2D }
    后台 ThreadPool 中构建，子地图冻结后不变

MaybeAddConstraint(submap_id, node_id, data, init_pose):
    // 局部约束（节点→子地图）

MaybeAddGlobalConstraint(submap_id, node_id, data):
    // 全局约束（全子地图回环检测）
```

**约束评分**：
```
sampling_ratio = 0.3           // 采样比例
max_constraint_distance = 15m  // 回环最大距离
```

#### 8.4.3 位姿图优化（Ceres PGO）

`optimization/optimization_problem_2d.h`：

**优化变量**：每个节点的全局位姿 + 每个子地图的全局位姿

**约束类型与权重**：
| 约束类型 | Translation Weight | Rotation Weight |
|---------|-------------------|-----------------|
| 子地图内扫描匹配 | 1e5 | 1e5 |
| 回环 | 1.1e4 | 1e5 |
| 里程计 | 1e5 | 1e5 |
| 固定帧（GPS） | - | - |
| 加速度 | 1.1e2 | - |
| 旋转 | - | 1.6e4 |

**Ceres 求解器**：
```
max_num_iterations = 50
num_threads = 7
huber_scale = 1e1
use_nonmonotonic_steps = false
```

### 8.5 3D LiDAR 管线（补充）

**3D 混合网格**（`mapping/3d/hybrid_grid.h`）：
- 高分辨率栅格：用于精确定位
- 低分辨率栅格：用于快速搜索和回环

**3D 扫描匹配**（`scan_matching/real_time_correlative_scan_matcher_3d.h`）：
- 类似 BnB 但在 6 DoF 空间
- 旋转搜索更复杂，使用预计算的旋转直方图加速

### 8.6 IMU + 位姿外推管线

**IMU 追踪器**（`mapping/imu_tracker.h`）：
- 使用加速度计估计重力方向
- 使用陀螺仪估计角速度
- 输出传感器朝向（roll/pitch）用于扫描匹配初始猜测

**位姿外推器**（`pose_extrapolator`）：
```
use_imu_based = false  // 可用 IMU 融合
constant_velocity:      // 匀速模型
    imu_gravity_time_constant = 10.0
    pose_queue_duration = 0.001
```
支持匀速模型和 IMU 模型两种外推策略。

### 8.7 初始化

- 无需静止状态
- 首帧扫描直接创建第一个子地图
- IMU 可选，开启后使用加速度计初始估计重力

### 8.8 降级

- 无 IMU → 使用纯匀速模型外推
- 无里程计 → 仅依靠 scan matching 结果
- 子地图冻结后不变 → 不可逆的全局修正通过 PGO 实现
- 3D 无 hybrid grid → 回环检测计算量极大

### 8.9 线程模型

```
Main 线程：       ROS 回调接收传感器数据 → SensorCollator
Local SLAM 线程： 扫描匹配 + 子地图插入（低延迟，实时输出位姿）
ThreadPool：      约束构建 + 子地图预计算 + 全局 PGO 优化（后台异步）
```

回环检测和全局优化不会阻塞实时定位。