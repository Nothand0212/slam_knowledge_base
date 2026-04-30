# FAST-LIO-SAM-SC-QN 深度源码分析

> 作者：engcang
> 版本：master (截至 2026-04-28)
> 平台：ROS1 (Noetic/Melodic), C++17, Ubuntu 20.04/22.04

---

## 1. 数据接收与预处理

### 1.1 系统架构概览

FAST-LIO-SAM-SC-QN **不是**一个完整的 LIO 框架，而是一个**后端 PGO + 回环检测系统**，其 LIO 前端依赖于外部运行的 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 节点。

**架构**:
```
FAST-LIO2 节点 (外部)              FAST-LIO-SAM-SC-QN 节点 (本仓库)
     │                                      │
     ├─ /Odometry (nav_msgs/Odometry)       │
     ├─ /cloud_registered (PointCloud2) ────┤
                                            ├─ odomPcdCallback (同步回调)
                                            ├─ keyframe 判断 → PGO
                                            ├─ loopTimerFunc (回环检测)
                                            └─ visTimerFunc (可视化)
```

### 1.2 数据入口

**源码路径**:
- 主类: `include/fast_lio_sam_sc_qn.h:53-117` (`FastLioSamScQn`)
- 构造函数: `src/fast_lio_sam_sc_qn.cpp:3-85`
- Odom+PCD 同步回调: `src/fast_lio_sam_sc_qn.cpp:87-206`
- main 函数: `src/main.cpp:1-17`

**ROS1 话题** (`fast_lio_sam_sc_qn.cpp:76-78`):
```cpp
sub_odom_ = make_shared<Subscriber<Odometry>>(nh_, "/Odometry", 10);
sub_pcd_ = make_shared<Subscriber<PointCloud2>>(nh_, "/cloud_registered", 10);
```

这两个话题通过 `message_filters::ApproximateTime` 同步策略 (`fast_lio_sam_sc_qn.h:50`):
```cpp
typedef message_filters::sync_policies::ApproximateTime<Odometry, PointCloud2> odom_pcd_sync_pol;
```

同步结果传入 `odomPcdCallback()`。

### 1.3 数据解析

回调函数中 (`fast_lio_sam_sc_qn.cpp:87-206`):
```cpp
current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_);
```

`PosePcd` 构造函数 (`include/pose_pcd.hpp:21-43`):
1. 从 odom 消息提取 `pose_eig_` (4x4 Eigen 矩阵)
2. 从 pcd 消息解序列化为 `pcl::PointCloud<PointType>`，其中 `PointType = pcl::PointXYZI`
3. **关键操作**: 点云存储为 **LiDAR frame** 下的坐标:
   ```cpp
   pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());
   // FAST-LIO publish data in world frame, so save it in LiDAR frame
   ```
   即 FAST-LIO2 输出世界坐标系下的点云，此代码将其转回 LiDAR 坐标系存储。

### 1.4 点云去畸变与预处理

**由 FAST-LIO2 在外部完成**。FAST-LIO2 使用其 IESKF 后端反向传播 (backward propagation) 完成点云去畸变，并输出 `cloud_registered` topic 中的已去畸变+全局对齐的点云。

本仓库 **不进行额外**的点云预处理（除 voxel grid 降采样外）。

### 1.5 IMU 数据

本仓库 **不直接接收 IMU 数据**。IMU 的处理完全由 FAST-LIO2 前端完成。FAST-LIO2 将 IMU 与 LiDAR 紧耦合后输出优化后的 odometry 和点云。

---

## 2. 点云处理

### 2.1 Voxel Grid 降采样

本仓库使用 **PCL VoxelGrid** 进行降采样 (`include/utilities.hpp:38-63`):

```cpp
inline pcl::PointCloud<PointType>::Ptr voxelizePcd(
    const pcl::PointCloud<PointType> &pcd_in, const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    // ...
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}
```

降采样分辨率配置 (`config.yaml:12-13`):
```yaml
quatro_nano_gicp_voxel_resolution: 0.3   # Quatro + Nano-GICP 使用的分辨率
save_voxel_resolution: 0.3               # 保存地图使用的分辨率
```

降采样在两个场景使用:
1. **回环匹配前**: `setSrcAndDstCloud()` 中使用，降低匹配点数 (`loop_closure.cpp:107`)
2. **地图保存时**: `saveFlagCallback()` 中使用 (`fast_lio_sam_sc_qn.cpp:414`)

### 2.2 特征提取

本仓库 **不进行**特征提取。所有的特征提取（包括 edge/planar 分类、曲率计算等）由 FAST-LIO2 前端完成。FAST-LIO2 使用原始点的直接点到平面/点到边匹配，不需要预先提取特征。

### 2.3 关键帧选择标准

**策略: 基于欧式距离的简单阈值**

`src/fast_lio_sam_sc_qn.cpp:503-506` (`checkIfKeyframe()`):
```cpp
bool FastLioSamScQn::checkIfKeyframe(const PosePcd &pose_pcd_in,
                                      const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3,1>(0,3)
                          - pose_pcd_in.pose_corrected_eig_.block<3,1>(0,3)).norm();
}
```

即当前帧到最新关键帧的平移距离 > `keyframe_thr_` 即创建新关键帧。配置默认 `keyframe_threshold: 1.5` 米 (`config.yaml:7`)。

**关键帧生命周期** (`fast_lio_sam_sc_qn.cpp:126-150`):
1. 存入 `keyframes_` 向量
2. 更新 ScanContext 全局描述子数据库
3. 添加 PGO 图因子: 当前帧与前一关键帧的 odom between factor
4. 调用 ISAM2 增量优化

---

## 3. 位姿计算

### 3.1 前端位姿估计：由 FAST-LIO2 完成

FAST-LIO2 使用 **IESKF (Iterated Extended Kalman Filter)** 进行 LiDAR-IMU 紧耦合位姿估计。其核心算法特点:

- 使用 **ikd-Tree** 作为增量地图数据结构
- **点到平面 + 点到边缘** 的复合残差
- **反向传播 (backward propagation)** 处理点云去畸变
- 状态维度 18: pos(3), rot(3), vel(3), bg(3), ba(3), gravity(3)

由于 FAST-LIO2 源代码未包含在本仓库中（为独立 ROS 节点），以上基于其公认特性描述。

### 3.2 后端位姿优化：GTSAM ISAM2

本仓库使用 **GTSAM ISAM2 (Incremental Smoothing and Mapping)** 进行因子图优化。

**初始化** (`fast_lio_sam_sc_qn.cpp:54-57`):
```cpp
gtsam::ISAM2Params isam_params_;
isam_params_.relinearizeThreshold = 0.01;
isam_params_.relinearizeSkip = 1;
isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
```

**因子图结构**:

1. **先验因子** (Prior Factor, `fast_lio_sam_sc_qn.cpp:113-118`):
   第一帧的 6-DOF 位姿先验:
   ```cpp
   auto variance_vector = (Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
   // 旋转方差 1e-4 rad^2, 平移方差 1e-2 m^2
   ```

2. **里程计因子** (Between Factor, `lines 135-146`):
   相邻关键帧之间的相对运动约束:
   ```cpp
   gtsam::BetweenFactor<Pose3>(idx-1, idx, pose_from.between(pose_to), odom_noise);
   ```
   噪声: 同样 1e-4 (rot), 1e-2 (trans)

3. **回环因子** (Loop Factor, `lines 228-238`):
   回环帧之间的相对位姿约束:
   ```cpp
   auto variance_vector = (Vector(6) << score, score, score, score, score, score).finished();
   gtsam::BetweenFactor<Pose3>(query_kf.idx_, candidate_idx, 
                                pose_from.between(pose_to), loop_noise);
   ```
   **关键**: 用 ICP score 作为信息矩阵的对角线值——score 越小（匹配越好），信息矩阵权重越大。

**ISAM2 更新** (`lines 163-170`):
```cpp
isam_handler_->update(gtsam_graph_, init_esti_);
isam_handler_->update();
if (loop_added_flag_) {
    isam_handler_->update();  // 回环后额外迭代 3 次促进收敛
    isam_handler_->update();
    isam_handler_->update();
}
```

**实时位姿外推** (`lines 94-104`):
```cpp
// realtime pose = last_corrected_pose * odom_delta
odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
```
即使用最近一次 ISAM2 结果作为基准，用 FAST-LIO2 的增量 odometry 进行实时位姿外推。

---

## 4. 初始化

### 4.1 系统初始化流程

`src/fast_lio_sam_sc_qn.cpp:107-122`:

1. 第一个同步的 Odom+PCD 回调到达时:
   - 设置为 `is_initialized_ = true`
   - 添加 first keyframe 到 `keyframes_`
   - 添加 prior factor (6-DOF 位姿先验) 到因子图
   - 插入初始估计到 `init_esti_`
   - 更新 ScanContext 描述子数据库

2. 后续帧进入正常处理流程 (`else` 分支, `lines 123-203`)

### 4.2 IMU 初始化

IMU 初始化完全由 FAST-LIO2 完成（不在本仓库中）。FAST-LIO2 使用如下初始化策略:
- 初始姿态从加速度方向估计重力
- 初始 bias 从静止 IMU 数据估计
- 不需要车辆静止即可初始化（但静止数据会提高质量）

### 4.3 初始地图

第一帧点云存储为 LiDAR 坐标系下的点云，同时其位姿设置为全局坐标系的 identity（通过 prior factor 固定）。

---

## 5. 后端与回环

### 5.1 回环检测方法：ScanContext

`include/loop_closure.h:77`: 使用 `SCManager sc_manager_`。

**更新 ScanContext** (`loop_closure.cpp:34-37`):
```cpp
void LoopClosure::updateScancontext(pcl::PointCloud<PointType> cloud) {
    sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}
```
每来一个关键帧就生成其 ScanContext 描述子并存入数据库。

**回环候选检索** (`loop_closure.cpp:39-56`, `fetchCandidateKeyframeIdx()`):
```cpp
std::pair<int, float> sc_detected_ = 
    sc_manager_.detectLoopClosureIDGivenScan(query_keyframe.pcd_);
int candidate_keyframe_idx = sc_detected_.first;
// 二次过滤: 欧式距离检查
if ((keyframes[candidate_keyframe_idx].pose_corrected_eig_.block<3,1>(0,3) - 
     query_keyframe.pose_corrected_eig_.block<3,1>(0,3)).norm() < 
    config_.scancontext_max_correspondence_distance_) {  // default 35m
    return candidate_keyframe_idx;
}
```

**关键设计**: ScanContext 负责在全局描述子空间中找最相似的关键帧（通过 column-wise 距离 + 行平移对齐），然后通过欧式距离做二次验证（config: `scancontext_max_correspondence_distance: 35.0`）。

### 5.2 回环几何验证：Quatro + Nano-GICP

**两阶段粗到细几何验证**:

`src/loop_closure.cpp:138-159` (`coarseToFineAlignment()`):

**阶段 1: Quatro 粗对齐** (`line 144`):
```cpp
reg_output.pose_between_eig_ = quatro_handler_->align(src, dst, reg_output.is_converged_);
```

Quatro 是基于 **FPFH + GNC (Graduated Non-Convexity)** 的全局点云配准:
- 提取 FPFH 局部特征描述子
- 使用 TEASER++ 求解全局最优变换
- 参数: `fpfh_normal_radius=0.9`, `fpfh_radius=1.5`, `noise_bound=0.3` (`config.yaml:33-36`)

**阶段 2: Nano-GICP 精细对齐** (`lines 152-156`):
```cpp
coarse_aligned_ = transformPcd(src, reg_output.pose_between_eig_);
const auto &fine_output = icpAlignment(coarse_aligned_, dst);
```

Nano-GICP (`loop_closure.cpp:110-136`, `icpAlignment()`):
- **NanoFLANN** 实现快速 KD-Tree 最近邻搜索
- **FastGICP** 实现体素级的 GICP (分布到分布匹配)
- 计算 **源/目标点云的局部分布协方差** (`lines 121-123`):
  ```cpp
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.calculateTargetCovariances();
  ```
- 参数: `max_iter=32`, `icp_score_threshold=1.5`, `correspondences_number=15`

**最终 transform** (`line 156`):
```cpp
reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
```

### 5.3 Submap 匹配

`loop_closure.cpp:58-108` (`setSrcAndDstCloud()`) 支持两种匹配策略:

**启用了 Quatro 时**:
- `enable_submap_matching=true`: 源和目标都用周围 ±submap_range 帧构建 submap
- `enable_submap_matching=false`: 源和目标各用单帧

**未启用 Quatro (纯 ICP) 时**:
- 源用单帧点云
- 目标用周围 ±submap_range 帧构建 submap（对 ICP 更有效）

config 默认: `enable_submap_matching: false`, `num_submap_keyframes: 10` (`config.yaml:7,9`)

### 5.4 位姿图优化: GTSAM ISAM2

**ISAM2 特性** (`README.md:13`):
- 增量式平滑与建图 (Incremental Smoothing and Mapping)
- 支持即时的因子图更新，不重新计算整个优化问题
- `relinearizeThreshold=0.01`: 线性化点变化超阈值时重新线性化
- `relinearizeSkip=1`: 每次更新都检查是否需要重新线性化

### 5.5 地图管理

- **存储方式**: 关键帧点云存储为 LiDAR frame 下的坐标（`pose_pcd.hpp:39-41`）
- **全局地图**: 按需将关键帧转换到世界系下拼接和体素降采样（`fast_lio_sam_sc_qn.cpp:308-322`）
- **保存格式**: 支持 .bag, .pcd, KITTI 格式三种输出 (`config.yaml:43-47`)

---

## 6. 局部优化与全局优化

### 6.1 局部优化

本仓库**没有独立的局部优化模块**。局部位姿估计完全委托给 FAST-LIO2 的 IESKF。

FAST-LIO2 的 IESKF 等效于:
- **Scan-to-Map** 匹配（使用 ikd-Tree 维护增量地图）
- IMU 紧耦合预测 (forward propagation)
- 点到平面/点到边复合残差

### 6.2 全局优化: ISAM2

ISAM2 提供贝叶斯树 (Bayes Tree) 上的增量优化:
- 每个新关键帧添加 odometry between factor
- 检测到回环时添加 loop factor
- **回环后的三次额外 update** (`fast_lio_sam_sc_qn.cpp:166-169`) 确保 ISAM2 充分收敛
- 优化后更新所有关键帧的 `pose_corrected_eig_` (`lines 187-193`):
  ```cpp
  for (size_t i = 0; i < corrected_esti_.size(); ++i) {
      keyframes_[i].pose_corrected_eig_ = 
          gtsamPoseToPoseEig(corrected_esti_.at<Pose3>(i));
  }
  ```

### 6.3 局部与全局的交互

1. FAST-LIO2 前端提供 odometry (相对位姿增量)
2. 本仓库后端提供 ISAM2 全局优化结果
3. 两者通过 `odom_delta_` 机制桥接:
   ```cpp
   // 实时位姿 = 最近一次全局优化结果 * 此后的 odometry 增量
   current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
   ```
4. 回环发生后，`odom_delta_` 重置为 Identity (`line 182`)，开始新一轮累积

---

## 7. 优缺点分析

### 7.1 算法优势

1. **模块化设计**: 后端 PGO 与前端 LIO 完全分离，可替换任意 LIO/LO 前端
   
2. **多层次回环验证**:
   - ScanContext (全局描述子) — 快速候选检索
   - 欧式距离 — 二次过滤
   - Quatro (FPFH + GNC) — 粗几何验证
   - Nano-GICP (分布匹配) — 精细配准验证
   四层验证确保回环质量

3. **Quatro 的全局匹配能力**: 对初始猜测不敏感，可在无先验信息下完成回环几何验证，优于纯 ICP

4. **Nano-GICP 的高效**: 结合 NanoFLANN 的快速最近邻和 FastGICP 的体素级分布匹配，比标准 GICP 快一个数量级

5. **ISAM2 增量优化**: 无需每次重新优化整个图，适合长时间运行

6. **KITTI/TUM 格式输出**: 方便与其他系统对比评估

### 7.2 算法局限

1. **依赖外部 LIO**: 自身不包含 LIO 前端，需要运行 FAST-LIO2 进程。若 LIO 前端漂移过大，ScanContext 回环检测可能失效。

2. **ScanContext 的旋转对称性**: 对旋转描述依赖 column-wise shift，在对称环境 (如方形走廊) 可能出现错误匹配。

3. **Quatro 计算开销**: FPFH 特征计算 + GNC 优化计算量较大，不适合高频运行

4. **无滑动窗口优化**:
   - 纯 PGO 不考虑关键帧间的点云共视关系
   - 不存在局部 BA 来修正 odometry drift
   - 因子图中只有相对运动约束，没有视觉/点云重投影约束

5. **点云存储冗余**: 每个关键帧存储完整点云 (非特征描述子)，长期运行内存占用线性增长

6. **单一信息矩阵**: 回环因子的信息矩阵直接用 ICP score 填充对角线，未区分位置/旋转的信息量

### 7.3 工程优缺点

**优点**:
- 代码结构清晰，注释详细
- 支持 `catkin build` 一键编译
- 多数据集预设启动配置 (KITTI, MulRan, Newer-College20)
- 丰富的可视化调试话题

**缺点**:
- ROS1 only，不支持 ROS2
- 需要预先编译多个第三方库 (GTSAM, TEASER++, tbb)
- 两个节点 (FAST-LIO2 + FAST-LIO-SAM-SC-QN) 运行，增加系统复杂度和通信延迟
- message_filters ApproximateTime 同步可能引入时间误差

### 7.4 适用场景

- **适合**: 大尺度室外 SLAM (自动驾驶，园区建图)，需要高质量回环检测的场景
- **不适合**: 实时性要求极高的场景（回环检测延迟较大），ROS2 环境，需要内置 LIO 的单进程方案

---

## 8. 对 phad_fusion 的关键参考

### 8.1 可借鉴的设计

1. **多阶段回环验证流水线** (`loop_closure.cpp:161-198`):
   - ScanContext → 欧式距离 → Quatro → Nano-GICP
   - phad_fusion 可以采用类似的多层验证策略，在粗检测后进行精细几何验证

2. **ScanContext 全局描述子** (`loop_closure.cpp:34-56`):
   - 轻量级的 LiDAR 场景描述子，仅依赖 `pcl::PointXYZI` 点云
   - 在 phad_fusion 中可作为回环候选检索的第一关

3. **GTSAM ISAM2 增量优化** (`fast_lio_sam_sc_qn.cpp:54-57`):
   - 增量式因子图优化，比 g2o/ceres 的重建+优化模式更高效
   - phad_fusion 若需要长时序 PGO，ISAM2 是更好的选择

4. **LIO + PGO 分离架构** (`fast_lio_sam_sc_qn.h:53-117`):
   - 前端 LIO 和后端 PGO 解耦，通过 ROS topic 通信
   - phad_fusion 可借鉴这种模块化设计，便于替换前端算法

5. **odom_delta 实时外推机制** (`fast_lio_sam_sc_qn.cpp:94-103`):
   - 用全局优化结果 + 里程计增量进行高频实时位姿输出
   - phad_fusion 定位模块可直接采用

6. **Multi-keyframe Submap 匹配** (`loop_closure.cpp:58-108`):
   - 用周围关键帧构建 submap 进行匹配，提高几何验证的鲁棒性
   - phad_fusion 可在回环验证中使用

### 8.2 应避免的陷阱

1. **不要完全依赖外部 LIO**: FAST-LIO-SAM-SC-QN 最大的局限是自身无 LIO 能力。phad_fusion 应内置完整的 LIO 前端，避免多进程通信开销和系统复杂度。

2. **Quatro 不适合高频运行**: FPFH + GNC 计算量大（每对候选需要数百毫秒），只适合在回环验证中使用，不应在每次位姿估计中调用。

3. **ScanContext 在对称场景的局限性**: 需要结合几何验证（如 ICP/Nano-GICP）一起使用，不可单独依赖。

4. **ISAM2 的内存累积**: 长期运行的 ISAM2 图会持续增长，需要设计 marginalization 或 pruning 策略。

5. **点云存储策略**: 存储完整点云 (而非特征) 导致关键帧数量增加时内存和 IO 压力线性增长。phad_fusion 应考虑存储降采样后点云或仅存储特征。

6. **ApproximateTime 同步的精度损失**: `message_filters::ApproximateTime` 允许时间窗口对齐，但可能引入 ms 级偏差。phad_fusion 应实现精确的时间戳关联机制。

---

*报告生成时间: 2026-04-28*
*分析基于源码完整阅读 (7 个核心源文件 + config/CMakeLists.txt)*
