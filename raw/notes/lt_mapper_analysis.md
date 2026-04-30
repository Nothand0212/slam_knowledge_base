# lt-mapper 深度源码分析报告

> 分析日期：2026-04-28
> 代码版本：ICRA 2022 发布版本
> 源码路径：`/home/lin/Projects/lin_ws/slam_ws/lt-mapper/`

---

## 1. 数据接收与预处理

### 1.1 传感器数据入口与 ROS 话题

lt-mapper 是一个**离线处理框架**，不从 ROS 实时接收传感器数据。它使用**文件 I/O 协议**读取预先保存的多会话数据：

| 模块 | 包名 | 源文件 | 输入来源 |
|------|------|--------|---------|
| LT-SLAM | `ltslam` | `src/main.cpp` | 会话目录 (SCDs, 点云 PCD, 位姿图 TXT) |
| LT-removert | `ltremovert` | `src/removert_main.cpp` | LT-SLAM 输出的对齐位姿 + 原始会话点云 |

### 1.2 会话数据结构

每个会话（session）需要预先由外部 SLAM 前端（SC-LIO-SAM / SC-A-LOAM / FAST_LIO_SLAM）生成以下文件：

```
session_dir/
├── keyframes/            # 关键帧点云 (PCD)
├── scd/                  # Scan Context 描述符
└── optimized_poses.txt   # 内部优化后的位姿图 (带回环边)
```

加载实现在 `Session::loadSessionGraph()` (`ltslam/src/Session.cpp`) 中。

### 1.3 点云预处理

- **点云类型**：`pcl::PointXYZI` (`PointType`)
- **关键帧降采样**：`downSizeFilterICP` 使用 PCL `VoxelGrid` 进行体素滤波
- **坐标系统**：每个会话有独立的局部坐标系，通过 **anchor node** 变换到统一中心坐标系
- **无去畸变**：离线处理假设输入数据已完成去畸变

### 1.4 Scan Context 描述符加载

`Session::loadSessionScanContextDescriptors()` 加载每个关键帧的 Scan Context 描述符（SCD），用于后续跨会话回环检测：
- 存储在 `SCManager.polarcontexts_` 向量中
- 包含 polar context 矩阵和 ring key / sector key

---

## 2. 特征提取

### 2.1 特征类型

**lt-mapper 本身不执行特征提取。** 所有特征点（角点和平面点）由前端 SLAM（SC-LIO-SAM 等）预先提取并保存为关键帧点云。lt-mapper 专注于后端的多会话融合与位姿图优化。

### 2.2 前端特征（依赖 SLAM）

lt-mapper 依赖的前端（SC-LIO-SAM）使用的特征类型：
- **角点（Corner/Edge）**：基于 LOAM 平滑度公式，提取曲率大的点
- **平面点（Surface/Plane）**：基于 LOAM 平滑度公式，提取曲率小的点
- 特征数量由前端参数控制

### 2.3 位姿图节点与边

`Session` 类 (`ltslam/include/ltslam/Session.h:24-68`) 加载的图数据：

```cpp
struct Node { int idx; gtsam::Pose3 initial; };
struct Edge { int from_idx; int to_idx; gtsam::Pose3 relative; };
```

- `SessionNodes` / `SessionEdges` 使用 `std::multimap<int, *>` 存储
- 边分为两类：里程计边 (`|to_idx - from_idx| == 1`) 和回环边 (其他)

---

## 3. 位姿计算

### 3.1 位姿估计方法

lt-mapper 使用 **位姿图优化 (Pose Graph Optimization, PGO)**，不涉及前端的 scan-to-scan 或 scan-to-map 匹配。所有底層位姿估计由前端 SLAM 完成。

### 3.2 PGO 方法论

核心实现在 `LTslam::optimizeMultisesseionGraph()` (`ltslam/src/LTslam.cpp:157-184`) 中：

```cpp
isam->update(gtSAMgraph, initialEstimate);
isam->update();
isam->update();  // 多次 update 确保收敛
isam->update();
isam->update();
isam->update();
isamCurrentEstimate = isam->calculateEstimate();
```

**使用的库**：GTSAM 的 `ISAM2`（增量平滑与建图）。

### 3.3 图结构设计

#### 3.3.1 节点索引方案

`genGlobalNodeIdx()` 和 `genAnchorNodeIdx()` 辅助函数：
- **全局节点索引** = `session_index * kSessionStartIdxOffset + node_index`（但代码中未启用 offset，直接使用组合数字）
- **Anchor 节点索引** = 每个会话的锚点节点（固定索引，如 `session_idx * 1000000 + 0`）

#### 3.3.2 Anchor 节点机制

这是 lt-mapper 的**核心创新**。每个会话有一个 anchor 节点 (`BetweenFactorWithAnchoring`, `ltslam/include/ltslam/BetweenFactorWithAnchoring.h:18-127`)：

```
hx1 = anchor_p1 ⊕ p1     // 局部坐标 + anchor 变换 = 中心坐标
hx2 = anchor_p2 ⊕ p2     // 中心坐标的位姿
measurement = hx1 ⊖ hx2  // 回环测量 = 中心坐标下的相对位姿
```

自定义的 `BetweenFactorWithAnchoring<gtsam::Pose3>` 是一个 **四变量因子** (key1, key2, anchor_key1, anchor_key2)，在误差计算时先将局部位姿通过 anchor 变换到中心坐标系，再计算相对约束。

#### 3.3.3 Anchor 节点初始化

`LTslam::initTrajectoryByAnchoring()` (`LTslam.cpp:565-576`)：

- **中心会话（target/base session）**：anchor 节点添加强先验 `priorNoise` (方差 1e-12)
- **查询会话（source session）**：anchor 节点添加弱先验 `largeNoise` (方差 π², 1e8)
- 所有 anchor 节点初始化为 `poseOrigin` (单位位姿)

### 3.4 噪声模型

`LTslam::initNoiseConstants()` (`LTslam.cpp:100-133`)：

| 噪声模型 | 值（方差 Vector6: rx,ry,rz,x,y,z） | 用途 |
|---------|-----------------------------------|------|
| `priorNoise` | 1e-12 全部 | 中心会话 anchor 强约束 |
| `odomNoise` | 1e-4 全部 | 单会话内里程计边 |
| `loopNoise` | 1e-4 旋转, 1e-3 位移 | 单会话内回环边 |
| `largeNoise` | π² 旋转, 1e8 位移 | 非中心会话 anchor 弱约束 |
| `robustNoise` | Cauchy M-estimator + 0.5 方差 | 跨会话回环边（鲁棒核） |

---

## 4. 初始化

### 4.1 初始图状态

`LTslam::run()` (`LTslam.cpp:79-98`) 执行完整的管道：

```
initOptimizer()        → ISAM2 实例化
initNoiseConstants()   → 噪声模型初始化
loadAllSessions()      → 从文件加载所有会话
addAllSessionsToGraph() → 将各会话节点和边加入图
optimizeMultisesseionGraph(true) → 初次优化（仅单会话内部边）
```

### 4.2 会话加载

`LTslam::loadAllSessions()` (`LTslam.cpp:625-658`)：

- 遍历 `sessions_dir_` 目录下的所有会话文件夹
- 仅加载 `central_sess_name_` 和 `query_sess_name_`（当前版本限制两会话，设计为可扩展）
- `Session` 构造函数内部调用 `loadSessionGraph()`, `loadSessionScanContextDescriptors()`, `loadSessionKeyframePointclouds()`

---

## 5. 多传感器融合

### 5.1 传感器

lt-mapper **不直接使用传感器数据**。它的输入是经过前端 SLAM 处理后的中间产品：
- **关键帧点云**：来自 LiDAR
- **位姿图信息**：来自 LiDAR-Inertial SLAM（SC-LIO-SAM 融合 LiDAR + IMU）
- **Scan Context 描述符**：用于回环检测

### 5.2 融合方式

融合在**前端 SLAM** 完成（紧耦合 LiDAR-IMU 融合，如 LIO-SAM 架构）。lt-mapper 仅做后端 PGO，属于**多会话层面的位姿图融合**，不涉及传感器层面的数据融合。

### 5.3 多会话融合架构

```
Session 1 (中心/目标)                Session 2 (查询)
     ↓                                    ↓
 [anchor node] ← 强先验              [anchor node] ← 弱先验
     ↓                                    ↓
 [内部节点 + 里程计边 + 回环边]      [内部节点 + 里程计边 + 回环边]
     ↓                                    ↓
     └─── SC 回环检测 ───→ ←─── ICP 验证 ───┘
     └─── RS 回环检测 ───→ ←─── ICP 验证 ───┘
                    ↓
              ISAM2 联合优化
                    ↓
              统一坐标系轨迹
```

---

## 6. 后端优化

### 6.1 回环检测方法

lt-mapper 实现两种跨会话回环检测 (`LTslam.cpp`)：

#### 6.1.1 SC 回环 (ScanContext-based)

`LTslam::detectInterSessionSCloops()` (`LTslam.cpp:304-333`)：

- 遍历查询会话的所有 Scan Context 描述符
- 在中心会话的 SC 数据库中搜索最近邻（`detectLoopClosureIDBetweenSession()`）
- 若未找到 SC 回环（返回 -1），该节点入 `RSLoopIdxPairs_` 等待后续半径搜索
- SC 回环的位姿约束通过 `doICPVirtualRelative()` (`LTslam.cpp:187-242`) 计算：
  - 使用 PCL `IterativeClosestPoint` 在**局部坐标系**下做 ICP
  - 搜索半径 150m，最大迭代 100 次
  - fitness score 阈值过滤
  - 采样策略：`equisampleElements()` 等间隔选取 `kNumSCLoopsUpperBound` 个回环

#### 6.1.2 RS 回环 (Radius-Search-based)

`LTslam::findNearestRSLoopsTargetNodeIdx()` (`LTslam.cpp:450-505`)：

- 对 SC 未匹配的节点，在中心会话中搜索 10m 半径内的候选目标节点
- 使用**信息增益 (Information Gain)** 选择最优匹配：
  ```cpp
  S = Sy + H_s1*Cov_s1*H_s1^T + H_s2*Cov_s2*H_s2^T
  IG = 0.5 * log(det(S) / det(Sy))
  ```
- RS 回环的位姿约束通过 `doICPGlobalRelative()` (`LTslam.cpp:245-301`) 计算：
  - 使用 PCL ICP 在**中心坐标系**下做 ICP
  - 同样的采样策略

### 6.2 全局优化

**库**：GTSAM `ISAM2`

**优化流程** (`LTslam::run()`, `LTslam.cpp:79-98`)：

1. 初始图优化（仅会话内部边）
2. 添加 SC 回环边后优化
3. 添加 RS 回环边后优化（使用 SC 优化后的估计作为初始值）

**优化器参数** (`LTslam::initOptimizer()`, `LTslam.cpp:136-142`)：
- `relinearizeThreshold = 0.1`
- `relinearizeSkip = 1`

### 6.3 地图管理

**LT-removert 模块** (`ltremovert/src/Removerter.cpp`) 实现动态对象移除和变化检测：

- 使用 Removert 算法（基于 range image 的视角判断）
- **高动态点移除**：利用 range image 投影，比较前后帧的 occupancy 状态，移除移动物体（汽车、行人）
- **低动态点检测**：跨会话比较点云，识别新增/消失的静态物体（如停放的车辆变化）
- **Delta Map**：生成差异地图，支持高效的增量更新

### 6.4 多会话合并

`LTslam::updateSessionsPoses()` (`LTslam.cpp:145-153`)：优化后将每个会话的局部位姿通过 anchor transform 变换到中心坐标系。

---

## 7. 优缺点分析

### 7.1 算法优缺点

**优点：**
- **Anchor 节点机制**：创新性地将多会话对齐问题转化为 PGO，避免了逐步配准的漂移累积
- **双模式回环检测**：SC（外观）+ RS（几何）互补，提高召回率
- **信息增益筛选**：不是简单地加所有回环边，而是选择信息量最大的约束
- **鲁棒核函数**：Cauchy M-estimator 处理错误回环

**缺点：**
- **两会话限制**（当前版本）：虽然论文描述为 N-session，但代码目前只支持两会话
- **依赖外部前端**：无法独立运行，需要 SC-LIO-SAM 等生成中间数据
- **ICP 初始化**：RS 回环的 ICP 没有利用 SC 提供的 yaw 偏移作为初始猜测
- **离线处理**：不支持在线实时运行

### 7.2 工程优缺点

**优点：**
- 模块化设计，LT-SLAM 和 LT-removert 可独立运行
- 文件 I/O 协议简洁，易于集成到不同前端
- OpenMP 并行化的 ICP 匹配
- Docker 支持

**缺点：**
- ROS1 依赖（Melodic/Noetic），未适配 ROS2
- 代码注释较少，部分函数（如 `collect_digits`）可读性较差
- 硬编码参数多（如 10m 搜索半径、150m ICP 距离等）
- 缺少单元测试

### 7.3 适用场景

**适合：**
- 长期自主建图（lifelong mapping），需要维护跨天/跨周的一致性地图
- 动态环境（停车场、城市道路）的变化检测
- 多机器人协作建图（每个机器人一次会话）
- 需要地图变化分析的场景

**不适合：**
- 实时 SLAM（设计上是离线批处理）
- 单次建图任务（前端 SLAM 已足够）
- 计算资源受限的嵌入式平台

---

## 8. 对 phad_fusion 参考

### 8.1 值得借鉴的设计

1. **Anchor 节点 PGO** (`BetweenFactorWithAnchoring.h:18-127`, `LTslam.cpp:565-622`)：
   - 对多子图/多会话融合极其实用，phad_fusion 若有多个局部建图模块，可采用此方案统一坐标框架

2. **信息增益引导的回环选择** (`LTslam.cpp:419-448`)：
   - 避免添加冗余/低质量回环边，提高优化效率和精度
   - 公式清晰，可直接复用

3. **双模态回环检测** (`LTslam.cpp:304-505`)：
   - SC 提供快速粗匹配 + ICP 验证 + RS 补充召回
   - 这种分层回环策略是 SLAM 系统回环模块的标准范式

4. **Cauchy 鲁棒核** (`LTslam.cpp:126-132`)：
   - 跨会话/跨子图回环边使用鲁棒核函数处理异常值，比固定阈值更优雅

5. **等间隔采样** (`equisampleElements()`, `LTslam.cpp:353-368`)：
   - 在大量候选回环中均匀选取，防止局部密集回环导致图扭曲

### 8.2 应避免的设计

1. **离线批处理限制**：phad_fusion 如果目标是在线系统，需要设计增量式的 anchor 管理策略

2. **两会话硬编码**：应从一开始就设计为真正的 N-会话架构

3. **无协方差的 ICP 结果**：ICP 只返回 fitness score 作为噪声，未使用更精确的协方差估计（如 SuperOdom 的 Ceres Covariance）

4. **过度依赖文件系统**：在线系统应使用 ROS topic/共享内存等低延迟通信方式

5. **缺少位姿初始化失败处理**：如果 ICP 全部失败，系统没有 fallback 机制