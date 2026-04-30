# fusions_slam 源码深度分析报告

## 1. 数据接收与预处理

### 1.1 传感器数据入口（ROS 回调链路）

数据流由 `FusionSlamFrontWrapper` 类统一管理（`src/wrapper/ros/fusion_slam_ros_wrapper.cpp:9-80`）。构造函数中初始化三个 subscriber：

```
cloudSubscriber  →  lidarCloudMsgCallBack   (sensor_msgs::PointCloud2)
imuSubscriber    →  imuMsgCallBack          (sensor_msgs::Imu)
rtkInspvaxSubscriber → rtkInspvaxMsgCallBack (novatel_msgs::INSPVAX)
```

支持 3 种 RTK 数据源类型（`rtkDateType`），分别对应 `INSPVAX`（类型=1）、`NavSatFix`（类型=2）、`Odometry`（类型=3），配置在 YAML 文件中。

**点云回调链路**：
```
lidarCloudMsgCallBack (fusion_slam_ros_wrapper.cpp:94-113)
  → VelodyneProcess::process 或 OusterProcess::process (ROS→PCL格式转换)
  → 车身坐标系裁剪 (lidarXYBox)
  → pcl::transformPointCloud (T_imu_lidar 外参变换)
  → pcl::transformPointCloud (T_imu_ant.inverse 天线杆臂补偿)
  → frontPtr->addLidar(cloud)
```

**IMU 回调链路**：
```
imuMsgCallBack (fusion_slam_ros_wrapper.cpp:116-119)
  → ImuProcess::process (imu_process.cpp:9-14)  // ROS IMU → ImuType
  → frontPtr->addImu(imu)
```

### 1.2 时间同步机制

核心函数 `Front::syncMeasureGroupAdd`（`front.cpp:160-224`）实现纳秒级精确时间对齐：

1. 取最前面一帧点云的时间 `lidarBeginTime`（`msg.lidarBeginTime = clouds.front().timeStamp.nsec()`，行 166）
2. 取点云末端点的时间 `lidarEndTime = beginTime + 最后一个点的 offset_time`（行 167）
3. 丢弃 `lidarBeginTime` 之前的 IMU 数据（行 178-181）
4. 收集 `[lidarBeginTime, lidarEndTime]` 区间内的所有 IMU 数据（行 183-191）
5. 至少需要 5 条 IMU 测量才认为同步成功（行 192）：`if(cnt < 5) return false`
6. 将点云数据和 RTK 姿态也插入到 `measureGroupAdd.mapDatas` 中，形成时间戳排序的测量组（行 193-216）

所有数据按 `uint64_t nsec` 纳秒时间戳 key 存入 `std::map<uint64_t, DataUnit>`，利用 map 的自动排序保证了时间序列的正确性。

### 1.3 点云去畸变（Deskewing）

去畸变在 `Propagate::run()`（`propagate.cpp:33-123`）中实现：

**步骤一：IMU 前向积分**（行 52-101）
遍历时间排序的 `datasTimeList`，对相邻 IMU 测量做零阶保持：
```
avrAngvel = 0.5*(head.gyroscope + tail.gyroscope)
avrAcc = 0.5*(head.acceleration + tail.acceleration)
ieskfPtr->predict(tmpImu)  // IESKF 预测步骤
```

同时，在此过程中遇到 RTK 观测（type==2）时即时插入：
- `rtkType == 0`：同时观测位置和姿态
- `rtkType == 1`：仅观测位置
- `rtkType == 2`：仅观测姿态

**步骤二：反向补偿**（行 105-121）
从最后一个 IMU 姿态开始，反向遍历点云中的每个点：
```
for (auto it = imuPoses.end()-1; it != imuPoses.begin(); --it) {
    for (; it_point->offset_time / 1e9 > head->time; --it_point) {
        double dt = it_point->offset_time*1e-9 - head->time;
        R_i = head->rot.toRotationMatrix() * so3Exp(tail->angvel * dt)
        P_i = (it_point->x, it_point->y, it_point->z)
        T_ei = head->pos + head->vel * dt + 0.5 * tail->acc * dt * dt - state.position
        P_e_imu = state.rotation.conjugate() * (R_i * P_i + T_ei)
        // 将畸变后的点写回 cloud
    }
}
```

这是 FAST-LIO2 经典的反向传播去畸变方法：利用估计的 IMU 轨迹将每个激光点从当前帧末时刻（统一时刻 t_end）补偿到世界坐标系，再通过当前状态估计投影回 IMU 坐标系。

### 1.4 点云预处理

**Voxel 栅格滤波**（`front.cpp:66-74`）：
```cpp
pcdVoxelFilter.setLeafSize(leafSize, leafSize, leafSize);  // 默认 0.2m
pcdVoxelFilter.setInputCloud(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr);
pcdVoxelFilter.filter(*pcdPtrFilterResult);
```
使用 PCL 的 `pcl::VoxelGrid<PointType>`（`base_type.h:24`）。

**激光线束降采样**（`velodyne_process.h:51-88`）：
- 64 线 → 32 线：跳过偶数索引的 ring
- 128 线 → 32 线：只保留 ring % 4 == 0 的点
- 支持按 `pointGapNum` 间隔采样

**距离滤除**（`fusion_slam_ros_wrapper.cpp:103-108`）：
- 根据 `lidarXYBox` 参数滤除车身区域
- 滤除 NaN 点

---

## 2. 特征提取与匹配

### 2.1 特征提取方法

fusions_slam **没有显式的特征点分类**（如 LOAM 中提取 edge/planar 特征），而是在观测方程的构建过程中隐式地使用基于局部平面拟合的方法。

### 2.2 平面拟合算法

核心实现为 `planarCheck` 函数（`include/fusion_slam/math/math.h:22-57`）：

```cpp
template<typename pointTypeT>
static bool planarCheck(const std::vector<pointTypeT>& points, Eigen::Vector4d &pabcd, float threshold) {
    A.resize(pointsSize, 3);  // 最小二乘矩阵 A
    B.resize(pointsSize);     // B 全为 -1
    normalVector = A.colPivHouseholderQr().solve(B);  // QR 分解求解法向量
    // 归一化得到平面参数 (a,b,c,d)  其中 d = 1.0/norm
    // 检查所有点到平面距离是否 < threshold（默认 0.1）
}
```

**使用步骤**（`ieskf.cpp:207-271`，`calculateLidarZH`）：
1. 对去畸变后的当前帧点云中的每个点，用 `globalMapIkdtreePtr->Nearest_Search()` 在全局地图中查找最近 5 个点
2. 检查最近点距离：`pointSearchSqDis[4] > 5` 则丢弃（防止外点）
3. 对这 5 个最近邻点调用 `planarCheck` 判断是否为有效平面
4. 对平面上的点计算残差：$r_d = a \cdot p_x + b \cdot p_y + c \cdot p_z + d$
5. 使用 Huber 启发式权重：`s = 1 - 0.9 * |pd| / sqrt(point.norm())`，要求 `s > 0.9`

### 2.3 特征类型

- **平面特征（Planar）**：通过最近邻平面拟合隐式提取，不显式存储为特征类型
- **边缘特征**：未使用
- **强度特征**：不使用

### 2.4 匹配策略

使用 KD-Tree 或 iKD-Tree 做最近邻搜索，当前帧点 → 全局地图中的最近 5 个点。提供了两种实现：

- **PCL KD-Tree** 版本：`calculateLidarZH(state, Z, H)`（`ieskf.cpp:273-333`）
  - 使用 `globalMapKdtreePtr->nearestKSearch(globalPoint, 5, pointIds, distance)`
  - 再从 `localMapPtr` 取回实际点坐标

- **iKD-Tree 版本**：`calculateLidarZH(state, Z, H, nearPoints)`（`ieskf.cpp:207-271`）
  - 使用 `globalMapIkdtreePtr->Nearest_Search(globalPoint, 5, pointsNear, pointSearchSqDis)`，直接返回点坐标
  - 性能更优，且在构建局部地图时维护了 `nearstPoints` 预计算容器

---

## 3. 位姿计算

### 3.1 IESKF 架构（自实现）

状态向量 18 维（`StateX` = rotation(4d quat) + position(3d) + velocity(3d) + bg(3d) + ba(3d) + gravity(3d)），实际误差状态 18 维（`ieskf.h:28-44`）：

$$
\mathbf{x} = [\mathbf{r}, \mathbf{p}, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a, \mathbf{g}]^T
$$

### 3.2 前向传播（Predict）

`IESKF::predict`（`ieskf.cpp:70-105`）实现经典 IMU 运动学：

```
rotation = rotation * so3Exp((gyro - bg) * dt)
position += velocity * dt
velocity += (R * (acc - ba) + gravity) * dt
```

雅可比矩阵：
- `Fx.block<3,3>(0,0) = so3Exp(-gyro*dt)` — 姿态对姿态的雅可比
- `Fx.block<3,3>(0,9) = -J_right(-gyro*dt)*dt` — 姿态对陀螺 bias 的雅可比
- `Fx.block<3,3>(6,0) = -R * skew(acc) * dt` — 速度对姿态的雅可比
- `Fx.block<3,3>(6,15) = I * dt` — 速度对重力的雅可比
- 噪声矩阵 `Fw` 映射 12 维过程噪声到 18 维状态

协方差传播：`P = Fx * P * Fx^T + Fw * Q * Fw^T`

### 3.3 迭代观测更新（Iterated Update）

`IESKF::lidarObserve`（`ieskf.cpp:107-155` 和 `157-205`）实现 IESKF 的迭代机制：

```cpp
for (int i = 0; i < iterTimes; i++) {  // 默认 10 次迭代
    errorState = getErrorStateX(curX, X);  // curX 相对于先验 X 的误差
    J_inv.setIdentity();
    J_inv.block<3,3>(0,0) = J_right(errorState.block<3,1>(0,0));  // SO3 右雅可比逆
    curP = J_inv * P * J_inv^T;  // 将先验协方差变换到当前迭代点
    K = (H^T * H + (curP/0.001)^{-1})^{-1} * H^T;  // 卡尔曼增益
    updateX = -K*Z - (I-K*H)*J_inv*errorState;  // 迭代更新公式
    // 收敛判断：updateX 每个分量 < 0.001
}
```

**点云观测的雅可比**（`calculateLidarZH`，IESKF.cpp:263-269）：
```
dr = -normal^T * R * skew(point)  // 旋转雅可比 (1×3)
H.block<1,3>(vi,0) = dr^T
H.block<1,3>(vi,3) = normal^T     // 平移雅可比 (1×3)
Z(vi,0) = pd = normal·point + d   // 点到平面距离
```

### 3.4 各类观测类型

| 观测类型 | 方法 | 文件行号 | 雅可比 |
|---------|------|---------|--------|
| Lidar 点云 | `lidarObserve` | ieskf.cpp:107-155 | point-to-plane |
| 姿态观测 | `rotationObserve` | ieskf.cpp:367-381 | H = [I, 0, 0, 0, 0, 0] |
| 位置观测 | `positionObserve` | ieskf.cpp:382-396 | H = [0, I, 0, 0, 0, 0] |
| 速度观测 | `velocityObserve` | ieskf.cpp:397-411 | H = [0, 0, I, 0, 0, 0] |

每种观测使用独立的卡尔曼增益 K，基于 `P_`（用于外协方差的协方差矩阵）计算：
```
K = P_ * H^T * (H * P_ * H^T + noise)^{-1}
dX = K * residual
P_ = (I - K*H) * P_
updateAndReset()
```

### 3.5 地图更新策略

`Map::mapIncremental`（`map.cpp:125-169`）实现 iKD-Tree 增量式地图维护：

1. 将当前帧点云变换到世界坐标系
2. 对每个点检查其对应的 `nearstPoints[i]`（上一步匹配时的邻居）
3. 计算该点在体素滤波网格中的中点位置 `mid_point`
4. 判断是否需要降采样：若已有邻居距离网格中点更近，则不添加（`needAdd = false`）
5. 分两路：需要降采样的通过 `PointToAdd`，不需要的直接 `PointNoNeedDownsample`

`lasermapFovSegment`（`map.cpp:79-118`）实现 FOV 动态窗口管理：
- 当机器人运动超过 `MOV_THRESHOLD * DET_RANGE` 时触发地图平移
- 删除离开局部窗口区域的点：`ikdtreePtr->Delete_Point_Boxes(cubNeedrm)`

### 3.6 IMU 积分方法

采用 **欧拉离散积分**，而非预积分方法。没有使用 GTSAM 或 Ceres-factor-graph 做 IMU 预积分，而是直接用 `IESKF::predict` 在每个 IMU 测量时刻做状态传播。

---

## 4. 初始化

### 4.1 IMU 静态初始化

`ImuStaticInit`（`imu_static_init.cpp:11-44`）实现：

1. 收集 `imuInitDuration`（默认 1.0 秒）内的 IMU 数据
2. 累加加速度计和陀螺仪测量值（`addImu`）
3. 完成后取平均：
   ```
   _staticalAcc /= _imuDataNum
   _staticalGyro /= _imuDataNum
   ```
4. 利用重力幅值标定加速度计尺度：
   ```
   _imuScale = GRAVITY / _staticalAcc.norm()   // GRAVITY = 9.81
   _gravity = -_staticalAcc.normalized() * GRAVITY
   ```
5. 加速度计 bias = 平均加速度 + 重力向量

### 4.2 初始位姿确定

`Front::init`（`front.cpp:101-158`）：
- 若启用 RTK 初始化（`initUseRtk = true`），使用第一个 RTK 位置和姿态作为初始状态
- 否则位置置零，姿态置单位矩阵
- 速度初始化为零（代码中有 TODO 注释：移动中车辆需通过其他传感器获取）
- 初始化完成后调用 `propagatePtr->lastPcdEndTime` 和 `lastImu` 赋值

### 4.3 首帧点云处理

初始化完成后（`front.cpp:50-61`）：
- 调用 `mapPtr->reset()` 清除地图
- 调用 `mapPtr->addPcdAndUpdateLocalMap` 将首帧点云加入地图并初始化 iKD-Tree（`map.cpp:174-195`）

---

## 5. ROS 松耦合设计

### 5.1 目录树分析

```
fusions_slam/
├── include/
│   ├── fusion_slam/           ← ROS 无关的算法核心（纯 C++ + Eigen + PCL）
│   │   ├── math/              ← 数学工具（SO3, 平面拟合, 旋转转换）
│   │   ├── modules/
│   │   │   ├── back/          ← 后端因子图优化（Ceres, LIO-GPS 融合）
│   │   │   ├── front/         ← 前端（IESKF + 地图 + 同步）
│   │   │   ├── ieskf/         ← IESKF 滤波器核心
│   │   │   ├── location/      ← 定位模式（基于先验地图）
│   │   │   ├── map/           ← 地图管理（iKD-Tree, PCL KD-Tree）
│   │   │   └── sensors/       ← 传感器抽象（imu_type, pointcloud, pose）
│   │   └── type/              ← 纯数据结构（无 ROS 依赖）
│   └── wrapper/
│       └── ros/               ← ROS 相关封装层
│           ├── fusion_slam_ros_wrapper.h  ← 建图模式 ROS 封装
│           ├── location_ros_wrapper.h     ← 定位模式 ROS 封装
│           ├── back_ros_wrapper.h         ← 后端 ROS 封装
│           ├── imu_process/              ← IMU ROS 消息转换
│           └── lidar_process/            ← 激光雷达 ROS 消息转换
├── src/
│   ├── app/                   ← 三大入口 main() 函数
│   ├── fusion_slam/           ← 算法核心实现
│   └── wrapper/ros/           ← ROS 封装实现
├── thirdparty/
│   ├── ikd-Tree/              ← FAST-LIO2 的增量 KD 树
│   └── ivox3d/                ← Faster-LIO 的增量体素（已拉取但未实际使用）
├── config/                    ← YAML 配置参数
├── script/                    ← 启动/可视化脚本
└── CMakeLists.txt             ← 编译定义
```

### 5.2 ROS-算法分离模式

fusions_slam 的松耦合设计核心体现在 **三层依赖关系**：

**第一层 - type（纯数据）**：
```cpp
// include/fusion_slam/type/ - 不依赖任何 ROS 头文件
// 只依赖 Eigen 和 PCL
struct PointType { float x,y,z; float intensity; uint32_t offset_time; int32_t ring; };
struct ImuType { Eigen::Vector3d acceleration, gyroscope; TimeStamp timeStamp; };
struct Pose { TimeStamp timeStamp; Eigen::Quaterniond rotation; Eigen::Vector3d position; };
```

**第二层 - 算法核心（fusion_slam/）**：
- `IESKF` 类只依赖 Eigen、Glog、YAML-CPP
- `Front` 类依赖 IESKF、Propagate、Map、ImuStaticInit，不依赖 ROS
- `Map` 类依赖 PCL 和 iKD-Tree
- `Propagate` 类只依赖 Eigen

**第三层 - ROS 封装（wrapper/ros/）**：
- `ImuProcess::process(sensor_msgs::ImuPtr → ImuType)`：单向转换，只把 ROS msg 转成内部类型
- `VelodyneProcess::process(sensor_msgs::PointCloud2Ptr → PointCloud)`：同理
- `FusionSlamFrontWrapper`：持有 `Front::Ptr` 做算法调用，处理 ROS 回调、外参变换、消息发布

**CMakeLists.txt 也体现了分离**：
```
IESKF_FILE  = algorithm files only (ieskf + front + propagate + map + math)
WRAPPER_FILE = ros wrapper files + app node
```

两者分别链接，但 IESKF 文件组不包含任何 `catkin_LIBRARIES` 依赖。

### 5.3 值得复用的设计模式

1. **DataUnit + MeasureGroupAdd 抽象**（`measure_group.h:18-29`）：
   - 使用 `std::map<uint64_t, DataUnit>` 做统一的多传感器时间排序
   - `DataUnit.type` 枚举（0=IMU, 1=Lidar, 2=GPS, 3=Camera, 4=Speed）支持任意传感器扩展

2. **Propagate 分离成 IMU 去畸变**：去畸变逻辑集中在 `Propagate` 类中，不污染 IESKF 逻辑

3. **Wrapper 持有 Ptr 管理生命周期**：`frontPtr`、`imuProcessPtr`、`rtkSwitchPtr` 等均用 `shared_ptr` 管理

4. **外参在 Wrapper 层处理**：`T_imu_lidar` 和 `T_imu_ant` 的变换在 ROS 回调中完成，算法核心接收的是统一坐标系的点云

---

## 6. 优缺点分析

### 6.1 算法优点

| 优点 | 说明 |
|------|------|
| IESKF 自实现 | 18 维状态估计，包含 IMU bias 和重力在线估计 |
| 多源观测融合 | 统一框架下支持 lidar point-to-plane、位置、姿态、速度四种观测 |
| iKD-Tree 增量地图 | 借鉴 FAST-LIO2 的增量 KD 树，支持大场景实时建图 |
| 反向传播去畸变 | 经典 FAST-LIO 方法，无需外部里程计辅助 |

### 6.2 算法缺点

| 缺点 | 说明 |
|------|------|
| 无闭环检测 | 纯 odometry 模式，长距离累计误差无法消除 |
| 仅使用平面特征 | 在结构特征不明显的场景（隧道、高速路）可能退化 |
| 无协方差自适应 | Q 矩阵固定，不随工况动态调整 |
| 欧拉积分精度 | 相比 IMU 预积分（如 Forster 2015），高频运动下精度较差 |

### 6.3 工程优点

| 优点 | 说明 |
|------|------|
| ROS 松耦合清皙 | fusion_slam/ 层完全可脱离 ROS 运行 |
| 多传感器类型适配 | 支持 Velodyne/Ouster 两种激光雷达，3 种 RTK 协议 |
| 配置化管理 | 所有参数通过 YAML 配置，无需重新编译 |
| 双模式支持 | 同一套 IESKF 既做建图又做定位 |
| 代码量适中 | 核心约 2000 行（不含第三方库），易于理解和修改 |

### 6.4 工程缺点

| 缺点 | 说明 |
|------|------|
| 测试覆盖为零 | 没有单元测试和集成测试 |
| 内存管理隐式 | 大量使用裸指针（`curCloudPtr`, `localMapPtr` 等类成员）传递引用 |
| 硬编码阈值 | `planarCheck` 阈值 0.1、最近邻阈值 5.0 等均硬编码 |
| 仅支持 ROS1 | 基于 catkin 构建，无 ROS2/非ROS 接口 |

### 6.5 适用场景
- 城市道路、园区建图（有 RTK 辅助）
- 已知地图内的实时定位
- 激光雷达 + IMU 的紧耦合 SLAM

### 6.6 不适用场景
- 开阔高速场景（缺少平面特征会发散）
- 大闭环场景（无回环检测）
- 纯视觉融合（无相机模块实现）

---

## 7. 对 phad_fusion 的关键参考

### 7.1 可借鉴的设计

| 设计元素 | 说明 | 来源 |
|---------|------|------|
| MeasureGroupAdd 机制 | 统一时间排序的多传感器容器 | `measure_group.h:25-29` |
| IESKF 多观测接口 | 统一 filter 支持不同 sensor 观测 | `ieskf.h:76-78` |
| 双向传播去畸变 | IMU 前向预测 + 反向点云补偿 | `propagate.cpp:52-121` |
| Wrapper 层外参处理 | 算法层不与具体传感器坐标绑定 | `fusion_slam_ros_wrapper.cpp:109-112` |
| Cere 后端松耦合 | 后端因子图接受通用 Pose 数据，不依赖前端实现 | `lio_gps_opt.cpp:51-160` |

### 7.2 应避免的陷阱

| 陷阱 | 说明 | 改进建议 |
|------|------|----------|
| 无时间序列检查 | `predict()` 中 dt > 5/imuHz 仅 skip 不补偿 | 增加丢帧重采样/插值 |
| sync 条件脆弱 | `cnt < 5` 的 IMU 数量要求可能在高速下不满足 | 改用自适应阈值 |
| 全局变量 static local | 多个 callback 中声明了 `static` 变量，多线程不安全 | 改用类成员变量 |
| 无协方差输出 | odom 消息不包含协方差信息 | 输出 6×6 位姿协方差 |
| planarCheck 无退化检测 | QR 求解奇异时不处理 | 增加条件数检查 |

### 7.3 后端融合的关键参考

`LioGpsOpt` 类（`lio_gps_opt.cpp:51-160`）展示了 LIO + GPS 的因子图式松耦合后端：

1. **LIO 相对约束**：使用 `RelativeRTError` 因子（`factors.h:130-192`），约束相邻帧间的相对位姿
2. **GPS 全局约束**：使用 `TError` 因子（`factors.h:28-52`），约束 LIO 轨迹在 GPS 位置附近
3. **Ceres 求解器**：`SPARSE_NORMAL_CHOLESKY` + `HuberLoss(1.0)` + `QuaternionParameterization`
4. **异步优化线程**：`threadOpt` 后台运行，不阻塞主流程

phad_fusion 可以复用这种 "LIO 相对因子 + 全局约束" 的因子图架构，并用更大规模的 `map<double, Pose>` 滑窗管理。

---

### 附录：关键文件索引

| 模块 | 头文件定义 | 实现文件 |
|------|-----------|---------|
| IESKF | `include/fusion_slam/modules/ieskf/ieskf.h:50` | `src/fusion_slam/modules/ieskf/ieskf.cpp` |
| Front | `include/fusion_slam/modules/front/front.h:21` | `src/fusion_slam/modules/front/front.cpp` |
| Propagate | `include/fusion_slam/modules/sensors/imu/propagate.h:20` | `src/fusion_slam/modules/sensors/imu/propagate.cpp` |
| Map | `include/fusion_slam/modules/map/map.h:17` | `src/fusion_slam/modules/map/map.cpp` |
| ImuStaticInit | `include/fusion_slam/modules/sensors/imu/imu_static_init.h:13` | `src/fusion_slam/modules/sensors/imu/imu_static_init.cpp` |
| LioGpsOpt | `include/fusion_slam/modules/back/lio_gps_opt.h:23` | `src/fusion_slam/modules/back/lio_gps_opt.cpp` |
| Location | `include/fusion_slam/modules/location/location.h:20` | `src/fusion_slam/modules/location/location.cpp` |
| MeasureGroupAdd | `include/fusion_slam/type/measure_group.h:25` | - |
| FustionSlamFrontWrapper | `include/wrapper/ros/fusion_slam_ros_wrapper.h:26` | `src/wrapper/ros/fusion_slam_ros_wrapper.cpp` |
| VelodyneProcess | `include/wrapper/ros/lidar_process/velodyne_process.h:43` | header-only |
| planarCheck | `include/fusion_slam/math/math.h:22` | template in header |
| SO3 函数 | `include/fusion_slam/math/so3.h:9-48` | header-only |
| Cere 因子 | `include/fusion_slam/modules/back/factors.h:28-193` | header-only |

---

## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| LiDAR (Velodyne/Ouster) | 10 Hz | `sensor_msgs::PointCloud2` | `fusion_slam_ros_wrapper.cpp:10` | FusionSlamFrontWrapper |
| IMU (9-axis) | ≥100 Hz | `sensor_msgs::Imu` | `fusion_slam_ros_wrapper.cpp:11` | FusionSlamFrontWrapper |
| RTK (NovAtel) | ~5-20 Hz | `novatel_msgs::INSPVAX` / `sensor_msgs::NavSatFix` / `nav_msgs::Odometry` | `fusion_slam_ros_wrapper.cpp:12` | FusionSlamFrontWrapper |

### 8.2 LiDAR 管线

#### 8.2.1 原始数据

- **Velodyne** → `VelodyneProcess::process`：ROS `PointCloud2` → PCL `PointXYZIRT`（`x, y, z, intensity, ring, offset_time`）
- **Ouster** → `OusterProcess::process`：同上转换
- **线束降采样**（`velodyne_process.h:51-88`）：64线→32线（跳过偶数 ring）、128线→32线（保留 `ring % 4 == 0`）、支持 `pointGapNum` 间隔采样
- **外参处理**（`fusion_slam_ros_wrapper.cpp:109-112`）：
  ```cpp
  pcl::transformPointCloud(input, cloud, T_imu_lidar);  // LiDAR→IMU
  pcl::transformPointCloud(cloud, output, T_imu_ant.inverse());  // 天线杆臂补偿
  ```
  `T_imu_lidar` 和 `T_imu_ant` 在 YAML 中配置，Wrapper 层统一处理坐标变换

#### 8.2.2 预处理

**车身裁剪**（`fusion_slam_ros_wrapper.cpp:103-108`）：根据 `lidarXYBox` 参数滤除车身区域点

**Voxel Grid 降采样**（`front.cpp:66-74`）：
```cpp
pcdVoxelFilter.setLeafSize(leafSize, leafSize, leafSize);  // 默认 0.2m
```

**Range 过滤**：滤除 NaN 点

#### 8.2.3 去畸变（Backward Propagation）

`propagate.cpp:33-123` `Propagate::run()`：

**步骤 1：IMU 前向积分**（`propagate.cpp:52-101`）：
- 遍历时间排序的 `datasTimeList`（`std::map<uint64_t, DataUnit>`）
- 相邻 IMU 测量零阶保持：
  ```
  avrAngvel = 0.5*(head.gyro + tail.gyro)
  avrAcc = 0.5*(head.acc + tail.acc)
  ieskfPtr->predict(tmpImu)
  ```
- IMU 预测方程（欧拉积分）：
  ```
  R = R * so3Exp((gyro - bg) * dt)
  p += v * dt
  v += (R*(acc - ba) + g) * dt
  ```

**步骤 2：反向补偿**（`propagate.cpp:105-121`）：
- 从最后一个 IMU 姿态开始，反向遍历点云中每个点：
  ```
  R_i = head.rot * so3Exp(tail.angvel * dt)           // 旋转外推
  T_ei = head.pos + head.vel*dt + 0.5*tail.acc*dt²    // 平移外推
  P_compensated = state.rot⁻¹ * (R_i*P_i + T_ei)       // 补偿到统一时刻
  ```
- 将点从当前帧末时刻统一补偿到 IMU 坐标系，消除帧内运动畸变

#### 8.2.4 特征提取（隐式平面拟合）

**不使用传统 edge/planar 分类**。在 IESKF 观测方程构建中隐式提取平面特征。

**平面拟合**（`math.h:22-57` `planarCheck()`）：
1. 在全局地图 iKD-Tree 中查找最近 5 个点
2. 用 `A.colPivHouseholderQr().solve(B)` 求解平面方程 `ax + by + cz + d = 0`
3. 验证所有 5 个点到平面距离 < `threshold`（默认 0.1）

**有效点筛选**（`ieskf.cpp:207-271` `calculateLidarZH()`）：
- 最近邻距离：`pointSearchSqDis[4] > 5` → 丢弃（防止外点）
- Huber 启发式权重：`s = 1 - 0.9·|pd| / sqrt(||point||)`，要求 `s > 0.9`

#### 8.2.5 匹配/配准（IESKF Iterated Update）

**IESKF 架构**（`ieskf.cpp:107-155` `lidarObserve()`）：
- 默认 10 次迭代
- 状态向量：18 维 `[r(3), p(3), v(3), bg(3), ba(3), g(3)]`
- 每次迭代：
  1. 计算误差状态 `errorState = getErrorStateX(curX, X)`
  2. 协方差变换到当前迭代点：`curP = J_inv * P * J_inv^T`
  3. 卡尔曼增益：`K = (H^T H + (curP/0.001)⁻¹)⁻¹ * H^T`
  4. 迭代更新：`updateX = -K*Z - (I-K*H)*J_inv*errorState`
  5. 收敛判断：`max(|updateX|) < 0.001`

**观测雅可比**（`ieskf.cpp:207-271`）：
```
dr = -normal^T * R * skew(point)  // 旋转雅可比 (1×3)
H = [dr^T | normal^T]             // 平移雅可比 (1×3)
Z = normal·point + d              // 点到平面距离残差
```

#### 8.2.6 算法消费（多源观测融合）

| 观测类型 | 方法 | 观测量 | 雅可比块 |
|---------|------|--------|---------|
| LiDAR 点云 | `lidarObserve` | 点到平面距离 | H = [rot_jac(3) \| trans_jac(3)] |
| RTK 姿态 | `rotationObserve` | SO3 姿态 | H = [I(3) \| 0...] |
| RTK 位置 | `positionObserve` | 3D 位置 | H = [0 \| I(3) \| 0...] |
| RTK 速度 | `velocityObserve` | 3D 速度 | H = [0 \| 0 \| I(3) \| 0...] |

每种观测独立计算卡尔曼增益 `K = P_ * H^T * (H * P_ * H^T + noise)⁻¹`

**后端 Cere 因子图**（`lio_gps_opt.cpp:51-160`）：
- **LIO 相对约束**：`RelativeRTError`（`factors.h:130-192`），约束相邻帧相对位姿
- **GPS 全局约束**：`TError`（`factors.h:28-52`），约束 LIO 轨迹在 GPS 位置附近
- **求解器**：`SPARSE_NORMAL_CHOLESKY` + `HuberLoss(1.0)` + `QuaternionParameterization`
- **异步线程**：`threadOpt` 后台运行，不阻塞前端

### 8.3 iKD-Tree 地图管线

**地图更新**（`map.cpp:125-169` `mapIncremental()`）：
1. 当前帧点云变换到世界坐标系
2. 检查每个点在体素滤波网格中的中点位置
3. 若邻居距离网格中点更近 → 不添加（`needAdd = false`）
4. iKD-Tree 增量插入

**FOV 动态窗口**（`map.cpp:79-118` `lasermapFovSegment()`）：
- 运动超过 `MOV_THRESHOLD * DET_RANGE` 触发地图平移
- 删除离开局部窗口的点：`ikdtreePtr->Delete_Point_Boxes(cubNeedrm)`

### 8.4 时间同步与数据缓冲

**核心机制**（`front.cpp:160-224` `syncMeasureGroupAdd()`）：

```
所有数据按 uint64_t nsec 纳秒时间戳 key 存入 std::map<uint64_t, DataUnit>
map 的自动排序保证时间序列正确性
```

1. 取最前面点云开始时间 `lidarBeginTime`（`front.cpp:166`）
2. 计算点云结束时间：`lidarEndTime = beginTime + 最后一个点的 offset_time`（`front.cpp:167`）
3. 丢弃 `lidarBeginTime` 之前的 IMU 数据（`front.cpp:178-181`）
4. 收集 `[lidarBeginTime, lidarEndTime]` 区间内的所有 IMU（`front.cpp:183-191`）
5. **至少 5 条 IMU 才同步成功**：`if(cnt < 5) return false`（`front.cpp:192`）
6. RTK 和点云也插入到 `measureGroupAdd.mapDatas` 中，形成时间戳排序的测量组

`DataUnit.type` 枚举：`0=IMU`, `1=Lidar`, `2=GPS`, `3=Camera`, `4=Speed`

### 8.5 初始化

**IMU 静态初始化**（`imu_static_init.cpp:11-44`）：
- 收集 `imuInitDuration`（默认 1.0s）内数据
- 平均加速度 → 估计重力方向：`gravity = -mean_acc.normalized() * 9.81`
- 加速度计 bias = 平均加速度 + 重力
- 加速度计尺度标定：`scale = 9.81 / mean_acc.norm()`

**初始位姿**（`front.cpp:101-158`）：若 `initUseRtk = true`，用第一个 RTK 位置/姿态；否则归零

**首帧点云**：`mapPtr->addPcdAndUpdateLocalMap` 初始化 iKD-Tree（`map.cpp:174-195`）

### 8.6 降级

- 无 RTK：位置/姿态/速度观测不可用，退化为纯 LiDAR-IMU 紧耦合
- IMU 数量 < 5：同步失败，跳过当前帧
- 无平面特征：IESKF 观测更新退化（协方差可能发散）