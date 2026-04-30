# slam_fusion_core 源码深度分析

> **分析时间**: 2026-04-28  
> **源码位置**: `/home/lin/.local/share/Trash/files/slam_fusion_core/`（已移至回收站）  
> **注意**: 该目录原应位于 `slam_ws/slam_fusion_core/`，但当前处于已删除状态。


## 1. 库概述

### 1.1 定位与目标

`slam_fusion_core` 是一个**纯 C++、无 ROS 依赖**的多传感器融合导航/SLAM **骨架库**。它不是完整的 SLAM 系统，而是一个**基础设施层**，为上层 SLAM 框架（如计划中的 `phad_fusion`）提供：

- **统一的多相机表示**（单目 N=1 到多目 N≥1，双目退化为 N=2）
- **传感器测量类型定义**（IMU、GNSS、视觉）
- **抽象接口层**（IMU 积分器、GNSS 处理器、视觉前端、优化后端）
- **Pipeline 调度骨架**（入队传感器数据、步进驱动后端）

### 1.2 目录/模块结构

```
slam_fusion_core/
├── CMakeLists.txt                          # 构建配置 (C++17, Eigen3)
├── .gitignore                              # 忽略 build/
├── include/slam_fusion_core/
│   ├── types.hpp                           # 基础类型：ImuMeasurement, GnssMeasurement, VisualObservation
│   ├── camera_rig.hpp                      # CameraRig: 多相机外参管理
│   ├── factor.hpp                          # Factor 基类（残差维度占位）
│   ├── optimization_backend.hpp            # IOptimizationBackend 接口
│   ├── pipeline.hpp                        # Pipeline: 传感器数据入队 + 步进驱动
│   ├── sensor_interfaces.hpp               # IImuIntegrator / IGnssHandler / IVisualFrontend 接口
│   └── backends/
│       └── null_optimization_backend.hpp   # 空后端实现（演示用）
├── src/
│   ├── camera_rig.cpp                      # CameraRig 实现
│   └── pipeline.cpp                        # Pipeline 实现
├── examples/
│   └── minimal_pipeline.cpp                # 最小示例（单目+双目 Pipeline 运行）
└── docs/
    └── architecture.md                     # 架构设计文档
```

### 1.3 依赖关系

| 依赖 | 版本/说明 | 用途 |
|------|-----------|------|
| **Eigen3** | 无版本限制 | 所有向量/矩阵/位姿表示 |
| C++17 | 编译标准 | `std::optional`/结构化绑定等 |
| **GTSAM** | **未集成**（计划中） | 目标优化后端 |
| ROS | **零依赖** | 有意设计为纯算法库 |

`CMakeLists.txt:1-22` 展示了极简构建：仅依赖 `Eigen3`，编译两个 `.cpp` 源文件为一个静态库 `slam_fusion_core`。

---

## 2. 类型系统设计

### 2.1 基础时间类型

```cpp
// include/slam_fusion_core/types.hpp:9
using Seconds = double;  // 统一时间戳类型（秒，浮点）
```

### 2.2 传感器测量类型

所有类型定义在 `include/slam_fusion_core/types.hpp:1-31`：

#### IMU 测量

```cpp
struct ImuMeasurement {
  Seconds stamp {};            // 时间戳
  Eigen::Vector3d accel {};    // 加速度计 (m/s²)
  Eigen::Vector3d gyro {};     // 陀螺仪 (rad/s)
};
```

**评析**：
- 极简化设计，仅包含原始 IMU 三轴数据
- **缺失**：无加速度计/陀螺仪偏置（bias）测量；无噪声协方差信息；无 IMU 帧约定注释
- 参考 `open_vins` 的 `ov_core::ImuData`，其包含更多字段（温度、ID 等），但核心测量字段一致

#### GNSS 测量

```cpp
struct GnssMeasurement {
  Seconds stamp {};
  Eigen::Vector3d positionEcef {};  // ECEF 坐标系下的位置
  bool positionValid { false };      // 位置有效性标志
};
```

**评析**：
- 仅支持 ECEF 定位；`VisualObservation` 和 `VisualFrame` 中**未指定坐标帧**
- 字段过于简略，`VisualObservation::featureCount` 仅为计数而非真正的特征点数据
- **缺失**：GNSS 速度（Doppler）、精度因子 DOP、星历数据、基线与双频信息
- 参考 `IC-GVINS` 的 GNSS 类型，其区分伪距/载波相位/RTK 等多种测量类型

需要统一指定坐标帧约定，建议以 ENU 为主（更常见于 VIO/SLAM），并在文档中写明转换接口。

| Sensor | Quantity | Coordinate frame |
|--------|----------|-----------------|
| IMU accel | \(m/s^2\) | body frame |
| IMU gyro | rad/s | body frame |
| GNSS position | m | ECEF（当前硬编码） |
| VisualObservation | — | 无姿态，仅 cameraId + featureCount |

---

## 3. 核心类型

### 3.1 CameraRig（相机组，`camera_rig.hpp:10-26`）

```cpp
class CameraRig {
public:
  explicit CameraRig(std::vector<Eigen::Isometry3d> bodyFromCamera);
  static CameraRig monocular(const Eigen::Isometry3d& bodyFromCam0);
  std::size_t cameraCount() const;
  const Eigen::Isometry3d& bodyFromCamera(std::size_t cameraId) const;
  void validateCameraId(std::uint32_t cameraId) const;
private:
  std::vector<Eigen::Isometry3d> m_bodyFromCamera;
};
```

**关键设计决策**：

| 设计点 | 决策 | 影响 |
|--------|------|------|
| 外参存储 | `bodyFromCamera`（IMU body → 相机） | 与 `open_vins` 约定一致 |
| 工厂方法 | `CameraRig::monocular()` 返回 N=1 | 单目即特化，无需单独类型 |
| 校验 | `validateCameraId` 在 `pushVisual` 时检查 | 运行时安全 |
| 最小相机数 | 至少 1（空构造抛出异常） | 避免非法状态 |

**实现** (`src/camera_rig.cpp:7-24`)：
- 构造函数中空向量检测：`if (m_bodyFromCamera.empty()) throw ...`
- 工厂方法 `monocular` 仅为语法糖：`return CameraRig({bodyFromCam0});`
- 外参校验在 `pushVisual` 时发生（`pipeline.cpp:24-27`）

**当前缺失**：
- 无相机内参（焦距、主点、畸变参数）——这些属于 `camera_rig` 吗？在 `open_vins` 中，内参存储在 `Camera` 对象中，外参在 `CameraRig` 中
- 无时间偏移补偿（相机-IMU 时间同步）
- 无外参可优化标记——对做相机-IMU 在线标定是刚需

### 3.2 Factor 基类（`factor.hpp:1-12`）

```cpp
class Factor {
public:
  virtual ~Factor() = default;
  virtual int residualDimension() const = 0;
};
```

**评析**：这是**纯占位符**。仅提供 `residualDimension()` 虚函数，无雅可比计算、无误差函数、无因子类型枚举、无与 GTSAM/Ceres 的关联。

在 `optimization_backend.hpp:18-19` 中有一处注释：
```cpp
virtual void addFactor(std::shared_ptr<Factor> /*factor*/) {
  // Default: ignore until GTSAM/Ceres adapter exists.
}
```
明确表明“不实现，等待适配器”。

### 3.3 IOptimizationBackend 接口（`optimization_backend.hpp:11-21`）

```cpp
class IOptimizationBackend {
public:
  virtual ~IOptimizationBackend() = default;
  virtual void onStep(uint64_t tick, size_t nImu, size_t nVisual, size_t nGnss) = 0;
  virtual void addFactor(shared_ptr<Factor> factor) { /* 空实现 */ }
};
```

**接口契约**：
- `onStep` 是核心回调，每次 `Pipeline::runStep()` 调用一次，传递各传感器**计数**而非数据
- 后端通过子类实现来**访问并消费 Pipeline 队列中的数据**

### 3.4 传感器处理接口（`sensor_interfaces.hpp:1-33`）

```cpp
class IImuIntegrator {
public:
  virtual void integrate(const ImuMeasurement& m) = 0;
};

class IGnssHandler {
public:
  virtual void handle(const GnssMeasurement& m) = 0;
};

class IVisualFrontend {
public:
  virtual void onObservation(const VisualObservation& obs) = 0;
  virtual void onSyncFrame(const VisualFrame& frame) {
    for (const auto& o : frame) { onObservation(o); }
  }
};
```

**设计评析**：

| 接口 | 输入 | 职责 | 缺失 |
|------|------|------|------|
| `IImuIntegrator` | `ImuMeasurement` | 预积分/传播 | 无偏置估计回调、无输出 state 接口、无 reset |
| `IGnssHandler` | `GnssMeasurement` | GNSS 处理 | 无多频/RTK 支持、无卫星可见性接口 |
| `IVisualFrontend` | `VisualObservation`/`VisualFrame` | 特征处理/跟踪 | 无特征匹配接口、无位姿输出、无关键帧判定 |

### 3.5 Pipeline（管线，`pipeline.hpp:13-41`）

```cpp
class Pipeline {
public:
  Pipeline(shared_ptr<CameraRig> cameraRig, unique_ptr<IOptimizationBackend> backend);
  void pushImu(const ImuMeasurement& m);
  void pushGnss(const GnssMeasurement& m);
  void pushVisual(const VisualObservation& obs);
  void pushVisualFrame(const VisualFrame& frame);
  void runStep();
private:
  deque<ImuMeasurement> m_imuQueue;
  deque<VisualObservation> m_visualQueue;
  deque<GnssMeasurement> m_gnssQueue;
  static constexpr size_t kMaxQueue = 4096;
};
```

**数据流**（`src/pipeline.cpp:7-56`）：

```
传感器 → push*(测量) → deque<测量> → runStep() → backend.onStep(tick, 队列长度)
                   ↓
              trimQueues() 保护
                   ↓
              (最大 4096 条目)
```

**关键行为**：
1. `pushImu` / `pushGnss` / `pushVisual`：各自入队到 `std::deque`
2. `pushVisualFrame`：将 `VisualFrame` 多相机测量**逐个展开**入队
3. `runStep()`：
   - 调用 `m_backend->onStep(m_tick, imu队列大小, visual队列大小, gnss队列大小)`
   - **清空所有队列**
   - tick 自增
4. `trimQueues()`：每入队一次调用，保持队列不超过 4096（弹出前端最旧数据）

**致命缺陷**：
- `runStep()` **仅传递队列长度**给后端，不传递实际数据（测量数据无法在 `onStep` 外访问——后端没有持有 Pipeline 引用，队列是私有成员）
- 因此 `NullOptimizationBackend::onStep` 和 `LoggingBackend::onStep` 只能打印计数
- 除非后端和 Pipeline 共享某种全局状态或后端持有 Pipeline 指针（当前设计中未体现）
- 实际数据消费路径：需要后端实现者在 `onStep` 内部**同时访问 Pipeline 对象**，但这破坏了封装

### 3.6 现有后端实现

**NullOptimizationBackend** (`backends/null_optimization_backend.hpp:7-11`)：
```cpp
class NullOptimizationBackend : public IOptimizationBackend {
public:
  void onStep(uint64_t, size_t, size_t, size_t) override {}
};
```
完全空实现，仅用于满足 Pipeline 构造要求。

**LoggingBackend**（定义在 `examples/minimal_pipeline.cpp:16-22`，不在库代码中）：
```cpp
class LoggingBackend : public IOptimizationBackend {
public:
  void onStep(uint64_t tick, size_t nImu, size_t nVisual, size_t nGnss) override {
    cout << "tick=" << tick << " imu=" << nImu ...;
  }
};
```
仅打印计数，无法访问实际数据。

---

## 4. 核心 API 设计

### 4.1 对外暴露的公共 API

```cpp
// 创建多相机配置
auto rig = make_shared<CameraRig>(vector<Isometry3d>{...});

// 创建管线
Pipeline pipe(rig, make_unique<SomeBackend>());

// 推送传感器数据
pipe.pushImu({stamp, accel, gyro});
pipe.pushGnss({stamp, ecefPos, true});
pipe.pushVisual({cameraId, stamp, featureCount});
pipe.pushVisualFrame({ ... });  // 批量同步帧

// 访问相机外参
const auto& T = pipe.cameraRig().bodyFromCamera(camIdx);

// 查询队列状态
size_t n = pipe.imuQueueSize();
```

### 4.2 预期使用模式

```
┌─────────────────────────────┐
│  ROS 2 Wrapper（独立包）     │
│  订阅话题 → 转换为 SFC 类型  │
└──────────┬──────────────────┘
           │ pushImu/pushGnss/pushVisual
┌──────────▼──────────────────┐
│  Pipeline                    │
│  队列管理 + 步进调度         │
└──────────┬──────────────────┘
           │ onStep(tick, nImu, nVisual, nGnss)
┌──────────▼──────────────────┐
│  IOptimizationBackend        │
│  自定义实现（如 GTSAM/Ceres）│
└─────────────────────────────┘
```

上层 SLAM 系统按需实现 `IImuIntegrator`、`IGnssHandler`、`IVisualFrontend`，并通过 Pipeline 汇聚到统一优化后端。

---

## 5. GTSAM 集成分析

### 5.1 当前状态：零集成

`factor.hpp` 中的 `Factor` 类与 GTSAM 无任何关联：

- **无 GTSAM 头文件包含**（`CMakeLists.txt` 中无 `find_package(GTSAM)`）
- **无 GTSAM 因子类继承**（GTSAM 因子须继承 `gtsam::NoiseModelFactor` 或 `gtsam::NonlinearFactor`）
- **无 GTSAM 节点类型**（`gtsam::Pose3`、`gtsam::Values` 未出现）

`optimization_backend.hpp:18-19` 明确注释：
> "Default: ignore until GTSAM/Ceres adapter exists."

### 5.2 架构文档中的 GTSAM 规划

`docs/architecture.md:27-28`：
> 实现 `IOptimizationBackend`，在 `onStep` 内从队列取出测量并写入 GTSAM `NonlinearFactorGraph`（或 Ceres `Problem`），勿在头文件中暴露 GTSAM，可仅在前向声明 + `.cpp` 链接。

**这一设计原则是合理的**：将 GTSAM 封装在 `.cpp` 中，头文件保持后端无关。

### 5.3 需要新增的 GTSAM 因子类型

根据任务文档 `任务.md` 要求，`phad_fusion` 需要以下因子（全部缺失于 `slam_fusion_core`）：

| 因子类型 | GTSAM 基类 | 所需数据 | slam_fusion_core 状态 |
|----------|-----------|---------|----------------------|
| IMU 预积分因子 | `CombinedImuFactor` | 偏置、积分结果 | 无（无 `IImuIntegrator` 实现） |
| 重投影因子 | `GenericProjectionFactor` | 3D 点、2D 测量、内外参 | 无（无地图点/特征类型） |
| GNSS 位置因子 | `PriorFactor<Pose3>` | ECEF → ENU 转换 | 无（仅有测量结构体） |
| 轮速因子 | `BetweenFactor<Pose3>` | 轮速预积分 | 无对应测量类型 |
| ZUPT 因子 | `PriorFactor<Vector3>` | 零速度检测 | 无 |
| 平面约束 | 自定义单目因子 | 地面平面参数 | 无 |

### 5.4 与 gtsam_points 的对齐

`gtsam_points` 项目提供了丰富的点云/配准类 GTSAM 因子扩展。如果 `phad_fusion` 的目标是激光+视觉+惯性的紧耦合融合，则需要：

1. 从 `gtsam_points` 学习其因子设计模式（`gtsam_points/factors/*`）
2. 在 `slam_fusion_core` 中建立 `CameraRig` → GTSAM `Cal3_S2` + `Pose3` 的转换桥梁
3. 实现从 `VisualObservation`（当前仅 featureCount）到实际 `Keypoint` + `Descriptor` 的完整视觉类型链

---

## 6. 与 phad_fusion 的关系

### 6.1 当前状态

**`phad_fusion` 项目尚未创建**。`slam_fusion_core` 是为其构建的基础设施层（已移至回收站）。

根据 `docs/framework_comparison.md:33,40`：
> **GTSAM 系**：`IC-GVINS`、`OB_GINS`、`FAST-LIO-SAM-SC-QN`、`lt-mapper`、`gtsam_points` — 适合作为 `slam_fusion_core` 第二迭代的对接目标。  
> **本仓库新库**：`slam_fusion_core` 用 `CameraRig` + `cameraId` 统一 N≥1。

### 6.2 设计意图

```
phad_fusion（上层 SLAM 系统）
    ↓ 使用
slam_fusion_core（基础设施库）
    ↓ 引用
GTSAM + Eigen3（数学后端）
```

`slam_fusion_core` 是 `phad_fusion` 的**类型系统 + 接口约定层**。`phad_fusion` 负责：
- 实现具体的前端算法（特征提取、匹配、跟踪）
- 实现具体的后端优化（GTSAM 因子图构建与求解）
- 管理地图（关键帧、地图点）
- 闭环检测与全局优化

### 6.3 slam_fusion_core 已提供 vs phad_fusion 需要

| 需求 | slam_fusion_core 状态 | 缺失程度 |
|------|----------------------|---------|
| **多相机外参管理** | ✅ CameraRig 完整实现 | 已完成 |
| **IMU 测量类型** | ✅ ImuMeasurement （基础） | 需扩展偏置/协方差 |
| **GNSS 测量类型** | ⚠️ GnssMeasurement （仅 ECEF） | 需大幅扩展 |
| **视觉测量类型** | ⚠️ VisualObservation （仅计数） | 几乎未开始 |
| **IMU 预积分** | ❌ 仅接口，无实现 | 完全缺失 |
| **特征/描述子** | ❌ 无类型定义 | 完全缺失 |
| **地图点** | ❌ 无类型定义 | 完全缺失 |
| **关键帧** | ❌ 无类型定义 | 完全缺失 |
| **GTSAM 因子** | ❌ 仅占位基类 | 完全缺失 |
| **优化后端** | ❌ 仅接口 | 完全缺失 |
| **闭环检测** | ❌ 无 | 完全缺失 |
| **轮速/ZUPT** | ❌ 无测量/接口 | 完全缺失 |
| **Pipeline 数据流** | ⚠️ 骨架存在，但数据不可达 | 需要修复 |

### 6.4 架构决策对 phad_fusion 的影响

**正面**：
- 纯 CMake/C++17 无 ROS 依赖，单元测试友好
- `CameraRig` 的单目/多目统一抽象是正确方向
- 接口分层（`sensor_interfaces.hpp` + `optimization_backend.hpp`）提供了清晰的扩展点

**问题**：
- Pipeline 数据流设计有根本缺陷（数据无法到达后端），`phad_fusion` 需要修复或替换 Pipeline
- 视觉类型链完全缺失（特征点、描述子、相机内参），`phad_fusion` 需要从零构建
- 因子系统几乎不存在，无法直接支持 GTSAM 集成

---

## 7. 优缺点分析

### 7.1 设计质量评估

| 维度 | 评分 | 说明 |
|------|------|------|
| **模块化** | ★★★★☆ | 8 个头文件各司其职，职责清晰，无循环依赖 |
| **可扩展性** | ★★★★☆ | 接口类 + 虚函数模式，易于插入新实现 |
| **ROS 松耦合** | ★★★★★ | 零 ROS 依赖，完全独立可测试 |
| **类型完整性** | ★☆☆☆☆ | 仅定义了最基础的骨架，核心类型大量缺失 |
| **可运行性** | ★★☆☆☆ | 可以编译运行，但实际不做任何优化/估计 |
| **文档** | ★★★☆☆ | 有 `architecture.md` 说明意图，但无 API 文档 |
| **正确性** | ★★★☆☆ | 逻辑正确但有数据流设计缺陷 |

### 7.2 主要问题

#### 数据流断裂（致命）

`Pipeline::runStep()` 仅传递队列**大小**到后端的 `onStep`，后端无法访问实际测量数据。见 `pipeline.cpp:38-43`。这是设计上最根本的问题。解决方案：

1. 在 `onStep` 签名中加入对队列的 const 引用/span
2. 或将后端设计为模板类，持有对 Pipeline 队列的访问
3. 或引入事件总线模式（observer/observable）

#### 视觉类型链完全缺失

`VisualObservation::featureCount` 为 `int` 类型，仅表示“检测到多少个特征点”。实际 SLAM 系统需要的远不止此：

- 特征点位置（亚像素坐标 + 去畸变归一化坐标）
- 描述子（`cv::Mat` 或自定义类型）
- 关键帧管理（参考帧选择策略）
- 地图点（3D 位置 + 观测射线）

**参考**：`open_vins` 中 `ov_core::Feature` 包含 `uv`（像素）、`uv_norm`（归一化）、`featid`、`timestamp` 等完整字段。

#### IMU 类型过于简化

`ImuMeasurement` 缺少：
- IMU 噪声模型（加速度计/陀螺仪噪声密度、随机游走）
- 偏置初值（`ba0`/`bg0`）
- 时间偏移（IMU-CAM 同步误差，对 VIO 性能影响巨大）

**参考**：`OB_GINS`/`gtsam_points` 的 IMU 数据结构包含完整的噪声参数，便于传递给 GTSAM 预积分器。

#### GNSS 类型过于简化

`GnssMeasurement` 仅支持 ECEF 位置。实际 GNSS-惯性紧耦合需要：
- 速度测量（Doppler）
- 伪距/载波相位（用于 RTK/PPP）
- 精度因子（HDOP/VDOP）
- 卫星星历、对流层延迟模型
- 多星座支持（GPS/GLONASS/Galileo/BDS）

**参考**：`IC-GVINS` 的 GNSS 处理区分了原始测量和观测误差模型，支持多频点。

#### 缺少在线标定支持

CameraRig 中 `m_bodyFromCamera` 存储为 `std::vector<Isometry3d>`（值语义），没有标记哪些外参待标定。在线标定系统需要能够将外参作为 GTSAM 图节点，并加入先验约束。

#### 无线程安全

Pipeline 的 `deque` 操作无任何锁保护。如果传感器数据来自多线程（实际工程必然如此），会导致数据竞争。需要引入 `std::mutex` 或使用 `lock-free` 队列。

#### 缺少序列化支持

大型 SLAM 系统需要保存/加载地图状态。当前无可持久化接口。建议添加：
- Protobuf/FlatBuffers 序列化
- YAML/JSON 配置读写

### 7.3 优点

1. **极低的编译门槛**：仅依赖 Eigen3，C++17 标准。无需复杂的依赖管理，新人可在 5 分钟内拉通编译。
2. **正确的架构分层**：接口与实现分离，后端适配器模式清晰，为后续 GTSAM/Ceres 集成留好了扩展点。
3. **CameraRig 设计优秀**：`vector<Isometry3d>` 而非“双目特化”，工厂方法 `monocular()` 返回 N=1——这是经验丰富的多传感器 SLAM 研究者才会做的设计选择。
4. **文档先行**：在代码极不完善时就写出了 `architecture.md`，明确了“等 GTSAM/Ceres 适配器来填充后端”的开发意图，这种规划方式值得保留。

### 7.4 对 phad_fusion 的启动建议

#### 阶段 1：修复 Pipeline 数据流

```cpp
// 方案：让后端直接接收 span<测量>
virtual void onStep(uint64_t tick, 
                    span<const ImuMeasurement> imuData,
                    span<const VisualObservation> visualData,
                    span<const GnssMeasurement> gnssData) = 0;
```

#### 阶段 2：充实视觉类型

参考 `open_vins::Feature` 结构，在 `slam_fusion_core` 中添加：

```cpp
struct Feature {
  Eigen::Vector2d uv;          // 去畸变像素坐标
  Eigen::Vector2d uv_norm;     // 归一化平面坐标
  size_t featId;               // 跨帧唯一 ID
  float response;              // 响应强度
};

struct Descriptor {
  std::vector<float> data;     // 浮点描述子（如 SuperPoint 256d）
  enum Type { ORB, SURF, SUPERPOINT, NETVLAD };
  Type type;
};
```

#### 阶段 3：引入 GTSAM

```cpp
// 在 backends/ 下实现 GTSAM 因子适配器
class GtsamBackend : public IOptimizationBackend {
  gtsam::NonlinearFactorGraph m_graph;
  gtsam::Values m_initial;
  void onStep(...) override {
    // 从测量数据构建因子并加入图
    // 调用 gtsam::LevenbergMarquardtOptimizer 求解
  }
};
```

#### 阶段 4：参考现有实现

- **视觉因子**：参考 `open_vins/src/update/UpdaterSLAM.cpp` 的 `Feature::computeReprojectionError()`
- **IMU 预积分**：参考 `gtsam_points/src/integration/imu_integrator.cpp` 的 `gtsam::PreintegratedImuMeasurements`
- **GNSS 紧耦合**：参考 `IC-GVINS/src/factors/gnss_psr_factor.cpp` 的伪距因子实现

---

## 附录 A：文件清单

| 文件 | 行数 | 语言 | 关键内容 |
|------|------|------|----------|
| `CMakeLists.txt` | 22 | CMake | 项目配置、Eigen3 依赖 |
| `include/slam_fusion_core/types.hpp` | 31 | C++ | ImuMeasurement, GnssMeasurement, VisualObservation |
| `include/slam_fusion_core/camera_rig.hpp` | 28 | C++ | CameraRig 类声明 |
| `include/slam_fusion_core/factor.hpp` | 12 | C++ | Factor 基类（残差维度） |
| `include/slam_fusion_core/optimization_backend.hpp` | 23 | C++ | IOptimizationBackend 接口 |
| `include/slam_fusion_core/pipeline.hpp` | 43 | C++ | Pipeline 类（队列 + 步进） |
| `include/slam_fusion_core/sensor_interfaces.hpp` | 33 | C++ | IImuIntegrator, IGnssHandler, IVisualFrontend |
| `include/slam_fusion_core/backends/null_optimization_backend.hpp` | 13 | C++ | NullOptimizationBackend（空实现） |
| `src/camera_rig.cpp` | 25 | C++ | CameraRig 实现 |
| `src/pipeline.cpp` | 58 | C++ | Pipeline 实现 |
| `examples/minimal_pipeline.cpp` | 53 | C++ | 最小示例 |
| `docs/architecture.md` | 34 | Markdown | 架构说明 |
| `.gitignore` | 1 | — | 忽略 build/ |

**总计**：约 375 行代码（含注释和空白），6 个头文件 + 2 个实现文件 + 1 个示例。

---

## 附录 B：关键 API 签名速查

```cpp
// CameraRig
CameraRig::CameraRig(std::vector<Eigen::Isometry3d> bodyFromCamera);
static CameraRig CameraRig::monocular(const Eigen::Isometry3d& bodyFromCam0);
std::size_t CameraRig::cameraCount() const;
const Eigen::Isometry3d& CameraRig::bodyFromCamera(std::size_t cameraId) const;
void CameraRig::validateCameraId(std::uint32_t cameraId) const;

// Pipeline
Pipeline::Pipeline(std::shared_ptr<CameraRig> rig, std::unique_ptr<IOptimizationBackend> backend);
void Pipeline::pushImu(const ImuMeasurement& m);
void Pipeline::pushGnss(const GnssMeasurement& m);
void Pipeline::pushVisual(const VisualObservation& obs);
void Pipeline::pushVisualFrame(const VisualFrame& frame);
void Pipeline::runStep();
const CameraRig& Pipeline::cameraRig() const;

// IOptimizationBackend
virtual void IOptimizationBackend::onStep(uint64_t tick, size_t nImu, size_t nVisual, size_t nGnss) = 0;
virtual void IOptimizationBackend::addFactor(std::shared_ptr<Factor> factor);

// Sensor interfaces
virtual void IImuIntegrator::integrate(const ImuMeasurement& m) = 0;
virtual void IGnssHandler::handle(const GnssMeasurement& m) = 0;
virtual void IVisualFrontend::onObservation(const VisualObservation& obs) = 0;
virtual void IVisualFrontend::onSyncFrame(const VisualFrame& frame);

// Factor
virtual int Factor::residualDimension() const = 0;
```