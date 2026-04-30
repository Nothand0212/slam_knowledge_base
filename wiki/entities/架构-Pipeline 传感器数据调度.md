---
type: entity
tags: [数据管理, 调度, 队列, 设计模式]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-slam_fusion_core-analysis.md
  - wiki/sources/2026-04-29-fusions_slam-analysis.md
---

# Pipeline 传感器数据调度

> 多传感器 SLAM 前端的输入调度层：负责按时间组织 IMU、LiDAR、相机、GNSS 等异步测量，并向估计器提供完整、可追溯的测量窗口。

## 定义

Pipeline 传感器数据调度不是简单的 ROS callback 转发，而是估计器前面的时序控制层。它要决定哪些测量属于同一个处理窗口、如何处理不同频率的数据、窗口不完整时是否等待，以及是否保留原始测量供后端访问。

## 常见设计模式

| 模式 | 代表工程 | 做法 | 风险 |
|------|----------|------|------|
| 多队列缓存 | slam_fusion_core | IMU/GNSS/Visual 各自进入 `std::deque`，统一由 `runStep()` 驱动后端 | 若只传数量不传数据，后端无法真实消费测量 |
| 时间戳有序容器 | fusions_slam `MeasureGroupAdd` | 使用 `std::map<uint64_t, DataUnit>` 按纳秒时间戳自动排序，`DataUnit.type` 区分 IMU/LiDAR/GPS/Camera/Speed | 容器统一但类型分发复杂，窗口条件必须明确 |
| 帧驱动窗口 | LIO/VIO 常见实现 | 以 LiDAR 帧或相机关键帧为处理触发，收集其时间范围内的 IMU | 高频 IMU 插值、边界点和延迟处理容易出错 |

## fusions_slam 案例：MeasureGroupAdd

`MeasureGroupAdd` 不适合作为独立方法页，它的价值在于展示一种时间排序缓冲模式：

- 所有传感器数据统一包装为 `DataUnit`。
- 使用纳秒时间戳作为 `std::map` key，天然得到全局时间顺序。
- `DataUnit.type` 预留 IMU、LiDAR、GPS、Camera、Speed 等类型扩展。
- 同步条件要求 IMU 数量不少于 5 条，并且 IMU 时间窗口覆盖完整 LiDAR 帧。

这个设计比“每个传感器一个队列”更容易做全局排序，但也更依赖清晰的类型分发和窗口完整性检查。

## slam_fusion_core 案例

slam_fusion_core 中的调度骨架通过三个 `std::deque` 队列接收 IMU/GNSS/Visual 测量，`runStep()` 驱动后端 `onStep()` 回调：

- 三个独立队列：`m_imuQueue`、`m_visualQueue`、`m_gnssQueue`。
- 各队列容量上限 4096，`trimQueues()` 超出时 `pop_front` 丢弃最旧数据。
- `pushVisualFrame()` 将多相机帧展开入队。
- 当前设计的主要缺陷是 `onStep()` 只传队列大小而非实际数据，后端无法访问测量值。

更合理的接口应传递测量窗口本身，例如 `onStep(span<const ImuMeasurement>, span<const VisualObservation>, span<const GnssMeasurement>)`。

## 设计检查项

- 时间戳单位必须统一，避免秒、毫秒、纳秒混用。
- 触发条件应由低频主传感器定义，例如 LiDAR 帧或相机关键帧。
- 高频 IMU 要覆盖窗口两端，必要时保留边界外的插值样本。
- 后端接口应接收实际测量，不只接收计数或状态位。
- 丢包、乱序和延迟到达要有可观测日志，否则很难定位估计器发散原因。

## 相关页面

- [[组件-slam_fusion_core]]
- [[算法-fusions_slam]]
- [[架构-后端适配器模式]]
- [[架构-多传感器融合架构]]
- [[IMU数据管线]]
