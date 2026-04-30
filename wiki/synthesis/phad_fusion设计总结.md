---
tags: [phad_fusion, SLAM, 因子图, GTSAM, 多传感器融合, 架构设计]
created: 2026-04-28
type: synthesis
---

# phad_fusion 设计总结

> phad_fusion 是一个基于 “Everything is a Factor” 哲学的 GTSAM iSAM2 多传感器融合 SLAM 框架，支持 LiDAR、视觉、IMU、GNSS 的组合应用，提供三种运行模式以适应从低空到越野的多样化场景。

## 一、设计哲学：Everything is a Factor

phad_fusion 的核心设计原则是**将所有传感器观测、运动约束和先验信息统一建模为因子图中的因子（Factor）**。这一哲学的直接收益是：

- **观测独立性**：每个传感器的每次观测作为独立因子加入图，噪声模型与因子一一对应，无需交叉耦合的状态推导
- **传感器可插拔**：新增或移除传感器仅需添加/移除对应因子节点，核心优化器无需修改
- **不确定性透明**：GTSAM 自动进行全因子图协方差恢复，运行时任一变量均可查询边际协方差
- **回环即因子**：回环检测结果作为 BetweenFactor 直接加入因子图，与前端因子同等对待

技术栈选择：**GTSAM iSAM2** 作为后端优化引擎，**C++17** 作为主力实现语言，**ROS 2 Humble** 作为通信中间件。

## 二、三种运行模式与切换机制

phad_fusion 根据传感器配置和应用场景，提供三种模式，支持在线动态切换：

### 模式 1：LIO 模式（LiDAR-IMU Odometry）
- 传感器：LiDAR + IMU
- 因子类型：IMU 预积分因子 + 激光 ICP 因子（点到面/点到线/GICP 自适应）
- 适用场景：弱光、无纹理、隧道、室内走廊
- 参考基线：FAST-LIO2 前端 + iSAM2 后端

### 模式 2：VIO 模式（Visual-IMU Odometry）
- 传感器：相机 + IMU
- 因子类型：IMU 预积分 + 视觉重投影因子（SmartProjectionFactor）
- 适用场景：光照充足、纹理丰富、开阔室外
- 参考基线：open_vins MSCKF 前端 + iSAM2 后端

### 模式 3：LVIO 模式（LiDAR-Visual-IMU Odometry）
- 传感器：LiDAR + 相机 + IMU
- 因子类型：IMU 预积分 + 激光 ICP + 视觉重投影
- 适用场景：全场景，精度最高
- 额外可接入：GNSS 先验因子、轮速计预积分因子

### 切换机制

- **手动切换**：通过 ROS 2 service 或参数配置指定模式
- **自动退化检测**：视觉前端跟踪点数 < 阈值时降级至 LIO；LiDAR 点云稀疏/退化时降级至 VIO
- **无缝衔接**：模式切换时保留 iSAM2 因子图状态，仅改变前端产生的因子类型
- **GNSS 增强**：任意模式下可叠加 GNSS 先验因子，提供全局约束

## 三、传感器抽象设计

### `SensorBase` 基类

所有传感器统一继承 `SensorBase` 抽象接口：

```cpp
class SensorBase {
public:
    virtual bool initialize() = 0;
    virtual std::vector<gtsam::NonlinearFactor::shared_ptr> generateFactors(
        const gtsam::Values& current_estimate) = 0;
    virtual bool isDegraded() const;
    virtual SensorType type() const = 0;
    virtual double informationWeight() const;
};
```

### 关键传感器实现

| 传感器 | 因子类型 | 频率 | 噪声模型 |
|--------|---------|------|----------|
| IMU | CombinedImuFactor / ImuFactor | 200 Hz | 艾伦方差标定 |
| LiDAR | 自定义 IcpFactor (点到面/线) | 10 Hz | 自适应协方差 |
| 单目相机 | SmartProjectionFactor | 30 Hz | 像素级噪声 (1.5 px) |
| 立体相机 | GenericStereoFactor | 30 Hz | 像素级噪声 |
| GNSS | PriorFactor<Pose3> | 1 Hz | 定位精度设定 |
| 轮速计 | BetweenFactor<Pose3> (预积分) | 50 Hz | 里程计噪声 |

### 因子权重自适应

每个因子的 information weight 由传感器实时状态动态调整：LiDAR 在结构化场景权重高，视觉在纹理丰富场景权重高，GNSS 由 HDOP/卫星数决定信任程度。权重差异确保了在部分传感器退化时，其他传感器能够补偿。

## 四、关键 GTSAM 因子类型

| 因子 | GTSAM 类 | 用途 |
|------|----------|------|
| 先验因子 | `PriorFactor<Pose3>`, `PriorFactor<Vector3>` | 初始位姿/速度锚定，GNSS 全局约束 |
| IMU 预积分 | `CombinedImuFactor` / `ImuFactor` | 高频 IMU 运动约束 |
| 激光 ICP | 自定义 `IcpFactor`（点到面/分布） | LiDAR 扫描匹配约束 |
| 视觉重投影 | `SmartProjectionPose3Factor` | 单目特征跟踪约束 |
| 立体视觉 | `GenericStereoFactor` | 双目深度约束 |
| 位姿间约束 | `BetweenFactor<Pose3>` | 回环闭合、里程计增量、外参约束 |
| 匀速模型 | `BetweenFactor<Pose3>` (速度积分) | 运动平滑性先验 |
| 零速更新 | `PriorFactor<Vector3>` (速度=0) | ZUPT 静止检测 |

## 五、初始化多级策略

初始化按传感器可用性采用递进式策略：

1. **Level 0 — 静止初始化**：ZUPT 检测静止状态下初始化 IMU 偏置和重力方向，初始化速度和位姿
2. **Level 1 — IMU+LiDAR 动态初始化**：LiDAR 前 2 秒扫描构建初始局部地图，IESKF 估计初始运动轨迹，IMU 在线估计偏置
3. **Level 2 — IMU+Visual 动态初始化**：参考 VINS 的松耦合初始化：纯视觉 SfM 恢复尺度未知的结构，视觉-IMU 对齐恢复尺度/重力/速度，在线 Refine 重力方向和加速度计偏置
4. **Level 3 — 多传感器联合初始化**：LIO + VIO 联合初始化，交叉验证外参，统一初始化所有 IMU 参数
5. **Level 4 — GNSS 辅助初始化**：GNSS 提供全局先验，直接初始化世界坐标系位姿

初始化失败时自动降级至低一级策略，确保系统始终能进入运行状态。

## 六、四线程模型

phad_fusion 采用四线程并行架构，通过无锁队列和原子变量进行线程间通信：

| 线程 | 职责 | 周期 |
|------|------|------|
| **前端线程** | 传感器数据预处理、去畸变、特征跟踪/ICP 匹配、生成因子 | 按传感器频率 |
| **优化线程** | iSAM2 增量更新、周期性重线性化、缓存整理 | 每次有新因子 |
| **回环线程** | 关键帧数据库维护、回环候选检索、几何验证、回环因子生成 | 按关键帧 |
| **可视化线程** | TF 发布、点云/轨迹/因子图状态发布、Rviz 可视化 | 10 Hz |

### 线程间同步

- **前端 → 优化**：因子和初值存入 thread-safe 队列，优化线程非阻塞消费
- **优化 → 回环**：关键帧信息（位姿 + 描述子 + 局部地图）发布至回环数据库
- **回环 → 优化**：验证通过的回环因子直接加入因子图队列
- **优化 → 可视化**：最新 `gtsam::Values` 快照以原子指针共享，避免拷贝

## 七、为什么是 iSAM2 而非 EKF 或批量 BA

| 考量 | EKF | 批量 BA | iSAM2 (选择) |
|------|-----|---------|-------------|
| 多传感器建模 | 需逐一推导耦合雅可比 | 图建模灵活 | 图建模灵活，增量更新 |
| 线性化质量 | 单次，不可逆 | 多次迭代至收敛 | 选择性重线性化 |
| 实时性 | 快 | 大图不可控 | 增量更新保证实时 |
| 精度 | 漂移累积 | 理论最优 | 接近批量 |
| 边缘化 | 不可逆 | 不边缘化 | Bayes 树可逆边缘化 |
| 不确定性查询 | 协方差实时可用 | 需额外计算 | 自动协方差恢复 |

iSAM2 是唯一同时满足“实时在线 + 因子图建模 + 接近批量精度 + 可逆边缘化 + 自动协方差恢复”五项需求的后端方案。

## 八、预期性能指标

以下为参考 SOTA 方案的预期性能基准（LVIO 模式，EuRoC / KITTI 级别数据集）：

| 指标 | 目标值 | 参考 SOTA |
|------|--------|----------|
| 绝对轨迹误差 (ATE RMSE) | < 0.1 m (室内) / < 1.0 m (室外) | ORB-SLAM3, FAST-LIO-SAM |
| 相对位姿误差 (RPE) | < 0.05 m, < 0.5° | VINS-Fusion, LIO-SAM |
| 单帧处理时间 | < 20 ms (LVIO) | FAST-LIO2 + VINS |
| IMU 偏置收敛 | < 10 s | VINS-Mono |
| 回环召回率 | > 95% | ScanContext, DBoW3 |
| 初始地图大小 | < 500 关键帧 | — |
| 内存占用 | < 500 MB (1km 轨迹) | LIO-SAM |
| 多传感器外参标定误差 | < 0.01 m, < 0.1° | Kalibr, LI-Calib |

## 九、八阶段实施路线图概要

| 阶段 | 内容 | 关键交付物 | 预估周期 |
|------|------|-----------|----------|
| **Phase 1** | 基础架构搭建 | 传感器抽象层、因子图管理器、四线程框架、CMake/ROS2 项目骨架 | 2 周 |
| **Phase 2** | IMU 预积分因子 | IMU 前端处理、CombinedImuFactor 集成、IMU 噪声标定模块 | 2 周 |
| **Phase 3** | LIO 模式实现 | LiDAR 前端（IESKF/ICP）、IcpFactor 自定义因子、LIO 模式上线 | 3 周 |
| **Phase 4** | VIO 模式实现 | 视觉前端（KLT + MSCKF）、SmartProjectionFactor、VIO 模式上线 | 3 周 |
| **Phase 5** | LVIO 模式集成 | LiDAR+视觉+IMU 联合因子图、模式切换机制、退化检测与自动降级 | 2 周 |
| **Phase 6** | GNSS+里程计扩展 | GNSS 先验因子、轮速计 BetweenFactor、ZUPT/零速检测 | 1 周 |
| **Phase 7** | 回环检测系统 | 多模态回环检测、几何验证、回环因子自动注入、因子图回填修复 | 2 周 |
| **Phase 8** | 优化与评估 | 全系统性能调优、ATRE/RPE Benchmark、跨数据集评估、文档与教程 | 2 周 |

总计预估周期：约 17 周（4 个月），视迭代速度可压缩至 12 周。

## 相关页面

- [[VIO方案对比]] — 视觉前端参考方案
- [[LiDAR方案对比]] — 激光前端参考方案
- [[因子图vs滤波]] — 后端优化方法深层对比