# 定位相关子项目框架对比

对比维度：**ROS 耦合**、**优化/滤波后端**、**因子扩展**、**状态与传感器**、**并发模型**、**依赖体量**。路径均相对于 `slam_ws/`。

| 项目 | ROS 耦合 | 后端 | 因子/残差扩展 | 状态表示要点 | 并发 | 备注 |
|------|----------|------|----------------|--------------|------|------|
| open_vins | ROS 节点 + `ov_*` 库；核心算法可脱离 launch 单独链 | MSCKF / EKF 类滑动窗口 | 改 MSCKF 更新与类型系统，扩展门槛中高 | 克隆体、相机状态、SLAM 特征 | 多线程异步 | 单目/双目/多相机；GNSS 非主路径 |
| SchurVINS | ROS + 自研前端链路 | 轻量 EKF + Schur 等价观测 | 绑定论文残差形式 | 与 SVO/前端强相关 | 多线程 | 偏视觉惯性轻量 |
| msckf_vio | ROS 紧耦合节点 | MSCKF | 经典 MSCKF 扩展 | 多状态约束 | 实时线程 | 立体+IMU 参考实现 |
| OpenMAVIS | 基于 ORB-SLAM3 结构 | 优化 + 预积分（ORB-SLAM3 系） | 改 ORB 系 KeyFrame/因子需熟悉大图 | 多相机 + IMU | 多线程 | 多目+VIO 概念验证 |
| IC-GVINS | ROS + 因子图 | GTSAM | GTSAM 因子插件化程度高 | GNSS/VIO/IMU 紧耦合 | 多线程 | **GNSS+VIO 后端重要参考** |
| OB_GINS | ROS / 组合导航 | GTSAM / 图优化思路 | 因子类型丰富 | GNSS、INS、ODO 等 | 依配置 | 组合导航教学与工程 |
| fusions_slam | `wrapper/ros` + `fusion_slam` 核 | IESKF + 自研后端模块 | 模块化 `modules/back` | LiDAR + 外接观测 | 依实现 | **ROS 与算法分层清晰** |
| FAST-LIO-SAM-SC-QN | catkin 多包 | FAST-LIO + GTSAM PGO | 回环边、ScanContext、Quatro 可换 | LiDAR 关键帧 | 多线程 | 适合学 GTSAM 拼图 |
| lightning-lm | ROS 2 | 自研 miao + 地图/闭环 | 非 GTSAM，扩展需跟 miao | 激光+IMU | 在线/离线模式 | 工程化激光 SLAM |
| genz-icp | 独立库/节点依仓库 | ICP 变体 | 算法内权重/退化 | 位姿+点云 | 通常单线程为主 | 激光前端 |
| gtsam_points | 库 | GTSAM | **点云/配准因子扩展参考** | 连续时间/体素等 | 可选并行 | 与激光紧耦合 |
| lt-mapper | ROS + `ltslam`/`ltremovert` | GTSAM 等 | 会话因子、全局优化 | 多会话地图 | 多模块 | 长周期地图 |
| BEV-LSLAM | ROS | 依论文实现 | 中等 | BEV + 激光相关 | - | 表征创新 |
| ROLO | ROS LiDAR SLAM | 优化+子图 | 运动约束等 | 地面车辆 | - | 崎岖地形垂向漂移 |
| SuperOdom | 多传感器融合 | 因子图/滤波混合（见论文） | 中 | 多源 | - | 预印本+代码 |

## 分项说明（精简）

### ROS 松耦合（算法核可单独编译测试）

- **较好**：`fusions_slam`（`include/wrapper/ros` 外为核）、`gtsam_points`（库）、`genz-icp`（偏库）。
- **中等**：`open_vins`（库化程度高但仍以 ROS 运行为主）、`IC-GVINS` / `OB_GINS`（ROS 驱动但图优化核清晰）。
- **较紧**：典型 `catkin` 单节点大仓库需拆包才好单测。

### 扩展优化因子（新增残差/边的便利度）

- **GTSAM 系**：`IC-GVINS`、`OB_GINS`、`FAST-LIO-SAM-SC-QN`、`lt-mapper`、`gtsam_points` — 适合作为 `slam_fusion_core` 第二迭代的对接目标。
- **自研后端**：`lightning-lm`（miao）、`fusions_slam`（自定义 back 模块）— 需读内部 API。
- **MSCKF/EKF**：`open_vins`、`SchurVINS`、`msckf_vio` — 扩展以「观测模型+状态增广」为主，与因子图路径不同。

### 视觉相机数扩展

- **显式多相机**：`OpenMAVIS`、`open_vins`（配置多相机）。
- **本仓库新库**：`slam_fusion_core` 用 `CameraRig` + `cameraId` 统一 N≥1。

### GNSS

- **主战场**：`IC-GVINS`、`OB_GINS`；其余多为激光或纯 VIO。

---

更新策略：若某项目大版本变更，请同步修改对应行并注明日期。
