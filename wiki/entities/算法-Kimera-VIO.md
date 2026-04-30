---
tags: [算法, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-kimera_vio-analysis.md
---
# Kimera-VIO

> MIT-SPARK 的双目惯性 VIO 系统，使用 GTSAM iSAM2、SmartStereoFactor 和多线程 pipeline 组织前端、后端、建图和回环。

## 核心管线

Kimera-VIO 是 stereo + IMU 紧耦合 VIO。前端用 GFTT/Shi-Tomasi 角点、KLT 光流、双目匹配和 OpenGV RANSAC 建立可靠特征轨迹；后端用 GTSAM iSAM2 / FixedLagSmoother 做增量因子图优化；视觉观测通过 [[方法-SmartStereoFactor]] 内部三角化并消去路标变量；IMU 通过预积分因子提供短时运动约束。

系统还把 VIO 和建图组织成多线程 Pipeline Module：Frontend、Backend、Mesher、LoopClosure、Display 等模块通过队列传递数据。Delaunay Mesh 重建和点面因子让 Kimera 不只是定位系统，也能输出几何网格。

## 工程特点

- SmartStereoFactor 减少显式路标变量，适合增量平滑。
- OnlineGravityAlignment 处理初始化阶段的重力方向和陀螺偏置。
- 支持在线 IMU-Camera 时间对齐，对真实传感器延迟更友好。
- Pipeline 模块化清晰，但需要控制 feature tracks 和队列长度，避免长时间运行内存增长。

## 适用边界

Kimera-VIO 更适合双目 + IMU 条件良好的室内外场景，尤其是需要在线 mesh 的 AR/VR 或机器人感知任务。它仍依赖稳定纹理、准确双目标定和 IMU 时间同步；强动态、低纹理和曝光剧烈变化会直接影响前端轨迹质量。

## 相关
- [[方法-SmartStereoFactor]]
- [[方法-OnlineGravityAlignment]]
- [[组件-GTSAM|GTSAM iSAM2]]
- [[VIO方案对比|VIO对比分析]]
