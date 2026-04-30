---
tags: [RGB着色, LiDAR, 视觉, 3D重建, R3LIVE]
sources:
  - wiki/sources/2026-04-29-r3live_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# RGB着色点云

> R3LIVE 首创的实时 RGB 着色 LiDAR 点云：LiDAR 点附着多视图 RGB 观测，通过协方差加权的颜色融合。

## 核心用途

RGB 着色点云把 LiDAR 的几何稳定性和相机的颜色信息绑定到同一个地图点上。对机器人来说，它不仅是可视化增强，也能为语义理解、人工验收、离线网格重建和多传感器标定检查提供更直观的地图输出。

## 着色流程
1. LIO 去畸变后点云通过 `RGBpointBodyToWorld()` 转换到世界系
2. `append_points_to_global_map()` 加入全局 RGB 地图
3. 每个 `RGB_pts` 维护多视图 RGB 观测和 RGB 协方差
4. `service_pub_rgb_maps()` 分多个 ROS topic 发布着色点云

## 后续处理
- `m_mvs_recorder` 输出 MVS 格式用于离线网格重建
- 支持离线建图模式重新处理已记录的 RGB 地图

## 关键依赖

颜色融合质量依赖相机-LiDAR 外参、时间同步、相机曝光和遮挡处理。若点云位姿漂移或相机曝光不稳定，同一 3D 点会融合到不一致颜色。协方差加权可以降低异常观测影响，但不能替代动态物体剔除和准确标定。

## 相关页面

- [[算法-R3LIVE]]
- [[架构-多传感器融合架构]]
- [[相机数据管线]]
- [[LiDAR数据管线]]
