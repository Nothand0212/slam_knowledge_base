---
tags: [4D毫米波雷达, 雷达SLAM, ICP, 回环检测]
sources:
  - wiki/sources/2026-04-28-4d-radar-slam-analysis.md
  - wiki/sources/2026-04-29-special-sensors-comparison.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# 4DRadarSLAM

> 基于4D毫米波雷达点云（含多普勒速度）的点云配准SLAM系统，使用APDGICP前端配准与ISC回环检测，支持大尺度室外环境鲁棒定位与建图。

## 核心管线

4DRadarSLAM 面向 4D 成像毫米波雷达，每帧点云包含 3D 位置、反射强度/RCS 和径向多普勒速度。系统先用多普勒观测估计雷达自速度，并把该速度用于动态目标过滤、运动畸变补偿和 ICP 初值生成；随后使用 APDGICP 建立带各向异性传感器噪声的点云配准；最后通过强度版 ScanContext 做回环候选检索，并把通过几何验证的回环加入后端图优化。

雷达点云相对 LiDAR 更稀疏，且方位角、俯仰角和距离方向的噪声差异明显。APDGICP 的价值在于不把所有点当成同方差点，而是把雷达硬件误差模型折算进协方差，使配准残差更符合传感器物理特性。

## 关键设计

- 多普勒自速度：用静态目标的径向速度估计 3D 自运动，同时剔除动态目标。
- APDGICP 配准：在点到分布和点到平面之间自适应选择约束，输出相对位姿和不确定性。
- ISC 回环：用雷达强度/SNR 替代 LiDAR 高度值，适配稀疏雷达扫描。
- 多层验证：描述子召回后仍需要 ICP fitness、里程计一致性和图一致性过滤。
- 后端图优化：融合连续里程计边、回环边，以及可选的 GPS/气压计/IMU 约束。

## 适用边界

4D 雷达的优势是雨雾烟尘中的鲁棒性和多普勒速度观测；短板是点云稀疏、多径反射和角分辨率低。因此它更适合作为全天候定位传感器或 LiDAR/Camera 的互补来源，而不是直接套用 LiDAR SLAM 的所有假设。

## 相关页面
- [[方法-4DRadarSLAM工程]]
- [[2026-04-28-4d-radar-slam-analysis]]
- [[传感器-Doppler 自速度估计]]
- [[方法-APDGICP 自适应概率分布 GICP]]
- [[方法-Intensity Scan Context]]
- [[方法-五重回环几何验证]]
