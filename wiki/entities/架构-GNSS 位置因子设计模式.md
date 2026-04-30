---
tags: [GNSS, 位置因子, 组合导航, 因子图, GTSAM]
sources:
  - wiki/sources/2026-04-28-ic_gvins-analysis.md
  - wiki/sources/2026-04-29-gnss-pipeline-comparison.md
created: 2026-04-29
updated: 2026-04-29
type: entity
---

# GNSS 位置因子设计模式

> 在因子图中将 GNSS 位置测量建模为位置因子的设计和实现模式，常见于 VIO/GNSS 组合导航系统。

## 核心设计

GNSS 位置因子将 GNSS 接收机输出的 ENU 或 ECEF 坐标与因子图中对应时刻的机器人位姿对齐，作为一元先验因子（`PriorFactor<Pose3>`）或位置因子（`GPSFactor`）加入优化图。

## 关键模式

- **一阶位置因子**：直接测量 ENU 位置，`GPSFactor` 仅约束 xyz
- **偏航辅助**：双天线 GNSS 提供航向角约束，降低偏航漂移
- **紧耦合 GNSS**：使用原始伪距/载波相位而非解算位置，精度更高但实现复杂
- **协方差自适应**：根据 GNSS 精度（HDOP/VDOP/卫星数）动态调整因子噪声
- **WGS84 坐标转换**：需要将 GNSS 坐标转换到局部 ENU 坐标系

## 代表实现

- VINS-Fusion：GPS 松耦合位置因子
- IC-GVINS：GNSS-VIO 紧耦合位置因子
- LIO-SAM：GPS 位置因子辅助 LiDAR-IMU

## 设计边界

GNSS 位置因子本质是全局锚点，不能无条件强约束轨迹。城市峡谷、多径和遮挡会让位置解出现非高斯误差；如果仍按小协方差加入后端，会把整张图拉偏。工程上应根据状态解类型、卫星数、DOP、RTK fix/float 状态和残差一致性动态调整噪声。

## 与坐标系的关系

GNSS 因子必须明确测量点是天线相位中心、IMU body，还是车辆参考点。杆臂外参错误会表现为随姿态变化的位置误差。所有 GNSS 测量也应先转换到同一个局部 ENU/NED 原点，避免在优化器里混用 WGS84 经纬高和局部笛卡尔坐标。

## 相关页面

- [[算法-IC-GVINS]]
- [[架构-坐标系管理]]
- [[架构-多传感器融合架构]]
