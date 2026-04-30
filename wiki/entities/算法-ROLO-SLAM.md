---
type: entity
tags: [LiDAR SLAM, 崎岖地形, 车辆, 旋转优化, SO(3)]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-rolo-analysis.md
---

# ROLO-SLAM

> 面向越野/崎岖地形的 LiDAR-only SLAM，核心是分离式旋转-平移估计、RotVGICP 和地面车辆运动约束。

## 定义

JFR 2025 发表，专为地面车辆在不平坦地形（越野、山地、矿区）设计的 LiDAR-Only SLAM 系统。核心创新为分离式旋转/平移估计 + 连续时间平移约束。

## 核心特征

- 四个 ROS1 节点管道：imageProjection → featureExtraction → lidarOdometry → backMapping
- **分离式位姿估计**：先平移插值预对齐 → RotVGICP 旋转匹配 → 连续时间平移约束
- 特征提取：LOAM 风格 6 扇区角点+平面点+地面点，每扇区 20 角点
- 前端旋转：RotVGICP 在 SO(3) 流形上做高斯-牛顿
- 后端：GTSAM ISAM2 + 地面车辆约束（roll/pitch/z 限幅）
- 回环：距离搜索 + ICP 验证（fitness_score < 0.3）
- 支持 Velodyne/Ouster，纯 LiDAR 模式下可无 IMU 运行

## 相关页面

- [[方法-RotVGICP]]
- [[方法-分离式旋转-平移估计]]
- [[方法-地面车辆运动约束]]
- [[LiDAR方案对比]]
- 对比：[[2026-04-28-superodom-analysis|SuperOdom]]（松散耦合多传感器）
