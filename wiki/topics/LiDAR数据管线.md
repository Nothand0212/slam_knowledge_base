---
tags: [LiDAR, LiDAR管线, 数据管线, 传感器]
created: 2026-04-29
updated: 2026-05-02
sources:
  - 2026-04-29-lidar-pipeline-comparison
  - wiki/sources/2026-05-02-p2v-slam.md
---

# LiDAR 数据管线

> 15 个 LiDAR SLAM 项目的深度分析：去畸变、预处理、特征提取、配准、地图表示全链路

## 运动畸变矫正（最关键步骤）

| 方法 | 代表项目 | 精度 @10m/s |
|------|---------|-----------|
| CT 连续时间 | CT-ICP（12DoF 参数化） | 2-5 cm |
| IMU 反向传播 | FAST-LIO2, FAST-LIVO2, fusions_slam | 1-3 cm |
| IMU 角速度积分 | LIO-SAM, LVI-SAM（仅旋转） | 5-10 cm |
| 恒速模型 | KISS-ICP | 5-15 cm |
| 神经隐式 | PIN-SLAM（SDF 连续空间） | 3-8 cm |

## 特征提取演化

```
传统 LOAM 曲率分类 (edge/planar)  →  在线平面拟合 + GM 核自适应
         LeGO-LOAM, LIO-SAM         fusions_slam, KISS-ICP, Lightning-LM
                                              →  神经隐式 (SDF)
                                                PIN-SLAM
```

## 地图表示演化

```
传统离散               结构化稀疏                  连续/神经
Point Cloud → VoxelHashMap/VoxelMap → SDF/Neural Points
KD-Tree      iKD-Tree/IVox3d/八叉树 → 隐式点-体素观测(P2V-SLAM)
```

## 鲁棒核函数分布

- **Geman-McClure**：KISS-ICP, PIN-SLAM（最激进截断）
- **Cauchy**：CT-ICP, Lightning-LM, lt-mapper（软截断）
- **Huber-like 自适应**：LIO-SAM, LeGO-LOAM, ROLO-SLAM, LVI-SAM

## 关键设计模式

- 两步解耦优化：LeGO-LOAM（地面约束 roll/pitch，边缘约束 yaw/平移）
- 反向传播去畸变：fusions_slam, Lightning-LM, FAST-LIVO2
- 统一 IESKF 紧耦合：FAST-LIVO2
- Anchor 节点 PGO：lt-mapper（多会话对齐）
- 隐式点-体素观测：P2V-SLAM（VE-Net 编码体素特征，IR-Net 输出残差与不确定度）

## 相关页面

- [[LiDAR方案对比]]
- [[算法-KISS-ICP]]
- [[算法-FAST-LIO]]
- [[算法-P2V-SLAM]]
- [[方法-体素地图]]
- [[方法-隐式点-体素观测模型]]
- [[概念-连续时间轨迹]]
- [[2026-04-29-lidar-pipeline-comparison]]
