---
tags: [ORB-SLAM3, 源码分析, Atlas, 多地图, BA, g2o]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-camera-pipeline-comparison.md
---

# ORB-SLAM3 分析

> ORB-SLAM3 完整源码级深度剖析，覆盖三线程架构、Atlas 多地图、多级 BA 体系

## 系统架构

**三线程异步并行**：
- **Tracking（主线程）**：ORB 特征提取 + 位姿估计（Motion-model/RefKF/重定位）+ 关键帧决策
- **LocalMapping**：局部 BA + 地图点剔除（MapPointCulling）+ 新地图点三角化（CreateNewMapPoints）+ KF 剔除
- **LoopClosing**：BoW 候选检测 + Sim3 计算 + Loop Fusion + Essential Graph + Global BA

## 六种传感器模式

`MONOCULAR=0 | STEREO=1 | RGBD=2 | IMU_MONOCULAR=3 | IMU_STEREO=4 | IMU_RGBD=5`

各模式的主要差异在初始化策略和跟踪线程：纯视觉模式（0-2）用两帧SfM初始化，IMU模式（3-5）需要额外的视觉-惯性联合初始化（分 gyro bias估计→尺度/重力粗估计→Full BA精化三步）；RGB-D模式（2/5）直接利用深度图获得3D点，无需三角化。

## Atlas 多地图系统

跟踪丢失自动创建新地图（不丢弃旧数据），通过 Common Regions 检测自动合并。Boost serialization 支持地图保存/加载。

## ORB 特征提取

- FAST 角点检测（双阈值 iniThFAST=20 / minThFAST=7）+ 四叉树均匀分布（DistributeOctTree）
- IC_Angle 灰度质心方向（31x31 圆形 patch）+ Steered BRIEF 旋转感知描述子（32 字节）
- 金字塔 8 层（scaleFactor=1.2），等比数列分配特征数量
- 48x64 网格空间索引加速邻域搜索

## 多级 BA 优化体系

```
Motion-only BA → Local BA → Local Inertial BA
  → Essential Graph → Full BA → Full Inertial BA
```

使用 g2o 框架，LM 算法 + BlockSolver_6_3，Huber 鲁棒核。

## IMU 初始化

1. 重力方向估计（dirG = Σ(R_wb·ΔV)）
2. InertialOnly 优化（Rwg, scale, bg, ba, 速度）
3. 视觉-惯性 Full BA（100 次迭代）
4. 阶段性精化：5s → SetInertialBA1, 15s → SetInertialBA2

## 适用场景

**适合**：室内外长期 SLAM、VIO 模式、多段不连续轨迹拼接、需要地图保存/重载

**不适合**：低纹理环境、高动态场景（大量运动物体）、实时性要求极高的嵌入式场景

## 相关页面

- [[算法-ORB-SLAM3|ORB-SLAM3]]
- [[2026-04-28-orb_slam3-analysis]]
- [[2026-04-29-orb-slam3-analysis]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]
- [[组件-DBoW2]]
- [[概念-直接法vs间接法]]
- [[方法-视觉特征跟踪]]