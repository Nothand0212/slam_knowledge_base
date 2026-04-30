---
tags: [ORB-SLAM3, ORB, 源码分析, Atlas, 多地图, g2o, BA, IMU初始化]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-deep-dive/orb_slam3_analysis.md
sources:
  - raw/docs-deep-dive/orb_slam3_analysis.md
---

# ORB-SLAM3 源码级深度剖析

> ORB-SLAM3 完整源码分析（基于代码仓库），覆盖系统入口、特征提取、初始化、三线程架构、BA 体系、Atlas 多地图

## 三线程架构

- **Tracking（主线程）**：特征提取 + 位姿估计 + 关键帧决策
- **LocalMapping**：BA + 新地图点创建 + KF 剔除
- **LoopClosing**：回环检测 + Sim3 计算 + 图优化 + Global BA

## 六种传感器模式

`MONOCULAR | STEREO | RGBD | IMU_MONOCULAR | IMU_STEREO | IMU_RGBD`

## Atlas 多地图系统

- 跟踪丢失自动创建新地图（不丢弃旧数据）
- 通过 Common Regions 检测自动合并
- Boost serialization 支持地图保存/加载

## 多级 BA 优化体系

```
Motion-only BA → Local BA → Local Inertial BA
  → Essential Graph → Full BA → Full Inertial BA
```

## ORB 特征提取

- FAST 角点（双阈值 iniThFAST=20 / minThFAST=7）
- 四叉树分布（DistributeOctTree）
- IC_Angle 灰度质心方向
- Steered BRIEF 32 字节描述子

## 优势与局限

**优势**：三线程异步并行、Atlas 多地图鲁棒性、多级 BA 精准匹配实时性

**局限**：ORB 特征低纹理/运动模糊不足、内存占用大、单目初始化脆弱

## 相关页面

- [[算法-ORB-SLAM3|ORB-SLAM3]]
- [[2026-04-28-orb_slam3-analysis]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]
- [[组件-DBoW2]]
- [[概念-直接法vs间接法]]