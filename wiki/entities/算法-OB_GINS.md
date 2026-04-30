---
tags: [VIO, GINS, 开源, GNSS, 紧耦合]
sources:
  - wiki/sources/2026-04-28-ic_gvins-analysis.md
  - wiki/sources/2026-04-28-ob_gins-analysis.md
  - wiki/sources/2026-04-29-framework-comparison.md
  - wiki/sources/2026-04-29-gnss-pipeline-comparison.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# OB_GINS

> Optimization-Based GNSS/INS/Vision Navigation System，基于优化的全球导航惯性系统参考实现。

## 核心思想

OB_GINS 是由 IC-GVINS 团队开发的参考系统实现（2022），在 GINS（全球惯性导航系统）框架基础上，以优化而非 EKF 实现 GNSS-INS 紧耦合。它将 GNSS 伪距/载波相位观测和 IMU 预积分作为因子图中的因子，通过 GTSAM 进行增量平滑优化。OB_GINS 同时提供 VINS 扩展路径，是 GNSS-INS-Visual 紧耦合的完整参考实现。

## 因子图建模

OB_GINS 把连续 IMU 传播压缩成预积分因子，把 GNSS 原始观测或定位结果转化为位置、速度、姿态或伪距/载波相位相关约束。与 EKF 单次线性化更新不同，优化式框架可以在滑动窗口或增量平滑中反复线性化相关状态，并自然处理异步传感器到达。

## 在 SLAM 中的应用

户外大尺度 SLAM 需要长期全局一致性，单纯 LIO/VIO 会随距离累积漂移。OB_GINS 的价值在于提供一个可复用的 GNSS/INS 优化后端：当 GNSS 质量好时提供全球锚点，当 GNSS 质量下降时依靠 IMU/VIO/LIO 短时维持连续性。它也为 [[算法-IC-GVINS]] 这类 GNSS-Visual-IMU 系统提供工程参考。

## 风险点

GNSS 观测质量强烈依赖环境。城市峡谷、多径、遮挡和周跳会让原始观测产生系统性误差，这类误差不能只靠普通高斯噪声建模解决。实际系统需要 NLOS 检测、周跳检测、鲁棒核、动态协方差和观测剔除，否则优化后端会把错误全球约束传播到整条轨迹。

## 相关页面

- [[组件-GTSAM]], [[概念-因子图]]
- [[概念-IMU预积分]], [[方法-GNSS-IMU 离线批优化]]
- [[算法-IC-GVINS]]
