---
tags: [GNSS, VIO, IMU, Ceres, 多传感器融合, IC-GVINS, 滑动窗口, 边缘化]
sources:
  - raw/docs-deep-dive/ic_gvins_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/ic_gvins_analysis.md
---

# IC-GVINS 深度源码分析

> 武汉大学 i2Nav 课题组，RA-L 2022，INS-Centric GNSS-Visual-Inertial Navigation System。以 INS 为核心的 GNSS-视觉-惯性紧耦合导航系统，Ceres 滑动窗口优化 + 边缘化，不使用 GTSAM。

## 摘要

IC-GVINS 以 INS 为中心，将 GNSS RTK 位置、相机特征重投影和 IMU 预积分在同一个 Ceres Problem 中联合优化。核心创新：(1) [[方法-GNSS 位置残差因子]]含杆臂补偿的 Ceres 实现范式；(2) [[方法-地球自转补偿预积分]]对高精度 IMU 有效益；(3) [[方法-INS-centric 初始化]]避免纯 VIO 初始化失败；(4) [[方法-Ceres 两轮优化 + 粗差剔除]]自适应质量控制；(5) 三线程分离架构保证实时性。

## 核心概念

- **位置级紧耦合**：对 GNSS 原始观测量是松耦合（仅 RTK 定位），对三传感器系统是紧耦合（联合优化）
- **GNSS 因子**：Ceres SizedCostFunction<3,7>，残差 e = p_IMU + R(q) x lever - p_GNSS，信息矩阵由定位标准差构造
- **可插拔预积分**：工厂模式支持标准/地球自转/里程计四种预积分变体
- **坐标系**：WGS84 --> ECEF --> 局部 NED，原点固定于首个 GNSS 位置
- **关键差异**：Ceres 无 ISAM2 支持，IC-GVINS 手动管理滑动窗口 + 边缘化

## 相关页面

- [[算法-IC-GVINS]]
- [[方法-GNSS 位置残差因子]]
- [[方法-地球自转补偿预积分]]
- [[方法-INS-centric 初始化]]
- [[方法-Ceres 两轮优化 + 粗差剔除]]
- [[架构-GNSS 位置因子设计模式]]
- [[算法-OB_GINS]]