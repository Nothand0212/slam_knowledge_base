---
tags: [GNSS, IMU, 组合导航, Ceres, OB_GINS, 离线批处理, Manifold]
sources:
  - raw/docs-deep-dive/ob_gins_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/ob_gins_analysis.md
---

# OB_GINS 深度源码分析

> 武汉大学 i2Nav 课题组，优化驱动的 GNSS/INS 组合导航系统。纯 GNSS+IMU 离线批处理，Ceres 滑动窗口，1 秒整秒节点采样，IC-GVINS 的前身。

## 摘要

OB_GINS 是 IC-GVINS 的纯 GNSS+IMU 前身，核心模块完全共享。(1) [[方法-固定整秒节点采样]]通过 IMU 插值精确对齐整秒节点；(2) [[组件-Ceres Manifold API]]已适配 Ceres 2.1+ 新流形接口；(3) 配置驱动的简单初始化（无自动重力调平）；(4) [[方法-GNSS-IMU 离线批优化|GNSS/IMU 离线批优化]]作为最小化 GNSS+IMU 组合导航参考实现。

## 核心概念

- **离线架构**：从文本文件读取数据，无 ROS/实时依赖，15 个源文件的极简实现
- **时间同步**：固定 INTEGRATION_LENGTH=1s，IMU 按时间比线性插值拆分增量对齐
- **Ceres Manifold 适配**：PoseManifold 实现 Plus/PlusJacobian/Minus/MinusJacobian，优于 IC-GVINS 的旧 LocalParameterization
- **里程计辅助**：支持单/双轮速计，通过 PreintegrationOdo 变体融入
- **与 IC-GVINS 关系**：共享 GNSS 因子和预积分体系，OB 多出 IMU_ACC_Z_SCALE 额外约束

## 相关页面

- [[算法-OB_GINS]]
- [[方法-固定整秒节点采样]]
- [[组件-Ceres Manifold API]]
- [[方法-GNSS-IMU 离线批优化]]
- [[方法-GNSS 位置残差因子]]
- [[算法-IC-GVINS]]
