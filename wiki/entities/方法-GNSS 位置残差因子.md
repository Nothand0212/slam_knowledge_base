---
type: entity
tags: [GNSS, Ceres, 位置因子, 残差模型, 杆臂]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-ic_gvins-analysis.md
  - wiki/sources/2026-04-28-ob_gins-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
---

# GNSS 位置残差因子

> 将 RTK/GNSS 位置作为绝对位置观测接入优化，并显式处理天线杆臂、各向异性标准差和粗差检验。

## 定义

IC-GVINS/OB_GINS 中实现的 Ceres CostFunction 模式 GNSS 绝对位置观测因子。继承 `ceres::SizedCostFunction<3,7>`，将 GNSS 定位结果作为约束接入非线性优化。

## 核心特征

- 残差维度 3（NED 位置），参数块 1 个 7 维 pose
- 核心公式：e = W · (p_IMU + R(q) · l_ant - p_GNSS) 其中 W = diag(1/σ_N, 1/σ_E, 1/σ_D)
- 雅可比：∂e/∂p = I₃，∂e/∂θ = -R(q) · [l_ant]×
- 杆臂 l_ant 必须显式补偿（天线→IMU）
- 两轮优化：第一轮 HuberLoss(1.0) → Chi² 粗差（阈值 7.815）→ 第二轮无核函数精调
- 使用 RTK 定位结果（非原始伪距/载波）

## 相关页面

- 实现于：[[算法-IC-GVINS]]、[[算法-OB_GINS]]
- [[架构-GNSS 位置因子设计模式]]
- [[GNSS数据管线]]
- [[方法-Ceres 两轮优化 + 粗差剔除]]
- 对比：GTSAM 等价于 `NoiseModelFactor1<Pose3>`
- 外部核验：GTSAM 官方 `GPSFactor` 文档明确区分普通位置因子与带杆臂版本
