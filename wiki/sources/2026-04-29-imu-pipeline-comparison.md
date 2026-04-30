---
tags: [IMU, IMU预积分, IMU管线, 滤波, 因子图, EKF, IESKF]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-comparison/imu_pipeline_comparison.md
sources:
  - raw/docs-comparison/imu_pipeline_comparison.md
---

# IMU 数据管线横向对比

> 21 个主流 SLAM/VIO 项目的 IMU 数据处理全管线对比，涵盖滤波系和优化系两种范式

## 对比范围

- **VIO (11)**：open_vins, vins_fusion, orb_slam3, dm_vio 等
- **LiDAR-inertial (8)**：fast_lio_sam, lio_sam, r3live, fast_livo2 等
- **GNSS-inertial (3)**：ic_gvins, ob_gins, superodom

## 两大范式

### 滤波系 (EKF/IESKF)
- IMU 角色：系统主线，高频持续传播状态
- 代表：open_vins, msckf_vio, fast_livo2, r3live
- 更新时机：事件驱动（图像/点云到达时）
- 协方差管理：显式 EKF 协方差传播+更新

### 优化系 (FGO)
- IMU 角色：因子图中二元约束，关键帧间预积分
- 代表：vins_fusion, orb_slam3, lio_sam, ic_gvins
- 更新时机：关键帧频率（1-10 Hz）批量优化
- 协方差管理：预积分累积协方差，边缘化用 Schur complement

## 关键对比维度

### 离散积分方法
| 精度 | 方法 | 代表项目 |
|------|------|---------|
| 最高 | SE2(3) 精确积分 | OpenMAVIS |
| 高 | RK4 + 三阶 Phi | msckf_vio, schurvins |
| 中 | 中值积分 | vins_fusion |
| 低 | 欧拉离散 | fusions_slam, lightning_lm |

### 初始化策略
- **静止 IMU 均值**：msckf_vio, open_vins（静态）
- **在线均值**：fast_livo2（3帧），lightning_lm（20帧）
- **SFM + IMU 对齐**：vins_fusion, orb_slam3
- **GNSS/INS 粗对齐**：ic_gvins

### 预积分 vs 滤波传播
| 维度 | 预积分 | 滤波传播 |
|------|--------|---------|
| 适用场景 | 精度优先，中频输出 | 速度优先，高频输出 |
| 零偏更新 | 一阶 Jacobian 校正 | 实时估计 |

## 相关页面

- [[概念-IMU预积分]]
- [[概念-IMU预积分]]
- [[方法-IESKF滤波器]]
- [[因子图vs滤波]]
- [[架构-滑动窗口优化]]
- [[概念-MSCKF]]