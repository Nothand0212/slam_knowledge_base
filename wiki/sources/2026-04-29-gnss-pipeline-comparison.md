---
tags: [GNSS, RTK, PPP, 组合导航, 紧耦合, 松耦合, 坐标转换]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-comparison/gnss_pipeline_comparison.md
sources:
  - raw/docs-comparison/gnss_pipeline_comparison.md
---

# GNSS 数据管线横向对比

> 基于 7 个项目的 GNSS 数据处理全链路对比，涵盖原始数据、坐标转换、质量控制、融合方式、初始化管线

## 三种 GNSS 集成范式

### 范式 A: GNSS-centric（ic_gvins, ob_gins）
- GNSS 是主传感器，提供绝对坐标锚定
- IMU/视觉是辅助，修正 GNSS 中断期间漂移
- 适用：开阔天空，高精度定位需求

### 范式 B: VIO/LIO-centric + GNSS Post-fusion（vins_fusion, lio_sam）
- VIO/LIO 是主里程计，GNSS 在后端做全局校正
- 适用：城市峡谷、GNSS 断续场景

### 范式 C: Multi-observation Semi-tight（fusions_slam）
- 位置、姿态、速度三种观测独立注入 IESKF
- 适用：有高性能 RTK 接收机

## 坐标转换管线

所有项目的转换链：WGS84 BLH → ECEF → 局部 NED/ENU

| 项目 | 坐标工具 | 局部坐标系 | 原点设置 |
|------|---------|-----------|---------|
| ic_gvins | 自写 Earth 类 | NED | 首个有效 GNSS |
| vins_fusion | GeographicLib | ENU | 首个 GPS |
| lio_sam | GTSAM GPSFactor | ENU | GTSAM 自动 |

## 质量控制

### ic_gvins/ob_gins 的两阶段粗差剔除
1. 第一轮优化：GNSS 因子使用 HuberLoss(1.0)
2. Chi² 检验（3自由度，阈值 7.815）
3. 第二轮优化：移除 HuberLoss 再优化

## 相关页面

- [[算法-IC-GVINS]]
- [[算法-OB_GINS]]
- [[架构-GNSS 位置因子设计模式]]
- [[架构-多传感器融合架构]]
- [[架构-坐标系管理]]