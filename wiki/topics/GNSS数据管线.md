---
tags: [GNSS, GNSS管线, 数据管线, 传感器]
created: 2026-04-29
updated: 2026-04-29
sources: [2026-04-29-gnss-pipeline-comparison]
---

# GNSS 数据管线

> 7 个项目的 GNSS 数据处理全链路对比：三种集成范式、坐标转换、质量控制、初始化策略

## 三种 GNSS 集成范式

### 范式 A: GNSS-centric（ic_gvins, ob_gins）
GNSS 是主传感器，提供绝对坐标锚定。IMU/视觉是辅助修正设备。

### 范式 B: VIO/LIO-centric + GNSS Post-fusion（vins_fusion, lio_sam）
VIO/LIO 是主里程计，GNSS 在后端做全局校正，不参与前端状态估计。

### 范式 C: Multi-observation Semi-tight（fusions_slam）
RTK 的位置、姿态、速度三维观测分别注入 IESKF。

## 坐标转换链

```
WGS84 BLH → ECEF(x,y,z) → 局部 NED/ENU → 局部笛卡尔坐标
```

| 项目 | 坐标工具 | 局部系 | 重力公式 |
|------|---------|--------|---------|
| ic_gvins | 自写 Earth | NED | WGS84 标准公式 |
| vins_fusion | GeographicLib | ENU | 固定 9.81 |
| lio_sam | GTSAM GPSFactor | ENU | GTSAM 内置 |

## 质量控制

ic_gvins/ob_gins 的两阶段粗差剔除：
1. 第一轮优化：HuberLoss(1.0)
2. Chi² 检验（3自由度，阈值 7.815，p=0.05）→ reweight std
3. 第二轮优化：无核函数

## GNSS 拒止策略

- ic_gvins：退化为纯 VIO（滑窗中无 GNSS 因子则自动退化）
- ob_gins：退化为纯惯性导航（协方差快速发散）
- vins_fusion：VIO alone（GPS 因素不加入 PGO）
- fusions_slam：退化为纯 LiDAR-IMU

## 相关页面
- [[方法-滑动窗口GNSS优化]]

- [[算法-IC-GVINS]]
- [[算法-OB_GINS]]
- [[架构-GNSS 位置因子设计模式]]
- [[架构-坐标系管理]]
- [[2026-04-29-gnss-pipeline-comparison]]