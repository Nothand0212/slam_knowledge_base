---
type: entity
tags: [回环检测, ScanContext, 4D雷达, 描述子, SNR]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-4d-radar-slam-analysis.md
---

# Intensity Scan Context (ISC)

> Scan Context 的 4D 雷达适配版，用 SNR/intensity 取代高度作为环扇区描述子值，服务于稀疏雷达点云回环检测。

## 定义

4DRadarSLAM 对 Scan Context 描述子做的雷达适配版。核心差异：使用雷达 SNR（信噪比，dB）而非最大高度值作为 SC 描述子 bin 的特征值。

## 核心特征

- 矩阵维度：20 扇区 × 40 环（与原始 SC 相同）
- 每个 bin 取该区域内点的最大 SNR 值（而非最大高度）
- 适配原因：雷达 z 测量不准（俯仰角精度仅 1°），但 SNR 对不同表面材质有稳定区分度
- Ring Key KD 树检索 → 3 个最近邻 → V-key 快速对齐 → ±5% 搜索范围余弦距离
- 匹配阈值：SC_DIST_THRES = 0.5
- 匹配后仍需三重几何验证（ICP Fitness + Odometry Check + Pairwise Consistency）

## 为什么不用高度

4D 雷达的高度/俯仰精度通常弱于 LiDAR，直接复用高度版 ScanContext 会放大垂直噪声。SNR/intensity 与目标反射特性相关，在稀疏雷达点云中更稳定，因此更适合作为环扇区统计量。但强度也会受距离、材质和入射角影响，需要后续几何验证兜底。

## 工程边界

ISC 适合雷达回环候选召回，不应直接作为后端约束。雷达多径、稀疏点云和动态车辆会造成描述子混淆；因此 4DRadarSLAM 继续使用 ICP fitness、里程计一致性和 pairwise consistency 检查。

## 相关页面

- 实现于：[[算法-4DRadarSLAM]] `Scancontext.cpp:162-214`
- [[方法-ScanContext]]
- [[概念-回环检测方法]]
- [[传感器-Doppler 自速度估计]]
- [[特殊传感器数据管线]]
