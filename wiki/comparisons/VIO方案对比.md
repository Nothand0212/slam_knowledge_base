---
tags: [VIO, VO, 对比分析, 方案选型]
sources:
  - wiki/sources/2026-04-29-camera-pipeline-comparison.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
  - wiki/sources/2026-04-29-open-vins-analysis.md
  - wiki/sources/2026-04-29-orb-slam3-analysis.md
  - wiki/sources/2026-04-29-vins-fusion-analysis-analysis.md
  - wiki/sources/2026-04-29-msckf-vio-analysis-analysis.md
  - wiki/sources/2026-04-29-schurvins-analysis-analysis.md
  - wiki/sources/2026-04-29-dm-vio-analysis-analysis.md
  - wiki/sources/2026-04-29-dso-analysis-analysis.md
  - wiki/sources/2026-04-29-svo-pro-analysis-analysis.md
  - wiki/sources/2026-04-29-rovio-analysis-analysis.md
  - wiki/sources/2026-04-29-droid_slam-analysis.md
  - wiki/sources/2026-04-29-nice_slam-analysis.md
  - wiki/sources/2026-04-29-paper-notes.md
  - wiki/sources/2026-04-30-image-preprocessing-comparison.md
created: 2026-04-28
updated: 2026-04-30
type: comparison
---

# VIO/VO 方案对比

> 对比传统滤波、滑动窗口优化、增量平滑、直接法、半直接法和深度学习 SLAM 在视觉前端、IMU 使用、后端范式和工程落地上的取舍。

## 对比边界

本页比较视觉主导的 VO/VIO/视觉 SLAM。纯 LiDAR、GNSS/INS 和特殊传感器系统不放在这里；它们分别进入 [[LiDAR方案对比]]、[[GNSS数据管线]] 和 [[特殊传感器数据管线]]。

## 方案族总览

| 方案族 | 代表系统 | 后端/估计范式 | 视觉观测 | IMU 使用 | 主要优势 | 主要限制 |
| ------ | -------- | ------------- | -------- | -------- | -------- | -------- |
| MSCKF/EKF | [[算法-OpenVINS]], [[概念-MSCKF\|MSCKF_VIO]] | EKF / OC-MSCKF | KLT/描述子特征轨迹 | 高频传播 + 零空间投影更新 | 速度快，状态维度受控，在线标定强 | 一次线性化，历史误差难修正 |
| Schur EKF | [[算法-SchurVINS]] | Schur complement EKF | 稀疏特征 | RK4 + Schur 边缘化 | 比传统 MSCKF 更接近优化式约束 | 仍保留滤波框架的线性化限制 |
| 滑动窗口 BA | [[2026-04-29-vins-fusion-analysis-analysis\|VINS-Fusion]] | Ceres 滑窗优化 | KLT/重投影 | 中值预积分因子 | 精度和工程成熟度平衡 | 边缘化先验固定，窗口大小敏感 |
| 增量平滑 | [[算法-Kimera-VIO]], [[2026-04-29-dm-vio-analysis-analysis\|DM-VIO]] | GTSAM iSAM2 / 延迟边缘化 | Smart factor / 光度因子 | GTSAM 预积分 | 因子图表达清晰，重线性化能力强 | 架构和调参复杂度高 |
| 全局 BA / Atlas | [[算法-ORB-SLAM3]] | G2O BA + Atlas | ORB 特征 + BoW | 多模式 IMU 初始化 | 回环、重定位、地图管理成熟 | 系统重，模块耦合强 |
| 直接法 VO/VIO | [[算法-DSO]], [[算法-DM-VIO]], [[算法-ROVIO]] | 光度 BA / patch EKF | 光度误差 | 有或无，视系统而定 | 弱纹理和亚像素对齐能力强 | 光照、曝光和标定敏感 |
| 半直接法 | [[算法-SVO-Pro]] | 直接法前端 + 重投影后端 | SparseImgAlign + 特征 | VIO 后端可选 | 速度快，前端轻 | 回环和全局一致性较弱 |
| 深度学习 | [[算法-DROID-SLAM]], [[算法-NICE-SLAM]], [[算法-MonoGS]] | Dense BA / 可微渲染 | 光流、隐式场、3DGS | 多数不以 IMU 为核心 | 表示能力强，弱化手工特征 | GPU 依赖强，工程可控性较弱 |

## 关键维度

### 视觉前端

| 前端类型 | 代表系统 | 适用场景 | 风险 |
|----------|----------|----------|------|
| 稀疏特征 | ORB-SLAM3, OpenVINS, VINS-Fusion, Kimera-VIO | 光照变化、长距离匹配、回环 | 低纹理区域跟踪困难 |
| 稀疏直接法 | DSO, DM-VIO, SVO Pro | 有稳定光度模型、纹理连续区域 | 曝光和光度标定敏感 |
| 图像块光度 | ROVIO | 飞行器、小窗口 EKF | patch 生命周期和外参敏感 |
| 稠密/学习式 | DROID-SLAM, NICE-SLAM, MonoGS | GPU 可用、需要稠密几何或渲染 | 模型依赖、实时性和部署复杂 |

### 图像预处理边界

VIO/VO 方案比较时，图像预处理要跟视觉观测模型一起看：

| 方案族 | 预处理重点 | 选型含义 |
|--------|------------|----------|
| KLT/角点 VIO | 灰度化、CLAHE/均衡化、KLT 金字塔、mask/grid、稀疏点去畸变 | 可通过 tracked count、inlier ratio 和 track lifetime 判断是否提升前端质量 |
| ORB/描述子 SLAM | rectify/resize、ORB pyramid、FAST 阈值、关键点去畸变 | 重点保证尺度、双目校正和描述子匹配一致性 |
| 直接法/半直接法 | response、vignette、exposure、affine brightness、patch pyramid | 预处理是光度残差模型的一部分，不能随意套用 CLAHE |
| 学习式/稠密 | RGB/BGR、`/255`、mean/std、crop/resize 后内参更新 | 首先保证输入契约匹配训练分布和可微几何 |

完整分析见 [[图像预处理与观测模型]]。

### IMU 链路

多数 VIO 系统把 IMU 重点放在估计层：传播、预积分、bias 在线估计和初始化。进入估计器前的滤波、陷波、温度补偿通常较轻，相关边界见 [[传感器-IMU预处理]]。

| 系统 | IMU 链路特点 |
|------|---------------|
| OpenVINS / MSCKF_VIO | 高频 EKF 传播，外参/时间偏移可配置或在线估计 |
| VINS-Fusion | 中值预积分 + 滑动窗口优化 + bias 校正 |
| ORB-SLAM3 / OpenMAVIS | 视觉初始化后 IMU 联合优化，OpenMAVIS 强调 SE2(3) 精确预积分 |
| DM-VIO | GTSAM 预积分与直接法光度图结合，延迟边缘化降低错误先验风险 |
| DSO / SVO Pro / ROVIO | IMU 不是总是核心，具体取决于系统变体 |
| DROID-SLAM / NICE-SLAM / MonoGS | 以视觉/渲染优化为主，IMU 通常不是主线 |

## 选型建议

| 目标 | 首选参考 | 理由 |
| ---- | -------- | ---- |
| 轻量实时 VIO 前端 | [[算法-OpenVINS]] | 模块边界清楚，滤波链路完整，适合抽象成多传感器前端 |
| 成熟开源 VIO 基线 | [[2026-04-29-vins-fusion-analysis-analysis\|VINS-Fusion]] | 滑窗 BA、预积分、边缘化和回环都有完整工程实现 |
| 需要回环/重定位/多地图 | [[算法-ORB-SLAM3]] | Atlas、BoW、全局 BA 和多传感器模式成熟 |
| 因子图融合研究 | [[算法-Kimera-VIO]], [[2026-04-29-dm-vio-analysis-analysis\|DM-VIO]] | GTSAM/iSAM2、smart factor、延迟边缘化对多传感器因子图有参考价值 |
| 弱纹理或光度前端研究 | [[算法-DSO]], [[算法-SVO-Pro]] | 直接法/半直接法提供非描述子路线 |
| 稠密重建或神经表示 | [[算法-DROID-SLAM]], [[算法-NICE-SLAM]], [[算法-MonoGS]] | 可微优化和场景表示能力强 |

## 设计结论

如果目标是多传感器 SLAM 的可维护工程骨架，视觉前端不应直接照搬一个完整 SLAM 系统。更稳妥的做法是：

- 用 OpenVINS/MSCKF 类结构理解视觉-IMU 状态和在线标定边界。
- 用 VINS-Fusion/DM-VIO 理解预积分、边缘化和滑窗先验。
- 用 ORB-SLAM3 借鉴回环、重定位和地图管理，而不是照搬其后端耦合。
- 将 IMU 预处理、IMU 预积分和视觉残差分层处理，避免把信号滤波问题混入后端因子设计。

## 相关页面

- [[相机数据管线]]
- [[IMU数据管线]]
- [[传感器-IMU预处理]]
- [[概念-IMU预积分]]
- [[因子图vs滤波]]
- [[LiDAR方案对比]]
- [[概念-直接法视觉里程计]]
- [[概念-深度学习SLAM]]
- [[图像预处理与观测模型]]
