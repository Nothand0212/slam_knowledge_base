---
tags: [相机, 相机管线, 标定, 特征提取, 跟踪, 直接法, 间接法]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-comparison/camera_pipeline_comparison.md
sources:
  - raw/docs-comparison/camera_pipeline_comparison.md
---

# 相机数据管线横向对比

> 基于 15 个 SLAM/VIO 算法的相机数据管线（原始数据、标定、预处理、特征提取、匹配、异常值剔除）全貌对比

## 对比范围

| 类别 | 算法 | 方法 |
|------|------|------|
| 直接法 | DSO, DM-VIO, ESVO | 稀疏光度 BA / 延迟边缘化 / 事件相机 |
| 间接法 | ORB-SLAM3, OpenVINS, VINS-Fusion, SVO Pro, Kimera-VIO | ORB特征/KLT光流/半直接法 |
| 深度学习 | DROID-SLAM, MonoGS, NICE-SLAM | CNN光流/Gaussian Splatting/神经隐式 |
| 滤波 | MSCKF_VIO, SchurVINS, ROVIO, OpenMAVIS | EKF/OC-MSCKF/Patch-based |

## 关键对比维度

### 标定策略
- **完全离线**：ORB-SLAM3, Kimera-VIO, VINS-Fusion（内参）
- **内参在线优化**：DSO, DM-VIO
- **外参在线全估计**：OpenVINS, MSCKF_VIO, ROVIO
- **时间偏移在线**：OpenVINS, VINS-Fusion, DM-VIO

### 特征提取
- **间接法主流**：FAST 角点检测（阈值 10-20）+ 网格/四叉树分布
- **唯一 Shi-Tomasi**：VINS-Fusion
- **ANMS 策略**：Kimera-VIO（不用 bucketing）
- **直接法**：梯度选点 + 自适应密度

### 异常值剔除
- 间接法标配：卡方检验（95% 置信度）
- 直接法：Huber/Tukey 鲁棒核
- 深度法：学习式置信度（无硬阈值）

### 直接法 vs 间接法取舍

| 维度 | 间接法 | 直接法 |
|------|--------|--------|
| 纹理要求 | 必须强角点 | 有梯度区域即可 |
| 光照鲁棒性 | 高 | 低 |
| 回环支持 | 强（描述子+BoW） | 弱 |

## 相关页面

- [[概念-直接法vs间接法]]
- [[方法-视觉特征跟踪]]
- [[传感器-传感器标定]]
- [[算法-ORB-SLAM3]]
- [[算法-OpenVINS]]