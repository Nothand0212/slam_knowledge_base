---
tags: [LiDAR, ICP, 极简, 里程计]
sources:
  - raw/docs-deep-dive/kiss_icp_analysis.md
created: 2026-04-29
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/kiss_icp_analysis.md
---

# KISS-ICP 源代码深潜分析

> 完整源码级分析：KissICP、Registration、VoxelHashMap、Preprocessing、AdaptiveThreshold、Metrics 全模块

## 摘要
KISS-ICP 以约 500 行核心 C++ 证明全点云 Point-to-Point ICP + Geman-McClure 鲁棒核 + 自适应阈值可匹敌复杂特征方法的里程计精度。纯 LiDAR 方案，零 IMU/GPS 依赖。

## 关键概念
- **[[算法-KISS-ICP]]**：基于 Gauss-Newton + GM 核的 ICP 优化，默认 500 次迭代，点到点欧氏距离残差
- **[[方法-VoxelHashMap]]**：基于 `tsl::robin_map` 的 3D 哈希体素地图，27 邻域最近邻搜索，滑动窗口裁剪（默认 100m）
- **[[方法-自适应阈值]]**：从运动模型预测偏差中自动学习 sigma，`3*sigma` 作对应截断，`sigma` 作 GM kernel_scale
- **[[方法-Geman-McClure鲁棒核]]**：`w=(σ/(σ+r²))²`，比 Huber 更激进拒绝大残差，实现隐式特征选择
- **[[方法-点到点ICP]]**：双层体素降采样（dense 0.5x 用于建图，sparse 1.5x 用于配准），恒速运动模型预测

## 相关页面
- [[算法-KISS-ICP]]
- [[方法-ICP配准方法]]
- [[方法-GICP配准方法]]
- [[LiDAR方案对比]]