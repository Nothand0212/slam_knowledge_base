---
tags: [GTSAM, 点云SLAM, 因子库, 点云配准, GPU, gtsam_points, ICP, GICP, VGICP, CT-ICP, bundle-adjustment]
sources:
  - raw/docs-deep-dive/gtsam_points_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/gtsam_points_analysis.md
---

# gtsam_points 深度源码分析

> Kenji Koide (AIST) 开发的基于 GTSAM 的点云 SLAM 因子与优化器扩展库，MIT License，提供 16 个核心因子类 + 4 个实验性因子，支持 CUDA GPU 加速。

## 摘要

gtsam_points 是专为 LiDAR SLAM 设计的 GTSAM 因子扩展库。核心贡献：(1) 层次化扫描配准因子体系（ICP --> GICP --> VGICP --> LOAM --> ColoredGICP --> CT-ICP），所有配准因子共用 [[方法-IntegratedMatchingCostFactor]] 基类；(2) [[方法-GaussianVoxelMap 体素化配准]]实现 O(1) 数据关联；(3) [[方法-连续时间 ICP 因子]]内建运动畸变校正；(4) [[方法-LiDAR 捆集调整因子]]将 BA 形式化为因子图优化。

## 核心概念

- **因子层次体系**：IntegratedMatchingCostFactor 定义 update_correspondences() 和 evaluate() 两个纯虚函数，所有 12 个扫描配准因子通过重写这两个方法即可接入 GTSAM 图
- **体素化加速**：VGICP 将目标点云预体素化为 GaussianVoxelMap，用 O(1) 体素索引替代逐点 KNN，每个 source 点可与多个体素建立多分布对应
- **连续时间 SLAM**：CT-ICP/CT-GICP 因子持有 t0、t1 两个 key，通过 slerp 插值每点位姿
- **BA 因子创新**：PlaneEVMFactor/EdgeEVMFactor 基于特征值最小化，LsqBundleAdjustmentFactor 代价与点数无关仅与帧数有关
- **帧抽象层**：frame_traits.hpp 模板特化实现 PointCloud/GaussianVoxelMap 统一 API，因子泛型可接受任意帧类型
- **GPU 批量线性化**：NonlinearFactorGPU 基类 + NonlinearFactorSetGPU 批量管理器，LM_ext/ISAM2_ext 原生支持

## 相关页面

- [[组件-gtsam_points]]
- [[组件-GTSAM]]
- [[概念-因子图]]
- [[方法-IntegratedMatchingCostFactor]]
- [[方法-GaussianVoxelMap 体素化配准]]
- [[方法-连续时间 ICP 因子]]
- [[方法-LiDAR 捆集调整因子]]
- [[方法-ICP配准方法]]
- [[方法-GICP配准方法]]