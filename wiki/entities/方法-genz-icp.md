---
tags: [ICP, 点云配准, LiDAR]
sources:
  - wiki/sources/2026-04-28-genz-icp-analysis.md
  - wiki/sources/2026-04-29-genz_icp_analysis.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# genz-icp

> 一种自适应广义 ICP（Generalized ICP）变体，基于点云局部平面度自动选择配准模型（point-to-plane 或 Mahalanobis 加权），输出 Hessian 逆作为可靠协方差。

## 概述

genz-icp 是 Koide 团队开发的点云配准算法，核心创新在于**平面度自适应权重**机制。对于每个配准点，算法分析其局部邻域的平面度（planarity）：当邻域高度平面化时，采用点到平面（point-to-plane）约束，赋予高权重；当邻域非平面（如边缘、杂乱区域）时，降级为马氏距离（Mahalanobis distance）约束，赋予低权重以减少离群点影响。

阈值设定是**自动**的——无需手动调节平面度判定阈值，算法基于统计分析自动估计。配准完成后，使用最终 Hessian 矩阵的逆（H^{-1}）作为估计位姿的协方差矩阵，量化配准不确定性，可用于后续因子图融合时调整约束权重。

## 在分析框架中的应用

- **fusions_slam / hdl_localization**：genz-icp 常用于 LiDAR 扫描到地图（scan-to-map）和扫描到扫描（scan-to-scan）配准模块，输出带协方差的相对位姿估计。
- 与[[概念-因子图]]集成时，配准结果的协方差可直接用于 `BetweenFactor` 或自定义 LiDAR 因子的信息矩阵，实现自适应加权融合。
- 与 **gtsam_points** 的 `IntegratedGICPFactor` 关系密切，genz-icp 的配准模型可视为 GICP 因子的一步优化等价物。

## 对 SLAM 算法的意义

多传感器 SLAM 系统需要 LiDAR 前端提供带协方差的帧间相对位姿，genz-icp 直接满足此需求。其平面度自适应机制确保了在结构化环境（走廊、室内）和非结构化环境（开阔地带、树林）均能产生可靠约束，协方差的输出使视觉-LiDAR 融合时能按不确定性均衡权重。

## 相关页面

- [[算法-FAST-LIO]]
- [[组件-GTSAM]]
- [[架构-坐标系管理]]