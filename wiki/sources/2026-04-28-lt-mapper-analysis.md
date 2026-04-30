---
tags: [长期建图, 多会话, 位姿图优化, GTSAM, 变化检测, lt-mapper]
sources:
  - raw/docs-deep-dive/lt_mapper_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/docs-deep-dive/lt_mapper_analysis.md
---

# lt-mapper 深度源码分析

> ICRA 2022，长期多会话 LiDAR SLAM 后端。LT-SLAM（多会话位姿图融合）+ LT-removert（动态变化检测），将跨天/跨周多次建图数据整合为一致全局地图。

## 摘要

lt-mapper 专注于长期 SLAM 的后端融合。(1) [[方法-Anchor 节点位姿图优化]]通过四变量因子将多会话局部坐标变换到统一中心坐标系；(2) [[算法-FAST-LIO-SAM-SC-QN|SC+RS 双模态回环]] ScanContext + Radius Search 互补提高召回；(3) [[方法-信息增益回环选择]]仅添信息量最大的回环边；(4) [[方法-Removert 动态变化检测]]通过 range image 投影移除移动物体并生成 Delta Map。

## 核心概念

- **Anchor 节点**：BetweenFactorWithAnchoring<Pose3> 四变量因子，中心会话强先验，查询会话弱先验
- **双模式回环**：SC 提供快速粗匹配 + RS 以信息增益补充召回 --> ICP 验证
- **鲁棒核**：跨会话回环使用 Cauchy M-estimator
- **离线依赖**：需外部前端（SC-LIO-SAM）生成关键帧、SCD、位姿图

## 相关页面

- [[算法-lt-mapper]]
- [[方法-Anchor 节点位姿图优化]]
- [[方法-信息增益回环选择]]
- [[方法-Removert 动态变化检测]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]