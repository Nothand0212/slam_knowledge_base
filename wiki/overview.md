---
tags: [overview, ROS, SLAM]
type: overview
created: 2026-04-26
updated: 2026-05-02
sources: []
---
# ROS 机器人导航与 SLAM 知识库 — 知识库总览

> 创建于 2026-04-26

---

## 关于这个知识库

这里收集 ROS 机器人导航与 SLAM 相关的算法、工程实践和方案取舍，重点覆盖 localization、mapping、planning、control、sensor fusion、TF/坐标系、调参排障和真实机器人落地经验。

每个素材都经过 AI 消化和整理，形成了互相链接的 wiki 页面。你可以通过以下方式浏览：

- **实体页**：人物、组织、概念、工具的详细介绍
- **主题页**：围绕某个研究主题的综合分析
- **素材摘要**：每篇素材的核心观点提取
- **对比分析**：不同方案、工具、观点的横向比较
- **综合分析**：跨素材的深度洞察

---

## 知识地图

- [[组件-GTSAM]]：factor graph optimization 库，覆盖 SLAM、SfM、navigation 和 sensor fusion。
- [[GTSAM API 使用索引]]：当前最重要入口，按“我要做什么”组织 API。
- [[GTSAM 因子图工作流]]：构图、初值、优化、结果查询的主流程。
- [[GTSAM Navigation 与 IMU API]]：服务 VIO/INS/GNSS 融合方向。
- [[GTSAM SLAM 与视觉因子 API]]：服务 pose graph、visual SLAM 和 stereo VO。
- [[素材索引]]：所有素材摘要页的统一入口。
- [[概念-基准测试数据集]]：EuRoC、KITTI 等评测数据集与指标入口。
- [[相机数据管线]]、[[IMU数据管线]]、[[LiDAR数据管线]]、[[GNSS数据管线]]、[[特殊传感器数据管线]]：跨算法传感器处理链路对比。
- [[图像预处理与观测模型]]：按 KLT/ORB/直接法/学习式/LIVO 观测模型组织图像预处理影响。
- [[算法-P2V-SLAM]]、[[方法-隐式点-体素观测模型]]：补充 LiDAR SLAM 从显式平面/直线残差走向学习式体素几何观测的路线。

---

## 快速导航

| 类型 | 数量 | 查看 |
|------|------|------|
| 素材 | 79 | [[素材索引]] |
| 实体 | 160 | 实体页 |
| 主题 | 14 | 主题页 |
| 对比 | 3 | 对比分析 |
| 综合 | 2 | 综合分析 |

---

## 最近更新

- 2026-05-02：消化 P2V-SLAM 公众号文章，新增隐式点-体素 LiDAR-IMU SLAM 算法页与观测模型方法页。
- 2026-04-27：消化 GTSAM 4.3a1 官方文档，新增 API 使用索引、geometry/nonlinear/navigation/SLAM/custom factor 专题页。
