---
tags: [架构设计, 软件工程, ROS, genz-icp]
sources:
  - wiki/sources/2026-04-29-genz_icp_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# cpp+ros分离架构

> GenZ-ICP 教科书级架构：算法核心 cpp/ 层零 ROS 依赖 + ros/ 薄 wrapper 层，ROS1/ROS2 条件编译。

## 设计目标

cpp+ros 分离的目标是让算法库可以独立于 ROS 编译、测试和复用。ROS 只负责消息、参数、TF 和节点生命周期，核心 ICP 逻辑只依赖 Eigen/Sophus/TBB 等普通 C++ 库。这样同一套算法可以同时服务 ROS1、ROS2、离线评测和嵌入式集成。

## 分层结构
- **cpp/genz_icp/core/**：纯 C++ 算法，依赖 Eigen+Sophus+TBB+robin_map，零 ROS 引用
- **cpp/genz_icp/pipeline/**：管线编排，GenZICP 类暴露纯 C++ 接口
- **ros/ros1/** + **ros/ros2/**：ROS 消息 ↔ Eigen 转换 + TF 发布，每个 < 300 行

## 优势
- 算法可脱离 ROS 编译运行（仅需 Eigen+Sophus+TBB）
- 同一核心代码库同时支持 ROS1 Noetic 和 ROS2 Humble
- `FetchContent` 支持自动从 GitHub 拉取核心代码
- Header-only Utils.hpp 零运行时开销的转换工具

## 工程边界

分离架构要求接口设计足够稳定。若核心层偷用 ROS 时间、参数服务器或消息类型，就会重新耦合；若 wrapper 层包含算法分支，ROS1/ROS2 行为会漂移。最好的边界是核心层只接收普通结构体和时间戳，wrapper 层只做转换和调度。

## 相关页面

- [[方法-genz-icp]]
- [[架构-Pipeline 传感器数据调度]]
- [[架构-后端适配器模式]]
- [[算法-fusions_slam]]
