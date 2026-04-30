---
type: entity
tags: [多相机, 外参管理, 传感器抽象, 设计模式]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-slam_fusion_core-analysis.md
---

# CameraRig 多相机抽象

> 用一个统一容器管理单目、双目和多目相机外参，避免为不同相机数量设计多套接口。

## 定义

slam_fusion_core 中统一管理单目/双目/多目相机外参的核心类，用 `std::vector<Eigen::Isometry3d>` 存储 IMU body→相机 的变换关系。

## 核心特征

- 外参存储为 `bodyFromCamera`（与 open_vins 约定一致）
- `monocular()` 工厂方法：返回 N=1 的特殊情况，无需单独类型
- `validateCameraId()` 在 `pushVisual` 时校验相机索引安全性
- 空构造抛出异常，避免非法状态
- **当前缺失**：相机内参（fx,fy,cx,cy）、时间偏移 td、外参可优化标记

## 设计价值

CameraRig 把“有几台相机”和“状态估计怎么用相机”解耦。前端只需通过 camera id 找到外参，就能统一处理单目、双目和多目观测；后端也可以用同一套接口构造重投影因子，避免为 monocular/stereo/multi-camera 写多套重复代码。

## 工程边界

只管理外参还不够构成完整相机模型。实际 VIO/LVI 系统还需要相机内参、畸变模型、曝光参数、时间偏移、外参是否参与优化等配置。若这些字段分散在其他模块，CameraRig 的接口契约必须写清楚，否则很容易出现坐标方向或时间补偿不一致。

## 相关页面

- 实现于：[[组件-slam_fusion_core]] `camera_rig.hpp:10-26`
- 参考：open_vins CameraRig 设计
- [[传感器-传感器标定]]
- [[架构-多传感器融合架构]]
- [[相机数据管线]]
- [[组件-slam_fusion_core]]
