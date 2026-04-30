---
tags: [LiDAR, 连续时间, ICP, 激光SLAM]
sources:
  - wiki/sources/2026-04-29-ct_icp-analysis.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
created: 2026-04-28
type: entity
updated: 2026-04-29
---

# CT-ICP

> CT-ICP（Continuous-Time ICP）将LiDAR扫描内运动建模为连续时间线性插值，在ICP配准中联合优化起始和终止位姿，自然消除运动畸变，无需IMU。

## 核心方法

CT-ICP 将一帧 LiDAR 扫描期间的轨迹建模为两个端点位姿之间的连续函数，而不是把整帧点云绑定到一个离散位姿。每个点根据自身时间戳 `alpha in [0,1]` 查询帧内位姿，例如用平移线性插值和旋转 slerp：

```text
T(alpha) = interp(T_begin, T_end, alpha)
```

优化变量因此从单帧 6-DoF 位姿扩展为 begin pose + end pose 的 12-DoF 轨迹片段。配准残差可使用点到平面、点到点、点到线或点到分布距离，并配合运动先验约束防止端点位姿发生不合理跳变。

## 关键设计

- 连续时间轨迹：把去畸变和配准放进同一个优化问题，而不是先 deskew 再 ICP。
- 运动先验：位置一致性、常速度、朝向和小速度约束用于抑制弱纹理场景中的漂移。
- 鲁棒注册：配准失败时可缩小体素、扩大邻域或切换鲁棒核。
- 协方差输出：Hessian 近似可用于估计当前 scan matching 的不确定性。

## 适用边界

CT-ICP 的优势是纯 LiDAR 即可处理扫描畸变，对没有可靠 IMU 的平台很有价值。限制是必须有每点时间戳，且它本质上仍是里程计，没有内置回环和全局图优化；长距离运行仍需要 [[概念-回环检测方法]] 和 [[概念-位姿图优化]] 补偿累计漂移。

## 相关页面
- [[2026-04-28-ct-icp-analysis]]
- [[LiDAR方案对比]]
- [[概念-连续时间轨迹]]
- [[方法-连续时间线性插值]], [[方法-运动先验约束]]
- [[方法-RobustRegistration]], [[方法-POINT_TO_DISTRIBUTION]]
