---
tags: [GTSAM, guide, SLAM, API]
created: 2026-04-27
updated: 2026-04-27
sources: [wiki/sources/2026-04-27-gtsam-4.3a1-docs.md]
---

# GTSAM 4.3a1 使用指南

> 综合官方 GTSAM 文档，为 ROS 机器人导航/SLAM 工程整理的使用导向指南。

## 一句话理解

GTSAM 不是“直接给你一个 SLAM 系统”，而是给你一套 factor graph optimization 工具：你把状态变量、传感器 measurement、误差模型和不确定性写成图，GTSAM 负责线性化、稀疏求解、增量更新和 covariance 查询。

## 推荐学习路径

1. 先看 [[GTSAM 因子图工作流]]：掌握 graph、Values、factor、noise model、optimizer 的主线。
2. 再看 [[GTSAM Geometry API]]：掌握 `Pose2`、`Pose3`、`Rot3`、`Cal3_S2`。
3. 做 pose graph：`PriorFactor` + `BetweenFactor` + LM/iSAM2。
4. 做视觉：projection/stereo/smart factors。
5. 做 VIO/INS：`NavState` + preintegration + IMU/GPS factors。
6. 最后再写 [[GTSAM 自定义因子与 Jacobian]]。

## 工程模板

```python
graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

# 1. keys
from gtsam.symbol_shorthand import X, V, B, L

# 2. noise
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([...]))
measurement_noise = gtsam.noiseModel.Isotropic.Sigma(dim, sigma)

# 3. factors
graph.addPriorPose3(X(0), initial_pose, prior_noise)
graph.add(gtsam.BetweenFactorPose3(X(0), X(1), odom, odom_noise))

# 4. initial values
initial.insert(X(0), initial_pose)
initial.insert(X(1), predicted_pose)

# 5. optimize
result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()

# 6. query
pose1 = result.atPose3(X(1))
marginals = gtsam.Marginals(graph, result)
cov1 = marginals.marginalCovariance(X(1))
```

## ROS/导航落地注意事项

- Frame convention 必须先定清：world/map/odom/base_link/imu/camera 的方向和外参要和 GTSAM factor 的假设一致。
- IMU gravity direction 要匹配 ENU/NED；`MakeSharedU` 和 `MakeSharedD` 不要混用。
- 时间同步和积分间隔 `dt` 错误会直接污染 preintegration。
- 初值比优化器选择更重要；VIO/visual SLAM 中 bad initial guess 很容易导致 cheirality、三角化或局部极小问题。
- bias 不要一开始就复杂化；官方 navigation 文档建议先让主 pipeline 工作，再逐步引入 bias evolution。
- 自定义 factor 必须用 numerical derivative 做单元测试。

## 后续查阅方式

- 查类名：先在 [[GTSAM API 使用索引]] 搜索。
- 查建图模式：看 [[GTSAM 因子图工作流]]。
- 查 IMU/GPS：看 [[GTSAM Navigation 与 IMU API]]。
- 查 projection/stereo/smart factor：看 [[GTSAM SLAM 与视觉因子 API]]。
- 查 Jacobian：看 [[GTSAM 自定义因子与 Jacobian]]。

## 相关页面

- [[组件-GTSAM]]
- [[GTSAM API 使用索引]]
- [[GTSAM Navigation 与 IMU API]]
