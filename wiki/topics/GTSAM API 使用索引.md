---
tags: [GTSAM, API, reference, cheat-sheet]
created: 2026-04-27
updated: 2026-05-29
superseded-by: [[方法-GTSAM-API族]]
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

> 本页内容已归并至 [[方法-GTSAM-API族]]。

# GTSAM API 使用索引

> 按“我要做什么”组织的 GTSAM API 查询入口。先定位类名，再跳转到对应 C++ API 页或专题页。全部 C++ 类签名可离线查阅。

## 版本与官方入口

- Python 官方文档：<https://borglab.github.io/gtsam/>（在线）
- C++ Doxygen：<https://gtsam.org/doxygen/>（在线；本地缓存见下方 API 速查表）
- 安装文档：<https://borglab.github.io/gtsam/install>（在线）
- 本地 C++ API：[[GTSAM C++ API 参考索引]] — 45 个核心类离线可查

## 最小工作流

```python
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X

graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

graph.addPriorPose2(X(0), gtsam.Pose2(0, 0, 0), prior_noise)
graph.add(gtsam.BetweenFactorPose2(X(0), X(1), gtsam.Pose2(1, 0, 0), odom_noise))

initial.insert(X(0), gtsam.Pose2(0.1, 0.0, 0.0))
initial.insert(X(1), gtsam.Pose2(1.1, 0.1, 0.02))

result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
print(result.atPose2(X(1)))
```

## 常用 API 速查

| 任务 | API | 说明 | 参考 |
|---|---|---|---|
| 创建变量 key | `symbol_shorthand.X/L/V/B/C`、`Symbol` | 生成可读 key，如 `x0`、`l1` | [[C++ API - Symbol]] |
| 保存初值/结果 | `Values.insert`、`atPose2`、`atPose3`、`atVector` | Python wrapper 常用 typed accessor | [[C++ API - Values]] |
| 建图 | `NonlinearFactorGraph()`、`graph.add(...)` | 容纳 nonlinear factors | [[C++ API - NonlinearFactorGraph]] |
| 添加 prior | `graph.addPriorPose2/Pose3`、`PriorFactor*` | 锚定 gauge freedom，提供先验 | [[C++ API - PriorFactor]] |
| 添加 odometry/relative pose | `BetweenFactorPose2`、`BetweenFactorPose3` | 约束两个同类型 Lie group 变量 | [[C++ API - BetweenFactor]] |
| batch 优化 | `LevenbergMarquardtOptimizer` | 鲁棒默认选择，适合非线性 SLAM | [[C++ API - LevenbergMarquardtOptimizer]] |
| batch 快速优化 | `GaussNewtonOptimizer` | 初值好且问题近似二次时快 | [[C++ API - GaussNewtonOptimizer]] |
| trust-region 优化 | `DoglegOptimizer` | Dogleg/trust region 方法 | [[C++ API - DoglegOptimizer]] |
| 增量优化 | `ISAM2.update`、`calculateEstimate*` | 在线 SLAM / incremental smoothing | [[C++ API - ISAM2]] |
| 不确定性 | `Marginals.marginalCovariance`、`jointMarginalCovariance` | batch result 的 covariance/information 查询 | [[C++ API - Marginals]] |
| 2D pose | `Pose2`、`Rot2` | planar SLAM 常用 | [[C++ API - Pose2]] |
| 3D pose | `Pose3`、`Rot3` | 3D SLAM/VIO/视觉常用 | [[C++ API - Pose3]] |
| 相机内参 | `Cal3_S2`、`Cal3_S2Stereo` | pinhole/stereo calibration | [[C++ API - Cal3_S2]] |
| 单目重投影 | `GenericProjectionFactorCal3_S2` | pose + landmark + fixed calibration | [[C++ API - GenericProjectionFactor]] |
| Smart factors | `SmartProjectionFactor*` | landmark 隐式三角化/边缘化 | [[C++ API - SmartProjectionFactor]] |
| Stereo | `GenericStereoFactor3D`、`StereoCamera` | stereo measurement 和 backprojection | [[C++ API - GenericStereoFactor]] |
| IMU 预积分参数 | `PreintegrationParams.MakeSharedU/D` | ENU/NED gravity + noise 配置 | [[C++ API - PreintegrationParams]] |
| IMU 预积分 | `PreintegratedImuMeasurements` | 积累 acc/gyro 到 PIM | [[C++ API - PreintegratedImuMeasurements]] |
| IMU factor | `ImuFactor`、`ImuFactor2` | pose/velocity/bias 约束 | [[C++ API - ImuFactorT]] |
| Combined IMU | `CombinedImuFactor` | 内建 bias evolution，15D covariance | [[C++ API - CombinedImuFactorT]] |
| GPS | `GPSFactor`、`GPSFactorArm`、`GPSFactor2` | local ENU/NED/ECEF position measurement | [[C++ API - GPSFactor]] |
| 自定义误差 | `CustomFactor` | Python 自定义 residual/Jacobian | [[GTSAM 自定义因子与 Jacobian]] |

## 查询建议

- 查“怎么建图/优化”：看 [[GTSAM 因子图工作流]]。
- 查 pose/rotation/calibration：看 [[GTSAM Geometry API]]。
- 查 optimizer、iSAM2、marginals：看 [[GTSAM Nonlinear 优化 API]]。
- 查 IMU/GPS/navigation：看 [[GTSAM Navigation 与 IMU API]]。
- 查 visual SLAM/SfM：看 [[GTSAM SLAM 与视觉因子 API]]。
- 写自己的 measurement factor：看 [[GTSAM 自定义因子与 Jacobian]]。

## 相关页面

- [[组件-GTSAM]]
- [[GTSAM 4.3a1 使用指南]]
