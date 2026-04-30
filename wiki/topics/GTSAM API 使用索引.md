---
tags: [GTSAM, API, reference, cheat-sheet]
created: 2026-04-27
updated: 2026-04-27
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

# GTSAM API 使用索引

> 按“我要做什么”组织的 GTSAM 4.3a1 API 查询入口。先从这里定位类，再跳到对应专题页和官方文档。

## 版本与官方入口

- Python 文档根目录：https://borglab.github.io/gtsam/
- C++ reference：https://gtsam.org/doxygen/
- 安装文档：https://borglab.github.io/gtsam/install
- Concepts：https://borglab.github.io/gtsam/gtsam-concepts

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

| 任务 | API | 说明 | 官方文档 |
|---|---|---|---|
| 创建变量 key | `symbol_shorthand.X/L/V/B/C`、`Symbol` | 生成可读 key，如 `x0`、`l1` | https://borglab.github.io/gtsam/symbol |
| 保存初值/结果 | `Values.insert`、`atPose2`、`atPose3`、`atVector` | Python wrapper 常用 typed accessor | [[GTSAM 因子图工作流]] |
| 建图 | `NonlinearFactorGraph()`、`graph.add(...)` | 容纳 nonlinear factors | https://borglab.github.io/gtsam/nonlinearfactorgraph |
| 添加 prior | `graph.addPriorPose2/Pose3`、`PriorFactor*` | 锚定 gauge freedom，提供先验 | https://borglab.github.io/gtsam/priorfactor |
| 添加 odometry/relative pose | `BetweenFactorPose2`、`BetweenFactorPose3` | 约束两个同类型 Lie group 变量 | https://borglab.github.io/gtsam/betweenfactor |
| batch 优化 | `LevenbergMarquardtOptimizer` | 鲁棒默认选择，适合非线性 SLAM | https://borglab.github.io/gtsam/levenbergmarquardtoptimizer |
| batch 快速优化 | `GaussNewtonOptimizer` | 初值好且问题近似二次时快 | https://borglab.github.io/gtsam/gaussnewtonoptimizer |
| trust-region 优化 | `DoglegOptimizer` | Dogleg/trust region 方法 | https://borglab.github.io/gtsam/doglegoptimizer |
| 增量优化 | `ISAM2.update`、`calculateEstimate*` | 在线 SLAM / incremental smoothing | https://borglab.github.io/gtsam/isam2 |
| 不确定性 | `Marginals.marginalCovariance`、`jointMarginalCovariance` | batch result 的 covariance/information 查询 | https://borglab.github.io/gtsam/marginals |
| 2D pose | `Pose2`、`Rot2` | planar SLAM 常用 | https://borglab.github.io/gtsam/pose2 |
| 3D pose | `Pose3`、`Rot3` | 3D SLAM/VIO/视觉常用 | https://borglab.github.io/gtsam/pose3 |
| 相机内参 | `Cal3_S2`、`Cal3_S2Stereo` | pinhole/stereo calibration | https://borglab.github.io/gtsam/cal3-s2 |
| 单目重投影 | `GenericProjectionFactorCal3_S2` | pose + landmark + fixed calibration | https://borglab.github.io/gtsam/projectionfactor |
| Smart factors | `SmartProjectionFactor*` | landmark 隐式三角化/边缘化 | https://borglab.github.io/gtsam/smartprojectionfactor |
| Stereo | `GenericStereoFactor3D`、`StereoCamera` | stereo measurement 和 backprojection | https://borglab.github.io/gtsam/stereofactor |
| IMU 预积分参数 | `PreintegrationParams.MakeSharedU/D` | ENU/NED gravity + noise 配置 | https://borglab.github.io/gtsam/preintegrationparams |
| IMU 预积分 | `PreintegratedImuMeasurements` | 积累 acc/gyro 到 PIM | https://borglab.github.io/gtsam/preintegratedimumeasurements |
| IMU factor | `ImuFactor`、`ImuFactor2` | pose/velocity/bias 约束 | https://borglab.github.io/gtsam/imufactor |
| Combined IMU | `CombinedImuFactor` | 内建 bias evolution，15D covariance | https://borglab.github.io/gtsam/combinedimufactor |
| GPS | `GPSFactor`、`GPSFactorArm`、`GPSFactor2` | local ENU/NED/ECEF position measurement | https://borglab.github.io/gtsam/gpsfactor |
| 自定义误差 | `CustomFactor` | Python 自定义 residual/Jacobian | https://borglab.github.io/gtsam/customfactor |

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
