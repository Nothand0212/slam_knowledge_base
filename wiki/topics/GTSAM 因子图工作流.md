---
tags: [GTSAM, factor-graph, workflow, SLAM]
created: 2026-04-27
updated: 2026-05-15
superseded-by: [[方法-GTSAM-API族]]
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

> 本页内容已归并至 [[方法-GTSAM-API族]]。

# GTSAM 因子图工作流

> GTSAM 的工程使用主线：定义 keys，建 `NonlinearFactorGraph`，填 `Values`，选择 optimizer/iSAM2，读取结果和 covariance。

## 核心对象

- `NonlinearFactorGraph`：factor 容器，常用 `add`、`addPriorPose2/Pose3`、`size`、`linearize`、`error`。
- `Values`：变量容器，常用 `insert`、`atPose2`、`atPose3`、`atVector`。
- `noiseModel`：噪声模型，常用 `Diagonal.Sigmas`、`Isotropic.Sigma`、`Unit.Create`、`Constrained`。
- `Key`：变量 ID；Python 常用 `X(0)`、`V(0)`、`B(0)`、`L(1)`。
- `Factor`：measurement/constraint；常用 `PriorFactor`、`BetweenFactor`、projection factor、IMU factor。

## Batch SLAM 模板

```python
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, L

graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

graph.addPriorPose2(X(0), gtsam.Pose2(0, 0, 0), prior_noise)
graph.add(gtsam.BetweenFactorPose2(X(0), X(1), gtsam.Pose2(1, 0, 0), odom_noise))

initial.insert(X(0), gtsam.Pose2(0, 0, 0))
initial.insert(X(1), gtsam.Pose2(1.1, 0.1, 0.02))

params = gtsam.LevenbergMarquardtParams()
result = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()
```

## 增量 SLAM 模板

```python
params = gtsam.ISAM2Params()
isam = gtsam.ISAM2(params)

new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()

new_factors.addPriorPose2(X(0), gtsam.Pose2(0, 0, 0), prior_noise)
new_values.insert(X(0), gtsam.Pose2(0, 0, 0))
isam.update(new_factors, new_values)

estimate = isam.calculateEstimate()
x0 = isam.calculateEstimatePose2(X(0))
cov = isam.marginalCovariance(X(0))
```

## Gauge Freedom 与 prior

SLAM 因子图常有 gauge freedom：没有绝对参考时，整张图可整体平移/旋转而 error 不变。工程上通常对首帧 pose 加 `PriorFactor` 或 `NonlinearEquality`：

- `PriorFactor`：软约束，受 noise model 权重影响。
- `NonlinearEquality`：强约束；官方 prior 文档提醒强 prior 可能使线性系统 condition number 变差，必要时考虑 equality。

## noise model 选择

- `Diagonal.Sigmas(sigmas)`：每个 residual 分量有不同 sigma，最常用。
- `Isotropic.Sigma(dim, sigma)`：各维同方差。
- `Unit.Create(dim)`：单位噪声。
- `Constrained`：硬约束/近似硬约束。

经验规则：

- residual 维度必须匹配 noise model 维度。
- sigma 越小，约束越强。
- 不要用极小 sigma 掩盖建模错误；先确认坐标系、单位、时间戳和外参。

## 常见调试顺序

1. 先打印 graph size 和 initial size，确认 key 没漏。
2. 对单个 factor 调 `error(values)` 或 `unwhitenedError(values)`。
3. 对自定义 factor 单元测试 numerical Jacobian。
4. 对优化前后调用 `graph.error(initial)` 和 `graph.error(result)`。
5. 对 3D/VIO 问题检查 frame convention、gravity direction、IMU body_P_sensor。

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM Nonlinear 优化 API]]
- [[GTSAM 自定义因子与 Jacobian]]
