---
tags: [GTSAM, nonlinear-optimization, ISAM2, Marginals]
created: 2026-04-27
updated: 2026-05-15
superseded-by: [[方法-GTSAM-API族]]
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

> 本页内容已归并至 [[方法-GTSAM-API族]]。

# GTSAM Nonlinear 优化 API

> 非线性优化模块负责把 factor graph 线性化、求解增量并更新 `Values`。工程上常用 LM 做 batch，用 ISAM2 做 online/incremental。

## Nonlinear 模块清单

官方文档：https://borglab.github.io/gtsam/nonlinear

核心类：

- `NonlinearFactorGraph`
- `NonlinearFactor`
- `NoiseModelFactor`
- `Values`
- `NonlinearOptimizer`
- `GaussNewtonOptimizer`
- `LevenbergMarquardtOptimizer`
- `DoglegOptimizer`
- `GncOptimizer`
- `ISAM2`
- `Marginals`

## NonlinearFactorGraph

官方文档：https://borglab.github.io/gtsam/nonlinearfactorgraph

常用方法：

- `add(factor)`
- `addPriorPose2(key, value, noise)`
- `addPriorPose3(key, value, noise)`
- `size()`、`empty()`
- `at(i)`、`front()`、`back()`
- `linearize(values)`
- `error(values)`

## Optimizer 选择

| Optimizer | 适用场景 | 注意 |
|---|---|---|
| `LevenbergMarquardtOptimizer` | 默认 batch 选择；非线性较强或初值一般 | damping 更稳，但参数多 |
| `GaussNewtonOptimizer` | 初值好、问题接近二次、希望更快 | 对 bad initial guess 更敏感 |
| `DoglegOptimizer` | trust-region 风格，结合 steepest descent 与 Gauss-Newton | 需要理解 trust region 参数 |
| `GncOptimizer` | robust optimization / outlier 问题 | 用于非凸 robust cost |

## LevenbergMarquardtParams

官方文档：https://borglab.github.io/gtsam/levenbergmarquardtoptimizer

常用参数：

- `lambdaInitial`
- `lambdaFactor`
- `lambdaUpperBound`
- `lambdaLowerBound`
- `verbosityLM`
- `minModelFidelity`
- `diagonalDamping`
- `maxIterations`
- `relativeErrorTol`
- `absoluteErrorTol`
- `orderingType`
- `linearSolverType`

模板：

```python
params = gtsam.LevenbergMarquardtParams()
params.setVerbosityLM("SUMMARY")  # wrapper 支持情况以实际版本为准
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
result = optimizer.optimize()
```

## ISAM2

官方文档：https://borglab.github.io/gtsam/isam2

用途：增量式 SLAM。每次加入新 factors 和新 variables，`ISAM2.update` 更新 Bayes tree 和估计。

常用 API：

- `ISAM2Params()`
- `ISAM2(params)`
- `update(new_factors, new_values)`
- `calculateEstimate()`
- `calculateEstimatePose2(key)` / `calculateEstimatePose3(key)` / typed estimate accessors
- `marginalFactor(key)`
- `marginalInformation(key)`
- `marginalCovariance(key)`
- `jointMarginalInformation(keys)`
- `jointMarginalCovariance(keys)`
- `getFactorsUnsafe()`：高级调试，不建议普通流程依赖

典型模式：

```python
isam = gtsam.ISAM2(gtsam.ISAM2Params())

new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()
# fill new_factors/new_values
isam.update(new_factors, new_values)

estimate = isam.calculateEstimate()
x = isam.calculateEstimatePose3(X(k))
cov = isam.marginalCovariance(X(k))
```

## Marginals

官方文档：https://borglab.github.io/gtsam/marginals

用途：在 batch result 周围计算 Gaussian marginal covariance/information。

常用 API：

- `Marginals(graph, result)`
- `marginalCovariance(key)`
- `marginalInformation(key)`
- `jointMarginalCovariance(keys)`
- `jointMarginalInformation(keys)`
- `JointMarginal.fullMatrix()`
- `JointMarginal.at(key_i, key_j)`

建议：查询 block 时优先用 `at(key_i, key_j)`，不要依赖 full matrix 的内部 block 排列。

## 相关页面

- [[GTSAM 因子图工作流]]
- [[GTSAM API 使用索引]]
