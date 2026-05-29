---
type: entity
tags: [GTSAM, C++ API, index]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
---

# GTSAM C++ API 参考索引

> 自动生成自 Doxygen XML | GTSAM `develop` 分支 | 2026-05-29

本文档是从 GTSAM 源码通过 Doxygen 生成的 C++ API 参考。包含 45 个核心类的构造函数、方法签名和参数说明。

在线 C++ 文档: [https://gtsam.org/doxygen/](https://gtsam.org/doxygen/)

## Geometry

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - Pose2]] | class | gtsam/geometry/Pose2.h |
| [[C++ API - Pose3]] | class | gtsam/geometry/Pose3.h |
| [[C++ API - Rot2]] | class | gtsam/geometry/Rot2.h |
| [[C++ API - Rot3]] | class | gtsam/geometry/Rot3.h |
| [[C++ API - Cal3_S2]] | class | gtsam/geometry/Cal3_S2.h |
| [[C++ API - Cal3_S2Stereo]] | class | gtsam/geometry/Cal3_S2Stereo.h |
| [[C++ API - StereoCamera]] | class | gtsam/geometry/StereoCamera.h |
| [[C++ API - PinholeCamera]] | class | gtsam/geometry/PinholeCamera.h |

## FactorGraph

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - NonlinearFactorGraph]] | class | gtsam/factorgraph/NonlinearFactorGraph.h |
| [[C++ API - Values]] | class | gtsam/factorgraph/Values.h |
| [[C++ API - Symbol]] | class | gtsam/factorgraph/Symbol.h |
| [[C++ API - GaussianFactorGraph]] | class | gtsam/factorgraph/GaussianFactorGraph.h |

## Optimization

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - LevenbergMarquardtOptimizer]] | class | gtsam/optimization/LevenbergMarquardtOptimizer.h |
| [[C++ API - GaussNewtonOptimizer]] | class | gtsam/optimization/GaussNewtonOptimizer.h |
| [[C++ API - DoglegOptimizer]] | class | gtsam/optimization/DoglegOptimizer.h |
| [[C++ API - LevenbergMarquardtParams]] | class | gtsam/optimization/LevenbergMarquardtParams.h |
| [[C++ API - GaussNewtonParams]] | class | gtsam/optimization/GaussNewtonParams.h |
| [[C++ API - IterativeOptimizationParameters]] | class | gtsam/optimization/IterativeOptimizationParameters.h |

## ISAM2

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - ISAM2]] | class | gtsam/isam2/ISAM2.h |
| [[C++ API - ISAM2Params]] | class | gtsam/isam2/ISAM2Params.h |
| [[C++ API - ISAM2Result]] | class | gtsam/isam2/ISAM2Result.h |
| [[C++ API - IncrementalFixedLagSmoother]] | class | gtsam/isam2/IncrementalFixedLagSmoother.h |

## SLAM_Factors

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - PriorFactor]] | class | gtsam/slam_factors/PriorFactor.h |
| [[C++ API - BetweenFactor]] | class | gtsam/slam_factors/BetweenFactor.h |
| [[C++ API - GenericProjectionFactor]] | class | gtsam/slam_factors/GenericProjectionFactor.h |
| [[C++ API - SmartProjectionFactor]] | class | gtsam/slam_factors/SmartProjectionFactor.h |
| [[C++ API - SmartProjectionPoseFactor]] | class | gtsam/slam_factors/SmartProjectionPoseFactor.h |
| [[C++ API - GenericStereoFactor]] | class | gtsam/slam_factors/GenericStereoFactor.h |
| [[C++ API - NonlinearEquality]] | class | gtsam/slam_factors/NonlinearEquality.h |

## Navigation

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - NavState]] | class | gtsam/navigation/NavState.h |
| [[C++ API - PreintegrationParams]] | class | gtsam/navigation/PreintegrationParams.h |
| [[C++ API - PreintegratedImuMeasurements]] | class | gtsam/navigation/PreintegratedImuMeasurements.h |
| [[C++ API - ImuFactorT]] | class | gtsam/navigation/ImuFactorT.h |
| [[C++ API - CombinedImuFactorT]] | class | gtsam/navigation/CombinedImuFactorT.h |
| [[C++ API - ImuFactor2T]] | class | gtsam/navigation/ImuFactor2T.h |
| [[C++ API - GPSFactor]] | class | gtsam/navigation/GPSFactor.h |
| [[C++ API - GPSFactor2]] | class | gtsam/navigation/GPSFactor2.h |
| [[C++ API - ConstantBias]] | class | gtsam/navigation/ConstantBias.h |
| [[C++ API - PreintegratedAhrsMeasurements]] | class | gtsam/navigation/PreintegratedAhrsMeasurements.h |

## Inference

| 类 | 类型 | 头文件 |
|----|------|--------|
| [[C++ API - BayesNet]] | class | gtsam/inference/BayesNet.h |
| [[C++ API - BayesTree]] | class | gtsam/inference/BayesTree.h |
| [[C++ API - HessianFactor]] | class | gtsam/inference/HessianFactor.h |
| [[C++ API - JacobianFactor]] | class | gtsam/inference/JacobianFactor.h |
| [[C++ API - Marginals]] | class | gtsam/inference/Marginals.h |
| [[C++ API - Ordering]] | class | gtsam/inference/Ordering.h |

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
