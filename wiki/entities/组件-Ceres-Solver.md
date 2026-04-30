---
tags: [优化, Ceres, 非线性]
sources:
  - wiki/sources/2026-04-29-vins-fusion-analysis-analysis.md
created: 2026-04-29
updated: 2026-04-29
type: entity
---

# Ceres-Solver

> Google 开源的 C++ 非线性最小二乘优化库，支持自动求导、鲁棒核函数和多种求解器，广泛用于 SLAM 中的 Bundle Adjustment 和位姿图优化。

## 核心抽象

Ceres Solver 的基本建模单元是 `Problem`、参数块和残差块。用户把位姿、速度、偏置、逆深度或外参注册为参数块，再把重投影误差、IMU 残差、相对位姿误差等写成 CostFunction。Ceres 负责线性化、组装正规方程、选择线性求解器并执行 LM 或 Dogleg 迭代。

常用能力包括 AutoDiff、NumericDiff、手写 analytic Jacobian、Huber/Cauchy/Tukey 等鲁棒核，以及 `DENSE_QR`、`SPARSE_NORMAL_CHOLESKY`、`ITERATIVE_SCHUR` 等线性求解器。对 SE(3)、四元数和单位球面等非欧变量，需要通过 Manifold/LocalParameterization 保证更新仍落在合法流形上。

## 在 SLAM 中的应用

VINS-Fusion 使用 Ceres 构建滑动窗口视觉惯性优化，把重投影误差、IMU 预积分、边缘化先验和外参/时间偏移放进同一问题；Cartographer 使用 Ceres 做局部 scan matching 和后端约束优化；一些多传感器融合工程会用 Ceres 把 LIO 里程计、GNSS 位置/姿态/速度约束和先验因子统一成批优化或固定窗口优化。

与 GTSAM 相比，Ceres 的优势是 CostFunction 灵活、自动求导方便、工程接入成本低；劣势是因子图语义、增量推理、变量消元顺序和边缘化管理需要用户自己组织。对于在线 SLAM，Ceres 更常用于滑窗优化或局部匹配，GTSAM/iSAM2 更常用于增量因子图后端。

## 建模注意

- 四元数不要直接用 4 维欧式加法更新，应使用 Ceres Manifold。
- 鲁棒核只能降低外点影响，不能替代前端几何验证。
- 信息矩阵和残差单位必须一致，否则某一类传感器会在优化中压制其他约束。
- 边缘化先验若手写实现，需要固定线性化点并维护稠密先验块。

## 相关页面

- [[组件-GTSAM]], [[概念-因子图]]
- [[架构-滑动窗口优化]], [[概念-位姿图优化]]
- [[概念-Schur补与边缘化]], [[组件-Ceres Manifold API]]
