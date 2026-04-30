---
type: entity
tags: [Ceres, API, 流形优化, 版本演进]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-28-ob_gins-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
---

# Ceres Manifold API

> Ceres 2.1+ 的流形参数化接口，用 6 维切空间更新 7 维位姿参数块，替代旧 LocalParameterization。

## 定义

Ceres Solver 2.1.0 引入的流形接口，替代旧版 LocalParameterization API。OB_GINS 已适配 Manifold，而 IC-GVINS 仍使用 LocalParameterization。

## 核心特征

- `ceres::Manifold` 替代 `ceres::LocalParameterization`
- 需实现：AmbientSize()、TangentSize()、Plus()、PlusJacobian()、Minus()、MinusJacobian()
- PoseManifold: AmbientSize=7 (t+q), TangentSize=6 (dt+dθ)
- Plus 实现：p' = p + dp, q' = (q * exp(dθ)).normalized()
- 对 phad_fusion 的影响：如果使用 Ceres 2.1+ 应参考 OB_GINS 而非 IC-GVINS 的实现

## 相关页面

- 实现于：[[算法-OB_GINS]]（pose_manifold.h）
- 对比：[[数学-流形优化]]
- 注意：Ceres 2.1+ 中 LocalParameterization 已被标记为 deprecated
- 外部核验：Ceres 官方建模文档使用 `Manifold` 表达非欧参数块的切空间更新接口
- [[组件-Ceres-Solver]]
- [[数学-SE3指数映射]]
