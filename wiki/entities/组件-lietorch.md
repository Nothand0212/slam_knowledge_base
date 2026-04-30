---
tags: [组件, 深度学习, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
---

# lietorch

> 面向 PyTorch 的 Lie 群张量库，让 DROID-SLAM 能在 GPU 上批量处理 SE(3) 位姿、投影和雅可比。

## 定义

lietorch 是为深度学习 SLAM 提供 Lie 群运算的 PyTorch 库。它把 SE(3)、SO(3) 等位姿对象做成可批量计算、可自动微分的张量结构，使网络输出、几何投影和优化层可以在同一个 GPU 计算图中运行。

## 在 DROID-SLAM 中的作用

DROID-SLAM 的 projective transform 需要大量帧对之间的位姿组合、逆变换、投影和雅可比计算。lietorch 提供：

- SE(3) 指数映射和 retraction，例如 `pose_retr`。
- 伴随变换 `adjT`，用于扰动和 Jacobian 传播。
- 批量投影变换及其显式 Jacobian。
- 与 PyTorch autograd 兼容的 GPU 张量接口。

## 与传统 SLAM 库的区别

| 维度 | GTSAM/Ceres | lietorch |
|------|-------------|----------|
| 主要场景 | C++ 非线性优化、因子图 | PyTorch 深度模型和可微几何 |
| 执行方式 | CPU/稀疏线性代数为主 | GPU 批量张量计算 |
| 优势 | 工程成熟、数值工具完整 | 易接入网络训练和 dense BA |
| 风险 | 与深度网络集成成本高 | 调试和数值可解释性弱 |

## 相关页面

- [[算法-DROID-SLAM]]
- [[数学-流形优化]]
- [[数学-SE3指数映射]]
- [[方法-Dense BA]]
- [[概念-深度学习SLAM]]
