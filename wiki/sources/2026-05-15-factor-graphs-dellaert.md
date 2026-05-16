---
tags: [因子图, 机器人感知, 非线性最小二乘, 高斯牛顿法, LM算法, 变量消除, QR分解, Dellaert, FnT]
type: source
sources:
  - https://doi.org/10.1561/2300000043
created: 2026-05-15
updated: 2026-05-16
paper_arxiv: null
paper_url: https://doi.org/10.1561/2300000043
paper_doi: 10.1561/2300000043
paper_authors: [Frank Dellaert, Michael Kaess]
paper_venue: Foundations and Trends in Robotics, vol. 6, no. 1-2, pp. 1-139, 2017
paper_isbn: 978-1-68083-308-9
paper_keywords: [factor graphs, robot perception, nonlinear least squares, Gauss-Newton, Levenberg-Marquardt, variable elimination, SLAM, sparse linear algebra]
---

# 机器人感知中的因子图

> Dellaert & Kaess 的 FnT Robotics 专著（2017），全面教程式阐述因子图作为 SLAM/机器人感知的
> 统一数学框架。从概率模型、非线性最小二乘、稀疏线性代数到增量推理，是全栈 SLAM 工程师必读教材。

## 论文信息

- **作者**：Frank Dellaert (Georgia Tech), Michael Kaess (CMU)
- **发表**：Foundations and Trends in Robotics (FnT), vol. 6, no. 1-2, pp. 1-139, 2017
- **DOI**：[10.1561/2300000043](https://doi.org/10.1561/2300000043)
- **免费版**：https://dellaert.github.io/publication/fnt-2017/
- **代码示例**：基于 [GTSAM](https://gtsam.org) 的 MATLAB/C++ 代码

## 核心贡献

1. **因子图作为 SLAM 统一语言**：将几乎所有机器人感知问题（位姿估计、建图、标定、多传感器融合）
   建模为因子图上的 MAP 估计问题，提供一致的数学和算法接口。

2. **从概率到优化到线性代数的完整链条**：
   概率模型 $\to$ 最大后验估计 $\to$ 非线性最小二乘 $\to$ 高斯牛顿迭代 $\to$ 稀疏线性系统求解
   $\to$ 变量消除 / QR 分解。每一步的等价性和推导均详细展开。

3. **稀疏代数与变量消除**：系统讲解如何利用因子图的稀疏性高效求解，包括 Cholesky 分解
   （`\` 算子）、QR 分解、变量消除与 fill-in 控制、COLAMD 排序。

4. **增量推理与 iSAM2**：将 Bayes Tree 增量求解器纳入教程体系，阐明批量消除 → Bayes Net →
   Bayes Tree 的数据结构转换链路。

5. **工程实践指导**：涵盖鲁棒范数（Huber, Cauchy, DCS 等）、马氏距离与信息矩阵、协方差恢复
   （Marginals）、退化检测等实际部署必需的技术细节。

## 关键数学公式

**MAP 推断 = 非线性最小二乘**：

$$\hat{\mathcal{X}} = \arg\min_{\mathcal{X}} \sum_i \|\mathbf{h}_i(\mathcal{X}_i) - \mathbf{z}_i\|^2_{\Sigma_i}$$

其中 $\mathbf{h}_i(\cdot)$ 为观测函数，$\mathbf{z}_i$ 为测量值，$\|\cdot\|^2_{\Sigma}$ 为
马氏距离。当且仅当噪声为高斯分布时，MAP 等价于最小二乘。

**高斯牛顿迭代步**：

$$\left(J^T \Sigma^{-1} J\right) \Delta = -J^T \Sigma^{-1} \mathbf{r}$$

$J = \partial\mathbf{h}/\partial\mathcal{X}$ 为 Jacobian，$\mathbf{r} = \mathbf{h}(\mathcal{X}) - \mathbf{z}$ 为残差。每次迭代 $J$ 和 $\mathbf{r}$ 在当前线性化点重新计算。

**Levenberg-Marquardt 阻尼**：

$$\left(J^T \Sigma^{-1} J + \lambda D\right) \Delta = -J^T \Sigma^{-1} \mathbf{r}$$

$\lambda$ 为阻尼因子，$D$ 通常取 $\text{diag}(J^T\Sigma^{-1}J)$。$\lambda$ 根据步长接受度
（actual/predicted reduction ratio）自适应调整。

**变量消除与 fill-in**：

$$\begin{bmatrix} A_{11} & A_{12} \\ A_{21} & A_{22} \end{bmatrix} \xrightarrow{\text{eliminate } x_1} \begin{bmatrix} R_{11} & S_{12} \\ 0 & A'_{22} \end{bmatrix}$$

消除 $x_1$ 后 Schur 补 $A'_{22} = A_{22} - A_{21}A_{11}^{-1}A_{12}$ 可能引入 fill-in，
变量排序是控制稀疏性的关键。

## 架构概述

```
传感器测量 z ← 机器人运动
      ↓
┌─────────────────────────────┐
│  因子图建模                  │
│  p(X|Z) ∝ Π f_i(X_i)       │
│  f_i = exp(-||h_i - z_i||²) │
└─────────────────────────────┘
      ↓
┌─────────────────────────────┐
│ 非线性优化 (Gauss-Newton/LM) │
│  J^T J Δ = -J^T r          │
└─────────────────────────────┘
      ↓
┌─────────────────────────────┐
│ 稀疏线性求解                  │
│  批量: Cholesky / QR         │
│  增量: iSAM2 (Bayes Tree)    │
└─────────────────────────────┘
      ↓
状态估计 X^* + 协方差 Σ
```

## 关键技术洞察

- 这本书的最大价值在于建立了"概率模型 → 优化问题 → 稀疏线性代数"的严格等价性，使读者能理解为什么
  一个 SLAM 系统的后端选型（g2o vs Ceres vs GTSAM）在数学上等价但在工程特性上不同。
- 实际实现时，Jacobian 的推导正确性直接决定优化收敛 — 数值 Jacobian 可验证解析 Jacobian，
  这是调试 SLAM 后端的第一步。
- 因子图的稀疏性（每个因子只连少量变量）是实时优化的底层原因。消除顺序决定了时间复杂度，
  这是 SLAM 后端工程师必须理解的"隐式知识"。

## 相关页面

- [[概念-因子图]]
- [[方法-ISAM2增量固定滞后平滑]]
- [[组件-GTSAM]]
- [[组件-Ceres-Solver]]
- [[概念-Schur补与边缘化]]
- [[因子图vs滤波]]
- [[GTSAM 因子图工作流]]
- [[GTSAM Nonlinear 优化 API]]
- [[2026-05-15-isam2]] — iSAM2 论文
- [[优化后端选型指南]]
