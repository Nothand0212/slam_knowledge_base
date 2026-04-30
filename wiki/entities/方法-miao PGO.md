---
tags: [PGO, 位姿图, 优化, 回环]
sources:
  - wiki/sources/2026-04-29-lightning_lm_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# miao PGO

> Lightning-LM 自研位姿图优化库，从 g2o 派生，更轻更快，支持增量模式。

## 定位

miao PGO 是 Lightning-LM 中用于全局后端和回环修正的轻量级位姿图优化库。它承担的不是每帧 LIO 配准，而是把关键帧里程计边、回环边和少量地标约束组织成稀疏图，在回环出现后修正全局轨迹。

## 技术参数
- 优化算法：Levenberg-Marquardt
- 线性求解器：Sparse Eigen
- 增量模式：`incremental_mode_ = true`
- 最大迭代：20 次
- SetupOptimizer<6, 3>：6-DOF 顶点 SE3，3-DOF 地标点

## 外点处理
- 回环边使用 Cauchy 鲁棒核 (delta=1.04)
- chi² 检验：`chi²(edge) > Delta(kernel)` → SetLevel(1) 屏蔽该边
- 收敛后去除鲁棒核（RemoveRobustKernel）

## 工程边界

自研 PGO 的优势是依赖少、接口可控、便于嵌入现有 LIO 工程；代价是要自己维护数值稳定性、鲁棒核策略和增量更新语义。回环边必须经过 NDT/ICP 等几何验证后再进入图，否则再轻量的优化器也无法抵消错误约束。

## 相关页面

- [[组件-lightning-lm]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]
- [[方法-多分辨率NDT回环]]
