---
tags: [论文, SLAM, 深度学习, DenseBA, 光流, ConvGRU]
type: source
sources:
  - "https://arxiv.org/abs/2108.08266"
created: 2026-05-15
updated: 2026-05-15
paper_arxiv: "2108.08266"
paper_title: "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras"
paper_authors: "Zachary Teed, Jia Deng"
paper_venue: "NeurIPS 2021"
paper_url: "https://arxiv.org/abs/2108.08266"
paper_code: "https://github.com/princeton-vl/DROID-SLAM"
---

# DROID-SLAM 论文精读

> DROID-SLAM 是首个端到端可微的深度视觉 SLAM 系统。它用 RAFT 风格的迭代光流 + ConvGRU 更新取代传统特征匹配/三角化，用 GPU Dense BA 层的 Schur 补消元取代稀疏 BA，在单目/双目/RGB-D 三种模式下统一工作。

## 论文信息

| 项目 | 内容 |
|------|------|
| 标题 | DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras |
| 作者 | Zachary Teed, Jia Deng |
| 发表 | NeurIPS 2021 (Oral) |
| arXiv | [2108.08266](https://arxiv.org/abs/2108.08266) |
| 代码 | [princeton-vl/DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) |
| 机构 | Princeton University |

## 核心贡献

### 1. 迭代光流更新取代传统前端

不再检测特征点、匹配描述子、RANSAC PnP 求解，而是对每帧图像配对计算稠密相关体，ConvGRU 反复查询并预测光流残差 $\Delta \mathbf{u}$ 和置信度 $\Sigma$：

每个 GRU 迭代步输出三个量：光流修正 $\delta_{ij}$、置信度权重 $\Sigma_{ij}$（对角逆协方差）、damping factor $\lambda_{ij}$。

### 2. Dense Bundle Adjustment 层（DBA）

将网络预测的稠密光流与相机运动模型预测的光流之间的差异定义为 BA 残差。所有帧间、所有像素的残差共同形成一个大型非线性最小二乘问题，通过 Schur 补消去逐像素深度，只求解 $6 \times K$（$K$ 为关键帧数）位姿系统：

$$
\arg\min_{\mathbf{T}, \mathbf{d}} \sum_{(i,j) \in \mathcal{E}} \|\mathbf{p}_i^* - \Pi(\mathbf{T}_{ij}, d_i)\|^2_{\Sigma_{ij}}
$$

其中 $\mathbf{p}_i^* = \mathbf{p}_i + \mathbf{u}_{ij}$，$\mathbf{u}_{ij}$ 为网络预测的 frame $i \rightarrow j$ 光流，$\Sigma_{ij}$ 为网络输出的置信度对角矩阵。

### 3. 训练时 Schur 补 — 运行时 Cholesky 求解

**训练时在网络前向中包含 DBA 层**：光流预测 → DBA 优化 → Schur 补消深度 → Cholesky 解位姿 → 损失回传。这使网络学会预测能帮助 BA 稳定收敛的光流，而不是仅最小化光流本身的误差。

**运行时**：每到达新帧，构造 DBA 并求解，更新位姿和深度。Schur 补的关键性质是深度变量 Hessian 逐像素对角独立，求逆仅为逐元素倒数，时间复杂度 $O(K^3 + PK)$（$K$ 帧数，$P$ 像素数）。

### 4. ConvGRU 全局上下文调制

ConvGRU 隐状态经全局平均池化得到 $g \in \mathbb{R}^{1\times 1\times C}$ 的全局上下文描述子，注入门控偏置：

$$
z_t = \sigma(\text{Conv}([h_{t-1}, x_t]) + \alpha \odot g + \beta)
$$

$\alpha, \beta$ 为可学习参数。这种设计使局部迭代同时感知全局信息（如当前帧与远距离帧的关系）。

### 5. 统一多模态架构

单目 Stereo 和 RGB-D 仅在输入表示上有差异：单目只有单视图相关体，双目在两视图之间构造相关体，RGB-D 在后端给深度赋值并降低其 Hessian 权重。位姿估计和 BA 层完全共享。

## 关键公式

**DBA 目标函数（单帧对）**：

$$
E(\mathbf{T}, \mathbf{d}) = \sum_{i=1}^P \|\mathbf{p}_i + \mathbf{u}_{ij} - \Pi(\mathbf{T}_{ij} \circ \Pi^{-1}(\mathbf{p}_i, d_i))\|^2_{\Sigma_{ij}}
$$

$\mathbf{p}_i$ 为像素坐标，$d_i$ 为逆深度，$\Pi$ 为投影函数，$\mathbf{T}_{ij}$ 为帧 $i$ 到 $j$ 的相对位姿变换。

**Schur 补消元后的位姿系统**：

$$
(\mathbf{H}_{TT} - \mathbf{H}_{Td} \mathbf{H}_{dd}^{-1} \mathbf{H}_{dT}) \Delta \xi = \mathbf{b}_T - \mathbf{H}_{Td} \mathbf{H}_{dd}^{-1} \mathbf{b}_d
$$

$\mathbf{H}_{dd}$ 为对角矩阵（每个像素独立），$\mathbf{H}_{dd}^{-1}$ 计算为逐元素倒数，无需矩阵分解。

**逆深度更新的 damping factor**：

深度 Hessian 对角加上网络预测的 damping 防止发散：

$$
H_{dd}' = H_{dd} + \lambda_{ij}
$$

$\lambda_{ij} > 0$（经 softplus 保证），由 GraphAgg 网络模块根据当前优化状态动态预测。

**训练损失（三重损失 + 测地线距离）**：

$$
\mathcal{L} = \mathcal{L}_\text{pose} + \alpha \mathcal{L}_\text{flow} + \beta \mathcal{L}_\text{residual}
$$

测地线位姿损失 $d_{geo} = \sqrt{\|\log(T^{-1}\hat{T})\|^2}$，训练中多次随机重启（随机位姿初始化）配合课程学习逐步延长序列。

## 与知识库现有内容的联动

### 与 [[算法-DROID-SLAM]] 的关系

现有实体页侧重于与传统 SLAM 的差异和工程边界。本文档补充了 DBA 层的数学推导（Schur 补消元结构、阻尼因子、逐像素对角 Hessian 优化）、ConvGRU 全局上下文调制的具体公式，以及训练时的三重损失函数和随机重启策略。

### 与 [[方法-Dense BA]] 的关系

DROID-SLAM 是 Dense BA 概念最具影响力的实现。与传统稀疏 BA 不同，DBA 对全图像素定义残差，信息量远超 200-1000 个特征点。Schur 补消深度的对角性质使稠密 BA 的计算代价可控。

### 与 [[方法-RAFT光流]]、[[方法-RAFT光流SLAM]] 的关系

DROID-SLAM 的迭代光流预测继承自 RAFT 架构（twin encoder + 4D 相关体 + ConvGRU），但在三个方面做了 SLAM 定制：
1. 输出置信度权重 $\Sigma_{ij}$（而不是仅光流值）
2. 输出 damping factor $\lambda_{ij}$ 馈入 DBA
3. 隐状态注入全局上下文描述子

### 与 [[概念-深度学习SLAM]] 的关系

DROID-SLAM 是深度学习 SLAM 中训练和推理一致性最好的工作。网络前向包含完整的 BA 层，使训练目标（位姿误差）与推理目标完全一致，避免了"学 A 做 B"的 mismatch。

### 与 [[概念-Schur补与边缘化]] 的关系

传统 VIO 的 Schur 补用于边缘化旧帧（删除变量），而 DROID-SLAM 的 DBA 层中的 Schur 补是消去深度变量（保留约束信息）以降低系统维度。两者的数学工具相同但工程用途不同。

### 与 [[架构-滑动窗口优化]] 的关系

DROID-SLAM 使用 12 帧滑动窗口 + 过去帧的逆深度因子（depth prior），并通过因子图管理帧间边。运行时每帧触发 DBA 求解并边缘化旧深度信息为深度先验。

### 与 [[算法-NICE-SLAM]] 的对比

| 维度 | DROID-SLAM | NICE-SLAM |
|------|-----------|------------|
| 观测 | 网络预测光流 | 渲染 RGB-D vs 传感器 |
| 地图 | 逐像素逆深度 | SDF 隐式场 |
| 后端 | DBA（Schur + Cholesky） | Adam（体积渲染梯度） |
| 训练 | 端到端（含 DBA 层） | 非学习式（无训练） |
| 传感器 | Mono/Stereo/RGB-D | RGB-D only |

## Agent 实现提示

### 适用场景

DROID-SLAM 的核心创新——DBA 层的 Schur 补消元 + 网络置信度加权——可用于设计新的可微优化模块。当需要设计一个"可学习优化器"时，DBA 的 Schur 结构是参考模板。适用于有 GPU 的定位系统，对无 GPU 或嵌入式平台不适用。

### 输入输出契约

- **输入**：图像序列（tensor, B×H×W×3），相机内参 K（可在线标定），初始位姿（可为 identity 或上一帧结果）
- **输出**：每帧位姿 $T \in SE(3)$，逐像素逆深度图 $d \in \mathbb{R}^{H\times W}$，GRU 隐状态用于下一帧
- **坐标**：归一化平面像素坐标经 $K^{-1}$ 变换，帧间相对运动在 SE(3) 流形上
- **单位**：位姿使用绝对尺度（训练时用 GT 尺度），逆深度 $1/d$ 单位为米

### 实现骨架（伪代码）

```pseudo
function droid_slam_frontend(images, K):
    states = [(T_0=identity, d_0=1.0, hidden_0=0)]  # 初始状态
    keyframe_indices = [0]

    for t in range(1, len(images)):
        # Step 1: 多帧光流迭代
        flow, weight, damping = iterative_update(
            images[keyframe_indices + [t]],
            states[-1].hidden,
            correlation_volumes)

        # Step 2: Dense BA (GPU Schur complement)
        T_new, d_new = dense_ba(
            flow, weight, damping,
            states, keyframe_indices)

        states.append(T_new, d_new, hidden_new)

        # Step 3: 关键帧选择与因子图管理
        if is_keyframe(T_new, keyframe_indices):
            keyframe_indices.append(t)
            marginalize_old_depth(keyframe_indices)

    return states, keyframe_indices
```

### 关键源码片段

- `droid_slam/droid_net.py:L172-L222` — 完整网络前向（特征提取 → 相关体 → ConvGRU 迭代 → DBA）
- `droid_slam/geom/ba.py:L31-L106` — DBA 实现（Schur 补 → Cholesky 求解 → 逆深度更新）
- `droid_slam/geom/chol.py:L46-L73` — GPU Cholesky 求解器（针对 6×K 位姿系统）
- `droid_slam/modules/gru.py:L5-L32` — ConvGRU 实现（含全局上下文注入）
- `droid_slam/factor_graph.py:L19-L50` — 因子图构建（帧间边 + 逆深度先验）

### 实现注意事项

- **深度 Hessian 对角假设**：DBA 中逐像素深度 Hessian 是对角矩阵的假设在实际中是近似（像素间通过光流 Jacobian 有微弱耦合）。在深度剧烈变化的场景（如近距离大视差）中，这个近似可能引入额外误差。
- **damping 因子的学习稳定性**：GraphAgg 预测的 $\lambda_{ij}$ 必须经 softplus 保证正值，否则 Cholesky 分解失败。
- **单目尺度歧义**：DROID-SLAM 单目的绝对尺度来自训练数据的 overfitting，对域外场景不保证，需要外部尺度信息（如 IMU、已知基线）。
- **GPU 依赖**：整个 pipeline（相关体、GRU、DBA）均在 GPU 上运行，不适合嵌入式/低功耗场景。
- **动态场景鲁棒性**：ConvGRU 的置信度预测可以在一定程度上抑制动态物体（低 $\Sigma$），但大规模动态仍会影响 BA。

### 源码检索锚点

- `droid_slam/droid_net.py` — 网络定义（主干 + GRU + DBA）
- `droid_slam/geom/ba.py` — Dense BA 的 Schur 补消元实现
- `droid_slam/geom/chol.py` — GPU Cholesky 求解器
- `droid_slam/geom/projective_ops.py` — 投影变换与 Jacobian
- `droid_slam/modules/gru.py` — ConvGRU（全局上下文调制）
- `droid_slam/factor_graph.py` — 因子图 + 滑动窗口管理
- `droid_slam/geom/losses.py` — 训练损失函数

## 相关页面

- [[算法-DROID-SLAM]] — 实体页（工程边界 + 传统对比）
- [[2026-04-28-droid-slam-analysis]] — 既有源码级分析摘要
- [[方法-Dense BA]] — Dense BA 概念（传统 vs 稠密对比）
- [[方法-RAFT光流]] — RAFT 光流架构
- [[方法-RAFT光流SLAM]] — RAFT 光流在 SLAM 中的应用
- [[概念-深度学习SLAM]] — 深度学习 SLAM 全貌
- [[概念-Schur补与边缘化]] — Schur 补数学基础与传统边缘化
- [[架构-滑动窗口优化]] — 滑动窗口优化管理
- [[方法-ConvGRU]] — ConvGRU 网络模块
- [[VIO方案对比]] — VIO/VO 方案族对比
- [[算法-NICE-SLAM]] — 可微渲染 RGB-D SLAM 对比
