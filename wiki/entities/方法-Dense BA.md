---
tags: [方法, 优化, 深度学习, SLAM]
type: entity
created: 2026-04-29
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
---

# Dense BA

> DROID-SLAM 中的稠密光束法平差：用逐像素光流残差替代稀疏重投影残差，并通过 Schur 补高效消去深度变量。

## 定义

Dense BA（Dense Bundle Adjustment）把传统 BA 的“稀疏特征点重投影误差”扩展为“稠密像素/光流残差”。DROID-SLAM 在 1/8 分辨率特征图上，为大量像素建立帧间几何约束，同时优化相机位姿和逆深度。

## 残差模型

对帧 `i` 中像素 `p_i` 及其逆深度 `d_i`，用当前位姿估计投影到帧 `j`：

```text
coords = Π(G_j G_i^(-1) Π^(-1)(p_i, d_i))
```

网络更新模块预测目标光流位置 `target` 和置信度权重，残差为：

```text
r = target - coords
```

这与传统 BA 的区别是：匹配目标不是手工特征对应点，而是 ConvGRU/相关体迭代预测出来的稠密流场。

## Schur 补求解

Dense BA 同时含有位姿变量和每像素深度变量。若直接求解，变量规模很大。DROID-SLAM 利用深度 Hessian 近似逐像素独立的结构，通过 Schur 补消去深度，只保留位姿系统：

- 深度块 `C` 接近对角，`C^(-1)` 可逐元素计算。
- 消元后只需求解 `P × 6` 维位姿增量。
- 深度再通过回代更新。

这使得“稠密约束”可以在 GPU 上以固定迭代次数运行，而不是退化成不可承受的全量 BA。

## 工程意义

- 稠密残差减少对角点/纹理点提取的依赖。
- 网络置信度替代了传统 RANSAC/鲁棒核的一部分职责。
- 计算依赖 GPU 和张量化实现，不适合直接移植到轻量 C++ 前端。
- 输出的位姿系统仍然保留 BA 的几何结构，因此比纯端到端回归更可解释。

## Agent 实现提示

### 适用场景

在稠密光流匹配而非稀疏特征点建立帧间约束的场景：学习型 SLAM 后端、GPU 端的多视角 BA、需要逐像素置信度加权的全局优化。适合用 Schur 补消去深度变量后在位姿流形上迭代的情况。

### 输入输出契约

- 输入：光流目标 `target` (B×N×ht×wd)，置信度权重 `weight` (B×N×ht×wd)，阻尼因子 `eta`，当前位姿 `poses`（SE3 对象），逆深度 `disps` (B×P×ht×wd)，内参 `intrinsics`，帧对索引 `ii,jj`（shape N），固定帧数 `fixedp`，rig 数 `rig`。
- 输出：更新后的 `poses`（retract 在流形上）和 `disps`（clamped 到 [0, 10]）。
- 前置条件：`poses.manifold_dim` 返回正确的流形维数；外点权重 `weight` 已在 ConvGRU 迭代中更新；`ii,jj` 中无负索引。
- 坐标系约定：帧 i 投影到帧 j 的像素坐标，深度定义在帧 i 坐标系中。

### 实现骨架（伪代码）

```text
function DenseBA(target, weight, eta, poses, disps, intrinsics, ii, jj, fixedp):
    B, P, ht, wd = disps.shape  // batch, keyframe数, H, W
    N = ii.shape[0]              // 帧对数量
    D = poses.manifold_dim       // SE3 流形维数 (通常 6)

    // 1. 计算投影 Jacobian 和残差
    coords, valid, (Ji, Jj, Jz) = projective_transform(poses, disps, ii, jj, jacobian=True)
    r = target - coords
    w = 0.001 * valid * weight    // 有效掩码 + 置信度加权

    // 2. 构架位姿 Hessian (Hii, Hij, Hji, Hjj) 和交叉项 E
    Hii = (wJi)ᵀ @ Ji;  Hij = (wJi)ᵀ @ Jj
    Hji = (wJj)ᵀ @ Ji;  Hjj = (wJj)ᵀ @ Jj
    vi  = (wJi)ᵀ @ r;   vj  = (wJj)ᵀ @ r
    Ei  = (wJi)ᵀ @ Jz;  Ej  = (wJj)ᵀ @ Jz

    // 深度 Hessian C 近似对角：逐像素 C = w * Jz²
    C = sum(w * Jz * Jz, dim=-1) + eta  // eta 作为阻尼项防止奇异

    // 3. Scatter 到关键帧维度的全局系统
    H = scatter_sum(Hii + Hij + Hji + Hjj, ii, jj)  // P×P×D×D
    E = scatter_sum(Ei + Ej, ii, kk)                // P×M×D×HW
    v = scatter_sum(vi + vj, ii)                    // P×D

    // 4. Schur 补求解
    S = H - E * (1/C) * Eᵀ           // 消去深度，位姿 Schur 补
    dx = cholesky_solve(S, v - E * (1/C) * w)  // 位姿增量
    dz = (1/C) * (w - Eᵀ @ dx)       // 回代求深度增量

    // 5. Retraction：位姿增量 AExp(dx) ⊕ poses，深度加法
    poses = pose_retr(poses, dx)
    disps = disp_retr(disps, dz)     // clamp 到 [0, 10]
```

### 关键源码片段

`raw/codes/DROID-SLAM/droid_slam/geom/ba.py:L38-L101`

```python
# 1: compute jacobians and residuals
coords, valid, (Ji, Jj, Jz) = pops.projective_transform(
    poses, disps, intrinsics, ii, jj, jacobian=True)
r = (target - coords).view(B, N, -1, 1)
w = .001 * (valid * weight).view(B, N, -1, 1)

# 2: construct linear system
Ji = Ji.reshape(B, N, -1, D); Jj = Jj.reshape(B, N, -1, D)
wJiT = (w * Ji).transpose(2,3); wJjT = (w * Jj).transpose(2,3)
Hii = torch.matmul(wJiT, Ji); Hij = torch.matmul(wJiT, Jj)
Hji = torch.matmul(wJjT, Ji); Hjj = torch.matmul(wJjT, Jj)
Ei = (wJiT.view(B,N,D,ht*wd,-1) * Jz[:,:,None]).sum(dim=-1)
Ej = (wJjT.view(B,N,D,ht*wd,-1) * Jz[:,:,None]).sum(dim=-1)
Ck = torch.sum(w*Jz*Jz, dim=-1)  # depth Hessian (diag per pixel)

# 3: solve the system (Schur complement)
dx, dz = schur_solve(H, E, C, v, w)
```

`raw/codes/DROID-SLAM/droid_slam/geom/chol.py:L46-L73`

```python
def schur_solve(H, E, C, v, w, ep=0.1, lm=0.0001):
    Q = (1.0 / C).view(B, M*HW, 1)
    # Schur complement: S = H - E * diag(1/C) * E^T
    Et = E.transpose(1,2)
    S = H - torch.matmul(E, Q*Et)
    v = v - torch.matmul(E, Q*w)
    dx = CholeskySolver.apply(S, v)       # 位姿增量
    dz = Q * (w - Et @ dx)                # 深度增量（回代）
    return dx, dz
```

### 实现注意事项

- 深度 Hessian `C` 近似为逐像素标量对角块，这要求 Jacobian `Jz` 的逐像素独立性成立（DROID-SLAM 中由 1/8 分辨率特征图和逐像素重投影结构保证）。
- `weight` 是 ConvGRU 输出的逐像素置信度，乘以 `0.001` 缩放防止 Hessian 数值过大；实际噪声模型隐含在网络权重中。
- Schur 补 `S = H - E * diag(1/C) * Eᵀ` 依赖深度 Hessian 可逆；`eta` 作为阻尼项（初始来自 CovGRU hidden state）防止深度 Hessian 奇异。
- 位姿 retraction 使用 `lietorch` 的 `poses.retr()` 在 SE3 流形上指数映射；深度 retraction 直接用加法 `disps + dz` 并 clamp。
- `fixedp` 通常 = 1，表示第一帧位姿固定为参考系（消去 gauge 自由度）。
- 该实现在 GPU 上以 batch 形式完成全部计算，不适合直接移植到单线程 C++ 实现。

### 源码检索锚点

- `BA` / `MoBA`
- `schur_solve`
- `projective_transform`
- `pose_retr` / `disp_retr`
- `safe_scatter_add_mat`

## 相关页面

- [[算法-DROID-SLAM]]
- [[概念-Schur补与边缘化]]
- [[架构-滑动窗口优化]]
- [[概念-深度学习SLAM]]
- [[方法-RAFT光流]]
