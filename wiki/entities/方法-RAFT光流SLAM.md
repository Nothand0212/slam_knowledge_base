---
tags: [方法, 深度学习, SLAM, 光流, DenseBA, 循环网络]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
---

# RAFT 光流 SLAM

> DROID-SLAM 的 RAFT 风格光流前端：用全对相关体和 ConvGRU 迭代预测稠密光流修正量，将光流残差接入 Dense Bundle Adjustment 层进行可微几何优化。

## 核心思路

DROID-SLAM 不把光流当作最终输出，而是作为可微几何优化的中间表示。核心循环是：

1. **特征提取**：双编码器网络为每帧图像提取匹配特征（128 维）和上下文特征（256 维）。
2. **相关体构造**：在关键帧之间计算全对特征内积，形成 4D 相关体并构建多尺度金字塔。
3. **迭代光流更新**：ConvGRU 在每轮迭代中查询相关体、拼接当前光流和残差，预测光流修正量 `delta` 和置信度权重 `weight`。
4. **Dense Bundle Adjustment**：将网络预测的稠密光流目标作为帧间约束，用 Schur 补求解位姿和逆深度。
5. **重复**：更新后的位姿和深度用于下一轮投影，新一轮相关查找 → 光流修正 → BA，共执行 12 轮。

## RAFT 风格迭代光流更新

### 特征提取

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L154-L169` 实现双编码器：

```python
def extract_features(self, images):
    # 图像归一化: [2,1,0] 通道重排 → ImageNet 均值/方差归一化
    images = images[:, :, [2,1,0]] / 255.0
    images = images.sub_(mean).div_(std)
    fmaps = self.fnet(images)              # 匹配特征: B×N×128×H/8×W/8
    net = self.cnet(images)               # 上下文特征: B×N×256×H/8×W/8
    net, inp = net.split([128,128], dim=2) # net: 隐藏态初始值, inp: 上下文输入
    net = torch.tanh(net); inp = torch.relu(inp)
    return fmaps, net, inp
```

两个编码器共享类似架构（BasicEncoder with 6 残差块），但输出维度不同。`fnet` 输出用于相关体计算（128 维），`cnet` 输出用于 ConvGRU 的隐藏态 `net` 和上下文 `inp`。

### 全对相关体

`raw/codes/DROID-SLAM/droid_slam/modules/corr.py:L23-L71` 实现相关体构造和多尺度金字塔：

```python
class CorrBlock:
    def __init__(self, fmap1, fmap2, num_levels=4, radius=3):
        self.corr_pyramid = []
        # 全对内积: B×N×H×W×H×W
        corr = CorrBlock.corr(fmap1, fmap2) / 4.0  # 缩放防溢出
        corr = corr.reshape(batch*num*h1*w1, 1, h2, w2)
        for i in range(self.num_levels):
            self.corr_pyramid.append(corr.view(batch*num, h1, w1, h2//2**i, w2//2**i))
            corr = F.avg_pool2d(corr, 2, stride=2)  # 逐层 2×2 池化

    @staticmethod
    def corr(fmap1, fmap2):
        batch, num, dim, ht, wd = fmap1.shape
        fmap1 = fmap1.reshape(batch*num, dim, ht*wd) / 4.0
        fmap2 = fmap2.reshape(batch*num, dim, ht*wd) / 4.0
        corr = torch.matmul(fmap1.transpose(1,2), fmap2)
        return corr.view(batch, num, ht, wd, ht, wd)
```

相关体是每对像素特征向量的内积 \(C(i,j,k,l) = \langle f_1(i,j), f_2(k,l) \rangle\)（缩放到 1/4）。构建 4 层金字塔，每层在第二帧上 2 倍下采样。

### 相关查找

给定当前估计的对应点坐标 `coords`，在每层金字塔中以 `coords/2^i` 为中心的 `(2r+1) × (2r+1)` 半径（r=3）局域查找 49 个相关值。4 层 × 49 = 196 维相关特征：

\[
\text{corr\_feature} = \text{concat}_{l=0..3} \left[ C_{\text{level}=l}(u, v, \frac{u'}{2^l} + dx, \frac{v'}{2^l} + dy) \right]_{dx,dy \in [-3,+3]}
\]

使用 CUDA 实现的 `CorrSampler`（`raw/codes/DROID-SLAM/droid_slam/modules/corr.py:L6-L20`）加速该操作。

### 运动特征

每轮迭代为 ConvGRU 构造运动特征向量（4 维）：

```python
flow = coords1 - coords0          # 当前估计的总光流 (2 维)
resd = target - coords1           # 光流残差 (2 维)
motion = cat([flow, resd]).clamp(-64, 64)  # → 4 维运动特征
```

### ConvGRU 更新算子

`raw/codes/DROID-SLAM/droid_slam/modules/gru.py:L5-L32` 实现带全局上下文的 ConvGRU：

```python
class ConvGRU(nn.Module):
    def __init__(self, h_planes=128, i_planes=128):
        self.convz = nn.Conv2d(h+i, h, 3)  # 更新门
        self.convr = nn.Conv2d(h+i, h, 3)  # 重置门
        self.convq = nn.Conv2d(h+i, h, 3)  # 候选隐藏态
        self.w     = nn.Conv2d(h, h, 1)    # 全局上下文权重

    def forward(self, net, *inputs):
        inp = torch.cat(inputs, dim=1)
        net_inp = torch.cat([net, inp], dim=1)

        # 全局上下文特征
        glo = sigmoid(self.w(net)) * net
        glo = glo.mean(dim=[2,3]).view(B, C, 1, 1)

        # GRU 门控
        z = sigmoid(convz(net_inp) + convz_glo(glo))  # 更新门
        r = sigmoid(convr(net_inp) + convr_glo(glo))  # 重置门
        q = tanh(convq(cat(r*net, inp)) + convq_glo(glo))  # 候选

        net = (1-z) * net + z * q   # 隐藏态更新
        return net
```

ConvGRU 输入为 `[corr_encoded(128), flow_encoded(64), inp(128)]` 共 320 通道。全局上下文 `glo` 通过空间平均池化聚合全图信息后重新注入每个空间位置。这使 ConvGRU 能感知全局运动一致性。

### 更新模块完整流程

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L78-L143` 实现 `UpdateModule`：

```python
class UpdateModule(nn.Module):
    def forward(self, net, inp, corr, flow, ii, jj):
        # 编码相关特征和光流特征
        corr = self.corr_encoder(corr)   # 196 → 128
        flow = self.flow_encoder(flow)   #   4 →  64
        # ConvGRU 更新隐藏态
        net = self.gru(net, inp, corr, flow)
        # 预测输出
        delta = self.delta(net)          # 光流修正量 (2 维)
        weight = self.weight(net)        # 置信度权重 (2 维)
        # 图聚合 (帧间平均)
        eta, upmask = self.agg(net, ii)
        return net, delta, weight, eta, upmask
```

输出含义：
- `delta`: 逐像素光流修正量，加到当前 `target` 上更新匹配目标位置。
- `weight`: 在 [0,1] 之间的逐像素置信度，用于 BA 中加权残差。
- `eta`: 从 ConvGRU 隐藏态导出的阻尼因子，用于 BA 深度 Hessian 的对角阻尼。
- `upmask`: 用于 8× 凸组合上采样（convex upsampling）的掩码。

## Dense Bundle Adjustment 层

### 投影 Jacobian

`raw/codes/DROID-SLAM/droid_slam/geom/projective_ops.py` 实现了带 Jacobian 的投影链：

**逆投影**（像素 → 相机坐标系三维点，带齐次分量 d = 逆深度）：
\[
\mathbf{X}_c = [\frac{x-c_x}{f_x},\; \frac{y-c_y}{f_y},\; 1,\; d]^T
\]
\[
\mathbf{J}_{\text{iproj}} = [0,0,0,1] \quad \text{(只有逆深度分量有 Jacobian)}
\]

**投影**（相机坐标系三维点 → 像素）：
\[
\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} f_x \cdot X/Z + c_x \\ f_y \cdot Y/Z + c_y \end{bmatrix}
\]
\[
\mathbf{J}_{\text{proj}} = \begin{bmatrix} f_x/Z & 0 & -f_x X/Z^2 & 0 \\ 0 & f_y/Z & -f_y Y/Z^2 & 0 \end{bmatrix}_{2 \times 4}
\]

**SE(3) 作用**：\( \mathbf{X}_j = \mathbf{G}_{ij} \cdot \mathbf{X}_i \)（用 lietorch 的 SE3 作用在四维齐次点云上）。

`projective_transform` 将这些步骤组合，用 autograd 链式法则计算端到端投影 Jacobian `Ji`（对帧 i 位姿）、`Jj`（对帧 j 位姿）、`Jz`（对逆深度）。

### BA 线性化与 Schur 补

`raw/codes/DROID-SLAM/droid_slam/geom/ba.py:L31-L107` 实现完整的 Dense BA：

```python
def BA(target, weight, eta, poses, disps, intrinsics, ii, jj, fixedp=1):
    # 1. 计算 Jacobian 和残差
    coords, valid, (Ji, Jj, Jz) = pops.projective_transform(poses, disps, intrinsics, ii, jj, jacobian=True)
    r = target - coords                          # 光流残差
    w = .001 * valid * weight                     # 置信度加权 + 缩放

    # 2. 构架 Hessian 矩阵
    Hii = Ji^T * w * Ji; Hij = Ji^T * w * Jj    # 位姿-位姿 Hessian 块
    Hji = Jj^T * w * Ji; Hjj = Jj^T * w * Jj
    vi  = Ji^T * w * r;  vj  = Jj^T * w * r     # 位姿-残差向量
    Ei  = Ji^T * w * Jz; Ej  = Jj^T * w * Jz    # 位姿-深度交叉项
    C   = sum(w * Jz * Jz, dim=-1) + eta + 1e-7 # 深度 Hessian (对角)

    # 3. Scatter 到关键帧维度的全局系统
    H = scatter_sum(Hii+Hij+Hji+Hjj, ii, jj)
    E = scatter_sum(Ei+Ej, ii, kk)
    v = scatter_sum(vi+vj, ii)

    # 4. Schur 补求解: 消去深度
    S = H - E * (1/C) * E^T       # Schur complement (位姿)
    dx = cholesky_solve(S, v - E * (1/C) * w)
    dz = (1/C) * (w - E^T * dx)  # 回代深度

    # 5. Retraction: SE3 流形更新 + 深度加法
    poses = pose_retr(poses, dx)
    disps = disp_retr(disps, dz.clamp(min=0))
    return poses, disps
```

Schur 补消去深度变量的数学形式：

\[
\begin{bmatrix} \mathbf{H}_{TT} & \mathbf{E} \\ \mathbf{E}^T & \mathbf{C} \end{bmatrix} \begin{bmatrix} \Delta\mathbf{T} \\ \Delta\mathbf{d} \end{bmatrix} = \begin{bmatrix} \mathbf{v}_T \\ \mathbf{v}_d \end{bmatrix}
\]

消元后：
\[
(\mathbf{H}_{TT} - \mathbf{E}\mathbf{C}^{-1}\mathbf{E}^T) \Delta\mathbf{T} = \mathbf{v}_T - \mathbf{E}\mathbf{C}^{-1}\mathbf{v}_d
\]
\[
\Delta\mathbf{d} = \mathbf{C}^{-1}(\mathbf{v}_d - \mathbf{E}^T \Delta\mathbf{T})
\]

深度 Hessian `C` 被近似为逐像素独立的标量对角度块，使 `C^{-1}` 可以逐元素计算，大幅降低计算复杂度。

## 迭代优化流程

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L172-L222` 展示完整优化循环：

```python
def forward(self, Gs, images, disps, intrinsics, graph, num_steps=12, fixedp=2):
    fmaps, net, inp = self.extract_features(images)
    net, inp = net[:,ii], inp[:,ii]
    corr_fn = CorrBlock(fmaps[:,ii], fmaps[:,jj])
    coords0 = coords_grid(ht//8, wd//8)
    coords1, _ = projective_transform(Gs, disps, intrinsics, ii, jj)
    target = coords1.clone()

    for step in range(num_steps):
        Gs = Gs.detach(); disps = disps.detach()  # 断开强耦合
        coords1 = coords1.detach(); target = target.detach()

        # 提取相关特征和运动特征
        corr = corr_fn(coords1)
        resd = target - coords1; flow = coords1 - coords0
        motion = cat([flow, resd]).clamp(-64, 64)

        # ConvGRU 更新
        net, delta, weight, eta, upmask = self.update(net, inp, corr, motion, ii, jj)
        target = coords1 + delta             # 更新光流目标

        # Dense BA (每轮 2 步)
        for i in range(2):
            Gs, disps = BA(target, weight, eta, Gs, disps, intrinsics, ii, jj)

        # 重新投影
        coords1, valid = projective_transform(Gs, disps, intrinsics, ii, jj)

    return Gs_list, disp_list, residual_list
```

关键设计：
1. **`.detach()` 截断**：每轮迭代开始时断开位姿和深度的计算图连接，防止多轮迭代的梯度在反向传播时指数放大，同时保证每轮迭代的激活内存可控。
2. **每轮 2 步 BA**：BA 是计算密集的，但每轮做 2 步而非 5-10 步，使 ConvGRU 能在 BA 收敛前持续提供更好的光流目标。
3. **12 轮迭代**：固定 12 轮，网络在训练中学会在有限迭代次数内收敛。

---

## Agent 实现提示

### 适用场景

Agent 需要实现**可微的光流+几何联合优化前端**时适用。适合输入为图像序列的多视角位姿和深度估计任务；需要 GPU 支持卷积和 PyTorch autograd。不适合纯特征点匹配、极低算力嵌入式或需要实时 30fps 以上的场景。

### 输入输出契约

- **输入**：图像序列 `images`（B×N×3×H×W，值域归一化至 ImageNet 统计量）、初始位姿 `Gs`（SE3 对象）、初始逆深度 `disps`（B×P×H/8×W/8）、内参 `intrinsics`（[fx, fy, cx, cy]）、因子图 `graph`（边索引 ii, jj）、迭代步数 `num_steps`（默认 12）、固定关键帧数 `fixedp`（默认 2）。
- **输出**：每轮迭代后的 `Gs_list`、`disp_list`（全分辨率深度，经 8× 凸组合上采样）、`residual_list`（光流残差，用于监控收敛）。
- **坐标约定**：逆深度定义在帧 i 坐标系中；像素坐标定义在 1/8 分辨率特征图空间；位姿为世界到相机变换（SE3）。

### 实现骨架（伪代码）

```pseudo
function runDroidSlam(images, poses_init, disps_init, intrinsics, ii, jj, graph):
    # 1. 特征提取
    fmaps, hidden, context = encode_features(images)

    # 2. 构造相关体
    corr_fn = build_corr_pyramid(fmaps[ii], fmaps[jj], levels=4, radius=3)

    # 3. 初始化光流和坐标
    coords0 = meshgrid(0..W-1, 0..H-1)  // 像素格
    coords1, _ = project(poses_init ⊕ disps_init)  // 当前投影
    target = coords1.clone()

    # 4. 迭代更新
    poses, disps = poses_init, disps_init
    for step in 1..12:
        poses = detach(poses); disps = detach(disps)  // 截断梯度

        # 4a. 相关查找
        corr = corr_fn.lookup(coords1)           // 196 维

        # 4b. 运动特征
        flow = coords1 - coords0                   // 总光流
        resd = target - coords1                    // 残差流
        motion = clamp(cat(flow, resd), -64, 64)   // 4 维

        # 4c. ConvGRU 更新
        hidden, delta, weight, eta, upmask = conv_gru(
            hidden, context, corr_encoded, flow_encoded, ii, jj)
        target = coords1 + delta                   // 更新目标

        # 4d. Dense BA 求解 (2 步)
        for _ in 1..2:
            coords, valid, (Ji, Jj, Jz) = proj_transform(poses, disps, ii, jj)
            r = target - coords; w = 0.001 * valid * weight

            Hii = Ji^T * w * Ji; Hij = Ji^T * w * Jj
            Hji = Jj^T * w * Ji; Hjj = Jj^T * w * Jj
            vi  = Ji^T * w * r;  vj  = Jj^T * w * r
            Ei  = Ji^T * w * Jz; Ej  = Jj^T * w * Jz
            C   = sum(w * Jz^2, dim=-1) + eta + 1e-7

            H = scatter_sum(Hii+Hij+Hji+Hjj, ii, jj)
            E = scatter_sum(Ei+Ej, ii, kk)
            v = scatter_sum(vi+vj, ii)

            // Schur 补 + 回代
            S = H - E * (1/C) * E^T
            dx = cholesky_solve(S, v - E * (1/C) * sum(w*r*Jz))
            dz = (1/C) * (sum(w*r*Jz) - E^T * dx)

            poses = SE3_retraction(poses, dx)
            disps = disps + dz  // clamped to [0, 10]

        # 4e. 更新投影坐标
        coords1, _ = proj_transform(poses, disps, ii, jj)

    # 5. 上采样深度
    disps_full = convex_upsample(disps, upmask)  // 8× 上采样

    return poses, disps_full, target - coords1
```

### 关键源码片段

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L172-L222` — 完整迭代循环：

```python
for step in range(num_steps):
    Gs = Gs.detach(); disps = disps.detach()
    coords1 = coords1.detach(); target = target.detach()

    corr = corr_fn(coords1)
    resd = target - coords1; flow = coords1 - coords0
    motion = torch.cat([flow, resd], dim=-1).permute(0,1,4,2,3).clamp(-64.0, 64.0)

    net, delta, weight, eta, upmask = self.update(net, inp, corr, motion, ii, jj)
    target = coords1 + delta

    for i in range(2):
        Gs, disps = BA(target, weight, eta, Gs, disps, intrinsics, ii, jj, fixedp=2)

    coords1, valid_mask = pops.projective_transform(Gs, disps, intrinsics, ii, jj)
    residual = target - coords1
```

`raw/codes/DROID-SLAM/droid_slam/geom/ba.py:L38-L106` — BA 线性化与 Schur 补：

```python
# compute jacobians and residuals
coords, valid, (Ji, Jj, Jz) = pops.projective_transform(poses, disps, intrinsics, ii, jj, jacobian=True)
r = (target - coords).view(B, N, -1, 1)
w = .001 * (valid * weight).view(B, N, -1, 1)
Hii = torch.matmul(wJiT, Ji); Hij = torch.matmul(wJiT, Jj)
Ei = (wJiT.view(B,N,D,ht*wd,-1) * Jz[:,:,None]).sum(dim=-1)
Ck = torch.sum(w*Jz*Jz, dim=-1)
# Schur complement solve
dx, dz = schur_solve(H, E, C, v, w)
poses = pose_retr(poses, dx, torch.arange(P) + fixedp)
disps = disp_retr(disps, dz.view(B,-1,ht,wd), kx).clamp(min=0.0)
```

`raw/codes/DROID-SLAM/droid_slam/modules/gru.py:L19-L32` — ConvGRU 更新门运算：

```python
glo = torch.sigmoid(self.w(net)) * net
glo = glo.view(b, c, h*w).mean(-1).view(b, c, 1, 1)  # global context
z = torch.sigmoid(self.convz(net_inp) + self.convz_glo(glo))
r = torch.sigmoid(self.convr(net_inp) + self.convr_glo(glo))
q = torch.tanh(self.convq(torch.cat([r*net, inp], dim=1)) + self.convq_glo(glo))
net = (1-z) * net + z * q
```

### 实现注意事项

- **梯度截断 `.detach()`**：每轮迭代开始必须对位姿、逆深度、坐标和目标做 `.detach()` 以断开计算图。否则 12 轮迭代的梯度会指数放大，导致训练不稳定或 NaN。
- **权重缩放 0.001**：`w = .001 * valid * weight` 中的 0.001 将网络输出的置信度（0~1 范围）映射到合理 Hessian 量级。如果更改特征图分辨率或网络输出范围，需重新校准。
- **深度 Hessian 对角近似**：Schur 补依赖于 `Jz` 逐像素独立的假设。在 1/8 分辨率下此假设成立（每个像素的重投影 Jacobian 主要来自该像素的逆深度），但在全分辨率下可能需要块对角近似。
- **ConvGRU 隐藏态初始值**：`net = tanh(cnet_output)` 初始化隐藏态，`inp = relu(cnet_output)` 作为固定上下文注入。`tanh` 将值限制在 [-1,1]，`relu` 作为非负特征输入。两者不宜互换。
- **`fixedp` 参数**：通常 `fixedp=1` 或 2，表示前几帧位姿固定不动（消去 gauge 自由度）。如果 `fixedp < 1`，系统可能在零空间上犯简。
- **多相机 rig**：通过 `rig` 参数支持多相机系统，BA 中 `P // rig` 计算有效相机数。单目系统中 `rig=1`。
- **Cholesky 失败回退**：`CholeskySolver` 在分解失败时返回全零增量而不终止程序。这在训练初期 Hessian 病态时有保护作用，但可能掩盖真正的数值问题。

### 源码检索锚点

- `DroidNet.forward` — `raw/codes/DROID-SLAM/droid_slam/droid_net.py:L172-L222`
- `UpdateModule` — `raw/codes/DROID-SLAM/droid_slam/droid_net.py:L78-L143`
- `ConvGRU` — `raw/codes/DROID-SLAM/droid_slam/modules/gru.py:L5-L32`
- `CorrBlock` — `raw/codes/DROID-SLAM/droid_slam/modules/corr.py:L23-L71`
- `BA` / `MoBA` — `raw/codes/DROID-SLAM/droid_slam/geom/ba.py:L31-L107`
- `schur_solve` — `raw/codes/DROID-SLAM/droid_slam/geom/chol.py:L46-L73`
- `projective_transform` — `raw/codes/DROID-SLAM/droid_slam/geom/projective_ops.py`
- `GraphAgg` — `raw/codes/DROID-SLAM/droid_slam/droid_net.py:L44-L75`
- `cvx_upsample` — `raw/codes/DROID-SLAM/droid_slam/droid_net.py:L21-L35`

## 相关页面

- [[算法-DROID-SLAM]]
- [[方法-RAFT光流]]
- [[方法-Dense BA]]
- [[方法-ConvGRU]]
- [[概念-Schur补与边缘化]]
- [[概念-深度学习SLAM]]
- [[架构-滑动窗口优化]]
