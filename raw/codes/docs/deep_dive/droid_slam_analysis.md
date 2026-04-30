# DROID-SLAM 深度源码分析

> 基于源码版本：Zachary Teed & Jia Deng, NeurIPS 2021  
> 分析文件：`droid_slam/` 目录全部 Python 源码 + C++/CUDA 扩展接口

---

## 1. 深度学习SLAM范式概述

### 1.1 传统SLAM vs. 深度学习SLAM的根本差异

传统SLAM（如 ORB-SLAM3、VINS-Mono）的核心流程是：

```
图像 → 手工特征提取(ORB/Harris) → 描述子匹配 → PnP/ICP位姿估计 → 局部BA优化 → 回环检测 → 全局BA
```

DROID-SLAM 的范式完全不同：

```
图像 → CNN提取稠密特征 → 全对(all-pairs)相关体 → 迭代GRU更新 → 预测光流修正 → Dense BA（GPU Schur补）→ 联合优化位姿+深度
```

**核心差异总结：**

| 维度 | 传统SLAM (ORB-SLAM3) | DROID-SLAM |
|------|---------------------|------------|
| 特征提取 | 手工ORB角点（稀疏，~1000个） | CNN稠密特征图（H/8 × W/8 分辨率） |
| 匹配方式 | 描述子Hamming距离 + RANSAC | 全对相关体(all-pairs correlation) |
| BA形式 | 稀疏BA（只优化特征点） | **Dense BA**（逐像素残差，~数万因子） |
| 残差定义 | 重投影误差（3D点→2D） | 光流残差（预测光流 ↔ 诱导光流） |
| 求解方式 | CPU上Ceres/g2o (LM法) | GPU上Schur补 + Cholesky分解 |
| 外点剔除 | RANSAC | 网络学习的置信度权重 |

### 1.2 DROID-SLAM的系统架构

`droid.py:16-85` 定义了顶层 `Droid` 类，系统由以下组件组成：

```
Droid
├── DroidNet (droid_net.py)          # 神经网络：特征提取 + 迭代更新
├── DepthVideo (depth_video.py)      # 共享状态存储：图像/位姿/深度/特征
├── MotionFilter (motion_filter.py)  # 运动滤波：丢弃静止帧
├── DroidFrontend (droid_frontend.py) # 前端：局部窗口优化
├── DroidBackend (droid_backend.py)  # 后端：全局BA优化
└── PoseTrajectoryFiller (trajectory_filler.py) # 后处理：填充非关键帧位姿
```

运行流程（`demo.py:121-134`）：
```python
for (t, image, intrinsics) in image_stream(...):
    droid.track(t, image, intrinsics=intrinsics)  # 逐帧处理
traj_est = droid.terminate(image_stream(...))     # 终止时全局BA + 位姿填充
```

---

## 2. 网络架构

### 2.1 特征提取网络

DROID-SLAM 使用两个编码器提取不同用途的特征（`droid_net.py:149-150`）：

```python
# droid_net.py:149-150
class DroidNet(nn.Module):
    def __init__(self):
        self.fnet = BasicEncoder(output_dim=128, norm_fn='instance')  # 匹配特征
        self.cnet = BasicEncoder(output_dim=256, norm_fn='none')      # 上下文特征
```

**fnet（feature network）= 用于构建相关体（correlation volume）的匹配特征**  
**cnet（context network）= 用于GRU隐状态初始化 + 输入投影的上下文特征**

`BasicEncoder`（`modules/extractor.py:118-198`）是一个 ResNet-style 编码器：

```python
# modules/extractor.py:136-145
self.conv1 = nn.Conv2d(3, 32, kernel_size=7, stride=2, padding=3)  # 1/2 分辨率
self.layer1 = self._make_layer(32,  stride=1)   # 1/2
self.layer2 = self._make_layer(64, stride=2)    # 1/4
self.layer3 = self._make_layer(128, stride=2)   # 1/8 ← 最终分辨率
self.conv2 = nn.Conv2d(128, output_dim, kernel_size=1)  # 输出投影
```

每个 layer 包含 2 个 `ResidualBlock`（`modules/extractor.py:6-55`），使用 3×3 卷积 + GroupNorm/InstanceNorm + ReLU。

图像预处理（`droid_net.py:154-169`）：

```python
# droid_net.py:154-169
def extract_features(self, images):
    images = images[:, :, [2,1,0]] / 255.0  # BGR→RGB, 归一化
    mean = torch.as_tensor([0.485, 0.456, 0.406], ...)
    std  = torch.as_tensor([0.229, 0.224, 0.225], ...)
    images = images.sub_(mean[:, None, None]).div_(std[:, None, None])

    fmaps = self.fnet(images)   # [B, N, 128, H/8, W/8]  匹配特征
    net   = self.cnet(images)   # [B, N, 256, H/8, W/8]  上下文特征

    net, inp = net.split([128,128], dim=2)
    net = torch.tanh(net)       # GRU 初始隐状态
    inp = torch.relu(inp)       # GRU 输入投影
    return fmaps, net, inp
```

**关键设计：** net 用 tanh 激活（作为 GRU 隐状态初值），inp 用 ReLU（作为每次迭代的恒定输入）。

### 2.2 相关体 (Correlation Volume)

这是 RAFT 光流的核心思想，也是 DROID-SLAM 继承的关键组件。

#### 全对相关 (All-Pairs Correlation)

```python
# modules/corr.py:64-71
@staticmethod
def corr(fmap1, fmap2):
    batch, num, dim, ht, wd = fmap1.shape
    fmap1 = fmap1.reshape(batch*num, dim, ht*wd) / 4.0  # L2 norm approx
    fmap2 = fmap2.reshape(batch*num, dim, ht*wd) / 4.0

    corr = torch.matmul(fmap1.transpose(1,2), fmap2)  # [BN, HW, HW]
    return corr.view(batch, num, ht, wd, ht, wd)  # [B, N, H, W, H, W]
```

计算的是**每个像素位置 i 的特征向量与所有位置 j 的特征向量的内积**：
```
C(i, j) = ⟨f_i/4, f_j/4⟩  ∈ ℝ
```
结果是一个 4D 张量 `[B, N, H, W, H, W]`。除以4是为了把内积控制在[-1,1]附近。

#### 多尺度查找 (Pyramid Lookup)

```python
# modules/corr.py:23-50
class CorrBlock:
    def __init__(self, fmap1, fmap2, num_levels=4, radius=3):
        corr = CorrBlock.corr(fmap1, fmap2)
        for i in range(self.num_levels):
            self.corr_pyramid.append(
                corr.view(batch*num, h1, w1, h2//2**i, w2//2**i))
            corr = F.avg_pool2d(corr, 2, stride=2)  # 逐级下采样

    def __call__(self, coords):
        # coords: [B, N, H, W, 2]  当前估计的匹配坐标
        for i in range(self.num_levels):
            # 在坐标 coords/2^i 周围 radius=3 范围内采样相关值
            corr = CorrSampler.apply(self.corr_pyramid[i], coords/2**i, self.radius)
            # corr: [B, N, (2r+1)^2, H, W] = [B, N, 49, H, W]
            out_pyramid.append(corr)
        return torch.cat(out_pyramid, dim=2)  # [B, N, 196, H, W]
```

`CorrSampler`（`modules/corr.py:6-20`）是一个自定义 autograd Function，实际采样由 CUDA kernel `droid_backends.corr_index_forward` 完成。

**总相关特征维度：** `4 levels × (2×3+1)^2 = 4 × 49 = 196 channels`

### 2.3 GRU 迭代更新模块 (UpdateModule)

这是 DROID-SLAM 最核心的网络组件（`droid_net.py:78-143`）：

```python
# droid_net.py:78-110
class UpdateModule(nn.Module):
    def __init__(self):
        cor_planes = 4 * (2*3 + 1)**2  # 196

        self.corr_encoder = nn.Sequential(        # 196→128 相关特征编码
            nn.Conv2d(cor_planes, 128, 1), nn.ReLU(),
            nn.Conv2d(128, 128, 3, padding=1), nn.ReLU())

        self.flow_encoder = nn.Sequential(        # 4→128→64  运动特征编码
            nn.Conv2d(4, 128, 7, padding=3), nn.ReLU(),
            nn.Conv2d(128, 64, 3, padding=1), nn.ReLU())

        self.gru = ConvGRU(128, 128+128+64)  # 隐维度128, 输入128+128+64=320

        self.weight = nn.Sequential(...)  # 输出: 逐像素置信度 [B,N,2,H,W]
        self.delta  = nn.Sequential(...)  # 输出: 光流修正量 [B,N,2,H,W]
```

#### ConvGRU（`modules/gru.py:5-32`）

```python
# modules/gru.py:19-31
class ConvGRU(nn.Module):
    def forward(self, net, *inputs):
        inp = torch.cat(inputs, dim=1)           # [corr_enc | ctx_inp | flow_enc]
        net_inp = torch.cat([net, inp], dim=1)

        # 全局上下文调制
        glo = torch.sigmoid(self.w(net)) * net   # 全局平均池化后的调制信号
        glo = glo.view(b, c, h*w).mean(-1).view(b, c, 1, 1)

        z = torch.sigmoid(self.convz(net_inp) + self.convz_glo(glo))  # 更新门
        r = torch.sigmoid(self.convr(net_inp) + self.convr_glo(glo))  # 重置门
        q = torch.tanh(self.convq(torch.cat([r*net, inp], dim=1))
                       + self.convq_glo(glo))     # 候选隐状态

        net = (1-z) * net + z * q   # GRU 标准更新
        return net
```

这里的创新是**全局上下文调制（global context modulation）**：将隐状态乘以 sigmoid 门控后做**空间平均**，得到一个全局描述子 `glo`，再用 1×1 卷积投影到门控的偏置项中。这让 GRU 能在**局部迭代中感知全局信息**。

### 2.4 GraphAgg 模块

```python
# droid_net.py:44-75
class GraphAgg(nn.Module):
    def forward(self, net, ii):
        # 对属于同一关键帧的所有边进行 scatter_mean 聚合
        net = scatter_mean(net, ix, dim=1)   # [B, num_kfs, 128, H, W]

        # 输出: damping factor η 和 上采样mask
        eta = self.eta(net)      # [B, num_kfs, H, W]  (经过Softplus保证>0)
        upmask = self.upmask(net)  # [B, num_kfs, 512*9, H, W]
        return .01 * eta, upmask
```

`eta` 是**每个关键帧的逐像素 damping factor**，作为 BA 中深度 Hessian 的对角增量（`H_d = H_d + eta + 1e-7`，见 `geom/ba.py:91`），防止深度变量发散。

`upmask` 是用于将 1/8 分辨率逆深度上采样到全分辨率的凸组合权重。

---

## 3. Dense Bundle Adjustment（DBA）

### 3.1 什么是 Dense BA？

传统 BA 只优化**稀疏的 3D 路标点**（ORB-SLAM3 约几千个），每个点贡献 2 个残差（x, y 重投影误差）。

DROID-SLAM 的 Dense BA 对**每个像素**（在 1/8 分辨率）都定义一个残差，即每个关键帧有 (H/8)×(W/8) 个因子。对 240×320 的图像，就是 30×40 = 1200 个因子/帧/边，比传统 BA 稀疏得多、但比纯光流多了几何一致性约束。

### 3.2 残差的定义

DROID-SLAM 的 BA 残差**不是重投影误差**，而是：

```
r_ij = Π(G_j ∘ G_i^{-1} ∘ Π^{-1}(p_i, d_i)) - p_ij*
```

其中：
- `p_ij*` = 网络预测的目标坐标 (target)
- `Π(G_j ∘ G_i^{-1} ∘ Π^{-1}(p_i, d_i))` = 从 i 帧逆投影再投影到 j 帧得到的坐标

**这本质上是"光流残差"**：网络预测了一个光流（target - coords0），BA 优化位姿 G 和深度 d，使得**相机运动诱导的光流**与**网络预测的光流**一致。

### 3.3 BA 的完整计算过程

`geom/ba.py:31-106` 完整实现了 Dense BA：

#### Step 1: 计算雅可比与残差

```python
# geom/ba.py:39-43
coords, valid, (Ji, Jj, Jz) = pops.projective_transform(
    poses, disps, intrinsics, ii, jj, jacobian=True)

r = (target - coords).view(B, N, -1, 1)              # 残差 [B, N, HW, 1]
w = .001 * (valid * weight).view(B, N, -1, 1)        # 权重 [B, N, HW, 1]
```

投影变换的链式法则（`geom/projective_ops.py:165-198`）：

```
projective_transform:  X_cam = Π⁻¹(p, d)  →  X' = GⱼGᵢ⁻¹ X  →  p' = Π(X')

jacobians:
  Jp = ∂p'/∂X'    (投影函数对3D点的导数)  [projective_ops.py:80]
  Ja = ∂X'/∂Gᵢⱼ   (SE3作用于点的导数)    [projective_ops.py:94-123]
  Jj = Jp · Ja    (链式法则, 对 j 帧位姿)  [projective_ops.py:190]
  Ji = -Gᵢⱼ.adjT(Jj)  (通过伴随变换得到对 i 帧的导数)  [projective_ops.py:191]
  Jz = Jp · (Gᵢⱼ作用于 ∂X_cam/∂d)  (对逆深度的导数)  [projective_ops.py:193-194]
```

**投影雅可比的具体形式** (对 `(x, y)` 关于 `(X, Y, Z)`):

```python
# geom/projective_ops.py:62-81
# x = fx * X/Z + cx,  y = fy * Y/Z + cy
proj_jac = [
    fx/Z,  0,    -fx*X/Z²,  0,     # ∂x/∂(X,Y,Z)
    o,     fy/Z, -fy*Y/Z²,  0,     # ∂y/∂(X,Y,Z)
]
```

**SE3 作用于点的雅可比** (Lie 代数求导):

```python
# geom/projective_ops.py:95-114
# ∂(RX+t)/∂δξ  in homogeneous coordinates [X,Y,Z,1]
Ja = [
    d,  0,  0,   0,  Z, -Y,    # ∂X'/∂(v₁,v₂,v₃,ω₁,ω₂,ω₃)
    0,  d,  0,  -Z,  0,  X,
    0,  0,  d,   Y, -X,  0,
    ...
]
```

#### Step 2: 构造正规方程

```python
# geom/ba.py:51-67
# Hesse矩阵的块结构
Hii = (w * Ji.T) @ Ji    # [B, N, D, D]   i帧自耦合
Hij = (w * Ji.T) @ Jj    # [B, N, D, D]   i-j交叉耦合
Hji = (w * Jj.T) @ Ji    # [B, N, D, D]   j-i交叉耦合
Hjj = (w * Jj.T) @ Jj    # [B, N, D, D]   j帧自耦合

vi = (w * Ji.T) @ r      # [B, N, D]      i帧梯度
vj = (w * Jj.T) @ r      # [B, N, D]      j帧梯度

# 姿态-深度交叉项
Ei = (w * Ji.T * Jz).sum(dim=-1)   # [B, N, D, HW]
Ej = (w * Jj.T * Jz).sum(dim=-1)   # [B, N, D, HW]

# 深度Hessian的对角块
Ck = sum(w * Jz * Jz)              # [B, M, HW]  每关键帧的逐像素深度Hessian
wk = sum(w * r  * Jz)              # [B, M, HW]  深度梯度
```

#### Step 3: Scatter 到全局矩阵

```python
# geom/ba.py:77-89
# 使用 scatter_sum 将 (i,j) 边贡献累加到对应的关键帧索引上
H = scatter(Hii, ii, ii) + scatter(Hij, ii, jj)
  + scatter(Hji, jj, ii) + scatter(Hjj, jj, jj)  # [B, P, P, D, D]

E = scatter(Ei, ii, kk) + scatter(Ej, jj, kk)   # [B, P, M, D, HW]

v = scatter(vi, ii) + scatter(vj, jj)            # [B, P, D]

C = scatter(Ck, kk) + eta + 1e-7                 # [B, M, HW]  (加damping)
w = scatter(wk, kk)                              # [B, M, HW]
```

这里 `scatter_sum`（`geom/ba.py:12-18`）是 `torch_scatter` 库的操作，将所有指向同一关键帧的边贡献累加。

#### Step 4: Schur 补求解

```python
# geom/ba.py:97:97
dx, dz = schur_solve(H, E, C, v, w)
```

Schur 消元过程（`geom/chol.py:46-73`）：

```python
# geom/chol.py:46-73
def schur_solve(H, E, C, v, w, ep=0.1, lm=0.0001):
    # 正规方程:  [H  E] [dx]   [v]
    #            [Eᵀ C] [dz] = [w]

    Q  = (1.0 / C)                         # 深度块求逆(对角⇒逐元素倒数)
    S  = H - E @ (Q * Eᵀ)                 # Schur补: S = H - E C⁻¹ Eᵀ
    v' = v - E @ (Q * w)                   # 右端项消元

    dx = CholeskySolver.apply(S, v')      # 仅求解P×D维的姿态系统
    dz = Q * (w - Eᵀ @ dx)                # 回代求深度增量

    return dx, dz  # [B, P, D],   [B, M, HW]
```

**关键洞察：** 深度变量的 Hessian `C` 是**逐像素独立的**（因为每个像素的逆深度只影响该像素的重投影），所以 `C` 是对角矩阵，求逆只需逐元素取倒数。这使得 **Schur 补的代价仅与关键帧数量 P 有关**（与像素数 M 无关），可以在 GPU 上高效完成。

#### Step 5: Retraction 更新

```python
# geom/ba.py:21-28, 100-103
def pose_retr(poses, dx, ii):
    return poses.retr(scatter_sum(dx, ii, ...))  # SE3 exponential map

def disp_retr(disps, dz, ii):
    return disps + scatter_sum(dz, ii, ...)      # 普通加法 (逆深度)

poses = pose_retr(poses, dx, torch.arange(P) + fixedp)
disps = disp_retr(disps, dz, kx)
disps = disps.clamp(min=0.0)                     # 保证逆深度非负
```

### 3.4 与原始 LOAM 的区别

原始 LOAM 中 BA 的雅可比是通过**解析推导 + CPU 数值实现**的，而 DROID-SLAM 使用 `lietorch` 库（PyTorch 自动微分 + Lie 群），在 `projective_ops.py` 中显式构造雅可比矩阵，所有操作在 GPU 上批量计算。

---

## 4. 关键帧管理与边缘化

### 4.1 因子图的数据结构

`factor_graph.py:19-50` 中的 `FactorGraph` 维护了以下核心数据结构：

```python
# factor_graph.py:32-50
self.ii = ...      # 边列表: 源帧索引  [N_edges]
self.jj = ...      # 边列表: 目标帧索引 [N_edges]
self.age = ...     # 每条边的年龄
self.target = ...  # 网络预测的目标坐标 [1, N_edges, H, W, 2]
self.weight = ...  # 网络预测的置信度   [1, N_edges, H, W, 2]

# 非活跃因子 (已被边缘化但仍参与优化的因子)
self.ii_inac, self.jj_inac = ...
self.target_inac, self.weight_inac = ...
```

### 4.2 边缘连接策略

`add_proximity_factors`（`factor_graph.py:346-412`）实现了**基于运动距离的边选择**：

```python
# factor_graph.py:359-361
d = self.video.distance(ii, jj, beta=beta).cpu()  # 计算两帧间距离
d[ii - rad < jj] = np.inf       # 排除近邻帧(已有稠密连接)
d[d > 100] = np.inf             # 排除过远帧

# 按距离从小到大排序，依次添加边，同时做 NMS
for k in torch.argsort(d):
    if d[k] > thresh: break
    es.append((i, j))
    es.append((j, i))   # 双向边
    # NMS: 抑制相邻边
```

`distance` 函数（`depth_video.py:181-211`）调用 CUDA 实现：

```python
# depth_video.py:196-202
d1 = droid_backends.frame_distance(
    poses, self.disps, self.intrinsics[0], ii, jj, beta)
d2 = droid_backends.frame_distance(
    poses, self.disps, self.intrinsics[0], jj, ii, beta)
d = .5 * (d1 + d2)   # 双向距离取平均
```

距离度量的 beta 参数（`demo.py:91, default=0.3`）权衡了平移和旋转分量对光流的影响。

### 4.3 关键帧删除（边缘化近似）

```python
# factor_graph.py:181-212
def rm_keyframe(self, ix):
    # 从 video 状态数组中移除索引 ix
    self.video.images[ix : t-1]  = self.video.images[ix+1 : t].clone()
    self.video.poses[ix : t-1]   = self.video.poses[ix+1 : t].clone()
    ...
    # 更新因子图中所有索引(>ix 的 -1)
    self.ii[self.ii >= ix] -= 1
    self.jj[self.jj >= ix] -= 1
```

这里**没有传统意义上的边缘化先验（marginalization prior）**。DROID-SLAM 采用更简单的策略：直接丢弃关键帧，后续帧的状态通过**继续参与后续优化来约束**。这在实践中有效，因为 Dense BA 已经提供了很强的几何约束。

### 4.4 前端滑动窗口

前端维护一个局部窗口（`droid_frontend.py:21-23`）：

```python
# droid_frontend.py:39-41
self.frontend_window = args.frontend_window     # default=25
self.frontend_thresh = args.frontend_thresh     # default=16.0
self.frontend_radius = args.frontend_radius     # default=2
```

每来一帧的处理（`droid_frontend.py:65-117`）：
1. 移除过老因子（`graph.rm_factors(graph.age > max_age)`）→ `max_age=20`
2. 添加新帧的 proximity 因子
3. 融合深度传感器读数（如 RGB-D）
4. 执行 `iters1=3` 轮因子图更新
5. 判断是否需要新关键帧：若运动距离 < `2×keyframe_thresh`，则**删除前一关键帧**

### 4.5 后端全局优化

`droid_backend.py:24-42`：

```python
# droid_backend.py:24-42
def __call__(self, steps=12, normalize=True):
    # 尺度归一化: 将逆深度均值缩放到1, 位姿平移同步缩放
    self.video.normalize()

    # 重建 factor graph (使用 AltCorrBlock 降低内存)
    graph = FactorGraph(self.video, self.update_op,
                        corr_impl="alt", max_factors=16*t,
                        upsample=self.upsample)

    graph.add_proximity_factors(rad=..., nms=..., thresh=...)
    graph.update_lowmem(steps=steps)   # 多轮全局优化
    graph.clear_edges()
```

终止时执行两次后端（`droid.py:78-82`）：
```python
self.backend(7)   # 7 轮迭代
self.backend(12)  # 12 轮迭代，最终优化
```

---

## 5. 网络前向传播的完整流程

`droid_net.py:172-222` 定义了 `DroidNet.forward`，每一步做：

```python
# droid_net.py:172-222
def forward(self, Gs, images, disps, intrinsics, graph, num_steps=12):
    fmaps, net, inp = self.extract_features(images)         # 1.特征提取
    net, inp = net[:,ii], inp[:,ii]
    corr_fn = CorrBlock(fmaps[:,ii], fmaps[:,jj], ...)      # 2.相关体

    target = pops.projective_transform(Gs, disps, ...)      # 3.初始投影

    for step in range(num_steps):                           # 4.迭代
        Gs, disps = Gs.detach(), disps.detach()             #   截断梯度

        corr    = corr_fn(coords1)                          #   采样相关
        resd    = target - coords1                          #   残差
        flow    = coords1 - coords0                         #   光流
        motion  = cat([flow, resd], dim=-1)                 #   运动特征

        net, delta, weight, eta, upmask = \
            self.update(net, inp, corr, motion, ii, jj)     #   GRU更新
        target = coords1 + delta                            #   目标修正

        for i in range(2):                                  #   2轮DBA
            Gs, disps = BA(target, weight, eta, ...)

        # 每步记录结果
        Gs_list.append(Gs)
        disp_list.append(upsample_disp(disps, upmask))
        residual_list.append(valid_mask * (target - coords1))

    return Gs_list, disp_list, residual_list
```

**注意 `fixedp=2`**：前2帧位姿固定不变（作为尺度锚定），只优化后续帧。

---

## 6. 训练策略

### 6.1 训练数据

使用 TartanAir 仿真数据集（`train.py:61`）：
```python
# train.py:61
db = dataset_factory(['tartan'], datapath=args.datapath,
                     n_frames=args.n_frames, fmin=args.fmin, fmax=args.fmax)
```

数据增强（`data_readers/augmentation.py:56-58`）包括：
- 色彩抖动 (brightness/contrast/saturation/hue)
- 随机灰度化
- 空间尺度变换 + 中心裁剪

### 6.2 监督信号（三重损失）

`train.py:110-113` 定义了三个损失项：

```python
# train.py:110-112
geo_loss, geo_metrics = losses.geodesic_loss(Ps, poses_est, graph)
res_loss, res_metrics = losses.residual_loss(residuals)
flo_loss, flo_metrics = losses.flow_loss(Ps, disps, poses_est, disps_est, intrinsics, graph)

loss = 10.0 * geo_loss + 0.01 * res_loss + 0.05 * flo_loss
```

#### (1) 测地线损失 (Geodesic Loss, w=10.0)

`geom/losses.py:30-74`：

```python
# geom/losses.py:30-74
def geodesic_loss(Ps, Gs, graph, gamma=0.9):
    ii, jj, kk = graph_to_edge_list(graph)
    dP = Ps[:,jj] * Ps[:,ii].inv()       # GT 相对位姿

    for i in range(n):
        w = gamma ** (n - i - 1)         # 指数衰减: 越晚迭代权重越大
        dG = Gs[i][:,jj] * Gs[i][:,ii].inv()  # 估计相对位姿

        d = (dG * dP.inv()).log()        # SE3/Sim3 lie algebra error
        tau, phi = d.split([3,3], dim=-1)
        geodesic_loss += w * (tau.norm() + phi.norm()).mean()
```

**这是监督信号的核心**：要求每一轮迭代输出的相对位姿与真值一致，且后期迭代权重更大（gamma=0.9，越接近收敛权重越高）。

#### (2) 残差损失 (Residual Loss, w=0.01)

`geom/losses.py:77-86`：

```python
# geom/losses.py:77-86
def residual_loss(residuals, gamma=0.9):
    for i in range(n):
        w = gamma ** (n - i - 1)
        residual_loss += w * residuals[i].abs().mean()
```

鼓励网络预测的光流 target 与 BA 优化后的投影坐标一致（即优化收敛）。

#### (3) 光流损失 (Flow Loss, w=0.05)

`geom/losses.py:89-118`：

```python
# geom/losses.py:89-118
def flow_loss(Ps, disps, poses_est, disps_est, intrinsics, graph):
    # 相邻帧之间的 GT 光流 vs. 估计光流
    coords0, val0 = projective_transform(Ps, disps, intrinsics, ii, jj)
    coords1, val1 = projective_transform(poses_est[i], disps_est[i], ...)

    epe = v * (coords1 - coords0).norm(dim=-1)   # end-point-error
    flow_loss += w * epe.mean()
```

提供逐像素的稠密监督，确保估计的位姿+深度能够解释所有相邻帧之间的像素运动。

### 6.3 随机重启 (Random Restarts)

`train.py:101-118`：

```python
# train.py:102-118
r = 0
while r < args.restart_prob:   # 0.2
    r = rng.random()
    # 从当前优化结果重新初始化，继续训练
    poses_est, disps_est, residuals = model(Gs, images, disp0, ...)
    loss.backward()
    Gs = poses_est[-1].detach()
    disp0 = disps_est[-1].detach()
```

这是一种**课程学习（curriculum learning）**：以概率 0.2 重启优化，迫使网络学习从**不完美的初始状态恢复**，提高鲁棒性。

---

## 7. GPU端CUDA实现

### 7.1 CUDA扩展入口

`setup.py:7-23` 编译了4个 CUDA 源文件：

```python
# setup.py:12-18
CUDAExtension('droid_backends',
    sources=[
        'src/droid.cpp',           # Python binding
        'src/droid_kernels.cu',    # BA + frame_distance + upsampling
        'src/correlation_kernels.cu', # corr_index_forward/backward
        'src/altcorr_kernel.cu',   # altcorr (低内存相关采样)
    ])
```

### 7.2 Python端调用

深度视频的关键操作都通过 `droid_backends` 调用 CUDA kernel：

```python
# depth_video.py:196-200  frame_distance
d1 = droid_backends.frame_distance(poses, self.disps, ...)

# depth_video.py:222-223  dense BA
droid_backends.ba(self.poses, self.disps, self.intrinsics[0],
                  self.disps_sens, target, weight, eta, ii, jj,
                  t0, t1, itrs, lm, ep, motion_only)
```

### 7.3 AltCorr 低内存模式

后端全局优化需要处理大量边，所以使用 `AltCorrBlock`（`modules/corr.py:89-117`）：

```python
# factor_graph.py:274
corr_op = AltCorrBlock(self.video.fmaps.view(1, num*rig, ch, ht, wd))

# 不预计算全对相关体，而是在每次查询时实时计算小范围相关
corr1 = corr_op(coords1[:,v], rig * iis, rig * jjs + (iis == jjs).long())
```

这种方式避免了后端存储大相关体的内存开销，代价是每次查询需要实时计算。

---

## 8. 数据管线

### 9.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| 灰度/RGB 相机 (单目/双目/RGB-D) | 15-30 Hz | 3ch uint8 torch tensor | `image_stream()` → `Droid.track()` | `DroidNet.extract_features()` |
| 相机内参 | 每帧 | fx,fy,cx,cy (4 floats) | `Droid.track(..., intrinsics=K)` | `DepthVideo` |
| 深度传感器 (RGB-D, 可选) | 15-30 Hz | H×W float32 depth map | `Droid.track(..., depth=depth)` | `DepthVideo.disps_sens` |
| 预训练权重 | 一次性 | `droid.pth` (PyTorch checkpoint) | `torch.load()` → `nn.Module.load_state_dict()` | `DroidNet` |

### 9.2 图像管线

#### 原始数据
```
规格: 无固定分辨率 (自适应), 通常 370×480 (TUM) or 240×320 (TartanAir)
格式: numpy uint8 array → torch tensor [B, N, 3, H, W]
```

#### 预处理 — 特征提取网络
**Step 1 — 图像归一化** (`droid_net.py:154-169`):
```
1. 颜色空间转换: images[:, :, [2,1,0]]  (BGR → RGB)
2. 归一化到 [0,1]: / 255.0
3. ImageNet 标准化:
   mean = [0.485, 0.456, 0.406]
   std  = [0.229, 0.224, 0.225]
   images = (images - mean) / std
```

**Step 2 — 稠密特征编码** (`BasicEncoder`, `modules/extractor.py:118-198`):
```
网络结构 (ResNet-style):
  conv1: 7×7, stride=2, 3→32ch  → H/2 × W/2
  layer1: 2×ResBlock, 32ch    → H/2 × W/2
  layer2: 2×ResBlock, 64ch, stride=2 → H/4 × W/4
  layer3: 2×ResBlock, 128ch, stride=2 → H/8 × W/8
  conv2: 1×1, 128→output_dim

两个 encoder:
  fnet (output_dim=128, norm='instance'): 匹配特征 fmaps [B,N,128,H/8,W/8]
  cnet (output_dim=256, norm='none'):     上下文特征 [B,N,256,H/8,W/8]
    分组: net(128ch, tanh) + inp(128ch, relu)
```

#### 特征提取 (匹配特征)
```
无传统特征点。使用稠密逐像素 CNN 特征 fmaps:
  每个像素位置 p = (i,j) 对应 128D 特征向量 f_p
  用于构建全对相关体 (all-pairs correlation)
```

#### 匹配 — 全对相关体 (All-Pairs Correlation)
**相关体构建** (`modules/corr.py:64-71`):
```python
# 特征向量 L2-norm 近似: fmap / 4.0  (控制内积范围 ≈ [-1,1])
corr = torch.matmul(fmap1.transpose(1,2), fmap2)  # [B,N, HW, HW]
C(i, j) = ⟨f_i/4, f_j/4⟩  (dot product)
```
结果存储为 4D 张量 `[B, N, H, W, H, W]`

**多尺度查找** (`CorrBlock.__call__()`, `modules/corr.py:23-50`):
```
参数: num_levels=4, radius=3
逐层下采样: avg_pool2d(corr, 2, stride=2)
在每层: 在当前估计坐标 coords 周围 radius=3 范围内采样
每层输出: (2r+1)² = 49 channels
总: 4 × 49 = 196 维相关特征 → 送入 GRU
```

**CUDA 加速**:
```
采样操作: CorrSampler (自定义 autograd Function)
  前向: droid_backends.corr_index_forward  (CUDA kernel)
  反向: droid_backends.corr_index_backward
AltCorrBlock (后端低内存模式):
  不预存全对相关体, 每次查询实时计算小范围相关
```

#### 异常值剔除 — 学习式置信度
**网络预测权重** (`droid_net.py:170-180`):
```
UpdateModule 输出 per-pixel weight:
  weight = self.weight(net_hidden)  →  [B,N,2,H,W]
  (每个像素的 x,y 方向分别有权重)
无硬阈值, 由网络学习自动判断匹配可靠性
```

**GRU 迭代中的隐式外点剔除**:
```
每次迭代 GRU 输出 delta (光流修正) + weight (置信度)
若某像素匹配不可靠 → 网络自动降低 weight
→ 在 DBA 中该像素贡献被自然衰减
```

#### 算法消费 — Dense BA 因子
**残差定义** (`geom/ba.py:39-43`):
```
r = target - coords  (2D 光流残差)
其中:
  target: GRU 预测的目标坐标 (每步迭代更新)
  coords = π(G_j ∘ G_i⁻¹ ∘ π⁻¹(p_i, d_i))
    (相机运动诱导的投影坐标)

雅可比 (projective_ops.py:165-198):
  Ji = ∂r/∂G_i  (2×6, via adjoint of G_j ∘ G_i⁻¹)
  Jj = ∂r/∂G_j  (2×6, standard rigid transform Jacobian)
  Jz = ∂r/∂d_i  (2×1, per-pixel inverse depth Jacobian)

信息矩阵:
  w_i = 0.001 × valid_mask_i × weight_i  (pixel-wise weight)
  Σ⁻¹ = w_i² · I  (diagonal, per-pixel independent)

正规方程 (Hessian block structure):
  H = [[Hii, Hij],   b = [vi]
       [Hji, Hjj]]       [vj]
  C = w·Jz²  (per-pixel diagonal depth Hessian)
  E = w·Ji·Jz  (pose-depth cross term)
```

**Schur Complement** (`geom/chol.py:46-73`):
```
H_schur = H - E·C⁻¹·E^T  (仅 P×6 维)
v_schur = v - E·C⁻¹·w
因 C 是对角阵 → C⁻¹ 是逐元素倒数 → O(P·M) cost
dx = Cholesky(H_schur, v_schur)  → P×6 位姿更新
dz = C⁻¹·(w - E^T·dx)            → M 个像素的深度更新
```

### 9.3 深度传感器管线 (RGB-D 模式)

#### 深度融合 (`depth_video.py:133-160`):
```
若 depth_sensor 有效:
  1. 将传感器深度反算为逆深度: disp_sens = 1.0 / depth
  2. 与当前估计的逆深度比较:
     若 |disp_est - disp_sens| > threshold → 用传感器值替换
     若已有关键帧约束 → 加权融合
  3. disps_sens 标记已融合的像素 (防止重复融合)
```

### 9.4 跨帧协同

#### 数据缓冲
```
DepthVideo (共享状态):
  images:   环形缓冲区 [t, rig*B, 3, H, W]  (默认 t=15)
  poses:   环形缓冲区 [t, rig*B, 7]          (SE3×t)
  disps:    [t, rig*B, H/8, W/8]            (逆深度)
  intrinsics: [t, rig*B, 4]
  fmaps:    [t, rig*B, 128, H/8, W/8]      (CNN 特征缓存)
  环形缓冲区策略: FIFO, 最老帧自动覆盖
```

#### 因子图管理
```
FactorGraph 边管理 (factor_graph.py):
  ii, jj: 边端点索引
  target: GRU 预测目标坐标 [N_edges, H, W, 2]
  weight: 网络预测置信度 [N_edges, H, W, 2]
  age: 边年龄 (迭代计数)
  ii_inac, jj_inac: 非活跃因子 (已边缘化)

边创建: 
  add_proximity_factors():
    distance(ii, jj, beta=0.3) → 光流基距离
    按距离排序 → NMS 抑制相邻边 → 双向连接
    threshold = kf_translation × median_depth (默认 2.5)
  
边删除:
  rm_factors(age > max_age, max_age=20)
```

#### 初始化管线
```
Warmup (warmup=8 帧):
  1. 前 8 帧不进行关键帧选择, 直接构建初始因子图
  2. MotionFilter 过滤静止帧 (motion_threshold)
  3. 前 2 帧位姿固定 (fixedp=2, 尺度锚定)

关键帧操作:
  is_keyframe(): 运动距离 + 共视度 → 决定
  rm_keyframe(): 直接删除 (无显式边缘化先验)
  video.normalize(): 逆深度均值→1, 位姿同步缩放
```

#### 降级策略
```
漂移处理: video.normalize() 定期尺度归一化
Tracking 退化: 后端每次终止执行两次全局 BA (7 iters + 12 iters)
静止场景: MotionFilter 丢弃低运动帧 (节省计算)
内存压力: AltCorrBlock 替代全对相关体 (后端节省 O(t²·H²·W²) 存储)
动态物体: 无显式处理, 依赖网络 weight 隐式抑制
```

---

## 9. 精度与鲁棒性对比

### 8.1 DROID-SLAM 的精度优势来源

| 因素 | 机制 |
|------|------|
| **稠密BA** | 每帧数千个残差 vs 传统几百个，信息量更高 |
| **学习式匹配** | 端到端训练的特征匹配，比手工ORB+SIFT更鲁棒 |
| **运动先验** | 阻尼速度模型 (`motion_damping`) 提供良好初值 |
| **置信度加权** | 网络输出逐像素置信度，自动抑制外点 |
| **尺度归一化** | `video.normalize()` 自动处理单目尺度漂移 |

### 8.2 与 ORB-SLAM3 的关键差异

```
ORB-SLAM3:
  特征提取 → 2D-2D匹配 → 三角化 → 3D-2D PnP → 局部BA(重投影误差)
  ↓ 依赖RANSAC处理外点
  ↓ 依赖词袋做回环

DROID-SLAM:
  特征提取 → correlation volume → GRU预测光流 → DenseBA(光流残差)
  ↓ 依赖学习式置信度处理外点
  ↓ 无显式回环检测 (依赖稠密BA隐式保持全局一致性)
```

### 8.3 弱纹理场景的鲁棒性

DROID-SLAM 在弱纹理场景下优于 ORB-SLAM3 的根本原因：
- ORB 角点在弱纹理区域**完全失效**（角点响应函数无响应）
- CNN 特征即使在弱纹理区域也能提取**有区分力的低层纹理**（梯度、颜色变化等）

### 8.4 已知局限

- **大尺度漂移**：无回环检测，长距离累计误差无法消除
- **初始化要求**：需要 warmup 帧（`args.warmup=8`）建立初始图
- **GPU 依赖**：必须在 GPU 上运行
- **动态场景**：无运动分割，动态物体会污染优化

---

## 10. DROID-SLAM 对 SLAM 算法的意义

### 9.1 深度学习对 SLAM 的启示

1. **端到端特征**替代手工特征：CNN 提取的特征比 ORB/SIFT 更鲁棒
2. **学习式匹配**替代几何匹配：相关体 + GRU 比描述子 + RANSAC 更高效
3. **稠密优化**替代稀疏优化：GPU 并行使稠密 BA 实时可行
4. **置信度学习**替代硬阈值外点剔除：网络自动判断哪些匹配可靠

### 9.2 可借鉴到传统方法的思路

| 思路 | 传统方法如何借鉴 |
|------|-----------------|
| 光流残差替代重投影残差 | 不需要三角化，直接优化光流一致性 |
| Dense BA 的 Schur 补 | 将逐像素深度作为独立参数，Schur消元后只有位姿 |
| 运动距离选择关键帧 | 基于光流均值的自适应关键帧选择策略 |
| damping factor 学习 | 为不同区域自适应加阻尼，避免深度发散 |

### 9.3 DROID-SLAM 的技术遗产

DROID-SLAM 开创了一条**"类直接法 + 深度学习"**的技术路线：

```
RAFT (光流) → DROID-SLAM (单目SLAM) → SimpleRecon (MVS) → ...
```

它的核心贡献是证明了**不需要显式特征匹配、三角化、回环检测**也能构建全局一致的 SLAM 系统，只要网络足够强大、BA 足够稠密。

---

## 附录: 关键文件索引

| 文件 | 行数 | 核心内容 |
|------|------|---------|
| `droid_slam/droid_net.py` | 222 | 网络结构定义，forward完整流程 |
| `droid_slam/factor_graph.py` | 412 | 因子图数据结构，边管理，update逻辑 |
| `droid_slam/geom/ba.py` | 158 | Dense BA (BA) 和 Motion-only BA (MoBA) |
| `droid_slam/geom/chol.py` | 73 | Cholesky求解器 + Schur消元算法 |
| `droid_slam/geom/projective_ops.py` | 214 | 投影变换 + 完整雅可比计算 |
| `droid_slam/geom/losses.py` | 118 | 训练损失函数：测地线/残差/光流 |
| `droid_slam/modules/extractor.py` | 198 | ResNet式特征编码器 |
| `droid_slam/modules/corr.py` | 117 | 全对相关体 + 多尺度金字塔采样 |
| `droid_slam/modules/gru.py` | 34 | 带全局上下文的ConvGRU |
| `droid_slam/depth_video.py` | 225 | 共享状态存储，BA/Distance调用 |
| `droid_slam/droid_frontend.py` | 164 | 前端局部窗口优化逻辑 |
| `droid_slam/droid_backend.py` | 88 | 后端全局BA |
| `droid_slam/droid.py` | 86 | 顶层Droid类，进程管理 |
| `droid_slam/motion_filter.py` | 91 | 运动滤波，特征提取 |
| `droid_slam/trajectory_filler.py` | 111 | 非关键帧位姿填充 |
| `train.py` | 187 | 训练主循环，DDP多GPU训练 |
| `demo.py` | 137 | 演示/推理入口脚本 |