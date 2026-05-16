---
tags: [论文, 渲染, 3DGS, Gaussian Splatting, 新视角合成, 实时渲染]
type: source
sources:
  - "https://arxiv.org/abs/2308.04079"
created: 2026-05-15
updated: 2026-05-15
paper_arxiv: "2308.04079"
paper_title: "3D Gaussian Splatting for Real-Time Radiance Field Rendering"
paper_authors: "Bernhard Kerbl, Georgios Kopanas, Thomas Leimkühler, George Drettakis"
paper_venue: "ACM SIGGRAPH 2023"
paper_url: "https://arxiv.org/abs/2308.04079"
paper_code: "https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/"
---

# 3D Gaussian Splatting 论文精读

> 3DGS 用一组可微的各向异性 3D Gaussian 椭球表示场景，替代 NeRF 的隐式 MLP 场，通过高效 tile-based 光栅化实现 SOTA 渲染质量和 30+ FPS 实时渲染。

## 论文信息

| 项目 | 内容 |
|------|------|
| 标题 | 3D Gaussian Splatting for Real-Time Radiance Field Rendering |
| 作者 | Bernhard Kerbl, Georgios Kopanas, Thomas Leimkühler, George Drettakis |
| 发表 | ACM SIGGRAPH 2023 (Best Paper) |
| arXiv | [2308.04079](https://arxiv.org/abs/2308.04079) |
| 代码 | [graphdeco-inria/gaussian-splatting](https://github.com/graphdeco-inria/gaussian-splatting) |
| 机构 | Inria, Université Côte d'Azur |

## 核心贡献

### 1. 3D Gaussian 作为显式场景表示

每个 Gaussian primitive 由以下可优化参数定义：

| 参数 | 维度 | 含义 |
|------|------|------|
| $\mu$（位置） | 3 | Gaussian 中心坐标 |
| $\Sigma$（协方差） | $3 \times 3$ | 各向异性形状，分解为 $R S S^T R^T$ |
| $\alpha$（不透明度） | 1 | 经 sigmoid 限制在 $[0,1]$ |
| SH 系数 | 每通道 48 维 | [[数学-球谐函数]] 表达视角相关颜色 |

相比于 NeRF 的隐式 MLP 查询，3DGS 是显式点云表示，可通过梯度直接移动/Split/Clone 点位置。

### 2. Tile-Based 可微光栅化器（Differentiable Rasterizer）

将 3D Gaussian 投影为 2D 椭圆，按 tile 分组排序，通过 Alpha Blending 合成像素颜色——全程可微，无需求解 NeRF 的大量射线采样积分：

$$
\Sigma' = J W \Sigma W^T J^T
$$

其中 $W$ 为世界到相机的旋转，$J$ 为仿射投影近似的 Jacobian。

### 3. 自适应密度控制（Adaptive Density Control）

在优化过程中动态调整 Gaussian 的数量和分布：

- **Clone**：对梯度较大的小 Gaussian，复制以增生几何细节
- **Split**：对梯度大的大 Gaussian，分裂为两个更小的 Gaussian
- **Prune**：移除不透明度 < 阈值的低贡献 Gaussian 及世界坐标外浮点

每经过固定迭代次数执行一次，使 Gaussian 数量从初始稀疏点云自动增长到百万量级。

### 4. 基于 COLMAP 的初始化

从 COLMAP / SfM 输出的稀疏点云初始化 3D Gaussian 的位置 $\mu$ 和初始颜色。初始协方差设为各向同性（由最邻近三个点的平均距离确定）。这种结构光质量初始化使优化快速收敛，无需像 NeRF 那样从零探索。

### 5. 随机梯度下降（SGD）优化

使用 Adam 优化器，损失函数为 L1 + SSIM D-SSIM：

$$
\mathcal{L} = (1 - \lambda) \mathcal{L}_1(I, \hat{I}) + \lambda \mathcal{L}_\text{D-SSIM}(I, \hat{I})
$$

每步迭代从训练集中采样一帧，减少内存占用，比 Mip-NeRF360 训练快 100-1000 倍。

## 关键公式

**3D Gaussian 的协方差正定分解**：

$$
\Sigma = R S S^T R^T
$$

$R$ 为旋转四元数对应的 $3\times3$ 旋转矩阵，$S$ 为各向异性尺度（对角矩阵）。这保证了 $\Sigma$ 始终为正定矩阵，且梯度可直接传播到 $R$ 和 $S$。

**2D 投影后的椭圆 footprint**：

$$
\Sigma' = (J W R) S S^T (J W R)^T
$$

其中 $J$ 是投影变换的 Jacobian（仅在 Gaussian 中心处取仿射近似），$W$ 为 view matrix。

**Alpha Blending 像素颜色合成**：

$$
C = \sum_{i \in \mathcal{N}} c_i \cdot \alpha_i' \cdot \prod_{j=1}^{i-1} (1 - \alpha_j')
$$

$$
\alpha_i' = \alpha_i \cdot \exp\left(-\frac{1}{2} (x' - \mu_i')^T (\Sigma_i')^{-1} (x' - \mu_i')\right)
$$

$\mathcal{N}$ 为与该像素重叠的 Gaussian 集合（按深度排序），$x'$ 为当前像素的 2D 坐标，$c_i$ 为第 $i$ 个 Gaussian 经 SH 评估的视角相关颜色。

**SH 视角相关颜色**：

$$
c(\mathbf{d}) = \sum_{l=0}^{L} \sum_{m=-l}^{l} k_l^m Y_l^m(\mathbf{d})
$$

使用 $L=3$（即 16 个系数/通道 × 3 通道 = 48 个 SH 参数），$\mathbf{d}$ 为从 Gaussian 中心到相机的观察方向。

## 与知识库现有内容的联动

### 与 [[方法-3D Gaussian Splatting]] 的关系

现有实体页侧重于 3DGS 的基本定义和渲染流程。本文档补充了论文级数学细节（协方差分解、投影 Jacobian、密度控制策略、SH 颜色公式、tile-based 排序与混合），并提供了与 SLAM 结合的实现提示。

### 与 [[概念-体积渲染]]、[[概念-可微渲染]] 的对比

| 维度 | 体积渲染 (NeRF) | 3DGS Splatting |
|------|----------------|----------------|
| 场景表示 | 隐式 MLP | 显式 3D Gaussian 群 |
| 渲染方式 | 沿射线采样积分 → MLP | 投影 + tile 排序 + α 混合 |
| 渲染速度 | 慢（~数秒/帧） | 快（1080p > 30 FPS） |
| 训练速度 | 慢（~数小时） | 快（~数十分钟） |
| 梯度流 | 稠密（沿射线全程） | 稀疏（仅命中像素的 Gaussian） |

### 与 [[数学-球谐函数]] 的关系

3DGS 的视角相关颜色由 SH 系数编码，$L=3$ 共 16 个系数/通道。SH 是 3DGS 表达 specular 效果的核心组件。

### 与 [[方法-微分栅格化]] 的关系

3DGS 的 tile-based 光栅化器是"微分栅格化"概念的旗舰实现：每个 tile 内的 Gaussian 排序、α 混合和颜色累积在上层都可见梯度，使 Gaussian 参数（位置、协方差、颜色、不透明度）均可通过 L1+D-SSIM 损失优化。

### 与 [[算法-MonoGS]] 的关系

MonoGS（单目 Gaussian Splatting SLAM）将 3DGS 引入 SLAM 前端：用 Gaussian 表示地图，通过体积/溅射渲染误差跟踪位姿，并在线更新 Gaussian 参数。3DGS 论文是 MonoGS 的渲染理论基础。

### 与 [[方法-RGB着色点云]] 的对比

| 维度 | RGB 着色点云 | 3DGS |
|------|-------------|------|
| 几何表示 | 离散 3D 点 + 颜色 | 连续 Gaussian 椭球 + SH 颜色 |
| 新视角质量 | 空洞、模糊 | 各向异性填充、高保真 |
| 实时性 | 快 | 快（投影 + α 混合） |
| 可微性 | 有限 | 完全可微 |

## Agent 实现提示

### 适用场景

3DGS 适合多视图图像的新视角合成任务，特别是需要高质量（SSIM > 0.9）和实时渲染（> 30 FPS）的场景。也适合作为 SLAM 的可微地图表示（如 MonoGS），但需额外设计 tracking 部分。

### 输入输出契约

- **输入**：校准后的多视图图像集 + COLMAP 估计的稀疏点云和相机位姿
- **输出**：一组 3D Gaussian primitive（位置 $\mu$、协方差 $\Sigma$、不透明度 $\alpha$、SH 系数），可渲染任意新视角图像
- **坐标**：世界坐标系下 3D 位置，投影到像素坐标系经相机内参 K 和位姿 T
- **格式**：Gaussian 导出为 `.ply` 文件，每点携带所有可学习参数

### 实现骨架（伪代码）

```pseudo
function train_3dgs(images, K, poses_T, initial_points):
    # 从 COLMAP 稀疏点云初始化
    gaussians = init_from_points(initial_points)
    # 初始化协方差为各向同性（3 近邻平均距离）
    for each gaussian in gaussians:
        gaussian.scale = log(mean_dist_to_3_nearest_neighbors)
        gaussian.rotation = identity_quaternion
        gaussian.opacity = inverse_sigmoid(0.1)
        gaussian.sh = random_init()

    for iter in range(30,000):
        # 随机采样视角
        cam = sample_view(images, poses_T, K)
        # Tile-based 可微渲染
        rendered = rasterize(gaussians, cam)
        # L1 + D-SSIM 损失
        loss = (1-0.2)*L1(rendered, cam.image) + 0.2*D_SSIM(rendered, cam.image)
        # 反向传播 + Adam 更新
        loss.backward()
        adam_step(gaussians)

        # 每 100 步检测是否需要自适应密度控制
        if iter % 100 == 0:
            densify_and_prune(gaussians)

    return gaussians
```

### 关键源码片段

官方的 tile-based 光栅化器以 CUDA kernel 实现：

- `submodules/diff-gaussian-rasterization/cuda_rasterizer/forward.cu` — 前向渲染（tile 分配 → 深度排序 → α 混合）
- `submodules/diff-gaussian-rasterization/cuda_rasterizer/backward.cu` — 反向传播（颜色/不透明度/协方差/位置梯度）
- `scene/gaussian_model.py` — GaussianModel 类（参数管理、密度控制、SH 更新）
- `utils/loss_utils.py` — L1 + D-SSIM 损失实现
- `train.py` — 训练主循环（迭代调度、密度控制触发）

### 实现注意事项

- **前向/反向的一致性**：深度排序必须在前向和反向前完全一致（包括 tie-breaking），否则梯度计算错误。官方实现中使用 32-bit 排序 key 并加 tie-breaking offset。
- **tile size**：16×16 像素是官方默认，过大则每个 tile 内 Gaussian 过多影响排序速度，过小则 tile 边界重复处理开销增大。
- **GPU 内存管理**：百万级 Gaussian 的光栅化占用大量显存；adaptive density 可能导致显存溢出（特别是在 1080p+ 分辨率）。
- **透明度阈值更新**：每 3000 步提高一次透明度阈值（inverse sigmoid），避免过早 prune 掉真正在收敛的 Gaussian。
- **梯度饱和**：投影 Jacobian 的仿射近似在大视场角边缘处精度退化，可能导致边缘 Gaussian 更新失真。

### 源码检索锚点

- `scene/gaussian_model.py:L100-L200` — 协方差构建（$RSS^TR^T$ 正定保证）
- `scene/gaussian_model.py:L250-L350` — 自适应密度控制（clone/split/prune 逻辑）
- `gaussian_renderer/__init__.py:L30-L80` — 渲染入口（投影 + 光栅化调度）
- `utils/graphics_utils.py:L20-L60` — 投影 Jacobian 计算
- `diff-gaussian-rasterization/` — CUDA 光栅化器（前向和反向 kernel）

## 相关页面

- [[方法-3D Gaussian Splatting]] — 实体页（基本定义 + 渲染流程）
- [[算法-MonoGS]] — 3DGS 在单目 SLAM 中的应用
- [[概念-可微渲染]] — 可微渲染理论对比（体积 vs 溅射）
- [[概念-体积渲染]] — 神经渲染的另一种范式
- [[数学-球谐函数]] — SH 视角相关颜色数学推导
- [[方法-微分栅格化]] — 微分栅格化通用概念
- [[方法-RGB着色点云]] — 传统点云着色 vs Gaussian 溅射
- [[概念-深度学习SLAM]] — 学习式 SLAM 全貌（含可微地图表示）
