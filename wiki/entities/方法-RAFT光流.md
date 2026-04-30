---
tags: [方法, 深度学习, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-30
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
---

# RAFT光流

> 用全对相关体和循环更新网络估计 dense optical flow；DROID-SLAM 借用这一路线，把光流更新变成多帧几何优化的前端。

## 定义

RAFT（Recurrent All-Pairs Field Transforms）是一种深度学习光流框架。它先计算两帧所有像素特征之间的 all-pairs correlation，再通过循环网络多次迭代更新光流。DROID-SLAM 继承这个思想，但把结果接入 Dense BA，使光流预测服务于位姿和深度优化。

## 核心结构

1. 特征网络提取两帧图像特征。
2. 计算所有像素对特征内积，形成 4D 相关体。
3. 构建多尺度相关金字塔。
4. 在当前光流位置附近做局部 lookup，例如 4 层、7x7 邻域，对应 196 维相关特征。
5. ConvGRU 根据相关特征、上下文和当前运动状态预测光流修正量。

## 与传统光流的区别

传统 KLT 光流依赖局部亮度一致性和小运动假设；RAFT 通过学习到的特征和全局相关体处理更大位移和更复杂外观变化。代价是需要 GPU 和训练好的网络。

## 在 DROID-SLAM 中的角色

DROID-SLAM 不把 RAFT 光流当作最终输出，而是把它作为几何优化的迭代前端：网络预测目标流和置信度，Dense BA 根据相机位姿和逆深度解释这些流。这样系统同时获得学习式匹配能力和几何可解释性。

## Agent 实现提示

### 适用场景

当 Agent 需要为 SLAM 前端实现“学习式密集匹配 + 几何后端”的模块时，可复用 RAFT 光流思想：先建立全对相关体，再在当前投影位置附近 lookup，最后用循环更新网络输出目标流和权重。它适合两帧或多帧边上的 dense correspondence，不适合低算力 CPU-only 的稀疏特征跟踪。

### 输入输出契约

- 特征输入 `fmap1/fmap2`: `Tensor[B, N, C, H, W]`，其中 `N` 可表示图中边或帧对数量。
- 坐标输入 `coords`: `Tensor[B, N, H, W, 2]`，表示在第二帧特征图上的查询位置。
- 相关输出 `corr`: `Tensor[B, N, L*(2r+1)^2, H, W]`，DROID-SLAM 默认 `L=4, r=3`，即 196 通道。
- 更新输出：`delta` 修正目标坐标，`weight` 作为后端几何优化置信度。

### 实现骨架（伪代码）

```text
function raft_slam_step(images, poses, inverse_depths, intrinsics, graph):
    fmaps, hidden, context = feature_encoder(images)
    corr_fn = CorrBlock(fmaps[edge_i], fmaps[edge_j], levels=4, radius=3)
    coords0 = feature_grid(H/8, W/8)
    coords1 = project(poses, inverse_depths, intrinsics, edge_i, edge_j)
    target = clone(coords1)
    repeat K times:
        corr = corr_fn(coords1)
        residual = target - coords1
        motion = concat(coords1 - coords0, residual)
        hidden, delta, weight = update(hidden, context, corr, motion)
        target = coords1 + delta
        poses, inverse_depths = dense_ba(target, weight)
        coords1 = project(poses, inverse_depths, intrinsics, edge_i, edge_j)
    return poses, inverse_depths, target, weight
```

### 关键源码片段

`raw/codes/DROID-SLAM/droid_slam/modules/corr.py:L23-L50`

```python
class CorrBlock:
    def __init__(self, fmap1, fmap2, num_levels=4, radius=3):
        self.num_levels = num_levels
        self.radius = radius
        self.corr_pyramid = []

        # all pairs correlation
        corr = CorrBlock.corr(fmap1, fmap2)

        batch, num, h1, w1, h2, w2 = corr.shape
        corr = corr.reshape(batch*num*h1*w1, 1, h2, w2)

        for i in range(self.num_levels):
            self.corr_pyramid.append(
                corr.view(batch*num, h1, w1, h2//2**i, w2//2**i))
            corr = F.avg_pool2d(corr, 2, stride=2)

    def __call__(self, coords):
        out_pyramid = []
        batch, num, ht, wd, _ = coords.shape
        coords = coords.permute(0,1,4,2,3)
        coords = coords.contiguous().view(batch*num, 2, ht, wd)

        for i in range(self.num_levels):
            corr = CorrSampler.apply(self.corr_pyramid[i], coords/2**i, self.radius)
            out_pyramid.append(corr.view(batch, num, -1, ht, wd))

        return torch.cat(out_pyramid, dim=2)
```

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L181-L214`

```python
        fmaps, net, inp = self.extract_features(images)
        net, inp = net[:,ii], inp[:,ii]
        corr_fn = CorrBlock(fmaps[:,ii], fmaps[:,jj], num_levels=4, radius=3)

        ht, wd = images.shape[-2:]
        coords0 = pops.coords_grid(ht//8, wd//8, device=images.device)

        coords1, _ = pops.projective_transform(Gs, disps, intrinsics, ii, jj)
        target = coords1.clone()

        Gs_list, disp_list, residual_list = [], [], []
        for step in range(num_steps):
            Gs = Gs.detach()
            disps = disps.detach()
            coords1 = coords1.detach()
            target = target.detach()

            # extract motion features
            corr = corr_fn(coords1)
            resd = target - coords1
            flow = coords1 - coords0

            motion = torch.cat([flow, resd], dim=-1)
            motion = motion.permute(0,1,4,2,3).clamp(-64.0, 64.0)

            net, delta, weight, eta, upmask = \
                self.update(net, inp, corr, motion, ii, jj)

            target = coords1 + delta

            for i in range(2):
                Gs, disps = BA(target, weight, eta, Gs, disps, intrinsics, ii, jj, fixedp=2)

            coords1, valid_mask = pops.projective_transform(Gs, disps, intrinsics, ii, jj)
```

### 实现注意事项

- 相关体内存开销随 `H*W*H*W` 增长，必须在低分辨率特征图上构建。
- `coords` 的最后一维是 `(x, y)`，进入 CUDA sampler 前会转成 `B*N x 2 x H x W`。
- RAFT 输出的 `delta` 在 DROID-SLAM 中是几何目标，不是最终位姿；位姿和逆深度由 `BA(...)` 更新。
- 对多帧图边，`ii/jj` 的索引必须和 `fmap/net/inp/coords` 的 `num` 维保持一致。

### 源码检索锚点

- `raw/codes/DROID-SLAM/droid_slam/modules/corr.py`：`class CorrBlock`、`CorrSampler.apply`、`all-pairs correlation`。
- `raw/codes/DROID-SLAM/droid_slam/droid_net.py`：`corr_fn = CorrBlock`、`motion = torch.cat([flow, resd]`、`BA(target, weight, eta`。
- `raw/codes/DROID-SLAM/src/correlation_kernels.cu`：`corr_index_cuda_forward` 可继续追踪底层采样核。

## 相关页面

- [[算法-DROID-SLAM]]
- [[方法-ConvGRU]]
- [[方法-Dense BA]]
- [[方法-视觉特征跟踪]]
- [[概念-深度学习SLAM]]
