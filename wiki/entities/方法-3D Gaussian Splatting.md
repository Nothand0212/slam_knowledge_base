---
tags: [方法, 3DGS, 渲染, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-30
sources:
  - wiki/sources/2026-04-29-monogs-analysis.md
---

# 3D Gaussian Splatting

> 用大量可微的各向异性 3D Gaussian primitive 表示场景，并通过投影、排序和 alpha blending 直接渲染图像。

## 定义

3D Gaussian Splatting（3DGS）把场景表示为一组 3D 高斯椭球。每个 Gaussian 是一个可优化 primitive，而不是传统 SLAM 中的稀疏 MapPoint 或体素网格。

每个 Gaussian 通常包含：

- 位置 `xyz`。
- 尺度 `scale`，通过 `exp` 保证为正。
- 旋转 `rotation`，用四元数并归一化。
- 不透明度 `opacity`，通过 `sigmoid` 限制在 `[0,1]`。
- 颜色或 [[数学-球谐函数]] 系数，用于表达视角相关外观。

## 渲染流程

1. 将 3D Gaussian 投影到图像平面，得到 2D 椭圆 footprint。
2. 按 tile 组织待渲染 Gaussian，降低逐像素遍历成本。
3. 对每个像素按深度排序。
4. 使用 front-to-back alpha blending 合成颜色：

```text
C = sum_i T_i α_i c_i
T_i = product_{j<i} (1 - α_j)
```

其中 `α_i` 由 Gaussian 对像素的 2D 密度和 opacity 决定，`T_i` 是前面 Gaussian 透过后的剩余透明度。

## 在 MonoGS 中的作用

MonoGS 将 3DGS 作为地图表示。Tracking 阶段固定 Gaussian 参数，优化相机位姿和曝光参数，使渲染图像接近当前观测；Mapping 阶段在关键帧窗口内联合优化 Gaussian 参数和关键帧位姿，并通过稠密化、剪枝和 isotropic 正则控制地图质量。

这使得地图不仅能定位，还能直接渲染新视角；代价是参数量和 GPU 内存显著高于稀疏特征地图。

## 工程取舍

- 优点：稠密外观表达强，可直接输出可渲染场景。
- 代价：百万级 Gaussian 带来显存和优化压力。
- SLAM 风险：位姿误差会驱动 Gaussian 沿视线方向拉伸，需要正则化和关键帧管理。
- 适用场景：RGB-D、室内稠密重建、AR/VR 视图合成；不适合作为低算力实时里程计前端。

## Agent 实现提示

### 适用场景

当 Agent 需要实现可实时渲染、可反向传播的稠密地图时使用 3DGS：Tracking 固定高斯地图优化相机位姿，Mapping 联合优化高斯参数和关键帧位姿。它适合 RGB-D/双目/单目稠密 SLAM 和新视角合成，不适合只需要稀疏定位、内存受限或 CPU-only 的场景。

### 输入输出契约

- 地图输入 `pc`: GaussianModel，核心张量包括 `xyz`、SH/颜色、`opacity`、`scaling`、`rotation`。
- 相机输入 `viewpoint_camera`: 提供图像尺寸、FOV、view/projection matrix、camera center，以及可优化 pose delta。
- 渲染输出 `render_pkg`: 至少包含 `render`、`depth`、`opacity`、`viewspace_points`、`visibility_filter`、`radii`。
- Mapping loss 输出标量 `loss_mapping`，反传到 Gaussian 参数和可能的关键帧 pose 参数。

### 实现骨架（伪代码）

```text
function map_gaussians(keyframes, gaussians):
    loss = 0
    for viewpoint in sample_window(keyframes):
        pkg = render(viewpoint, gaussians, pipeline, background)
        loss += photometric_depth_loss(pkg.render, pkg.depth, viewpoint.gt, pkg.opacity)
        collect_visibility_stats(pkg.viewspace_points, pkg.visibility_filter, pkg.radii)
    loss += isotropic_regularization(gaussians.scaling)
    loss.backward()
    update_densification_stats()
    maybe_prune_or_split_gaussians()
    gaussians.optimizer.step()
    keyframe_optimizer.step()
```

### 关键源码片段

`raw/codes/MonoGS/gaussian_splatting/gaussian_renderer/__init__.py:L24-L58`

```python
def render(
    viewpoint_camera,
    pc: GaussianModel,
    pipe,
    bg_color: torch.Tensor,
    scaling_modifier=1.0,
    override_color=None,
    mask=None,
):
    """
    Render the scene.

    Background tensor (bg_color) must be on GPU!
    """

    # Create zero tensor. We will use it to make pytorch return gradients of the 2D (screen-space) means
    if pc.get_xyz.shape[0] == 0:
        return None

    screenspace_points = (
        torch.zeros_like(
            pc.get_xyz, dtype=pc.get_xyz.dtype, requires_grad=True, device="cuda"
        )
        + 0
    )
    try:
        screenspace_points.retain_grad()
    except Exception:
        pass

    # Set up rasterization configuration
    tanfovx = math.tan(viewpoint_camera.FoVx * 0.5)
    tanfovy = math.tan(viewpoint_camera.FoVy * 0.5)

    raster_settings = GaussianRasterizationSettings(
```

`raw/codes/MonoGS/gaussian_splatting/gaussian_renderer/__init__.py:L115-L153`

```python
    # Rasterize visible Gaussians to image, obtain their radii (on screen).
    if mask is not None:
        rendered_image, radii, depth, opacity = rasterizer(
            means3D=means3D[mask],
            means2D=means2D[mask],
            shs=shs[mask],
            colors_precomp=colors_precomp[mask] if colors_precomp is not None else None,
            opacities=opacity[mask],
            scales=scales[mask],
            rotations=rotations[mask],
            cov3D_precomp=cov3D_precomp[mask] if cov3D_precomp is not None else None,
            theta=viewpoint_camera.cam_rot_delta,
            rho=viewpoint_camera.cam_trans_delta,
        )
    else:
        rendered_image, radii, depth, opacity, n_touched = rasterizer(
            means3D=means3D,
            means2D=means2D,
            shs=shs,
            colors_precomp=colors_precomp,
            opacities=opacity,
            scales=scales,
            rotations=rotations,
            cov3D_precomp=cov3D_precomp,
            theta=viewpoint_camera.cam_rot_delta,
            rho=viewpoint_camera.cam_trans_delta,
        )

    # Those Gaussians that were frustum culled or had a radius of 0 were not visible.
    # They will be excluded from value updates used in the splitting criteria.
    return {
        "render": rendered_image,
        "viewspace_points": screenspace_points,
        "visibility_filter": radii > 0,
        "radii": radii,
        "depth": depth,
        "opacity": opacity,
        "n_touched": n_touched,
    }
```

### 实现注意事项

- `screenspace_points.retain_grad()` 是稠密化统计的关键，不要在 render 后 detach 掉。
- `opacity/scaling/rotation` 通常以未约束参数存储，通过 getter 激活；优化器应更新原始参数而不是激活后的值。
- Mapping 中需要约束尺度各向同性，否则位姿误差会让 Gaussian 沿视线方向异常拉伸。
- 渲染返回的 `visibility_filter/radii/n_touched` 不只是调试信息，还会影响 split/prune 策略。

### 源码检索锚点

- `raw/codes/MonoGS/gaussian_splatting/gaussian_renderer/__init__.py`：`def render`、`GaussianRasterizationSettings`、`GaussianRasterizer`。
- `raw/codes/MonoGS/gaussian_splatting/scene/gaussian_model.py`：`training_setup`、`densify_and_prune`、`get_opacity`。
- `raw/codes/MonoGS/utils/slam_backend.py`：`loss_mapping.backward()`、`self.gaussians.optimizer.step()`。

## 相关页面

- [[算法-MonoGS]]
- [[方法-微分栅格化]]
- [[数学-球谐函数]]
- [[概念-可微渲染]]
- [[概念-深度学习SLAM]]
