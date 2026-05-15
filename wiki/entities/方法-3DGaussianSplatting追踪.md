---
tags: [方法, 3DGS, 位姿优化, SLAM, 可微渲染]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-29-monogs-analysis.md
---

# 3D Gaussian Splatting 追踪

> MonoGS 的相机追踪：固定 3D Gaussian 地图，通过可微栅格化渲染图像并与观测图像比较，用光度损失和深度损失在 SE(3) 流形上优化相机位姿。

## 核心思路

传统直接法 SLAM 在光度一致性假设下，通过最小化当前帧与上一帧/局部地图的逐像素光度残差估计位姿增量。MonoGS 的追踪继承了直接法框架，但将"地图"从稀疏点云/深度图替换为可微 3D Gaussian 辐射场：

1. **前向渲染**：将当前 3D Gaussian 地图通过投影 + tile-based 排序 + alpha blending，渲染当前位姿估计下的 RGB 图像和深度图。
2. **误差计算**：将渲染图像与真实观测比较，计算光度 L1 损失（RGB-D 模式下额外加入深度损失）。
3. **梯度回传**：微分栅格化的 CUDA backward 将损失梯度传回相机旋转增量 `cam_rot_delta`、平移增量 `cam_trans_delta` 和曝光参数。
4. **流形更新**：用 SE(3) 指数映射将增量作用于当前位姿，并重置增量变量为零，进入下一轮迭代。

## 相机参数化

MonoGS 在位姿优化中使用右乘 SE(3) 扰动模型。相机位姿表示为 `T_WC`（world-to-camera 变换），每轮迭代优化增量 \(\tau = [\rho; \theta] \in \mathfrak{se}(3)\)：

\[\mathbf{T}_{\text{new}} = \exp(\tau^{\wedge}) \cdot \mathbf{T}_{\text{old}}\]

源码头文件 `raw/codes/MonoGS/utils/pose_utils.py:L76-L93` 实现了 `update_pose`：

```python
def update_pose(camera, converged_threshold=1e-4):
    tau = torch.cat([camera.cam_trans_delta, camera.cam_rot_delta], axis=0)
    T_w2c = torch.eye(4, device=tau.device)
    T_w2c[0:3, 0:3] = camera.R; T_w2c[0:3, 3] = camera.T
    new_w2c = SE3_exp(tau) @ T_w2c   # 右乘扰动
    new_R = new_w2c[0:3, 0:3]; new_T = new_w2c[0:3, 3]
    converged = tau.norm() < converged_threshold
    camera.update_RT(new_R, new_T)
    camera.cam_rot_delta.data.fill_(0)
    camera.cam_trans_delta.data.fill_(0)
    return converged
```

其中 `SE3_exp` 使用罗德里格斯公式计算 SO(3) 指数映射，并用 V 矩阵将平移向量 τ 映射到 SE(3) 平移分量：\( \text{SE3\_exp}(\tau) = \begin{bmatrix} \exp(\theta^\wedge) & V(\theta)\rho \\ 0 & 1 \end{bmatrix} \)。

## 追踪优化器配置

每帧初始化追踪优化器时，为旋转增量、平移增量和曝光参数分配独立的学习率（`raw/codes/MonoGS/utils/slam_frontend.py:L128-L162`）：

```python
opt_params = [
    {"params": [viewpoint.cam_rot_delta],  "lr": lr_rot,  "name": "rot"},
    {"params": [viewpoint.cam_trans_delta], "lr": lr_trans, "name": "trans"},
    {"params": [viewpoint.exposure_a], "lr": 0.01, "name": "exposure_a"},
    {"params": [viewpoint.exposure_b], "lr": 0.01, "name": "exposure_b"},
]
pose_optimizer = torch.optim.Adam(opt_params)
```

每帧执行 `self.tracking_itr_num` 次迭代（通常 40 次），每次迭代流程为：render → compute loss → backward → optimizer.step → update_pose。

## 追踪损失函数

### 单目模式：RGB L1 损失 + 梯度掩码

单目模式下仅依赖光度损失，通过梯度掩码排除低纹理区域，并对边缘施加 opacity 加权：

\[
\mathcal{L}_{\text{tracking}}^{\text{rgb}} = \frac{1}{|M_G|}\sum_{p \in M_G} \alpha_p \cdot |I_{\text{render}}(p) - I_{\text{gt}}(p)|
\]

其中：
- \(M_G\) 是图像梯度掩码，滤除 Scharr 梯度响应过低的像素；
- \(\alpha_p\) 是像素 p 处 alpha blending 累积的透明度，接近 1 表示前景物体；
- 渲染图像先经曝光校正：\(I_{\text{ab}} = \exp(a) \cdot I_{\text{render}} + b\)。

源码 `raw/codes/MonoGS/utils/slam_utils.py:L63-L71`：

```python
def get_loss_tracking_rgb(config, image, depth, opacity, viewpoint):
    gt_image = viewpoint.original_image.cuda()
    rgb_boundary_threshold = config["Training"]["rgb_boundary_threshold"]
    rgb_pixel_mask = (gt_image.sum(dim=0) > rgb_boundary_threshold).view(1, h, w)
    rgb_pixel_mask = rgb_pixel_mask * viewpoint.grad_mask
    l1 = opacity * torch.abs(image * rgb_pixel_mask - gt_image * rgb_pixel_mask)
    return l1.mean()
```

### RGB-D 模式：加权 RGB + Depth L1 损失

深度传感器可用时，加入深度监督项：

\[
\mathcal{L}_{\text{tracking}}^{\text{rgbd}} = \alpha \cdot \mathcal{L}_{\text{rgb}} + (1-\alpha) \cdot \frac{1}{|M_D|}\sum_{p \in M_D} |D_{\text{render}}(p) - D_{\text{gt}}(p)|
\]

其中：
- \(\alpha = 0.95\)（默认），即深度损失的权重约为 5%；
- \(M_D = \{p \mid D_{\text{gt}}(p) > 0.01 \land \alpha_p > 0.95\}\)，仅在高 opacity 区域监督深度；
- 两者均为 L1 而非 L2，增强对 outlier 的鲁棒性。

源码 `raw/codes/MonoGS/utils/slam_utils.py:L74-L88`：

```python
def get_loss_tracking_rgbd(config, image, depth, opacity, viewpoint):
    alpha = 0.95
    gt_depth = torch.from_numpy(viewpoint.depth).to(dtype=torch.float32, device=image.device)[None]
    depth_pixel_mask = (gt_depth > 0.01).view(*depth.shape)
    opacity_mask = (opacity > 0.95).view(*depth.shape)
    l1_rgb = get_loss_tracking_rgb(config, image, depth, opacity, viewpoint)
    depth_mask = depth_pixel_mask * opacity_mask
    l1_depth = torch.abs(depth * depth_mask - gt_depth * depth_mask)
    return alpha * l1_rgb + (1 - alpha) * l1_depth.mean()
```

### 曝光在线估计

MonoGS 在线学习两个 per-frame 曝光参数：\(a\)（尺度）和 \(b\)（偏移），渲染输出经变换 `exp(a)·render + b` 后再与观测比较。这使系统能自适应当前帧的光照变化，类似传统直接法中的增益估计。

## 关键帧选择与滑动窗口

### 帧间可见性度量

追踪结束后，MonoGS 使用两种度量来决定是否创建新关键帧：

**共可见性比率**（Szymkiewicz–Simpson coefficient）：
\[
\text{overlap} = \frac{|V_{\text{curr}} \cap V_{\text{last\_kf}}|}{\min(|V_{\text{curr}}|, |V_{\text{last\_kf}}|)}
\]
其中 \(V\) 是 visibility filter（`n_touched > 0` 的像素）指示哪些 Gaussian 对该帧有贡献。

**关键帧判定**：源码 `raw/codes/MonoGS/utils/slam_frontend.py:L198-L225`：

```python
def is_keyframe(self, cur_frame_idx, last_keyframe_idx,  cur_frame_visibility_filter, occ_aware_visibility):
    union = torch.logical_or(cur_frame_visibility_filter, occ_aware_visibility[last_keyframe_idx]).count_nonzero()
    intersection = torch.logical_and(cur_frame_visibility_filter, occ_aware_visibility[last_keyframe_idx]).count_nonzero()
    point_ratio_2 = intersection / union
    return (point_ratio_2 < kf_overlap and dist_check2) or dist_check
```

条件：重叠比率低于 `kf_overlap`（默认 0.8）且帧间距超过最小距离，或位移超过 `kf_translation * median_depth`。

### 滑动窗口管理

`add_to_window` 维护固定容量（默认 10 帧）的优化窗口：新关键帧加入队首，逐帧计算与窗口内现有帧的 Szymkiewicz–Simpson 重叠系数，剔除低重叠帧；窗口满时用距离加权移除最远端帧。

## 各向同性正则化

建图阶段（BackEnd）对各向异性的 Gaussian 尺度施加 isotropic regularization，防止 Gaussian 沿视线方向过度拉伸：

\[
\mathcal{L}_{\text{iso}} = \frac{1}{N}\sum_{i=1}^N \left\| s_i - \frac{1}{3}\sum_{d=1}^3 s_{i,d} \right\|
\]

源码 `raw/codes/MonoGS/utils/slam_backend.py:L229-L231`：

```python
scaling = self.gaussians.get_scaling
isotropic_loss = torch.abs(scaling - scaling.mean(dim=1).view(-1, 1))
loss_mapping += 10 * isotropic_loss.mean()
```

权重系数 10 较大，确保正则化在优化中起到实质约束作用。在跟踪阶段不需要该正则项（Gaussian 参数已固定）。

## 深度中值估计与单目初始化

单目模式没有传感器深度，MonoGS 使用渲染深度和 opacity 的中值进行尺度校准：

源码 `raw/codes/MonoGS/utils/slam_utils.py:L131-L142`：

```python
def get_median_depth(depth, opacity=None, mask=None, return_std=False):
    depth = depth.detach().clone(); opacity = opacity.detach()
    valid = depth > 0
    if opacity is not None: valid = torch.logical_and(valid, opacity > 0.95)
    if mask is not None: valid = torch.logical_and(valid, mask)
    valid_depth = depth[valid]
    if return_std: return valid_depth.median(), valid_depth.std(), valid
    return valid_depth.median()
```

`median_depth` 用于自适应调整关键帧平移阈值（乘以该值），使判定能适应不同尺度场景。

---

## Agent 实现提示

### 适用场景

Agent 需要实现**可微渲染驱动的相机位姿优化**时适用。场景中有稠密 3D 表示（3D Gaussian、NeRF/HashGrid/SDF）且可通过可微渲染产生 RGB/深度图像；适合需要端到端可微且能输出高质量合成视图的 SLAM 前端。不适合纯 CPU-only、极低算力嵌入式系统或基于特征匹配的间接法 SLAM。

### 输入输出契约

- **输入**：当前帧 RGB 图像（H×W×3，值域 [0,255]）、深度图（可选，H×W）、三维 Gaussian 地图（中心/尺度/旋转/opacity/颜色/SH 系数）、相机内参（fx, fy, cx, cy）、初始位姿估计、渲染背景色。
- **输出**：优化后的相机位姿 T_WC（4×4 Tensor）、渲染图像、渲染深度图、opacity 图、中值深度、visibility filter。
- **坐标约定**：位姿为 world-to-camera 变换 `T_WC`；深度沿相机 Z 轴，单位与场景一致（米级）；像素使用 pinhole 模型投影。

### 实现骨架（伪代码）

```pseudo
function trackMonoGS(frame, gaussians, camera_prev, intrinsics, config):
    # 1. 初始化追踪参数
    camera = Camera.from_prev(frame, camera_prev)
    camera.cam_rot_delta = zeros(3)
    camera.cam_trans_delta = zeros(3)
    camera.set_exposure(exposure_a, exposure_b)

    # 2. 建立优化器
    optimizer = Adam([
        camera.cam_rot_delta with lr_rot,
        camera.cam_trans_delta with lr_trans,
        camera.exposure_a, camera.exposure_b with lr_exposure
    ])

    # 3. 迭代优化
    for itr in range(tracking_iters):
        # 前向渲染
        render_pkg = differentiable_rasterizer(camera, gaussians, bg_color)
        image_rendered, depth_rendered, opacity_rendered = render_pkg

        # 曝光校正
        image_corrected = exp(exposure_a) * image_rendered + exposure_b

        # 光度损失
        mask = (gt.sum(2) > threshold) AND grad_mask
        loss_rgb = L1(image_corrected[mask], frame[mask]) * opacity[mask]

        # 深度损失（如有传感器深度）
        if has_depth:
            mask_depth = (gt_depth > 0.01) AND (opacity > 0.95)
            loss_depth = L1(depth_rendered[mask_depth], gt_depth[mask_depth])
            loss = alpha * loss_rgb + (1 - alpha) * loss_depth
        else:
            loss = loss_rgb

        # 反向传播
        loss.backward()
        optimizer.step()

        # SE(3) 流形更新
        tau = [camera.cam_trans_delta, camera.cam_rot_delta]
        T_new = SE3_exp(tau) @ camera.T
        camera.update_RT(T_new.R, T_new.t)
        camera.cam_rot_delta.zero_(); camera.cam_trans_delta.zero_()

        if tau.norm() < threshold: break

    # 4. 计算关键帧判定指标
    median_depth = depth_rendered[opacity > 0.95].median()
    visibility = render_pkg.n_touched > 0
    overlap = intersect(visibility, last_kf_visibility) / union(visibility, last_kf_visibility)
    is_kf = (overlap < kf_overlap AND dist > kf_min_trans * median_depth) OR dist > kf_trans * median_depth

    return camera.T, render_pkg, is_kf, median_depth, visibility
```

### 关键源码片段

`raw/codes/MonoGS/utils/slam_frontend.py:L128-L196` — 追踪主循环，包含优化器设置、渲染、损失计算和位姿更新：

```python
def tracking(self, cur_frame_idx, viewpoint):
    prev = self.cameras[cur_frame_idx - self.use_every_n_frames]
    viewpoint.update_RT(prev.R, prev.T)
    pose_optimizer = torch.optim.Adam([...])  # rot_delta, trans_delta, exposure_a, exposure_b
    for tracking_itr in range(self.tracking_itr_num):
        render_pkg = render(viewpoint, self.gaussians, self.pipeline_params, self.background)
        image, depth, opacity = render_pkg["render"], render_pkg["depth"], render_pkg["opacity"]
        loss_tracking = get_loss_tracking(self.config, image, depth, opacity, viewpoint)
        loss_tracking.backward()
        with torch.no_grad():
            pose_optimizer.step()
            converged = update_pose(viewpoint)
        if converged: break
    self.median_depth = get_median_depth(depth, opacity)
    return render_pkg
```

`raw/codes/MonoGS/utils/pose_utils.py:L27-L40` — SO(3) 指数映射（罗德里格斯公式）：

```python
def SO3_exp(theta):
    W = skew_sym_mat(theta); W2 = W @ W
    angle = torch.norm(theta)
    if angle < 1e-5: return I + W + 0.5 * W2
    else: return I + (torch.sin(angle)/angle)*W + ((1-torch.cos(angle))/(angle**2))*W2
```

### 实现注意事项

- **SE(3) 扰动顺序**：MonoGS 使用 **右乘** 扰动（`exp(τ) @ T_WC`），意味着增量在 world 坐标系下作用于相机位姿。如果是左乘模型，Jacobian 朝向需要调整。
- **渲染梯度**：微分栅格化对相机位姿的梯度通过 `cam_rot_delta` / `cam_trans_delta` 的 `requires_grad` 传播。确保 CUDA 扩展已正确编译且支持位姿梯度通道。
- **曝光参数**：渲染图像值域通常与 Gaussian 颜色参数的范围一致（约为 [0, 1]），曝光参数 a, b 初始化为 0 以保持恒等映射。如果离线渲染和在线观测的曝光不匹配，曝光估计会向错误方向修正，影响跟踪精度。
- **深度 mask 条件**：RGB-D 模式只在 opacity > 0.95 的像素做深度监督，这确保仅对"充分渲染的表面点"计算深度误差，避免半透明或未收敛区域干扰优化。
- **单目初始化**：单目模式第一帧使用随机深度分布初始化 Gaussian，后续通过多视角联合优化恢复尺度。如果前几帧之间存在纯旋转，初始化将失败。
- **关键帧阈值**：`kf_translation` 乘以 `median_depth` 决定平移阈值，这是自适应的——近景物体降低阈值，远景场景提高阈值。

### 源码检索锚点

- `FrontEnd.tracking` — `raw/codes/MonoGS/utils/slam_frontend.py:L128-L196`
- `update_pose` / `SE3_exp` — `raw/codes/MonoGS/utils/pose_utils.py:L61-L93`
- `get_loss_tracking` / `get_loss_tracking_rgb` / `get_loss_tracking_rgbd` — `raw/codes/MonoGS/utils/slam_utils.py:L56-L88`
- `FrontEnd.is_keyframe` — `raw/codes/MonoGS/utils/slam_frontend.py:L198-L225`
- `BackEnd.map` (isotropic regularization) — `raw/codes/MonoGS/utils/slam_backend.py:L229`
- `Camera` 类（cam_rot_delta / cam_trans_delta）— `raw/codes/MonoGS/utils/camera_utils.py`
- `get_median_depth` — `raw/codes/MonoGS/utils/slam_utils.py:L131-L142`

## 相关页面

- [[算法-MonoGS]]
- [[方法-3D Gaussian Splatting]]
- [[方法-微分栅格化]]
- [[数学-SE3指数映射]]
- [[概念-直接法视觉里程计]]
- [[概念-可微渲染]]
- [[数学-球谐函数]]
