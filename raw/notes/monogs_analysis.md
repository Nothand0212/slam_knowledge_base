# MonoGS 深度源码分析

> 基于 MonoGS 仓库 (Matsuki et al., Gaussian Splatting SLAM, CVPR 2024) 的完整源码级分析。
> 核心文件: `slam.py` (258行), `slam_frontend.py` (496行), `slam_backend.py` (482行), `gaussian_model.py` (695行), `gaussian_renderer/__init__.py` (153行), `slam_utils.py` (142行), `pose_utils.py` (93行), `dataset.py` (532行)。

---

## 1. 3D Gaussian Splatting (3DGS) 基础

### 1.1 Gaussian Primitives 参数化

3DGS (Kerbl et al., SIGGRAPH 2023) 将场景表示为**各向异性 3D 高斯椭球的集合**。MonoGS 在 `gaussian_model.py:34-68` 中完整实现了这一表示:

```python
# gaussian_model.py:39-48
self._xyz          = torch.empty(0, device="cuda")  # 3D 位置 (N, 3)
self._features_dc  = torch.empty(0, device="cuda")  # 球谐 DC (RGB base, N, 1, 3)
self._features_rest= torch.empty(0, device="cuda")  # 球谐高阶系数 (N, (max_sh+1)²-1, 3)
self._scaling      = torch.empty(0, device="cuda")  # 各向异性尺度 (N, 3) 或各向同性 (N, 1)
self._rotation     = torch.empty(0, device="cuda")  # 四元数旋转 (N, 4)
self._opacity      = torch.empty(0, device="cuda")  # 不透明度 (N, 1)
```

**激活函数** (`gaussian_model.cpp:53-62`):
- `_scaling → exp()`: 保证正定的尺度
- `_opacity → sigmoid()`: 归一化到 [0,1]
- `_rotation → normalize()`: 保证单位四元数

### 1.2 Splatting 渲染

渲染过程在 `gaussian_renderer/__init__.py:24-153` 的 `render()` 函数中实现:

**步骤1 — 栅格化设置** (`renderer:55-72`):
```python
raster_settings = GaussianRasterizationSettings(
    image_height=int(viewpoint_camera.image_height),
    image_width=int(viewpoint_camera.image_width),
    tanfovx=tanfovx, tanfovy=tanfovy,
    bg=bg_color,
    viewmatrix=viewpoint_camera.world_view_transform,
    projmatrix=viewpoint_camera.full_proj_transform,
    sh_degree=pc.active_sh_degree,           # 球谐阶数
    campos=viewpoint_camera.camera_center,
)
```

`full_proj_transform` 在 `camera_utils.py:99-104` 中计算:
```python
@property
def full_proj_transform(self):
    return (self.world_view_transform.unsqueeze(0).bmm(
        self.projection_matrix.unsqueeze(0))).squeeze(0)
```

**步骤2 — 微分栅格化** (`renderer:130-141`):
```python
rendered_image, radii, depth, opacity, n_touched = rasterizer(
    means3D=means3D, means2D=means2D,
    shs=shs, opacities=opacity,
    scales=scales, rotations=rotations,
    theta=viewpoint_camera.cam_rot_delta,   # 用于位姿求导
    rho=viewpoint_camera.cam_trans_delta,   # 用于位姿求导
)
```

**对 SLAM 研究者**: `theta` 和 `rho` 参数是关键 — MonoGS 扩展了原始 3DGS 的微分栅格化器, 支持将**相机位姿增量参数 (`cam_rot_delta`, `cam_trans_delta`) 传入栅格化器**, 使得渲染过程可对相机位姿直接求导。这是 MonoGS 能将 3DGS 用于 SLAM 的核心技术基础。

**步骤3 — 球谐颜色** (`renderer:100-109`):
```python
if pipe.convert_SHs_python:
    shs_view = pc.get_features.transpose(1, 2).view(-1, 3, (pc.max_sh_degree+1)**2)
    dir_pp = pc.get_xyz - viewpoint_camera.camera_center.repeat(...)
    dir_pp_normalized = dir_pp / dir_pp.norm(dim=1, keepdim=True)
    sh2rgb = eval_sh(pc.active_sh_degree, shs_view, dir_pp_normalized)
    colors_precomp = torch.clamp_min(sh2rgb + 0.5, 0.0)
```

球谐函数 (Spherical Harmonics) 用于编码**视角依赖的外观**。MonoGS 默认 `sh_degree=0` (单目模式下 `slam.py:51`): `model_params.sh_degree = 3 if self.use_spherical_harmonics else 0`, 即仅使用 DC 分量 (无视角依赖)。

---

## 2. MonoGS 如何将 3DGS 用于 SLAM

### 2.1 同时优化相机位姿 + 高斯场景

MonoGS 的核心公式:

$$\min_{\{\mathbf{g}_k\}, \{\mathbf{T}_i\}} \sum_{i} \mathcal{L}\big(\text{Render}(\{\mathbf{g}_k\}, \mathbf{T}_i), \mathbf{I}_i^{\text{gt}}\big)$$

其中 $\{\mathbf{g}_k\}$ 是高斯场景参数 (xyz, color, scale, rotation, opacity), $\mathbf{T}_i$ 是相机位姿。MonoGS 通过**交替优化**实现:

**Frontend (Tracking)** — `slam_frontend.py:128-196`:
```python
def tracking(self, cur_frame_idx, viewpoint):
    prev = self.cameras[cur_frame_idx - self.use_every_n_frames]
    viewpoint.update_RT(prev.R, prev.T)  # 恒速模型初始猜测
    
    opt_params = [
        {"params": [viewpoint.cam_rot_delta],  "lr": lr_rot},
        {"params": [viewpoint.cam_trans_delta],"lr": lr_trans},
        {"params": [viewpoint.exposure_a],     "lr": 0.01},
        {"params": [viewpoint.exposure_b],     "lr": 0.01},
    ]
    pose_optimizer = torch.optim.Adam(opt_params)
    for tracking_itr in range(self.tracking_itr_num):  # 默认 ~60-100 次
        render_pkg = render(viewpoint, self.gaussians, ...)
        loss_tracking = get_loss_tracking(config, image, depth, opacity, viewpoint)
        loss_tracking.backward()
        pose_optimizer.step()
        converged = update_pose(viewpoint)
```

**Backend (Mapping)** — `slam_backend.py:142-318`:
```python
def map(self, current_window, prune=False, iters=1):
    for _ in range(iters):
        for cam_idx in range(len(current_window)):
            render_pkg = render(viewpoint, self.gaussians, ...)
            loss_mapping += get_loss_mapping(config, image, depth, viewpoint, opacity)
        
        # isotropic 正则 (防止高斯坍缩为针状)
        scaling = self.gaussians.get_scaling
        isotropic_loss = torch.abs(scaling - scaling.mean(dim=1).view(-1, 1))
        loss_mapping += 10 * isotropic_loss.mean()
        loss_mapping.backward()
        
        # 稠密化 & 剪枝 (3DGS 原始流程)
        self.gaussians.densify_and_prune(...)
        self.gaussians.optimizer.step()        # 场景参数更新
        self.keyframe_optimizers.step()        # 位姿更新 (窗口内关键帧)
```

### 2.2 系统架构: Frontend/Backend 分离

`SLAM` 类 (`slam.py:25-198`) 使用 Python `multiprocessing` 将 Frontend 和 Backend 分离为独立进程:

```python
# slam.py:63-109
frontend_queue = mp.Queue()  # Backend → Frontend 通信
backend_queue  = mp.Queue()  # Frontend → Backend 通信

self.frontend = FrontEnd(self.config)   # Tracking + Keyframe 决策
self.backend  = BackEnd(self.config)    # Mapping + 优化

backend_process = mp.Process(target=self.backend.run)
backend_process.start()
self.frontend.run()  # 主线程运行 Frontend
```

**通信协议** (通过 Queue 发送 list):
- `["init", frame_idx, viewpoint, depth_map]` — Frontend 请求初始化
- `["keyframe", frame_idx, viewpoint, window, depthmap]` — 添加关键帧
- `["map", frame_idx, viewpoint]` — 请求非关键帧优化
- `["sync_backend", gaussians, visibility, keyframes]` — Backend 同步结果到 Frontend
- `["pause"]`, `["unpause"]` — GUI 控制

---

## 3. SLAM 管线 (Tracking + Mapping + Keyframe)

### 3.1 管线总览

```
Frame → [Tracking] → [Keyframe Decision] → [Mapping] → 更新 Gaussian
        优化 6-DOF      基于共视度判断       窗口内联合优化      densify/prune
          位姿                                位姿+场景
```

主循环在 `slam_frontend.py:316-496` `run()`:

```python
def run(self):
    cur_frame_idx = 0
    while True:
        if self.reset:
            self.initialize(cur_frame_idx, viewpoint)       # 首帧初始化
            continue
        
        # 1. Tracking: 优化当前帧位姿
        render_pkg = self.tracking(cur_frame_idx, viewpoint)
        
        # 2. Keyframe Decision: 共视度 + 平移距离判断
        create_kf = self.is_keyframe(cur_frame_idx, last_kf, ...)
        
        if create_kf:
            self.current_window, removed = self.add_to_window(...)
            depth_map = self.add_new_keyframe(cur_frame_idx, ...)
            self.request_keyframe(cur_frame_idx, viewpoint, current_window, depth_map)
```

### 3.2 Tracking 细节

`slam_frontend.cpp:128-196`:

**损失函数** (`slam_utils.py:56-71`):

- **单目模式** (`get_loss_tracking_rgb`, `slam_utils.py:63-71`):
  $$\mathcal{L}_{\text{track}} = \frac{1}{|M|}\sum_{p\in M} o_p \cdot |I_p^{\text{render}} - I_p^{\text{gt}}|$$
  其中 $o_p$ 是 opacity (降低低置信度像素权重), $M$ 是有效 RGB 掩码 (`gt_image.sum > rgb_boundary_threshold`)

- **RGB-D 模式** (`get_loss_tracking_rgbd`, `slam_utils.py:74-88`):
  $$\mathcal{L}_{\text{track}} = \alpha \cdot \mathcal{L}_{\text{rgb}} + (1-\alpha) \cdot \text{mean}(|d_p - d_p^{\text{gt}}|)$$
  深度损失仅在 `opacity > 0.95` 的像素上计算, $\alpha$ 默认 0.95

**曝光补偿** (`slam_utils.py:57`):
```python
image_ab = (torch.exp(viewpoint.exposure_a)) * image + viewpoint.exposure_b
```
每个相机帧都维护自己的曝光参数对 `(exposure_a, exposure_b)`, 建模自动曝光变化。

**位姿更新** (`pose_utils.py:76-93`):
```python
def update_pose(camera, converged_threshold=1e-4):
    tau = torch.cat([camera.cam_trans_delta, camera.cam_rot_delta], axis=0)
    T_w2c = torch.eye(4, device=tau.device)
    T_w2c[0:3, 0:3] = camera.R
    T_w2c[0:3, 3] = camera.T
    new_w2c = SE3_exp(tau) @ T_w2c       # 右乘扰动模型
    new_R, new_T = new_w2c[0:3, 0:3], new_w2c[0:3, 3]
    converged = tau.norm() < converged_threshold
    camera.update_RT(new_R, new_T)
    camera.cam_rot_delta.data.fill_(0)    # 重置增量
    camera.cam_trans_delta.data.fill_(0)
```

使用 **SE(3) 指数映射** (`pose_utils.py:61-73`):
$$\exp(\tau) = \begin{bmatrix} \text{SO3\_exp}(\theta) & V(\theta) \cdot \rho \\ 0 & 1 \end{bmatrix}$$

其中 $V(\theta)$ 是 SE(3) 的指数映射伴随矩阵 (`pose_utils.py:43-58`):
$$V(\theta) = I + \frac{1-\cos\|\theta\|}{\|\theta\|^2} [\theta]_\times + \frac{\|\theta\| - \sin\|\theta\|}{\|\theta\|^3} [\theta]_\times^2$$

### 3.3 Keyframe 决策

`slam_frontend.py:198-225` `is_keyframe()`:

```python
def is_keyframe(self, cur_frame_idx, last_keyframe_idx,
                cur_frame_visibility_filter, occ_aware_visibility):
    # 条件 1: 平移距离 > kf_translation * median_depth
    dist_check = dist > kf_translation * self.median_depth
    dist_check2 = dist > kf_min_translation * self.median_depth
    
    # 条件 2: 共视度 ratio = |交集|/|并集| < kf_overlap
    union = torch.logical_or(cur_visibility, occ_aware_visibility[last_kf]).count_nonzero()
    intersection = torch.logical_and(cur_visibility, occ_aware_visibility[last_kf]).count_nonzero()
    point_ratio_2 = intersection / union
    
    return (point_ratio_2 < kf_overlap and dist_check2) or dist_check
```

关键帧创建策略结合了**平移距离**和**共视度**两个条件: 当视野内容变化足够大 (共视度低) 或移动距离足够远时, 添加新关键帧。

**窗口管理** (`slam_frontend.py:227-286` `add_to_window()`):
- 维护一个固定大小的滑动窗口 (默认 8-10 帧 `window_size`)
- 新帧加入后, 与窗口内帧比较共视度 (Szymkiewicz-Simpson 系数), 移除重叠最低的帧
- 若窗口满, 选择与其他帧距离最大的帧移除 (保持空间分布的均匀性)

### 3.4 Mapping 细节

`slam_backend.cpp:142-318` `map()`:

**损失函数** (`slam_utils.py:91-128`):
- Mapping 使用与 Tracking 相同的 RGB(RGB-D) 残差, 但**不加 opacity 加权**
- 额外加入 **isotropic 正则** (`slam_backend.py:229-231`):
  ```python
  scaling = self.gaussians.get_scaling
  isotropic_loss = torch.abs(scaling - scaling.mean(dim=1).view(-1, 1))
  loss_mapping += 10 * isotropic_loss.mean()
  ```
  这防止 Gaussians 坍缩为极度各向异性的针状 (在 SLAM 中尤为常见, 因为不准确的位姿会驱使 Gaussians 沿视线方向拉长)

**稠密化与剪枝** (`gaussian_model.py:674-689`):
```python
def densify_and_prune(self, max_grad, min_opacity, extent, max_screen_size):
    grads = self.xyz_gradient_accum / self.denom
    self.densify_and_clone(grads, max_grad, extent)  # 小尺度 → clone
    self.densify_and_split(grads, max_grad, extent)  # 大尺度 → split
    
    prune_mask = (self.get_opacity < min_opacity).squeeze()
    if max_screen_size:
        big_points_vs = self.max_radii2D > max_screen_size
        big_points_ws = self.get_scaling.max(dim=1).values > 0.1 * extent
        prune_mask = torch.logical_or(prune_mask, torch.logical_or(big_points_vs, big_points_ws))
    self.prune_points(prune_mask)
```

**共视度剪枝 (SLAM-specific)** (`slam_backend.py:247-275`):
```python
if prune_mode == "slam":
    sorted_window = sorted(current_window, reverse=True)
    mask = self.gaussians.unique_kfIDs >= sorted_window[2]  # 仅对新关键帧引入的 Gaussians
    to_prune = torch.logical_and(self.gaussians.n_obs <= prune_coviz, mask)  # n_obs ≤ 3
    self.gaussians.prune_points(to_prune.cuda())
```

`unique_kfIDs` (`gaussian_model.py:48-49`) 追踪每个 Gaussian 由哪个关键帧引入, 实现**帧级高斯管理**:
```python
self.unique_kfIDs = torch.empty(0).int()
self.n_obs = torch.empty(0).int()  # 被观察到次数
```

---

## 4. 与传统 SLAM (ORB-SLAM3) 的关键区别

| 维度 | ORB-SLAM3 | MonoGS |
|------|-----------|--------|
| **场景表示** | 稀疏 3D 点 + 关键帧 | **密集 3D Gaussians** (百万级参数, `gaussian_model.py:34`) |
| **前端匹配** | ORB 特征提取 + 匹配 → PnP | **基于渲染的 photometric 对齐** (微分栅格化, `renderer:130`) |
| **后端优化** | 稀疏 BA (g2o / gtsam) | 全密集 RGB(-D) 光度损失 (联合优化场景 + 位姿, `slam_backend.py:142`) |
| **地图利用** | 仅用于 tracking (3D-2D) 和 BA | 地图即场景 — 每帧 tracking 用到的就是完整渲染 |
| **初始化** | 两帧几何 + BA (单目) | 单帧点云初始化 (`slam_frontend.cpp:110-126`), 单目用随机深度 |
| **GPU** | 不需要 | **必须** (CUDA 微分渲染 + PyTorch) |
| **内存** | ~MB | **GB 级** (数百万 Gaussian × 59 参数) |
| **重定位** | 有 (词袋 + PnP) | **无** — 丢失需 `reset` |

**最本质的区别**: ORB-SLAM3 的优化目标是最小化**特征重投影误差** (稀疏几何约束), 而 MonoGS 最小化的是**光度误差** (密集外观约束)。前者有成熟的 BA 理论支撑, 后者依赖于**神经网络风格的梯度下降** (Adam optimizer, `gaussian_model.py:283`)。

---

## 5. 计算需求与 GPU 依赖

### 5.1 硬件需求

```python
# slam.py:43: self.live_mode = config["Dataset"]["type"] == "realsense"
# slam.py:59: ... = torch.tensor(..., device="cuda")
# gaussian_model.py:39: self._xyz = torch.empty(0, device="cuda")
# gaussian_renderer/__init__.py:44: requires_grad=True, device="cuda"
```

**整个系统运行在 GPU 上**, CPU 仅负责数据集加载 (`dataset.py:257`) 和进程调度。微分栅格化器 (`diff_gaussian_rasterization`, `renderer:15-18`) 是 CUDA 实现的。

### 5.2 内存分析

每个 Gaussian 的参数 (`gaussian_model.py:39-48`):
- xyz: float32 × 3 = 12B
- features_dc: float32 × 3 = 12B
- features_rest (sh=0): 0B (无高阶 SH)
- scaling: float32 × 3 = 12B
- rotation: float32 × 4 = 16B
- opacity: float32 × 1 = 4B
- **总计: ~56B/Gaussian** + Adam optimizer 状态 (2×momentum) ≈ **224B/Gaussian**

百万 Gaussian → ~224MB (参数) + ~448MB (optimizer) ≈ **0.7-1GB**。

实际上场景通常有 50万-200万 Gaussians (`densify_and_prune()` 动态调节), 加上渲染中间量, 推荐 **≥8GB VRAM**。

### 5.3 实时性

MonoGS 不是实时系统 (论文报告 ~3 FPS, `slam_frontend.py:478`):
```python
if create_kf:
    duration = tic.elapsed_time(toc)
    time.sleep(max(0.01, 1.0 / 3.0 - duration / 1000))  # 限速 3fps
```

瓶颈在 Mapping: `slam_backend.py:431`: 单帧 `iter_per_kf=300` 次渲染迭代, 每次迭代包含微分栅格化 + 反向传播。

---

## 6. 优缺点 + 对SLAM算法的意义

### 6.1 优点

1. **密集光度建图**: 输出**可直接渲染 3D 场景**(Gaussians), 而非稀疏点云 — 这对 AR/VR、视图合成极具价值
2. **无特征依赖**: 不需要特征提取/匹配, 对低纹理、重复纹理场景比特征法鲁棒
3. **微分端到端**: Tracking→Mapping→更新全流程可微 (`renderer:139-140` 的 theta/rho 支持位姿求导)
4. **单目深度 "涌现"**: `slam_frontend.py:65-104`: 单目模式下从随机深度 (2.0 + noise) 开始, 经 BA 优化收敛到合理深度
5. **曝光自适应**: 每帧的 `exposure_a/b` (`camera_utils.py:56-61`) 自动补偿光照变化

### 6.2 缺点

1. **GPU 强制**: CPU-only 不可用 — 整个系统是 CUDA/PyTorch + 定制 CUDA kernel
2. **慢**: Mapping 迭代 300 次/关键帧 (`slam_backend.py:431`), Tracking 迭代 ~100 次/帧 — 比 ORB-SLAM3 慢 1-2 个数量级
3. **无全局 BA / 位姿图**: 只有滑动窗口内的联合 BA, 无 GTSAM/iSAM2 风格的全局优化
4. **回环检测缺失**: 代码中无任何回环检测或闭环机制
5. **初始化敏感**: `slam_frontend.cpp:226`: 若窗口内关键帧重叠不足, 直接 `reset`:
   ```python
   if self.monocular and not self.initialized and removed is not None:
       self.reset = True  # 彻底重启
   ```
6. **浮点问题 — isotropic 正则**: `slam_backend.py:229-231` 的 isotropic 惩罚是硬编码的 `10×`, 且对非初始化阶段仍然施加, 可能过度约束真实场景中的各向异性结构。

### 6.3 对SLAM领域的意义

MonoGS 代表 SLAM 领域的**范式转换**: 从 "优化稀疏几何" 走向 "优化密集外观 + 隐式几何"。

- **地图即场景**: Gaussian 地图不仅是定位的工具, 本身就是输出 (可渲染的 3D 场景) — 这模糊了 SLAM 和 NeRF 的边界
- **梯度驱动的 SLAM**: 用 `optimizer.step()` 替代 `g2o::optimize()` — 这是**学习范式对传统优化范式的侵蚀**
- **2059 stars** 的 stars 表明社区对 "3DGS + SLAM" 组合的高度关注

但 MonoGS 也暴露了 3DGS-SLAM 的**工程瓶颈**: 微分渲染 (splatting) 是昂贵的, Mapping 迭代次数限制了其实时性。未来工作 (如 SplaTAM, GS-SLAM) 在尝试用更高效的优化策略缓解此问题。

---

## 7. 对比: FAST-LIVO2 vs R3LIVE vs LVI-SAM 的技术路线差异

### 整体范式

| 框架 | 传感器 | 场景表示 | 优化策略 | 地图输出 | 速度 |
|------|--------|---------|---------|---------|------|
| **MonoGS** | 单目/RGB-D | 3D Gaussians (密集) | Adam 梯度下降 (光度) | 可渲染 3D 场景 | ~3 FPS |
| **FAST-LIVO2** | LiDAR+RGB+IMU | LiDAR 点 (稀疏) + 视觉特征 | IESKF + VIO 因子 (tight) | 点云地图 | ~10-20 Hz |
| **R3LIVE** | LiDAR+RGB+IMU | 3D 点 + RGB 纹理体素 | ESIKF + 光度 VIO (tight) | 彩色点云地图 | ~10-20 Hz |
| **LVI-SAM** | LiDAR+RGB+IMU | LiDAR 点 + ORB 视觉词袋 | 因子图 (loose coupling) | 点云地图 | ~10-20 Hz |

---

### 详细技术对比

#### MonoGS vs FAST-LIVO2

| 维度 | MonoGS | FAST-LIVO2 |
|------|--------|-----------|
| **传感器** | 仅视觉 (单目/RGB-D) | LiDAR+RGB+IMU 三模态 |
| **初始化** | 单帧 + 随机深度 | LiDAR-IMU 联合 (LiDAR 直接提供尺度) |
| **位姿估计** | 渲染-比较 (photometric align) | IESKF (LiDAR 残差 + VIO 因子) |
| **建图** | 密集 Gaussians (数百万参数) | 稀疏 LiDAR 特征点 |
| **GPU** | 必须 | 不需要 |

**FAST-LIVO2 的优势**: 多传感器冗余 → 更高鲁棒性; IESKF → 更快收敛; LiDAR 直接测距 → 无尺度模糊
**MonoGS 的优势**: 输出可直接渲染 (view synthesis, AR overlay); 无 LiDAR 成本

#### MonoGS vs R3LIVE

R3LIVE 做了 MonoGS "想做但没做" 的事: **RGB 着色重建 + 视觉光度 VIO**。区别在于:
- R3LIVE 的**光度误差**是**体素内的 RGB 亮度差异** (渲染 vs 观测), 场景表示为**体素网格**
- MonoGS 的光度误差是**Gaussian splatting 渲染 vs 观测**, 场景表示为**3D Gaussians**

两者本质上都在优化光度一致性, 但 MonoGS 的输出质量远高于 R3LIVE (Gaussian 是各向异性、视角依赖的表示, 体素网格是各向同性的)。

#### MonoGS vs LVI-SAM

LVI-SAM 是最 "传统" 的: 它将 **LIO-SAM (LiDAR+IMU factor graph)** 和 **VINS-Mono (视觉 VIO)** 松散耦合 — 两个子系统独立运行, 通过因子图**仅交换位姿约束**。

MonoGS 则彻底不同: 它在 single-pass 中**联合优化场景表示和位姿** — 没有独立的 LiDAR 子系统, 也没有特征匹配。

---

### 技术路线总结

```
                        稀疏几何 ←————————————————→ 密集外观
                              |
        LVI-SAM (最传统)      |     R3LIVE (混合)
        因子图 loose coupling  |     光度 VIO + 体素
        独立LiDAR+视觉子系统   |     RGB 重建
                              |
             FAST-LIVO2 (最鲁棒)  MonoGS (最激进)
             tight VIO+IESKF    Gaussian Splatting SLAM
             三模态互补          端到端梯度下降
                              |
        需要LiDAR ←—————————|——————————→ 仅需相机
```

- **LVI-SAM**: 组合两个成熟系统 (LIO-SAM + VINS-Mono), 工程最优, 但本质是**松耦合**
- **FAST-LIVO2**: tight coupling 先驱, IESKF 统一 LiDAR+视觉, **鲁棒性最强**
- **R3LIVE**: 加入 RGB 着色, 体素级光度匹配, **可视化最优**
- **MonoGS**: 抛弃传统 SLAM 的所有假设 (特征、稀疏性、LiDAR), **新范式开拓者**