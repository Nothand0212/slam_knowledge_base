---
tags: [FAST-LIVO2, 直接法, LiDAR-视觉融合, IESKF, 光度对齐, 点云着色]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/FAST-LIVO2
---

# 直接法LiDAR-视觉融合 (FAST-LIVO2)

> FAST-LIVO2 在 FAST-LIO2 的 LiDAR-惯性里程计基础之上，增加直接法视觉惯性子系统，两者共享统一的 19 维 IESKF 状态和协方差，实现 LiDAR、视觉和 IMU 三类传感器在同一滤波器内的串行紧耦合。

## 系统架构

FAST-LIVO2 的核心设计是**统一的 19 维 IESKF**：LiDAR 模块和视觉模块操作同一份状态和协方差，LIO 更新后的后验状态直接被 VIO 作为先验线性化点继收缩。这与 R3LIVE 的分离式双 ESIKF 形成对比。

```
IMU 高频传播 ──► state_propagat (IMU 先验)
                       │
         ┌─────────────▼──────────────┐
         │  统一 19 维 IESKF 状态      │
         │  {R, p, v, b_g, b_a, g, τ}│
         └─────┬──────────┬───────────┘
               │          │
     ┌─────────▼──┐  ┌───▼──────────┐
     │ LIO 更新    │  │ VIO 更新      │
     │ 点到面残差  │  │ 光度patch残差 │
     │ 体素地图    │  │ 视觉稀疏地图  │
     └────────────┘  └──────────────┘
               │          │
         ┌─────▼──────────▼───────┐
         │   协方差自动传播        │
         │   I - KH 标准公式      │
         └────────────────────────┘
```

## 与 FAST-LIO2 的核心差异

FAST-LIO2 仅包含 LiDAR-惯性估计，状态为 18 维。FAST-LIVO2 扩展为 19 维，新增了**曝光时间倒数** `τ = 1/expo_time` 状态，用于在线估计相机自动曝光（AE）变化对光度测量的一致性影响。

关键区别总结：

| 特性 | FAST-LIO2 | FAST-LIVO2 |
|------|-----------|------------|
| 状态维度 | 18 (含重力) | 19 (新增曝光) |
| 传感器 | LiDAR + IMU | LiDAR + IMU + 单目相机 |
| 观测残差 | 点到面距离 | 点到面 + 光度 patch 差值 |
| 地图表示 | 体素八叉树 | 体素八叉树 + 视觉稀疏点 |
| VIO 融合方式 | 无 | 统一 IESKF，LIO→VIO 串行 |

## 直接法视觉前端

### Patch 光度残差

FAST-LIVO2 的 VIO 使用围绕稀疏视觉特征的多层金字塔 patch 的光度误差作为观测。对于每个在体素地图中有对应平面的视觉点 `p_w`，其参考帧像素坐标 `u_ref` 与当前帧像素 `u_cur` 之间的光度残差为：

$$
r_k = I_{\text{ref}}(u_{\text{ref}}) - I_{\text{cur}}(u_{\text{cur}}, \tau)
$$

其中 `τ` 为曝光时间倒数，用于校正两帧之间的全局曝光变化：
$$
I_{\text{cur}}(u, \tau) = \tau \cdot I_{\text{cur}}^{\text{raw}}(u)
$$

### 仿射 Warp 与多级金字塔

对于每个视觉点，通过当前帧与参考帧之间的相对位姿变换计算仿射 Warp 矩阵 `A_cur_ref`，将参考帧 patch 映射到当前帧视角。系统支持两种 Warp 策略：

- **仿射 Warp**（`getWarpMatrixAffine`）：适用于深度连续区域，基于平面诱导的单应性；
- **单应 Warp**（`getWarpMatrixAffineHomography`）：当体素地图提供了法向信息时，利用平面单应性 `H = R + t * n^T/d` 提高 Warp 精度；

两种策略存在代码中 `normal_en` 开关控制（`raw/codes/FAST-LIVO2/include/vio.h:L100`）。

## 点云着色与视觉稀疏地图

### LiDAR 点云投影着色

系统利用已知的 LiDAR-相机外参，将 LiDAR 体素地图中的每个平面点投影到图像平面，采样颜色赋给对应 LiDAR 点。这个过程在 `retrieveFromVisualSparseMap` 和 `projectPatchFromRefToCur` 中完成：

1. 遍历体素地图的活跃体素，提取其中的平面点；
2. 利用相机模型 `cam->world2cam()` 将世界坐标点投影到像素；
3. 在图像上提取多层金字塔 patch 作为视觉描述子；
4. 对每个成功匹配的视觉点，计算 patch 光度误差和 NCC 相似度；

### VisualPoint 数据结构

`raw/codes/FAST-LIVO2/include/visual_point.h:L23-L46` 定义的 `VisualPoint` 类：

```cpp
class VisualPoint {
    Vector3d pos_;        // 3D 位置 (world frame)
    Vector3d normal_;     // 表面法向 (来自体素平面)
    list<Feature*> obs_;  // 多帧观测 (Feature 链表)
    Feature *ref_patch;   // 当前参考 patch
    bool is_converged_;   // 收敛标志
};
```

每个 `Feature` 包含图像 patch 像素值、投影坐标、相机位姿和曝光时间倒数，用于后续帧的光度对齐。

### VIO 处理流水线

`raw/codes/FAST-LIVO2/src/vio.cpp:L1786-L1876` — `processFrame` 函数展示完整 VIO 处理顺序：

```
1. resetGrid()                     // 清空网格状态
2. retrieveFromVisualSparseMap()  // 从体素地图检索视觉点
3. computeJacobianAndUpdateEKF()  // 光度误差雅可比 + IESKF
4. generateVisualMapPoints()      // 在当前帧生成新视觉点
5. updateVisualMapPoints()        // 给已有视觉点追加观测
6. updateReferencePatch()         // 更新参考patch (如进入平面)
```

## LIO-VIO 串行链式更新

FAST-LIVO2 的核心融合逻辑在 `stateEstimationAndMapping`（`raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L267-L278`）：

```cpp
void LIVMapper::stateEstimationAndMapping() {
    switch (LidarMeasures.lio_vio_flg) {
        case VIO: handleVIO(); break;
        case LIO:
        case LO:  handleLIO(); break;
    }
}
```

每次 IMU 传播后（`processImu`），根据事件标志串行调用对应处理器。关键点在于 VIO 处理器持有的 `state` 和 `state_propagat` 是指向 `LIVMapper` 成员 `_state` 和 `state_propagat` 的指针（`raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L135-L136`），因此 LIO 和 VIO 更新的是**同一份状态**。

VIO IESKF 更新中的先验传递公式与 LIO 完全一致（`raw/codes/FAST-LIVO2/src/vio.cpp:L1497-L1501`）：

```cpp
auto vec = (*state_propagat) - (*state);
solution = -K_1 * HTz + vec - G * vec.block<6,1>(0,0);
(*state) += solution;
```

这确保了 IMU 先验信息（编码在 `state_propagat` 中）在视觉更新中也得到保留。

## Agent 实现提示

### 适用场景

当系统需要 LiDAR + IMU + 单目相机的统一融合，并且希望 LiDAR 和视觉模块共享同一协方差矩阵以保持统计一致性时，使用统一 IESKF 架构。典型应用为手持/无人机实时直接法 LiDAR-视觉-惯性 SLAM。

### 输入输出契约

- **输入**：
  - LiDAR 点云（`pcl::PointXYZINormal`，经 IMU deskew）；
  - IMU 数据（加速度 + 角速度，含重力对齐）；
  - 单目图像（灰度/彩色，含标定的内参和 LiDAR-相机外参）；
  - LIO/VIO 事件标志（`lio_vio_flg`），由传感器调度器决定；
  - 体素八叉树地图（`unordered_map<VOXEL_LOCATION, VoxelOctoTree*>`）；
- **输出**：
  - 统一 19 维状态 `_state`（`rot_end, pos_end, vel_end, bias_g, bias_a, gravity, inv_expo_time`）；
  - 视觉稀疏地图 `feat_map`（VOXEL_POINTS 体素索引的 VisualPoint 集合）；
  - 着色点云 `/cloud_visual_sub_map_before`；
- **坐标系**：LiDAR body → IMU（外参 `extR, extT`）→ world（状态估计），LiDAR → camera（外参 `cameraextrinR, cameraextrinT`）；

### 实现骨架（伪代码）

```pseudo
function stateEstimationAndMapping():
    processImu()  // IMU 传播: _state → state_propagat
    if lio_vio_flg == LIO:
        // 体素地图点到面 IESKF
        voxelmap.state = _state
        voxelmap.StateEstimation(state_propagat)
        _state = voxelmap.state
    else if lio_vio_flg == VIO:
        // 直接法视觉 IESKF
        vio_manager.processFrame(image, pv_list, voxel_map, img_time)

function VIOManager.processFrame(img, pg, plane_map, img_time):
    new_frame = Frame(cam, img)
    resetGrid()
    retrieveFromVisualSparseMap(img, pg, plane_map)
        // 1. 遍历体素地图平面
        // 2. 投影到当前帧，取patch，计算光度误差
        // 3. 筛选内点（NCC阈值、outlier_threshold）
        // 4. 构建visual_submap候选集
    for level in pyramid..0:
        computeJacobianAndUpdateEKF(img, level)
            for each visual_point:
                for each patch_pixel:
                    patch_error = I_ref_subpix - I_cur_pix
                    J_dR = dL/de * de/dR, J_dt = dL/de * de/dt
                    H_sub.row[6] = [J_dR, J_dt]
            // IESKF 求解: K_1 = inv(HTH + inv(cov/img_point_cov))
            // solution = -K_1*HTz + vec_propagat - G*vec
            (*state) += solution
    generateVisualMapPoints(img, pg)  // 生成新的视觉地图点
    updateVisualMapPoints(img)        // 已有点的观测追加
    updateReferencePatch(plane_map)   // 进入平面时更新法向和参考patch
```

### 关键源码片段

`raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L128-L147` — VIO 管理器初始化，包括外参设置、状态指针绑定和参数配置：

```cpp
vio_manager->setImuToLidarExtrinsic(extT, extR);
vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
vio_manager->state = &_state;              // 共享状态指针
vio_manager->state_propagat = &state_propagat;
vio_manager->max_iterations = max_iterations;
vio_manager->img_point_cov = IMG_POINT_COV;
vio_manager->normal_en = normal_en;
vio_manager->inverse_composition_en = inverse_composition_en;
vio_manager->exposure_estimate_en = exposure_estimate_en;
vio_manager->initializeVIO();
```

`raw/codes/FAST-LIVO2/src/vio.cpp:L1488-L1506` — VIO IESKF 核心更新，与 LIO IESKF 结构一致，残差形式为光度误差：

```cpp
if (error <= last_error) {
    old_state = (*state);
    last_error = error;
    H_T_H.block<6,6>(0,0) = H_sub_T * H_sub;
    MD(DIM_STATE,DIM_STATE) &&K_1 =
        (H_T_H + (state->cov/img_point_cov).inverse()).inverse();
    auto &&HTz = H_sub_T * z;
    auto vec = (*state_propagat) - (*state);    // IMU先验
    G.block<DIM_STATE,6>(0,0) =
        K_1.block<DIM_STATE,6>(0,0) * H_T_H.block<6,6>(0,0);
    auto solution = -K_1.block<DIM_STATE,6>(0,0) * HTz
                  + vec - G.block<DIM_STATE,6>(0,0) * vec.block<6,1>(0,0);
    (*state) += solution;
    if ((rot_add.norm()*57.3f<0.001f) && (t_add.norm()*100.0f<0.001f))
        EKF_end = true;
}
```

`raw/codes/FAST-LIVO2/src/vio.cpp:L700-L770` — `retrieveFromVisualSparseMap` 中的单应 Warp + 光度误差计算 + 外点剔除：

```cpp
SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();
getWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec,
                              T_cur_ref, 0, A_cur_ref_zero);
search_level = getBestSearchLevel(A_cur_ref_zero, 2);
for (int pyramid_level = 0; pyramid_level <= patch_pyrimid_level-1; pyramid_level++)
    warpAffine(A_cur_ref_zero, ref_ftr->img_, ref_ftr->px_, ...);
getImagePatch(img, pc, patch_buffer.data(), 0);
// 光度残差
float error = 0.0;
for (int ind = 0; ind < patch_size_total; ind++)
    error += (ref_ftr->inv_expo_time_ * patch_wrap[ind]
            - state->inv_expo_time * patch_buffer[ind])^2;
if (error > outlier_threshold * patch_size_total) continue;
visual_submap->voxel_points.push_back(pt);
visual_submap->warp_patch.push_back(patch_wrap);
visual_submap->inv_expo_list.push_back(ref_ftr->inv_expo_time_);
```

`raw/codes/FAST-LIVO2/src/voxel_map.cpp:L407-L477` — 体素地图 LIO IESKF 的 H 矩阵构造和测量向量组装（点到面）：

```cpp
V3D point_world = state_propagat.rot_end * point_this
                + state_propagat.pos_end;
J_nq.block<1,3>(0,0) = point_world - ptpl.center_;
J_nq.block<1,3>(0,3) = -ptpl.normal_;
V3D A(point_crossmat * state_.rot_end.transpose() * ptpl.normal_);
Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl.normal_[0], ...;
meas_vec(i) = -ptpl.dis_to_plane_;
```

### 实现注意事项

- **统一状态指针赋值**：`vio_manager->state = &_state` 必须确保在 `initializeVIO` 之前完成，否则 VIO 的相机标定会使用无效状态；
- **曝光估计的正确性**：`inv_expo_time` 参与光度残差计算（`raw/codes/FAST-LIVO2/src/vio.cpp:L749`），如果 AE 变化剧烈且不启用 `exposure_estimate_en`，光度残差会被错误放大；
- **体素平面法向依赖**：`updateReferencePatch` 仅在体素地图提供了平面法向时才更新视觉点的 `normal_` 并设置 `is_normal_initialized_`，影响后续单应 Warp 的质量；
- **NCC 外点过滤**：启用 `ncc_en` 时会额外计算 NCC 相似度（`raw/codes/FAST-LIVO2/src/vio.cpp:L333-L350`），过滤光度不变假设失效的区域；
- **逆合成 vs 正合成**：`inverse_composition_en` 控制使用逆合成 `updateStateInverse`（参考帧梯度预计算）还是正合成 `updateState`（每轮重算），前者更快但假设更严；
- **多级金字塔收敛**：IESKF 从粗到细逐级迭代（`for level in pyramid..0`），每层用前层结果作为初始状态，增强大运动下的收敛性；

### 源码检索锚点

- `raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L128-L147` — VIO 初始化与共享状态绑定
- `raw/codes/FAST-LIVO2/src/LIVMapper.cpp:L248-L278` — `processImu` 与状态分派
- `raw/codes/FAST-LIVO2/src/voxel_map.cpp:L407-L477` — 体素地图 LIO IESKF
- `raw/codes/FAST-LIVO2/src/vio.cpp:L352-L782` — `retrieveFromVisualSparseMap` 视觉点检索
- `raw/codes/FAST-LIVO2/src/vio.cpp:L784-L1517` — 雅可比与 EKF 更新（`updateState`/`updateStateInverse`）
- `raw/codes/FAST-LIVO2/src/vio.cpp:L908-L967` — `updateVisualMapPoints` 视觉点观测追加
- `raw/codes/FAST-LIVO2/src/vio.cpp:L969-L1060` — `updateReferencePatch` 参考 patch 更新
- `raw/codes/FAST-LIVO2/src/vio.cpp:L1786-L1876` — `processFrame` 处理流水线
- `raw/codes/FAST-LIVO2/include/vio.h:L83-L80` — VIOManager 类定义
- `raw/codes/FAST-LIVO2/include/visual_point.h:L22-L46` — VisualPoint 数据结构

## 相关页面

- [[算法-FAST-LIVO2]]
- [[方法-统一IESKF融合]]
- [[方法-IESKF滤波器]]
- [[算法-FAST-LIO]]
- [[架构-双ESIKF架构]]
- [[方法-RGB着色点云]]
- [[算法-R3LIVE]]
- [[方法-体素地图]]
