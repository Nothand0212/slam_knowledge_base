---
tags: [R3LIVE, LiDAR-IMU-视觉, ESIKF, 光电融合, 双ESIKF, 辐射图]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/r3live
---

# LiDAR-IMU-视觉融合 (R3LIVE)

> R3LIVE 通过双 ESIKF 架构分别进行 LiDAR-惯性里程计（LIO）和视觉-惯性里程计（VIO）在线估计，同时构建全局 RGB 着色的辐射图，实现 LiDAR、IMU、视觉三模态紧耦合。

## 系统架构

R3LIVE 将系统分解为两个子系统，各自运行一个独立的 ESIKF：

- **LIO 子系统**：基于 FAST-LIO2 的 IESKF + iKD-Tree，利用 LiDAR 点到面残差更新位姿、速度、IMU bias、重力矢量；
- **VIO 子系统**：基于最小化稀疏视觉特征点的光度/重投影误差，进一步在线更新状态并估计 IMU-相机外参、相机内参和时间延迟；

两个 ESIKF 共享全局状态 `g_lio_state`，通过两个独立线程协调更新。全局辐射图 `m_map_rgb_pts` 在每次 LIO 更新后被新点云扩展，并为 VIO 提供 3D 地图点用于视觉跟踪。

```
┌─────────┐  LIO thread    ┌──────────────┐
│  LiDAR  │───▶ ESIKF-LIO ──▶ g_lio_state │
│   IMU   │    (iKD-Tree)    └──────┬───────┘
└─────────┘                        │ 共享状态
                      ┌────────────▼───────┐
┌─────────┐  VIO thread│  g_lio_state +    │
│  Camera │───▶ ESIKF-VIO ──▶  辐射图渲染  │
└─────────┘            └────────────────────┘
```

## 双ESIKF 融合机制

### 状态定义

LIO 和 VIO 共享同样的 29 维名义状态：

$$
\mathbf{x} = [^G\mathbf{R}_I^T, {}^G\mathbf{p}_I, {}^G\mathbf{v}_I, \mathbf{b}_g, \mathbf{b}_a, {}^G\mathbf{g}, {}^C\mathbf{R}_I, {}^C\mathbf{p}_I, \tau_d, \mathbf{K}, {}^C\tau_d] \in \mathbb{R}^{29}
$$

其中前 18 维是与 FAST-LIO2 一致的 IMU-位姿状态，后 11 维是 VIO 特有的外参、时偏、内参状态。LIO 只估计前 18 维，VIO 可估计全部 29 维。

### LIO 子系统的 ESIKF 更新

LIO 更新循环在 `service_LIO_update()` 中运行。对于每帧 LiDAR 点云：

1. IMU 预积分传播名义状态，得到 `state_propagate`；
2. 对每个降采样后的点，用 kd-tree 搜索最近 5 个地图点，通过 PCA 拟合平面；
3. 计算点到面的距离作为观测残差；
4. 构造雅可比矩阵并执行 IESKF 迭代；

### VIO 子系统的 ESIKF 更新

VIO 的迭代在 `vio_esikf()` 中实现。每次 VIO 帧到达时：

1. `vio_preintegration()` 用 IMU 队列传播 LIO 最新状态到当前图像时间戳；
2. 利用辐射图的 3D 点投影到当前图像，建立透视点对应；
3. 最小化重投影误差：

$$
\mathbf{r}_i(\mathbf{x}) = \mathbf{p}_{\text{proj},i}(\mathbf{x}) - \mathbf{p}_{\text{meas},i}
$$

投影函数涉及 IMU 位姿、IMU-相机外参和相机内参的完整链式变换。

## 辐射图渲染与光度更新

R3LIVE 的全局辐射图 `Global_map` 以体素哈希表存储 RGB 点。每个点存储其 3D 坐标、颜色向量和 `m_img_vel`（图像平面速度，用于补偿卷帘快门效应）。

点云着色流程：
1. LIO 每帧输出的世界坐标点云通过 `RGBpointBodyToWorld()` 转换；
2. 点被追加到 `m_map_rgb_pts` 全局辐射图中；
3. VIO 线程从辐射图中取出投影范围内的 3D 点，用于光流跟踪和 ESIKF 更新；

`vio_photometric()` 提供备选的光度误差 ESIKF 更新——直接最小化辐射图点投影 patch 与当前帧像素值的差异，而非使用稀疏特征点的重投影。

## 线程模型与数据流

`raw/codes/r3live/r3live/src/r3live.hpp:L450` 和 `r3live.cpp:L92-L101` 显示系统同时运行三个主线程：

1. **LIO 线程** `service_LIO_update()`：高频运行（~5000 Hz polling），处理 LiDAR 扫描和 IMU 数据；
2. **VIO 线程** `service_VIO_update()`：按图像帧率运行，做视觉跟踪和 ESIKF；
3. **渲染线程** `service_pub_rgb_maps()`：发布 RGB 点云到 ROS 可视化；

关键同步通过 `m_mutex_lio_process` 互斥锁保护共享状态 `g_lio_state` 的读写（`raw/codes/r3live/r3live/src/r3live_vio.cpp:L1161`）。

## Agent 实现提示

### 适用场景

当系统同时配备 LiDAR、IMU 和 RGB 相机，需要在实时性要求高的场景（无人机、手持设备）进行三模态融合定位与 RGB 建图时，使用双 ESIKF 架构。LIO 提供高频率鲁棒位姿，VIO 利用全局辐射图提供精化并在线标定传感器外参/内参。

### 输入输出契约

- **输入**：
  - LiDAR 点云（`/laser_cloud_flat`，`PointCloud2`，每点含 intensity 和法向）；
  - IMU 数据（`/livox/imu`，200 Hz+，含加速度和角速度）；
  - RGB 图像（`/camera/image_color`，含内参/畸变系数和外参）；
- **输出**：
  - 全局位姿 `g_lio_state`（`SO(3) × R^15`，29 维）；
  - 注册后的 RGB 着色点云 `/cloud_registered`；
  - 视觉跟踪点云 `/track_pts`；
  - 辐射图渲染点云 `/render_pts`；
- **坐标系**：LiDAR body frame → IMU frame（外参 `Lidar_offset_to_IMU`）→ world frame（估计位姿），IMU → camera（外参 `rot_ext_i2c, pos_ext_i2c`）；

### 实现骨架（伪代码）

```pseudo
function r3live_main():
    g_lio_state = init_state()
    ikdtree = build_iKDTree(first_scan_points)
    global_rgb_map = init_voxel_hash_map(resolution=0.05)
    spawn_thread(service_LIO_update)
    spawn_thread(service_VIO_update)
    spawn_thread(service_pub_rgb_maps)

function service_LIO_update():
    while ros_ok:
        sync_lidar_imu_package()
        state_propagate = imu_preintegrate(g_lio_state, imu_buf)
        feats_undistort = deskew_lidar(scan, imu_buf)
        for iter in 0..max_iter:
            for pt in feats_undistort:
                plane = pca_fit_plane(ikdtree.knn(pt, k=5))
                H[i] = [skew(pt_body) * R^T * n, n]
                z[i] = -dist_to_plane(pt, plane)
            K = inv(H^T*R_inv*H + inv(cov))
            solution = K * H^T * R_inv * z
            g_lio_state += solution
            if converged: break
        ikdtree.add_points(feats_down)
        global_rgb_map.append_from_lidar(scan_xyz, timestamps)

function service_VIO_update():
    while ros_ok:
        image_frame = next_image_with_pose()
        state_out = vio_preintegrate(g_lio_state, image_frame.time)
        tracked_pts = optical_flow_track(image_frame, prev_frame)
        for iter in 0..esikf_iters:
            for each tracked_rgb_point:
                pt_3d_cam = R_w2c * pt_world + t_w2c
                pt_proj = project(pt_3d_cam, fx, fy, cx, cy)
                reproj_err = pt_proj - pt_meas
                H = compute_reprojection_jacobian(pt_world, state)
                meas_vec = reproj_err
            solution = esikf_solve(H, meas_vec, state_in.cov)
            state_iter += solution
        state_iter.cov = (I - KH) * state_iter.cov
        g_lio_state = state_iter
```

### 关键源码片段

`raw/codes/r3live/r3live/src/r3live_lio.cpp:L661-L900` — LIO ESIKF 迭代主循环，包含 kd-tree 近邻搜索、PCA 平面拟合、点到面残差与雅可比构造、及协方差更新：

```cpp
for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
    // PCA平面拟合
    cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);
    float pa = matX0.at<float>(0,0), pb = matX0.at<float>(1,0);
    float pc = matX0.at<float>(2,0), pd = 1;
    float ps = sqrt(pa*pa + pb*pb + pc*pc);
    pa/=ps; pb/=ps; pc/=ps; pd/=ps;
    // ... planar check ...
    float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y
              + pc * pointSel_tmpt.z + pd;
    // 雅可比
    Eigen::Vector3d A(point_crossmat * g_lio_state.rot_end.transpose() * norm_vec);
    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
    meas_vec(i) = -norm_p.intensity;
    // IESKF 求解
    auto &&Hsub_T = Hsub.transpose();
    H_T_H.block<6,6>(0,0) = Hsub_T * Hsub;
    K_1 = (H_T_H + (g_lio_state.cov/LASER_POINT_COV).inverse()).inverse();
    K = K_1.block<DIM_OF_STATES,6>(0,0) * Hsub_T;
    auto vec = state_propagate - g_lio_state;
    solution = K * (meas_vec - Hsub * vec.block<6,1>(0,0));
    g_lio_state = state_propagate + solution;
}
```

`raw/codes/r3live/r3live/src/r3live_vio.cpp:L609-L772` — VIO ESIKF 的完整迭代，含相机投影雅可比和外参/内参状态块：

```cpp
for (int iter_count = 0; iter_count < esikf_iter_times; iter_count++) {
    mat_3_3 R_imu = state_iter.rot_end;
    vec_3 t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
    mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c;
    for (auto it : tracked_pts) {
        pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
        pt_img_proj = vec_2(fx*pt_3d_cam(0)/pt_3d_cam(2)+cx, ...);
        mat_pre << fx/pt_3d_cam(2), 0, -fx*pt_3d_cam(0)/(pt_3d_cam(2));
        mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
        mat_B = -state_iter.rot_ext_i2c.transpose() * (R_imu.transpose());
        H_mat.block(pt_idx*2, 0, 2, 3) = mat_pre * mat_A * huber_loss_scale;
        H_mat.block(pt_idx*2, 3, 2, 3) = mat_pre * mat_B * huber_loss_scale;
    }
    temp_inv_mat = ((H_T_H_spa.toDense() + (state_in.cov*weight).inverse()).inverse()).sparseView();
    solution = (temp_inv_mat*(...*(meas_vec)...) -(I-KH_spa)*vec_spa).toDense();
    state_iter = state_iter + solution;
}
```

`raw/codes/r3live/r3live/src/r3live_vio.cpp:L1088-L1165` — `service_VIO_update()` 主循环：图像取帧 → 等待LIO数据就绪 → IMU预积分传播 → 视觉跟踪 → ESIKF更新：

```cpp
void R3LIVE::service_VIO_update() {
    while (ros::ok()) {
        std::shared_ptr<Image_frame> img_pose = m_queue_image_with_pose.front();
        m_queue_image_with_pose.pop_front();
        while (g_camera_lidar_queue.if_camera_can_process() == false) {
            ros::spinOnce(); yield(); // 等待LIO新状态
        }
        m_mutex_lio_process.lock();
        vio_preintegration(g_lio_state, state_out, img_pose->m_timestamp);
        set_image_pose(img_pose, state_out);
        op_track.track_img(img_pose, -20);     // 光流跟踪
        vio_esikf(state_out, op_track);         // ESIKF更新
        g_lio_state = state_out;
        m_mutex_lio_process.unlock();
    }
}
```

### 实现注意事项

- **双滤波器耦合**：两个 ESIKF 不是完全独立的——VIO 的 `state_in` 是最近一次 LIO 更新后的状态，通过 `m_mutex_lio_process` 保护。如果 LIO 长期不更新，VIO 的 IMU 预积分会累积漂移；
- **测量权重自适应**：`m_cam_measurement_weight` 在 `raw/codes/r3live/r3live/src/r3live_vio.cpp:L1165` 随新访问体素数量动态调整，避免辐射图稀疏时视觉信息污染 LIO 估计；
- **外参/内参在线估计**：`m_if_estimate_i2c_extrinsic` 和 `m_if_estimate_intrinsic` 控制是否同时优化 IMU-相机外参和相机内参，这些参数直接参与投影雅可比链；
- **时间延迟估计**：`td_ext_i2c` 和 `td_ext_i2c_delta` 分别存储当前时间延迟和增量，在每轮迭代后累加（`raw/codes/r3live/r3live/src/r3live_vio.cpp:L769`）；
- **辐射图投影约束**：`m_tracker_minimum_depth`/`m_tracker_maximum_depth` 限制投影深度范围，避免深度异常点破坏视觉更新；
- **LiDAR 点更新步长**：`m_lio_update_point_step` 控制 LIO 每帧实际使用的点比例，以平衡精度和实时性；

### 源码检索锚点

- `raw/codes/r3live/r3live/src/r3live.cpp:L92-L101` — 系统入口，实例化和启动
- `raw/codes/r3live/r3live/src/r3live_lio.cpp:L493-L900` — `service_LIO_update` 与 LIO ESIKF 主循环
- `raw/codes/r3live/r3live/src/r3live_vio.cpp:L548-L589` — `vio_preintegration` IMU 传播
- `raw/codes/r3live/r3live/src/r3live_vio.cpp:L609-L772` — `vio_esikf` 视觉 ESIKF
- `raw/codes/r3live/r3live/src/r3live_vio.cpp:L775-L799+` — `vio_photometric` 光度 ESIKF
- `raw/codes/r3live/r3live/src/r3live_vio.cpp:L1087-L1170` — `service_VIO_update` 主循环
- `raw/codes/r3live/r3live/src/rgb_map/rgbmap_tracker.cpp:L60-L100` — 视觉特征跟踪
- `raw/codes/r3live/r3live/src/r3live.hpp:L127-L450` — 类定义和成员变量
- `raw/codes/r3live/r3live/src/r3live.hpp:L284-L302` — VIO 子系统函数声明
- `raw/codes/r3live/r3live/src/rgb_map/pointcloud_rgbd.hpp` — RGB 点数据结构

## 相关页面

- [[算法-R3LIVE]]
- [[方法-IESKF滤波器]]
- [[架构-双ESIKF架构]]
- [[方法-统一IESKF融合]]
- [[算法-FAST-LIO]]
- [[方法-IMU deskew]]
- [[方法-在线平面拟合]]
- [[方法-RGB着色点云]]
