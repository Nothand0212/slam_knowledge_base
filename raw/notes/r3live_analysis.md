# R3LIVE 深度源码分析

> 论文: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package
> 开发者: Jiarong Lin, Prof. Fu Zhang (HKU Mars Lab)
> GitHub: 2390 stars


## 1. 数据接收与预处理 (Livox LiDAR + 全局快门相机 + IMU)

### 1.1 多线程架构

R3LIVE 的核心是多线程设计 (`r3live.hpp`, `r3live.cpp:92-101`)。Main函数创建 `R3LIVE*` 实例后，初始化启动4个并发线程:

1. **主线程**: ros::spin() 处理传感器回调
2. **LIO线程**: `service_LIO_update()` → LiDAR-Inertial里程计
3. **VIO线程**: `service_VIO_update()` → Visual-Inertial里程计
4. **RGB发布线程**: `service_pub_rgb_maps()` → RGB地图发布

关键同步机制 (`r3live_lio.cpp:536-541`):
```cpp
while (g_camera_lidar_queue.if_lidar_can_process() == false) {
    ros::spinOnce();
    std::this_thread::yield();
    std::this_thread::sleep_for(chrono::milliseconds(THREAD_SLEEP_TIM));
}
```

### 1.2 传感器回调

**IMU回调** (`r3live_lio.cpp:50-78`):
- 时间同步: `g_camera_lidar_queue.imu_in(timestamp)` 检查IMU数据合法性
- **同时入两个队列**: `imu_buffer_lio` (LIO用) 和 `imu_buffer_vio` (VIO用)，两个线程独立消费
- 支持加速度乘以G的配置 (`m_if_acc_mul_G`)

**LiDAR回调** (`r3live_lio.cpp:462-482`, `feat_points_cbk()`):
- `g_camera_lidar_queue.lidar_in(timestamp+0.1)` 注册LiDAR帧进入队列
- 存入 `lidar_buffer`（共享的PointCloud2 deque）

**图像回调** (`r3live_vio.cpp:324-359`):
- 支持原始图像 (`sensor_msgs::Image`) 和压缩图像 (`CompressedImage`)
- 首次接收时启动 `service_process_img_buffer` 线程异步处理
- `process_image()`: resize → remap (去畸变) → 直方图均衡化 → 存入 `m_queue_image_with_pose`

### 1.3 LiDAR数据同步 (`r3live_lio.cpp:151-197`)

`sync_packages()`: 取最老的 LiDAR 帧，将所有时间戳在其结束前的IMU数据打包到 `MeasureGroup`。

### 1.4 图像预处理 (`r3live_vio.cpp:364-431`)

1. 降采样 resize → `m_vio_image_width / m_vio_scale_factor`
2. `cv::remap()` 去畸变（预计算 `m_ud_map1`, `m_ud_map2`）
3. `init_cubic_interpolation()` + `image_equalize()` 直方图均衡


## 2. 前端

### 2.1 LiDAR前端: 点到平面 + iKd-Tree (`r3live_lio.cpp:661-898`)

R3LIVE的LiDAR前端基于Fast-LIO2:
- **特征提取**: 在外部节点完成，R3LIVE消费 `feats_down`（下采样后的特征点）
- **最近邻搜索**: `ikdtree.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf)` (r3live_lio.cpp:688)
- **PCA平面拟合**: OpenCV的 `cv::solve(matA0, matB0, matX0, cv::DECOMP_QR)` 计算平面参数 (r3live_lio.cpp:716)
- **平面验证**: 检查5个最近点是否都在平面内 (`fabs(pa·x + pb·y + pc·z + pd) > m_planar_check_dis` ⇒ invalid)
- **测量: 有符号距离**: `pd2 = pa*pt.x + pb*pt.y + pc*pt.z + pd`，存入 `coeff.intensity`

### 2.2 视觉前端: LK光流 + RGB点跟踪 (`r3live_vio.cpp:1088-1180`)

VIO使用 `Rgbmap_tracker op_track` 系统:

**跟踪流程**:
1. 获取当前帧前 `m_queue_image_with_pose.front()`
2. `op_track.track_img(img_pose, -20)` 跟踪RGB地图点到当前图像
   - LK光流 (`lkpyramid.cpp`) 用于帧间跟踪
   - 全局地图点的重投影匹配
3. 跟踪结果存入 `op_track.m_map_rgb_pts_in_current_frame_pos`
   - Key: `RGB_pts*` (全局地图点指针)
   - Value: `cv::Point2f` (像素坐标)

**视觉特征**: 不是传统角点，而是全局RGB地图点（已着色的3D点）。

### 2.3 LiDAR-视觉协同

R3LIVE 通过 RGB-coloured pointcloud 实现 LiDAR-视觉协同:
- LiDAR点在 `service_LIO_update()` 完成后通过 `m_map_rgb_pts.append_points_to_global_map()` (r3live_lio.cpp:977-989) 添加到全局RGB地图
- 这些带有RGB颜色的LiDAR点后续被VIO系统跟踪

**FOV分割** (`r3live_lio.cpp:260-460`): LiDAR map分为前方FOV内的特征和FOV外，使用 `lasermap_fov_segment()` 管理地图可见性。


## 3. 初始化

### 3.1 LIO初始化 (`r3live_lio.cpp:527-528`)

启动时:
- `set_initial_state_cov(g_lio_state)` 设置初始协方差
- 使用 `ImuProcess` 中的IMU初始化（与Fast-LIO2相同）: 估计重力方向和bias
- 累积足够的IMU数据后开始LiDAR配准

### 3.2 VIO初始化 (`r3live_vio.cpp:385-397, 1134-1143`)

首次图像到达时:
1. 加载相机内参和外参 (`set_initial_camera_parameter()`)
2. 初始化畸变校正映射 (`initUndistortRectifyMap()`)
3. 启动 `service_VIO_update` 线程和 `service_pub_rgb_maps` 线程
4. 首帧需要等待足够的地图点: `while (m_map_rgb_pts.m_rgb_pts_vec.size() <= 100)` spin等待

### 3.3 初始协方差设置 (`r3live_vio.cpp:157-173`)

VIO状态扩展了LIO状态，包含相机-IMU外参和内参:
```cpp
state.cov.block(0,0,3,3) = I * 1e-5;     // R
state.cov.block(3,3,3,3) = I * 1e-5;     // T
state.cov.block(18,18,6,6) = I * 1e-3;   // 相机-IMU外参
state.cov.block(25,25,4,4) = I * 1e-3;   // 相机内参 (fx,fy,cx,cy)
```


## 4. 逐帧估计

### 4.1 双估计器架构

```
                    ┌─ IMU数据 ────┬──► LIO IESKF ──► LiDAR位姿
                    │               │
IMU+LiDAR+Camera ───┤               └──► VIO IESKF ──► 视觉增强位姿
                    │                       ▲
                    └─ RGB地图点 ───────────┘
                               (LIO位姿作为先验)
```

### 4.2 LIO IESKF (`r3live_lio.cpp:493-1081`)

**状态向量** (Fast-LIO2 继承):
```
x = [R(3), p(3), v(3), b_g(3), b_a(3), g(3)]  // 18维
```

**IESKF迭代更新** (`r3live_lio.cpp:836-893`):
```
for iterCount = 0..NUM_MAX_ITERATIONS:
    // 1. 最近邻搜索 + PCA平面拟合
    for each feature point:
        ikdtree.Nearest_Search() → points_near
        cv::solve(5点平面拟合) → [pa,pb,pc,pd]

    // 2. 构建测量雅可比 H (laserCloudSelNum × 6)
    Hsub.row(i) = [point_crossmat * R^T * n_vec, n_vec]

    // 3. IESKF更新
    H_T_H = Hsub^T * Hsub
    K_1 = (H_T_H + (P / LASER_POINT_COV)^{-1})^{-1}
    K = K_1 * Hsub^T
    vec = x_propagate - x
    solution = K * (meas_vec - Hsub * vec)     // 显式增益形式
    x = x_propagate + solution

    // 协方差更新
    G = K * Hsub
    P = (I - G) * P
```

**IMU传播** 由 `p_imu->Process()` 执行（前向+后向去畸变，与Fast-LIO2相同）。

### 4.3 VIO IESKF (`r3live_vio.cpp:607-772`)

**扩展状态向量**: 在LIO的18维基础上增加:
- `pos_ext_i2c(3)`: IMU→相机平移
- `rot_ext_i2c(3)`: IMU→相机旋转
- `td_ext_i2c(1)`: IMU-相机时间偏移
- `cam_intrinsic(4)`: fx, fy, cx, cy
- `td_ext_i2c_delta(1)`: 时间偏移增量
→ **总维度 = 18+6+1+4+1 = 29+** (`DIM_OF_STATES`)

**ESIKF迭代** (`r3live_vio.cpp:654-762`):
```
for iter_count = 0..esikf_iter_times:
    for each tracked RGB point:
        p_3d_w = rgb_pt->get_pos()
        p_img_measure = tracked pixel position
        p_3d_cam = R_w2c * p_3d_w + t_w2c
        p_img_proj = project(p_3d_cam) + time_td * img_vel

        repro_err = |p_img_proj - p_img_measure|

        mat_pre = [fx/z, 0, -fx*x/z²; 0, fy/z, -fy*y/z²]
        mat_A = R_ext_i2c^T * skew(R_imu^T*(p_3d_w - t_imu))
        mat_B = -R_ext_i2c^T * R_imu^T
        mat_C = skew(p_3d_cam)            // 对外参旋转的雅可比
        mat_D = -R_ext_i2c^T              // 对外参平移的雅可比

        H_mat.block[0:2, 0:3] = mat_pre * mat_A  // ∂proj/∂δθ
        H_mat.block[0:2, 3:3] = mat_pre * mat_B  // ∂proj/∂δp
        H_mat.block[0:2, 24:1] = img_vel          // ∂proj/∂td (如果估计)
        H_mat.block[0:2, 18:3] = mat_pre * mat_C  // ∂proj/∂R_ext (如果估计)
        H_mat.block[0:2, 21:3] = mat_pre * mat_D  // ∂proj/∂t_ext (如果估计)
        H_mat[pt_idx*2, 25] = x/z                  // ∂proj/∂fx
        H_mat[pt_idx*2+1, 26] = y/z                // ∂proj/∂fy
        H_mat[pt_idx*2, 27] = 1                    // ∂proj/∂cx
        H_mat[pt_idx*2+1, 28] = 1                  // ∂proj/∂cy

    // Huber loss scale
    huber_loss_scale = get_huber_loss_scale(repro_err)

    // 稀疏ESIKF求解
    vec = (state_iter - state_in).sparseView()
    H_T_H_spa = H^T * H
    temp_inv = (H_T_H + (P * m_cam_measurement_weight)^{-1})^{-1}
    solution = temp_inv*(H^T*(-meas_vec)) - (I-KH)*vec
    state_iter += solution

    // 协方差更新
    cov = (I - KH) * cov
```

### 4.4 可选光度误差 (`r3live_vio.cpp:774-957`)

`vio_photometric()` 使用3通道RGB光度误差（而非2D重投影误差）。`mat_photometric(3×2) = [dR/dx, dR/dy; dG/dx, dG/dy; dB/dx, dB/dy]` 构建3通道图像梯度。

### 4.5 LIO→VIO先验传递 (`r3live_vio.cpp:549-589`)

`vio_preintegration()`:
1. 从 `state_in` (LIO最后的状态) 开始
2. 使用 `m_imu_process->imu_preintegration()` 传播到当前图像时间
3. 输出 `state_out` 作为VIO的初始猜测和先验

**关键**: `g_lio_state` 是全局共享状态的拷贝，LIO更新它，VIO读取+更新它（通过 `m_mutex_lio_process` 互斥锁保护）。


## 5. 关键创新点

### 5.1 RGB着色 + 实时3D重建

**RGB点云着色流程** (`r3live_vio.cpp:959-1037`):
1. LIO去畸变后的点云通过 `RGBpointBodyToWorld()` 转换到世界系
2. `m_map_rgb_pts.append_points_to_global_map()` 将点加入全局RGB地图
3. 每个 `RGB_pts` 维护多视图RGB观测 (`m_N_rgb` 计数)，通过 `get_rgb_cov()` 估计RGB协方差
4. `service_pub_rgb_maps()` 定期发布RGB着色点云（分多个ROS topic避免瓶颈）

**Mesh重建** (`r3live_reconstruct_mesh.cpp`, `MVS/`):
- 使用 `m_mvs_recorder` 对象
- `export_to_mvs()` 输出 MVS (Multi-View Stereo) 格式用于离线网格重建

### 5.2 双ESIKF架构

与FAST-LIVO2的单一IESKF不同，R3LIVE使用**两个独立的ESIKF**:
- LIO ESIKF: 高效点到平面配准
- VIO ESIKF: 视觉重投影约束 + 可选光度约束
- 两者通过 `g_lio_state` 共享状态，以LIO估计作为VIO先验

**优势**: 模块化，LIO和VIO可独立调试优化
**劣势**: 信息可能重复或丢失，不如单一KF的协方差一致

### 5.3 LIO-as-Prior 设计

VIO不从头估计位姿，而是:
1. 从LIO估计的状态开始
2. IMU预积分到图像时间戳
3. 使用 `(state_iter - state_in)` 构建先验约束

这使得VIO在视觉跟踪失败时能优雅降级回LIO。

### 5.4 在线标定 (Online Calibration)

VIO可以同时在线估计:
- 相机内参 (`m_if_estimate_intrinsic`)
- IMU-相机外参 (`m_if_estimate_i2c_extrinsic`)
- 时间偏移 (`td_ext_i2c`)

这在 `vio_esikf()` 中通过扩展状态维度实现。

### 5.5 离线建图模式 (`offline_map_recorder.cpp`)

支持离线精细化建图，可以重新处理已记录的RGB地图进行高质量纹理重建。


## 6. 优缺点 + 意义

### 6.1 优势

1. **RGB着色实时输出**: 业界首创的高质量实时RGB着色LiDAR点云
2. **双线程架构**: LIO和VIO并行运行，不互相阻塞
3. **在线标定**: 支持内参/外参/时间偏移的在线估计
4. **模块化**: LIO基于Fast-LIO2，VIO独立模块，易于升级

### 6.2 局限性

1. **双ESIKF信息冗余**: 两个KF维护各自的协方差，不如单一KF一致
2. **依赖LIO质量**: VIO严重依赖LIO先验，LIO退化时VIO也受影响
3. **无回环**: 纯里程计
4. **内存开销大**: RGB点云 + MVS重建数据结构
5. **仅限于Livox LiDAR**: 不像FAST-LIVO2支持多种LiDAR

### 6.3 意义

1. **首个实用RGB着色LiDAR SLAM**: 视觉和LiDAR数据深度融合到着色输出
2. **验证了双ESIKF可行性**: 为后续的FAST-LIVO2等统一方案提供了对比基线


## 7. 三大框架技术路线差异

| 维度 | FAST-LIVO2 | R3LIVE | LVI-SAM |
|------|-----------|--------|---------|
| KF架构 | 单一IESKF | 双ESIKF | 无KF (因子图) |
| 视觉方法 | 直接法(光度) | 重投影+LK光流 | 特征点+KLT |
| LiDAR地图 | VoxelMap八叉树 | iKd-Tree增量kd树 | 体素+FLANN kd树 |
| RGB着色 | 简单投影 | 全局多视图优化 | 无 |
| 3D重建 | 无 | MVS Mesh | 无 |
| 并行性 | 串行LIO→VIO | 并行LIO+VIO线程 | 多ROS节点 |
| 在线标定 | 曝光 | 外参+内参+时间偏移 | 无 |
| 回环 | 无 | 无 | DBoW+ICP |

**核心差异**: R3LIVE关注视觉质量（RGB着色+重建），FAST-LIVO2关注里程计精度（统一IESKF），LVI-SAM关注系统工程（松耦合+回环）。
