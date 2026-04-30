# FAST-LIVO2 深度源码分析

> 论文: Fast, Direct LiDAR-Inertial-Visual Odometry v2
> 开发者: Chunran Zheng, Prof. Fu Zhang (HKU Mars Lab)
> GitHub: 3984 stars (截至2026)


## 1. 数据接收与预处理 (LiDAR + RGB + IMU)

### 1.1 传感器架构概述

FAST-LIVO2 支持三种运行模式（`common_lib.h:49-53`）：`SLAM_MODE` 枚举定义 `ONLY_LO=0`（纯LiDAR里程计，无IMU，恒定速度模型）、`ONLY_LIO=1`（LiDAR-Inertial里程计，无视觉）、`LIVO=2`（LiDAR-Inertial-Visual全融合）。

核心是 `LIVMapper` 类（`LIVMapper.h:24`），通过ROS三个回调接收各传感器数据：

**LiDAR回调** (`LIVMapper.cpp:703-767`): 支持两种驱动格式。`livox_pcl_cbk()` 处理 Livox 的 `CustomMsg`，`standard_pcl_cbk()` 处理标准 `PointCloud2`（Velodyne/Ouster/Hesai/Robosense）。原始点云通过 `Preprocess::process()` 提取特征点（边缘/平面），存入 `lid_raw_data_buffer` 和 `lid_header_time_buffer`。

**IMU回调** (`LIVMapper.cpp:769-820`): 接收 `sensor_msgs::Imu`，应用 `imu_time_offset` 时间校正，存入 `imu_buffer`。同时备份到 `prop_imu_buffer` 用于独立的IMU传播线程。

**图像回调** (`LIVMapper.cpp:829-882`): BGR8转换，`img_time_offset` 同步校正，存入 `img_buffer` 和 `img_time_buffer`。针对 Hilti2022 数据集做 4:1 降频（40Hz→10Hz）。

### 1.2 时间同步与数据对齐 (`LIVMapper.cpp:884-1119`)

`sync_packages()` 是最关键的同步函数。LIVO模式采用 **状态机交替更新**（`common_lib.h:54-60`, `EKF_STATE: WAIT=0, VIO=1, LIO=2, LO=3`）:

1. **WAIT → LIO**: 以图像时间 `img_capture_time = img_time_buffer.front() + exposure_time_init` 为界切割LiDAR扫描。点云按时间戳分为 `pcl_proc_cur`（图像时间前）和 `pcl_proc_next`（图像时间后）。IMU也以该时刻同步。

关键代码 (`LIVMapper.cpp:946-1042`):
```
case WAIT / VIO: img_capture_time 作切割轴
  → 遍历 lid_raw_data_buffer
  → curvature < (m.lio_time - frame_header_time)*1000 → pcl_proc_cur
  → curvature >= ... → pcl_proc_next

case LIO: img_buffer.pop → measures.push_back(m) → lio_vio_flg=VIO
```

2. **LIO → VIO**: LIO更新后立即使用同一图像进行VIO。两者**串行执行**在同一线程中，共享 `_state`。

### 1.3 点云特征提取 (`preprocess.h:24-34`)

LiDAR特征分类枚举: `LiDARFeature { Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint }`

支持7种LiDAR (`LID_TYPE: AVIA=1, VELO16=2, OUST64=3, L515=4, XT32=5, PANDAR128=6, ROBOSENSE=7`)。

特征提取 `give_feature()` 使用 `plane_judge()` 和 `edge_jump_judge()` 判断平面和边缘跳跃点。原理类似LOAM: 计算相邻激光点角度变化，大角度跳变标记为边缘点。

### 1.4 点云去畸变 (`IMU_Processing.cpp:237-541`)

`UndistortPcl()` 使用 **后向传播**（backward propagation）:

**前向传播** (`IMU_Processing.cpp:324-431`): 在每个IMU测量处做状态预测，记录各时刻的位姿到 `IMUpose` 向量。

**后向传播** (`IMU_Processing.cpp:494-539`):
```
R_i = R_imu * Exp(angvel, dt)
T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt^2 - pos_end
P_compensate = extR_Ri*(R_i*(R_lidar*P + T_lidar) + T_ei) - extR_extT
```
代码位于 `IMU_Processing.cpp:519-526`。核心思想: 将每个LiDAR点从自身时间戳补偿到扫描结束帧，消除运动畸变。

### 1.5 图像预处理 (`vio.cpp:1786-1876`)

图像接收后: resize（若尺寸不匹配）→ 彩色转灰度 → 创建 `Frame` 对象。保留了原始RGB图像(`img_rgb`)和灰度图像(`img`)，用于后续RGB着色。

---


## 2. 前端 (LiDAR特征 + 视觉特征协同)

### 2.1 视觉前端: 直接法光度误差 (Direct Photometric)

FAST-LIVO2 不提取FAST角点，采用**直接法**（稀疏图像块光度对齐），继承自 SVO/Semi-Direct VO 的思想。

**视觉稀疏地图结构** (`vio.h:26-57`):
```cpp
struct SubSparseMap {
  vector<float> propa_errors;        // IMU传播后的光度误差
  vector<float> errors;              // 优化后的光度误差
  vector<vector<float>> warp_patch;  // 从参考帧Warp的patch (仿射变换)
  vector<int> search_levels;         // 最佳搜索层级
  vector<VisualPoint*> voxel_points; // 视觉地图点
  vector<double> inv_expo_list;      // 各参考帧曝光倒数
  vector<pointWithVar> add_from_voxel_map; // 来自LiDAR体素地图的面心点
};
```

**视觉Feature** (`feature.h:19-54`): `Feature` 结构存储了像素坐标 `px_`、单位球面坐标 `f_`、金字塔层级 `level_`、关联的3D点 `point_`、图像patch `patch_`、参考帧位姿 `T_f_w_`、曝光倒数 `inv_expo_time_`。

### 2.2 LiDAR前端: 点到VoxelMap平面残差

`VoxelMapManager::BuildResidualListOMP()` (`voxel_map.cpp:643-711`) 对每个下采样后的LiDAR特征点:
1. 通过 `VOXEL_LOCATION` 哈希找到体素 → `VoxelOctoTree` 八叉树
2. 递归搜索 `find_correspond()` 获取叶节点平面 → `VoxelPlane`
3. 计算点到平面距离: `dis_to_plane = n(0)*p_w(0) + n(1)*p_w(1) + n(2)*p_w(2) + d`
4. 计算平面协方差: `sigma_l = J_nq * plane.plane_var_ * J_nq^T + n^T * point_var * n`

OpenMP并行加速 (`voxel_map.cpp:660`: `#pragma omp parallel for`)。

验证条件 (`voxel_map.cpp:730-737`):
- 点距圆心 `≤ 3 * plane.radius_`（径向约束）
- 点到平面距离 `< sigma_num * sqrt(sigma_l)`（3σ 统计检验）

### 2.3 LiDAR-视觉数据互馈的前端协同

**三个协同通道**:

**A. LiDAR深度图辅助视觉深度连续性验证** (`vio.cpp:371-428`):
LiDAR点投影到图像平面构建 `depth_img`（`cv::Mat depth_img = zeros(height, width, CV_32FC1)`）。视觉点重投影后检查周围 `patch_size` 范围内LiDAR深度是否连续（`vio.cpp:619-641`），差异 >0.5m 则丢弃，防止错误遮挡匹配。

**B. LiDAR平面更新视觉点法向量** (`vio.cpp:969-1099`, `updateReferencePatch()`):
视觉点查找 VoxelMap 对应平面 → 用法向量更新 `pt->normal_` → 判断收敛 (`normal_update < 0.0001 && obs_.size() > 10` ⇒ `is_converged_=true`)。收敛点只保留参考patch，释放其他Feature的内存。

**C. LiDAR点创建视觉地图点** (`vio.cpp:804-906`, `generateVisualMapPoints()`):
对每个图像网格计算 Shi-Tomasi 分数最高的LiDAR点 → 创建 `VisualPoint + Feature` → `insertPointIntoVoxelMap()` 插入全局视觉稀疏地图。

---


## 3. 初始化

### 3.1 IMU初始化 (`IMU_Processing.cpp:104-149`)

`IMU_init()` 使用 `MAX_INI_COUNT`（默认3帧）的IMU数据:
1. 计算加速度和角速度的在线均值（Welford's online mean）
2. 估计重力方向: `gravity = -mean_acc / |mean_acc| * G_m_s2`
3. 初始旋转为单位阵, bias为零
4. 保存平均加速度范数 `IMU_mean_acc_norm` 用于后续加速度归一化（消去传感器比例因子）

### 3.2 重力对齐 (`LIVMapper.cpp:230-246`)

IMU初始化完成后调用:
```cpp
V3D ez(0, 0, -1), gz(_state.gravity);
Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
M3D G_R_I0 = G_q_I0.toRotationMatrix();
_state.pos_end = G_R_I0 * _state.pos_end;
_state.rot_end = G_R_I0 * _state.rot_end;
_state.vel_end = G_R_I0 * _state.vel_end;
_state.gravity = G_R_I0 * _state.gravity;
```
将估计的重力方向旋转到世界系(0,0,-1)方向。

### 3.3 LiDAR地图初始化 (`voxel_map.cpp:532-591`)

`BuildVoxelMap()` 仅在第一帧调用:
1. 世界坐标点按体素大小离散化
2. 每体素创建 `VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold)`
3. `init_octo_tree()`: `temp_points_` > `points_size_threshold_` 时 → PCA 特征值分解 → `evalsReal(evalsMin) < planer_threshold_` (默认0.01) 判定为平面 → 计算法向量和平面方程
4. 若非平面 → 递归 `cut_octo_tree()` 分割为8个子体素

### 3.4 视觉初始化 (`vio.cpp:41-160`)

`VIOManager::initializeVIO()` 在系统启动时执行:
- 加载相机模型、内外参
- 设置图像网格参数 (`grid_n_width`, `grid_n_height`)
- 计算旋转链雅可比 `Jdphi_dR = Rci`, `Jdp_dR = -Rci * skew(Pic)`
- 可选 Ray Casting 模块预计算每个网格的采样射线

---


## 4. 逐帧估计 (核心数学细节)

### 4.1 状态向量定义 (`common_lib.h:31, 126-223`)

```
DIM_STATE = 19 = 3 + 3 + 1 + 3 + 3 + 3 + 3
```

| 分量 | 维度 | 代码字段 | 说明 |
|------|------|----------|------|
| δθ | 3 | rot_end | SO(3) 误差旋转 (Lie代数) |
| δp | 3 | pos_end | 世界系位置误差 |
| δτ | 1 | inv_expo_time | 曝光时间倒数误差 |
| δv | 3 | vel_end | 速度误差 |
| δb_g | 3 | bias_g | 陀螺仪bias误差 |
| δb_a | 3 | bias_a | 加速度计bias误差 |
| δg | 3 | gravity | 重力加速度误差 |

状态更新使用右乘模型 (`common_lib.h:182-192`):
```cpp
StatesGroup& operator+=(const Matrix<double,DIM_STATE,1>& state_add) {
    this->rot_end = this->rot_end * Exp(state_add(0), state_add(1), state_add(2));
    this->pos_end += state_add.block<3,1>(3,0);
    this->inv_expo_time += state_add(6,0);
    this->vel_end += state_add.block<3,1>(7,0);
    this->bias_g += state_add.block<3,1>(10,0);
    this->bias_a += state_add.block<3,1>(13,0);
    this->gravity += state_add.block<3,1>(16,0);
}
```

`StatesGroup` 减法 (`common_lib.h:194-206`) 使用 `Log(rotd)` 提取旋转误差的 Lie 代数表示，用于计算 `vec = x_propagate - x_current`。

### 4.2 IMU状态传播 (`IMU_Processing.cpp:324-431`)

**旋转传播** (Rodrigues公式, `so3_math.h:24-42`):
```
R_{k+1} = R_k * Exp(angvel_avr - b_g, dt)
```
代码 `IMU_Processing.cpp:412`: `R_imu = R_imu * Exp_f;`

**位置和速度传播**:
```
a_biasless = a_avr * G_m_s2 / |g_mean| - b_a    // 去偏差、归一化
a_global = R * a_biasless + g                    // 转换到世界系
p_{k+1} = p_k + v_k * dt + 0.5 * a_global * dt^2
v_{k+1} = v_k + a_global * dt
```
代码 `IMU_Processing.cpp:353, 415-421`。

**协方差传播** (`IMU_Processing.cpp:382-401`):
```
F_x = I_{19} +
  [ 0      0   0   0   -I·dt  0      0     ]  // row 0-2: ∂ΔR/∂bg
  [ 0      0   0   I·dt 0      0      0     ]  // row 3-5: ∂Δp/∂v
  [ -R·a^·dt 0  0   0   0      -R·dt  I·dt ]  // row 7-9: ∂Δv/∂R,ba,g
  // bg, ba, g = 0 (constant motion model)

cov_w = diag( cov_gyr·dt^2, 0, cov_inv_expo·dt^2,
              R·cov_acc·R^T·dt^2, cov_bias_gyr·dt^2, cov_bias_acc·dt^2, 0 )

P_{k+1} = F_x · P_k · F_x^T + cov_w
```

### 4.3 LiDAR测量雅可比 (`voxel_map.cpp:414-455`)

LiDAR残差 `r_L = n^T · (R·(R_ext·p_b + t_ext) + t) + d`（有符号点到平面距离）。

雅可比推导 (对状态误差 `δθ, δp`):
```
∂r_L/∂δθ = n^T · R · skew(p_body_imu) · R_ext^T
∂r_L/∂δp = n^T   (仅法向量分量)
```
代码:
```cpp
V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0],
               ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
```
其中 `point_crossmat = skew(R_ext·p_b + t_ext)`。

测量噪声: `R_inv(i) = 1.0 / (0.001 + sigma_l + n^T·var·n)`，`var = R·R_ext·body_cov·R_ext^T·R^T`。

### 4.4 VIO测量雅可比 (`vio.cpp:1520-1687`)

光度残差 `r_ij = τ_cur·I_cur(π(T_cw·p_w)) − τ_ref·I_warped_j`。

**级联链式求导** (`vio.cpp:1614-1617`):
```
∂r/∂δθ = J_img * ∂π/∂p_c * [Jφ_dR + Jp_dR]
∂r/∂δp = J_img * ∂π/∂p_c * [Jp_dt]
∂r/∂τ = I_cur(p)    // 曝光雅可比
```
其中:
- `J_img = [du, dv]` (中心差分图像梯度, `vio.cpp:1600-1610`)
- `∂π/∂p_c = [fx/z, 0, -fx*x/z²; 0, fy/z, -fy*y/z²]` (`computeProjectionJacobian()`, `vio.cpp:190-201`)
- `Jφ_dR = R_ci` (SO(3)链式 `vio.cpp:63`)
- `Jp_dR = -R_ci·skew(P_ic)` (平移耦合 `vio.cpp:65`)
- `Jp_dt = R_ci·R_wi^T = Jdp_dt`

### 4.5 IESKF更新方程

**LiDAR IESKF** (`voxel_map.cpp:461-498`):

```
for iteration = 0..max_iterations:
    BuildResidualListOMP() → ptpl_list_ (点到平面残差列表)

    for each residual i in ptpl_list_:
        Hsub.row(i) = [∂r/∂δθ(3), ∂r/∂δp(3)]   // 1×6
        meas_vec(i) = -dis_to_plane

    H_T_H = Hsub^T * R_inv * Hsub                         // 6×6
    K_1 = (H_T_H + P^{-1})^{-1}                           // 19×19
    HTz = Hsub^T * R_inv * meas_vec                       // 6×1
    vec = x_propagate - x_current                         // 19×1 (先验约束)
    G = K_1 * H_T_H                                       // 19×19

    solution = K_1*HTz + vec - G*vec                      // 完整IESKF解

    x += solution  (Lie代数加法)
    收敛: |δθ|*57.3 < 0.01° && |δp|*100 < 0.015cm

协方差更新: P = (I - G) * P
```

**VIO IESKF** (`vio.cpp:1520-1687`):

```
for level = patch_pyrimid_level-1 downto 0:    // 粗到精
    for iteration = 0..max_iterations:
        for each visual point i:
            for each pixel in patch:
                z = τ_cur*I_cur - τ_ref*I_warped
                H_sub.row = [∂r/∂δθ(3), ∂r/∂δp(3), ∂r/∂δτ(1)]

        H_T_H = H_sub^T * H_sub                         // (6 or 7)×(6 or 7)
        K_1 = (H_T_H + (P / img_point_cov)^{-1})^{-1}   // 19×19
        HTz = H_sub^T * z
        vec = x_propagate - x_current
        G = K_1 * H_T_H
        solution = -K_1*HTz + vec - G*vec               // 负号因为极小化

        x += solution

        if error > last_error: state = old_state; break  // 回退
        收敛: |δθ|*57.3 < 0.001° && |δp|*100 < 0.001cm

协方差更新: P = P - G * P
```

### 4.6 IESKF中LiDAR-VIO融合的关键洞察

FAST-LIVO2 不分别做LIO和VIO然后平均。融合机制是:

1. **串行链式**: LIO更新 `_state` → VIO以更新后的 `_state` 作为起点，用 `state_propagat`（IMU传播的最新状态）作为先验
2. **同状态空间**: VIO的雅可比是相对于完整19维状态的，不是只约束视觉6/7-DOF子空间
3. **先验传递**: `vec = x_propagate - state` 携带了 IMU传播信息 → 先验正则化项 `(vec - G*vec)`
4. **协方差连贯**: LIO更新后的 `_state.cov` 被VIO继承，VIO进一步收缩它

这意味着: 即使VIO不直接观测bias和gravity，它们也被视觉残差通过协方差矩阵间接约束。

### 4.7 帧到地图 (Frame-to-Map) 策略

**LiDAR**: 严格 Frame-to-Map。每帧点在 VoxelMap 中找对应平面，VoxelMap 通过 `UpdateVoxelMap()` 增量更新。地图大小由 `mapSliding()` 限定为局部窗口。

**视觉**: Frame-to-Reference-Map。`retrieveFromVisualSparseMap()` 将历史视觉3D点重投影到当前帧，选择光度误差最小的参考patch。可选 Ray Casting 处理遮挡（沿射线采样检查前方是否有遮挡物平面）。

---


## 5. 关键创新点

### 5.1 统一IESKF: Tight Coupling 的核心

与 R3LIVE (双ESIKF) 和 LVI-SAM (GTSAM+Ceres 分离) 的本质区别:

```
FAST-LIVO2 架构:
  单一IESKF (DIM_STATE=19)
    ├── IMU Forward Propagation (预测)
    ├── LIO Update: VoxelMap 点-平面残差 → IESKF
    └── VIO Update: 直接法光度误差 → IESKF (共享同一协方差)
```

优势: 协方差矩阵自动关联 LiDAR 和视觉不确定性。不需要外部约束来对齐两者的估计。

### 5.2 VoxelMap: 新型LiDAR-平面关联结构

数据结构 (`voxel_map.h:33-183`):
- 哈希表 `VOXEL_LOCATION → VoxelOctoTree` 的顶层索引
- 八叉树递归切割，叶节点存储 `VoxelPlane`（中心、法向量、6×6协方差、半径）
- 平面协方差由点的不确定性传播得出: `plane_var_ = Σ J(6×3) * point_var(3×3) * J^T`

与iKd-Tree对比:
- VoxelMap: 固定网格+自适应八叉树，O(logN)，内存效率更高
- iKd-Tree (R3LIVE): 增量kd树，支持动态删除插入，适合大规模动态场景

### 5.3 直接法视觉+曝光在线估计

传统VIO用特征点（VINS-Mono），FAST-LIVO2用直接法:
- 光度残差: `τ_cur * I_cur - τ_ref * I_warped_ref`
- `inv_expo_time` 作为状态的一部分在IESKF中在线估计
- 适应相机自动曝光变化

### 5.4 视觉点的生命周期管理

完整的视觉地图点生命周期 (`visual_point.h:23-47`, `visual_point.cpp`):
1. **创建**: LiDAR点 + Shi-Tomasi 角点评分 → VisualPoint
2. **追踪**: 重投影 + 光度误差 + NCC验证
3. **法向量更新**: 由VoxelMap平面更新
4. **收敛**: 法向量稳定 && obs ≥ 10 → is_converged_
5. **内存**: obs ≥ 30 → 删除最低score的Feature

### 5.5 v2 vs v1 的核心改进

| 特性 | FAST-LIVO (v1) | FAST-LIVO2 (v2) |
|------|---------------|-----------------|
| 视觉方法 | 特征点+描述子 | **直接法 (光度误差)** |
| LiDAR关联 | iKd-Tree | **VoxelMap (八叉树+平面)** |
| 曝光估计 | 无 | **IESKF中在线估计** |
| LiDAR型号 | Livox only | **7种** (AVIA,Velo,Ouster,L515,XT32,Pandar,Robosense) |
| RGB着色 | 无 | **LiDAR投影RGB取色** |
| Colmap输出 | 无 | **支持** |

### 5.6 为什么叫 "FAST" - 性能优化

1. **直接法免特征提取**: 跳过FAST/SIFT描述子计算
2. **网格化**: 图像分 grid_n_width×grid_n_height 网格，每网格只保留一个最佳点
3. **粗到精金字塔**: patch_pyrimid_level 级，逐级细化
4. **OpenMP**: 雅可比计算并行 (`#pragma omp parallel for`)
5. **Warp缓存**: `warp_map` 缓存仿射变换矩阵

---


## 6. 优缺点 + 意义

### 6.1 优势

1. **真正的 tight coupling**: 单一协方差矩阵统一管理LiDAR和视觉不确定性
2. **直接法鲁棒**: 弱纹理场景优于特征点法
3. **曝光在线估计**: 解决相机自动曝光问题
4. **多LiDAR即插即用**: 7种主流LiDAR型号

### 6.2 局限性

1. **光度一致性假设** (`vio.cpp:749`): Lambertian反射假设，镜面/高光失效
2. **IMU运动激励需求**: bias可观测性需要加速度变化
3. **无回环**: 纯里程计，长距离漂移
4. **小平面假设**: 大曲率表面PCA不准确
5. **单目深度依赖LiDAR**: 纯视觉尺度不可观

### 6.3 对SLAM领域的意义

1. 证明统一IESKF可实现LiDAR-Visual-Inertial的tight coupling
2. 直接法+LiDAR联合优化范例
3. HKU Mars Lab技术栈 (Fast-LIO→R2LIVE→R3LIVE→FAST-LIVO2) 形成完整生态系统


## 7. 三大框架技术路线差异对比

### 7.1 融合架构

| 维度 | FAST-LIVO2 | R3LIVE | LVI-SAM |
|------|-----------|--------|---------|
| 融合方式 | **统一IESKF** (tight) | **双ESIKF** (tight) | **VINS+LIO-SAM** (loose) |
| 优化 | 单一卡尔曼滤波 | LIO/VIO两个并行KF | GTSAM因子图+Ceres滑窗 |
| 视觉 | 直接法(光度) | 稀疏对齐ESIKF | 特征点(Shi-Tomasi+KLT) |
| 状态维 | 19 | LIO:18, VIO:29+ | 因子图变量 |

### 7.2 核心差异总结

| 特性 | FAST-LIVO2 | R3LIVE | LVI-SAM |
|------|-----------|--------|---------|
| LiDAR前端 | VoxelMap八叉树 | iKd-Tree+ikd-Tree | 体素+FLANN kd-tree |
| RGB着色 | 投影取色 | RGB全局优化 | 无 |
| 3D重建 | 无 | Mesh+纹理(MVS) | 无 |
| 回环 | 无 | 无 | DBoW视觉+LiDAR ICP |
| 部署难度 | 低(单节点) | 中(多线程) | 高(5节点ROS) |


## 8. 数据管线

### 8.1 传感器输入总览

| 传感器 | 频率 | 原始格式 | ROS接口 | 消费模块 |
|--------|------|----------|---------|----------|
| LiDAR (7种) | 10Hz | AVIA: `livox_ros_driver::CustomMsg` / 标准: `sensor_msgs::PointCloud2` | `livox_pcl_cbk` / `standard_pcl_cbk` | LIO更新 |
| IMU | 200Hz+ | `sensor_msgs::Imu` | `imu_cbk` → `imu_buffer` + `prop_imu_buffer` | IMU传播+LIO+VIO |
| 全局快门相机 | 10-40Hz | `sensor_msgs::Image` (BGR8) | `image_cbk` → `img_buffer` + `img_time_buffer` | VIO更新 |

### 8.2 LiDAR管线

```
原始点云(CustomMsg / PointCloud2)
  → standard_pcl_cbk(): imu_time_offset校正
  → Preprocess::process(): 提取特征 → LiDARFeature {Nor/Plane/Edge_Jump/Edge_Plane/Wire/ZeroPoint}
  → 入队 lid_raw_data_buffer + lid_header_time_buffer
  → sync_packages(): 以img_capture_time为界切割 → pcl_proc_cur(前段) + pcl_proc_next(后段)
  → UndistortPcl(): 前向传播(记录每一步IMUpose) → 后向传播: P_compensate = extR_Ri*(R_i*(extR*P+extT) + T_ei) - extR_extT
  → VoxelMapManager::BuildResidualList(): 哈希查体素 → 找到VoxelPlane → dis_to_plane = n^T·p_w + d
```

**标定**: LiDAR-IMU外参(const预标定), LiDAR时间偏移 `lidar_time_offset`  
**预处理**: 7种LiDAR统一接口, `plane_judge()` + `edge_jump_judge()` 分类特征类型  
**特征**: Poss_Plane / Real_Plane / Edge_Jump / Edge_Plane / Wire / ZeroPoint 六类  
**匹配**: VOXEL_LOCATION哈希 → VoxelOctoTree递归 → VoxelPlane叶节点 → 点到平面  
**因子构建**: `Hsub = [point_crossmat * R^T * normal, normal^T]` (1×6), `R_inv = 1/(0.001+sigma_l+n^T·var·n)`

### 8.3 视觉管线

```
原始BGR图像
  → image_cbk(): img_time_offset校正 → 彩色转灰度(保留img_rgb)
  → sync_packages(): 以曝光时间中点为界切割LiDAR，构建MeasureGroup
  → generateVisualMapPoints(): 网格内Shi-Tomasi分最高LiDAR点 → 创建VisualPoint+Feature
  → 直接法残差: r_ij = τ_cur*I_cur(π(T_cw*p_w)) - τ_ref*I_warped_j
  → 粗到精金字塔: patch_pyrimid_level级, 每级迭代max_iterations次
```

**标定**: 相机内参+IMU-相机外参预标定, inv_expo_time在线估计(IESKF状态)  
**预处理**: BGR→灰度, resize若尺寸不匹配, 保留RGB用于着色  
**特征**: 非传统角点! 直接法稀疏图像块, `Feature{px_, f_, level_, point_, patch_, T_f_w_, inv_expo_time_}`  
**匹配**: 光度误差直接最小化, 不使用描述子匹配, 通过重投影+仿射变换warp比较patch  
**因子构建**: `∂r/∂δθ = J_img * ∂π/∂p_c * [Rci - Rci*skew(Pic)]`, `∂r/∂δp = J_img*∂π/∂p_c*[Rci*R_wi^T]`, `∂r/∂τ = I_cur`

### 8.4 IMU管线

```
sensor_msgs::Imu (≥200Hz)
  → imu_cbk(): imu_time_offset校正 → imu_buffer + prop_imu_buffer(独立传播用)
  → IMU_init(max=3帧): Welford在线均值 → gravity = -mean_acc/|mean_acc|*G_m_s2 → 重力对齐
  → IMU传播: R_k+1=R_k*Exp(angvel-b_g, dt) / p_k+1=p_k+v*dt+0.5*a_global*dt²
  → 协方差传播: P_k+1 = F_x*P_k*F_x^T + cov_w (F_x含∂Δrot/∂bg, ∂Δpos/∂v, ∂Δvel/∂R,ba,g)
```

**预处理**: `imu_time_offset` 时间校正, `IMU_mean_acc_norm` 归一化消去传感器比例因子  
**因子**: 纯IMU传播(预测步), 不单独构造成因子, 通过 `vec = x_propagate - x_current` 构建先验约束

### 8.5 跨传感器协同

| 协同机制 | 实现位置 | 说明 |
|----------|----------|------|
| 时间同步 | `sync_packages()` | 以图像时间戳为切割轴对齐LiDAR和IMU, img_capture_time + exposure_time_init |
| 缓冲策略 | `lid_raw_data_buffer` + `img_buffer` + `imu_buffer` | 三个独立deque, sync_packages()统一消费 |
| 状态机设计 | `EKF_STATE: WAIT→LIO→VIO` | 串行交替: LIO更新_state后VIO以更新后的_state为起点继续优化 |
| 初始化顺序 | IMU初始化→重力对齐→VoxelMap首帧建图→VIO初始化 | 线性串行, 无等待依赖 |
| LiDAR辅助视觉深度验证 | `vio.cpp:619-641` | 投影LiDAR构建depth_img, 视觉点周围LiDAR深度差异>0.5m丢弃 |
| LiDAR平面更新视觉法向量 | `updateReferencePatch()` | 视觉点查VoxelMap平面→更新pt->normal_→收敛判断 |
| 降级策略 | `SLAM_MODE: LIVO→LIO→LO` | 图像缺失自动降级LIO, IMU缺失降级LO(恒速模型) |
| 统一协方差 | `_state.cov` (19×19) | 单一协方差矩阵, LIO收缩后VIO继承继续收缩, 自动关联不确定性 |
