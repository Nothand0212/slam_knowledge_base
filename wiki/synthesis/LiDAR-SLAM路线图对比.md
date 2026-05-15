---
tags: [LiDAR, SLAM, 对比, 路线图, 激光SLAM, 方案选型]
type: synthesis
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/comparisons/LiDAR方案对比.md
  - wiki/topics/LiDAR数据管线.md
  - wiki/sources/2026-04-29-lidar-pipeline-comparison.md
  - wiki/sources/2026-04-29-framework-comparison.md
  - wiki/synthesis/phad_fusion设计总结.md
---

# LiDAR SLAM 路线图全系统对比

> 综合本知识库覆盖的全部 LiDAR SLAM 系统，从方法论演进、技术选型到场景推荐的结构化对比。

## 一、全系统对比总表

| 系统 | 传感器 | 后端 / 估计范式 | 回环检测 | 地图表示 | 核心创新 |
|------|--------|-----------------|----------|----------|----------|
| [[算法-LeGO-LOAM]] | LiDAR + IMU (可选) | GTSAM iSAM2 | 欧式距离 + PCL ICP | 点云子图 (局部) | 地面分割引导特征 + 两步解耦优化 (分变量 QR) |
| [[算法-LIO-SAM]] | LiDAR + IMU + GPS | LM 前端 + GTSAM ISAM2 | ScanContext + ICP | 点云关键帧 + 先验图 | IMU 预积分因子图 + 两级优化 + GPS 因子 |
| [[算法-FAST-LIO]] | LiDAR + IMU | IESKF (迭代误差状态 KF) | 无 (纯里程计) | ikd-Tree 动态 KD 树 | IESKF 紧耦合 + 反向传播去畸变 |
| [[算法-FAST-LIO-SAM-SC-QN]] | LiDAR + IMU (外接 LIO 前端) | GTSAM ISAM2 PGO | ScanContext → Quatro(GICP+GNC) | LiDAR 帧坐标系关键帧 | 四阶段回环验证: ScanContext→欧式→Quatro→Nano-GICP |
| [[算法-fusions_slam]] | LiDAR + IMU | IESKF + 后端因子图 | 可选 | IVox 体素地图 (GREEDY_VOXEL) | ROS-算法核强分离 + 在线平面拟合 + GM 核 |
| [[算法-KISS-ICP]] | LiDAR (纯) | 无全局后端 | 无 | 局部点云地图 | 极简纯 LiDAR baseline, 点到点 ICP, 恒速去畸变, 零 IMU 依赖 |
| [[算法-CT-ICP]] | LiDAR (IMU 可选) | Ceres 里程计 | 无 (纯里程计) | 局部点云地图 | 12-DoF 连续时间轨迹 (线性+slerp), Ceres OpenMP 并行 |
| [[方法-ICP变体族|GenZ-ICP]] | LiDAR (纯) | 纯前端库 | 无 | 最近邻 / 协方差地图 | 自适应平面度权重 α + 协方差 planar/non-planar 分类 |
| [[算法-Cartographer]] | LiDAR + IMU (可选) | Ceres scan-to-submap + 后端 PGO | Branch-and-Bound 网格搜索 | 概率占据网格子地图 | 子图系统异步回环 + Lua 场景自适应 + BnB 多分辨率 |
| [[算法-ROLO-SLAM]] | LiDAR + IMU + 轮速 | 局部约束 + IMU 预积分 | — | 点云子图 | RotVGICP 非结构化配准 + 地面车辆运动约束 |
| [[算法-lt-mapper]] | LiDAR (多会话) | GTSAM 后端 PGO | 长期回环 | 多会话拓扑 + GPS 校准 | Anchor 节点 PGO + Removert 动态变化检测 |
| [[算法-R3LIVE]] | LiDAR + 相机 + IMU | 双 ESIKF (LIO+VIO 独立) | 无 | RGB 着色点云 + 3D Mesh | 双 ESIKF 松耦合 + 全球首个 RGB 着色 LiDAR SLAM |
| [[算法-FAST-LIVO2]] | LiDAR + 相机 + IMU | 统一 19 维 IESKF (串行 LIO→VIO) | 无 | VoxelMap 八叉树 | 直接法光度误差代替特征 + 曝光在线估计 + 统一协方差传播 |
| [[算法-LVI-SAM]] | LiDAR + 相机 + IMU | GTSAM ISAM2 (松耦合因子图) | ScanContext + DBoW2 | 点云关键帧 + 视觉路标 | LiDAR-视觉松耦合因子图, 两路前端独立 + 后端联合 PGO |
| [[算法-PIN-SLAM]] | LiDAR (纯) | 自研 PGO + 可微优化 | Neural Point Map Context | Neural Points + SDF 连续场 | 神经隐式表示 + 全可微 pipeline (tracking+mapping+回环) |
| [[算法-BEV-LSLAM]] | LiDAR (纯) | Fast-VGICP + 4DoF PGO | DBoW3 回环 | BEV 图像 (高度+强度) | LiDAR→2D BEV 投影 → ORB 特征 + GMS 匹配, 双模态 |
| [[算法-P2V-SLAM]] | LiDAR (纯) | VE-Net 编码 + IR-Net 残差 | — | 隐式点-体素 SDF | 隐式点-体素观测模型: VE-Net 编码体素特征, IR-Net 输出残差与不确定度 |
| [[组件-lightning-lm\|Lightning-LM]] | LiDAR + IMU | IESKF + 后端 | 可选 | 局部点云 | 反向传播去畸变 + Cauchy 核 + 工程优化版 IESKF 前端 |
| [[算法-4DRadarSLAM]] | 4D Radar | GICP 变体 + PGO | ScanContext + GICP | 雷达子地图 | 4D 毫米波雷达替代 LiDAR, 全天候/雨雾环境 SLAM |

## 二、LiDAR SLAM 演进路线图

### 第一阶段：传统 LOAM 系 (2014–2018)

```
LOAM (2014)
 ├─ 曲率分类 edge/planar 特征
 ├─ 高频里程计 + 低频建图
 └─ → LeGO-LOAM (2018): 地面分割 + 两步解耦优化
      └─ → LIO-SAM (2020): IMU 预积分因子图 + GPS 因子 + iSAM2
           └─ → FAST-LIO-SAM-SC-QN: LIO 前端外接 + 四阶段回环
```

**特征**：手工特征提取, 边缘/平面分类, 解耦优化, 工程成熟但结构化假设强。

副线即 Cartographer (2016): Google 的工业方案, 用概率占据网格子图 + Branch-and-Bound 回环, 不依赖特征提取, 走的是"稠密配准 + 异步维护"另一条路。

---

### 第二阶段：IESKF 紧耦合 (2020–2022)

```
FAST-LIO (2021)
 ├─ IESKF 统一状态 (IMU 传播 + 点云更新)
 ├─ ikd-Tree 增量 KD 树
 └─ → FAST-LIO2 (2022): VoxelMap 替代 KD 树, 反向传播去畸变
      ├─ → fusions_slam: ROS 解耦 + IVox 体素 + GM 核
      ├─ → lightning-lm: 工程优化版 IESKF + Cauchy 核
      └─ → R3LIVE (2022): 双 ESIKF + RGB 着色
           └─ → FAST-LIVO2 (2023): 统一 19D IESKF + 直接法光度 + VoxelMap 八叉树
```

**特征**：IMU 状态传播和点云配准在单一误差状态框架中求解，去畸变更精确，前端速度极快 (实时单帧 < 10ms)，但自身缺乏全局一致性后端的项目需要外接 PGO。

---

### 第三阶段：自适应与连续时间 (2021–2023)

```
标准 ICP → KISS-ICP (极简纯 LiDAR, 恒速模型去畸变)
GICP → GenZ-ICP (自适应 α 权重 planar/non-planar 协方差分类)
离散时间 → CT-ICP (12-DoF 连续时间轨迹, Ceres 并行)
```

**特征**：不再假设"特征是 edge/planar"或"恒速 / 零速模型"。GICP 用协方差自适应建模表面的不确定度, CT-ICP 把整个扫描建模为 12 自由度连续轨迹而非单一位姿, 天然解决扫描畸变且不需要 IMU。GenZ-ICP 证明纯 LiDAR ICP (点到分布) 在非结构化环境中可以接近 LIO 精度。

---

### 第四阶段：神经隐式与可微地图 (2023–)

```
传统离散地图 (Point Cloud/KD-Tree)
  → 结构化稀疏 (VoxelHashMap/ikd-Tree/IVox)
    → PIN-SLAM: Neural Points + MLP SDF 解码器, 可微 tracking+mapping+回环
    → P2V-SLAM: 隐式点-体素观测 (VE-Net + IR-Net)
    → BEV-LSLAM: LiDAR→2D BEV 投影 → ORB 视觉管线复用
```

**特征**：地图不再只是显式点云或体素, 而是连续隐式场 (SDF/密度/占用), 通过可微渲染/可微 SDF 求导统一优化。地图压缩率比离散表示高 1-2 个数量级, 但需要 GPU 训练, 在线部署仍有长尾挑战。

---

### 演进规律总结

| 代际 | 数学 | 去畸变 | 地图 | 后端 | 局限性 |
|------|------|--------|------|------|--------|
| 经典 LOAM | 曲率手工特征 | 恒速/IMU 角速度 | 离散点云子图 | LM / iSAM2 | 结构化场景假设 |
| IESKF 紧耦合 | 迭代误差状态 + IMU 传播 | 反向传播 / 统一状态 | 体素 / 八叉树 | 可外接 PGO | 协方差退化处理 |
| 自适应 / 连续时间 | GICP 协方差 / 连续轨迹 | 连续时间模型 / 自适应配准 | 协方差地图 | 多依赖外接 | 纯里程计无回环 |
| 神经隐式 / 可微 | 可微 SDF + Neural Points | 连续 SDF 插值 | 隐式场 (MLP) | 可微 PGO | GPU 依赖, 长距离鲁棒性待验证 |
| 工业长周期 (Cartographer/lt-mapper) | 概率网格 / 子图复用 | 异步处理 | 子图 + 拓扑 | 长期 PGO | 大图内存/计算 |

## 三、关键维度对比

### 3.1 前端配准方法

| 配准方式 | 系统示例 | 残差类型 | 鲁棒核 | 优点 | 代价 |
|----------|----------|----------|--------|------|------|
| 点到点 ICP | [[算法-KISS-ICP]] | 欧式距离 | GM | 极简无依赖 | 对初值/退化敏感 |
| 边缘/平面特征 | [[算法-LeGO-LOAM]], [[算法-LIO-SAM]] | edge-to-edge / point-to-plane | Huber-like | 速度快, 结构化好 | 特征提取阈值/地面假设敏感 |
| IESKF 紧耦合 | [[算法-FAST-LIO]], [[算法-fusions_slam]], [[算法-FAST-LIVO2]] | 点到面 / 点到体素 | GM / Cauchy | 去畸变统一, 状态紧耦合 | 雅可比实现复杂 |
| GICP / 点到分布 | [[方法-ICP变体族|GenZ-ICP]], [[方法-Fast-VGICP]], [[方法-RotVGICP]] | 马氏距离 | Cauchy | 协方差建模更稳 | KNN 计算和邻域估计成本 |
| 连续时间轨迹 | [[算法-CT-ICP]] | POINT_TO_PLANE / DISTRIBUTION | Cauchy/Huber/Truncated | 自然去畸变 | 优化变量多, 无回环 |
| 直接法光度误差 | [[算法-FAST-LIVO2]] | 光度残差 | — | 利用纹理, 弱纹理也能用 | 曝光/标定敏感 |
| 神经隐式 SDF | [[算法-PIN-SLAM]] | BCE / MAE SDF 损失 | GM | 连续表示 | GPU, 训练收敛敏感 |
| BEV 投影 + ORB | [[算法-BEV-LSLAM]] | 重投影 + ICP | — | 复用成熟视觉管线 | 投影参数敏感 |

### 3.2 运动畸变矫正 (Deskewing)

这是 LiDAR 管线最关键的单步处理。不同方案的去畸变精度差异可达数量级。

| 方法 | 代表系统 | 精度 @10m/s | 计算成本 | IMU 依赖 |
|------|----------|------------|----------|----------|
| CT 连续时间 | [[算法-CT-ICP]] | 2–5 cm | 高 (12 DoF) | 无 |
| IMU 反向传播 | [[算法-FAST-LIO]], [[算法-FAST-LIVO2]], [[算法-fusions_slam]] | 1–3 cm | 中 | 强 (需要高频 IMU) |
| IMU 角速度积分 | [[算法-LIO-SAM]], [[算法-LVI-SAM]] | 5–10 cm | 低 | 中 (仅旋转) |
| 恒速模型 | [[算法-KISS-ICP]] | 5–15 cm | 最低 | 无 |
| 神经隐式插值 | [[算法-PIN-SLAM]] | 3–8 cm | 高 (SDF 查询) | 无 |

### 3.3 IMU 使用深度

| 使用深度 | 系统 | IMU 角色 |
|----------|------|----------|
| 无 IMU | [[算法-KISS-ICP]], [[方法-ICP变体族|GenZ-ICP]], [[算法-BEV-LSLAM]] | 纯 LiDAR baseline |
| 弱 IMU | [[算法-Cartographer]], [[算法-LeGO-LOAM]] | 去畸变辅助 / 重力辅助 |
| 预积分 | [[算法-LIO-SAM]], [[算法-LVI-SAM]] | GTSAM 预积分因子 + 去畸变 |
| IESKF 紧耦合 | [[算法-FAST-LIO]], [[算法-fusions_slam]], [[组件-lightning-lm\|lightning-lm]] | 高频传播 + 状态直接修正 |
| 统一 IESKF (多传感器) | [[算法-FAST-LIVO2]], [[算法-R3LIVE]] | 19D 状态, 协方差跨传感器传播 |

### 3.4 后端与全局一致性

| 后端方式 | 代表系统 | 关键因子 | 回环方式 |
|----------|----------|----------|----------|
| 无后端 (纯里程计) | [[算法-KISS-ICP]], [[算法-CT-ICP]], [[方法-ICP变体族|GenZ-ICP]] | — | — |
| IESKF 局部 | [[算法-FAST-LIO]] | IMU 传播 + 点云更新 | — |
| Ceres 扫描匹配 | [[算法-Cartographer]] | 概率网格 scan-to-submap | Branch-and-Bound 网格搜索 |
| GTSAM iSAM2 | [[算法-LIO-SAM]], [[算法-FAST-LIO-SAM-SC-QN]], [[算法-LVI-SAM]] | PriorFactor + BetweenFactor + IMU + GPS | ScanContext / ICP |
| 长期 PGO | [[算法-lt-mapper]] | Anchor 节点 + BetweenFactor | Removert 动态过滤 |
| 可微 PGO | [[算法-PIN-SLAM]] | SDF 约束 + 回环因子 | Neural Point Map Context |

## 四、地图表示进化图谱

```
                    ┌→ PIN-SLAM: Neural Points + MLP SDF (连续, 可微, 压缩)
                    │
Point Cloud ────────┼→ P2V-SLAM: 隐式点-体素 SDF (VE-Net + IR-Net)
   │                │
   ├→ KD-Tree ─────→ ikd-Tree (FAST-LIO) ──→ IVox (fusions_slam)
   │                │
   ├→ VoxelGrid ───→ VoxelMap 八叉树 (FAST-LIVO2)
   │                │
   ├→ SubMap ───────→ 概率占据网格 (Cartographer)
   │                │
   └→ BEV Image ────→ BEV 高度图+强度图 (BEV-LSLAM)
```

### 地图维度比较

| 地图类型 | 分辨率 | 压缩率 | 可查询性 | 可微性 | 内存 (1km) |
|----------|--------|--------|----------|--------|-----------|
| 原始点云 | 毫米级 | 1x | O(n) | 否 | >1 GB |
| ikd-Tree | 厘米级 | ~10x | O(log n) | 否 | ~100 MB |
| IVox 体素 | 厘米级 | ~10x | O(1) 哈希 | 否 | ~50 MB |
| 概率网格子图 | 5 cm | ~5x | O(1) 表查 | 否 | ~200 MB |
| VoxelMap 八叉树 | 自适应 | ~20x | O(log n) | 否 | ~30 MB |
| Neural Points SDF | 连续 | ~100x | O(1) 哈希+MLP | 是 | ~10 MB |

## 五、场景推荐

### 5.1 按应用场景

| 场景 | 首选参考 | 理由 |
|------|----------|------|
| **快速搭建纯 LiDAR baseline** | [[算法-KISS-ICP]] | 依赖最少, 代码清晰, 适合最小可用里程计和教学 |
| **实时 LIO 前端 (LiDAR+IMU)** | [[算法-FAST-LIO]], [[算法-fusions_slam]] | IESKF 范式成熟, 速度最快, 单帧 < 10ms |
| **需要回环和全局一致性** | [[算法-LIO-SAM]], [[算法-FAST-LIO-SAM-SC-QN]] | GTSAM iSAM2 + ScanContext + GPS + PGO 完整链路 |
| **长时间 / 多会话建图** | [[算法-Cartographer]], [[算法-lt-mapper]] | 子图系统 + BnB 回环 / Anchor PGO + 动态滤除 |
| **非结构化 / 越野 / 退化地形** | [[算法-ROLO-SLAM]], [[方法-ICP变体族|GenZ-ICP]] | RotVGICP 协方差建模 + 地面运动约束 |
| **3D 稠密重建 + RGB 着色** | [[算法-R3LIVE]], [[算法-FAST-LIVO2]] | 双 IESKF 着色 + 统一融合 |
| **极端弱光 / 无纹理室内** | [[算法-FAST-LIO]] (纯 LIO) | 不受光照影响, IESKF 鲁棒 |
| **白天夜晚全时段** | [[算法-FAST-LIVO2]] (直接法 VIO) | 光度误差 + 曝光估计 |
| **雨雾 / 扬尘环境 (雷达)** | [[算法-4DRadarSLAM]] | 4D 毫米波雷达全天候 |
| **地图压缩 + 连续重建** | [[算法-PIN-SLAM]] | Neural Points SDF, 100x 压缩 |
| **复用视觉回环管线** | [[算法-BEV-LSLAM]] | LiDAR→BEV→ORB→DBoW3, 视觉管线直接复用 |

### 5.2 按系统复杂度和工程约束

| 约束 | 推荐 | 理由 |
|------|------|------|
| **ROS 耦合低, 可独立编译核** | [[算法-fusions_slam]], [[方法-ICP变体族|GenZ-ICP]] | 算法核 ROS-free, 薄 wrapper 层 |
| **最小依赖 (无 PCL/Ceres/GTSAM)** | [[算法-KISS-ICP]], [[方法-ICP变体族|GenZ-ICP]] | 仅 Eigen 或仅 Eigen+OpenMP |
| **ROS 2 优先** | [[方法-ICP变体族|GenZ-ICP]] | ROS1/ROS2 条件编译双支持 |
| **需要 GPS / GNSS 因子** | [[算法-LIO-SAM]], [[算法-OB_GINS]], [[算法-IC-GVINS]] | GPSFactor 内建或 GNSS-INS 紧耦合 |
| **GPU 可用, 需要神经表示** | [[算法-PIN-SLAM]], [[算法-P2V-SLAM]] | 需要 CUDA 训练和推理 |
| **算力受限 / 嵌入端** | [[算法-KISS-ICP]] + [[算法-FAST-LIO]] | 前端极轻, 后端可选 |

### 5.3 按设计模式参考

| 参考目标 | 系统 | 可取的设计模式 |
|----------|------|---------------|
| 因子图架构 | [[算法-LIO-SAM]], [[算法-LVI-SAM]] | IMU 预积分因子 + GPS 因子 + BetweenFactor PGO |
| ROS 解耦分层 | [[算法-fusions_slam]] | fusion_slam 核 (ROS-free) + ROS wrapper |
| 传感器抽象 | [[算法-FAST-LIO-SAM-SC-QN]] | 外部 LIO 前端 + 后端独立 PGO |
| 多级回环验证 | [[算法-FAST-LIO-SAM-SC-QN]] | ScanContext 召回 → 欧式 → Quatro → GICP |
| 连续时间建模 | [[算法-CT-ICP]] | 12 DoF 帧轨迹 + 线性模型 + Ceres 并行 |
| 自适应配准 | [[方法-ICP变体族|GenZ-ICP]] | 协方差 planar/non-planar 分类 + 自适应权重 |
| 双线程前端-后端 | [[算法-Cartographer]] | Local SLAM (scan-to-submap) + Global SLAM (PGO) |

## 六、系统详细技术剖解

### 6.1 LeGO-LOAM：轻量地面优化

LeGO-LOAM (IROS 2018, 2800+ stars) 是 LOAM 最成功的工程改进版。其核心创新是**地面分割引导特征分类**：将非地面点划为边缘特征 (用于水平约束)，地面点划为平面特征 (用于 roll/pitch/z 约束)，随后执行 [[方法-两步解耦优化]] —— 地面 patch 约束 roll/pitch/tz，边缘约束 yaw/tx/ty，两步各解一个 3×3 QR 问题。分离后的低维线性问题比原始 LOAM 的全 6-DOF 非线性优化快数倍。

**工程边界**：硬编码 `N_SCAN=16` (VLP-16 优化)，对非结构化环境 (空中无人机、陡坡) 地面假设失效。无 IMU 预积分，仅 IMU 旋转去畸变。

### 6.2 LIO-SAM：因子图 LiDAR-IMU 标杆

LIO-SAM (3822 stars) 是工程化 LiDAR-IMU 标杆系统。它采用 [[架构-两级优化架构]]：30 次 LM 扫描-子图匹配作为前端，GTSAM ISAM2 作为后端全局因子图。因子图设计精巧：PriorFactor 给 roll/pitch 紧约束 (1e-2)，yaw/trans 弱约束 (π²/1e8)；BetweenFactor 中旋转约束远紧于平移 (rot 1e-6 vs trans 1e-4)；GPSFactor 仅在移动 > 5m 且协方差合适时激活；回环用 PCL ICP fitness < 0.3。每 100 帧重置 ISAM2 以控制图规模，同时保留边缘协方差 prior。

**关键细节**：ImageProjection 节点通过 IMU 角速度积分做逐点旋转去畸变，精度 ~5-10 cm @10m/s，低于反向传播方案但实现简单。

### 6.3 FAST-LIO 系列：IESKF 范式

FAST-LIO (2021) 开创了 IESKF 紧耦合范式：将 IMU 状态传播和 LiDAR 点云配准统一在一个误差状态卡尔曼滤波器 (IESKF) 中。核心数学：IMU 驱动前向传播 (`x_{k+1} = x_k ⊞ f(x_k, u, 0) Δt`)，点云到达后执行迭代误差状态更新 (`K = P H^T (H P H^T + R)^{-1}`)，每帧内可迭代多次 (通常 1-4 次) 直至收敛。

FAST-LIO2 (2022) 的升级：用 ivox (增量体素) 替代 ikd-Tree，查邻速度 O(1)；反向传播 (back-propagation) 在每个点云到达时，用每点时间戳从 IMU 测量序列反向计算从帧末到测量时刻的运动补偿，精度 ~1-3 cm @10m/s，是目前去畸变能力的上限之一。

FAST-LIVO2 (2023) 进一步将视觉直接法光度误差接入同一 IESKF，统一 19 维状态 (IMU 15D + 曝光时间 + 时间偏移等)，在协方差级别自动关联 LiDAR 和视觉不确定性。

### 6.4 Cartographer：工业级双 SLAM 架构

Google Cartographer 以其 **子图系统** 闻名：Local SLAM 每 90 帧 scan 生成一个概率占据网格子图，子图冻结后变为不可变共享指针。Global SLAM 维护子图间位姿图，通过 [[方法-Branch-and-Bound回环检测]] 在全部子图中搜索回环候选 —— 多分辨率预计算网格栈将复杂度从 O(n²) 降至 O(n log n)。

Cartographer 不依赖特征提取，而是直接做稠密 scan-to-submap 匹配 (ceres)，对任何 LiDAR 形态 (单线/多线、2D/3D) 都适用。Lua 配置系统支持场景自适应。代价是内存占用大 (子图+网格长期保留)，且无原生 IMU 预积分。

### 6.5 CT-ICP：连续时间的优雅

CT-ICP 解决了 LiDAR SLAM 最根本的问题 —— **扫描内运动畸变** —— 不通过 IMU，而是通过把每帧建模为 12-DoF 连续轨迹：`T(α) = T_begin * Exp(α * Log(T_begin⁻¹ * T_end))`。平移用线性插值，旋转用四元数 slerp。

四种残差统一在 Ceres Problem 中：POINT_TO_PLANE (默认)、POINT、LINE、DISTRIBUTION (GICP)；权重 = planarity^power × exp(-dist/threshold)；鲁棒核 Cauchy (默认)/Huber/Truncated。Ceres OpenMP 并行计算每点的 KNN 邻域，适合多核 CPU。

**工程局限**：纯里程计无回环/全局优化，长距离漂移无法纠正；需每点精确时间戳。对高质量 IMU 场景，IESKF 反向传播可能更简洁。

### 6.6 KISS-ICP：极简纯 LiDAR Baseline

KISS-ICP 是"最简可用 LiDAR 里程计"的参考实现：仅有三种依赖 (Eigen、nanoflann、tbb)，ROS 可选。核心仅约 2000 行 C++。恒速模型去畸变 + 点到点 ICP + GM 鲁棒核。没有特征提取、没有 IMU、没有后端。

KISS-ICP 证明了在简单场景下 (KITTI, MulRan, 结构化城市)，纯 LiDAR ICP 里程计可以接近复杂 LIO 系统的性能。其 zero-dependency 哲学使其成为教学基准和算力受限平台的首选。

### 6.7 PIN-SLAM：神经隐式 LiDAR SLAM

PIN-SLAM (IPB Bonn) 用 neural points (3D 位置 + 8 维特征 + 四元数方向) 替代离散点云/体素，通过极轻量 MLP (11→64→1) 解码为连续 SDF 值。整个 pipeline —— tracking、mapping、回环、PGO —— 都可微，梯度从 SDF 损失回流到 neural point 特征和 MLP 参数。

地图压缩率 ~100x 对比离散表示，稠密几何重建质量高。代价是 GPU 训练需求 (每帧迭代数十步梯度下降)，在线部署和大规模长距离鲁棒性仍需验证。

### 6.8 4D Radar SLAM：全天候替代方案

[[算法-4DRadarSLAM]] 用 4D 毫米波雷达 (点云 + 多普勒速度 + RCS/SNR) 替代 LiDAR，GICP 变体配准 + ScanContext 回环 + PGO。雷达在雨雾扬尘场景穿透力远强于 LiDAR，但点云稀疏 (~200-500 点/帧 vs LiDAR 数万点) 且噪声更大，需要更鲁棒的协方差建模和回环验证。

## 七、典型系统性能对标

| 系统 | 室内 ATE RMSE | 室外 ATE RMSE | 单帧耗时 | 内存/1km | 去畸变精度 |
|------|--------------|--------------|----------|----------|-----------|
| [[算法-KISS-ICP]] | 0.05-0.2 m | 0.5-2.0 m | < 10 ms | ~50 MB | 低 (恒速) |
| [[算法-LeGO-LOAM]] | 0.02-0.1 m | 0.3-1.0 m | ~20 ms | ~100 MB | 中 (IMU 旋转) |
| [[算法-LIO-SAM]] | 0.01-0.05 m | 0.1-0.5 m | ~30 ms | ~300 MB | 中高 (IMU 角速度) |
| [[算法-FAST-LIO]] | 0.01-0.03 m | 0.05-0.3 m | < 10 ms | ~100 MB | 高 (反向传播) |
| [[算法-Cartographer]] | 0.02-0.1 m | 0.2-1.0 m | ~50 ms | ~500 MB | 中 |
| [[算法-CT-ICP]] | 0.01-0.05 m | 0.1-0.5 m | ~30 ms | ~80 MB | 最高 (连续时间) |
| [[算法-PIN-SLAM]] | 0.02-0.08 m | 0.1-0.5 m | ~100 ms (GPU) | ~5 MB | 中高 |
| [[算法-FAST-LIVO2]] | 0.01-0.02 m | 0.05-0.2 m | < 20 ms | ~200 MB | 高 |
| [[算法-R3LIVE]] | 0.02-0.05 m | 0.1-0.3 m | < 30 ms | ~300 MB | 高 |

> 注：以上为各系统论文/社区报告的典型值范围，实际性能取决于传感器、平台速度和场景特性。

## 八、策略建议：不同 LiDAR 传感器适配

| LiDAR 传感器 | 特征 | 推荐系统 | 理由 |
|-------------|------|----------|------|
| **VLP-16 / RS-LiDAR-16** (16 线) | 低线束, 稀疏 | [[算法-LeGO-LOAM]], [[算法-LIO-SAM]] | 地面分割和 LOAM 特征对低线束优化 |
| **OS1-64 / VLS-128** (64-128 线) | 高线束, 稠密 | [[算法-FAST-LIO]], [[算法-KISS-ICP]] | 稠密点云使最简单方法也能工作 |
| **Livox Mid-40 / Avia** (非重复扫描) | 非规则扫描, 无传统 ring | [[算法-FAST-LIO]], [[算法-CT-ICP]] | IESKF 无需 ring 假设, CT-ICP 天然处理不规则时间戳 |
| **Ouster / Hesai** (高分辨率, 低反射强度) | 高垂直角分辨率 | [[算法-FAST-LIVO2]] | 视觉直接法补强低反射特征, 直接法对纹理连续区域好 |
| **单线激光 (SICK / Hokuyo)** (2D) | 单平面扫描 | Cartographer (2D) | Cartographer 2D 模式专门设计 |
| **固态/半固态 LiDAR** (非 360°) | 受限 FOV | [[算法-fusions_slam]] + [[组件-gtsam_points]] | 小 FOV 下回环更依赖描述子, gtsam_points 集成多种 LiDAR 因子 |
| **4D Radar** (替代 LiDAR) | 极稀疏, 多普勒 | [[算法-4DRadarSLAM]] | GICP 变体 + ScanContext 针对雷达点云稀疏性设计 |

## 九、快速选型决策树

```
是否需要全局一致性和回环?
├── 否 → 纯里程计即可
│   ├── 有 IMU? → [[算法-FAST-LIO]] (高精度) / [[算法-fusions_slam]] (解耦)
│   │   └── GPU 可用 + 需要稠密重建? → [[算法-PIN-SLAM]]
│   ├── 无 IMU? → [[算法-KISS-ICP]] (极简) / [[算法-CT-ICP]] (连续时间)
│   │   └── 非结构化场景? → [[方法-ICP变体族|GenZ-ICP]] (自适应 GICP)
│   ├── 全场景全天候? → [[算法-4DRadarSLAM]] (4D 雷达)
│   └── 车载/越野? → [[算法-ROLO-SLAM]] (RotVGICP + 运动约束)
│
├── 是 → 需要回环 + 后端
│   ├── 仅 LiDAR+IMU?
│   │   ├── 前端外接 LIO? → [[算法-FAST-LIO-SAM-SC-QN]] (四阶段回环)
│   │   ├── 完整系统? → [[算法-LIO-SAM]] (因子图 PGO)
│   │   └── 多会话/长期? → [[算法-Cartographer]] / [[算法-lt-mapper]]
│   ├── LiDAR+视觉+IMU (LVIO)?
│   │   ├── 紧耦合? → [[算法-FAST-LIVO2]] (统一 IESKF)
│   │   ├── 松耦合? → [[算法-LVI-SAM]] (GTSAM 因子图)
│   │   └── RGB 着色? → [[算法-R3LIVE]] (双 ESIKF + Mesh)
│   └── 复用视觉回环? → [[算法-BEV-LSLAM]] (BEV→ORB→DBoW3)
│
└── 需要多传感器因子图统一后端? → [[phad_fusion设计总结\|phad_fusion]]
    (LiDAR ICP + 视觉投影 + IMU 预积分 + GNSS + 回环 → GTSAM iSAM2)
```

## 十、实施复杂度评估

| 复杂度级别 | 系统 | 学习成本 | 适合人群 |
|------------|------|----------|----------|
| **极低** (< 2 天可复现) | [[算法-KISS-ICP]] | 入门 | 学生、快速原型、教学 |
| **低** (~ 1 周可跑通) | [[方法-ICP变体族|GenZ-ICP]], [[算法-CT-ICP]] | 基础 | CV/SLAM 研究者、纯 LiDAR 入门 |
| **中** (~ 2 周可集成) | [[算法-LeGO-LOAM]], [[算法-LIO-SAM]], [[算法-Cartographer]], [[算法-FAST-LIO]] | 进阶 | 有 ROS/SLAM 基础的工程师 |
| **中高** (~ 1 月可修改) | [[算法-FAST-LIO-SAM-SC-QN]], [[算法-fusions_slam]], [[算法-R3LIVE]], [[算法-LVI-SAM]], [[组件-lightning-lm\|lightning-lm]] | 高阶 | 需要深度定制的项目 |
| **高** (~ 2 月可移植) | [[算法-FAST-LIVO2]], [[算法-PIN-SLAM]], [[算法-lt-mapper]] | 专家 | 学术界魔改、新范式验证 |
| **极高** (需团队) | [[算法-BEV-LSLAM]], [[算法-4DRadarSLAM]], [[算法-P2V-SLAM]], [[phad_fusion设计总结\|phad_fusion]] | 架构师 | 新传感器框架、工业级部署 |

> 复杂度评估基于：依赖体量、代码规模、调参经验要求、ROS 耦合度、硬件需求 (GPU/FPGA)、多传感器标定复杂度。

## 十一、设计原则总结

在对本知识库所有 LiDAR SLAM 系统进行综合对比后，提取以下可复用设计原则：

1. **运动补偿层、局部配准层、全局约束层应明确分离**。不要将 FAST-LIO、LIO-SAM 或 Cartographer 整套搬进新系统，而应抽取模式 (去畸变方法、因子图后端、回环验证管线) 作为可替换模块。

2. **IESKF 前端 + iSAM2 后端是目前最成熟的前端-后端解耦范式**。前端保证实时性和去畸变精度, 后端保证全局一致性和多传感器因子可扩展性。

3. **去畸变方法应按平台选择**：机载/高振动 → 反向传播 or IESKF 统一状态；车载/低振动 → 恒速模型可能足够；CT-ICP 连续时间模型对缺少 IMU 的传感器最友好。

4. **回环检测应分层**：ScanContext/描述子负责广度召回 → 几何验证 (GICP/ICP/Quatro) 负责精度 → 位姿图 (PGO) 负责一致性。避免单步直接判定回环。

5. **地图表示取决于最终用途**：实时定位 → VoxelMap/ikd-Tree; 多会话 → 子图+拓扑; 压缩传输 → Neural Points SDF; 复用视觉管线 → BEV 投影。

6. **鲁棒核的选择应与场景匹配**：结构化室内 → Huber; 动态外点多 → Cauchy; 需要强外点剔除 → Geman-McClure。鲁棒核不能替代前端的外点过滤。

## 十二、标准数据集与评测

| 数据集 | 场景 | 传感器 | 推荐评估系统 |
|--------|------|--------|-------------|
| **KITTI** (odometry) | 城市/高速, 车载 | Velodyne HDL-64E + IMU + GPS | [[算法-FAST-LIO]], [[算法-LIO-SAM]], [[算法-KISS-ICP]] |
| **MulRan** | 城市/河滨, 车载 | Velodyne VLP-16 | [[算法-KISS-ICP]], [[方法-ICP变体族|GenZ-ICP]], [[算法-Cartographer]] |
| **Newer College** | 室内/室外/公园, 手持 | OS1-64 + IMU | [[算法-FAST-LIVO2]], [[算法-FAST-LIO]], [[算法-LVI-SAM]] |
| **Hilti SLAM** | 室内/室外/隧道, 手持+AGV | Hesai Pandar + 多 IMU | [[算法-FAST-LIO]], [[算法-fusions_slam]] |
| **M2DGR** | 室内/室外/电梯, 地面+手持 | RS-Helios + IMU + 6 相机 + 轮速 | LIO + VIO + GNSS 融合方案 |
| **NTU VIRAL** | 室内/室外, 无人机+手持 | OS1-16 + 双目 + IMU | [[算法-R3LIVE]], [[算法-FAST-LIVO2]], [[算法-LVI-SAM]] |
| **SubT** (DARPA) | 洞穴/矿井/城市地下, 多平台 | 多传感器组合 | 退化检测 + 多传感器融合 ([[算法-fusions_slam]] 类) |
| **KITTI-360** | 郊区, 车载 | Velodyne HDL-64E + 4 鱼眼 + 双目 | LiDAR-视觉联合系统 |

> 评估指标: ATE RMSE (绝对轨迹误差均方根), RPE (相对位姿误差), RMSE 的 t/m 和 r/m, 以及单帧耗时、内存、初始化时间。

## 十三、未来方向

- **连续时间 + 因子图融合**：CT-ICP 的连续轨迹建模 + GTSAM B 样条因子, 目前 gtsam_points 已有原型。
- **神经-几何混合**：像 P2V-SLAM 一样用可微神经观测替代手工配准残差, 但保留因子图作为全局优化骨架。
- **4D 毫米波雷达 SLAM**：LiDAR 的雷达替代方案, 在全天候、雨雾、扬尘场景有天然优势。
- **多模态统一后端**：视觉因子 + LiDAR 因子 + GNSS 因子 + 雷达因子在同一个 iSAM2 因子图中联合优化, phad_fusion 正在探索此范式。

## 相关页面

- [[LiDAR方案对比]]
- [[LiDAR数据管线]]
- [[VIO方案全景对比]]
- [[因子图vs滤波]]
- [[phad_fusion设计总结]]
- [[方法-GICP配准方法]]
- [[方法-体素地图]]
- [[方法-ScanContext]]
- [[概念-连续时间轨迹]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]
- [[组件-GTSAM]]
- [[组件-Ceres-Solver]]
