---
tags: [VIO, SLAM, 对比, 视觉惯性里程计, 方案选型, 滤波, 优化, 深度学习]
type: synthesis
created: 2026-05-15
updated: 2026-05-15
sources:
  - wiki/comparisons/VIO方案对比.md
  - wiki/sources/2026-04-29-open-vins-analysis.md
  - wiki/sources/2026-04-29-vins-fusion-analysis-analysis.md
  - wiki/sources/2026-04-29-orb-slam3-analysis.md
  - wiki/sources/2026-04-29-kimera_vio-analysis.md
  - wiki/sources/2026-04-29-dm-vio-analysis-analysis.md
  - wiki/sources/2026-04-29-dso-analysis-analysis.md
  - wiki/sources/2026-04-29-svo-pro-analysis-analysis.md
  - wiki/sources/2026-04-29-schurvins-analysis-analysis.md
  - wiki/sources/2026-04-29-rovio-analysis-analysis.md
  - wiki/sources/2026-04-29-msckf-vio-analysis-analysis.md
  - wiki/sources/2026-04-29-openmavis-analysis-analysis.md
  - wiki/sources/2026-04-29-esvo-analysis-analysis.md
  - wiki/sources/2026-04-29-droid_slam-analysis.md
  - wiki/sources/2026-04-29-nice_slam-analysis.md
  - wiki/sources/2026-04-29-monogs-analysis.md
  - wiki/sources/2026-04-30-image-preprocessing-comparison.md
  - wiki/synthesis/phad_fusion设计总结.md
---

# VIO 方案全景对比

> 综合本知识库覆盖的全部视觉-惯性里程计 (VIO) 和视觉里程计 (VO) 系统，涵盖滤波、滑动窗口优化、增量平滑、直接法、半直接法和深度学习各范式。

## 一、全系统对比总表

### 1.1 滤波类 VIO

| 系统 | 后端 / 估计范式 | 视觉前端 | IMU 使用 | 初始化 | 关键帧 / 边缘化策略 | 核心创新 |
|------|----------------|----------|----------|--------|---------------------|----------|
| [[算法-OpenVINS]] | EKF / OC-MSCKF | KLT 光流 / ORB 描述子 | 高频 EKF 传播, ACI² 解析积分 | 静止 / 动态异步初始化, 三种 IMU 积分可切换 | MSCKF 零空间投影 + 滑窗 clone 边缘化 | ROS-Free 内核, 模块化协方差, 在线 IMU-Cam 外参/时偏标定 |
| [[概念-MSCKF\|MSCKF_VIO]] | OC-MSCKF | KLT 光流 | 传播 + 零空间投影更新 | 视觉-IMU 松耦合对齐 | 零空间消除特征变量, clone 按光度/视差策略边缘化 | 可观测性约束 (FEJ-OC) 保证线性化一致性 |
| [[算法-SchurVINS]] | Schur Complement EKF | 稀疏特征 (共视点跟踪率筛选) | RK4 直接积分 (无预积分框架) | FEJ 线性化一致性 | Schur Complement 替代零空间投影, 3×3 矩阵求逆, FEJ | Schur 补边缘化: 梯度+Hessian 累积 → 比传统 MSCKF 更接近 BA 约束 |
| [[算法-ROVIO]] | IEKF + patch EKF | 图像块 (patch) 光度误差 | IMU 传播 | 多级 patch 初始化 | patch 生命周期管理 (4 层: unseen→tracking→lost→deleted) | 飞行器专用, patch 光度 + EKF 紧耦合, 无描述子/特征 |

### 1.2 滑动窗口优化类 VIO

| 系统 | 后端 / 估计范式 | 视觉前端 | IMU 使用 | 初始化 | 关键帧 / 边缘化策略 | 核心创新 |
|------|----------------|----------|----------|--------|---------------------|----------|
| VINS-Fusion | Ceres 滑动窗口 BA (DENSE_SCHUR + DogLeg) | Shi-Tomasi 角点 + KLT 金字塔光流 | 中值预积分 (RK2), 含 Jacobian/协方差传播 | 三种: Mono+IMU (SFM+对齐), Stereo+IMU (PnP), Stereo only | Schur 补边缘化 MARGIN_OLD / MARGIN_SECOND_NEW 按视差选择 | 完整工程实现: GPS 全局融合, DBoW2 回环, 4-DOF PGO |
| [[算法-ORB-SLAM3]] | g2o 多级 BA + Atlas | ORB 特征 (FAST + Steered BRIEF) | 多模式 IMU 联合 BA (Inertial BA) | 三线程异步 (视觉后接 IMU), 多级递进 | 关键帧按条件插入: 视角/距离/质量, Local BA 滑动窗口 | Atlas 多地图, 6 种传感器模式, DBoW2 回环, 全局 BA |
| [[组件-OpenMAVIS]] | g2o BA (SE2(3) 精确预积分) | ORB 特征 (继承 ORB-SLAM3 模式) | SE2(3) 精确 IMU 预积分 | INS-centric 初始化 (惯性为主) | 继承 ORB-SLAM3 的边缘化策略 | SE2(3) Lie 群精确预积分, 避免 SO(3)×R³ 叉乘近似 |

### 1.3 增量平滑 / 因子图类 VIO

| 系统 | 后端 / 估计范式 | 视觉前端 | IMU 使用 | 初始化 | 关键帧 / 边缘化策略 | 核心创新 |
|------|----------------|----------|----------|--------|---------------------|----------|
| [[算法-Kimera-VIO]] | GTSAM iSAM2 FixedLagSmoother | GFTT → KLT 光流 → OpenGV RANSAC 三层验证 | GTSAM 预积分 + BetweenFactor | OnlineGravityAlignment (陀螺偏置+重力切空间) | Fixed-Lag smoother 固定窗口, SmartStereoFactor Schur 消去路标 | SmartStereoFactor + Delaunay Mesh + 语义分割平面正则 |
| [[算法-DM-VIO]] | GTSAM iSAM2 延迟边缘化 | DSO 光度 BA 前端 | GTSAM 预积分因子 | 多延迟图架构: 初始化前大延迟纯视觉 → 成功后升级主图 | 延迟边缘化: 等待 scale/gravity/bias 收敛后再 Schur complement | 延迟边缘化避免错误先验锁定 + BAGTSAMIntegration 桥接 DSO+GTSAM + 动态 DSO 权重 |

### 1.4 直接法 / 半直接法 VIO

| 系统 | 后端 / 估计范式 | 视觉前端 | IMU 使用 | 初始化 | 关键帧 / 边缘化策略 | 核心创新 |
|------|----------------|----------|----------|--------|---------------------|----------|
| [[算法-DSO]] | 滑动窗口 7-DOF 光电 BA (Schur complement 求逆逆深度) | 光度误差 (仿射亮度 a,b 建模) | 无 (纯视觉) | 20+ 种恒速/扰动初始化策略 | Schur complement 消去逆深度, FEJ + 零空间正交化 | 仅优化 1 参数/点 (逆深度), 在线光度标定, 仿射亮度响应 |
| [[算法-SVO-Pro]] | Ceres 滑动窗口 VIO BA | SparseImgAlign 稀疏直接法 + 1D 极线块对齐 | VIO 后端可选 | 深度滤波器 (高斯-均匀贝叶斯) | Reprojector 管理关键帧, 深度滤波器收敛后三角化 | 半直接: 直接法前端 + 重投影后端, 多相机原生支持 |
| [[算法-ESVO]] | 双目 event-based VO | Time-Surface 事件图 + 反深度估计 | 无 | 即时 (事件驱动) | 事件窗口异步 | 事件相机立体视觉 + Time-Surface 表示 |

### 1.5 深度学习类 SLAM

| 系统 | 后端 / 估计范式 | 视觉前端 | IMU 使用 | 初始化 | 关键帧 / 边缘化策略 | 核心创新 |
|------|----------------|----------|----------|--------|---------------------|----------|
| [[算法-DROID-SLAM]] | Dense BA (可微 BA 层, RAFT 光流) | 稠密光流 (RAFT 递归 BA) | 否 | 端到端训练 | Dense BA 层内部隐式边缘化 | 可微 Bundle Adjustment 层, RAFT 光流递归迭代 |
| [[算法-NICE-SLAM]] | 可微渲染 + 分层网格 | 隐式 SDF + 颜色场 | 否 | 逐帧可微跟踪 | 分层网格激活/释放 | 分层神经隐式: 粗网格几何 + 细网格颜色, 支持场景编辑 |
| [[算法-MonoGS]] | 可微 3DGS 渲染 | 3D Gaussian Splatting 渲染 | 否 | 首帧密集初始化 Gaussians | 基于梯度的 Gaussian 增删 | 显式 3DGS 表示替代隐式场, 可微渲染, 城市级实时 |

## 二、技术路线演进

### 第一阶段：滤波时代 (2007–2017)

```
MSCKF (2007)
 ├─ 多状态约束 KF: 滑动窗口位姿 clone + 零空间投影消除特征
 ├─ 核心思想: 状态维度受控, 计算量不随特征数增长
 └─ → OpenVINS (2022): 工程化顶点, ACI²/RK4/离散三种积分, 在线标定
 └─ → MSCKF_VIO: FEJ-OC 保证一致性
 └─ → ROVIO (2015): patch 光度 + EKF, 飞行器场景

相关变体: SchurVINS (2023) — 用 Schur Complement 替代零空间投影,
           使 EKF 约束更接近 BA 结构, 3×3 矩阵求逆替代高维零空间计算
```

**特征**：一次线性化, 协方差递推总结历史, 状态维度受控 (MSCKF 典型 39 DOF), 低延迟。劣势是历史线性化点无法回溯修正。

---

### 第二阶段：滑动窗口 + 批优化时代 (2015–2020)

```
PTAM / SVO (2014) 直接法前端
  └─ → DSO (2016): 稀疏直接法, 每个点仅 1 参数 (逆深度), 在线光度标定
  └─ → SVO Pro (2019): SparseImgAlign + 1D 极线对齐 + 重投影后端, 半直接

OKVIS / VINS (2015–2017) 预积分 + 滑动窗口
  └─ → VINS-Fusion (2019): 单/双目+IMU+GPS, Ceres DENSE_SCHUR, 完整工程

ORB-SLAM (2015) 特征法
  └─ → ORB-SLAM3 (2020): Atlas 多地图 + 6 种传感器 + 三线程并行
```

**特征**：窗口内可反复重线性化, 精度显著优于单次线性化滤波。Schur 补边缘化旧窗口变量为先验, 但先验线性化点固定后无法修正 (与滤波面临相同问题)。

---

### 第三阶段：增量平滑 / iSAM2 时代 (2019–)

```
GTSAM iSAM2 (2012 理论, 2019+ 工程爆发)
 ├─ Kimera-VIO (2019): SmartStereoFactor + FixedLagSmoother + Mesh
 ├─ DM-VIO (2022): 延迟边缘化, 等待参数收敛后再消元
 └─ LIO-SAM / FAST-LIO-SAM (LiDAR 侧, 同样受益)
```

**特征**：Bayes 树增量更新, 受影响变量可选择重线性化 (不自动固定所有历史线性化点), 因子化扩展好, 多传感器统一为因子。DM-VIO 的延迟边缘化是对 sliding-window 先验固定问题的关键缓解。

---

### 第四阶段：深度学习 SLAM 时代 (2020–)

```
DROID-SLAM (2021): 可微 BA 层 + RAFT 光流, 端到端可微 dense BA
NICE-SLAM (2022): 分层神经隐式 SDF + 颜色, 可微渲染 + 场景编辑
MonoGS (2023): 3D Gaussian Splatting 替代隐式场, 显式椭球体 + 可微光栅化
```

**特征**：表示能力跳变 (隐式场/3DGS 替代手工特征+点云+网格), 可微性贯穿全管线 (跟踪+建图+回环统一在梯度优化中)。但 GPU 依赖强, 部署/调试/长距离鲁棒性仍待验证。

---

### 演进规律

| 代际 | 后端 | 视觉观测 | 线性化 | 历史修正 | 表示 |
|------|------|----------|--------|----------|------|
| 滤波 (MSCKF) | EKF | 描述子/KLT 特征 + 零空间投影 | 一次, 不可逆 | 协方差压缩 (不可回溯) | 稀疏点 |
| 滑动窗口 (VINS) | Ceres/g2o | KLT/重投影/光度 | 窗口内反复, 先验固定 | 先验固定不可修正 | 稀疏/半稠密点 |
| 增量平滑 (iSAM2) | GTSAM | 各种因子 | 选择性重线性化 | 可逆边缘化 (Bayes 树) | 因子化地图 |
| 深度学习 | PyTorch/TensorFlow | 光流/隐式场/3DGS | 可微梯度 | 全部变量可重优化 | 隐式/3DGS |

## 三、关键维度深度对比

### 3.1 视觉前端

| 前端类型 | 代表系统 | 特征类型 | 跟踪方式 | 适用场景 | 核心风险 |
|----------|----------|----------|----------|----------|----------|
| 稀疏角点 + KLT | VINS-Fusion, [[算法-OpenVINS]], [[算法-Kimera-VIO]] | Shi-Tomasi / GFTT / FAST | KLT 金字塔光流, 描述子可选 | 光照变化, 长距离, 回环 (描述子) | 低纹理区域跟踪失败 |
| ORB 特征 | [[算法-ORB-SLAM3]], [[组件-OpenMAVIS]] | FAST + Steered BRIEF | ORB 匹配 + BoW 回环 | 描述子回环, 多地图, 重定位 | 运动模糊/低纹理性能差 |
| 稀疏直接法 | [[算法-DSO]], [[算法-DM-VIO]] | 高梯度像素 | 光度误差最小化 (仿射亮度 a,b) | 弱纹理连续区域, 有稳定光度模型 | 曝光/AGC/光照变化敏感 |
| 半直接法 | [[算法-SVO-Pro]] | SparseImgAlign 直接法 + 特征块 | 直接法前端 + 1D 极线对齐 | 快前端, 低计算 | 回环和全局一致性弱 |
| 图像块光度 | [[算法-ROVIO]] | 4 层 patch | IEKF + patch 光度 | 飞行器关键区域 | patch 生命周期和 IMU 噪声耦合 |
| 稠密/可微 | [[算法-DROID-SLAM]], [[算法-NICE-SLAM]], [[算法-MonoGS]] | RAFT 光流 / SDF / 3DGS | 可微渲染 / Dense BA 层 | GPU 可用, 需要稠密或渲染 | GPU 依赖, 长距离鲁棒性, 部署复杂 |

### 3.2 图像预处理要求 (按前端类型)

| 系统族 | 预处理重点 | 输入契约 |
|--------|------------|----------|
| KLT/角点 VIO | 灰度化, CLAHE/均衡化 (可选), KLT 金字塔, mask/grid | 灰度 8-bit, 畸变参数, 内参 |
| ORB/描述子 SLAM | rectify/resize, ORB pyramid, FAST 阈值 | RGB/Gray, 双目校正, 内参 |
| 直接法/半直接法 | response, vignette, affine brightness, patch pyramid | RAW/Bayer 优先, 线性响应, 标定 photometric calib |
| 学习式/稠密 | RGB/BGR, `/255`, mean/std, crop/resize + 内参更新 | 匹配训练分布, float 归一化 |

完整预处理链路分析见 [[图像预处理与观测模型]]。

### 3.3 IMU 链路的三个层次

所有 VIO 系统都涉及 IMU, 但集成深度分三级：

| 层次 | 描述 | 代表系统 |
|------|------|----------|
| **传播层** (高频状态预测) | IMU 直接驱动 EKF 预测步, 200 Hz 以上 | [[算法-OpenVINS]], [[概念-MSCKF\|MSCKF_VIO]], [[算法-SchurVINS]], [[算法-ROVIO]] |
| **预积分层** (关键帧间压缩) | IMU 测量压缩为关键帧间运动约束, 10-30 Hz 进入优化/图 | VINS-Fusion, [[算法-DM-VIO]], [[算法-Kimera-VIO]], [[算法-ORB-SLAM3]] |
| **预处理层** (滤波/标定/补偿) | 进入估计器前的信号质量保障 | 所有系统, 详见 [[传感器-IMU预处理]] |

### 3.4 初始化策略

| 系统 | 初始化方法 | 所需条件 | 鲁棒性 |
|------|-----------|----------|--------|
| [[算法-OpenVINS]] | 静止 (ZUPT) + 动态异步 | 静止 1-2 s 或充足视觉 | 高, 模块化, 失败可降级 |
| VINS-Fusion | Mono: SFM + IMU-visual 对齐; Stereo: PnP + 陀螺估计 | 充分平移+旋转激励 | 需要视觉纹理 |
| [[算法-ORB-SLAM3]] | 视觉初始化 → 10 s 后 IMU 联合初始化 | 前 10 s 需要丰富视觉 | 单目弱纹理脆弱 |
| [[组件-OpenMAVIS]] | INS-centric: IMU 先验 → 视觉辅助 | 高质量 IMU | 高, 以惯性为主 |
| [[算法-Kimera-VIO]] | OnlineGravityAlignment (陀螺偏置 + 重力切空间) | 双目 + 初始运动 | 中高 |
| [[算法-DM-VIO]] | 多延迟图架构, 参数收敛后升级 | 充足视觉 + IMU | 高, 延迟边缘化降低错误先验 |
| [[算法-DSO]] | 20+ 种恒速/扰动初始化策略 | 充分视觉运动 | 无 IMU, 完全依赖视觉 |

### 3.5 边缘化策略评估

| 边缘化方法 | 系统 | 优点 | 关键风险 |
|------------|------|------|----------|
| 零空间投影 (MSCKF) | [[算法-OpenVINS]], [[概念-MSCKF\|MSCKF_VIO]] | 特征变量不进入状态, 维度低 | 线性化点一次, 不可修正 |
| Schur Complement EKF | [[算法-SchurVINS]] | 比零空间投影更接近 BA, 3×3 矩阵求逆快 | EKF 框架线性化限制 |
| Schur 补 + 滑动窗口 (固定先验) | VINS-Fusion | 窗口内重线性化, 精度高 | 边缘化先验固定后不可逆 |
| 延迟边缘化 | [[算法-DM-VIO]] | 等待参数收敛, 避免错误先验锁定 | 架构复杂, 多图状态管理 |
| Fixed-Lag Smoother + Smart Factor | [[算法-Kimera-VIO]] | SmartStereoFactor 隐式消路标, 窗口管理自动 | 固定滞后可能丢失长期约束 |
| ISAM2 Bayes 树 (可逆) | [[算法-DM-VIO]], [[算法-Kimera-VIO]] | 受影响变量可重线性化, 可逆 | 图结构管理和调参复杂 |

### 3.6 回环与全局一致性

| 系统 | 回环检测 | 回环约束 | 全局校正 |
|------|----------|----------|----------|
| VINS-Fusion | DBoW2 + BRIEF | 4-DOF PGO | 位姿图重优化 |
| [[算法-ORB-SLAM3]] | DBoW2 + ORB 描述子 | Sim3 + Essential Graph | Full BA / Full Inertial BA |
| [[算法-Kimera-VIO]] | Kimera-RPGO (外部) | 位姿图因子 | RPGO 增量 PGO |
| [[算法-DSO]] / [[算法-SVO-Pro]] / [[算法-DM-VIO]] | 无内建 | — | 纯里程计, 无全局一致性 |
| [[算法-OpenVINS]] / [[概念-MSCKF\|MSCKF_VIO]] | 无内建 | — | 纯里程计 |
| 深度学习类 | 多数无内建 | — | 依赖可微回环 (研究阶段) |

## 四、场景推荐

### 4.1 按应用场景

| 场景 | 首选参考 | 理由 |
|------|----------|------|
| **轻量实时 VIO 前端** | [[算法-OpenVINS]] | MSCKF 模块边界清楚, 状态维度受控, 在线标定, 适合抽象成多传感器前端 |
| **完整开源 VIO 基线** | VINS-Fusion | 滑窗 BA + 预积分 + 边缘化 + GPS + 回环, 工程覆盖最完整 |
| **需要回环 / 重定位 / 多地图** | [[算法-ORB-SLAM3]] | Atlas 多地图 + BoW + 全局 BA, 6 种传感器模式 |
| **因子图融合研究** | [[算法-Kimera-VIO]], [[算法-DM-VIO]] | GTSAM iSAM2 + SmartFactor + 延迟边缘化, 对多传感器因子图有启发 |
| **弱纹理环境** | [[算法-DSO]], [[算法-DM-VIO]] | 直接法不依赖角点, 光度残差在弱纹理区域仍有信号 |
| **无人机 / 飞行器** | [[算法-ROVIO]], [[算法-SVO-Pro]] | ROVIO 的 patch 光度 + EKF 适合快速旋转; SVO Pro 前端极轻 |
| **事件相机** | [[算法-ESVO]] | Time-Surface 事件图表示, 高速运动+高动态范围 |
| **稠密重建** | [[算法-DROID-SLAM]], [[算法-NICE-SLAM]] | 稠密 BA / 神经隐式, GPU 驱动重建 |
| **实时 3D 渲染 SLAM** | [[算法-MonoGS]] | 3DGS 可微光栅化, 城市级新视角合成 |
| **GPS + VIO 户外** | VINS-Fusion | 全球融合松耦合, GeographicLib ENU 坐标转换 |

### 4.2 按工程约束

| 约束 | 推荐 | 理由 |
|------|------|------|
| **无 IMU (纯视觉)** | [[算法-DSO]], [[算法-SVO-Pro]], [[算法-ORB-SLAM3\|ORB-SLAM3 (Mono)]] | 纯视觉 SLAM, 不需要 IMU |
| **最小依赖** | [[算法-DSO]] | 无 GTSAM/Ceres 重型依赖, 自研求解器 |
| **需在线标定 (外参/时偏)** | [[算法-OpenVINS]] | 内建 IMU-Cam 外参 + 时间偏移在线标定 |
| **多相机** | [[算法-SVO-Pro]], [[算法-OpenVINS]] | CameraRig/FrameBundle 原生多相机支持 |
| **GPU 可用** | [[算法-DROID-SLAM]], [[算法-NICE-SLAM]], [[算法-MonoGS]] | 深度学习 / 可微渲染, GPU 加速 |
| **CPU 受限嵌入端** | [[算法-OpenVINS]] (MSCKF) | MSCKF 计算量最小, 状态维度固定 |
| **不依赖 ROS** | [[算法-DSO]], [[算法-OpenVINS]] (ROS-Free 核) | 核可独立编译 |

### 4.3 按学习 / 参考价值

| 学习目标 | 参考系统 | 可取的设计模式 |
|----------|----------|---------------|
| 滤波 VIO 模块化 | [[算法-OpenVINS]] | CameraBase 虚基类 + TrackBase 多相机抽象 |
| 预积分 + 滑窗工程 | VINS-Fusion | 中值预积分 + Schur 边缘化 + GPS 松耦合 |
| 增量因子图 API | [[算法-Kimera-VIO]], [[算法-DM-VIO]] | SmartStereoFactor + FixedLagSmoother + 延迟边缘化 |
| 回环与多地图 | [[算法-ORB-SLAM3]] | Atlas 多地图 + 多级 BA + 三线程异步 |
| 直接法实现 | [[算法-DSO]] | 仅 1 参数/点的逆深度参数化 + 仿射亮度建模 |
| IMU 精确预积分 | [[组件-OpenMAVIS]] | SE2(3) Lie 群精确预积分 (避免 SO(3)×R³ 叉乘近似) |
| 深度学习 + BA | [[算法-DROID-SLAM]] | 可微 BA 层, RAFT 光流递归 |

## 五、系统详细技术剖解

### 5.1 OpenVINS：滤波 VIO 的工程化顶点

OpenVINS (v2.7) 是 MSCKF 滤波范式的工程化巅峰。核心链路：IMU 高频 EKF 传播 (200+ Hz) → 位姿 clone (augment_clone) → MSCKF 更新 (零空间投影 + Chi² 检验) → EKF 更新 → 边缘化老 clone。三种 IMU 积分模式可选：离散积分 (最快)、RK4 (平衡)、ACI² 解析 (最高精度)。

特征处理 100% 基于 OpenCV：FAST 角点 + KLT 光流 (TrackKLT) 或 ORB 描述子匹配 (TrackDescriptor)。在线标定系统 (IMU-Cam 外参、时偏) 模块化。CameraBase 虚基类和 TrackBase 多相机抽象使其天然支持多目。

**局限**：MSCKF 的零空间投影仍是一次线性化，EKF 无法回溯修正历史线性化错误；无内建回环检测，纯 CPU。

### 5.2 VINS-Fusion：滑动窗口工程最完整

VINS-Fusion 是当前工程覆盖最完整的开源 VIO 系统。前端：Shi-Tomasi 角点 + KLT 金字塔光流 (goodFeaturesToTrack + calcOpticalFlowPyrLK)，利用 Estimator 传入的预测 3D 点作为 LK 初始猜测，加速显著。后端：Ceres DENSE_SCHUR + DogLeg，三种初始化模式 (Mono+IMU/Stereo+IMU/Stereo only)。GPS 全球融合用 GeographicLib 做局部 ENU 坐标转换。回环检测用 DBoW2 + BRIEF 词袋，PnP RANSAC 几何验证，4-DOF PGO 全局优化。

**IMU 细节**：中值预积分 (mid-point, RK2)，含完整 Jacobian/协方差传播和偏置重传播 (bias change 时用一阶近似修正)。边缘化用 Schur 补保留历史信息，MARGIN_OLD / MARGIN_SECOND_NEW 两种策略依据视差和跟踪质量动态选择。

**临界点**：边缘化先验一旦固定 (通过常数 Jacobian 锁定)，后续无法修正，窗口大小对精度/性能 tradeoff 敏感 (~10 帧)。

### 5.3 ORB-SLAM3：全局 BA + Atlas 多地图

ORB-SLAM3 是视觉 SLAM 中功能最全的系统：6 种传感器模式 (Mono/Stereo/RGBD + 各三种 IMU 组合)，三线程异步并行 (Tracking/LocalMapping/LoopClosing)。其 Atlas 多地图系统是它区别于其他 VIO 系统的核心能力：跟踪丢失自动创建新地图（不丢弃旧数据），通过 DBoW2 Common Regions 检测自动合并地图。

多级 BA 体系按精度和计算量分层：Motion-only BA (6DoF 位姿优化, 毫秒级) → Local BA (局部窗口) → Local Inertial BA → Essential Graph → Full BA → Full Inertial BA。ORB 特征使用 FAST 双阈值角点 (iniThFAST=20/minThFAST=7) + 四叉树分布 (DistributeOctTree) + IC_Angle 灰度质心方向 + Steered BRIEF 32 字节描述子。

**局限**：ORB 特征在低纹理/运动模糊下跟踪失败 (FAST 角点失效率高)；内存占用大 (多地图 + 全部关键帧 + 路标)；单目初始化脆弱。

### 5.4 Kimera-VIO：因子图 VIO + Mesh 重建

Kimera-VIO (MIT-SPARK) 是 GTSAM iSAM2 的视觉 SLAM 最佳实践。PipelineModule 架构实现 SISO/SIMO 纯函数式接口，多线程数据流。前端经过三层几何验证：GFTT (Shi-Tomasi) 检测 → KLT 光流跟踪 → OpenGV RANSAC 五点/三点法 RANSAC。后端用 FixedLagSmoother (iSAM2 固定滞后平滑)，视觉因子使用 SmartStereoFactor —— GTSAM 的 SmartStereoProjectionPoseFactor 内部用 Schur 补消去路标变量。

额外能力：OnlineGravityAlignment 初始化 (陀螺偏置+重力切空间)、在线 IMU-Camera 时间对齐、Delaunay Mesh 重建 (基于左右关键帧 DLT 三角化 + 平面分割 + PointPlaneFactor 正则)。

**工程注意**：feature_tracks_ 无限增长需窗口裁剪；iSAM2 的 fixed-lag 平滑中旧变量边缘化仍会固定线性化点。

### 5.5 DM-VIO：延迟边缘化的关键贡献

DM-VIO 的**延迟边缘化**是 VIO 边缘化策略的重大改进。核心洞察：传统边缘化在 scale/gravity/bias 尚未收敛时就将旧变量固化为先验，一旦先验错误，系统被永久污染。DM-VIO 的答案是：等待关键参数收敛后再执行 Schur complement。

实现上使用多延迟图架构：初始化前用大延迟纯视觉图 (主图不边缘化)，待 scale/gravity/bias 收敛后，将旧视觉帧连同 IMU 因子一次性升级进主图，此时边缘化更安全。动态 DSO 权重：光度 RMSE 上升时自动降低视觉权重，系统更依赖 IMU。BAGTSAMIntegration 桥接 DSO 视觉 Hessian (自研求解器) 和 GTSAM IMU 因子 (iSAM2)，Hessian 直接相加融合。

**局限**：架构和调参复杂度高 (延迟图数量 + 延迟帧数 + 动态权重参数)。

### 5.6 DSO：直接法的极致表达

DSO (TUM) 是稀疏直接法的标杆。核心设计哲学是"每个 3D 点只优化 1 个参数 (逆深度)"：Schur complement 消去逆深度几乎零开销 (标量求逆)，使得 Dense 光度 BA 的计算量仍可控。

显式建模帧间仿射亮度参数 (a,b)：`r = (I_ref - b) - eᵃ (I_target - b)`，适应相机曝光/增益漂移。滑动窗口中手动实现 FEJ (First Estimate Jacobian) + 零空间正交化，处理 7 DOF 不可观测性 (6DoF 相似变换 + 尺度)。粗跟踪用金字塔 LM 优化 8-DOF (6DoF 位姿 + 2 仿射参数)，20+ 种初始化策略 (多种恒速/扰动/匀速模型) 确保位姿跟踪不丢失。

**局限**：需要线性相机响应 (RAW/Bayer)、无内建回环、纯视觉无 IMU 导致长距离尺度漂移不可避免。

### 5.7 SVO Pro：半直接法的工程实践

SVO Pro (UZH RPG) 采用"直接法前端 + 重投影后端"的半直接法：SparseImgAlign 用上一帧全部 3D 点通过光度误差直接估计位姿 (无需 2D-2D 特征匹配)；1D 极线块对齐用逆组合法沿极线精化特征位置，Hessian 只计算一次；后端 PoseOptimizer 用重投影误差 BA (三种误差类型：单位球面/归一化平面/像素平面) + Ceres 滑动窗口 VIO BA。

CameraBundle + FrameBundle 原生多相机支持，每个相机独立 Reprojector 管理关键帧和深度滤波器。深度滤波器用高斯-均匀贝叶斯混合物估计每个特征的逆深度。

**局限**：SparseImgAlign 对大旋转/快速旋转可能失败 (直接法假设帧间小位移)；回环和全局一致性弱。

### 5.8 深度学习 VIO/VO：端到端可微

**DROID-SLAM**：用 RAFT 光流构建可微 BA 层，递归迭代光流 → 光流残差 → Dense BA 更新 → 修正光流 → ...，循环中更新 pose+深度。端到端训练的稠密 BA 在特征缺失、反射、薄结构等传统方法失败场景表现极好，但 GPU 推理性能高度依赖硬件。

**NICE-SLAM**：分层神经隐式表示 —— 粗网格存储几何 (SDF)，细网格存储颜色，可微渲染实现端到端 tracking+mapping。支持场景编辑 (移除/添加对象后重新渲染)。在线 5 Hz tracking。

**MonoGS**：3D Gaussian Splatting 替代隐式场作为地图表示，显式各向异性椭球体光栅化可微，城市级实时新视角合成质量极高。tracking 用梯度下降在 3DGS 地图中优化当前帧位姿，mapping 按梯度增删 Gaussians。

## 六、典型系统性能对标

| 系统 | 室内 ATE RMSE | 室外 ATE RMSE | 单帧耗时 | 回环 | 表示 |
|------|--------------|--------------|----------|------|------|
| [[算法-OpenVINS]] | 0.02-0.1 m | — | ~5 ms | 无 | 稀疏点 |
| VINS-Fusion | 0.02-0.08 m | 0.1-0.5 m | ~20 ms | DBoW2+PGO | 稀疏路标点 |
| [[算法-ORB-SLAM3]] | 0.01-0.05 m | 0.05-0.3 m | ~30 ms | DBoW2+Atlas+Full BA | 稀疏路标点 + 关键帧 |
| [[算法-Kimera-VIO]] | 0.01-0.04 m | 0.1-0.4 m | ~40 ms | Kimera-RPGO | 路标 + 3D Mesh |
| [[算法-DM-VIO]] | 0.01-0.03 m | 0.05-0.3 m | ~30 ms | 无 | 稀疏点 (光度) |
| [[算法-DSO]] | 0.02-0.1 m | 0.3-2.0 m | ~10 ms | 无 | 高梯度点 + 逆深度 |
| [[算法-SVO-Pro]] | 0.02-0.08 m | 0.2-1.0 m | ~5 ms | 无 | 稀疏路标点 |
| [[算法-DROID-SLAM]] | 0.01-0.03 m | 0.05-0.3 m | ~50 ms (GPU) | 可微回环 | Dense 深度 |
| [[算法-NICE-SLAM]] | 0.02-0.05 m | — | ~200 ms (GPU) | 隐式 | SDF + 颜色场 |
| [[算法-MonoGS]] | 0.01-0.03 m | 0.05-0.2 m | ~100 ms (GPU) | 无 | 3DGS 椭球体 |

> 注：以上为各系统论文/社区报告的典型值范围，实际性能取决于传感器、场景纹理和光照条件。GPU 方法的延迟和功耗不直接可比。

## 七、跨范式对比：何时选滤波，何时选优化

除了本页的系统对比，需理解更深层的范式差异。相关分析见 [[因子图vs滤波]]。以下是从 VIO 视角的补充：

| 维度 | EKF/MSCKF 滤波 | 滑动窗口 BA | iSAM2 增量平滑 |
|------|---------------|------------|----------------|
| 延迟 | 最低 (~1 ms) | 中 (~10 ms) | 中低 (~5 ms) |
| 精度 | 中 (单次线性化) | 高 (重线性化) | 接近批量 (选择性重线性化) |
| 新传感器扩展 | 需推导耦合雅可比 | 新增残差块 | 添加因子 (最灵活) |
| 线性化质量保障 | FEJ / OC | 重线性化但先验固定 | 受影响变量可重线性化 |
| 多传感器融合 | 状态维度爆炸 | 残差块管理复杂 | 统一因子图 |

对"需要 LIO + VIO + GNSS 统一融合"的项目，iSAM2 是最自然的选择 (phad_fusion 的设计证明了这一点)。

## 八、VIO 的常见陷阱

1. **IMU 噪声单位错误**：GTSAM 预积分参数期望的是连续白噪声方差 (gyro/accel noise density)，而非离散方差。错误单位导致预积分因子权重差一个数量级。

2. **重力方向混用**：VINS-Fusion 用 ENU (Z-up)，ORB-SLAM3 IMU 用 World-frame gravity，GTSAM 有 `MakeSharedU` (ENU) 和 `MakeSharedD` (NED) 两种。不统一会导致初始化计算的重力、尺度和偏置全错。

3. **图像预处理破坏光度模型**：直接法/半直接法 (DSO/DM-VIO/SVO Pro/SVO) 的残差基于光度不变假设。对输入做 CLAHE、gamma 校正、曝光补偿等预处理会破坏此假设。详见 [[图像预处理与观测模型]]。

4. **边缘化先验的固定线性化点陷阱**：滑窗边缘化或 iSAM2 fixed-lag smoothing 中，边缘化后先验的线性化点被固化为当前状态估计。如果此时 scale/gravity/bias 尚未收敛，这个错误将永远留在系统中。DM-VIO 的延迟边缘化正是为此设计。

5. **回环的 Sim3 到 SE3 跃迁**：ORB-SLAM3 的回环使用 Sim3 (7-DOF 相似变换) 连接地图，但因子图中多数约束是 SE3。Sim3 桥接后的尺度修正传播到所有关键帧和地图点，处理不当会导致地图几何畸变。

## 九、设计原则

1. **视觉前端、IMU 链路、后端优化应当分层独立设计**。不应直接照搬一个完整 VIO 系统, 而应拆出可替换的模块。OpenVINS 的模块化 + VINS-Fusion 的 IMU 预积分 + ORB-SLAM3 的回环管理 + DM-VIO 的边缘化策略, 各自提供了不同层次的参考。

2. **初始化是 VIO 最脆弱环节**。ORB-SLAM3 需要前 10 s 丰富视觉; VINS-Fusion 需要充足平移激励; DM-VIO 用延迟边缘化缓解错误初始化锁定; OpenVINS 用静止 ZUPT 保底。设计时应准备多策略降级机制。

3. **边缘化必须显式管理**。无论是滤波的零空间投影、滑窗的 Schur 补还是 iSAM2 的 fixed-lag smoothing, 旧变量一旦边缘化, 其线性化点可能被永久固化为先验。DM-VIO 的延迟边缘化、iSAM2 的选择性重线性化是缓解此问题的工程方向。

4. **图像预处理与视觉观测模型不可分离设计**。直接法对 CLAHE/vignette 敏感, KLT 对灰度均衡化受益, ORB 对 resize/rectify 敏感。预处理参数变化会改变残差假设, 必须在视觉因子中一同标定或补偿。

5. **因子图是通向多传感器融合的最通用路径**。无论是 LiDAR ICP、视觉重投影、IMU 预积分、GNSS 先验, 在 iSAM2 因子图中平等表达, 避免为每种传感器组合重新推导耦合雅可比。

## 十、未来方向

- **SE2(3) 精确预积分**：OpenMAVIS 采用的 SE2(3) Lie 群预积分消除了传统 SO(3)×R³ 积分的叉乘近似误差。
- **延迟边缘化泛化**：DM-VIO 的延迟边缘化思路可推广到任何"等参数收敛再消元"的场景。
- **光度 + 几何联合**：FAST-LIVO2 统一用 IESKF 融合直接法亮度误差和 LiDAR 几何误差, 视觉可接入点云位姿的协方差。
- **深度学习与传统融合**：可微 BA (DROID-SLAM) 与因子图后端 (GTSAM) 结合的趋势, 在学习前端 + 图优化后端的混合架构中可能产生更鲁棒的系统。
- **神经隐式 / 3DGS 后端**：NICE-SLAM / MonoGS 证明隐式场/3DGS 可作为地图表示, 未来可能接入因子图作为全局约束。

## 相关页面

- [[VIO方案对比]]
- [[VIO方案全景对比]]
- [[因子图vs滤波]]
- [[图像预处理与观测模型]]
- [[相机数据管线]]
- [[IMU数据管线]]
- [[传感器-IMU预处理]]
- [[概念-IMU预积分]]
- [[概念-直接法视觉里程计]]
- [[概念-深度学习SLAM]]
- [[概念-视觉惯性初始化策略]]
- [[概念-Schur补与边缘化]]
- [[概念-直接法vs间接法]]
- [[phad_fusion设计总结]]
- [[组件-Ceres-Solver]]
- [[组件-GTSAM]]
- [[优化后端选型指南]]
