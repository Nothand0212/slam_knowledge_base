# 论文与资料索引

本地 PDF 根目录：[papers/](papers/)。闭源或需订阅的文献仅记录 DOI / 链接。

## open_vins

- **文献**：OpenVINS: A Research Platform for Visual-Inertial Estimation (ICRA 2020). 本地：`papers/open_vins/openvins_icra2020.pdf`
- **创新点（提炼）**：
  - 开源 MSCKF 类滑动窗口 VIO 平台，覆盖单目/双目/多相机与可选 SLAM 模式。
  - 相机畸变、时间偏移、IMU 内参等在线标定入口统一。
  - 以「可复现研究平台」为目标，模块边界（状态、特征、更新）清晰。

## SchurVINS

- **文献**：SchurVINS: Schur Complement-Based Lightweight Visual Inertial Navigation System (CVPR 2024). 本地：`papers/SchurVINS/schurvins_cvpr2024.pdf`
- **创新点**：
  - 将高维视觉约束通过 Schur 补等价为低维梯度/Hessian 形式，减轻 EKF 更新负担。
  - 面向资源受限平台的轻量 VINS 精度与速度权衡。

## msckf_vio

- **文献**：与仓库相关的 MSCKF 立体视觉惯性里程计论文（arXiv:1712.00036）。本地：`papers/msckf_vio/msckf_stereo_vio.pdf`
- **创新点**：
  - 经典 MSCKF 管线在立体+IMU 上的实现与实时飞行导向设计。
  - 多状态约束、特征剔除与零空间投影等 MSCKF 标准要素的工程化参考。

## OpenMAVIS

- **文献**：MAVIS: Multi-Camera Augmented Visual-Inertial SLAM using SE2(3) Based Exact IMU Pre-integration (ICRA 2024). 本地：`papers/OpenMAVIS/mavis_icra2024.pdf`
- **创新点**：
  - 多相机与 SE2(3) 上精确 IMU 预积分结合，强化视觉-惯性一致性。
  - 与 ORB-SLAM3 系架构结合的多目 VIO/SLAM 演示（本仓库为再实现版）。

## IC-GVINS

- **文献**：IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System for Wheeled Robot (IEEE RA-L 2022). arXiv:2204.04962。本地：`papers/ic_gvins/ic_gvins.pdf`
- **创新点**：
  - 以 INS 为中心的 GNSS/视觉/惯性紧耦合，强调鲁棒性与实时性。
  - 因子图框架下多源融合，适合作为 GNSS-VIO 后端参考。

## OB_GINS

- **文献**：多篇（IMU 预积分与地球自转补偿、GNSS/VINS 等）。见 [OB_GINS/README.md](../OB_GINS/README.md)。
- **本地 PDF**：无（期刊站点自行下载）。
- **创新点**：
  - 优化型 GNSS/INS/里程计等组合导航工程实现。
  - 对预积分中地球自转/重力变化等细节的建模可参考。

## fusions_slam

- **文献**：无单一主论文；工程说明见 [fusions_slam/readme](../fusions_slam/readme)。理论可对照 FAST-LIO 系列。
- **创新点（工程）**：
  - 在 FAST-LIO 思路上增加位置/姿态/速度等观测与建图、先验地图定位。
  - `wrapper/ros` 与算法核分层，便于对照 ROS 耦合度。

## FAST-LIO-SAM-SC-QN

- **文献**：组合型 — FAST-LIO2、Scan Context、LIO-SAM、Quatro、Nano-GICP 等（见项目 README 链接）。
- **本地 PDF**：未集中存放；可按 README 逐篇获取。
- **创新点（本仓库工程）**：
  - 将 LIO 与 Scan Context 回环、Quatro 粗配准、Nano-GICP 等模块拼接为可学习 GTSAM 教程向工程。
  - PGO 模块化，便于替换前端 LIO。

## lightning-lm

- **文献**：以 README 功能描述为主；无单一 arXiv 主论文标注。
- **创新点（工程）**：
  - 完整激光建图与定位、动态图层、地图分区加载、轻量增量优化器 miao。
  - ROS 2 集成；强调单核 CPU 性能。

## genz-icp

- **文献**：GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry (RA-L). 本地：`papers/genz-icp/genz_icp_ral.pdf`
- **创新点**：
  - 自适应权重提升跨场景泛化与退化鲁棒性。
  - 学习式/传统 ICP 结合的 LiDAR 里程计思路（以论文为准）。

## gtsam_points

- **文献**：库说明与引用见 [gtsam_points/README.md](../gtsam_points/README.md)；Zenodo DOI: 10.5281/zenodo.13378351。
- **本地 PDF**：未放 Zenodo 快照；需要时可从 Zenodo 下载 release 归档。
- **创新点**：
  - 在 GTSAM 上扩展点云配准、体素、连续时间轨迹等因子与工具。
  - 适合作为「点云 + 因子图」扩展参考，而非单一 SLAM 论文。

## lt-mapper

- **文献**：LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping (ICRA). 本地：`papers/lt-mapper/ltmapper_icra2022.pdf`（复制自 `lt-mapper/doc/ltmapper.pdf`）。
- **创新点**：
  - 多会话激光建图、处理场景随时间变化（含 Removert 等模块协同）。
  - 会话间对齐与全局一致性维护。

## BEV-LSLAM

- **文献**：IEEE RA-L（DOI 见 [BEV-LSLAM/README.md](../BEV-LSLAM/README.md)）。
- **本地 PDF**：无（IEEE）。
- **创新点**：BEV 表征与激光 SLAM 结合（细节以论文为准）。

## ROLO

- **文献**：ROLO-SLAM: Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle (JFR 2025). arXiv:2501.02166。本地：`papers/rolo/rolo.pdf`
- **创新点**：面向崎岖地面的激光 SLAM，前向位置预测、体素匹配与旋转估计、运动约束优化等减轻垂向漂移。

## ORB-SLAM3

- **文献**：ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM (TRO 2021). arXiv:2007.11898。本地：`papers/orb_slam3/orb_slam3.pdf`
- **创新点**：基于 ORB 特征的多地图系统，支持单目/双目/RGB-D/视觉惯性模式，Atlas 多地图管理与无缝重定位。

## DSO

- **文献**：Direct Sparse Odometry (TPAMI 2018). arXiv:1607.02565。本地：`papers/dso/dso.pdf`
- **创新点**：直接法稀疏视觉里程计，联合优化光度误差与几何参数，无需特征提取。

## VINS-Mono

- **文献**：VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator (TRO 2018). arXiv:1708.03852。本地：`papers/vins_fusion/vins_mono.pdf`
- **创新点**：紧耦合单目 VIO，滑动窗口优化 + 回环检测与全局位姿图优化。VINS-Fusion 为其多传感器扩展，无独立论文。

## LIO-SAM

- **文献**：LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping (IROS 2020). arXiv:2007.00258。本地：`papers/lio_sam/lio_sam.pdf`
- **创新点**：紧耦合激光-惯性里程计，因子图融合 IMU 预积分、激光里程计、GPS 与回环因子。

## FAST-LIO2

- **文献**：FAST-LIO2: Fast Direct LiDAR-inertial Odometry (TRO 2022). arXiv:2107.06829。本地：`papers/fast_lio_sam/fast_lio2.pdf`
- **创新点**：直接法激光-惯性里程计，ikd-Tree 增量 kd 树高效管理点云，无需特征提取的快速配准。

## R3LIVE

- **文献**：R3LIVE++: A Robust, Real-time, Radiance reconstruction package with a tightly-coupled LiDAR-Inertial-Visual state Estimator (arXiv 2022). arXiv:2202.06668。本地：`papers/r3live/r3live.pdf`
- **创新点**：紧耦合激光-惯性-视觉融合，实时 RGB 着色三维重建与高精度状态估计。

## Kimera-VIO

- **文献**：Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping (ICRA 2020). arXiv:1910.02490。本地：`papers/kimera_vio/kimera.pdf`
- **创新点**：度量-语义 SLAM 管线，集成 VIO、鲁棒位姿图优化与语义 3D 网格重建。

## LeGO-LOAM

- **文献**：LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain (IROS 2018). 本地：`papers/lego_loam/lego_loam.pdf`
- **创新点**：轻量地面优化激光 SLAM，分割地面/非地面点云，双步骤 Levenberg-Marquardt 优化。

## KISS-ICP

- **文献**：KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Registration If Done the Right Way (RA-L 2023). arXiv:2209.15397。本地：`papers/kiss_icp/kiss_icp.pdf`
- **创新点**：极简 ICP 激光里程计，自适应阈值、运动补偿与纯 point-to-point 配准即可达到 SOTA。

## DROID-SLAM

- **文献**：DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras (NeurIPS 2021). arXiv:2108.10869。本地：`papers/droid_slam/droid_slam.pdf`
- **创新点**：基于 RAFT 光流的深度学习 SLAM，Bundle Adjustment 迭代更新相机位姿与深度，无需训练即泛化。

## NICE-SLAM

- **文献**：NICE-SLAM: Neural Implicit Scalable Encoding for SLAM (CVPR 2022). arXiv:2112.12130。本地：`papers/nice_slam/nice_slam.pdf`
- **创新点**：层次化神经隐式 SLAM，结合可学习的网格特征编码与预训练解码器，多层级场景表示。

## FAST-LIVO2

- **文献**：FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry (TRO 2024). arXiv:2405.02097。本地：`papers/fast_livo2/fast_livo2.pdf`
- **创新点**：高效直接法激光-惯性-视觉里程计，统一优化框架处理 LiDAR 与视觉观测。

## LVI-SAM

- **文献**：LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping (ICRA 2021). arXiv:2104.10831。本地：`papers/lvi_sam/lvi_sam.pdf`
- **创新点**：激光-视觉-惯性紧耦合 SLAM，VIS 与 LIS 两个子系统互补，退化场景下相互辅助。

## CT-ICP

- **文献**：CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure (ICRA 2022). arXiv:2109.12979。本地：`papers/ct_icp/ct_icp.pdf`
- **创新点**：弹性扫描配准，连续时间轨迹表示解决扫描内运动畸变，结合回环检测。

## DM-VIO

- **文献**：DM-VIO: Delayed Marginalization Visual-Inertial Odometry (RA-L 2022). arXiv:2201.04114。本地：`papers/dm_vio/dm_vio.pdf`
- **创新点**：延迟边缘化策略，保留历史信息但推迟引入先验，动态选择关键帧与边缘化时机。

## ESVO

- **文献**：ESVO: Event-based Stereo Visual Odometry (TRO 2020). arXiv:2007.15532。本地：`papers/esvo/esvo.pdf`
- **创新点**：事件相机立体视觉里程计，基于事件的时间表面匹配与深度估计，低延迟高动态范围。

## MonoGS

- **文献**：Gaussian Splatting SLAM (CVPR 2024). arXiv:2404.07486。本地：`papers/monogs/monogs.pdf`
- **创新点**：首个基于 3D Gaussian Splatting 的稠密 SLAM，实时可微渲染同时估计相机位姿与场景表示。

## PIN-SLAM

- **文献**：PIN-SLAM: LiDAR SLAM Using a Point-Based Implicit Neural Representation for Achieving Global Map Consistency (TRO 2024). arXiv:2401.09101。本地：`papers/pin_slam/pin_slam.pdf`
- **创新点**：基于点的隐式神经 SDF 地图，弹性可变形全局一致建图，回环时直接调整神经点位置。

## 4D Radar SLAM

- **文献**：4D iRIOM: 4D Imaging Radar Inertial Odometry and Mapping (RA-L 2023). arXiv:2303.13962。本地：`papers/4d_radar_slam/4d_radar_slam.pdf`
- **创新点**：首个 4D 成像雷达惯性 SLAM 系统，submap 匹配 + 多普勒速度估计，应对恶劣天气。

## Cartographer

- **文献**：Real-Time Loop Closure in 2D LIDAR SLAM (ICRA 2016) — Hess et al., Google.
- **本地 PDF**：无（Google 发表，非 arXiv）。论文可通过 Google Research 获取。
- **创新点**：2D/3D 激光 SLAM，分支定界快速相关扫描匹配回环，工业级实时建图。

## SVO / SVO Pro

- **文献**：SVO: Semidirect Visual Odometry for Monocular and Multicamera Systems (TRO 2017).
- **本地 PDF**：无（IEEE TRO）。DOI 见仓库 README。
- **创新点**：半直接法视觉里程计，稀疏直接法对齐 + 特征法优化，Pro 版扩展多相机与回环。

## ROVIO

- **文献**：ROVIO: Robust Visual Inertial Odometry Using a Direct EKF Based Approach (ICRA 2015).
- **本地 PDF**：无（IEEE）。DOI 见仓库 README。
- **创新点**：基于直接法 EKF 的 VIO，光度误差 + 像素级路标参数化，紧耦合滤波框架。

## SuperOdom

- **文献**：见仓库 readme 中 ICRA/IROS 与 arXiv:2412.02901。本地：`papers/SuperOdom/superodom_arxiv.pdf`
- **创新点**：多传感器融合里程计/SLAM 框架（具体以论文与代码模块为准）。

---

## 维护说明

新增下载：`papers/<项目名>/`，并在此文件增加一行本地路径。IEEE-only 论文请勿违规爬取；保留 DOI 即可。
