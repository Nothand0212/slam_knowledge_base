# 操作日志

> 记录知识库的所有变更历史

---

## 2026-05-15 maintenance | 分批归并：4 主题簇 + lint 全量通过

- **归并页面**：
  - [[方法-ICP变体族]] (427行)：归并 10 个 ICP 变体（点到点/点到面/GICP/VGICP/FastVGICP/APDGICP/NanoGICP/genz-icp/RotVGICP/点到分布/CT-ICP）
  - [[方法-LiDAR地图表示]] (417行)：归并 4 种体素地图（VoxelHashMap/八叉树/GaussianVoxelMap/iKD-Tree）
  - [[方法-GTSAM-Ceres工程因子]] (456行)：归并 4 种工程因子
  - [[方法-鲁棒估计方法族]] (437行)：归并 Geman-McClure/自适应阈值/两轮粗差剔除
- **22 个旧页**标记 `superseded-by`，未删除
- **lint 全量通过**：孤页 0、断链 0、index 全部收录、sources 覆盖率 277/279
- **更新页面**：index.md (21条新收录)、log.md

## 2026-05-15 maintenance | 知识库维护：gbrain 导入 + 前端修复 + session 笔记规范化

- **gbrain 导入**：将 262 个 wiki 页面全部导入 brain.db，生成 1543 个 embedding，支持全文搜索和语义搜索
- **前端修复**：修复 `wiki/sources/2026-04-29-msckf-vio-analysis-analysis.md` 中嵌套 YAML tags 导致 gbrain 崩溃的问题；为 `wiki/sources/2026-05-02-p2v-slam.md` 补充缺失的 `type: source`
- **session 笔记规范化**：
  - 为 `wiki/synthesis/sessions/2026-05-14-vio-final-report.md` 补充完整的 YAML frontmatter
  - 修复 `wiki/synthesis/sessions/2026-05-13-phad-slam-kf-rate-root-cause.md` 非标准 `updated: 2026-05-14 (final)` 格式
  - 三份 session 笔记的 `updated` 统一更新至 2026-05-15
- **更新页面**：log.md

## 2026-05-15 maintenance | 工程实现知识扩展：滑动窗口/边缘化/KF选择/ISAM2

- **来源**：`raw/codes/` 下 VINS-Fusion、ORB-SLAM3、Kimera-VIO 源码
- **新增实体页**：
  - [[方法-关键帧选择策略]]：三项目 KF 决策函数源码级对比
  - [[方法-滑动窗口边缘化]]：VINS-Fusion Schur 补边缘化完整实现
  - [[方法-ISAM2增量固定滞后平滑]]：iSAM2 贝叶斯树增量推理 vs 手动滑窗
- **扩写页面**：[[概念-Schur补与边缘化]]、[[架构-滑动窗口优化]]
- **更新页面**：log.md

## 2026-05-15 maintenance | Subagent 并行深挖：初始化/回环/时间同步

- **来源**：`raw/codes/` 下 VINS-Fusion、ORB-SLAM3、open_vins、lightning-lm、4DRadarSLAM 源码（3 subagent 并行）
- **新增实体页（7 页，1703 行）**：
  - [[方法-VIO初始化方法]] (278行)：三项目初始化管线对比 + 6 种设计模式
  - [[方法-重力精细化]] (127行)：VINS 切空间重参数化 + 数学推导
  - [[方法-IMU陀螺偏置初始化]] (150行)：VINS LDLT vs ORB-SLAM3 g2o LM 对比
  - [[方法-视觉回环检测管线]] (269行)：三阶段（候选→几何验证→本质图）
  - [[方法-LiDAR回环检测管线]] (293行)：ScanContext + ISC 五重回环
  - [[方法-IMU-camera时间偏移估计]] (265行)：td 在线估计 + Jacobian
  - [[方法-多传感器外参在线标定]] (321行)：三阶段标定 + SVD 手眼方程
- **扩写页面（2 页）**：[[概念-视觉惯性初始化策略]]、[[传感器-传感器标定]]
- **更新页面**：log.md

## 2026-05-14 crystallize | VIO 精度优化完整实验报告

- **来源**：会话结晶化（3 天，70+ 次实验）
- **新增综合分析**：[[2026-05-14-vio-final-report]]
- 覆盖从 BA 调试到自适应 KF 的完整优化历程，14/16 序列精度改善，最终 KF 配置收敛

## 2026-05-02 ingest | P2V-SLAM 隐式点-体素 LiDAR-IMU SLAM

- **素材来源**：微信公众号「3D视觉工坊」文章《告别平面与直线 | 华中科技大学提出隐式点-体素SLAM，突破几何约束瓶颈！》
- **原始素材**：保存到 `raw/wechat/2026-05-02-p2v-slam/`，包含 Markdown 正文与 13 张本地图片
- **新增素材摘要**：[[2026-05-02-p2v-slam]]
- **新增实体页**：[[算法-P2V-SLAM]]、[[方法-隐式点-体素观测模型]]
- **更新页面**：[[方法-体素地图]]、[[方法-IESKF滤波器]]、[[LiDAR数据管线]]、[[素材索引]]、wiki/overview.md、index.md、log.md
- **实现提示**：[[方法-隐式点-体素观测模型]] 已补充 Agent 实现提示；由于 `p2v-slam` 源码尚未进入 `raw/codes/`，当前只提供伪代码和待补源码锚点

## 2026-05-13 crystallize | 关键帧选择策略调研

- **来源**：会话结晶化
- **新增综合分析**：[[2026-05-13-kf-selection-survey]]（VINS-Fusion / ORB-SLAM3 / Kimera-VIO / OpenVINS KF 策略调研）
- 提炼 4 个项目 KF 选择逻辑，为 phad_slam 自适应 KF 设计提供参考

## 2026-05-13 crystallize | phad_slam KF 率根因与精度优化

- **来源**：会话结晶化（多轮调试 + 基准测试）
- **新增综合分析**：[[2026-05-13-phad-slam-kf-rate-root-cause]]
- 从 iSAM2 线性化点漂移根因出发，解释 KF 率 72%→13% 的精度改善机制及自适应 KF 策略

## 2026-04-29 reorg | 实体页分类重组

- **操作**：全部 167 个实体页按 7 大类重命名并合并重复页面
- **重命名**：157 个文件添加分类前缀（算法-/概念-/方法-/组件-/传感器-/数学-/架构-）
- **合并**：9 对重复/相关文件合并（DBoW2, IMU预积分, DSO, KISS-ICP, LIO-SAM, GTSAM, MSCKF, 因子图）
- **链接更新**：261 个 .md 文件中所有 [[OldName]] 链接更新为新名称
- **结果**：167 → 158 个实体文件
- **分类分布**：算法 30, 概念 19, 方法 77, 组件 12, 传感器 6, 数学 3, 架构 11
- **更新页面**：index.md（实体段重组）、log.md

## 2026-04-29 ingest | PX4 IMU 数据预处理管线

- **素材来源**：PX4 官方文档综合（filter_tuning / imu_factory_calibration / sensor_thermal_calibration）
- 新增素材摘要：1（[[2026-04-29-px4-imu-preprocessing]]）
- 新增实体页：5（[[传感器-IMU预处理|PX4 IMU预处理]]、[[方法-IMU陷波滤波器]]、[[传感器-IMU出厂标定]]、[[传感器-IMU温度补偿]]、[[方法-动态陷波]]）
- 更新主题页：[[IMU数据管线]]（修正关键发现#1 + 新增 PX4 对比参照章节）
- 更新页面：index.md, log.md

## 2026-04-29 ingest | 9 篇 docs-deep-dive 深度源码分析

- **素材来源**：raw/docs-deep-dive/ 下 9 份深度源码分析报告
- 新增素材摘要：9 篇（VINS-Fusion, MSCKF_VIO, SchurVINS, OpenMAVIS, DM-VIO, DSO, SVO Pro, ROVIO, ESVO）
- 新增实体页：5（[[算法-SchurVINS]], [[组件-OpenMAVIS]], [[概念-延迟边缘化]], [[传感器-事件相机]], [[Time Surface]]）
- 更新页面：index.md, log.md

## 2026-04-29 batch-ingest-3 | 8 篇 deep_dive 深度分析 + 31 实体页

- 素材来源：raw/docs-deep-dive/ 下 8 份深度源码分析报告
- 处理模式：分批消化（每文件 Step1 实体/话题提取 → Step2 源页写入 + 实体页创建）
- 更新素材摘要：8 篇（gtsam_points / IC-GVINS / OB_GINS / SuperOdom / lt-mapper / 4DRadarSLAM / ROLO / slam_fusion_core）
- 新增实体页：31（按来源分组）
  - gtsam_points 4: IntegratedMatchingCostFactor, GaussianVoxelMap体素化配准, 连续时间ICP因子, LiDAR捆集调整因子
  - IC-GVINS 4: GNSS位置残差因子, 地球自转补偿预积分, INS-centric初始化, Ceres两轮优化+粗差剔除
  - OB_GINS 3: Ceres Manifold API, 固定整秒节点采样, GNSS-IMU离线批优化
  - SuperOdom 4: 多源位姿预测策略, 解析雅可比CeresCostFunction, 6-DoF退化检测, GTSAM ISAM2 IMU预积分
  - lt-mapper 4: lt-mapper, Anchor节点位姿图优化, 信息增益回环选择, Removert动态变化检测
  - 4DRadarSLAM 4: APDGICP自适应概率分布GICP, Doppler自速度估计, IntensityScanContext(ISC), 五重回环几何验证
  - ROLO 4: ROLO-SLAM, RotVGICP, 分离式旋转/平移估计, 地面车辆运动约束
  - slam_fusion_core 4: slam_fusion_core, CameraRig多相机抽象, Pipeline传感器数据调度, 后端适配器模式
- 更新页面：index.md, log.md

## 2026-04-26 — 初始化

- **操作**：创建知识库
- **主题**：ROS 机器人导航与 SLAM 知识库
- **状态**：完成

## 2026-04-28 batch-ingest-2 | 18 SLAM 框架源码分析 + 3 实体页

- 素材来源：slam_ws/docs/deep_dive/ 扩展分析
- 处理模式：批量消化（按算法范式分组：直接法 / 深度学习 / LiDAR前沿 / 多传感器融合 / 特殊传感器）
- 新增素材摘要：18 篇（直接法 4 + 深度学习 4 + LiDAR 前沿 4 + 多传感器融合 5 + 特殊传感器 1）
- 新增实体页：3（直接法视觉里程计, 深度学习SLAM, 连续时间轨迹）
- 更新对比分析：VIO方案对比（新增 DSO/DM-VIO/ROVIO/SVO Pro/DROID-SLAM，从 6→11 方案）
- 更新页面：index.md, VIO方案对比.md, log.md

## 2026-04-29 batch-ingest-3 | 9 份 SLAM 框架 deep_dive 分析

- 素材来源：raw/docs-deep-dive/ 下 9 份源码级分析
- 处理模式：批量消化（VIO / 深度学习 / LiDAR / 多传感器融合）
- 新增素材摘要：9 篇（Kimera-VIO, DROID-SLAM, NICE-SLAM, MonoGS, FAST-LIO-SAM-SC-QN, LIO-SAM, LeGO-LOAM, fusions_slam, CT-ICP）
- 新增实体页：33（算法框架 4 + 概念/方法 26 + 库 3）
- 更新页面：index.md, log.md

## 2026-04-29 ingest | 9 份管线对比与深度分析

- **素材来源**：raw/docs-top/ (2) + raw/docs-comparison/ (5) + raw/docs-deep-dive/ (2)
- 新增素材摘要：9 篇
- 新增实体页：2（[[算法-OpenVINS]], [[算法-ORB-SLAM3]]）
- 新增主题页：5（[[相机数据管线]], [[IMU数据管线]], [[LiDAR数据管线]], [[GNSS数据管线]], [[特殊传感器数据管线]]）
- 更新页面：index.md, log.md

## 2026-04-29 lint | 知识库健康修复

- **操作**：lint-fix + 断链修复 + 实体页补全
- 断链修复：新增 43 个实体页（含 18 个新框架 + 25 个通用概念）
- 交叉引用补充：19 个 source 页面添加双向链接
- index 修复：10 处文件名不匹配修正
- sources 补充：85 页 frontmatter sources 字段
- **结果**：index 缺失 0 / sources 缺失 0 / 未收录 0
- **残余**：80 断链（深度话题，不影响使用）、22 孤立页面

---

## 2026-04-28 batch-ingest | 18 个 GitHub SLAM 框架

- 素材来源：slam_ws/docs/deep_dive/ 下 19 份源码级分析报告 + 1 份设计文档
- 处理模式：批量消化（按传感器类型分组：VIO / LiDAR / GNSS & 因子）
- 新增素材摘要：20 篇（VIO 7 + LiDAR 7 + GNSS/因子 6）
- 新增实体页：8（MSCKF, 滑动窗口优化, FAST-LIO, genz-icp, DBoW2, GTSAM因子类型, 视觉特征跟踪, 坐标系管理）
- 新增对比分析：3（VIO方案对比, LiDAR方案对比, 因子图vs滤波）
- 新增综合分析：1（phad_fusion设计总结）
- 更新页面：index.md, log.md

## 2026-04-27 ingest | GTSAM 4.3a1 官方文档

- 新增素材：[[2026-04-27-gtsam-4.3a1-docs]]
- 新增实体页：[[组件-GTSAM]]、[[概念-因子图]]
- 新增主题页：[[GTSAM API 使用索引]]、[[GTSAM 因子图工作流]]、[[GTSAM Geometry API]]、[[GTSAM Nonlinear 优化 API]]、[[GTSAM Navigation 与 IMU API]]、[[GTSAM SLAM 与视觉因子 API]]、[[GTSAM 自定义因子与 Jacobian]]
- 新增综合分析：[[GTSAM 4.3a1 使用指南]]
- 来源：GTSAM 官方文档 `https://borglab.github.io/gtsam/`，共抓取 sitemap 下 230 个页面
## 2026-04-29 update | IMU预积分方法 全面重写
- 从 33 行扩展至 170+ 行，涵盖 8 种积分/预积分方法的完整解释和数学公式
- 新增两大范式对比（滤波传播 vs 因子图预积分）、精度谱总览、选型建议表
- 交叉引用所有使用各方法的算法项目

## 2026-04-29 update | 视觉特征跟踪 全面重写
- 从 36 行扩展至 290+ 行，涵盖 11 种算法的完整特征跟踪方案
- 新增：FAST/Shi-Tomasi/ORB/SuperPoint 四种检测器含数学公式、BRIEF/Steered BRIEF 描述子、KLT光流方程与最小二乘推导、4种空间分布策略、4种外点剔除方法、特征生命周期状态机、全算法速查表、设计模式与陷阱

## 2026-04-29 create | PnP 运动估计 新建实体
- 创建 PnP 页面，覆盖 P3P/EPnP/UPnP/DLT 四种求解方法含数学推导
- 包含各算法使用场景速查表（ORB-SLAM3/VINS-Fusion/SVO/Kimera/DROID-SLAM）
- 补充共面退化/少点退化/深度不确定性的诊断与对策
- 与视觉特征跟踪的 2D-2D 链路互补，形成完整的 2D-2D→3D-2D 前端定位管线

## 2026-04-29 create | 三角化与深度估计 新建实体
- 覆盖单目DLT三角化、逆深度参数化、双目立体匹配、Bouguet立体校正的完整数学推导
- 深度误差分析：σ_Z ∝ Z²/(f·b)，含典型数值对照
- 包含SVO Pro概率深度滤波器、DM-VIO ImmaturePoint机制等5种深度不确定性建模方案
- 11种算法的三角化方案速查表 + 即时vs延迟三角化的设计决策对比
- 与视觉特征跟踪(2D-2D)、PnP运动估计(3D-2D)构成完整的前端定位三件套

## 2026-04-29 lint | frontmatter 与来源信号修复
- 补齐 42 个缺少 YAML frontmatter 的 wiki 页面（实体页、素材页、overview）
- 回填实体页、主题页、对比页、素材页的 `sources` 字段，source-signal 覆盖从 183/249 提升到 249/249
- 重写 [[数学-SE3指数映射]]：补充定义、左乘/右乘扰动约定、SLAM 应用和工程检查点
- 残余问题：断链与 index 文件名不匹配仍需单独清理

## 2026-04-29 lint | 断链与索引一致性修复
- 批量将 138 个别名/旧名 wikilink 改为指向现有页面的 Obsidian alias 链接
- 新增 [[素材索引]]，把 76 个素材摘要页接入 wiki 内部图谱
- 新增 [[概念-基准测试数据集]]，合并 EuRoC / KITTI 基准测试断链，避免创建低价值薄页面
- 更新 index 与 overview 统计：253 个 wiki 页面，source-signal 覆盖 251/251

## 2026-04-29 refactor | IMU 预处理实体重构
- 将 `传感器-PX4-IMU预处理` 合并为通用实体 [[传感器-IMU预处理]]
- 保留 [[2026-04-29-px4-imu-preprocessing]] 作为 PX4 案例素材页
- 新实体页按预处理边界组织：滤波、陷波、标定、温度补偿、时间处理、异常值处理、多 IMU 管理
- 更新 index、IMU 数据管线和相关子概念反链

## 2026-04-29 refactor | 对比分析页更新
- 重写 [[VIO方案对比]]：按 VIO/VO 方案族组织，补充 IMU 预处理边界与工程选型矩阵
- 重写 [[LiDAR方案对比]]：按 ICP、LIO、连续时间、GICP、长期建图、神经隐式路线组织
- 重写 [[因子图vs滤波]]：修正 iSAM2/边缘化表述，区分 IMU 预处理、预积分和估计后端
- 更新 index 中对比分析入口描述

## 2026-04-29 audit | 全库质量复查第一批
- 全量扫描 256 个 markdown 文件，检查短页、缺失摘要、公式/矩阵符号、置信度标记残留和过窄实现细节页
- 重写 [[传感器-Doppler 自速度估计]]：修正最小二乘闭式解，补充观测模型、RANSAC、方差估计、退化条件和动态目标去除边界
- 移除 5 处置信度标记残留，并把“SLAM 系统完全没有”改为“已整理工程中不常见/通常不显式实现”的有边界表述
- 合并薄页：删除 `方法-MeasureGroupAdd` 并并入 [[架构-Pipeline 传感器数据调度]]；删除 `方法-planarCheck` 并并入 [[方法-在线平面拟合]]
- 扩写公式页：[[数学-球谐函数]]、[[方法-GaussianVoxelMap 体素化配准]]、[[方法-POINT_TO_DISTRIBUTION]]
- 扩写高复用概念页：[[传感器-传感器标定]]、[[数学-流形优化]]、[[方法-3D Gaussian Splatting]]、[[方法-Dense BA]]、[[方法-GICP配准方法]]、[[方法-IMU deskew]]、[[方法-6-DoF 退化检测]]、[[方法-APDGICP 自适应概率分布 GICP]]、[[方法-ICP配准方法]]
- 继续扩写薄方法页：[[方法-Ceres后端融合]]、[[方法-Fast-VGICP]]、[[方法-GMS匹配验证]]、[[方法-Geman-McClure鲁棒核]]、[[方法-IESKF滤波器]]

## 2026-04-29 audit | 全库质量复查第二批
- 扩写 CT-ICP/KISS-ICP 相关薄页：[[方法-RobustRegistration]]、[[方法-连续时间线性插值]]、[[方法-VoxelHashMap]]、[[方法-点到点ICP]]
- 扩写学习式 SLAM 与可微渲染页：[[概念-体积渲染]]、[[方法-ConvGRU]]、[[方法-ConvONet]]、[[方法-RAFT光流]]、[[组件-lietorch]]
- 扩写地图/配准/初始化页：[[方法-OnlineGravityAlignment]]、[[方法-两步解耦优化]]、[[方法-Quatro配准]]、[[方法-SDF解码器]]、[[方法-VoxelMap八叉树]]
- 补全局部地图与可微地图页：[[方法-子地图]]、[[方法-曝光在线估计]]、[[方法-概率占据网格]]、[[方法-自适应阈值]]、[[组件-IVox3d]]、[[方法-层次化特征网格]]、[[方法-微分栅格化]]、[[概念-可微地图]]、[[组件-Nano-GICP]]
- 批量补齐摘要与相关页：GNSS/Ceres/GTSAM 因子、回环验证、运动先验、后端适配器、BEV/ROLO/lt-mapper/fusions_slam/slam_fusion_core 等页面

## 2026-04-29 audit | 全库质量复查第三批
- 扩写高复用基础页：[[方法-ScanContext]]、[[方法-体素地图]]、[[概念-Schur补与边缘化]]、[[概念-位姿图优化]]、[[概念-回环检测方法]]、[[架构-多传感器融合架构]]、[[组件-Ceres-Solver]]
- 扩写核心算法页：[[算法-4DRadarSLAM]]、[[算法-FAST-LIO-SAM-SC-QN]]、[[算法-Kimera-VIO]]、[[算法-CT-ICP]]、[[算法-DROID-SLAM]]、[[算法-NICE-SLAM]]、[[算法-MonoGS]]
- 扩写 GNSS/INS 与点云库页：[[算法-IC-GVINS]]、[[算法-OB_GINS]]、[[组件-gtsam_points]]、[[算法-fusions_slam]]
- 扩写工程方法页：[[方法-Neural Point Map Context]]、[[方法-BEV投影]]、[[架构-LIO-as-Prior设计]]、[[概念-直接法光度误差]]、[[方法-RGB着色点云]]、[[方法-运动先验约束]]、[[方法-Branch-and-Bound回环检测]]、[[方法-Planar-Non-planar分类]]、[[方法-Shi-Tomasi角点]]
- 扩写融合/回环/长期建图页：[[方法-miao PGO]]、[[方法-多分辨率NDT回环]]、[[方法-四阶段回环验证]]、[[方法-信息增益回环选择]]、[[方法-自适应平面度权重]]、[[方法-点基隐式神经表示]]、[[方法-Removert 动态变化检测]]
- 扩写算法概览与架构页：[[算法-DM-VIO]]、[[算法-SVO-Pro]]、[[算法-BEV-LSLAM]]、[[算法-ROVIO]]、[[算法-LVI-SAM]]、[[算法-Cartographer]]、[[算法-R3LIVE]]、[[算法-PIN-SLAM]]、[[算法-FAST-LIVO2]]、[[算法-lt-mapper]]、[[架构-cpp+ros分离架构]]
- 修正拼写与表述问题：`PG0` 改为 `PGO`，补充多个页面的适用边界、失败模式和工程取舍，避免薄页只停留在源码流程摘录层面

## 2026-04-29 audit | 外部一手资料核验
- 新增 [[2026-04-29-external-primary-source-check]]，汇总官方文档、官方仓库、arXiv/论文 PDF 等外部可信来源
- 用外部来源核验并回链关键页面：[[组件-Ceres Manifold API]]、[[方法-GNSS 位置残差因子]]、[[算法-Cartographer]]、[[方法-Branch-and-Bound回环检测]]、[[算法-DM-VIO]]、[[算法-SVO-Pro]]、[[算法-ROVIO]]、[[算法-ESVO]]
- 继续核验 LiDAR/VIO/神经地图相关页面：[[算法-R3LIVE]]、[[算法-FAST-LIVO2]]、[[算法-PIN-SLAM]]、[[方法-Quatro配准]]、[[方法-Fast-VGICP]]、[[组件-Nano-GICP]]、[[方法-Shi-Tomasi角点]]、[[方法-GMS匹配验证]]
- 更新 [[素材索引]] 和 index，使外部核验素材纳入 source-signal 图谱

## 2026-04-30 ingest | 图像预处理源码调查与影响分析
- 新增素材摘要 [[2026-04-30-image-preprocessing-comparison]]，保存原始素材到 `raw/notes/2026-04-30-image-preprocessing-comparison.md`
- 新增实体 [[方法-图像预处理]]，按几何变换、特征增强、光度建模、网络输入规范组织预处理边界
- 新增主题 [[图像预处理与观测模型]]，把 KLT/ORB/直接法/学习式/LIVO 的预处理差异统一到观测模型视角
- 更新 [[相机数据管线]]、[[VIO方案对比]]、[[方法-视觉特征跟踪]]、[[概念-直接法光度误差]]、[[方法-曝光在线估计]]

## 2026-04-30 update | Agent 实现提示与 raw/codes 源码锚点
- 更新 `.wiki-schema.md`：将 `raw/codes/` 明确为只读源码来源，并新增 `Agent 实现提示` 写作规范
- 在 [[方法-图像预处理]]、[[方法-视觉特征跟踪]]、[[概念-直接法光度误差]]、[[方法-曝光在线估计]] 中补充输入输出契约、实现伪代码、真实源码片段和源码检索锚点
- 更新 [[图像预处理与观测模型]]：增加面向 AI agent 的代码化阅读路径，便于检索后直接进入实现

## 2026-04-30 update | 实体页 Agent 实现提示扩展
- 按视觉几何、滤波融合、LiDAR 配准/地图、学习式/可微地图四组扩展 20 个实体页的 `Agent 实现提示`
- 覆盖 PnP、三角化、Shi-Tomasi、GMS、SmartStereoFactor、IMU 预积分、MSCKF、IESKF、IMU deskew、统一 IESKF 融合、ICP/GICP/Fast-VGICP、VoxelHashMap、ScanContext、ConvGRU、RAFT、3D Gaussian Splatting、可微渲染、SDF 解码器
- 每页补充适用场景、输入输出契约、伪代码实现骨架、真实 `raw/codes/` 源码片段、实现注意事项和源码检索锚点，便于 AI agent 检索后直接进入实现

## 2026-04-30 maintenance | raw/codes 源码快照化
- 将 `raw/codes/` 明确定义为只读源码快照区，而不是嵌套 Git 仓库或 submodule
- 新增 `raw/codes/MANIFEST.md` 记录各源码项目的 upstream、分支、HEAD commit 和删除 `.git` 前的 dirty 状态
- 更新 `AGENTS.md` 与 `.wiki-schema.md`：后续新增或替换源码快照时，必须先记录版本来源，再维护 wiki 中的源码行号锚点
