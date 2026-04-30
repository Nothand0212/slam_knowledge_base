---
tags: [source, 外部核验, primary-sources, SLAM, VIO, LiDAR]
type: source
created: 2026-04-29
updated: 2026-04-29
sources:
  - "https://ceres-solver.readthedocs.io/latest/nnls_modeling.html"
  - "https://borglab.github.io/gtsam/gpsfactor/"
  - "https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html"
  - "https://research.google.com/pubs/archive/45466.pdf"
  - "https://arxiv.org/abs/2201.04114"
  - "https://github.com/uzh-rpg/rpg_svo_pro_open"
  - "https://github.com/ethz-asl/rovio"
  - "https://github.com/HKUST-Aerial-Robotics/ESVO"
  - "https://arxiv.org/abs/2109.07982"
  - "https://arxiv.org/abs/2408.14035"
  - "https://github.com/PRBonn/PIN_SLAM"
  - "https://github.com/url-kaist/Quatro"
  - "https://github.com/koide3/fast_gicp"
  - "https://docs.opencv.org/master/d4/d8c/tutorial_py_shi_tomasi.html"
  - "https://docs.opencv.org/3.4/db/dd9/group__xfeatures2d__match.html"
---

# 外部一手资料核验

> 对本轮复查中涉及的算法、优化库和视觉/LiDAR 前端方法进行外部可信来源核验；优先使用官方文档、官方仓库、作者论文或项目主页。

## 核验原则

- **一手优先**：官方文档、官方 GitHub、作者 arXiv/论文 PDF。
- **二手不用作定论**：博客、论坛、论文解读只用于定位线索，不写入结论。
- **写入边界**：只补充可由来源直接支持的设计定位、算法管线、适用边界和工程风险。

## 已使用来源

- Ceres Solver 官方建模文档：确认 Manifold、鲁棒损失、非线性最小二乘建模边界。
- GTSAM GPSFactor 官方文档：确认 `GPSFactor` 约束的是位姿平移部分，并区分杆臂版本。
- Cartographer 官方调参文档与 Google 论文：确认子图、FastCorrelativeScanMatcher、Branch-and-Bound 和全局 pose graph 优化。
- DM-VIO arXiv：确认 delayed marginalization、第二图、photometric BA 与 IMU 初始化动机。
- SVO Pro 官方仓库：确认 SVO Pro 是 RPG 的半直接视觉里程计新版开源实现。
- ROVIO 官方/ETH 资料：确认 ROVIO 是基于同步 IMU 与单目/双目相机输入的鲁棒 VIO 框架。
- ESVO 官方仓库：确认 ESVO 使用 stereo event camera、Time Surface、direct/geometric 管线。
- R3LIVE 与 FAST-LIVO2 论文：确认 LIO/VIO 子系统、RGB 着色地图、统一/序贯 ESIKF、直接视觉/LiDAR融合边界。
- PIN-SLAM 官方仓库：确认点基隐式神经 LiDAR SLAM 的全局一致性目标。
- Quatro 官方仓库：确认其面向地面机器人 LiDAR 全局配准和回环粗配准。
- fast_gicp 官方仓库：确认 FastGICP、FastVGICP、CUDA VGICP 与 PCL registration 接口。
- OpenCV 官方文档：确认 Shi-Tomasi `goodFeaturesToTrack` 和 GMS `matchGMS` 的接口与假设。

## 相关页面

- [[组件-Ceres Manifold API]]
- [[方法-GNSS 位置残差因子]]
- [[算法-Cartographer]]
- [[方法-Branch-and-Bound回环检测]]
- [[算法-DM-VIO]]
- [[算法-SVO-Pro]]
- [[算法-ROVIO]]
- [[算法-ESVO]]
- [[算法-R3LIVE]]
- [[算法-FAST-LIVO2]]
- [[算法-PIN-SLAM]]
- [[方法-Quatro配准]]
- [[方法-Fast-VGICP]]
- [[组件-Nano-GICP]]
- [[方法-Shi-Tomasi角点]]
- [[方法-GMS匹配验证]]
