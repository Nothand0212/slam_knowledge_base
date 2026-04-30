---
tags: [SLAM, 3DGS, GaussianSplatting, 单目]
sources:
  - raw/docs-deep-dive/monogs_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/monogs_analysis.md
---

# MonoGS 源码级分析摘要

> 将3D Gaussian Splatting引入SLAM，微分栅格化替代特征匹配，Adam梯度下降替代传统优化器，实现单目/双目端到端密集外观建图与Tracking

## 核心发现
- 场景表示为3D Gaussians（各向异性椭球，含xyz/color/scale/rotation/opacity，约56B/Gaussian），渲染过程支持对相机位姿直接求导（theta/rho参数）
- Frontend（Tracking）+ Backend（Mapping）分离为独立进程，通过multiprocessing Queue通信
- Tracking：恒速模型初始化→Adam优化6-DOF位姿（光度误差+曝光补偿），约60-100次渲染迭代/帧
- Mapping：滑动窗口内联合优化场景参数+位姿，isotropic正则防高斯坍缩，共视度剪枝管理帧级Gaussian生命周期
- 单目模式从随机深度（2.0+noise）开始经BA收敛，无尺度约束

## 技术要点
| 维度 | 方案 |
|------|------|
| 前端 | 微分栅格化渲染→光度对齐（L1+opacity加权），SE(3)指数映射位姿更新，曝光在线补偿 |
| 后端 | 滑动窗口联合BA（Adam优化场景+位姿），isotropic正则，densify_and_prune动态密度管理 |
| 独特创新 | 地图即场景（可渲染输出）+梯度驱动SLAM+单目深度"涌现"+曝光自适应 |

## 关键引用
- Gaussian参数化: `gaussian_model.py:34-68`
- 微分栅格化（含theta/rho位姿求导）: `gaussian_renderer/__init__.py:130-141`
- Frontend Tracking: `slam_frontend.py:128-196`
- Backend Mapping: `slam_backend.py:142-318`
- 关键帧决策: `slam_frontend.py:198-225`
- 共视度剪枝: `slam_backend.py:247-275`
- 位姿更新（SE(3)指数映射）: `pose_utils.py:76-93`
- Frontend/Backend进程通信: `slam.py:63-109`

## 相关页面
- [[2026-04-28-fast-livo2-analysis]]
- [[2026-04-28-r3live-analysis]]