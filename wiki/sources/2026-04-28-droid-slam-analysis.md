---
tags: [VIO, 深度学习, DenseBA]
sources:
  - raw/docs-deep-dive/droid_slam_analysis.md
created: 2026-04-28
type: source
updated: 2026-04-29
source_path: raw/notes/droid_slam_analysis.md
---

# DROID-SLAM 源码级分析摘要

> 以 CNN + GRU 迭代预测光流修正，再通过 GPU 端 Dense BA（Schur 补）联合优化位姿和逐像素深度，全流程无需显式特征匹配、三角化或回环检测。

## 核心发现
- **Dense BA** 对每个像素定义残差：残差 = 网络预测光流 - 相机运动诱导光流，信息量远超传统稀疏 BA
- 深度变量 Hessian 逐像素对角独立，求逆仅为逐元素倒数，Schur 补代价仅与关键帧数相关
- ConvGRU 引入**全局上下文调制**：隐状态经空间平均得到全局描述子，注入门控偏置，兼顾局部迭代与全局信息
- **GraphAgg** 模块输出逐像素 damping factor η，作为 BA 深度 Hessian 对角增量防止发散
- 三重训练损失（测地线 + 残差 + 光流）+ 随机重启课程学习，确保网络从粗糙初始状态恢复

## 技术要点
| 维度 | 方案 |
|------|------|
| 范式 | 深度学习 SLAM（端到端可微） |
| 前端 | CNN 特征提取 + 全对相关体 + GRU 迭代预测光流修正 |
| 后端 | GPU Dense BA：Schur 补消去逐像素深度 → Cholesky 求解位姿 |
| 独特创新 | 光流残差替代重投影残差、Dense BA 的 GPU Schur、学习式置信度加权 |

## 关键引用
- 网络前向完整流程：`droid_slam/droid_net.py:172-222`
- Dense BA 实现：`droid_slam/geom/ba.py:31-106`
- Schur 补求解：`droid_slam/geom/chol.py:46-73`
- 投影变换与雅可比：`droid_slam/geom/projective_ops.py:165-198`
- 因子图与边管理：`droid_slam/factor_graph.py:19-50, 346-412`
- ConvGRU：`droid_slam/modules/gru.py:5-32`
- 训练损失：`droid_slam/geom/losses.py:30-118`

## 相关页面
- [[2026-04-29-nice_slam-analysis|NICE-SLAM方案分析]]
- [[方法-Dense BA|传统BA vs DenseBA]]
- [[VIO方案对比]]