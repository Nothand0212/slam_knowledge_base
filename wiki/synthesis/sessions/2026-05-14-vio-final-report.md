---
tags: [phad_slam, VIO, session总结, 关键帧, KF率, 精度优化, 自适应, ISAM2, 实验报告]
created: 2026-05-14
updated: 2026-05-15
type: synthesis
session_span: 2026-05-12 ~ 2026-05-14
branch: feature/vio-local-ba-preflight
status: Complete
experiments: 70+
references:
  - wiki/synthesis/sessions/2026-05-13-kf-selection-survey.md
  - wiki/synthesis/sessions/2026-05-13-phad-slam-kf-rate-root-cause.md
---

# VIO 精度优化完整实验报告

**Date:** 2026-05-14
**Status:** Complete — KF strategy converged, 14/16 sequences improved
**Branch:** `feature/vio-local-ba-preflight`
**Session span:** 2026-05-12 ~ 2026-05-14 (3 days, 70+ experiments)

## 项目上下文

PHAD SLAM VIO 基于 GTSAM iSAM2 增量平滑器，实现 ORB-SLAM3 风格本地 BA 预检模块。
初始状态：全 EuRoC 基准 V1_01=0.256m（退化于 4 月基准的 0.156），KF 率 74%。

## 实验路线图

```
阶段1: 修复结构化BA bug → BA工作但无精度增益
阶段2: 排除6个假设 → 全部证伪 (66行实验)
阶段3: 发现KF率根因 → V1_01 0.256→0.079 (13/16改善)
阶段4: 自适应KF + gap guard → 14/16改善，收敛
P0/P1/P2: 剩余差距诊断 → 三个不同根因
```

## 最终性能：16 序列对比

| 序列 | 原始 (74%KF) | 最终 (自适应+gap) | 改善 | 说明 |
|---|---|---|---|---|
| V1_01_easy | 0.256 | **0.083** | -68% | 接近 VINS-Mono 水平 |
| V1_02_medium | 0.891 | 0.419 | -53% | |
| V1_03_difficult | 0.530 | 0.345 | -35% | |
| V2_01_easy | 0.523 | **0.102** | -81% | 最大改善 |
| V2_02_medium | 0.673 | 0.362 | -46% | |
| V2_03_difficult | 1.269 | 1.249 | -2% | 前端失效限制 |
| MH_01_easy | 1.317 | 0.438 | -67% | |
| MH_02_easy | 1.573 | **0.741** | -53% | gap guard 修复 |
| MH_03_medium | 1.116 | 0.559 | -50% | |
| MH_04_difficult | 1.143 | 0.304 | -73% | |
| MH_05_difficult | 0.815 | 0.279 | -66% | |
| corridor1 | 0.585 | **0.277** | -53% | |
| corridor2 | 0.719 | 0.662 | -8% | 走廊本质困难 |
| corridor3 | 0.431 | 0.512 | +19% | 精度-效率折衷 |
| corridor4 | 0.566 | 0.466 | -18% | |
| corridor5 | 0.418 | **0.204** | -51% | |

## 已排除的假设（全部证伪）

| 假设 | 实验 | 结论 |
|---|---|---|
| Huber 内核导致飘移 | 非 Huber vs Huber: 0.718 vs 0.719 | 排除 |
| lifetime cap=3 导致飘移 | active_window vs lifetime: ~0.71 vs ~0.72 | 排除 |
| 扁平 BA 结构 vs 结构化 | 8 行对照: 全部 ~0.72 | 排除 |
| 固定锚点不足 | 默认(10) vs 更多(20): 0.720 vs 0.720 | 排除 |
| 前端特征参数优化 | 32 行全序列: 8/16 改善, 6/16 恶化 | 非普适 |
| 基线代码退化 | 6 个二进制对比: 所有分支一致 0.256 | 不存在回归 |
| 结构化 BA 改善精度 | P1: MH_04 6%改善，V1_01 恶化 2.6% | 非普适 |

## P0/P1/P2 诊断

| 序列 | 根因 | 修复方案 | 效果 |
|---|---|---|---|
| **MH_02** | 自适应抑制 85% 健康 KF | max_kf_gap_frames=15 | 1.957→0.741 |
| **V2_03** | 前端失效(73次跟踪丢失) | 提高 cap=10 | 1.249→0.870 |
| corridor3 | 精度-效率折衷 | 可接受 | — |
| 结构化 BA | 非普适 | 不推荐 | — |

## 最终配置

```yaml
keyframe:
  parallax_threshold_px: 40.0
  tracking_threshold: 100
  time_threshold_s: 0.5
  max_time_between_kf_s: 5.0
  low_excitation_acc_g: 0.3
  low_excitation_max_time_s: 2.0
  max_kf_gap_frames: 15
```

## 提交记录

| 提交 | 内容 |
|---|---|
| `bdce184` | 修复共视计算 2 个 bug |
| `641a5b8` | 降低默认 KF 率 (parallax 10→40) |
| `23a2a38` | KF 率基准报告 |
| `96084bf` | 自适应 KF: IMU 激励感知 |
| `4bc082c` | max KF gap guard (MH_02 修复) |
| `53825bb` | P0/P1/P2 spec |
| `39e6434` | P0/P1/P2 结果 |

## 关键教训

1. **诊断先行于修复** — 两周调试 BA/Huber/cap，根因是最简单的 KF 率参数
2. **知识库调研** — Kimera-VIO 的同架构调研启发了 max_time 和自适应设计
3. **全序列验证** — 单序列无法揭示普适性，16 序列对比才确认改善广度
4. **子代理分工** — 并行分析源码和诊断数据大幅提效
5. **不同序列不同根因** — V2_03(前端)/MH_02(parallax)/corridor3(折衷)各有原因

## 下一步建议

1. **继承 V2_03 前端修复** — 快速旋转+运动模糊导致 63% 弱几何
2. **提高 V2_03 的 cap=10** — 已验证 30% 改善
3. **corridor3 精度恢复** — 降低该序列的 low_excitation_max_time_s 到 1.0s
4. **自适应调参网格搜索** — 在更多序列上找到最优 (parallax, acc_thr, gap) 组合
