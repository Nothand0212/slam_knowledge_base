---
tags: [phad_slam, VIO, session总结, 调试, iSAM2, 关键帧, KF率, 精度优化, 自适应]
created: 2026-05-13
updated: 2026-05-15
type: synthesis
session_span: 2026-05-02 ~ 2026-05-14
branch: feature/vio-local-ba-preflight
commits:
  - bdce184: fix structured BA covisibility bugs
  - 641a5b8: reduce default KF rate
  - 23a2a38: KF rate benchmark report
  - 96084bf: adaptive KF with IMU excitation
  - 4bc082c: max KF gap guard (MH_02 fix)
references:
  - docs/superpowers/benchmarks/2026-05-13-vio-kf-rate-systematic-benchmark.md
  - wiki/synthesis/sessions/2026-05-13-kf-selection-survey.md
---

# PHAD SLAM VIO 实验总结：从 BA 调试到自适应 KF

## 项目状态

**仓库**: `phad_slam` (GTSAM iSAM2 立体 VIO)
**分支**: `feature/vio-local-ba-preflight`
**状态**: KF 策略已收敛，精度大幅改善

## 五个阶段

### 阶段 1：结构化 BA 修复 (5/12)
发现修复两个阻止共视计算的 bug (`bdce184`)。修复后 BA 正常工作但无精度增益。

### 阶段 2：排除假设 (5/12-13)
66 行实验排除：Huber、cap policy、BA 结构、固定锚点、前端参数、基线退化。

### 阶段 3：KF 率根因 (5/13)
高 KF 率 (74%) 是主导瓶颈。降至 13% 后 13/16 改善（V1_01: 0.256→0.079），corridor3/5 回归。提交 `641a5b8`。

### 阶段 4：自适应 KF (5/13-14)
调研 VINS-Fusion/ORB-SLAM3/Kimera-VIO/OpenVINS，设计 IMU 激励感知的三阶段触发。15/16 不低于原始。提交 `96084bf`。

### 阶段 5：gap guard + P0 诊断 (5/14)
诊断 V2_03/MH_02/corridor3 根因——三个各不相同。MH_02 是自适应抑制健康 KF 导致。加 `max_kf_gap_frames=15` 守卫后 MH_02: 1.957→0.741。提交 `4bc082c`。

## 最终性能：16 序列

| 序列 | 原始 (74%KF) | 自适应+gap | 改善 |
|---|---|---|---|
| V1_01_easy | 0.256 | **0.083** | -68% |
| V1_02_medium | 0.891 | 0.419 | -53% |
| V1_03_difficult | 0.530 | 0.345 | -35% |
| V2_01_easy | 0.523 | **0.102** | -81% |
| V2_02_medium | 0.673 | 0.362 | -46% |
| V2_03_difficult | 1.269 | 1.249 | -2% (前端限制) |
| MH_01_easy | 1.317 | **0.438** | -67% |
| MH_02_easy | 1.573 | **0.741** | -53% |
| MH_03_medium | 1.116 | 0.559 | -50% |
| MH_04_difficult | 1.143 | **0.304** | -73% |
| MH_05_difficult | 0.815 | 0.279 | -66% |
| corridor1 | 0.585 | **0.277** | -53% |
| corridor2 | 0.719 | 0.662 | -8% |
| corridor3 | 0.431 | 0.512 | +19% (精度-效率折衷) |
| corridor4 | 0.566 | 0.466 | -18% |
| corridor5 | 0.418 | **0.204** | -51% |

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

## P0 诊断关键发现

- **V2_03 (1.25m)**: 前端失效（73 次跟踪丢失，63% 弱几何）。非 KF 策略问题。
- **MH_02 (0.74m)**: 视差阈值过严抑制健康 KF。gap guard 修复。
- **corridor3 (0.51m)**: 可接受精度-效率折衷（5.8× 加速换 19% 精度损失）。

## 已知遗留

1. V2_03 前端鲁棒性（运动模糊/快速旋转）
2. 结构化 BA 在低 KF 率下待重测 (P1)
3. lifetime cap=3 可能限制约束密度 (P2)
