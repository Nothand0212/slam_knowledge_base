---
tags: [紧耦合, 先验传递, R3LIVE]
sources:
  - wiki/sources/2026-04-29-r3live_analysis.md
created: 2026-04-29
type: entity
updated: 2026-04-29
---

# LIO-as-Prior设计

> R3LIVE 中 VIO 以 LIO 位姿为起点、IMU 预积分对齐图像时间戳的先验传递机制。

## 设计动机

R3LIVE 的 LIO 和 VIO 不是完全平级的两个估计器。LIO 在几何结构稳定时提供高频、尺度可靠的运动估计，VIO 则补充颜色和视觉重投影约束。LIO-as-Prior 的设计就是把 LIO 当前状态作为 VIO 优化的起点和先验，避免 VIO 在每帧图像上从弱初值重新搜索。

## 工作流程
1. VIO 从 `state_in`（LIO 最后状态）开始
2. `m_imu_process->imu_preintegration()` 传播到当前图像时间
3. 输出 `state_out` 作为 VIO 初始猜测和先验约束
4. 用 `(state_iter - state_in)` 构建先验约束项

## 关键优势
- VIO 在视觉跟踪失败时能优雅降级回 LIO
- 先验传递使 VIO 不从头估计位姿
- 但 VIO 严重依赖 LIO 质量，LIO 退化时 VIO 也受影响

## 工程边界

这个架构的核心风险是先验过强。若 LIO 在退化场景中已经产生错误位姿，VIO 会围绕错误初值收敛，甚至把视觉残差解释成颜色或外参误差。实际系统需要退化检测、协方差膨胀和视觉残差门限，让 VIO 在 LIO 不可靠时有能力拒绝或弱化先验。

## 相关页面

- [[算法-R3LIVE]]
- [[架构-双ESIKF架构]]
- [[方法-统一IESKF融合]]
- [[架构-多传感器融合架构]]
