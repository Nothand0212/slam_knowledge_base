---
tags: [IMU, IMU管线, 数据管线, 传感器]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-px4-imu-preprocessing.md
  - wiki/sources/2026-04-29-imu-pipeline-comparison.md
---

# IMU 数据管线

> 21 个主流 SLAM/VIO 项目的 IMU 数据处理全管线对比：滤波系 vs 优化系两大范式

## 两大范式对比

| 维度 | 滤波系（EKF/IESKF） | 优化系（FGO） |
|------|-------------------|-------------|
| IMU 角色 | 系统主线，高频传播 | 因子图二元约束 |
| 更新时机 | 事件驱动 | 关键帧频率批量 |
| 线性化 | FEJ/OC 保证一致性 | 每次优化重线性化 |
| 零偏估计 | 实时，直接代入 | 一阶 Jacobian 校正 |

## 离散积分方法精度谱

| 精度 | 方法 | 代表项目 |
|------|------|---------|
| 最高 | SE2(3) 精确积分 | OpenMAVIS |
| 高 | RK4 + 三阶 Phi | msckf_vio, schurvins |
| 中 | 中值积分 | vins_fusion |
| 中 | GTSAM 标准预积分 | lio_sam, kimera_vio |
| 低 | 欧拉离散 | fusions_slam, lightning_lm |

## 初始化多样性

- 静止 IMU 均值：msckf_vio, open_vins（静态）
- 在线均值（不要求静止）：fast_livo2（3帧），lightning_lm（20帧）
- SFM + IMU 对齐：vins_fusion, orb_slam3
- GNSS/INS 粗对齐：ic_gvins
- 外部指定：ob_gins

## 关键发现

1. **多数 SLAM 项目的 IMU 预处理很轻**：通常依赖离线标定、时间配置和估计器内 bias 建模；飞控领域的 PX4 则实施了完整的 Butterworth LPF + 静态/动态陷波，详见 [[传感器-IMU预处理]]
2. **所有项目不做在线 Allan 方差估计或温度补偿**
3. IC-GVINS/OB_GINS 唯一实现了地球自转补偿（可选）
4. IMU-Camera 时间偏移多数离线设置，少数在线估计

## 相关页面

- [[概念-IMU预积分]]
- [[方法-IESKF滤波器]]
- [[因子图vs滤波]]
- [[架构-滑动窗口优化]]
- [[2026-04-29-imu-pipeline-comparison]]
- [[传感器-IMU预处理]]

## PX4 飞控的对比参照

PX4 飞控的 IMU 预处理管线可作为 SLAM 系统的"上限参考"，展示了 IMU 信号处理可以做到多深：

| 预处理环节 | PX4 方案 | SLAM 系统现状 |
|-----------|---------|-------------|
| 硬件低通滤波 | IMU 芯片寄存器配置（20-250Hz） | 无配置 |
| 软件低通滤波 | Butterworth 二阶，IMU_GYRO_CUTOFF 30-80Hz | 无 |
| 陷波滤波器 | 2组静态 + FFT 动态自适应 | 无 |
| 出厂标定 | 6面翻滚 → scale + misalignment + offset | 多为离线 Kalibr（仅外参） |
| 温度补偿 | 三阶多项式，-10°C~+60°C 温控箱标定 | 无 |
| 振动隔离 | 3M 4011 泡沫胶带等硬件减震指导 | 无指导 |
| 多 IMU 冗余 | 欧氏距离一致性投票 | 无 |

**核心差异根因**：飞控中 IMU 数据直接用于 PID 控制，噪声和延迟直接影响飞行稳定性；SLAM 中 IMU 通过预积分在关键帧间累积，对延迟容忍度更高。但若在高振动平台（无人机）上做 SLAM，缺少陷波滤波会导致陀螺仪数据被电机振动污染，姿态估计严重退化。

**最低推荐**：在无人机平台运行 SLAM 时，至少添加一个与电机转速基频匹配的陷波滤波器。

- PX4 完整管线作为案例收录在 [[传感器-IMU预处理]]，相关子概念见 [[方法-IMU陷波滤波器]]、[[方法-动态陷波]]、[[传感器-IMU出厂标定]]、[[传感器-IMU温度补偿]]
