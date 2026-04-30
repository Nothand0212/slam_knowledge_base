---
tags: [4D雷达, 事件相机, 里程计, RGB-D, 气压计, CAN, 多传感器融合]
type: source
created: 2026-04-29
updated: 2026-04-29
source: raw/docs-comparison/special_sensors_comparison.md
sources:
  - raw/docs-comparison/special_sensors_comparison.md
---

# 特殊传感器数据管线横向对比

> 覆盖 4D 毫米波雷达、事件相机、轮速/里程计、深度相机、磁力计/气压计、CAN/车辆传感器的综合对比

## 4D 毫米波雷达（4DRadarSLAM）

- 每点输出 (x, y, z, doppler, SNR)
- 配准方法：**APDGICP**（传感器模型驱动各向异性协方差）
- Doppler 速度四重利用：自运动估计、前端初值、动态去除、配准降级
- 与 LiDAR 关键差异：极度稀疏、各向异性噪声、每点自带速度

## 事件相机（ESVO）

- 异步事件流 → Time Surface（指数衰减）→ 类图像表示
- 立体匹配：ZNCC Block Matching（25×25 patch）
- Tracking：逆向组合法（Inverse Compositional）
- 三线程并行：Time Surface / Mapping / Tracking

## 轮速/里程计

- **紧耦合**（IC-GVINS, OB_GINS）：里程计入 IMU 预积分，sodo 在线估计
- **松耦合**（Cartographer）：BetweenFactor in PGO
- 标定：里程计比例因子 sodo + 安装角 abv 在线估计

## RGB-D 深度相机

| 范式 | 代表 | 深度角色 |
|------|------|---------|
| 传统 | ORB-SLAM3 | 已知几何，直接反投影 |
| 学习式 | DROID-SLAM | 传感器深度 + 网络预测融合 |
| 隐式 | NICE-SLAM/MonoGS | 可微渲染训练信号 |

## 关键结论

1. **4D 雷达核心价值在 Doppler**，后端仍为传统位姿图
2. **事件相机关键是 Time Surface 表示转换**
3. **里程计最佳实践是紧耦合**预积分多态设计
4. **RGB-D 在传统 vs 学习范式中的角色截然不同**
5. **气压计/磁力计工程价值有限**，用于辅助回环筛选
6. **车辆约束是低成本提升精度的有效手段**

## 相关页面

- [[算法-4DRadarSLAM]]
- [[算法-ESVO]]
- [[算法-IC-GVINS]]
- [[概念-深度学习SLAM]]
- [[架构-多传感器融合架构]]
- [[传感器-传感器标定]]