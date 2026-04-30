---
tags: [IMU, 传感器标定, PX4, 出厂标定]
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-px4-imu-preprocessing.md
---

# IMU出厂标定

> PX4 的 6 面翻滚测试标定：测量各轴比例因子、轴向正交矩阵和零偏，存入 CAL_ACC*/CAL_GYRO* 参数，实时校正 IMU 原始读数。

## 标定矩阵模型

PX4 的出厂标定使用完整的仿射校正模型：

**加速度计校正**：
```
a_corrected = scale * (a_raw - offset)
```

**陀螺仪校正**：
```
w_corrected = misalignment * scale * (w_raw - offset)
```

其中：
- **scale**：3x3 对角线矩阵，修正各轴灵敏度差异
- **misalignment**：3x3 满矩阵，修正三轴不正交性
- **offset/bias**：3×1 向量，零偏补偿

## 标定流程：6 面翻滚测试

将飞控依次放在 6 个面上（每个轴分别朝向和背向重力），在每面静止采集加速度计数据：

1. 利用重力作为已知基准信号，解算 scale 和 misalignment
2. 利用零速率假设，估计陀螺仪偏置
3. 参数存入 `CAL_ACC*` 和 `CAL_GYRO*` 系列参数（板载存储）
4. 运行时在 `src/modules/sensors/vehicle_imu.cpp` 中读取并逐采样校正

## 与 SLAM 标定的对比

已整理的 SLAM/VIO 工程通常更强调相机-IMU 外参、时间偏移、bias 在线估计和噪声参数配置；IMU 内部的 scale factor、轴不正交和零偏出厂校正，往往依赖传感器厂商、驱动层或离线标定流程，而不一定在 SLAM 主程序中显式实现。

PX4 的 6 面翻滚标定适合作为 IMU 内参校正的工程参考：它把比例因子、轴向不正交和零偏统一落到运行时校正模型中。对高精度或低成本 IMU 平台，这类标定能减少估计器需要吸收的系统误差；对高端 IMU，则需要先确认设备自身是否已提供等价校正。

## 相关页面

- [[传感器-IMU预处理]]
- [[传感器-传感器标定]]
- [[IMU数据管线]]
- [[2026-04-29-px4-imu-preprocessing]]
