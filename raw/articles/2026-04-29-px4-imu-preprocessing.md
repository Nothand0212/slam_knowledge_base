# PX4 IMU 数据预处理管线

来源: https://docs.px4.io/main/en/config_mc/filter_tuning.html + https://docs.px4.io/main/en/advanced_config/imu_factory_calibration.html + https://docs.px4.io/main/en/advanced_config/sensor_thermal_calibration.html

PX4 是目前最成熟的开源无人机飞控，其 IMU 预处理管线远比 SLAM 系统完善。

## 1. 硬件低通滤波

IMU 芯片（ICM-20689/ICM-42688/BMI088 等）内置可配置的抗混叠低通滤波器。PX4 通过 SPI 配置寄存器设定硬件截止频率，通常在 20-250Hz 范围。硬件 LPF 是抗混叠的第一道防线，在 ADC 采样之前切除奈奎斯特频率以上的噪声。

## 2. 软件低通滤波

PX4 在数字域叠加可配置的 Butterworth 二阶低通滤波器：

- `IMU_GYRO_CUTOFF`：陀螺仪低通截止频率，默认 30-80 Hz
- `IMU_ACCEL_CUTOFF`：加速度计低通截止频率，默认 30-80 Hz
- `IMU_DGYRO_CUTOFF`：陀螺仪微分项（D-term）的额外低通截止频率

滤波器在 sensors/vehicle_angular_velocity.cpp 和 sensors/vehicle_acceleration.cpp 中实现，使用 AlphaFilter（一阶 IIR）或 Butterworth2（二阶 IIR）类。

## 3. 陷波滤波器（Notch Filter）

针对多旋翼的电机/旋翼振动，PX4 实现了可配置的陷波滤波器：

- `IMU_GYRO_NF0_FREQ` / `IMU_GYRO_NF0_BW`：第一组陀螺仪陷波频率和带宽
- `IMU_GYRO_NF1_FREQ` / `IMU_GYRO_NF1_BW`：第二组陀螺仪陷波频率和带宽
- `IMU_ACCEL_NF0_FREQ` / `IMU_ACCEL_NF0_BW`：加速度计陷波频率和带宽

陷波频率应设置为电机转速的基频及其谐波。典型的 5 寸穿越机电机转速在 150-300Hz 范围，陷波频率常设为电机转速频率值。动态陷波（Dynamic Notch）是 PX4 1.13+ 新增特性，使用 FFT 实时检测主振动频率并自适应调节陷波位置，参数：`IMU_GYRO_DNF_EN`、`IMU_GYRO_DNF_BW`。

## 4. 加速度计偏置与重力校准

PX4 启动时执行传感器校准：
- 要求飞行器静止平放
- 采集多轴加速度计数据取均值，提取重力方向
- 计算加速度计偏置和初始姿态（roll/pitch）
- 陀螺仪偏置通过静止时零速率假设估计

## 5. IMU 出厂标定（Factory Calibration）

PX4 支持写入工厂标定参数到板载存储器：

- **比例因子（Scale Factor）**：3x3 对角线矩阵，修正各轴的灵敏度差异，从 6 面翻滚测试获得
- **轴正交矩阵（Misalignment）**：3x3 满矩阵，修正三轴不正交性
- **偏置（Offset/Bias）**：3×1 向量

参数存储在 `CAL_ACC*` 和 `CAL_GYRO*` 系列参数中，在 `src/modules/sensors/vehicle_imu.cpp` 中读取并应用。典型格式：
```
加速度计：a_corrected = scale * (a_raw - offset)  
陀螺仪：w_corrected = misalignment * scale * (w_raw - offset)
```

## 6. 温度补偿

PX4 支持基于多项式拟合的传感器温度漂移补偿：

- `TC_A*` 和 `TC_G*` 参数：定义三阶多项式系数
- 补偿公式：`correction = TC_X0 + TC_X1*T + TC_X2*T^2 + TC_X3*T^3`
- 温度从 IMU 芯片内部温度传感器读取
- 标定流程：将飞控放入温控箱，遍历工作温度范围（-10°C~+60°C），记录各温度点的偏置漂移，离线拟合多项式系数

## 7. 振动隔离（机械层面）

PX4 文档强烈建议使用减震安装（如 3M 双面胶、硅胶减震柱），将飞控与机架的高频振动隔离。这是在电信号层面之前的第一道防线。推荐使用双面泡沫胶带（如 3M 4011）而非尼龙柱硬连接。

## 8. 多 IMU 冗余与投票

PX4 支持同时接入多颗 IMU（如板载 ICM-20689 + 外置 ICM-42688）：
- 每颗 IMU 独立走滤波管线
- 输出经过一致性投票（voting），剔除离群传感器读数
- 投票策略基于各传感器读数的欧氏距离

## 与 SLAM 系统的对比

| 维度 | PX4 飞控 | 主流 SLAM 系统 |
|------|---------|--------------|
| 低通滤波 | Butterworth 二阶可配置 | 无 |
| 陷波滤波 | 静态+动态 FFT 自适应 | 无 |
| 出厂标定 | 6面翻滚 + 矩阵校正 | 多为离线 Kalibr |
| 温度补偿 | 三阶多项式 | 无 |
| 振动隔离 | 有硬件指导 | 无 |
| 多IMU冗余 | 投票机制 | 无 |

## 对 SLAM 系统的启示

PX4 的管线说明：飞控对 IMU 信号质量要求远高于 SLAM（直接用于 PID 控制，延迟和噪声直接影响飞行稳定性），因此滤波投入很大。SLAM 系统对 IMU 延迟容忍度更高（预积分在关键帧之间累积即可），但如果在高振动平台（无人机）上运行 SLAM，缺少陷波滤波器会导致陀螺仪数据被电机振动严重污染，直接影响姿态估计精度。

最低推荐：在无人机上做 SLAM 时，至少添加一个与电机转速匹配的陷波滤波器。