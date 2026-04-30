---
tags: [方法, IMU, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-30
sources:
  - wiki/sources/2026-04-29-lio_sam-analysis.md
---

# IMU deskew

> 利用 IMU 在一帧 LiDAR 扫描期间的姿态/位姿变化，把每个点补偿到统一参考时刻，降低扫描畸变对配准的影响。

## 定义

LiDAR 扫描不是瞬时完成的。一帧点云通常跨越几十到上百毫秒，平台在这段时间内会旋转和平移。如果直接把所有点当作同一时刻采集，scan-to-scan 或 scan-to-map 配准会看到被拉弯的几何结构。IMU deskew 的目标是利用高频 IMU 估计帧内运动，把每个点转换到帧起始或帧结束坐标系。

## LIO-SAM 流程

LIO-SAM 的 deskew 主要使用 IMU 角速度积分做旋转补偿：

1. 维护覆盖当前 LiDAR 帧时间范围的 IMU 队列。
2. 清除早于 `timeScanCur - 0.01` 的过期 IMU。
3. 对角速度做零阶保持和前向欧拉积分，得到帧内离散姿态序列。
4. `findRotation` 根据点的相对时间插值得到该点采样时刻的旋转。
5. 用 `transStartInverse * transFinal` 将点转换回帧起始参考系。

LIO-SAM 中平移 deskew 被注释掉，原因是作者假设 walking speed 下平移畸变不显著。这是工程取舍，不是通用结论。

## 与连续时间方法的区别

| 方法 | 运动模型 | 优点 | 限制 |
|------|----------|------|------|
| IMU deskew | 用 IMU 积分提供帧内姿态/位姿 | 实时、实现简单，适合 LIO 前端 | 依赖 IMU 时间同步和 bias 质量 |
| 恒速 deskew | 假设帧间匀速运动 | 不依赖 IMU | 高动态场景误差大 |
| 连续时间轨迹 | 优化扫描期间的连续轨迹 | 更自然处理非同步传感器 | 变量和求解复杂 |

## 工程注意

- 每个点必须有相对时间戳，否则只能做整帧近似补偿。
- IMU 时间偏移会直接表现为 deskew 方向错误。
- 高速车辆、无人机或手持快速旋转场景不能忽略平移 deskew。
- deskew 后的点云仍需配准残差验证，不能把 IMU 积分误差当作真值。

## Agent 实现提示

### 适用场景

当 LiDAR 一帧扫描持续时间内平台存在明显旋转或平移，而每个点带有相对时间戳时，实现 IMU deskew。它适合 LIO 前端进入 scan-to-map 前的预处理；若点云没有逐点时间，只能退化为整帧位姿补偿或关闭 deskew。

### 输入输出契约

- **输入**：当前 LiDAR 帧起止时间、逐点相对时间 `relTime`、覆盖扫描区间的 IMU 队列、可选里程计增量、点云原始坐标。
- **输出**：补偿到扫描起点或终点坐标系的点云、`imuAvailable` 标志、帧起始姿态初值。
- **时序约束**：IMU 队列至少要覆盖 `[timeScanCur, timeScanEnd]`，常见实现会额外保留少量时间余量。

### 实现骨架（伪代码）

```pseudo
function prepareImuDeskew(imu_queue, scan_start, scan_end):
    drop imu where imu.time < scan_start - margin
    rot_table = [(first_imu.time, zero_rotation)]
    for imu in imu_queue until imu.time > scan_end + margin:
        omega = transformAngularVelocityToLidarFrame(imu.gyro)
        dt = imu.time - rot_table.last.time
        rot_table.append(rot_table.last.rot + omega * dt)
    imu_available = len(rot_table) > 1

function deskewPoint(point, rel_time):
    point_time = scan_start + rel_time
    rot = interpolate(rot_table, point_time)
    pos = interpolateOdomOrZero(rel_time)
    if first_point: T_start_inv = inverse(T(pos, rot))
    return T_start_inv * T(pos, rot) * point
```

### 关键源码片段

`raw/codes/LIO-SAM/src/imageProjection.cpp:L322-L361`

```cpp
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
```

`raw/codes/LIO-SAM/src/imageProjection.cpp:L489-L519`

```cpp
    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }
```

### 实现注意事项

- LIO-SAM 示例只启用了旋转 deskew，`findPosition` 中平移补偿被注释；高速平台不应照搬这个假设。
- `relTime` 单位必须与扫描时间单位一致，Livox/Velodyne/Ouster 的字段含义不同，需要在预处理阶段统一。
- 第一帧点的变换被当作参考系，若点云排序不稳定，应显式使用扫描起点而不是“第一个有效点”。
- IMU 到 LiDAR 外参必须在积分或变换前处理，否则 deskew 方向可能看似合理但残差系统性偏大。

### 源码检索锚点

- `imuDeskewInfo`
- `findRotation`
- `deskewPoint`
- `imuRotX`
- `transStartInverse * transFinal`

## 相关页面

- [[算法-LIO-SAM]]
- [[LiDAR数据管线|LiDAR去畸变]]
- [[概念-连续时间轨迹]]
- [[方法-反向传播去畸变]]
- [[传感器-IMU预处理]]
