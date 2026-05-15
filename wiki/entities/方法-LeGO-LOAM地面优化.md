---
tags: [LeGO-LOAM, LiDAR SLAM, 地面分割, 地面车辆, 特征提取, 点云分割, UGV]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp
  - raw/codes/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp
  - raw/codes/LeGO-LOAM/LeGO-LOAM/src/mapOptmization.cpp
  - raw/codes/LeGO-LOAM/LeGO-LOAM/include/utility.h
---

# LeGO-LOAM: 地面优化稀疏特征 LiDAR SLAM

> LeGO-LOAM (Lightweight and Ground-Optimized LiDAR Odometry and Mapping) 在 LOAM 的基础上引入**地面分割**和**点云聚类分割**，剔除不可靠特征（树叶、草丛等），专为 UGV（地面无人车）在可变地形上的定位建图设计。

## 系统架构

```
点云输入
  │
  ├─ [imageProjection] ─→ 距离图像投影 + 地面分割 + 点云聚类
  │     ├─ 地面点提取 (groundRemoval)
  │     └─ 非地面点聚类分割 (cloudSegmentation)
  │
  ├─ [featureAssociation] ─→ 特征提取 + 帧间匹配
  │     ├─ 平滑度计算 (同LOAM)
  │     ├─ 角点/平面点提取 (从分割后的可靠点中)
  │     └─ 两步 L-M 优化 (scan-to-scan)
  │
  ├─ [mapOptmization] ─→ scan-to-map 优化 + 回环检测
  │     └─ 关键帧选择 + ISAM2 图优化
  │
  └─ [transformFusion] ─→ IMU 里程计融合 + TF 发布
```

## 地面分割 (Ground Removal)

LeGO-LOAM 的创新核心：**利用 VLP-16 等多线 LiDAR 的垂直扫描结构，在距离图像上逐列分析相邻扫描线的角度关系，分离地面点和非地面点**。

### 距离图像投影

**源码锚点**: `raw/codes/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp:L211-L257`

首先将原始点云投影为距离图像（range image），行索引为 LiDAR 的扫描线 ID（ring），列索引为水平角度：

```cpp
void projectPointCloud(){
  for (size_t i = 0; i < cloudSize; ++i){
    // 行索引 = ring (或通过垂直角度计算)
    if (useCloudRing == true) rowIdn = laserCloudInRing->points[i].ring;
    else {
      verticalAngle = atan2(z, sqrt(x*x + y*y)) * 180 / M_PI;
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
    }
    // 列索引 = 水平角度映射
    horizonAngle = atan2(x, y) * 180 / M_PI;
    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    // 写入距离图像
    rangeMat.at<float>(rowIdn, columnIdn) = range;
  }
}
```

### 地面点提取算法

**源码锚点**: `raw/codes/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp:L260-L310`

核心思想：**在同一列、相邻两扫描线之间，计算两点连线与水平面的夹角。如果夹角小于阈值（默认 10°），则判定为地面点。**

```cpp
void groundRemoval(){
    for (size_t j = 0; j < Horizon_SCAN; ++j){       // 每列
        for (size_t i = 0; i < groundScanInd; ++i){   // 地面扫描线范围[0, groundScanInd)
            lowerInd = j + i*Horizon_SCAN;           // 下方扫描线点
            upperInd = j + (i+1)*Horizon_SCAN;       // 上方扫描线点

            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10){   // 夹角 ≤ 10°
                groundMat.at<int8_t>(i,j) = 1;         // 标记为地面
                groundMat.at<int8_t>(i+1,j) = 1;       // 上方相邻线也标记
            }
        }
    }
}
```

算法细节：
- 只遍历 `groundScanInd` 行以下的扫描线（VLP-16 配置下 `groundScanInd=7`，即下方 8 条线被视为可能的地面扫描范围）
- `sensorMountAngle` 为 LiDAR 安装俯仰角（默认 0°）
- 标记 `groundMat` 同时用于后续分割排除地面点

为什么有效：地面点在相邻扫描线间表现为几乎水平的连线（与水平面夹角小），而非地面点（垂直结构如墙壁、树）则呈现较大的垂直角度。

## 点云聚类分割 (Cloud Segmentation)

**源码锚点**: `raw/codes/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp:L312-L400`

对非地面点进行连通域分割（BFS on range image）：

```
labelComponents(i, j):
    BFS from (i, j) on rangeMat
    邻域: 4-邻域
    条件: 相邻点深度差 < 阈值 (segmentTheta)
    标记: labelCount 递增
    小簇 (点数 < 30): 标记为 999999 (outlier)，不用于特征提取
```

```cpp
// 小簇剔除
if (labelMat.at<int>(i,j) == 999999){
    if (i > groundScanInd && j % 5 == 0){
        outlierCloud->push_back(...);  // 保留少量作为可视化
    }
    continue;  // 不参与后续特征提取
}
```

分割后的处理策略：
- **地面点**：下采样保留（`j%5!=0 && j>5 && j<Horizon_SCAN-5` 时跳过），用于平面特征
- **可靠分割点**：`labelCount > 30`，正常提取特征
- **小簇/噪声点**（树叶、草丛）：标记为 outlier，不提取特征

**核心价值**：地面分割 + 聚类分割使 LeGO-LOAM 只从**稳定可靠的物体表面**提取特征，大幅减少了环境噪声对里程计的影响，提高了在非结构化环境中的鲁棒性。分割后的可靠点数量通常比原始点云减少 30-50%。

## 特征提取

**源码锚点**: `raw/codes/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp:L250-L450`

与 LOAM 相同的平滑度公式，但仅从分割后的可靠点中提取：

$$
c = \frac{1}{|\mathcal{S}| \cdot \| \mathbf{r}_i \|} \left\| \sum_{j \in \mathcal{S}, j \neq i} (\mathbf{r}_j - \mathbf{r}_i) \right\|
$$

特征提取策略：
- 将点云按水平角度分为 6 个扇区，每个扇区独立提取
- 每个扇区取 `edgeFeatureNum=2` 个最大平滑度（角点）和 `surfFeatureNum=4` 个最小平滑度（平面点）
- 邻域抑制：已选特征周围 `5×5` 范围不重复选点

## 帧间匹配 (Scan-to-Scan Odometry)

采用两步 L-M 优化（与 LOAM 一致），但增加了地面约束：

1. **角点匹配**：`cornerPointsSharp` → 上帧 `cornerPointsLessSharp`，edge-to-edge 距离
2. **平面点匹配**：`surfPointsFlat` → 上帧 `surfPointsLessFlat`，point-to-plane 距离

两步优化的好处：先用角点约束旋转（旋转对平移的耦合较小），固定旋转后再优化平移，避免局部极值。

## UGV 针对性优化总结

| 优化点 | 方法 | 收益 |
|--------|------|------|
| 地面分割 | 逐列角度阈值 | 分离地面结构，提取更稳定的平面约束 |
| 点云聚类 | BFS 连通域 + 小簇剔除 | 过滤树叶、草丛等不稳定特征 |
| 特征降采样 | 地面点下采样 + 可靠点筛选 | 减少 30-50% 特征点，加速计算 |
| 扇区均衡 | 6 扇区独立提取 | 避免特征在单一方向集中 |
| IMU 紧耦合 | 9-DOF IMU 预积分 | 去除运动畸变，提供初始位姿 |

相比 LOAM 的改进：LeGO-LOAM 通过预处理阶段的语义级分割（地面/非地面/噪声），使 L-M 优化用到的特征更加可靠，特别适合地面无人车在城市、乡村、园区等有地面平面的环境中运行。

## Agent 实现提示

### 适用场景

UGV（地面无人车）在非结构化环境中的 LiDAR SLAM，需要快速、轻量的前端里程计。地面平面存在且 LiDAR 安装高度已知的场景最优。VLP-16/HDL-32E/OS1 等多线机械旋转 LiDAR 可直接使用（Ouster 需要适配 ring 字段）。

### 输入输出契约

- **输入**：LiDAR 点云（`PointXYZI` 或 `PointXYZIR`，含 ring）、9-DOF IMU（可选但推荐）、传感器安装参数（`N_SCAN`、`Horizon_SCAN`、`groundScanInd`、`sensorMountAngle`）
- **输出**：6-DOF 里程计位姿、分割后的点云（地面、分割、异常点）、角点和平面特征

### 实现骨架（伪代码）

```pseudo
function LegoLoam.run():
    // 阶段1: 预处理
    for each LiDAR scan:
        project_pointcloud_to_range_image(scan, N_SCAN, Horizon_SCAN)
        // 地面分割
        for col in 0..Horizon_SCAN:
            for row in 0..groundScanInd:
                angle = atan2(dz, sqrt(dx*dx+dy*dy))
                if |angle - mount_angle| <= 10°:
                    mark ground
        // 聚类分割
        for each unlabeled pixel:
            BFS cluster in 4-neighborhood
            if cluster_size < 30: mark as outlier

    // 阶段2: 特征提取（仅从可靠点）
    for each sub-region (6 sectors):
        compute smoothness c
        extract top edge points (max c)
        extract top surf points (min c), prefer ground

    // 阶段3: scan-to-scan 优化
    LM_optimize(corner_to_edge_residuals)
    LM_optimize(surface_to_plane_residuals)

    // 阶段4: scan-to-map 优化 + 回环
    if keyframe:
        add to local submap
        scan_to_map_registration()
        if loopClosureEnableFlag:
            detect_and_optimize_loop()
```

### 关键源码片段

**地面角度判断**（`imageProjection.cpp:L280-L289`）：
```cpp
diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
if (abs(angle - sensorMountAngle) <= 10){
    groundMat.at<int8_t>(i,j) = 1;
    groundMat.at<int8_t>(i+1,j) = 1;
}
```

**小簇剔除逻辑**（`imageProjection.cpp:L326-L340`）：
```cpp
if (labelMat.at<int>(i,j) == 999999){
    if (i > groundScanInd && j % 5 == 0){
        outlierCloud->push_back(...);
        continue;
    } else { continue; }
}
if (groundMat.at<int8_t>(i,j) == 1){
    if (j%5!=0 && j>5 && j<Horizon_SCAN-5) continue;
}
```

### 实现注意事项

1. **`groundScanInd` 是关键参数**：根据 LiDAR 安装高度和扫描线数量决定哪些线可能看到地面。VLP-16 安装在 1.8m 高度时设为 7，更低安装需调小
2. **`sensorMountAngle`**：LiDAR 的俯仰安装角（从水平面算起），不平装时需要精确设置
3. **Ouster LiDAR 适配**：需取消 `cloudHeader.stamp` 注释，Ouster 的 ring 字段名不同，需映射
4. **分割参数**：`segmentTheta`（默认 60°）决定两点视为连通的角度阈值，值越小越严格
5. **地面点降采样率**：默认 `j%5!=0`，可根据场景密度调整

### 源码检索锚点

| 模块 | 文件 | 行号 |
|------|------|------|
| 点云投影 | `imageProjection.cpp` | L211-L257 |
| 地面分割 | `imageProjection.cpp` | L260-L310 |
| 聚类分割 | `imageProjection.cpp` | L312-L400 |
| 主处理流程 | `imageProjection.cpp` | L181-L197 |
| 平滑度计算 | `featureAssociation.cpp` | L250+ |
| 参数配置 | `utility.h` | L62-L68 |
| 两种 LiDAR 参数 | `utility.h` | L62-L84 |

## 相关页面

- [[方法-KISS-ICP简易ICP]]
- [[方法-ROLO-SLAM旋转解耦]]
- [[方法-地面车辆运动约束]]
- [[方法-分离式旋转-平移估计]]
