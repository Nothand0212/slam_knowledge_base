---
tags: [BEV, 投影, 地面分割, LiDAR, BEV-LSLAM, 回环检测, 地点识别, 高度图, 强度图]
type: entity
created: 2026-05-15
updated: 2026-05-15
sources: []
---

# BEV 投影工程

> BEV-LSLAM 的核心数据转换：将 3D LiDAR 点云压缩为 200×200 像素的鸟瞰图（Bird's Eye View），通过高度图 + 强度图双通道互补，使 LiDAR 地点识别可复用 ORB 特征、GMS 匹配和 DBOW3 词袋等成熟视觉工具链。

## 数学表述

### 投影映射

给定 LiDAR 点云 $\mathcal{P} = \{\mathbf{p}_i = (p_{i,x}, p_{i,y}, p_{i,z}, I_i)\}$ 中的每个点，BEV 投影将其映射到图像坐标 $(u, v)$：

$$u = \frac{H/2 - p_{i,x}}{r}, \quad v = \frac{W/2 - p_{i,y}}{r}$$

其中 $H$ 和 $W$ 是 BEV 图像覆盖的物理范围（单位：米），$r$ 是像素分辨率。坐标原点（图像中心）对应 LiDAR 传感器的地面投影点。

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L268-L269` — 坐标映射实现：

```cpp
int BEV_X = (image_height/2 - point_temp.x) / image_resolution;
int BEV_Y = (image_length/2 - point_temp.y) / image_resolution;
```

### 参数规范

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `image_height` | 80 m | X 方向感知范围 (±40m) |
| `image_length` | 80 m | Y 方向感知范围 (±40m) |
| `image_resolution` | 0.4 m/pixel | 单像素物理尺寸 |
| `BEV_rows` | 200 | height / resolution |
| `BEV_cols` | 200 | length / resolution |
| `min_distance` | 2 | 最小编码值（防止全零区域） |

## 双通道生成

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L246-L309` 实现了完整的高度图 + 强度图生成流程：

### 高度图 (Height Map)

对每个像素 $(u, v)$，取落在该像素区域内所有点的**最大 z 值**，归一化到 `[min_distance, 255]`：

$$\text{BEV\_height}(u, v) = \max_{i: (u_i, v_i) = (u, v)} \left( (255 - d_{\min}) \cdot \frac{p_{i,z} - z_{\min}}{z_{\max} - z_{\min}} + d_{\min} \right)$$

```cpp
// raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L271-L278
int norm_z = (255-min_distance)*((point_temp.z - min_z)/(max_z - min_z))+min_distance;
if(norm_z > BEV_hight.at<uchar>(BEV_X, BEV_Y)) {
    BEV_hight.at<uchar>(BEV_X, BEV_Y) = norm_z;
    height_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
}
```

### 强度图 (Intensity Map)

对每个像素 $(u, v)$，取落在该像素区域内所有点的**最大强度值**，归一化到 `[min_distance, 255]`：

$$\text{BEV\_intensity}(u, v) = \max_{i: (u_i, v_i) = (u, v)} \left( (255 - d_{\min}) \cdot \frac{I_i - I_{\min}}{I_{\max} - I_{\min}} + d_{\min} \right)$$

```cpp
// raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L280-L284
int norm_intensity = (255-min_distance)*((point_temp.intensity - min_intensity)
                        /(max_intensity - min_intensity))+min_distance;
if(norm_intensity > BEV_intensity.at<uchar>(BEV_X, BEV_Y)) {
    BEV_intensity.at<uchar>(BEV_X, BEV_Y) = norm_intensity;
    intensity_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
}
```

**设计意图**：高度图捕获几何结构（建筑物立面产生高值，平坦路面产生低值且均匀），强度图捕获材质差异（车道线、路面标线有反射率差异），二者互补增强地点区分度。

## 后处理管线

```
原始 BEV 图像 (高度图 / 强度图)
├── pixel_detect: 邻域填充 (填补因稀疏点云产生的空洞)
│   对每个零值像素，若 3×3 邻域内非零点数 > threshold，取邻域均值填充
├── equalizeHist: 直方图均衡化 (增强对比度)
├── GaussianBlur: 高斯模糊
│   ├── 高度图: 5×5 kernel
│   └── 强度图: 3×3 kernel
└── MASK: 边缘裁切 (左右各 256 像素 = 102.4m 外的无效区域置零)
```

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L290-L299`：

```cpp
pixel_detect(height_cloud, &BEV_hight);
pixel_detect_intensity(intensity_cloud, &BEV_intensity);
cv::equalizeHist(BEV_hight, BEV_hight);
cv::GaussianBlur(BEV_hight, BEV_hight, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
cv::equalizeHist(BEV_intensity, BEV_intensity);
cv::GaussianBlur(BEV_intensity, BEV_intensity, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
```

## BEV 特征提取与匹配

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L311-L343`：

### ORB 特征提取

对高度图和强度图分别提取 2500 个 ORB 特征点：

```cpp
orb->ORB_feature(BEV_now.BEV_height);
BEV_now.keypoints_height = orb->mvKeys;
BEV_now.description_height = orb->mDescriptors;
orb->ORB_feature(BEV_now.BEV_intensity);
BEV_now.keypoints_intensity = orb->mvKeys;
BEV_now.description_intensity = orb->mDescriptors;
```

### GMS 匹配验证

对两帧之间的 ORB 匹配应用 GMS（Grid-based Motion Statistics）验证，过滤误匹配：

```cpp
cv::BFMatcher matcher(NORM_HAMMING);
matcher.match(BEV_now.description_height, BEV_last.description_height, matches_height);
gms_matcher gms_height(BEV_now.keypoints_height, BEV_now.BEV_height.size(),
                        BEV_last.keypoints_height, BEV_last.BEV_height.size(),
                        matches_height);
int num_inliers_height = gms_height.GetInlierMask(inlierMatches_height);
```

GMS 基于"正确匹配的邻域内应有更多支持匹配"的平滑约束，在无额外几何信息的情况下高效剔除误匹配。

## 回环检测管线

BEV-LSLAM 的回环检测复用视觉词袋范式（DBOW3）：

```
当前关键帧 BEV 图像
├── 高度图 → ORB 描述子 → DBOW3 向量 → 词袋相似度检索
├── 强度图 → ORB 描述子 → DBOW3 向量 → 词袋相似度检索
├── 候选关键帧筛选 (历史关键帧搜索半径 10m, 搜索数量 50)
├── 距离/角度过滤 (历史回环几何距离 > 20m)
├── FastVGICP 几何配准验证 (fitness_score < 0.6)
└── 位姿图优化回环边
```

`raw/codes/BEV-LSLAM/orb_lio/src/scantomap_kitti.cpp:L200-L220` — 关键参数：

```cpp
float historyKeyframeSearchRadius = 10;   // 子地图构建搜索半径
float historyKeyframeFitnessScore = 0.6;  // ICP fitness 阈值
int historyKeyframeSearchNum = 50;         // 历史关键帧融合数量
float loopKeyframeDisDiff = 20;             // 回环最小距离间隔
float DRIFT_FACTOR = 0.02;                 // 漂移系数
```

## 前端里程计

BEV-LSLAM 的前端基于 LOAM/LIO-SAM 的边-面特征配准范式：

```
去畸变点云
├── 边缘特征提取 (根据局部曲率)
├── 平面特征提取
├── scan-to-scan 边特征匹配 (Ceres 优化, 2轮迭代, 自适应权重)
│   ├── 角点→边线距离残差
│   └── 平面点→面距离残差
├── scan-to-map 面特征匹配 (FastVGICP 精配准)
└── 关键帧判断 (位移 > 1.0m 或 旋转 > 0.2rad)
```

配准使用 FastVGICP (`raw/codes/BEV-LSLAM/orb_lio/src/fast_gicp/gicp/fast_vgicp.cpp`)，通过体素化协方差近似实现比标准 GICP 更快的计算。

## 信息损失与工程边界

| 信息损失 | 原因 | 影响 |
|----------|------|------|
| 多层结构丢失 | 高度图取最大值 | 立交桥、隧道、树冠下结构不可区分 |
| 3D 几何退化 | XY 平面压缩 | 坡度、高程变化信息丢失 |
| 物体形状扭曲 | 透视投影压缩 | 车辆侧面与顶面混合在同一像素 |
| 材质泛化弱 | 强度依赖 LiDAR 型号和入射角 | 跨设备/跨数据集匹配不稳定 |
| 远距分辨率低 | 固定像素分辨率 0.4m | 远处物体合并为同一像素 |

**核心原则**：BEV 投影适合作为地点识别/回环候选的粗匹配表示，不应替代 3D 几何配准。BEV-LSLAM 中 BEV 仅用于 DBOW3 回环检测，最终的几何约束仍由 FastVGICP 3D 配准提供。

## Agent 实现提示

### 适用场景

当需要为 LiDAR SLAM 系统引入基于图像的回环检测/地点识别能力，但不希望引入额外的相机传感器和同步标定负担时，使用 BEV 投影将 LiDAR 点云转换为图像描述子。适用于室外结构化环境（城市道路、园区），不适合多层立交、密集森林、室内环境。

### 输入输出契约

- **输入**：
  - LiDAR 点云：`pcl::PointCloud<PointXYZI>`，intensity 被缩放到 [0, 255]
  - BEV 参数：`image_height`, `image_length`, `image_resolution`
  - 标定：LiDAR-IMU 外参 `R_il`, `t_rl`
- **输出**：
  - 高度图：`cv::Mat` (8UC1, BEV_rows × BEV_cols)
  - 强度图：`cv::Mat` (8UC1, BEV_rows × BEV_cols)
  - 高度特征点云和强度特征点云（每个像素最多保留一个 3D 点）
  - ORB 关键点和描述子
- **坐标约定**：点云在 LiDAR 坐标系（x 向前，y 向左，z 向上），BEV 图像原点在左上角，x→行，y→列

### 实现骨架（伪代码）

```pseudo
function bevProject(pointcloud, params):
    rows = params.image_height / params.resolution
    cols = params.image_length / params.resolution
    height_img = zeros(rows, cols, uint8)
    intensity_img = zeros(rows, cols, uint8)
    height_cloud = PointCloud(rows * cols)  // 每个像素保留一个3D点
    intensity_cloud = PointCloud(rows * cols)

    // 第一遍：统计极值
    z_min, z_max, i_min, i_max = findExtrema(pointcloud, params.bounds)

    // 第二遍：填充图像
    for each point p in pointcloud:
        if outOfBounds(p.x, p.y, params): continue
        u = (params.height/2 - p.x) / params.resolution
        v = (params.length/2 - p.y) / params.resolution
        norm_z = normalize(p.z, z_min, z_max, params.min_distance, 255)
        norm_i = normalize(p.intensity, i_min, i_max, params.min_distance, 255)
        if norm_z > height_img(u, v):
            height_img(u, v) = norm_z
            height_cloud(u * cols + v) = p
        if norm_i > intensity_img(u, v):
            intensity_img(u, v) = norm_i
            intensity_cloud(u * cols + v) = p

    // 后处理
    fillHoles(height_cloud, height_img)   // pixel_detect
    fillHoles(intensity_cloud, intensity_img)
    equalizeHist(height_img)
    equalizeHist(intensity_img)
    GaussianBlur(height_img, kernel=5)
    GaussianBlur(intensity_img, kernel=3)

    return height_img, intensity_img, height_cloud, intensity_cloud

function bevLoopDetection(cur_bev, history_bevs, voc):
    // 1. ORB 特征
    kp_h, desc_h = extractORB(cur_bev.height_img, nFeatures=2500)
    kp_i, desc_i = extractORB(cur_bev.intensity_img, nFeatures=2500)

    // 2. DBOW3 检索
    candidates = voc.query(desc_h, topK=50).intersect(voc.query(desc_i, topK=50))

    // 3. 几何过滤
    for each candidate in candidates:
        if geometricDistance(cur_bev.pose, candidate.pose) < 20m: continue

    // 4. GMS + ICP 验证
    for each candidate in candidates:
        matches = bruteForceMatch(cur_bev.desc_h, candidate.desc_h)
        gms_inliers = gmsVerify(kp_h, candidate.kp_h, matches, img_size)
        if gms_inliers < threshold: continue
        T = fastVgicpAlign(cur_bev.cloud, candidate.cloud)
        if T.fitness > 0.6: continue
        addLoopEdge(cur_bev, candidate, T)

    return detected_loops
```

### 关键源码片段

**BEV 投影核心循环**

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L246-L287`

```cpp
cv::Mat BEV_hight, BEV_intensity;
BEV_hight = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
BEV_intensity = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);

for(int i = 0; i < cloudSize; i++) {
    PointType point_temp = laserCloudIn.points[i];
    // bounds check
    if(point_temp.x < (-image_height/2+0.5) || point_temp.x > (image_height/2-0.5)
        || point_temp.y < (-image_length/2+0.5) || point_temp.y > (image_length/2-0.5))
        continue;

    int BEV_X = (image_height/2 - point_temp.x) / image_resolution;
    int BEV_Y = (image_length/2 - point_temp.y) / image_resolution;

    int norm_z = (255-min_distance)*((point_temp.z - min_z)/(max_z - min_z)) + min_distance;
    int norm_intensity = (255-min_distance)*((point_temp.intensity - min_intensity)
                          / (max_intensity - min_intensity)) + min_distance;

    if(norm_z > BEV_hight.at<uchar>(BEV_X, BEV_Y)) {
        BEV_hight.at<uchar>(BEV_X, BEV_Y) = norm_z;
        height_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
    }
    if(norm_intensity > BEV_intensity.at<uchar>(BEV_X, BEV_Y)) {
        BEV_intensity.at<uchar>(BEV_X, BEV_Y) = norm_intensity;
        intensity_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
    }
}
```

### 实现注意事项

1. **强度值预处理**：`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L222` 中 `laserCloudIn.points[i].intensity = laserCloudIn.points[i].intensity * 255`。不同的 LiDAR 型号（Velodyne HDL-64E vs Ouster OS1）原始强度范围不同（0-1 或 0-255），必须在投影前统一归一化。
2. **取最大值语义**：高度图取 max z → 捕获建筑物/护栏等垂直结构顶部；强度图取 max intensity → 捕获高反射率路标材质。这意味着路面（低 z、低强度）在双通道中均表现为低值，对平坦路面的地点区分依赖稀疏的垂直结构投影像素。
3. **空洞填充 `pixel_detect`**：稀疏 LiDAR（如 32 线）在远距离会产生大量空像素，`pixel_detect` 对零值像素取邻域均值填充，但局部信息是内插而非真实观测。如果相邻多帧 BEV 叠加可在一定程度上缓解，但需注意运动物体的拖影。
4. **GaussianBlur 差异**：高度图用 5×5 核，强度图用 3×3 核——因为强度图噪声更大且细节更丰富，大核会模糊路标边缘，降低 ORB 角点的可重复性。
5. **DBOW3 词汇需预训练**：`voc = new DBoW3::Vocabulary(voc_path)`，词汇文件需用目标环境的 BEV 图像离线训练，直接复用图像数据集（如 KITTI 原始图像）训练的词汇在 BEV 域泛化差。
6. **信息矩阵计算**：BEV 匹配的平移和旋转不是独立估计的，且视觉特征在 BEV 域的深度信息只有通过关联的 3D 点云间接获得，因此由 BEV 匹配衍生的回环边的信息矩阵应比 LiDAR 直接配准低一个量级。

### 源码检索锚点

- BEV 投影核心：`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L210-L330`
- BEV 回环检测：`raw/codes/BEV-LSLAM/orb_lio/src/scantomap_kitti.cpp:L500-L750`
- 前端里程计配准：`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L330-L550`
- GMS 匹配：`raw/codes/BEV-LSLAM/orb_lio/include/orb_feature/gms_matcher.hpp`
- ORB 修改版特征：`raw/codes/BEV-LSLAM/orb_lio/include/orb_feature/ORB_modify.hpp`
- DBOW3 词袋：`raw/codes/BEV-LSLAM/orb_lio/src/DBoW3/`
- FastVGICP 配准：`raw/codes/BEV-LSLAM/orb_lio/src/fast_gicp/gicp/fast_vgicp.cpp`
- 配置文件/参数：`raw/codes/BEV-LSLAM/orb_lio/`

## 相关页面

- [[方法-BEV投影]]
- [[方法-GMS匹配验证]]
- [[方法-回环验证方法族]]
- [[方法-Fast-VGICP]]
- [[方法-Intensity Scan Context]]
- [[LiDAR数据管线]]
