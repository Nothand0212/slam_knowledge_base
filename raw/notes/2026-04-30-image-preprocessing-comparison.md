# 图像预处理源码调查与影响分析

> 范围：围绕“预处理算法对比”和“对前端、后端精度及观测模型的影响”两个方向，梳理当前 workspace 中图像相关 SLAM/VIO/LIVO 项目的源码证据。
> 日期：2026-04-30

---

## 1. 结论先行

图像预处理在这些项目里不是一个统一模块，而是直接绑定到观测模型：

1. **KLT/角点类 VIO** 主要关心能否稳定跟踪足够多、分布均匀的特征点。可调查项是灰度化、CLAHE/均衡化、金字塔、mask/grid、稀疏点去畸变。
2. **ORB/描述子类 SLAM** 主要关心特征尺度一致性和双目几何一致性。可调查项是 resize/rectify、灰度化、ORB pyramid、FAST 阈值、关键点去畸变。
3. **直接法/半直接法** 的预处理就是光度残差模型的一部分。可调查项是响应函数、暗角、曝光、仿射亮度、patch 金字塔、梯度选点。这里不应把 CLAHE/直方图均衡当成通用增强。
4. **学习式/稠密方法** 的预处理是网络或可微渲染输入契约。可调查项是 BGR/RGB 顺序、`/255`、mean/std、resize/crop 后内参更新、ray sampling、color/depth loss。
5. **LIVO/RGB map 项目** 混合了几何预处理和光度预处理，例如 R3LIVE 会 resize、remap、CLAHE；FAST-LIVO2 则在 patch 残差里显式估计曝光。

最重要的判断：**能改善 KLT 特征跟踪的图像增强，不一定适合直接法后端；能降低算力的 resize/downsample，如果没有同步修改内参，会直接制造系统性几何误差。**

---

## 2. 预处理项横向矩阵

| 预处理项 | 代表项目 | 源码入口 | 前端作用 | 后端/观测模型影响 | 建议 |
|---|---|---|---|---|---|
| 降采样/resize/crop | OpenVINS, ORB-SLAM3, DROID-SLAM, NICE-SLAM, FAST-LIVO2, R3LIVE | OpenVINS `cv::pyrDown`: `open_vins/ov_msckf/src/core/VioManager.cpp:268`; OpenVINS 内参同步缩放: `open_vins/ov_msckf/src/core/VioManagerOptions.h:251`; DROID intrinsics scale/crop: `DROID-SLAM/droid_slam/data_readers/stream.py:51`; NICE `update_cam`: `nice-slam/src/NICE_SLAM.py:113`; FAST-LIVO2 resize: `FAST-LIVO2/src/vio.cpp:1786`; R3LIVE resize: `r3live/r3live/src/r3live_vio.cpp:402` | 降低计算量，匹配模型输入分辨率 | 必须同步变换 `fx/fy/cx/cy` 和图像尺寸；否则三角化、重投影、ray direction 都会偏 | 每个项目先检查 resize 是否发生在相机模型初始化前后，以及内参是否一起更新 |
| 去畸变/立体校正/remap | Kimera-VIO, ORB-SLAM3/OpenMAVIS, ESVO, R3LIVE | Kimera `cv::remap`: `Kimera-VIO/src/frontend/UndistorterRectifier.cpp:115`; Kimera map 初始化: `Kimera-VIO/src/frontend/UndistorterRectifier.cpp:230`; ORB-SLAM3 stereo remap: `ORB_SLAM3/src/System.cc:253`; ESVO map/mask: `ESVO/esvo_core/src/container/CameraSystem.cpp:60`; R3LIVE `initUndistortRectifyMap` + `remap`: `r3live/r3live/src/r3live_vio.cpp:394` | 双目行对齐，消除畸变导致的非线性像素运动 | 后端残差可以在 rectified pixel 坐标、undistorted keypoint 或 normalized bearing 上定义；坐标系选择必须一致 | 区分“整图 remap”和“稀疏点 lift/undistort”，不要混用像素误差阈值 |
| 稀疏点去畸变/归一化 | VINS-Fusion, OpenVINS, ORB-SLAM3, LVI-SAM | VINS `liftProjective`: `VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:391`; OpenVINS KLT 后 normalized RANSAC: `open_vins/ov_core/src/track/TrackKLT.cpp:860`; ORB keypoint undistort: `ORB_SLAM3/src/Frame.cc:747`; LVI-SAM undistorted points: `LVI-SAM/src/visual_odometry/visual_feature/feature_tracker.cpp:258` | 在原图上跟踪，在几何检验或后端前转换到归一化坐标 | RANSAC 阈值、速度估计、重投影残差单位发生变化；像素阈值不能直接照搬 | 文档里要标出每个阈值是在 pixel、rectified pixel 还是 normalized plane |
| 直方图均衡/CLAHE | OpenVINS, LVI-SAM, R3LIVE, VINS-Fusion 注释代码 | OpenVINS `HISTOGRAM/CLAHE`: `open_vins/ov_core/src/track/TrackKLT.cpp:56`; OpenVINS 配置: `open_vins/ov_msckf/src/core/VioManagerOptions.h:435`; LVI-SAM CLAHE: `LVI-SAM/src/visual_odometry/visual_feature/feature_tracker.cpp:87`; R3LIVE CLAHE: `r3live/r3live/src/rgb_map/image_frame.cpp:272`; VINS-Fusion 注释掉的 CLAHE: `VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:102` | 提升弱光、低纹理场景下角点与 KLT 成功率 | 对几何特征法风险较小；对直接法会改变亮度函数，可能破坏光度一致性 | 只建议在 KLT/角点链路中做 ablation；直接法需优先用光度标定/曝光模型 |
| 图像金字塔 | OpenVINS, VINS/MSCKF/LVI KLT, ORB-SLAM3/OpenMAVIS, DSO/DM-VIO, ROVIO, SVO, FAST-LIVO2 | OpenVINS build pyramid: `open_vins/ov_core/src/track/TrackKLT.cpp:69`; ORB pyramid: `ORB_SLAM3/src/ORBextractor.cc:1096`; DSO pyramid/gradient: `dso/src/FullSystem/HessianBlocks.cpp:128`; ROVIO pyramid: `rovio/include/rovio/ImagePyramid.hpp:81`; SVO pyramid: `rpg_svo_pro_open/svo_common/src/frame.cpp:372`; FAST-LIVO2 pyramid helper: `FAST-LIVO2/src/frame.cpp:54` | coarse-to-fine 跟踪，扩大运动捕获范围 | 不同层的尺度、梯度、噪声和 patch 尺寸会影响残差权重、鲁棒核和阈值 | 比较时不要只看“层数”，还要看每层是否重新计算梯度、是否同步缩放误差阈值 |
| 光度标定/曝光补偿 | DSO, DM-VIO, ROVIO, SVO, MonoGS, FAST-LIVO2 | DSO response/vignette/exposure: `dso/src/util/Undistort.cpp:213`; DSO remap 后光度图像: `dso/src/util/Undistort.cpp:386`; DM-VIO mode: `dm-vio/src/util/MainSettings.cpp:130`; ROVIO patch brightness fix: `rovio/include/rovio/MultilevelPatchAlignment.hpp:238`; SVO affine illumination: `rpg_svo_pro_open/svo/include/svo/frame_handler_base.h:140`; MonoGS exposure params: `MonoGS/utils/slam_backend.py:457`; FAST-LIVO2 exposure residual: `FAST-LIVO2/src/vio.cpp:1619` | 抗曝光、暗角、相机响应变化 | 直接进入 residual 公式，是状态量、优化变量或强先验的一部分 | 这是直接法核心，不要用“图像增强”替代；优先调查 residual 公式 |
| 特征/像素筛选 | VINS-Fusion, Kimera, OpenVINS, MSCKF, DSO, FAST-LIVO2, NICE-SLAM | VINS `goodFeaturesToTrack`: `VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:184`; Kimera detection mask: `Kimera-VIO/src/frontend/feature-detector/FeatureDetector.cpp:174`; MSCKF FAST/grid: `msckf_vio/src/image_processor.cpp:723`; DSO gradient hist: `dso/src/FullSystem/PixelSelector2.cpp:78`; FAST-LIVO2 grid/map point retrieval: `FAST-LIVO2/src/vio.cpp:144`; NICE ray sampling: `nice-slam/src/common.py:92` | 控制特征数量、空间分布和外点率 | 影响三角化基线、信息矩阵条件数、EKF/BA 可观测性和动态物体鲁棒性 | 与图像增强一起实验：看 tracked count、inlier ratio、spatial entropy，而不是只看 ATE |
| 网络输入归一化 | DROID-SLAM, MonoGS, NICE-SLAM | DROID BGR->RGB + ImageNet normalize: `DROID-SLAM/droid_slam/droid_net.py:154`; DROID training jitter/crop: `DROID-SLAM/droid_slam/data_readers/augmentation.py:12`; MonoGS PIL->tensor: `MonoGS/gaussian_splatting/utils/general_utils.py:24`; NICE ray/depth/color sample: `nice-slam/src/common.py:125` | 满足训练分布和网络输入尺寸 | 网络特征、相关体、可微渲染 loss 都依赖这个输入契约 | 不建议任意加传统增强；先保证 resize/crop 与训练配置一致 |

---

## 3. 按观测模型分组

### 3.1 KLT/角点 VIO：VINS-Fusion、OpenVINS、MSCKF、Kimera、LVI-SAM

这类前端通常是：

```text
灰度图/可选增强 -> KLT 金字塔跟踪 -> mask/grid 补点 -> 稀疏点去畸变 -> RANSAC/后端
```

关键源码证据：

- OpenVINS 可以先整体降采样图像和 mask，随后进入 tracking：`open_vins/ov_msckf/src/core/VioManager.cpp:268`。
- OpenVINS 在读取相机标定时会随 `downsample_cameras` 同步除以 2 缩放 `fx/fy/cx/cy` 和分辨率：`open_vins/ov_msckf/src/core/VioManagerOptions.h:251`。
- OpenVINS KLT 前支持 `NONE/HISTOGRAM/CLAHE`，并构建 optical-flow pyramid：`open_vins/ov_core/src/track/TrackKLT.cpp:56`、`open_vins/ov_core/src/track/TrackKLT.cpp:69`。
- OpenVINS 在 RANSAC 前把点转成 normalized/undistorted 坐标，避免 distorted UV 的非线性：`open_vins/ov_core/src/track/TrackKLT.cpp:860`。
- VINS-Fusion 的主路径是 KLT + Shi-Tomasi，CLAHE 代码存在但被注释：`VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:102`、`VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:120`、`VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:184`。
- VINS-Fusion 后续通过 `cam->liftProjective` 转归一化坐标：`VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:391`。
- LVI-SAM 的视觉前端更接近 VINS，区别是 `EQUALIZE` 开关会真正启用 CLAHE：`LVI-SAM/src/visual_odometry/visual_feature/feature_tracker.cpp:87`。
- Kimera 通过 IMU/旋转预测 sparse flow，再调用 `calcOpticalFlowPyrLK`：`Kimera-VIO/src/frontend/Tracker.cpp:127`。
- Kimera 的 feature detector 会用 mask 避免在已有 tracked landmarks 附近重复补点：`Kimera-VIO/src/frontend/feature-detector/FeatureDetector.cpp:174`。
- MSCKF 读取 MONO8、先构建金字塔，再做 KLT 和 grid FAST 补点：`msckf_vio/src/image_processor.cpp:215`、`msckf_vio/src/image_processor.cpp:459`、`msckf_vio/src/image_processor.cpp:723`。

影响链路：

```text
CLAHE/均衡化
  -> 更多角点 / 更强局部梯度
  -> KLT 成功率和 RANSAC inlier ratio 提高
  -> 后端获得更多可用约束
  -> 但低纹理噪声可能被放大，外点率和短寿命 tracks 也可能升高

downsample
  -> 计算量降低、KLT 更快
  -> 小尺度纹理丢失、角点数量下降
  -> 若内参同步缩放正确，几何模型不变；若不同步，所有 bearing/reprojection 都偏
```

这组最适合做 `NONE/HISTOGRAM/CLAHE`、`downsample on/off`、`min_px_dist/grid` 的对比实验。

### 3.2 ORB/描述子类：ORB-SLAM3、OpenMAVIS

这类前端通常是：

```text
可选 rectify/resize -> 灰度化 -> ORB pyramid -> FAST/OctTree 分布 -> descriptor -> keypoint undistort -> tracking/BA
```

关键源码证据：

- ORB-SLAM3 双目入口会根据配置先 `cv::remap` 或 `cv::resize`：`ORB_SLAM3/src/System.cc:253`、`ORB_SLAM3/src/System.cc:262`。
- ORB-SLAM3 monocular 入口也支持 resize：`ORB_SLAM3/src/System.cc:414`。
- ORB-SLAM3 Tracking 中根据 `mbRGB` 做 RGB/BGR 到灰度转换：`ORB_SLAM3/src/Tracking.cc:1454`、`ORB_SLAM3/src/Tracking.cc:1566`。
- ORB-SLAM3 ORBextractor 会先 `ComputePyramid`，再 `ComputeKeyPointsOctTree`：`ORB_SLAM3/src/ORBextractor.cc:1096`。
- ORB-SLAM3 每层 pyramid 由上一层 resize 而来：`ORB_SLAM3/src/ORBextractor.cc:1170`。
- ORB-SLAM3 对 keypoint 做 `cv::undistortPoints`：`ORB_SLAM3/src/Frame.cc:747`。
- OpenMAVIS 延续 ORB-SLAM3 管线，但多相机下为侧向相机创建更多 ORB features：`OpenMAVIS/src/Tracking.cc:617`。
- OpenMAVIS 多相机 Frame 同时对四个相机并行 ORB extraction，并合并 descriptors：`OpenMAVIS/src/Frame.cc:1542`、`OpenMAVIS/src/Frame.cc:1617`。

影响链路：

```text
rectify/remap
  -> 双目匹配沿同一行搜索
  -> disparity/depth 稳定
  -> 局部 BA 的重投影误差模型更干净

ORB pyramid/FAST 阈值
  -> keypoint 尺度覆盖和重复性改变
  -> map point 数量、回环词袋匹配、重定位稳定性改变
  -> 但直方图增强不一定带来描述子匹配提升，可能改变 BRIEF pattern 的二值关系
```

这组应重点调查 rectification/resize 是否改变相机矩阵、ORB 参数是否和分辨率匹配、多相机特征数是否均衡。

### 3.3 直接法/半直接法：DSO、DM-VIO、ROVIO、SVO、FAST-LIVO2

这类前端/后端通常是：

```text
光度标定/曝光处理 -> 几何 remap 或相机模型投影 -> pyramid/patch/gradient -> 光度残差 -> 优化或 EKF 更新
```

关键源码证据：

- DSO `PhotometricUndistorter::processFrame` 会应用相机响应 `G[]`、可选 vignette inverse、曝光处理：`dso/src/util/Undistort.cpp:213`。
- DSO `Undistort::undistort` 先做光度处理，再按 remapX/remapY 做双线性重采样：`dso/src/util/Undistort.cpp:386`。
- DSO `FrameHessian::makeImages` 构建金字塔、强度、dx/dy、梯度强度，并可用 gamma 权重修正选点梯度：`dso/src/FullSystem/HessianBlocks.cpp:128`。
- DSO `PixelSelector` 在 32x32 block 上做梯度直方图阈值：`dso/src/FullSystem/PixelSelector2.cpp:78`。
- DM-VIO 显式提供不同 photometric mode，包括有标定、无标定、perfect images、有标定但曝光不可靠：`dm-vio/src/util/MainSettings.cpp:130`。
- ROVIO 的 image pyramid 可以用自写 halfSample 或 OpenCV `pyrDown`：`rovio/include/rovio/ImagePyramid.hpp:81`。
- ROVIO 的 innovation 可以是直接像素强度误差，也可以是 reprojection error：`rovio/include/rovio/ImgUpdate.hpp:398`。
- ROVIO patch alignment 中有线性亮度修正 `patch_extracted - reg_a * patch - reg_b`：`rovio/include/rovio/MultilevelPatchAlignment.hpp:238`。
- SVO Frame 会把彩色图转灰度并创建半采样 pyramid：`rpg_svo_pro_open/svo_common/src/frame.cpp:65`、`rpg_svo_pro_open/svo_common/src/frame.cpp:372`。
- SVO sparse image alignment 是最小化 photometric error，并可估计 illumination gain/offset：`rpg_svo_pro_open/svo/include/svo/frame_handler_base.h:98`、`rpg_svo_pro_open/svo/include/svo/frame_handler_base.h:140`。
- FAST-LIVO2 接收图像后若尺寸不符会 resize，随后转灰度：`FAST-LIVO2/src/vio.cpp:1786`。
- FAST-LIVO2 patch residual 显式使用参考帧和当前帧的 `inv_expo_time`：`FAST-LIVO2/src/vio.cpp:747`、`FAST-LIVO2/src/vio.cpp:1619`。
- FAST-LIVO2 `exposure_estimate_en` 是配置参数，并会传给 VIO/IMU process：`FAST-LIVO2/src/LIVMapper.cpp:64`、`FAST-LIVO2/src/LIVMapper.cpp:144`。

影响链路：

```text
响应函数/暗角/曝光标定
  -> 原始灰度更接近 irradiance
  -> 光度 residual 更接近线性/仿射亮度假设
  -> Hessian 条件更好，直接法 pose/structure 优化更稳定

CLAHE/直方图均衡
  -> 改变局部强度映射，且映射随图像内容变化
  -> patch residual 不再只对应曝光/仿射亮度差
  -> 直接法可能跟踪“更锐利”，但后端残差物理含义变差
```

这组后续调查应围绕 residual 公式，而不是只统计角点数。

### 3.4 学习式/稠密：DROID-SLAM、MonoGS、NICE-SLAM

这类系统的预处理是模型输入契约：

```text
resize/crop/normalize -> 网络/可微渲染输入 -> dense BA 或 ray/color/depth loss
```

关键源码证据：

- DROID 在 dataset reader 中 resize 图像并同步缩放 intrinsics：`DROID-SLAM/droid_slam/data_readers/stream.py:51`、`DROID-SLAM/droid_slam/data_readers/stream.py:112`、`DROID-SLAM/droid_slam/data_readers/stream.py:169`。
- DROID crop 时会移动 `cx/cy`：`DROID-SLAM/droid_slam/data_readers/stream.py:61`。
- DROID 网络入口做 BGR->RGB、除以 255、ImageNet mean/std：`DROID-SLAM/droid_slam/droid_net.py:154`。
- DROID training augmentation 会做 ColorJitter、RandomGrayscale、scale/crop，并更新 intrinsics：`DROID-SLAM/droid_slam/data_readers/augmentation.py:12`、`DROID-SLAM/droid_slam/data_readers/augmentation.py:20`。
- MonoGS 把 PIL 图像 resize 后转 tensor 并 `/255`：`MonoGS/gaussian_splatting/utils/general_utils.py:24`。
- MonoGS 对每帧维护 exposure 参数，使用 `exp(exposure_a) * image + exposure_b`：`MonoGS/gaussian_splatting/scene/gaussian_model.py:107`。
- MonoGS 后端优化器显式优化 `exposure_a/b`：`MonoGS/utils/slam_backend.py:457`。
- NICE-SLAM 根据 crop_size/crop_edge 更新内参和图像尺寸：`nice-slam/src/NICE_SLAM.py:113`。
- NICE-SLAM 通过随机采样 UV 生成 rays，并参与 tracking/mapping 的 depth/color loss：`nice-slam/src/common.py:92`、`nice-slam/src/Tracker.py:91`、`nice-slam/src/Mapper.py:457`。

影响链路：

```text
输入归一化/颜色顺序
  -> 网络特征分布是否匹配训练分布
  -> DROID correlation/update 模块稳定性
  -> 非训练分布增强可能直接降低性能

crop/resize + intrinsics
  -> ray direction / projection scale 正确性
  -> dense BA 或 NeRF/GS 渲染 loss 是否几何一致
```

这组不建议优先尝试 CLAHE；应先验证输入尺寸、颜色顺序、归一化、内参更新是否和训练/配置一致。

### 3.5 LIVO/RGB Map：R3LIVE、FAST-LIVO2

这类系统把图像用于 RGB map、patch tracking 或 VIO update，常见特点是图像时间、LiDAR/IMU 同步、曝光都进入系统设计。

关键源码证据：

- R3LIVE 接收 BGR8 图像，首次图像时根据 downsample ratio 设置相机参数并创建 undistort map：`r3live/r3live/src/r3live_vio.cpp:357`、`r3live/r3live/src/r3live_vio.cpp:388`、`r3live/r3live/src/r3live_vio.cpp:394`。
- R3LIVE 处理链路为 resize -> remap -> init cubic interpolation -> image_equalize：`r3live/r3live/src/r3live_vio.cpp:402`、`r3live/r3live/src/r3live_vio.cpp:415`、`r3live/r3live/src/r3live_vio.cpp:418`。
- R3LIVE `Image_frame::init_cubic_interpolation` 会生成灰度图：`r3live/r3live/src/rgb_map/image_frame.cpp:110`。
- R3LIVE `image_equalize` 使用 CLAHE，且对灰度图和 YCrCb 的亮度通道做处理：`r3live/r3live/src/rgb_map/image_frame.cpp:272`、`r3live/r3live/src/rgb_map/image_frame.cpp:309`。
- FAST-LIVO2 通过 `exposure_time_init` 切分 LiDAR/视觉时刻：`FAST-LIVO2/src/LIVMapper.cpp:953`、`FAST-LIVO2/src/LIVMapper.cpp:1047`。
- FAST-LIVO2 光度 residual 里当前图像强度和参考 patch 都乘以 inverse exposure：`FAST-LIVO2/src/vio.cpp:747`、`FAST-LIVO2/src/vio.cpp:1621`。

影响链路：

```text
图像时间 + exposure_time_init
  -> LiDAR/IMU/Camera 数据切分点
  -> 投影到图像的点云位置
  -> patch residual 和 RGB map 更新的一致性

R3LIVE CLAHE
  -> RGB map/KLT tracking 更抗光照
  -> 如果后续使用原始颜色做建图或渲染，需要区分 raw image 与 equalized image
```

---

## 4. 重点对比：预处理如何影响前端/后端精度

### 4.1 几何一致性类预处理

包括 resize、crop、downsample、rectify、undistort。

主要风险：

- 内参没有同步缩放：`fx/fy/cx/cy` 错，所有 bearing 和 ray direction 错。
- 稀疏点在 distorted image 跟踪，却用 pixel RANSAC 阈值做 normalized plane 判断。
- stereo rectification 后的 keypoint 和原图 keypoint 混用。

建议评价指标：

- tracking count / new feature count
- RANSAC inlier ratio
- stereo match success ratio
- triangulation depth distribution
- reprojection residual 分布
- 对 ATE/RPE 的影响

### 4.2 光度一致性类预处理

包括 response、vignette、exposure、affine brightness、CLAHE、histogram equalization、ImageNet normalization。

主要风险：

- 把 KLT 用的 CLAHE 套到 DSO/ROVIO/FAST-LIVO2 这类 patch residual 上，会改变 residual 的物理意义。
- 自动曝光、滚动曝光、暗角如果不建模，会使直接法残差出现空间/时间偏置。
- 神经网络方法如果改变颜色顺序或归一化，会造成训练-推理分布不一致。

建议评价指标：

- direct residual mean/median/std
- photometric outlier ratio
- patch NCC / ZNCC
- exposure 参数收敛曲线
- 跟踪失败帧和低光场景片段

### 4.3 特征/像素选择类预处理

包括 mask、grid、min distance、ANMS、gradient histogram、ray sampling。

主要风险：

- 只提高特征数量，不提高空间分布，后端仍然退化。
- 过强 min distance 导致纹理集中区域被过度稀释。
- gradient selector 在曝光/响应不稳定时偏向噪声边缘。
- ray sampling 忽略无效 depth/dynamic 区域，会污染 tracking/mapping loss。

建议评价指标：

- feature spatial entropy / grid occupancy
- track lifetime histogram
- marginalization 后剩余约束数量
- information matrix condition number 或 Hessian condition proxy
- dynamic mask / invalid depth mask 后的 loss 变化

---

## 5. 推荐后续实验矩阵

### 实验 A：KLT 预处理增强

对象：OpenVINS、LVI-SAM、VINS-Fusion。

变量：

- `NONE`
- `HISTOGRAM`
- `CLAHE`
- `downsample on/off`
- `min_px_dist` 或 `MIN_DIST`

观察：

- tracked features
- new features
- RANSAC inlier ratio
- average track length
- front-end runtime
- ATE/RPE

预期：

- CLAHE 在低光/强曝光变化下可能提升 KLT 成功率。
- CLAHE 在纹理噪声多的场景可能提升短寿命点和外点。
- downsample 提升速度但降低小纹理特征和深度精度。

### 实验 B：直接法光度模型

对象：DSO、DM-VIO、ROVIO、FAST-LIVO2。

变量：

- photometric calibration on/off
- exposure estimation on/off
- affine brightness on/off
- 不建议直接加入 CLAHE，除非作为“破坏光度假设”的对照组

观察：

- photometric residual 分布
- patch NCC
- tracking failure
- exposure/affine 参数曲线
- ATE/RPE

预期：

- 有响应/暗角/曝光标定时，直接法 residual 更稳定。
- 未建模的自动曝光会表现为残差随时间偏置。
- CLAHE 可能降低视觉观感噪声，但不一定降低可解释的光度 residual。

### 实验 C：学习式输入契约

对象：DROID-SLAM、MonoGS、NICE-SLAM。

变量：

- resize 分辨率
- crop/crop_edge
- intrinsics 是否同步更新
- RGB/BGR 顺序
- normalization 是否匹配源码默认

观察：

- DROID tracking loss / update convergence
- MonoGS exposure 参数和渲染 loss
- NICE tracking/mapping depth/color loss
- ATE/RPE、重建质量

预期：

- 内参不同步会直接破坏几何。
- 颜色顺序/归一化错误通常比传统增强是否开启更致命。

---

## 6. 可复用调查清单

后续看一个新项目时，可以按以下顺序定位：

1. **输入格式**：RGB/BGR/GRAY、8bit/float、是否深度图。
2. **几何变换**：resize/crop/downsample/remap/rectify 是否存在。
3. **内参同步**：`fx/fy/cx/cy/W/H` 是否随几何变换更新。
4. **光度处理**：CLAHE/equalize/response/vignette/exposure/normalize。
5. **尺度结构**：OpenCV KLT pyramid、ORB pyramid、direct image pyramid、网络 stride。
6. **坐标系统**：distorted pixel、rectified pixel、normalized plane、bearing vector。
7. **观测残差**：reprojection、bearing、photometric patch、dense BA、ray depth/color。
8. **阈值单位**：pixel、normalized coordinate、pyramid level pixel、photometric intensity。

---

## 7. 当前优先级建议

如果目标是为后续做“预处理算法对比”，优先级建议如下：

1. **OpenVINS/LVI-SAM/VINS-Fusion 的 KLT 增强对比**：最容易做 ablation，也最能解释 CLAHE/均衡化对特征数和外点率的影响。
2. **DSO/DM-VIO/FAST-LIVO2 的光度残差对比**：重点验证“光度标定/曝光补偿”和“普通增强”不是同一类东西。
3. **DROID/NICE/MonoGS 的输入契约检查**：重点验证 resize/crop/intrinsics/normalization，不优先引入传统增强。
4. **R3LIVE 单独看**：它显式用了 resize、remap、CLAHE，且还涉及 RGB map，适合做“增强对建图颜色和跟踪的双重影响”分析。
