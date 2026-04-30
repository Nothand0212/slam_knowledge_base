# ESVO (Event-based Stereo Visual Odometry) 深度源码分析

> 仓库: https://github.com/HKUST-Aerial-Robotics/ESVO | 500+ stars
> 分析时间: 2026-04-29

---

## 1. 事件相机原理

### 1.1 事件相机 vs 帧相机

传统帧相机以固定帧率曝光整个传感器阵列，输出完整图像。事件相机（如 DVS, DAVIS）则完全不同：每个像素独立、异步地检测对数亮度变化，当变化超过阈值时立即输出一个"事件"。

事件的数据格式为 `(x, y, t, polarity)`：
- **x, y**: 像素坐标
- **t**: 微秒级时间戳（异步，非均匀采样）
- **polarity**: 亮度变化方向（ON=增亮, OFF=变暗）

ESVO 代码中，事件通过 ROS 消息 `dvs_msgs/Event` 传递：

```cpp
// esvo_core/include/esvo_core/tools/utils.h:50
using EventQueue = std::deque<dvs_msgs::Event>;
```

该类型定义了事件缓冲区，利用 `std::deque` 维持按时间排序的事件队列。关键查找函数：

```cpp
// esvo_core/include/esvo_core/tools/utils.h:51-56
inline static EventQueue::iterator EventBuffer_lower_bound(
  EventQueue& eb, ros::Time& t) {
  return std::lower_bound(eb.begin(), eb.end(), t,
    [](const dvs_msgs::Event & e, const ros::Time & t) {return e.ts.toSec() < t.toSec();});
}
```

### 1.2 事件相机的核心优势

- **极高时间分辨率** (μs级) — 无运动模糊
- **高动态范围** (120dB+) — 强光暗光同时成像
- **低带宽** — 仅传输变化部分（边缘信息）
- **低延迟** — 像素级异步输出

### 1.3 事件相机的挑战

- 仅输出边缘信息，缺乏纹理区域
- 噪声多（热噪声、漏电流）
- 数据稀疏，传统帧图像算法不直接适用
- 需要新的表示方法来从中提取有用信息

---

## 2. 事件处理: Time Surface 生成与事件累积

### 2.1 Time Surface 核心概念

Time Surface 是 ESVO 的核心数据结构，它将异步事件流转换为类图像的 2D 表示。每个像素存储最近一次事件的时间戳，形成一张"时间表面"。

Time Surface 的生成在 `esvo_time_surface` 包中实现。Time Surface 结构定义：

```cpp
// esvo_core/include/esvo_core/container/TimeSurfaceObservation.h:27-157
struct TimeSurfaceObservation {
  Eigen::MatrixXd TS_left_, TS_right_;   // 左右目的 Time Surface
  Transformation tr_;                     // 对应的相机位姿
  size_t id_;                            // Time Surface ID

  // 构造函数：从 cv::Mat 转换为 Eigen::MatrixXd
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    Transformation &tr,
    size_t id,
    bool bCalcTsGradient = false)
    : tr_(tr), id_(id) {
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);
    if (bCalcTsGradient) {
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);  // 计算 x 方向梯度
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);  // 计算 y 方向梯度
      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
    }
  }
};
```

### 2.2 Time Surface 的平滑与增强

ESVO 支持对 Time Surface 进行高斯平滑以抑制噪声：

```cpp
// esvo_core/include/esvo_core/container/TimeSurfaceObservation.h:107-116
inline void GaussianBlurTS(size_t kernelSize) {
  cv::Mat mat_left_, mat_right_;
  cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                   cv::Size(kernelSize, kernelSize), 0.0);
  cv::GaussianBlur(cvImagePtr_right_->image, mat_right_,
                   cv::Size(kernelSize, kernelSize), 0.0);
  cv::cv2eigen(mat_left_, TS_left_);
  cv::cv2eigen(mat_right_, TS_right_);
}
```

此外，还支持生成 Negative Time Surface（翻转后的时间表面），用于处理极性信息：

```cpp
// esvo_core/include/esvo_core/container/TimeSurfaceObservation.h:118-134
inline void getTimeSurfaceNegative(size_t kernelSize) {
  Eigen::MatrixXd ceilMat(TS_left_.rows(), TS_left_.cols());
  ceilMat.setConstant(255.0);
  if (kernelSize > 0) {
    cv::Mat mat_left_;
    cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                     cv::Size(kernelSize, kernelSize), 0.0);
    cv::cv2eigen(mat_left_, TS_blurred_left_);
    TS_negative_left_ = ceilMat - TS_blurred_left_;
  } else {
    TS_negative_left_ = ceilMat - TS_left_;
  }
}
```

### 2.3 Time Surface 历史管理

系统维护 Time Surface 的滑动窗口历史：

```cpp
// esvo_core/include/esvo_core/container/TimeSurfaceObservation.h:167-168
using TimeSurfaceHistory = std::map<ros::Time, TimeSurfaceObservation, ROSTimeCmp>;
using StampedTimeSurfaceObs = std::pair<ros::Time, TimeSurfaceObservation>;
```

查找时使用二分搜索以保证 O(log n) 效率：

```cpp
// esvo_core/include/esvo_core/container/TimeSurfaceObservation.h:170-176
inline static TimeSurfaceHistory::iterator TSHistory_lower_bound(
  TimeSurfaceHistory &ts_history, ros::Time &t) {
  return std::lower_bound(ts_history.begin(), ts_history.end(), t,
    [](const std::pair<ros::Time, TimeSurfaceObservation> &tso, const ros::Time &t) {
      return tso.first.toSec() < t.toSec();
    });
}
```

---

## 3. 立体匹配 (Event-based Stereo)

### 3.1 立体匹配架构

ESVO 的立体匹配是事件驱动型 Block Matching (BM)，核心类为 `EventBM`：

```cpp
// esvo_core/include/esvo_core/core/EventBM.h:16-91
class EventBM {
  CameraSystem::Ptr camSysPtr_;
  StampedTimeSurfaceObs* pStampedTsObs_;   // 当前 Time Surface
  StampTransformationMap * pSt_map_;       // 位姿时间戳映射
  std::vector<dvs_msgs::Event*> vEventsPtr_; // 待匹配的事件列表

  size_t patch_size_X_;     // 模板 patch 宽度 (默认 25)
  size_t patch_size_Y_;     // 模板 patch 高度 (默认 25)
  size_t min_disparity_;    // 最小视差 (默认 1)
  size_t max_disparity_;    // 最大视差 (默认 40)
  size_t step_;             // 粗搜索步长
  double ZNCC_Threshold_;   // ZNCC 匹配阈值 (默认 0.1)
};
```

### 3.2 单事件匹配流程

每个事件的匹配过程在 `match_an_event()` 中：

```cpp
// esvo_core/src/core/EventBM.cpp:80-168
bool EventBM::match_an_event(
  dvs_msgs::Event* pEvent,
  std::pair<size_t, size_t>& pDisparityBound,
  EventMatchPair& emPair)
{
  // 1. 畸变校正 + 坐标转整
  Eigen::Vector2d x_rect = camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(
    pEvent->x, pEvent->y);

  // 2. 边界检查
  if(x_rect(0) < 0 || x_rect(0) > camSysPtr_->cam_left_ptr_->width_ - 1 || ...)
    return false;

  // 3. Mask 区域检查
  if(camSysPtr_->cam_left_ptr_->UndistortRectify_mask_(x_rect(1), x_rect(0)) <= 125)
    return false;

  Eigen::Vector2i x1(std::floor(x_rect(0)), std::floor(x_rect(1)));

  // 4. 提取左图 template patch
  Eigen::MatrixXd patch_src = pStampedTsObs_->second.TS_left_.block(
    x1_left_top(1), x1_left_top(0), patch_size_Y_, patch_size_X_);

  // 5. 信噪比检查：有效像素必须 > 5%
  if((patch_src.array() < 1).count() > 0.95 * patch_src.size())
    return false;

  // 6. 粗搜索：大步长沿极线搜索
  if(!epipolarSearching(min_cost, bestMatch, bestDisp, patch_dst,
    lowDisparity, upDisparity, step_, x1, patch_src, bUpDownConfiguration_))
    return false;

  // 7. 精搜索：在粗搜最优位置 ±(step-1) 内逐像素搜索
  size_t fine_searching_start_pos = bestDisp-(step_-1) >= 0 ? bestDisp-(step_-1) : 0;
  if(!epipolarSearching(min_cost, bestMatch, bestDisp, patch_dst,
    fine_searching_start_pos, bestDisp+(step_-1), 1, x1, patch_src, ...))
    return false;

  // 8. 输出匹配结果
  emPair.x_left_ = x_rect;
  emPair.x_right_ = bestMatch;
  emPair.t_ = pEvent->ts;
  emPair.invDepth_ = 1.0 / depth;  // 三角化得到逆深度
  emPair.cost_ = min_cost;
  emPair.disp_ = disparity;
  return true;
}
```

### 3.3 ZNCC (Zero-mean Normalized Cross-Correlation) 代价函数

```cpp
// esvo_core/src/core/EventBM.cpp:317-333
double EventBM::zncc_cost(Eigen::MatrixXd &patch_left, Eigen::MatrixXd &patch_right, bool normalized) {
  if(!normalized) {
    Eigen::MatrixXd patch_left_normalized, patch_right_normalized;
    tools::normalizePatch(patch_left, patch_left_normalized);
    tools::normalizePatch(patch_right, patch_right_normalized);
    cost = 0.5 * (1 - (patch_left_normalized.array() * patch_right_normalized.array()).sum()
      / (patch_left.rows() * patch_left.cols()));
  }
  return cost;
}
```

其中 `normalizePatch` 执行零均值化 + 单位方差归一化：

```cpp
// esvo_core/include/esvo_core/tools/utils.h:84-92
static inline void normalizePatch(Eigen::MatrixXd& patch_src, Eigen::MatrixXd& patch_dst) {
  double mean = 0, sigma = 0;
  meanStdDev(patch_src, mean, sigma);
  patch_dst = (patch_src.array() - mean) / sigma;
}
```

### 3.4 多线程并行匹配

系统支持超线程并行匹配，每个线程处理交错的 event 索引：

```cpp
// esvo_core/src/core/EventBM.cpp:299-315
void EventBM::match(EventBM::Job& job) {
  size_t i_thread = job.i_thread_;
  auto ev_it = job.pvEventPtr_->begin();
  std::advance(ev_it, i_thread);
  for(size_t i = i_thread; i < totalNumEvents; i+=NUM_THREAD_, std::advance(ev_it, NUM_THREAD_)) {
    EventMatchPair emp;
    if(match_an_event(*ev_it, pDisparityBound, emp))
      job.pvEventMatchPair_->push_back(emp);
  }
}
```

默认使用 4 线程（`NUM_THREAD_MAPPING`）：

```cpp
// esvo_core/include/esvo_core/tools/utils.h:35-36
#define NUM_THREAD_TRACKING 1
#define NUM_THREAD_MAPPING 4
```

---

## 4. 位姿估计

### 4.1 Tracking 管线

ESVO 的 Tracking 线程独立运行，主要流程：

```cpp
// esvo_core/src/esvo_Tracking.cpp:79-200
void esvo_Tracking::TrackingLoop() {
  ros::Rate r(tracking_rate_hz_);  // 默认 100Hz
  while(ros::ok()) {
    // 1. 等待 Mapping 线程发布参考地图 (refPCMap_)
    if(refPCMap_.size() < 1 || TS_history_.size() < 1) { r.sleep(); continue; }

    // 2. 数据搬运：加载新的参考/当前帧
    refDataTransferring();  // 从 refPCMap_ 提取最新的 RefFrame
    curDataTransferring();  // 从 TS_history_ 提取当前 Time Surface

    // 3. 创建并求解 Registration Problem (逆向组合法)
    if(rpSolver_.resetRegProblem(&ref_, &cur_)) {
      if(rpType_ == REG_NUMERICAL)
        rpSolver_.solve_numerical();   // 数值优化
      if(rpType_ == REG_ANALYTICAL)
        rpSolver_.solve_analytical();  // 解析优化 (ESVO 论文核心)
      T_world_cur_ = cur_.tr_.getTransformationMatrix();
      publishPose(cur_.t_, cur_.tr_);
    }
    r.sleep();
  }
}
```

### 4.2 Registration Problem (逆向组合法逆向组合)

位姿估计问题定义为：将当前帧 (cur) 的 Time Surface 对齐到参考帧 (ref) 的反投影地图。

```cpp
// esvo_core/include/esvo_core/core/RegProblemLM.h:77-150
struct RegProblemLM : public optimization::OptimizationFunctor<double> {
  // 核心参数
  CameraSystem::Ptr camSysPtr_;
  RegProblemConfig::Ptr rpConfigPtr_;
  ResidualItems ResItems_;          // 残差项集合

  // 位姿状态
  Eigen::Matrix<double,4,4> T_world_left_;  // 当前位姿
  Eigen::Matrix<double,4,4> T_world_ref_;   // 参考位姿
  Eigen::Matrix3d R_;  // 相对旋转 R_ref_cur_
  Eigen::Vector3d t_;  // 相对平移 t_ref_cur
};
```

配置参数：

```cpp
// esvo_core/include/esvo_core/core/RegProblemLM.h:19-57
struct RegProblemConfig {
  size_t patchSize_X_;              // Patch 宽度 (默认 25)
  size_t patchSize_Y_;              // Patch 高度 (默认 25)
  size_t kernelSize_;               // 高斯核大小 (默认 15)
  std::string LSnorm_;              // 损失函数 ("l2" 或 Huber)
  double huber_threshold_;          // Huber 阈值
  double invDepth_min_range_;       // 逆深度范围下限
  double invDepth_max_range_;       // 逆深度范围上限
  size_t MIN_NUM_EVENTS_;           // 最小事件数 (默认 1000)
  size_t MAX_REGISTRATION_POINTS_;  // 最大注册点数 (默认 500)
  size_t BATCH_SIZE_;               // 批量大小 (默认 200)
  size_t MAX_ITERATION_;            // 最大迭代次数 (默认 10)
};
```

### 4.3 位姿优化

使用 6 自由度李代数参数化位姿，支持两种解法：

1. **数值法** (Numerical): 使用 Eigen 的 LM 非线性优化
2. **解析法** (Analytical): ESVO 论文的原始贡献 —— 基于逆向组合法（Inverse Compositional）推导解析雅可比，大幅加速

核心残差：3D 点投影到 Time Surface，比较模板 patch 的 photometric 差异。

### 4.4 位姿外推 (`pose_extrapolator`)

Tracking 模块同时维护 `T_world_cur_` 作为帧间位姿先验：

```cpp
// esvo_core/src/esvo_Tracking.cpp:256-260
if(ESVO_System_Status_ == "WORKING" || (...) ) {
  cur_.tr_ = Transformation(T_world_cur_);
}
```

---

## 5. 深度估计 (Mapping)

### 5.1 Mapping 管线

```cpp
// esvo_core/include/esvo_core/esvo_Mapping.h:43-223
class esvo_Mapping {
  EventBM ebm_;               // 块匹配引擎
  DepthProblemSolver dpSolver_;  // 深度优化
  DepthFusion dFusor_;        // 深度融合 (跨帧)
  DepthRegularization dRegularizor_;  // 深度正则化
  Visualization visualizor_;

  // 关键参数
  size_t PROCESS_EVENT_NUM_;   // 每帧处理的事件数
  size_t TS_HISTORY_LENGTH_;   // Time Surface 历史长度
  double BM_ZNCC_Threshold_;   // 块匹配 ZNCC 阈值
};
```

### 5.2 DepthProblem: 非线性逆深度优化

```cpp
// esvo_core/include/esvo_core/core/DepthProblem.h:53-90
struct DepthProblem : public optimization::OptimizationFunctor<double> {
  // 对每个 EventMatchPair 的逆深度进行优化
  int operator()( const Eigen::VectorXd &x, Eigen::VectorXd & fvec ) const;

  bool warping(const Eigen::Vector2d &x, double d,
    const Eigen::Matrix<double, 3, 4> &T_left_virtual,
    Eigen::Vector2d &x1_s, Eigen::Vector2d &x2_s) const;
};
```

优化配置：

```cpp
// esvo_core/include/esvo_core/core/DepthProblem.h:15-51
struct DepthProblemConfig {
  size_t patchSize_X_, patchSize_Y_;
  std::string LSnorm_;
  double td_nu_;         // Student-t 分布自由度
  double td_scale_;      // Student-t 分布尺度参数
  double td_stdvar_;     // 推断标准差: sqrt(td_nu / (td_nu - 2) * td_scale^2)
  size_t MAX_ITERATION_;
  size_t RegularizationRadius_;
  size_t RegularizationMinNeighbours_;
};
```

该配置将深度残差建模为 Student-t 分布以增强对离群值的鲁棒性。

### 5.3 DepthFusion: 跨帧深度融合

```cpp
// esvo_core/include/esvo_core/core/DepthFusion.h:16-70
class DepthFusion {
  // 传播：将前一帧的深度点投影到当前帧
  bool propagate_one_point(DepthPoint &dp_prior, DepthPoint &dp_prop,
    Eigen::Matrix<double, 4, 4> &T_prop_prior);

  // 融合：在新帧中 merge 深度观测
  int fusion(DepthPoint &dp_prop, DepthMap::Ptr &dm, int fusion_radius = 0);

  // 卡方检验：判断两个深度观测是否兼容
  bool chiSquareTest(double invD1, double invD2, double var1, double var2);

  // Student-t 兼容性检验
  bool studentTCompatibleTest(double invD1, double invD2, double var1, double var2);
};
```

融合半径为 `fusion_radius_` 像素，只有统计兼容的深度才会被融合。

### 5.4 事件去噪与边缘提取

在处理事件之前需要去噪和提取有用事件：

```cpp
// esvo_core/include/esvo_core/esvo_Mapping.h:86-103
void createEdgeMask(...);      // 仅在边缘区域保留事件
void createDenoisingMask(...);  // 空间密度去噪
void extractDenoisedEvents(...); // 最多保留 maxNum 个事件
```

---

## 6. 系统架构总结

ESVO 采用三线程并行架构：

1. **Time Surface 线程** (`esvo_time_surface`): 实时生成 Time Surface
2. **Mapping 线程**: 立体匹配 → 深度优化 → 深度融合 → 输出 3D 点云
3. **Tracking 线程**: 将当前 Time Surface 对齐到反投影地图 → 输出位姿

线程间通过 ROS Topic 通信：
- `events_left/right` → 事件流
- `time_surface_left/right` → Time Surface 图像
- `pointcloud` → Mapping 输出的 3D 点云
- `/esvo_tracking/pose_pub` → Tracking 输出的位姿

系统状态机：

```cpp
// esvo_core/src/esvo_Tracking.cpp:38-39
ESVO_System_Status_("INITIALIZATION"),
ets_(IDLE)
```

状态流转: `INITIALIZATION → WORKING → TERMINATE`，支持通过 dynamic_reconfigure 在线重置。

---

## 7. 优缺点与对 SLAM 算法的意义

### 优势
1. **极高时间分辨率**: 事件相机 μs 级输出，天然抗运动模糊，适合高速场景
2. **异步处理**: 不依赖固定帧率，可按需处理任意时长的累积事件
3. **低延迟**: 纯事件驱动的 pipeline，延迟远低于传统帧图像
4. **高动态范围**: 120dB+ 动态范围，室内外自适应
5. **逆向组合法**: 解析雅可比大幅降低 Tracking 计算量，实现 100Hz+ 实时跟踪

### 劣势
1. **场景依赖**: 需要场景有足够的纹理边缘，白色墙壁等无纹理区域失效
2. **噪声敏感**: 事件相机噪声多（热噪声等），需要复杂的去噪和滤波策略
3. **深度稀疏**: 仅能在边缘处估计深度，密度远低于 RGB-D
4. **无回环检测**: 纯 VO 方案，缺乏全局一致性，累积漂移不可避免
5. **硬件不成熟**: 事件相机硬件仍在发展期，分辨率和可靠性不如 RGB 相机

### 对 SLAM 算法的意义
- **新型传感器融合**: 事件相机作为第四种主流传感器（RGB/Depth/LiDAR/Event），为高速、高动态场景 SLAM 开辟新方向
- **异步处理范式**: 突破了传统帧同步处理框架，对 event-driven SLAM 理论研究有重要推动作用
- **低延迟定位**: 无人机高速飞行控制等场景中对延迟要求极高，ESVO 展示了事件相机在此方面的潜力
- **与帧图像互补**: 事件相机 + 传统相机的混合系统可能是未来视觉 SLAM 的重要方向