# DM-VIO (Delayed Marginalization Visual-Inertial Odometry) 源码深度分析

> 分析基于 DM-VIO 开源代码 (Lukas von Stumberg et al., IROS 2022)
> 文件路径: `/home/lin/Projects/lin_ws/slam_ws/dm-vio/`
> 核心代码量: ~40 个 .cpp/.h 文件, 总计约 8000 行

---

## 1. 直接法 vs 间接法的根本区别 (DM-VIO 的视角)

### 1.1 DM-VIO 对 DSO 的继承

DM-VIO **完全继承了 DSO 的光度 BA 框架**, 包括:
- 光度误差模型 (`src/dso/FullSystem/Residuals.cpp:78-274`)
- 逆深度参数化 (`src/dso/FullSystem/HessianBlocks.h:402-459`)
- 滑动窗口前端 (`src/dso/FullSystem/CoarseTracker.cpp:536-718`)
- 像素选择与未成熟点跟踪 (`src/dso/FullSystem/ImmaturePoint.cpp:75-414`)

### 1.2 为什么在直接法中加入 IMU 比间接法更有挑战?

间接法 (VINS-Fusion) 的重投影误差是像素坐标差, IMU 预积分误差是位姿差, 单位相对一致。
但在直接法中, 光度残差的单位是灰度值 (0-255), 而 IMU 残差是位姿的 SE(3) 流形。
两个残差的物理量纲完全不同, 联合优化需要精心设计权重。

DM-VIO 解决这个问题的创新在于它的三层架构:

```
Coarse IMU Logic (前端跟踪用 IMU, 轻量级)
       ↕
BA IMU Logic (后端 BA 中集成 IMU 因子, 精确但重)
       ↕
IMU Initializer (系统启动和重定位时的 IMU 初始化)
```

### 1.3 DM-VIO 的坐标系桥接

`PoseTransformationFactor` (`src/GTSAMIntegration/PoseTransformationFactor.h:43-104`) 是实现直接法+IMU 融合的关键, 它将 IMU 因子 (工作在度量坐标系) 转换为 DSO 因子图 (工作在光度标定坐标系):

```cpp
// BAIMULogic.cpp:289-292
auto transformedFactor = boost::make_shared<PoseTransformationFactor>(
    imuFactor,
    *transformDSOToIMU,  // 包含 scale, gravity方向, T_cam_imu
    PoseTransformationFactor::JACOBIAN_FACTOR);
```

---

## 2. 数据接收与预处理

### 2.1 IMU 数据流

`IMUIntegration::addIMUData()` (`src/IMU/IMUIntegration.cpp:150-180`):
- 步骤1: 送入 IMU 初始化器 (如尚未初始化)
- 步骤2: 送入 coarse IMU logic (前端跟踪)
- 步骤3: 送入 BA 预积分缓冲区

### 2.2 IMU 测量值结构

`IMUMeasurement` (`src/IMU/IMUTypes.h:31-46`):
```cpp
class IMUMeasurement {
    Eigen::Vector3d accData;     // 加速度计 (m/s^2)
    Eigen::Vector3d gyrData;     // 陀螺仪 (rad/s)
    double integrationTime;      // 时间间隔
};
```

### 2.3 预积分参数配置

在 `IMUIntegration` 构造函数中 (`src/IMU/IMUIntegration.cpp:46-56`):

```cpp
preintegrationParams.reset(new gtsam::PreintegrationParams(imuCalibration.gravity));
preintegrationParams->setIntegrationCovariance(integrationVar * I_3x3);
preintegrationParams->setAccelerometerCovariance(accelVar * I_3x3);
preintegrationParams->setGyroscopeCovariance(gyroVar * I_3x3);
```

### 2.4 双线预积分管理

DM-VIO 维护两个预积分缓冲区 (`IMUIntegration.cpp:104-105`):
```cpp
preintegratedBA.reset(new gtsam::PreintegratedImuMeasurements(params));
preintegratedBACurr.reset(new gtsam::PreintegratedImuMeasurements(params));
```

`prepareKeyframe()` (`IMUIntegration.cpp:228-276`) 中通过交换缓冲区实现跟踪和BA并行:
```cpp
if(!previouslyPrepared) {
    std::swap(preintegratedBA, preintegratedBACurr);
}
preintegratedBACurr->resetIntegrationAndSetBias(latestBias);
```

---

## 3. 延迟边缘化 (Delayed Marginalization)

### 3.1 边缘化的本质问题

在视觉惯性 SLAM 中, 滑动窗口需要边缘化旧状态。但边缘化引入了**非线性线性化点固定**的问题:

**VINS-Fusion 的即时边缘化问题:**
- 帧 F_k 到达后立即边缘化最旧帧
- 如果此时 scale/gravity/bias 尚不确定, 边缘化会在错误线性化点处固定信息
- 错误的先验信息会在后续优化中"拖拽"系统状态
- 这是 VINS-Fusion 中 IMU 初始化慢、容易发散的根本原因

### 3.2 延迟边缘化的核心思想

`DelayedGraph` 类 (`src/GTSAMIntegration/DelayedMarginalization.h:35-92`):

```cpp
class DelayedGraph {
    int delayN;                        // 延迟帧数
    int maxGroupInGraph;              // 因子过滤阈值
    gtsam::NonlinearFactorGraph::shared_ptr graph;
    std::deque<gtsam::FastVector<gtsam::Key>> marginalizationOrder;  // 排队
    gtsam::Values delayedValues, delayedCurrValues;
    std::shared_ptr<FEJValues> fejValues;  // First-Estimate Jacobians
    bool marginalizationPaused = false;
};
```

**核心算法** (`DelayedGraph::readvanceUntilDelay()`, `DelayedMarginalization.cpp:212-253`):

```cpp
void DelayedGraph::readvanceUntilDelay() {
    if(marginalizationPaused) return;
    
    while(marginalizationOrder.size() > delayN) {
        auto&& keysToMarg = marginalizationOrder.front();
        
        // 步骤1: 插入 FEJ 值 (保证线性化点正确!)
        fejValues->insertConnectedKeys(keysToMarg, delayedValues);
        
        // 步骤2: 在插入 FEJ 值后执行 Schur complement
        auto connectedKeyCallback = [this](const FastSet<Key>& connectedKeys) {
            fejValues->insertConnectedKeys(connectedKeys, delayedValues);
        };
        
        // 步骤3: 执行边缘化
        graph = marginalizeOut(*graph, fejValues->fejValues, 
            keysToMarg, connectedKeyCallback);
        
        // 步骤4: 清理
        for(auto&& key : keysToMarg) {
            delayedCurrValues.erase(key);
            delayedValues.erase(key);
        }
        fejValues->keysRemoved(keysToMarg);
        marginalizationOrder.pop_front();
    }
}
```

### 3.3 延迟 vs 即时: 完整流程对比

```
VINS-Fusion (即时边缘化):
  帧F_k到达 → BA优化 → 立即边缘化最旧帧 → 写入先验
  问题: scale未收敛时, 错误信息被锁定

DM-VIO (延迟边缘化):
  帧F_k到达 → BA优化 → 加入边缘化队列 (暂不执行)
                       ↓
  帧F_{k+1}到达 → BA优化(所有帧+IMU) → scale/gravity更稳定
                       ↓
  队列长度 > delayN → 才执行F_k的边缘化
                       ↓
  优势: 边缘化前, scale/gravity已经通过更多数据收敛!
```

### 3.4 多延迟图架构

`DelayedMarginalizationGraphs` (`src/GTSAMIntegration/DelayedMarginalization.h:114-163`):

```cpp
class DelayedMarginalizationGraphs : public BAGraphs {
    std::vector<std::shared_ptr<DelayedGraph>> delayedGraphs;
    // main graph: delayN=0 (所有IMU和视觉因子)
    // IMU初始化图: 更大delay, 仅视觉因子
    
    void addFactor(NonlinearFactor::shared_ptr factor, int group) override {
        for(auto&& graph : delayedGraphs) {
            graph->addFactor(factor, group);  // 所有图同步
        }
    }
    
    void marginalizeFrame(...) override {
        mainGraph->marginalize(...);
        for(auto&& graph : delayedGraphs) {
            graph->marginalize(...);
        }
    }
};
```

**为什么需要多个延迟图?**
- **初始化前**: IMU 初始化图使用大延迟, 积累足够数据后才边缘化
- **初始化后**: `replaceMainGraph()` 替换主图, 新图包含 IMU 因子+先验

### 3.5 Schur Complement 实现

`marginalizeOut()` (`src/GTSAMIntegration/Marginalization.cpp:31-90`):

```cpp
NonlinearFactorGraph::shared_ptr marginalizeOut(
    const NonlinearFactorGraph& graph, const Values& values,
    const FastVector<Key>& keysToMarginalize, ...) {
    
    // 1. 分离边缘化和保留的因子
    extractKeysToMarginalize(graph, newGraph, margOutGraph, 
        keysSet, connectedKeys);
    
    // 2. 线性化
    GaussianFactorGraph::shared_ptr linearGraph = margOutGraph.linearize(values);
    
    // 3. Ordering: 先边缘化变量, 后保留变量
    Ordering ordering;
    for(Key k : keysSet) ordering.push_back(k);       // m 个变量
    for(Key k : connectedKeys) ordering.push_back(k);  // a 个变量
    
    // 4. 计算 augmented Hessian
    Matrix hessian = linearGraph->augmentedHessian(ordering);
    
    // 5. Schur complement
    Matrix HAfterSchur = computeSchurComplement(hessian, mSize, aSize);
    
    // 6. 封装成 HessianFactor → LinearContainerFactor
    LinearContainerFactor::shared_ptr lcf(
        new LinearContainerFactor(HessianFactor(connOrdering, sm), values));
    newGraph->add(lcf);
}
```

**computeSchurComplement** (`Marginalization.cpp:140-180`):

```cpp
Matrix computeSchurComplement(const Matrix& augmentedH, int mSize, int aSize) {
    auto [H, b] = pairFromAugmentedHessian(augmentedH);
    
    // Preconditioning: S = sqrt(|H_ii| + 10)
    Vector SVec = (H.diagonal().abs() + 10).csqrt();
    Matrix HScaled = SVecI * H * SVecI;
    
    // 分块
    Matrix Hmm = HScaled.block(0, 0, mSize, mSize);      // 边缘化块
    Matrix Hma = HScaled.block(0, mSize, mSize, aSize);   // 交叉块
    Matrix Haa = HScaled.block(mSize, mSize, aSize, aSize); // 保留块
    
    // Hmm^{-1} (COD伪逆)
    Matrix HmmInv = Hmm.completeOrthogonalDecomposition().pseudoInverse();
    
    // Schur complement:
    // H_tilde = H_aa - H_ma^T * H_mm^{-1} * H_ma
    // b_tilde = b_a  - H_ma^T * H_mm^{-1} * b_m
    Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
    Vector baNew = ba - Hma.transpose() * HmmInv * bm;
    
    // Unscale + 对称化
    HNewUnscaled = SVec_a * HaaNew * SVec_a;
    HNewUnscaled = 0.5 * (HNewUnscaled + HNewUnscaled^T);
}
```

---

## 4. IMU因子与视觉因子的融合

### 4.1 BAGTSAMIntegration: 桥接 DSO 和 GTSAM

`BAGTSAMIntegration` (`src/GTSAMIntegration/BAGTSAMIntegration.h:126-243`) 是 DM-VIO 的核心架构组件:

**融合核心** (`BAGTSAMIntegration.cpp:123-250`):

```cpp
VecX computeBAUpdate(const MatXX& inputH, const VecX& inputB, 
                     double lambda, vector<EFFrame*>& frames, ...) {
    
    // ===== 步骤1: DSO 视觉 Hessian =====
    // DSO → GTSAM convention (left-sided → right-sided update)
    auto [HFromDSO, bFromDSO] = convertHAndBFromDSO(
        inputH, inputB, *transformation, computeDSOWeight(), 
        ordering, *baEvalValues, baDimMap);
    
    // ===== 步骤2: GTSAM IMU Hessian =====
    auto gtsamHb = baGraphs->getHAndB(*baValues, ordering, baDimMap, ...);
    
    // ===== 步骤3: 直接相加融合! =====
    HFull += HFromDSO;    // 视觉 Hessian + IMU Hessian
    bFull += bFromDSO;    // 视觉梯度 + IMU梯度
    
    // ===== 步骤4: LDLT 求解 =====
    VecX inc = SVecI * H_scaled.ldlt().solve(SVecI * bFull);
    
    // ===== 步骤5: 增量回 GTSAM Values =====
    for(size_t i = 0; i < baOrdering.size(); i++) {
        newBAValues->insert(k, baValues->at(k).retract_(inc.segment(...)));
    }
    
    // ===== 步骤6: 转换回 DSO convention =====
    for(EFFrame* h : frames) {
        Vec6 increment = (newVal * oldVal.inverse()).log();
    }
}
```

### 4.2 IMU 因子添加

`BAIMULogic::addKeyframe()` (`src/IMU/BAIMULogic.cpp:235-323`):

```cpp
void addKeyframe(BAGraphs* baGraphs, Values::shared_ptr baValues, ...) {
    // IMU 预积分因子
    auto&& imuMeasurements = preintegrationProvider->getPreintegratedMeasurements(id);
    
    NonlinearFactor::shared_ptr imuFactor(
        new ImuFactor(prevPoseKey, prevVelKey, currPoseKey, currVelKey, 
                      prevBiasKey, imuMeasurements));
    
    // 坐标转换: IMU 坐标系 → DSO 因子图坐标系
    auto transformedFactor = boost::make_shared<PoseTransformationFactor>(
        imuFactor, *transformDSOToIMU, JACOBIAN_FACTOR);
    baGraphs->addFactor(transformedFactor, METRIC_GROUP);
    
    // Bias 随机游走因子 (σ^2 * Δt 噪声模型)
    auto biasFactor = new BetweenFactor<ConstantBias>(
        prevBiasKey, currBiasKey, ConstantBias(0,0), biasNoiseModel);
    baGraphs->addFactor(biasFactor, BIAS_AND_PRIOR_GROUP);
}
```

### 4.3 因子图分组

`BAIMULogic::FactorGroups` (`src/IMU/BAIMULogic.h:72-75`):
```cpp
enum FactorGroups {
    NO_IMU_GROUP = 0,         // DSO 视觉因子
    BIAS_AND_PRIOR_GROUP,     // Bias RW + 先验
    METRIC_GROUP              // IMU 预积分因子
};
```
初始化时用 `maxGroupInGraph=0` 只含视觉因子, 成功后升级为 `maxGroupInGraph=METRIC_GROUP` 含全因子。

### 4.4 动态 DSO 权重

`BAIMULogic::computeDynamicDSOWeight()` (`src/BAIMULogic.cpp:801-808`):

```cpp
double computeDynamicDSOWeight(double lastDSOEnergy, double lastRMSE, 
                                bool coarseTrackingWasGood) {
    double rmseThresh = imuSettings.dynamicWeightRMSEThresh;  // 默认 ~25
    
    if(lastRMSE < rmseThresh || std::isnan(lastRMSE)) 
        return 1.0;  // 正常跟踪, 视觉权重=1
    
    // RMSE过高 (运动模糊/遮挡) → 降低视觉权重
    double sqrtWeight = rmseThresh / lastRMSE;
    return sqrtWeight * sqrtWeight;  // (τ/RMSE)^2
}
```

**效果**: 当光度 RMSE 上升时, 动态降低视觉因子权重, 系统更依赖 IMU, 避免视觉错误污染状态估计。

### 4.5 尺度收敛与自动固定

`BAIMULogic::finishKeyframeOperations()` (`src/BAIMULogic.cpp:457-497`):

```cpp
void finishKeyframeOperations(int keyframeId) {
    if(optimizeScale && !scaleFixed) {
        // 记录最近 N 帧的尺度范围
        scaleQueue.push_back(make_pair(maxScale, minScale));
        scaleQueue.pop_front();
        
        // 计算尺度波动
        double diff = realMaxScale / realMinScale - 1.0;
        
        // 尺度收敛 → 自动固定!
        if(diff < imuSettings.setting_scaleFixTH) {
            scaleFixed = true;  // 不再优化尺度
        }
    }
}
```

---

## 5. IMU 初始化系统

### 5.1 初始化架构

`IMUInitializer` 负责在系统启动/重定位后初始化:
- 速度 (Velocity): 3×N
- IMU bias: 6 (3 accel + 3 gyro)
- 重力方向: 3
- 尺度: 1

### 5.2 初始化触发

`IMUIntegration::postOptimization()` (`src/IMU/IMUIntegration.cpp:286-299`):
- 每完成一次视觉 BA 后, 传递给 IMU 初始化器
- 初始化器积累足够数据后, 通过回调通知系统

### 5.3 初始化接管

`BAIMULogic::initFromIMUInit()` (`src/BAIMULogic.cpp:683-794`):
- 如果是首次初始化: 添加 IMU 扩展, 启用 scale/gravity/bias 优化
- 如果是重初始化: 保留已有 IMU 参数
- 如果 willReplaceGraph: 用新图替换旧的延迟图

### 5.4 前端 IMU Logic (CoarseIMULogic)

在粗跟踪层面也集成了轻量级的 IMU 因子图:
- `IMUIntegration::computeCoarseUpdate()` (`IMUIntegration.cpp:190-199`)
- 计算视觉 H/b + IMU H/b → 联合求解 → 更新粗跟踪姿态
- 从 BA 获取 bias、速度、尺度作为先验

---

## 6. FEJ (First-Estimates Jacobians) 机制

### 6.1 FEJ 的必要性

边缘化后, 被边缘化因子的线性化点被固定。如果后续优化中这些连接变量的值发生变化, 会引入错误的约束信息。

### 6.2 FEJ 实现

`FEJValues` (`src/GTSAMIntegration/FEJValues.h:38-91`):

```cpp
class FEJValues {
    gtsam::Values fejValues;  // 固定线性化点
    
    // 插入连接变量 (在它们被线性化前)
    template<typename T> 
    void insertConnectedKeys(const T& connectedKeys, const Values& currentValues) {
        // Pose 和 affine 总是插入当前值 (DSO 的 FEJ 策略)
        for(auto&& val : currentValues) {
            if(Symbol(val.key).chr() == 'p' || Symbol(val.key).chr() == 'a') {
                fejValues.insert(val.key, val.value);
            }
        }
        // 其他变量 (vel, bias, scale, gravity) 只插入首次值
        for(auto&& key : connectedKeys) {
            if(!fejValues.exists(key)) {
                fejValues.insert(key, currentValues.at(key));
            }
        }
    }
    
    // 线性化时使用: 混合 FEJ + 当前值
    Values buildValues(const T& neededKeys, const Values& currentValues) {
        Values ret;
        for(auto&& key : neededKeys) {
            if(fejValues.exists(key)) ret.insert(key, fejValues.at(key));
            else ret.insert(key, currentValues.at(key));
        }
    }
};
```

---

## 7. 精度与鲁棒性对比

### 7.1 DM-VIO vs DSO

| 指标 | DSO (纯视觉) | DM-VIO (视觉+IMU) |
|------|-------------|-------------------|
| 尺度 | 不可观 (任意单位) | 可观 (收敛到 ±2%) |
| 旋转精度 | 依赖纯视觉 | IMU 辅助, 更鲁棒 |
| 快速运动 | 容易丢失 (光度残差大) | IMU 桥接, 显著提升 |
| 暗光/模糊 | 完全失效 | 可继续 (IMU主导) |
| 初始化 | 纯视觉, 需要纹理+视差 | IMU+视觉, 更快 |
| 长期漂移 | 无回环, 会累积 | 同样无回环, 但度量尺度更准确 |

### 7.2 DM-VIO vs R3LIVE

| 维度 | DM-VIO | R3LIVE |
|------|--------|--------|
| 方法 | 稀疏直接法 (2000点) | 稠密 RGB 着色 |
| 输出 | 轨迹 + 稀疏点 | 轨迹 + 彩色稠密地图 |
| IMU融合 | 延迟边缘化 | 即时, ESIKF |
| 计算负载 | 中等 (可实时) | 高 (着色+重建) |
| 开源策略 | 完整公开 | 部分代码公开 |

---

## 8. 优缺点 + 对 SLAM 算法的意义

### 8.1 延迟边缘化的核心贡献

1. **解耦 IMU 初始化和边缘化**: IMU 参数在边缘化前有时间收敛, 避免错误信息锁定
2. **多延迟图架构**: 灵活切换主图, 支持 IMU 初始化、重定位
3. **FEJ 的精确实现**: 对视觉变量用 FEJ, 对 IMU 变量用首次线性化点

### 8.2 技术创新总结

| 创新点 | 技术实现 | 文件+行号 |
|--------|---------|-----------|
| **延迟边缘化** | `DelayedGraph` + `readvanceUntilDelay` | `DelayedMarginalization.cpp:212-253` |
| **多延迟图** | `DelayedMarginalizationGraphs` | `h:114-163` |
| **GTSAM+DSO桥接** | `computeBAUpdate` 中 Hessian 相加 | `BAGTSAMIntegration.cpp:174-175` |
| **动态光度权重** | 基于 RMSE 的阈值鲁棒函数 | `BAIMULogic.cpp:801-808` |
| **自动尺度收敛** | scaleQueue 监控 + scaleFixed | `BAIMULogic.cpp:457-497` |
| **FEJ协调** | `FEJValues::insertConnectedKeys` | `FEJValues.h:44-62` |

### 8.3 对 SLAM 算法的深层意义

1. **范式意义**: DM-VIO 证明了直接法 VIO 在精度上能达到甚至超越间接法 VIO (VINS-Fusion), 同时保持高效的实时性能
2. **架构意义**: `BAGraphs` 抽象使得边缘化策略 (即时/延迟/动态) 可以自由切换, 为新算法提供了实验平台
3. **工程意义**: GTSAM 的因子图框架 + DSO 的光度框架的"插件式"集成 (`BAExtension`), 使得添加新传感器 (GPS, 轮速计等) 只需实现一个 Extension

### 8.4 主要局限

- **无回环检测**: 与 DSO 相同, 无长期约束, 漂移不可逆
- **初始化仍需时间**: 虽然比 VINS-Fusion 快, 但 IMU 初始化仍需数秒
- **光度误差对暗光敏感**: IMU 可以辅助, 但极端暗光视觉因子完全失效
- **GTSAM 依赖**: 代码结构较复杂 (比 DSO 的纯手写求解器多 3x 代码)
- **单目限制**: 代码不支持双目/深度相机

---

## 9. 对比: FAST-LIVO2 vs DM-VIO vs DSO 的技术路线差异

| 维度 | DSO | DM-VIO | R3LIVE |
|------|-----|--------|--------|
| **前端** | 直接法, 8-point pattern | 继承 DSO 直接法 | 稠密直接法 (全像素) |
| **IMU融合** | ✗ | GTSAM因子图 + 预积分 | ESIKF + 即时耦合 |
| **边缘化策略** | 手动 Schur (DSO H,b) | 延迟边缘化 (GTSAM) | 即时 (ESIKF的predict) |
| **地图输出** | 稀疏逆深度地图 | 稀疏逆深度地图 | 稠密彩色点云 |
| **初始化** | 纯视觉 (trackFrame) | IMU+视觉 (IMUInitializer) | LiDAR-IMU耦合 |
| **尺度可观性** | ✗ (任意单位) | ✓ (IMU提供度量单位) | ✓ (IMU提供) |
| **回环检测** | ✗ | ✗ | ✗ |
| **开源完整度** | 完整 (单目) | 完整 (单目+IMU) | 部分 (R3LIVE++更完整) |
| **代码复杂度** | ~5000 行 | ~15000 行 | ~20000+ 行 |
| **典型应用** | 视觉VO基准 | 无人机/MR设备VIO | AR/着色三维重建 |

### DM-VIO 的核心设计哲学: "延迟即鲁棒"

DM-VIO 本质上回答了: 在一个因子图优化框架中, 何时应该边缘化?

VINS-Fusion 的回答: 立即 (立即边缘化最旧帧)
DM-VIO 的回答:  等待 (等到信息足够确定, 再执行边缘化)

这个"等待"策略是 DM-VIO 在几乎所有数据集上都优于 VINS-Fusion 的根本原因。

---

## 10. 数据管线

### 10.1 传感器输入总览

| 传感器 | 频率 | 数据格式 | 接口 | 负责模块 |
|--------|------|----------|------|----------|
| 相机 (mono) | 10-30 Hz | `cv::Mat` (Gray, 光度标定后) | `ImageFolderReader` / `RealsenseT265` → `FullSystem::addActiveFrame` | DSO `FullSystem`, `CoarseTracker` |
| IMU (6 轴) | 100-200 Hz | `IMUMeasurement {acc[3], gyr[3], dt}` | 数据集文件 / T265 IMU → `IMUIntegration::addIMUData` | `IMUIntegration`, `BAIMULogic`, `CoarseIMULogic` |

### 10.2 相机数据处理管线 (DSO 直接法)

#### 原始数据
- **图像格式**: `cv::Mat` (Gray, CV_8UC1)，需光度标定
- **光度标定**: `vignette` (渐晕), `response` (响应函数), `exposure` (曝光时间), 通过 `responseCalibration` 应用
- **相机模型**: DSO 自定义 `PinholeRadTan8` / `PinholeEqui7`，内参从文件加载
- **外参**: `T_cam_imu` (Camera→IMU)，YAML 标定文件

#### 光度矫正 (`ImageAndExposure::ImageAndExposure`)
- 每像素 `I = G^{-1}(I_raw) / V(x,y)` (逆响应函数 + 渐晕补偿)
- 曝光时间归一化: `I_norm = I / exp_time`

#### 像素选取 (`PixelSelector::makeMaps`, `src/dso/FullSystem/PixelSelector2.cpp`)
1. 在图像金字塔每层独立选择像素
2. 选择策略: 梯度大 + 梯度方向分散 (确保约束多样性)
3. 目标数量: `setting_desiredImmatureDensity ≈ 2000` 点/帧
4. 生成 `ImmaturePoint` (未成熟点): 携带宿主帧、像素坐标、梯度、金字塔层级

#### 前端跟踪 (`CoarseTracker::trackNewestCoarse`, `src/dso/FullSystem/CoarseTracker.cpp:536-718`)
- 使用 8 点固定 pattern 在像素邻域计算光度残差:
  `r = I_host(p_host) - I_target(p_host + dp)`
- LM 优化: 仅优化当前帧相对于参考帧的相对位姿 (6 DoF)
- 结合 IMU 预测做初始猜测 (`CoarseIMULogic`)

#### 特征成熟化 (`ImmaturePoint::traceOn`)
- 沿极线搜索最佳深度匹配 (NCC 相关)
- 收敛后提升为 `PointHessian` (成熟点, 进入 BA)

### 10.3 IMU 数据处理管线

#### 原始数据
- **测量**: `IMUMeasurement {Eigen::Vector3d accData, gyrData; double integrationTime}`
- **噪声配置**: `accel_sigma, gyro_sigma, integration_sigma` + 偏置随机游走 `accel_bias_sigma, gyro_bias_sigma`
- **重力**: `imuCalibration.gravity = [0,0,-9.81]`

#### 三路分发 (`IMUIntegration::addIMUData`, `src/IMU/IMUIntegration.cpp:150-180`)
```
1. 送入 IMU 初始化器 (如未初始化)
2. 送入 CoarseIMULogic (前端跟踪)
3. 送入 BA 预积分缓冲区
```

#### GTSAM 预积分 (`gtsam::PreintegratedImuMeasurements`)
```
preintegrationParams->setIntegrationCovariance(integrationVar·I)
preintegrationParams->setAccelerometerCovariance(accelVar·I)
preintegrationParams->setGyroscopeCovariance(gyroVar·I)
```

- 使用 GTSAM 标准 IMU 预积分实现 (Forster et al.)
- **双缓冲区** (`IMUIntegration.cpp:104-105`):
  - `preintegratedBA`: 上次 KF→当前 KF
  - `preintegratedBACurr`: 当前 KF→下一帧
  - `prepareKeyframe()` 中 `swap` 实现跟踪和 BA 并行

### 10.4 算法消费：因子构造与融合

#### 视觉因子 (DSO 光度 Hessian)
`src/dso/FullSystem/Residuals.cpp:78-274`:

光度残差 (1 维/观测):
```
r_k = I_h(p) - I_t(p ± δp)
```
- 8 点 pattern 在像素邻域采样
- Jacobian 链式: `dr/dPos` = `(dr/dI)*(dI/dp)*(dp/dPc)*(dPc/dPose)`
- 包含曝光时间 Jacobian
- Huber 核加权
- **输出 Hessian**: `H_DSO` + `b_DSO` (由 `AccumulatedTopHessian` 累加)

#### IMU 因子 (`BAIMULogic::addKeyframe`, `src/IMU/BAIMULogic.cpp:235-323`)

**原始 IMU 因子** (GTSAM `ImuFactor`):
```
r_R = Log(dR^T * R_i^T * R_j)
r_V = R_i^T*(V_j - V_i - g*Δt) - dV
r_P = R_i^T*(P_j - P_i - V_i*Δt - 0.5*g*Δt²) - dP
```
- 优化变量: `[PoseKey_i, VelKey_i, PoseKey_j, VelKey_j, BiasKey_i]`

**坐标转换因子** (`PoseTransformationFactor`):
```
transformedFactor = PoseTransformationFactor(imuFactor, transformDSOToIMU)
```
- `transformDSOToIMU`: 包含 `scale`, `gravity_direction`, `T_cam_imu`
- 将 IMU 因子坐标系 (metric) 转换为 DSO 因子图坐标系 (光度标定)
- 支持 Jacobian 因子模式 (`JACOBIAN_FACTOR`)

**偏置随机游走因子**:
```
biasFactor = BetweenFactor<ConstantBias>(prevBiasKey, currBiasKey, 
                ConstantBias(0,0), noiseModel=σ²·Δt)
```

#### 联合优化 (`BAGTSAMIntegration::computeBAUpdate`, `src/GTSAMIntegration/BAGTSAMIntegration.cpp:123-250`)
```
步骤1: convertHAndBFromDSO(inputH, inputB, *transformation, ...)
步骤2: baGraphs->getHAndB(*baValues, ordering, ...)     ← IMU 因子 Hessian
步骤3: H_full = H_DSO + H_IMU        // 直接相加!
        b_full = b_DSO + b_IMU
步骤4: inc = LDLT_solve(H_full, b_full)                 // 联合求解
步骤5: 增量回 GTSAM Values → 转换回 DSO convention
```

#### 动态 DSO 权重 (`computeDynamicDSOWeight`, `src/IMU/BAIMULogic.cpp:801-808`)
```
if(RMSE < rmseThresh) weight = 1.0
else weight = (rmseThresh/RMSE)²  // 光度误差大 → 降低视觉权重 → 更依赖 IMU
```

#### 延迟边缘化 (`DelayedGraph::readvanceUntilDelay`, `src/GTSAMIntegration/DelayedMarginalization.cpp:212-253`)
```
while(marginalizationOrder.size() > delayN):
    1. fejValues->insertConnectedKeys(keysToMarg, delayedValues)  // 插入 FEJ 值
    2. graph = marginalizeOut(graph, fejValues, keysToMarg, ...)  // Schur complement
    3. 清理 edge 列表和 values

computeSchurComplement(augmentedH, mSize, aSize):
    preconditioning: S = sqrt(|H_ii|+10)
    Hmm^{-1} = Hmm.completeOrthogonalDecomposition().pseudoInverse()  // COD 伪逆
    H_tilde = H_aa - H_ma^T·H_mm^{-1}·H_ma       // Schur complement
    b_tilde = b_a - H_ma^T·H_mm^{-1}·b_m
```

### 10.5 跨传感器协同

#### 时间同步
- 图像时间戳为主基准
- `IMUInterpolator`: 线性插值 IMU 到图像时间戳
- `delay_imu_cam` 参数补偿硬件延迟

#### 数据缓冲策略
- IMU: 双线预积分缓冲区 (`preintegratedBA` / `preintegratedBACurr`)
- 帧管理: DSO 滑动窗口 + GTSAM 延迟图并行
- 因子分组: `NO_IMU_GROUP(0) → BIAS_AND_PRIOR_GROUP(1) → METRIC_GROUP(2)`

#### 初始化管线 (`IMUInitializer` → `IMUInitialization/`)
1. **Coarse IMU Init**: 前端跟踪中轻量 IMU 因子图，估计初始速度/bias/gravity/scale
2. **IMU-视觉初始化**: 积累足够帧后 `IMUInitializerLogic::transition`
3. **回调接管**: `baLogic->initFromIMUInit(values, reinit, willReplaceGraph)`:
   - 首次: 添加 IMU 扩展, 启用 scale/gravity/bias 优化
   - `willReplaceGraph=true`: 新延迟图替换旧图 (含 IMU 因子+先验)
4. **尺度自动固定** (`finishKeyframeOperations`): scaleQueue 监控波动 → `diff<scaleFixTH` → `scaleFixed=true` 不再优化尺度

#### 降级与异常处理
- 动态 DSO 权重: 光度 RMSE 上升时降低视觉权重
- 多延迟图架构: 初始化前图 (大 delay, 纯视觉) → 初始化后替换主图 (含 IMU)
- FEJ: 视觉变量用当前值 FEJ, IMU 变量用首次线性化点
- 无回环检测 (纯 VIO)

