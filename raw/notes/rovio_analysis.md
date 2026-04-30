# ROVIO 深度源码分析

> **Robust Visual Inertial Odometry (ROVIO)**  
> ETH Zurich, Autonomous Systems Lab  
> EKF-based, patch-based direct VIO  
> GitHub: 1245 stars

---

## 1. 核心创新: 直接使用图像块 (patch-based, 不提取特征点!)

ROVIO 最核心的哲学创新在于：**完全摒弃传统视觉 SLAM 的特征点提取与匹配流程**，转而直接对原始图像块 (image patch) 进行操作。传统 SLAM 系统（如 ORB-SLAM、MSCKF）都依赖 FAST 角点检测 + ORB 描述子 + KLT 光流（或描述子匹配）这一经典范式，而 ROVIO 从根本上质疑：为什么需要这些中间表示？

### 1.1 为什么不提取特征点？

传统特征点方法的问题：
1. **FAST/Shi-Tomasi 角点提取**本质上是启发式的——它选出"看起来像角"的像素块，但并不保证这些块对跟踪是最优的
2. **描述子匹配**引入了额外的计算开销和匹配模糊性
3. **特征点的生命周期管理**（什么时候初始化、什么时候删除）增加了系统复杂性
4. 从信息论角度看，描述子的量化过程丢失了信息

ROVIO 的做法：直接将图像块 (patch) 作为"特征"——在 EKF 状态中同时估计 IMU 位姿和这些图像块对应的路标的 bearing vector + depth，通过**光度误差 (photometric error)** 最小化来同时更新两者。

### 1.2 Patch 的数据结构

核心实现在 `include/rovio/Patch.hpp:41-305`：

```cpp
template<int patchSize>
class Patch {
public:
  float patch_[patchSize*patchSize];  // 图像块像素值
  float patchWithBorder_[(patchSize+2)*(patchSize+2)];  // 带边界的扩展块（用于梯度计算）
  float dx_[patchSize*patchSize];  // x方向梯度
  float dy_[patchSize*patchSize];  // y方向梯度
  Eigen::Matrix3f H_;  // Hessian（用于块对齐）
  float s_;  // Shi-Tomasi 分值
};
```

关键细节（`Patch.hpp:72-101`）：
- 梯度通过中心差分计算：`J[0] = 0.5 * (it[1] - it[-1])`（x方向），`J[1] = 0.5 * (it[refStep] - it[-refStep])`（y方向）
- Hessian 矩阵为 3x3（包含仿射光照分量 `J[2] = 1`）
- Shi-Tomasi 分值 `s_ = e0_ + e1_`（两个特征值之和）

多层级版本在 `include/rovio/MultilevelPatch.hpp:44-211`：
- 包含 `nLevels_` 个层级的 patch
- 每个层级的 patch 尺寸相同，但对应不同分辨率的图像金字塔
- 多层级 Shi-Tomasi 分值按 `pow(0.25, i)` 加权各层贡献

### 1.3 Patch 的提取与 warp

Patch 提取（`Patch.hpp:240-304`）支持两种模式：
1. **无 warp 模式** (`isNearIdentityWarping()`)：简单的双线性插值
2. **有 warp 模式**：先通过 warp 矩阵变换坐标，再双线性插值

warp 信息存储在 `FeatureCoordinates` 中（`FeatureCoordinates.hpp:60-100`）：
- `warp_c_`（Eigen::Matrix2f）：像素坐标的仿射变换矩阵
- `warp_nor_`（Eigen::Matrix2d）：bearing vector 的仿射变换矩阵
- `trackWarping_`：是否跟踪 warp 变化

warp 在 `ImuPrediction.hpp:124-130` 中通过 IMU 动力学预测：
```cpp
if(state.CfP(i).trackWarping_){
    bearingVectorJac_ = output.CfP(i).get_nor().getM().transpose()*(dt*gSM(...)*Lmat(dm)*(...)+MPD(qm).matrix())*oldC_.get_nor().getM();
    output.CfP(i).set_warp_nor(bearingVectorJac_*oldC_.get_warp_nor());
}
```

这意味着 warp 不仅考虑了几何变换，还通过 IMU 运动学将旋转/平移运动传播到 patch 上，使得大运动下的直接法跟踪成为可能。

---

## 2. EKF状态设计

ROVIO 的状态向量定义在 `include/rovio/FilterStates.hpp:110-444`，是模板化的：

```cpp
template<unsigned int nMax, int nLevels, int patchSize, int nCam, int nPose>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,   // pos + vel + acb + gyb
    LWF::QuaternionElement,                                 // att (qWM)
    LWF::ArrayElement<LWF::VectorElement<3>,nCam>,         // vep (MrMC)
    LWF::ArrayElement<LWF::QuaternionElement,nCam>,        // vea (qCM)
    LWF::ArrayElement<RobocentricFeatureElement,nMax>,     // fea (路标)
    LWF::ArrayElement<LWF::VectorElement<3>,nPose>,        // pop
    LWF::ArrayElement<LWF::QuaternionElement,nPose>,       // poa
    StateAuxiliary<nMax,nLevels,patchSize,nCam>>           // 辅助状态
```

### 2.1 IMU 状态 (15 维 + nCam*6 维外参)

| 索引 | 变量名 | 含义 | 维度 |
|------|--------|------|------|
| `_pos` | `WrWM` | IMU 在世界系中的位置 | 3 |
| `_vel` | `MvM` | IMU 在自身系中的速度 | 3 |
| `_acb` | `acb` | 加速度计偏置 | 3 |
| `_gyb` | `gyb` | 陀螺仪偏置 | 3 |
| `_att` | `qWM` | IMU 到世界系的四元数 | 4(3) |
| `_vep` | `MrMC[i]` | IMU 到相机 i 的平移 (外参) | nCam*3 |
| `_vea` | `qCM[i]` | IMU 到相机 i 的旋转 (外参) | nCam*4(3) |

关键设计（`FilterStates.hpp:279-294`）：外参可以是**在线的**（通过 `doVECalibration_` 控制）：
```cpp
inline QPD& qCM(const int camID = 0){
    if(this->template get<_aux>().doVECalibration_){
        return this->template get<_vea>(camID);  // 在线估计
    } else {
        return this->template get<_aux>().qCM_[camID];  // 固定参数
    }
}
```

### 2.2 路标状态参数化

每个路标在 EKF 中被参数化为 **robotcentric** 的坐标（`FilterStates.hpp:144` `_fea`），存储在 `RobocentricFeatureElement` 中（`RobocentricFeatureElement.hpp`）：

```cpp
struct RobocentricFeatureElement {
    FeatureCoordinates coordinates_;  // bearing vector + 像素坐标
    FeatureDistance distance_;         // 深度参数
};
```

**Bearing Vector** (`FeatureCoordinates.hpp:66`)：
- `nor_`：`LWF::NormalVectorElement` 类型，即单位向量，表示路标在相机系中的方向
- `c_`：`cv::Point2f`，对应的像素坐标
- 二者通过 `mpCamera_` 指针中的相机模型互相转换

**深度参数** (`FeatureDistance.hpp:36-247`)：
- 支持 4 种参数化方式：
  - `REGULAR`：d = p（常规距离）
  - `INVERSE`：d = 1/p（逆深度）
  - `LOG`：d = exp(p)（对数深度）
  - `HYPERBOLIC`：d = sinh(p)（双曲深度）
- 每种参数化都提供了 `getDistance()`, `getDistanceDerivative()`, `getParameterDerivative()` 等方法
- 选择通过 `depthTypeInt_` 控制（`RovioFilter.hpp:189`）

**为什么是 robotcentric？**
- 将路标坐标表示在当前相机帧中（而非世界坐标系）
- 优势：雅可比更稀疏，数值更稳定
- 劣势：在状态传播时需要更新所有路标的 bearing vector

每个路标状态共 3 维（2 维 bearing + 1 维 depth）= `特征总数 * 3` 维

### 2.3 额外 Pose 状态

`_pop` 和 `_poa`（`FilterStates.hpp:145-146`）允许在状态中加入一个或多个额外的 6-DoF 位姿。这通常是一个外部测量位姿（如 GPS/Vicon），用于：
1. 对齐坐标系
2. 提供全局约束
3. `PoseUpdate` 模块处理此前缀状态

---

## 3. 图像块 warp 与光度误差

这是 ROVIO 的**核心更新机制**，实现在 `include/rovio/ImgUpdate.hpp:159-1121`。

ROVIO 支持两种更新模式，通过 `useDirectMethod_` 参数切换：

### 3.1 直接法 (Photometric Update, useDirectMethod_=true)

流程（`ImgUpdate.hpp:406-439`, `evalInnovation`）：

1. **获取路标在目标相机中的投影** (`FeatureOutput`)：
```cpp
transformFeatureOutputCT_.setFeatureID(ID);
transformFeatureOutputCT_.setOutputCameraID(activeCamID);
transformFeatureOutputCT_.transformState(state, featureOutput_);
```

2. **计算光度对齐方程** (`MultilevelPatchAlignment`)：
```cpp
if(alignment_.getLinearAlignEquationsReduced(
       meas_.aux().pyr_[activeCamID],
       *state.aux().mpCurrentFeature_->mpMultilevelPatch_,
       featureOutput_.c(), endLevel_, startLevel_,
       A_red_, b_red_))
```

这里 `A_red_` 是**降阶的雅可比矩阵**（2x2），`b_red_` 是**降阶的亮度误差向量**（2x1）。关键数学：
- `A_red_` 封装了图像梯度与像素坐标对 bearing vector 的雅可比之间的关系
- `b_red_` 是参考块与当前图像在投影位置处的残差

3. **创新项 (innovation)**：
```cpp
y.template get<mtInnovation::_pix>() = b_red_ + noise;
```

4. **状态雅可比** (`ImgUpdate.hpp:554-562`, `jacState`)：
```cpp
transformFeatureOutputCT_.jacTransform(featureOutputJac_, state);
mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);
F = -A_red_ * c_J_ * featureOutputJac_.template block<2,mtState::D_>(0,0);
```

这里展示了链式法则：
- `featureOutputJac_`：FeatureOutput 对状态的雅可比
- `c_J_`：像素坐标对 bearing vector 的雅可比
- `A_red_`：光度误差对像素坐标的降阶雅可比

**多相机跨相机测量** (`useCrossCameraMeasurements_=true`)：
- 特征在相机 A 中初始化，但更新可以在相机 B 中执行
- `noiseGainForOffCamera_` 对非主相机的噪声进行缩放

### 3.2 间接法 (Reprojection Error, useDirectMethod_=false)

流程（`ImgUpdate.hpp:434-439`）：
```cpp
Eigen::Vector2d pixError;
pixError(0) = state.aux().feaCoorMeas_[ID].get_c().x - featureOutput_.c().get_c().x;
pixError(1) = state.aux().feaCoorMeas_[ID].get_c().y - featureOutput_.c().get_c().y;
y.template get<mtInnovation::_pix>() = pixError + noise;
```

此时 `feaCoorMeas_[ID]` 是在 `preProcess` 中通过 2D 对齐找到的匹配位置。

### 3.3 迭代 EKF 候选生成

ROVIO 使用 IEKF (Iterated EKF) 来进一步优化，候选状态通过 `generateCandidates` 方法（`ImgUpdate.hpp:442-475`）：
```cpp
canditateGenerationH_ = -c_J_ * featureOutputJac_.template block<2,mtState::D_>(0,0);
canditateGenerationPy_ = canditateGenerationH_ * filterState.cov_ * canditateGenerationH_.transpose();
```

候选点在像素空间中沿协方差特征向量方向采样，确保在 2σ 范围内覆盖。

### 3.4 异常值检测

在 `extraOutlierCheck` (`ImgUpdate.hpp:477-537`) 中：
1. 检查迭代是否收敛
2. 检查块是否在图像内
3. 计算平均光度误差，与 `patchRejectionTh_` 比较
4. 可选的 discriminative 检查：在周围 4 个方向采样，至少 2 个超过阈值（确保特征位置确实有纹理变化）

### 3.5 块更新策略

块（参考块）不会每帧更新，而是满足条件时才更新（`ImgUpdate.hpp:908-920`）：
- 特征在上一次更新中被成功跟踪
- 距离上次块更新超过 `minTimeBetweenPatchUpdate_`
- 新的 Shi-Tomasi 分值 > `minRelativeSTScore_ * 之前的Shi-Tomasi分值` 且 > `minAbsoluteSTScore_`

这避免了块漂移问题。

---

## 4. IMU 传播

IMU 预测在 `include/rovio/ImuPrediction.hpp:39-291` 中实现。

### 4.1 核心方程 (`evalPrediction`, 行 100-156)

**IMU 状态传播**（标准 EKF 预测）：

1. 旋转传播（`ImuPrediction.hpp:105`）：
```cpp
QPD dQ = dQ.exponentialMap(dt * imuRor);
output.qWM() = state.qWM() * dQ;
```

2. 位置传播（行 133）：
```cpp
output.WrWM() = state.WrWM() - dt * (state.qWM().rotate(state.MvM()) - noise/sqrt(dt));
```

3. 速度传播（行 134）：
```cpp
output.MvM() = (I - gSM(dOmega)) * state.MvM() 
    - dt * (acc_meas - state.acb() + state.qWM().inverseRotate(g_) - noise/sqrt(dt));
```

4. 偏置传播（行 135-136）：随机游走模型
```cpp
output.acb() = state.acb() + noise * sqrt(dt);
output.gyb() = state.gyb() + noise * sqrt(dt);
```

**路标状态传播** (robotcentric)：

Bearing vector 传播（`ImuPrediction.hpp:117-122`）：
```cpp
const V3D camRor = state.qCM(camID).rotate(imuRor);
const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID)) - state.MvM()));
// bearing vector 的运动学更新
V3D dm = -dt*(gSM(oldC_.get_nor().getVec())*camVel/oldD_.getDistance()
    + (I-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())*camRor)
    + oldC_.get_nor().getN()*noise*sqrt(dt);
QPD qm = qm.exponentialMap(dm);
output.CfP(i).set_nor(oldC_.get_nor().rotated(qm));
```

深度参数传播（行 117）：
```cpp
output.dep(i).p_ = oldD_.p_ - dt * oldD_.getParameterDerivative() * oldC_.get_nor().getVec().transpose() * camVel + noise*sqrt(dt);
```

这里 `camVel` 是**相机系中看到的路标速度**，与 `camRor` 一起决定了 bearing vector 在切空间中的变化量 `dm`。

### 4.2 雅可比矩阵

`jacPreviousState` (`ImuPrediction.hpp:161-241`) 计算了完整的 F 矩阵，包含：
- 标准 IMU 状态间的耦合（pos-vel-att 交叉项）
- 路标 bearing vector 对 vel, gyb, att 的偏导
- 路标深度对 vel 的偏导
- 外参（vep, vea）对路标的偏导（如果在线标定）

这保证了 EKF 协方差传播的正确性。

### 4.3 运动检测

`detectInertialMotion` (`ImuPrediction.hpp:285-289`)：
```cpp
bool detectInertialMotion(const mtState& state, const mtMeas& meas) const{
    const V3D imuRor = meas.gyr - state.gyb();
    const V3D imuAcc = meas.acc - state.acb() + state.qWM().inverseRotate(g_);
    return (imuRor.norm() > 0.1) | (imuAcc.norm() > 0.1);
}
```

用于零速更新检测。

---

## 5. 与 MSCKF (open_vins) 的对比

### 5.1 相似点

| 方面 | ROVIO | MSCKF |
|------|-------|-------|
| 滤波框架 | EKF/IEKF | EKF |
| 路标处理 | 在状态中显式估计 | 边缘化掉（不在状态中） |
| IMU 积分 | 标准 IMU 动力学 | 标准 IMU 动力学 |
| 外参在线标定 | 支持 | 支持 |
| 零速更新 | 支持 | 支持 |

### 5.2 关键差异

**路标管理范式**：

- **MSCKF**：路标不在状态中，通过跨帧的多视角约束对位姿/IMU 状态进行更新，然后边缘化掉路标。状态维度固定（～21 维）。
- **ROVIO**：路标在状态中，每个路标 3 维（bearing + depth）。状态维度随路标数量线性增长（nMax × 3 + 15 + nCam×6）。

**观测模型**：

- **MSCKF**：重投影误差（传统的几何误差），需要特征匹配。
- **ROVIO**：光度误差（直接法），不需要特征匹配，直接比较像素强度。

**精度与鲁棒性**：

- MSCKF 通过 FEJ (First-Estimates Jacobian) 避免了 EKF 的线性化误差累积，理论上一致性更好。
- ROVIO 使用 IEKF（迭代 EKF），通过重新线性化缓解线性化误差，但不如 FEJ 彻底。
- 在光照变化场景中，ROVIO 的亮度恒定假设可能失效，而 MSCKF 的几何约束更鲁棒。
- 在纹理丰富但特征不明显的场景中（如白墙上的微小纹理），ROVIO 的直接法优势明显。

**计算效率**：

- MSCKF：状态维度固定，协方差矩阵大小固定（O(1)），但需要维护滑动窗口中的相机位姿。
- ROVIO：状态维度随路标增长（O(n)），协方差更新复杂度 O(n²)。好在 nMax 通常限制在 20-30。

### 5.3 技术选择的哲学差异

ROVIO 代表"**将尽可能多的信息放入滤波器**"（full-smoothing 方向），MSCKF 代表"**边缘化无关信息以保持效率**"（marginalization 方向）。ROVIO 在路标被持续跟踪期间能积累更多信息，而 MSCKF 通过边缘化失去了路标间的时间关联性。

---

## 6. 优缺点 + 对SLAM算法的意义

### 6.1 优点

1. **无特征提取/匹配开销**：直接从像素到 EKF 更新，省去了描述子计算和匹配搜索
2. **信息利用效率高**：每一帧图像直接通过光度误差作用于滤波器的全部状态（IMU + 路标），不像 MSCKF 需要累积足够视差才能更新
3. **低纹理场景鲁棒**：不依赖角点检测，只要有强度梯度的地方都能工作
4. **多相机原生支持**：`nCam` 模板参数 + `useCrossCameraMeasurements_` 使得多相机扩展非常自然（`MultiCamera.hpp`）
5. **在线外参标定**：`doVECalibration_` 机制无缝集成

### 6.2 缺点

1. **亮度恒定假设**：在光照变化、曝光变化、自动白平衡场景中容易失败。虽然支持 `alpha`/`beta` 仿射补偿（`alignment_.useIntensityOffset_`/`useIntensitySqew_`），但仍不完美
2. **状态维度线性增长**：nMax 路标 × 3 维，限制了可长期维护的路标数量（通常 20-30 个）
3. **线性化误差累积**：EKF 对 bearing vector 和 depth 的线性化误差可能在不一致的线性化点下累积
4. **初始化依赖**：初始深度 `initDepth_` 和初始化协方差 `initCovFeature_` 的选择对收敛速度影响大
5. **无回环检测**：纯滤波方案，没有全局优化环节

### 6.3 对 SLAM 算法的意义

ROVIO 证明了一个重要观点：**在 VIO 场景中，描述子匹配不是必需的**。IMU 的运动学先验已经提供了足够好的位姿预测，使得直接光度误差的最小化成为可能。这是一条"从 feature-based 到 direct"的路径，与 DSO (Direct Sparse Odometry) 在纯视觉 SLAM 中的理念一脉相承。

然而，ROVIO 的方法论也揭示了直接法 VIO 的根本矛盾：
- **IMU 提供短时运动预测**（高频但漂移）→ 使直接法能在大运动下收敛
- **光度误差提供长期修正**（低频但无漂移）→ 但在光照变化时不可靠

因此 ROVIO 最适合的场景是：**恒光、短程、低延迟**的机器人导航（如无人机悬停、仓库机器人），而非大规模、长程、光照多变的室外场景。

### 6.4 代码架构总结

ROVIO 的架构高度模板化和泛化：
- `lightweight_filtering/`：通用的 EKF/IEKF 框架
- `rovio/FilterStates.hpp`：通过模板参数定义状态维度（nMax, nLevels, patchSize, nCam, nPose）
- `rovio/ImuPrediction.hpp`：IMU 传播，自动处理 robotcentric 路标的运动学
- `rovio/ImgUpdate.hpp`：1126 行的核心更新逻辑，包含预/后处理、候选生成、IEKF 迭代
- `rovio/FeatureManager.hpp`：路标/块的生命周期管理（添加、删除、桶排序）

---

## 附录：关键源文件索引

| 文件 | 行数 | 核心内容 |
|------|------|----------|
| `include/rovio/Patch.hpp` | 310 | 单层图像块数据结构与提取 |
| `include/rovio/MultilevelPatch.hpp` | 216 | 多层图像块与Shi-Tomasi分值 |
| `include/rovio/FilterStates.hpp` | 662 | EKF状态向量完整定义 |
| `include/rovio/ImuPrediction.hpp` | 295 | IMU传播与robotcentric路标运动学 |
| `include/rovio/ImgUpdate.hpp` | 1126 | 图像更新：直接法/间接法、IEKF、路标管理 |
| `include/rovio/FeatureDistance.hpp` | 252 | 四种深度参数化 |
| `include/rovio/FeatureCoordinates.hpp` | 323 | Bearing vector + 像素坐标 + warp |
| `include/rovio/FeatureManager.hpp` | 492 | 路标集合管理与候选筛选 |
| `include/rovio/RovioFilter.hpp` | 238 | 顶层滤波器组装 |
| `include/rovio/MultilevelPatchAlignment.hpp` | - | 多层块对齐（光度误差最小化核心） |
| `include/rovio/RobocentricFeatureElement.hpp` | - | Robotcentric路标元素 |
| `include/rovio/Camera.hpp` | - | 相机模型接口 |
| `include/rovio/MultiCamera.hpp` | - | 多相机管理 |
| `src/rovio_node.cpp` | - | ROS节点入口 |
| `src/FeatureDistance.cpp` | - | 深度参数化实现 |