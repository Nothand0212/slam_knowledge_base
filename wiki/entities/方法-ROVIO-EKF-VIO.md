---
tags: [ROVIO, EKF, VIO, 光度误差, 多级斑块追踪, IMU, 直接法]
created: 2026-05-15
updated: 2026-05-15
type: entity
sources:
  - raw/codes/rovio/include/rovio/RovioFilter.hpp
  - raw/codes/rovio/include/rovio/FilterStates.hpp
  - raw/codes/rovio/include/rovio/ImuPrediction.hpp
  - raw/codes/rovio/include/rovio/ImgUpdate.hpp
  - raw/codes/rovio/include/rovio/MultilevelPatchAlignment.hpp
  - raw/codes/rovio/include/rovio/FeatureDistance.hpp
---

# ROVIO: EKF-based Visual Inertial Odometry

> ROVIO (Robust Visual Inertial Odometry) 是 ETH Zurich ASL 开发的高效 EKF-based VIO。核心创新：将特征以 robocentric bearing vector + 距离参数形式放入 EKF 状态，通过多级斑块（multilevel patch）的光度误差直接驱动更新，无需显式特征提取-匹配管线。

## 系统架构

```
IMU 测量 ──→ [ImuPrediction] ──→ 状态预测 ──→ EKF 传播
                                          │
图像帧 ────→ [ImgUpdate] ──→ 斑块对齐 ──→ EKF 更新
                │
                ├─ 特征检测 (FAST on pyr)
                ├─ 多级斑块提取
                ├─ 光度误差计算
                └─ 直接/间接创新项
```

ROVIO 的核心设计：**每个特征都是一个 EKF 状态**。Feature 参数化采用 robocentric 表示：bearing vector `n`（2 DOF on $S^2$）加上距离参数 `ρ`（1 DOF）。每个特征绑定一个多级图像斑块，更新时不使用重投影误差而直接用**光度误差**驱动（direct method）。

## EKF 状态向量

状态定义在 `FilterStates.hpp:L110-L148`：

$$
\mathbf{x} =
\begin{bmatrix}
_W \mathbf{r}_{WM} \\  % 位置 (世界系IMU->世界)
_M \mathbf{v}_M \\      % 速度 (IMU系)
\mathbf{b}_a \\         % 加速度计bias
\mathbf{b}_g \\         % 陀螺仪bias
\mathbf{q}_{WM} \\      % 姿态 (IMU->世界)
_M \mathbf{r}_{MC}^{(i)} \\  % IMU->相机外参平移 (每个相机)
\mathbf{q}_{CM}^{(i)} \\     % IMU->相机外参旋转 (每个相机)
\mathbf{n}^{(j)}, \rho^{(j)} \\  % robocentric特征 (最多nMax个)
\mathbf{p}_{\text{add}}^{(k)}, \mathbf{q}_{\text{add}}^{(k)}  % 附加pose
\end{bmatrix}
$$

**源码锚点**: `raw/codes/rovio/include/rovio/FilterStates.hpp:L137-L147`

```cpp
static constexpr unsigned int _pos = 0;   // WrWM 位置
static constexpr unsigned int _vel = _pos+1;  // MvM 速度
static constexpr unsigned int _acb = _vel+1;  // 加速度计bias
static constexpr unsigned int _gyb = _acb+1;  // 陀螺仪bias
static constexpr unsigned int _att = _gyb+1;  // qWM 姿态
static constexpr unsigned int _vep = _att+1;  // IMU->相机平移
static constexpr unsigned int _vea = _vep+1;  // IMU->相机旋转
static constexpr unsigned int _fea = _vea+1;  // robocentric特征
```

关键设计点：
- **Robocentric**：特征在相机系下参数化，bearing vector + 距离，避免世界坐标系线性化点问题
- **距离参数化可选**：支持 `REGULAR`（`d`）、`INVERSE`（`1/d`）、`LOG`（`ln(d)`）、`HYPERBOLIC`（`asinh(d)`）
- **在线外参标定**：`doVECalibration_` 控制是否将 `MrMC`/`qCM` 作为状态估计

距离参数化实现：`raw/codes/rovio/include/rovio/FeatureDistance.hpp:L36-L48`

```cpp
enum Type {
    REGULAR,    // p = d
    INVERSE,    // p = 1/d
    LOG,        // p = ln(d)
    HYPERBOLIC  // p = asinh(d)
};
```

## IMU 预测 (ImuPrediction)

预测方程在 `ImuPrediction.hpp:L100-L156`：

**源码锚点**: `raw/codes/rovio/include/rovio/ImuPrediction.hpp:L100-L148`

```cpp
void evalPrediction(mtState& output, const mtState& state,
                    const mtNoise& noise, double dt) const {
  // IMU 角速度估计 = 测量 - 陀螺仪bias
  output.aux().MwWMest_ = meas_.gyr() - state.gyb();
  const V3D imuRor = output.aux().MwWMest_ + noise.att()/sqrt(dt);
  const V3D dOmega = dt * imuRor;
  QPD dQ = dQ.exponentialMap(dOmega);

  // 每个特征的状态传播
  for(unsigned int i=0;i<mtState::nMax_;i++){
    const int camID = state.CfP(i).camID_;
    // 相机速度 = qCM * (ω × MrMC - MvM)
    const V3D camVel = state.qCM(camID).rotate(
        V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
    // 距离变化率
    output.dep(i).p_ = oldD_.p_
        - dt*oldD_.getParameterDerivative()
          *oldC_.get_nor().getVec().transpose()*camVel + noise;
    // bearing vector 变化 - 利用 S^2 上测地线运动
    V3D dm = -dt*(gSM(n)*camVel/d + (I-n*n^T)*camRor) + noise;
    QPD qm = qm.exponentialMap(dm);
    output.CfP(i).set_nor(oldC_.get_nor().rotated(qm));
  }
  // 位姿传播
  output.WrWM() = state.WrWM() - dt*(qWM.rotate(MvM) - noise);
  output.MvM() = (I - skew(dOmega))*MvM
      - dt*(acc - acb + qWM.inverseRotate(g) - noise);
  output.qWM() = state.qWM() * dQ;
}
```

Jacobi 传播矩阵在 `jacPreviousState` 中，覆盖了特征距离参数、bearing vector 对位姿、速度、bias、外参的全雅可比链。

**源码锚点**: `raw/codes/rovio/include/rovio/ImuPrediction.hpp:L161-L240`

## 图像更新 (ImgUpdate)

更新流程的核心在 `ImgUpdate.hpp:L645-L700+`：

```
for each valid feature:
  1. 将特征投影到当前帧（通过 IMU 位姿传播）
  2. 检查特征是否在图像范围内
  3. 执行多级斑块对齐 (patch alignment)
  4. 计算光度创新项 y = b_red (直接法) 或 重投影误差 (间接法)
  5. EKF 更新：K = P H^T (H P H^T + R)^{-1}
```

**源码锚点**: `raw/codes/rovio/include/rovio/ImgUpdate.hpp:L645-L700`

EKF 创新项计算 (直接法)：

```cpp
void evalInnovation(mtInnovation& y, const mtState& state,
                    const mtNoise& noise) const {
  if(useDirectMethod_){
    // 计算简化线性对齐方程 A*x=b
    if(alignment_.getLinearAlignEquationsReduced(
        pyr_[activeCamID], *mpCurrentFeature_->mpMultilevelPatch_,
        featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)){
      y.pix() = b_red_ + noise.pix();  // 光度误差创新
    }
  } else {
    // 间接法：重投影误差
    y.pix() = (feaCoorMeas_[ID] - featureOutput_.c()) + noise.pix();
  }
}
```

## 多级斑块对齐 (MultilevelPatchAlignment)

ROVIO 使用图像金字塔上的多级斑块（multilevel patch）进行特征追踪。斑块对齐在金字塔的各级上执行，从粗到精。

**源码锚点**: `raw/codes/rovio/include/rovio/MultilevelPatchAlignment.hpp:L44-L120`

```cpp
template<int nLevels, int patch_size>
class MultilevelPatchAlignment {
  mutable Eigen::MatrixXf A_;  // 像素强度对像素坐标的雅可比
  mutable Eigen::MatrixXf b_;  // 强度误差

  bool getLinearAlignEquations(
      const ImagePyramid<nLevels>& pyr,
      const MultilevelPatch<nLevels,patch_size>& mp,
      const FeatureCoordinates& c, int l1, int l2,
      Eigen::MatrixXf& A, Eigen::MatrixXf& b);
};
```

对齐过程中使用：
- **Huber norm**（`huberNormThreshold_`）对异常像素强度进行鲁棒加权
- **Gaussian 加权**：中心像素权重高，边缘权重低
- **强度偏移/倾斜校正**：`useIntensityOffset_` 和 `useIntensitySqew_` 补偿曝光变化
- **梯度加权**：`gradientExponent_` 控制梯度方向残差权重

简化对齐方程 `getLinearAlignEquationsReduced()` 将 $(N \times 2)$ 雅可比矩阵通过 QR 分解压缩为 $(2 \times 2)$ 形式，使 EKF 更新仅需要 $2 \times 2$ 矩阵求逆。

## 特征管理

ROVIO 维护一个特征集合，涉及特征得分（Shi-Tomasi 得分）、tracking status、patch replacement 等。

- **特征检测**：在图像金字塔各级上用 FAST 检测，按 Shi-Tomasi 得分排序
- **特征替换**：tracking 质量下降时替换为新特征
- **跨相机 tracking**：`useCrossCameraMeasurements_` 允许 stereo 匹配
- **零速度更新**：静止时用 `ZeroVelocityUpdate` 约束漂移

特征质量评估包括：
1. 斑块是否在图像范围内
2. 对齐是否收敛
3. 平均光度误差是否超标 (`patchRejectionTh_`)
4. 特征位置是否具有判别力（discriminativity check，4个采样点）

**源码锚点**: `raw/codes/rovio/include/rovio/ImgUpdate.hpp:L477-L537`

## Pose / Velocity Update

除图像更新外，ROVIO 还支持：
- **PoseUpdate**：外部位姿测量（如 motion capture）直接约束
- **VelocityUpdate**：外部速度测量约束

三类更新通过 `LWF::FilterBase` 模板统一管理，`updateToUpdateMeasOnly_ = true` 表示 EKF 更新仅通过测量驱动。

## Agent 实现提示

### 适用场景

需要快速、轻量级的单目/双目 VIO，运行在计算受限平台（如无人机板载计算机）。直接法先天对特征缺失鲁棒，IMU 与视觉在 EKF 框架下紧耦合。适合纹理丰富、运动平滑的场景。

### 输入输出契约

- **输入**：IMU 角速度/加速度（100-200 Hz）、单目/双目图像（20-30 Hz）、相机内参、IMU-相机外参（可在线标定）
- **输出**：6-DOF 位姿（IMU 频率）、速度、IMU bias、特征深度估计、更新后协方差
- **创新项**：直接法用 $2 \times 1$ 光度误差 `b_red`，间接法用 $2 \times 1$ 重投影误差

### 实现骨架（伪代码）

```pseudo
function RovioFilter.run():
    init: state = {pos=0, vel=0, att=identity, bias=0, features=empty}
    cov = initial_covariance

    for each IMU measurement:
        state, cov = evalPrediction(state, cov, imu_dt)
        accumulate IMU buffer

    for each image frame:
        deskew IMU between frames
        detect_new_features(image_pyramid, FAST, ShiTomasi)
        for each tracked feature:
            project feature to current frame via IMU propagation
            multilevel_patch_align(feature.patch, current_pyr)
            compute reduced align equations -> A_red, b_red
            innovation = b_red  (direct) or reproj_error (indirect)
            H = jacobian of pixel intensity w.r.t. state
            K = cov * H^T * inv(H*cov*H^T + R)
            state += K * innovation
            cov = (I - K*H) * cov
        replace low_quality_features()
```

### 关键源码片段

**IMU 传播距离状态**（`ImuPrediction.hpp:L117`）：
```cpp
output.dep(i).p_ = oldD_.p_
    - dt*oldD_.getParameterDerivative()
      *oldC_.get_nor().getVec().transpose()*camVel
    + noise;
```

**直接法创新项**（`ImgUpdate.hpp:L422-L423`）：
```cpp
if(alignment_.getLinearAlignEquationsReduced(
    pyr_[activeCamID], *mpCurrentFeature_->mpMultilevelPatch_,
    featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)){
  y.pix() = b_red_ + noise.pix();
```

**雅可比 w.r.t. 状态**（`ImgUpdate.hpp:L555-L558`）：
```cpp
F = -A_red_ * c_J_ * featureOutputJac_
    .block<2,mtState::D_>(0,0);
```

### 实现注意事项

1. **距离参数化选择**：INVERSE 形式对数收敛性更好，适合远距离特征；REGULAR 形式更直观
2. **斑块大小**：模块参数为 `patchSize`（6-8 常用），必须在编译期确定
3. **特征数量上限**：`nMax` 控制 EKF 协方差矩阵大小，通常 20-30 个特征即可
4. **光度标定**：`useIntensityOffset_` 和 `useIntensitySqew_` 对曝光变化场景很重要
5. **多相机支持**：`nCam` 支持单目/双目/多目，每个相机有独立的外参和外参雅可比

### 源码检索锚点

| 模块 | 文件 | 行号 |
|------|------|------|
| 状态定义 | `FilterStates.hpp` | L110-L148 |
| IMU 预测 | `ImuPrediction.hpp` | L100-L156 |
| 预测雅可比 | `ImuPrediction.hpp` | L161-L240 |
| 图像更新主流程 | `ImgUpdate.hpp` | L645-L700 |
| 创新项计算 | `ImgUpdate.hpp` | L406-L440 |
| 雅可比计算 | `ImgUpdate.hpp` | L546-L568 |
| 特征候选生成 | `ImgUpdate.hpp` | L442-L475 |
| 外点检查 | `ImgUpdate.hpp` | L477-L537 |
| 距离参数化 | `FeatureDistance.hpp` | L36-L100 |
| 斑块对齐 | `MultilevelPatchAlignment.hpp` | L44-L120 |
| Filter 构造 | `RovioFilter.hpp` | L46-L90 |

## 相关页面

- [[方法-IESKF滤波器]]
- [[方法-VIO初始化方法]]
- [[方法-直接法SLAM族]]
- [[方法-Shi-Tomasi角点]]
- [[方法-MSCKF-VIO实现]]
