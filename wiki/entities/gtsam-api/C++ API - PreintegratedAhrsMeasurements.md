---
type: entity
tags: [GTSAM, C++ API, Navigation, PreintegratedAhrsMeasurements]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::PreintegratedAhrsMeasurements

> **类** | 头文件: `AHRSFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::PreintegratedRotation`

## 构造函数

```cpp
PreintegratedAhrsMeasurements()
```
Default constructor, only for serialization and wrappers.

```cpp
PreintegratedAhrsMeasurements(const std::shared_ptr< Params > & p, const Vector3 & biasHat)
```

```cpp
PreintegratedAhrsMeasurements(const std::shared_ptr< Params > & p, const Vector3 & bias_hat, double deltaTij, const Rot3 & deltaRij, const Matrix3 & delRdelBiasOmega, const Matrix3 & preint_meas_cov)
```

```cpp
PreintegratedAhrsMeasurements(const Vector3 & biasHat, const Matrix3 & measuredOmegaCovariance)
```

## 公开方法

### 方法

```cpp
Params & p() const
```

```cpp
const Vector3 & biasHat() const
```

```cpp
const Matrix3 & preintMeasCov() const
```

```cpp
print(const std::string & s = "Preintegrated ) const
```
print

```cpp
bool equals(const PreintegratedAhrsMeasurements & expected, double tol = 1e-9) const
```
equals

```cpp
resetIntegration()
```
Reset integrated quantities to zero.

```cpp
integrateMeasurement(const Vector3 & measuredOmega, double deltaT)
```

```cpp
Rot3 predict(const Rot3 & Ri, const Vector3 & bias, OptionalJacobian< 3, 3 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

```cpp
Vector3 computeError(const Rot3 & Ri, const Rot3 & Rj, const Vector3 & bias, OptionalJacobian< 3, 3 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}, OptionalJacobian< 3, 3 > H3 = {}) const
```

## 详细说明

PreintegratedAHRSMeasurements accumulates (integrates) the gyroscope measurements (rotation rates) and the corresponding covariance matrix. Can be built incrementally so as to avoid costly integration at time of factor construction. The preintegrated rotation is updated incrementally with each gyroscope measurement. Given a gyroscope measurement $ \omega_k $ at time $ t_k $, the preintegrated rotation $ \Delta R_{ij} $ from time $ t_i $ to $ t_j $ is the product of many small rotations:  \[
\Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\omega_k - b_g) \Delta t)
\] where $ b_g $ is the gyroscope bias, and $ \text{Exp}(\cdot) $ is the exponential map from $ \mathbb{R}^3 $ to SO(3). This class also propagates the covariance of the preintegrated rotation.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`PreintegratedAhrsMeasurements` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-GTSAM-API族]]
- [[GTSAM Geometry API]]
- [[GTSAM API 使用索引]]
- [[GTSAM 4.3a1 使用指南]]
