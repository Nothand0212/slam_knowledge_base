---
type: entity
tags: [GTSAM, C++ API, Navigation, PreintegratedImuMeasurementsT]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::PreintegratedImuMeasurementsT

> **类** | 头文件: `ImuFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `PreintegrationType`

## 构造函数

```cpp
PreintegratedImuMeasurementsT()
```
Default constructor for serialization and wrappers.

```cpp
PreintegratedImuMeasurementsT(const std::shared_ptr< PreintegrationParams > & p, const imuBias::ConstantBias & biasHat)
```

```cpp
PreintegratedImuMeasurementsT(const PreintegrationType & base, const Matrix9 & preintMeasCov)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "Preintegrated ) const
```
print

```cpp
bool equals(const PreintegratedImuMeasurementsT< PreintegrationType > & expected, double tol = 1e-9) const
```
equals

```cpp
resetIntegration()
```
Re-initialize PreintegratedImuMeasurements.

```cpp
integrateMeasurement(const Vector3 & measuredAcc, const Vector3 & measuredOmega, const double dt)
```

```cpp
integrateMeasurements(const Matrix & measuredAccs, const Matrix & measuredOmegas, const Matrix & dts)
```
Add multiple measurements, in matrix columns.

```cpp
Matrix preintMeasCov() const
```
Return pre-integrated measurement covariance.

```cpp
mergeWith(const PreintegratedImuMeasurementsT< TangentPreintegration > & pim12, Matrix9 * H1, Matrix9 * H2)
```

## 详细说明

PreintegratedImuMeasurements accumulates (integrates) the IMU measurements (rotation rates and accelerations) and the corresponding covariance matrix. The measurements are then used to build the Preintegrated IMU factor. Integration is done incrementally (ideally, one integrates the measurement as soon as it is received from the IMU) so as to avoid costly integration at time of factor construction.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`PreintegratedImuMeasurementsT` 用于 GTSAM factor graph 优化流程中。

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
