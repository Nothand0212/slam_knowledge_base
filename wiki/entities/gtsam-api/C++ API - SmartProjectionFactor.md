---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, SmartProjectionFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::SmartProjectionFactor

> **类** | 头文件: `SmartProjectionFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::SmartFactorBase< CAMERA >`

## 构造函数

```cpp
SmartProjectionFactor()
```

```cpp
SmartProjectionFactor(const SharedNoiseModel & sharedNoiseModel, const SmartProjectionParams & params)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```

```cpp
bool equals(const NonlinearFactor & p, double tol = 1e-9) const
```
equals

```cpp
bool decideIfTriangulate(const Cameras & cameras) const
```
Check if the new linearization point is the same as the one used for previous triangulation.

```cpp
TriangulationResult triangulateSafe(const Cameras & cameras) const
```
Call gtsam::triangulateSafe iff we need to re-triangulate.

```cpp
bool triangulateForLinearize(const Cameras & cameras) const
```
Possibly re-triangulate before calculating Jacobians.

```cpp
std::shared_ptr< RegularHessianFactor< Base::Dim > > createHessianFactor(const Cameras & cameras, const double _lambda = 0.0, bool diagonalDamping) const
```
Create a Hessianfactor that is an approximation of error(p).

```cpp
std::shared_ptr< RegularImplicitSchurFactor< CAMERA > > createRegularImplicitSchurFactor(const Cameras & cameras, double _lambda) const
```

```cpp
std::shared_ptr< JacobianFactorQ< Base::Dim, 2 > > createJacobianQFactor(const Cameras & cameras, double _lambda) const
```
Create JacobianFactorQ factor.

```cpp
std::shared_ptr< JacobianFactorQ< Base::Dim, 2 > > createJacobianQFactor(const Values & values, double _lambda) const
```
Create JacobianFactorQ factor, takes values.

```cpp
std::shared_ptr< JacobianFactor > createJacobianSVDFactor(const Cameras & cameras, double _lambda) const
```
Different (faster) way to compute a JacobianFactorSVD factor.

```cpp
std::shared_ptr< RegularHessianFactor< Base::Dim > > linearizeToHessian(const Values & values, double _lambda = 0.0) const
```
Linearize to a Hessianfactor.

```cpp
std::shared_ptr< RegularImplicitSchurFactor< CAMERA > > linearizeToImplicit(const Values & values, double _lambda = 0.0) const
```
Linearize to an Implicit Schur factor.

```cpp
std::shared_ptr< JacobianFactorQ< Base::Dim, 2 > > linearizeToJacobian(const Values & values, double _lambda = 0.0) const
```
Linearize to a JacobianfactorQ.

```cpp
std::shared_ptr< GaussianFactor > linearizeDamped(const Cameras & cameras, const double _lambda = 0.0) const
```

```cpp
std::shared_ptr< GaussianFactor > linearizeDamped(const Values & values, const double _lambda = 0.0) const
```

```cpp
std::shared_ptr< GaussianFactor > linearize(const Values & values) const
```
linearize

```cpp
bool triangulateAndComputeE(Matrix & E, const Cameras & cameras) const
```

```cpp
bool triangulateAndComputeE(Matrix & E, const Values & values) const
```

```cpp
computeJacobiansWithTriangulatedPoint(typename Base::FBlocks & Fs, Matrix & E, Vector & b, const Cameras & cameras) const
```

```cpp
bool triangulateAndComputeJacobians(typename Base::FBlocks & Fs, Matrix & E, Vector & b, const Values & values) const
```
Version that takes values, and creates the point.

```cpp
bool triangulateAndComputeJacobiansSVD(typename Base::FBlocks & Fs, Matrix & Enull, Vector & b, const Values & values) const
```
takes values

```cpp
Vector reprojectionErrorAfterTriangulation(const Values & values) const
```
Calculate vector of re-projection errors, before applying noise model.

```cpp
double totalReprojectionError(const Cameras & cameras, std::optional< Point3 > externalPoint = {}) const
```

```cpp
double error(const Values & values) const
```
Calculate total reprojection error.

```cpp
TriangulationResult point() const
```

```cpp
TriangulationResult point(const Values & values) const
```

```cpp
bool isValid() const
```
Is result valid?

```cpp
bool isDegenerate() const
```

```cpp
bool isPointBehindCamera() const
```

```cpp
bool isOutlier() const
```

```cpp
bool isFarPoint() const
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< This >
```
```cpp
using Camera = CAMERA
```
```cpp
using Cameras = CameraSet< CAMERA >
```

## 详细说明

SmartProjectionFactor: triangulates point and keeps an estimate of it around. This factor operates with monocular cameras, where a camera is expected to behave like PinholeCamera or PinholePose. This factor is intended to be used directly with PinholeCamera, which optimizes the camera pose and calibration. This also requires that values contains the involved cameras (instead of poses and calibrations separately). If the calibration is fixed use SmartProjectionPoseFactor instead!

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`SmartProjectionFactor` 用于 GTSAM factor graph 优化流程中。

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
