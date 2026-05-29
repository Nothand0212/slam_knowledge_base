---
type: entity
tags: [GTSAM, C++ API, Navigation, GPSFactor2]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::GPSFactor2

> **类** | 头文件: `GPSFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NoiseModelFactorT< NavState >`

## 构造函数

```cpp
GPSFactor2()
```
default constructor - only use for serialization

```cpp
GPSFactor2(Key key, const Point3 & gpsIn, const SharedNoiseModel & model)
```

## 公开方法

### 方法

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```
print

```cpp
bool equals(const NonlinearFactor & expected, double tol = 1e-9) const
```
equals

```cpp
Vector evaluateError(const NavState & nTb, OptionalMatrixType H) const
```
vector of errors

```cpp
const Point3 & measurementIn() const
```
return the measurement, in the navigation frame

```cpp
OutputVec evaluateError(const ValueTypes &... x, OptionalMatrixTypeT< ValueTypes >... H) const
```

```cpp
Vector evaluateError(const ValueTypes &... x, MatrixTypeT< ValueTypes > &... H) const
```

```cpp
Vector evaluateError(const ValueTypes &... x) const
```

```cpp
AreAllMatrixRefs< Vector, OptionalJacArgs... > evaluateError(const ValueTypes &... x, OptionalJacArgs &&... H) const
```

```cpp
AreAllMatrixPtrs< Vector, OptionalJacArgs... > evaluateError(const ValueTypes &... x, OptionalJacArgs &&... H) const
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< GPSFactor2 >
```
```cpp
using This = GPSFactor2
```

## 详细说明

Version of GPSFactor for NavState, assuming zero lever arm between body frame and GPS. If there exists a non-zero lever arm between body frame and GPS antenna, instead use GPSFactor2Arm.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`GPSFactor2` 用于 GTSAM factor graph 优化流程中。

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
