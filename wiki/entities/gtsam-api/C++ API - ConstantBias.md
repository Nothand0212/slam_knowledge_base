---
type: entity
tags: [GTSAM, C++ API, Navigation, ConstantBias]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::ConstantBias

> **类** | 头文件: `ImuBias.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
ConstantBias()
```

```cpp
ConstantBias(const Vector3 & biasAcc, const Vector3 & biasGyro)
```

```cpp
ConstantBias(const Vector6 & v)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "") const
```
print with optional string

```cpp
bool equals(const ConstantBias & expected, double tol = 1e-5) const
```

```cpp
ConstantBias operator-() const
```

```cpp
ConstantBias operator+(const Vector6 & v) const
```

```cpp
ConstantBias operator+(const ConstantBias & b) const
```

```cpp
ConstantBias operator-(const ConstantBias & b) const
```

```cpp
static ConstantBias Identity()
```

```cpp
ConstantBias retract(const Vector6 & v) const
```
The retract function.

```cpp
Vector6 localCoordinates(const ConstantBias & other) const
```
The local coordinates function.

### 方法

```cpp
Vector6 vector() const
```

```cpp
const Vector3 & accelerometer() const
```

```cpp
const Vector3 & gyroscope() const
```

```cpp
Vector3 correctAccelerometer(const Vector3 & measurement, OptionalJacobian< 3, 6 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

```cpp
Vector3 correctGyroscope(const Vector3 & measurement, OptionalJacobian< 3, 6 > H1 = {}, OptionalJacobian< 3, 3 > H2 = {}) const
```

## 公开成员变量

```cpp
const size_t dimension
```

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`ConstantBias` 用于 GTSAM factor graph 优化流程中。

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
