---
type: entity
tags: [GTSAM, C++ API, Navigation, PreintegrationParams]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::PreintegrationParams

> **结构体** | 头文件: `PreintegrationParams.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::PreintegratedRotationParams`

## 构造函数

```cpp
PreintegrationParams()
```
Default constructor for serialization only.

```cpp
PreintegrationParams(const Vector3 & n_gravity_)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "") const
```

```cpp
bool equals(const PreintegratedRotationParams & other, double tol) const
```

```cpp
setAccelerometerCovariance(const Matrix3 & cov)
```

```cpp
setIntegrationCovariance(const Matrix3 & cov)
```

```cpp
setUse2ndOrderCoriolis(bool flag)
```

```cpp
const Matrix3 & getAccelerometerCovariance() const
```

```cpp
const Matrix3 & getIntegrationCovariance() const
```

```cpp
const Vector3 & getGravity() const
```

```cpp
bool getUse2ndOrderCoriolis() const
```

### 静态方法

```cpp
static std::shared_ptr< PreintegrationParams > MakeSharedD(double g = 9.81)
```

```cpp
static std::shared_ptr< PreintegrationParams > MakeSharedU(double g = 9.81)
```

## 公开成员变量

```cpp
Matrix3 accelerometerCovariance
```
```cpp
Matrix3 integrationCovariance
```
```cpp
bool use2ndOrderCoriolis
```
```cpp
Vector3 n_gravity
```

## 详细说明

Parameters for pre-integration: Usage: Create just a single Params and pass a shared pointer to the constructor

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`PreintegrationParams` 用于 GTSAM factor graph 优化流程中。

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
