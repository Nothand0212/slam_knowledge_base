---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, NonlinearEquality]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::NonlinearEquality

> **类** | 头文件: `NonlinearEquality.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `NonlinearEqualityConstraint`

## 构造函数

```cpp
NonlinearEquality(Key j, const T & feasible, const CompareFunction & _compare = std::bind()
```

```cpp
NonlinearEquality(Key j, const T & feasible, double error_gain, const CompareFunction & _compare = std::bind()
```

```cpp
NonlinearEquality()
```
Default constructor - only for serialization.

## 公开方法

### 方法

```cpp
Key key() const
```

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter) const
```

```cpp
bool equals(const NonlinearFactor & f, double tol = 1e-9) const
```

```cpp
double error(const Values & c) const
```
Actual error function calculation.

```cpp
bool isHardConstraint() const
```
Whether this constraint should be treated as hard.

```cpp
Vector evaluateError(const T & xj, OptionalMatrixType H) const
```
Error function.

```cpp
Vector unwhitenedError(const Values & x, OptionalMatrixVecType H) const
```

```cpp
GaussianFactor::shared_ptr linearize(const Values & x) const
```
Linearize is over-written, because base linearization tries to whiten.

```cpp
NonlinearFactor::shared_ptr clone() const
```

## 类型别名

```cpp
using T = VALUE
```
```cpp
using CompareFunction = std::function< bool(const T &, const T &)>
```

## 公开成员变量

```cpp
CompareFunction compare_
```

## 详细说明

Equality constraint that pins a single variable to a constant value. Behavior is controlled by allow_error_:
Exact mode (default): throws at linearization if the current value is not equal to the feasible point (within compare_), and returns infinite error.
Allow-error mode: returns a smooth squared error scaled by error_gain_. Exact mode (default): throws at linearization if the current value is not equal to the feasible point (within compare_), and returns infinite error. Allow-error mode: returns a smooth squared error scaled by error_gain_.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`NonlinearEquality` 用于 GTSAM factor graph 优化流程中。

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
