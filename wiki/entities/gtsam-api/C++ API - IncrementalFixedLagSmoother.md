---
type: entity
tags: [GTSAM, C++ API, ISAM2, IncrementalFixedLagSmoother]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::IncrementalFixedLagSmoother

> **类** | 头文件: `IncrementalFixedLagSmoother.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::FixedLagSmoother`

## 构造函数

```cpp
IncrementalFixedLagSmoother(double smootherLag = 0.0, const ISAM2Params & parameters)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "IncrementalFixedLagSmoother:\n", const KeyFormatter & keyFormatter) const
```

```cpp
bool equals(const FixedLagSmoother & rhs, double tol = 1e-9) const
```

```cpp
Result update(const NonlinearFactorGraph & newFactors, const Values & newTheta, const KeyTimestampMap & timestamps, const FactorIndices & factorsToRemove)
```

```cpp
Values calculateEstimate() const
```

```cpp
VALUE calculateEstimate(Key key) const
```

```cpp
const ISAM2Params & params() const
```

```cpp
const NonlinearFactorGraph & getFactors() const
```

```cpp
const Values & getLinearizationPoint() const
```

```cpp
const VectorValues & getDelta() const
```

```cpp
Matrix marginalCovariance(Key key) const
```
Calculate marginal covariance on given variable.

```cpp
const ISAM2Result & getISAM2Result() const
```
Get results of latest isam2 update.

```cpp
const ISAM2 & getISAM2() const
```
Get the iSAM2 object which is used for the inference internally.

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< IncrementalFixedLagSmoother >
```

## 详细说明

This is a base class for the various HMF2 implementations. The HMF2 eliminates the factor graph such that the active states are placed in/near the root. This base class implements a function to calculate the ordering, and an update function to incorporate new factors into the HMF.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`IncrementalFixedLagSmoother` 用于 GTSAM factor graph 优化流程中。

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
