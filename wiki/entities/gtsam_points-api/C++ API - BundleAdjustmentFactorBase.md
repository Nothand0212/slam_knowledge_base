---
type: entity
tags: [gtsam_points, C++ API, Bundle Adjustment, BundleAdjustmentFactorBase]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::BundleAdjustmentFactorBase

> **类** | 头文件: `bundle_adjustment_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Base class of range-based bundle adjustment factors.

## 继承关系

- 继承自 `gtsam::NonlinearFactor`

## 公开方法

### 方法

```cpp
add(const Point3 & pt, const Key & key)
```
Assign a point to the factor.

```cpp
int num_points() const
```
Number of points assigned to the factor.

```cpp
set_scale(double scale)
```
Set a constant error scale to boost the weight of the constraint.

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< BundleAdjustmentFactorBase >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`BundleAdjustmentFactorBase` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
