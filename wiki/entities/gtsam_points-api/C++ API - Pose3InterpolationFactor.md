---
type: entity
tags: [gtsam_points, C++ API, Point Cloud & Trajectory, Pose3InterpolationFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::Pose3InterpolationFactor

> **类** | 头文件: `pose3_interpolation_factor.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Factor(xi, xj, xk) s.t. xk = Slerp(xi, xj, t)

## 继承关系

- 继承自 `gtsam::NoiseModelFactor3< gtsam::Pose3, gtsam::Pose3, gtsam::Pose3 >`

## 构造函数

```cpp
Pose3InterpolationFactor(Key xi, Key xj, Key xk, const double t, const SharedNoiseModel & noise_model)
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter = gtsam::DefaultKeyFormatter) const
```

```cpp
Vector evaluateError(const Pose3 & xi, const Pose3 & xj, const Pose3 & xk, OptionalMatrixType H_xi = NoneValue, OptionalMatrixType H_xj = NoneValue, OptionalMatrixType H_xk = NoneValue) const
```

### 静态方法

```cpp
static Pose3 initial_guess(const Pose3 & xi, const Pose3 & xj, const double t)
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`Pose3InterpolationFactor` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
