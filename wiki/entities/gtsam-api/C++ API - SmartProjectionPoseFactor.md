---
type: entity
tags: [GTSAM, C++ API, SLAM_Factors, SmartProjectionPoseFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::SmartProjectionPoseFactor

> **类** | 头文件: `SmartProjectionPoseFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::SmartProjectionFactor< PinholePose< CALIBRATION > >`

## 构造函数

```cpp
SmartProjectionPoseFactor()
```

```cpp
SmartProjectionPoseFactor(const SharedNoiseModel & sharedNoiseModel, const std::shared_ptr< CALIBRATION > K, const SmartProjectionParams & params)
```

```cpp
SmartProjectionPoseFactor(const SharedNoiseModel & sharedNoiseModel, const std::shared_ptr< CALIBRATION > K, const std::optional< Pose3 > body_P_sensor, const SmartProjectionParams & params)
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
double error(const Values & values) const
```

```cpp
const std::shared_ptr< CALIBRATION > calibration() const
```

```cpp
Base::Cameras cameras(const Values & values) const
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< This >
```

## 详细说明

If you are using the factor, please cite: L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating conditionally independent sets in factor graphs: a unifying perspective based on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014. This factor assumes that camera calibration is fixed, and that the calibration is the same for all cameras involved in this factor. The factor only constrains poses (variable dimension is 6). This factor requires that values contains the involved poses (Pose3). If the calibration should be optimized, as well, use SmartProjectionFactor instead!

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`SmartProjectionPoseFactor` 用于 GTSAM factor graph 优化流程中。

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
