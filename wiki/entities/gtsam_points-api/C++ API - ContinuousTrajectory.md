---
type: entity
tags: [gtsam_points, C++ API, Point Cloud & Trajectory, ContinuousTrajectory]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::ContinuousTrajectory

> **类** | 头文件: `continuous_trajectory.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Continuous trajectory class for offline batch optimization.

## 构造函数

```cpp
ContinuousTrajectory(char symbol, double start_time, double end_time, double knot_interval)
```
Construct a continuous trajectory instance.

## 公开方法

### 方法

```cpp
double knot_stamp(int i) const
```
Time of a spline knot.

```cpp
int knot_id(double t) const
```
Key knot ID for a given time.

```cpp
int knot_max_id() const
```
Number of spline knots.

```cpp
Pose3_ pose(double t, const Double_ & t_)
```
Get an expression of the interpolated time at t.

```cpp
Pose3 pose(const Values & values, double t)
```
Calculate the interpolated time at t.

```cpp
Vector6_ imu(double t, const Double_ & t_, const Eigen::Vector3d & g = Eigen::Vector3d(0.0, 0.0, 9.80665))
```
Get an expression of the linear acceleration and angular velocity at t.

```cpp
Vector6 imu(const Values & values, double t, const Eigen::Vector3d & g = Eigen::Vector3d(0.0, 0.0, 9.80665))
```
Calculate the linear acceleration and angular velocity at t.

```cpp
Values fit_knots(const std::vector< double > & stamps, const std::vector< Pose3 > & poses, double smoothness, const LevenbergMarquardtParams & lm_params) const
```
Optimize spline knots to fit the interpolated trajectory to a set of poses.

```cpp
Values fit_knots(const std::vector< double > & stamps, const std::vector< Pose3 > & poses, double smoothness = 1e-3, bool verbose = false) const
```

## 公开成员变量

```cpp
const char symbol
```
```cpp
const double start_time
```
```cpp
const double end_time
```
```cpp
const double knot_interval
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`ContinuousTrajectory` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
