---
type: entity
tags: [gtsam_points, C++ API, Scan Matching Factors, IntegratedVGICPFactorGPU]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::IntegratedVGICPFactorGPU

> **类** | 头文件: `integrated_vgicp_factor_gpu.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

GPU-accelerated Voxelized GICP matching cost factor Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA2021 Koide et al., "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors", RA-L2021.

## 继承关系

- 继承自 `gtsam_points::NonlinearFactorGPU`

## 构造函数

```cpp
IntegratedVGICPFactorGPU(Key target_key, Key source_key, const GaussianVoxelMap::ConstPtr & target, const PointCloud::ConstPtr & source, CUstream_st * stream = nullptr, std::shared_ptr< TempBufferManager > temp_buffer = nullptr)
```
Create a binary VGICP_GPU factor between target and source poses.

```cpp
IntegratedVGICPFactorGPU(const Pose3 & fixed_target_pose, Key source_key, const GaussianVoxelMap::ConstPtr & target, const PointCloud::ConstPtr & source, CUstream_st * stream = nullptr, std::shared_ptr< TempBufferManager > temp_buffer = nullptr)
```
Create a unary VGICP_GPU factor between a fixed target pose and an active source pose.

```cpp
IntegratedVGICPFactorGPU(const IntegratedVGICPFactorGPU & )
```

## 公开方法

### 方法

```cpp
print(const std::string & s = "", const KeyFormatter & keyFormatter = gtsam::DefaultKeyFormatter) const
```
Print the factor information.

```cpp
size_t memory_usage() const
```
Calculate the CPU memory usage of this factor.

```cpp
size_t memory_usage_gpu() const
```
Calculate the GPU memory usage of this factor.

```cpp
set_enable_offloading(bool enable)
```
Enable or disable GPU memory offloading.

```cpp
set_enable_surface_validation(bool enable)
```
Enable or disable surface orientation validation for correspondence search.

```cpp
set_inlier_update_thresh(double trans, double angle)
```
Set the threshold values to trigger inlier points update. Setting larger values reduces GPU sync but may affect the registration accuracy.

```cpp
int num_inliers() const
```
Get the number of inlier points.

```cpp
double inlier_fraction() const
```
Get the fraction of inlier points.

```cpp
GaussianVoxelMapGPU::ConstPtr get_target() const
```
Get the target voxelmap.

```cpp
Eigen::Isometry3f get_fixed_target_pose() const
```
Get the pose of the fixed target. This function is only valid for unary factors.

```cpp
IntegratedVGICPFactorGPU & operator=(const IntegratedVGICPFactorGPU & )
```

```cpp
NonlinearFactor::shared_ptr clone() const
```

```cpp
size_t dim() const
```

```cpp
double error(const Values & values) const
```

```cpp
GaussianFactor::shared_ptr linearize(const Values & values) const
```

```cpp
size_t linearization_input_size() const
```
Size of data to be uploaded to the GPU before linearization.

```cpp
size_t linearization_output_size() const
```
Size of data to be downloaded from the GPU after linearization.

```cpp
size_t evaluation_input_size() const
```
Size of data to be uploaded to the GPU before cost evaluation.

```cpp
size_t evaluation_output_size() const
```
Size of data to be downloaded from the GPU after cost evaluation.

```cpp
set_linearization_point(const Values & values, void * lin_input_cpu)
```
Write linearization input data to the upload buffer.

```cpp
issue_linearize(const void * lin_input_cpu, const void * lin_input_gpu, void * lin_output_gpu)
```
Issue linearization task.

```cpp
store_linearized(const void * lin_output_cpu)
```
Read linearization output data from the download buffer.

```cpp
set_evaluation_point(const Values & values, void * eval_input_cpu)
```
Write cost evaluation input data to the upload buffer.

```cpp
issue_compute_error(const void * lin_input_cpu, const void * eval_input_cpu, const void * lin_input_gpu, const void * eval_input_gpu, void * eval_output_gpu)
```
Issue cost evaluation task.

```cpp
store_computed_error(const void * eval_output_cpu)
```
Read cost evaluation output data from the download buffer.

```cpp
sync()
```
Perform CPU-GPU synchronization and wait for the task.

## 类型别名

```cpp
using shared_ptr = gtsam_points::shared_ptr< IntegratedVGICPFactorGPU >
```

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`IntegratedVGICPFactorGPU` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
