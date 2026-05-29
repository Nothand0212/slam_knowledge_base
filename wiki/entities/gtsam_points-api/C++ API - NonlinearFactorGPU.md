---
type: entity
tags: [gtsam_points, C++ API, CUDA, NonlinearFactorGPU]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::NonlinearFactorGPU

> **类** | 头文件: `nonlinear_factor_gpu.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Base class for GPU-based nonlinear factors.

## 继承关系

- 继承自 `gtsam::NonlinearFactor`

## 构造函数

```cpp
NonlinearFactorGPU(const CONTAINER & keys)
```

## 公开方法

### 方法

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
using shared_ptr = gtsam_points::shared_ptr< NonlinearFactorGPU >
```

## 详细说明

To efficiently perform linearization (and cost evaluation) on a GPU, we issue all linearization tasks of GPU-based factors and copy all required data for linearization (e.g., current estimate) at once.

To allow gtsam_points::NonlinearFactorSetGPU to manage linearization, you need to implement the following methods:
linearization_(input|output)_size() : Define the size of input and result data for linearization
set_linearization_point() : Write the data to be uploaded to the GPU before linearization
issue_linearization() : Issue the linearization task
sync() : Perform CPU-GPU synchronization to wait for linearization completion
store_linearized() : Read back the data from the download buffer after linearization To efficiently perform linearization (and cost evaluation) on a GPU, we issue all linearization tasks of GPU-based factors and copy all required data for linearization (e.g., current estimate) at once. linearization_(input|output)_size() : Define the size of input and result data for linearization set_linearization_point() : Write the data to be uploaded to the GPU before linearization issue_linearization() : Issue the linearization task sync() : Perform CPU-GPU synchronization to wait for linearization completion store_linearized() : Read back the data from the download buffer after linearization For example, implementation for the linearization of the standard ICP factor would be as follows:
linearization_input_size() : sizeof(Eigen::Isometry3f) Current estimate of T_target_source
linearization_output_size() : sizeof(LinearizedSystem6) Linearized Hessian factor
set_linearization_point() : Write the current T_target_source to the lin_input_cpu
issue_linearization() : Issue the ICP computation task
sync() : Wait for the ICP computation task
store_linearized() : Read the linearized factor from lin_output_cpu linearization_input_size() : sizeof(Eigen::Isometry3f) Current estimate of T_target_source linearization_output_size() : sizeof(LinearizedSystem6) Linearized Hessian factor set_linearization_point() : Write the current T_target_source to the lin_input_cpu issue_linearization() : Issue the ICP computation task sync() : Wait for the ICP computation task store_linearized() : Read the linearized factor from lin_output_cpu Optimizers in gtsam_points calls NonlinearFactorSetGPU's linearization routine before calling linearize() of each factor. You should thus store the linearized factor to a temporary member and just return it when linearize() is called.

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`NonlinearFactorGPU` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
