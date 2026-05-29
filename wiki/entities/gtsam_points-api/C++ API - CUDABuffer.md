---
type: entity
tags: [gtsam_points, C++ API, CUDA, CUDABuffer]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://koide3.github.io/gtsam_points/doc_cpp/index.html
  - raw/codes/gtsam_points
---

# gtsam_points::CUDABuffer

> **类** | 头文件: `cuda_buffer.hpp` | [在线文档](https://koide3.github.io/gtsam_points/doc_cpp/index.html)

Device buffer for asynchronous data transfer.

## 构造函数

```cpp
CUDABuffer(bool use_pinned_buffer = true)
```

## 公开方法

### 方法

```cpp
resize(size_t size, CUstream_st * stream)
```
Resize the buffer size. This method only expands the device/host buffers and doesn't shrink them when buffer_size < size.

```cpp
upload(CUstream_st * stream)
```
Upload data from the host pinned buffer to the device buffer.

```cpp
upload(size_t size, CUstream_st * stream)
```
Upload data from the host pinned buffer to the device buffer.

```cpp
upload(const void * buffer, size_t size, CUstream_st * stream)
```
Upload data to the device buffer. If size > buffer_size, the buffers will be resized before uploading.

```cpp
download(CUstream_st * stream)
```
Download data from the device buffer to the pinned host buffer.

```cpp
download(void * buffer, size_t size, CUstream_st * stream)
```
Download data from the device buffer.

```cpp
size_t size() const
```
Buffer size.

```cpp
void * host_buffer()
```
Pinned host buffer.

```cpp
void * device_buffer()
```
Pinned device buffer.

```cpp
upload(const T * buffer, size_t size, CUstream_st * stream)
```

```cpp
download(T * buffer, size_t size, CUstream_st * stream)
```

```cpp
T * host_buffer()
```

```cpp
T * device_buffer()
```

## 详细说明

To enable asynchronous upload/download, use_pinned_buffer needs to be true. To enable asynchronous upload/download, use_pinned_buffer needs to be true.

## 源码位置

- 远程: https://github.com/koide3/gtsam_points.git
- 本地快照: `raw/codes/gtsam_points` (v1.2.1)

## Agent 实现提示

### 适用场景

`CUDABuffer` 用于 GTSAM factor graph 优化流程中。

### 输入输出契约

参见上方 [公开方法](#公开方法) 的签名。

### 实现注意事项

- 所有 Lie group 类型使用右扰动（right perturbation）约定
- 使用 `OptionalJacobian` 参数可选的链式求导

## 相关页面

- [[方法-gtsam_points因子封装模式]]
- [[GTSAM C++ API 参考索引]]
