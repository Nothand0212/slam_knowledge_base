---
type: entity
tags: [GTSAM, C++ API, Optimization, IterativeOptimizationParameters]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::IterativeOptimizationParameters

> **类** | 头文件: `IterativeSolver.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
IterativeOptimizationParameters(Verbosity v)
```

## 公开方法

### 方法

```cpp
Verbosity verbosity() const
```

```cpp
std::string getVerbosity() const
```

```cpp
setVerbosity(const std::string & s)
```

```cpp
print() const
```

```cpp
print(std::ostream & os) const
```

```cpp
bool equals(const IterativeOptimizationParameters & other, double tol = 1e-9) const
```

### 静态方法

```cpp
static Verbosity verbosityTranslator(const std::string & s)
```

```cpp
static std::string verbosityTranslator(Verbosity v)
```

## 类型别名

```cpp
using shared_ptr = std::shared_ptr< IterativeOptimizationParameters >
```

## 详细说明

parameters for iterative linear solvers

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`IterativeOptimizationParameters` 用于 GTSAM factor graph 优化流程中。

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
