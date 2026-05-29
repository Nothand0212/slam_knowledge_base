---
type: entity
tags: [GTSAM, C++ API, ISAM2, ISAM2Params]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::ISAM2Params

> **结构体** | 头文件: `ISAM2Params.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
ISAM2Params(OptimizationParams _optimizationParams, RelinearizationThreshold _relinearizeThreshold = 0.1, int _relinearizeSkip = 10, bool _enableRelinearization, bool _evaluateNonlinearError, Factorization _factorization, bool _cacheLinearizedFactors, const KeyFormatter & _keyFormatter, bool _enableDetailedResults)
```

## 公开方法

### 方法

```cpp
OptimizationParams getOptimizationParams() const
```

```cpp
RelinearizationThreshold getRelinearizeThreshold() const
```

```cpp
std::string getFactorization() const
```

```cpp
KeyFormatter getKeyFormatter() const
```

```cpp
setOptimizationParams(OptimizationParams optimizationParams)
```

```cpp
setRelinearizeThreshold(RelinearizationThreshold relinearizeThreshold)
```

```cpp
setFactorization(const std::string & factorization)
```

```cpp
setKeyFormatter(KeyFormatter keyFormatter)
```

```cpp
GaussianFactorGraph::Eliminate getEliminationFunction() const
```

```cpp
static Factorization factorizationTranslator(const std::string & str)
```

```cpp
static std::string factorizationTranslator(const Factorization & value)
```

### 方法

```cpp
print(const std::string & str = "") const
```
print iSAM2 parameters

## 类型别名

```cpp
using OptimizationParams = std::variant< ISAM2GaussNewtonParams, ISAM2DoglegParams, ISAM2DoglegLineSearchParams >
```
```cpp
using RelinearizationThreshold = std::variant< double, FastMap< char, Vector > >
```

## 公开成员变量

```cpp
OptimizationParams optimizationParams
```
```cpp
RelinearizationThreshold relinearizeThreshold
```
```cpp
int relinearizeSkip
```
```cpp
bool enableRelinearization
```
```cpp
bool evaluateNonlinearError
```
```cpp
Factorization factorization
```
```cpp
bool cacheLinearizedFactors
```
```cpp
KeyFormatter keyFormatter
```
```cpp
bool enableDetailedResults
```
```cpp
bool enablePartialRelinearizationCheck
```
```cpp
bool findUnusedFactorSlots
```
```cpp
bool enableAdaptiveReorder
```
```cpp
double adaptiveReorderThreshold
```

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`ISAM2Params` 用于 GTSAM factor graph 优化流程中。

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
