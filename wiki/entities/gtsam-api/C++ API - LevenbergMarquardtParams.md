---
type: entity
tags: [GTSAM, C++ API, Optimization, LevenbergMarquardtParams]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::LevenbergMarquardtParams

> **类** | 头文件: `LevenbergMarquardtParams.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::NonlinearOptimizerParams`

## 构造函数

```cpp
LevenbergMarquardtParams()
```

## 公开方法

### 方法

```cpp
bool getDiagonalDamping() const
```

```cpp
double getlambdaFactor() const
```

```cpp
double getlambdaInitial() const
```

```cpp
double getlambdaLowerBound() const
```

```cpp
double getlambdaUpperBound() const
```

```cpp
bool getUseFixedLambdaFactor()
```

```cpp
std::string getLogFile() const
```

```cpp
std::string getVerbosityLM() const
```

```cpp
setDiagonalDamping(bool flag)
```

```cpp
setlambdaFactor(double value)
```

```cpp
setlambdaInitial(double value)
```

```cpp
setlambdaLowerBound(double value)
```

```cpp
setlambdaUpperBound(double value)
```

```cpp
setUseFixedLambdaFactor(bool flag)
```

```cpp
setLogFile(const std::string & s)
```

```cpp
setVerbosityLM(const std::string & s)
```

```cpp
std::shared_ptr< NonlinearOptimizerParams > clone() const
```

### 静态方法

```cpp
static VerbosityLM verbosityLMTranslator(const std::string & s)
```

```cpp
static std::string verbosityLMTranslator(VerbosityLM value)
```

```cpp
static SetLegacyDefaults(LevenbergMarquardtParams * p)
```

```cpp
static SetCeresDefaults(LevenbergMarquardtParams * p)
```

```cpp
static LevenbergMarquardtParams LegacyDefaults()
```

```cpp
static LevenbergMarquardtParams CeresDefaults()
```

```cpp
static LevenbergMarquardtParams EnsureHasOrdering(LevenbergMarquardtParams params, const NonlinearFactorGraph & graph)
```

```cpp
static LevenbergMarquardtParams ReplaceOrdering(LevenbergMarquardtParams params, const Ordering & ord)
```

### 方法

```cpp
print(const std::string & str = "") const
```

## 类型别名

```cpp
using OptimizerType = LevenbergMarquardtOptimizer
```

## 公开成员变量

```cpp
double lambdaInitial
```
```cpp
double lambdaFactor
```
```cpp
double lambdaUpperBound
```
```cpp
double lambdaLowerBound
```
```cpp
VerbosityLM verbosityLM
```
```cpp
double minModelFidelity
```
```cpp
std::string logFile
```
```cpp
bool useFixedLambdaFactor
```
```cpp
LMDampingParams dampingParams
```

## 详细说明

Parameters for Levenberg-Marquardt optimization. Note that this parameters class inherits from NonlinearOptimizerParams, which specifies the parameters common to all nonlinear optimization algorithms. This class also contains all of those parameters.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`LevenbergMarquardtParams` 用于 GTSAM factor graph 优化流程中。

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
