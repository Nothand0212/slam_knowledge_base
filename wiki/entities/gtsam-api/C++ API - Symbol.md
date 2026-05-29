---
type: entity
tags: [GTSAM, C++ API, FactorGraph, Symbol]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::Symbol

> **类** | 头文件: `Symbol.h` | [在线文档](https://gtsam.org/doxygen/)

## 构造函数

```cpp
Symbol()
```

```cpp
Symbol(const Symbol & key)
```

```cpp
Symbol(unsigned char c, std::uint64_t j)
```

```cpp
Symbol(Key key)
```

## 公开方法

### 方法

```cpp
Key key() const
```

```cpp
operator Key() const
```

```cpp
print(const std::string & s = "") const
```
Print.

```cpp
bool equals(const Symbol & expected, double tol = 0.0) const
```
Check equality.

```cpp
unsigned char chr() const
```

```cpp
std::uint64_t index() const
```

```cpp
operator std::string() const
```

```cpp
std::string string() const
```
Return string representation of the key.

```cpp
bool operator<(const Symbol & comp) const
```

```cpp
bool operator==(const Symbol & comp) const
```

```cpp
bool operator==(Key comp) const
```

```cpp
bool operator!=(const Symbol & comp) const
```

```cpp
bool operator!=(Key comp) const
```

### 静态方法

```cpp
static std::function< bool(Key)> ChrTest(unsigned char c)
```

## 详细说明

Character and index key used to refer to variables. Will simply cast to a Key, i.e., a large integer. Keys are used to retrieve values from Values, specify what variables factors depend on, etc.

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`Symbol` 用于 GTSAM factor graph 优化流程中。

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
