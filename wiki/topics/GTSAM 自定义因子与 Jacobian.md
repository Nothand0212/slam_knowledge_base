---
tags: [GTSAM, CustomFactor, Jacobian, Python]
created: 2026-04-27
updated: 2026-04-27
sources:
  - wiki/sources/2026-04-27-gtsam-4.3a1-docs.md
---

# GTSAM 自定义因子与 Jacobian

> Python 中用 `CustomFactor` 写自定义 measurement residual；关键风险是 Jacobian convention、矩阵内存布局和 callback 性能。

## 官方入口

- CustomFactor：https://borglab.github.io/gtsam/customfactor
- CustomFactor example：https://borglab.github.io/gtsam/customfactorexample
- CustomFactor localization example：https://borglab.github.io/gtsam/customfactorlocalizationexample

## Python 函数签名

```python
def error_func(this: gtsam.CustomFactor, values: gtsam.Values, H: list[np.ndarray]) -> np.ndarray:
    ...
```

参数含义：

- `this`：当前 factor，可用 `this.keys()` 获取变量 keys。
- `values`：当前变量估计。
- `H`：Jacobian 输出列表；如果 `H is None`，当前调用不需要 Jacobian。
- 返回值：1D `np.ndarray` residual。

创建 factor：

```python
noise = gtsam.noiseModel.Isotropic.Sigma(dim, sigma)
factor = gtsam.CustomFactor(noise, [key0, key1], error_func)
graph.add(factor)
```

## Jacobian 约定

官方文档特别提醒：GTSAM 对 Lie group 变量使用右侧 exponential map 更新，形式类似：

```text
x_new = x * Exp(delta)
```

因此 Jacobian 是相对于右扰动的线性化，不要直接套用左扰动或欧式加法推导。

## Fortran order 要求

Python callback 中写入 `H[i]` 时，矩阵应使用 column-major/Fortran order，保证和 C++ interop：

```python
H[0] = np.zeros((residual_dim, variable_dim), order="F")
```

官方示例里也建议数值 Jacobian 验证时使用 `order='F'`。

## Numerical derivative 验证

```python
from gtsam.utils.numerical_derivative import numericalDerivative21, numericalDerivative22

def f(T1, T2):
    v = gtsam.Values()
    v.insert(key0, T1)
    v.insert(key1, T2)
    return error_func(factor, v)

num_H0 = numericalDerivative21(f, values.atPose2(key0), values.atPose2(key1))
num_H1 = numericalDerivative22(f, values.atPose2(key0), values.atPose2(key1))
np.testing.assert_allclose(H[0], num_H0, rtol=1e-5, atol=1e-8)
np.testing.assert_allclose(H[1], num_H1, rtol=1e-5, atol=1e-8)
```

## 性能注意

- Python `CustomFactor` 每次 linearization 会从 C++ 调 Python callback，有额外开销。
- pybind11 会获取 Python GIL，因此 Python callback factor 不能并行 evaluation。
- 如果 measurement 很多，优先考虑 batch 一个 factor 处理多条测量，减少 callback 次数。
- 性能关键路径最终考虑 C++ factor。

## 相关页面

- [[GTSAM API 使用索引]]
- [[GTSAM 因子图工作流]]
