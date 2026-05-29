---
type: entity
tags: [GTSAM, C++ API, Inference, JacobianFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::JacobianFactor

> **类** | 头文件: `JacobianFactor.h` | [在线文档](https://gtsam.org/doxygen/)

## 继承关系

- 继承自 `gtsam::GaussianFactor`

## 构造函数

```cpp
JacobianFactor(const GaussianFactor & gf)
```

```cpp
JacobianFactor(const JacobianFactor & jf)
```

```cpp
JacobianFactor(const HessianFactor & hf)
```

```cpp
JacobianFactor()
```

```cpp
JacobianFactor(const Vector & b_in)
```

```cpp
JacobianFactor(Key i1, const Matrix & A1, const Vector & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(Key i1, const Eigen::Matrix< double, M, N1 > & A1, const Eigen::Matrix< double, M, 1 > & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(Key i1, const Matrix & A1, Key i2, const Matrix & A2, const Vector & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(Key i1, const Eigen::Matrix< double, M, N1 > & A1, Key i2, const Eigen::Matrix< double, M, N2 > & A2, const Eigen::Matrix< double, M, 1 > & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(Key i1, const Matrix & A1, Key i2, const Matrix & A2, Key i3, const Matrix & A3, const Vector & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(Key i1, const Eigen::Matrix< double, M, N1 > & A1, Key i2, const Eigen::Matrix< double, M, N2 > & A2, Key i3, const Eigen::Matrix< double, M, N3 > & A3, const Eigen::Matrix< double, M, 1 > & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(const TERMS & terms, const Vector & b, const SharedDiagonal & model)
```

```cpp
JacobianFactor(const KEYS & keys, const VerticalBlockMatrix & augmentedMatrix, const SharedDiagonal & sigmas)
```

```cpp
JacobianFactor(const KEYS & keys, VerticalBlockMatrix && augmentedMatrix, const SharedDiagonal & model)
```

```cpp
JacobianFactor(const GaussianFactorGraph & graph)
```

```cpp
JacobianFactor(const GaussianFactorGraph & graph, const VariableSlots & p_variableSlots)
```

```cpp
JacobianFactor(const GaussianFactorGraph & graph, const Ordering & ordering)
```

```cpp
JacobianFactor(const GaussianFactorGraph & graph, const Ordering & ordering, const VariableSlots & p_variableSlots)
```

## 公开方法

### 方法

```cpp
JacobianFactor & operator=(const JacobianFactor & jf)
```

```cpp
GaussianFactor::shared_ptr clone() const
```

```cpp
bool isJacobian() const
```
Identify JacobianFactor-based types.

```cpp
print(const std::string & s = "", const KeyFormatter & formatter) const
```
print with optional string

```cpp
bool equals(const GaussianFactor & lf, double tol = 1e-9) const
```
assert equality up to a tolerance

```cpp
Vector unweighted_error(const VectorValues & c) const
```

```cpp
Vector error_vector(const VectorValues & c) const
```

```cpp
double error(const VectorValues & c) const
```
0.5*(A*x-b)'*D*(A*x-b).

```cpp
double deltaError(const VectorValues & c, double * oldError, double * newError) const
```

```cpp
Matrix augmentedInformation() const
```

```cpp
Matrix information() const
```

```cpp
hessianDiagonalAdd(VectorValues & d) const
```
Add the current diagonal to a VectorValues instance.

```cpp
hessianDiagonal(double * d) const
```
Raw memory access version of hessianDiagonal.

```cpp
std::map< Key, Matrix > hessianBlockDiagonal() const
```
Return the block diagonal of the Hessian for this factor.

```cpp
std::pair< Matrix, Vector > jacobian() const
```
Returns (dense) A,b pair associated with factor, bakes in the weights.

```cpp
std::pair< Matrix, Vector > jacobianUnweighted() const
```
Returns (dense) A,b pair associated with factor, does not bake in weights.

```cpp
Matrix augmentedJacobian() const
```

```cpp
Matrix augmentedJacobianUnweighted() const
```

```cpp
const VerticalBlockMatrix & matrixObject() const
```

```cpp
VerticalBlockMatrix & matrixObject()
```

```cpp
GaussianFactor::shared_ptr negate() const
```

```cpp
bool isConstrained() const
```

```cpp
DenseIndex getDim(const_iterator variable) const
```

```cpp
size_t rows() const
```

```cpp
size_t cols() const
```

```cpp
const SharedDiagonal & get_model() const
```

```cpp
SharedDiagonal & get_model()
```

```cpp
const constBVector getb() const
```

```cpp
constABlock getA(const_iterator variable) const
```

```cpp
constABlock getA() const
```

```cpp
BVector getb()
```

```cpp
ABlock getA(iterator variable)
```

```cpp
ABlock getA()
```

```cpp
ABlock getA(const Key & key)
```

```cpp
updateHessian(const KeyVector & keys, SymmetricBlockMatrix * info) const
```

```cpp
updateHessian(const KeyVector & keys, SymmetricBlockMatrix * info, DenseIndex beginCol, DenseIndex endCol) const
```

```cpp
Vector operator*(const VectorValues & x) const
```

```cpp
transposeMultiplyAdd(double alpha, const Vector & e, VectorValues & x) const
```

```cpp
multiplyHessianAdd(double alpha, const VectorValues & x, VectorValues & y) const
```

```cpp
multiplyHessianAdd(double alpha, const double * x, double * y, const std::vector< size_t > & accumulatedDims) const
```

```cpp
VectorValues gradientAtZero() const
```
A'*b for Jacobian.

```cpp
gradientAtZero(double * d) const
```
A'*b for Jacobian (raw memory version)

```cpp
Vector gradient(Key key, const VectorValues & x) const
```
Compute the gradient wrt a key at any values.

```cpp
JacobianFactor whiten() const
```

```cpp
std::pair< std::shared_ptr< GaussianConditional >, shared_ptr > eliminate(const Ordering & keys)
```

```cpp
setModel(bool anyConstrained, const Vector & sigmas)
```

```cpp
std::shared_ptr< GaussianConditional > splitConditional(size_t nrFrontals)
```

```cpp
double error(const VectorValues & c) const
```

```cpp
double error(const HybridValues & hybridValues) const
```

```cpp
VectorValues hessianDiagonal() const
```
Using the base method.

```cpp
hessianDiagonal(double * d) const
```
Using the base method.

## 类型别名

```cpp
using This = JacobianFactor
```
```cpp
using Base = GaussianFactor
```
```cpp
using shared_ptr = std::shared_ptr< This >
```
```cpp
using ABlock = VerticalBlockMatrix::Block
```
```cpp
using constABlock = VerticalBlockMatrix::constBlock
```
```cpp
using BVector = ABlock::ColXpr
```
```cpp
using constBVector = constABlock::ConstColXpr
```

## 详细说明

A Gaussian factor in the squared-error form. JacobianFactor implements a Gaussian, which has quadratic negative log-likelihood \[ E(x) = \frac{1}{2} (Ax-b)^T \Sigma^{-1} (Ax-b) \] where $ \Sigma $ is a *diagonal* covariance matrix. The matrix $ A $, r.h.s. vector $ b $, and diagonal noise model $ \Sigma $ are stored in this class. This factor represents the sum-of-squares error of a *linear* measurement function, and is created upon linearization of a NoiseModelFactor, which in turn is a sum-of-squares factor with a nonlinear measurement function. Here is an example of how this factor represents a sum-of-squares error: Letting $ h(x) $ be a *linear* measurement prediction function, $ z $ be the actual observed measurement, the residual is \[ f(x) = h(x) - z . \] If we expect noise with diagonal covariance matrix $ \Sigma $ on this measurement, then the negative log-likelihood of the Gaussian induced by this measurement model is \[ E(x) = \frac{1}{2} (h(x) - z)^T \Sigma^{-1} (h(x) - z) . \] Because $ h(x) $ is linear, we can write it as \[ h(x) = Ax + e \] and thus we have \[ E(x) = \frac{1}{2} (Ax-b)^T \Sigma^{-1} (Ax-b) \] where $ b = z - e $. This factor can involve an arbitrary number of variables, and in the above example $ x $ would almost always be only be a subset of the variables in the entire factor graph. There are special constructors for 1-, 2-, and 3- way JacobianFactors, and additional constructors for creating n-way JacobianFactors. The Jacobian matrix $ A $ is passed to these constructors in blocks, for example, for a 2-way factor, the constructor would accept $ A1 $ and $ A2 $, as well as the variable indices $ j1 $ and $ j2 $ and the negative log-likelihood represented by this factor would be \[ E(x) = \frac{1}{2} (A_1 x_{j1} + A_2 x_{j2} - b)^T \Sigma^{-1} (A_1 x_{j1} + A_2 x_{j2} - b) . \] HessianFactor, which represent a Gaussian likelihood over a set of variables.
**Discrete** factors, such as **Discrete** factors, such as

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`JacobianFactor` 用于 GTSAM factor graph 优化流程中。

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
