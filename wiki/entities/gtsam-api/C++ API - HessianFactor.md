---
type: entity
tags: [GTSAM, C++ API, Inference, HessianFactor]
created: 2026-05-29
updated: 2026-05-29
sources:
  - https://gtsam.org/doxygen/
  - raw/codes/gtsam
---

# GTSAM::HessianFactor

> **类** | 头文件: `HessianFactor.h` | [在线文档](https://gtsam.org/doxygen/)

A Gaussian factor using the canonical parameters (information form)

## 继承关系

- 继承自 `gtsam::GaussianFactor`

## 构造函数

```cpp
HessianFactor()
```

```cpp
HessianFactor(Key j, const Matrix & G, const Vector & g, double f)
```

```cpp
HessianFactor(Key j, const Vector & mu, const Matrix & Sigma)
```

```cpp
HessianFactor(Key j1, Key j2, const Matrix & G11, const Matrix & G12, const Vector & g1, const Matrix & G22, const Vector & g2, double f)
```

```cpp
HessianFactor(Key j1, Key j2, Key j3, const Matrix & G11, const Matrix & G12, const Matrix & G13, const Vector & g1, const Matrix & G22, const Matrix & G23, const Vector & g2, const Matrix & G33, const Vector & g3, double f)
```

```cpp
HessianFactor(const KeyVector & js, const std::vector< Matrix > & Gs, const std::vector< Vector > & gs, double f)
```

```cpp
HessianFactor(const KEYS & keys, const SymmetricBlockMatrix & augmentedInformation)
```

```cpp
HessianFactor(const JacobianFactor & cg)
```

```cpp
HessianFactor(const GaussianFactor & factor)
```

```cpp
HessianFactor(const GaussianFactorGraph & factors, const Scatter & scatter)
```

```cpp
HessianFactor(const GaussianFactorGraph & factors)
```

## 公开方法

### 方法

```cpp
GaussianFactor::shared_ptr clone() const
```

```cpp
print(const std::string & s = "", const KeyFormatter & formatter) const
```

```cpp
bool equals(const GaussianFactor & lf, double tol = 1e-9) const
```

```cpp
double error(const VectorValues & c) const
```

```cpp
double deltaError(const VectorValues & c, double * oldError, double * newError) const
```

```cpp
DenseIndex getDim(const_iterator variable) const
```

```cpp
size_t rows() const
```

```cpp
GaussianFactor::shared_ptr negate() const
```

```cpp
double constantTerm() const
```

```cpp
double & constantTerm()
```

```cpp
SymmetricBlockMatrix::constBlock linearTerm(const_iterator j) const
```

```cpp
SymmetricBlockMatrix::constBlock linearTerm() const
```

```cpp
SymmetricBlockMatrix::Block linearTerm()
```

```cpp
const SymmetricBlockMatrix & info() const
```
Return underlying information matrix.

```cpp
SymmetricBlockMatrix & info()
```

```cpp
Matrix augmentedInformation() const
```

```cpp
Eigen::SelfAdjointView< SymmetricBlockMatrix::constBlock, Eigen::Upper > informationView() const
```
Return self-adjoint view onto the information matrix (NOT augmented).

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
Return (dense) matrix associated with factor.

```cpp
Matrix augmentedJacobian() const
```

```cpp
updateHessian(const KeyVector & keys, SymmetricBlockMatrix * info) const
```

```cpp
updateHessian(const KeyVector & keys, SymmetricBlockMatrix * info, DenseIndex beginCol, DenseIndex endCol) const
```

```cpp
updateHessian(HessianFactor * other) const
```

```cpp
multiplyHessianAdd(double alpha, const VectorValues & x, VectorValues & y) const
```

```cpp
VectorValues gradientAtZero() const
```
eta for Hessian

```cpp
gradientAtZero(double * d) const
```
Raw memory access version of gradientAtZero.

```cpp
Vector gradient(Key key, const VectorValues & x) const
```

```cpp
std::shared_ptr< GaussianConditional > eliminateCholesky(const Ordering & keys)
```

```cpp
VectorValues solve()
```
Solve the system A'*A delta = A'*b in-place, return delta as VectorValues.

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
using Base = GaussianFactor
```
```cpp
using This = HessianFactor
```
```cpp
using shared_ptr = std::shared_ptr< This >
```
```cpp
using Block = SymmetricBlockMatrix::Block
```
```cpp
using constBlock = SymmetricBlockMatrix::constBlock
```

## 详细说明

HessianFactor implements a general quadratic factor of the form \[ E(x) = 0.5 x^T G x - x^T g + 0.5 f \] that stores the matrix $ G $, the vector $ g $, and the constant term $ f $. When $ G $ is positive semidefinite, this factor represents a Gaussian, in which case $ G $ is the information matrix $ \Lambda $, $ g $ is the information vector $ \eta $, and $ f $ is the residual sum-square-error at the mean, when $ x = \mu $. Indeed, the negative log-likelihood of a Gaussian is (up to a constant) $ E(x) = 0.5(x-\mu)^T P^{-1} (x-\mu) $ with $ \mu $ the mean and $ P $ the covariance matrix. Expanding the product we get  \[
E(x) = 0.5 x^T P^{-1} x - x^T P^{-1} \mu + 0.5 \mu^T P^{-1} \mu
\] We define the Information matrix (or Hessian) $ \Lambda = P^{-1} $ and the information vector $ \eta = P^{-1} \mu = \Lambda \mu $ to arrive at the canonical form of the Gaussian:  \[
E(x) = 0.5 x^T \Lambda x - x^T \eta + 0.5 \mu^T \Lambda \mu
\] This factor is one of the factors that can be in a GaussianFactorGraph. It may be returned from NonlinearFactor::linearize(), but is also used internally to store the Hessian during Cholesky elimination. This can represent a quadratic factor with characteristics that cannot be represented using a JacobianFactor (which has the form $ E(x) = \Vert Ax - b \Vert^2 $ and stores the Jacobian $ A $ and error vector $ b $, i.e. is a sum-of-squares factor). For example, a HessianFactor need not be positive semidefinite, it can be indefinite or even negative semidefinite. If a HessianFactor is indefinite or negative semi-definite, then in order for solving the linear system to be possible, the Hessian of the full system must be positive definite (i.e. when all small Hessians are combined, the result must be positive definite). If this is not the case, an error will occur during elimination. This class stores G, g, and f as an augmented matrix HessianFactor::matrix_. The upper-left n x n blocks of HessianFactor::matrix_ store the upper-right triangle of G, the upper-right-most column of length n of HessianFactor::matrix_ stores g, and the lower-right entry of HessianFactor::matrix_ stores f, i.e. HessianFactor::matrix_=[G11G12G13...g1
0G22G23...g2
00G33...g3
::::
000...f]
 Blocks can be accessed as follows: G11=info(begin(),begin());
G12=info(begin(),begin()+1);
G23=info(begin()+1,begin()+2);
g2=linearTerm(begin()+1);
f=constantTerm();
.......

## 源码位置

- 分支: `develop`
- 远程: https://github.com/borglab/gtsam.git
- 本地快照: `raw/codes/gtsam`

## Agent 实现提示

### 适用场景

`HessianFactor` 用于 GTSAM factor graph 优化流程中。

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
