---
type: entity
tags: [Ceres, Manifold, LocalParameterization, API迁移, OB_GINS, IC-GVINS]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/OB_GINS/src/factors/pose_manifold.h
  - raw/codes/OB_GINS/src/factors/pose_manifold.cc
  - raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/pose_parameterization.h
  - raw/codes/SuperOdom/super_odometry/src/LidarProcess/pose_local_parameterization.cpp
---

# Ceres LocalParameterization → Manifold 迁移指南

> Ceres 2.1.0 将 `LocalParameterization` 升级为 `Manifold` 接口，新增 `Minus()` 操作，提供更完整的流形数学抽象。

## 为什么要迁移

| 特性 | LocalParameterization (旧) | Manifold (新) |
|------|--------------------------|---------------|
| Plus 操作 | `Plus(x, delta, x_plus)` | `Plus(x, delta, x_plus)` — 相同 |
| 雅可比 | `ComputeJacobian(x, J)` — 7×6 | `PlusJacobian(x, J)` — 相同尺寸 |
| 逆操作 | 无 | `Minus(y, x, y_minus_x)` — 6 维切空间差 |
| Minus 雅可比 | 无 | `MinusJacobian(x, J)` — 6×7 |
| Ceres 状态 | 已废弃 (deprecated) | 推荐使用 |
| 收敛分析 | 不完整 | 流形感知的信任域 |

关键区别：**Minus 操作使 Ceres 能在流形上计算两点之间的测地线距离**，这对信任域和收敛判断至关重要。旧 API 仅支持 `Plus`（把切空间增量"加"到流形点上），但从不执行逆操作。

## API 接口对比

### 旧 API：LocalParameterization

```cpp
class PoseParameterization : public ceres::LocalParameterization {
public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool ComputeJacobian(const double *x, double *jacobian) const override;
    int GlobalSize() const override;  // 流形维度：7
    int LocalSize() const override;   // 切空间维度：6
};
```

### 新 API：Manifold

```cpp
class PoseManifold : public ceres::Manifold {
public:
    int AmbientSize() const override;   // 替代 GlobalSize()
    int TangentSize() const override;   // 替代 LocalSize()
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;  // 替代 ComputeJacobian
    bool Minus(const double *y, const double *x, double *y_minus_x) const override;  // 新增
    bool MinusJacobian(const double *x, double *jacobian) const override; // 新增
};
```

## 迁移步骤

### 步骤 1：重命名基类和尺寸方法

```diff
- class PoseParameterization : public ceres::LocalParameterization {
+ class PoseManifold : public ceres::Manifold {
-     int GlobalSize() const override { return 7; }
+     int AmbientSize() const override { return 7; }
-     int LocalSize() const override { return 6; }
+     int TangentSize() const override { return 6; }
```

### 步骤 2：重命名 ComputeJacobian → PlusJacobian

`PlusJacobian` 语义与 `ComputeJacobian` 完全相同 — 计算 $\frac{\partial}{\partial \delta} \text{Plus}(x, \delta)$ 在 $\delta=0$ 处的雅可比（即 $7 \times 6$ 的秩-6 矩阵）。

两者的实现可以不做修改直接改名：

`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L51-L58`：

```cpp
bool PoseManifold::PlusJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jaco(jacobian);
    jaco.topRows<6>().setIdentity();
    jaco.bottomRows<1>().setZero();
    return true;
}
```

### 步骤 3：实现 Minus

`Minus(y, x)` 计算 $y \ominus x$，即流形上从 $x$ 到 $y$ 的切空间增量 $y \ominus x \in \mathbb{R}^6$。

**平移部分**：$p_y - p_x$

**旋转部分**：$q_x^{-1} \otimes q_y$ → 旋转向量 $\theta$（通过四元数取虚部 × 2 或 `Rotation::quaternion2vector`）

`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L60-L74`：

```cpp
bool PoseManifold::Minus(const double *y, const double *x, double *y_minus_x) const {
    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);
    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

    Eigen::Map<Eigen::Vector3d> p_y_minus_x(y_minus_x);         // δt
    Eigen::Map<Eigen::Vector3d> q_y_minus_x(y_minus_x + 3);     // δθ

    p_y_minus_x = p_y - p_x;
    q_y_minus_x = Rotation::quaternion2vector((q_x.inverse() * q_y).normalized());
    return true;
}
```

其中 `Rotation::quaternion2vector(q)` 提取四元数的旋转向量表示：$\theta = 2 \cdot \text{atan2}(\|\text{vec}(q)\|, q_w) \cdot \frac{\text{vec}(q)}{\|\text{vec}(q)\|}$。

### 步骤 4：实现 MinusJacobian

`MinusJacobian(x)` 计算 $\frac{\partial}{\partial y} (y \ominus x)$ 在 $y = x$ 处的雅可比，是一个 $6 \times 7$ 矩阵。

`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L76-L83`：

```cpp
bool PoseManifold::MinusJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jaco(jacobian);
    jaco.rightCols<6>().setIdentity();  // 对旋转向量的求导为单位阵（δθ ≈ 2·vec(q)）
    jaco.leftCols<1>().setZero();       // 平移部分的横列
    return true;
}
```

注意：尺寸为 **6×7**（切空间行 × 流形列），与 PlusJacobian 的 7×6 互为转置关系。

### 步骤 5：修改 Problem 中的注册方式

**旧 API（LocalParameterization）：**

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1858-L1859`：

```cpp
ceres::LocalParameterization *parameterization = new PoseParameterization;
problem.AddParameterBlock(pose, 7, parameterization);
```

**新 API（Manifold）：**

`raw/codes/OB_GINS/src/ob_gins.cc:L311-L312`：

```cpp
ceres::Manifold *manifold = new PoseManifold;
problem.AddParameterBlock(pose, Preintegration::numPoseParameter(), manifold);
```

## OB_GINS vs IC-GVINS：完整对比

| 项目 | 接口 | Plus | Minus | PlusJacobian | MinusJacobian |
|------|------|------|-------|-------------|---------------|
| **IC-GVINS** | `LocalParameterization` | 左乘 $q' = q \cdot dq$ | 无 | 7×6 单位阵 | 无 |
| **OB_GINS** | `Manifold` | 左乘 $q' = q \cdot dq$ | $q_x^{-1} \cdot q_y$ → $\theta$ | 7×6 单位阵 | 6×7 单位阵 |
| **SuperOdom `PoseLocalParameterization`** | `LocalParameterization` | 右乘 $q' = q \cdot dq$，一阶近似 | 无 | 7×6 单位阵 | 无 |
| **SuperOdom `PoseSE3Parameterization`** | `LocalParameterization` | 左乘 $T' = \exp(\xi) \cdot T$，精确 exp | 无 | 7×6 单位阵 | 无 |

### 左乘 vs 右乘的区别

- **左乘 (OB_GINS / IC-GVINS)**：$\delta T$ 作用在全局坐标系 → `p' = delta_q * p + delta_t`
- **右乘 (SuperOdom PoseLocal)**：$\delta T$ 作用在局部坐标系 → `p' = p + dp`

两者等价，仅在 $\text{PlusJacobian}$ 的表达上有 $R^T$ 的差异。

## Minus 伪代码

```
Minus(y, x):
    p_diff = y.p - x.p
    q_diff = normalize(inverse(x.q) * y.q)
    theta  = quaternion_to_rotation_vector(q_diff)
    y_minus_x = [p_diff, theta]    // 6维切空间向量
```

## 常见陷阱

1. **四元数归一化**：`Plus` 和 `Minus` 中四元数乘法后必须 `.normalized()`，防止迭代中漂移
2. **Jacobian 大小**：`PlusJacobian` 是 `AmbientSize × TangentSize`（7×6），`MinusJacobian` 是 `TangentSize × AmbientSize`（6×7）
3. **四元数符号**：`Minus` 计算 `q_x^{-1} * q_y` 时要确保选择正半球的四元数（w > 0），否则旋转方向可能翻转 2π
4. **LocalParameterization 已废弃**：Ceres 2.2+ 仍兼容但会发出警告，新项目应直接用 `Manifold`

## Agent 实现提示

### 适用场景
- 将基于 Ceres 2.0 的 Optimization-based SLAM（IC-GVINS, VINS-Mono 等）升级到 Ceres 2.1+
- 需要更稳定的收敛行为（信任域方法利用 Minus）
- 编写新的流形参数化（如 IMU 偏差的黎曼流形、尺度因子无界约束等）

### 实现骨架（伪代码）

```
class PoseManifold : public Manifold:
    AmbientSize → 7
    TangentSize → 6

    Plus(x, delta) → x_plus_delta:
        p = p_x + delta_p
        q = normalize(q_x * exp(delta_theta/2 → quat))
        return [p, q]

    PlusJacobian(x) → J(7×6):
        J[:6,:] = I_6
        J[6,:]  = 0

    Minus(y, x) → y_minus_x(6):
        δp = p_y - p_x
        dq = normalize(inverse(q_x) * q_y)
        δθ = quat_to_rotvec(dq)
        return [δp, δθ]

    MinusJacobian(x) → J(6×7):
        J[:, 1:7] = I_6   # or J.rightCols(6) = I_6
        J[:, 0]   = 0     # or J.leftCols(1) = 0
```

### 源码检索锚点
- OB_GINS Manifold 定义：`raw/codes/OB_GINS/src/factors/pose_manifold.h:L28-L39`
- OB_GINS Manifold 实现：`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L26-L83`
- OB_GINS 注册：`raw/codes/OB_GINS/src/ob_gins.cc:L311-L312`
- IC-GVINS 旧 API 定义：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/pose_parameterization.h:L30-L66`
- IC-GVINS 注册：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1858-L1859`
- SuperOdom PoseSE3 (精确 EXP)：`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L83-L150`

## 相关页面

- [[组件-Ceres Manifold API]]
- [[组件-Ceres-Solver]]
- [[数学-SE3指数映射]]
- [[方法-Ceres解析雅可比CostFunction]]
