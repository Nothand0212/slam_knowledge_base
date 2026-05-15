---
type: entity
tags: [Ceres, API, 流形优化, 版本演进, 解析雅可比]
created: 2026-04-29
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-ob_gins-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
  - raw/codes/OB_GINS/src/factors/pose_manifold.cc
  - raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp
---

# Ceres Manifold API

> Ceres 2.1+ 的流形参数化接口，用 6 维切空间更新 7 维位姿参数块，替代旧 LocalParameterization。

## 定义

Ceres Solver 2.1.0 引入 `ceres::Manifold` 作为 `ceres::LocalParameterization` 的升级替代。核心区别在于 Manifold 提供了 `Minus()` 操作，使 Ceres 能在流形上计算两点之间的测地线距离，对信任域方法和收敛分析至关重要。

OB_GINS 已适配 Manifold，而 IC-GVINS 仍使用 LocalParameterization。

## 核心特征

- `ceres::Manifold` 继承自 `ceres::LocalParameterization` 的语义扩展
- 需实现：`AmbientSize()`, `TangentSize()`, `Plus()`, `PlusJacobian()`, `Minus()`, `MinusJacobian()`
- PoseManifold: `AmbientSize=7 (t+q)`, `TangentSize=6 (dt+dθ)`
- Plus 实现：`p' = p + dp, q' = (q * exp(dθ)).normalized()`
- Minus 实现：`δp = p_y - p_x, δθ = quat2rotvec(inv(q_x) * q_y)`
- 对 phad_fusion 的影响：如果使用 Ceres 2.1+ 应参考 OB_GINS 而非 IC-GVINS 的实现

## SuperOdom 中的解析雅可比示例

SuperOdom 提供两个 CostFunction 实现，展示如何在 `Evaluate` 中手写雅可比，绕过 AutoDiff 的开销。

### EdgeAnalyticCostFunction（点到线，3维残差）

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L12-L47`

```cpp
class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<3, 7> {
    bool Evaluate(...) const {
        // 残差：点到直线距离（叉积面积 / 边长）
        Eigen::Vector3d lp = q_w_curr * curr_point + t_w_curr;
        Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
        residuals[0..2] = nu / da_to_db.norm();

        // 雅可比：∂e/∂ξ = skew(d) * [I_3 | -R·[p_c]_×]  / |d|
        Eigen::Matrix<double, 3, 6> dp_by_so3;
        dp_by_so3 << I_3,  -R * skew(p_c);
        J_se3(0:3, 0:6) = skew(d) * dp_by_so3 / |d|;  // 3×6
    }
};
```

雅可比推导链式法则：

$$\frac{\partial e}{\partial \xi} = \frac{\partial e}{\partial p_w} \cdot \frac{\partial p_w}{\partial \xi} = \frac{[d]_\times}{\|d\|} \cdot [I \;\; -R[p_c]_\times]_{3\times 6}$$

### SurfNormAnalyticCostFunction（点到面，1维残差）

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L55-L80`

```cpp
class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
    bool Evaluate(...) const {
        // 残差：有向点到面距离
        residuals[0] = n.dot(R*p_c + t) + D;

        // 雅可比：1×6，直接 n^T * [I_3 | -R·[p_c]_×]
        J_se3(0, 0:6) = n.transpose() * dp_by_so3;  // 1×6
    }
};
```

### 设计模式：`SizedCostFunction` + `LocalParameterization`

解析 CostFunction 通常与自定义 LocalParameterization 配合使用：

1. CostFunction 的 Evaluate 接收 `parameters[0]`（7维 `[t,q]`）
2. 雅可比矩阵以 RowMajor 布局存入 `jacobians[0]`（残差维 × 7）
3. LocalParameterization 的 `ComputeJacobian` 提供 7→6 的链式映射
4. Ceres 自动将两者合成最终雅可比：$J_{\text{total}} = J_{\text{residual}} \cdot J_{\text{local}}$

参见 [[方法-Ceres解析雅可比CostFunction]] 的完整数学推导。

## OB_GINS Manifold 迁移实战

OB_GINS 的 `PoseManifold` 是完整的 Ceres 2.1+ Manifold 实现，包含原 LocalParameterization 没有的 `Minus` 和 `MinusJacobian`。

### 接口声明

`raw/codes/OB_GINS/src/factors/pose_manifold.h:L28-L39`：

```cpp
class PoseManifold : public ceres::Manifold {
public:
    int AmbientSize() const override;     // 7
    int TangentSize() const override;     // 6
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;   // 7×6
    bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;  // 6×7
};
```

### Plus 实现

`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L34-L49`：

```cpp
bool PoseManifold::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    // p' = p + dp
    p_x_plus = p_x + p_delta;
    // q' = normalize(q * exp(dθ))
    q_delta = Rotation::rotvec2quaternion(Eigen::Map<const Eigen::Vector3d>(delta + 3));
    q_x_plus = (q_x * q_delta).normalized();
    return true;
}
```

### Minus 实现

`raw/codes/OB_GINS/src/factors/pose_manifold.cc:L60-L74`：

```cpp
bool PoseManifold::Minus(const double *y, const double *x, double *y_minus_x) const {
    p_y_minus_x = p_y - p_x;
    // δθ = log(q_x^{-1} * q_y) — 旋转向量表示
    q_y_minus_x = Rotation::quaternion2vector((q_x.inverse() * q_y).normalized());
    return true;
}
```

### 在 Problem 中注册

`raw/codes/OB_GINS/src/ob_gins.cc:L311-L312`：

```cpp
ceres::Manifold *manifold = new PoseManifold();
problem.AddParameterBlock(statedatalist[k].pose,
                          Preintegration::numPoseParameter(), manifold);
```

### IC-GVINS 旧 API 对比

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/factors/pose_parameterization.h:L30-L66`：

```cpp
class PoseParameterization : public ceres::LocalParameterization {
    bool Plus(...) const override;             // 与 Manifold 的 Plus 相同
    bool ComputeJacobian(...) const override;  // 即 PlusJacobian
    int GlobalSize() const override { return 7; }
    int LocalSize() const override { return 6; }
    // 缺少 Minus, MinusJacobian
};
```

注册方式：

```cpp
ceres::LocalParameterization *parameterization = new PoseParameterization;
problem.AddParameterBlock(pose, 7, parameterization);
```

完整迁移指南见 [[方法-Ceres-Manifold迁移指南]]。

## 两轮优化中的 Manifold 角色

OB_GINS 和 IC-GVINS 都采用两轮优化 + 卡方检验的粗差剔除策略：

1. **Round 1**：带 Huber Loss 求解，获得初步状态
2. **卡方检验**：`problem.EvaluateResidualBlock(id, ...)` 计算每个因子的 cost，若 `2 * cost > χ²_threshold` 则判定为粗差
3. **清理**：GNSS 粗差重赋权（`std *= sqrt(chi²/threshold)`），视觉粗差直接删除 ResidualBlock
4. **Round 2**：不带核函数再次求解

在这个流程中，Manifold 的 `Minus` 操作使 Ceres 能在 Round 2 中更精确地评估参数更新方向和步长。详见 [[方法-Ceres两轮优化粗差剔除]]。

## 相关页面

- 实现于：[[算法-OB_GINS]]（`src/factors/pose_manifold.h`）
- 对比：[[数学-流形优化]]
- 数学基础：[[数学-SE3指数映射]]
- 解析雅可比模式：[[方法-Ceres解析雅可比CostFunction]]
- API 迁移：[[方法-Ceres-Manifold迁移指南]]
- 粗差剔除：[[方法-Ceres两轮优化粗差剔除]]
- Ceres 原始参考：[[组件-Ceres-Solver]]
- 外部核验：Ceres 官方建模文档使用 `Manifold` 表达非欧参数块的切空间更新接口
