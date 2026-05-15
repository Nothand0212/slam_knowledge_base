---
type: entity
tags: [Ceres, 解析雅可比, CostFunction, SuperOdom, LiDAR, SE3]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp
  - raw/codes/SuperOdom/super_odometry/include/super_odometry/LidarProcess/factor/lidarOptimization.h
  - raw/codes/SuperOdom/super_odometry/src/LidarProcess/pose_local_parameterization.cpp
---

# Ceres 解析雅可比 CostFunction

> 用 Ceres `SizedCostFunction` 手写 LiDAR 点-线/点-面残差雅可比，避免自动求导在大量点云因子中的开销。

## 为什么用解析雅可比

LiDAR 优化中每帧可能产生数百个边缘/平面因子。AutoDiff 对每个因子单独做前向自动微分，内存和计算开销显著。解析雅可比：

- **速度**：LiDAR 因子残差简单（点变换+距离），手动推导的雅可比只需几行矩阵运算
- **稳定性**：AutoDiff 的双精度 `Jet` 类型在 SE(3) 参数化中可能产生数值精度问题
- **可控性**：显式控制 SE(3) 李代数求导的细节，避免 Jet 类型在四元数归一化等操作中的隐式行为

## EdgeAnalyticCostFunction：点到线边缘残差

### 残差定义

对于 LiDAR 边缘特征点，残差是当前帧变换后的点到上一帧边缘直线两点连线的叉积距离。

当前点 $p_c$ 通过位姿 $T = (t, q) \in \mathbb{R}^7$ 变换到世界坐标系：

$$p_w = q \otimes p_c + t$$

残差 $\mathbf{e} \in \mathbb{R}^3$ 为 $p_w$ 到直线 $AB$ 的距离（叉积面积除以边长）：

$$\mathbf{e} = \frac{(p_w - A) \times (p_w - B)}{\|A - B\|}$$

### 雅可比推导

残差对 SE(3) 李代数 $\delta\xi = [\delta t, \delta\theta] \in \mathbb{R}^6$ 的雅可比：

$\frac{\partial p_w}{\partial t} = I_{3\times3}$，$\frac{\partial p_w}{\partial \theta} = -R \cdot [p_c]_\times$

其中 $R = q.\text{toRotationMatrix}()$，$[p_c]_\times$ 是 $p_c$ 的反对称矩阵。

令 $\mathbf{d} = B - A$，$[\mathbf{d}]_\times$ 为 $\mathbf{d}$ 的反对称矩阵，则残差雅可比：

$$\frac{\partial \mathbf{e}}{\partial \xi} = \frac{[\mathbf{d}]_\times}{\|A - B\|} \cdot \begin{bmatrix} I_{3\times3} & -R \cdot [p_c]_\times \end{bmatrix}_{3\times 6}$$

结果是一个 $3 \times 6$ 矩阵，打包进 $3 \times 7$ 行主序输出（最后 1 列为 0，因为 SE(3) 7 维参数空间 → 6 维切空间）。

### 继承结构

```cpp
class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<3, 7>
```

- 残差维度：3（三维距离向量）
- 参数块维度：7（t + q）

### 源码实现

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L12-L47`：

```cpp
bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters,
                                         double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0] + 3);

    // 点到直线距离（叉积面积 / 边长）
    Eigen::Vector3d lp = q_w_curr * curr_point + t_w_curr;
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;
    residuals[0] = nu.x() / de.norm();
    residuals[1] = nu.y() / de.norm();
    residuals[2] = nu.z() / de.norm();

    if (jacobians != NULL && jacobians[0] != NULL) {
        Eigen::Matrix3d skew_lp = skew(curr_point);
        Eigen::Matrix<double, 3, 6> dp_by_so3;
        dp_by_so3.block<3,3>(0,0).setIdentity();
        dp_by_so3.block<3,3>(0,3) = -q_w_curr.toRotationMatrix() * skew_lp;

        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
        J_se3.setZero();
        Eigen::Vector3d re = last_point_b - last_point_a;
        Eigen::Matrix3d skew_re = skew(re);
        J_se3.block<3,6>(0,0) = skew_re * dp_by_so3 / de.norm();
    }
    return true;
}
```

关键点：
- `dp_by_so3` 是 $\partial p_w / \partial \xi$，即变换后点对 6 维李代数的雅可比
- `skew_re` 是 $\mathbf{d} = B-A$ 的反对称矩阵，完成链式求导 $\partial e / \partial p_w \cdot \partial p_w / \partial \xi$
- 雅可比存入 $3 \times 7$ RowMajor 矩阵，第 7 列为 0

## SurfNormAnalyticCostFunction：点到面残差

### 残差定义

当前帧的平面点 $p_c$ 变换后到上一帧拟合的平面 $(n, D)$ 的有向距离：

$$e = n^\top \cdot (q \otimes p_c + t) + D$$

其中 $n$ 是平面单位法向量，$D = -n \cdot P_A$（$P_A$ 为平面上任一点）。

残差维度为 1（标量距离）。

### 雅可比推导

$$\frac{\partial e}{\partial \xi} = n^\top \cdot \begin{bmatrix} I_{3\times3} & -R \cdot [p_c]_\times \end{bmatrix}_{1\times 6}$$

结果打包进 $1 \times 7$ 行主序矩阵。

### 继承结构

```cpp
class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
```

### 源码实现

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L55-L80`：

```cpp
bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters,
                                             double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0] + 3);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;

    residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;

    if (jacobians != NULL && jacobians[0] != NULL) {
        Eigen::Matrix3d skew_point_w = skew(curr_point);
        Eigen::Matrix<double, 3, 6> dp_by_so3;
        dp_by_so3.block<3,3>(0,0).setIdentity();
        dp_by_so3.block<3,3>(0,3) = -q_w_curr.toRotationMatrix() * skew_point_w;

        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
        J_se3.setZero();
        J_se3.block<1,6>(0,0) = plane_unit_norm.transpose() * dp_by_so3;
    }
    return true;
}
```

## PoseLocalParameterization：SE(3) 流形参数化

SuperOdom 提供两个 PoseParameterization 实现：

| 实现 | 文件 | Plus 含义 |
|------|------|-----------|
| `PoseLocalParameterization` | `pose_local_parameterization.cpp` | $T' = T \cdot \exp(\delta\xi)$（local frame） |
| `PoseSE3Parameterization` | `lidarOptimization.cpp` | $T' = \exp(\delta\xi) \cdot T$（global frame） |

### PoseLocalParameterization::Plus（右乘/局部）

`raw/codes/SuperOdom/super_odometry/src/LidarProcess/pose_local_parameterization.cpp:L7-L23`：

```cpp
bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                      double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);
    Eigen::Map<const Eigen::Vector3d> dp(delta);

    // 小角度近似：dq ≈ [1, δθ_x/2, δθ_y/2, δθ_z/2]
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();  // 右乘：局部更新
    return true;
}
```

注意 `deltaQ` 是 $\exp(\delta\theta/2)$ 的一阶近似（非精确 $\exp$），适用于优化迭代中增量较小的情况。

### PoseSE3Parameterization::Plus（左乘/全局）

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L83-L99`：

```cpp
bool PoseSE3Parameterization::Plus(const double *x, const double *delta,
                                   double *x_plus_delta) const {
    // 使用精确的 SE(3)→SE(3) 映射（exp + J）
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta),
                        delta_q, delta_t);
    // ...
    quater_plus = delta_q * quater;          // 左乘：全局更新
    trans_plus = delta_q * trans + delta_t;
    return true;
}
```

使用 `getTransformFromSe3` 精确计算 $\exp(\delta\xi)$，包含罗德里格斯公式和左雅可比 $J$。

### ComputeJacobian

两个实现相同（`lidarOptimization.cpp:L102-L109` / `pose_local_parameterization.cpp:L24-L31`）：

```cpp
bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();   // 对 δt, δθ 的平凡求导
    j.bottomRows<1>().setZero();    // 四元数 w 分量不在切空间更新（q = dq * q）
    return true;
}
```

7×6 雅可比中前 6 行为单位阵，最后一行（对应四元数标量部分）为 0。

### getTransformFromSe3：SE(3) 指数映射

`raw/codes/SuperOdom/super_odometry/src/LaserMapping/lidarOptimization.cpp:L112-L150`：

```cpp
void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3,
                         Eigen::Quaterniond& q, Eigen::Vector3d& t) {
    Eigen::Vector3d omega(se3.data() + 3);   // 旋转向量
    Eigen::Vector3d upsilon(se3.data());      // 平移（非实际 t）

    double theta = omega.norm();
    double half_theta = 0.5 * theta;

    // 四元数：罗德里格斯 → 四元数
    double real_factor = cos(half_theta);
    double imag_factor;
    if (theta < 1e-10) {
        imag_factor = 0.5 - 0.0208333 * theta² + 0.000260417 * theta⁴;  // sin(half_theta)/theta 的泰勒展开
    } else {
        imag_factor = sin(half_theta) / theta;
    }
    q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(),
                           imag_factor * omega.y(), imag_factor * omega.z());

    // 平移：左雅可比 J(ω) * upsilon
    Eigen::Matrix3d J;
    if (theta < 1e-10) {
        J = q.matrix();
    } else {
        Eigen::Matrix3d Omega2 = Omega * Omega;
        J = I + (1 - cos(theta)) / theta² * Omega + (theta - sin(theta)) / theta³ * Omega2;
    }
    t = J * upsilon;
}
```

## 手写解析 CostFunction 模板

```cpp
// 1. 继承 SizedCostFunction<残差维度, 参数块1维度, 参数块2维度, ...>
class MyAnalyticCostFunction : public ceres::SizedCostFunction<M, N1, N2> {
public:
    // 2. 构造函数：保存测量值、权重等常量数据
    MyAnalyticCostFunction(const Measurement& meas) : meas_(meas) {}

    // 3. 实现 Evaluate
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals, double **jacobians) const override {
        // 3a. 提取参数：使用 Eigen::Map
        Eigen::Map<const Eigen::Vector3d> t(parameters[0]);
        Eigen::Map<const Eigen::Quaterniond> q(parameters[0] + 3);

        // 3b. 计算残差
        residuals[0] = /* 残差公式 */;

        // 3c. 计算雅可比（检查 NULL）
        if (jacobians != NULL && jacobians[0] != NULL) {
            // 解析导数推导
            Eigen::Map<Eigen::Matrix<double, M, N1, Eigen::RowMajor>>
                jacobian0(jacobians[0]);
            jacobian0.setZero();
            // 填充解析雅可比
        }
        return true;
    }

private:
    Measurement meas_;
};
```

### 关键注意事项

1. **`ceres::SizedCostFunction` vs `ceres::AutoDiffCostFunction`**：手写时继承 `SizedCostFunction`，模板参数为 `<残差维, 参数维...>`
2. **雅可比矩阵布局**：Ceres 使用行主序（RowMajor），非 Eigen 默认的列主序
3. **NULL 检查**：`jacobians[i]` 可能为 `NULL`（仅计算残差时），必须判空
4. **SE(3) 求导**：$\partial(p_w) / \partial \theta = -R[p_c]_\times$（扰动在切空间，不是直接对四元数分量求导）
5. **参数内存布局**：`parameters[0]` 是 7 维数组 `[x, y, z, qx, qy, qz, qw]`
6. **四元数顺序**：Eigen::Quaterniond 内部存储为 (x, y, z, w)，而 Ceres 默认用 Hamilton 顺序 (w, x, y, z)。但 `Eigen::Map<const Eigen::Quaterniond>(x+3)` 因为 Eigen::Quaterniond 内部为 (x,y,z,w)，从 x+3 起正好是 4 个 double，构成 (x,y,z,w)

## Agent 实现提示

### 适用场景
- LiDAR SLAM 中需要大量点云残差因子的场景
- SE(3) 位姿优化，AutoDiff 开销不可接受
- 需要精确控制求导路径（如使用自定义参数化）

### 输入输出契约
- **输入**：`const double *const *parameters` — 参数块指针数组，每块包含位姿 `[tx, ty, tz, qx, qy, qz, qw]`
- **输出**：`double *residuals` — 残差向量（行主序），`double **jacobians` — 每个参数块的雅可比（行主序，`M_i × N_j`）
- **返回**：`true` 表示成功

### 实现注意事项
- 直接使用 Eigen::Map 进行零拷贝转换，不要创建临时 Eigen 对象
- 雅可比输出必须 RowMajor，否则 Ceres 读取布局错误
- 小角度时用一阶近似 `deltaQ` 可避免 `sin/cos` 调用；对精度要求高的场景用精确 `exp`
- 所有不用的雅可比块填入 0

## 相关页面

- [[组件-Ceres-Solver]]
- [[组件-Ceres Manifold API]]
- [[数学-SE3指数映射]]
- [[方法-Ceres-Manifold迁移指南]]
