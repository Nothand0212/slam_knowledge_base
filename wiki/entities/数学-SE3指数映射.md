---
tags: [数学, 李群, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-monogs-analysis.md
---
# SE3指数映射

> 将 6 维李代数扰动映射为 SE(3) 位姿增量，是 SLAM/VIO 中在流形上更新相机或机器人位姿的基础操作。

## 定义

SE(3) 位姿写作：

$$
T =
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix},\quad R \in SO(3),\ t \in \mathbb{R}^3
$$

优化器通常不直接对 4x4 矩阵做加法，而是在切空间里优化 6 维扰动：

$$
\xi =
\begin{bmatrix}
\rho \\
\phi
\end{bmatrix}
$$

其中 $\rho$ 是平移扰动，$\phi$ 是旋转扰动。指数映射把它变成一个合法的 SE(3) 增量：

$$
\mathrm{Exp}(\xi)=
\begin{bmatrix}
\mathrm{Exp}_{SO(3)}(\phi) & J(\phi)\rho \\
0 & 1
\end{bmatrix}
$$

$J(\phi)$ 是 SO(3) 左雅可比：

$$
J(\phi)=I+\frac{1-\cos\theta}{\theta^2}[\phi]_\times+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,\quad \theta=\|\phi\|
$$

当 $\theta$ 很小时，需要用 Taylor 展开近似 $J(\phi)$ 和 $\mathrm{Exp}_{SO(3)}(\phi)$，否则分母里的 $\theta^2/\theta^3$ 会造成数值不稳定。

## 在 SLAM 中的作用

SE3 指数映射让优化变量保持在位姿流形上。前端 tracking 或后端 BA 只优化小量 $\xi$，再把增量组合回当前位姿：

- 光度法：通过渲染图像和观测图像的误差反传到相机位姿增量。
- 几何法：通过点到点、点到面、重投影误差求解位姿增量。
- VIO/LIO：IMU 预积分、ICP 和视觉重投影都依赖同一套位姿更新约定。

在 [[算法-MonoGS]] 中，位姿增量参与可微渲染，`cam_rot_delta` 和 `cam_trans_delta` 的梯度通过 [[方法-微分栅格化]] 回传；SE3 指数映射负责把这类 6-DoF 增量变成合法的相机位姿更新。

## 左乘与右乘约定

这里最容易出错的是更新顺序：

$$
T_{\text{new}}=\mathrm{Exp}(\xi)T
$$

这是左乘扰动，增量表达在当前估计的外部坐标约定下。

$$
T_{\text{new}}=T\mathrm{Exp}(\xi)
$$

这是右乘扰动，增量表达在机体/相机局部坐标约定下。

两种写法都可以，但残差定义、雅可比和代码里的 `pose_retr` / `retraction` 必须一致。把“右乘扰动”写成 `Exp(xi) @ T_current` 这类混合表达，会导致方向、符号或坐标系解释错误，尤其在相机位姿优化和 IMU 预积分融合时很难排查。

## 工程检查点

- 小角度分支：当 $\|\phi\|$ 接近 0 时使用 Taylor 展开。
- 更新约定：确认代码到底是 `Exp(xi) * T` 还是 `T * Exp(xi)`。
- 收敛阈值：`||xi|| < 1e-4` 只能作为增量停止条件，还应同时检查残差下降。
- 单位尺度：平移 $\rho$ 的单位通常是米，旋转 $\phi$ 的单位是弧度，直接比较范数时要注意量纲混合。

## 相关页面

- [[算法-MonoGS]]
- [[数学-流形优化]]
- [[组件-lietorch]]
- [[组件-Ceres Manifold API]]
