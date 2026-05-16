---
tags: [LiDAR, 回环检测, 描述子, ScanContext, 地点识别]
type: source
sources:
  - "https://doi.org/10.1109/IROS.2018.8593953"
created: 2026-05-15
updated: 2026-05-15
paper_title: "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map"
paper_authors: "Giseop Kim, Ayoung Kim"
paper_venue: "IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)"
paper_year: 2018
paper_url: "https://doi.org/10.1109/IROS.2018.8593953"
---

# ScanContext：自中心空间描述子用于 3D 点云地图地点识别

> 将 3D LiDAR 扫描压缩为 2D 极坐标矩阵（Ring × Sector），高度编码 + 旋转不变匹配，为 LiDAR SLAM 提供高效、轻量的回环候选生成器。

## 核心贡献

1. **极坐标鸟瞰描述子**：将一帧 3D LiDAR 扫描按极坐标 $(r, \theta)$ 离散化为 $N_r \times N_s$（如 20×60）的 2D 矩阵——ScanContext。每个格子存储该区域点的**最大高度**（$\max z$），得到一个稠密的场景签名。描述子是 egocentric（以传感器为中心），天然跟随载体旋转
2. **双阶段检索策略**：阶段一用 ring key（每行高度的均值向量，$N_r$ 维）做 KD-tree 快速近似搜索，召回少量候选；阶段二对候选用 ScanContext 完整矩阵做 column-wise 距离比较，同时枚举列循环移位以匹配 yaw 变化，取最小距离
3. **旋转不变性通过列移位实现**：ScanContext 的列对应方位角 sector，列的循环平移等价于传感器绕 z 轴旋转。通过穷举 $N_s$ 个列移位，取最小 column-wise 距离，不需要预先做 ICP 或特征匹配就能得到粗 yaw 估计
4. **轻量高效**：描述子生成 $O(N_{pts})$，ring key 检索 $O(\log M)$，候选距离计算 $O(K \cdot N_r \cdot N_s^2)$（$K$ 为 KD-tree 候选数）。远轻于 Global ICP 或全局特征匹配

## 核心公式

### ScanContext 生成

给定一帧 LiDAR 点云 $P = \{p_1, \dots, p_N\}$，对每个点计算极坐标 $(r, \theta)$ 并分配 bin：

$$
\begin{aligned}
r(p_k) &= \sqrt{x_k^2 + y_k^2} \\
\theta(p_k) &= \text{atan2}(y_k, x_k)
\end{aligned}
$$

将有效范围（$r \in [0, L_{\max}]$，$\theta \in [-\pi, \pi]$）划分为 $N_r$ 个环和 $N_s$ 个扇形，ScanContext $I \in \mathbb{R}^{N_r \times N_s}$ 的每个元素为：

$$
I_{ij} = \max_{p_k \in \text{bin}(i,j)} z_k
$$

无点区域填入 $-1000$（零高度和缺测点区分开，但实际上缺测点参与距离计算时会特殊处理）。

### Ring Key

从 ScanContext 中按行求均值得到 ring key $k \in \mathbb{R}^{N_r}$：

$$
k_i = \frac{1}{N_i^{\text{valid}}} \sum_{j \in \text{valid}(i)} I_{ij}
$$

只对有效（非空）列求均值。ring key 编码场景的总体垂直结构（树高、建筑高）。检索时用 KD-tree（nanoflann）做最近邻搜索。

### Column-wise 距离与旋转估计

对查询描述子 $I_q$ 和候选 $I_c$，列移位 $s$ 下的距离为：

$$
d(I_q, I_c, s) = \frac{1}{N_s} \sum_{j=1}^{N_s} \left(1 - \frac{I_q^{(:,j)} \cdot I_c^{(:,j+s \bmod N_s)}}{\|I_q^{(:,j)}\| \cdot \|I_c^{(:,j+s \bmod N_s)}\|}\right)
$$

取最小距离的列移位作为相对 yaw：

$$
d_{\min} = \min_{s \in \{0,\dots,N_s-1\}} d(I_q, I_c, s), \quad s^* = \arg\min_s d(I_q, I_c, s)
$$

粗 yaw 估计为 $\Delta\theta = s^* \cdot \frac{2\pi}{N_s}$。

### 回环判定

当 $d_{\min} < \tau$（典型阈值 $\tau \approx 0.3$）且距离上一次回环超过一定帧数（排除连续帧）时，判定为回环候选。回环相对位姿的旋转部分由 $s^*$ 对应的粗 yaw 给出，但需后续几何验证（ICP/GICP）确认。

## 工程要点

| 维度 | 方案 |
|------|------|
| 描述子维度 | 典型 $N_r=20, N_s=60$（1200 元浮点数） |
| 编码内容 | 最大高度（可替换为强度、SNR、语义标签等） |
| 检索阶段一 | ring key KD-tree (nanoflann) 做 Top-K 近似召回 |
| 检索阶段二 | 完整 ScanContext 列移位距离（余弦距离），枚举 $N_s$ 次 |
| 旋转估计 | 列移位给出粗 yaw（分辨率 $2\pi/N_s = 6^\circ$），不可替代几何配准 |
| 排除策略 | 排除最近 $T$ 帧（典型 $T=50$）+ 欧式距离门限 |
| 变体 | Intensity Scan Context（强度编码，适配 4D Radar）、Semantic Scan Context（语义标签编码） |
| ROS 集成 | 作为独立回环检测模块，常见于 LIO-SAM、FAST-LIO-SAM-SC-QN 的改进版本 |

## 方法定位

ScanContext 已成为 LiDAR SLAM 回环检测的事实标准工具，广泛应用于 LIO-SAM (SC-LIO-SAM)、FAST-LIO-SAM-SC-QN、LT-Mapper 等多个系统。其设计哲学——"轻量描述子快速召回 + 几何配准最终确认"——构成了 LiDAR 回环检测的标准分层架构。ScanContext 的列移位机制也被后续的 Radar ScanContext、Stable Triangle Descriptor 等其他模态的回环描述子所借鉴。

## 相关页面

- [[方法-ScanContext]] — ScanContext 实现细节和 Agent 实现提示
- [[概念-回环检测方法]] — 回环检测方法全景对比（视觉 / LiDAR / 雷达）
- [[方法-LiDAR回环检测管线]] — LiDAR 回环检测的完整处理链
- [[方法-四阶段回环验证]] — ScanContext → 欧式距离 → Quatro → GICP 多级验证管道
- [[方法-Intensity Scan Context]] — 强度编码变体，适配 4D 雷达
- [[方法-Branch-and-Bound回环检测]] — Cartographer 的回环方法（BBS 分支定界）
- [[概念-位姿图优化]] — 回环约束在因子图后端的注入方式
- [[算法-LIO-SAM]] — LIO-SAM 的原始 ICP 回环 vs ScanContext 改进
- [[算法-FAST-LIO-SAM-SC-QN]] — 基于 ScanContext 的四层回环验证实现
