---
type: entity
tags: [位姿图优化, 坐标变换, 多会话, GTSAM]
created: 2026-04-29
updated: 2026-05-15
sources:
  - wiki/sources/2026-04-28-lt-mapper-analysis.md
---

# Anchor 节点位姿图优化

> 多会话建图中的坐标系对齐模式：为每个会话引入 anchor 位姿，让局部轨迹在统一中心坐标系下联合优化。

## 定义

lt-mapper 的核心创新机制：每个会话引入一个 anchor 节点，通过四变量因子 `BetweenFactorWithAnchoring<Pose3>` 将各会话独立的局部坐标系变换到统一中心坐标系后进行联合位姿图优化。

## 核心特征

- 四变量因子 (key1, key2, anchor_key1, anchor_key2)
- 误差计算：hx1 = anchor_p1 ⊕ p1（局部→中心），hx2 = anchor_p2 ⊕ p2，e = Log(measured⁻¹ · (hx1 ⊖ hx2))
- 对四个变量均解析计算雅可比，使用 GTSAM traits<T>::Compose/Between 自动推导
- 中心会话 anchor 强先验 (priorNoise=1e-12)，查询会话 anchor 弱先验 (largeNoise: π²/1e8)
- 优势：避免逐步 ICP 配准漂移累积，所有会话在统一坐标系联合优化

## BetweenFactorWithAnchoring 数学形式

### 因子结构

该因子继承 `NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE>`，连接 4 个变量：

| 变量 | 含义 | 符号 |
|------|------|------|
| key1 | Session i 中帧 k 的局部位姿 | $p_i^k$ |
| key2 | Session j 中帧 l 的局部位姿 | $p_j^l$ |
| anchor_key1 | Session i 的 anchor 变换 | $a_i$ |
| anchor_key2 | Session j 的 anchor 变换 | $a_j$ |

### 误差函数

分三步计算，`raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h:L86-L99`：

**Step 1** — 局部→中心坐标系变换：

$$h_1 = a_i \circ p_i^k \quad (\text{Compose: } h_1 = T_a^i \cdot T_l^{i,k})$$

$$h_2 = a_j \circ p_j^l \quad (\text{Compose: } h_2 = T_a^j \cdot T_l^{j,l})$$

其中 $\circ$ 对应 GTSAM `traits<T>::Compose`（SE(3) 复合），同时解析计算雅可比：
- $\mathbf{H}_1^{\text{step1}} = \frac{\partial h_1}{\partial p_i^k}$（局部帧位姿雅可比）
- $\mathbf{H}_{a1}^{\text{step1}} = \frac{\partial h_1}{\partial a_i}$（anchor 雅可比）

**Step 2** — 中心坐标系下相对位姿：

$$h = h_1 \ominus h_2 = h_1^{-1} \cdot h_2$$

其中 $\ominus$ 对应 GTSAM `traits<T>::Between`，计算雅可比：
- $\mathbf{H}_1^{\text{step2}} = \frac{\partial (h_1 \ominus h_2)}{\partial h_1}$
- $\mathbf{H}_2^{\text{step2}} = \frac{\partial (h_1 \ominus h_2)}{\partial h_2}$

**Step 3** — 与测量值比较（切空间误差）：

$$\mathbf{e} = \text{Local}(T_{\text{measured}}, h) = \log\left(T_{\text{measured}}^{-1} \cdot h\right) \in \mathbb{R}^6$$

其中 $\log$ 是 SE(3) 上的对数映射，$\text{Local}$ 对应 GTSAM `traits<T>::Local`。

### 完整雅可比链

最终雅可比通过链式法则传递：

$$\frac{\partial \mathbf{e}}{\partial p_i^k} = \frac{\partial \mathbf{e}}{\partial h} \cdot \mathbf{H}_1^{\text{step2}} \cdot \mathbf{H}_1^{\text{step1}}$$

$$\frac{\partial \mathbf{e}}{\partial a_i} = \frac{\partial \mathbf{e}}{\partial h} \cdot \mathbf{H}_1^{\text{step2}} \cdot \mathbf{H}_{a1}^{\text{step1}}$$

对 $p_j^l$ 和 $a_j$ 同理。所有雅可比由 GTSAM 的 Lie 群 traits 自动解析推导，无需手动编写导数。

## lt-mapper 中的实现

### 源码锚点

| 组件 | 文件 | 行号 |
|------|------|------|
| BetweenFactorWithAnchoring 类定义 | `raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h` | L18-L127 |
| evaluateError 核心实现 | `raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h` | L86-L99 |
| anchor 初始化（先验添加） | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L565-L576 |
| 内部图构建（odom + loop 边） | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L579-L621 |
| SC 回环添加（使用四变量因子） | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L370-L416 |
| 全局 ID 生成 | `raw/codes/lt-mapper/ltslam/src/utility.cpp` | L37-L47 |
| 优化后位姿更新 | `raw/codes/lt-mapper/ltslam/src/Session.cpp` | L66-L88 |
| Session 数据结构 | `raw/codes/lt-mapper/ltslam/include/ltslam/Session.h` | L24-L68 |

### 关键设计：Anchor 先验的强弱策略

`raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L565-L576`

```cpp
void LTslam::initTrajectoryByAnchoring(const Session& _sess) {
    int anchor_idx = genAnchorNodeIdx(_sess.index_);
    if(_sess.is_base_session_) {
        // 中心会话：强先验，固定 anchor 在原点
        gtSAMgraph.add(PriorFactor<Pose3>(anchor_idx, poseOrigin, priorNoise));
    } else {
        // 查询会话：弱先验，允许 anchor 充分移动
        gtSAMgraph.add(PriorFactor<Pose3>(anchor_idx, poseOrigin, largeNoise));
    }
    initialEstimate.insert(anchor_idx, poseOrigin); // 初始化为单位位姿
}
```

**设计原理**：
- `priorNoise` 方差 ∼10⁻¹²：将中心 session 的 anchor 紧紧固定在原点，避免整个图发生刚体漂移（规范自由度）
- `largeNoise` 方差 ∼π²（旋转）、10⁸（平移）：使查询 session 的 anchor 在优化中几乎不受约束，仅通过跨 session 回环边确定其变换
- 所有 anchor 初始化为单位位姿有利于优化器从原点开始搜索

### 因子使用模式：内部边 vs 跨 session 边

**内部边**（同一 session 内的 odom/loop）：使用普通 `BetweenFactor<Pose3>`（2 变量）：

```cpp
// raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L613-L614
gtSAMgraph.add(BetweenFactor<Pose3>(from_global_idx, to_global_idx, relative, odomNoise));
```

**跨 session 边**（SC/RS 回环）：使用 `BetweenFactorWithAnchoring<Pose3>`（4 变量）：

```cpp
// raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L401-L404
gtSAMgraph.add(BetweenFactorWithAnchoring<Pose3>(
    genGlobalNodeIdx(target_sess, target_node),
    genGlobalNodeIdx(source_sess, source_node),
    genAnchorNodeIdx(target_sess),
    genAnchorNodeIdx(source_sess),
    relative_pose, robustNoise));
```

**为什么需要区分？** 内部边不涉及 anchor，因为同一个 session 内的所有帧共享同一个 anchor 变换。跨 session 边必须包含 anchor 变量，因为两个 session 的 anchor 不同，相对位姿必须在中心坐标系中表达。

### 优化流程中的因子图演化

`raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L79-L98` 主运行循环：

```
初始因子图：
  [anchor_1] (强先验)
  [anchor_2] (弱先验)
  [p1_0] [p1_1]--[p1_2]--...   (odom edges)
  [p2_0] [p2_1]--[p2_2]--...   (odom edges)
  (无跨 session 边，两图独立优化)

第一次优化后：
  [anchor_1] 固定在原点
  [anchor_2] 因弱先验几乎未移动
  (两图仍独立)

添加 SC 回环后：
  [anchor_1]--[p1_k]----[p2_l]--[anchor_2]
                  ↑ 四变量因子：(p1_k, p2_l, anchor_1, anchor_2)
  (两图通过跨 session 边连接，联合优化)

最终优化后：
  [anchor_1] 保持原点
  [anchor_2] 收敛到 T_actual (Session 2 在中心坐标系的真实变换)
  (所有帧位姿在中心坐标系对齐)
```

### ISAM2 增量优化

`raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L157-L184`

```cpp
void LTslam::optimizeMultisesseionGraph(bool _toOpt) {
    if(!_toOpt) return;
    isam->update(gtSAMgraph, initialEstimate);  // 增量添加新因子
    isam->update(); isam->update();             // 多次迭代
    isam->update(); isam->update();
    isam->update();                              // 共 5 次 update 确保收敛
    isamCurrentEstimate = isam->calculateEstimate();
    gtSAMgraph.resize(0);                        // 清空，准备下一轮增量添加
    initialEstimate.clear();
    updateSessionsPoses();                       // 将优化结果写入点云位姿
}
```

多次 `isam->update()` 调用确保增量优化收敛。`relinearizeSkip=1` 配置意味着每次新增因子后都会重新线性化受影响变量。

### 坐标系转换的关键代码

`raw/codes/lt-mapper/ltslam/src/Session.cpp:L66-L88`

```cpp
void Session::updateKeyPoses(const ISAM2* _isam, const Pose3& _anchor_transform) {
    auto isamCurrentEstimate = _isam->calculateEstimate();
    for (int node_idx = 0; node_idx < numPoses; ++node_idx) {
        int global_idx = genGlobalNodeIdx(index_, node_idx);
        Pose3 pose_self_coord = isamCurrentEstimate.at<Pose3>(global_idx);
        Pose3 pose_central_coord = _anchor_transform * pose_self_coord;
        // 写入 cloudKeyPoses6D（中心坐标系）
        cloudKeyPoses6D->points[node_idx].x = pose_central_coord.translation().x();
        // ... y, z, roll, pitch, yaw
    }
}
```

转换关系：$T_{\text{central}} = T_{\text{anchor}} \cdot T_{\text{local}}$，其中 $T_{\text{anchor}} = T_a^i$ 是 session i 的 anchor 位姿。

### 信息增益引导的近邻回环选择

`raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L419-L448`

RS（Radius Search）回环不仅依赖 ICP，还使用**信息增益**选择最优目标节点：

```cpp
double calcInformationGainBtnTwoNodes(target_node, source_node):
    // 获取当前估计
    pose_s1 = estimate.at(target_global_idx)
    pose_s2 = estimate.at(source_global_idx)
    anchor_s1 = estimate.at(target_anchor_idx)
    anchor_s2 = estimate.at(source_anchor_idx)

    // 构造临时因子并计算雅可比
    loop_factor.evaluateError(pose_s1, pose_s2, anchor_s1, anchor_s2,
                              H_s1, H_s2, H_anchor_s1, H_anchor_s2)

    // 计算边缘协方差和增量信息
    S = noise_cov + H_s1*cov_s1*H_s1' + H_s2*cov_s2*H_s2'
    info_gain = 0.5 * log(det(S) / det(noise_cov))
    return info_gain  // 值越大，表示该回环边带来的信息量越大
```

选择信息增益最大的目标节点，确保每条新增的回环边都对图优化贡献最大化。

## Agent 实现提示

### 适用场景

- 多会话 LiDAR/视觉 SLAM 系统中，需要将多个独立 session 的局部轨迹统一到同一中心坐标系
- 需要实现增量式的多 session 联合位姿图优化（ISAM2 增量求解）
- 跨 session 的回环边（SC/RS 配准）需要同时约束两个 session 的 anchor 变量和关键帧位姿

### 输入输出契约

**initTrajectoryByAnchoring（anchor 初始化）**
- 输入：`Session` 对象（含 `is_base_session_` 标志）
- 输出：anchor 节点插入因子图（base session 用 `priorNoise ~1e-12` 强先验，查询 session 用 `largeNoise ~π²` 弱先验），初始估计为单位位姿 `poseOrigin`
- 前置条件：全局节点索引 `genAnchorNodeIdx(sess.index_)` 产生唯一整数 ID

**addSCLoops（跨 session 回环添加）**
- 输入：源/目标 session 的节点索引、SC 配准得到的相对位姿 `relative_pose`
- 输出：`BetweenFactorWithAnchoring<Pose3>` 四变量因子插入因子图
- 四变量：`(key1=target_node, key2=source_node, anchor_key1=target_anchor, anchor_key2=source_anchor)`

**optimizeMultisesseionGraph（增量优化）**
- 输入：`toOpt` 布尔标志
- 输出：`isamCurrentEstimate` 更新为最新估计值，`gtSAMgraph` 清空待下一轮增量添加
- 优化次数：`isam->update()` × 5 次确保收敛

### 实现骨架（伪代码）

```pseudo
class BetweenFactorWithAnchoring(Pose3):
    # 继承 NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE>
    # 四变量: p1, p2, anchor_p1, anchor_p2

    def evaluateError(p1, p2, anchor_p1, anchor_p2, H1, H2, aH1, aH2):
        # Step 1: 局部 → 中心坐标系
        hx1 = anchor_p1.Compose(p1)     # T_center = T_anchor * T_local
        hx2 = anchor_p2.Compose(p2)
        # Step 2: 中心坐标系相对位姿
        hx = hx1.Between(hx2)           # T_rel = hx1⁻¹ * hx2
        # Step 3: 与测量值比较（切空间误差）
        return measured.Local(hx)       # e = Log(T_measured⁻¹ * hx)

def addSessionGraph(session):
    # 1. 添加 anchor 节点（强弱先验）
    anchor_idx = genAnchorNodeIdx(session.index)
    if session.is_base_session:
        graph.add(PriorFactor(anchor_idx, identity, priorNoise=1e-12))
    else:
        graph.add(PriorFactor(anchor_idx, identity, largeNoise=π²/1e8))

    # 2. 添加 odom/loop 边（同一 session 内部）
    for each (from, to, relative_pose) in session.edges:
        graph.add(BetweenFactor(from, to, relative_pose, odomNoise))

    # 3. 添加跨 session 回环边（四个变量）
    for each (target, source, relative_pose) in cross_session_loops:
        graph.add(BetweenFactorWithAnchoring(
            target.node_idx, source.node_idx,
            target.anchor_idx, source.anchor_idx,
            relative_pose, robustNoise))

def optimizeMultisesseionGraph():
    isam.update(gtSAMgraph, initialEstimate)
    repeat(5): isam.update()           # 5次迭代确保收敛
    isamCurrentEstimate = isam.calculateEstimate()

def updateSessionPoses():
    for each frame in each session:
        T_local = isamCurrentEstimate.at(frame.global_idx)
        T_central = anchor_transform * T_local
        write_to_cloudKeyPoses6D(T_central)
```

### 关键源码片段

`raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h:L86-L99` — evaluateError 核心实现：
```cpp
Vector evaluateError(
    const T& p1, const T& p2, const T& anchor_p1, const T& anchor_p2,
    boost::optional<Matrix&> H1 = boost::none,
    boost::optional<Matrix&> H2 = boost::none,
    boost::optional<Matrix&> anchor_H1 = boost::none,
    boost::optional<Matrix&> anchor_H2 = boost::none) const
{
    T hx1 = traits<T>::Compose(anchor_p1, p1, anchor_H1, H1);
    T hx2 = traits<T>::Compose(anchor_p2, p2, anchor_H2, H2);
    T hx  = traits<T>::Between(hx1, hx2, H1, H2);
    return traits<T>::Local(measured_, hx);
}
```

`raw/codes/lt-mapper/ltslam/src/LTslam.cpp:L565-L576` — anchor 先验注入：
```cpp
void LTslam::initTrajectoryByAnchoring(const Session& _sess) {
    int this_session_anchor_node_idx = genAnchorNodeIdx(_sess.index_);
    if(_sess.is_base_session_) {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, priorNoise));
    } else {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, largeNoise));
    }
    initialEstimate.insert(this_session_anchor_node_idx, poseOrigin);
}
```

### 实现注意事项

1. **雅可比链自动推导**：GTSAM 的 `traits<T>::Compose`、`traits<T>::Between`、`traits<T>::Local` 自动解析计算所有解析雅可比和链式法则传递，**禁止手动编写**——否则与 GTSAM 的 Lie 群 conventions 可能不一致。
2. **先验强弱选择是关键**：base session 的 anchor 用 `priorNoise ~1e-12` 紧紧固定，防止整个因子图刚体漂移（gauge freedom）。查询 session 用 `largeNoise ~π²`，允许 anchor 在优化中充分移动。
3. **ISAM2 多轮 update 必须**：ISAM2 的增量优化单次 `update()` 可能未收敛，lt-mapper 连续调用 5 次。`relinearizeSkip=1` 确保新增因子后受影响变量重新线性化。
4. **内部边 vs 跨 session 边**：同一 session 内的 odom/loop 边用普通二变量 `BetweenFactor`（不涉及 anchor），跨 session 边必须用四变量 `BetweenFactorWithAnchoring`。
5. **信息增益选择回环**：RS 回环不是盲目添加最近邻，而是通过 `calcInformationGainBtnTwoNodes` 计算边缘协方差选择信息增益最大的目标节点。

### 源码检索锚点

| 组件 | 文件 | 行号 |
|------|------|------|
| BetweenFactorWithAnchoring 类定义 | `raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h` | L18–L127 |
| evaluateError 核心实现 | `raw/codes/lt-mapper/ltslam/include/ltslam/BetweenFactorWithAnchoring.h` | L86–L99 |
| anchor 先验初始化 | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L565–L576 |
| 内部图构建（odom + loop 边） | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L579–L621 |
| SC 回环（四变量因子） | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L370–L416 |
| RS 回环 + 信息增益选择 | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L419–L448 |
| ISAM2 增量优化 | `raw/codes/lt-mapper/ltslam/src/LTslam.cpp` | L157–L184 |
| 优化后位姿更新 | `raw/codes/lt-mapper/ltslam/src/Session.cpp` | L66–L88 |
| 全局 ID 生成 | `raw/codes/lt-mapper/ltslam/src/utility.cpp` | L37–L47 |
| Session 数据结构 | `raw/codes/lt-mapper/ltslam/include/ltslam/Session.h` | L24–L68 |

## 相关页面

- [[方法-多会话坐标系对齐]] — anchor 机制在多 session 后端中的完整应用流程
- [[方法-Atlas多地图管理]] — Atlas 的地图合并方案对比
- [[算法-lt-mapper]] — `BetweenFactorWithAnchoring.h:18-127`
- [[概念-位姿图优化]]
- [[架构-坐标系管理]]
- [[概念-因子图]]
- [[组件-GTSAM]]
