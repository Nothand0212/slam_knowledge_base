---
type: entity
tags: [Ceres, 粗差剔除, 两轮优化, GNSS, IC-GVINS, OB_GINS, 鲁棒估计]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc
  - raw/codes/OB_GINS/src/ob_gins.cc
superseded-by: [[方法-鲁棒估计方法族]]
---

> 本页内容已归并至 [[方法-鲁棒估计方法族]]。

# Ceres 两轮优化粗差剔除

> 利用 Ceres `EvaluateResidualBlock` 在第一轮优化后用卡方检验识别粗差（outlier），移除/重赋权后在第二轮得到干净的估计。

## 问题背景

GNSS 观测在多径、NLOS（非视距）等场景下存在大量粗差。直接最小二乘会严重偏离真值：

- **GNSS 位置残差**：单点定位误差在恶劣环境下可达 10-50m
- **视觉重投影残差**：误匹配、运动模糊导致的 outlier 特征点
- **Huber Loss 的局限**：Huber 核函数对极大粗差的抑制能力有限，卡方检验提供的统计判定更为严格

## 核心思路

**Round 1（含 Huber）** → 拟合并获得初步状态 → 评价每个因子的标准化残差 → 剔除/重赋权粗差 → **Round 2（无核函数）** → 得到最终结果。

两个开源项目（IC-GVINS 和 OB_GINS）都采用此模式，实现了高度一致的接口。

## IC-GVINS 的两轮优化

### 整体流程

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1130-L1238`：

```
gvinsOptimization():
    1. 构建 Problem，添加所有因子（State, GNSS, IMU, Visual）
    2. Round 1：LM 优化（少量迭代，带 Huber 核）
    3. 粗差检测：
        a. gnssOutlierCullingByChi2()    → 卡方检验 GNSS 残差
        b. removeReprojectionFactorsByChi2() → 剔除视觉粗差因子
    4. 移除全部 GNSS 因子，重新添加（不带 Huber）
    5. Round 2：LM 优化（更多迭代，无核函数）
    6. 更新参数，gvinsOutlierCulling() 移除路标点级别粗差
```

### 关键参数设置

```cpp
static int first_num_iterations  = optimize_num_iterations_ / 4;   // Round 1: ~25% 迭代
static int second_num_iterations = optimize_num_iterations_ - first_num_iterations; // Round 2: ~75% 迭代
```

第一轮只用少量迭代，因为含粗差时 Hessian 近似不可靠；第二轮用大部分迭代精细收敛。

### GNSS 粗差检测

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1241-L1267`：

```cpp
void GVINS::gnssOutlierCullingByChi2(ceres::Problem &problem,
                                     vector<pair<ResidualBlockId, GNSS *>> &residual_block) {
    double chi2_threshold = 7.815;  // 自由度 3, α=0.05 的卡方临界值

    for (auto &block : residual_block) {
        double cost;
        problem.EvaluateResidualBlock(block.first, false, &cost, nullptr, nullptr);
        double chi2 = cost * 2;  // Ceres cost = 1/2 * ||r||²

        if (chi2 > chi2_threshold) {
            double scale = sqrt(chi2 / chi2_threshold);
            block.second->std *= scale;  // 重赋权：放大噪声标准差
            outliers_counts++;
        }
    }
}
```

卡方阈值 `7.815` 对应 $\chi^2_3(0.05)$：GNSS 位置残差 3 个自由度，置信度 95%。

### 视觉重投影粗差检测

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1269-L1297`：

```cpp
int GVINS::removeReprojectionFactorsByChi2(ceres::Problem &problem,
                                           vector<ResidualBlockId> &residual_ids, double chi2) {
    double cost;
    vector<ResidualBlockId> outlier_residual_ids;  // 先收集再删除

    for (auto &id : residual_ids) {
        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
        if (cost * 2.0 > chi2) {
            outlier_residual_ids.push_back(id);  // 收集粗差
        }
    }

    for (auto &id : outlier_residual_ids) {
        problem.RemoveResidualBlock(id);  // 从 Problem 中移除
    }
    return outlier_features;
}
```

注意两阶段：先收集所有粗差 ID，再统一删除。在遍历时直接删除会导致迭代器失效。

$\chi^2_2(0.05) = 5.991$ 是重投影误差的阈值（2 自由度：像素 x, y）。

### Round 1 与 Round 2 之间

`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1190-L1208`：

```cpp
    // 粗差检测和剔除
    {
        gnssOutlierCullingByChi2(problem, gnss_redisual_block);       // GNSS 粗差 → 重赋权
        removeReprojectionFactorsByChi2(problem, residual_ids, 5.991); // 视觉粗差 → 删除

        // 移除所有旧 GNSS 因子（含 Huber 核的）
        for (auto &block : gnss_redisual_block) {
            problem.RemoveResidualBlock(block.first);
        }

        // 重新添加 GNSS 因子（无 Huber，但 std 已更新）
        addGnssFactors(problem, false);  // false = 不加核函数
    }

    // 第二次优化
    {
        options.max_num_iterations = second_num_iterations;
        solver.Solve(options, &problem, &summary);
    }
```

## OB_GINS 的两轮优化

OB_GINS 与 IC-GVINS 同源（i2Nav Group），结构几乎一致：

`raw/codes/OB_GINS/src/ob_gins.cc:L359-L417`：

```cpp
    // Round 1：带核函数求解
    options.max_num_iterations = num_iterations / 4;
    solver.Solve(options, &problem, &summary);

    // 粗差检测
    if (is_outlier_culling && !gnss_residualblock_id.empty()) {
        double chi2_threshold = 7.815;  // 同上：χ²₃(0.05)

        set<double> gnss_outlier;
        for (size_t k = 0; k < gnsslist.size(); k++) {
            double cost;
            problem.EvaluateResidualBlock(gnss_residualblock_id[k].second,
                                          false, &cost, nullptr, nullptr);
            double chi2 = cost * 2;

            if (chi2 > chi2_threshold) {
                gnss_outlier.insert(gnss_residualblock_id[k].first);
                double scale = sqrt(chi2 / chi2_threshold);
                gnsslist[k].std *= scale;  // 重赋权
            }
        }

        // 移除旧 GNSS 因子，重新添加（无核函数）
        for (const auto &block : gnss_residualblock_id) {
            problem.RemoveResidualBlock(block.second);
        }
        // ... 重新 addGnssFactors (无 Huber)
    }

    // Round 2：充分迭代
    options.max_num_iterations = num_iterations * 3 / 4;
    solver.Solve(options, &problem, &summary);
```

OB_GINS 相较 IC-GVINS 的简化：
- 只做 GNSS 粗差剔除（没有视觉重投影因子）
- 使用 `std::unordered_set<double>` 记录粗差时间戳用于日志，但不影响算法逻辑

## 数学原理

### 标准化残差与卡方检验

对于残差向量 $\mathbf{r}_i \in \mathbb{R}^m$，Ceres 的 `cost` 定义为：

$$\text{cost} = \frac{1}{2} \|\mathbf{r}_i\|^2$$

构造检验统计量：

$$\chi^2 = 2 \cdot \text{cost} = \|\mathbf{r}_i\|^2$$

在正态假设下 $\mathbf{r}_i \sim \mathcal{N}(0, \Sigma_i)$，若信息矩阵 $\Omega_i = \Sigma_i^{-1}$ 已在因子中正确编码，则 $\|\mathbf{r}_i\|_{\Omega_i}^2 \sim \chi^2_m$。

**判定**：若 $\chi^2 > \chi^2_m(\alpha)$（自由度 $m$ 的 $\alpha$ 分位数），则拒绝 "该观测为内点" 的假设。

常用阈值：
| 自由度 $m$ | $\alpha=0.05$ | $\alpha=0.01$ | 对应观测 |
|-----------|---------------|---------------|---------|
| 1 | 3.841 | 6.635 | 点到面距离 |
| 2 | 5.991 | 9.210 | 重投影误差 |
| 3 | 7.815 | 11.345 | GNSS 位置 |
| 6 | 12.592 | 16.812 | 完整位姿残差 |

### 重赋权 vs 删除

| 策略 | 方法 | 适用于 |
|------|------|--------|
| **重赋权** | `std *= sqrt(chi² / threshold)` | GNSS：粗差可能是信号退化而非完全错误 |
| **删除** | `RemoveResidualBlock(id)` | 视觉重投影：误匹配应完全移除 |

重赋权的物理含义：如果 $\chi^2$ 是阈值的 $k$ 倍，则将观测噪声扩大 $\sqrt{k}$ 倍，等效于降低该观测在优化中的权重。

## 伪代码

```python
def two_round_optimization(problem, factors):
    # Round 1: 带 Huber 核
    solve(problem, max_iterations=N // 4)

    # 粗差检测
    outliers = []
    for factor in factors:
        cost = problem.EvaluateResidualBlock(factor.id)
        chi2 = cost * 2
        if chi2 > chi2_threshold(α=0.05, df=factor.dof):
            if factor.type == GNSS:
                factor.std *= sqrt(chi2 / threshold)  # 重赋权
            outliers.append(factor)

    # 重建问题
    for factor in outliers:
        problem.RemoveResidualBlock(factor.id)
    re_add_factors(outliers, use_loss_function=False)  # 无核

    # Round 2: 无核，纯最小二乘
    solve(problem, max_iterations=N * 3 // 4)
```

## Agent 实现提示

### 适用场景
- 含 GNSS 观测的融合导航（多径、NLOS 环境）
- 视觉 SLAM 外点剔除（重投影误差 > 阈值）
- 任何需要在大比例粗差环境中进行鲁棒估计的最小二乘问题

### 输入输出契约
- **输入**：Ceres Problem 对象、因子列表（含 ResidualBlockId 和观测数据指针）
- **Round 1 输出**：初步状态估计、每个因子的 cost 值
- **Round 2 输入**：剔除粗差后的 Problem（无核函数）
- **最终输出**：干净的状态估计

### 实现注意事项
- 第一轮使用 **Huber Loss**（而非 Cauchy）：Huber 保持对小残差的二次形式，利于内点收敛
- 第二轮去掉所有核函数：因为粗差已被剔除，使用普通最小二乘得到理论最优解
- **不要在第一轮过程中删除 ResidualBlock**：先收集再统一删除（`RemoveResidualBlock` 会改变内部索引）
- 启用 `problem_options.enable_fast_removal = true` 加速删除操作
- `EvaluateResidualBlock` 的 cost 是 $\frac{1}{2}\|r\|^2$，要乘 2 才得到卡方统计量
- GNSS 重赋权后，Round 2 不加核函数但**信息矩阵已衰减**，等价于软加权

### 源码检索锚点
- IC-GVINS 整体：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1130-L1238`
- IC-GVINS GNSS 卡方检验：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1241-L1267`
- IC-GVINS 视觉卡方剔除：`raw/codes/IC-GVINS/ic_gvins/ic_gvins/ic_gvins.cc:L1269-L1297`
- OB_GINS 两轮优化：`raw/codes/OB_GINS/src/ob_gins.cc:L359-L417`

## 相关页面

- [[组件-Ceres-Solver]]
- [[方法-GNSS 位置残差因子]]
- [[方法-Ceres解析雅可比CostFunction]]
- [[方法-Ceres-Manifold迁移指南]]
