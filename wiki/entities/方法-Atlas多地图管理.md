---
type: entity
tags: [Atlas, 多地图, ORB-SLAM3, 子图管理, 地图合并, 重定位]
created: 2026-05-15
updated: 2026-05-15
sources:
  - raw/codes/ORB_SLAM3/src/Atlas.cc
  - raw/codes/ORB_SLAM3/src/LoopClosing.cc
  - raw/codes/ORB_SLAM3/src/Map.cc
---

# Atlas 多地图管理

> ORB-SLAM3 的核心创新机制：Atlas 管理器维护多个独立子地图，支持地图创建、激活切换、回环合并与全局修正。

## 系统架构

### 数据结构

`Map` 类 (`raw/codes/ORB_SLAM3/include/Map.h:L41-L207`) 是一个独立子图，包含：

- **关键帧集合** `mspKeyFrames` (`std::set<KeyFrame*>`)：该地图的所有关键帧
- **地图点集合** `mspMapPoints` (`std::set<MapPoint*>`)：所有地图点
- **地图 ID** `mnId`：全局唯一递增 ID（`Map::nNextId` 静态变量维护）
- **激活状态** `mIsInUse`：标记地图是否为当前活跃地图
- **坏地图标记** `mbBad`：标记被合并后应当废弃的地图
- **初始关键帧 ID** `mnInitKFid`：地图创建时第一个关键帧的 ID
- **惯导标志** `mbIsInertial`, `mbImuInitialized`：惯性传感器状态
- **位姿图原始点** `mvpKeyFrameOrigins`：Essential Graph 传播的根节点

`Atlas` 类 (`raw/codes/ORB_SLAM3/include/Atlas.h:L49-L169`) 管理所有地图：

- 使用 `std::set<Map*>` 存储多地图（按指针排序）
- `mpCurrentMap` 指向当前活跃地图
- `mspBadMaps` 存放待清理的废弃地图
- 全局相机列表 `mvpCameras`、关键帧数据库 `mpKeyFrameDB`、词袋词汇表 `mpORBVocabulary` 共享于所有地图

### 构图设计思想

- **每个 Map 是独立的自洽 SLAM 子图**：有自己的位姿图、地图点和关键帧
- **Atlas 是管理容器**：不参与子图内部细节，只提供地图级别的生命周期管理
- **共享基础设施**：所有地图共享同一个关键帧数据库和 ORB 词袋，因此重定位和回环可以在跨地图之间发生
- **地图 ID 强制为静态全局计数器**：即使地图被删除，ID 不回收，保证序列化/日志的可追溯性

## 子地图生命周期

### CreateNewMap：主动创建新地图

`raw/codes/ORB_SLAM3/src/Atlas.cc:L58-L77`

当跟踪丢失时触发，或将当前地图存为已存储、创建一个新地图从零开始建图：

```cpp
// 伪代码
void Atlas::CreateNewMap():
    锁住 Atlas 互斥锁
    if mpCurrentMap 存在:
        // 记录当前最大 KF ID，新地图的初始 KF ID 为其 +1
        更新 mnLastInitKFidMap = max(mnLastInitKFidMap, mpCurrentMap->GetMaxKFid() + 1)
        mpCurrentMap->SetStoredMap()   // 标记为"已存储"（不再活跃）
    创建新 Map(mnLastInitKFidMap)
    新地图 -> SetCurrentMap()          // 标记为"活跃"
    mspMaps.insert(newMap)
    mpCurrentMap = newMap
```

关键设计：`mnLastInitKFidMap` 记录上一次初始化地图时的最大关键帧 ID，确保新地图的第一个关键帧 ID 不会与已有地图冲突。

### ChangeMap：切换活跃地图

`raw/codes/ORB_SLAM3/src/Atlas.cc:L79-L89`

```cpp
// 伪代码
void Atlas::ChangeMap(Map* pMap):
    锁住 Atlas 互斥锁
    if mpCurrentMap 存在:
        mpCurrentMap->SetStoredMap()
    mpCurrentMap = pMap
    mpCurrentMap->SetCurrentMap()
```

仅改变指向，不移动关键帧或地图点。由调用方（MergeMaps 或系统控制）确保目标地图有效。

### 地图失效与清理

- `SetMapBad(pMap)` (`raw/codes/ORB_SLAM3/src/Atlas.cc:L260-L266`)：将地图从活跃集合 `mspMaps` 移除，移入 `mspBadMaps`
- `RemoveBadMaps()` (`raw/codes/ORB_SLAM3/src/Atlas.cc:L268-L276`)：清空坏地图容器（内存未释放，由序列化管理）

## MergeMaps：回环驱动的地图合并

### 触发条件

当 Tracking 检测到当前关键帧与另一个地图中的关键帧存在共视关系（通过共视匹配 `mpMergeMatchedKF`），且该关联经过几何验证（Sim3 求解），LoopClosing 线程调用 `MergeLocal()` 或 `MergeLocal2()`。

### 合并流程（MergeLocal）

`raw/codes/ORB_SLAM3/src/LoopClosing.cc:L1215-L1780`

整体分六个阶段：

**阶段一：停止并发线程**
- 如果正在运行全局 BA，终止它并标记需重新启动
- 请求 LocalMapping 停止，清空其队列

**阶段二：构建局部窗口（Welding Area）**
- 当前地图的局部窗口 `spLocalWindowKFs`：以 `mpCurrentKF` 为中心，扩展到最佳共视关键帧（至少 25 个），同时收集这些关键帧观测的地图点 `spLocalWindowMPs`
- 合并对端地图的局部窗口 `spMergeConnectedKFs`：以 `mpMergeMatchedKF` 为中心，同样扩展共视邻域
- 惯性模式下，额外沿时间链（`mPrevKF`/`mNextKF`）扩展

**阶段三：计算校正变换**
```cpp
// 伪代码
// 对当前地图局部窗口的每个关键帧 pKFi:
if pKFi == mpCurrentKF:
    g2oCorrectedSiw = mg2oMergeScw   // 直接使用 Sim3 校正
else:
    Tiw = pKFi->GetPose()
    Tic = Tiw * Twc                   // 当前帧到 KF 的相对位姿
    g2oCorrectedSiw = g2oSic * mg2oMergeScw  // 传播校正

// 对地图点同样传播校正：
corrected_world = T_corrected_ref_world * T_uncorrected_ref_world⁻¹ * uncorrected_world
```

**阶段四：焊接局部窗口（关键帧+地图点迁移）**
```cpp
// 伪代码
for pKFi in spLocalWindowKFs:
    保存原始位姿 pKFi->mTcwBefMerge
    设置为校正后位姿 pKFi->SetPose(pKFi->mTcwMerge)
    pKFi->UpdateMap(pMergeMap)       // 更新关键帧的地图归属
    pMergeMap->AddKeyFrame(pKFi)
    pCurrentMap->EraseKeyFrame(pKFi)

for pMPi in spLocalWindowMPs:
    pMPi->SetWorldPos(mPosMerge)     // 校正后的世界坐标
    pMPi->UpdateMap(pMergeMap)
    pMergeMap->AddMapPoint(pMPi)
    pCurrentMap->EraseMapPoint(pMPi)

// 激活合并后的地图，标记被合并地图为 Bad
mpAtlas->ChangeMap(pMergeMap)
mpAtlas->SetMapBad(pCurrentMap)
pMergeMap->ChangeId(pCurrentMap->GetId())  // 继承原活跃地图的 ID
```

**阶段五：Essential Graph 重连**
- 翻转当前关键帧的生成树父子关系：`mpCurrentKF` 原来是 `pNewParent` 的父节点，现在 `pNewParent` 成为 `mpCurrentKF` 的父节点
- `mpMergeMatchedKF` 成为 `mpCurrentKF` 的新父节点
- 本质上是把"当前地图→合并地图"的方向翻转，`mpCurrentKF` 的位姿以 `mpMergeMatchedKF` 为参照

**阶段六：焊接 BA + 非局部区域校正**
- 对焊接区域进行局部 BA（仅优化局部窗口内的关键帧和地图点）
- 对剩余关键帧（非局部窗口）应用 `Sim3` 传播校正：`corrected_Tiw = Tic * mg2oMergeScw`
- 剩余地图点也传播校正并迁移到合并地图
- 对于非单目传感器，运行 `OptimizeEssentialGraph` 进一步优化全局位姿图

**阶段七：恢复并发**
- 释放 LocalMapping
- 如果地图之前正在运行全局 BA，重新启动
- 添加合并边 `AddMergeEdge` 记录两关键帧的合并关系

### 伪代码（MergeLocal 完整流程）

```python
# 伪代码：MergeLocal 主流程
def MergeLocal():
    # 1. 停止并发
    if isRunningGBA():
        stop_and_detach_GBA()
        bRelaunchBA = True
    mpLocalMapper.RequestStop()
    wait_until_stopped()
    mpLocalMapper.EmptyQueue()

    # 2. 构建局部窗口
    pCurrentMap = mpCurrentKF.GetMap()
    pMergeMap = mpMergeMatchedKF.GetMap()

    spLocalWindowKFs = build_covisible_window(mpCurrentKF, numTemporalKFs=25)
    spLocalWindowMPs = collect_map_points(spLocalWindowKFs)

    spMergeConnectedKFs = build_covisible_window(mpMergeMatchedKF, numTemporalKFs=25)
    spMapPointMerge = collect_map_points(spMergeConnectedKFs)

    # 3. 计算 Sim3 校正
    Twc = mpCurrentKF.GetPoseInverse()
    g2oCorrectedScw = mg2oMergeScw
    for pKFi in spLocalWindowKFs:
        传播 Sim3 校正位姿
    for pMPi in spLocalWindowMPs:
        传播校正世界坐标

    # 4. 焊接迁移
    with_lock(pCurrentMap, pMergeMap):
        for pKFi in spLocalWindowKFs:
            pKFi.SetPose(corrected_pose)
            pKFi.UpdateMap(pMergeMap)
            pMergeMap.AddKeyFrame(pKFi)
            pCurrentMap.EraseKeyFrame(pKFi)

        for pMPi in spLocalWindowMPs:
            pMPi.SetWorldPos(merged_pos)
            pMPi.UpdateMap(pMergeMap)
            pMergeMap.AddMapPoint(pMPi)
            pCurrentMap.EraseMapPoint(pMPi)

        mpAtlas.ChangeMap(pMergeMap)
        mpAtlas.SetMapBad(pCurrentMap)
        pMergeMap.ChangeId(pCurrentMap.GetId())

    # 5. Essential Graph 重连
    pCurrentMap.GetOriginKF().SetFirstConnection(False)
    mpCurrentKF.ChangeParent(mpMergeMatchedKF)

    # 6. 焊接 BA
    Optimizer.MergeInertialBA(
        mpCurrentKF, mpMergeMatchedKF, vCorrectedSim3
    )  # or LocalBundleAdjustment for non-inertial

    # 7. 非局部区域迁移 + EssentialGraph 优化
    for remaining KFs/MPs in pCurrentMap:
        传播 Sim3 校正 -> 迁移到 pMergeMap

    if not MONOCULAR:
        Optimizer.OptimizeEssentialGraph(
            mpCurrentKF, vpMergeConnectedKFs,
            vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs
        )

    # 8. 恢复
    mpLocalMapper.Release()
    if bRelaunchBA:
        启动 RunGlobalBundleAdjustment(pMergeMap, nLoopKF)

    mpAtlas.RemoveBadMaps()
```

### MergeLocal2：简化合并

`raw/codes/ORB_SLAM3/src/LoopClosing.cc:L1783-L1832`

与 MergeLocal 相似，使用更小的局部窗口（11 个关键帧），适用于检测到的是回环而非完全不同的地图合并时，流程更轻量。

## LoopCorrectConn：全局 BA 与 Essential Graph 传播

`raw/codes/ORB_SLAM3/src/LoopClosing.cc:L2268-L2400`

### 全局 BA 流程

```cpp
// 伪代码
void RunGlobalBundleAdjustment(Map* pActiveMap, nLoopKF):
    if 非惯性:
        Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF)
    else:
        Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA)
```

非惯性模式：迭代 10 次全局 BA（优化所有关键帧位姿 + 所有地图点世界坐标）。惯性模式：7 次优化，含 IMU 预积分约束。

### Essential Graph 传播

全局 BA 期间 LocalMapping 可能创建了新的关键帧，这些新关键帧没有参与 BA，需要把 BA 的校正传播到它们：

```cpp
// 伪代码
// 从第一帧根节点开始，沿生成树逐层传播
list<KeyFrame*> lpKFtoCheck = pActiveMap->mvpKeyFrameOrigins

while !lpKFtoCheck.empty():
    pKF = lpKFtoCheck.front()
    lpKFtoCheck.pop_front()
    Twc = pKF->GetPoseInverse()

    for each child pChild of pKF:
        if pChild->mnBAGlobalForKF != nLoopKF:  // 未参与 BA
            Tchildc = pChild->GetPose() * Twc
            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA  // 传播校正
            Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3()
            if 有速度:
                pChild->mVwbGBA = Rcor * pChild->GetVelocity()
            pChild->mnBAGlobalForKF = nLoopKF
            lpKFtoCheck.push_back(pChild)

    // 同时传播地图点校正
    for each MapPoint observed by pKF:
        if pMP->mnBAGlobalForKF != nLoopKF:
            pMP->SetWorldPos(pKF->mTcwGBA * old_camera_pos_of_MP)
            pMP->UpdateNormalAndDepth()
            pMP->mnBAGlobalForKF = nLoopKF
```

校正传播完成后，将校正后的位姿写入：
```cpp
pKF->mTcwBefGBA = pKF->GetPose()
pKF->SetPose(pKF->mTcwGBA)
```

## 重定位跨地图搜索

当 Tracking 丢失时，系统触发重定位流程。Atlas 的关键帧数据库 `mpKeyFrameDB` 是所有地图共享的，因此：

1. **重定位候选搜索**：在当前帧的 ORB 词袋向量空间查找相似关键帧，候选可以来自任意地图
2. **多地图候选项**：对每个候选关键帧尝试 PnP + RANSAC 求解位姿
3. **地图切换**：如果候选所属地图不是当前活跃地图，系统可以选择切换到该地图继续跟踪（`ChangeMap`），或创建新地图 (`CreateNewMap`)
4. **回环检测**：LoopClosing 也在所有地图的关键帧间进行共视匹配，发现跨地图关联即触发地图合并

## 位姿传播的数学基础

ORB-SLAM3 中地图合并的关键数学工具是 **Sim3** 变换（相似变换，包含尺度 s）：

$$\text{Sim}(3) = \left\{ S = \begin{bmatrix} s\mathbf{R} & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix} \;\middle|\; \mathbf{R} \in SO(3), \mathbf{t} \in \mathbb{R}^3, s \in \mathbb{R}^+ \right\}$$

对地图点位置传播：$\mathbf{P}_w^{\text{corrected}} = \mathbf{T}_{\text{corrected, ref}}^{\text{world}} \cdot \left(\mathbf{T}_{\text{uncorrected, ref}}^{\text{world}}\right)^{-1} \cdot \mathbf{P}_w^{\text{uncorrected}}$

对关键帧速度传播：$\mathbf{v}^{\text{corrected}} = \mathbf{R}_{\text{cor}} \cdot \mathbf{v}^{\text{uncorrected}}$，其中 $\mathbf{R}_{\text{cor}} = \text{spin}(T_{\text{corrected}}^{\text{world}}) \cdot \text{spin}(T_{\text{uncorrected}}^{\text{world}})^{-1}$

## 相关页面

- [[算法-ORB-SLAM3]]
- [[概念-位姿图优化]]
- [[概念-回环检测方法]]
- [[方法-多会话坐标系对齐]]
- [[方法-Anchor 节点位姿图优化]]
