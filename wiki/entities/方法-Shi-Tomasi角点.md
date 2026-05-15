---
tags: [视觉特征, 角点检测, VINS-Mono]
sources:
  - wiki/sources/2026-04-29-lvi_sam_analysis.md
  - wiki/sources/2026-04-29-vins-fusion-analysis-analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-视觉特征跟踪]]
type: entity
---

# Shi-Tomasi角点

> VINS-Mono 视觉前端特征提取算法：通过 `cv::goodFeaturesToTrack()` 提取 150 个 Shi-Tomasi 角点，通过 KLT 光流金字塔跟踪。

> **⚠ 本页面内容已合并至 [[方法-视觉特征跟踪]]，保留此页仅供历史引用。请以主页面为准。**

## 判定原理

Shi-Tomasi 角点使用图像梯度二阶矩阵的最小特征值作为角点质量分数。只有当两个主方向上的梯度都足够强时，最小特征值才大；边缘点只有一个方向梯度强，会被压低分数。因此它比单纯梯度阈值更适合稀疏特征跟踪。

## 在 LVI-SAM 中的使用
- 每帧提取 `MAX_CNT=150` 个角点
- KLT 金字塔跟踪：搜索窗口 21×21
- 外点剔除：基本矩阵 RANSAC（`cv::findFundamentalMat()`）
- 网格覆盖保证新特征均匀分布（`setMask()`）

## vs 直接法
- **优点**：对光照变化不敏感，匹配速度快
- **缺点**：弱纹理场景角点不足，快速运动和大视角变化下跟踪易丢失

## 工程边界

Shi-Tomasi 只是“选点”，不是完整视觉前端。系统还需要金字塔 KLT、前后向一致性、RANSAC、视差筛选和关键帧策略。若相机曝光剧烈变化或运动模糊，角点质量分数仍可能较高但跟踪会失败，因此后续几何验证不可省略。

## Agent 实现提示

### 适用场景

当 Agent 需要实现稀疏视觉前端的补点逻辑时使用本提示：已有特征先由 KLT 延续，数量不足时再用 Shi-Tomasi 在 mask 允许区域补充新角点，保证空间分布和长期跟踪优先级。

### 输入输出契约

- **输入**：当前灰度图、已有跟踪点 `cur_pts`、点 ID、跟踪时长 `track_cnt`、最大点数 `MAX_CNT`、最小间距 `MIN_DIST`。
- **输出**：新增角点列表 `n_pts`，以及更新后的点集、ID 和 mask。
- **失败契约**：图像为空、mask 类型不是 `CV_8UC1`、可补点数量小于等于 0 时，不调用检测器；不要把新增点直接标为已三角化点。

### 实现骨架（伪代码）

```python
def replenish_shi_tomasi_features(gray, tracked_points, ids, track_counts):
    mask = full_white_mask(gray.size)
    tracked_points, ids, track_counts = keep_long_tracks_first(
        tracked_points, ids, track_counts, min_distance=MIN_DIST, mask=mask
    )

    need = MAX_CNT - len(tracked_points)
    if need <= 0:
        return []

    new_points = good_features_to_track(
        gray,
        max_corners=need,
        quality_level=0.01,
        min_distance=MIN_DIST,
        mask=mask,
    )
    return new_points
```

### 关键源码片段

`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L55-L84`

```cpp
void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}
```

`raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp:L169-L189`

```cpp
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());
```

### 实现注意事项

- mask 不是可选优化项；没有空间抑制会导致角点集中在高纹理区域，后续 PnP/三角化退化。
- `qualityLevel=0.01` 和 `MIN_DIST` 是工程参数，应按图像分辨率和目标特征数调参。
- Shi-Tomasi 只负责检测，新增点还需要分配 ID、去畸变、速度估计和后续 KLT/RANSAC 验证。
- 优先保留长跟踪点能提升滑窗约束质量，但会牺牲局部新纹理响应速度。

### 源码检索锚点

- `raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp`：`setMask`、`goodFeaturesToTrack`、`track_cnt`
- `raw/codes/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp`：`calcOpticalFlowPyrLK`、`rejectWithF`
- `raw/codes/ORB_SLAM3/src/ORBextractor.cc`：可对照 ORB 金字塔特征提取的分布策略

## 相关页面

- [[方法-视觉特征跟踪]]
- [[2026-04-29-vins-fusion-analysis-analysis|VINS-Fusion]]
- [[算法-LVI-SAM]]
- [[VIO方案对比]]
- [[2026-04-29-external-primary-source-check]]
