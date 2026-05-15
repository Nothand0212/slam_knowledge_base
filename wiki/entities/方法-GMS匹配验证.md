---
tags: [特征匹配, 几何验证, BEV-LSLAM]
sources:
  - wiki/sources/2026-04-29-bev_lslam_analysis.md
  - wiki/sources/2026-04-29-external-primary-source-check.md
created: 2026-04-29
updated: 2026-05-15
superseded-by: [[方法-视觉特征跟踪]]
type: entity
---

# GMS匹配验证

> Grid-based Motion Statistics：一种快速几何验证方法，利用运动平滑性假设滤除外点。

> **⚠ 本页面内容已合并至 [[方法-视觉特征跟踪]]，保留此页仅供历史引用。请以主页面为准。**

## 核心原理

假设正确匹配在空间上平滑分布（邻域内支持度高），错误匹配呈随机分布。通过统计网格内匹配支持度快速区分内外点，运算速度远优于 RANSAC。

GMS 不直接估计基础矩阵或单应矩阵，而是把图像划分成网格，统计相邻网格之间的匹配数量是否符合局部运动一致性。正确匹配通常会在相邻网格中形成连续支持，随机误匹配则很难形成稳定局部模式。

## 在 BEV-LSLAM 中的应用

- 对 height 和 intensity 描述符分别进行 GMS 滤波
- 替代传统 RANSAC 减少计算开销
- 后端 DBoW3 回环验证中也使用 GMS 替代基础矩阵验证

## 与 RANSAC 的关系

GMS 是基于局部运动一致性的统计过滤，不直接估计几何模型；RANSAC 则通过反复采样估计基础矩阵、单应矩阵或 PnP 模型。GMS 更快，适合大量初匹配的前置筛选；RANSAC 几何含义更明确，更适合最终验证。BEV-LSLAM 使用 GMS 是因为 BEV 图像运动近似平滑，网格统计假设更容易成立。

## 适用边界

- 适合大量 2D 特征匹配后的快速外点过滤。
- 对小视差、近似平面或 BEV 图像这类运动较平滑的场景有效。
- 不提供精确几何模型，后续仍需要 PnP、ICP、VGICP 或位姿图验证。
- 若场景中存在大面积重复纹理，网格统计可能给错误匹配过高支持。

## Agent 实现提示

### 适用场景

当 Agent 需要在 ORB/BFMatcher 初匹配之后快速过滤外点时使用本提示。它特别适合 BEV 图像、近似平面运动、回环候选初筛和需要低延迟的前端匹配验证。

### 输入输出契约

- **输入**：两帧关键点、两帧图像尺寸、初始 `DMatch` 列表、是否枚举尺度和旋转。
- **输出**：`vbInliers: List[bool]`、内点数量、可选的过滤后匹配列表。
- **失败契约**：初匹配太少、关键点坐标未归一化到各自图像尺寸、或图像尺寸错误时，返回空内点集合；不要把 GMS 内点直接当作几何位姿。

### 实现骨架（伪代码）

```python
def verify_matches_by_gms(kp1, size1, kp2, size2, raw_matches):
    norm1 = normalize_keypoints(kp1, size1)
    norm2 = normalize_keypoints(kp2, size2)

    best_mask = []
    best_count = 0
    for scale in allowed_scales:
        for rotation in allowed_rotations:
            motion_stats = assign_matches_to_grid_pairs(norm1, norm2, raw_matches, scale)
            cell_pairs = choose_best_right_cell_per_left_cell(motion_stats)
            mask = mark_matches_supported_by_neighbor_cells(
                motion_stats, cell_pairs, rotation
            )
            if count(mask) > best_count:
                best_count, best_mask = count(mask), mask
    return best_mask, best_count
```

### 关键源码片段

`raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp:L333-L344`

```cpp
        vector<cv::DMatch> matches_height,matches_intensity;
        cv::BFMatcher matcher(cv::NORM_HAMMING);

        matcher.match(BEV_now.description_height,BEV_last.description_height,matches_height);
        std::vector<bool> height_vbInliers;
        gms_matcher gms_height(BEV_now.keypoints_height, BEV_now.BEV_height.size(), BEV_last.keypoints_height, BEV_last.BEV_height.size(), matches_height);
        int height_lnliers = gms_height.GetInlierMask(height_vbInliers, false, true);

        matcher.match(BEV_now.description_intensity,BEV_last.description_intensity,matches_intensity);
        std::vector<bool> intensity_vbInliers;
        gms_matcher gms_intensity(BEV_now.keypoints_intensity, BEV_now.BEV_intensity.size(), BEV_last.keypoints_intensity, BEV_last.BEV_intensity.size(), matches_intensity);
        int intensity_lnliers = gms_intensity.GetInlierMask(intensity_vbInliers, false, true);
```

`raw/codes/BEV-LSLAM/orb_lio/src/orb_feature/gms_matcher.cc:L216-L241`

```cpp
void gms_matcher::AssignMatchPairs(int GridType) {

	for (size_t i = 0; i < mNumberMatches; i++)
	{
		Point2f &lp = mvP1[mvMatches[i].first];
		Point2f &rp = mvP2[mvMatches[i].second];

		int lgidx = mvMatchPairs[i].first = GetGridIndexLeft(lp, GridType);
		int rgidx = -1;

		if (GridType == 1)
		{
			rgidx = mvMatchPairs[i].second = GetGridIndexRight(rp);
		}
		else
		{
			rgidx = mvMatchPairs[i].second;
		}

		if (lgidx < 0 || rgidx < 0)	continue;

		mMotionStatistics.at<int>(lgidx, rgidx)++;
		mNumberPointsInPerCellLeft[lgidx]++;
	}

}
```

`raw/codes/BEV-LSLAM/orb_lio/src/orb_feature/gms_matcher.cc:L255-L290`

```cpp
		int max_number = 0;
		for (int j = 0; j < mGridNumberRight; j++)
		{
			int *value = mMotionStatistics.ptr<int>(i);
			if (value[j] > max_number)
			{
				mCellPairs[i] = j;
				max_number = value[j];
			}
		}

		int idx_grid_rt = mCellPairs[i];

		const int *NB9_lt = mGridNeighborLeft.ptr<int>(i);
		const int *NB9_rt = mGridNeighborRight.ptr<int>(idx_grid_rt);

		int score = 0;
		double thresh = 0;
		int numpair = 0;

		for (size_t j = 0; j < 9; j++)
		{
			int ll = NB9_lt[j];
			int rr = NB9_rt[CurrentRP[j] - 1];
			if (ll == -1 || rr == -1)	continue;

			score += mMotionStatistics.at<int>(ll, rr);
			thresh += mNumberPointsInPerCellLeft[ll];
			numpair++;
		}

		thresh = THRESH_FACTOR * sqrt(thresh / numpair);

		if (score < thresh)
			mCellPairs[i] = -2;
	}
}
```

`raw/codes/BEV-LSLAM/orb_lio/src/orb_feature/gms_matcher.cc:L293-L309`

```cpp
int gms_matcher::run(int RotationType) {

	mvbInlierMask.assign(mNumberMatches, false);

	// Initialize Motion Statisctics
	mMotionStatistics = Mat::zeros(mGridNumberLeft, mGridNumberRight, CV_32SC1);
	mvMatchPairs.assign(mNumberMatches, pair<int, int>(0, 0));

	for (int GridType = 1; GridType <= 4; GridType++)
	{
		// initialize
		mMotionStatistics.setTo(0);
		mCellPairs.assign(mGridNumberLeft, -1);
		mNumberPointsInPerCellLeft.assign(mGridNumberLeft, 0);

		AssignMatchPairs(GridType);
		VerifyCellPairs(RotationType);
```

### 实现注意事项

- GMS 的核心是局部网格支持度，不估计基础矩阵、单应矩阵或位姿；后续仍需位姿级验证。
- `WithRotation=true` 会枚举旋转模式，适合回环和 BEV 帧间匹配；如果运动先验很强，可以关闭以降耗。
- 图像尺寸要与关键点来源一致；BEV 高度图和强度图应分别计算各自 mask，不能混用。
- 过滤后建议再做距离阈值、空间分布检查或 Homography/PnP/ICP，避免重复纹理造成虚假局部一致。

### 源码检索锚点

- `raw/codes/BEV-LSLAM/orb_lio/src/orb_feature/gms_matcher.cc`：`GetInlierMask`、`AssignMatchPairs`、`VerifyCellPairs`、`run`
- `raw/codes/BEV-LSLAM/orb_lio/include/orb_feature/gms_matcher.hpp`：网格尺寸、阈值和旋转模式定义
- `raw/codes/BEV-LSLAM/orb_lio/src/scantoscan_kitti.cpp`：`matches_height`、`matches_intensity`、`GetInlierMask`
- `raw/codes/BEV-LSLAM/orb_lio/src/scantomap_kitti.cpp`：回环候选中的 GMS 调用

## 相关页面

- [[算法-BEV-LSLAM]]
- [[方法-BEV投影]]
- [[概念-回环检测方法]]
- [[方法-Fast-VGICP]]
- [[2026-04-29-external-primary-source-check]]
