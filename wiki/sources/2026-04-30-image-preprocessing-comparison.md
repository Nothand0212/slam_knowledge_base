---
tags: [素材摘要, 图像预处理, SLAM, VIO, 观测模型]
created: 2026-04-30
updated: 2026-04-30
sources:
  - raw/notes/2026-04-30-image-preprocessing-comparison.md
source_type: Markdown/文本/HTML
source_path: raw/notes/2026-04-30-image-preprocessing-comparison.md
images: 0
image_paths: []
---

# 图像预处理源码调查与影响分析

> 源码级梳理 SLAM/VIO/LIVO 项目中的图像预处理算法，并按观测模型分析其对前端跟踪和后端残差的影响。

## 基本信息

- **来源类型**：本地 Markdown 调查文档
- **原文位置**：`raw/notes/2026-04-30-image-preprocessing-comparison.md`
- **消化日期**：2026-04-30
- **覆盖项目**：OpenVINS、VINS-Fusion、Kimera-VIO、ORB-SLAM3、OpenMAVIS、DSO、DM-VIO、ROVIO、SVO、DROID-SLAM、MonoGS、NICE-SLAM、R3LIVE、FAST-LIVO2、LVI-SAM、ESVO 等

## 核心观点

1. **图像预处理必须绑定观测模型讨论**：KLT/角点前端、ORB 描述子前端、直接法光度残差、学习式网络输入和 LIVO/RGB map 对预处理的需求不同，不能把 CLAHE、resize、光度标定混成同一类增强。
2. **几何类预处理的首要风险是内参同步**：resize、downsample、crop、rectify 和 remap 都会改变像素坐标系统；如果 `fx/fy/cx/cy/W/H` 没有同步更新，后端 bearing、reprojection 或 ray direction 会系统性偏移。
3. **CLAHE/直方图均衡主要适合特征法前端实验**：它可能提升弱光、低纹理下的角点数量和 KLT 成功率，但对直接法会改变局部亮度映射，破坏光度一致性假设。
4. **直接法的预处理是残差模型的一部分**：DSO/DM-VIO 的 response/vignette/exposure、ROVIO/SVO 的仿射亮度、FAST-LIVO2 的 inverse exposure 都应作为光度观测模型处理，而不是普通图像增强。
5. **学习式/稠密系统优先检查输入契约**：DROID-SLAM 的 BGR->RGB、`/255`、ImageNet normalization，NICE-SLAM 的 crop/crop_edge 内参更新，MonoGS 的 per-frame exposure 参数都比传统增强更关键。

## 关键概念

- [[方法-图像预处理]]
- [[方法-视觉特征跟踪]]
- [[概念-直接法光度误差]]
- [[方法-曝光在线估计]]
- [[图像预处理与观测模型]]
- [[相机数据管线]]

## 与其他素材的关联

- 对 [[2026-04-29-camera-pipeline-comparison]] 的扩展：从“相机管线有哪些步骤”进一步细化到“每类预处理如何影响观测模型”。
- 对 [[VIO方案对比]] 的补充：将视觉前端选型从 KLT/ORB/直接法扩展到预处理边界和残差假设。
- 对 [[方法-视觉特征跟踪]] 的补充：解释 CLAHE、金字塔、mask/grid、稀疏点去畸变如何影响 tracked count、inlier ratio 和 track lifetime。
- 对 [[概念-直接法光度误差]] 与 [[方法-曝光在线估计]] 的补充：强调直接法应优先建模 response/vignette/exposure，而不是任意套用图像增强。

## 原文精彩摘录

> 图像预处理在这些项目里不是一个统一模块，而是直接绑定到观测模型。

> 能改善 KLT 特征跟踪的图像增强，不一定适合直接法后端。

> resize/downsample 如果没有同步修改内参，会直接制造系统性几何误差。

## 相关页面

- [[方法-图像预处理]]
- [[图像预处理与观测模型]]
- [[相机数据管线]]
- [[VIO方案对比]]
- [[方法-视觉特征跟踪]]
- [[概念-直接法光度误差]]
- [[方法-曝光在线估计]]
