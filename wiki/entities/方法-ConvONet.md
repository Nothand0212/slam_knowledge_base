---
tags: [方法, 深度学习, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-29
sources:
  - wiki/sources/2026-04-29-nice_slam-analysis.md
---

# ConvONet

> NICE-SLAM 使用的预训练隐式解码器，把多层特征网格查询结果转成 occupancy、颜色和几何细节。

## 定义

ConvONet（Convolutional Occupancy Networks）在 NICE-SLAM 中作为隐式场解码器。它接收层次化特征网格插值得到的局部特征和位置编码，输出 occupancy 或 RGB，使地图既有可优化的网格表示，又能通过 MLP 表达连续几何。

## 解码器层次

NICE-SLAM 使用多个解码器分工：

| 解码器 | 作用 |
|--------|------|
| `coarse_decoder` | 粗层 occupancy 判断，不显式拼接 xyz |
| `middle_decoder` | 中层几何，标准 MLP 并带 skip 连接 |
| `fine_decoder` | 结合 middle 特征，恢复高频几何细节 |
| `color_decoder` | 输出 RGB 和 occupancy，供体积渲染合成图像 |

位置编码使用 Gaussian Fourier Feature Transform，把 3D 坐标扩展到约 93 维，提高 MLP 表达高频几何的能力。

## 在 SLAM 中的意义

ConvONet 提供的是几何先验和连续场表达，不是独立定位器。Tracking 时，系统通过体积渲染把当前地图渲染为深度/颜色，与输入 RGB-D 对齐；Mapping 时，优化特征网格和关键帧位姿，让解码器输出更符合观测。

## 工程边界

- 预训练解码器带来先验，但也可能在未见场景中产生错误外推。
- 解码器固定时，主要优化负担转移到特征网格；解码器可训练时，优化更强但更不稳定。
- 相比显式 TSDF/点云，隐式场更难做局部编辑和确定性调试。

## 相关页面

- [[算法-NICE-SLAM]]
- [[方法-层次化特征网格]]
- [[概念-体积渲染]]
- [[概念-可微地图]]
- [[概念-深度学习SLAM]]
