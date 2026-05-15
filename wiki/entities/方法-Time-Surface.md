---
tags: [事件相机, 数据表示, ESVO]
sources: [wiki/sources/2026-04-29-esvo-analysis.md]
created: 2026-04-29
updated: 2026-05-14
type: entity
---

# Time Surface

> Time Surface（时间表面）是事件相机的核心 2D 数据表示方法，通过指数衰减将异步事件流转换为类似灰度图像的表示，使传统帧图像算法可应用于事件数据。

## 定义

对于每个像素 (x,y)，Time Surface 值表示该像素最近一次事件发生的时间距今的衰减：

```
TS(x,y) = exp(-(T - t_e) / τ)
```

其中：
- T 为当前参考时间
- t_e 为该像素最近一次事件的时间戳
- τ 为衰减时间常数（典型值 30ms）

## 工程实现（ESVO）

1. **生成模式**：
   - BACKWARD：直接逐像素计算
   - FORWARD：先校正畸变再做双线性插值（避免畸变误差）
2. **极性处理**：
   - `ignore_polarity_=true`：TS = exp(-Δt/τ)（忽略 ON/OFF）
   - `ignore_polarity_=false`：TS = polarity × exp(-Δt/τ)（保留极性）
3. **后处理**：
   - 高斯模糊（kernelSize 15）减少噪声影响
   - Negative Time Surface（255-TS）用于补充 OFF 事件信息
   - Sobel 梯度计算用于 Tracking 解析雅可比

## 在 ESVO 中的应用

- Time Surface 作为立体匹配的输入（替代传统灰度图像）
- ZNCC 块匹配在 Time Surface 上搜索最佳视差
- Time Surface 历史窗口（TS_history）支持追踪
- Tracking 使用 Time Surface 梯度信息做逆向组合位姿优化

## 相关页面

- [[算法-ESVO]]
- [[传感器-事件相机]]