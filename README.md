# ROS 机器人导航与 SLAM 知识库

这个仓库是一个面向 ROS/ROS 2、机器人导航、SLAM、VIO/LIO、多传感器融合与工程调试的本地知识库。它的目标不是保存聊天记录，而是把论文、开源项目、源码阅读、工程经验和方案取舍整理成可持续维护、可检索、可复用的 Markdown wiki。

## 仓库作用

- 沉淀 SLAM / localization / mapping / planning / control / sensor fusion 的工程知识。
- 横向比较典型开源系统，例如 OpenVINS、VINS-Fusion、ORB-SLAM3、DSO、DM-VIO、FAST-LIO、FAST-LIVO2、LIO-SAM、Cartographer、DROID-SLAM、MonoGS、NICE-SLAM 等。
- 为 AI agent 提供结构化上下文，使其检索到页面后能直接理解概念、方案取舍和代码实现方式。
- 保存源码快照与 wiki 页面之间的稳定引用关系，例如 `raw/codes/...:Lx-Ly` 行号锚点。

## 目录结构

```text
.
├── purpose.md              # 知识库研究目标、关键问题和范围
├── index.md                # 全库索引入口
├── log.md                  # 知识库维护日志
├── .wiki-schema.md         # wiki 写作与维护规范
├── AGENTS.md               # agent 工作提示和维护约束
├── raw/                    # 原始素材与源码快照
│   ├── articles/           # 网页/文章素材
│   ├── notes/              # 手写或生成的原始笔记
│   ├── docs-deep-dive/     # 项目深度分析原始材料
│   └── codes/              # 开源项目源码快照，只读
└── wiki/                   # 结构化知识库主体
    ├── entities/           # 算法、方法、概念、组件、架构等实体页
    ├── topics/             # 跨项目主题页
    ├── sources/            # 素材摘要页
    ├── comparisons/        # 方案对比页
    └── synthesis/          # 综合分析页
```

## 主要入口

- [purpose.md](purpose.md) - 这个知识库要解决的问题和研究边界。
- [index.md](index.md) - 全部实体页、主题页、素材页的导航入口。
- [wiki/overview.md](wiki/overview.md) - 知识库总览。
- [wiki/topics/素材索引.md](wiki/topics/素材索引.md) - 素材摘要统一入口。
- [wiki/comparisons/VIO方案对比.md](wiki/comparisons/VIO方案对比.md) - VIO/VO 方案横向对比。
- [wiki/topics/图像预处理与观测模型.md](wiki/topics/图像预处理与观测模型.md) - 图像预处理与视觉观测模型专题。

## raw/codes 的语义

`raw/codes/` 是只读源码快照区，不是开发工作区。

- 子项目不保留 `.git`，不作为 submodule 使用。
- 当前源码版本记录在 [raw/codes/MANIFEST.md](raw/codes/MANIFEST.md)。
- wiki 页面中的真实代码片段应引用这里的稳定路径和行号。
- 如果后续分析新版本源码，应复制新快照并更新 manifest，而不是在现有目录内直接开发。

## Agent 实现提示

部分 `wiki/entities/` 页面包含 `Agent 实现提示`，用于让 AI agent 检索后直接理解如何写代码。典型内容包括：

- 适用场景
- 输入输出契约
- 实现骨架（伪代码）
- 关键源码片段
- 实现注意事项
- 源码检索锚点

这类页面会优先引用 `raw/codes/...:Lx-Ly` 的真实源码短片段，并配合伪代码说明可迁移实现结构。

## 维护原则

- 修改 wiki 页面时，同步更新 frontmatter 的 `updated: YYYY-MM-DD`。
- 新增可落地实现的方法页或概念页时，优先补充 `Agent 实现提示`。
- 表格中的 wikilink 别名应写成 `[[页面名\|别名]]`，避免 Markdown 表格解析错误。
- `raw/codes/` 只读；不要在其中做源码开发。
- 所有重要维护动作记录到 [log.md](log.md)。

## 推荐使用方式

1. 从 [index.md](index.md) 或 [wiki/overview.md](wiki/overview.md) 进入主题。
2. 阅读 `wiki/topics/` 获取横向脉络。
3. 阅读 `wiki/entities/` 获取具体算法、方法、组件和实现提示。
4. 需要确认实现细节时，沿页面中的 `raw/codes/...:Lx-Ly` 锚点查看源码快照。
