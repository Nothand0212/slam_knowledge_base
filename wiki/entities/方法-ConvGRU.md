---
tags: [方法, 深度学习, SLAM]
type: entity
created: 2026-04-29
updated: 2026-04-30
sources:
  - wiki/sources/2026-04-29-droid_slam-analysis.md
---

# ConvGRU

> DROID-SLAM 用于迭代更新光流和几何状态的卷积循环模块，把 RAFT 式局部相关信息和全局上下文结合起来。

## 定义

ConvGRU 是把 GRU 的全连接门控替换为卷积门控的循环单元。它保留空间结构，适合在特征图上迭代更新光流、匹配置信度或隐藏状态。DROID-SLAM 的 UpdateModule 使用 ConvGRU 反复修正帧间光流和 Dense BA 的输入。

## 输入结构

DROID-SLAM 中一次更新通常融合三类信息：

- 相关编码：来自 all-pairs correlation lookup，约 128 维。
- 上下文特征：由上下文网络提供，约 128 维。
- 运动编码：当前光流、残差和几何状态编码，约 64 维。

合计约 320 维输入，经 ConvGRU 更新隐藏状态后预测下一轮修正量。

## 全局上下文调制

标准 RAFT 主要依赖局部相关查找。DROID-SLAM 增加全局上下文调制：把隐藏状态经过 sigmoid 门控和空间平均得到全局描述子 `glo`，再用 1x1 卷积投影到门控偏置项中。这样局部光流更新能感知全局几何状态，适合多帧 SLAM 而不是单纯两帧光流。

## 工程意义

- 用固定次数迭代替代传统前端的“匹配 + RANSAC + 优化”手工流程。
- 输出仍进入 Dense BA，因此不是纯黑箱位姿回归。
- 依赖 GPU 和训练数据，工程可控性弱于传统解析前端。

## Agent 实现提示

### 适用场景

当 Agent 需要实现“在特征图上反复修正状态”的学习式前端时使用 ConvGRU：例如 DROID-SLAM 式光流/残差更新、局部地图匹配状态更新、或把相关体、上下文和运动编码融合成下一轮几何目标。它适合保持 `B x C x H x W` 空间结构，不适合直接处理无网格的点集或稀疏图节点。

### 输入输出契约

- 输入隐藏状态 `net`: `Tensor[B*N, 128, H, W]`，来自上下文网络或上一轮更新。
- 输入特征 `inp/corr/flow`: 在通道维拼接后为 `Tensor[B*N, 128+128+64, H, W]`。
- 输出隐藏状态 `net`: `Tensor[B*N, 128, H, W]`。
- 在 DROID-SLAM `UpdateModule` 中，更新后的隐藏状态继续预测 `delta`、`weight` 等几何优化输入。

### 实现骨架（伪代码）

```text
function conv_gru_update(net, context, corr, motion):
    inp = concat(context, encode_corr(corr), encode_motion(motion), dim=channel)
    global_state = mean(sigmoid(conv1x1(net)) * net, spatial_dims)
    z = sigmoid(conv_z(concat(net, inp)) + conv_z_global(global_state))
    r = sigmoid(conv_r(concat(net, inp)) + conv_r_global(global_state))
    q = tanh(conv_q(concat(r * net, inp)) + conv_q_global(global_state))
    net_next = (1 - z) * net + z * q
    return net_next
```

### 关键源码片段

`raw/codes/DROID-SLAM/droid_slam/modules/gru.py:L5-L32`

```python
class ConvGRU(nn.Module):
    def __init__(self, h_planes=128, i_planes=128):
        super(ConvGRU, self).__init__()
        self.do_checkpoint = False
        self.convz = nn.Conv2d(h_planes+i_planes, h_planes, 3, padding=1)
        self.convr = nn.Conv2d(h_planes+i_planes, h_planes, 3, padding=1)
        self.convq = nn.Conv2d(h_planes+i_planes, h_planes, 3, padding=1)

        self.w = nn.Conv2d(h_planes, h_planes, 1, padding=0)

        self.convz_glo = nn.Conv2d(h_planes, h_planes, 1, padding=0)
        self.convr_glo = nn.Conv2d(h_planes, h_planes, 1, padding=0)
        self.convq_glo = nn.Conv2d(h_planes, h_planes, 1, padding=0)

    def forward(self, net, *inputs):
        inp = torch.cat(inputs, dim=1)
        net_inp = torch.cat([net, inp], dim=1)

        b, c, h, w = net.shape
        glo = torch.sigmoid(self.w(net)) * net
        glo = glo.view(b, c, h*w).mean(-1).view(b, c, 1, 1)

        z = torch.sigmoid(self.convz(net_inp) + self.convz_glo(glo))
        r = torch.sigmoid(self.convr(net_inp) + self.convr_glo(glo))
        q = torch.tanh(self.convq(torch.cat([r*net, inp], dim=1)) + self.convq_glo(glo))

        net = (1-z) * net + z * q
        return net
```

`raw/codes/DROID-SLAM/droid_slam/droid_net.py:L111-L132`

```python
    def forward(self, net, inp, corr, flow=None, ii=None, jj=None):
        """ RaftSLAM update operator """

        batch, num, ch, ht, wd = net.shape

        if flow is None:
            flow = torch.zeros(batch, num, 4, ht, wd, device=net.device)

        output_dim = (batch, num, -1, ht, wd)
        net = net.view(batch*num, -1, ht, wd)
        inp = inp.view(batch*num, -1, ht, wd)
        corr = corr.view(batch*num, -1, ht, wd)
        flow = flow.view(batch*num, -1, ht, wd)

        corr = self.corr_encoder(corr)
        flow = self.flow_encoder(flow)
        net = self.gru(net, inp, corr, flow)

        ### update variables ###
        delta = self.delta(net).view(*output_dim)
        weight = self.weight(net).view(*output_dim)
```

### 实现注意事项

- 不要把 `net` 和输入特征的 batch 维混淆；DROID-SLAM 会把 `batch*num_edges` 展平成卷积 batch。
- `flow` 在这里是 4 通道，通常包含当前投影流和目标残差；只传 2 通道光流会与 `flow_encoder` 不匹配。
- 全局上下文 `glo` 是空间平均后的门控偏置，保留它能增强多帧几何一致性。
- `delta/weight` 是下游 Dense BA 的目标和置信度，不应在 ConvGRU 内部直接更新位姿或深度。

### 源码检索锚点

- `raw/codes/DROID-SLAM/droid_slam/modules/gru.py`：`class ConvGRU`、`convz_glo`、`def forward(self, net, *inputs)`。
- `raw/codes/DROID-SLAM/droid_slam/droid_net.py`：`class UpdateModule`、`self.gru = ConvGRU(128, 128+128+64)`、`self.delta`、`self.weight`。

## 相关页面

- [[算法-DROID-SLAM]]
- [[方法-RAFT光流]]
- [[方法-Dense BA]]
- [[概念-深度学习SLAM]]
