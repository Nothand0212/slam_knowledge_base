# raw/codes 源码快照清单

> 生成日期：2026-04-30

`raw/codes/` 是知识库的只读源码快照区，用于源码检索、行号锚点和 `Agent 实现提示` 中的短代码片段。这里的项目不作为可继续开发的 Git 仓库维护；删除子项目 `.git` 后，版本信息以本清单为准。

## 使用规则

- 不在 `raw/codes/` 内直接开发或提交源码。
- 如果需要分析新版本，复制一个新版本源码快照进来，并更新本清单。
- 如果覆盖同名目录，必须同步检查 wiki 中的 `raw/codes/...:Lx-Ly` 行号锚点是否失效。
- `dirty=true` 表示删除 `.git` 前该源码目录存在本地未提交内容，这些内容也被纳入当前快照。

## 快照版本

| 项目 | 分支 | HEAD | Remote | dirty | 本地未提交内容 |
|------|------|------|--------|-------|----------------|
| 4DRadarSLAM | main | `dd2ee8c2378630a84361f37331ba14665b203c14` | `https://github.com/zhuge2333/4DRadarSLAM.git` | false | - |
| BEV-LSLAM | main | `85b87bdda81229e6c655d77af63a8a7ef598a484` | `https://github.com/ROBOT-WSC/BEV-LSLAM.git` | false | - |
| DROID-SLAM | main | `2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99` | `https://github.com/princeton-vl/DROID-SLAM.git` | false | - |
| ESVO | master | `1639a43ca3c5411b73a083c654f1aef73d8e0319` | `https://github.com/HKUST-Aerial-Robotics/ESVO.git` | false | - |
| FAST-LIO-SAM-SC-QN | master | `2ab179a3ec7254783553dd1b263a6cd6e819b8dc` | `https://github.com/engcang/FAST-LIO-SAM-SC-QN.git` | false | - |
| FAST-LIVO2 | main | `0d2c0346107b75b59934975adec9a6eeeb913c64` | `https://github.com/hku-mars/FAST-LIVO2.git` | false | - |
| IC-GVINS | main | `644eed9e02c1239d6788f5b19123559735ca1b9e` | `https://github.com/i2Nav-WHU/IC-GVINS.git` | false | - |
| LIO-SAM | master | `0be1fbe6275fb8366d5b800af4fc8c76a885c869` | `https://github.com/TixiaoShan/LIO-SAM.git` | false | - |
| LVI-SAM | master | `0d822f6dcac3378312f6703b4f45829e049f221a` | `https://github.com/TixiaoShan/LVI-SAM.git` | false | - |
| LeGO-LOAM | master | `896a7a95a8bc510b76819d4cc48707e344bad621` | `https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git` | false | - |
| MonoGS | main | `6c9254c319d8bff5caeef65259e6bb0941a9b9f6` | `https://github.com/muskie82/MonoGS.git` | false | - |
| OB_GINS | main | `e96c69ae84d09f0e8c1c69bdd9323eaec020db86` | `https://github.com/i2Nav-WHU/OB_GINS.git` | false | - |
| ORB_SLAM3 | master | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | `https://github.com/UZ-SLAMLab/ORB_SLAM3.git` | false | - |
| OpenMAVIS | master | `b13b1c20e84efa4bb63564e26308541af70d03f2` | `https://github.com/MAVIS-SLAM/OpenMAVIS.git` | false | - |
| PIN_SLAM | main | `e2b7fbf328e5fac1d638e217d70a69cbe9f30a52` | `https://github.com/PRBonn/PIN_SLAM.git` | false | - |
| ROLO | main | `b88ac0bf45e9309eb8b0eb78f677a6aaaf89b37e` | `https://github.com/sdwyc/ROLO.git` | false | - |
| SchurVINS | master | `d8ab6dff20860694d8d879d716d7fd6cb2c2530e` | `https://github.com/bytedance/SchurVINS.git` | false | - |
| SuperOdom | ros2 | `3f35d67d4ac3f343cce0e18274ed6a90fad315f4` | `https://github.com/superxslam/SuperOdom.git` | false | - |
| VINS-Fusion | master | `be55a937a57436548ddfb1bd324bc1e9a9e828e0` | `https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git` | false | - |
| cartographer | master | `877157a0d91788a7700221d87232d412cb3c1ef4` | `https://github.com/cartographer-project/cartographer.git` | false | - |
| ct_icp | master | `d467813176c81728f904a5c13a142a67aafebc46` | `https://github.com/jedeschaud/ct_icp.git` | true | `ct_icp.pdf` |
| dm-vio | master | `e306328098e08facbec1bfd45d22e19c77a3ccad` | `https://github.com/lukasvst/dm-vio.git` | false | - |
| dso | master | `7b0c99f01d238f801c625beaff90240bcb007198` | `https://github.com/JakobEngel/dso.git` | true | `dso.pdf` |
| fusions_slam | master | `e362c6458db383ae7ebf708c5575d508ce1f8ce3` | `https://github.com/LiangHongY/fusions_slam.git` | false | - |
| genz-icp | master | `d3ca3c4897d7e1df611bc60c0c8f1a19ecd593a9` | `https://github.com/cocel-postech/genz-icp.git` | false | - |
| kiss-icp | main | `b16835283aee62f7d5e2bdf6c1c3bb2930de74ff` | `https://github.com/PRBonn/kiss-icp.git` | false | - |
| lightning-lm | master | `7062399a93a6844230f9fd5ad6f4d72ab45f5938` | `https://github.com/gaoxiang12/lightning-lm.git` | false | - |
| lt-mapper | main | `80b67567b356850a10e883c1530e4e69314eaa62` | `https://github.com/gisbi-kim/lt-mapper.git` | false | - |
| msckf_vio | master | `68065ad48e6e6f411fdd64d153938a0130eb835f` | `https://github.com/KumarRobotics/msckf_vio.git` | false | - |
| nice-slam | master | `7af15cc33729aa5a8ca052908d96f495e34ab34c` | `https://github.com/cvg/nice-slam.git` | false | - |
| open_vins | master | `69488123ed9362dd44b6f28e7f4680abbff1442b` | `https://github.com/rpng/open_vins.git` | true | modified `.clang-format`; untracked `任务.md` |
| r3live | master | `6143a38537f28cb36eb24e9bbe2e39c8f7967157` | `https://github.com/hku-mars/r3live.git` | true | `r3live.pdf` |
| rovio | master | `34735c0bfd786d44f6d21d31c0776290f1f91185` | `https://github.com/ethz-asl/rovio.git` | false | - |
| rpg_svo_pro_open | master | `ca371f304637e7fb355cf4624d0a02da4e3da220` | `https://github.com/uzh-rpg/rpg_svo_pro_open.git` | false | - |
