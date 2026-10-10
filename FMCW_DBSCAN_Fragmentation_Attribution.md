# FMCW DBSCAN Fragmentation Attribution — 冻结 DEV/SILVER 只读诊断

日期：2026-10-10。状态：**TESTED / VERIFIED RESULT（仅限 DEV/SILVER 邻域代理）**。

## 结论先行

**A. 本轮没有发现 range adaptation 通过修复 fragmentation 增加 candidate-stage silver recall。** Fixed ε=0.6 m 与现有 published-style adaptive ε 均为 **71/80**，新增匹配 **0**、丢失匹配 **0**。Adaptive 确实改善邻域连通性，但原来 fragmented 的多数帧已经有一个合格 component 足以命中。其主要可验证作用是**扩大连通尺度、合并场景 component，并通过既有几何规则压缩候选量**，不是增加被恢复的 silver frames；也不能简单称为“增加候选 permissiveness”。

**B. ε 不是这些细表示漏检的主要瓶颈。** 9 个 miss 中，8 个 100 m component 超过冻结的 **32 voxel 上限**，1 个 300 m frame 是 singleton。改变 DBSCAN minPts=1/2 无效，因为几何层仍独立要求至少2 voxel。另一方面，相同点级过滤输入的4 cm→1 m支持压缩很强：少于2 cell 的帧从 **1/80→31/80**。这证明 coarse representation 的支持损失，但并非完整粗/细 detector 性能比较，也不允许选出“最佳 voxel/参数”。

没有修改历史 baseline、published reconstruction、B、DP、标签或旧 cache；没有运行 tracking、Doppler gate，或更新正式方法。

## 1. 固定输入与比较边界

输入为原有 `results/published_baseline_dp_mvp/raw_fine_cache/` 五个4 cm NPZ，以及原有 `results/research_dev_v0/silver_labels.csv`。共272个实际 LiDAR frames、80个已有 observed silver centers；不插值、不增删标签、不使用银标邻域生成候选。

| 工况 | 全部 LiDAR frames | observed silver frames |
|---|---:|---:|
| 100m_cross_5 | 32 | 19 |
| 100m_cross_10 | 46 | 11 |
| 200m_cross_5 | 83 | 22 |
| 200m_cross_10 | 48 | 18 |
| 300m_cross_10 | 63 | 10 |

这里的100/200/300 m是 nominal 条件分组，不是精确 range bin；真实距离由既有审计提供。仅覆盖已确认的横飞 DEV 片段，不能外推到 longitudinal 或全部未标注帧。

Fine cache 已在点级执行：finite xyz/intensity/velocity、nonzero → sensor ROI `[8,510]×[-200,200]×[-60,60] m` → intensity `[10,250]` → provisional leveled height >10 m → sensor-frame、零原点、`floor(xyz/0.04)` voxel mean。它不是未过滤 raw cloud。所有比较使用完全相同的 cache。

候选链停在 **DBSCAN → geometric validation**，不加入 spatial-jump/M-of-K，因为要求是归因 candidate stage。几何参数全部保持：2–32 voxel、raw support ≤120、max-axis extent <6 m、centroid range 8–510 m。≥3 voxel 用不加权 component-wise median，2 voxel 用 mean；不替换成 B 的 weighted median。

### Adaptive 的真实实现语义

严格复用现有冻结 reconstruction，而不是新设计一个逐点或分 range 组的算法：

```text
r_frame = unweighted mean(||voxel_xyz||) over ALL surviving fine voxels
eps_frame = 0.6 + 0.004 * max(r_frame - 10, 0)     # m, no cap
```

每帧只有一个 ε；计算不使用 silver center 或 target range。全帧 ε 范围为 **1.6697–1.6908 m**；nominal100/200/300 m组中位数分别 **1.6770/1.6778/1.6792 m**。所以本轮是 ε=0.6 与约1.68 m的全局场景尺度对照，**不是100/200/300 m UAV专属的 ε(r_target) 对照**。

原 E13 参数溯源仍有效：ε0=0.6 m、4 cm voxel、minPts=2、rref=10 m来自已有论文审计；**α=0.004以及几何上下限是项目适配参数，论文数值 UNKNOWN**。本轮不重新声称精确复现论文，也不由结果改写 adaptation 语义。

四项比较在主运行前冻结：fixed/adaptive × minPts=1/2。其他处理一项未变。

## 2. Candidate-stage silver recall 与全帧候选量

Recall：银标帧中，存在任一几何合格候选 centroid 到既有 center ≤3 m。它是**银标中心邻域匹配**，不证明候选所有点属于 UAV。Candidate volume 分母为全部 LiDAR frames，不是80个银标帧；不称 precision、false alarms 或 clutter classification accuracy。

下面每行同时适用于 minPts=1 和2，二者逐帧 accepted component sets 完全相同。

| nominal range | silver n / 全帧 n | fixed 命中 / recall | adaptive 命中 / recall | fixed candidates/frame | adaptive candidates/frame | adaptive新增 / 丢失 |
|---|---:|---:|---:|---:|---:|---:|
| 100 m | 30 / 78 | 22/30，73.33% | 22/30，73.33% | 109.38 | 36.05 | 0 / 0 |
| 200 m | 40 / 131 | 40/40，100% | 40/40，100% | 113.11 | 36.52 | 0 / 0 |
| 300 m | 10 / 63 | 9/10，90% | 9/10，90% | 112.83 | 34.94 | 0 / 0 |
| ALL | 80 / 272 | 71/80，88.75% | 71/80，88.75% | 111.97 | 36.02 | 0 / 0 |

固定/自适应总候选分别 **30,457/9,797**，净减少20,660（67.83%）。这里没有可靠 negative labels，因此 operational precision、false detections/frame、真实新增 UAV frames 均为 **UNKNOWN**。

### minPts=1/2 为什么结果相同？

scikit-learn `min_samples` 包含自身。minPts=1把 ε 图的所有连通分量（包括 singleton）保留；minPts=2仅把孤立 singleton 标为 noise。任何至少2 voxel的 ε 连通分量，每点都有邻居，均为 core。由于几何层自己要求 ≥2 voxel，两种 minPts 的合格候选必然相同。

| ε schedule | minPts=1 pregeometry components | 其中singleton，全部被geometry拒绝 | minPts=2 components | minPts=2 noise voxel | 两者最终候选 |
|---|---:|---:|---:|---:|---:|
| fixed | 57,750 | 24,162 | 33,588 | 24,162 | 30,457 |
| adaptive | 20,122 | 7,202 | 12,920 | 7,202 | 9,797 |

这不表示 singleton 不重要；表示只改 DBSCAN minPts 不能绕过当前几何 nmin。没有放宽 nmin 来制造 minPts=1 的收益。

## 3. ε=0.6 下 target-neighborhood support 是否已经连通？

银标只用于聚类后的诊断。将各帧 cache 中距离 center ≤3 m的 voxel取出，单独计算 ε 邻接图的 induced connected components；singleton记为连通，空集记 UNKNOWN。另记录完整场景 component 如何穿过邻域，防止把外部点形成的 bridge 误认为 UAV内部连通。

| nominal range | fixed 单一induced component | fixed fragmented | adaptive单一component | 连通改善帧 | 因此新增candidate匹配 |
|---|---:|---:|---:|---:|---:|
| 100 m | 27/30 | 3/30 | 29/30 | 2 | 0 |
| 200 m | 32/40 | 8/40 | 38/40 | 6 | 0 |
| 300 m | 6/10 | 4/10 | 9/10 | 3 | 0 |
| ALL | 65/80 | 15/80 | 76/80 | 11 | 0 |

Fixed 的15个 fragmented proxy 中 **14个原来已命中**。唯一未命中的 fragmented 帧为 `100m_cross_5/frame20`：主 component 已超过32 voxel，另有singleton；adaptive连接后仍超过上限。Candidate recall只要求一个合格组件，不要求整个3 m ball全部连在一起。

按预先保留的1 m内邻域，fixed连通为 **29/30、35/40、6/10**（70/80），adaptive为 **30/30、40/40、10/10**（80/80），仍没有新增candidate匹配。两种邻域全部保留，不择优报告。3 m外围点身份不确定；“连通改善”仅是 proxy 描述，独立点级 UAV fragmentation 是否修复仍 **UNKNOWN**。

## 4. Adaptive 的候选与大簇变化：净数量不能替代 merge 归因

“大簇”固定定义为 `voxel>32 OR raw_support>120 OR axis_extent>=6m`；各拒绝flag可重叠。逐帧 component lineage 在完全相同 fine input 上建立：ε增加不会拆开 fixed component。若adaptive大簇没有任何 fixed 大簇 parent，记为“新形成大簇”；多个已有大簇被合为一个，单独记录 coalesced 数。

| nominal range | 净Δcandidates / frame | fixed→adaptive大簇数 | 净Δ大簇 | 新形成大簇 | 旧大簇coalesced减少量 |
|---|---:|---:|---:|---:|---:|
| 100 m | -73.33 | 902→917 | +15 | 99 | 84 |
| 200 m | -76.59 | 1,515→1,495 | -20 | 194 | 214 |
| 300 m | -77.89 | 714→711 | -3 | 97 | 100 |
| ALL | -75.96 | 3,131→3,123 | -8 | 390 | 398 |

`390−398=−8`，所以净大簇数略少不代表没有新增merge。另有 **19,184个 fixed 合格候选 component 被吸收到 adaptive不合格大簇**中，1,836个adaptive合格component合并了至少两个fixed非noisecomponent；候选减少不只是合格候选间的简单合并。

全场景几何flag也显示更宽连接尺度的影响：

| 拒绝flag（非互斥） | fixed | adaptive | Δ |
|---|---:|---:|---:|
| voxel>32 | 3,128 | 3,069 | -59 |
| raw_support>120 | 1,461 | 1,751 | +290 |
| axis_extent≥6 m | 423 | 1,293 | +870 |

这些是场景 component 的归因统计，不是被独立确认的 clutter/UAV merge事件。没有本轮新增silver命中；银标邻域大簇拒绝均限于既有8个100 m miss。不能由较少输出宣称 precision 提升、clutter 被正确过滤或ε为最佳值。

## 5. 未恢复的9帧

| condition | frame_index（0-based） | 原因（fixed与adaptive均如此） |
|---|---|---|
| 100m_cross_5 | 16,20,27,28,30 | 邻域主component >32 fine voxels；ball occupied count分别38,47,43,39,36 |
| 100m_cross_10 | 19,23,24 | component >32 fine voxels；分别40,35,34 |
| 300m_cross_10 | 2 | 仅1 voxel：minPts=2 noise；minPts=1被geometry nmin=2拒绝 |

Frame20的47包含外围singleton，不能把ball count当作 fixed主component自身大小；逐component数据在CSV中。其余这些100 m miss的主component均已连通，增加ε不会降低occupied voxel数。未发现这些miss由centroid match gate、raw support上限或extent上限主导。

因此，当前fine pipeline的局部失效首先是 **representation-dependent geometry 上限**，其次是真singleton与最低support规则；不是 fixed0.6 m无法连通UAV support的普遍失效。更改这些阈值未获本轮授权，也未实施。

## 6. 4 cm 与1 m support collapse（独立描述，不联合调参）

三种口径分别保留：

1. E15原始邻域点重放完全相同的点级ROI/intensity/leveled-height过滤，直接计算4 cm与1 m固定grid occupancy。此处是相同输入下的纯cell-count变化，不运行粗表示detector。
2. 在 actual fine cache 与 existing1m postfilter cache中，按相同silver ball统计voxel和raw support。旧1m cache先聚合再用均值做过滤，与fine的先过滤再聚合顺序不同，不能假定两种pipeline等价。
3. 对同一整帧fine voxel means做1 m weighted coarsening，仅统计support，不聚类。4 cm均值可能跨1 m边界，因此它不是一般情况下精确的raw1m构建替代。

本数据的160个paired radius记录中，各occupied count恰好一致；这是当前邻域的实际验证结果，不消除pipeline顺序的一般性混杂。4 cm的raw support不会因voxel数改变而等价于新的物理返回数。

以下为3 m primary，逐帧统计的中位数：

| nominal range | 过滤后raw proxy points | 4 cm cells | 同输入1 m cells | old1m actual voxels | median逐帧cell collapse | 4 cm support<2 | 1 m support<2 | 1 m support<3 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 m | 39.5 | 27.5 | 2 | 2 | 93.75% | 0/30 | 12/30 | 24/30 |
| 200 m | 17.5 | 14 | 2 | 2 | 86.36% | 0/40 | 14/40 | 29/40 |
| 300 m | 5 | 5 | 1.5 | 1.5 | 64.58% | 1/10 | 5/10 | 8/10 |

整体 support<2 为 **1/80→31/80**，support<3为 **1/80→61/80**。近距离coarse collapse同样严重；远距离既有返回数低，且进一步变成单cell。粗表示造成的“singleton”不能直接称 sensor只有一次return。

没有在1 m输入上联合扫 ε/minPts/geometry；本轮不能给出coarse/fine完整detector性能差、最佳voxel大小或消除clutter后的真实accuracy。

## 7. 可复现性与限制

来源 skill：academic-research-suite / experiment-agent，validate口径；Material Passport：2026-10-10，validation_v1，VERIFIED（代码级诊断），科学外推置信度 **CAUTION**。Skill用于冻结、复现与解释检查，没有授权新算法或调参。

主run：2026-10-10 02:56:14–02:57:13 UTC；输入/cache/旧source/协议/标签等 **74个受保护文件 SHA-256全相同**。原 `published_clusters` 独立内存replay对照 **272/272帧、9,797/9,797候选**的center xyz、voxel数、raw support、extent和ε严格相等。6个合成单元测试通过；两个诊断脚本与测试AST检查通过。

环境：Python3.10.12、NumPy2.2.6、SciPy1.15.3、scikit-learn1.7.2、Linux。无随机算法。耗时仅作运行记录，不比较部署性能。

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python -m unittest discover -s tests -p test_dbscan_fragmentation_attribution.py -v
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/dbscan_fragmentation_attribution.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/dbscan_fragmentation_attribution.py run
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/summarize_dbscan_fragmentation.py
```

已存在FROZEN或完成结果时脚本会拒绝覆盖；复现需使用独立checkout/输出副本，不能删除旧记录来重跑。结果不是新正式baseline。

| 身份 | SHA-256 |
|---|---|
| Git parent | `7e865cb55bc61b49d6debcb3ed69dd950f1d0a62` |
| 新诊断脚本 | `99f21bcc1716be1bb857084f529233b0dadac4ae4133bbd1fc33c091c6e6ba26` |
| summary/QA脚本 | `c7b93bc212876856df388b8dd24cbcb550c5725066708d1b0bb65916b592ea9c` |
| 本轮冻结protocol | `f74375153991a536a54a65870edb6f910937d5961806060abbabd63a8f37d418` |
| E13 source protocol | `5552c35cf9de614c98a668647b66d3ece27efa60695e13bc856c5488b58324af` |
| silver labels | `37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db` |

11/11统计/方法学谬误项已检查并记录于 `independent_validation.json`：主要限制为observed-trajectory selection/survivorship、无negative base rate、邻域点身份不明、同一DEV后验解释、跨帧依赖、场景均值ε不等于target-range adaptation。不计算p值、不宣称统计显著或独立论文性能。

当前证据只支持这些冻结输入和stage定义上的代码级归因。既有E12/E13 temporal/DP负结果不被覆盖；最终GT、未观测UAV returns、operational precision以及其它range-adaptive实现的效果仍 **UNKNOWN**。

## 8. 机器可读文件

全部保存于 [`results/dbscan_fragmentation_attribution/`](results/dbscan_fragmentation_attribution/)，未写入旧结果目录：

- `summary_by_range.csv`：四项variant×100/200/300/ALL，含recall、候选量、大簇、eps。
- `frame_metrics.csv`：1,088行，完整272帧×四项，含几何flag、noise/singleton和silver命中。
- `cluster_metrics.csv`：124,380行，全场景DBSCANcomponent及accepted/rejection属性。
- `silver_components.csv`：640行，80帧×四项×1/3 m邻域，induced与full-scene connectivity分开。
- `adaptive_transitions.csv`：320行，逐silver帧的新增/丢失与归因口径。
- `connectivity_by_range.csv`、`adaptive_delta_by_range.csv`：完整分组汇总。
- `representation_support.csv`：160行，raw→same-input filtered occupancy→actual cache→coarsening分开；`representation_by_range.csv`汇总。
- `component_merge_audit.csv`：12,920个adaptive/minPts2组件的fixed parents；`component_merge_summary.csv`区分新增大簇、旧簇合并与净变化。
- `miss_attribution.csv`：全部9个miss；`replay_validation.csv`及`original_function_replay.csv`保留复现证据。
- `protocol.json`、`FROZEN`、`freeze_identity.json`、`input_identity.json`、`run_manifest.json`、`verification.json`、`independent_validation.json`：冻结、命令、环境、输入/输出hash与验证。

全部负结果保留；没有根据这轮结果改labels、选择最佳参数或更新正式方法。
