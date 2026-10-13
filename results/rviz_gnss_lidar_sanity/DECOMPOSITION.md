# Frozen GNSS–anchor discrepancy decomposition

2026-10-13；状态：**TESTED / VERIFIED RESULT（数值描述）**；机制判断：**SPECULATIVE**。范围仅为现有五条 closure-PASS 横飞序列的 80 个非插值 DEV/SILVER observed anchors，不能解释为独立 GT、定位精度或算法性能。

**VERIFIED RESULT：各序列 95.48%–98.71% 的差异平方误差沿 GNSS chord，全部 80 anchors 合计为 97.9506%。最大差异 `200m_cross_5/frame21` 为 26.575401 m，其中 along-track +26.389269 m，占该帧平方误差的 98.6041%；cross-track −3.056353 m，vertical-normal +0.719090 m。** 所有序列至少一个原始 GNSS chord 内发生沿轨误差变号；单一常数 clock residual 不足以描述当前残差。稀疏 GNSS chord 的速度/转弯表达不足与未解决时间/航段对应更兼容整体模式，小的空间/参考点残差仍可能存在；根因 **UNKNOWN**。

## 数据与冻结边界

逐帧位置、chord、时间与 anchor 身份取自原始 [run_02/frames.csv](../gnss_conditioned_uav_phenotype/run_02/frames.csv)；80 个观测位置逐一与 [silver_labels.csv](../research_dev_v0/silver_labels.csv) 核对；GNSS chord 来自 [gnss_source_segments.csv](../gnss_conditioned_uav_phenotype/run_02/gnss_source_segments.csv)，原始支持计数来自 [region_features.csv](../gnss_conditioned_uav_phenotype/run_02/region_features.csv)。冻结 pose 与各序列既有 offset 来源于 [calibration_closure/alignment.json](../calibration_closure/alignment.json) 及其 alignment-results 表，输入 SHA-256 记录在 [decomposition_provenance.json](decomposition_provenance.json)。

本次没有重新插值、拟合 pose、搜索 offset、重运行 closure、改标签、改阈值、调用检测/跟踪评分或读写 bag。误差定义是 **anchor − projected GNSS**，单位为 m；时间单位为 s；速度单位为 m/s。坐标仍为冻结分析坐标 `[sensor_x, sensor_z, -sensor_y]`。查询时间仍为 `bag_record_time + frozen_sequence_offset`，ROS header 的 2022 年 epoch 仅保留作原始身份字段，不参与 GNSS 对齐。

## 分解定义与统计约定

冻结外参为 `p_analysis = R p_ENU + translation`。冻结 ENU-Up 在分析坐标下为 `u = R[:,2]`，保留原值：`[0.11708669986145206, 0.025829168126318722, 0.9927857567418342]`。对每个 anchor 的对应既有三维 GNSS chord velocity `v`：

```text
e = p_anchor − p_GNSS
speed = ||v||
t = v / speed
c = normalize(u × t)           # 水平左向 cross-track
b = t × c                      # vertical-normal，尽可能朝 Up 且垂直 chord
[d_parallel, d_cross, d_vnormal] = [t c b]^T e
||e||² = d_parallel² + d_cross² + d_vnormal²
tau_equivalent = d_parallel / speed = (e · v) / (v · v)
```

这是完整三维 chord 的正交分解。**vertical-normal 不是严格 ENU 高度差**：当 chord 爬升/下降，`b` 是 Up 在 chord 法平面内的归一化投影。严格 frozen ENU-Up 另保存为 `d_up = e·u`；水平-only along-track 与 strict Up 构成第二组正交分解，另存 `d_parallel_horizontal_m` 与 `equivalent_time_bias_horizontal_s`。原始 chord 相对于冻结 ENU-Up 的倾角覆盖 −0.719066°…+7.405055°，因此两种垂直定义不能混称；这些 Up/水平轴依赖现有 provisional 外参，不是新增 surveyed gravity 证书。直接使用 analysis +Z 的敏感性分解也保存，但不把受既有 pitch/roll 影响的传感器轴称为物理 Up。

`tau_equivalent > 0` 表示沿**当前局部线性 chord**增加 GNSS 查询时间（取更晚时刻）才会减少其沿轨投影差异；它不是测量得到的 clock error，不是新 offset，也不含 cross/vertical 修正。CSV 保留这个线性反事实是否跨 chord 或超出 GNSS 支持；未对反事实时间进行插值或改动原查询。

每个已存 anchor 等权；median 为第 50 百分位，IQR 为 Q75−Q25，分位数使用 `numpy.quantile(method='linear')`。幅度表对 `|component|` 计算 median/IQR/max，signed 统计另保留完整 CSV。以下保留 6 位小数只是便于复核，不意味着 GNSS/anchor 实测具有该精度。

## 逐 sequence 幅度与等效时间

| Sequence | Anchors | 总距离 median / IQR / max（m） |
|---|---:|---:|
| `100m_cross_5` | 19 | 3.735887 / 2.564580 / 6.349919 |
| `100m_cross_10` | 11 | 5.836332 / 2.868926 / 10.509431 |
| `200m_cross_5` | 22 | 5.317290 / 7.108744 / 26.575401 |
| `200m_cross_10` | 18 | 4.976735 / 3.943688 / 12.162975 |
| `300m_cross_10` | 10 | 5.060992 / 4.617160 / 8.787091 |

| Sequence | `|along|` median / IQR / max（m） | `|horizontal cross|` median / IQR / max（m） | `|vertical-normal|` median / IQR / max（m） |
|---|---:|---:|---:|
| `100m_cross_5` | 3.521702 / 2.537591 / 6.265098 | 0.572093 / 0.679498 / 1.441958 | 0.063452 / 0.039427 / 0.136541 |
| `100m_cross_10` | 5.656874 / 2.993150 / 10.430876 | 1.276713 / 0.319372 / 1.500051 | 0.103980 / 0.070795 / 0.197679 |
| `200m_cross_5` | 5.295531 / 7.221845 / 26.389269 | 0.453910 / 0.490433 / 3.056353 | 0.340959 / 0.173096 / 0.719090 |
| `200m_cross_10` | 4.953365 / 3.952169 / 12.089087 | 0.201038 / 0.874432 / 1.315471 | 0.348029 / 0.224336 / 0.663473 |
| `300m_cross_10` | 4.973839 / 4.678214 / 8.652539 | 0.507507 / 0.887743 / 1.501397 | 0.758185 / 0.225530 / 1.024509 |

| Sequence | signed τ median（s） | signed τ IQR（s） | signed τ min … max（s） | `|τ|` max（s） |
|---|---:|---:|---:|---:|
| `100m_cross_5` | -0.258591 | 2.214841 | -2.897897 … 1.696119 | 2.897897 |
| `100m_cross_10` | -0.228173 | 2.411642 | -2.240144 … 1.338099 | 2.240144 |
| `200m_cross_5` | 0.104314 | 2.348492 | -2.519855 … 6.707545 | 6.707545 |
| `200m_cross_10` | 0.655924 | 1.912990 | -1.791032 … 3.471995 | 3.471995 |
| `300m_cross_10` | -0.150282 | 1.028393 | -1.143408 … 1.111570 | 1.143408 |

完整 signed/absolute Q25、median、Q75、IQR、min/max 及最大值帧见 [sequence_component_summary.csv](sequence_component_summary.csv)。Chord speed 是端点距离除以原始采样间隔，不能用文件名 5/10 m/s 或该数值替代实际瞬时飞行速度。

## 误差方向一致性

固定分析坐标中的方向一致性定义为 `R_fixed = ||mean(e_i / ||e_i||)||`，范围 0…1；接近 1 表示误差单位向量较一致。它不检验等幅平移，也不是外参拟合。另报 `R_chord = ||mean([d_parallel,d_cross,d_vnormal]_i / ||e_i||)||`，其基底随对应 chord 转动。沿轨/横轨正负计数只以数值零容差 `1e-12` 区分，不引入新的研究门限。

| Sequence | Along 正 / 负 | Cross 正 / 负 | `R_fixed` | `R_chord` | Along 平方误差占比 |
|---|---:|---:|---:|---:|---:|
| `100m_cross_5` | 9 / 10 | 8 / 11 | 0.312474 | 0.161267 | 95.4772% |
| `100m_cross_10` | 5 / 6 | 7 / 4 | 0.787775 | 0.084658 | 96.3866% |
| `200m_cross_5` | 11 / 11 | 8 / 14 | 0.258158 | 0.133961 | 98.7138% |
| `200m_cross_10` | 12 / 6 | 6 / 12 | 0.136803 | 0.367058 | 98.5067% |
| `300m_cross_10` | 4 / 6 | 3 / 7 | 0.315698 | 0.315698 | 95.5760% |

`100m_cross_10` 的固定坐标方向一致性 0.787775 在数值上高于其他序列，而 chord-relative 一致性仅 0.084658。这可与部分固定空间方向残差兼容，也受去/回程 chord 反向影响；不能仅凭 signed along 的改变就排除所有 extrinsic residual。相反，固定方向较一致也不能证明存在一套应重新拟合的平移/旋转。

所有 80 个 vertical-normal 分量为正；严格 physical-up 为 **79/80 正**，唯一负值出现在 `100m_cross_5`，为 −0.012907 m。严格 Up 的逐序列结果为：

| Sequence | signed Up median（m） | signed Up IQR（m） | `|Up|` max（m） | Up 正 / 负 |
|---|---:|---:|---:|---:|
| `100m_cross_5` | 0.067544 | 0.076295 | 0.124614 | 18 / 1 |
| `100m_cross_10` | 0.137496 | 0.353640 | 0.785538 | 11 / 0 |
| `200m_cross_5` | 0.351947 | 0.154443 | 0.629154 | 22 / 0 |
| `200m_cross_10` | 0.363877 | 0.242393 | 0.682659 | 18 / 0 |
| `300m_cross_10` | 0.778752 | 0.210387 | 1.027652 | 10 / 0 |

较远序列的 Up/vertical-normal 中位差异较大，与小的空间、GNSS 高度或 antenna-to-body 参考点残差兼容；这些项没有独立测量，不足以确认 residual pitch、roll、translation 或杆臂。详见 [sequence_direction_consistency.csv](sequence_direction_consistency.csv)。

## 同一 chord 内的变化

| Sequence | GNSS segment（0-based） | 覆盖 frame（anchor 个数） | τ 正 / 负 | τ median（s） | τ IQR（s） | τ min … max（s） |
|---|---:|---|---:|---:|---:|---:|
| `100m_cross_5` | 0 | 11–14 (3 个已存 anchor) | 3 / 0 | 0.968763 | 0.703654 | 0.003943 … 1.411250 |
| `100m_cross_5` | 1 | 15–23 (8 个已存 anchor) | 3 / 5 | -0.369397 | 2.151986 | -1.714644 … 1.696119 |
| `100m_cross_5` | 2 | 24–31 (8 个已存 anchor) | 3 / 5 | -0.714334 | 2.462784 | -2.897897 … 1.425578 |
| `100m_cross_10` | 0 | 18–21 (4 个已存 anchor) | 4 / 0 | 0.911128 | 0.496137 | 0.363659 … 1.213747 |
| `100m_cross_10` | 1 | 22–31 (7 个已存 anchor) | 1 / 6 | -1.362514 | 1.152816 | -2.240144 … 1.338099 |
| `200m_cross_5` | 1 | 0–9 (10 个已存 anchor) | 6 / 4 | 0.479654 | 1.533314 | -0.480786 … 2.198442 |
| `200m_cross_5` | 2 | 10–21 (12 个已存 anchor) | 5 / 7 | -0.332180 | 4.231045 | -2.519855 … 6.707545 |
| `200m_cross_10` | 1 | 30–34 (5 个已存 anchor) | 4 / 1 | 0.684191 | 0.636279 | -0.639412 … 1.300931 |
| `200m_cross_10` | 2 | 35–47 (13 个已存 anchor) | 8 / 5 | 0.618722 | 3.051972 | -1.791032 … 3.471995 |
| `300m_cross_10` | 1 | 0–10 (10 个已存 anchor) | 4 / 6 | -0.150282 | 1.028393 | -1.143408 … 1.111570 |

10 个有 anchor 支持的 chord 中 8 个出现两种符号；每条 sequence 至少一个 chord 内变号。一个恒定 time shift 在精确同一线性 chord 上应给出恒定沿轨等效时间；当前数值不符合这一单机制模式。该观察仅说明**常数时钟残差加精确线性运动**不足，不证明不存在真实 clock residual：真实速度变化、折返点、GNSS 点稀疏、关联/参考点误差可叠加其中。完整表见 [chord_time_bias_summary.csv](chord_time_bias_summary.csv)。

## 最大差异帧与邻帧

`200m_cross_5/frame21`，候选 `200m_cross_5_e0003`：

- Bag record time：`1770276618.331014269`；冻结 offset：`122.31215786933898` s；冻结 GNSS query：`1770276740.643172`。
- projected GNSS analysis XYZ：`[209.152855, -11.312745, -4.676603]` m；observed anchor：`[212.235681, -37.691398, -3.720169]` m。
- GNSS→anchor analysis XYZ：`[3.082827, -26.378653, 0.956435]` m；总距离 `26.575401` m。
- 主分量：along `+26.389269` m，cross `-3.056353` m，vertical-normal `+0.719090` m；严格 Up `+0.629154` m。
- 三分量平方误差占比：along `98.604127%`，cross `1.322657%`，vertical-normal `0.073216%`；误差与前向 chord 夹角 `6.785178`°。
- chord speed `3.934267` m/s；沿轨等效时间 `+6.707545` s。
- GNSS 1/3/10 m raw counts 为 **0/0/0**；anchor 1/3 m raw counts 为 **15/15**。这些是原始邻域计数，不是独立 target-point 身份或物理 miss 标签。

此帧正好落在最后原始 GNSS 样本 `A70`，对应 `A69→A70` 的 segment 2：`segment_fraction=1`、nearest-knot distance=0、segment interval=11.725332 s。**冻结线性插值结果在此恰等于原始端点位置，算子没有相对该端点的额外插值偏移；不能据此宣称物理 interpolation error 为零。** 原始 GNSS 位置精度、实际同步与物理参考点仍 UNKNOWN。在已冻结 offset 下，终点支持之外也没有 GNSS；局部 `query + τ = 1770276747.3507166` 超出原始支持，不能把 +6.707545 s 当成可执行的新 offset。

下表所有点均来自同一 `e0003` observed track，同一最终 GNSS chord，位置与计数均沿用原始表；frame index 为 0-based。

| Frame | Bag record time | 总距（m） | Along（m） | Cross（m） | Vertical-normal（m） | τ（s） | GNSS raw 1/3/10 m | Anchor raw 1/3 m |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 15 | 1770276612.315919517 | 5.691893 | -5.656181 | -0.556123 | 0.309826 | -1.437671 | 0/0/19 | 14/19 |
| 16 | 1770276613.308137012 | 2.328348 | -2.226906 | -0.607087 | 0.305846 | -0.566028 | 1/16/16 | 15/16 |
| 17 | 1770276614.326128641 | 3.119557 | 3.000940 | -0.768895 | 0.367143 | 0.762770 | 0/3/12 | 12/12 |
| 18 | 1770276615.320188335 | 8.825108 | 8.728320 | -1.232602 | 0.423868 | 2.218538 | 0/0/14 | 14/14 |
| 19 | 1770276616.309198751 | 15.254384 | 15.133369 | -1.844465 | 0.524699 | 3.846554 | 0/0/0 | 10/10 |
| 20 | 1770276617.325639535 | 20.852423 | 20.670240 | -2.671641 | 0.653480 | 5.253899 | 0/0/0 | 11/11 |
| 21 | 1770276618.331014269 | 26.575401 | 26.389269 | -3.056353 | 0.719090 | 6.707545 | 0/0/0 | 15/15 |

frame15→21 的 along 由 −5.656181 m 经过零附近变为 +26.389269 m；frame17→21 持续增加，末三帧 GNSS 10 m 区域全空但 anchor 邻域保持 10/11/15 个 raw returns。这比**孤立单帧 anchor 跳点**更像连续运动相位、速度或航段对应不一致。持续错误关联仍可形成平滑轨迹，所以不能据此认证 anchor 真值。数据见 [maximum_discrepancy_neighbors.csv](maximum_discrepancy_neighbors.csv)。

## 机制兼容性与尚缺证据

| 候选机制 | 当前模式支持与限制 | 结论等级 |
|---|---|---|
| Time-offset residual | 差异强烈沿轨；既有 closure 记载 200 m boundary offset 与 300 m 多峰。不过同一 chord 内 τ 变号、非恒定；frame21 的局部时间反事实越过终点支持。 | **SPECULATIVE：可能参与；不足以用单一常数 clock residual 解释全部现象。** |
| Extrinsic / reference-point residual | 小的 Up 同号与随距离变化模式兼容；100m_cross_10 固定误差方向较一致。但主误差随同一 chord 的 frame 明显变化，且跨序列固定方向一致性不统一。 | **SPECULATIVE：小的系统空间残差可能存在；未证实为主导来源。** |
| Sparse GNSS interpolation / motion-model error | 每条横飞只有四个原始 GNSS 样本，chord 不是瞬时速度；同 chord 的 along/τ 起伏与变号更兼容未记录加减速/折返、运动相位表达不足。最大帧位于原始端点，不能单独归因于区间内插值。 | **SPECULATIVE：对整体模式较兼容；与真实时钟和航段关联误差无法分离。** |
| Isolated anchor mismatch | 最大值前已有连续沿轨增长，同一候选且有连续 raw support，缺少孤立跳点形态。关联本身仍是 closure-derived selected evidence。 | **SPECULATIVE：孤立单帧错配解释较弱；连续错误候选/航段对应仍 UNKNOWN。** |

区分这些机制需要独立共同时间事件、更密 GNSS/RTK 状态、可核对的独立目标身份，以及测量外参与 antenna/body 杆臂。本次仅指出缺失证据，没有重新拟合或提出待采用的新 calibration。所有结果保留 DEV/SILVER、selected-observation 条件性；不自动升级为 GT，不用于 E18 新评分或方法排名。

## 数值验证与复现

[all_anchor_gnss_discrepancies.csv](all_anchor_gnss_discrepancies.csv) 恰含原有 80 个唯一 observed anchors，逐帧误差向量、单位基、三分量、两种 Up/along 定义、signed τ、原始 record/header 身份、chord/原始单元格身份与邻域计数全部保留。生产脚本及六个输入的 SHA-256、执行命令、Python/NumPy 环境与原文件读取前后 hash 记录在 provenance 中。

独立 [QA](independent_decomposition_audit.json) 在 ENU 内重新构造基，再通过冻结 R 检查 analysis 分量；没有导入生产计算函数。手写 sorted-data linear quantile 核对统计表，检查 80 行身份/计数、primary 正交重建、horizontal-along + cross + strict-Up 第二重建、方向/同 chord 统计及 input/output hashes。**5815 个标量检查 PASS**，最大数值差 `1.08491e-12`（最大为角度数值传播；各字段差值见 JSON），未发现生产脚本实际 bug。首次独立验证器仅因 `numpy.bool_` 类型识别中止，修正验证器后全量通过，失败记录保留在 QA JSON；生产产物没有为此改变。

从仓库根目录复现纯数值计算与验证：

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/rviz_gnss_lidar_sanity/decompose_discrepancies.py --root /home/xin/FMCW_LIDAR --out /home/xin/FMCW_LIDAR/results/rviz_gnss_lidar_sanity
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/rviz_gnss_lidar_sanity/independent_decomposition_audit.py
```

这两条命令只生成/核对本归档的描述性 CSV/JSON，不读取或转换 bag，不运行 ROS，不修改既有 calibration、labels、E18 或历史研究结果。图像与先前 RAW/processed 检查作为诊断材料归档，视觉印象不构成新增真值。
