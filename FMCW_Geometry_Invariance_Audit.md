# FMCW Geometry Invariance Audit

日期：2026-10-10。E17；**只读描述性诊断，DEV/SILVER，不是独立论文GT**。

## 结论：A / B / C

**A：voxel count明显依赖距离和representation，不宜解释为固定UAV物理size prior。** 100/200/300 m的raw点数中位数为39.5/17.5/5，4 cm cells为27.5/14/5；同一批过滤后raw children改用1 m cells后为2/2/1.5。31/80帧变为单cell，其中30帧原来有多个raw returns。当前2–32 voxel规则的9个miss是8个近距离计数过多、1个远距离singleton，不是9个物理尺寸异常。

**B：部分米制observed geometry比计数更稳定，但没有证明跨距离不变。** raw diameter的三个组中位数最大/最小比为1.79，低于raw点数7.9、4 cm cells5.5；raw最大轴向span为1.92。另一方面，x-span和range thickness比值仍达4.88/3.73，300 m有退化singleton，邻域外围点影响尺寸尾部。z-span在3 m邻域的组中位数接近，但1 m敏感性比值变为1.33，不能挑选它作为已验证的固定规则。

**C：这80个已观察到的正银标样本，没有验证任何UAV-specific单帧geometry feature可用于固定hard rejection。** 计数上下限已删除银标；物理尺寸、NN/MST、intensity既有采样/姿态/表示依赖，又没有独立point GT及可靠negative distribution。结论是“当前证据不支持”，不是证明所有可能规则永远无效。没有设计新threshold、扫参、修改labels或算法。

## 1. 数据范围与冻结边界

来源为[E15 range/sparsity](FMCW_Range_Sparsity_Audit.md)及[E16 DBSCAN attribution](FMCW_DBSCAN_Fragmentation_Attribution.md)，只取既有`research_dev_v0`的80个**observed** silver observations，不插值、不加帧、不加入旧纵向pseudo observations。

| 名义距离 | 序列 | observations | 实际silver center range / m | E16 candidate-stage hit / miss |
|---|---|---:|---:|---:|
| 100 m | cross_5、cross_10 | 30 | 102.06–117.61 | 22 / 8 |
| 200 m | cross_5、cross_10 | 40 | 201.70–215.59 | 40 / 0 |
| 300 m | cross_10 | 10 | 293.60–305.14 | 9 / 1 |
| 合计 | 5条横飞序列 | 80 | — | 71 / 9 |

分母是帧，不是汇总后的所有点；每帧在统计中权重相同。连续帧非独立样本，且距离与序列/速度/姿态混杂。所有分组、完整分位数及逐序列描述性range关联均保留CSV；没有p-value、置信区间、feature选优或operational precision。

主分析沿用3 m center neighborhood；1 m仅为**原有**敏感性分析。这些是银标中心周围的raw returns，**没有证明每一点都属于UAV**。因此本文“target support”均指这种中心邻域proxy，不是点身份GT；更不是机体真实长宽高。

## 2. 字段、坐标与representation语义

- raw：E15有限且非全零XYZ returns；sensor-relative坐标`[x,z,-y]`，单位m。没有重新标定或旋转几何坐标。
- XYZ extent：各轴`max−min`；`max_axis_span`取其中最大值。受坐标方向、姿态和可见面影响。不是body-frame尺寸，也不是拟合bounding box。
- diameter：同一observed点集的最大欧氏点间距。对该点集的刚体变换不变，但换成voxel means后**点集已改变**。
- range thickness：`max(||p||)−min(||p||)`，相对于LiDAR原点；不是机体厚度。
- raw NN：每个return的最近另一return距离，再取帧中位数；重复XYZ允许0。unique NN先去重。MST是unique XYZ的最小生成树最大边。两者描述采样/连通性，不直接描述机体大小。
- raw intensity：原字段的逐帧中位数；单位、绝对反射率标定、跨配置可比性为**UNKNOWN**。
- raw support：过滤后children数，或saved component的children数之和；和raw count有明确lineage，不是额外独立传感证据。
- counts既保留E15 raw occupied cells，也保留E16相同point-filtered children的4 cm/1 m cells，以及actual fine cache/old coarse cache的邻域support。原raw总数1883，既有point filter后1877；不混用两种分母。
- isolated proxy cell means：沿用E16过滤、固定零原点`floor(xyz/size)`，只从同一邻域children重建arithmetic means，描述shape collapse；**没有运行DBSCAN或detector**。重建means用double，actual cache最终float32；不宣称cache centroid字节级重放，也不把isolated proxy当完整scene component。
- full-scene component：另从E16已存CSV读取最近silver center的component及其voxels/raw support/max-axis extent和rejection flags；不重新聚类。诊断lookup不等于UAV membership。frame hit仍按原定义“3 m内存在accepted component”校验。

`300m_cross_10/frame4/fixed`是特别例子：最近component是距中心0.258 m的拒绝singleton，但距0.909 m另有2-voxel/3-raw的accepted component，原frame为HIT。CSV保留这种nearest≠hit，不改变原结果。

## 3. 距离与计数：不能把采样数当机体size

主3 m邻域，下表为**median [Q25,Q75]**；完整min/max见`feature_summary.csv`。

| 特征 | 100 m，n=30 | 200 m，n=40 | 300 m，n=10 |
|---|---:|---:|---:|
| raw returns | 39.5 [32.5,45.75] | 17.5 [10.75,22.25] | 5 [3.5,10.25] |
| 同children过滤后raw support | 39.5 [32.5,45.75] | 17.5 [10,22.25] | 5 [3.5,10.25] |
| 同children 4 cm cells | 27.5 [22.25,33.5] | 14 [9.75,17] | 5 [3.25,7.75] |
| 同children 1 m cells | 2 [1,2] | 2 [1,3] | 1.5 [1,2] |

未过滤的raw occupied 4 cm/1 m cells组中位数恰好相同，但逐帧值和分位数不应据此当成等价；CSV分列记录。没有据此重跑或改动正式geometry规则。

同children表示对照：<2 cells由4 cm的1/80变为1 m的31/80；<3 cells由1/80变为61/80。这里的2/3只是已有support限制的描述，不是新增threshold实验。

31个1 m单cell分布在100/200/300 m的12/14/5帧。它们原raw count中位数21、范围1–52；原raw diameter中位数0.347 m、范围0–0.893 m。**30/31不是raw singleton**。cell-mean表示的extent/diameter变0，是内部shape被压缩丢失，不是UAV真实尺寸为0，也不是sensor无返回。

## 4. 米制observed geometry：相对稳定不等于invariant

仍为3 m主分析，单位m，median [Q25,Q75]；NN/MST的有效n为30/40/9，far singleton为`UNKNOWN`而非0。

| 特征 | 100 m | 200 m | 300 m |
|---|---:|---:|---:|
| XYZ spans（各轴median） | 0.373 / 0.371 / 0.201 | 0.194 / 0.436 / 0.189 | 0.076 / 0.162 / 0.191 |
| 最大轴向span | 0.456 [0.341,0.626] | 0.436 [0.249,0.690] | 0.238 [0.133,0.748] |
| diameter | 0.524 [0.384,0.696] | 0.467 [0.291,0.830] | 0.292 [0.141,0.771] |
| range thickness | 0.410 [0.235,0.552] | 0.205 [0.138,0.292] | 0.110 [0.060,0.168] |
| raw NN median | 0.0229 [0.0205,0.0248] | 0.0400 [0.0200,0.0423] | 0.0356 [0.0200,0.0596] |
| unique NN median | 0.0230 [0.0227,0.0257] | 0.0404 [0.0350,0.0436] | 0.0595 [0.0356,0.0596] |
| MST最大边 | 0.224 [0.130,0.337] | 0.220 [0.164,0.531] | 0.278 [0.077,0.776] |
| intensity median（原单位） | 40.25 [38,41.875] | 30 [27,32.125] | 23.5 [22.125,25.75] |

三个组median最大/最小比：raw count7.90、fine cells5.50、diameter1.79、max-axis1.92、x-span4.88、y-span2.70、z-span1.06、range thickness3.73、unique NN2.59、MST1.26。**这些是描述性摘要，不是稳定性检验、feature排名或threshold候选选择。** MST的median近似并不能消除其长尾或证明target/clutter separation。

representation本身也影响米制几何：相同children的1 m means diameter组median变为0.182/0.226/0.067 m，相比raw的0.524/0.467/0.292 m进一步缩水。raw与fine diameter组median相同，不意味着所有帧形状不变；逐帧fine/coarse spans、diameter、range thickness、NN/MST均在CSV。

敏感性：1→3 m扩大邻域仅6/80帧加入额外returns，但raw diameter最大值由1 m的1.158/1.483/1.165 m变成3 m的2.497/2.996/3.408 m。MST最大值也由0.711/1.151/1.157 m变成2.133/2.401/2.248 m。外围点是否属于UAV为**UNKNOWN**；不能把3 m大尺寸尾部都当机体，也不能为了得到紧凑分布删除它们或挑较好半径。

singleton的observed XYZ/diameter/range thickness为0，NN/MST不可定义。`200m_cross_10/frame37`另有29 returns、21 unique XYZ，raw NN median=0但unique NN约0.010 m；零NN不能被自动解释为clutter。

## 5. 2–32 voxel rule的9个miss

这里解释E16原有candidate-stage结果，不是重新运行或评估temporal detector。fixed/adaptive和minPts1/2的80帧hit flags都一致；nearest saved components的9个miss拒绝原因也一致，但component尺寸不完全一致。

| condition / frame | raw / 4 cm / 1 m proxy数 | fixed主component voxels / raw support | raw diameter / m | fixed主component max-axis / m | 原rejection |
|---|---:|---:|---:|---:|---|
| 100m_cross_10 / 19 | 46 / 40 / 2 | 40 / 46 | 0.713 | 0.641 | >32 voxels |
| 100m_cross_10 / 23 | 44 / 35 / 1 | 35 / 44 | 0.362 | 0.339 | >32 voxels |
| 100m_cross_10 / 24 | 46 / 34 / 2 | 34 / 46 | 0.516 | 0.458 | >32 voxels |
| 100m_cross_5 / 16 | 48 / 38 / 1 | 38 / 48 | 0.565 | 0.455 | >32 voxels |
| 100m_cross_5 / 20 | 57 / 47 / 7 | 46 / 56 | 1.583 | 0.813 | >32 voxels |
| 100m_cross_5 / 27 | 55 / 43 / 2 | 43 / 54 | 0.590 | 0.547 | >32 voxels |
| 100m_cross_5 / 28 | 45 / 39 / 1 | 39 / 45 | 0.598 | 0.582 | >32 voxels |
| 100m_cross_5 / 30 | 52 / 36 / 1 | 36 / 52 | 0.382 | 0.268 | >32 voxels |
| 300m_cross_10 / 2 | 1 / 1 / 1 | 1 / 1 | 0 | 0 | <2 voxels |

proxy列的cells采用相同point-filtered children；raw列为过滤前returns。frame27的1个过滤掉的return没有伪装成component support；frame20的外围point不属于fixed主component，所以47-cell proxy不能替代46-voxel component。

8个near miss的**fixed** saved主component为34–46 voxels、44–56 raw support、max-axis0.268–0.813 m。adaptive在frame20合并外围point后为47 voxels/57 raw/max-axis1.583 m，其余miss component描述不变；拒绝原因仍仅为>32 voxels。**没有一个因raw>120或max-axis≥6 m被删**。其中frame30包含52个raw returns，但diameter仅0.382 m、max-axis0.268 m、MST0.135 m，仍因36 voxels失败，是count不等于physical size的直接反例。

### 与命中帧的公平描述对照

全部71HIT/9MISS的CSV都保留；下面单列100 m的22HIT/8MISS，以避免把唯一far singleton与near high-count miss混成一个“典型miss”。HIT/MISS是同一既有count规则的结果，**不是target/clutter二分类**。

| 特征，median [Q25,Q75] | 100 m HIT，n=22 | 100 m MISS，n=8 |
|---|---:|---:|
| raw returns | 36 [27.75,40] | 47 [45.75,52.75] |
| 同children fine cells | 25.5 [21.25,28] | 38.5 [35.75,40.75] |
| raw diameter / m | 0.489 [0.361,0.713] | 0.577 [0.483,0.627] |
| raw最大轴向span / m | 0.450 [0.296,0.626] | 0.503 [0.426,0.597] |
| raw NN median / m | 0.0224 [0.0204,0.0252] | 0.0229 [0.0228,0.0246] |
| raw MST最大边 / m | 0.238 [0.125,0.345] | 0.203 [0.142,0.308] |
| intensity median | 40.75 [38,42.375] | 39.25 [38.625,41.125] |

saved全scene主component的max-axis median/IQR为HIT0.441 [0.253,0.531]、MISS0.503 [0.426,0.597] m；范围分别0.134–1.059和0.268–0.813 m。**不是raw球的max-axis**。物理extent分布重叠，没有显示miss是一类异常大机体；NN/MST和intensity也不表现为near miss的特殊退化。count分离是count规则本身的条件化结果，不能称作发现了新判别特征。

frame20对1/3 m尤其敏感：1 m球56 returns/46 fine，raw diameter1.003 m、max-axis0.813 m、MST0.352 m；3 m球57/47，diameter1.583 m、max-axis1.583 m、MST0.971 m。fixed主component仍是max-axis0.813 m。额外point身份UNKNOWN，不手工清理。

far singleton frame2 intensity26；不是0 intensity或无返回。diameter=0不能作为物理过小的证据。minPts1已保留它进入component，后续既有geometry nmin2仍将其删掉；这里不更改任何规则。

## 6. 为什么不能据此认证固定hard rejection

| 特征 | 此次可验证的限制 | 固定UAV-specific rejection是否获验证 |
|---|---|---|
| raw/voxel count、raw support | range/representation/point-filter依赖；8 near max32和1 far min2删除银标；support并非独立证据 | 否 |
| XYZ/max-axis/diameter/range thickness | observed support不是全机体；singleton退化；方向/视角/邻域/means影响；near hit/miss重叠 | 否 |
| NN/unique NN/MST | 采样、重复returns、数量、外围point影响；singleton UNKNOWN；非机体尺寸 | 否 |
| intensity | 明显range关联；原单位/传感器标定UNKNOWN；本轮无negative distribution | 否 |

既有max-axis<6 m和raw≤120在这些selected目标proxy/component上没有删除，并不验证它们能拒绝clutter。尤其**半径3 m球内diameter≤6 m是几何构造的必然上界**；从这种截取结果“发现6 m尺寸安全”会是circular证据。saved full-scene component并非限于此球，须与proxy分开；即便本集合的saved主component未触及6 m，也只有selected positive evidence。

UAV型号/真实尺寸/飞行姿态、beam/angular sampling规格、弱返回漏帧point身份、独立negative/held-out distribution均为**UNKNOWN或未建立**。因此无法认证UAV-versus-clutter固定hard rejection，也不能用这80帧挑一个包住所有点的新上限。本文不提出替代算法或下一轮调参方案。

## 7. 复现、验证与文件

Material Passport：Origin Skill=`academic-research-suite/experiment-agent`；Mode=`validate`；Date=`2026-10-10`；Version=`geometry_audit_v1`；Verification=`VERIFIED`仅限artifact join和描述数值，scientific generalization=`CAUTION`。

skill要求的复现/统计解释检查促使保留：所有帧、两个旧半径、按序列/距离/命中状态的全部分布、不可定义值分母、负结果，以及sampled support和真实尺寸的边界。没有据此扩展算法工作。

- Git run parent：`8912fb8eb67dd441483fb378ddb5bb532fbcfd1a`。
- 脚本SHA-256：`8900eb3f3e6aa3116e1514ed231d80adbf3e68cd4a1756af045ae8250e1a533c`。
- protocol SHA-256：`c0961702c184d7d3cf0bd0fe8d88d56c50d5f2e82242b5cd6b434ece9fd97438`。
- unchanged silver labels SHA-256：`37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db`。
- Python3.10.12、NumPy2.2.6、SciPy1.15.3、Linux；无随机过程。主run为2026-10-10 08:14:44–08:14:45 UTC，1.5005 s，仅descriptor审计，**不是detector runtime**。

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python -m unittest discover -s tests -p test_geometry_invariance_audit.py -v
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/geometry_invariance_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/geometry_invariance_audit.py run
```

`freeze/run`拒绝覆写已完成目录；命令是原始执行记录，不应直接在已有结果目录重跑。现有baseline、source protocol、labels、缓存、历史结果均未改：**100 protected files SHA一致**。6个synthetic descriptor/lookup tests通过；80帧time/center与旧四种hit flags校验一致；160行raw descriptors重算误差≤1e−12；同children cell counts/support与E16一致。独立复核1802行分组摘要、106行median ratios全部一致，并独立重算1440项三个representation的NN/unique NN/MST，误差≤1e−12。

全部机器可读输出在[`results/geometry_invariance_audit/`](results/geometry_invariance_audit/)：

| 文件 | 数据行 | 内容 |
|---|---:|---|
| [observation_features.csv](results/geometry_invariance_audit/observation_features.csv) | 160 | 80×1/3 m：raw count/cells/XYZ/diameter/range thickness/NN/MST/intensity，旧representation支持和同children mean geometry |
| [feature_summary.csv](results/geometry_invariance_audit/feature_summary.csv) | 1802 | 所有分组n、有效n、min/Q25/median/Q75/max；空组为UNKNOWN |
| [range_stability.csv](results/geometry_invariance_audit/range_stability.csv) | 106 | 3个距离组的描述性median比例，不选feature |
| [range_associations.csv](results/geometry_invariance_audit/range_associations.csv) | 636 | 总体与5序列的真实range Spearman描述，无p-value/因果主张 |
| [existing_component_geometry.csv](results/geometry_invariance_audit/existing_component_geometry.csv) | 160 | 80×fixed/adaptive已存nearest component及geometry/rejection；nearest与hit分开 |
| [miss_geometry.csv](results/geometry_invariance_audit/miss_geometry.csv) | 9 | 9个primary miss完整特征及fixed component原因 |
| [join_validation.csv](results/geometry_invariance_audit/join_validation.csv) | 80 | time/center/hit/raw descriptor/representation join校验 |

另有`protocol.json`、`FROZEN`、`input_identity.json`、`freeze_identity.json`、`run_manifest.json`、`verification.json`、`review_checks.json`。原raw ZIP/bag/PCD和cache未复制、改写或上传。

### 统计解释检查：11/11

| 检查项 | 本轮处理 / 边界 |
|---|---|
| Simpson | 总体与各序列range描述都保存；不把总体相关当序列内定律 |
| Ecological | 组median接近不是每帧不变；保留IQR/min/max |
| Berkson | observed silver为selection-conditioned positives，不能推广全部弱目标 |
| Collider | hit/miss由既有geometry规则产生，count差异不是独立分类发现 |
| Base rate | 没有reliable negatives；不报告precision、specificity或false alarms |
| Regression to mean | 无pre/post improvement结论；same-child表示对照不声称改进算法 |
| Survivorship | 所有80银标及9miss保留；原银标未覆盖的漏帧仍UNKNOWN |
| Look-elsewhere | 全部指标输出，无p-values、挑feature或阈值搜索 |
| Forking paths | 分组/描述冻结；保留旧1/3 m敏感性，不清理外围点 |
| Correlation≠causation | 跨距离观测关联与同children表示变化分开，不把姿态/速度混杂归因于range alone |
| Reverse causality | 没有从count/shape与range相关推断传感器或目标因果机制 |

**本轮结束：诊断记录到此，不改变正式方法、不推荐新threshold、不启动后续算法实验。**
