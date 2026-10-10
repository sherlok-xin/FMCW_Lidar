# FMCW Background-relative Evidence + Motion-coherence DP MVP

日期：2026-10-10。状态：**IMPLEMENTED / TESTED / VERIFIED RESULT，仅限冻结 DEV_SILVER**。独立结果目录：[results/background_motion_dp_mvp](results/background_motion_dp_mvp/README.md)。历史 baseline、DBSCAN/geometry 实现、silver labels 和原 cache 均未修改。

## 结论

这一版通过了运行前冻结的 MVP gate，**未触发 kill criterion**，但不是 operational validation：

- 在同一 fine graph 内，无条件最优路径的 silver 命中从 **56/80（70%）到79/80（98.75%）**；100/200/300 m 分别19/30→30/30、37/40→39/40、0/10→10/10。
- 五条固定 silver feasible witnesses 的排序证据在 **4/5** 序列改善；同一个 silver evaluation window 内相对旧参考的 score margin 也在 **4/5** 改善。100m_cross_5 已在旧 score 下 rank1，保持不变。
- 在完全复现的历史1 m graph上，原五条 persistent-clutter 胜出路径全部明显降分、降序；无条件命中0/80→79/80。这是独立的 score-only 控制，不能把 fine/coarse 候选差异记为打分收益。
- 两套新 score 均保留同一个 miss：200m_cross_5/frame21，输出中心距 silver 约3.254 m，超过原3 m匹配门。**不移动标签、不扩大门限。**
- 79/80是对既有回溯性 LiDAR silver centers 的一致性，不是独立 GT recall；candidate/output volume不是precision。Background和motion的独立增益 **UNKNOWN**，本轮只测试二者的联合版本，未做拆分ablation。

本轮仅完成这一个预冻结版本和独立核查，不继续叠加feature、调整权重或更新正式baseline。

## 1. 输入与公平比较

仓库证据表明，E13的冻结 candidate graph实际来自 **1 m representation**；4 cm cache/candidates是另一条处理分支，不能称其为既有fine graph。本轮因此分成两个明确namespace：

| family | 固定输入 | 本轮允许的变化 |
|---|---|---|
| `fine_4cm_pregeometry`（primary） | E16已保存的全部`adaptive_minPts1` components，geometry acceptance之前；20,122nodes、479,081edges | 只比较旧/新score；不按固定2–32voxel、raw support、extent或旧accepted flag hard删候选 |
| `historical_1m_graph`（control） | 原E13 generator/graph逐包重放；16,995nodes、371,600edges | 只替换score；历史geometry仍保留，原E13胜出路径逐点复现 |

五包共272帧。fine保留7,202singletons和10,325个原geometry-rejected components；不是选择新的geometry threshold，也没有重新运行/调整fine DBSCAN。**这是新增实验支路的候选支持边界，不能称其为未改变E13输入。** 对该支路先冻结graph，再让两套score读同一nodes/edges CSV。因此56→79的对比才是同输入归因；历史0→fine56不是score收益。

输入仍是原broad ROI、intensity和provisional leveled-height之后的cache，不是完整raw bag；本轮没有恢复其之前被删掉的点。位置保留sensor-relative `[x,z,-y]` 坐标；未重新标定或移动labels。80个观察标签覆盖19/11/22/18/10帧，仍为**DEV_SILVER，不是论文GT**。

## 2. 运行前冻结的最小公式

完整参数/函数形式见[protocol.json](results/background_motion_dp_mvp/protocol.json)，源代码身份见[freeze_identity.json](results/background_motion_dp_mvp/freeze_identity.json)。只有这一套全局参数，无bag-specific或range-specific权重，无参数搜索。

原node score不变：

`s_old = 0.50 + 0.30 exp(−extent/2 m) + 0.20 min(log(1+raw_support)/log(6), 1)`。

这些compactness/support只是继承的soft evidence，不被解释为不变的UAV physical size。

### 2.1 因果background-relative evidence

1 m、零原点floor的sensor-relative grid；历史来自全部4 cm有效cache点，不只来自accepted candidates。每帧同cell最多增加一次占用；空帧计入分母。对frame f的所有nodes先query历史，再加入该帧点，**不使用当前/未来点更新当前node背景概率**。

`p_f(cell) = (prior occupied frames + 1)/(prior frames + 2)`；Beta(1,1)使首帧p=0.5。

`b_f = 1−2 p_f`，`s_new = s_old + 1×b_f`。不hard delete、不设置新node threshold；允许负node score。p只是smoothed occupancy proxy，**不是校准过的P(clutter|data)**。Grid不同于历史gravity-aligned occupancy；没有声称复现旧background probability。

### 2.2 Real-dt motion coherence

每条edge的`v=(x_j−x_i)/真实Δt`，相邻edge的`a_dt=(dt_previous+dt_current)/2`：

`speed_penalty = 0.5 [(||v||−||v_previous||)/(15 a_dt)]²`。

`direction_penalty = 0.5 [||v||||v_previous||−v·v_previous] / [||v||||v_previous||+(1 m/a_dt)²]`。

Path score为new nodes总和减去原gap penalty和上述两项；motion weight固定1。两个node的起始路径没有previous velocity，motion term为0。15 m/s²来自原acceleration gate；1 m来自既有occupancy scale，**不是实测位置噪声**。不加入最小速度奖励/门限；静止与匀速运动均可具有低motion penalty。因此motion coherence本身不是target identity evidence。

第一版**完全不用Doppler**，也没有新intensity term。单位权重是运行前指定的dimensionless contrast/penalty，并非通过silver拟合的最优权重。特征选择受既有failure forensics启发，不能称为blind held-out研究。

### 2.3 Graph/search保持与正确性

原numeric gates/pruning保留：frame gap≤3、dt≤3.5s、speed≤25m/s、每prior frame最近12predecessors、acceleration≤15m/s²、每跳过一帧罚0.25、至少3nodes且duration≥2s。没有hard M-of-K或单帧detection threshold。

负分下，旧“一edge只保留一history，再post-hoc查资格”的代码可能让不合格短history遮掉合法长history。新独立solver在资格形成之前保留`(capped count,start frame)`，形成之后保留一个qualified winner；**两套score共用相同solver**。这是资格处理正确性，不是新增target feature。所有五条历史old-score胜出路径均复现；小图exhaustive tests覆盖两套score与负node scores。没有修改历史solver源文件。

每包输出一条最高累计score的合格离线路径，不设score threshold。**Background features因果不等于整个DP输出online**：DP仍遍历完整bag。

## 3. Rank含义与防止circular evaluation

Freeze先于graph构造和真实评分。`run`不解析/使用labels，完成20次full-bag无条件选择并保存hash之后，`evaluate`才解析labels。提前读取labels字节仅为完整性SHA，并不参与推断。

- Historical graph：原E14五条silver feasible paths和原persistent-clutter full/window paths身份固定。
- Fine graph：在evaluation阶段仅按**旧score**的lexicographic银标匹配构造一个feasibility witness，同时固定旧score的无条件full winner及其window限制。**同一reference path**随后被旧/新score重算；不按新score另选silver oracle。五包两family均有覆盖全部80labels的合格witness。
- Witness是silver-guided存在性证据，不能当无条件检测输出。Silver gate沿用3 m。
- 排序池是每个合格terminal edge的最佳history representatives，**不是全组合top-K paths**。strict-exceedance count+1是保守rank下界；只有reference本身保留在池内，才报告池内exact competition rank。下表`≥`表示下界；全组合path rank一律**UNKNOWN**。
- Window为同一个silver frame区间；本次两reference实际端点与duration也相同（所有duration delta=0）。Full-bag old winner更长，不能把它与短silver witness的raw累计分直接解释为同duration竞争。

Fine的CSV保留schema名`top_clutter_*`，但`reference_role`明确为**OLD_SCORE_UNCONDITIONED_WINNER_NOT_ASSUMED_CLUTTER**。它们并非全是clutter：100m_cross_5旧路径已命中全部19labels，另外两条200 m旧路径也有大量命中。

## 4. 五条fixed silver witnesses

### 4.1 Primary fine graph

Score为固定witness在matched window内的累计分；margin为witness减去固定old winner的同window分。

| 序列 | witness score 旧→新 | witness代表池rank 旧→新 | margin 旧→新 | 无条件hits 旧→新 |
|---|---:|---:|---:|---:|
| 100m_cross_5 | 19.665→35.497 | 1→1 | 0→0 | 19/19→19/19 |
| 100m_cross_10 | 11.843→22.909 | 296→1 | −1.295→9.986 | 0/11→11/11 |
| 200m_cross_5 | 20.583→34.155 | 3→1 | −0.065→0.569 | 20/22→21/22 |
| 200m_cross_10 | 16.771→32.151 | ≥5→≥2 | −0.094→−0.034 | 17/18→18/18 |
| 300m_cross_10 | 8.992→13.821 | 291→≥8 | −0.949→8.518 | 0/10→10/10 |

保留null/negative：第一包ranking/margin均无增益；200m_cross_10固定witness仍略输给旧参考；300 m固定witness仍有7个代表分更高，不是rank1。无条件full输出可以选另一条银标邻域内路径，所以fixed-witness rank不完美与full输出10/10并不矛盾。没有用新score重新选择更好silver reference。

### 4.2 Historical graph score-only控制

| 序列 | fixed witness score 旧→新 | witness rank 旧→新 | 原full clutter score 旧→新 | 原full clutter rank 旧→新 |
|---|---:|---:|---:|---:|
| 100m_cross_5 | 20.412→36.297 | 4→1 | 31.158→12.381 | 1→≥26,720 |
| 100m_cross_10 | 12.196→23.241 | 259→1 | 44.745→6.394 | 1→≥50,944 |
| 200m_cross_5 | 21.094→34.729 | 9→1 | 81.084→5.610 | 1→≥105,146 |
| 200m_cross_10 | 17.227→32.424 | ≥21→≥2 | 46.979→5.555 | 1→≥58,323 |
| 300m_cross_10 | 9.100→13.919 | ≥466→≥4 | 61.480→5.536 | 1→≥76,919 |

本轮rank来自共同资格正确solver的代表池，不把其retention状态当E14原pool或全组合rank。全部同window margin由负转正：−0.126→31.275、−1.455→22.167、−0.301→29.985、−0.389→32.125、−1.586→10.063。

## 5. Primary旧full参考变化（不全部叫clutter）

| 序列 | old full reference score 旧→新 | rank 旧→新 | 旧路径silver hits |
|---|---:|---:|---:|
| 100m_cross_5 | 29.985→49.673 | 1→≥2 | 19/19 |
| 100m_cross_10 | 42.099→36.974 | 1→≥24,301 | 0/11 |
| 200m_cross_5 | 77.366→117.119 | 1→≥13,900 | 20/22 |
| 200m_cross_10 | 44.930→76.195 | 1→≥2 | 17/18 |
| 300m_cross_10 | 57.535→9.036 | 1→≥94,593 | 0/10 |

不能要求UAV-like旧参考也必须降分；score变高但rank变差也不矛盾，因为竞争路径score变化不同。完整base/compactness/support/background/gap/speed/direction分解保留在[fixed_path_scores.csv](results/background_motion_dp_mvp/fixed_path_scores.csv)，共60个fixed-path rescore records。

历史static参考的background项分别−16.068/−37.769/−74.972/−41.042/−55.399，speed+direction penalty约2.709/0.582/0.502/0.382/0.545。**分解显示这些固定路径的降分主要来自background contrast，不是motion penalty**；但这不是background-only性能ablation，不能因此宣布motion独立增益。

## 6. 100/200/300 m无条件结果与volume/runtime

| graph | range | silver recall 旧→新 | candidates/frame（两score完全相同） | outputs/frame 旧→新 | runtime秒 旧→新 |
|---|---:|---:|---:|---:|---:|
| fine | 100 m | 19/30→30/30 | 73.846 | 0.987→0.962 | 10.35→13.17 |
| fine | 200 m | 37/40→39/40 | 74.435 | 0.992→0.992 | 18.47→23.63 |
| fine | 300 m | 0/10→10/10 | 73.190 | 1.000→0.873 | 8.84→11.21 |
| historical | 100 m | 0/30→30/30 | 62.218 | 1.000→0.962 | 7.30→9.59 |
| historical | 200 m | 0/40→39/40 | 63.130 | 1.000→0.992 | 13.52→17.02 |
| historical | 300 m | 0/10→10/10 | 61.460 | 1.000→0.857 | 6.48→7.89 |

分组volume按全部bag帧加权，而recall分母只含观察到的silver帧。没有可靠negative labels，**operational precision、false alarms/frame、PR curve均UNKNOWN**。一条full-bag路径在未标帧的身份也UNKNOWN。更少output不证明更高precision。

Runtime包括DP、代表池重建和结果导出，不含raw bag decoding/cache/候选生成，也不是onlineFPS。整个无条件阶段151.837s，包括两family×五包×两score及加载/导出。全部逐包runtime保存于`unconditioned_summary.csv`。

唯一miss的机器记录见[observed_miss_diagnostic.csv](results/background_motion_dp_mvp/observed_miss_diagnostic.csv)。该帧有candidate、有覆盖全部silver的fixed feasible witness，但两套新full赢家均选了距silver约3.254m的另一个center。它是冻结3 m门下的**observed localization/association miss**，不是证明sensor non-return，也不是缺graph evidence；不凭该帧调权重或移label。

## 7. Kill criterion与决策

预声明：ranking strict-exceedance reduction≥4/5、window score-margin improvement≥4/5（分别计数），且旧参考仍占优<3/5、新零命中near-static winner<3/5、至少一个silver hit恢复；若无完整可评估witness，单独STOP_INSUFFICIENT_WITNESS。Near-static诊断尺度为1 m/median真实bagΔt，不参与算法gating。

| family | rank改善 | margin改善 | 新score下旧参考window仍≥UAV | 新near-static零hit赢家 | 新无条件hits | 判定 |
|---|---:|---:|---:|---:|---:|---|
| fine primary | 4/5 | 4/5 | 2/5 | 0/5 | 79/80 | MVP_GATE_PASSED_NOT_OPERATIONAL_VALIDATION |
| historical control | 5/5 | 5/5 | 0/5 | 0/5 | 79/80 | MVP_GATE_PASSED_NOT_OPERATIONAL_VALIDATION |

Fine的2/5包含100m_cross_5相同UAV-like参考的tie、200m_cross_10轻微劣势，**不是两条persistent static clutter仍胜出**。历史五条原static参考均失去优势。故这一个冻结组合在当前DEV图上确实缓解了原static-clutter排序偏好，没有触发停止该方向的预声明kill；不升级Benchmark v1、不选择最终operating point、不宣称论文独立检测效果，也不自动继续更多实验。

## 8. 局限、复现与验证

- 只有既有五条cross序列/80个observed silver centers；不是9包全部、longitudinal或held-out/negative测试。方法设计使用过历史failure知识，未在独立test上验证。
- Center-cell occupancy proxy可能漏掉centroid所在cell为空的大component，可能惩罚停留/重访UAV，未估计clutter likelihood校准质量。No hard deletion不等于zero target suppression。
- Graph仍有原nearest12/frame、最大speed/acceleration和gap剪枝；其对其它未观察弱returns的召回UNKNOWN。无threshold单最大path也无法证明target-absent时不报警。
- Soft compactness/support继承自原node函数；本轮不验证它们是物理invariants。每帧raw support、fine cells、UAV physical size仍需分开。
- 没有background-only/motion-only或Doppler试验，因此独立贡献UNKNOWN。本轮没有为提升结果另跑版本。
- Sourcefreeze：2026-10-10 09:48:37.422 UTC；run：09:49:17.523–09:51:49.360 UTC。Parent `bf149c8eaeea1734736ab589285103b27d10c26b`；dirty-run以source SHA为身份。
- Wrapper SHA `9c536437b0e8ec6ddb75b9caa9d7da7910d319fef2ddf0fc7b76cf5e2757c0d3`；core SHA `f056d21196afc3c7b89366c4d15b2ddc5aaa77bb3fd9f9b9c8f9ce2efbcb875f`；protocol SHA `ef29bba4a15e909248fc5b790d1401938e3799ffcd9c2a55f19826545f18cb36`。
- Labels SHA不变：`37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db`。Python3.10.12、NumPy2.2.6、SciPy1.15.3、scikit-learn1.7.2；无随机实验。
- 14个synthetic unit tests通过（含两score exhaustive/负分资格、past-only prefix invariance、真实dt、Doppler不访问、rank资格/serialization/decomposition）。独立causal审核逐个重建37,117nodes并验证850,681edges、20graphCSV与124protected文件。结果数值独立复核见独立audit JSON。
- 另7个历史synthetic regression assertions直接调用通过。`fmcw`没有pytest，尝试pytest命令报`No module named pytest`，未安装/改环境；新tests用unittest执行，旧tests无fixtures故直接调用。该环境限制与通过记录保存在`validation_checks.json`，没有据此新增真实数据score试验。

精确命令、有效/实际graph invocation、CSV字段解释及机器artifact映射见[结果README](results/background_motion_dp_mvp/README.md)。Raw bags/PCDs和NPZ cache不上传；哈希记录保留，另机完整重建的输入可用性UNKNOWN。**先保存这一版完整证据，不根据79/80调整labels、参数或论文结论。**
