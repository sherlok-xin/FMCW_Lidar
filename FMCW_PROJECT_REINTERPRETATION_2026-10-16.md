# FMCW 项目重新解释与历史流水线重置

日期：2026-10-16。范围：**只读历史审计＋解释重置＋下一步计划，非新算法实验。** 不改变E18、labels、GNSS/calibration/offset、graphs、阈值或历史结果；不继续Stone Soup、不安装其它repo、不新读bag/PCD计算结果、不优化80个anchors。未提交/推送。

**当前最简单可辩护的描述：固定传感器在较开放的监视空域中，从稀疏、间歇且可能混有孤立干扰的FMCW回波中，保留目标相关证据并恢复时间连贯轨迹。** 原始扫描的低处密集静态点，与真正需要监视的空域，不应混为一个难度相同的分类问题。当前证据支持工程血缘和选定正例的机制诊断，尚不支持独立目标存在性检测、精确定位或泛化精度。

## 1. 新采集背景：接受事实，区分未测量部分

用户在本次任务说明：传感器固定在静止车辆上；室外场景开阔，树木/地形/路侧等密集回波多在较低位置且持久；UAV约半米，在多数密集背景上方的开放空域飞行；RViz里回波稀疏但局部集中，近邻密集物体通常少，仍有孤立异常点。这些是 **USER_PROVIDED_ACQUISITION_CONTEXT**，作为解释输入接受，不是本轮独立测量，也不作为新标签。

这意味着不必先假定ego-motion补偿/复杂城市目标分离是主要矛盾；固定场景的持久性有清楚物理解释。但静止不等于传感器水平，开放不等于没有竞争点，半米机体不等于每帧观测bbox半米。真实安装pose、地面高度、可靠FOV/飞行高度、radial velocity标定和全时段presence仍是DATA_GAP。

GNSS表只作为粗飞行参考。每条横飞仅四个采样点及其chord，不是逐帧LiDAR目标坐标。原closure的PASS只表明选定轨迹/模型满足内部一致性门槛，不认证precise GT、硬件时钟或外参。冻结值继续保留，不重拟合。

## 2. 从底层重建：历史与当前不是同一条链

```text
历史可确认的源级子链
native PointCloud2 → get_lidar：五字段repack，native xyz不动
                 → [缺失的历史PCD exporter/命令]
PCD → multi_filter：q=[x,z,-y] → I∈[10,250] → compressed filtered PCD
                 → Open3D速度着色代码可见
                 → [师兄清洁云RViz publisher/topic/config 未恢复]

当前研究 fine/E18 链
raw bag named字段 → rich80执行q=[x,z,-y] → finite/nonzero
→ broad ROI → 点级I[10,250] → closure-derived provisional level z>10 mask
→ 4cm均值体素 → E16保存全部adaptive_minPts1 pregeometry components
→ E18 past-only1m occupancy + real-dt motion → 离线全包best path
```

[完整预处理审计](FMCW_HISTORICAL_PREPROCESSING_AUDIT.md) 给出每项来源、参数和分歧版本。历史PCD脚本**没有**height/ROI；z5、z10、I20是其它共存tracker支路。当前ROI/leveled-height/4cm是后来的研究适配。height mask使用`R.T@(q-t)`第三坐标，存储位置仍是q，不能把fine缓存当已经旋转到真实重力坐标。

因此E18处理的是过滤后的candidate图，而非完整raw场景；“pregeometry”只是未使用旧count/extent acceptance，不能恢复更早已删的点。已有[scene audit](FMCW_SCENE_SPATIOTEMPORAL_BACKGROUND_AUDIT.md)记载fine保留support **1,922,878/30,941,328=6.2146%**；这些是已有cohort按原定义聚合的return计数，不是UAV保留率，更不是各阶段单独删除比例。此前E18和temporal报告已经披露输入过滤，问题在高层解释丢失了这个条件。

早期[Sparse-UAV报告](FMCW_Sparse_UAV_MVP.md)也已记载师兄三阶段链。不能改写成“以前完全不知道”；真正缺失的是可重复的版本、导出/发布连接和逐阶段定量/视觉验证。

## 3. 主要历史结论重新分类

分类表完整版：[conclusion_reassessment.csv](results/project_reinterpretation_20261016/conclusion_reassessment.csv)。分类针对具体结论/解释，不能一整篇一票否定。

| 主题 | 分类 | 保留什么，改正什么 |
|---|---|---|
| v0 GNSS小球空、projected center | PREVIOUSLY_MISUNDERSTOOD | 原v0已标low confidence；空球不能推出UAV无回波，GNSS不能定精确中心 |
| GNSS closure / provisional 6DoF | VALID_BUT_NEEDS_REINTERPRETATION | 保留选定轨迹内部对应与冻结变换；不是测绘标定或逐帧GT；边界offset/多峰保留 |
| GNSS-anchor decomposition | STILL_VALID | 最大26.575401m主要沿chord；连续残差不支持只叫孤立错点；不能把等效时间重设为offset |
| 80 DEV_SILVER anchors | VALID_BUT_NEEDS_REINTERPRETATION | 是LiDAR/closure选择的observed邻域，非272帧完整标签；缺anchor不代表absence；共享校准来源 |
| raw sparsity与体素化 | STILL_VALID | 31/80邻域在1m只有1cell，其中30不是raw singleton；也保留真实far singleton，不能用体素数当物理大小 |
| adaptive DBSCAN | VALID_BUT_NEEDS_REINTERPRETATION | connectivity65→76/80，matches71→71；实际单帧scene-mean epsilon，不是UAV瞬时range-specific；合并杂波/硬geometry影响保留 |
| geometry audit | STILL_VALID | 8个近距compact例被>32fine-cell拒绝，span0.268–0.813m；另有far singleton被min2删。半米先验不能恢复硬count正当性 |
| DP failure | STILL_VALID | persistent competitor路径存在，长时累积可能胜过稀疏target-like证据；开放空域并不消除这种机制 |
| E18联合结果 | VALID_BUT_NEEDS_REINTERPRETATION | 同fine graph从56到79个anchor邻域一致；是后端时间选择，不是独立检测准确率；保留1miss和ablation反例 |
| fine/height lineage | POSSIBLY_CONFUNDED_BY_PREPROCESSING | 过滤定义了可见证据，provisional level与银标共享来源；性能不能归给raw detector或E18单独作用 |
| scene background | VALID_BUT_NEEDS_REINTERPRETATION | raw1m高重访符合静止场景；fine是另一过滤population；distance-to-scene未显稳定UAV分离，保留弱/反向结果 |
| temporal occupancy | STILL_VALID | 同频率条件下无五包稳定支持，严格kill与STOP不变；数值非冗余不等于有用，缺支持不等于零效果 |
| Stone Soup | STILL_VALID | 7官方test通过、合法空扫描posterior mismatch、完整实验STOP；外部软件结果不决定本UAV任务，继续暂停 |
| 高度、背景类别、独立presence与精度 | DATA_GAP | 用户几何背景不能补齐surveyed pose、全时段标注、负例、heldout或可靠Pd/clutter模型 |

还应保留[Temporal-Evidence MVP](FMCW_Temporal_Evidence_MVP.md)正式固定/adaptive/adaptive+Doppler门槛下仅7/7/8 of80的负结果；adaptive窗和Doppler没有稳定收益。诊断sweep中的较好点不再作为待优化起点。[Published Baseline/DP MVP](FMCW_Published_Baseline_and_DP_TBD_MVP.md)的58/80属于paper-structure reconstruction，不是作者实现精确复现或operational superiority；历史DP 0/80也保留。

Decomposition原表：`200m_cross_5/frame21` 总距26.575401m、along+26.389269m、cross−3.056353m、vertical-normal+0.719090m。其等效时间+6.707545s是局部chord线性反事实，**不是校准修正值**。同chord误差变号/变化仍兼容稀疏插值、相位/速度/对应误差，不能证明唯一常数time-offset。这里vertical-normal沿原报告定义，不能改称测得真垂直偏差。

Scene audit的raw1m冻结persistent集合回波占比约96.60–98.07%（F≥2；past H≥2且p_beta11≥0.5），任意prior-seen回波占比则为98.82–99.70%；二者不同。大部分raw点落在已见或高历史频率cell，不能等价为“98%点有静态GT标签”。过筛后fine novel比例与这个全raw分母不同。开放空域解释不授权重新探索distance特征或阈值。

[E18 mechanism audit](FMCW_E18_Mechanism_Generalization_Audit.md)的fine四arms为original56、background-only56、motion-only0、joint79/80；联合收益并非任一单项普适。`200m_cross_5`慢片段有原6/6→joint0/6的既有反例；不能用总79/80抹去。那个局部机制比较有其窗口定义，与完整包总一致率不同。

## 4. 56/80、79/80：组会应怎样说

五包分别32、46、83、48、63帧，总共 **272帧**。既有anchors分别19、11、22、18、10，总共 **80个选定observed DEV_SILVER anchors**。余下192帧没有这套anchor记录，presence/正确关联未知，不是192个negative。

| 包 | 帧数 | anchor帧 | 原score路径/anchor一致 | E18 joint一致 |
|---|---:|---:|---:|---:|
|100m_cross_5|32|19|19|19|
|100m_cross_10|46|11|0|11|
|200m_cross_5|83|22|20|21|
|200m_cross_10|48|18|17|18|
|300m_cross_10|63|10|0|10|
|合计|272|80|56|79|

数值来自[既有unconditioned_evaluation.csv](results/background_motion_dp_mvp/unconditioned_evaluation.csv)同fine family；“原”指相同fine graph上的original_score，不是历史1m graph（后者0/80）。逐包命中保留旧3m中心距离门。

```text
272帧曝光 ──其中80帧有已选observed anchor ──192帧未由这套anchor评价
                      │
           同一帧已有路径中心● 距 anchor○ ≤3m？
                      │
          原saved best path：56个“是” /80
          E18 joint path：  79个“是” /80
```

口头表述：“在已经挑出的80个可观察银标帧上，同一fine候选图原最优路径有56帧经过anchor的3米邻域，E18联合背景与运动项后有79帧经过。说明它更常选中这条已有目标相关证据链，**不等于独立检测召回率，不是precision，不是最终accuracy**。” 门内仍不证明每点UAV身份；1miss不等于真实唯一漏检。目标附近候选未标注/选定positives的结构不足以计算全部误警。

## 5. 最简单架构及物理先验的恰当位置

**这是一张解释架构，不是新方法或新阈值提案：** raw FMCW → 正确且标明basis的轴变换 → 有来源、逐步可核查的固定预处理 → 保留fine稀疏证据 → 静态/persistent证据抑制或降权 → 时间连贯轨迹。历史参数是否真有物理依据必须另核，不能先称“physically justified”再拿历史常量充证。

| 角色 | 判定 | 当前能说/不能说 |
|---|---|---|
| 正确轴与named fields | KNOWN USEFUL | 防止将横轴当高度、I/v换列；不是新标定 |
| 高度/监视体积 | PLAUSIBLE SOFT PRIOR；现数值 UNVALIDATED；任意硬切 UNSAFE AS HARD RULE | 静止开放空域给出意义；q_z非AGL，旧wrong-frame z>10曾删除1877/1877邻域support；不得重选门限 |
| intensity | KNOWN USEFUL（已执行压缩/筛选）；目标判别 UNVALIDATED | 既有1883邻域点保留1877是条件描述；绝对反射率/跨bag阈值有效性未知，不再优化 |
| 半米compactness | PLAUSIBLE SOFT PRIOR；count/extent UNSAFE AS HARD RULE | raw点局部集中可解释；采样/姿态/voxel改变extent和count，不保证硬bbox |
| 局部隔离 | PLAUSIBLE SOFT PRIOR；独立分离效用 UNVALIDATED | 用户观察有意义，但scene audit未建立稳定增量，孤立噪声仍在；不转成新E18列/score |
| Doppler/velocity | KNOWN USEFUL（辅助着色/运动解释）；普适拒绝 UNSAFE AS HARD RULE | 径向投影弱、符号证据有限；横飞近零不等于静态；现有DEV未验证独立收益，E18没用它 |
| 因果occupancy | KNOWN USEFUL（冻结DEV机制层）；新场景/存在判断 UNVALIDATED | 固定传感器适合查历史；p是cell占用proxy，不是clutter density/P(target)，不能硬删除慢/停留证据 |
| 时间motion coherence | KNOWN USEFUL（联合冻结图的选路机制）；单项普适性 UNVALIDATED | 连贯轨迹比单帧硬计数合适；单独motion-only失败，不能说平滑就是真UAV |

固定预处理不应隐藏目标support损失；降点/clean视觉也不证明提升检测。视觉与细粒度support核查先于下一轮复杂模型。

## 6. 是否过度复杂：明确决策

A. **必要的审计**：原始数据/字段/时钟血缘，GNSS/closure不确定性，raw与voxel区别，DBSCAN/geometry错误归因，DP累计竞争，E18联合/拆分失败，scene与conditional occupancy的否定结果。它们阻止把银标一致性当检测精度，有可保留科学价值。

B. **可能过早的部分**：历史清洁基线未闭环前，把全场景复杂分类、精细background特征挖掘、存在概率和外部tracker移植变为主线。Stone Soup作为小软件语义实验有价值，但其继续/接入早于本项目观测模型与baseline；不能因为已花时间而继续。

C. **可以推迟/暂停**：Bernoulli存在性（不修补/换版本）、DopplerTrack/UnLiDAR等额外repo复现、多目标复杂滤波/学习方法、E21/E22、DEV上的新阈值/背景特征。外部路线图保留为历史计划，当前下一步优先级被此次用户任务覆盖，不继续安装/执行。

D. **最低可辩护研究问题**：“在固定传感器、开放空域的小UAV序列中，坐标/点过滤/表示如何影响可观察稀疏回波保留，以及冻结候选图上的时间一致性为何有助于关联这些证据？” 当前可形成可复核的工程/机制案例研究；发表资格/新颖性UNKNOWN，不能靠五包同场景选定positives支撑通用检测性能。新的完整检测论断仍需独立数据/标注和heldout，不重新开放DEV挖掘。

**E18应放回后端候选轨迹选择组件的位置**，而不是让整条采集/清理/保留链都围绕其评分解释。E18继续冻结；这不否定它在相同图上选路的已测价值，也不授权更改架构实现。

## 7. 唯一下一任务的条件与交付边界

优先恢复并冻结师兄原轴变换＋强度/空间过滤版本与数据/显示血缘，再做逐阶段点量、已有anchor邻域support retention和正确坐标下的可视化核查。**完整原链目前BLOCKED**：缺exporter/具体空间版本/filtered RViz发布配置；先填证据缺口，不能先在三包上试不同版本挑最像UAV的一个。现有batch子阶段可恢复，与完整复现状态分开。

本轮仅产出[受控计划](results/project_reinterpretation_20261016/proposed_historical_reproduction_plan.md)。100m_cross_5/200m_cross_5/300m_cross_10只预先指定为范围代表，不执行；anchors只能在流程冻结并计算完stage统计之后作描述保留检查。不要把一次原流程重放变成新的算法优化。

## 最终决策

1. "What is the simplest correct description of the current problem?"

静止FMCW LiDAR在开放监视空域中，经过有来源且坐标正确的固定预处理，保留稀疏UAV相关回波，并从残余干扰中关联出时间连贯轨迹。现阶段是机制/工程诊断，而非已验证的通用存在性检测。

2. "What did we previously make more complicated than necessary?"

在历史清洁链与表示影响尚未端到端确认前，过早以完整raw复杂场景、精细背景与存在性模型统领问题；同时高层口径淡化了既有报告早已说明的GNSS粗参考、过滤输入与银标条件性。必要的负结果审计保留，额外模型探索暂停。

3. "If we are allowed to do exactly one next technical task, what should it be?"

恢复并锁定原coordinate-transform＋intensity/spatial过滤的实际版本/导出/发布血缘，随后只做忠实的逐阶段数值与正确轴视觉验证。缺原版环节则保持BLOCKED，不猜替代参数、不设计另一个算法。
