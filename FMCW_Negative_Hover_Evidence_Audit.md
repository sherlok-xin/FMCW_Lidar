# FMCW Negative / Hover Evidence Audit

日期：2026-10-11。E20，仅审查已有资料。结果目录：[negative_hover_evidence_audit](results/negative_hover_evidence_audit/README.md)。

**VERIFIED RESULT：资料库存、来源、时钟与旧曲线已核查。独立 negative、hover/slow 认证及其 E18 泛化结果：`DATA_GAP / UNKNOWN`。** 本轮未运行新的 E18 stress test；未修改公式、权重、graph、labels、3 m gate、历史 baseline，未加 Doppler、选 threshold 或修 miss。

## A / B / C：直接结论

**A. 尚未找到可用于评价的可靠 negative window。** 全部现存 bag/ZIP 的成员与 topic 元数据、完整 Excel、照片 EXIF、来源日志和项目路径级 Git 历史均没有提供独立、带时间且能连接到 LiDAR 的“不在 FOV / 未起飞 / 已降落”记录。缺标签、无回波、failed closure、静态赢家和 background initialization 不构成 negative。

**B. 有 short slow-like LiDAR 观测和较慢 GNSS 采样段，但没有通过独立认证的真正 hover/slow UAV window。** 三条连续 silver 片段有运动→减速→近静止形状，其中两条还有再加速；物理身份/状态并非独立记录。GNSS 最慢段约0.996 m/s，是两端弦长速度而非连续低速或悬停证明。照片早于所有 February bags，不能认证 bag 内状态。

**C. 不能确认 E18 在真正 moving→slow/hover 过程中发生系统性退化。** 旧 E19 的200m_cross_5帧7–12固定 silver witness，background novelty 从0.777778降至0.142857，增广 node score 从1.717021降至1.108642，均仍正；离开 cell 后 novelty 回升。这是既有回溯性机制证据，不是本轮新 stress 或独立 hover 泛化结果。真实 UAV hover、长期影响、跨片段复现均 **UNKNOWN**。

资格决策：[stress_execution_decision.json](results/negative_hover_evidence_audit/stress_execution_decision.json)。合格 negative / slow-hover 窗口均0，**表示没有窗口通过认证，不是证明不存在 UAV/hover 或零误警**。新 stress 状态：`NOT_RUN_DATA_GAP`。

## 1. GitHub 同步与冻结

用户要求先上传的 E19 报告、结果、复现材料已在 [`5ad38b9`](https://github.com/sherlok-xin/FMCW_Lidar/commit/5ad38b94b87949657f803485da129e3a738e9a64)。本轮再用 `git ls-remote --heads origin main` 核验：远端 main 为 `5ad38b94b87949657f803485da129e3a738e9a64`，与本地 HEAD 相同。没有重复提交或 force push。本轮 E20 新文件在本地，未混入该历史提交。

E18 冻结版本仍是 `a53702da1dc5357149e3e472a9e489ff08e9afe4`。当前 core/wrapper 与该 Git blob SHA 一致；另保护302个旧文件及完整 E18/E19 结果树文件集合，包括旧协议、graph、labels、cache、源文件。全部不变，见 [freeze_identity.json](results/negative_hover_evidence_audit/freeze_identity.json)、[protected_input_validation.json](results/negative_hover_evidence_audit/protected_input_validation.json)。无关 `.local/` 保留、不读取内容或上传。

新增代码只做只读库存、来源导出和 hash 核对，不导入/执行 E18 solver、不生成 candidates/graph、不更新 calibration。[protocol.json](results/negative_hover_evidence_audit/protocol.json)固定资格规则，早于汇总但晚于首次资料探查；不声称是盲审或预注册运动实验。

## 2. 证据等级与资格门

| 等级 | 定义 | 资格边界 |
|---|---|---|
| L3 | 独立 target identity/state、明确时间窗、LiDAR 时钟/可见性链接 | 可作冻结 stress；正式评价另需独立标注、冻结 held-out 协议；本轮无L3 |
| L2 | 独立 GNSS/照片原始上下文，事件/身份/采样/时间/FOV 链不完整 | 不够认证；独立测量不自动等于 hover/negative GT |
| L1 | LiDAR-derived DEV_SILVER、manual ROI pseudo、未认证候选 | 仅回溯性描述，不是独立物理状态 |
| L0 | 文件名/topic/无旁证推断 | 不够 |

Negative 要求独立 absence 证据、明确区间、bag 时钟链接和可用 LiDAR；FOV 外还需可信 pose/FOV。Slow/hover 要求物理 UAV 身份与状态、足够采样/定位误差证据及 LiDAR 可见性。没有新速度 cutoff，不按 E18 分数挑证据。未起飞/已降落不能由低高度或设备原点自行推断。

[candidate_windows.csv](results/negative_hover_evidence_audit/candidate_windows.csv)逐条保留来源、定位、等级、起止与时间基准、LiDAR/冻结bag graph可用性、资格失败和formal evaluation状态：80条重叠库存记录，L0/L2/L1=57/11/12。**并非80独立窗口或80个实验，恰好与silver数量相同但没有对应关系。** 全部 stress/formal qualification=False；物理presence/absence/hover未认证项为UNKNOWN。

## 3. 全数据覆盖与新增发现

| 资料 | 本轮实际覆盖 | 结论/限制 |
|---|---|---|
| bag/data | 697文件：6bags、14ZIP、1xlsx、1jpg、674PCD、fishros；含Git忽略的数据 | 无项目视频、飞控、RTK sidecar；没有记录不等于没有目标 |
| 14ZIP | 全52个非目录成员、所有archive/member comments | 51bags + 唯一49-byte `record_bag.sh`；comments全空，无GNSS/视频/状态成员 |
| 51归档bags | 每包最多1MiB非压缩前缀，总53,477,376bytes；header/首chunk connections | 每包声明1并观察到唯一cloud0 PointCloud2；覆盖全部声明connection，未读footer/完整stream/尾时间 |
| 6standalone | 完整final connection/chunk-info index，首消息header小样本 | 3Feb仅cloud0；2Nov各cloud81/82；March仅cloud81；无flight/GNSS/camera topic；未解码点云 |
| 8PCD目录 | 674文件元数据，3raw/5filtered | 不新增点级身份或absence推断 |
| 来源/日志 | 19项带行号/SHA，原README、回收原始用户请求、旧审计和项目笔记 | hovering示例分类、tracker静止删除均不是采集事件 |
| Git历史 | 3个project-scoped路径级查询，含媒体、flight/GNSS、删除资料、bag/data | 无可恢复状态记录；raw资料本就未入Git，不能从缺历史推断缺UAV |

数据证据：[raw_bag_inventory.csv](results/negative_hover_evidence_audit/raw_bag_inventory.csv)、[archive_member_inventory.csv](results/negative_hover_evidence_audit/archive_member_inventory.csv)、[data_file_inventory.csv](results/negative_hover_evidence_audit/data_file_inventory.csv)、[log_source_inventory.csv](results/negative_hover_evidence_audit/log_source_inventory.csv)、[log_history_coverage.json](results/negative_hover_evidence_audit/log_history_coverage.json)。三个Feb standalone是extracted-copy路径，不增加独立序列数；未全文件hash验证副本逐字一致。

Topics仅LiDAR不代表bag无UAV，只说明没有额外flight-state记录。`scene_a`、`rotate_90`、`problem`不能认证negative/hover。采集script只录`/aqronos_cloud_0`，没有状态事件。E10已推翻旧100m“无return”解释，不能将其复用为negative。

**新时钟警示：** March文件名`2026-03-19-14-11-42`，实际bag record UTC为 **2026-03-20 04:18:43.558423–04:22:10.758542**，57messages/207.200119s，首ROS header仍2022 epoch。文件名、record、header不等价；重录/来源关系的确切原因UNKNOWN，不按filename拼接flight记录。现有Feb offset仍有别名/边界解，本轮没有重新优化offset或外参。

## 4. GNSS与照片：有独立记录，没有状态闭合

完整读取workbook/sheets/sharedStrings/docProps等ZIP/XML。openpyxl在venv和系统Python均缺失，采用标准库只读fallback，不安装包、不保存workbook。唯一visible Sheet1、格式范围A1:O85；实际82个非空cells全在A列：71 flight-labelled GNSS rows、1device、10标题。无hidden sheets、comments、external links或armed/landed/hover/events。

沿用已有GPS week/TOW、Lat/Lon/Ellh、GPS−UTC18s与WGS84→device-origin ENU解析约定；N/E/V/Q及未命名字段保留opaque，含义UNKNOWN，不把V当物理速度。设备A85与300m_roundtrip_10首点A2逐字相同；零ENU距离不证明未起飞、已降落或不在FOV。

| 条件 | cells / 样本数 | GNSS UTC区间，2026-02-05 | endpoint chord speed范围m/s |
|---|---|---|---:|
| 300m_roundtrip_10 | A2–A29 /28 | 06:39:54.048–06:43:35.167 | 0.996–4.420 |
| 300m_roundtrip_13 | A31–A42 /12 | 06:50:56.186–06:53:16.667 | 4.717–7.147 |
| 300m_roundtrip_5 | A44–A50 /7 | 07:03:27.976–07:04:59.458 | 5.775–7.815 |
| 100m_cross_10 | A52–A55 /4 | 07:10:17.145–07:10:47.898 | 2.605–4.656 |
| 100m_cross_5 | A57–A60 /4 | 07:14:02.518–07:14:29.528 | 1.736–3.654 |
| 200m_cross_10 | A62–A65 /4 | 07:18:26.425–07:19:04.894 | 3.482–5.978 |
| 200m_cross_5 | A67–A70 /4 | 07:31:43.548–07:32:20.643 | 3.934–5.770 |
| 300m_cross_10 | A72–A75 /4 | 07:37:25.064–07:38:12.273 | 5.206–7.567 |
| 300m_cross_5 | A77–A80 /4 | 07:42:03.485–07:42:48.232 | 4.876–7.267 |

全部62段保留，无cutoff选样。最慢A2→A3，week2404、TOW369612.048255→369617.183778，UTC06:39:54.048255→06:39:59.183778：5.113035574m/5.135523s＝0.995621200m/s。**只是端点位移/间隔，不是瞬时速度、平均实际路程速度或hover证明。** 往返约5–18s采样、横飞每包仅4点，无法解析短暂停留；不能证明hover，也不能证明采样间隔内没有hover。缺状态/定位误差认证、同步与真实FOV。

完整来源：[gnss_samples.csv](results/negative_hover_evidence_audit/gnss_samples.csv)、[gnss_adjacent_segments.csv](results/negative_hover_evidence_audit/gnss_adjacent_segments.csv)、[gnss_condition_summary.csv](results/negative_hover_evidence_audit/gnss_condition_summary.csv)、[gnss_workbook_inventory.json](results/negative_hover_evidence_audit/gnss_workbook_inventory.json)。这里是GPS UTC，不直接等于silver bag UTC，不套统一108s。

场景图`bag/20260205/场景图.jpg`中quadrotor与LiDAR同框、似为地面搭建状态，不认证飞行或持续静止。nested EXIF DateTimeOriginal=2026:02:05 14:16:50、Subsec957、offset+08:00；另一GPS EXIF clock=06:16:43 UTC、CELLID、无经纬度，两clock不一致。按DateTimeOriginal解释，早于最早Feb bag约 **21m19.386s**，无同时LiDAR数据。它支持拍摄瞬间有无人机/设备，不支持任一bag的target absence、起飞/降落事件或hover区间；照片不是视频，不外推之后21分钟。

## 5. moving→slowing→near-static形状：保留，不认证

只读导出全部80silver observations的75相邻step（5跨缺帧gap逐条标记），以及全部10个maximal frame-contiguous fragments；没有新增slow cutoff/标签。以下是较完整的线索，不是新stress子集或hover GT。

| silver连续片段 | bag UTC区间，2026-02-05 | 可复核中心变化m/s | 资格 |
|---|---|---|---|
| 200m_cross_5 /0–21 | 07:29:57.326–07:30:18.331 | steps4→5/5→6/6→7：11.016→9.415→3.945；7–12：0.070–0.134；随后2.329→4.632并加速 | L1；有graph，无独立状态，formal NO |
| 200m_cross_10 /30–47 | 07:17:11.336–07:17:28.319 | 13.652→9.759→6.202→2.281；34–37：0.080–0.116；再运动，44–47又减速 | L1；同上 |
| 100m_cross_10 /18–26 | 07:08:47.319–07:08:55.324 | 6.513→3.893→1.172→0.526/0.079/0.504，随后4.063/7.498 | L1；同上 |

来源：[silver_motion_segments.csv](results/negative_hover_evidence_audit/silver_motion_segments.csv)、[silver_continuous_fragments.csv](results/negative_hover_evidence_audit/silver_continuous_fragments.csv)。这是LiDAR return-center变化，不是独立物理UAV速度；返回点变化、选簇/轨迹、centroid jitter等影响的实际程度UNKNOWN。

旧E19五窗全部保留为L1，不重新跑或扩标签。日期2026-02-05、time为bag UTC：

| 条件/帧 | UTC起止 | 旧silver median speed m/s | LiDAR/原graph | 独立认证/正式评价 |
|---|---|---:|---|---|
| 100m_cross_5 /23–26 | 07:12:44.318825–07:12:47.330655 | 0.205 | YES | UNKNOWN / NO |
| 100m_cross_10 /21–24 | 07:08:50.345695–07:08:53.322669 | 0.504 | YES | UNKNOWN / NO |
| 200m_cross_5 /7–12 | 07:30:04.316519–07:30:09.323702 | 0.083 | YES | UNKNOWN / NO |
| 200m_cross_10 /34–37 | 07:17:15.316531–07:17:18.325653 | 0.102 | YES | UNKNOWN / NO |
| 200m_cross_10 /44–47 | 07:17:25.326555–07:17:28.319304 | 0.466 | YES | UNKNOWN / NO |

November旧cross pseudo帧0–3近静止（3.006s、end displacement0.184m），但manual ROI/largest-cluster身份不独立，且无E18 frozen graph。三个未选closure candidates的median speed低也不认证hover；全部保留拒绝理由，不换成silver。300m帧9→10只有两观测，不能认证持续hover，也不能称300m完全没有slow-like证据。

## 6. 分数：旧曲线不是新 stress

[prior_e19_slow_score_trace.csv](results/negative_hover_evidence_audit/prior_e19_slow_score_trace.csv)复制旧E19固定witness全部168nodes，明确标`PREVIOUS_E19_STORED_DIAGNOSTIC_NOT_NEW_STRESS_TEST`。160行对应两family的80silver observations，8行是既有连接补帧、不是新增label。旧五slow窗两family共44行，是同22observations的两种表征，不能当44独立样本。

Fine200m_cross_5帧7–12同cell `[199,31,-5]`，prior(F,H)=(7,0)→(12,5)，background依次 **0.777778、0.600000、0.454545、0.333333、0.230769、0.142857**；增广node1.717021→1.108642。帧13离开cell，background回升0.866667。旧E19短窗witness score8.228914、winner9.360877（winner0/6silver），但full-bag E18仍命中六帧。不能把短窗重root结果与full-bag输出混为同一曲线，或把node novelty下降等同累计path score下降/物理hover失检。

已有公式提供repeated-cell novelty机制风险，不是新simulation或真实hover因果验证。本轮独立moving→hover target-neighborhood score曲线 **UNKNOWN**。

[negative_stress_winners.csv](results/negative_hover_evidence_audit/negative_stress_winners.csv)、[qualified_hover_score_trace.csv](results/negative_hover_evidence_audit/qualified_hover_score_trace.csv)仅表头，保留请求字段，不填假数。不是0score/0duration/0false alarm或“测试通过”。

## 7. 可复现执行与验证

既有Python3.10.12、NumPy2.2.6/Pillow12.2.0；openpyxl缺失。无安装、随机过程、参数搜索、全cloud解码、E18求解或外部新数据获取。实际命令与源/输入/输出SHA见各execution JSON：

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_evidence_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_bag_inventory.py --output-dir results/negative_hover_evidence_audit --prefix-bytes 1048576
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_gnss_inventory.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_log_inventory.py --out results/negative_hover_evidence_audit
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_window_inventory.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_evidence_audit.py check
```

库存并行执行；freeze固定旧输入与资格规则，不声称所有metadata检查均在其后。脚本拒绝覆盖输出，完整复现需新工作副本/空输出目录，不删除当前实验，详见结果README。

验证：302protected SHA/E18与E19树集合不变；697raw输入size/mtime/mode前后相同（不是全bag内容SHA）；workbook/photoSHA不变；51prefix字节限额/declared-connection覆盖；全部新脚本AST；62GNSS段/75silver步distance/dt、来源/数量；输出hash与报告链接/计数。只是metadata验证，不是detector accuracy测试。

保留两次inventory-only失败：新freeze helper起初引用不存在的旧inventory script，在写freeze输出前停止，核实后只改新helper；log导出初次假定160witness nodes，实际168含8非label，写输出前停止，仅修新计数/身份标记。旧来源和score均未变。见[preflight_failure.json](results/negative_hover_evidence_audit/preflight_failure.json)、[log_evidence_execution.json](results/negative_hover_evidence_audit/log_evidence_execution.json)。

## 8. 解释边界与停止

Material Passport：Origin Skill=`experiment-agent`（academic-research-suite）、Mode=`validate`、Date=`2026-10-11`、Verification Status=`ANALYZED`、Version=`e20_evidence_audit_v1`。源/hash库存是VERIFIED RESULT；物理状态/E18泛化没有认证。技能的证据审查影响等级与资格门，不是方法验证证书。

11/11 fallacy checked：Simpson/ecological（逐来源/片段，不由汇总推个体）；Berkson/collider/survivorship（observed silver是选择条件）；base-rate（未知absence/prevalence，不报precision/FA）；regression-to-mean（旧失败不当新复现）；look-elsewhere/forking-paths（全部库存/采样段，无cutoff或score选样）；correlation≠causation（慢中心/重复占cell不证明物理hover因果）；reverse-causality（同LiDAR-derived标签不作独立认证）。没有p值、独立样本CI或泛化显著性结论。

现有资料足够做**DEV机制研究**，不够下一阶段独立negative/hover评价。缺带时间的flight-controller/RTK、同步视频或operator events；可认证takeoff/landing/FOV-away区间；持续moving→slow/hover身份、可见性、定位误差；独立clock/pose/FOV依据。仓库外是否存在这些记录 **UNKNOWN**，本轮不获取/采集、不伪造标注、不新建graph，也不授权继续算法。

**停止点：已有证据审计完成；保持E18完全冻结，下一阶段证据门未通过：`DATA_GAP`。**
