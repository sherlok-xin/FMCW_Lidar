# FMCW E18 Mechanism and Generalization Audit

日期：2026-10-11。冻结对象：`a53702da1dc5357149e3e472a9e489ff08e9afe4` 的 E18。

**IMPLEMENTED / TESTED / VERIFIED RESULT，仅限冻结 graph 和 DEV_SILVER；独立 negative/hover 泛化 UNKNOWN。** 结果目录：[results/e18_mechanism_generalization_audit](results/e18_mechanism_generalization_audit/README.md)。未修改 E18 公式、权重、graph、labels、3 m gate 或任何历史 detector/tracker；未加 Doppler、feature、threshold，未修剩余 miss。

## 结论：A / B / C

**A. 不能把 E18 的最终 recall 收益归给单一 term。** 历史 persistent-clutter 固定路径的降分主要来自 background；但无条件 winner 的恢复依赖 background 与 motion 的交互。Fine 四组命中是 **56 / 56 / 0 / 79（分母80）**，历史1 m控制是 **0 / 56 / 0 / 79**。Background-only 无法恢复100m_cross_10和300m_cross_10；motion-only在两套 graph 均0/80，而且会把 fine 原本命中的路径换成近静态路径。加入 background 后，motion 才产生净+23个命中；逐帧为 **新增24、丢失1**，不是只增不减。

**B. 无 UAV 场景是否仍有高分伪轨迹：UNKNOWN。** 已核查14个ZIP的51个bag members、6个standalone bag路径及既有标注/采集元数据；没有可靠 target-absent 认证。Negative 分支 **NOT_RUN_NO_RELIABLE_TARGET_ABSENCE**，不是“零误警”。未标帧、零return、failed closure和看似静态的路径均不作为negative；没有选择运行阈值。

**C. Background novelty 存在可解释的 hover/slow 风险，但不能声称已证明长期 hover 的系统性失败。** 单列的5个短 slow-cross 窗口、22个silver observations，在每套graph上都是 original14/22、background-only12/22、motion-only14/22、E1812/22。尤其200m_cross_5/frame7–12由 original/motion6/6变为 background/E180/6。该窗 witness 的background总项仍为正+2.539，而更novel的竞争路径获得更高奖励：这是**相对排序损失，不是hard deletion，也不是该窗出现负background分**。独立hover bag、真正UAV悬停状态和长期hover recall仍UNKNOWN。

本轮到此结束：保留全部四组、短slow负结果和UNKNOWN边界；不修miss、不叠加feature、不改权重，不升级Benchmark v1。

## 1. 冻结、输入与公平性

先读项目状态、E18报告、代码、协议和结果；实际Git HEAD与指定commit一致。新增的是独立审计adapter，不覆盖E18实现。协议于 **2026-10-11 10:56:16.657898 UTC** 冻结，早于四组真实评分；细节见[protocol.json](results/e18_mechanism_generalization_audit/protocol.json)、[freeze_identity.json](results/e18_mechanism_generalization_audit/freeze_identity.json)。

| family | 本轮只读的 E18 snapshot | 固定标签/评价 |
|---|---|---|
| `fine_4cm_pregeometry`，primary | E16已保存adaptive-minPts1全部pregeometry components；20,122nodes / 479,081edges | 五包272帧；80observed DEV_SILVER，原3 m gate |
| `historical_1m_graph`，control | 原E13候选支路；16,995nodes / 371,600edges | 同五包与同80observations，单独比较 |

每个arm直接读相同family的nodes/edges CSV，沿用既有occupancy annotations；**没有重新生成候选、graph或background，也没有恢复更早ROI/intensity/height已删的原始点。** 四组共用未修改的eligibility-aware E18 solver、gates、顺序和tie处理。Fine与historical是不同representation；不能把0→56的跨family差异当作score效果。

Silver SHA-256：`37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db`。标签是回溯性LiDAR-derived silver centers，不是独立GT/点级UAV annotation。五包19/11/22/18/10个observations，未插值扩充。其余帧target身份UNKNOWN。

新增审计脚本冻结SHA：`ba7334de48dfae21f0e5bc221b12f306ddafd18db9165f6886da8f193d70c8d4`；新增tests SHA：`919c60e12c31b4af76f7ef6a372467950a295cf7eb59c145ffcbbbd425e8e72a`。本轮是该parent上的新增未提交工作，用SHA标识，不虚构新commit。

## 2. 仅关闭已有 term 的四组

令原node总项为S、既有background总项为B、原gap penalty为G、既有speed+direction penalty为M：

| arm | path score | 改动方式 |
|---|---|---|
| original | S−G | 原score字段、原score模式 |
| background-only | S+B−G | 只把已有augmented node score交给原score模式 |
| motion-only | S−G−M | motion模式的node字段路由为已有原score |
| background+motion E18 | S+B−G−M | 未改E18原模式 |

这只是term开关，不是新公式拟合；所有启用term保留E18权重1。

继承原node：`s=0.50+0.30 exp(−extent/2 m)+0.20 min(log(1+raw_support)/log6,1)`。

继承background：sensor-relative `[x,z,−y]`，1 m floor零原点cell；当前帧之前所有fine有效cache点的逐帧cell occupancy。`p=(H+1)/(F+2)`，`b=1−2p`；查询后才加入当前帧。p是occupancy proxy，不是校准的clutter probability。

继承motion：真实edge Δt，`v=(x_j−x_i)/Δt`，相邻edge的`a_dt=(Δt_prev+Δt)/2`。

- `speed_penalty=0.5[(||v||−||v_prev||)/(15 a_dt)]²`。
- `direction_penalty=0.5[||v||||v_prev||−v·v_prev]/[||v||||v_prev||+(1 m/a_dt)²]`。

原numeric gates保持：frame gap≤3、Δt≤3.5s、speed≤25m/s、每prior frame最多12nearest predecessors、acceleration≤15m/s²、每跳过一帧罚0.25、至少3nodes且duration≥2s。没有新增最小速度gate、M-of-K、detection threshold或Doppler。首两个node无previous edge，motion term为0。

40次完整bag无条件求解结束并保存结果后，才解析labels评价。E18的fixed silver witnesses和旧参考路径全部复用，没有在新arm下重新挑更优silver oracle。原模式与完整E18的 **20次路径及全部qualified terminal scores精确重放**，见[e18_exact_replay.csv](results/e18_mechanism_generalization_audit/e18_exact_replay.csv)。

## 3. 无条件输出：逐包与距离分层

来源：[unconditioned_evaluation.csv](results/e18_mechanism_generalization_audit/unconditioned_evaluation.csv)、[silver_frame_evaluation.csv](results/e18_mechanism_generalization_audit/silver_frame_evaluation.csv)。每包只有一条无条件最高分合格路径；无score threshold。以下分母仅为已观察silver帧，不是全部bag帧。

| family | 序列 | original | background-only | motion-only | E18 |
|---|---|---:|---:|---:|---:|
| fine | 100m_cross_5 | 19/19 | 19/19 | 0/19 | 19/19 |
| fine | 100m_cross_10 | 0/11 | 0/11 | 0/11 | 11/11 |
| fine | 200m_cross_5 | 20/22 | 20/22 | 0/22 | 21/22 |
| fine | 200m_cross_10 | 17/18 | 17/18 | 0/18 | 18/18 |
| fine | 300m_cross_10 | 0/10 | 0/10 | 0/10 | 10/10 |
| historical | 100m_cross_5 | 0/19 | 19/19 | 0/19 | 19/19 |
| historical | 100m_cross_10 | 0/11 | 0/11 | 0/11 | 11/11 |
| historical | 200m_cross_5 | 0/22 | 20/22 | 0/22 | 21/22 |
| historical | 200m_cross_10 | 0/18 | 17/18 | 0/18 | 18/18 |
| historical | 300m_cross_10 | 0/10 | 0/10 | 0/10 | 10/10 |

| family / nominal range | original | background-only | motion-only | E18 |
|---|---:|---:|---:|---:|
| fine / 100m | 19/30 | 19/30 | 0/30 | 30/30 |
| fine / 200m | 37/40 | 37/40 | 0/40 | 39/40 |
| fine / 300m | 0/10 | 0/10 | 0/10 | 10/10 |
| historical / 100m | 0/30 | 19/30 | 0/30 | 30/30 |
| historical / 200m | 0/40 | 37/40 | 0/40 | 39/40 |
| historical / 300m | 0/10 | 0/10 | 0/10 | 10/10 |
| fine / 全部 | 56/80（70%） | 56/80（70%） | 0/80 | 79/80（98.75%） |
| historical / 全部 | 0/80 | 56/80（70%） | 0/80 | 79/80（98.75%） |

Fine background-only和original命中的是完全相同的56帧，而非换了一套恰好同数量的命中。完整E18相对background-only两family都新增24、丢失1，净+23；丢失的是200m_cross_5/frame21，原3 m门下距silver分别3.253693/3.253690m。该miss与E18历史结果相同；不修、不扩大gate、不移动标签。

候选量在四arm间完全相同；输出节点数fine为270/270/271/260，historical为272/271/272/259。完整volume与runtime见[summary_by_range.csv](results/e18_mechanism_generalization_audit/summary_by_range.csv)和`unconditioned_summary.csv`。候选量/输出量不是precision；未标输出的身份不明，不能数为false alarm。

配对增量见[term_contrasts.csv](results/e18_mechanism_generalization_audit/term_contrasts.csv)：fine的background增量（无motion）0、motion增量（无background）−56、motion增量（有background）+23；historical分别+56、0、+23。**Winner选择非线性，不能把这些不同context的增量换算成“各贡献百分之多少”的唯一归因。** 固定路径分数可加，winner recall不可加。

## 4. 机制：background压旧静态参考，motion限制novel但不连贯的竞争路径

### 4.1 固定 historical persistent-clutter 路径

下表始终是同一条E18保存的历史路径，不是各arm的新winner。原full参考更长，不能将它与短silver witness的累计分直接当同duration比较。

| 序列 | original score | background-only | motion-only | E18 | raw B | raw M |
|---|---:|---:|---:|---:|---:|---:|
| 100m_cross_5 | 31.158 | 15.090 | 28.448 | 12.381 | −16.068 | 2.709 |
| 100m_cross_10 | 44.745 | 6.976 | 44.163 | 6.394 | −37.769 | 0.582 |
| 200m_cross_5 | 81.084 | 6.112 | 80.582 | 5.610 | −74.972 | 0.502 |
| 200m_cross_10 | 46.979 | 5.937 | 46.597 | 5.555 | −41.042 | 0.382 |
| 300m_cross_10 | 61.480 | 6.081 | 60.935 | 5.536 | −55.399 | 0.545 |

这些固定参考的E18降分由background主导；motion penalty通常很小。静止/匀速直线在理想情况下motion penalty均为0，因此motion不是“必须移动才能是UAV”的identity测试。Fine旧winner本就有56个silver命中，不能统称persistent static clutter。

### 4.2 为什么background-only仍失败，motion-only反而失败

Fine的100m_cross_10和300m_cross_10 background-only赢家分别得分74.921/102.426、duration44.984/62.009s、46/63nodes，均0silver命中。两条路径background奖励+35.069/+48.010；对其重算**未启用**的既有motion penalty为24.049/32.149。它们满足原speed/acceleration硬约束，但许多soft不一致可在长路径中累积。加入motion后选中另一条路径，不能把重算分直接称新arm winner。

反过来，fine motion-only五条赢家median speed仅0.062–0.130m/s，近静态路径很容易获得低motion penalty；historical4/5 motion-only赢家与original完全同一条路径。它们在silver邻域0命中，**并不证明整个bag无UAV或这些bag是negative**。这些controlled term ablations支持两种作用相互补充，但仅限当前候选/graph。

### 4.3 固定silver witness的matched-span rank

来源：[fixed_path_ranks.csv](results/e18_mechanism_generalization_audit/fixed_path_ranks.csv)、[score_margins.csv](results/e18_mechanism_generalization_audit/score_margins.csv)。排序是每个合格terminal edge保留的最佳history representatives；`≥`为strict-exceedance count+1下界，仅reference自身被保留时才写池内exact competition rank。**全部路径的combinatorial rank均UNKNOWN**。

| family | 序列 | original | background-only | motion-only | E18 |
|---|---|---:|---:|---:|---:|
| fine | 100m_cross_5 | 1 | 1 | 5 | 1 |
| fine | 100m_cross_10 | 296 | 15 | 182 | 1 |
| fine | 200m_cross_5 | 3 | 2 | 1 | 1 |
| fine | 200m_cross_10 | ≥5 | ≥2 | ≥7 | ≥2 |
| fine | 300m_cross_10 | 291 | 49 | ≥374 | ≥8 |
| historical | 100m_cross_5 | 4 | 1 | 11 | 1 |
| historical | 100m_cross_10 | 259 | 16 | 167 | 1 |
| historical | 200m_cross_5 | 9 | 2 | 3 | 1 |
| historical | 200m_cross_10 | ≥21 | ≥2 | ≥15 | ≥2 |
| historical | 300m_cross_10 | ≥466 | 30 | ≥300 | ≥4 |

不只报告成功：fine100m_cross_5 combined仍rank1，无增益；fine200m_cross_10相对旧同窗参考margin仍−0.033916；300m witness仍不是rank1。Fine200m_cross_5同窗margin由−0.065337→−0.065337（background-only不变）→+0.569165（motion-only/E18），但full-bag motion-only仍0/22，说明固定短witness、同窗竞争和无条件full输出不能混为一项metric。

固定路径score的精确可加性30组全部通过；分解见[fixed_component_differences.csv](results/e18_mechanism_generalization_audit/fixed_component_differences.csv)，原120条完整分解保留。Rank和winner evidence共同表明：**压旧static不是最终恢复UAV的充分条件**。

## 5. Target-absent inventory：没有可执行的可靠negative

库存32条记录及完整bag成员见[generalization_inventory.csv](results/e18_mechanism_generalization_audit/generalization_inventory.csv)和[generalization_inventory.json](results/e18_mechanism_generalization_audit/generalization_inventory.json)。

- 9个2026-02-05 ZIP + 5个较早DronesBag ZIP；共51个bag members（9+42）。ZIP/archive comments均为空。
- 6个standalone路径中3个为Feb extracted-copy路径，不计为额外独立序列；本次没有对这三副本重新做全文件字节一致性hash。
- 3个raw PCD目录 + 5个filtered PCD目录，以及README、项目日志、采集script、manifest、历史negative audits与closure status。
- `scene_a`、`rotate_90`、`problem`只是目录名；唯一archive采集script只表明录制`/aqronos_cloud_0`，不认证是否有UAV。

本次轻量库存读取归档成员/metadata及既有证据，**没有逐一解码所有raw bags、检查完整视频或获取外部飞行记录**。现有资料没有timestamp-linked在场/不在场annotation。未标帧、背景初始化帧、未选candidate、空filter输出、failed closure、此前已被E10推翻的100m“无return”解释都不构成absence证据。

[reliable_negative_winners.csv](results/e18_mechanism_generalization_audit/reliable_negative_winners.csv)保留用户所需score/duration/background/motion/path性质schema，但仅表头。以下为明确状态：

| 项目 | 状态 |
|---|---|
| 可靠target-absent bags/windows | UNKNOWN；未认证任何一个 |
| negative上的winner score/duration/组成 | NOT_RUN_NO_RELIABLE_TARGET_ABSENCE |
| negative中的高分伪轨迹/false alarms/operational precision | UNKNOWN，不填0 |
| 新score threshold / operating point | NONE_SELECTED |

E18目前是无条件argmax，图中有合格路径便返回winner，并非“有UAV才输出”的presence测试。这个代码性质提示必须有negative验证，但**不能代替无UAV数据上的经验score或误警统计**。需要已有或新采集的明确absence记录、同步video/flight-state和可定位bag时间窗才能回答B；本轮不启动额外采集或阈值选择。

## 6. Hover / very-slow stress，独立于cross汇总

**独立认证的专门hover/very-slow UAV序列：UNKNOWN。** 旧cross pseudo trajectory前三秒近静止，但来自manual/trajectory-guided ROI，且无E18冻结graph；其他closure候选低speed不等于hover UAV身份。没有新建其graph或改labels来扩充数据。

已有cross银标确有短slow observed-center片段，因此不能说“完全没有慢速证据”。本轮预冻结选择**所有** maximal连续observed runs，每个相邻observed-center真实速度≤继承E18 static诊断尺度`1 m / median full-bag Δt`，且≥3observations、duration≥2s。实际得5窗，22observations，每family一份；不是手挑表现好的4窗。Selector只定义事后stress subset，不改变detector/graph速度gate。

位置变化是return-center geometric speed，不是独立测量的物理UAV速度。Subwindow求解沿用该graph节点/edges及**全部完整bag的past occupancy history**，不重置background；但按原solver从窗内root初始化velocity，不承接窗前edge，不能等同截取full-bag赢家。没有混合回主cross调参。

| family | slow window | duration s | observed median speed m/s | original | background-only | motion-only | E18 |
|---|---|---:|---:|---:|---:|---:|---:|
| fine | 100m_cross_5 / 23–26 | 3.012 | 0.205 | 0/4 | 0/4 | 4/4 | 0/4 |
| fine | 100m_cross_10 / 21–24 | 2.977 | 0.504 | 0/4 | 4/4 | 0/4 | 4/4 |
| fine | 200m_cross_5 / 7–12 | 5.007 | 0.083 | 6/6 | 0/6 | 6/6 | 0/6 |
| fine | 200m_cross_10 / 34–37 | 3.009 | 0.102 | 4/4 | 4/4 | 4/4 | 4/4 |
| fine | 200m_cross_10 / 44–47 | 2.993 | 0.466 | 4/4 | 4/4 | 0/4 | 4/4 |
| historical | 100m_cross_5 / 23–26 | 3.012 | 0.205 | 0/4 | 0/4 | 0/4 | 0/4 |
| historical | 100m_cross_10 / 21–24 | 2.977 | 0.504 | 4/4 | 4/4 | 4/4 | 4/4 |
| historical | 200m_cross_5 / 7–12 | 5.007 | 0.083 | 6/6 | 0/6 | 6/6 | 0/6 |
| historical | 200m_cross_10 / 34–37 | 3.009 | 0.102 | 4/4 | 4/4 | 4/4 | 4/4 |
| historical | 200m_cross_10 / 44–47 | 2.993 | 0.466 | 0/4 | 4/4 | 0/4 | 4/4 |
| 每family合计 | 5窗/22observations | — | — | 14/22 | 12/22 | 14/22 | 12/22 |

没有满足条件的300m≥3observations/≥2s slow窗，不能解释为不存在300m hover。两family评的是同22observations，不合并成44个独立样本。总计也会遮住局部gain/loss，必须保留逐窗表。

### 6.1 200m_cross_5 / 7–12：relative novelty失选

固定witness六个node查询同1m cell `[199,31,−5]`；prior `(F,H)`从(7,0)到(12,5)，b逐帧 **0.7778、0.6000、0.4545、0.3333、0.2308、0.1429**。全部仍正，总B=+2.539283；没有hard删除任何候选。

| fine graph路径 | S | B（已启用） | M（已启用） | E18 score |
|---|---:|---:|---:|---:|
| 原固定silver witness | 5.723293 | 2.539283 | 0.033662 | 8.228914 |
| E18该短窗无条件winner，0/6silver | 5.211493 | 4.423066 | 0.273682 | 9.360877 |

Gap项均0，端点duration一致。Winner原node证据更低、motion罚更高，但background奖励更高，故逆转排序。Historical同样：witness score8.519695 vs winner9.382123。Background-only也0/6。来源：[short_slow_fixed_path_scores.csv](results/e18_mechanism_generalization_audit/short_slow_fixed_path_scores.csv)、[short_slow_evaluation.csv](results/e18_mechanism_generalization_audit/short_slow_evaluation.csv)、`witness_background/*.csv`。

其余四窗witness的所有b也为正，B分别+3.258/+3.353/+3.473/+3.666。所以本轮实际观察到的是repeated-cell novelty奖励衰减及竞争排序风险，不是“已测得所有slow target变负分”。完整bag E18仍能命中这些六个frames；短窗失选不与79/80矛盾，因为长程上下文与边界初始状态不同。

### 6.2 不运行新算法的公式级hover风险

由E18已有公式直接得到 `b=(F−2H)/(F+2)`：当一个cell在过半prior frames里有returns，b变负。这不是速度门限；hover、slow同cell、路径revisit和static clutter都会提高H。

理想化、无漏测、此前未占用的同cell从frame f0开始持续有return，frame f查询时F=f、H=f−f0，故 **b=(2f0−f)/(f+2)**：

- late arrival起初仍有较大正奖励，随后衰减；f>2f0才为负。
- 从采集开始持续占cell，f0=0，则b=−f/(f+2)，趋近−1。原node score≤1，最高可能augmented node score≤2/(f+2)，低于该上界的node可以变负。
- 静止/恒速直线的理想motion penalty为0；motion term本身不能区分hover UAV和静态背景，也不会给hover补偿其novelty损失。

这是**条件性代数结论/机制风险，不是新仿真实验或真实hover性能结果**。漏return、跨cell、jitter、grid边界、目标arrival time、前期背景占用及centroid cell内是否有实际点都会改变H。故可说“长时间反复占同cell有结构性失分风险”，不能说“所有slow UAV都会系统性失检”。真实hover身份、长时效应和跨日期泛化仍UNKNOWN。

## 7. 执行与验证

环境：既有`fmcw`，Python3.10.12 / NumPy2.2.6 / Linux6.8.0-136-generic；不安装新package、不改环境、无随机性。完整命令见[README](results/e18_mechanism_generalization_audit/README.md)。关键阶段：

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py run
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py evaluate
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_summaries.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/audit_e18_mechanism_outputs.py
```

Full run：10:56:22.146790–11:01:25.199857 UTC，303.053s。逐包runtime包含求解、既有pool核对与分解，不含raw bag/候选生成；总wall还含数据加载/hash/输出。评价完成UTC11:07:04.128440。没记录独立评价起始时间/peak RSS，不虚构这些数值；这些不是online FPS。

一次失败完整保留：首次`evaluate`在labels读取前的output hash检查退出。库存agent在run manifest生成后完善了`generalization_inventory.csv`来源，唯一非solver元数据hash不符。原manifest逐字副本和[manifest_reconciliation.json](results/e18_mechanism_generalization_audit/manifest_reconciliation.json)保留旧/新hash；当前manifest仅修该hash并加provenance note，**没有改任何score/path输出或冻结脚本**。核对后才执行评价；没有重跑full推断。不能声称原run manifest本身逐字未变。

验证结果：

- 12新增synthetic term-ablation/小图exhaustive tests通过，14既有E18 tests通过。
- 212个protected输入/旧结果/源文件hash保持不变；37,117nodes的background algebra一致。
- 20个original/完整E18 full pools精确重放；40个full赢家、120fixed rescores、40同窗margin、640银标帧、24range rows、全部10个family-window描述和40stress输出通过独立只读核查。
- 全部40 full terminal score文件保留，共3,328,536个representative records，便于结果审计；不是top-K全组合paths。
- Original/E18的40条span rank记录从已有E18 span pools独立重计。两个新增ablation arm的span rank记录验证scope/语义，**未独立重新求解其最优性/重计完整span pools**；不能把此项说成全数独立rank复现。新增12个synthetic tests覆盖四arm求解正确性，但不构成实图穷举证明。
- 本轮继承E18此前已核查的causal occupancy历史，本次只从冻结节点检查代数，不重新解码raw cache生成背景。

完整scope见[independent_audit.json](results/e18_mechanism_generalization_audit/independent_audit.json)及[validation_checks.json](results/e18_mechanism_generalization_audit/validation_checks.json)。最终报告/文档/脚本/结果身份另见[final_deliverables_manifest.json](results/e18_mechanism_generalization_audit/final_deliverables_manifest.json)。未修改旧E18报告或旧结果；新执行身份单列。

## 8. 统计解释审计与结论边界

Material Passport：Origin Skill=`experiment-agent`（academic-research-suite）；Origin Mode=`validate`；Origin Date=`2026-10-11`；Verification Status=`VERIFIED`（仅执行/存储结果核对）；Version Label=`e19_validation_v1`。Overall Confidence：**CAUTION**，因为独立GT/negative/hover/held-out证据缺失；不是论文级方法认证。

没有p值、显著性检验或独立样本置信区间；80frames存在强时序相关，四arm是同数据确定性比较。Statistical/methodological fallacy覆盖 **11/11 checked**：

| 检查 | 状态与本轮处理 |
|---|---|
| Simpson's paradox | 分开两family、五包、100/200/300m；另列slow local gain/loss，不以总体掩盖失败 |
| Ecological fallacy | 不由range组计数断言每种UAV或每个hover个体效应 |
| Berkson's paradox | CAUTION：closure成功、可观察silver的选择可能诱发feature关联；不是全体UAV采样 |
| Collider bias | CAUTION：closure/observability-conditioned DEV，不能推断总体feature因果关系 |
| Base-rate neglect | UNKNOWN target prevalence；不报operational precision/negative FA |
| Regression to mean | 同一冻结graph/labels做确定性开关，不以先选失败包后恢复称独立泛化 |
| Survivorship bias | CAUTION：只评80observed silver；未观察frames和failed closure不能变negative或消失于覆盖说明 |
| Look-elsewhere effect | 四arm/所有既定窗全保留，无parameter sweep或threshold选择 |
| Garden of forking paths | 参数/公式预冻结；slow subset明确为retrospective；唯一metadata修正及失败保留 |
| Correlation vs causation | 仅在固定graph内归因term开关；不称真实UAV总体因果增益或hover系统性结论 |
| Reverse causality | Background只用past；银标事后评价不进入full winner选择；完整DP仍离线且非因果输出 |

## 9. 交付与停止

新增报告、独立results、审计adapter/tests、只读checker及状态/实验/研究日志。协作者最少接收本报告、结果[README](results/e18_mechanism_generalization_audit/README.md)、四arm evaluation、fixed components/ranks、slow四文件和generalization inventory；完整复核还需本轮脚本及E18冻结graphs、protocol/input identities。原始大bag不需要为读懂本轮结论全部传输，但数据级重建需要原cache/bag和其hash。

**最终判断：机制审计完成，泛化认证未完成。** 证据支持“background打压既有persistent static参考，motion在有background时限制novel incoherent竞争路径”的有界解释；也已经保留slow相对novelty失选的负结果。B及真正长期hover问题无法从当前仓库可靠回答。保持E18冻结、remaining miss不动，不选threshold、不加feature，不继续算法实验。
