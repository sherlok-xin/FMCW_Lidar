# FMCW External Reproduction Roadmap

核查日期：**2026-10-14（Asia/Shanghai）**。范围严格限于下列三个仓库。**VERIFIED RESULT** 仅指公开页面、源码、树及metadata已核对；**IMPLEMENTED** 不等于本机成功运行。本轮未安装依赖、下载数据集、构建/运行方法或编写实现。难度、成功标准与停止条件是本项目的 **PROPOSED** 复现计划，不是实验结果。

## 先完成的归档与项目决定

已首先将 [Occupancy Temporal-Structure Audit](FMCW_OCCUPANCY_TEMPORAL_STRUCTURE_AUDIT.md) 和完整37文件结果目录按原字节提交、推送并核验远端main：[92cf79d01bbb84eee52dde7fd22210f78067d6cd](https://github.com/sherlok-xin/FMCW_Lidar/commit/92cf79d01bbb84eee52dde7fd22210f78067d6cd)。报告SHA-256仍为 `471469a6a82958449310b4af6d4919ac26e7086af1f1d7c9841f207f5dad62d3`；其manifest、说明、图、CSV、code均未改。旧文件中的未推送文字是完成时快照，本轮不回写该快照。[发布/原字节核验收据](results/external_reproduction_roadmap/audit_publication_receipt.json)另存于新目录。

**STOP_DEV_FEATURE_MINING 已生效**，见 [项目决策](docs/DEV_FEATURE_MINING_STOP.md) 与 `AGENTS.md`。五条cross DEV bags和80个DEV_SILVER anchors不得再用于提出/调节E18 background features、weights、windows、thresholds或E21/E22 score。新表示、子段、外部tracker或synthetic成功都不解除这一决定。E18保持不变；下列三项的第一轮均不使用我们的DEV数据。

## 1. 当前公开状态与第一轮数据选择

以下为本次实际读取的snapshot；`pushed_at`/HEAD时间与正式release时间分开记录，不以stars、metadata updated_at或论文接收代替代码发布。三个仓库均为public、非fork、未archived/disabled，默认branch均为main。API/tree摘录、完整性标记、源文件hash与核查摘要见 [repository_verification.json](results/external_reproduction_roadmap/repository_verification.json)。[Stone Soup metadata](https://api.github.com/repos/dstl/Stone-Soup)、[DopplerTrack metadata](https://api.github.com/repos/12w2/DopplerTrack)、[UnLiDAR metadata](https://api.github.com/repos/lianghanfang/UnLiDAR-UAV-Est)。

| 项目 | 本次HEAD / HEAD日期UTC | 正式release与实现状态 | 第一轮数据 | 难度判断 |
|---|---|---|---|---|
| [dstl/Stone-Soup](https://github.com/dstl/Stone-Soup) Bernoulli Particle Filter | `56f711c50cb577276dc0e4d95c09998acbb15c08`；2026-10-13 14:40:18 | 最新release **v1.9.1**，2026-06-24；官方组件/合成测试已发布，完整Bernoulli端到端gallery例未核实 | **Synthetic clutter first**；不连接FMCW | 低至中；API与empty-scan语义是实质门槛 |
| [12w2/DopplerTrack](https://github.com/12w2/DopplerTrack) | `780d0e47c629e21df35c026b9de1313e6d3ee167`；2025-07-18 08:09:43 | C++方法、launch和eval存在；无GitHub release | **作者支持的Aq-car路径先**；数据需申请 | 中高；数据获取、ROS1环境和帧对应 |
| [lianghanfang/UnLiDAR-UAV-Est](https://github.com/lianghanfang/UnLiDAR-UAV-Est) | `2b8e7ba10c2e6883899adc88f4b6455bfd3e5bc9`；2024-08-03 09:02:30 | 官方局部片段存在；完整可运行论文方法发布未建立；无release | 将来需**作者数据+完整官方路径**；当前 **BLOCKED** | 官方完整复现目前被阻塞；自行重建歧义高 |

DopplerTrack的pushed_at为2025-07-18 08:09:44 UTC，其余两项与表中HEAD时间相同。Stone Soup未来执行拟锁定release **v1.9.1 / `a4336b920a799cfe0a77ecb05867c5deeb371c7a`**，不混用浮动latest文档和另一安装版本。[Release](https://github.com/dstl/Stone-Soup/releases/tag/v1.9.1)。

## 2. Stone Soup Bernoulli Particle Filter

**实现身份与license。** 这是DSTL维护的Stone Soup官方框架及其Bernoulli实现，不是Ristic等人原作者独立发布的代码。其Bernoulli类引用2013 Bernoulli tutorial。代码MIT许可；维护者、Python要求与项目身份均在官方metadata声明。[pyproject](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/pyproject.toml)、[LICENSE](https://github.com/dstl/Stone-Soup/blob/56f711c50cb577276dc0e4d95c09998acbb15c08/LICENSE)。

**依赖与样例。** Python≥3.10；基础依赖为NumPy≥1.17、SciPy、matplotlib、ordered-set、plotly≥5、pymap3d、rtree、ruamel.yaml≥0.16.5、utm。构建metadata要求setuptools≥64、setuptools_scm≥8；组件测试另需pytest。没有理由为此安装全部dev/docs/video extras或torch。版本兼容性和rtree平台依赖仍待将来隔离环境验证，本轮未安装。[依赖清单](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/pyproject.toml)。

明确存在的官方起点是 `BernoulliParticlePredictor`、`BernoulliParticleUpdater`、`BernoulliParticleState`，以及源码中直接生成小规模synthetic states/detections的组件测试，不需数据集或weights。已核实的测试为predictor的 `test_bernoulli_particle_no_detection` / `test_bernoulli_particle_detection`、updater的 `test_bernoulli_particle`。[Predictor tests](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/tests/test_particle.py)、[updater tests](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/tests/test_particle.py)。

没有核实到一个现成的官方“Bernoulli synthetic clutter全流程”gallery入口，不能虚构下载/运行命令。将来最小多帧存在性例子若由我们连接官方组件，应写成 **local synthetic harness using official Stone Soup components**，与官方组件测试分开，不能冒称原作者端到端example。已有测试采用指定先验和检测，不能作为自主target birth验证。

**精确科学问题。** 在至多一个目标的已知synthetic观测模型下，能否把“目标是否存在”的Bernoulli概率，与“已存在时在哪里”的particle分布分开？出生、生存、missed detections及clutter intensity怎样改变存在信念，什么时候仅有clutter仍会产生较高信念？这是对当前缺少target-absent推断的概念与数值检查，不能回答一个FMCW点是否为UAV，也不是E18性能试验。[Predictor](https://github.com/dstl/Stone-Soup/blob/56f711c50cb577276dc0e4d95c09998acbb15c08/stonesoup/predictor/particle.py)、[updater](https://github.com/dstl/Stone-Soup/blob/56f711c50cb577276dc0e4d95c09998acbb15c08/stonesoup/updater/particle.py)。

**与本项目不匹配的假设。** Bernoulli只表示0/1目标；synthetic点测量、给定P_D/运动噪声/Poisson或uniform clutter，不等于FMCW多返回、相关背景、稀疏候选及未知曝光。E18的occupancy p不能直接替代clutter measurement density或P_D；spatial density与每scan clutter count也不能混同。实现中的birth/survival按predictor调用发生，不能无转换移植到不均匀bag cadence。指定空间先验不等于盲搜索成功；后续synthetic presence truth只用于评价，不用于挑birth particles。

**成功标准及严格stop gate（未来计划）。** 先让锁定版本的官方组件测试通过；随后单一、预定义synthetic presence/absence/birth/survival/miss/clutter-only时间表，用固定seed和事先记录的观测参数，记录每步predicted/updated existence与归一化particle weights。检查概率有限且在[0,1]、权重归一、particle数量受控、输入无oracle。不得用“clutter-only时永远不报警”作未经证明的承诺，也不选择一个DEV-informed存在阈值。

先核对Bernoulli Bayes算术：`r_pred=p_birth*(1-r_prev)+p_survival*r_prev`；对固定scalar P_D和空measurement set，应验证 `r_post=r_pred*(1-P_D)/(1-r_pred*P_D)`。这是预定理论sanity check，不是已执行结果。**源码观察**：当前及v1.9.1的Bernoulli updater把显式existence测量修正放在 `if any(hypotheses)` 内；无真实detection的路径需要先澄清，不能先保证它已正确体现absence evidence。正确构造missed-detection hypothesis container；不要把空Python list直接当合法update输入。[固定源码检查位置](https://github.com/dstl/Stone-Soup/blob/56f711c50cb577276dc0e4d95c09998acbb15c08/stonesoup/updater/particle.py#L474-L540)。

若官方组件在合法输入下不符合上述约定、基本测试/依赖语义未解决，或只有oracle初始化/修改官方算法才能展示预期行为，就保留失败证据并**停止、核实上游**。不静默patch，不宣称已运行发现bug，不接入FMCW。数值测试通过也不等于真实FMCW存在概率已校准。

## 3. DopplerTrack

**实现身份与license。** 这是作者官方Doppler/tracking参考实现：仓库内作者preprint直接给该URL，第一作者与README联系人一致；公开了publisher/detector/tracker、launch及评估。代码根LICENSE为MIT；数据另为CC BY-NC-SA4.0。`package.xml`的license仍为TODO，记录这个metadata缺口，不将其当作已解决的全依赖许可审计。[作者preprint](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/preprint_paper/DopplerTrack.pdf)、[code LICENSE](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/LICENSE)、[DATA_LICENSE](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/DATA_LICENSE.md)。仓库preprint是原投稿版，不能假定包含最终IEEE修订。[版本声明](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/preprint_paper/README.md)。

**依赖。** 作者测试环境为Ubuntu20.04、ROS1 Noetic/catkin、PCL1.10、OpenCV4.2；CMake/package另依赖ROS messages、pcl_ros/conversions、tf等。源码引用 `nlohmann/json.hpp`，在所见manifest中未声明，需要未来仅作环境/packaging缺口处理，不能混作算法修改。评估依赖实际位于 **eval/requirements.txt**：NumPy、pandas、SciPy、xmltodict、python-dateutil、pytz、tzdata、six，以及未精确锁定的opencv-python、Open3D、shapely。没有声明GPU或训练需求；精确runtime UNKNOWN。[Build](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/CMakeLists.txt)、[package](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/package.xml)、[eval dependencies](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/eval/requirements.txt)。

**数据/样例与官方入口。** 作者列出Straight、Intersection、Aq-car、Aeva-car；README要求通过Google Form申请下载链接。仓库data目录只有README，未提供可直接运行的点云样本；四份GT文本和vendored motmetrics test fixtures不能代替作者LiDAR数据。数据规模、访问批准与最小子集可用性 **UNKNOWN**，本轮未提交表单、未下载。[官方数据说明](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/data/README.md)、[官方运行说明](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/README.md)。

第一轮固定走作者README的Aq-car例：`roslaunch dopplertrack dopplertarck_aq_car.launch`（文件名原拼写），输入 `data/original_bin/Aq-car`。AQ输入按六float32的24B记录读取：x,y,z,intensity,velocity,time；tracker输出 `eval/output/<counter>.json`。评价在eval目录运行 `python eval.py Aq-car`，还需匹配的 `data/dynamic_pcd/Aq-car` 与GT。以上仅记录入口，本轮未执行。[Launch](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/launch/dopplertarck_aq_car.launch)、[publisher](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/src/publisher_node.cpp)、[eval](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/eval/eval.py)。

**需要先解决的编号问题。** 该launch从start_frame=500跳过输入，tracker输出counter从1开始，Aq-car eval选择70–140；仓库preprint记载Aq-car350帧。可能涉及作者分段/预处理/版本差异，**未执行、未证实数据矛盾**。取得作者manifest后必须核对input filenames→output counter→GT/PCD的映射，不能猜改start_frame并宣称官方指标复现。[Tracker源码](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/src/tracker_node.cpp)。

**精确科学问题。** 在作者支持的道路FMCW数据上，能否复现Doppler预处理、速度方向搜索/重构、关联与轨迹管理，并解释其radial-velocity和输入/输出约定？它能提供一个经过理解的tracking参考，不能给当前稀疏UAV target-absent推断或E18改进背书。

**不匹配。** 源码使用平面x/y预测、0.1s常数、道路物体尺寸/类别和地面/距离支持过滤，可能删除极稀疏returns；我们的3D UAV、约1Hz/不均匀cadence及弱support不满足这些现成假设。自定义bin和 `fmcw_lidar` frame也不同于原PointCloud2/`aqronos_lidar`；Doppler符号、sensor motion及timestamp约定必须重新核对，不能默认通用。源码的birth/death计数不是Bernoulli existence posterior。[Detector](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/src/detector_node.cpp)、[tracker](https://github.com/12w2/DopplerTrack/blob/780d0e47c629e21df35c026b9de1313e6d3ee167/src/tracker_node.cpp)。

**成功标准。** 锁定作者commit/隔离ROS1环境和数据身份，先解决schema、velocity、time、frame映射；原样跑通Aq-car发布/检测/跟踪，保存官方输出，在正确支持帧上完成作者eval。分别记录“pipeline跑通”和“论文数字复现”；后者需作者可比的版本/metric/数据范围与事先确定的数值误差容忍，不能事后挑子段或阈值。CPU论文性能不在本轮验证范围。

**Kill/stop。** 作者数据不可得、编号/GT支持仍不清，或必须自创预处理/改算法/调阈值才能继续，就停止官方指标复现并记录BLOCKED。先理解官方pipeline，不移植整个tracker到我们的UAV bags，不通过DEV补齐数据或修指标。难度中高是基于这些依赖/访问/对应风险的判断，不是实测工时。

## 4. UnLiDAR：官方发布程度必须单独判定

**身份。** 作者论文直接指向 `lianghanfang/UnLiDAR-UAV-Est`，所以这是指定的官方仓库，未替换第三方实现。论文arXiv v5已注明ICASSP2025接收；README仍保留under-review/待release文字，不能据它断言论文现在未被接收。[作者论文与代码链接](https://arxiv.org/abs/2412.12716)。

**源码/许可结论。** 完整、未截断recursive tree只有6个entries、4个文件：README、空 `includes/Readme.md`、`src/motion detect.py`、`src/voxel compare.py`。两个Python片段确实存在，不能写“完全没有代码”；但它们不是已建立可运行的完整论文方法。没有package/环境锁、完整配置/入口、端到端eval或sample。无LICENSE、API license=null：仓库代码许可 **未声明/UNKNOWN**，不能继承论文许可或数据网站许可。[固定完整树](https://github.com/lianghanfang/UnLiDAR-UAV-Est/tree/2b8e7ba10c2e6883899adc88f4b6455bfd3e5bc9)、[recursive-tree API](https://api.github.com/repos/lianghanfang/UnLiDAR-UAV-Est/git/trees/2b8e7ba10c2e6883899adc88f4b6455bfd3e5bc9?recursive=1)。

**依赖/数据。** 两片段imports可推得NumPy、SciPy、scikit-learn、Open3D，Python/库版本和支持OS **UNKNOWN**；这只是片段的依赖草图，不是官方方法环境。脚本依赖硬编码Windows本地Anti-UAV seq54/seq37路径和未说明来源的派生PLY文件。[Motion片段](https://github.com/lianghanfang/UnLiDAR-UAV-Est/blob/2b8e7ba10c2e6883899adc88f4b6455bfd3e5bc9/src/motion%20detect.py)、[voxel片段](https://github.com/lianghanfang/UnLiDAR-UAV-Est/blob/2b8e7ba10c2e6883899adc88f4b6455bfd3e5bc9/src/voxel%20compare.py)。论文使用MMAUD V2/V3；作者数据页面有外部下载/评价入口，但UnLiDAR未发布与片段对应的完整input manifest或sample，其外链实际可下载性、数据大小和最小可用子集本轮未验证。[论文v5](https://arxiv.org/html/2412.12716v5)、[作者数据入口](https://github.com/ntu-aris/MMAUD)。

**精确科学问题。** 若作者完整发布，在作者支持的稀疏LiDAR UAV序列上，原论文的全局/局部时空聚类、前景关联和轨迹拟合是否能按原protocol产生可比较轨迹？它是外部稀疏几何/时间序列参考，不回答FMCW Doppler或target existence问题。

**不匹配。** 论文的序列聚合/轨迹拟合不能自动称因果past-only方法；扫描/采样、同步与GT、强度/速度字段及range/support条件不同。未建立FMCW Doppler likelihood、target-absent推断或我们100–300m稀疏条件下的可转移性。将论文方法改成新的E18背景/score并在原DEV试验，明确受STOP_DEV_FEATURE_MINING禁止。

**成功标准、难度与stop。** 官方完整复现目前 **BLOCKED**；先等完整作者代码、明确许可、环境、作者支持input/config/eval/reference输出。将来只能先作者数据，跑通片段不算完整论文复现。若继续需要猜补聚类/scoring/spline/预处理或评价选择，就停止“official reproduction”路径；任何后来获独立授权的自写都必须标 **PAPER-STRUCTURE RECONSTRUCTION**，逐项记录推断选择。第三方实现必须另列出处与改动，不能冒充官方；本路线图没有选用任何第三方替代，也没有启动重建。官方复现工时目前UNKNOWN，paper reconstruction的歧义/浪费风险高。

## 5. 唯一首个复现任务推荐

**只推荐一项：Stone Soup Bernoulli存在性最小synthetic-clutter复现。** 从锁定v1.9.1的官方Bernoulli组件测试开始；若该门槛和empty-scan Bayes核查通过，再在同一个任务内连接明确标注的本地synthetic harness，观察预定义birth→presence→miss→absence与clutter-only阶段。它不接FMCW，不用80anchors，不拟合分类阈值，不改变E18。本轮仅提出该任务，尚未实现或运行。

选择依据：**relevance**——直接针对当前“有candidate/path却不能判断目标是否存在”的证据缺口；**reproducibility**——官方组件/测试可定位、无需申请真实数据或模型；**information gain**——区分existence与conditional state，显式理解birth/survival/P_D/clutter假设及空扫描更新；**浪费时间风险**——CPU小例可先用解析门槛快速否决错误语义，避免直接承担DopplerTrack数据/ROS移植或UnLiDAR缺发布的成本。若empty-scan/官方组件门槛失败，就保留负结果并停止核实，不自动转向DEV或E21/E22。
