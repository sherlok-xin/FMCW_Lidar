# Stone Soup v1.9.1 — stopped semantic reproduction

**STOPPED_AT_EMPTY_SCAN_SEMANTIC_GATE；不是成功的完整Bernoulli存在性复现。** 官方锁定组件7项测试通过，但合法空扫描返回r=.495，独立Bayes后验=.08926961226330025。未修改上游，未进入多帧实验。[完整报告](../../../FMCW_STONESOUP_BERNOULLI_REPRODUCTION.md)。

| 文件 | 含义 |
|---|---|
| `source_identity.json` | 官方tag→commit、源码/已安装模块身份 |
| `environment.txt`, `pip_freeze.txt` | 独立Python环境、准确安装命令/版本、失败和警告 |
| `source_fetch.log`, `venv_creation.log`, `virtualenv_creation.log`, `install.log`, `install_mirror.log` | 保留setup尝试；ensurepip失败、首次慢下载取消、成功镜像安装 |
| `commands.log` | 重要实际命令与两次本地调用的说明 |
| `official_test_results.txt`, `official_test_summary.json`, `official_tests.junit.xml` | 7个官方node ID与实际输出，未改上游测试 |
| `source_semantics.md`, `independent_review.md` | 锁定源码语义、独立只读复核；后者未复跑滤波器 |
| `protocol.json`, `protocol_receipt.json` | **仅冻结单步先决门槛**，不冒称A–E场景已冻结 |
| `closed_form_scalar_reference.json`, `closed_form_check.json`, `closed_form_execution.txt` | 独立binary64/Decimal算术、实际合法Stone Soup返回值与完整日志 |
| `failed_attempt_01*`, `closed_form_execution_attempt_01.txt`, `closed_form_scalar_reference_attempt_01.json` | 本地脚本漏用StateVectors，执行滤波器前失败；只修正输入类型，参数不变 |
| `synthetic_schedule.csv` | **NOT_GENERATED**，仅header |
| `existence_trace.csv` | A **NOT_RUN**，仅header |
| `control_trace_clutter_only.csv` | B **NOT_RUN**，仅header |
| `control_trace_empty_scan.csv` | C短区间 **NOT_RUN**，仅header；前置单步核查另见JSON |
| `execution_status.json`, `validation.json` | 明确未运行、局部通过和失败；零数据行不是阴性结果 |
| `closed_form_check.pdf`, `closed_form_check.png` | 实际单步对照图，非时间序列；数学参考没有覆盖返回值 |
| `artifact_manifest.json` | 新增文档/本地helper及本目录文件SHA-256（不含自身），不扫描FMCW数据 |

本地执行helper在 [隔离工作区](../../../external_reproductions/stonesoup_bernoulli_v191/README.md)。`.venv/`与未修改的`upstream/`由该工作区.gitignore排除，不能加入Git；本轮没有commit/push。未读取或使用FMCW bags/anchors/scores/GNSS/calibration进行参数选择或成功判断，未安装系统包/ROS或修改原fmcw环境。

不存在新的完整synthetic harness、A/B/C结果、存在性阈值或FMCW接入。未来动作须先核实锁定官方空扫描语义；本归档不授权修补或换版本补救。
