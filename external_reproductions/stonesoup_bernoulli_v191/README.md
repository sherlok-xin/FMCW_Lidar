# Local Stone Soup v1.9.1 semantic reproduction workspace

官方来源：`https://github.com/dstl/Stone-Soup.git`；固定tag `v1.9.1` → `a4336b920a799cfe0a77ecb05867c5deeb371c7a`。`upstream/`与`.venv/`仅本地，已忽略，未改变官方算法。此处是本地audit helper，不是作者发布的端到端synthetic example。

[完整报告](../../FMCW_STONESOUP_BERNOULLI_REPRODUCTION.md) / [运行归档](../../results/external_reproductions/stonesoup_bernoulli_v191/README.md)。

- `run_official_tests.py`：先运行锁定源的7个标准Bernoulli相关官方测试，NumPy seed191，stdout另存归档。
- `check_empty_scan.py`：仅一个冻结参数的合法空扫描核查；独立算术先于Stone Soup导入，**不覆盖/修正官方r**。返回code2明确表示语义门槛不通过、必须停止；本轮实际如此。
- `plot_closed_form.py`：只画已执行单步对照，不生成轨迹。
- `validate_artifacts.py`：只验证本轮证据、源码身份和算术，不再次运行滤波器，不读FMCW科学数据。

环境安装失败/重试、准确命令与版本见归档`environment.txt`和`commands.log`。首次本地输入类型错误与修正前源码也保留。未生成完整多帧protocol、synthetic schedule或A/B/C traces；对应CSV仅header且标NOT_RUN。不要把再次调用helper视为授权改公式或继续原本被停止的实验。

本轮保持STOP_DEV_FEATURE_MINING、E18和所有历史科学结果，不commit/push，等待用户审阅。
