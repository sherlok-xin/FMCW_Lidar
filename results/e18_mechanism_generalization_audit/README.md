# E19 — E18 mechanism/generalization audit

独立诊断目录，基于冻结 E18 `a53702da1dc5357149e3e472a9e489ff08e9afe4`。不修改 E18 公式、权重、graphs、labels、3 m gate 或历史 baseline，不用 Doppler，不选择 threshold。

报告：[FMCW_E18_Mechanism_Generalization_Audit.md](../../FMCW_E18_Mechanism_Generalization_Audit.md)。本目录四个 arm 为 `original`、`background_only`、`motion_only`、`background_motion_E18`。只开/关已有 term；共用未修改的 E18 solver。两套 representation 分开比较，不把跨 graph 差异归因为 score 收益。

## Artifact map

| 文件 | 用途与边界 |
|---|---|
| `protocol.json`, `FROZEN`, `freeze_identity.json` | 真实评分前冻结的 arm、函数、scope 与新脚本身份 |
| `input_identity.json` | 212 个 E18/上游 protected 文件身份；不包含新审计报告或后来更新的状态文档 |
| `run_manifest.json` | 40 次 full-bag label-free 求解的命令、环境、时间、输出哈希 |
| `run_manifest_before_inventory_finalization.json`, `manifest_reconciliation.json` | 保留首次 run manifest；记录唯一库存 CSV 的后续元数据完成及哈希核对。所有 solver 输出未变 |
| `e18_exact_replay.csv` | 20 次 original/完整 E18 的路径和全部 qualified terminal score 精确重放 |
| `selected_paths.csv`, `unconditioned_summary.csv` | 四个 arm 的全部无条件 winner、duration、raw/applied score components、路径运动/背景性质 |
| `terminal_scores/*.csv` | 40 个完整 bag 的 qualified terminal-edge representative scores；不是全组合路径列表 |
| `unconditioned_evaluation.csv`, `silver_frame_evaluation.csv`, `summary_by_range.csv` | 事后以原 3 m gate 评价，40 个 sequence-arm 结果、640 个 observation-arm rows、24 个 range-arm 汇总；只评观察到的 DEV_SILVER |
| `fixed_path_scores.csv`, `fixed_path_ranks.csv`, `score_margins.csv` | 不重新选择的 E18 witnesses/旧参考重算；matched-span rank 下界，不是 combinatorial rank |
| `term_contrasts.csv`, `fixed_component_differences.csv`, `matched_frame_changes.csv` | 已保存 CSV 的配对描述性归因；固定路径可加性不等于 winner recall 可加性 |
| `witness_background/*.csv` | 已冻结 witness 的 prior occupancy、node term、observed geometric speed |
| `short_slow_windows.csv`, `short_slow_evaluation.csv`, `short_slow_selected_paths.csv`, `short_slow_fixed_path_scores.csv` | 所有满足冻结 slow 诊断条件的 cross-silver 子窗，单列 stress；保持完整 bag 的历史背景，非独立 hover GT |
| `generalization_inventory.csv`, `generalization_inventory.json` | 原始 bag/ZIP/PCD 和历史 annotation/protocol 的 evidence inventory；无可靠 target-absence 或独立 hover 认证 |
| `reliable_negative_winners.csv` | **仅表头，无 observation**：negative 求解未运行；不代表零虚警 |
| `evaluation_manifest.json`, `summary_manifest.json` | 事后评价与纯 CSV accounting 的输出身份 |
| `independent_audit.json`, `validation_checks.json` | 独立只读核对、synthetic tests、来源身份及已知验证边界 |
| `final_deliverables_manifest.json` | 报告、更新文档、新脚本及本目录所有既有交付文件的最终 SHA；不包含 manifest 自身 |

## Commands and preservation

在原 frozen inputs、原环境和本轮脚本身份下执行过：

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python -m unittest discover -s tests -p 'test_e18_mechanism_audit.py' -v
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python -m unittest discover -s tests -p 'test_background_motion*.py' -v
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py run
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py evaluate
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_summaries.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/audit_e18_mechanism_outputs.py
```

完成目录禁止覆盖；`freeze/run/evaluate/summaries` 会拒绝相应已完成阶段。今后复核可执行最后一条只读 checker；重新执行完整实验需独立 checkout/空的实验输出位置并保留本次目录，不删除历史目录来满足脚本的覆盖检查。

首个 `evaluate` 在读取 labels 前因库存 CSV 哈希不一致退出；唯一变化和原 manifest 已保留。核对后再执行评价，没有重新执行 full-bag 求解或改变算法。库存更新不是受保护 E18 输入更新。

没有可靠 negative 时，target-absent winner/false alarms/operational precision 一律 **UNKNOWN**，未标帧不是 negative。短 slow 子窗来自既有回溯 silver，仅为机制 stress，不是独立泛化验证；子窗起始 velocity history 按原 solver 初始化，不承接窗前 edge，background history 则完整保留。
