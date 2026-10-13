# GNSS–LiDAR 目视核对与冻结误差分解归档

本目录归档 2026-10-13 的临时 ROS2 目视诊断，并保存本轮基于现有 frozen calibration、80 个 observed anchors 和既有 GNSS 插值表的误差分解。它是诊断记录，不是新方法、GT、标定结果或 E18 评分更新。没有重新拟合外参、时间 offset 或 GNSS 插值。

## 已复制的历史材料

- [REPORT.md](REPORT.md)、[SELECTED_FRAMES.md](SELECTED_FRAMES.md)、[WORKFLOW.md](WORKFLOW.md)、[selected_frames.csv](selected_frames.csv) 和 [selected_frames.jsonl](selected_frames.jsonl) 均为 `/tmp/fmcw_rviz_check/` 原文件的逐字节副本；历史叙述和命令没有被改写。
- `markers.py` 按用户要求复制到本地归档。它是临时 ROS2 runtime helper，已由本目录 [.gitignore](.gitignore) 排除，不提交到 GitHub。它仍从既有冻结表读取坐标，使用原始 bag/frame identity 同步；历史 ROS header epoch 不作为 GNSS 查询时间。
- [visual_run_manifest.json](visual_run_manifest.json) 和 [visual_repo_integrity.json](visual_repo_integrity.json) 是旧 `run_manifest.json` 和 `repo_integrity.json` 的逐字节副本。改名只为与本轮 decomposition provenance 区分；其内部路径、hash、环境和历史核对结果没有改动。
- `figures/` 保存五个 sequence contact sheets、`200m_cross_5/frame21` 的 top/side detail，以及两份原始 static figure manifests。图中为已保存的 raw region-union crop，不是完整点云，也不是点级 GT。图像没有重绘、裁剪或修改。
- `screenshots/` 保存实际 RViz overview：[200m_cross_5/frame21](screenshots/raw_200m5_f21_rviz.png) 和 [200m_cross_10/frame40](screenshots/raw_200m10_f40_rviz.png)。图内 condition/frame 标签与文件名一致。它们仅说明实际显示状态；精确空间比较应联合静态正交图和数值表进行。

## 历史链接与可执行性的边界

`REPORT.md` 和 `WORKFLOW.md` 保留当时的文本，因此其中“全部材料位于 `/tmp/`”“未修改 repository”等语句描述的是前一轮任务结束时状态。本轮新增了本目录归档和诊断分解文件。

`REPORT.md` 内 `processed/100m_cross_5_checked` 与 `processed/100m_cross_10_checked` 的相对链接指向当时的临时录制；本归档没有复制这些 ROS2 bags。对应历史位置分别是 `/tmp/fmcw_rviz_check/processed/100m_cross_5_checked/` 和 `/tmp/fmcw_rviz_check/processed/100m_cross_10_checked/`，它们可能随临时目录清理而消失。`WORKFLOW.md` 的绝对命令同样引用历史 `/tmp/fmcw_rviz_check/` 环境、player、adapter 和 RViz 配置，不能把该文档理解为独立可执行安装包。

原 static figure manifests 引用其完整旧输出清单；本归档仅选择代表图，其余历史图和运行材料仍在原临时目录。复制清单以本目录 [archive_manifest.json](archive_manifest.json) 为准。

## 复制身份与排除项

[archive_manifest.json](archive_manifest.json) 为每份复制材料记录原 source path、source SHA-256、归档相对 target path、target SHA-256、文件字节数以及 `git_publish`。复制后逐文件核验两侧 SHA-256 相同；`git_publish=false` 的 `markers.py` 仅保存在本地。

没有复制或提交大 bag、原始数据、临时 ROS 环境、player/adapter、`.rviz` 配置、ROS 日志、receipt logs、bag receipt SQLite 文件、Python bytecode 或字体/渲染 cache。仓库根目录的既有 `fmcw_rviz_check/` 与 `fmcw_range_sparsity_mpl/` 仅新增精确 ignore 规则保护；原目录保留。

两张旧截图没有纳入代表证据：`raw_100m5_f12_top.png` 是失败的黑色 hardware-GL viewport；`processed_100m10_f18_rviz.png` 文件名与画面身份不一致，画面标签实际为 `100m_cross_5 frame31`。不根据这些文件名推断帧身份；历史失败仍保留在原临时材料中。

## 本轮分解材料

[all_anchor_gnss_discrepancies.csv](all_anchor_gnss_discrepancies.csv) 覆盖全部 80 observed anchors。分解定义、逐 sequence 汇总、方向一致性、等效时间偏差、最大偏差相邻帧和因果限制由同目录新生成的 decomposition 文件及 [decomposition_provenance.json](decomposition_provenance.json) 记录。原校准、标签、E18、历史报告和数据不因归档而修改；数值分解不能自动将 observed anchors 升级为独立真值。
