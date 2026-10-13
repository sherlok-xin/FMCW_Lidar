# GNSS–LiDAR sanity archive

本目录包含上一轮目视材料和本轮冻结的 80 observed anchors 误差分解。

- [误差分解报告](DECOMPOSITION.md)：定义、逐 sequence median/IQR/max、方向一致性、等效时间和机制限制。
- [全部 80 anchors 的逐行 CSV](all_anchor_gnss_discrepancies.csv)、[分量汇总](sequence_component_summary.csv)、[方向一致性](sequence_direction_consistency.csv)、[同 chord 时间等效汇总](chord_time_bias_summary.csv)。
- [最大差异及相邻帧](maximum_discrepancy_neighbors.csv)、[最大帧分量图](figures/max_frame21_components.png)、[全序列时间图](figures/discrepancy_components_by_time.png)。
- [原始目视报告](REPORT.md)、[17 个诊断帧](SELECTED_FRAMES.md)、[归档范围与复制身份](ARCHIVE_README.md)。
- [计算 provenance](decomposition_provenance.json)、[独立审计](independent_decomposition_audit.json)、[最终输入完整性](integrity_audit.json)、[提交清单](publish_manifest.json)。

最大 `200m_cross_5/frame21` 是沿航迹主导：+26.389269 m / −3.056353 m / +0.719090 m（along / horizontal-cross / vertical-normal），严格冻结 ENU-Up 为 +0.629154 m。局部等效时间 +6.707545 s 已越过原 GNSS 终点，仅为描述性投影，不应用为新 offset。

所有 calibration、time offsets、GNSS 插值、labels、E18 和历史研究输入保持冻结；没有新拟合、bag 读取、ROS运行或方法评分。Observations 为 selected DEV/SILVER，不是独立 GT。

`markers.py` 按要求保留在本地归档，但由目录 .gitignore 排除 GitHub；bag、ROS runtime/config/log/cache 同样排除。GitHub 仅包含小体积文档、CSV、图像和无需 ROS 的数值计算/核验脚本。旧 REPORT/WORKFLOW 的 `/tmp` 路径属于历史记录，详见归档说明。

从仓库根目录复现（依赖现有 Python/NumPy/Matplotlib 环境，无 ROS）：

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/rviz_gnss_lidar_sanity/decompose_discrepancies.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/rviz_gnss_lidar_sanity/independent_decomposition_audit.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/rviz_gnss_lidar_sanity/plot_discrepancies.py
```

复现会覆盖本目录的派生 CSV/JSON/图像，保留原 inputs。已有历史归档副本不会被上述脚本重写。
