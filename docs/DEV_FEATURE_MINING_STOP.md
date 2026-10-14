# STOP_DEV_FEATURE_MINING

**状态：生效。日期：2026-10-14（Asia/Shanghai）。决策来源：用户明确指令。**

现有五条 cross-flight DEV bags 及80个 DEV_SILVER observed anchors，停止用于提出或调整任何新的 E18 background features、weights、windows、thresholds，以及 E21/E22 score。范围包括：

- `100m_cross_5`
- `100m_cross_10`
- `200m_cross_5`
- `200m_cross_10`
- `300m_cross_10`

不得通过重新分箱、选择子段、重命名实验、组合既有特征、替换表示或引入外部参考实现，在同一DEV证据上继续这一轮特征/评分探索。保持E18、labels、calibration、offsets、graphs、阈值和历史结果不变。保存、检查和引用已有证据不构成重新开放该DEV集。

依据：[Occupancy Temporal-Structure Audit](../FMCW_OCCUPANCY_TEMPORAL_STRUCTURE_AUDIT.md) 的严格停止规则未获五序列稳定共同支持。其原报告与完整结果目录已按原字节提交并推送：`92cf79d01bbb84eee52dde7fd22210f78067d6cd`；远端main已核验。原文件中的“未提交/未推送”描述属于审计完成时快照，不能为更新发布状态而改动该快照。

下一阶段限于理解和规划三个指定外部实现：Stone Soup Bernoulli Particle Filter、DopplerTrack、官方UnLiDAR。第一步只能是外部支持的数据路径或预定义synthetic例子，不将当前DEV bags接入或当调参集。外部复现成功不会自动解除本决策。

本轮仅做仓库/文献核查与路线图，不安装大依赖、不下载大数据、不实现方法。未来是否执行外部复现属于独立任务，不是E21/E22授权。
