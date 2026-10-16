# 组会重置简报：先把点云链看清楚

2026-10-16；约8页的中文讲稿/幻灯片提纲。只引用既有结果和本轮源码审计；未新跑实验、未改E18、未提交。详细来源见[主报告](FMCW_PROJECT_REINTERPRETATION_2026-10-16.md)及[历史审计](FMCW_HISTORICAL_PREPROCESSING_AUDIT.md)。

## 第1页｜采集几何：车没动，目标在开阔空中

- 用户补充：LiDAR固定在静止车辆，场景开放，树/地形/路侧密集点通常较低且持久。
- UAV约半米，通常在大部分密集背景上方；这是采集背景，不是新测绘/逐帧GT。
- 最简单的问题：从固定空域中保住稀疏目标相关回波，连成轨迹。不要先按移动平台/复杂城市场景想象。

图示：`静止车载LiDAR → 低处密集持久回波；上方开放空域 → 稀疏UAV点＋少量异常点`。

## 第2页｜raw很密，不等于监视空域很复杂

- raw包含大量低处、重复的场景回波；用户在RViz里看到的UAV点虽少，但局部集中，附近密集对象通常少。
- 已有scene audit：raw1m冻结persistent集合回波占比约96.60–98.07%（F≥2、past H≥2且p_beta11≥0.5）；任意历史已见cell占比98.82–99.70%；不是静态点GT比例。
- fine缓存只保留约6.2146% raw support，不能把fine里的问题讲成“从未处理完整raw”。孤立噪声/持久竞争点仍可能存在。

## 第3页｜师兄流程：字段压缩和真正过滤是两步

```text
get_lidar：xyz/I/v五字段，native xyz不动 → _81 repack
          ↓ 历史exporter待恢复
multi_filter：q=[x,z,-y] + I10..250 → filtered PCD
          ↓ 原cleaned-RViz publisher/topic待恢复
正确坐标显示；velocity辅助着色
```

- 现存batch脚本没有height/ROI；其它tracker才有z5/z10、I20等分歧版本。
- intended up是q_z=-native_y，不等于已surveyed gravity/AGL。
- 完整原流程目前BLOCKED；原有Sparse报告已写过链条，本次是补血缘/验证，非新发现。

## 第4页｜真实稀疏与体素压缩要分开

- 原始点数、4cm cells、1m cells是不同量。80邻域中31个在1m只剩1cell，其中30个不是raw单点。
- 1m体素把多个点的形状压成一个中心，不是“UAV只有一个回波/尺寸为零”。
- 300m也确有极稀疏/真实singleton；保留反例，不能笼统说稀疏全是人为造成。

建议图：`一簇raw dots → 多个4cm cells → 一个1m cell`，标注“表示改变，不是机体变小”。

## 第5页｜半米目标也不能用硬点数/硬尺寸裁掉

- 已有8个近距compact例因>32fine cells被拒，span仅0.268–0.813m；另有far singleton被min2删。
- adaptive DBSCAN改善连通性65→76/80，但matches仍71/80；不是检测率提升。
- 小尺寸/局部隔离是合理物理解释或soft prior；现有硬count/extent、velocity=0排除都不安全。不要再用银标找新门限。

## 第6页｜E18真正增加的是候选图上的时间选路

五包32+46+83+48+63=**272帧**，只有19+11+22+18+10=**80个已选DEV_SILVER observed anchors**。

```text
272帧 ──80帧有内部anchor── 同一fine图的saved最优路径距anchor≤3m？
                           原score：56个“是” /80
                           E18联合：79个“是” /80
       └─其余192帧未由这套anchor评价，不是negative
```

口头一句：“原路径在80个选定银标帧里有56帧经过参考邻域，E18联合背景与运动后79帧经过。” **不是独立检测recall，不是precision，也不是最终accuracy。** 原指fine原score；历史1m原路径是另一个0/80对照。E18吃的是ROI/I/provisional-height之后的pregeometry fine nodes。

保留负结果：background-only56、motion-only0、joint79；仍1miss，局部慢片段有退化。不能把总数变成普适成功。

## 第7页｜哪些理解需要重置，哪些否定结果保留

- GNSS只是粗飞行参考；最大26.575m残差主要along-track，不做逐帧UAV中心/无回波判定。
- 当前get_lidar不是师兄clean filter；一次缺轴变换的RViz视角不代表所有数值计算错，原诊断点/marker可在native basis内自洽。
- pregeometry≠raw；closure-derived height mask不是测得地平面。
- 场景/occupancy否定结果保留，STOP_DEV_FEATURE_MINING继续；Stone Soup保留空扫描STOP结果，存在性建模与其它外部repo复现先暂停。

## 第8页｜只做一个下一任务：原流程血缘与忠实核查

先找回原exporter、实际filter版本/参数、filtered-RViz发布与配置。缺失则明确BLOCKED，不把共存版本挑一个拼起来。

证据齐全后只按历史固定参数，在预定100m_cross_5、200m_cross_5、300m_cross_10做：每阶段点量守恒/删除accounting，正确轴顶/侧视，固定velocity着色；流程冻结后才看已有anchor邻域support保留。**不调参、不重拟合、不改E18、不生成新方法成绩。**

最低研究表述：固定空域稀疏回波保留与时间关联的可复核案例；独立presence/误警/泛化仍需新证据。

1. "What is the simplest correct description of the current problem?"——静止传感器、开放监视空域中的稀疏UAV证据保留与轨迹关联。
2. "What did we previously make more complicated than necessary?"——预处理未闭环前，过早将复杂背景/存在性模型当主线；必要审计与负结果不删。
3. "If we are allowed to do exactly one next technical task, what should it be?"——恢复并锁定实际历史链，再忠实核查逐阶段点量与正确轴视觉；缺证据保持BLOCKED。
