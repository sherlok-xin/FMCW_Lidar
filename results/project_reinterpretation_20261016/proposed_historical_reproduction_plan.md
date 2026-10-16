# 单次受控历史流程复现计划（PROPOSED；本轮 NOT_RUN）

日期：2026-10-16。**精确完整师兄流程：BLOCKED。** 仅现存 `multi_filter_with_intensity.py` 的q轴变换＋I10–250子阶段源代码可恢复。下述执行必须先满足历史版本/数据血缘门槛；不把缺口用新算法补齐，也不自动授权执行。

## 固定范围与选择理由

代表包预先固定为 `100m_cross_5`、`200m_cross_5`、`300m_cross_10`，各完整包，不按点云好看/anchor保留率再换包。覆盖100/200/300 m三档，只作工程/描述性检查；不是新评价集。200m_cross_5保留已知最大GNSS差异所在包，但不据此选门限、不重算offset。帧身份沿用bag-record/index，不能拿过期header epoch对齐GNSS。需要小样本展示时，单纯从各包现有metadata的索引取首/中/末帧，完整包accounting不挑帧；anchor展示只在冻结执行完成后追加。

## 执行前必须恢复的证据

1. 师兄实际raw bag→PCD exporter、topic/remap、字段名/NaN规则/压缩方式及PCD↔bag-record映射。源码`get_lidar`没有PCD export，不能冒充有。
2. 当次 `multi filter with intensity` 的原文件hash/启动命令/参数或可核查版本确认：I10–250还是独立tracker I20–250？是否另有z5/z10/ROI？注释velocity块有没有启用？无法确认则**完整流程继续BLOCKED**，只可另行标识“现存脚本子阶段验证”，不得称exact senior reproduction。
3. filteredPCD→ROS publisher、输出topic/frame_id、RViz配置、是否只做一次q变换；明确native/q/level三种坐标。缺失topic不能用`_81`填补。
4. 源码/参数/输入身份SHA-256、环境可用性、输出新目录、预期field schema、统计定义写入冻结protocol；在读anchor保留结果前锁定。若需修改parser/publisher才能跑，先单独提交可审阅的工程方案，不无声修补历史源码。

## 唯一允许的流程与参数边界

路径A（已知子阶段）：原始named字段→历史导出（待恢复）→q=[x,z,-y]→闭区间I10–250→原compressed `filtered_*.pcd`。**此路径没有高度/ROI/background门。** 轴变换是点数守恒操作。

路径B（完整历史流程）：只在找到其精确历史配置后登记额外空间/高度等阶段；本计划不从共存z5/z10/I20版本中挑一个。当前fine的ROI/closureheight/4cm可以用旧metadata说明差异，不能悄悄塞进路径A并宣称原版复现。

不扫描参数、不改E18、不跑新graph/DP、不根据保留结果开关阶段；原方法默认和实际历史CLI不同则优先原记录并同时记载差别。必须有可靠field-name parser，不按列号猜velocity/intensity；不把新的兼容读写器当历史算法。

## 逐阶段记录规范

对每包每帧记录native输入总行数、finite有效数、非零数；逐步 survivor、incremental removed、removed/N_previous、removed/N_valid_input。强度/ROI/height不存在时写NOT_APPLICABLE，不填0暗示执行。原始/解析无效值分别统计，不让字段压缩看起来像几何删点；q前后坐标范数及点身份对应核对。

若历史实际链包含空间门，分别记录实际顺序下ROI/height的增量删除；顺序重叠需明确，不能把联合keepcounts拆成各自贡献。当前fine元数据只有联合`roi_intensity_height_points`，不足以完成这项归因；**新的百分比仍UNKNOWN/NOT_RUN**。

低密集背景：先冻结传感器轴下全场景正交投影/固定网格count展示定义，报告全场景与各阶段点密度/计数，不以anchor定位背景、不用场景照片mask，也不把未标注点自动叫树/车辆/地面。若没有独立ground/altitude几何，只能称“低q_z区域的点量变化”，不能宣称正确清除了多少真实地面。

velocity：所有阶段核对named velocity数值/缺失/均值规则，最终只做同一固定色标可视化；不按速度删点、不把横飞近零径向速度判成静态。颜色可用固定预登记全输入百分位/原viewer规则，但不能看anchor后调色范围；记录截断与色标，不把色差当分类证据。

## 冻结之后才做的anchor描述检查

只复用已有anchor身份与既有1m/3m邻域，不增改/插值标签、不用GNSS中心代替、不根据结果改半径。现有anchor表已经是q分析坐标，原始点只转换一次到q，q-anchor保持不动，严禁再次轴变换；若显示native raw，则marker用逆映射[q_x,-q_z,q_y]。如果检查level mask则anchor只在诊断阶段用同一既有R/t映射，绝不反拟合。

逐anchor输出每阶段邻域计数/从有到无的发生阶段，标为“selected DEV_SILVER邻域support retention”；邻域杂波身份UNKNOWN，未标帧不算negative。保留删掉既有support、噪声未消失、velocity不能分离等不利结果。不能形成threshold sweep、detector recall或新score结果。

## 可视化与验收

每包保留同一帧native/q对照、q顶视/侧视、raw与final同视角/point size对照，intensity默认＋velocity辅助。必须记录一次且仅一次轴变换、frame basis、point field、bag-record/index；R/t高度mask并未使缓存q自动成为level坐标。RViz publisher配置未恢复前无确切可执行topic命令，保持BLOCKED，不给占位命令冒充复现。

验收仅是：原source/config/字段/时间身份明确；轴变换点数守恒；每步accounting闭合；PCD/ROS发布schema正确；正确轴显示可解释；anchor retention是冻结后的描述。**不以更干净、更多命中、某阈值 crossing 或79/80改善为验收。** 若证据缺失、需要试不同参数或须调calibration才能得到历史外观，停止并记录BLOCKED。

输出只能进新的临时/独立结果目录，不覆盖既有raw、filtered、labels、cache或历史图；不提交bag/临时ROS数据。当前任务仅创建此计划，没有任何新点云运行或截图结果。
