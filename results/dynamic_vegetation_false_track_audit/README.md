# Historical false-track / dynamic-vegetation forensic audit

日期：2026-10-16。[主报告](../../FMCW_DYNAMIC_VEGETATION_FALSE_TRACK_AUDIT.md)。只读冻结历史结果，未修改算法、阈值、E18、GNSS/calibration、labels或PCD；STOP_DEV_FEATURE_MINING有效；未commit/push。

| 文件 | 内容 |
|---|---|
|`track_visual_identity.csv`|18行手工RViz上下文判断：operator1、vegetation-associated4、duplicate3、UNKNOWN10；非独立GT|
|`track_lifecycle_summary.csv`|每ID first/last/span、OBS/PRED及比例、observed XYZ及XY bbox、累计/median/max XY端点位移、自然gap、结束原因、最终存活；另列all-state跨度|
|`birth_forensics.csv`|15个首批组件及3个pending births，逐ID点数/坐标/成员/时间证据|
|`pending_distinct_frame_audit.csv`|三个promotion的总observations、不同扫描及最近六条扫描序列|
|`pending_scan_history.csv`|只读source语义重建的逐扫描pending观察索引历史|
|`duplicate_association_events.csv`, `initial_seed_reuse.csv`|冻结实际点集复用证据|
|`representative_frames/`, `rviz_inspection_manifest.csv`|47幅实际RViz截图，另加18ID contact sheet；仅birth/实际observed点高亮，无预测物体|
|`lifecycle_descriptive.png/.pdf`|只做描述的OBS/PRED及扫描跨度图，无判别阈值|
|`candidate_order_audit.csv`, `support_membership.json`, `membership_checks.csv`|冻结候选成员/顺序、逐帧组件与pending核对|
|`protocol.json`, `frozen_input_identity.json`, `numeric_validation.json`|边界、数据/源码hash及核验|
|`instrumentation_notes.md`|source语义重建范围、日志/GUI工具局限与已纠正的问题|

## Reproduce read-only numerical bookkeeping

```bash
cd /home/xin/FMCW_LIDAR
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/dynamic_vegetation_false_track_audit/audit_frozen.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/dynamic_vegetation_false_track_audit/make_assets.py
```

前提：保留既有RViz工作流的60帧缓存；缓存缺失时本审计脚本会停止，不会隐式重跑滤波。需要恢复时，按已审阅的viewer流程另行恢复临时缓存。

不重跑filter/full tracker。audit_frozen.py只读冻结CSV和现有`/tmp/fmcw_historical_pcd_viewer/frames/`。原source pending代码块原样用于还原冻结剩余候选的append/晋升历史；每扫描pool及promotion输出必须与原记录一致。manual分类是显式写入的人工判断，不是自动分类规则；make_assets不读取特征决定类别，不拟合模型。初始source DBSCAN label→ID映射及其初始seed复用另有逐点证据。

`observed_fraction`分母包括移除当帧的冻结terminal state；它与旧active-only截图行数不是同一分母。初始seed不重复算observed association；五个零post-update OBS的ID仍有一个真实seed。晋升当帧按原CSV编码OBSERVED，截图明确标PENDING BIRTH。无观测的位移指标留空=UNKNOWN；单观测span0不表示静止。gap定义相邻observed scan差>1，所有缺失量可查，不是调出的检测门限。

## RViz review / screenshot reproduction

使用现有Humble/Open3D和软件渲染fallback，不安装依赖。请避免同时运行另一个发布同名`/historical/*`topics的player。

终端1：

```bash
cd /home/xin/FMCW_LIDAR
source tools/historical_pcd_viewer_env.sh
fmcw/bin/python results/dynamic_vegetation_false_track_audit/rviz_review.py
```

终端2：先启动普通RViz，后续截图脚本会加载仅在`/tmp/fmcwforensics/`生成的forensic view，不改项目默认配置。

```bash
cd /home/xin/FMCW_LIDAR
source tools/historical_pcd_viewer_env.sh
rviz2 -d rviz/historical_pcd_inspection.rviz
```

终端3（会切换本地RViz帧/相机取证；不适合同时手动拖相机）：

```bash
cd /home/xin/FMCW_LIDAR
source tools/historical_pcd_viewer_env.sh
fmcw/bin/python results/dynamic_vegetation_false_track_audit/capture_rviz.py
```

截图脚本重用已存在的图并重建完整manifest。要重新取证，可将本次新目录的图片另存后再生成；不可删原PCD/既有科学结果。manifest的camera字段是请求的GUI设置，科学坐标只取冻结数组；显示框不是算法ROI。原full-window PNG保持，contact sheet只裁掉工具面板并缩放排版。

手动查看例如ID2/frame10实际关联支持：

```bash
source tools/historical_pcd_viewer_env.sh
ros2 topic pub --once /historical_forensics/review std_msgs/msg/String '{data: "{\"track_id\":2,\"frame\":10,\"kind\":\"observed_association\"}"}'
```

kind须是support_membership.json内实际存在的birth_seed / pending_birth / observed_association；不接受prediction作为物体证据。场景为stored cloud；magenta是冻结所选实际点，不是新候选提取；关闭旧track路径和预测标记，避免把预测位置误当点云。ID1只承认用户提供的OPERATOR_VISUAL_IDENTITY，未认证其它ID的物理类别。
