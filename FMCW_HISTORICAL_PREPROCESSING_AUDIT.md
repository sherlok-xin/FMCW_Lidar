# FMCW 历史预处理流水线审计

日期：2026-10-16。状态：**源码子阶段 IMPLEMENTED；既有结果引用 VERIFIED RESULT；完整师兄流程复现 BLOCKED；本轮未运行点云处理。** 本文只审计源代码、Git 历史、配置与已保存报告/元数据，不读取原始 bag/PCD、不安装依赖、不调参数。E18、labels、calibration、offset、graphs、阈值、历史结果不变。STOP_DEV_FEATURE_MINING 继续生效。

## 1. 核心结论与证据等级

`get_lidar.py` 负责压缩字段；`multi_filter_with_intensity.py` 才负责轴变换与强度过滤。当前 E18/fine 输入包含这两项，但还包含后续研究加入的 ROI、provisional leveled-height mask 和4 cm体素表示，**既不是完整 raw，也不是原样的历史 filtered PCD**。不能用最近 `_81` repack 前后相同的视觉结果推断历史过滤不起作用。

[FMCW_Sparse_UAV_MVP.md](FMCW_Sparse_UAV_MVP.md) §2 已经记载三阶段链。本轮不是首次发现，而是重新核对其执行血缘、版本差异和解释边界。用户描述的静止车载、开放低背景、约半米 UAV 与速度着色流程，作为 **2026-10-16 用户提供的采集背景** 接受；不是新测得的安装角度、AGL 或逐点身份。

机器表：[pipeline_provenance.csv](results/project_reinterpretation_20261016/pipeline_provenance.csv)、[old_vs_current_preprocessing.csv](results/project_reinterpretation_20261016/old_vs_current_preprocessing.csv)。所有“历史参数”均指源码中的默认/active-main 值；没有当次运行配置，不保证师兄原运行恰用默认值。

## 2. 搜索范围与可建立的年代关系

本轮检索当前 Python/C++/头文件、README、shell、CMake/package、报告和 Git 全 refs 的相关路径/对象历史，排除环境/外部上游副本的同名噪声；没有把 bag/PCD 当代码搜索或执行输入。`git log --all -- get_lidar.py multi_filter_with_intensity.py` 仅返回初始导入 `0d03a07`（2026-09-22）。两文件及多种 tracker 在该次导入共存，未找到这两文件的历史修改版/删除重命名版。**导入时间不是编写时间，文件名演进不是可靠先后关系，师兄作者身份和原运行版本 UNKNOWN。** 不据目录名里的2025/2026日期给算法排年表。

可确定的后续关系来自 source dependency：closure → research_dev_v0 输入 addendum → fine cache/E16 → E18；当前 Git HEAD `9461d06f227746bf40b12b43be28625fa88ebb88`。旧 uncommitted Stone Soup 报告和无关用户文件保留。源码存在不等于已重现历史实际运行。

## 3. 已恢复的逐阶段实现

| 阶段 | 输入 → 操作 → 输出 | 精确值与移除对象 | 来源与参数解释 |
|---|---|---|---|
| 1 字段提取 | `/aqronos_cloud` PointCloud2 → 按字段名提取 xyz/intensity/velocity，float32 → `/aqronos_cloud_81` | `read_points(skip_nans=True)`；保留 native xyz/header；丢弃其余字段，非目标抑制 | `get_lidar.py:15–58,65–67`；存储/传输选择，无空间/强度/速度门限；无 PCD export |
| 2a 轴变换 | PCD named positions → `q=[x,z,-y]` | 没有移除点，没有拟合外参 | `multi_filter_with_intensity.py:23–28`；声明 x-forward/y-left/z-up；只是约定轴变换 |
| 2b 强度 | q/I/v → `10<=I<=250` → retained named fields | 删除区间外点，边界保留；velocity随mask同步保留 | 同文件 `:31–37,64–67`；默认历史数值，非已标定反射率阈值，物理推导 UNKNOWN |
| 2c 输出 | retained fields → Open3D tensor compressed binary PCD → `filtered_<basename>.pcd` | 无 z/height、ROI、体素、background、邻域或 velocity rejection | 同文件 `:44–53`；字段次序须读header，不能假设与raw相同 |
| 3 显示分支 | PCD → `visualize_pcd.py --mode velocity` / `single_filter.py` → Open3D | 速度着色，不等于速度筛除 | `visualize_pcd.py:116–136`；`single_filter.py:117–143`；**不是 RViz publisher** |

历史入口 `get_lidar.py` 是 ROS1 `rospy`，其硬编码订阅 `/aqronos_cloud`；本批原始 February topic 经既有 sanity 确认为 `/aqronos_cloud_0`，frame_id=`aqronos_lidar`。历史实际 remap/启动命令未完整恢复。不能悄悄把源码 topic 改成实包 topic，再称原版运行成功。最近 ROS2 临时 shim 是另一次 transport 诊断；不构成原版 ROS1 构建执行证据。

### 分歧版本，不能串成一条“最优历史链”

| 共存源码支路 | active 操作/差异 | 审计解释 |
|---|---|---|
| `single_filter.py` | `[x,z,-y]`、I10–250；两个 velocity rejection 调用块被注释；active velocity colors | 函数有参数不表示运行了速度门限 |
| `static_and_dynamic.py`、`static_and_dynamic_density.py`、`target_track.py` | main 中 z>5 | 后续低场景过滤的一个版本，不在 multi_filter 中 |
| `target_track_without_initialize.py`、`target_track_without_initialize_and_grid.py` | 两者filtered PCD后z>10；仅`_and_grid.py`另有background/neighborhood/grid，空间支持 x[0,500)、y[-150,150)、z[-5,50)，上界依 floor/index-valid 实现 | non-grid只有height/clustering/tracking；仅grid支路有occupancy/neighborhood；不是原PCD导出阶段ROI |
| `target_track_without_initialize_ros.py` | I10–250、第三坐标>10；未调用该轴变换 | raw-third-axis与q_z不可混用，历史实现有分歧 |
| grid ROS、GM-PHD/EKF ROS、`src/target_track_without_initialize_grid_ros.cpp` | `[x,z,-y]`、I20–250、q_z>10；部分另有grid/background | 不得替换PCD脚本I10–250；不是相同版本的无差别翻译 |
| 研究 benchmark_v0 | raw点级强度→analysis z>10→occupancy/27邻域/DBSCAN | 研究wrapper，不能反向当师兄完整原版 |
| closure→sparse/E13 coarse | 1m均值表示→mean-I及provisional leveled-height；另保留raw-height baseline分支 | 与benchmark_v0的过滤顺序/height frame不同，不得混作一条 |

z>5、z>10 和 grid 范围具有“避开低密集背景”的合理意图，但源码未证明这些**数值**来自测得安装高度/地平面/FOV/UAV飞行高度。归类为历史人工/默认常量，物理数值依据 UNKNOWN；当前 leveled placement 是后来研究修正，不能写成师兄测量校准。

### 独立grid tracker的后续过滤快照（不是batch脚本）

`target_track_without_initialize_and_grid.py:1030–1048,1167–1178` 的active main设定height10m、grid voxel1m、static_threshold0.05、init_frames8（旁注“10帧”与实参8不符，以实参为准）；函数默认voxel1.5/static0.1、类默认init10/prob0.3不能当实际main参数。`:155–194` 在前8次处理调用只建背景、不输出dynamic；之后用历史占用计数除以已递增的当前frame_count，保留有效grid且p<0.05的点，再做27cell邻域检查；`:204–287` 在已有占用邻域中static_ratio>0.6时删除候选。每次只对最终static mask的cell去重加计数，初始化用全部有效cell。它不是E18的past-only Beta(1,1)频率定义，含分母/更新策略差异；“dynamic”是代码命名，不是认证物体标签。

随后DBSCAN源码多处固定eps2m、minPts1（`:570–572,637–639,720–722`）；active tracker search_radius15m、radial_threshold5、max_association_distance=2×search_radius=30m，属于后续跟踪关联，不是PCD强度阶段的velocity rejection。数值为历史工程常量，物理推导/师兄当次采用UNKNOWN，不能用80anchors优化。既有benchmark wrapper又有自身输入/索引/aging等缺陷，仍不能把这些active参数当经过验证的检测基准。

### 显示与导出缺口

`visualize_pcd.py:19–45` 用固定5float/未压缩binary/固定字段顺序，还在header后额外 `f.read(1)`；不能直接当任意 compressed/reordered filtered PCD 的可靠读取器。`:121` 把第5列当velocity，已知 filtered PCD 可为 `x y z velocity intensity`，有混色风险。这是源级兼容性风险，**未新运行失败测试，也未修复脚本**。`single_filter.py` named-field 读取的velocity着色是另一支；`visualize_pcd.sh` 在raw目录循环 `pcl_viewer`。因此早期报告中可确认“速度着色的代码意图”，不能直接确认师兄 RViz 中每次颜色都正确。

没有恢复出完整的 raw bag→PCD 导出脚本/命令、确切数据身份/时间映射，以及 filtered PCD→PointCloud2 publisher、清洁云 topic、原 `.rviz` 配置。源码 `_81` 是 repack topic，**不能当清洁云topic**。用户的RViz流程记忆保留为采集/工程背景，未被这个检索缺口否定。

## 4. A–G 明确回答

A. 历史 PCD stage 是 `[native_x,native_y,native_z]→[native_x,native_z,-native_y]`，不是closure的6DoF。

B. intended vertical是第三轴 `q_z=-native_y`。车静止不等于传感器已水平，q_z不等于测得离地高度；provisional level z 又是另一坐标。

C. batch PCD active是闭区间 I10–250；另有ROS/C++ I20–250，原当次运行选择 UNKNOWN。

D. batch PCD没有高度/空域门；其它tracker有z5/z10/grid。师兄是否另外启用了哪一种空间过滤 UNKNOWN。

E. get_lidar移除NaN行并减字段；轴变换本身不删点；batch只按强度删点；后续高度/grid/background/邻域分支各自另删。未凭合并计数编造逐阶段百分比。

F. confirmed ROS repack `/aqronos_cloud_81`；confirmed文件 `filtered_*.pcd`；confirmed Python viewer为Open3D。**历史清洁RViz topic UNKNOWN。** 最近raw `_0` 与repack `_81` 的显示不能代替它。

G. batch保留velocity；single_filter active仅着色，速度拒绝调用注释。其它tracker使用Doppler作关联/一致性是另一阶段；E18本身未用Doppler。

## 5. 对应当前 fine/E18：相同部分少于完整等价

当前 `scripts/published_baseline_dp_mvp.py:266–285` 直接读bag字段，不通过`get_lidar`或filtered PCD；rich80包执行相同q轴变换，然后 finite XYZ/I/v、norm²>1e−8、ROI x[8,510]/y±200/z±60、点级I[10,250]、`(R.T@(q-t))_z>10`，再4cm均值体素。R/t是冻结closure `full_6dof`，**仅用于height mask，存储坐标仍是q**。该pose与银标共享closure来源，不是独立测绘地平面。

ROI来自research_dev_v0；10m数值来自旧tracker，但置于provisional level frame是后来[input addendum](results/research_dev_v0/input_protocol_addendum.json)。4cm和E16 candidate表示是研究选择。fine点级强度在体素前；旧1m控制的均值强度过滤在体素后，顺序也不同。

E18从E16保存的 `adaptive_minPts1` components读取全部pregeometry nodes（20,122nodes），不再按旧2–32voxels、120raw support、6m extent或accepted flag硬删；其背景来自同一个prefiltered fine cache的全部场景点，past-only1m计数。**pregeometry不等于preprocessing-free**，无法恢复上游已删点。whole-bag DP选路仍是离线任务，背景因果不等于整体在线。

所以：E18不是“原历史filtered cloud原样输入”，更不是“从完整raw找UAV”；它是经过研究适配的过滤/精细表示之后的候选图时间选择。已有结果已嵌入这些ROI、I、height和representation选择。不能把它们的收益全部记在E18路径score上。旧E18报告已明确披露；本次纠正高层口径，不修改既有数值。

## 6. 下一步复现状态

**完整历史链 BLOCKED。** 精确轴变换＋I10–250的源级子阶段可冻结，但不假称其等价全部师兄流程。先找原exporter、实际filter版本/配置和cleaned-RViz publisher/config；任一缺失保持UNKNOWN，不选一个看起来漂亮的z门/强度版本替代。

[单次受控计划](results/project_reinterpretation_20261016/proposed_historical_reproduction_plan.md) 固定100/200/300 m各一包，只计划逐阶段accounting与正确轴显示；本轮不执行。不利用anchors挑版本或阈值，不新增检测器、score或E21/E22。
