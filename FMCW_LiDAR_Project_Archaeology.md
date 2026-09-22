# FMCW LiDAR 小型无人机探测项目考古与技术审计

> 审计日期：2026-09-18  
> 审计范围：`/home/xin/FMCW_LIDAR` 代码、已解压/归档数据、现有结果与轻量复现实验  
> 原则：未修改源代码、原始数据或既有结果；复现实验仅写入 `/tmp/fmcw_audit_benchmark_dual`。本文只诊断现状，不提出新模型或论文方法。

## 1. Executive Summary

本项目已形成一条可运行的、以规则和几何约束为核心的 FMCW LiDAR 动目标跟踪链路：ROS `PointCloud2` 被裁剪为 `x,y,z,intensity,velocity` 五个字段，经过坐标轴转换、强度过滤、高度过滤、在线体素背景建模和邻域过滤后，对剩余“动态点”做 DBSCAN/欧氏聚类，再以空间距离和径向速度一致性做数据关联，最后用多帧运动一致性管理轨迹。核心意图与 README 一致（`ReadMe.md:24-37`），实际离线主实现位于 `target_track_without_initialize_and_grid.py`，C++/ROS 版本位于 `src/`，另有 GM-PHD、EKF-GM-PHD 和 FMT/“Grassmann”实验分支。

项目当前最大的科学风险不是“轨迹模型不够复杂”，而是没有真值，因而无法知道动态候选和轨迹中哪些是无人机、哪些是杂波。现有五组 benchmark 只报告动态候选数、活动轨迹数、删除轨迹数、轨迹步长方差、启发式残差和运行时间；没有 Precision、Recall、F1、虚警率、定位误差、MOTA/MOTP、ID switch 或轨迹 RMSE。`scripts/benchmark_fmt.py:832-875` 甚至把“被删除的 tracker 数量”命名为 `fragmentations`，这并不等价于真实轨迹碎片化。因此，当前结果只能说明代码的内部行为，不能说明无人机检测/跟踪性能。

逐帧复用 benchmark 参数进行只读统计后，发现候选生成是最明显的损失点。强度过滤后的每帧点数约 1.4 万至 26.9 万，但 `z>10 m` 和背景差分后，每帧只剩极少动态候选：五组数据的全序列中位数分别为 0、2、6.5、16、22 点。100 m 序列在 8 帧背景初始化之后的 38 帧全部输出 0 个动态点；它不是“跟踪失败”，而是跟踪器根本没有收到候选。其他序列中，动态候选也只是背景过滤残留，不能直接称为 UAV 点。

原始 rich-format bag 的单点实际含 16 个字段，包括 `x,y,z,rgba,velocity,radius,theta,phi,intensity,ring,velocity0,time,velocity_direction,acceleration,acceleration_direction,surface_evenness`。然而 `get_lidar.py:15-56` 明确只保留五个字段，点级 `time`、球坐标、第二速度字段和其他派生属性均被丢弃。PCD 之后只剩 `x,y,z,intensity,velocity`。项目利用了 FMCW 径向速度，但主要是在轨迹关联门控中；背景差分和聚类不使用速度。其符号约定没有文档：主基线使用 `abs(-v_r-v_r^{geom})`（`target_track_without_initialize_and_grid.py:658-680`），benchmark FMT 却使用正号（`scripts/benchmark_fmt.py:556-584`），构成关键一致性问题。

时间信息的使用也有限。系统是“单帧候选生成后再跟踪”，不是 track-before-detect：背景模型使用历史占据次数，pending candidate 需要多帧确认，tracker 在局部缺测时预测，但弱点不会在检测前跨帧积累。代码把文件名时间戳截断到整数秒（例如 `target_track_without_initialize_and_grid.py:27-31`），运动方程多处固定 `dt=1`。原始 bag 的 record time 属于 2025/2026，而 `PointCloud2.header.stamp` 却停留在 2022；这种时钟不一致尚未处理。

2026-02-05 的 GNSS 表提供九种名义距离/速度工况的稀疏轨迹采样。把经纬高局部投影后，各段位置协方差第一主成分解释 94.1%–100.0% 的方差，说明这些采集航线基本是一维往返/横飞线段；这只是特定试飞几何的诊断证据，不是“轨迹流形”的证据。表格也没有 LiDAR 帧对齐关系、坐标外参或字段定义，不能作为当前评估真值。

排名最高的瓶颈是：① 无真值导致传感器漏检、预处理漏检和虚警不可分；② 候选生成极端脆弱，尤其背景初始化和高度/占据阈值；③ 径向速度符号、时间与坐标标定不完整；④ benchmark 与主实现不等价且指标语义不足；⑤ 数据资产丰富但缺少可追溯的序列清单、导出流程和固定评估划分。当前对“基线为什么失败”的理解只能评为 **Partially**，尚不足以负责任地选择研究方法。

## 2. Repository Architecture

### 2.1 科学相关目录树

```text
FMCW_LIDAR/
├── bag/                         # 约 115 GB：ROS bag、ZIP、场景图、GNSS 表
│   ├── 20260205/
│   └── DronesBag/
├── data/                        # 约 4.0 GB：原始/过滤后 PCD 序列
├── results/                     # 约 5.2 MB：5 组 benchmark JSON/帧图
├── get_lidar.py                 # ROS PointCloud2 字段裁剪/转发
├── multi_filter_with_intensity.py # 批量坐标变换、强度过滤、PCD 写出
├── single_filter.py             # 单文件版本
├── static_and_dynamic*.py       # 早期动静分离实验
├── target_track.py              # 已知/直接初始化的早期跟踪版本
├── target_track_without_initialize.py
├── target_track_without_initialize_and_grid.py # 当前离线规则基线
├── target_track_without_initialize*_ros.py     # Python ROS 版本
├── target_track_gmphd_ros.py / target_track_ekf_gmphd_ros.py
├── target_track_fmt.py          # FMT 后处理/可选动态滤波实验
├── src/manifold_modules.py      # “Grassmann”滤波与 FMT 平滑器
├── scripts/benchmark_fmt.py     # 当前已有结果的评估脚本
├── include/ + src/*.cpp         # C++/ROS 规则基线
├── CMakeLists.txt / package.xml # catkin 构建配置
└── requirements.txt             # 未锁版本的 Python 依赖
```

### 2.2 主要模块、输入输出与使用状态

| 模块 | 输入 | 输出 | 调用/使用状态 | 判断 |
|---|---|---|---|---|
| `get_lidar.py` | `/aqronos_cloud` rich `PointCloud2` | `/aqronos_cloud_81` 五字段云 | ROS 采集链前端；`get_lidar.py:65-67` | 有效但造成信息不可逆丢失 |
| `multi_filter_with_intensity.py` | PCD | 坐标变换并按强度过滤的 compressed PCD | 独立 CLI；默认写 300 m 目录（`:58-67`） | 当前 PCD 预处理来源 |
| `target_track_without_initialize_and_grid.py` | 过滤 PCD 序列 | 可视化/轨迹 | 独立 main，路径和参数硬编码（`:1163-1178`） | 最完整规则基线 |
| `scripts/benchmark_fmt.py` | 过滤 PCD 序列 | JSON + PNG | 生成 `results/*/comparison_study` | 当前结果来源，但不是主基线的精确副本 |
| `target_track_fmt.py` | 过滤 PCD | 跟踪 + 可选 FMT 后处理 | 默认 voxel + FMT smooth（`:1260-1271`） | 实验分支 |
| `src/manifold_modules.py` | 点/完整轨迹 | 动态残差、平滑轨迹、启发式分数 | 仅由 `target_track_fmt.py:14` 导入 | 实验性；未见标准评估 |
| `target_track_*gmphd_ros.py` | ROS 点云 | 在线 GM-PHD 轨迹/图像 | 无结果、日志或入口调用证据 | 实验性、未验证 |
| `include/`, `src/*.cpp` | ROS/PCL | ROS C++ tracker | `CMakeLists.txt:35-48` 编译目标 | 源码存在；`build/` 空、环境无 catkin/ROS，未构建验证 |
| `.vscode/tasks.json` | VS Code 任务 | 构建/运行 | 指向不存在的 `dynablox` 与 `dynablox_run`（`:7-30`） | 陈旧配置 |

未发现 notebook、模型 checkpoint、训练脚本、学习型模型、标注清单、数据划分文件或第三方代码声明。该项目本质上是规则式点云筛选与多目标跟踪工程，而非训练型 ML 项目。

### 2.3 实际执行路径

```text
ROS bag / PointCloud2
  → 字段裁剪为 x,y,z,intensity,velocity
  → PCD 导出（导出脚本/命令未保留）
  → 坐标 [x,y,z] → [x,z,-y]
  → intensity ∈ [10,250]
  → z > 10 m
  → 1 m 体素在线占据背景（前 8 帧初始化）
  → 27 邻域静态密度过滤
  → 动态点 DBSCAN（eps=2 m, minPts=1）
  → 空间近邻 + 径向速度一致性关联
  → 缺测预测、轨迹删除；主实现另含 6 帧 pending 确认
  → 可视化 / 内部代理指标
```

其中“PCD 导出”缺少可追溯脚本：`get_lidar.py` 只转发 ROS topic，不写 PCD；这解释不了 bag 到 `data/` 的完整映射。

## 3. Dataset and Sensor Representation

### 3.1 原始点结构

对四个已解压 ROS1 bag 做只读 schema 解码得到：

| 数据 | topic | 单帧点数/字段 | 记录概况 |
|---|---|---|---|
| `100m_v10m_s_2026-02-05-15-08-28.bag` | `/aqronos_cloud_0` | 1,250,000；16 字段 | 46 帧，44.984 s，约 1.02 Hz |
| `300m_v10_ms_2026-03-19-14-11-42.bag` | `/aqronos_cloud_81` | 1,250,000；5 字段 | 57 帧，bag record time 跨 207.2 s，间隔不规则 |
| `81-82-pm-x10_...15-38-07.bag` | `_81`,`_82` | 每 topic rich 16 字段 | 各 65 帧，64.149 s，约 1 Hz/传感器 |
| `81-82-pm-cross-x10_...15-57-13.bag` | `_81`,`_82` | 每 topic rich 16 字段 | 各 132 帧，131.115 s，约 1 Hz/传感器 |

rich 点的精确字段为：

```text
[x, y, z, rgba, velocity, radius, theta, phi, intensity, ring,
 velocity0, time, velocity_direction, acceleration,
 acceleration_direction, surface_evenness]
```

五字段 bag/PCD 为：

```text
[x, y, z, intensity, velocity]
```

### 3.2 属性可用性与当前使用

| Attribute | 原始可用 | PCD 可用 | 单位 | 当前算法使用 | 证据/限制 |
|---|---:|---:|---|---:|---|
| `x,y,z` | 是 | 是 | 推定 m | 是 | 单位未由传感器元数据直接声明；代码阈值以“米”注释 |
| `radius,theta,phi` | rich bag 有 | 否 | 未知 | 否 | 在字段裁剪时丢弃 |
| `intensity` | 是 | 是 | 未注明 | 预处理使用 | `[10,250]`，`multi_filter_with_intensity.py:31-37` |
| `velocity`（径向速度） | 是 | 是 | 推定 m/s | 关联使用 | 符号与模糊范围无文档 |
| `velocity0` | rich bag 有 | 否 | 未知 | 否 | 含义未说明 |
| 点级 `time` | rich bag 有 | 否 | 未知 | 否 | 采集前端丢弃 |
| 帧 timestamp | 是 | 文件名/头中有 | s | 仅排序；运动多按 1 s | 文件名小数被截断 |
| `rgba`,`ring` | rich bag 有 | 否 | — | 否 | 丢弃 |
| acceleration / direction / surface_evenness | rich bag 有 | 否 | 未知 | 否 | 名称可能是传感器派生量，语义未验证 |
| SNR / confidence | 未见 | 否 | — | 否 | intensity 不能在无说明时等同于 SNR |
| sensor pose / ego motion | 未见 | 否 | — | 否 | PCD `VIEWPOINT` 为单位位姿，无轨迹 |
| 外参/内参标定 | 未见 | — | — | 否 | 双雷达数据尤其缺关键外参 |

坐标在预处理中从注释所述的“`x` 前、`y` 下、`z` 左”变为“`x` 前、`y` 左、`z` 上”，实现为 `[x,z,-y]`（`multi_filter_with_intensity.py:23-28`）。但没有独立标定文件验证该约定。

### 3.3 时间、组织和标签

- PCD 文件通常以约 1 s 间隔命名；300 m 序列有一次约 2 s 缺口。代码通过 `split('.')[0]` 去掉小数（`scripts/benchmark_fmt.py:27`），损失亚秒精度。
- rich bag 的 record time 与目录名匹配 2025/2026，但消息 `header.stamp` 是 2022 epoch。哪个时钟是权威时间源尚不明确。
- `bag/20260205` 含 9 个命名工况 ZIP：100/200/300 m 横飞 5/10 m/s，以及 300 m 往返 5/10/13 m/s。`DronesBag` 还含多个 2025 年归档，文件名暗示不同无人机/频率，但没有 manifest；不能把文件名当作真值。
- 已处理 PCD 只有 5 组。原始/过滤帧数分别为：100 m `?/46`、旧序列 `?/343`、300 m `57/50`、双雷达命名序列 `30/28`、单雷达 cross `60/60`。缺帧原因未记录。
- 无 train/validation/test split，无点标签、框、中心、track ID、presence 或人工候选标注。GNSS 表不是已对齐的评估标签。

## 4. Dataset Statistics

### 4.1 过滤链统计

下表使用 `scripts/benchmark_fmt.py:904-950` 的参数和处理逻辑逐帧重放，但不运行 tracker。`dynamic` 是背景过滤残留，不是已标注 UAV 点。

| 序列 | 帧数 | 强度后点数/帧（均值） | `z>10`（均值） | dynamic 全帧均值 / 中位数 / 最大 | 初始化后 0 候选帧 |
|---|---:|---:|---:|---:|---:|
| 100m v10 | 46 | 111,512 | 631 | 0 / 0 / 0 | 38/38 = 100% |
| 2025-10-23 | 343 | 14,283 | 1,807 | 3.25 / 2 / 47 | 57/335 = 17.0% |
| 300m v10 | 50 | 113,262 | 5,949 | 7.28 / 6.5 / 27 | 1/42 = 2.4% |
| 81-82 pm x10 | 28 | 269,045 | 4,002 | 12.07 / 16 / 24 | 0/20 |
| 81 pm cross x10 | 60 | 253,674 | 7,567 | 20.08 / 22 / 36 | 0/52 |

对原始每帧 1,250,000 点的序列，强度过滤存活率约为 8.92%（100 m）、9.06%（300 m）、21.52%（81-82）和 20.29%（cross）。随后 `z>10` 仅保留强度后点数的 0.57%、5.25%、1.49% 和 2.98%。背景差分输出又仅占高度过滤点均值的 0%、0.12%、0.30% 和 0.27%。这些是流水线点数损失率，不能直接换算为 UAV recall。

对非零动态帧，现有 JSON 中动态点分布为：

| 序列 | 非零帧数 | mean / median / sd | min / Q1 / Q3 / max | 1 / 2 / 3–5 / 6–10 / >10 点帧 |
|---|---:|---|---|---|
| 2025-10-23 | 278 | 4.01 / 3 / 4.53 | 1 / 2 / 5 / 47 | 66 / 66 / 85 / 52 / 9 |
| 300m v10 | 41 | 8.88 / 8 / 5.60 | 1 / 5 / 12 / 27 | 2 / 1 / 9 / 18 / 11 |
| 81-82 pm x10 | 20 | 16.9 / 17 / 3.88 | 7 / 16 / 19 / 24 | 0 / 0 / 0 / 1 / 19 |
| 81 pm cross x10 | 52 | 23.17 / 23 / 6.43 | 8 / 19 / 28 / 36 | 0 / 0 / 0 / 1 / 51 |

这组数据只证明“算法送给 tracker 的观测很稀疏”；由于没有 UAV 点身份，题目要求的真实 `N_UAV(t)`、其与距离/姿态/机型/飞行方向的关系目前 **不可计算**。

### 4.2 速度统计

过滤 PCD 的所有点中，`|velocity|≤0.3` 的比例为 59.9%–88.6%；动态残留中相应比例为 15.7%–75.0%。300 m rich-like 序列的速度范围达到约 `[-50,50]`，双雷达命名序列达到约 `[-100,100]`，但这可能包含不同量程配置、异常值或速度模糊；没有传感器说明，不能判定。旧序列范围约 `[-15,15]`。跨数据的速度动态范围明显不一致，是必须先澄清的元数据问题。

### 4.3 GNSS 轨迹诊断

`bag/20260205/飞行轨迹坐标.xlsx` 的九段轨迹共 71 个记录，另有一行“设备坐标”。每段仅 4–28 个稀疏采样，包含 GPS week/time-of-week、经纬高和未定义的 N/E/V、质量字段；没有到 LiDAR 帧的映射。

以每段首点建立局部 ENU 近似后：300 m 往返 10/13/5 工况分别覆盖约 677/813/618 m 路径、持续 221/140/91 s；横飞段持续约 27–47 s。位置 PCA 第一主成分解释 94.1%–100.0% 方差，符合近直线往返/横飞采集设计。由稀疏相邻 GNSS 点得到的中位弦速度约 3.0–6.9 m/s，与文件名 5/10/13 m/s 并不一致；转弯、稀疏采样及“名义速度”语义都可能造成差异，所以不能拿文件名速度当逐帧真值。

## 5. Current Algorithm Pipeline

### 5.1 基线数学描述

1. 强度筛选：保留 `10 ≤ I ≤ 250`。
2. 高度筛选：当前 main/benchmark 使用 `z > 10 m`。
3. 体素背景：对体素 `q` 累计静态占据次数 `C_t(q)`。前 8 帧所有点都当静态；之后若

   `C_t(q) / t < 0.05`

   则先判为动态（`target_track_without_initialize_and_grid.py:162-192`）。再检查 3×3×3 邻域；静态邻居比例过高的动态点被去除。
4. 聚类：DBSCAN `eps=2 m`、`minPts=1`。因此每个孤点都可以成为 cluster，算法并未依赖真正的密度支持。
5. 已有轨迹关联：在预测中心 `2*search_radius=30 m` 内聚类；选择距离最小且

   `|-mean(v_r) - dot(Δp, p_hat)| < 5 m/s`

   的候选，并要求中心距离 `<30 m`（`target_track_without_initialize_and_grid.py:624-705`）。
6. 新轨迹：主实现对未用点再次聚类并形成 pending candidate；累计至少 6 次观测后，要求平均帧间位移 ≥1 m、速度变异系数 ≤0.5、平均相邻方向余弦 ≥0.5 才确认（`:743-795`）。
7. 删除：缺测超过 5 次，或长期近静止/轨迹稳定性不足时删除。空动态帧会直接返回，不增加 `lost_count`，所以“连续缺测”按被处理的非空帧而非真实时间计数。

### 5.2 伪代码

```text
for frame in sorted(PCD):
    P, I, vr, timestamp = load(frame)
    P, I, vr = keep(z > 10)

    if background_frame <= 8:
        update_background_with_all(P)
        continue

    dynamic = occupancy_probability(P) < 0.05
    dynamic = remove_if_static_neighbors_dominate(dynamic)
    update_background_with(non_dynamic)

    if dynamic is empty:
        continue                  # tracker 不老化

    for active_track:
        local_clusters = DBSCAN(points within 30 m, eps=2, minPts=1)
        gate clusters by distance and radial-velocity consistency
        update with nearest valid cluster; else predict and increment lost_count

    clusters = DBSCAN(unassigned points)
    match clusters to pending candidates
    confirm candidate after 6 observations and motion-consistency tests
    remove lost/static/unstable tracks
```

### 5.3 关键参数与敏感性风险

| 参数 | 当前值 | 位置 | 风险 |
|---|---:|---|---|
| intensity | 10–250 | `multi_filter_with_intensity.py:31-37` | 跨传感器/距离未校准 |
| height | 10 m | baseline `:1167`; benchmark `:932` | 直接决定是否看到低飞目标 |
| voxel size | 1 m | baseline `:1168`; benchmark `:905` | 稀疏点与背景混叠敏感 |
| background init | 8 帧 | benchmark `:906` | 运动目标在初始化期被写入背景 |
| occupancy threshold | 0.05 | benchmark `:905-906` | 100 m 工况全部候选被清空 |
| DBSCAN eps/minPts | 2 m / 1 | baseline `:637-639` | `minPts=1` 使单点成为目标候选 |
| search / association | 15 / 30 m | benchmark `:909-912` | 约 1 Hz 下门宽很大，易串轨/吸杂波 |
| radial residual | 5 m/s | 同上 | 符号、量程、dt 未校准 |
| pending frames | 6 | baseline `:559` | 对间歇稀疏回波敏感 |

未发现参数 sweep 或按序列验证记录。Python main、C++ 节点、ROS 分支之间默认值还不一致，例如 C++ 使用更窄的搜索/径向阈值，结果不可直接比较。

## 6. Current Experimental Results

### 6.1 已有 benchmark

| 数据集 | Baseline active / generated / removed | FMT active / generated / removed | smoothness B/F | FMT residual |
|---|---|---|---:|---:|
| 100m v10, 46 帧 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 | 0 |
| 2025-10-23, 343 帧 | 0 / 2 / 2 | 0 / 2 / 2 | 0 / 0 | 0 |
| 300m v10, 50 帧 | 0 / 8 / 8 | 3 / 8 / 5 | 0 / 50.0868 | 5.1896 m |
| 81-82 pm, 28 帧 | 2 / 6 / 4 | 2 / 6 / 4 | 36.4648 / 97.0921 | 20.6905 m |
| 81 cross, 60 帧 | 1 / 15 / 14 | 5 / 15 / 10 | 1.6023 / 140.4894 | 21.6190 m |

`smoothness` 是活动轨迹帧间步长的方差，越小才更平滑（`scripts/benchmark_fmt.py:837-849`）。现有两组数据中 FMT 数值反而显著更大。更多 active track 既可能是目标保存，也可能是虚警保存；在无真值时两种解释不可区分。

### 6.2 复现

执行命令：

```bash
PYTHONDONTWRITEBYTECODE=1 MPLBACKEND=Agg \
  fmcw/bin/python scripts/benchmark_fmt.py \
  --data data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data \
  --output /tmp/fmcw_audit_benchmark_dual
```

复现得到 Baseline/FMT 的 active、generated、removed、smoothness 和 residual 与已有 JSON 完全一致。平均延迟从既有 `1.37/1.40 ms` 变为 `1.53/2.04 ms`，属于未预热、单次墙钟计时的正常波动，说明延迟指标不是稳定 benchmark。运行使用 Python 3.10.12、Open3D 0.19.0、NumPy 2.2.6 等当前虚拟环境版本；`requirements.txt` 没有锁版本。

复现只验证“同一实现可产生相同内部状态”，没有验证检测正确性。benchmark 的 Baseline 在第一帧动态点上初始化所有 cluster，之后不创建新轨迹（`scripts/benchmark_fmt.py:411-495`），而主实现有 pending births，因此脚本开头声称的“exact copy”不成立。

## 7. Failure Atlas

| Failure category | Observable symptom | Likely pipeline stage | Approx. frequency | Representative evidence |
|---|---|---|---:|---|
| 候选完全消失 | 高度后每帧约 631 点，背景后为 0 | 背景初始化/占据与邻域过滤 | 100 m：38/38 后初始化帧 | 100 m JSON 无任何动态帧/轨迹 |
| 间歇无候选 | tracker 不收到输入，也不老化 | 背景差分 + 空帧控制流 | 旧序列 57/335；300 m 1/42 | `scripts/benchmark_fmt.py:949-950` |
| 单点即目标 | 极少动态点生成多条初始轨迹 | `minPts=1` 聚类/初始化 | 首个双雷达动态帧 7 点→6 tracks | 复现 frame 9 输出 |
| 轨迹大量删除 | generated 远大于 final active | 关联/track management 或初始虚警 | baseline：300 m 8/8 删除；cross 14/15 删除 | 五组 JSON |
| FMT 路径抖动/偏离 | FMT smoothness 高于 baseline，残差 20 m 量级 | FMT 运动约束/符号/dt | 双雷达与 cross 都出现 | frame 20/40 对比图、JSON |
| 运动目标污染背景 | 前 8 帧全部当静态 | 背景初始化 | 每序列必然发生，实际 UAV 影响未知 | baseline `:162-169` |
| 时间缺口不参与状态演化 | 空帧跳过；dt 通常固定 1 | temporal association | 频率见“间歇无候选” | benchmark `:949-962` |
| 双雷达来源/坐标混合不明 | bag 有两个 topic，但 PCD 仅 28–30 帧 | 数据导出/标定 | unknown | 无导出日志与外参 |
| 切向飞行时 Doppler 判别弱 | 真实速度大但 `v_r≈0` | radial gate | unknown — requires annotation | 数据命名含横飞工况，但无对齐标签 |
| 环境杂波成为轨迹 | 图中同帧存在多条长轨迹，但身份不可判 | 背景/聚类/关联 | unknown — requires annotation | `results/.../frame_0020.png`, `frame_0040.png` |

主导失败模式按现有证据排序：候选清空；单点/杂波触发大量 tracker；轨迹快速删除或错误保留；时间与速度约定不一致；双传感器及 GNSS 未标定对齐。传感器“完全没有返回”的频率无法从当前资产判断，因为没有原始点的 UAV 身份。

## 8. FMCW-Specific Information Usage

当前算法并未用 `|v_r|>τ` 直接删除点。径向速度只在聚类后取均值，并与由两帧中心位移投影得到的几何径向分量做门控；背景差分、邻域过滤和 DBSCAN 均忽略 `velocity`。因此：

- FMCW 速度没有帮助最关键的候选生成阶段；
- 横飞目标的 `v_r≈0` 不会被显式阈值删掉，但它与大量静态杂波更难区分，径向门控的判别力会下降；
- 主基线的负号暗示传感器正方向与几何外向速度相反，但无标定实验或文档；
- benchmark FMT 用 `v_r p_hat` 重构速度而没有负号（`scripts/benchmark_fmt.py:556-584`），与基线 `scripts/benchmark_fmt.py:458-466` 自相矛盾；
- 代码默认 `dt=1`，当帧率不是严格 1 Hz 或存在缺口时，m/frame 与 m/s 被混用；
- 速度范围在序列间为约 ±15、±50、±100，可能存在量程/包裹差异，但 velocity ambiguity/wrapping 完全未知。

rich bag 中的点级时间、`velocity0`、方向与其他字段在 `get_lidar.py:31-55` 被舍弃。它们是否是可靠的物理测量、怎样标定，当前无法判断；本审计只确认“存在字段”，不主张它们必然有用。

## 9. Temporal and Trajectory Information Usage

### 9.1 时间范式

当前是 **detect-then-track**：

```text
单帧高度/背景筛选 → 单帧聚类 → 多帧关联/确认 → 轨迹
```

不是 track-before-detect。背景占据虽跨帧，但目的是删除重复位置；它不会把多个帧的弱点联合起来形成检测证据。pending candidate 跨 6 次 cluster 观测确认，tracker 也能在非空帧中的局部失配后预测，但候选完全为空时状态不更新。无多帧点云配准、ego-motion compensation、时间窗联合聚类或原始回波级累积。

### 9.2 轨迹信息

主实现用中心位移的速度大小稳定性和方向一致性确认新轨迹，并用短期运动稳定性删除轨迹。`target_track_fmt.py` 默认只在跟踪结束后调用 FMT smoother；`src/manifold_modules.py:508-519` 明确把平滑结果另存，不反馈检测/关联。因此它不能恢复在背景或聚类阶段已经消失的 UAV 点。

`GrassmannDynamicFilter` 实际维护每体素点均值并用欧氏点到均值残差判断动态（`src/manifold_modules.py:55-130`）；代码未估计 Grassmann 子空间或流形距离。名称不应被当作已实现流形学习的证据。

GNSS 位置 PCA 表明特定采集航线高度受约束，但这是飞行任务设计造成的近直线结构。没有与 LiDAR 对齐的真实轨迹、没有跨环境/机型样本，也没有基于完整状态 `[p,v,a]` 的维数分析，所以对“UAV 轨迹占据低维流形”的结论仍是 **未验证假设**。

## 10. Code and Experimental Reliability Audit

### Critical

1. **没有真值与有效性能指标。** 所有 track 指标都可能同时奖励真目标和虚警；`fragmentations` 只是删除数量（`scripts/benchmark_fmt.py:860-875`）。这阻止任何性能结论。
2. **径向速度符号不一致。** baseline 使用负号，FMT 使用正号；会直接影响关联与残差解释。
3. **benchmark baseline 不等于主实现。** benchmark 没有后续 birth/pending 逻辑，无法代表 `target_track_without_initialize_and_grid.py`。
4. **时间轴不一致。** bag record time 为 2025/2026、消息头为 2022；文件名又被截断，算法固定 `dt=1`。速度/加速度量纲和同步可能失真。
5. **双传感器无外参和来源追踪。** 两 topic 的 bag 与单序列 PCD 之间缺少导出说明，无法判断是否融合、择一路或错误混合。

### Important

1. 前 8 帧所有运动点写入背景（`target_track_without_initialize_and_grid.py:162-169`），可造成目标吸收。
2. 空动态帧直接跳过（benchmark `:949-950`；tracker `:429-430`），轨迹寿命与真实时间脱钩。
3. benchmark 高度过滤后用 `intensities[:len(f_pts)]` 而非相同 mask（`:931-939`）。当前 intensity 下游未参与 tracker，暂未改变本次指标，但字段已错位。
4. 过滤后数据只保留 5/16 字段，点级时间等信息不可恢复；导出流程不可追溯。
5. `target_track_fmt.py:1184-1186` 连续调用 `process_trackers` 两次，冗余且可能造成未来状态副作用。
6. benchmark 会删除目标输出目录下已有 `frame_plots/*.png`（`:882-888`）；复现必须使用新目录。
7. 不同实现中阈值不同，且无配置文件、参数日志或 sweep；硬编码活动数据目录。

### Minor

1. benchmark 表头固定写“28 frames”（`:1004-1007`），其他序列展示错误。
2. 依赖不锁版本；当前虚拟环境可运行，但不可保证跨机器一致。
3. 项目不是 git 仓库，无法追溯代码/结果版本。
4. C++ `build/` 为空，当前环境无 ROS/catkin；C++ 路径未经本次执行验证。
5. `package.xml:7` 仍是占位维护者，VS Code 构建任务指向不存在的 dynablox。
6. 16 个 Python 文件均能通过 AST 语法解析，但除 benchmark 外没有测试。

### 事实、合理推测与未知

**Established from repository/data**

- PCD 点只有 `x,y,z,intensity,velocity`；rich bag 有 16 字段。
- 背景初始化 8 帧、体素 1 m、阈值 0.05、高度 10 m、DBSCAN `eps=2,minPts=1` 是现有 benchmark 配置。
- 100 m 序列后 38 帧没有任何动态候选。
- 现有结果没有真值相关指标；双雷达 28 帧 benchmark 可重复内部轨迹结果。
- 当前流水线是 detect-then-track，FMT 主脚本中的 smoother 是后处理。

**Plausible but unverified**

- 100 m 目标可能在初始化/邻域背景过滤中被吸收；也可能目标在 `z>10` 前就没有可靠回波。
- 横飞时近零径向速度可能降低杂波判别力。
- `minPts=1` 和 30 m 门宽可能产生大量杂波轨迹。
- FMT 的高残差/步长方差可能来自速度符号、固定 dt 或错误关联。
- GNSS 线性结构主要来自预设航线，而非普适 UAV 动力学低维性。

**Unknown**

- 每帧真实 UAV 点数、漏检与虚警、实际 UAV 数/类型及逐帧中心/ID。
- 传感器 velocity 的精确定义、正方向、量程、包裹/解模糊和噪声模型。
- bag→PCD 的导出脚本、丢帧原因、双雷达选择/融合方式。
- LiDAR–GNSS 时间偏移、坐标外参、设备坐标含义和传感器姿态。
- 强度随距离/角度的标定以及阈值选择依据。

## 11. Dominant Bottlenecks

### 1. 不可观测的检测质量

**Observation：** 无标签、无标准检测/跟踪指标。  
**Evidence：** `results/` 只有内部 tracker 统计；代码库无 annotation/ground-truth loader。  
**Mechanism：** 无法把“传感器没有返回”“算法删掉了真点”和“杂波被保留”分开。  
**Research question：** 在极稀疏 FMCW 点云中，最低成本但足以支持阶段级归因的真值应该包含什么？

### 2. 候选生成先于跟踪完全失败

**Observation：** 100 m 后初始化帧 100% 为零候选；其他序列候选常只有个位数。  
**Evidence：** 第 4 节逐帧统计。  
**Mechanism：** 高度、体素占据、初始化污染和邻域规则连续压缩点集，tracker 无法恢复已删除信息。  
**Research question：** 哪一预处理阶段在何种距离/姿态/速度下删除了真实 UAV 返回，哪些帧本来就是传感器 miss？

### 3. FMCW 速度与时空几何未标定

**Observation：** 速度符号不一致、范围跨序列不同、dt 固定、消息与 bag 时钟冲突。  
**Evidence：** baseline/FMT 正负号及 bag schema/时间审计。  
**Mechanism：** 错误的符号或时间尺度使正确目标无法通过门控，也会把残差解释成“模型不适配”。  
**Research question：** 在每个传感器配置下，径向速度与几何位移的定量一致性到底是多少？

### 4. 稀疏候选上的单点聚类与宽门关联

**Observation：** 7 个动态点可初始化 6 条轨迹；cross 基线 15 条中删除 14 条。  
**Evidence：** `minPts=1`、30 m 门宽和复现日志。  
**Mechanism：** 单点杂波可成为目标，宽门在低帧率下又允许远距离错误匹配；删除数混合了虚警清除和真轨断裂。  
**Research question：** 在标注后，候选点数、空间门、Doppler 残差与真假关联概率之间是什么关系？

### 5. 数据资产与实验链不可追溯

**Observation：** 115 GB bag 只有少量 PCD/结果；无导出 manifest、划分、配置快照或版本控制。  
**Evidence：** 第 2–3 节目录审计。  
**Mechanism：** 数据选择和隐含手工步骤可能比算法变化更影响结果。  
**Research question：** 哪些采集条件已覆盖，哪些结论能跨序列/传感器重复？

## 12. Scientific Questions Emerging from the Audit

1. 当单帧只有 0–5 个真实 UAV 返回时，现有预处理各阶段的条件召回率分别是多少？
2. 如何实验地区分“无物理返回”与“有返回但被强度/高度/背景规则删除”？
3. 径向速度的符号、噪声和可能的模糊如何随设备、距离与方位变化？
4. 横飞、径向飞行和转弯时，`v_r` 对 UAV/杂波区分的实际信息量有多大？
5. 现有体素背景模型在目标经过初始化窗口、重复航线和植被运动时分别产生多少漏检/虚警？
6. 单点 cluster 在多少帧中对应真实目标，多少对应噪声；`minPts=1` 是否是必要妥协还是主要虚警源？
7. 空观测帧和不规则帧间隔对轨迹寿命、速度估计和 ID 连续性造成多大误差？
8. 特定直线航线上的低维 PCA 结构是否在不同航线、机型和环境中仍成立？

## 13. Missing Information

| 缺失项 | 为什么重要 | 如何获得 | 是否阻塞方法选择 |
|---|---|---|---:|
| 逐帧 UAV presence/中心/ID，最好含点级归属 | 计算 recall、虚警、阶段损失 | 对代表序列人工标注并双人复核 | 是 |
| bag→PCD manifest 与脚本 | 确认丢帧、topic、字段和手工步骤 | 恢复采集/导出命令，记录 hash/帧映射 | 是 |
| LiDAR velocity 说明书/标定 | 统一符号、单位、量程和解模糊 | 静止靶、已知径向运动靶对照 | 是 |
| LiDAR–GNSS 时间同步与外参 | 将轨迹变成可用真值 | 时间偏移估计、坐标标定、设备位姿记录 | 是 |
| 双雷达外参和 PCD 来源 | 判断是否融合及几何合法性 | topic→文件映射、外参标定 | 是（涉及双雷达结论） |
| UAV 类型、尺寸、姿态、真实速度日志 | 分析距离/姿态/机型效应 | 建立采集 manifest，关联飞控日志 | 重要 |
| 环境/天气/传感器配置元数据 | 判断域变化与速度范围差异 | 从采集日志补录 | 重要 |
| 固定数据划分与评估协议 | 防止挑序列和参数泄漏 | 按日期/环境/机型冻结划分 | 是（涉及比较） |
| 参数选择记录与 sweep | 判断阈值脆弱性 | 小规模受控敏感性测试 | 重要 |

## 14. Recommended Next Diagnostic Experiments

以下均是诊断，不是新方法：

1. **建立最小标注集：** 从 100 m、300 m、单/双雷达各选连续片段，标注 presence、3D 中心、ID，并标出可确认的 UAV 点。
2. **阶段损失账本：** 对每个真值点/中心记录 raw→intensity→height→background→neighborhood→cluster→association 的存活状态，输出条件召回率。
3. **速度符号校准：** 用 GNSS 对齐后的径向位移与点 `velocity` 做散点图和符号/尺度回归，分别按传感器与序列报告。
4. **100 m 零候选剖析：** 可视化前 8 帧占据体素及后续 UAV 真值位置，判断是初始化污染、邻域过滤还是原始 miss。
5. **候选质量曲线：** 在标注集上画真假 cluster 比例随点数、range、intensity、`|v_r|` 和占据概率变化的曲线。
6. **时间完整性审计：** 对 bag record time、header stamp、PCD filename 和 GNSS time 建立逐帧对照，量化 offset、jitter 与缺帧。
7. **tracker 事件日志：** 不改变算法，只记录每次 birth/match/predict/delete 的候选 ID、门控残差和原因，以统计真轨断裂与虚轨寿命。
8. **小规模参数敏感性：** 仅在固定标注片段上逐一改变 height、voxel、occupancy、eps、minPts、search/radial gate，避免联合大搜索，观察阶段 recall/false alarms。
9. **主实现与 benchmark 一致性测试：** 在同一动态候选流上比较两者 births、IDs 和删除事件，明确 benchmark 能否代表主系统。

## 15. Research Readiness Assessment

### A. Do we currently understand why the baseline fails?

**Partially。** 已确定至少一种明确失败：100 m 序列在候选生成阶段被完全清空；也确认了单点初始化、大量删除、空帧不老化、符号/dt 不一致等机制。但无真值，无法给这些机制分配真实漏检/虚警占比，也无法排除传感器本身无回波。

### B. Do we currently have enough information to start literature-driven idea generation?

**Partially, but not enough to select or justify a method。** 可以围绕“极稀疏、带 Doppler、间歇观测”的问题做背景文献梳理，但现在不应据此选择 Grassmann、深度网络、TBD 或其他具体方案。缺失的标注与传感器标定会让方法选择被错误现象驱动。

### C. 三个最重要的缺失证据

1. 代表性连续片段上的逐帧/逐点 UAV 真值及 track ID。
2. 径向速度的物理定义、符号、量程/包裹与同 GNSS 位移的实测一致性。
3. bag→PCD→结果的完整帧映射，以及 LiDAR/GNSS/双雷达的时间和空间标定。

### D. 提出研究方法前下一步应调查什么？

优先完成第 14 节的前三项：最小标注集、阶段损失账本和 Doppler 符号/尺度校准。只有得到“真实 UAV 返回在哪一阶段、因何条件丢失”的统计后，才有足够证据判断瓶颈主要来自传感器、单帧候选生成、时序关联还是轨迹约束，并进入有针对性的文献与方案阶段。

