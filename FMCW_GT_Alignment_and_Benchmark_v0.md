# FMCW-LiDAR GT Alignment and Benchmark v0

日期：**2026-09-22**  
数据范围：`bag/20260205/飞行轨迹坐标.xlsx` 与 `bag/20260205/*.zip`  
结论状态：**PROVISIONAL / NOT CALIBRATION-VALIDATED**

## 1. 结论先行

本轮完成了九个工况的包级 manifest、GNSS 解析、四类时钟审计、无外推逐帧 GNSS 插值、原始点云真值邻域检查以及当前 baseline 的 Benchmark v0。机器可读结果为：

- [`gt_manifest.csv`](gt_manifest.csv)：9 行，一工况一包；
- [`frame_ground_truth.csv`](frame_ground_truth.csv)：807 行，一 LiDAR 帧一行，其中 680 帧位于 GNSS 覆盖区间；
- [`benchmark_v0.csv`](benchmark_v0.csv)：36 行，9 工况 × `measured/predicted/stale/all` 四类状态；
- `results/gt_benchmark_v0/`：对齐证据、baseline 状态、日志、缓存与 100 m 专项诊断。

但是，仓库中没有可验证的 LiDAR heading/extrinsic，九个包的真实高度候选也没有给出任何满足自动空间/多普勒一致性门限的 LiDAR–GNSS 对应点。因此：

1. ENU→LiDAR 航向只能由三条名为“往返”的 GNSS 轨迹主轴估计为 **173.2017°**，三序列间标准差 **0.8142°**；这不是测量外参。
2. 平移/杆臂为 **UNKNOWN**。输出暂以设备 ENU 原点与 LiDAR 原点重合，即 `[0,0,0]`，但没有把这一约定伪装成已标定事实。
3. 每包 GNSS–LiDAR offset 独立估计，没有硬编码 108 s；由于没有唯一事件或真实回波对应，最终采用区间中点配准，全部为 **LOW confidence**。
4. Benchmark v0 在 5 m 匹配门下得到零匹配，但只能解释为“当前低置信空间对齐下未匹配”，不能作为论文中的最终定位精度或传感器检出率结论。

因此本轮交付的是一个可审计的 **Benchmark v0 与标定失败证据**，而不是已经达到论文 headline 指标要求的最终 GT benchmark。

## 2. 工况—bag—topic—时间 manifest

Excel 的 9 个标题、9 个 ZIP 目录名以及各 ZIP 中唯一的 bag 文件名可以一一规范化匹配。每个 bag 只有一个 `sensor_msgs/PointCloud2` connection，topic 均为 `/aqronos_cloud_0`，点格式均为 16 字段、`point_step=80` 的 rich cloud。

| 工况 ID | LiDAR 帧 | bag record duration (s) | GNSS 点 | GNSS duration (s) | GNSS−LiDAR offset (s) | offset uncertainty (s) | GT 区间帧 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `300m_roundtrip_10` | 235 | 231.986 | 28 | 221.118 | 98.272 | 9.666 | 223 |
| `300m_roundtrip_13` | 145 | 142.994 | 12 | 140.481 | 99.590 | 7.437 | 141 |
| `300m_roundtrip_5` | 91 | 89.992 | 7 | 91.482 | 98.381 | 8.178 | 91 |
| `100m_cross_10` | 46 | 44.984 | 4 | 30.753 | 100.695 | 12.274 | 30 |
| `100m_cross_5` | 32 | 31.002 | 4 | 27.010 | 99.198 | 6.667 | 28 |
| `200m_cross_10` | 48 | 46.988 | 4 | 38.469 | 100.834 | 10.765 | 38 |
| `200m_cross_5` | 83 | 80.002 | 4 | 37.095 | 84.768 | 27.686 | 38 |
| `300m_cross_10` | 63 | 62.009 | 4 | 47.209 | 100.359 | 15.550 | 47 |
| `300m_cross_5` | 64 | 62.993 | 4 | 44.747 | 100.034 | 16.911 | 44 |

完整的 UTC 起止时间、header 时间、filename 时间、archive CRC、bag member 和不确定性字段见 `gt_manifest.csv`。

### 匹配可信度

- **HIGH（包级）**：Excel 工况标题与 ZIP/目录/bag 名的距离、运动类型、速度和采集顺序一致；每个 ZIP 仅含一个 bag。
- **UNKNOWN（内容级独立验证）**：没有相机事件、触发脉冲或其他外部记录能证明每个 bag 内容确为该次飞行。文件命名证据很强，但不能完全排除采集时误命名或数据内容错配。

## 3. GNSS 解析

Workbook 共解析出 71 个飞行点和 1 个“设备坐标”点。设备原点为：

```text
Lat  = 31.83749046 deg
Lon  = 118.76606266 deg
Ellh = 25.544 m
```

处理方法：

1. 只解释有明确标签的 `GPS week`、`time-of-week`、`Lat`、`Lon`、`Ellh`。
2. GPS week/TOW 按 GPS epoch 转 Unix UTC，并显式采用 2026-02-05 的 `GPS−UTC=18 s`。
3. 使用 WGS-84 ECEF→ENU 精确变换，以“设备坐标”为局部 ENU 原点。
4. Excel 中的 `N/E/V/Q` 及其前面的三个无标签小数原样保留为 opaque/raw 字段；没有找到字段文档，因此没有把它们解释成速度、质量或标准差。

## 4. 四类时间戳审计与同步

### 4.1 审计结果

- **bag record time**：位于 2026-02-05，连续性约 1 Hz，是 LiDAR 帧排序和本轮同步的主时钟。
- **filename time**：与 bag 首 record time 相差约 `-1.32` 至 `-2.33 s`，符合文件名按整秒记录开始时间的行为，但不作为逐帧时钟。
- **ROS `header.stamp`**：所有包绝对时间落在 2022-04-07，尽管帧间仍约 1 s 递增。它与 2026 bag/GNSS 绝对时间不一致，不能直接用于 UTC 同步。
- **GPS time**：位于 2026-02-05，但与 bag record time 存在约 85–101 s 的包级配准量。

### 4.2 offset 估计

对每个包先计算：

```text
offset_mid = midpoint(GNSS interval) - midpoint(LiDAR bag interval)
```

然后在 `offset_mid ± 30 s`、0.25 s 步长内，用每帧真实高度候选 cluster 与 GNSS 插值中心的 3D 距离和 Doppler/range-rate 一致性搜索。接受门为 8 m 与 5 m/s。九个包均为 0 个接受对应，因此没有用杂波强行修正 offset，最终保留各自的区间中点估计。

offset uncertainty 同时包含 LiDAR/GNSS 区间长度差与 GNSS 稀疏采样间隔。`200m_cross_5` 的不确定性达到 27.686 s，是因为 80.002 s 的 bag 只对应 37.095 s 的 GNSS 区间。

这些 offset 是每包独立估计，不是统一的约 108 s 常数，但由于没有共同事件，它们仍不是已验证 clock calibration。

## 5. 空间对齐

### 5.1 已有外参与航向搜索

仓库源码、配置、README、历史结果、bag 连接元数据和场景照片中没有找到数值化 LiDAR heading、GNSS-to-LiDAR translation、roll/pitch 或传感器杆臂。场景照片能确认三脚架设备布置，但不能提供可复现的测量航向。

### 5.2 自动几何估计

三条“300 m 往返”GNSS 轨迹分别做 2D PCA，并把从设备指向最远点的主轴映射到 LiDAR `+x`：

| 序列 | ENU 主轴角 | ENU→LiDAR yaw | PCA 线性度 |
|---|---:|---:|---:|
| 300 m 往返 10 m/s | -173.532° | 173.532° | 0.99094 |
| 300 m 往返 13 m/s | -173.799° | 173.799° | 0.99882 |
| 300 m 往返 5 m/s | -172.274° | 172.274° | 0.99966 |

圆均值为 **173.2017°**。该变换使横飞轨迹大致落在 `x≈100/200/300 m`、主要沿 y 变化，因而与工况命名相容。

但是，真实点云没有给出任何可用于唯一刚体优化的已接受 correspondence。特别是平移沿轨迹方向会与时间 offset 耦合，四点横飞序列又无法提供足够约束。因此没有执行人为坐标微调，输出中的平移保持 `[0,0,0]`，状态明确标为 UNKNOWN lever arm。

航向的 0.814° 序列间标准差在 300 m 处对应约 4.3 m 横向量级，尚未包含“往返是否严格等于雷达径向”这一系统误差。

## 6. 逐帧 GT 构建

- 只对 `bag_time + per-package offset` 落入 `[GNSS first, GNSS last]` 的帧做分段线性插值。
- 区间外 127 帧不外推，`gt_valid=false`，坐标留空。
- 680 帧有插值坐标。
- 六个横飞序列均只有 4 个 GNSS 点，`interpolation_confidence=LOW`。
- 7 点往返为 MEDIUM，12/28 点往返为 HIGH；这只描述插值采样密度，不代表空间外参置信度。
- 所有帧还单独记录 `time_offset_confidence` 与 `spatial_confidence`，避免把“可插值”混同为“已标定”。

## 7. GT 邻域中的真实点云返回

### 7.1 统计定义

每个有效 GT 中心使用 5 m 三维球邻域。组织化点云中的 `(0,0,0)` no-return 占位样本被排除。统计链为：

```text
nonzero raw return
  → intensity 10..250
  → z > 10 m
  → current occupancy background
  → current neighborhood filter
  → DBSCAN eps=2 m, min_samples=1
```

由于空间外参为 LOW confidence，这些是“GT 中心邻域返回”，不是经过人工点级确认的 UAV 点标签。

### 7.2 0/1/2/3–5/6–10/>10 点帧比例

| 工况 | 0 | 1 | 2 | 3–5 | 6–10 | >10 |
|---|---:|---:|---:|---:|---:|---:|
| 300 m 往返 10 | 94.2% | 0.4% | 1.3% | 1.3% | 0.0% | 2.7% |
| 300 m 往返 13 | 95.7% | 2.1% | 0.0% | 0.7% | 1.4% | 0.0% |
| 300 m 往返 5 | 93.4% | 0.0% | 0.0% | 4.4% | 1.1% | 1.1% |
| 所有横飞（225 帧） | 100.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |
| 所有往返（455 帧） | 94.5% | 0.9% | 0.7% | 1.8% | 0.7% | 1.5% |

### 7.3 逐阶段 survival

| 分组 | raw | intensity | height | background | neighborhood | cluster |
|---|---:|---:|---:|---:|---:|---:|
| 300 m 往返 10 | 1,538 | 1,528 | 0 | 0 | 0 | 0 |
| 300 m 往返 13 | 22 | 20 | 0 | 0 | 0 | 0 |
| 300 m 往返 5 | 993 | 989 | 0 | 0 | 0 | 0 |
| 所有往返 | 2,553 | 2,537 | 0 | 0 | 0 | 0 |
| 所有横飞 | 0 | 0 | 0 | 0 | 0 | 0 |

往返序列的非零邻域返回只出现在少量、通常靠近设备且低于当前 `z>10 m` 阈值的帧；跨这些非零帧，per-frame median 的中位数为 intensity 26、radial velocity -1.40 m/s、range 25.90 m。由于它们没有通过高度条件且空间标定不足，不能确认这些是 UAV 返回；更可能混有起降区地物/低空背景。

整个当前流水线在全部 807 帧上仍产生了一些非真值邻域候选：height/background/neighborhood 总数分别可在缓存中审计。例如 `200m_cross_5` 为 53,128/263/214 点，说明 tracker 的输入并非全空，但这些点没有位于当前 GT 邻域。

## 8. 当前 baseline 的 Benchmark v0

### 8.1 执行定义

没有修改 `target_track_without_initialize_and_grid.py`。评估脚本直接导入当前 `MultiTargetTracker`，使用默认研究基线参数：

```text
intensity 10..250; z>10 m; voxel 1 m; init 8 frames;
background probability 0.05; neighborhood filter;
DBSCAN eps=2 m, min_points=1;
search radius 15 m; radial threshold 5 m/s;
max association distance 30 m.
```

真值匹配使用 5 m 3D center gate。一个 GT 帧只匹配最近状态，其他状态计为 false detection。unique track ID 从未匹配任何 GT 时计为 false track。

状态按实际更新拆分：

- `measured`：当前帧有真实 cluster 更新；
- `predicted`：当前帧调用预测更新；
- `stale`：空候选帧触发现有 baseline 的提前返回，内部状态未更新也未老化；
- `all`：三者合集。

这种拆分避免把沿用上一帧 metadata 的 stale state 误报为 measured state。

### 8.2 `all` 状态结果

| 工况 | GT 帧 | 状态数 | recall | false detections | false tracks | best-ID completeness | max miss |
|---|---:|---:|---:|---:|---:|---:|---:|
| 300 m 往返 10 | 223 | 60 | 0 | 60 | 1 | 0 | 223 |
| 300 m 往返 13 | 141 | 0 | 0 | 0 | 0 | 0 | 141 |
| 300 m 往返 5 | 91 | 0 | 0 | 0 | 0 | 0 | 91 |
| 100 m 横飞 10 | 30 | 0 | 0 | 0 | 0 | 0 | 30 |
| 100 m 横飞 5 | 28 | 0 | 0 | 0 | 0 | 0 | 28 |
| 200 m 横飞 10 | 38 | 0 | 0 | 0 | 0 | 0 | 38 |
| 200 m 横飞 5 | 38 | 16 | 0 | 16 | 4 | 0 | 38 |
| 300 m 横飞 10 | 47 | 0 | 0 | 0 | 0 | 0 | 47 |
| 300 m 横飞 5 | 44 | 6 | 0 | 6 | 1 | 0 | 44 |

总计：680 个有效 GT 帧、82 个未匹配状态、6 个从未匹配 GT 的 track ID、0 个 matched state。

### 8.3 measured / predicted / stale

| subset | 有效 GT 区间状态数 | matched | false detections |
|---|---:|---:|---:|
| measured | 7 | 0 | 7 |
| predicted | 10 | 0 | 10 |
| stale | 65 | 0 | 65 |
| all | 82 | 0 | 82 |

因为没有匹配，center localization error、gap recovery 和 duplicate-on-target 均没有可计算样本：

- localization error：**UNKNOWN / empty**, 不是 0；
- gap recovery：没有“检测—缺测—恢复”事件，不能计算 recovery rate；
- duplicate track IDs：0 个匹配目标重复 ID，但这不表示 baseline 没有杂波轨迹；
- track completeness：全部为 0；
- consecutive miss：等于各序列有效 GT 帧数。

## 9. `100m横飞10m/s` 专项复核

### 9.1 包与时间

- ZIP 与 Excel 标题/速度/顺序一致，内部唯一 bag 为 `100m_v10m_s_2026-02-05-15-08-28.bag`。
- bag record 首帧为 15:08:29.335（UTC+8），文件名为 15:08:28，二者相差约 1.335 s。
- ROS header 错在 2022，不能用于绝对同步。
- 本包独立 offset 为 100.695 s，uncertainty 12.274 s，不再硬编码旧的约 108 s。

### 9.2 与航向无关的原始回波检查

为了排除 yaw 误差，另外只比较 GNSS 水平距离和相对高度，不使用估计航向：

- 30 个 GNSS 覆盖帧中，逐帧 `±5 m range` 且 `±5 m height` 的 azimuth-invariant shell 内为 **0 个真实返回**；
- 更宽的整段 envelope：horizontal range 90.137–132.069 m、height 3.899–15.786 m，46 帧总计只有 **1 个点**；
- 该点位于 frame 3，`(x,y,z)=(107.189, 6.035, 4.182) m`，intensity 8，radial velocity -2.39 m/s；
- intensity `[10,250]` 后为 **0 个点**；
- `±10 m` shell 共 1,500 点，但它允许明显偏离目标高度的返回，且与更严格 envelope 矛盾，不能作为 UAV 证据。
- 当前 baseline 没有初始化任何 track。

### 9.3 判定

**可排除的解释：**不是“一个清晰可见的 UAV 轨迹被 background/neighborhood/tracker 才删除”。在强度过滤之前，GNSS 距离—高度 envelope 已基本没有回波。

**仍无法二选一：**

1. sensor non-return / acquisition miss；
2. 尚未测量的 sensor tilt/translation/time event 导致 GNSS 空间落点错误；
3. 文件命名正确但内容在采集环节发生错配。

文件名、bag record time、采集顺序均支持“包匹配正确”，所以单纯文件名错配的证据较弱；但缺少外部同步事件和测量外参，不能科学地把最终原因唯一归为 sensor non-return。当前最严格表述是：

> `100m横飞10m/s` 的失败发生在当前 preprocessing 之前；sensor non-return 与未解决的采集/时空标定问题仍不可区分，数据包内容错配没有正证据但不能完全排除。

## 10. 主要限制与论文使用边界

1. 没有 surveyed heading/extrinsic；PCA yaw 利用了“往返=径向”这一工况语义。
2. translation/lever arm 为 UNKNOWN。
3. 时间没有共同硬件事件，区间中点 offset 全部 LOW confidence。
4. 横飞只有 4 个 GNSS 点，分段线性轨迹不能恢复转弯细节。
5. 5 m 邻域是评估定义，不是点级人工标签。
6. 当前零 recall 可能是真实 non-return，也可能受残余时空误差影响；不能用作最终论文 headline。
7. baseline 保留了现有空帧不老化、单点 cluster 和 `dt=1` 等行为；本轮按要求没有修算法。

在获得至少一个可见的同步标定目标、测量 LiDAR pose/lever arm 或可靠共同事件之前，本结果适合支持“数据与标定瓶颈”的论文实验设计，不适合支持传感器最终检出率或定位误差结论。

## 11. 可复现命令与快照

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase extract
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase align
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase raw-stats
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase benchmark
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase diagnose-100m
```

环境：Python 3.10.12、NumPy 2.2.6、SciPy 1.15.3、scikit-learn 1.7.2、Open3D 0.19.0。

| 文件 | SHA-256 |
|---|---|
| `scripts/build_gt_benchmark_v0.py` | `508b0264485109f75c754f05e070d4a8e1e1244569a4b9f9bd16f0c3605493d0` |
| `target_track_without_initialize_and_grid.py` | `5bd6b74bb19a532b3a66660328871ca4b7fbfc998595d134be365f9557c5643b` |
| workbook | `5adc1f090b84731b23a2c8cd02f38c002cafef94b9bdaaee98827981a8e54c69` |
| `gt_manifest.csv` | `ece744b77b89b0306c476c66b2c6689fb9b749b08c86bc12db98b934af39a4a6` |
| `frame_ground_truth.csv` | `4b52c3d69086e3a2329a2a8f2d7cccd18935ef77ee93f6125b5b893d8fbae9f7` |
| `benchmark_v0.csv` | `3f9e7fa700ca2c9340edf9132b9b91c60694ebf836c819235a0edb96f37920c6` |
