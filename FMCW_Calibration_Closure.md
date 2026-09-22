# FMCW-LiDAR Calibration Closure

## Material Passport

- **Artifact type:** Experiment result and calibration-closure audit
- **Run date:** 2026-09-22
- **Verification status:** Execution artifacts **VERIFIED**; scientific interpretation **ANALYZED / exploratory**
- **Inputs:** `bag/20260205/飞行轨迹坐标.xlsx` and the nine matched ZIP-contained bags
- **Raw-data policy:** bags and workbook opened read-only; current baseline not imported or modified
- **Primary code:** `scripts/calibration_closure.py`
- **Protocol:** `results/calibration_closure/protocol.json`
- **Main machine-readable outputs:** `candidate_summary.csv`, `lidar_trajectories.csv`, `alignment_results.csv`, `offset_profiles.csv`, `offset_diagnostics.csv`, `extrinsic_sensitivity.csv`, and `alignment.json` under `results/calibration_closure/`

## 1. 结论先行

### A. 哪些 bag 中存在可独立识别并能与 GNSS 闭合的 UAV trajectory？

**VERIFIED RESULT：五条横飞序列达到预先冻结的完整闭合门限。**

| 工况 | LiDAR-only 证据 | GNSS closure | 当前判定 |
|---|---|---|---|
| `100m_cross_5` | 两段高支持轨迹，约 40–41 点/观测，range 102–106 m | calibration PASS | 对应 UAV trajectory 有强证据 |
| `200m_cross_5` | 两段主要轨迹，约 5–13 点/观测，range 185–216 m | calibration PASS；offset 为边界解 | 对应 UAV trajectory 有强证据，时间仍不可靠 |
| `300m_cross_5` | 三段稀疏轨迹，约 4–8 点/观测，range 295–310 m | FAIL | 独立存在 target-like 轨迹，但尚不能确认与该 GNSS 轨迹对应 |
| `100m_cross_10` | 多段高支持轨迹，约 26.5–52 点/观测，range 105–118 m | held-out PASS | 对应 UAV trajectory 有强证据 |
| `200m_cross_10` | 两段高支持轨迹，约 19.5–22.5 点/观测，range 204–218 m | held-out PASS；offset 为边界解 | 对应 UAV trajectory 有强证据，时间仍不可靠 |
| `300m_cross_10` | 四段稀疏轨迹，约 3.5–5.5 点/观测，range 293–307 m | held-out PASS，但 offset 多峰 | 对应 UAV trajectory 有强证据，时间别名未消除 |
| `300m_roundtrip_10` | 两条长的去/回程段，range 约 73–303 m | 空间闭合，但 Doppler FAIL | likely UAV trajectory；不是完整 calibration closure |
| `300m_roundtrip_13` | 只有短、小覆盖率片段 | held-out FAIL | **insufficient evidence**；不能判成 no-return 或 wrong package |
| `300m_roundtrip_5` | 只有 8 帧左右的远端片段 | 无合格 calibration trajectory | **insufficient evidence**；不能判成 no-return 或 wrong package |

这里的“强证据”要求 LiDAR-only 候选先独立成立，然后共享外参在 GNSS 闭合中通过位置、range、方向、Doppler 和轨迹覆盖率门限。`300m_cross_5` 与 `300m_roundtrip_10` 仍只能称为 target-like/likely UAV，不能升级成已确认真值。

### B. 能否得到一套跨序列一致的 extrinsic 和可靠 time offset？

**空间上得到了一套有解释力但仍属 provisional 的 6-DoF 解；时间上没有得到九包都可靠的 offset。**

共享 6-DoF 拟合为：

```text
yaw         = 173.4498 deg
pitch       =  -6.5149 deg
roll        =   2.2361 deg
translation = [1.2056, -6.2698, 2.0066] m
```

相同候选上的 planar 模型为 `yaw=173.5379°`、`translation=[1.7634,-5.9050,-21.1439] m`。允许 roll/pitch 后，robust objective cost 从 283.27 降到 180.34（36.3%），且：

- `100m_cross_10` 留出 RMSE 从 12.51 m 降到 6.44 m；
- `300m_cross_10` 留出 RMSE 从 13.09 m 降到 5.58 m；
- `200m_cross_10` 变化很小，为 6.69 m 到 6.61 m。

因此 pitch 进入模型是数据支持的，不是为了视觉效果进行手调。其物理迹象也直接存在于 LiDAR-only 轨迹中：横飞目标的 LiDAR z 从约 100 m 处的 `+2 m`，下降到 200 m 处的 `−4～−7 m`，再到 300 m 处的 `−8～−11 m`。

但该外参尚不是 authoritative calibration：

- leave-one-condition-out 中 pitch 仅变化 `−6.62°…−6.48°`，较稳定；
- yaw 变化 `172.17°…178.42°`；
- translation-y 变化 `−16.34…−3.18 m`；
- calibration 内 `300m_cross_5` 不闭合，`300m_roundtrip_10` 的 Doppler 不闭合；
- 没有测量杆臂、共同硬件时间事件或 surveyed pose。

time offset 仍存在边界解、多个局部极小值和重复航段别名。不能把某一段轨迹的最佳 offset 当成整个包的可靠时钟标定。

### C. 是否足够升级为 Benchmark v1？

**否。当前结果不足以将九包整体升级为 Benchmark v1。**

建议立即停止把以下数据用于检测/跟踪算法的定量排名：

- `300m_roundtrip_5`、`300m_roundtrip_13`：无可独立确认的完整 UAV 轨迹；
- `300m_cross_5`：LiDAR 轨迹存在，但 GNSS closure 失败；
- `300m_roundtrip_10`：空间一致但 Doppler 不闭合，不适合速度评价。

五条 closure-PASS 横飞序列可以作为校准诊断和候选标注的起点，但在包级 offset 被共同事件或完整多航段联合约束以前，也不应作为最终论文 Benchmark v1 的独立测试集。尤其本轮虽有计算上的 calibration/validation 分离，但分析协议并非在查看所有数据之前前瞻预注册，因此这些 10 m/s 序列属于内部 held-out 验证，不是 untouched final test set。

## 2. LiDAR-only 全空间搜索

### 2.1 搜索范围与输入

搜索未使用 v0 的 5 m GT 邻域、工况标称距离或 GNSS 坐标。每个 PointCloud2 按仓库已有约定转换为 `[x,z,-y]`，排除 `(0,0,0)` 占位值，在统一空间：

```text
x:   0 .. 520 m
y: -250 .. 250 m
z: -100 .. 120 m
```

内搜索。九包的 86,611,167 个非零返回全部位于该范围，因此没有因该空间边界丢弃非零返回。807 帧被汇总为 6,300,361 个 1 m 体素记录。

### 2.2 候选生成

所有工况共用一套冻结参数：

1. 计算每个体素跨帧占用率；主搜索使用 10%，同时保留 5% 和 20% 敏感性结果。
2. 对低占用体素做 2.25 m DBSCAN 连通聚类。
3. 保留 extent≤6 m、≤120 点、≤32 体素的 compact components。
4. 用真实 bag record time 进行跨帧关联；速度、加速度、方向连续性和 `measured radial velocity ≈ −dr/dt` 同时约束物理合理性。
5. 将至少 5 个多点观测组成的连续段单独输出，避免把密集目标段与后续 singleton clutter 一起提升。

原始搜索共产生 5,410 条去重 tracklets；严格的多点证据分段剩 133 条。它们全部保留在机器可读文件中，不能把“存在 tracklet”自动解释为 UAV。

## 3. 时空联合对齐方法

### 3.1 防止 circular fitting

- LiDAR 候选生成完全不读取 GNSS。
- 所有 bag 使用相同候选参数，没有按 100/200/300 m 或速度单独调阈值。
- calibration split：`300m_roundtrip_5/10` 与三条 5 m/s 横飞；其中 `300m_roundtrip_5` 因无合格轨迹未进入拟合。
- validation split：`300m_roundtrip_13` 与三条 10 m/s 横飞。
- validation 使用 calibration 冻结的 shared extrinsic，只允许搜索该包的 time offset；不重新拟合空间外参。
- 对候选/offset 的综合评分同时包含 trajectory position/shape、range、motion direction 和 Doppler consistency。

限制：这是探索性内部留出，不是前瞻盲测。候选协议的开发看过这些 LiDAR 数据，不能把 held-out 结果宣传为最终泛化性能。

### 3.2 模型

先比较：

```text
planar:    p_lidar = Rz(yaw) p_enu + t
full 6DoF: p_lidar = Rz(yaw) Ry(pitch) Rx(roll) p_enu + t
```

每个包只有一个 offset 变量，共享一套空间外参。最终连续优化使用 soft-L1 位置残差和 Doppler 残差；离散候选/offset 选择与闭合评价同时使用位置、range、方向和 Doppler 四项。没有为每个序列拟合自由 extrinsic。

## 4. 闭合结果

冻结门限：position RMSE≤10 m、range RMSE≤5 m、方向中位误差≤35°、Doppler RMSE≤3 m/s、候选路径长度≥对应 GNSS 总路径的 15%。

| 工况 | split | candidate | offset (s) | trajectory RMSE (m) | range RMSE (m) | direction (deg) | Doppler RMSE (m/s) | closure |
|---|---|---|---:|---:|---:|---:|---:|---|
| 100m cross 5 | calibration | e0001 | 96.31 | 3.74 | 0.51 | 2.59 | 0.73 | PASS |
| 200m cross 5 | calibration | e0003 | 122.31 | 9.98 | 1.84 | 2.31 | 1.98 | PASS, boundary offset |
| 300m cross 5 | calibration | e0001 | 69.15 | 33.88 | 6.02 | 6.06 | 2.64 | FAIL |
| 300m roundtrip 10 | calibration | e0003 | 95.61 | 4.94 | 4.20 | 2.74 | 6.40 | FAIL (Doppler) |
| 300m roundtrip 5 | calibration | — | — | — | — | — | — | no eligible trajectory |
| 100m cross 10 | validation | e0003 | 96.58 | 6.44 | 0.69 | 2.44 | 1.54 | PASS |
| 200m cross 10 | validation | e0001 | 96.34 | 6.61 | 0.53 | 2.05 | 0.89 | PASS, boundary offset |
| 300m cross 10 | validation | e0003 | 129.76 | 5.58 | 1.41 | 1.72 | 2.08 | PASS, 3 local minima |
| 300m roundtrip 13 | validation | e0007 | 71.88 | 12.31 | 11.31 | 42.50 | 4.85 | FAIL, boundary offset |

`offset_profiles.csv` 记录完整 0.25 s 搜索曲线。通过序列也不等于 offset 已可靠：

- `100m_cross_5` 与 `100m_cross_10` 的近最优宽度均约 2.25 s；
- `200m_cross_5`、`200m_cross_10` 最佳解贴到允许区间边界；
- `300m_cross_10` 虽不是边界解，但存在 3 个内部局部极小值；
- 不同 target-like 航段可以与四点 GNSS 折线的不同航段对齐，产生时间别名。

## 5. `100m横飞10m/s` 复核

v0 的“严格 GNSS 邻域无返回”不是 sensor non-return 的可靠结论。本轮在完全不使用 GNSS 的搜索中发现：

- 高支持目标段位于约 `x=105–116 m`；
- 横向运动约覆盖 `y=−31…+19 m`；
- z 约为 `−3.7…+2.3 m`；
- 主要片段每观测中位点数约 26.5–52；
- 留出闭合轨迹 RMSE 6.44 m、range RMSE 0.69 m、方向误差 2.44°、Doppler RMSE 1.54 m/s。

因此该包中存在与 100 m 横飞 GNSS 形状相符的真实 LiDAR trajectory。v0 使用的 yaw-only、zero-translation 投影把 GNSS 高度放在约 7–13 m，而真实目标回波接近 z=2 m 并受约 −6.5° pitch 影响，5 m 三维邻域因而系统性错开。

**更新判定：**`100m横飞10m/s` 不是 likely no-return，也没有 package mismatch 的正证据；主要问题是 v0 空间模型缺少 pitch/roll/translation，加上 time-offset 航段别名。

## 6. 对 package/content 的判断

- 六条横飞包都在标称距离附近产生 compact、persistent、Doppler-consistent 的 LiDAR-only 轨迹。文件名与内容“完全错包”的解释因此明显弱化。
- 五条横飞进一步与共享外参下的 GNSS 闭合；`300m_cross_5` 的 LiDAR 目标存在，但其对应关系仍 unresolved。
- `300m_roundtrip_5/13` 没有足够轨迹证据。缺少证据不能升级为 likely no-return，也不能升级为 likely wrong package。
- 当前没有任何一包达到“有正证据表明 wrong package/content”的标准。

## 7. 论文与 Benchmark 使用边界

可以支持的结论：

1. 2026-02-05 横飞数据中确实存在此前 v0 空间投影漏掉的 compact UAV-like returns。
2. 共同 pitch 约 −6.5° 是跨距离、跨留出序列一致的主要空间修正。
3. 一套 6-DoF 空间变换能解释五条横飞轨迹，但不能解释所有九包的完整位置、Doppler 与时间。
4. `100m横飞10m/s` 不能再归类为 sensor non-return。

不能支持的结论：

1. 当前数值是 surveyed LiDAR↔GNSS 外参；
2. 每包 offset 已达到可靠硬件同步；
3. 九包构成 Benchmark v1；
4. 两条未闭合往返数据不含 UAV；
5. 当前候选搜索的 tracklet 数量等于 UAV 检测性能。

## 8. 可复现命令、环境和产物

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase cache
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase search
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase align
```

环境：Python 3.10.12、NumPy 2.2.6、SciPy 1.15.3、scikit-learn 1.7.2。

关键文件：

| 文件 | 用途 |
|---|---|
| `results/calibration_closure/protocol.json` | 冻结搜索、split 和 closure 门限 |
| `candidate_summary.csv` | 133 条多点证据轨迹摘要 |
| `lidar_trajectories.csv` | 每条证据轨迹的逐观测坐标和点特征 |
| `alignment_results.csv` | calibration/validation、planar/full 指标 |
| `offset_profiles.csv` | 全部 offset 网格诊断 |
| `offset_diagnostics.csv` | offset 边界、多峰和近最优宽度摘要 |
| `extrinsic_sensitivity.csv` | leave-one-condition-out 外参稳定性 |
| `alignment.json` | 完整候选选择、拟合和验证记录 |
| `plots/*_lidar_candidates.png` | 不使用 GNSS 的候选轨迹图 |
| `plots/*_alignment.png` | LiDAR 与变换后 GNSS 的位置/range/z 图 |

最终脚本 SHA-256：`111e59a6a66a8336d67d42ca6ea68c2988d1cd6b159bac1c98c50900ebd60c55`。

## 9. 最终决策

**Benchmark v1：NO-GO。**

空间 closure 已从 LOW-confidence yaw-only 推进到“跨五条横飞有支持的 provisional 6-DoF extrinsic”，但 time offset 和往返序列仍未闭合。升级前至少需要一个独立共同时间事件，或能证明同一包内多个 LiDAR 航段与完整 GNSS 轨迹采用同一 offset 的记录；随后应在未参与本轮协议开发的新序列上验证。
