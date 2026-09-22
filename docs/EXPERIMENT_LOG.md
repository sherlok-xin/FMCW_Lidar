# Experiment Log

Reconstruction date: **2026-09-20**

This log records experiments that can be inferred from repository artifacts. It does not claim that every historical run is present. No conventional run logs, notebooks, Git commits, or experiment registry were found.

## Provenance rules for recovered experiments

- Artifact modification times are approximate evidence of when outputs were written, not guaranteed execution timestamps.
- At the 2026-09-20 reconstruction the repository was not under Git. Git is now initialized on `main`; E11's tested script/tests are in `e62848f787dcfd1bb2f91536ca23e6a50580a77b`. SHA-256 remains the authoritative identity for older artifacts and dirty working-tree runs.
- Pseudo-label results are diagnostics, not synchronized ground truth.
- Internal tracker metrics are not UAV accuracy metrics.

### Current script snapshot identities

| Script | SHA-256 |
|---|---|
| `scripts/benchmark_fmt.py` | `9d69d7197790f7fc61028ed3005a384808a35ecc48e4c8a8677758269ec81cab` |
| `scripts/data_archaeology.py` | `c9fc368abe156f0e79172b6f1c12717f340a36afbbcbc287c2a0a72a685e2f24` |
| `scripts/uav_pseudo_label.py` | `734c041d371321643a40544712e065db8228b47d0f3b60e8c706b25ac2984610` |
| `scripts/compile_uav_evidence.py` | `b2739a583cc6867e06748b41377a2ae335c9861a0daf46c04954b6cc26491512` |
| `scripts/scan_100m_expected_band.py` | `27b48571eebd31d6f412ee8c531e5d44493b295e5bb79be7d971c1c091febe71` |
| `scripts/reverse_engineer_baseline_uav.py` | `33d6d86f0bcd6a287728ed196caa2b8fe2875755e63a5527c13a116825a708de` |
| `scripts/reverse_engineer_longitudinal_baseline.py` | `4c8b1cfe5037a836010fc9dc8d9b6c3e56f402ad9645a4f9ecbf54191ee3e3e7` |
| `src/manifold_modules.py` | `26870d5fc14fa6877da611ba5a62ac04170522c62c6a591f7e55aeb8875b41e1` |
| `target_track_without_initialize_and_grid.py` | `5bd6b74bb19a532b3a66660328871ca4b7fbfc998595d134be365f9557c5643b` |
| `scripts/build_gt_benchmark_v0.py` | `508b0264485109f75c754f05e070d4a8e1e1244569a4b9f9bd16f0c3605493d0` |
| `scripts/calibration_closure.py` | `111e59a6a66a8336d67d42ca6ea68c2988d1cd6b159bac1c98c50900ebd60c55` |
| `scripts/sparse_uav_mvp.py` | `2ff01d1b0e3906968b99886c4c793fe3e84c5628d78c3a99e3f9cf0a2bd3d3dc` |

## E11 — Frozen sparse-UAV DEV benchmark and Doppler ablation

**Run date:** 2026-09-22

**Objective:** Freeze closure-confirmed cross-flight LiDAR trajectories as development-only silver labels, audit preprocessing survival, reproduce a Khosravi-style temporal detector, and isolate Doppler value.

**Status:** **TESTED / VERIFIED RESULT on DEV silver labels; not final paper GT.**

**Code/version:** `scripts/sparse_uav_mvp.py`, hash above.

**Git snapshot:** `e62848f787dcfd1bb2f91536ca23e6a50580a77b` contains the tested script/tests; this experiment-log/report update remained uncommitted at handoff.

**Inputs:** five closure-passing cross sequences, closure evidence trajectories, raw bags, and existing raw-voxel/current-stage caches.
**Outputs:** `FMCW_Sparse_UAV_MVP.md` and `results/research_dev_v0/`.

### Commands and environment

```bash
python3 scripts/sparse_uav_mvp.py freeze
python3 scripts/sparse_uav_mvp.py audit
python3 scripts/sparse_uav_mvp.py benchmark
python3 -m pytest -q tests/test_sparse_uav_mvp.py
```

Python 3.10; NumPy 2.2.6; SciPy 1.15.3; scikit-learn 1.7.2. Three targeted tests passed.

### Frozen data and configuration

- 80 observed, non-interpolated LiDAR evidence frames in five 100/200/300 m cross-flight sequences.
- Labels SHA-256 `37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db`.
- Protocol SHA-256 `b8575fc94efa1b33624a0ac92f1e464c2e58e1955817ff125c7154d176490b67`.
- Input addendum SHA-256 `2edf3d49dae537fe4b8df1ea8065944bcf0941c4b186a090d9cf09fdefb0f9ee`.
- The addendum applies the unchanged 10 m height threshold after provisional 6-DoF gravity alignment. The uncorrected pilot is preserved.

### Results

- Intensity retained 1,877/1,883 target-neighborhood raw points. Raw LiDAR `z>10` retained 0/1,877; leveled `z>10` retained 1,877/1,877.
- Current baseline detected 0/80 silver frames.
- No-Doppler temporal baseline detected 41/80 (51.25%) with 1,914 false candidates on labeled positive frames and 2.10% candidate precision.
- Fixed and observability-aware Doppler each detected 40/80. Fixed Doppler had 1,817 false candidates; aware Doppler had 1,840. No independent recall gain was observed.
- Removing temporal consistency increased recall to 61.25% with essentially unchanged 2.10% candidate precision; temporal consistency itself was not shown to improve the detector.
- At 300 m, minPts 1/2/3 recall was 0.90/0.30/0.00; target support had median 5 raw points and 1.5 occupied 1 m voxels.

### Interpretation and decision

The original height filter is in the wrong coordinate frame for the February data. The Khosravi-style pipeline improves recall over the current baseline, but remains far too cluttered for an operational detector and does not establish a temporal-consistency benefit. Doppler provides no verified independent gain in this cross-flight-only DEV set. A controlled TBD study is justified at 300 m after independent labels and negative scenes exist.

### Limitations

- Labels are LiDAR-derived and selection-conditioned; localization error is self-consistency, not independent accuracy.
- Precision is measured only on labeled positive frames because negative-scene labels are absent.
- The gravity transform is provisional and shares calibration-closure provenance with label selection.
- This is a project adaptation, not an exact parameter reproduction of Khosravi et al. 2026.

## E10 — Independent LiDAR trajectory search and calibration closure

**Run date:** 2026-09-22  
**Objective:** Search all nine raw bags for compact, persistent moving trajectories without using GNSS neighborhoods, then test one shared LiDAR↔GNSS extrinsic with per-package time offsets on a calibration/validation split.  
**Status:** **TESTED / VERIFIED RESULT for the recorded execution; exploratory rather than prospectively blinded.**  
**Code/version:** `scripts/calibration_closure.py`, hash above.  
**Inputs:** the nine ZIP-contained bags and `bag/20260205/飞行轨迹坐标.xlsx`.  
**Outputs:** `FMCW_Calibration_Closure.md` and machine-readable/plot artifacts under `results/calibration_closure/`.

### Commands

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase cache
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase search
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/calibration_closure.py --phase align
```

### Results

- All 86,611,167 nonzero returns in 807 frames were searched; none fell outside the common broad search volume. They were reduced to 6,300,361 auditable 1 m voxel records.
- LiDAR-only search produced 5,410 deduplicated tracklets and 133 stricter multi-point evidence segments. Track existence alone was not treated as UAV evidence.
- Five cross-flight conditions passed the frozen closure gates: `100m_cross_5`, `200m_cross_5`, and held-out `100m_cross_10`, `200m_cross_10`, `300m_cross_10`.
- The shared full-pose estimate was yaw 173.4498°, pitch -6.5149°, roll 2.2361°, translation `[1.2056,-6.2698,2.0066] m`.
- Held-out cross-flight trajectory RMSE was 6.44, 6.61, and 5.58 m at nominal 100/200/300 m. Held-out `300m_roundtrip_13` failed all closure dimensions.
- `300m_cross_5` contained an independent target-like LiDAR trajectory but failed GNSS closure. `300m_roundtrip_10` closed spatially but failed Doppler consistency. `300m_roundtrip_5/13` remained insufficient evidence.
- The `100m_cross_10` bag contains a strong raw LiDAR trajectory around x=105–116 m; the old v0 zero-neighborhood result was caused by the provisional yaw-only/zero-translation projection, not sensor non-return.
- Time offsets remained ambiguous: two conditions hit allowed boundaries and `300m_cross_10` had three internal minima. A reliable nine-package clock calibration was not obtained.

### Interpretation

Roll/pitch are required by the data, especially the approximately -6.5° pitch implied across ranges. The spatial estimate is a useful provisional cross-sequence solution, not a surveyed extrinsic. The nine packages do not pass a common calibration closure, so Benchmark v1 remains a no-go.

### Limitations

- Candidate generation did not use GNSS, and validation froze the calibration extrinsic, but the protocol was developed while inspecting these LiDAR data; this is not an untouched final test.
- Four-point cross-flight GNSS trajectories and repeated/fragmented LiDAR passes create time-offset aliases.
- Leave-one-condition-out yaw spans 172.17°–178.42° and translation-y spans -16.34 to -3.18 m.
- No common hardware event or surveyed pose exists.

## E09 — 2026-02-05 GNSS alignment attempt and Benchmark v0

**Run date:** 2026-09-21 to 2026-09-22  
**Objective:** Match all nine workbook conditions to raw bags, audit clocks, build non-extrapolated per-frame GNSS centers, measure stage survival around those centers, and score the unchanged current baseline.  
**Ground-truth status:** **PROVISIONAL / LOW spatial and temporal confidence**; no surveyed extrinsic or common time event.  
**Code/version:** `scripts/build_gt_benchmark_v0.py`, hash above; unchanged baseline hash above.  
**Inputs:** `bag/20260205/飞行轨迹坐标.xlsx` plus all nine `bag/20260205/*.zip` archives.  
**Outputs:** repository-root `gt_manifest.csv`, `frame_ground_truth.csv`, `benchmark_v0.csv`, `FMCW_GT_Alignment_and_Benchmark_v0.md`; supporting artifacts under `results/gt_benchmark_v0/`.

### Commands

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase extract
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase align
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase raw-stats
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase benchmark
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/build_gt_benchmark_v0.py --phase diagnose-100m
```

### Configuration and definitions

- WGS-84 ECEF→ENU, device-coordinate row as ENU origin.
- GPS−UTC leap offset: 18 s.
- Per-package interval-midpoint time registration, followed by ±30 s/0.25 s automatic height-cluster search.
- ENU→LiDAR yaw from the three named roundtrip GNSS PCA axes; no hand tuning.
- GT point neighborhood: 5 m 3D sphere; `(0,0,0)` organized-cloud placeholders excluded.
- Baseline match gate: 5 m 3D center distance.
- Baseline parameters unchanged: intensity 10..250, z>10 m, 1 m voxel, 8 initialization frames, probability 0.05, DBSCAN eps 2/min points 1, 15 m search radius, 5 m/s radial gate, 30 m maximum association distance.

### Results

- Nine workbook conditions matched one-to-one to nine ZIPs and their unique bag members; all use `/aqronos_cloud_0` rich PointCloud2.
- 807 LiDAR frames were decoded; 680 lie inside their shifted GNSS intervals. No out-of-range extrapolation was performed.
- Filename time is within about 1.3–2.3 s of first bag record time. PointCloud2 headers are in 2022 and cannot supply 2026 absolute time.
- Per-package estimated offsets span 84.768–100.834 s; all remain LOW confidence because zero accepted LiDAR/GNSS cluster correspondences were found.
- Trajectory-geometry yaw is 173.2017°, with 0.8142° between-roundtrip standard deviation. Translation/lever arm is UNKNOWN and provisionally zero.
- In the 5 m GT neighborhood, all 225 valid cross-flight frames contain zero nonzero raw returns. Roundtrip frames are zero-return in 94.5% of 455 valid frames; their 2,553 neighborhood returns fall to zero at the z>10 m stage and are not confirmed UAV points.
- Current baseline: zero matched frames, 82 unmatched states inside the GT intervals, and six never-matched track IDs. State split is 7 measured, 10 predicted, and 65 stale.
- 100 m cross 10 m/s yaw-invariant check: zero points in the per-frame ±5 m range/height shell; one point in the broader 90.137–132.069 m / 3.899–15.786 m envelope, intensity 8, so zero survive the normal intensity threshold.

### Interpretation

This run establishes an auditable manifest and evaluator but does **not** establish calibration-quality ground truth. It rejects the claim that a clearly visible `100m横飞10m/s` UAV trajectory is first deleted by background/tracking: the expected raw range-height volume is already empty. Sensor non-return remains confounded with unresolved sensor pose/time and possible acquisition-content mismatch.

### Limitations

- No hardware synchronization event, surveyed heading, roll/pitch, or lever arm.
- Four-point cross-flight interpolation is low confidence.
- PCA yaw relies on the semantic assumption that named round trips are radial to the LiDAR.
- A 5 m neighborhood is an operational region, not point-level target annotation.
- Zero matches leave localization error and recovery metrics undefined rather than zero.

## E01 — Baseline versus FMT benchmark suite

**Artifact timestamp:** 2026-07-04  
**Objective:** Compare the reconstructed rule baseline with the online FMT tracker on all then-selected filtered PCD sequences.  
**Code/version:** `scripts/benchmark_fmt.py`; historical byte identity **UNKNOWN**, current hash listed above.  
**Inputs:** five filtered PCD directories.  
**Artifacts:** `results/*/comparison_study/benchmark_metrics.json` and available `frame_plots/`.

### Configuration

The five JSON files record the same configuration:

| Parameter | Value |
|---|---:|
| voxel size | 1 m |
| static probability threshold | 0.05 |
| background initialization | 8 frames |
| height threshold | 10 m |
| search radius | 15 m |
| radial threshold | 5 m/s |
| maximum association distance | 30 m |
| DBSCAN `eps` | 2 m |
| DBSCAN `min_points` | 1 |

### Results

Format for track counts is `active / generated / removed`. The JSON field called `fragmentations` is reported here as “removed” because it is not a ground-truth fragmentation metric.

| Dataset | Frames | Baseline tracks | FMT tracks | Smoothness baseline / FMT | FMT residual (m) | Latency baseline / FMT (ms) |
|---|---:|---:|---:|---:|---:|---:|
| 100 m | 46 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 | 0 | 0 / 0 |
| 2025-10-23 | 343 | 0 / 2 / 2 | 0 / 2 / 2 | 0 / 0 | 0 | 0.09 / 0.18 |
| 300 m | 50 | 0 / 8 / 8 | 3 / 8 / 5 | 0 / 50.0868 | 5.1896 | 0.72 / 1.53 |
| longitudinal `81-82` | 28 | 2 / 6 / 4 | 2 / 6 / 4 | 36.4648 / 97.0921 | 20.6905 | 1.37 / 1.40 |
| cross-range `81` | 60 | 1 / 15 / 14 | 5 / 15 / 10 | 1.6023 / 140.4894 | 21.6190 | 0.55 / 2.64 |

### Interpretation

- **VERIFIED RESULT:** The two methods produced no tracks on 100 m and no final tracks on the 2025-10-23 sequence.
- **VERIFIED RESULT:** FMT retained more final tracks on 300 m and cross-range.
- **VERIFIED RESULT:** In the two datasets where baseline smoothness was defined, the reported FMT smoothness value was substantially larger, not smaller.
- **VERIFIED RESULT:** Mean FMT residuals were 5.19–21.62 m on the three nonzero cases.
- **Not established:** that FMT improves target recall, identity preservation, or accuracy. Extra surviving tracks may be false tracks.

### Limitations

- No ground truth or fixed evaluation split.
- Baseline in this script is not equivalent to the main tracker: births/pending candidates are not added after first dynamic initialization.
- Empty candidate frames do not age tracks.
- Radial-velocity sign handling differs between baseline and FMT.
- Cluster reuse is not excluded.
- Intensity masking is misaligned, although intensity is not subsequently used by this benchmark.
- Latency is a small, un-warmed wall-clock measurement and is not a robust performance study.
- “Smoothness” and residual are internal diagnostics with no truth-relative validation.

## E02 — Longitudinal benchmark reproduction

**Audit date:** 2026-09-18, as documented in `FMCW_LiDAR_Project_Archaeology.md`.  
**Objective:** Check whether a checked-in benchmark result could be regenerated from current data and code.  
**Command:**

```bash
PYTHONDONTWRITEBYTECODE=1 MPLBACKEND=Agg fmcw/bin/python \
  scripts/benchmark_fmt.py \
  --data data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data \
  --output /tmp/fmcw_audit_benchmark_dual
```

**Code/version:** current benchmark hash listed above.  
**Result:** All reported non-latency metrics matched the checked-in JSON exactly. Average latency changed from the historical 1.37/1.40 ms to approximately 1.53/2.04 ms in that reproduction.  
**Interpretation:** The current script and input can reproduce the benchmark's logical output for this sequence.  
**Limitations:** This does not validate the metric semantics, tracker correctness, or target identity; `/tmp` output was not retained as a repository artifact.

## E03 — Stage-by-stage filtering archaeology

**Artifact timestamp:** 2026-09-18  
**Objective:** Determine where points disappear in the intensity, height, background, and neighborhood stages.  
**Code/version:** `scripts/data_archaeology.py`, current hash listed above.  
**Artifacts:** `results/data_archaeology/{100m,300m,cross}/stage_counts.json`, `stage_cache.npz`, pipeline/raw-view figures, and `run_summary.json`.

### Configuration

The analysis reconstructs the benchmark preprocessing: named field loading, intensity threshold, height threshold, eight-frame occupancy initialization, static probability threshold, and neighbor filtering. See the JSON artifacts for exact per-frame counts.

### Results

| Sequence | Frames | Raw mean pts/frame | After intensity mean | After height mean | After background mean | Final mean | Final median | Final max | Post-init zero frames |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 m | 46 | 1,250,000 | 111,512 | 631 | 0.54 | 0.00 | 0 | 0 | 38 / 38 |
| 300 m filtered | 50 | 1,250,000 | 113,262 | 5,949 | 14.34 | 7.28 | 6.5 | 27 | 1 / 42 |
| cross-range filtered | 60 | 1,250,000 | 253,674 | 7,567 | 48.52 | 20.08 | 22 | 36 | 0 / 52 |

The project audit additionally records final dynamic-point summaries derived for the other benchmark inputs:

- 2025-10-23: mean 3.25, median 2, max 47, with 57/335 post-initialization zero frames.
- longitudinal: mean 12.07, median 16, max 24, with 0/20 post-initialization zero frames.

### Interpretation

- **VERIFIED RESULT:** The 100 m sequence provides no final dynamic points after initialization.
- **VERIFIED RESULT:** The 300 m and cross-range sequences retain sparse but nonzero dynamic candidates.
- **Inference:** Candidate supply, not only downstream association, is a major determinant of tracker behavior.

### Limitations

- Counts are not UAV recall: most points are unlabeled.
- The 100 m filtered-data failure cannot localize the cause without examining raw data and registration; E06 addresses one expected raw region only.
- Threshold values were not swept, and no independent validation set exists.

## E04 — Trajectory-guided pseudo-label survival

**Artifact timestamp:** 2026-09-18  
**Objective:** Estimate whether points along manually identified target-like trajectories survive each processing stage.  
**Code/version:** `scripts/uav_pseudo_label.py` plus `scripts/compile_uav_evidence.py`; current hashes listed above.  
**Artifacts:** pseudo-label CSVs, point CSVs, feature NPZ files, survival/trajectory plots, `pseudo_label_summary.json`, and `analysis_summary.json`.

### Configuration

- Sequences: cross-range and 300 m.
- Target regions/trajectories were selected using manual and trajectory-guided inspection.
- Nearby non-target points were used as a clutter comparison.
- Stage survival was counted through raw, intensity, height, background, neighbor, and clustering stages.

### Results

| Sequence | Raw pseudo points | Intensity | Height | Background | Neighbor | Cluster | End-to-end survival |
|---|---:|---:|---:|---:|---:|---:|---:|
| cross-range | 504 | 499 | 499 | 438 | 419 | 419 | 83.1% |
| 300 m | 200 | 198 | 198 | 163 | 163 | 163 | 81.5% |

Additional observations:

- Cross-range, after initialization: background retained 438/443 points (98.9%); neighbor filtering retained 419/438 (95.7%).
- At cross-range frame 30, all five pseudo-target points were removed by background suppression during a path revisit.
- At cross-range frames 28 and 47, neighbor filtering removed all remaining pseudo-target points; partial neighbor losses also occurred at frames 29 and 59.
- For 300 m, all 163 post-initialization height-surviving pseudo-target points also survived the background, neighbor, and cluster stages. Its aggregate loss at background was entirely attributable to the eight initialization frames.

### Interpretation

- **VERIFIED RESULT within the selected pseudo trajectories:** initialization, revisits, and isolated-point removal are distinct candidate-recall failure modes.
- **Inference:** Any method applied only after hard candidate generation cannot recover these deleted points.

### Limitations

- Regions were selected retrospectively and are not independent labels.
- The pseudo trajectories may omit weak or ambiguous target returns and can favor the existing pipeline.
- Results cover two sequences, not all ranges or operating conditions.

## E05 — Point-feature and velocity-sign analysis

**Artifact timestamp:** 2026-09-18  
**Objective:** Test whether intensity, radial velocity, and geometric range evolution provide stable target cues.  
**Code/version:** same evidence pipeline as E04.  
**Artifacts:** `analysis_summary.json`, feature NPZ/CSV files, and UAV-versus-clutter plots.

### Configuration

The experiment used the cross-range and 300 m trajectory-guided pseudo regions from E04, compared their point features with nearby clutter regions, and compared sign-inverted measured velocity with frame-to-frame geometric range change. Selection radii and per-frame records are preserved in the scripts and CSV/NPZ artifacts; independent truth was not used.

### Results

| Statistic | Cross-range | 300 m |
|---|---:|---:|
| pseudo-UAV intensity median [IQR] | 27 [22, 33] | 22.5 [18, 28] |
| nearby clutter intensity median [IQR] | 24 [19, 29] | 26 [22, 31] |
| pseudo-UAV radial velocity median | 0 m/s | 0.14 m/s |
| nearby clutter radial velocity median | -0.11 m/s | -0.05 m/s |
| frames with `|median vr| <= 0.25 m/s` | 46.7% | 30.0% |
| correlation of `-vr` with geometric range rate | 0.973 | 0.966 |

Range versus radial-speed rank correlations were not significant in these selected trajectories: cross-range Spearman 0.020 (`p=0.882`), 300 m -0.105 (`p=0.468`).

### Interpretation

- **VERIFIED RESULT:** Measured velocity sign is opposite increasing geometric range in these two analyzed sequences.
- **VERIFIED RESULT:** Intensity distributions overlap strongly; at 300 m the selected UAV points are weaker than nearby clutter by median intensity.
- **VERIFIED RESULT:** Moving cross-range targets can have near-zero radial speed.
- **Not supported:** a universal rule that large Doppler magnitude identifies the UAV, or that intensity alone separates it from clutter.

### Limitations

- Pseudo-label selection and nearby-clutter definitions can bias the distributions.
- Correlation does not establish the sensor's formal units, wrapping, uncertainty, or behavior in all sequences.
- No uncertainty interval or independent replication set was recorded.

## E06 — Nominal 100 m GNSS-informed expected-band search

**Artifact timestamp:** 2026-09-18  
**Objective:** Determine whether target-like raw returns exist near a broad volume implied by sparse GNSS waypoints, before background subtraction.  
**Code/version:** `scripts/scan_100m_expected_band.py`, current hash listed above.  
**Input:** raw 100 m rich bag/point stream as reconstructed by the audit.  
**Artifact:** `results/data_archaeology/100m/gnss_expected_band.json` and plot.

### Configuration

- Expected horizontal range from waypoints: approximately 105–120 m.
- Expected relative height: approximately 6.9–12.8 m.
- Deliberately widened search band: horizontal range 90–130 m and z 4–16 m.
- A low-altitude comparison band used the same horizontal range with z 0–4 m.

### Result

- Across all 46 frames, the widened expected band contained one point in total.
- That point had intensity 8 and measured velocity -2.39 m/s.
- Zero points survived the normal intensity filter.
- The low-altitude comparison band contained 9,357 points with median intensity 31.

### Interpretation

The 100 m tracker failure cannot be attributed solely to background filtering: the expected raw 3D region itself is effectively empty under the assumed pose/alignment. Plausible categories include acquisition failure, incorrect pose/extrinsics, wrong time alignment, wrong coordinate assumptions, or incorrect waypoint interpretation.

### Limitations

- GNSS-to-LiDAR synchronization and extrinsics are unresolved.
- The search band is broad but still depends on the assumed coordinate transform and device position.
- Therefore “the sensor did not see the UAV” is **not established**; the cause remains **UNKNOWN**.

## E07 — Reverse engineering the successful cross-range baseline track

**Artifact timestamp:** 2026-09-18  
**Objective:** Identify why the single final baseline track survives on the cross-range sequence.  
**Code/version:** `scripts/reverse_engineer_baseline_uav.py`, current hash listed above.  
**Artifacts:** `results/data_archaeology/cross/baseline_reverse/`.

### Configuration

- Same candidate-generation and reconstructed baseline settings as the comparison benchmark.
- Retrospective focus on final surviving baseline track ID 1.
- Frames analyzed: 8–59.

### Result

- The track was observed in 48 frames and prediction-only in frames 28, 30, 47, and 50.
- Fifteen tracks were initialized; fourteen were later removed.
- Observed target-like clusters had mean 8.73 points, median 8.5, range 3–17.
- Median cluster diameter was 0.394 m; median z-span was 0.201 m.
- Median intensity was 27; median velocity was -0.11 m/s, with all observed median magnitudes at or below 1 m/s.
- The selected cluster was the largest dynamic cluster in 95.8% of observed frames.
- Median nearest-other-cluster distance was 58.2 m.
- Reconstructed center and pseudo-label center differed by approximately `4e-6` m median, showing that both analyses selected essentially the same points.
- Velocity/range-rate correlation was 0.965 after sign inversion.

### Interpretation

The track survives because its cluster is temporally persistent, compact, and unusually isolated, and because position prediction continues through a small number of candidate dropouts. Large radial speed is not the success mechanism in this tangential-motion case.

### Limitations

- Retrospective analysis selected the successful track and therefore cannot estimate false-alarm probability.
- “Largest cluster” and intensity are sequence-specific, selection-conditioned observations.
- Prediction-only states retain previous point-count/velocity values, which can be mistaken for current measurements.

## E08 — Reverse engineering the successful longitudinal baseline track

**Artifact timestamp:** 2026-09-18  
**Objective:** Determine whether the cross-range success mechanism transfers to an approaching/longitudinal motion sequence.  
**Code/version:** `scripts/reverse_engineer_longitudinal_baseline.py`, current hash listed above.  
**Artifacts:** `results/data_archaeology/longitudinal/baseline_reverse/`.

### Configuration

The experiment reconstructed the comparison benchmark's candidate generation and baseline association on `data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data`, then retrospectively inspected the target-like surviving trajectory over post-initialization frames 8–27 and compared its behavior with the cross-range reverse analysis.

### Result

- Target-like baseline track ID 3 was observed in all analyzed frames 8–27.
- Six tracks were initialized; final active IDs were 1 and 3.
- From frames 17–27, the two active IDs were associated to the same physical cluster because cluster reuse was not prevented.
- Target-like clusters had mean 5.95 points, median 6, range 1–13.
- Median cluster diameter was 0.575 m.
- Median intensity was 26; median velocity magnitude reflected strong approach motion, approximately 7.38 m/s.
- Range decreased from approximately 324 m to 195 m.
- The target-like cluster was largest in 90% of frames and its median nearest-other-cluster distance was 50.4 m.
- Velocity/range-rate correlation was 0.984 after sign inversion.

### Interpretation

Persistence, compactness, isolation, and Doppler/range-change agreement transfer from the cross-range case. Absolute Doppler magnitude does not: it is near zero in the cross-range case and large in the longitudinal case. The two final active tracks do not represent two physical targets.

### Limitations

- The target identity remains pseudo-verified rather than synchronized ground truth.
- One physical cluster can update multiple IDs, so track-count metrics are confounded.
- Only 20 post-initialization frames were analyzed.

## Implemented ideas with no recoverable completed experiment

The following must not be reported as evaluated results:

| Component | Source evidence | Result evidence |
|---|---|---|
| Python ROS online tracker | source exists | UNKNOWN |
| C++ ROS tracker | source/header/build target exists | no build or run artifact |
| GM-PHD tracker | source exists | UNKNOWN |
| EKF GM-PHD tracker | source exists | UNKNOWN |
| `target_track_fmt.py` integration | source exists | UNKNOWN |
| `GrassmannDynamicFilter` as a manifold method | named source exists, but inspected math is running-mean/residual | no validating experiment; claimed manifold behavior not implemented |

## Required format for future entries

Every new experiment should append:

```text
ID and date:
Objective / falsifiable question:
Input dataset and immutable identity:
Ground truth or label status:
Code commit or script SHA-256:
Environment:
Exact command:
Configuration and random seed:
Output directory:
Metrics with definitions:
Result:
Interpretation:
Limitations / failure notes:
Decision caused by the result:
```
