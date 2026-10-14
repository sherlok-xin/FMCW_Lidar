# Current Status

Status date: **2026-10-12**
Basis: repository-wide source and artifact inspection; no chat-history claims are treated as evidence.

Git is now initialized on branch `main` (unlike the 2026-09-20 audit). The tested sparse-MVP script and tests are present in snapshot `e62848f787dcfd1bb2f91536ca23e6a50580a77b`. Necessary diagnostic CSV/JSON/report artifacts are now tracked; raw bags/PCDs, virtual environment and NPZ caches remain excluded. Run-manifest hashes remain required for provenance, including local caches.

## Executive status

The repository contains a substantial rule-based FMCW-LiDAR prototype, ROS/offline variants, retrospective archaeology, provisional calibration and frozen DEV_SILVER comparisons. It does **not** yet provide independently certified GT, reliable target-absent/long-hover labels or held-out operational evaluation needed for reliable UAV detection/tracking accuracy claims.

The latest completed work is an oracle-conditioned rich-return phenotype study on five closure-PASS cross sequences: 272 raw frames, 144 nominal GNSS-supported frames and 80 unchanged selected observations. It exposes localization/selection sensitivity, coarse-voxel support collapse and mostly empty small clutter proxies; it is not independent detector evaluation. E20's independently timestamp-linked negative and hover/slow evidence remains DATA_GAP. E18 is unchanged, and its earlier 79/80 DEV_SILVER recovery does not establish operational or hover generalization. The historical FMT benchmark still does not demonstrate an improvement over its baseline.

### 2026-10-11 negative / hover evidence audit — E20 complete, DATA_GAP

**TESTED / VERIFIED RESULT for read-only metadata/provenance; physical state and new stress results UNKNOWN.** See `FMCW_Negative_Hover_Evidence_Audit.md` and `results/negative_hover_evidence_audit/`. E19 is already pushed as `5ad38b94b87949657f803485da129e3a738e9a64` (live remote main verified). E18 remains frozen at `a53702d`;302protected SHA and complete E18/E19 tree sets unchanged. No algorithm, weight, graph, label, gate, Doppler or threshold changes; no new solver runs.

Expanded inventory:697bag/data files;14ZIP/all52non-directory members (51bags+recordscript); all51bounded1MiB prefixes;6standalone full final connection/chunk-info indexes. Only LiDAR topics, no flight-state/GNSS/video topic. ZIP footer/full payload not decoded; extracted-copy paths are not new sequences. Raw metadata before/after unchanged, not full content SHA. March filename and record date differ; names cannot provide synchronization.

Excel all82nonempty cells=71flight GNSS+device+10titles, no state/events/hidden/comments. All62endpoint segments exported; minimum0.995621m/s is not instantaneous or continuous hover evidence. DeviceA85 duplicatesA2, not a not-airborne certificate. Scene photo EXIF14:16:50.957+08 precedesfirstFebbag21m19.386s and cannot certify a bag interval; its GPS EXIF clock differs. No independent timed flight-state emerged from19source entries or project-path Git history.

All75silver adjacent steps/10continuous fragments retained. Three moving→slow-like shapes and prior5slow windows remain LiDAR-derivedL1, not physical hover GT. PriorE19witness trace168rows=160observed-silver+8nonlabel connectors is stored evidence, not a new run. No qualified negative or hover/slow windows: new header-only stress CSVs mean no measurements, not zero false alarms. Independent moving→hover score degradation UNKNOWN. Stop at DATA_GAP and retain freeze; any further evidence acquisition/evaluation requires a separate scoped request.

### 2026-10-11 E18 mechanism/generalization audit — frozen a53702d

**TESTED / VERIFIED RESULT on the same frozen DEV_SILVER graphs; independent generalization UNKNOWN.** `FMCW_E18_Mechanism_Generalization_Audit.md` and `results/e18_mechanism_generalization_audit/` add only four term switches through unchanged E18 solver. No E18 source/formula/weight/graph/label/3m gate edit, Doppler, threshold selection or remaining-miss fix. Parent HEAD remains `a53702da1dc5357149e3e472a9e489ff08e9afe4`.

Fine original/background-only/motion-only/E18 hits56/56/0/79 of80; historical control0/56/0/79. Background dominates fixed historical static-reference score losses, but final recovery is interactive: motion adds net23hits only with background (24gained/1lost); motion alone can favor near-static paths and erase fine old hits. Candidate volume remains identical within each graph, not precision. Same200m_cross_5/frame21 miss remains (~3.254m).

Inventory14ZIP/51bag members/6standalone paths (3extracted-copy paths) provides no reliable target-absence certificate. Negative winners NOT_RUN_NO_RELIABLE_TARGET_ABSENCE, false alarms/negative scores UNKNOWN, not zero. Independent dedicated hover identity UNKNOWN. Separate ALL5short-slow cross-silver windows/22observations per family give14/12/14/12hits;200m_cross_5/frames7–12 drops6/6→0/6 with background. Its witness B remains+2.539, but a more novel competitor gets a larger reward: relative ranking failure, not negative-score hard deletion. Repeated-cell occupancy has a conditional hover/revisit penalty risk; long-hover/systematic generalization is still UNKNOWN.

12new/14existing tests pass,212protected hashes unchanged,20original/E18 full pools exactly replayed; independent stored-artifact audit passes with new-arm span-rank optimality verification limits disclosed. One inventory-only manifest reconciliation is retained, all solver outputs unchanged. Audit complete; preserve negative/null results and stop, without further tuning, feature stacking, operational threshold or Benchmark v1 upgrade.

### 2026-10-10 isolated background-relative + motion-coherence DP MVP

**TESTED / VERIFIED RESULT on frozen DEV_SILVER only; new user-authorized E18 after the completed E17 diagnostic.** `FMCW_Background_Motion_DP_MVP.md` and `results/background_motion_dp_mvp/` freeze a single causal prior-frame occupancy contrast plus real-dt soft speed/direction transition. No Doppler, sweep, silver-derived weight selection or historical baseline/DBSCAN/geometry/cache/label edit.

Historical E13 graph is1m, not4cm. Primary uses all20,122saved E16adaptive-minPts1components before geometry, including7,202singletons/10,325old rejected components, with479,081frozenedges; both scores share identical inputs. Historical control separately replays16,995nodes/371,600edges and all five old winners. Common eligibility-aware DP corrects negative-score qualification handling without changing historical source/gates.

Primary unconditioned silver hits56/80→79/80;100/200/300m19/30→30/30,37/40→39/40,0/10→10/10. Fixed-witness rank-surrogate and matched-window margin each improve4/5; fine100m_cross_5 alreadyrank1 has no improvement, fine200m_cross_10 margin remains−0.034 and300m fixed-witness rank is only≥8. Historical original static winners all lose rank/score, unconditioned0/80→79/80. New full paths in both families miss200m_cross_5/frame21, distance~3.254m at unchanged3m gate. All80observations have fixed feasible witnesses in both graphs.

Predeclared gate **MVP_GATE_PASSED_NOT_OPERATIONAL_VALIDATION**; no kill triggered, no additional features/weights/experiments run. Operational precision/false alarms, unlabeled path identity, independent GT/held-out performance and separate background/motion contributions remain UNKNOWN. Features are causal but full-bag DP is offline. Rank is terminal-edge representative evidence, not global combinatorial rank.14synthetic tests pass;124protected files unchanged; independent causal audit validates37,117nodes/850,681edges. Completed result audit and manifests preserve all negative/null results. No Benchmark v1 or final operating point selected.

### 2026-10-10 final read-only geometry invariance audit

**TESTED / VERIFIED RESULT on 80 existing observed DEV/SILVER center-neighborhood proxies only.** `FMCW_Geometry_Invariance_Audit.md` and `results/geometry_invariance_audit/` compare raw/4 cm/1 m support, XYZ spans, diameter, range thickness, NN/MST and intensity; no detector/clustering or threshold search was run. All historical 71hits/9misses remain unchanged.

Raw-count/fine-cell group-median max/min ratios are7.9/5.5, while raw diameter/max-axis are1.79/1.92. This is relative descriptive stability, not invariance: x-span/range thickness still vary4.88/3.73;1 m representation erases internal shape in31/80frames,30of which are not raw singletons.8near fixed-component misses have34–46fine voxels but max-axis only0.268–0.813 m, overlapping near hits; adaptive frame20 becomes47voxels/max-axis1.583m but retains the same max32 rejection. The remaining far miss is singleton. No UAV-specific single-frame geometry hard rejection is validated by these selected positive observations; independent point/negative labels and true body dimensions remain UNKNOWN.

100protected files remain identical;6synthetic tests pass;160raw descriptor rows and same-child support reconstruct correctly; independent1802summary/106ratio/1440NN-uniqueNN-MST comparisons pass. Nearest saved component is not automatically the accepted component explaining a frame hit (`300m_cross_10/frame4/fixed`); lookup and historical hit remain separate. Final diagnostic requested by the user is complete; no new rule, feature selection, parameter or algorithm work authorized.

### 2026-10-10 frozen DBSCAN fragmentation attribution

**TESTED / VERIFIED RESULT on DEV/SILVER center-neighborhood proxies only.** `FMCW_DBSCAN_Fragmentation_Attribution.md` and `results/dbscan_fragmentation_attribution/` isolate fixed eps0.6 vs the existing reconstruction's scene-mean adaptive eps (~1.678 m), each at DBSCAN minPts1/2, with the same 4 cm cache and unchanged geometric nmin2/max32. All four recover71/80:100/200/300 m22/30,40/40,9/10. Adaptive has0 gained/0 lost silver frames, but reduces accepted volume111.97→36.02/frame through full-scene component merging and geometry rejection. 3 m proxy connectivity65→76/80 does not translate to new detection matches.

The9 fine-stage misses are8 near-range max32 rejections plus1 far singleton; minPts1 is still rejected by geometric nmin2. Same point-level input4 cm→1 m support<2 rises1→31/80; this is representation evidence, not a complete detector voxel sweep.390 newly-large merged components coexist with398 old-large coalescences, so net large-count−8 must not be read as reduced merge risk. No formal algorithm/parameter/label/cache changes;74 protected files unchanged,6 synthetic tests pass,272-frame original-function candidate replay is exact. No operating point selected; operational precision remains UNKNOWN.

### 2026-10-10 read-only range/sparsity audit

**TESTED / VERIFIED RESULT on observed DEV/pseudo neighborhoods only.** `FMCW_Range_Sparsity_Audit.md` and `results/range_sparsity_audit/` reconstruct raw/4 cm/1 m occupied count, raw extent/NN/pairwise/MST, intensity, Doppler, and surrounding clutter from 80 frozen cross observations plus 48 legacy cross and 20 legacy approach observations. No detector/tracker or labels changed; 39 protected files retain identical SHA-256.

Feb 100/200/300 m raw-count medians are 39.5/17.5/5; unique-xyz NN medians are 0.023/0.040/0.060 m, but pairwise/MST/extent do not share a stable monotonic range law. Surrounding 3–30 m raw clutter NN also rises, 0.027/0.050/0.062 m. The 3–15 m shell is often empty, especially at 300 m, so its conditional NN is not a stable density estimate.

Legacy approach range decreases 324.18→194.67 m; count rho=-0.695, but successive count changes are 9 increases/1 tie/9 decreases. Its raw counts equal the old target cluster, 1–13 points, not 3→18. Exact old claim source UNKNOWN; frame15 contains target3 and whole-dynamic18. Evidence supports sparse-support/representation concerns and a temporal-context motivation, not an adaptive-epsilon function or proven adaptive-window gain. Prior algorithm negative results remain unchanged.

### 2026-09-29 DP failure-forensics update

The frozen E13 candidate graph was replayed exactly and analyzed without algorithm or parameter changes. A full silver-consistent feasible path exists in all five sequences, so the 0/80 output is a ranking failure rather than graph/candidate loss. See `FMCW_DP_Failure_Forensics.md` and `results/dp_failure_forensics/`.

The always-positive `0.50` per-node base term makes a full-bag stationary return accumulate the largest score. On equal time spans, compactness slightly favors clutter; the capped raw-support component removes most of the target's support advantage. In the two sparse-gap cases, one missing node and the gap penalty further lower the UAV path.

Legacy occupancy/revisit, nonzero speed, directional smoothness, absolute Doppler, and raw support separate the selected UAV/clutter paths descriptively. They are not validated general detector features: labels are retrospective, only winning clutter is compared, and Doppler residual itself favors near-static clutter.

### 2026-09-29 published-baseline and candidate-graph DP update

The independent Khosravi-2026 processing-chain reconstruction and minimal candidate-graph DP are **IMPLEMENTED and TESTED** on the frozen 80-frame `research_dev_v0` silver set. See `FMCW_Published_Baseline_and_DP_TBD_MVP.md` and `results/published_baseline_dp_mvp/`. A machine-readable parameter audit separates paper-reported values, paper-`UNKNOWN` values, and project adaptations; the implementation is not presented as an exact reproduction.

The published-structure reconstruction detects 58/80 silver observations with 26.39 outputs/frame, versus project-adapted B at 41/80 and 23.88 outputs/frame. The difference is material on this DEV set but confounded by voxel, clustering, centroid, and temporal-semantic changes. With no reliable negative labels, operational improvement remains **UNKNOWN**.

The singleton candidate-graph DP preserves 80/80 observations at input and reduces 62.48 candidates/frame to one selected path point/frame, but its output recall is 0/80. It consistently selects long-lived near-static clutter, usually around 204 m. This frozen minimal DP is rejected as a solution; its weights must not be tuned on the same labels.

### 2026-09-29 temporal-evidence DEV experiment update

The frozen lightweight accumulator experiment is **TESTED** on the unchanged 80-frame `research_dev_v0` silver set. The report is `FMCW_Temporal_Evidence_MVP.md`; protocol, per-frame results, accepted candidates, corrected-legacy stage survival, threshold sweep, and figures are in `results/temporal_evidence_mvp/`.

The corrected legacy A′ baseline reruns legacy background, neighborhood, and DBSCAN in provisional leveled coordinates from the pre-stage 1 m raw-voxel artifact. It detects 41/80 silver observations with 5.98 outputs per LiDAR frame, compared with the current Khosravi-style B at 41/80 and 23.88 outputs/frame. A′ remains an approximation to raw-point legacy processing and its near-zero matched-center errors are representation self-consistency, not independent accuracy.

Unaccumulated `minPts=1` reaches 80/80 but emits 62.48 candidates/frame. The prospectively frozen soft threshold of 0.68 over-suppresses the target: fixed/adaptive/adaptive+Doppler recover only 7/7/8 observations. A predeclared diagnostic threshold of 0.55 gives fixed-window 52/80 and 17.23 outputs/frame, but this same-DEV observation must not be promoted to a selected default without held-out data. The simple adaptive window does not improve the overall trade-off; soft Doppler has no consistent Pareto advantage.

Operational precision, false alarms/frame, and PR remain **UNKNOWN**. All bags are target-flight bags and frames outside fragmented silver trajectories are not reliable negatives.

### 2026-09-22 sparse-UAV DEV benchmark update

`research_dev_v0` is frozen with 80 non-interpolated **DEV/silver** observations from five closure-passing cross-flight sequences. It is a method-development set, not final paper GT. The machine-readable labels, protocol, stage audit, A–D results, full ablations, observability strata, and plots are in `results/research_dev_v0/`; the report is `FMCW_Sparse_UAV_MVP.md`.

The original LiDAR-frame `z>10` stage removes all 1,877 intensity-surviving target-neighborhood points. The same threshold after provisional 6-DoF gravity alignment retains all 1,877. The current baseline therefore has 0/80 DEV recall. The Khosravi-style no-Doppler baseline reaches 41/80, but candidate precision on positive frames is only 2.10%. Temporal consistency is not independently beneficial in the frozen ablation, and neither fixed nor observability-aware Doppler improves recall. At 300 m, minPts sensitivity (0.90/0.30/0.00 recall for 1/2/3) supports studying pre-cluster temporal evidence, subject to an independent-label/negative-scene gate.

### 2026-09-22 calibration-closure update

An independent full-space LiDAR trajectory search has superseded the v0 interpretation of the cross-flight raw visibility. Five cross-flight sequences pass a shared-extrinsic GNSS closure, including held-out 100/200/300 m 10 m/s sequences. The provisional shared pose is yaw 173.4498°, pitch -6.5149°, roll 2.2361°, translation `[1.2056,-6.2698,2.0066] m`.

This is not a final calibration. `300m_cross_5`, both 5/13 m/s roundtrip cases, and the 10 m/s roundtrip Doppler test do not achieve complete closure; time offsets include boundary and multi-modal solutions. Benchmark v1 is therefore still blocked. Full evidence and machine-readable trajectories are in `FMCW_Calibration_Closure.md` and `results/calibration_closure/`.

Most importantly, `100m_cross_10` contains a strong independent LiDAR trajectory around x=105–116 m. The v0 zero-neighborhood result was a consequence of the incomplete spatial model, not evidence of sensor non-return.

### 2026-09-22 GT/Benchmark v0 update

The nine 2026-02-05 packages now have an auditable manifest and provisional per-frame GNSS interpolation in `gt_manifest.csv` and `frame_ground_truth.csv`. `benchmark_v0.csv` scores the unchanged current baseline with measured, predicted, and stale states separated. However, no real point-cloud correspondences supported a unique GNSS–LiDAR rigid/time fit. The spatial and time confidence therefore remains LOW, and Benchmark v0 is not yet suitable for final localization or detection claims.

Verified run-level facts:

- 807 LiDAR frames; 680 within GNSS intervals; no extrapolation.
- all nine topics are `/aqronos_cloud_0` rich PointCloud2;
- per-package time offsets are independently estimated at 84.768–100.834 s, not hard-coded;
- ROS headers carry a wrong 2022 absolute epoch;
- trajectory-geometry yaw is 173.2017°; translation/lever arm is UNKNOWN;
- baseline has zero provisional GT matches, 82 unmatched in-interval states, and six false track IDs;
- the 100 m cross 10 m/s strict yaw-invariant raw envelope remains essentially empty before preprocessing.

The last bullet is a **historical v0 result and is superseded by E10**: its height envelope used an uncalibrated yaw-only/zero-translation spatial projection.

## Implementation status matrix

| Component | Implemented | Matching execution evidence | Current assessment |
|---|---|---|---|
| ROS rich-cloud to five-field republisher | Yes | UNKNOWN | Source exists; complete bag-to-PCD route missing |
| Coordinate/intensity PCD filter | Yes | Derived PCDs exist; exact lineage UNKNOWN | Operational artifacts exist, but commands/configs were not recorded |
| Voxel occupancy background filter | Yes | Yes | Produces dynamic candidates; initialization/revisit failure modes verified |
| 27-neighbor density filter | Yes | Yes | Reduces isolated clutter and sometimes removes sparse pseudo-target returns |
| DBSCAN clustering | Yes | Yes | Runs with permissive `min_points=1` in current benchmark |
| Manual-initialization tracker | Yes | UNKNOWN | Superseded prototype |
| Automatic rule tracker | Yes | Yes, through reconstructed benchmark/reverse analyses | Most mature offline baseline, but not objectively scored |
| Python ROS tracker | Yes | UNKNOWN | Defaults differ from other variants |
| C++ ROS tracker | Yes | No build/run artifact | ROS1 toolchain unavailable at audit time |
| GM-PHD tracker | Yes | No matching results | IMPLEMENTED only |
| EKF GM-PHD tracker | Yes | No matching results | IMPLEMENTED only |
| `GrassmannDynamicFilter` | Yes | No isolated evaluation found | Implementation is running-mean/residual, not verified Grassmann learning |
| FMT tracker/benchmark | Yes | Five benchmark result sets | TESTED, but comparison is confounded and accuracy is UNKNOWN |
| Data archaeology pipeline | Yes | Reports, JSON/CSV, plots | TESTED and currently the best evidence base |
| Frozen sparse-UAV DEV benchmark | Yes | 80 silver observations, A–D and ablations | TESTED; development-only, selection-conditioned |
| Lightweight temporal evidence accumulator | Yes | Frozen E12 run, threshold sweep, tests, and artifacts | TESTED; formal high-recall objective failed; diagnostic 0.55 point needs independent validation |
| Synchronized truth evaluation | No | No | Critical blocker |
| Reproducible experiment/config registry | No | No | Critical blocker |

## What currently works

### Data access and preprocessing

- Filtered PCD sequences are readable and have been processed across all five benchmark datasets.
- The repository can extract position, intensity, and velocity fields by name.
- The coordinate transform, intensity filter, height filter, voxel background filter, neighbor filter, and DBSCAN stages execute in the local Python environment.
- Data-archeology scripts generated stage counts, pseudo-label CSVs, plots, reverse-engineered trajectories, and summary JSON files.

### Offline benchmark execution

Five baseline-versus-FMT `benchmark_results.json` files exist. A documented read-only reproduction on the longitudinal filtered sequence reproduced all internal non-latency metrics exactly; timing changed, as expected for a one-shot wall-clock measurement.

This establishes reproducibility of that specific script/data snapshot, not correctness of its tracker or metrics.

### Target-like trajectories in two sequences

- In the cross-range filtered sequence, one baseline track was retrospectively matched to a compact, isolated, persistent target-like cluster over frames 8–59, with four prediction-only frames.
- In the longitudinal filtered sequence, one target-like trajectory was observed in all analyzed frames 8–27.
- In both cases, the sign-inverted point velocity agrees strongly with geometric range-rate changes.

These statements are based on manually checked, trajectory-guided pseudo evidence; identity as the true UAV remains less certain than synchronized ground truth would provide.

## What has already been implemented

### Processing evolution recovered from source

1. Point-cloud repackaging and intensity filtering.
2. Preliminary voxel static/dynamic separation.
3. Neighborhood-density suppression.
4. Manual-ROI track initialization.
5. Automatic track initialization.
6. Integrated grid background model plus pending-candidate and track-removal logic.
7. Python and C++ ROS-facing variants.
8. GM-PHD and EKF GM-PHD experimental trackers.
9. A named Grassmann/FMT experimental branch.
10. Benchmark, pseudo-label, expected-band, and reverse-engineering analysis scripts.

The exact chronological order is inferred from file naming and code structure because Git history is absent.

## Experiments already run

Detailed records are in [`EXPERIMENT_LOG.md`](EXPERIMENT_LOG.md). Recovered completed experiment families are:

1. five-sequence baseline-versus-FMT benchmark;
2. reproducibility check on the longitudinal benchmark sequence;
3. stage-by-stage filtering archaeology for 100 m, 300 m, cross-range, and summary statistics for two additional sequences;
4. trajectory-guided pseudo-label survival analysis for cross-range and 300 m;
5. point-feature and velocity-sign analysis;
6. GNSS-informed raw-data search for the nominal 100 m sequence;
7. reverse engineering of the successful cross-range baseline track;
8. reverse engineering of the successful longitudinal baseline track;
9. frozen corrected-legacy and lightweight temporal-evidence comparison on `research_dev_v0`.

No matching experimental output was found for the GM-PHD, EKF GM-PHD, C++ ROS, or `target_track_fmt.py` integration paths.

## Strongest verified findings

1. **The 100 m failure is upstream of the tested background pipeline.** Across 46 raw frames, a widened GNSS-consistent band contained only one point, with intensity 8, and zero points survived the normal intensity threshold. This is an acquisition, recording, pose, or alignment ambiguity—not evidence that voxel background subtraction alone failed.

2. **The first eight initialization frames are a guaranteed blind interval in current processing.** On the 300 m pseudo trajectory, all aggregate background-stage loss occurred during initialization; post-initialization all 163 height-surviving pseudo-target points survived the background stage.

3. **Path revisits and neighbor suppression can remove target-like returns.** In the cross-range pseudo trajectory, the background stage lost all five candidate target points at revisit frame 30, and the neighbor stage eliminated all remaining candidates at frames 28 and 47.

4. **Doppler sign is empirically reversed relative to increasing geometric range in two analyzed sequences.** Correlations were 0.973 and 0.966 after sign inversion.

5. **Doppler magnitude alone cannot be a universal motion discriminator.** In the cross-range sequence, 46.7% of pseudo-labeled frames had absolute median radial velocity at or below 0.25 m/s.

6. **Single-frame shape is not reliably stable.** The reconstructed target clusters contain few points and vary substantially in point count and extent.

7. **The current FMT benchmark does not establish an improvement.** It retains more final tracks in two datasets, but has no truth labels, shows higher reported smoothness variance in both sequences where baseline smoothness is defined, and has large residual errors.

8. **Corrected-coordinate legacy processing is a necessary baseline.** A′ reaches the same 41/80 silver recall as B with 5.98 rather than 23.88 outputs per LiDAR frame, although its 1 m pre-stage voxel input is only an approximation to raw-point legacy processing.

9. **Simple formal soft accumulation does not solve singleton clutter.** At the frozen threshold, it reduces outputs by about 97% but retains only 7–8 of 80 silver observations.

10. **A fixed-window diagnostic point is promising but unvalidated.** Threshold 0.55 yields 52/80 with 17.23 outputs/frame; adaptive windows and soft Doppler do not consistently improve on it. Negative scenes and held-out validation are still required.

## Major observed problems

### Evidence and evaluation

- No synchronized UAV positions, object identities, boxes, or point labels.
- No negative-scene definition or standard data split.
- No detection/tracking accuracy metrics.
- Existing active/generated/removed counts can reward false tracks.
- Pseudo-label selection is retrospective and can bias feature conclusions.

### Data provenance and calibration

- Bag-to-PCD conversion commands, source bag/topic, preprocessing parameters, and dropped-frame reasons are not recorded.
- GNSS-to-LiDAR time alignment and extrinsics are missing.
- Dual-sensor identity/extrinsics are missing.
- The formal velocity definition, sign, units, range, wrapping behavior, and uncertainty are missing.
- PCD timestamps and bag dates are not consistently authoritative.

### Candidate generation

- Background initialization suppresses all output for eight frames and can absorb a moving target.
- Occupancy-based background removal fails on path revisits.
- Neighborhood filtering can delete sparse target points.
- `min_points=1` converts single points into candidate clusters.
- Fixed thresholds are shared across ranges and motion geometries without calibrated sensitivity analysis.

### Tracking and benchmark semantics

- Many paths assume `dt=1` and truncate timestamps.
- Baseline/FMT radial-velocity sign treatment is inconsistent.
- A populated `used` cluster set does not prevent multiple tracks from using one cluster.
- Empty-candidate frames can skip state aging in the benchmark.
- The benchmark's baseline birth logic differs from the main baseline.
- The benchmark intensity array is not masked consistently with points.
- A 30 m association gate can be overly permissive for sparse clusters.
- “Fragmentations” counts removals rather than truth-relative track fragmentation.
- Prediction-only frames retain prior point-count and velocity values, which can look like measurements in the GUI.

### Software and reproducibility

- No Git repository or historical commits.
- No pinned environment lock file.
- No centralized configuration schema.
- No unit/regression test suite was found.
- No notebooks or conventional scientific log files were found.
- ROS1 source exists, but the audited host lacked `roscore` and `catkin_make`.
- Stale `.vscode` and `.claude` paths reference missing/unrelated projects.

## Current bottlenecks

Ranked by impact:

1. **Ground truth and evaluation protocol.** Without these, algorithm changes cannot be ranked scientifically.
2. **Raw-to-derived provenance.** It is not possible to prove which bag/topic/config produced each PCD/result directory.
3. **Sensor calibration and time alignment.** Doppler and geometry cannot be fused rigorously without timestamps, sign/unit validation, and extrinsics.
4. **Candidate recall.** Detect-then-track cannot recover returns deleted during initialization, revisit suppression, or neighborhood filtering.
5. **Tracker correctness.** Association reuse, track aging, `dt`, sign, and benchmark divergence confound comparisons.
6. **Small and biased experimental coverage.** Strong reverse analyses cover only two successful sequences, and the 100 m sequence is unresolved upstream.

## Incomplete or unverified work

- GM-PHD and EKF GM-PHD performance: **UNKNOWN**.
- C++ build/runtime behavior: **UNKNOWN**.
- `target_track_fmt.py` end-to-end output: **UNKNOWN**.
- Real Grassmann-manifold model: not found.
- Online performance on live ROS topics: **UNKNOWN**.
- 200 m and round-trip sequence performance: **UNKNOWN**.
- UAV identity for every reconstructed track: **UNKNOWN** without synchronized truth.
- Validity of 100 m GNSS envelope in the LiDAR frame: **UNKNOWN**.
- Generalization across dates, ranges, UAVs, weather, and sensor placements: **UNKNOWN**.

## Unresolved technical questions

1. What precisely does `velocity` measure, in which units and sign convention, and how does it wrap or saturate?
2. Which raw bag and topic produced each PCD sequence, with which transform/filter command?
3. What are the LiDAR-to-GNSS and dual-LiDAR extrinsics and clock offsets?
4. Was the UAV present and in the expected volume during all nominal 100 m frames?
5. How should background modeling avoid absorbing initialization targets and suppressing revisits?
6. Can a multi-frame weak-return method improve recall before hard thresholding without causing an unacceptable false-alarm rate?
7. Which association model is justified after fixing timestamp, sign, and cluster-reuse issues?
8. Are intensity and point-count distributions stable across sensor settings, range, aspect, and weather?
9. How much of the current tracker output is target versus vegetation, birds, multipath, or other dynamic clutter?
10. What evaluation split and metrics reflect the intended operational use?

## Readiness assessment

- **Engineering prototype:** yes.
- **Reproducible historical research baseline:** partial.
- **Validated UAV detector/tracker:** no evidence yet.
- **Ready for algorithm ranking:** no; truth/provenance/calibration must be addressed first.
- **Ready for carefully scoped diagnostic experiments:** yes, provided existing data/results are preserved and limitations are recorded.

## 2026-10-12 completed — GNSS-conditioned rich-return phenotype

- **IMPLEMENTED / TESTED:** separate diagnostic helpers and 50 tests; original-workbook crosscheck and cached-frame geometry export. No baseline/E18 edits.
- **VERIFIED RESULT (metadata only):** 272 cached frames / 144 nominal GNSS-supported / 80 saved observations; existing sensitivity union covers 64/80 anchors, so is not a certified location bound.
- **FAILED attempt 01 preserved; TESTED run_02 completed after explicit user approval:** export-only correction preserves original bytes/native fields. All five full-member hashes/EOF CRC checks and frame identities validated; extraction elapsed 905.881 s.
- **VERIFIED RESULT, descriptive:** 3,602 region records; GNSS 3 m empty in 103/144 frames versus nonempty selected anchors in 80/80. This is localization/selection sensitivity, not a physical missed-detection rate. Same-child 1 m occupancy has 31 singleton-cell observations versus one unique-XYZ singleton; 311/318 small anchor controls are empty, limiting feature separation.
- **TESTED independent QA:** all 80 anchor descriptors exactly match E15 across 1,920 comparisons; 62,832 statistical bookkeeping checks pass. Rich fields retain UNKNOWN physical semantics; acceleration/evenness are constant and direction repeats scalar sign.
- **Next evidence requirement:** independent point/presence/FOV and positional/time evidence, occupied clutter controls, high-eta radial and hover transitions on preregistered held-out data. Do not overwrite either run, retune E18, infer physical sensor non-return from oracle-region zeros, or use descriptive AUC as independent accuracy.
- [Report](../FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md) / [artifact readiness](../results/gnss_conditioned_uav_phenotype/README.md). E18 scientific inputs and silver labels remain frozen; no method or benchmark result is claimed.


## 2026-10-13 — 已完成 ROS2 目视归档与 frozen discrepancy decomposition

- **IMPLEMENTED / TESTED：** `results/rviz_gnss_lidar_sanity/` 保存上一轮 `/tmp` 目视报告、17 帧表、代表图和本轮全部 80 observed anchors 的分解 CSV、sequence/chord 汇总、方向一致性与可复现纯数值脚本。临时 `markers.py` 仅本地归档并 Git 忽略；大 bag 与 ROS runtime/config/log/cache 不在 GitHub 提交范围。
- **VERIFIED RESULT：** 26.575401 m 的 `200m_cross_5/frame21` 为 along-track 主导：+26.389269 / −3.056353 / +0.719090 m（along/horizontal-cross/vertical-normal）；等效时间 +6.707545 s，只作诊断，不应用 offset。该帧在 GNSS 最后样本 A70，正向等效查询超出支持。
- **UNKNOWN 因果：** 同一 chord 内的误差变号及 frame17→21 连续增长不支持单一恒定 clock residual 或孤立跳点的充分解释；稀疏 chord/转弯表达、时间/航段别名与较小空间残差可共存，未重新拟合。
- **TESTED QA：** 80 行重建、同字节归档、独立 5,815 个标量计算通过；source identity 与全 frozen inputs 最终核验存于新 archive。无 E18、外参、时间 offset、GNSS、标签、阈值或原 bag 变化，不构成新方法或 Benchmark v1。


## 2026-10-13 — Scene Spatiotemporal Background Audit completed

**TESTED / descriptive only：** 已保存 [场景审计](../FMCW_SCENE_SPATIOTEMPORAL_BACKGROUND_AUDIT.md) 和 [新结果目录](../results/scene_spatiotemporal_background_audit/README.md)。五包272帧的raw/fine因果历史和20,122fine-node特征先冻结/计算，之后才join80anchors/旧路径。原E18 H/p/b全量复现，358protected科学输入未变。

1m raw scene高度persistent；distance含与occupancy不完全相同的几何信息，但100m_cross_10在raw下的competitor反例及raw/fine/cutoff翻转阻止把它升级score。4cm低revisit不等于dynamic。保持E18、标签、标定、offset、历史图/结果；没有新detector或threshold、未commit/push。下一证据需求仍是独立presence/identity、occupied negatives、exposure与heldout场景，而非在此DEV集调参。


## 2026-10-13 — Occupancy Temporal-Structure Audit completed; no E21

**TESTED / descriptive：** [时间结构审计](../FMCW_OCCUPANCY_TEMPORAL_STRUCTURE_AUDIT.md) 和 [新结果目录](../results/occupancy_temporal_structure_audit/README.md) 已完成。原fine_1m cell history查询37,117nodes，E18 p/原时间字段精确复现；同frame/H配对和strict kill独立复核通过，437protected文件未变。

局部100m3帧与300m1帧有recency/recent差异，但没有跨五包稳定共同支持。**当前证据层面E18occupancy足够，不提出E21backgroundmodel。** 不代表时间排列数学冗余或零效应；UNKNOWN/cold-start/辅助bin反向保留。未改E18、score、graphs、labels/calibration/offset/历史结果，未跑DP/路径rescore；新审计未commit/push。


## 2026-10-14 — STOP_DEV_FEATURE_MINING；外部复现路线图完成

**VERIFIED RESULT（发布/保存）：** 先将完整occupancy temporal报告和37文件结果目录原样提交并推送至GitHub main，commit `92cf79d01bbb84eee52dde7fd22210f78067d6cd`；SSH `ls-remote`核验相同SHA。报告/目录38个文件与该commit字节一致，437个protected科学文件hash一致。[独立发布收据](../results/external_reproduction_roadmap/audit_publication_receipt.json)。原审计中的未推送文字保留为当时快照。

**项目决定已生效：STOP_DEV_FEATURE_MINING。** 既有五条cross DEV bags及80 DEV_SILVER anchors不再用于提出/调节E18 background features、weights、windows、thresholds或E21/E22 score。E18保持冻结；新表示/子段/外部tracker不能绕过该决定。详见 [决策文件](DEV_FEATURE_MINING_STOP.md)、`AGENTS.md`与TODO顶部。

**VERIFIED RESULT（公开源核查），非方法TESTED：** [路线图](../FMCW_EXTERNAL_REPRODUCTION_ROADMAP.md)仅含三个指定项目。Stone Soup官方组件/合成测试存在，未来锁定v1.9.1；DopplerTrack官方代码存在，需作者数据和帧号对应；UnLiDAR有两个官方局部片段，但完整可运行方法发布未建立、许可未声明，官方完整复现BLOCKED。未来补写必须标PAPER-STRUCTURE RECONSTRUCTION。[当前公开metadata/源码证据](../results/external_reproduction_roadmap/repository_verification.json)。

**唯一首个任务推荐（PROPOSED）：** Stone Soup最小synthetic-clutter Bernoulli存在性复现，先官方组件测试与empty-scan Bayes gate，再明确标注本地连接的synthetic harness；不连接FMCW/DEV。当前仅路线图，无方法实现/执行、大依赖安装或数据集下载，不构成E21/E22授权。
