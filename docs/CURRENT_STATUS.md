# Current Status

Status date: **2026-09-22**  
Basis: repository-wide source and artifact inspection; no chat-history claims are treated as evidence.

Git is now initialized on branch `main` (unlike the 2026-09-20 audit). The tested sparse-MVP script and tests are present in snapshot `e62848f787dcfd1bb2f91536ca23e6a50580a77b`; generated `results/` artifacts are ignored, so their run-manifest hashes remain required for provenance.

## Executive status

The repository contains a substantial rule-based FMCW-LiDAR processing and tracking prototype, several ROS/offline variants, and a useful body of retrospective data archaeology. It does **not** yet contain the ground truth, provenance, calibration, evaluation protocol, or controlled comparisons required to make reliable UAV detection/tracking accuracy claims.

The strongest current outcome is diagnostic rather than algorithmic: two successful motion sequences show that compactness, spatial isolation, temporal persistence, and agreement between Doppler and range change are more transferable cues than raw Doppler magnitude or a stable single-frame shape. The existing FMT benchmark does not demonstrate an improvement over the baseline.

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
8. reverse engineering of the successful longitudinal baseline track.

No matching experimental output was found for the GM-PHD, EKF GM-PHD, C++ ROS, or `target_track_fmt.py` integration paths.

## Strongest verified findings

1. **The 100 m failure is upstream of the tested background pipeline.** Across 46 raw frames, a widened GNSS-consistent band contained only one point, with intensity 8, and zero points survived the normal intensity threshold. This is an acquisition, recording, pose, or alignment ambiguity—not evidence that voxel background subtraction alone failed.

2. **The first eight initialization frames are a guaranteed blind interval in current processing.** On the 300 m pseudo trajectory, all aggregate background-stage loss occurred during initialization; post-initialization all 163 height-surviving pseudo-target points survived the background stage.

3. **Path revisits and neighbor suppression can remove target-like returns.** In the cross-range pseudo trajectory, the background stage lost all five candidate target points at revisit frame 30, and the neighbor stage eliminated all remaining candidates at frames 28 and 47.

4. **Doppler sign is empirically reversed relative to increasing geometric range in two analyzed sequences.** Correlations were 0.973 and 0.966 after sign inversion.

5. **Doppler magnitude alone cannot be a universal motion discriminator.** In the cross-range sequence, 46.7% of pseudo-labeled frames had absolute median radial velocity at or below 0.25 m/s.

6. **Single-frame shape is not reliably stable.** The reconstructed target clusters contain few points and vary substantially in point count and extent.

7. **The current FMT benchmark does not establish an improvement.** It retains more final tracks in two datasets, but has no truth labels, shows higher reported smoothness variance in both sequences where baseline smoothness is defined, and has large residual errors.

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
