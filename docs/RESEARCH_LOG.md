# Research Log

Reconstruction date: **2026-09-20**

This document separates tested evidence from scientific interpretation. Chronology before the artifact timestamps is partly inferred because no Git history, lab notebook, or conventional run log was found.

## Reconstructed research progression

### Phase A — Point-cloud reduction and filtering

**IMPLEMENTED:** The project first establishes a reduced five-field representation (`x`, `y`, `z`, `intensity`, `velocity`), coordinate conversion, and intensity filtering. Preliminary static/dynamic scripts then add voxel occupancy and neighbor-density filtering.

**Evidence:** `get_lidar.py`, `multi_filter_with_intensity.py`, `single_filter.py`, `static_and_dynamic.py`, and `static_and_dynamic_density.py`.

**UNKNOWN:** exact chronology, original design notes, and which script/config generated each checked-in filtered PCD sequence.

### Phase B — Rule-based tracking

**IMPLEMENTED:** Tracking evolves from manual ROI initialization to automatic initialization, voxel-grid background suppression, pending candidates, stability scoring, and lost/static removal. Python ROS and C++ ROS variants are present.

**Evidence:** `target_track.py`, `target_track_without_initialize.py`, `target_track_without_initialize_and_grid.py`, ROS variants, and C++ sources.

**Current interpretation:** The architecture is detect-then-track. Its ceiling is set by candidate-generation recall: once sparse UAV points are removed, the tracker cannot recover them.

### Phase C — Alternative tracking ideas

**IMPLEMENTED:** GM-PHD, EKF GM-PHD, a running-mean filter named `GrassmannDynamicFilter`, an FMT smoother, and an online FMT benchmark branch.

**TESTED:** Only the reconstructed baseline-versus-FMT benchmark has recoverable matching output.

**UNKNOWN:** whether the GM-PHD, EKF GM-PHD, C++ ROS, and integrated `target_track_fmt.py` paths have ever run successfully.

### Phase D — Data archaeology and mechanism analysis

**TESTED:** Stage-count diagnostics, pseudo-label survival, raw expected-band search, feature comparisons, and reverse analyses of two successful baseline tracks were created on 2026-09-18.

**Current interpretation:** This work materially improves understanding of failure mechanisms, but it does not replace synchronized ground truth.

## Hypotheses investigated

### H1 — Online voxel occupancy can separate moving targets from static background

**Status:** partially supported, with verified failure modes.

**Evidence for:** Post-initialization, the 300 m pseudo trajectory retained all 163 height-surviving selected points through the background stage. Cross-range background survival after initialization was 438/443 (98.9%).

**Evidence against/generalization limits:** The first eight frames emit no candidates; a cross-range path revisit lost all five selected points at frame 30; 100 m contains essentially no points in the assumed raw target volume before this stage.

**Conclusion:** Useful on some trajectories, but not a target-safe background model.

### H2 — Neighborhood density filtering removes clutter while preserving the UAV

**Status:** mixed.

**Evidence for:** It reduces spatially isolated output and retains 95.7% of post-background selected cross-range points in aggregate.

**Evidence against:** It removed all remaining selected target points at cross-range frames 28 and 47 and partially removed them at frames 29 and 59.

**Conclusion:** The filter trades clutter rejection for sparse-target recall; its operating point is not calibrated.

### H3 — Large radial-velocity magnitude is a reliable UAV discriminator

**Status:** rejected as a universal rule by current evidence.

**Evidence:** Cross-range motion has a median selected velocity near zero, and 46.7% of selected frames have absolute median velocity at or below 0.25 m/s. The longitudinal target-like trajectory has much larger approach speed, approximately 7.38 m/s.

**Conclusion:** Radial velocity should be used geometrically and temporally, not as a universal magnitude threshold.

### H4 — The stored velocity can be checked against temporal range change

**Status:** supported on two selected sequences.

**Evidence:** Correlation between `-velocity` and geometric range rate is 0.973 for cross-range, 0.966 for 300 m, and 0.984 in the longitudinal reverse analysis.

**Conclusion:** Doppler/range-change consistency is a promising association or validation cue. The formal sensor convention remains **UNKNOWN**.

### H5 — Intensity separates UAV returns from nearby clutter

**Status:** not supported as a general standalone discriminator.

**Evidence:** Distributions overlap. Cross-range selected UAV median intensity is 27 versus 24 for nearby clutter; at 300 m the selected UAV median is lower, 22.5 versus 26.

**Conclusion:** Intensity may be contextual but is not a stable universal threshold from current evidence.

### H6 — The UAV has a stable single-frame point-cloud shape

**Status:** rejected for current sparse observations.

**Evidence:** Successful tracks have only about 6–9 points per typical observed frame, counts range from 1 to 17, and observed extents vary.

**Conclusion:** A rigid or stable single-frame shape prior is poorly supported. Multi-frame motion structure is more plausible.

### H7 — Persistence, compactness, and isolation explain successful tracking

**Status:** supported retrospectively on two sequences, not yet validated prospectively.

**Evidence:** The selected cross-range and longitudinal clusters persist through most/all analyzed frames, have median diameters below 0.6 m, are largest in 95.8%/90% of frames, and are typically more than 50 m from the nearest other cluster.

**Conclusion:** These cues transfer across tangential and longitudinal motion better than absolute Doppler magnitude. Because the tracks were selected after seeing success, prospective validation is required.

### H8 — FMT improves tracking stability or survival

**Status:** not supported by the current benchmark.

**Evidence:** FMT retains more final tracks on 300 m and cross-range, but truth is absent. Its reported smoothness is worse on both datasets where the baseline statistic is defined, residual errors are large, and sign/aging/birth differences confound the comparison.

**Conclusion:** No improvement claim is justified. The experiment should be repeated only after tracker semantics and evaluation are fixed.

### H9 — A Grassmann-manifold background or trajectory representation improves the system

**Status:** untested and partly misnamed.

**Evidence:** `GrassmannDynamicFilter` implements a per-voxel running mean with Euclidean residual/neighborhood logic. No learned subspace basis, principal angle, or geodesic update was found.

**Conclusion:** The repository does not currently test the stated manifold hypothesis.

### H10 — GM-PHD/EKF GM-PHD improves multi-target tracking

**Status:** **UNKNOWN**.

**Evidence:** Implementations exist, but no matching result artifacts or comparisons were found.

## Ideas tried

| Idea | Status | Evidence-based outcome |
|---|---|---|
| Fixed intensity filtering | TESTED indirectly | Removes the only point in the 100 m expected band; selected target/clutter intensity overlaps |
| Height thresholding | TESTED | Reduces point count sharply; truth-relative recall is only known in selected pseudo regions |
| Voxel occupancy background subtraction | TESTED | Useful after initialization; blind during initialization and vulnerable to revisits |
| 27-neighbor density suppression | TESTED | Reduces isolated points but deletes some sparse selected target returns |
| DBSCAN with singleton clusters | TESTED | Supplies candidates, but permissive singleton behavior is not accuracy-validated |
| Manual ROI initialization | IMPLEMENTED | Historical prototype; no comparative artifact |
| Automatic initialization/pending tracks | IMPLEMENTED | Present in main baseline; benchmark variant does not reproduce all semantics |
| Position + radial-velocity association | TESTED | Can maintain two selected trajectories; sign, timing, and duplicate-association issues remain |
| GM-PHD | IMPLEMENTED | Experimental result UNKNOWN |
| EKF GM-PHD with radial observation | IMPLEMENTED | Experimental result UNKNOWN |
| FMT radial/tangential tracker | TESTED | Current evidence does not show benefit |
| Trajectory-guided pseudo labels | TESTED | Valuable for failure diagnosis; not unbiased ground truth |
| GNSS-informed 100 m volume search | TESTED | Expected 3D band effectively empty; source of discrepancy unresolved |

## Ideas rejected or suspended

These decisions are evidence-bounded, not permanent prohibitions.

1. **Use high absolute Doppler as the primary motion detector — rejected as universal.** Tangential UAV motion can have near-zero radial velocity.
2. **Use intensity as a universal UAV classifier — rejected.** Target and clutter distributions overlap and reverse ordering across two sequences.
3. **Assume a stable single-frame UAV shape — rejected for current data.** The target return is too sparse and variable.
4. **Claim FMT improvement from more surviving tracks — rejected.** No ground truth, worse internal smoothness, high residuals, and confounded implementations.
5. **Call the current running-mean filter a validated Grassmann method — rejected.** The required manifold computation was not found.
6. **Treat both longitudinal final track IDs as separate targets — rejected.** Reverse analysis shows duplicate use of one physical cluster.
7. **Attribute 100 m failure to background subtraction — suspended.** The assumed raw target region is already empty; upstream pose/acquisition/alignment must be resolved.

## Evidence currently available

### Strong evidence

- Exact source implementation at the current filesystem snapshot.
- Five benchmark JSON files and hundreds of associated plots.
- Machine-readable stage counts and cached arrays for three sequences.
- Pseudo-label CSV/NPZ diagnostics for cross-range and 300 m.
- Reverse-analysis CSV/JSON and plots for cross-range and longitudinal sequences.
- A raw-band search result for 100 m.
- A documented longitudinal benchmark reproduction.

### Weak or conditional evidence

- Directory naming as a proxy for range/speed/trajectory.
- Sparse GNSS waypoints without frame synchronization or extrinsics.
- Retrospective pseudo-label identity.
- File modification times as execution dates.
- Current script hashes as identities for older result artifacts.

### Missing evidence

- Synchronized and calibrated ground truth.
- Negative scenes and representative operational sampling.
- Raw-to-PCD manifests and checksums.
- Sensor specification and velocity calibration.
- Frozen evaluation protocol and standard metrics.
- Repeat runs, uncertainty estimates, or statistical significance for algorithm comparisons.

## Research decisions implied by current evidence

1. Do not rank new trackers until candidate truth and evaluation semantics exist.
2. Prioritize candidate recall and provenance ahead of sophisticated association models.
3. Treat radial velocity as a line-of-sight measurement whose sign and timing must be calibrated.
4. Preserve motion-mode diversity: cross-range and longitudinal trajectories stress different Doppler regimes.
5. Audit initialization and revisit behavior explicitly in every future candidate-generation experiment.
6. Require one-to-one association or clearly defined multi-hypothesis semantics before using track counts.
7. Treat `FMCW_GT_Alignment_and_Benchmark_v0.md` as a calibration-gap result, not as final ground truth: interpolation availability does not imply spatial calibration validity.
8. Do not tune algorithms against the current zero-recall Benchmark v0 until an independent heading/lever-arm/time event resolves the alignment ambiguity.

### H11 — The 2026-02-05 workbook alone can provide calibration-quality per-frame truth

**Status:** not supported.

**Evidence:** Nine package intervals can be matched and interpolated, but automatic real-height-cluster alignment found zero accepted LiDAR/GNSS correspondences. Time offsets remain interval-midpoint estimates; translation is UNKNOWN; yaw is inferred from named flight geometry.

**Conclusion:** The workbook is sufficient for a provisional audit frame and for detecting gross absence of returns, but not for final localization-error or recall claims.

### H12 — `100m横飞10m/s` is primarily a downstream preprocessing failure

**Status:** the downstream-preprocessing hypothesis remains contradicted, but the v0 sensor-non-return interpretation is also contradicted by E10.

**Evidence:** The per-frame ±5 m horizontal-range/height shell contains zero raw returns, and the broader full-sequence envelope contains one intensity-8 point across 46 frames. No normal-threshold point reaches the height/background stages.

**Conclusion:** E10 independently recovers a dense, persistent LiDAR trajectory around x=105–116 m and closes it against held-out GNSS with 6.44 m trajectory RMSE. The v0 GT neighborhood missed it because pitch/translation were absent. The failure is in the v0 spatial alignment, not sensor non-return or downstream filtering.

### H13 — A shared yaw-only/translation model is sufficient for the 2026-02-05 data

**Status:** contradicted.

**Evidence:** A shared 6-DoF model reduced robust calibration cost by 36.3%. On held-out 100 m and 300 m cross flights, trajectory RMSE fell from 12.51 to 6.44 m and from 13.09 to 5.58 m. The fitted pitch is -6.5149°, and leave-one-condition-out pitch remains within -6.62° to -6.48°.

**Conclusion:** Roll/pitch, particularly pitch, must be represented. The resulting extrinsic remains provisional because yaw and translation are less stable and some sequences fail closure.

### H14 — One shared spatial extrinsic plus per-package offsets explains all nine packages

**Status:** not supported.

**Evidence:** Five cross sequences pass closure, but `300m_cross_5` fails spatially, `300m_roundtrip_10` fails Doppler, `300m_roundtrip_13` fails held-out validation, and `300m_roundtrip_5` has no eligible full trajectory. Several offset solutions are boundary-limited or multi-modal.

**Conclusion:** The data support a provisional cross-flight spatial solution, not a nine-package calibration or Benchmark v1.

### H15 — The Khosravi-style temporal layer is the main source of improvement over the current baseline

**Status:** not supported.

**Evidence:** The full B pipeline reaches 51.25% DEV recall versus zero for A, but A loses every silver target point at raw LiDAR `z>10`. Within B, disabling M-of-K raises recall to 61.25% while candidate precision remains approximately 2.10%.

**Conclusion:** Corrected coordinate handling and sparse candidate permissiveness explain the demonstrated gain. Temporal consistency reduces candidate volume but was not shown to improve the recall/precision trade-off.

### H16 — FMCW Doppler provides independent detection gain after spatial temporal consistency

**Status:** not supported on `research_dev_v0`.

**Evidence:** B/C/D detect 41/40/40 of 80 silver frames. Fixed gating reduces false candidates from 1,914 to 1,817; observability-aware gating leaves 1,840. C and D have identical recall in low/mid/high eta strata.

**Conclusion:** A small false-candidate reduction is observed, but it costs one true frame and does not constitute independent detection gain. The cross-flight set has only seven high-eta frames, so observability dependence remains UNKNOWN.

### H17 — Per-frame clustering is a bottleneck at 300 m

**Status:** supported as a DEV diagnosis.

**Evidence:** Median support is five raw points and 1.5 occupied 1 m voxels. At 300 m, minPts 1/2/3 recall is 0.90/0.30/0.00, while minPts=1 has only 1.76% positive-frame candidate precision.

**Conclusion:** There is a strong cluster-before-track recall/false-candidate trade-off. Track-Before-Detect is a justified next hypothesis, not a verified solution.

## Possible research directions

Everything in this section is **PROPOSED** or **SPECULATIVE**, not an experimental result.

### R1 — Calibrated Doppler–geometry consistency

**Hypothesis:** After calibrating timestamps and velocity sign/units, a residual between measured radial velocity and track-implied range rate can reject clutter and improve association across both tangential and longitudinal motion.

**Required test:** Frozen labeled sequences, corrected `dt`, one-to-one association, and an ablation against position-only association.

### R2 — Multi-frame weak-return accumulation before hard filtering

**Hypothesis:** A track-before-detect or temporal evidence accumulator can recover sparse/intermittent UAV returns lost by per-frame neighbor and background thresholds.

**Required test:** Point/region labels that include weak returns and false-alarm measurements on negative scenes. Avoid claiming benefit from track persistence alone.

### R3 — Background model with revisit awareness

**Hypothesis:** Decaying occupancy, velocity-conditioned occupancy, or separate short/long-term maps can reduce target suppression during path revisits without flooding the detector with static clutter.

**Required test:** The verified cross frame-30 revisit case plus static-only control scenes.

### R4 — Range- and uncertainty-adaptive candidate generation

**Hypothesis:** Range-dependent neighborhood and clustering thresholds will better preserve far, sparse returns than fixed 1 m/27-neighbor/2 m settings.

**Required test:** Calibrated target recall and false alarms across multiple ranges; parameters chosen without evaluating on the final test split.

### R5 — Trajectory-level compactness and isolation priors

**Hypothesis:** Prospective use of persistence, compactness, and isolation can improve target ranking beyond raw velocity/intensity thresholds.

**Required test:** Apply fixed features and thresholds to held-out scenes, including negative clutter, because the current evidence is retrospective.

### R6 — Probabilistic multi-target filtering

**Hypothesis:** A corrected GM-PHD/EKF or another probabilistic tracker can handle births, missed detections, and uncertainty better than the current hand-written rules.

**Required test:** First validate observation models, time steps, one-to-one/mixture semantics, and labeled multi-target data. Existing source alone is not evidence.

### R7 — Dual-sensor fusion

**Hypothesis:** Calibrated dual-LiDAR fusion can improve coverage and weak-target persistence.

**Required test:** Identify topics/devices, estimate extrinsics and clock offset, retain sensor provenance per point, and compare single- versus dual-sensor performance on synchronized truth.

## How to update this log

- Add a hypothesis before testing it, with a falsifiable expected outcome.
- Link the completed experiment entry in [`EXPERIMENT_LOG.md`](EXPERIMENT_LOG.md).
- Record whether evidence supports, contradicts, or leaves the hypothesis unresolved.
- Do not delete failed ideas; mark why they were rejected or suspended.
- Keep proposed directions clearly separate from results.
