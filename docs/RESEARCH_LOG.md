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
| Lightweight soft temporal evidence over singleton candidates | TESTED | Frozen formal threshold retains only 7–8/80; a fixed-window DEV diagnostic point is promising but unvalidated |
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

### H18 — Lightweight soft temporal evidence resolves the `minPts=1` recall–clutter trade-off

**Status:** not supported by the frozen formal experiment; a narrower fixed-window operating point remains unresolved.

**Evidence:** Unaccumulated `minPts=1` detects 80/80 silver observations with 62.48 outputs per LiDAR frame. At the frozen score threshold 0.68, fixed/adaptive/adaptive+Doppler retain only 7/7/8 observations while reducing output to 1.89/1.90/1.72 per frame. In the predeclared diagnostic sweep, fixed threshold 0.55 obtains 52/80 with 17.23 outputs/frame, compared with current B at 41/80 and 23.88; this is same-DEV evidence without reliable negatives. A maximally compact first-observation candidate scores approximately 0.552, so the 0.55 point does not enforce multi-frame evidence.

**Conclusion:** Simple persistence, motion consistency, and compactness do not provide enough separation at the frozen formal operating point. The adaptive window is not an overall improvement and soft Doppler has no consistent Pareto advantage. Fixed-window threshold 0.55 may be tested prospectively, but must not be promoted from this DEV sweep to a verified result.

### H19 — Correcting the legacy coordinate frame is sufficient to create a competitive baseline

**Status:** supported as a DEV diagnostic, subject to raw-voxel approximation and non-independent labels.

**Evidence:** A′ reruns legacy background/neighborhood/DBSCAN after provisional leveling and reaches the same 41/80 recall as B while emitting 5.98 rather than 23.88 candidates per LiDAR frame. Survival falls from 1,883 target-neighborhood raw points/80 frames at input to 1,032/41 after neighborhood, exposing initialization/background/neighborhood losses that were hidden by the historical raw-coordinate `z>10` failure.

**Conclusion:** Future method comparisons must include A′ or a raw-point equivalent. The result does not establish final accuracy because the calibration and labels are provisional and center self-consistency is circular.

### H20 — A closer published-structure reconstruction is equivalent to project-adapted B

**Status:** contradicted on the frozen DEV set; operational superiority remains unresolved.

**Evidence:** The independent reconstruction obtains 58/80 silver recall and 26.39 outputs/frame versus B at 41/80 and 23.88. At 100 m it is slightly worse, while at 200/300 m it is substantially more permissive. The paper omits several required numeric parameters and no reported configuration combines range adaptation with Layer 3.

**Conclusion:** B must not be described as an exact Khosravi reproduction. The new reconstruction is materially different, but the observed recall increase cannot be isolated to one paper component and is not precision evidence.

### H21 — Global singleton-path DP resolves the recall–clutter conflict

**Status:** contradicted for the frozen minimal candidate-graph formulation.

**Evidence:** All 80 silver observations exist in the DP input, but the single maximum-score paths detect 0/80. Four selected paths are nearly stationary at approximately 204 m; one is near 447 m. Volume falls from 62.48 to 1.00 output/frame by selecting persistent clutter rather than the UAV.

**Conclusion:** Persistence, compactness, and upper-bound kinematics are insufficient to rank the target above static structured clutter. This negative result does not rule out full point-level TBD, but it removes the current minimal score from consideration without same-DEV retuning.

### H22 — The E13 DP fails because the UAV path is absent from the graph

**Status:** contradicted.

**Evidence:** Under the exact frozen graph and constraints, a label-guided feasible path reaches every silver observation in all five sequences. Exact state-winner ranks are 4 and 9 in two bags; lower bounds are 259, 21, and 466 in the others. Replayed top paths exactly match E13 outputs.

**Conclusion:** The failure occurs at scoring/ranking, not candidate generation or graph feasibility. Positive per-node accumulation rewards full-bag persistent clutter. Compactness and the saturated support component do not distinguish the selected paths, while several unscored quantities do. These post-hoc differences remain diagnostic rather than a basis for method selection.

### H23 — Range reduction yields stable point-count growth and motivates a larger far-range epsilon

**Status:** mixed descriptive support; stable/causal law not established (E15, 2026-10-10).

**Evidence:** Same-date observed cross medians39.5/17.5/5 raw returns at100/200/300 m. Legacy approach324→195 m has count rho=-0.695, but only9/19 intervals increase and9 decrease. Unique-xyz NN medians rise0.023/0.040/0.060 m, yet pairwise/MST/extent do not share a monotonic law; longitudinal MST rho=+0.063. Six Feb frames have 1→3 m radius-dependent long edges of UNKNOWN point identity.

**Conclusion:** Support attenuation is more directly evidenced than a full-cluster connectivity law. Raw counts, occupied cells and extent of voxel means must not be conflated. Larger epsilon, adaptive minPts and adaptive-window performance remain UNKNOWN; no parameters or labels changed.

### H24 — Target spacing range dependence is distinct from surrounding clutter

**Status:** not established; surrounding sampling changes too (E15).

**Evidence:** The 3–30 m raw clutter NN rises0.027/0.050/0.062 m across Feb groups and has longitudinal rho=+0.755. The 3–15 m NN is defined in only3/10 far-cross and1/20 longitudinal frames. No independent clutter point labels or operational merge experiment exist.

**Conclusion:** Range-dependent spacing is not target-specific evidence. Merge frequency remains UNKNOWN. Preserve the temporal/DP negative results; physical sparse-support motivation is not proof of adaptive temporal gain. Historical approach “3→18 target points” is unsupported by current raw/old cluster records (1–13); exact original attribution UNKNOWN, frame15 target3/dynamic18 is only a possible confusion example.

### H25 — Existing range-adaptive epsilon improves candidate recall by repairing UAV fragmentation

**Status:** not supported on the frozen E16 fine-cache DEV/SILVER comparison (2026-10-10); independent point-level UAV fragmentation UNKNOWN.

**Evidence:** fixed0.6 and existing scene-mean adaptive~1.678 m have identical71/80 silver matches,0 gains/0 losses.3 m proxy connectivity65→76/80 (1 m70→80) improves without recall recovery;14/15 initially fragmented3 m proxies were already matched. Candidate volume111.97→36.02/frame reflects merging and geometry, including19184 accepted fixed components entering adaptive rejected large components. Newly-large390 minus coalesced-old398 gives net−8, not evidence of no merge risk.

**Conclusion / decision:** Larger connectivity scale changes scene components but does not recover new silver frames here; do not label the observation target-fragmentation treatment or precision improvement. This is one global scene-mean implementation, not a falsification of every target-range-adaptive DBSCAN. Parameters and formal methods remain frozen.

### H26 — Apparent singleton sparsity is primarily physical non-return rather than representation/geometry

**Status:** contradicted as a blanket interpretation by E16 support accounting; relative full-detector performance UNKNOWN.

**Evidence:** On the same filtered proxy points,4 cm→1 m occupied support medians27.5→2,14→2,5→1.5; <2-cell frames1→31/80. Actual old1 m and same-fine coarsening neighborhood counts agree in this set, but their construction order differs generally.8of9 fine-stage misses exceed frozen max32; the remaining single-voxel frame is dropped by DBSCANminPts2 or geometricnmin2 even at minPts1.

**Conclusion / decision:** Coarse voxel collapse and representation-dependent geometric count limits must be separated from real raw non-return. Epsilon is not the observed recall bottleneck here. No voxel/geometry/minPts default chosen or changed, and no complete coarse/fine performance experiment inferred from cell counts.

### H27 — A fixed voxel-count interval describes an invariant physical UAV size

**Status:** contradicted as a physical-size interpretation on E17 observed DEV/SILVER support (2026-10-10), not a validation of alternative rejection rules.

**Evidence:** Raw/fine-cell group-median ratios7.9/5.5 across100/200/300m; samechildren1m representation produces31singlecell frames,30with multiple raw returns.8of9existing geometry misses exceed32fine voxels while fixed saved component max-axis0.268–0.813m overlaps near hits; adaptive frame20 becomes1.583m but still only fails max32. Far singleton is removed by minimum2.100m_cross_5/frame30 has52raw/36fine/1coarse but raw diameter0.382m.

**Conclusion / decision:** Cell count measures sampled/represented support, not body dimensions. Preserve the count-rule miss record; no new interval, representation, parameter or method selected.

### H28 — One observed single-frame metric supports a cross-range UAV-specific hard rejection

**Status:** NOT VERIFIED by current evidence; no validated feature established. This is not a theorem excluding every conceivable fixed constraint.

**Evidence:** Raw diameter/max-axis group-median ratios1.79/1.92 are smaller than count ratios, but x-span/range thickness ratios4.88/3.73, sparse singleton degeneracy, mean-cell shape collapse and1/3m peripheral sensitivity remain. NN/MST/intensity also depend on sampling/range/selection; near hits/misses overlap. All80samples are retrospective observed positives, not independent point/body or negative GT. Radius3m'sdiameter≤6m is tautological, not safe-bound evidence.

**Conclusion / decision:** True dimensions/attitude, point membership, negative and held-out validity remain UNKNOWN. Do not promote near-constant medians or positive-support envelopes to thresholds, and do not tune on these observations. User's final descriptive diagnostic is complete; no further algorithm experiment initiated.

### H29 — Causal background-relative evidence and motion coherence improve fixed UAV path ranking

**Status before the requested E18 run (2026-10-10): PROPOSED; no real-data scores evaluated yet.** This new user request authorizes one isolated MVP after E17's completed read-only stop, not changes to historical baselines or labels.

**Falsifiable test:** Freeze one Beta(1,1)-smoothed prior-frame occupancy contrast and real-dt speed/direction transition penalty; compare old/new scoring on identical frozen graphs. Primary uses all existing E16 adaptive-minPts1 4 cm components before geometry; exact E13 1 m graph is a separate control. No Doppler, DBSCAN/geometry tuning, hard count-size rejection or silver-derived weight selection. Silver is opened only after unconditioned outputs exist, to fix old-score feasibility witnesses and rescore the same paths. All-path combinatorial rank remains UNKNOWN.

**Prospective stop:** Require rank-surrogate reduction and same-span score-margin improvement in at least4/5 sequences each; stop if old reference or new near-static zero-hit output dominates in at least3/5, zero observations are recovered, or complete evaluable witnesses cannot be established. Preserve failures and do not stack features or retune.

**Completed E18 evidence:** One frozen run passed the prospective gate. Same-fine-graph unconditioned hits56/80→79/80, fixed-witness rank and matched-window margin each improve4/5; historical control0/80→79/80, allfive original persistent-clutter paths demoted. Background terms account for most of these historical fixed-path score losses; this is decomposition, not a separate background-only performance ablation. Full80-observation feasible witnesses exist in both families;124protected files/source identity remain unchanged.

**Null/negative evidence:** fine100m_cross_5alreadyrank1/margin0 stays unchanged; fine200m_cross_10fixed margin remains−0.034;300m fixed-witness new rank only≥8. Both new winners miss200m_cross_5/frame21(~3.254m at unchanged3m gate). Fine old winners are not all clutter (56silver hits), and new unobserved path nodes are not validated target points.

**Conclusion / bounded decision:** Supported on this retrospective DEV graph for the combined score, not independent GT/negative/held-out performance, operational precision or each feature's independent value. Gate passed, so kill not triggered; end this one experiment without further feature stacking, weight changes, label edits or formal-baseline adoption. Global combinatorial rank and separate background/motion/Doppler value remain UNKNOWN. See E18 in `EXPERIMENT_LOG.md` and the independent report.

### H30 — E18 static demotion, winner recovery and slow-target safety are different mechanisms

**2026-10-11 status:** TESTED / VERIFIED RESULT for term ablations on frozen `a53702d` E18 graphs, not independent target/negative/hover generalization. See `FMCW_E18_Mechanism_Generalization_Audit.md`, E19 and its independent result directory.

**Evidence:** Original/background-only/motion-only/combined hits are fine56/56/0/79 of80 and historical0/56/0/79. Fixed historical persistent-clutter score losses are background-dominated, yet background alone does not recover100m_cross_10 or300m_cross_10. Motion alone favors low-motion static-like paths and removes fine old hits. With background, motion gives net23hits (24gain/1loss); this is computational interaction, not an additive percentage attribution of real-world effect. Same graphs/witnesses/gates; no weights or labels changed.

**Counterevidence/slow stress:** All5maximal shortslow cross-silver windows/22obs per family yield14/12/14/12hits.200m_cross_5/frames7–12 original/motion6/6→background/E180/6; witness B+2.539 stays positive but a competing winner has stronger novelty. Positive node terms do not imply target-safe ranking. Context-limited subwindow outputs differ from full offline DP, while full prior occupancy is preserved.

**Conditional mechanism, not measured hover failure:** b=(F−2H)/(F+2) penalizes cells occupied in over half of prior frames; ideal continuous arrivals at f0 give b=(2f0−f)/(f+2). Persistent same-cell hover/revisit has a structural loss-of-novelty risk. Motion penalties can be zero for static and constant-velocity paths, so do not encode UAV identity or protect hover. Actual slow windows have positive witness b throughout, not observed negative-score deletion. Long-duration, independently identified hover performance UNKNOWN.

**Evidence limit:**14ZIP/51bag members/6standalone paths do not provide reliable target absence; negative run not executed and FA/precision/negative winner scores UNKNOWN. Dedicated independently authenticated hover bag UNKNOWN. Existing observed slow-cross centers are not independent hover GT. Do not relabel failed closure, zero returns, outside-silver frames or selected static-looking paths as negatives.

**Decision:** End the requested mechanism/generalization audit without tuning, fixingframe21, Doppler or new features. Combined E18 remains a frozen DEV result, not a validated operational detector. Independent negative/hover evidence is required to answer unresolved safety/generalization claims; this requirement is not authorization to launch another experiment.

### H31 — Existing independent records can certify negative and moving→hover windows

**2026-10-11 status:** NOT ESTABLISHED / DATA_GAP after E20 read-only existing-evidence audit. See `FMCW_Negative_Hover_Evidence_Audit.md`. No new E18 solver run or algorithm change.

**Evidence:**14ZIP/51bounded bag topic probes,6standalone final indexes,completeExcel,photoEXIF,19source entries andpath-level Git history yield no independently certified absence/hover window. Photo precedesfirstFebbag21m19.386s; minimumGNSS endpointchord speed0.996m/s and device-rowduplicate do not establish flightstate. Three continuous silver moving→near-static shapes are observed-center descriptors sharing LiDAR-derived identity/selection; they cannot independently validate E18.

**Boundary:** stored200m_cross_5/7–12 novelty falls0.777778→0.142857, augmentednodesstaypositive; supports repeated-cell mechanismrisk, not new true-hover/systematicdegradation. Archivefooter/fullcloud notdecoded; outside-repositoryrecordsUNKNOWN. Missinglabels/returns andfailedclosure are notnegative.

**Decision:** E18a53702d remains fullyfrozen; independent negative/slow-hover stress NOT_RUN_DATA_GAP. No threshold, Doppler, weights, features, missfix orgraph expansion. Authentic timedidentity/state/visibility records are needed before evaluation advances; thislog authorizes neither newacquisition noralgorithm.

## Possible research directions

Everything in this section is **PROPOSED** or **SPECULATIVE**, not an experimental result.

### R1 — Calibrated Doppler–geometry consistency

**Hypothesis:** After calibrating timestamps and velocity sign/units, a residual between measured radial velocity and track-implied range rate can reject clutter and improve association across both tangential and longitudinal motion.

**Required test:** Frozen labeled sequences, corrected `dt`, one-to-one association, and an ablation against position-only association.

### R2 — Multi-frame weak-return accumulation before hard filtering

**Hypothesis:** A track-before-detect or temporal evidence accumulator can recover sparse/intermittent UAV returns lost by per-frame neighbor and background thresholds.

**Current evidence:** E12 tested one lightweight candidate-path accumulator. Its frozen formal operating point failed, while a lower fixed-window DEV threshold was only diagnostically promising. This does not test a complete DP-TBD formulation.

**Required test:** Point/region labels that include weak returns, false-alarm measurements on negative scenes, and a held-out prospective evaluation of one frozen operating point. Avoid claiming benefit from track persistence alone.

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

## 2026-10-12 — Oracle-conditioned phenotype preparation (not a method result)

**VERIFIED RESULT, metadata-only:** existing GNSS/closure sensitivity replay covers 272 cached frames, 144 nominal GNSS-supported frames and 80 observed anchors. Only 64/80 anchors lie inside the existing scenario union of 3 m balls. Nominal anchor/GNSS discrepancy reaches 26.575 m. These scenarios must not be promoted into calibrated uncertainty bounds, and small-neighborhood emptiness cannot establish physical UAV non-return.

**Decision:** keep all-frame GNSS occupancy and selected observed-anchor phenotypes separate. Preserve uncertain/failed sequences outside the cross cohort and retain E20 DATA_GAP. Coarse endpoint chords, low radial observability and unknown body attitude do not justify instantaneous-state, hover or aspect conclusions.

**Unresolved:** conditional target support, feature separation and Doppler residual structure await full rich extraction. The first attempt failed on output serialization, not a sensing phenomenon; an export-only fix passed tests. Retry awaits user confirmation under the academic-research-suite workflow. No E18 change, fitted weight, new detector or independent evaluation evidence was produced. [Checkpoint report](../FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md).

### Approved run_02 completion — 2026-10-12

The preparatory entry above records the earlier pause; the user subsequently approved the export-only retry. Full raw extraction and independent source/statistical checks now complete in `results/gnss_conditioned_uav_phenotype/run_02`.

**VERIFIED RESULT:** 103/144 GNSS 3 m regions are empty, compared with nonempty neighborhoods in all 80 previously localized observations. Existing uncertainty union covers only 64/80 centers. These results contradict treating a small GNSS tube as exact point GT or calibrated physical non-return evidence; they do not show a sensor missed-detection rate.

**VERIFIED RESULT:** identical anchor children give 31 coarse singleton-cell frames but only one unique-XYZ singleton. Acceleration/evenness fields are constant; velocity-direction repeats scalar sign. Coordinate-coincident rows differ in time/intensity and are not literal byte copies. Representation/field naming can create misleading physical interpretations.

**Negative/ambiguous evidence retained:** 311/318 small matched anchor controls are empty. High count AUC mainly contrasts selected return-bearing neighborhoods with empty space; reliable occupied-clutter discrimination is not established. Broad-region intensity/geometry/Doppler contrasts vary or reverse across range/recipe. Within-sequence support/spacing correlations do not consistently follow between-range trends. All nominal states remain low-eta cross motion; hover/high-eta behavior is UNKNOWN.

**SPECULATIVE directions only:** range/exposure/uncertainty-conditioned observation descriptions and representation-aware support may be worth held-out falsification. Current data do not identify node likelihood ratios, physical `P_D`, transition acceleration priors or valid uncertainty bounds. No formula, weight, cutoff, detector, tracker or E18 feature was chosen. Required tests are independently synchronized dense state/point identity, occupied-clutter/negative controls, radial/cross transitions and true hover under immutable held-out splits. [Report's final observation/mechanism/falsification table](../FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md).


## 2026-10-13 — GNSS–anchor discrepancy decomposition（无重标定）

**VERIFIED RESULT：** 在已有五个 closure-PASS sequence 的 80 个 observed DEV/SILVER anchors 上，使用 frozen ENU-Up 和原 GNSS chord 构建正交分量。各 sequence 的 along-track 平方误差占比约 95.5%–98.7%。26.575 m 的 `200m_cross_5/frame21` 由 +26.389 m along-track 主导（98.60%），不是 vertical 主导；cross −3.056 m、vertical-normal +0.719 m、严格 ENU-Up +0.629 m。

**模式判断，非因果确认：** 恒定 time-offset residual 会在同一线性 chord 内表现为近恒定的 `d_parallel/v_chord`；现有序列在同一 chord 内仍有符号和幅度变化，所以该单因解释不足。frame17→21 的连续同候选误差增长更符合未解决的时间/航段对应和稀疏 GNSS chord 的速度/转弯表达问题，弱化孤立跳点解释。终点 frame21 直接对应 A70，不存在“段内插值权重误差”这一数学项，但 GNSS 采样、时间对应和参考点误差仍 UNKNOWN。+6.708 s 的局部等效时间超出支持，不能作为合法 clock correction。

**保留空间可能性：** vertical-normal 80/80 为正，严格 Up 79/80 为正，量级较小；`100m_cross_10` 的 fixed-frame 方向 resultant 约 0.788，说明该序列存在较一致的全局误差方向。空间/参考点残差可共存；没有拟合或排除完整 extrinsic residual，也不能排除连续错误航段关联。没有在这些 labels 上优化任何 clock/extrinsic。

**决定：** 把数学定义、signed/absolute summaries、方向一致性、支持边界、原材料复制 hash 和独立审计存入 `results/rviz_gnss_lidar_sanity/`。保持 E18、标定、offset、GNSS 插值、labels 和历史结果。唯一因果归因需要独立同步事件、较密 GNSS/RTK、明确天线/点云参考点或外部飞行记录；本轮未取得或生成这些证据，未自动升级 GT。


## 2026-10-13 — Scene-relative background 的描述证据与反证

**VERIFIED RESULT：** 既有五cross场景在1m尺度高度重复；同fine输入4cm却低persistence，说明occupancy strongly scale-sensitive，不能把fine novelty直接当运动或UAV。E18fine background已覆盖大量重复结构，但只保留完整raw support约6.21%，其“干净”场景不等同物理自由空间。

**反证保留：** 100m_cross_10的raw1m同anchor帧，anchor距persistent scene反而比BGonly路径近（paired median−2.763m），而fine1m反向；persistence cutoff.25/.5/.75在raw也有符号改变。300m同帧+14.091m且固定敏感性同方向。不能用这轮结果选择fine输入/cutoff或设“越远越UAV”的score。

**非冗余，非检测效用：** rawscene距离与既有E18occupancy rho−.812至−.830，但同帧同H/F仍有距离差异；这证明非代数重复而非类别独立信息。既有300m BGonly存在低occupancy近结构连续节点片段，但大反向/步长和缺identity证据阻止把连线认证为一个coherent物体。

**决定：** 维持描述性/hypothesis-generation边界；独立身份/targetabsence/occupiedclutter、scanexposure、moving-hover和不同scene/date heldout证据到位前，不改E18、不加feature、不选score或gate。80anchors为selectedDEV_SILVER，地图在feature完成后才join；4coldstartdistanceUNKNOWN与100m反例保留。


## 2026-10-13 — Occupancy temporal structure：局部非冗余，跨五包证据不足

**VERIFIED RESULT：** 固定同frame/H/F下，recency/run/episodes/前5帧recent仍有数值展开，不是E18 background term1−2p的确定函数；H0/H1时run/episodes退化。对既有selectedanchor与competitor的主比较，100m_cross_10只有3个非退化帧、300m只有1帧，其余三包没有不同cell共同支持。窄frequency bins的方向有混杂和反向，不能替代同帧exactH主检验。

**SPECULATIVE局部解释：** 100m frame22–24与300m frame10的anchor-associated过去占用更近期，可能反映局部时间排列；同表面扫描、不同对象混occupancy、DEV选择或路径co-association也兼容。单cell history不认证同一对象，4个局部帧不能解释jointE18完整路径胜出；未rescore或干预。

**决定：strict kill触发。** 当前证据层面E18occupancy足够，不提出E21backgroundmodel、新score或feature组合。该表述是研究证据门槛而非数学充分性证明，也不把共同支持缺失当零效应。需要独立identity/presence、occupied动态clutter/negative、moving-hover/revisit和不同scene/date heldout数据，在同H/F有非退化共同支持情况下证伪；保持唯一5priorframes定义，不用当前silver选择窗口/阈值。
