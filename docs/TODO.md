# Prioritized TODO

## Binding decision — 2026-10-14: STOP_DEV_FEATURE_MINING

**External reproduction update (2026-10-14):** the first Stone Soup v1.9.1 task reached a legal empty-scan posterior mismatch after 7 official component tests passed. [Report](../FMCW_STONESOUP_BERNOULLI_REPRODUCTION.md). Stop before multi-frame A/B/C; no upstream patch, version substitution or FMCW hookup is authorized. The first-task recommendation below is the preserved pre-execution roadmap, not a request to repeat or extend the stopped experiment. Report and evidence remain uncommitted for user review.

The existing five cross-flight DEV bags and 80 DEV_SILVER anchors are closed to proposing/tuning E18 background features, weights, windows, thresholds, or E21/E22 scores. Preserve E18 and all frozen scientific inputs/results. This user-directed decision overrides any older feature-exploration suggestions retained below; external reproduction does not reopen DEV. See [decision and exact scope](DEV_FEATURE_MINING_STOP.md).

The completed occupancy temporal audit was committed and pushed unchanged at `92cf79d01bbb84eee52dde7fd22210f78067d6cd`. The [external roadmap](../FMCW_EXTERNAL_REPRODUCTION_ROADMAP.md) covers exactly Stone Soup, DopplerTrack and official UnLiDAR. **Only first task recommended:** Stone Soup Bernoulli component tests followed, if gates pass, by a minimal synthetic-clutter existence example. This remains a plan: no implementation, installation, method execution or DEV hookup was performed in this roadmap task.

## Historical priorities (preserved)

Status date: **2026-10-11**

The ordering below reflects dependency and scientific value. Algorithm novelty is intentionally placed after provenance, labels, calibration, and evaluation because the current evidence cannot otherwise distinguish improvement from additional false tracks or implementation drift.

## P0 — Establish an auditable baseline

### 2026-10-11 E20 evidence search — complete, DATA_GAP

- Preserve `FMCW_Negative_Hover_Evidence_Audit.md` / `results/negative_hover_evidence_audit/`, rejected candidates, unknowns and inventory-only failures. E19 is onGitHubmain at5ad38b9; E18a53702d fullyfrozen,302protectedfiles unchanged.
- No independently timestamp-linked negative/hover/slow window qualifies. NewstressNOT_RUN_DATA_GAP; emptywinner/hoverCSV mean unmeasured, notzero. Topics, deviceorigin,photooutsidebagtime, GNSSchords orsilver-gaps do not certify absence/hover.
- Keep all75silversteps/10continuousfragments andpriorE19witnesscurves asDEVmechanism evidence, notphysicalhoverGT. Do not tuneE18 orconstructnewgraphs fromthese candidates.
- Next evidence dependency: authentic existing flight-state/RTK/video/operator records with targetidentity, interval, clockmapping, visibility/FOV and localizationuncertainty. Outside-checkout availability UNKNOWN; newacquisition/evaluation requires a separate scopedrequest.
- Until then, negativewinnerstats,operationalprecision/FA andindependent moving→hover degradation remainUNKNOWN. Stopalgorithmwork;noDoppler/weights/thresholds/remainingmissfix.

### 2026-10-11 E18 mechanism/generalization audit — complete; retain freeze

- Preserve `a53702d` E18 formulas/weights/graphs/labels/3m gate and `FMCW_E18_Mechanism_Generalization_Audit.md` / `results/e18_mechanism_generalization_audit/`; this scoped audit does not authorize a new method, threshold or remaining-miss fix.
- Four arms on same saved graph: fine56/56/0/79 of80, historical0/56/0/79. Keep motion-only regression, background-only failures and24gain/1loss interaction; static-reference demotion is not sufficient detection recovery.
- Keep ALL5shortslow windows/22obs per family and14/12/14/12result.200m_cross_5/frames7–12 loss6/6→0/6 is relative novelty competition with positive witness background, not hard deletion or proof of true-hover systemic failure.
- No reliable negative or independent hover certification was found in existing artifacts. Negative scores/false alarms/precision remain UNKNOWN and negative CSV is header-only, not zero. Never use outside-silver/no-return/failed-closure frames as negatives.
- Data/evidence blocker remains independently verified target-absence/long-hover windows tied to bag timestamps and independent GT/held-out sequences. Any new acquisition/evaluation requires a separate scoped request; do not infer authorization from this TODO.
- 12new+14E18tests/212protected hashes/20exactfullpool replays pass. Keep metadata-only run-manifest reconciliation and new-arm span-rank independent-verification limits.
- Stop at audit handoff; no feature stacking, weight tuning, Doppler, baseline adoption or Benchmark v1 upgrade.

### 2026-10-10 one frozen background/motion DP MVP — complete

- E18 is the new explicitly authorized experiment after E17's stop. Preserve `FMCW_Background_Motion_DP_MVP.md`, all graph/path/rank/score artifacts and `results/background_motion_dp_mvp/`; do not alter old baseline, labels, caches or weights.
- Primary score-only same-fine-graph comparison56/80→79/80; historical exact-graph control0/80→79/80. Predeclared gate passed, but this is not permission for feature stacking, tuning or automatic next experiments.
- Keep the one200m_cross_5/frame21 miss (~3.254m), rank1→1null, fine200m margin stillnegative and300m witness rank≥8; do not widen the3m gate or choose a new silver oracle.
- Fine graph old winners already include56silver hits: never call every old reference persistent static clutter or count cross-family candidate/geometry differences as score gain.
- Global combinatorial rank, operational precision/false alarms, target-absent behavior, independent held-out performance and background-versus-motion independent gains remain UNKNOWN. Existing independent point/negative/GT gates still apply; execution needs a new scoped request.
- Stop work at this frozen MVP handoff. No formal default change, Benchmark v1 upgrade or paper headline claim is justified by DEV recall alone.

### 2026-10-10 final geometry invariance diagnostic — complete

- E17 complete: preserve `FMCW_Geometry_Invariance_Audit.md` and `results/geometry_invariance_audit/`, all80observations/9misses and old1/3m sensitivity.100protected files unchanged; no algorithm/parameter/label/cache edits.
- Do not equate cell count with UAV dimensions, singleton centroid extent0 with a zero-size UAV, or near-constant group medians with per-frame invariance.
- No validated UAV-specific single-frame geometry hard rejection emerged. Record this evidence limit, not a new threshold proposal or a universal impossibility claim.
- Nearest saved component is only a descriptor lookup; frame hit is existence of an accepted match within the old gate. Keep these separate.
- This was the user's final read-only diagnostic. Stop new diagnostic/algorithm experimentation here; existing independent-label/negative/held-out gates remain, but their execution requires a new user request.

### 2026-10-10 DBSCAN attribution diagnostic note

- E16 is complete: preserve `FMCW_DBSCAN_Fragmentation_Attribution.md` and `results/dbscan_fragmentation_attribution/`; no selected/default parameter changes.
- Do not claim adaptive fragmentation recovery: fixed/adaptive both71/80, despite increased proxy connectivity and fewer candidate outputs.
- Keep geometric nmin2 separate from DBSCANminPts;8 fine-stage misses are max32 and1 is singleton. Record representation-dependent cell support, not just raw point count.
- Preserve same-input4 cm→1 m collapse1→31 frames below2cells as descriptive evidence, not an authorized joint parameter/voxel sweep.
- Operational precision and target/clutter merge identity still require independently defined point/object/negative labels and held-out evaluation. No further algorithm work was authorized by E16.

### 2026-10-10 range/sparsity diagnostic note

- E15 raw audit is complete; preserve `FMCW_Range_Sparsity_Audit.md` and `results/range_sparsity_audit/` as descriptive evidence, not an adaptive-parameter selection run.
- Keep raw return counts, occupied-voxel counts, whole dynamic counts and ROI counts separate. The old approach 3→18 claim source is UNKNOWN; verified target counts are1–13.
- Before general range-adaptive claims, independently label weak/missed observations and clutter, replicate an approach sequence, and establish sensor angular/beam metadata. Do not tune epsilon/minPts/window on E15 neighborhoods.
- No detector/tracker change was authorized in E15. Prior frozen temporal/DP negative results and the independent-negative/held-out evaluation gate remain unchanged.

### 2026-09-22 progress note

- Package manifest and non-extrapolated per-frame interpolation: **IMPLEMENTED and TESTED**, but temporal confidence is LOW.
- Initial Benchmark v0 and state-separated evaluator: **IMPLEMENTED and TESTED**, but spatial calibration is not validated.
- Independent full-space LiDAR trajectory search and shared-extrinsic closure: **IMPLEMENTED and TESTED**. Five cross sequences pass provisional closure; pitch near -6.5° is stable across leave-one-out fits.
- Remaining P0 blocker: resolve per-package time aliases/boundary solutions and validate the provisional 6-DoF pose against an independent common time event or surveyed target. Do not promote the provisional CSV coordinates to Benchmark v1 before this gate.
- Exclude `300m_roundtrip_5`, `300m_roundtrip_13`, and `300m_cross_5` from quantitative algorithm ranking; use `300m_roundtrip_10` only as a spatial diagnostic until its Doppler inconsistency is resolved.
- `research_dev_v0` is now frozen and tested on five cross sequences. Keep it development-only: the labels are LiDAR-derived silver trajectories and may not be used for final paper claims.
- Before any parameter revision, add independently labeled negative/static-clutter frames and at least one high-radial-observability longitudinal sequence. The current positive-frame precision and eta analysis cannot validate operational false alarms or Doppler dependence.
- Preserve the current no-gain Doppler result. Do not retune Doppler gates on these 80 frames and then report the same set as held-out evidence.

### 2026-09-29 temporal-evidence progress note

- Corrected-coordinate legacy A′ and the lightweight singleton evidence accumulator are **IMPLEMENTED and TESTED** on the frozen DEV set; see `FMCW_Temporal_Evidence_MVP.md` and `results/temporal_evidence_mvp/`.
- The formal soft accumulator result is negative: fixed/adaptive/adaptive+Doppler recover only 7/7/8 of 80 silver observations at the frozen threshold.
- Do not change the formal threshold after seeing E12. Fixed threshold 0.55 is only a candidate for a new prospectively frozen experiment because it was identified on the same DEV set.
- Highest-priority data task: identify and label truly target-absent/static windows. Until then, false alarms/frame, operational precision, and PR curves remain UNKNOWN.
- Reproduce A′ directly from raw points before treating its 1 m raw-voxel approximation as the authoritative corrected legacy baseline.
- Validate one fixed-window operating point on held-out data before adding model complexity. Do not proceed to full DP-TBD, deep learning, or manifold work on the strength of this DEV sweep.

### 2026-09-29 published-baseline / DP progress note

- The independent published-structure reconstruction and minimal candidate-graph DP are **IMPLEMENTED and TESTED**; see `FMCW_Published_Baseline_and_DP_TBD_MVP.md`.
- Preserve `parameter_provenance.csv`: the public paper leaves multiple thresholds `UNKNOWN`, so do not relabel this implementation as an exact reproduction.
- Do not retune the failed DP node score or motion limits on `research_dev_v0`. Its 0/80 result and stationary-clutter failure are part of the record.
- Before any richer graph/TBD or Doppler ablation, acquire reliable target-absent windows and create a held-out sequence split with independently defined labels.
- If the published reconstruction is studied further, isolate 4 cm voxelization, DBSCAN schedule, centroid rule, and M-of-K semantics one at a time under a newly frozen protocol; do not infer causality from E13's bundled difference.

### 2026-09-29 DP failure-forensics progress note

- The E13 graph contains a fully silver-consistent feasible UAV path in every sequence. Do not attribute the DP failure to missing singleton evidence or graph pruning.
- Preserve the diagnostic rank result: exact ranks 4/9 in two sequences and lower bounds 259/21/466 in the others. Exact rank among all raw paths is `UNKNOWN` because the original Viterbi stores only one winner per edge state.
- Do not tune score weights or gap/kinematic parameters on these paths. The current evidence explains the failure but does not authorize a new operating point.
- Treat occupancy/revisit, speed, curvature, Doppler magnitude, and raw support as post-hoc selected-path diagnostics only. Their general separation must be tested on independently labeled target and negative clutter data.

### 1. Put the project under version control

**Deliverable:** initialize Git, add a data-aware `.gitignore`, and commit source/config/docs without committing the 115 GB bag archive, 4 GB PCD corpus, virtual environment, or generated caches unless intentionally managed through an external data system.

**Done when:** every future experiment can cite a commit ID and dirty/clean state.

### 2. Create a raw-to-derived data manifest

**Deliverable:** one machine-readable record per sequence containing:

- stable sequence ID and checksum inventory;
- source bag/archive, ROS topic, sensor identity, and time range;
- bag-to-PCD export command/version;
- coordinate transform and field mapping;
- intensity/height/filter parameters;
- expected and actual frame counts, with dropped-frame reasons;
- links to GNSS/scene metadata.

**Done when:** every PCD and result directory is traceable to a raw source and processing configuration. Unknown legacy lineage must remain labeled `UNKNOWN`.

### 3. Define and produce a minimal ground-truth set

**Deliverable:** synchronized per-frame target presence, 3D center/volume, identity, visibility/occlusion status, and annotation confidence for at least:

- cross-range sequence;
- longitudinal sequence;
- 300 m sequence;
- unresolved 100 m sequence;
- one negative/static-clutter sequence.

Use an explicit schema and double-check a subset independently.

**Done when:** precision/recall, false alarms, localization error, and track continuity can be computed without retrospective selection.

### 4. Resolve time, coordinate, and velocity calibration

**Deliverable:** documented sensor frame, LiDAR-to-GNSS extrinsics, clock alignment, dual-sensor mapping/extrinsics, and an empirical velocity calibration covering sign, units, ambiguity/wrapping, saturation, and uncertainty.

**Done when:** a point/track can be transformed into the truth frame, and measured radial velocity can be compared to geometric range rate using actual `dt`.

### 5. Freeze an evaluation protocol

**Deliverable:** immutable train/development/test sequence split; metric definitions; matching tolerances; initialization policy; treatment of missed/occluded frames; and a baseline command.

Minimum metrics:

- point/candidate recall at each preprocessing stage;
- object precision, recall, F1, and false alarms per frame/time;
- center/range/velocity error;
- track completeness, identity switches, true fragmentation, and an accepted MOT metric where applicable;
- latency measured with warm-up and stated hardware.

**Done when:** the same frozen evaluator scores every method and prevents tuning on the held-out test set.

## P1 — Correct and consolidate baseline behavior

These should be separate, reviewable changes with before/after regression results.

### 6. Make the benchmark match a declared baseline

- Choose whether `target_track_without_initialize_and_grid.py` or a factored library implementation is authoritative.
- Align birth, pending-candidate, confirmation, loss, and removal semantics.
- Remove the inaccurate “exact copy” claim unless equivalence is tested.
- Add a regression fixture for each of the five current sequences.

**Done when:** benchmark and authoritative baseline produce matching states/metrics on the same fixture.

### 7. Fix known correctness issues

- Apply the same point mask to intensity and velocity arrays.
- Use real timestamps and non-unit `dt`; stop integer truncation where precision is needed.
- Centralize and test the velocity sign convention.
- Age tracks on empty candidate frames.
- Enforce one-to-one cluster assignment, or explicitly implement/document a probabilistic alternative.
- Distinguish measured state from prediction-only carried metadata.
- Rename “fragmentations” to removals until truth-relative fragmentation is computed.

**Done when:** targeted unit tests reproduce each old failure and pass after its isolated fix.

### 8. Centralize configuration and units

**Deliverable:** one versioned config schema for preprocessing, clustering, association, and removal thresholds, with units and per-entry-point overrides.

**Done when:** offline, Python ROS, C++ ROS, GM-PHD, and benchmark defaults cannot silently diverge.

### 9. Add a compact automated test suite

Tests should cover:

- PCD named-field loading for both header orders;
- `[x, z, -y]` coordinate transform;
- timestamp parsing and `dt`;
- radial-velocity projection/sign;
- occupancy initialization and revisit behavior;
- neighbor filtering edge cases;
- DBSCAN singleton semantics;
- one-to-one association;
- empty-frame aging;
- metric definitions.

**Done when:** a fast local command guards the scientific baseline before experiments.

### 10. Pin the runnable environment

**Deliverable:** lock exact Python dependencies and document ROS1/C++ build requirements separately from the observed host environment.

**Done when:** a clean environment can parse and run the baseline reproduction without using the checked-in `fmcw/` virtual environment.

## P2 — Run controlled diagnostic experiments

### 11. Re-evaluate the corrected baseline first

Run the frozen baseline on every labeled split, record stage recall and object/track metrics, and write a new entry in `EXPERIMENT_LOG.md`. Preserve the old July results as historical artifacts rather than overwriting them.

### 12. Perform preprocessing ablations

Independently vary:

- initialization length and policy;
- occupancy decay/revisit handling;
- intensity and height thresholds;
- neighbor radius/count;
- voxel size;
- DBSCAN `eps` and `min_points`.

Report target recall and false alarms together. Include the cross revisit frames, sparse 300 m returns, 100 m diagnostic case, and negative scenes.

### 13. Validate Doppler–geometry consistency

After calibration, compare measured radial velocity with truth- and track-implied range rate across motion modes. Quantify residual distributions by range, intensity, and aspect. Use this to define uncertainty-aware gates rather than fixed uncalibrated thresholds.

### 14. Prospectively validate compactness/persistence/isolation cues

Freeze feature definitions and thresholds using development data, then evaluate on held-out sequences. Do not reuse the retrospectively selected cross/longitudinal tracks as the sole test.

### 15. Re-run FMT as a clean ablation

Only after P1 fixes:

- share identical candidate inputs, birth logic, association constraints, `dt`, and velocity sign;
- vary only the FMT state/prediction update;
- report truth-relative accuracy and false alarms, not only survival/smoothness;
- include uncertainty and repeated latency measurement.

**Decision gate:** retain FMT only if it improves a predeclared primary metric on held-out data.

## P3 — Research extensions after the evaluation gate

### 16. Test revisit-aware or multi-timescale background models

**PROPOSED HYPOTHESIS:** occupancy decay or separate short-/long-term maps can preserve revisiting targets while controlling static clutter.

### 17. Test multi-frame weak-return accumulation / track-before-detect

**PARTIALLY TESTED HYPOTHESIS:** E12 tested a lightweight soft path accumulator, not full DP-TBD. Its frozen formal operating point failed; a lower fixed-window DEV point remains only hypothesis-generating. Next acquire reliable negative windows and evaluate one prospectively frozen setting on held-out data. Do not compare using active-track count alone.

### 18. Evaluate probabilistic multi-target tracking

First create unit-tested observation/dynamics models. Then compare GM-PHD/EKF GM-PHD with the corrected rule baseline on labeled multi-target sequences. Existing implementations are starting points, not validated methods.

### 19. Calibrate and evaluate dual-sensor fusion

Identify both sensor streams, calibrate space/time, retain per-point sensor provenance, and compare single-sensor and fused performance under identical truth.

### 20. Decide whether a true manifold model is scientifically justified

Define the data object that lies on a manifold, the mathematical representation and distance/update, and a falsifiable benefit. Rename the current running-mean component if no genuine Grassmann formulation is implemented.

## Maintenance tasks

- Update `CURRENT_STATUS.md` after each completed milestone.
- Append every material run to `EXPERIMENT_LOG.md` before interpreting it.
- Update `RESEARCH_LOG.md` when evidence changes a hypothesis or decision.
- Remove or quarantine stale `.vscode`/`.claude` references after confirming they are unused.
- Document the unrelated `.local/` trash/GVFS content and exclude it from version control; do not delete it without user authorization.
- Replace placeholder ROS package metadata when the package owner/license/build target are confirmed.

## Recommended immediate sequence

The shortest scientifically useful path is:

```text
Git/provenance
    -> minimal synchronized labels
    -> calibration and frozen evaluator
    -> isolated baseline correctness fixes
    -> corrected baseline measurement
    -> preprocessing/Doppler ablations
    -> only then compare FMT, probabilistic, or track-before-detect methods
```

## 2026-10-12 active checkpoint

1. **DONE:** explicit user approval received; export-only retry completed in `results/gnss_conditioned_uav_phenotype/run_02`, with original failure and frozen E18 preserved.
2. **DONE:** raw-frame identity, full member/EOF hashes, 272/144/80 accounting, native byte export and independent descriptive arithmetic validated; completed phenotype report includes nulls and effect missingness.
3. **NEXT DATA REQUIREMENT, not an algorithm task:** independently synchronized position/presence/FOV, populated matched clutter, certified negatives and radial/moving→hover transitions under the existing preregistration.
4. Do not retune E18/labels/controls or estimate physical missed-detection probabilities/node likelihoods from the current oracle-region/selected-anchor data. Check the final observation→mechanism→held-out-falsification table before promoting any descriptive association.


## Current steering — 2026-10-16 project reinterpretation

The user-directed [historical pipeline audit](../FMCW_HISTORICAL_PREPROCESSING_AUDIT.md) supersedes the next-step priority of older external-roadmap text: pause Bernoulli/other external reproductions; do not continue Stone Soup or install another repo. STOP_DEV_FEATURE_MINING remains binding. Sole proposed next technical task: recover actual historical exporter/filter config/filtered-RViz chain, freeze it, then a separately authorized faithful stagewise accounting/axis validation on fixed representative ranges. Exact full pipeline currently BLOCKED; no parameter selection, E18 changes or new algorithm. Report/plan are uncommitted for review.
