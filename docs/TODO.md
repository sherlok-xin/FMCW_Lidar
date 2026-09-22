# Prioritized TODO

Status date: **2026-09-20**

The ordering below reflects dependency and scientific value. Algorithm novelty is intentionally placed after provenance, labels, calibration, and evaluation because the current evidence cannot otherwise distinguish improvement from additional false tracks or implementation drift.

## P0 — Establish an auditable baseline

### 2026-09-22 progress note

- Package manifest and non-extrapolated per-frame interpolation: **IMPLEMENTED and TESTED**, but temporal confidence is LOW.
- Initial Benchmark v0 and state-separated evaluator: **IMPLEMENTED and TESTED**, but spatial calibration is not validated.
- Independent full-space LiDAR trajectory search and shared-extrinsic closure: **IMPLEMENTED and TESTED**. Five cross sequences pass provisional closure; pitch near -6.5° is stable across leave-one-out fits.
- Remaining P0 blocker: resolve per-package time aliases/boundary solutions and validate the provisional 6-DoF pose against an independent common time event or surveyed target. Do not promote the provisional CSV coordinates to Benchmark v1 before this gate.
- Exclude `300m_roundtrip_5`, `300m_roundtrip_13`, and `300m_cross_5` from quantitative algorithm ranking; use `300m_roundtrip_10` only as a spatial diagnostic until its Doppler inconsistency is resolved.

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

**PROPOSED HYPOTHESIS:** accumulating sub-threshold evidence before hard clustering can improve far-range recall. Evaluate false alarms on negative scenes and do not compare using active-track count alone.

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
