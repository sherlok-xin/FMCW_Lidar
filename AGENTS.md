# FMCW_LIDAR repository instructions

These instructions are persistent project policy for Codex and other coding agents. They apply to the entire repository unless a more specific `AGENTS.md` exists below a subdirectory.

## Start every task from repository evidence

Before changing research code:

1. Read `docs/PROJECT_CONTEXT.md`, `docs/CURRENT_STATUS.md`, `docs/EXPERIMENT_LOG.md`, `docs/RESEARCH_LOG.md`, and `docs/TODO.md`. Read `docs/RECOVERED_CODEX_SESSION_2026-09-18.md` when the provenance of the recovered September 2026 work matters.
2. Inspect the code, configuration, data manifest, and result artifacts relevant to the requested change. Do not rely on chat history as the source of truth.
3. Check whether the working tree is version-controlled. At the 2026-09-20 audit, this directory was **not** a Git repository; do not invent commit IDs or history.
4. Preserve unrelated user changes and data. Raw bags and PCD files are research records, not disposable test fixtures.

## Evidence and status language

Use these labels consistently in documentation, reports, and experiment notes:

- **IMPLEMENTED**: code exists and can be inspected. This does not imply that it ran successfully.
- **TESTED**: a matching execution artifact or a new reproducible run exists.
- **VERIFIED RESULT**: the stated value is present in a repository artifact or was reproduced with a recorded command.
- **PROPOSED**: a concrete next step that has not yet been evaluated.
- **SPECULATIVE**: a research hypothesis or interpretation without sufficient evidence.
- **UNKNOWN**: the repository does not establish the fact.

Never turn an absence of evidence into a negative result. Never describe a pseudo-label, heuristic trajectory, dynamic point, active track, or visually plausible plot as ground truth.

## Scientific rigor and reproducibility

- Do not fabricate measurements, experimental outcomes, dataset properties, citations, or implementation status.
- Report failed, null, and ambiguous results alongside favorable ones.
- Distinguish code availability from execution evidence and internal metrics from task accuracy.
- Record the exact command, input paths, parameters, environment, output paths, date, and code identity for every important run. Until Git is initialized, use SHA-256 hashes of relevant scripts as a snapshot identity.
- Add completed runs to `docs/EXPERIMENT_LOG.md`; add scientific interpretations and decisions to `docs/RESEARCH_LOG.md`; update `docs/CURRENT_STATUS.md` when the project state changes.
- Keep generated artifacts outside existing result directories unless intentionally extending a recorded experiment. `scripts/benchmark_fmt.py` deletes `frame_plots/*.png` in its selected output directory.
- Use fixed seeds wherever randomness is introduced and record them.
- Prefer data-derived timestamps over an assumed `dt=1`; if an assumption remains, state it explicitly.
- Preserve units, coordinate frames, field names, sign conventions, and sensor/topic identities in code and documentation.

## Change discipline

- Make small, reviewable changes. Avoid large uncontrolled refactors, especially while ground truth and regression tests are incomplete.
- Establish a baseline and a verification method before altering an algorithm.
- Do not silently change default thresholds or make the Python, ROS, C++, and benchmark variants diverge further.
- Separate bug fixes from algorithm experiments so their effects can be attributed.
- Add or update tests for parsing, coordinate transforms, gating, association, track aging, and metric definitions when those areas change.
- Do not overwrite or rename raw data, bags, pseudo-labels, or prior results without explicit authorization.

## Project-specific cautions

- The project is small-UAV detection and tracking with FMCW LiDAR. The current system is a detect-then-track heuristic pipeline, not a validated production detector.
- Existing raw PCD files use named fields `x y z intensity velocity`; filtered PCD files use `x y z velocity intensity`. Read by field name, not column position.
- The filter transforms sensor coordinates as `[x, z, -y]`, documented as x-forward/y-left/z-up. Do not change this without confirming sensor calibration.
- Empirical analyses found that measured radial velocity has the opposite sign to increasing geometric range in two analyzed sequences. Treat the convention as dataset evidence, not a universal sensor specification.
- The first eight frames are commonly used for occupancy initialization. Target returns in these frames can be absorbed into the background model.
- `min_points=1` permits singleton clusters; a 30 m association gate is used in the current offline baseline. These choices can create false or ambiguous tracks.
- In the reconstructed benchmark, empty candidate frames do not age tracks, cluster reuse is not prevented, and an intensity-mask indexing bug exists. Do not treat benchmark output as definitive accuracy evidence.
- `GrassmannDynamicFilter` is currently a voxel running-mean/residual heuristic despite its name; no learned Grassmann subspace or geodesic implementation was verified.
- Pseudo-label analyses cover selected trajectories only and are unsuitable for unbiased headline accuracy claims.

## Validation expectations

Choose validation proportional to the change:

- Documentation-only: verify links, paths, artifact counts, and stated numbers against source files.
- Python edits: at minimum parse/compile affected modules, then run targeted tests or a small non-destructive dataset slice.
- ROS/C++ edits: report whether the required ROS/catkin environment exists. At the 2026-09-20 audit, `roscore` and `catkin_make` were unavailable, so source presence was not build evidence.
- Algorithm changes: compare against a frozen baseline on identical inputs and configurations. Do not claim improvement from active-track count or qualitative plots alone.

## Communication

Match the user's language. Lead with the observed outcome, then evidence and uncertainty. If a requested conclusion cannot be supported, write `UNKNOWN` and identify the artifact or experiment needed to resolve it.
