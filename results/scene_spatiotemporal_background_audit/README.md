# Scene spatiotemporal background audit

**TESTED / descriptive only.** Main report: [FMCW_SCENE_SPATIOTEMPORAL_BACKGROUND_AUDIT.md](../../FMCW_SCENE_SPATIOTEMPORAL_BACKGROUND_AUDIT.md). No score, detector, graph, label, calibration, offset, threshold or raw-data changes; no operational classifier metrics. Local machine artifacts include complete causal cell tables; bags and ROS runtime files are absent.

## Execution and phase boundary

Working directory `/home/xin/FMCW_LIDAR`; Python is the already available `fmcw/bin/python` (3.10.12), NumPy 2.2.6, SciPy 1.15.3. No installation or random data sampling. Freeze Git parent/source/protocol/input hashes are in [freeze_manifest.json](freeze_manifest.json). Raw/fine input identities and scene output hashes are preserved. Historical reports were read as requested: this is prospectively frozen descriptive processing of previously studied DEV data, not a blind held-out study.

Exact commands executed, in order:

```bash
cd /home/xin/FMCW_LIDAR
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/scene_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/scene_audit.py compute
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/interpret_scene.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/supplement_summaries.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/plot_scene_audit.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/validate_scene_audit.py
```

The independent validator and plotting preparation ran in parallel with other work, after required inputs existed. `freeze`/`compute` refuse completed-output overwrite. [SCENE_FEATURES_COMPLETE.json](SCENE_FEATURES_COMPLETE.json) predates anchor/path parsing in [interpretation_manifest.json](interpretation_manifest.json) and [figure_manifest.json](figure_manifest.json). All new feature inputs exclude anchors/GNSS. Frozen existing fine inputs inherit the historical leveled-height filter; primary raw maps do not use it.

An independent contrast calculation originally ran as `/tmp/scene_independent_contrast_audit.py`. Its exact bytes are archived as [independent_contrast_audit.py](independent_contrast_audit.py); original command/hash/input identities remain in [independent_contrast_audit.json](independent_contrast_audit.json). This checks selected descriptive contrasts without another map/path search.

To replay the full calculation without overwriting this audit, copy the compute/interpret/supplement/plot/validator scripts together to a **new sibling directory under this repository's `results/`**; the scripts resolve ROOT from this location. Execute the same phases from that sibling. Existing raw/fine caches and saved graphs are required; their availability on another machine is not guaranteed. Validation of this stored run can be repeated directly with the final command above.

## Tables and schemas

| Artifact | Contents |
|---|---|
| [protocol.json](protocol.json), [FROZEN](FROZEN) | Fixed causal definitions, persistent cutoffs, density/distance, cohort and research boundary |
| [input_lineage.csv](input_lineage.csv) | 5 bags / 272 frames / 30,941,328 raw returns / 20,122 fine nodes; saved component identities |
| [frame_scene_statistics.csv](frame_scene_statistics.csv) | Every frame, three source/scale representations; past overlap, novelty, persistent support, concentration and lag1 Jaccard |
| [node_scene_features.csv](node_scene_features.csv) | All 20,122 fine nodes, raw/fine 1m at three fixed cutoffs and fine4cm at primary cutoff: 140,854 rows |
| [sequence_scene_summary.csv](sequence_scene_summary.csv) | Return-weighted all-frame, F≥2 and fixed chronological thirds; counts and median/IQR |
| [persistent_cutoff_scene_sensitivity.csv](persistent_cutoff_scene_sensitivity.csv) | Fixed 0.25/0.5/0.75 persistent support, both return and current-cell denominators |
| [fixed_causal_time_milestones.csv](fixed_causal_time_milestones.csv) | Fixed frame2/5/10/20/last statistics, not an optimized stabilization time |
| [anchor_node_join.csv](anchor_node_join.csv) | All 80 original observed anchors, every node within existing3m gate, nearest-node descriptive features |
| [anchor_same_frame_contrasts.csv](anchor_same_frame_contrasts.csv) | Anchor-node feature percentile/difference against all existing nodes of the same frame |
| [population_scene_features.csv](population_scene_features.csv) | All/matched/nearest nodes, fixed silver witness, saved four-arm winners, historical references, direct anchor-position queries |
| [population_distribution_summary.csv](population_distribution_summary.csv), [descriptive_effect_sizes.csv](descriptive_effect_sizes.csv) | Finite n, linear quantiles/IQR, min/max and Cliff delta; dependent-sample proxy contrasts, not AUC |
| [anchor_vs_winner_same_frame.csv](anchor_vs_winner_same_frame.csv), [same_frame_winner_contrast_summary.csv](same_frame_winner_contrast_summary.csv) | Paired observed-frame comparisons, including missing path frames and undefined cold-start distances |
| [existing_path_characterization.csv](existing_path_characterization.csv) | Saved path duration, real-dt geometric speed/span, occupancy, scene proximity; no rescore |
| [occupancy_distance_associations.csv](occupancy_distance_associations.csv), [distance_within_occupancy_bins.csv](distance_within_occupancy_bins.csv), [same_frame_H0_distance_spread.csv](same_frame_H0_distance_spread.csv) | Occupancy/distance dependence and same-H/same-frame spread, not class independence |
| [corrected_spatial_concentration.csv](corrected_spatial_concentration.csv) | Physical10m-cube concentration for all three representations; correction described below |
| [validation.json](validation.json), [validation_tests.txt](validation_tests.txt) | Independent arithmetic/causality/geometry/source integrity checks |
| [figure_manifest.json](figure_manifest.json), [figures](figures/) | Five PDF/300dpiPNG diagnostics plus two-page PDF; fixed midpoint frames, no anchor ROI |
| [final_manifest.json](final_manifest.json) | Final deliverable identities and scope/integrity checks |

Thirty local compressed NPZs cover three representations × five bags × two types:

`cell_history__<source>__<condition>.npz`: arrays `cells[N,3]`, `offsets[frames+1]`, `times[frames]`, scalar `cell_m`; per occupied cell `prior_hits`, `prior_rate`, `p_beta11`, `first_visit`, `revisit`, `last_frame`, `age_s`, `longest_previous_run`, `previous_episodes`, `raw_count`, `fine_count`, `persistent`. The frame slice is `[offsets[f]:offsets[f+1]]`; prior denominator is f. `raw_count` is current raw support in that cell (fine-source weights only retained preprocessing support). `fine_count` is the number of input elements contributing to the cell; for raw_1m it equals1 per original raw voxel, not a fine-point count. NPZ dtype/schema is inspectable without ROS.

`persistent_snapshots__<source>__<condition>.npz`: `cells`, `offsets`, `times`, `cell_m` store the primary0.5 causal persistent set before each current frame, including cells not currently observed. They use no future frames. Fixed-cutoff node features were all computed before anchor joining.

Fine children are recovered existing voxel-centre members with raw-support weights, not raw point IDs. Empty scene distances are NaN; density can be0. Primary raw/fine1m centre grids share the E18 zero-origin definition, but use different historical return sources. Names/columns do not certify target/clutter identity.

## Retained limitation and research boundary

The original frozen `frame_scene_statistics.csv` concentration columns for fine_004m aggregate `cell_index//10`, physically0.4m rather than10m despite their names. Original outputs are retained. `corrected_spatial_concentration.csv` independently recomputes physical10m grouping for every source; reports use that table. Raw/fine1m original concentrations, occupancy, distances, memberships and all main results are unaffected.

Primary findings include the **100m_cross_10 raw-scene counterexample and raw/fine/cutoff sign reversal**. Do not choose a cutoff or input based on anchors, promote isolation to a UAV label, reuse all-node comparisons as certified negatives, or add any quantity to E18 score from this audit. Four cold-start anchor distances remain UNKNOWN. No bags, PCDs, ROS conversions, recordings, runtime configs or logs were generated. This work has not been committed or pushed.
