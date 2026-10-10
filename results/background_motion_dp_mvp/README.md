# Background-relative + motion-coherence DP MVP (E18)

Development-only experiment; all labels remain `DEV_SILVER`, not independent paper GT. See the root report `FMCW_Background_Motion_DP_MVP.md` for the outcome and stop decision. No historical baseline, cache or labels are overwritten.

## Two separate graph families

- `fine_4cm_pregeometry`: every previously saved E16 `adaptive_minPts1` component, before the old geometric acceptance flag. No DBSCAN rerun, no new epsilon, and no hard support/voxel/extent rejection. Old geometric flags remain diagnostic columns only.
- `historical_1m_graph`: exact E13 candidates and edges replayed; old geometry remains part of this historical control. These are not fine candidates. E14 recorded paths are fixed reference identities here.

Within each family, `original_score` and `background_motion` load byte-identical nodes/edges. Representation/candidate changes between families cannot be attributed to scoring.

## Artifact map

| Artifact | Meaning |
|---|---|
| `protocol.json`, `FROZEN`, `freeze_identity.json` | Prospectively frozen functions, constants, gates, source/test hashes, parent commit and kill rule |
| `input_identity.json` | Protected historical inputs, caches, labels, sources and evidence hashes |
| `graph_index.json`, `graphs/*__nodes.csv`, `graphs/*__edges.csv` | Immutable candidate/edge snapshots, real timestamps, causal annotations and identities |
| `run_manifest.json` | Completed label-unconditioned run, environment, timing and output hashes |
| `selected_paths.csv`, `unconditioned_summary.csv` | One unconditioned offline winner per bag/method; score decomposition and volume/runtime |
| `top100_full_paths.csv`, `winner_pools/*__full.csv` | Eligible edge-state winner representatives, NOT global top-K combinatorial paths |
| `fixed_reference_paths.csv`, `reference_feasibility.csv` | Fixed old-score silver witnesses, historical/old-score reference paths, per-label candidate presence |
| `fixed_path_scores.csv`, `fixed_path_ranks.csv` | Same fixed paths rescored; rank evidence and explicit UNKNOWN global rank |
| `score_margins.csv`, `winner_pools/*__span.csv` | Comparisons inside the same silver evaluation window; actual endpoints/durations retained |
| `unconditioned_evaluation.csv`, `summary_by_range.csv` | Existing 3 m center-gate DEV recall and 100/200/300 m summaries |
| `kill_decision.csv`, `evaluation_manifest.json` | Predeclared stop/gate decision; unchanged-source/input verification |
| `independent_graph_audit.json`, `independent_result_audit.json` | Separate review checks, not another score trial |
| `observed_miss_diagnostic.csv` | Post-run read-only record of the retained frame21 mismatch; supplemental artifact, not a scored run/evaluation output |
| `validation_checks.json` | 14 unittest +7 legacy synthetic assertion checks, exact direct-invocation logic and the unavailable-pytest limitation |

Node positions are sensor-relative `[x,z,-y]` coordinates. Background cells are zero-origin floor(XYZ/1 m), not the old gravity-aligned occupancy implementation. The probability column is a Beta-smoothed occupancy proxy, **not calibrated target/clutter probability**. Every occupied cell is counted once per frame; current-frame points are added only after all that frame's node annotations.

`top_clutter_*` path-type identifiers preserve the comparison schema. In the fine family their explicit `reference_role` is `OLD_SCORE_UNCONDITIONED_WINNER_NOT_ASSUMED_CLUTTER`; do not infer object identity from that identifier. Historical references were the E14 persistent-clutter paths.

`score_rank_lower_bound` is 1 plus the number of distinct retained eligible representatives strictly exceeding a fixed reference score (tolerance 1e-9). `exact_rank_in_representatives` is available only when that exact path is itself retained. The global combinatorial path rank is UNKNOWN; no exact global rank or operational precision is claimed.

## Reproduction

Run from the repository root in the existing `fmcw` environment. Every phase refuses completed-output overwrite. The output directory must not already exist for a new freeze; do not delete these research records to rerun.

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python -m unittest discover -s tests -p 'test_background_motion*.py' -v
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py graph
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py run
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py evaluate
```

The actual graph invocation also contained an unused `PYTHONDONTWRITECODE=1` environment variable; it had no code effect. All effective variables and phases are as shown above. Parameters and source hashes were frozen before graph construction and real-data scoring. Labels were hashed as bytes for integrity earlier, but parsed/used only after unconditioned outputs were complete.

Raw bags/PCDs, virtual environment and NPZ caches are intentionally not uploaded. Full regeneration requires the local protected caches identified in `input_identity.json`; their availability on another machine is UNKNOWN. Saved graph CSVs preserve the scoring input and can be inspected without raw data. Candidate/output volume is not precision; unobserved frames are not reliable negatives. A stop decision forbids further feature stacking or retuning under this task.
