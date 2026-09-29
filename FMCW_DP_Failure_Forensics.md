# FMCW Candidate-Graph DP Failure Forensics

## Material Passport

- **Origin:** post-hoc diagnostic of the frozen candidate-graph DP
- **Date:** 2026-09-29
- **Verification status:** **VERIFIED RESULT on DEV/SILVER diagnostics; not independent GT**
- **Source revision:** Git `c913dcd6598ba9f0c06433a50d4a9105b417058e`
- **Source DP protocol SHA-256:** `5552c35cf9de614c98a668647b66d3ece27efa60695e13bc856c5488b58324af`
- **Forensics protocol SHA-256:** `d088ed8a0500abf771842dfb2be8cdef48226ee4a9499b65134dc68b247d83a8`
- **Forensics script SHA-256:** `9116d6e724b4e8460ee2914a55b83ec9f6bb9dcae50a86b6f17d5852db8be328`
- **Labels:** unchanged `research_dev_v0` `DEV_SILVER`, SHA-256 `37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db`
- **Algorithm/parameter changes:** **NONE**
- **Command:** `python3 scripts/dp_failure_forensics.py freeze`; `python3 scripts/dp_failure_forensics.py run`

## Executive diagnosis

The UAV path is not missing from the graph. A feasible path that reaches every silver observation exists in all five sequences: `19/19`, `11/11`, `22/22`, `18/18`, and `10/10`. The failure is therefore a ranking failure, not candidate deletion or graph infeasibility.

The frozen score favors persistent clutter for two reasons:

1. every retained node contributes a positive base term of `0.50`, so a path that exists in every frame accumulates score indefinitely;
2. the only scored appearance terms—compactness and a quickly saturated raw-support term—do not separate the silver UAV nodes from the selected clutter nodes.

Speed, acceleration, curvature, Doppler, and legacy occupancy/revisit are not positive evidence in the DP objective. Speed and acceleration are only upper-bound feasibility gates. Consequently a nearly stationary background return is the easiest possible trajectory: it passes the gates, pays no gap penalty, is compact, and persists through the complete bag.

## 1. Diagnostic definitions

### 1.1 Current top clutter path

The original graph and Viterbi recurrence were replayed without modification. Replayed paths match every recorded `candidate_graph_DP` output frame and center to within `1e-5 m`.

### 1.2 Silver-consistent feasible UAV path

Within the inclusive first-to-last silver frame, the diagnostic DP uses the unchanged graph, score, `dt`, speed, acceleration, predecessor pruning, and gap penalty. It selects paths lexicographically:

1. maximize the number of silver frames with a node within the frozen 3 m radius;
2. among equal-hit paths, maximize the original frozen cumulative DP score.

This label-guided path is for diagnosis only. It is not a detector output and cannot be evaluated as independent ground truth.

### 1.3 Rank definition

The original Viterbi implementation retains one best predecessor per edge state. Therefore the exact rank among the exponentially many raw graph paths is **UNKNOWN**. The report provides:

- exact rank when the silver path itself is one of the unique Viterbi edge-state winners;
- otherwise, a strict lower bound: `1 + number of qualifying edge-state winner scores greater than the silver-path score`;
- coverage of the top 100 unique edge-state winner paths.

## 2. Does the UAV path exist?

| Sequence | Silver hits in best feasible path | Full path exists | Silver-path score | Rank evidence |
|---|---:|---|---:|---|
| `100m_cross_5` | 19/19 | Yes | 20.412 | exact state-winner rank 4 |
| `100m_cross_10` | 11/11 | Yes | 12.196 | rank lower bound 259; top 100 contain 0 silver hits |
| `200m_cross_5` | 22/22 | Yes | 21.094 | exact state-winner rank 9 |
| `200m_cross_10` | 18/18 | Yes | 17.227 | rank lower bound 21; best top-100 path reaches 17/18 |
| `300m_cross_10` | 10/10 | Yes | 9.100 | rank lower bound 466; top 100 contain 0 silver hits |

Answer to question A: **yes for all five sequences**. The 100 m/10 m/s and 300 m/10 m/s UAV paths are especially poorly ranked even though they are fully feasible.

## 3. Score decomposition

The frozen score is

`sum(0.50 + compactness_component + support_component) - gap_penalty`.

There is no positive edge score for plausible speed, smooth velocity, or motion direction. Kinematics only reject paths above maximum speed/acceleration.

### 3.1 Full-bag persistence advantage

| Sequence | Top clutter nodes / score | UAV nodes / score |
|---|---:|---:|
| `100m_cross_5` | 32 / 31.158 | 21 / 20.412 |
| `100m_cross_10` | 46 / 44.745 | 13 / 12.196 |
| `200m_cross_5` | 83 / 81.084 | 22 / 21.094 |
| `200m_cross_10` | 48 / 46.979 | 18 / 17.227 |
| `300m_cross_10` | 63 / 61.480 | 10 / 9.100 |

The clutter paths occupy every LiDAR frame. At roughly `0.97` score/node, persistence alone gives them an overwhelming full-bag cumulative advantage. This is not target persistence; it is persistence of a stable background return.

### 3.2 Common silver-span comparison

To separate duration from per-node scoring, the current top clutter path was restricted to the same first-to-last silver frame span.

| Sequence | Clutter−UAV total score | Base contribution | Compactness contribution | Support contribution | Gap contribution |
|---|---:|---:|---:|---:|---:|
| `100m_cross_5` | +0.126 | +0.000 | +0.534 | −0.409 | +0.000 |
| `100m_cross_10` | +1.455 | +0.500 | +0.485 | +0.220 | +0.250 |
| `200m_cross_5` | +0.301 | +0.000 | +0.256 | +0.045 | +0.000 |
| `200m_cross_10` | +0.389 | +0.000 | +0.409 | −0.020 | +0.000 |
| `300m_cross_10` | +1.586 | +0.500 | +0.301 | +0.536 | +0.250 |

Interpretation:

- In three sequences both paths contain the same number of span nodes. Clutter still wins by only `0.126–0.389`, mainly because the compactness term slightly favors it.
- At `100m_cross_10` and `300m_cross_10`, the UAV path skips one frame. It loses `0.50` base score and pays a `0.25` gap penalty; compactness/support add the remaining disadvantage.
- Raw support often favors the UAV, but `support_component` caps at `0.20` once support reaches five points. Its pooled target/clutter AUC falls from `0.850` for raw support to `0.512` after score saturation.

Answer to question B: the main full-run cause is **unbounded positive persistence/base accumulation**. On a matched time span, the remaining causes are a small compactness bias and, in the two sparse-gap sequences, the extra-node base reward plus gap penalty. Raw support is not the main loss because its usable signal is saturated away.

## 4. Physical and sensing comparison

Values below are medians for `top clutter within silver span / silver-feasible UAV path`.

### 4.1 Motion, range, and Doppler

| Sequence | Speed m/s | Acceleration m/s² | Curvature deg | Range m | |Doppler| m/s | Doppler residual m/s |
|---|---:|---:|---:|---:|---:|---:|
| `100m_cross_5` | 0.167 / 1.963 | 0.281 / 2.481 | 141.10 / 9.53 | 447.13 / 102.55 | 0.131 / 0.311 | 0.136 / 0.185 |
| `100m_cross_10` | 0.151 / 4.336 | 0.186 / 2.456 | 119.41 / 1.83 | 204.07 / 116.68 | 0.051 / 1.742 | 0.070 / 0.726 |
| `200m_cross_5` | 0.065 / 7.022 | 0.118 / 1.099 | 146.07 / 1.68 | 204.05 / 203.30 | 0.059 / 1.733 | 0.065 / 1.183 |
| `200m_cross_10` | 0.111 / 3.572 | 0.168 / 2.205 | 131.33 / 2.45 | 204.05 / 205.22 | 0.021 / 0.598 | 0.036 / 0.227 |
| `300m_cross_10` | 0.072 / 8.576 | 0.117 / 3.187 | 158.64 / 1.39 | 204.06 / 297.77 | 0.089 / 1.553 | 0.071 / 1.242 |

`Doppler residual = |-measured Doppler - geometric dr/dt|`, following the dataset-observed sign convention.

The UAV paths are faster in all five sequences and have smoother direction changes. The large clutter curvature is produced by tiny, noise-dominated displacements, so curvature should be interpreted together with speed. UAV acceleration is higher but remains below the frozen 15 m/s² ceiling; because acceleration is not rewarded, both classes pass.

Absolute measured Doppler is higher for the UAV in all five selected comparisons. However, Doppler–range-rate residual is also higher for the UAV in all five. Near-static clutter trivially has both near-zero range rate and near-zero Doppler, so a “small residual is good” interpretation favors clutter here. This agrees with the earlier finding that Doppler consistency did not establish independent detector gain; the current result only separates the selected top clutter path from silver nodes by magnitude.

Range is not a general separator: the dominant clutter happens to lie near 204 m in four bags and 447 m in one, while the target range is defined by the flight condition. At 200 m, ranges substantially overlap.

### 4.2 Compactness, support, and legacy revisit

| Sequence | Extent m | Raw support | Legacy bg probability | Cell revisit fraction | 3×3×3 revisit fraction |
|---|---:|---:|---:|---:|---:|
| `100m_cross_5` | 0.000 / 0.083 | 6 / 40 | 0.864 / 0.000 | 0.905 / 0.000 | 0.905 / 0.074 |
| `100m_cross_10` | 0.154 / 0.278 | 8 / 37 | 0.961 / 0.000 | 1.000 / 0.000 | 1.000 / 0.000 |
| `200m_cross_5` | 0.181 / 0.146 | 9 / 13 | 0.913 / 0.000 | 1.000 / 0.000 | 1.000 / 0.000 |
| `200m_cross_10` | 0.121 / 0.223 | 9 / 22.5 | 0.975 / 0.000 | 1.000 / 0.025 | 1.000 / 0.047 |
| `300m_cross_10` | 0.214 / 0.000 | 8 / 5 | 0.833 / 0.000 | 1.000 / 0.000 | 1.000 / 0.000 |

Legacy quantities are prior-only: the current frame is not included. Background probability replays A′'s static-count rule; revisit fraction independently counts prior input frames occupying the same cell or its 3×3×3 neighborhood.

The top clutter paths repeatedly occupy the same cells and have high legacy background probability. The UAV cells are mostly previously unvisited. This is the strongest observed discriminator between these two selected path classes. It is diagnostic only: the legacy model can absorb initialization targets and revisiting UAVs, so this result must not be generalized into an algorithm claim.

## 5. Feature separation audit

The following descriptive statistics compare silver-compatible UAV nodes with top-clutter nodes at the same 80 silver frames. `Separation AUC` is direction-free (`max(AUC, 1−AUC)`). It is not operational precision and uses the same DEV/silver data that selected the UAV path.

| Feature | Target median | Clutter median | Separation AUC | Direction consistent across sequences |
|---|---:|---:|---:|---:|
| Legacy background probability | 0.000 | 0.913 | 0.985 | 5/5, target lower |
| Cell revisit fraction | 0.000 | 1.000 | 0.983 | 5/5, target lower |
| Neighborhood revisit fraction | 0.037 | 1.000 | 0.980 | 5/5, target lower |
| Speed | 4.551 | 0.086 | 0.922 | 5/5, target higher |
| Acceleration | 2.240 | 0.166 | 0.898 | 5/5, target higher |
| Curvature | 2.459° | 138.960° | 0.867 | 5/5, target lower |
| Absolute measured Doppler | 0.724 | 0.065 | 0.866 | 5/5, target higher |
| Doppler–range-rate residual | 0.379 | 0.071 | 0.870 | 5/5, target higher/worse |
| Raw support | 21.5 | 9.0 | 0.850 | 4/5, target higher |
| Range | 202.72 m | 204.06 m | 0.638 | 3/5 |
| Frozen node score | 0.977 | 0.980 | 0.539 | 3/5, target lower |
| Compactness component | 0.281 | 0.283 | 0.547 | 2/5 |
| Saturated support component | 0.200 | 0.200 | 0.512 | 0/5; median tie |

Answer to question C:

- **Strong observed separation for the selected paths:** legacy occupancy/revisit, nonzero speed, directional smoothness, absolute Doppler magnitude, and raw support. These signals are absent from the score or, for raw support, largely destroyed by saturation.
- **No useful observed separation:** the actual frozen node score, compactness component, saturated support component, and range.
- **Separates in the wrong direction:** a small Doppler–geometric residual strongly favors stationary clutter, not the UAV.
- **Persistence is not target-specific:** it is the main reason clutter wins because the score rewards duration regardless of whether the persistent source moves.

“Target–clutter separation” remains **UNKNOWN as a general detector claim**. The present evidence is restricted to five retrospectively selected silver paths versus five winning clutter paths, not the full clutter population or independently labeled negative scenes.

## 6. Per-sequence failure interpretation

- `100m_cross_5`: the UAV path is almost tied with clutter over the common span and ranks fourth. Greater raw support nearly compensates for the clutter's compactness advantage. Full-bag persistence decides the final output.
- `100m_cross_10`: the UAV path exists but ranks below at least 258 state winners. One skipped frame, a gap penalty, and lower compactness/node score combine with full-bag persistence. No top-100 state-winner path reaches even one silver observation.
- `200m_cross_5`: the full UAV path ranks ninth. With equal node count, clutter wins by only 0.301 score, mostly compactness; full-bag persistence then dominates.
- `200m_cross_10`: the full UAV path exists; its exact all-path rank is unknown and at least 21. The best top-100 state winner reaches 17/18 silver frames, indicating a near-UAV competitor, but compactness still favors clutter.
- `300m_cross_10`: all singleton evidence is graph-feasible, yet the UAV path ranks below at least 465 state winners. Losing one node, paying a gap penalty, and having low/saturated support produces the largest common-span score deficit. No top-100 state winner reaches a silver observation.

## 7. Conclusion

The frozen DP fails because it solves its stated objective correctly: maximize a sum of always-positive per-node evidence under permissive upper-bound motion constraints. In these bags, a static structured return is more persistent and at least as compact as the UAV, so it is the optimal path.

This diagnosis does not change weights or propose a replacement. It establishes:

1. the UAV is present in every graph;
2. graph pruning and feasibility constraints are not the primary failure;
3. the current scored features do not separate UAV from winning clutter;
4. several unscored diagnostics do separate these selected paths, but their general validity remains unverified.

## 8. Machine-readable artifacts

| File | Contents |
|---|---|
| `results/dp_failure_forensics/protocol.json` | frozen diagnostic definitions and source hashes |
| `path_summary.csv` | per-sequence full clutter, common-span clutter, and silver-path summaries |
| `path_nodes.csv` | node-level score, kinematics, Doppler, silver distance, occupancy, and revisit values |
| `score_component_comparison.csv` | direct clutter-versus-UAV component deltas |
| `rank_diagnostics.csv` | graph existence, state-winner rank/lower bound, and top-100 coverage |
| `feature_separation.csv` | paired descriptive AUC and per-sequence direction audit |
| `graph_replay.csv` | graph sizes and exact replay verification |
| `run_manifest.json` | execution identity and hashes |
