# FMCW Published Baseline and Candidate-Graph DP MVP

**Experiment date:** 2026-09-29

**Status:** **TESTED on frozen DEV/SILVER labels; not paper ground truth**

**Primary artifacts:** `results/published_baseline_dp_mvp/`

**Code:** `scripts/published_baseline_dp_mvp.py`

**Frozen protocol SHA-256:** `5552c35cf9de614c98a668647b66d3ece27efa60695e13bc856c5488b58324af`

## Executive result

This experiment implemented two new, independent methods without modifying A′, the historical baseline, or project-adapted B:

1. a processing-chain reconstruction of [Khosravi et al. (2026)](https://arxiv.org/abs/2603.11586): range-adaptive DBSCAN → geometric validation → spatial-jump validation → M-of-K temporal consistency;
2. a minimal singleton-preserving candidate-graph DP/Viterbi baseline with real `dt`, hard speed/acceleration constraints, and no Doppler.

The published-structure reconstruction differs materially from B on this DEV set: it detects `58/80` silver observations versus B's `41/80`, but emits `26.39` rather than `23.88` outputs per LiDAR frame. Because reliable negative labels do not exist, this is **not evidence of higher operational precision or a superior detector**.

The candidate-graph DP result is a clear negative result. It reduces `62.48` singleton candidates/frame to one selected path point/frame, but detects `0/80` silver observations. Its best paths are nearly stationary persistent clutter, usually near 204 m range. Therefore this minimal global DP is worse than both the frozen greedy soft accumulator and B in recall, despite its strong volume reduction.

## Material Passport

| Item | Frozen identity / status |
|---|---|
| Research question | Can a paper-structured detector and a minimal global candidate-path DP improve the sparse-UAV recall–clutter trade-off? |
| Dataset | Five confirmed cross-flight sequences: `100m_cross_5`, `100m_cross_10`, `200m_cross_5`, `200m_cross_10`, `300m_cross_10` |
| Labels | `research_dev_v0/silver_labels.csv`; 80 observed LiDAR evidence frames; `DEV_SILVER`, not independent GT |
| Label SHA-256 | `37c4452a68e544d75034389f82bbaf1e868903bb2225b395b9d66d0cdd48b8db` |
| Negative labels | **NONE RELIABLE**; operational precision, false alarms/frame, and PR curves are not reported |
| Paper source | Khosravi, Ventura, Basiri, *Unsupervised LiDAR-Based Multi-UAV Detection and Tracking Under Extreme Sparsity*, arXiv:2603.11586 |
| Retrieved paper PDF SHA-256 | `68e5e1aba52abd00d085de91c3d55ff31dcfa4d7a26b37a0f5405e760cdd8174` |
| Frozen source revision | Git `1a069b2` before the new uncommitted experiment files |
| Random seed | `20260929`; neither evaluated algorithm uses stochastic sampling |
| Environment | Python 3.10.12; NumPy 2.2.6; SciPy 1.15.3; scikit-learn 1.7.2; Linux x86_64 |
| Exact commands | `python3 scripts/published_baseline_dp_mvp.py freeze`; `... cache`; `... run` |
| Test command | Direct execution of four test functions in `tests/test_published_baseline_dp_mvp.py`; `pytest` was unavailable |
| New code SHA-256 | `0601d93a804ecb66dcb1d0185fc6f2c272a5c5ac9f47674bc4d34cefa7766d9f` |

Raw bags were opened read-only. The 4 cm voxel caches are reproducible intermediates; their per-package hashes are recorded in `run_manifest.json`.

## 1. What “faithful Khosravi baseline” means here

This is **not an exact reproduction**. The paper describes the stage order and equations, but omits several numerical values. In addition, its experiments use a Livox Mid-360 in approximately 5–25 m air-to-air scenarios, whereas this project uses ground-based FMCW LiDAR at nominal 100–300 m. All provenance is machine-readable in `parameter_provenance.csv`.

### 1.1 Parameters reported by the paper

The reconstruction uses Table I configuration C for `voxel=0.04 m`, `eps0=0.60 m`, and `minPts=2`; the paper gives `rref=10 m` and

`eps(r) = eps0 + alpha * max(r-rref, 0)`.

It also uses the stated centroid rule: component-wise median for clusters with at least three points/voxels, arithmetic mean below three.

### 1.2 Parameters not numerically reported by the paper

The public paper does not provide numeric values for:

- `alpha`, `hmin`, `rmax`, `rexcl`;
- `nmin`, `nmax`, `emax`;
- `tau_min`, `vmax`;
- `M`, `K`, `d_cons`, `T_cons`.

Moreover, Table I reports `alpha>0` only for configurations C/D and Layer 3 only for S1/S4/MR. No published row combines the full range-adaptive + M-of-K chain requested here. Calling the present code an exact paper reproduction would therefore be unsupported.

### 1.3 Frozen project adaptations

| Component | Adaptation frozen before comparison |
|---|---|
| Input transform | rich-80 packets `[x,z,-y]`; provisional 6-DoF leveled height |
| ROI/intensity | x 8–510 m, y ±200 m, z ±60 m; intensity 10–250; leveled z > 10 m |
| Range adaptation | `alpha=0.004`; no epsilon cap, matching the published equation form |
| Geometry | 2–32 voxels, ≤120 raw support points, max axis extent <6 m, range 8–510 m |
| Jump | `max(3 m, 25 m/s * dt)` using real bag `dt` |
| Temporal | `M=2` qualifying prior observations among `K=3`; `d_cons=30 m`; `T_cons=3.5 s` |
| Association | one-to-one greedy nearest last centroid |
| Output interpretation | current candidate requires two prior qualifying observations, hence at least three total hits |

These are project adaptations, not claims about missing paper values.

## 2. Minimal candidate-graph DP/Viterbi

The DP uses exactly the same frozen 1 m, range-adaptive `minPts=1` candidate generator as the previous temporal-evidence study. It does not use Doppler and does not require any node to cross a single-frame detection threshold.

### 2.1 Frozen definition

- Node score: `0.50 + 0.30 exp(-extent/2 m) + 0.20 min(log(1+raw_support)/log(6), 1)`.
- Edges: maximum frame gap 3, maximum elapsed time 3.5 s, speed ≤25 m/s.
- Second-order transition: acceleration ≤15 m/s².
- Graph control: at most 12 nearest predecessors per prior frame.
- Gap penalty: 0.25 per skipped frame.
- Valid output: at least 3 nodes and 2 s duration.
- Per bag output: the single highest cumulative-score admissible path.

The score intentionally contains only simple compactness/support evidence. Doppler is absent by design. This is a candidate-graph temporal baseline, **not a full point-level DP-TBD implementation**.

## 3. Unified results

“Input/frame” is the candidate volume before each method's temporal output stage. “Output/frame” is the reported candidate/path-point volume. Unmatched outputs on labeled frames are diagnostic only and must not be interpreted as operational false alarms.

| Method | Range | Silver recall | Input/frame | Output/frame |
|---|---:|---:|---:|---:|
| A′ corrected legacy | 100 m | 21/30 (70.0%) | 6.62 | 6.62 |
| A′ corrected legacy | 200 m | 18/40 (45.0%) | 6.11 | 6.11 |
| A′ corrected legacy | 300 m | 2/10 (20.0%) | 4.94 | 4.94 |
| Project-adapted B | 100 m | 15/30 (50.0%) | 28.26 | 23.50 |
| Project-adapted B | 200 m | 23/40 (57.5%) | 28.44 | 24.15 |
| Project-adapted B | 300 m | 3/10 (30.0%) | 28.25 | 23.81 |
| Published-structure reconstruction | 100 m | 14/30 (46.7%) | 36.05 | 25.95 |
| Published-structure reconstruction | 200 m | 37/40 (92.5%) | 36.52 | 27.27 |
| Published-structure reconstruction | 300 m | 7/10 (70.0%) | 34.94 | 25.11 |
| Candidate-graph DP | 100 m | 0/30 (0.0%) | 62.22 | 1.00 |
| Candidate-graph DP | 200 m | 0/40 (0.0%) | 63.13 | 1.00 |
| Candidate-graph DP | 300 m | 0/10 (0.0%) | 61.46 | 1.00 |

Across all ranges:

| Method | Silver recall | Output/frame |
|---|---:|---:|
| A′ corrected legacy | 41/80 (51.25%) | 5.98 |
| Project-adapted B | 41/80 (51.25%) | 23.88 |
| Published-structure reconstruction | 58/80 (72.50%) | 26.39 |
| Candidate-graph DP | 0/80 (0.00%) | 1.00 |

The very small center residuals of matched published-structure candidates are not headline accuracy evidence: the silver centers were derived from LiDAR evidence in the same data and are not independent truth.

## 4. Stage and failure diagnostics

Before its temporal layer, the published-structure candidate stage detects `22/30`, `40/40`, and `9/10` silver observations at 100/200/300 m, respectively. After jump + M-of-K, these become `14/30`, `37/40`, and `7/10`. Thus the reconstructed temporal layer suppresses target evidence as well as clutter, especially at 100 m.

The DP input candidate generator retains all `80/80` silver observations. Nevertheless, every selected maximum-score path misses the labels. The selected paths span every LiDAR frame in each package, but their motion is characteristic of persistent background:

| Sequence | Path nodes | Range median | Start–end displacement | Median speed | Silver hits |
|---|---:|---:|---:|---:|---:|
| `100m_cross_5` | 32 | 447.13 m | 7.79 m | 0.208 m/s | 0/19 |
| `100m_cross_10` | 46 | 204.06 m | 0.20 m | 0.120 m/s | 0/11 |
| `200m_cross_5` | 83 | 204.05 m | 0.03 m | 0.067 m/s | 0/22 |
| `200m_cross_10` | 48 | 204.06 m | 0.08 m | 0.063 m/s | 0/18 |
| `300m_cross_10` | 63 | 204.05 m | 0.05 m | 0.079 m/s | 0/10 |

This failure is mechanistically consistent with the frozen score: positive persistence and compactness reward a stationary, long-lived clutter path, while speed/acceleration are only upper bounds and there is no target-versus-static evidence.

## 5. Answers to the research questions

### A. Does the faithful baseline materially differ from project-adapted B?

**Yes on this DEV set, but the cause and operational benefit are unresolved.** The reconstruction raises silver recall from `41/80` to `58/80` and output volume from `23.88` to `26.39` per frame. The difference is strongest at 200 and 300 m, while it is slightly worse at 100 m. Multiple factors change together—4 cm rather than 1 m voxelization, paper-C DBSCAN parameters, uncapped range adaptation, centroid definition, and a different temporal interpretation—so the gain cannot be attributed to a single paper component. Without negative scenes, it is not valid to say the reconstruction is a better operational detector.

### B. Does global DP accumulation beat the previous greedy soft accumulator?

**No.** The frozen greedy method at threshold 0.68 obtains `7/80` with `1.89` outputs/frame; its post-hoc threshold-0.55 diagnostic obtains `52/80` with `17.23` outputs/frame. The new DP obtains `0/80` with `1.00` output/frame. It is more aggressive in volume reduction but selects persistent clutter globally. This is a negative result, not a reason to retune the frozen score on the same labels.

### C. Can 300 m singleton evidence be retained by trajectory accumulation while suppressing clutter?

**Not with this minimal DP.** The singleton candidate stage contains `10/10` 300 m silver observations and `61.46` candidates/frame. DP reduces this to one output/frame but retains `0/10`; its chosen path is nearly stationary at approximately 204 m. The experiment confirms that singleton evidence exists, but simple maximum cumulative persistence/compactness is insufficient to identify it.

## 6. Limitations and decision

- Labels are retrospective LiDAR-only `DEV_SILVER`, not independent GNSS-aligned truth.
- All selected bags are UAV-flight bags; unlabeled frames are not reliable negatives.
- The published baseline is a documented reconstruction because important paper parameters are `UNKNOWN`.
- Paper and project sensing geometries differ substantially.
- Candidate/output volume is not precision and cannot establish false-alarm performance.
- A′ and B numbers are imported unchanged from the prior frozen artifact; the new methods were run against the same labels and sequences.
- Runtime excludes raw bag-to-cache creation and should not be treated as online latency.

**Decision:** preserve the reconstruction as a stronger detection-layer reference, but do not call it an exact reproduction or a validated improvement. Reject the current minimal DP score as a solution to the singleton recall–clutter problem. Do not tune it further on `research_dev_v0`; the next scientifically useful step is to obtain reliable negative windows and a held-out split before introducing additional motion evidence or a Doppler ablation.

## 7. Artifact index

| File | Role |
|---|---|
| `results/published_baseline_dp_mvp/protocol.json` | immutable parameter/provenance protocol |
| `parameter_provenance.csv` | paper-reported, paper-unknown, and project-adapted parameter separation |
| `benchmark_results.csv` | per-sequence and 100/200/300 m unified metrics |
| `frame_results.csv` | per-silver-frame decisions |
| `candidate_detections.csv` | machine-readable published-baseline outputs and DP path nodes |
| `candidate_stage_recall.csv` | pre-temporal diagnostic survival |
| `graph_diagnostics.csv` | node/edge/path and selected-path motion diagnostics |
| `prior_greedy_reference.csv` | frozen 0.68 and post-hoc 0.55 greedy references |
| `negative_window_audit.json` | reason operational precision is withheld |
| `run_manifest.json` | commands, code/protocol/label/cache hashes |
| `raw_fine_cache/*.json` | per-frame source/cache metadata; `.npz` files are reproducible intermediates |
