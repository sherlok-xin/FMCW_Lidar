# temporal_evidence_mvp artifacts

This directory contains the frozen 2026-09-29 `research_dev_v0` experiment reported in `../../FMCW_Temporal_Evidence_MVP.md`.

Label status is `DEV_SILVER`; these artifacts are not final paper ground truth. There are no reliable target-absent windows, so no operational precision, false-alarms/frame, or PR curve is claimed.

- `protocol.json`, `FROZEN`: comparison protocol frozen before method results were read.
- `negative_window_audit.json`: negative-window decision and allowed metrics.
- `run_manifest.json`: execution time, command, Git state, and hashes.
- `benchmark_results.csv`: formal per-sequence and per-range metrics.
- `frame_results.csv`: formal method outcomes on each of 80 silver observations.
- `candidate_detections.csv`: candidates accepted at the formal operating points.
- `legacy_corrected_stage_survival.csv`: A′ target-neighborhood survival.
- `score_tradeoff.csv`: predeclared diagnostic score-threshold sweep.
- `figures/`: formal method and diagnostic trade-off plots.

Reproduce with:

```bash
python3 scripts/temporal_evidence_mvp.py freeze
python3 scripts/temporal_evidence_mvp.py run
python3 -m pytest -q tests/test_temporal_evidence_mvp.py tests/test_sparse_uav_mvp.py
```
