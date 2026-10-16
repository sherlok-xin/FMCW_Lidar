# Project reinterpretation / historical pipeline audit — 2026-10-16

**Documentation/source audit only; no new point-cloud processing or algorithm experiment.**

- [Main reinterpretation](../../FMCW_PROJECT_REINTERPRETATION_2026-10-16.md)
- [Historical preprocessing audit](../../FMCW_HISTORICAL_PREPROCESSING_AUDIT.md)
- [Eight-slide Chinese group brief](../../FMCW_GROUP_MEETING_RESET_BRIEF.md)
- `pipeline_provenance.csv`: 27 source stages/variants, input/operation/output/default parameters, provenance and limits.
- `old_vs_current_preprocessing.csv`: 14 historical-versus-fine/E18 comparisons.
- `conclusion_reassessment.csv`: 19 conclusion classifications; history and negative evidence preserved.
- `misunderstanding_ledger.csv`: 10 possible misreadings and corrections; earlier disclosures distinguished from actual new context.
- [Proposed historical reproduction plan](proposed_historical_reproduction_plan.md): full exact chain BLOCKED; three fixed nominal-range bags planned, NOT_RUN.
- `validation.json`: protected prior bytes, append-only handoff, file/link/CSV and existing 272/80/56/79 crosschecks.
- `audit_manifest.json`: source identities, scope, commands and new artifact SHA-256.

The new acquisition geometry is user testimony from this task, not a surveyed pose or new labels. The inherited height mask uses provisional closure; existing anchors already use q=[raw_x,raw_z,-raw_y]. Transform raw points once; do not transform q anchors again. The exact historical cleaned RViz topic remains UNKNOWN. No historical placeholder commands or new stage-loss percentages are invented.

The 96.60–98.07% raw figure is the original causal **persistent-set** return fraction at F>=2 (past H>=2 and p_beta11>=0.5), not arbitrary prior-revisit. Any previously seen cell return fraction is98.82–99.70%. Fine retained raw-support6.2146% is a different prefiltered population and is not target retention.

Commands used for this documentation audit included `git status --short`, `git log --all --format=... -- get_lidar.py multi_filter_with_intensity.py`, `rg --files`, `rg -n`, `nl -ba`/`sed` source reads, existing CSV/JSON inspection, and `git diff --check`. Local scratch authoring and validation commands were `python3 /tmp/write_fmcw_reinterpretation.py` (initial draft, then reviewed corrections) and `python3 /tmp/validate_fmcw_reinterpretation.py`; scratch scripts are not scientific methods or historical pipeline implementations. Final artifacts, rather than the initial draft generator, are authoritative.

No raw bag/PCD/cache was processed. No E18/source/label/calibration/threshold/historical-result edits. Four project status/log documents were append-only extended as required by AGENTS. Pre-existing unrelated changes and prior unreviewed Stone Soup artifacts were preserved. STOP_DEV_FEATURE_MINING continues. No commit or push; user review required.
