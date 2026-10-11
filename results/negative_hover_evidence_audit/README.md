# E20 negative / hover evidence inventory

2026-10-11; parent `5ad38b94b87949657f803485da129e3a738e9a64`, frozen E18 `a53702da1dc5357149e3e472a9e489ff08e9afe4`.

Read [the report](../../FMCW_Negative_Hover_Evidence_Audit.md) first. Metadata/provenance analysis, **not a new detector run**. `DATA_GAP`: no independently qualified negative or slow/hover window. E18 scores/weights/graphs/labels/gates unchanged; no Doppler or threshold selection.

## Files

- `protocol.json`, `freeze_identity.json`, `protected_input_identity.json`, `protected_input_validation.json`: qualification rules, 302 protected old-file identities and full E18/E19 result trees.
- `raw_bag_inventory.json/csv`, `archive_member_inventory.csv`, `pcd_inventory.csv`, `data_file_inventory.csv`: all bag/data paths, ZIP members, bounded bag header/topic probes and scene EXIF. No full cloud or ZIP bag tail decode.
- `gnss_workbook_inventory.json`, `gnss_samples.csv`, `gnss_adjacent_segments.csv`, `gnss_condition_summary.csv`: complete nonempty workbook cells/metadata, 71flight+device rows, 62endpoint segments. Opaque fields UNKNOWN; chord speed is not instantaneous state.
- `log_source_inventory.csv`, `log_history_coverage.json`: exact locators and project-path Git coverage. Old audits/memories are not new collection events.
- `silver_motion_segments.csv`, `silver_continuous_fragments.csv`: all75adjacent observed-center steps /10continuous fragments, no new slow threshold or labels.
- `prior_e19_slow_score_trace.csv`: 168 previously saved witness nodes, 160observed-silver+8nonlabel connectors; not new stress/recomputed scores.
- `candidate_windows.csv`: 80 overlapping inventory records, not80independent windows or80labels. L0/L2/L1=57/11/12; qualification=False, unknowns retained.
- `stress_execution_decision.json`: `NOT_RUN_DATA_GAP`; reliable negative/hover scores UNKNOWN.
- `negative_stress_winners.csv`, `qualified_hover_score_trace.csv`: header-only schemas, not zero-valued results.
- `*_execution.json`: exact commands/env/source/input/output hashes; `preflight_failure.json` and log execution retain failed inventory attempts.
- `validation_checks.json`, `final_deliverables_manifest.json`: independent count/hash/identity/link checks and final delivery identities.

## Recorded commands

```bash
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_evidence_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_bag_inventory.py --output-dir results/negative_hover_evidence_audit --prefix-bytes 1048576
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_gnss_inventory.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_log_inventory.py --out results/negative_hover_evidence_audit
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_window_inventory.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_evidence_audit.py check
```

Inventories ran in parallel. Qualification freeze preceded synthesis, not all exploratory source inspection. Use recorded Python3.10 environment; workbook falls back to standard-library ZIP/XML since openpyxl is absent. No package installation or protected-source edits.

Scripts refuse overwrites. Full replay: a separate copy at the E19 parent, add these E20 inventory scripts, make original raw data/caches available read-only, leave E20 output directory absent. Bag/log helpers also support a different new output directory; GNSS/synthesis/freeze use the recorded default. Do not delete/move retained results. The optional external legacy project-memory file is only provenance; if unavailable record UNKNOWN. Freeze checks the specified parent rather than treating later HEAD as the same run identity.

E19 is already on GitHub main. New E20 files are not part of that historical commit. Unrelated `.local/`, raw and cache data remain untouched.
