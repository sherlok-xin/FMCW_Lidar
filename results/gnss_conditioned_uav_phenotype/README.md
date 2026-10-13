# GNSS-conditioned UAV phenotype artifacts

**Completed oracle-conditioned descriptive study; not a detector/benchmark result.** See [the report](../../FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md). Completed outputs are exclusively in [`run_02`](run_02/README.md).

Root files preserve attempt 01 and successful preparatory workbook/geometry work. `point_records/100m_cross_5/frame_0005.npz` is a **failed/incomplete write**, not usable extracted observations. `extraction_failed.json` records the failure after six frames were seen. No completed full-bag identity or full-cohort return statistics exists for this attempt.

`run_02/protocol.json` freezes identical scientific definitions after an export-only fix. The user explicitly approved the rerun; `run_02/retry_approval.json` records this. Extraction and descriptive summary completed. Do not overwrite or retry either attempt in place; scripts refuse existing outputs.

Available machine-readable evidence:

- `gnss_source_samples.csv`: 72 original-workbook/E20-crosschecked rows, including device origin; opaque undocumented GNSS fields retained.
- `gnss_source_segments.csv`: 62 original-workbook endpoint intervals, all nine sequences; not instantaneous state GT.
- `alignment_sequence_summary.csv` / `sequence_inventory.csv`: PASS cohort and separate failed/ambiguous inventory.
- `geometry_from_existing_frame_inventory.csv`: 272 prior-frame timestamps with nominal/sensitivity geometry; explicitly metadata-only.
- `preparatory_accounting.csv`: 144 nominal supported frames, 80 saved observations, 64 anchors covered by existing scenario 3 m union.
- `workbook_crosscheck.json` / `metadata_preparation_execution.json`: provenance and execution identity.
- Original and corrected `protocol.json`: code/source SHA-256 identities; descriptive definitions unchanged.

Completed `run_02` outputs include `frames.csv`, `alignment_scenarios.csv`, `region_features.csv`, `rich_field_distributions.csv`, `rich_field_relationships.csv`, `control_exclusions.csv`, `raw_source_identity.csv`, `support_frequencies.csv`, `representation_support_frequencies.csv`, `descriptive_summaries.csv`, `matched_exploratory_effects.csv`, `within_sequence_associations.csv`, `region_empty_runs.csv`, `rich_duplicate_structure.csv`, figures and three independent audit JSON files. Full source/EOF and 272/144/80 accounting passed; E18 and 318 protected inputs remain unchanged.

Corrected per-frame NPZ format: `raw_point_bytes` plus `field__<name>` native columns, `field_schema_json`, `raw_slot_indices`, `region_names`, `region_offsets`, and `region_record_indices`. Region `i` indexes selected records via `region_record_indices[region_offsets[i]:region_offsets[i+1]]`; raw-slot IDs recover the original organized message. Point bytes/types are retained without float coercion, including out-of-order fields. All raw source records remain immutable.

GNSS-conditioned neighborhoods, observed-anchor neighborhoods and local clutter proxies remain distinct. No certified negative labels, classifier thresholds, trained weights, detector/tracker changes or independent test evidence are created.

Key cautions: GNSS 3 m regions are empty on 103/144 frames, not 103 proven sensor misses. Selected anchors yield 31 one-metre singleton cells but only one unique-XYZ singleton. Most small matched controls are empty (311/318 for anchor 3 m), so high count AUC is not representative clutter discrimination. High-radial, hover, precise positional GT and calibrated uncertainty remain UNKNOWN/DATA_GAP.
