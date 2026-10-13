# Completed oracle-conditioned phenotype — run 02

Read [the report](../../../FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md) first. This is descriptive DEV/oracle evidence, not a benchmark or independent test. E18, labels, graph and gates are unchanged. Original failed artifacts remain in the parent directory.

## Main artifacts

| File | Content and evidence boundary |
|---|---|
| `protocol.json`, `retry_approval.json` | Pre-extraction definitions/source hashes and explicit export-only retry approval. Scientific definitions match attempt 01. |
| `extraction_started.json`, `extraction_completed.json` | Command/environment, elapsed 905.881 s, five raw-source audits; do not confuse stream identity with independent rehash. |
| `raw_source_identity.csv` | SHA-256 of compressed ZIPs and entire uncompressed bag members, CRC/byte/EOF accounting, topic/connection declaration. |
| `frames.csv` | All 272 rich frames; actual record/header times, frame ID/sequence/schema/slots, nominal GNSS support and existing-anchor availability. |
| `gnss_source_samples.csv`, `gnss_source_segments.csv`, `workbook_crosscheck.json` | Read-only workbook rows, source cells, opaque fields and 62 endpoint-chord intervals across all nine sequences; not instantaneous-state truth. |
| `sequence_inventory.csv`, `alignment_sequence_summary.csv` | Five PASS cross sequences included; four failed/ambiguous sequences separate and not negatives. |
| `alignment_scenarios.csv` | 4,885 in-support states from fixed prior offsets/poses; no extrapolation or calibrated confidence bounds. |
| `region_features.csv` | 3,602 recipe/frame records: 736 UAV-conditioned, 2,866 matched proxies. Includes empty regions, rich-field summaries and raw/unique/cell geometry. |
| `control_exclusions.csv` | Geometry-only proxy rejection records; no post-outcome control replacement. |
| `rich_field_distributions.csv`, `rich_field_relationships.csv` | 4,352 per-field/frame and 272 empirical relationship rows; physical semantics of undocumented fields UNKNOWN. |
| `support_frequencies.csv`, `representation_support_frequencies.csv` | Raw and raw/unique/4 cm/1 m empirical occupancy categories by recipe/range/eta; not physical detection probabilities. |
| `descriptive_summaries.csv`, `within_sequence_associations.csv` | Frame-level distributions and descriptive rho; no p-values, causal range law or weights. |
| `matched_exploratory_effects.csv` | 3,240 same-frame effect rows, fixed higher-UAV AUC orientation, Cliff delta, paired differences, IQR scaling and missingness. 1,532 AUC rows undefined. |
| `region_empty_runs.csv` | 70 supported zero-region runs, 28 distinct observation gaps; actual-time spans, knot/support breaks and censoring. |
| `rich_duplicate_structure.csv` | 80 observed-anchor raw/unique support and coincident-coordinate group diagnostics; not literal byte copies or physical-return identity GT. |
| `independent_*_audit.json`, `validation_checks.json` | Source/arithmetic/export checks with explicit scientific scope limits. |
| `figures/`, `figure_manifest.json`, `support_figure_v2_manifest.json` | CSV-derived scatter/time plots, no fitted curves. Original support layout retained; v2 fixes labels only. |

## Field/metric definitions

- `nominal_range_m`, `nominal_speed_mps`: filename/condition labels. `range_m`, `speed_mps`: provisional GNSS-to-sensor range and endpoint-chord speed. `region_reference_range_m`: observed-anchor/reference range where applicable, not independent true UAV range.
- XYZ analysis coordinates: `[sensor_x, sensor_z, -sensor_y]`. Raw coordinate fields remain untouched in NPZ. Voxel occupancy uses floor(XYZ/size) with sensor-zero origin, not leveled/gravity coordinates.
- `gnss_supported`: nominal query inside GNSS sample endpoints. Valid scenarios are separately recorded; scenario envelopes are not probability/accuracy certificates.
- `alignment_scenario`: **region recipe**, e.g. `GNSS_3m`, `ANCHOR_3m`, `SCENARIO_UNION_3m`. In the separate alignment CSV, `scenario_id` identifies a pose/offset sensitivity state. Do not conflate them.
- `region_role`: `UAV_CONDITIONED` or `LOCAL_CLUTTER_PROXY`; neither supplies point-level identity GT. Controls preserve range/elevation via fixed azimuth rotations and can overlap one another.
- `raw_points`: finite XYZ with at least one nonzero coordinate, no intensity/height/motion rejection. `unique_xyz_points`: exact XYZ triples, not necessarily independent physical returns. `occupied_voxels_*`: representation-dependent cells.
- Observed geometry/covariance is centered on observed support; LOS derives from the recipe reference. Empty features are blank/unknown; singleton geometry is degenerate, not physical body size. NN/MST require sufficient distinct support; exact pairwise budget failures are flagged.
- `*_signed_range_rate_residual_mps`: `−median(measured field)−nominal GNSS chord range-rate`. Opposite sign and existing sensitivity limits also retained. Working m/s convention is not manufacturer-certified. Proxy residuals reference the hypothetical UAV-motion template, not proxy object's true velocity.
- Intensity is uncalibrated field units. Ring is stored uint32; physical beam semantics UNKNOWN. Time is stored float32; units/anchor UNKNOWN. Constant acceleration/evenness and sign direction fields are not independent physical motion/shape measurements.
- Matched effects average controls within a frame, then frames equally; source frames remain temporally dependent. Higher-value AUC is not always desirable (e.g. Doppler residual). Empty controls support count comparison but not intensity/NN/Doppler comparison. AUC missingness is explicit.

## Native per-point exports

There are 144 `point_records/<sequence_id>/frame_XXXX.npz` files. Load with `numpy.load(path, allow_pickle=False)`.

- `raw_slot_indices`: original flattened organized-message indices, not reindexed detections.
- `raw_point_bytes`: exact original point bytes, including the out-of-order field layout.
- `field__<name>`: native integer/float fields, no common floating-point coercion.
- `field_schema_json`: offsets/datatype/count/endian-related declarations, with semantics UNKNOWN.
- `region_names`, `region_offsets`, `region_record_indices`: recipe memberships in the exported union. For region `i`, take `region_record_indices[region_offsets[i]:region_offsets[i+1]]`, then index named columns or raw-slot IDs.

The exports union overlapping regions to avoid storage duplication; membership retains overlap. The union has 76,785 stored records, not 76,785 UAV points. Small controls are predominantly empty. Neither this export nor any effect table defines negative labels or a final classifier.
