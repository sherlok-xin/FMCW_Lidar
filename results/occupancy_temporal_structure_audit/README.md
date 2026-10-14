# Occupancy temporal-structure audit

**TESTED / descriptive only.** [Main report](../../FMCW_OCCUPANCY_TEMPORAL_STRUCTURE_AUDIT.md). No E18 edits, new score, classifier, regression, weight/threshold/window search, path rescore or DP. This audit is not an E21 background model.

The strict five-sequence kill criterion fails: **E18 occupancy is sufficient at the current evidence level; E21 background model is not justified.** Local ordering differences exist, but cross-sequence nondegenerate same-frequency contrast support is insufficient. This is an evidence-level decision, not mathematical sufficiency or proof of zero effects.

## Fixed definitions and execution

Working directory: `/home/xin/FMCW_LIDAR`. Existing environment: `fmcw/bin/python` (Python3.10.12, NumPy2.2.6). No installation or random sampling.

Historical source is solely the **fine_1m** events in the prior [scene audit](../scene_spatiotemporal_background_audit/README.md), using exactly the existing E18 frequency reference. The five original compressed cell tables were frozen before the prior anchor/path joins. They contain all scene fine-input cell events, not only node/path events. Fine inputs inherit historical ROI/intensity/calibrated-height preprocessing and are not full raw scene evidence. No bags, new candidate extraction or new scene map are involved.

The four structure quantities are bag-time recency, longest past consecutive cell run, previous occupied episodes, and hit fraction in the **five preceding frames**. The fifth quantity is unchanged E18 p=(H+1)/(F+2). The new recent fraction is derived from immutable pre-join events; the original scene outputs did not already store this column. All original H/p/age/run/episode values are verified, with declared F=0 UNKNOWN masking only. F<5 recent fraction is UNKNOWN; H=0 recency is UNKNOWN. Known empty history at F>0 gives run=episodes=0, not a cold-start imputation.

Window, bins and kill rules were frozen before new temporal contrasts; prior scientific reports/outcomes were already known, so this is not a blind held-out preregistration. Five preceding acquisition frames were chosen from approx1Hz cadence, not anchor performance. Actual window elapsed time is stored; it is not always5s. Only one window length is calculated.

Exact executed commands:

```bash
cd /home/xin/FMCW_LIDAR
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/temporal_audit.py freeze
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/temporal_audit.py compute
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/interpret_temporal.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/independent_temporal_validation.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/independent_conditioning_audit.py
PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/plot_temporal.py
```

Independent validation began after feature completion and in parallel with interpretation; it initially did not parse joined outcomes. Conditional validation followed interpretation. Plotting uses stored tables only. Command/source/environment/input/output identities are in the manifests and validation JSONs. `freeze`/`compute` refuse completed-output overwrite. Replay in a fresh sibling output directory under repository `results/` with the scripts copied together; original scene tables and graph/path files remain required. A GitHub clone without local scene NPZs cannot run the full validator.

## Machine-readable outputs

| Artifact | Contents |
|---|---|
| [protocol.json](protocol.json), [FROZEN](FROZEN), [freeze_manifest.json](freeze_manifest.json) | Fixed definitions,5-prior-frame window,.05 bins,conditioning/kill rule,cadence input identities and source SHA |
| [acquisition_cadence.csv](acquisition_cadence.csv) | Five bags,median/IQR/min/max real Δt,chosen before new joined contrasts |
| [all_node_temporal_features.csv](all_node_temporal_features.csv), [feature_identity.csv](feature_identity.csv) | All20,122fine and16,995historical graph nodes; H,F,H/F,p,recency,run,episodes,recent,last-frame/support state,actual window time span |
| [TEMPORAL_FEATURES_COMPLETE.json](TEMPORAL_FEATURES_COMPLETE.json) | Feature-completion receipt and hashes, preceding any new anchor/path parsing |
| [population_temporal_features.csv](population_temporal_features.csv), [population_exposure.csv](population_exposure.csv), [H_stratum_support.csv](H_stratum_support.csv) | All seven specified populations; overlap identities, durations/frame exposure,cold/never-hit/H0/H1/H≥2 support |
| [frequency_bin_support.csv](frequency_bin_support.csv), [frequency_bin_distributions.csv](frequency_bin_distributions.csv), [frequency_bin_contrasts.csv](frequency_bin_contrasts.csv) | Fixed narrow p bins,finite n/linear quantiles,frame/H support and dependent-sample descriptive contrasts; not incremental inference or classifier metrics |
| [same_frame_exact_H_pairs.csv](same_frame_exact_H_pairs.csv), [exact_frequency_contrast_summary.csv](exact_frequency_contrast_summary.csv) | Same-frame exactH comparisons by original population; explicitly NONMATCHING_H,FEATURE_UNKNOWN,identical-cell ties and distinct-cell comparisons |
| [kill_frame_contrasts.csv](kill_frame_contrasts.csv), [kill_sequence_summary.csv](kill_sequence_summary.csv), [kill_decision.json](kill_decision.json) | Frozen warm/nondegenerate comparison pool,control-cell deduplication,equal frame weight,support/sign/strict stopping decision |
| [all_fine_same_frame_H_structure_spread.csv](all_fine_same_frame_H_structure_spread.csv), [global_reference_associations.csv](global_reference_associations.csv) | Numeric nonredundancy at fixed H/F; global correlations are exposure-confounded context, not an incremental test |
| [interpretation_manifest.json](interpretation_manifest.json) | Existing anchor IDs and selected/fixed path identities loaded only after feature completion; no rescore |
| [validation.json](validation.json), [tests.txt](tests.txt) | Eight synthetic tests; independent37,117node/148,656cell-event arithmetic and437protected/18input hashes |
| [independent_conditioning_audit.json](independent_conditioning_audit.json) | Independent1,600pair/320frame/20sequence/kill decision check |
| [figure_manifest.json](figure_manifest.json), [support PNG](temporal_support_direction.png), [bin-versus-exact PNG](temporal_bins_vs_exact_frequency.png) | PDF+300dpiPNG of support/direction and fixed-bin exposure comparison; UNKNOWN/low support visible |
| [final_manifest.json](final_manifest.json) | Final report/artifact SHA and scope/links/append-only integrity validation |

IDs are local to sequence and graph family. H/F is an occupancy record frequency, p is the inherited Beta(1,1) proxy, not calibrated clutter probability. Centroid-cell history may mix objects or surface sampling. The 84 gate-matched nodes and80nearest nodes are selected observed DEV_SILVER associations, not independent GT. Fixed witnesses and joint winners often share the same cells; deterministic ties are not negative-control evidence. Historical persistent-reference names do not certify clutter labels.

Primary test uses identical frame,H,F,p. Narrow-bin populations can differ in frame/H and sample span; these cannot substitute for the primary test. F≥5 comparison and fixed chronological thirds are exposure controls, not selected windows. No path durations or future frames enter node statistics. No support is recorded as UNKNOWN, not a zero difference.

Local 100m_cross_10 recency/recent differences are supported by only frames22–24;300m_cross_10 by only frame10. The other three sequences lack distinct-cell,nondegenerate exact-frequency competitor support. All quantities fail the frozen five-sequence criterion; numeric temporal-order information is not proof of detector value. No E21 is proposed. This new audit has not been committed or pushed.
