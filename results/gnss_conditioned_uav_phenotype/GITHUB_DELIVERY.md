# GitHub delivery scope — 2026-10-13

This delivery publishes the completed GNSS-conditioned descriptive study only. E18, historical detector/tracker implementations, scientific definitions, silver labels and all 318 protected scientific inputs remain unchanged.

Included: the phenotype report; CSV/JSON results, hashes, protocols, failure/retry records, independent audits and figures; the eight diagnostic scripts and four test modules; the four updated project logs/status documents; and all **144 completed run_02 per-frame NPZ point-record exports** (5,099,049 bytes total).

The completed NPZ files are **study outputs, not disposable caches**. They retain selected oracle/control-region records and memberships so the rich-field, coordinate-coincidence and count audits can be reproduced without transferring the original full bags. They are explicitly staged from `run_02/point_records/` despite the repository's generic `*.npz` ignore rule; that ignore rule itself is not changed. These records are not isolated UAV point GT or independent test data. Research GNSS locations and local source paths are disclosed as part of the already-authorized research provenance.

Excluded and preserved locally: original bags/ZIPs/PCDs, environments/general caches, `.local/`, the **corrupt attempt-01 NPZ**, and unrelated pending literature-matrix/data-collection-protocol work. Attempt-01 JSON, its frozen protocol and the corrupt file's hash/size in `run_02/artifact_inventory.csv` are published; the failed NPZ must not be consumed.

The original handoff/artifact inventories describe the completed study snapshot before this additive shipping note and before the new Git commit. Their old Git HEAD and timestamps are historical provenance, not claims that the new delivery commit existed during acquisition/extraction. No sealed experiment artifact is rewritten for publication.
