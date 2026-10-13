#!/usr/bin/env python3
"""Export geometry from the existing frame inventory; never read raw bags.

This preparatory inventory is not the completed rich-return phenotype study.
"""
import json
from pathlib import Path

import gnss_conditioned_uav_phenotype as study
import gnss_phenotype_geometry as geometry


def main():
    root, out = study.ROOT, study.OUT
    context = geometry.load_geometry(root)
    rows, accounting = [], []
    for cid in study.SEQUENCES:
        source = root / f"results/calibration_closure/raw_voxel_cache/{cid}.json"
        frames = json.loads(source.read_text())["frames"]
        selected = []
        for frame in frames:
            flat, scenarios = geometry.annotate_frame(context, cid, frame["bag_time"], frame["frame_index"])
            selected.append({"sequence_id": cid, "source": str(source.relative_to(root)),
                             "source_sha256": study.sha(source), "evidence": "EXISTING_FRAME_METADATA_ONLY_NOT_NEW_RAW_EXTRACTION", **flat})
        supported = sum(r["gnss_nominal_supported"] for r in selected)
        anchors = [r for r in selected if r["observed_closure_anchor_available"]]
        expectation = study.SETTINGS["expected_counts"][cid]
        assert [len(selected), supported, len(anchors)] == expectation
        accounting.append({"sequence_id": cid, "inventory_frames": len(selected),
                           "nominal_gnss_supported": supported, "observed_closure_anchors": len(anchors),
                           "anchors_inside_existing_scenario_3m_union": sum(r["anchor_in_existing_scenario_3m_union"] for r in anchors),
                           "raw_rich_cohort_extraction_complete": False})
        rows.extend(selected)
    study.cwrite(out / "geometry_from_existing_frame_inventory.csv", rows)
    study.cwrite(out / "preparatory_accounting.csv", accounting)
    study.jwrite(out / "metadata_preparation_execution.json", {
        "utc": study.utc(), "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/gnss_phenotype_prepare_metadata.py",
        "script_sha256": study.sha(Path(__file__)), "source_sha256": context["source_hashes"],
        "rows": len(rows), "accounting": accounting,
        "boundary": "cached frame timestamps with unchanged GNSS/closure annotations; no new return statistics"})
    print(json.dumps(accounting))


if __name__ == "__main__":
    main()
