#!/usr/bin/env python3
"""Seal descriptive handoff provenance; no raw data/method execution."""
import csv
import json
import subprocess
from pathlib import Path

import gnss_conditioned_uav_phenotype as study


def main():
    root, out = study.ROOT, study.OUT / "run_02"
    protocol = study.verify(out)
    prior = json.loads((study.OUT / "protocol.json").read_text())
    skip = {"code_sha256", "frozen_utc"}
    assert {k: v for k, v in protocol.items() if k not in skip} == {k: v for k, v in prior.items() if k not in skip}
    rows = []
    for path in sorted(study.OUT.rglob("*")):
        if not path.is_file():
            continue
        count = None
        if path.suffix == ".csv":
            with path.open(encoding="utf-8-sig", newline="") as f:
                count = max(sum(1 for _ in csv.reader(f)) - 1, 0)
        rows.append({"path": str(path.relative_to(root)), "bytes": path.stat().st_size,
                     "sha256": study.sha(path), "csv_data_rows": count,
                     "evidence_status": "FAILED_ATTEMPT_DO_NOT_CONSUME" if path == study.OUT / "point_records/100m_cross_5/frame_0005.npz" else
                         "FAILED_ATTEMPT_PROVENANCE" if path.name == "extraction_failed.json" else
                         "COMPLETED_RUN_02" if out in path.parents else "PREPARATORY_OR_FAILED_ATTEMPT_PROVENANCE"})
    study.cwrite(out / "artifact_inventory.csv", rows)
    files = [root / "FMCW_GNSS_CONDITIONED_UAV_PHENOTYPE.md"]
    files += sorted((root / "scripts").glob("gnss*phenotype*.py"))
    files += sorted((root / "tests").glob("test_gnss*.py"))
    files += [root / "docs" / name for name in ["CURRENT_STATUS.md", "EXPERIMENT_LOG.md", "RESEARCH_LOG.md", "TODO.md"]]
    audits = ["independent_geometry_audit.json", "independent_rich_audit.json", "independent_statistics_audit.json"]
    study.jwrite(out / "handoff_manifest.json", {
        "utc": study.utc(), "status": "COMPLETED_ORACLE_CONDITIONED_DESCRIPTIVE_STUDY",
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/gnss_phenotype_finalize.py",
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "report_code_test_log_sha256": {str(p.relative_to(root)): study.sha(p) for p in files},
        "artifact_inventory_sha256": study.sha(out / "artifact_inventory.csv"),
        "independent_audit_sha256": {name: study.sha(out / name) for name in audits},
        "guarded_scientific_inputs_unchanged": len(protocol["source_sha256"]),
        "retry_scientific_definitions_equal": True, "test_count": 50,
        "method_experiments_run": 0, "E18_modified": False, "labels_modified": False,
        "threshold_or_weights_selected": False, "independent_test_evidence": False,
        "full_study_second_reproduction": "NOT_RUN",
        "remaining_data_gaps": ["exact point/positional GT", "calibrated localization uncertainty",
                                "independently certified negatives", "populated small matched clutter controls",
                                "high-radial-observability trajectories", "independently verified hover/slow transitions"],
        "historical_failure_preserved": "../extraction_failed.json"})
    print(json.dumps({"artifact_files": len(rows), "protected_inputs": len(protocol["source_sha256"]),
                      "definitions_unchanged": True, "status": "SEALED"}))


if __name__ == "__main__":
    main()
