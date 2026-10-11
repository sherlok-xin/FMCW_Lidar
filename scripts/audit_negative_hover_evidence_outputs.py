#!/usr/bin/env python3
"""Read-only E20 accounting checker; writes only new validation/manifest files."""
from __future__ import annotations

import ast
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/negative_hover_evidence_audit"


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def read(name):
    return json.loads((OUT / name).read_text())


def csv_rows(name):
    with (OUT / name).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main():
    for name in ("validation_checks.json", "final_deliverables_manifest.json"):
        if (OUT / name).exists():
            raise FileExistsError(f"Refusing to overwrite {name}")
    protected = read("protected_input_identity.json")
    for rel, expected in protected.items():
        assert sha(ROOT / rel) == expected, rel
    frozen = read("freeze_identity.json")
    trees = sorted(str(p.relative_to(ROOT)) for tree in
                   ("results/background_motion_dp_mvp", "results/e18_mechanism_generalization_audit")
                   for p in (ROOT / tree).rglob("*") if p.is_file())
    assert trees == frozen["protected_tree_paths"]
    assert sha(OUT / "protocol.json") == frozen["protocol_sha256"]
    assert sha(ROOT / "scripts/negative_hover_evidence_audit.py") == frozen["source_sha256"]
    executions = {
        "bag_inventory_execution.json": ("negative_hover_bag_inventory.py", "output_hashes"),
        "gnss_evidence_execution.json": ("negative_hover_gnss_inventory.py", "output_sha256"),
        "log_evidence_execution.json": ("negative_hover_log_inventory.py", "outputs_sha256"),
        "window_inventory_execution.json": ("negative_hover_window_inventory.py", "output_hashes")}
    for name, (script, output_key) in executions.items():
        execution = read(name)
        expected = execution.get("script_sha256", execution.get("source_sha256"))
        assert sha(ROOT / "scripts" / script) == expected, script
        for rel, digest in execution[output_key].items():
            assert sha(OUT / rel) == digest, rel
    bag_exec = read("bag_inventory_execution.json")
    before, after = bag_exec["raw_data_stat_identity_before"], bag_exec["raw_data_stat_identity_after"]
    assert before == after and len(before) == 697
    for rel, expected in before.items():
        stat = (ROOT / rel).stat()
        assert (stat.st_size, stat.st_mtime_ns, stat.st_mode) == (
            expected["size_bytes"], expected["mtime_ns"], expected["mode"]), rel
    for name, key in (("log_evidence_execution.json", "inputs_sha256"),
                      ("window_inventory_execution.json", "input_sha256")):
        for rel, expected in read(name)[key].items():
            path = Path(rel) if rel.startswith("/") else ROOT / rel
            assert sha(path) == expected, rel
    gnss = read("gnss_evidence_execution.json")
    assert sha(ROOT / "bag/20260205/飞行轨迹坐标.xlsx") == gnss["input_sha256"]
    assert sha(ROOT / gnss["verified_conversion_source"]) == gnss["verified_conversion_source_sha256"]
    expected_rows = {"raw_bag_inventory.csv": 57, "archive_member_inventory.csv": 71,
        "data_file_inventory.csv": 697, "pcd_inventory.csv": 8, "gnss_samples.csv": 72,
        "gnss_adjacent_segments.csv": 62, "gnss_condition_summary.csv": 9,
        "log_source_inventory.csv": 19, "silver_motion_segments.csv": 75,
        "silver_continuous_fragments.csv": 10, "prior_e19_slow_score_trace.csv": 168,
        "candidate_windows.csv": 80, "negative_stress_winners.csv": 0,
        "qualified_hover_score_trace.csv": 0}
    for name, expected in expected_rows.items():
        assert len(csv_rows(name)) == expected, name
    members = csv_rows("archive_member_inventory.csv")
    nondir = [r for r in members if r["is_directory"] == "False"]
    assert len(nondir) == 52
    assert sum(r["member"].endswith(".bag") for r in nondir) == 51
    bags = csv_rows("raw_bag_inventory.csv")
    archived = [r for r in bags if r["source_kind"] == "ZIP_BAG_PREFIX"]
    assert len(archived) == 51
    assert all(int(r["bytes_read"]) <= 1048576 for r in archived)
    assert sum(int(r["bytes_read"]) for r in archived) == 53477376
    assert all(r["all_declared_connections_observed"] == "True" for r in bags)
    assert all(int(r["declared_connection_count"]) == int(r["observed_connection_count"]) for r in bags)
    assert sum(int(r["pcd_files"]) for r in csv_rows("pcd_inventory.csv")) == 674
    for r in csv_rows("gnss_adjacent_segments.csv"):
        assert math.isclose(float(r["chord_speed_mps"]), float(r["chord_distance_m"]) / float(r["dt_s"]), rel_tol=1e-12)
        assert r["formal_evaluation_qualified"] == "False"
    segments = csv_rows("silver_motion_segments.csv")
    assert sum(int(r["frame_gap"]) > 1 for r in segments) == 5
    for r in segments:
        assert float(r["dt_s"]) > 0
        assert math.isclose(float(r["observed_center_speed_mps"]), float(r["displacement_m"]) / float(r["dt_s"]), rel_tol=1e-12)
    trace = csv_rows("prior_e19_slow_score_trace.csv")
    assert sum(r["observed_silver_frame"] == "True" for r in trace) == 160
    for r in trace:
        assert r["execution_status"] == "PREVIOUS_E19_STORED_DIAGNOSTIC_NOT_NEW_STRESS_TEST"
        source = list(csv.DictReader((ROOT / r["source_file"]).open(encoding="utf-8-sig")))
        matched = next(s for s in source if s["node_id"] == r["node_id"])
        for key in ("background_term", "augmented_node_score", "old_node_score", "prior_frames", "prior_hits"):
            assert float(r[key]) == float(matched[key]), (r["node_id"], key)
    windows = csv_rows("candidate_windows.csv")
    assert Counter(r["evidence_level"] for r in windows) == Counter(L0=57, L2=11, L1=12)
    assert all(r["stress_qualified"] == r["formal_evaluation_qualified"] == "False" for r in windows)
    decision = read("stress_execution_decision.json")
    assert decision["new_solver_runs"] == 0
    assert decision["qualified_negative_windows"] == decision["qualified_slow_hover_windows"] == []
    scripts = sorted(ROOT.glob("scripts/*negative_hover*.py"))
    for path in scripts:
        ast.parse(path.read_text(), filename=str(path))
    documents = [ROOT / "FMCW_Negative_Hover_Evidence_Audit.md", OUT / "README.md"]
    for doc in documents:
        for target in re.findall(r"\]\(([^)]+)\)", doc.read_text()):
            if target.startswith(("https://", "http://", "#")):
                continue
            assert (doc.parent / target.split("#")[0]).exists(), (doc, target)
    assert subprocess.check_output(["git", "diff", "--name-only"], cwd=ROOT, text=True).splitlines() == [
        "docs/CURRENT_STATUS.md", "docs/EXPERIMENT_LOG.md", "docs/RESEARCH_LOG.md", "docs/TODO.md"]
    subprocess.run(["git", "diff", "--check"], cwd=ROOT, check=True)
    validation = {"utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "status": "PASS", "reviewer": "root artifact checker", "protected_files": len(protected),
        "protected_trees_unchanged": True, "raw_file_stats_unchanged": 697,
        "script_AST_count": len(scripts), "csv_row_counts": expected_rows,
        "archive_nondirectory_members": 52, "archive_directory_members": len(members)-52,
        "gnss_segment_accounting": "PASS", "silver_step_accounting": "PASS",
        "prior_E19_source_trace_equality": "PASS_168_STORED_ROWS_NOT_NEW_SCORES",
        "qualified_stress_window_count": 0, "new_E18_solver_runs": 0,
        "local_links_and_diff_check": "PASS",
        "scientific_generalization_verification": "UNKNOWN_DATA_GAP",
        "subagent_final_report_reviews": "NOT_COMPLETED_USAGE_LIMIT; root checker performed final review"}
    (OUT / "validation_checks.json").write_text(json.dumps(validation, indent=2) + "\n")
    deliverables = documents + scripts + [ROOT / rel for rel in (
        "docs/CURRENT_STATUS.md", "docs/EXPERIMENT_LOG.md", "docs/RESEARCH_LOG.md", "docs/TODO.md")]
    deliverables += [p for p in OUT.rglob("*") if p.is_file()]
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in sorted(set(deliverables))}
    manifest = {"utc": datetime.now(timezone.utc).isoformat(), "parent_commit": "5ad38b94b87949657f803485da129e3a738e9a64",
        "status": "LOCAL_E20_DELIVERABLES_PRIOR_E19_ALREADY_PUSHED", "file_count_excluding_self": len(hashes),
        "sha256": hashes, "raw_bags_PCDs_cache_or_local_included": False}
    (OUT / "final_deliverables_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(validation))


if __name__ == "__main__":
    main()
