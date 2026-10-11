#!/usr/bin/env python3
"""E20 evidence-only freeze/check. Never imports or executes an E18 solver."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/negative_hover_evidence_audit"
FROZEN_E18 = "a53702da1dc5357149e3e472a9e489ff08e9afe4"
PRIOR_AUDIT = "5ad38b94b87949657f803485da129e3a738e9a64"
TREES = ("results/background_motion_dp_mvp", "results/e18_mechanism_generalization_audit")


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write(name: str, value: dict) -> None:
    path = OUT / name
    if path.exists():
        raise RuntimeError(f"Refusing to overwrite {path}")
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def tree_paths() -> list[str]:
    return sorted(str(p.relative_to(ROOT)) for tree in TREES
                  for p in (ROOT / tree).rglob("*") if p.is_file())


def freeze() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if head != PRIOR_AUDIT:
        raise RuntimeError("Expected the already-pushed E19 parent")
    protected = json.loads((ROOT / TREES[1] / "input_identity.json").read_text())
    extra = tree_paths() + [
        "FMCW_Background_Motion_DP_MVP.md", "FMCW_E18_Mechanism_Generalization_Audit.md",
        "scripts/background_motion_core.py", "scripts/background_motion_dp_mvp.py",
        "scripts/e18_mechanism_audit.py", "scripts/e18_mechanism_summaries.py",
        "scripts/audit_e18_mechanism_outputs.py",
        "tests/test_background_motion_core.py", "tests/test_background_motion_wrapper.py",
        "tests/test_e18_mechanism_audit.py", "gt_manifest.csv",
        "scripts/build_gt_benchmark_v0.py", "scripts/calibration_closure.py",
        "bag/20260205/飞行轨迹坐标.xlsx", "bag/20260205/场景图.jpg",
    ]
    for rel in extra:
        protected[rel] = sha(ROOT / rel)
    for rel, expected in protected.items():
        if sha(ROOT / rel) != expected:
            raise RuntimeError(f"Existing protected identity mismatch: {rel}")
    frozen_source_checks = {}
    for rel in ("scripts/background_motion_core.py", "scripts/background_motion_dp_mvp.py"):
        blob = subprocess.check_output(["git", "show", f"{FROZEN_E18}:{rel}"], cwd=ROOT)
        expected = hashlib.sha256(blob).hexdigest()
        if sha(ROOT / rel) != expected:
            raise RuntimeError(f"E18 source differs from a53702d: {rel}")
        frozen_source_checks[rel] = expected
    write("protocol.json", {
        "experiment": "E20", "version": "evidence_audit_v1", "parent_commit": head,
        "frozen_E18_commit": FROZEN_E18,
        "mode": "READ_ONLY_EXISTING_EVIDENCE", "randomness": "NONE",
        "negative_qualification": ["independent target identity and explicit absence evidence",
            "bounded interval, clock linkage to a bag", "LiDAR stream available",
            "absence in LiDAR FOV established, not just missing labels/returns"],
        "slow_hover_qualification": ["independent target identity and measured/recorded motion state",
            "bounded interval with clock linkage", "target visibility in available LiDAR stream",
            "sampling and localization uncertainty sufficient to resolve slow/hover state"],
        "formal_evaluation_additional_requirements": ["independent annotations and time/space validation",
            "no reuse of retrospective DEV silver as final GT", "held-out evaluation protocol"],
        "evidence_levels": {"L3": "independent timestamp-linked certified window",
            "L2": "independent sensor/context record but incomplete event/identity/time/FOV linkage",
            "L1": "LiDAR-derived DEV_SILVER or retrospective pseudo observation",
            "L0": "name/inference/unsupported annotation"},
        "stress_execution_gate": "ONLY qualified windows with existing unchanged frozen E18 graph",
        "if_no_qualified_window": "NOT_RUN_DATA_GAP; scores UNKNOWN, not zero",
        "unlabelled_as_negative": "FORBIDDEN", "new_graph_or_labels": "FORBIDDEN",
        "E18_modifications": "FORBIDDEN", "doppler": "NOT_USED",
        "weight_or_threshold_selection": "FORBIDDEN",
        "archived_bag_probe_max_uncompressed_bytes": 1048576,
        "archive_payload_scope": "bounded prefix metadata; no full stream or tail decode",
        "GNSS_scope": "all workbook cells/metadata; existing GPS and WGS84 conventions only",
        "GNSS_N_E_V_Q": "OPAQUE_UNKNOWN", "new_E18_solver_runs": 0,
    })
    write("protected_input_identity.json", dict(sorted(protected.items())))
    write("freeze_identity.json", {
        "utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "parent_commit": head, "frozen_E18_commit": FROZEN_E18,
        "frozen_source_checks": frozen_source_checks, "protected_file_count": len(protected),
        "protected_tree_paths": tree_paths(), "source_sha256": sha(Path(__file__).resolve()),
        "protocol_sha256": sha(OUT / "protocol.json"),
        "environment": {"python": sys.version, "platform": platform.platform()},
        "scope_note": "evidence qualification rules fixed before synthesis; prior initial inventory already inspected",
    })
    print(json.dumps({"frozen_protected_files": len(protected), "solver_runs": 0}))


def check() -> None:
    expected = json.loads((OUT / "protected_input_identity.json").read_text())
    changed = [rel for rel, value in expected.items() if not (ROOT / rel).is_file()
               or sha(ROOT / rel) != value]
    identity = json.loads((OUT / "freeze_identity.json").read_text())
    if tree_paths() != identity["protected_tree_paths"]:
        changed.append("PROTECTED_TREE_FILE_SET")
    if sha(OUT / "protocol.json") != identity["protocol_sha256"]:
        changed.append("E20_PROTOCOL")
    if sha(Path(__file__).resolve()) != identity["source_sha256"]:
        changed.append("E20_FREEZE_CHECK_SOURCE")
    result = {"utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
              "protected_files_checked": len(expected), "changed": changed,
              "protected_trees_unchanged": not changed, "new_E18_solver_runs": 0,
              "status": "PASS" if not changed else "FAIL"}
    if changed:
        print(json.dumps(result))
        raise RuntimeError("Protected research record changed")
    write("protected_input_validation.json", result)
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("freeze", "check"))
    args = parser.parse_args()
    freeze() if args.phase == "freeze" else check()
