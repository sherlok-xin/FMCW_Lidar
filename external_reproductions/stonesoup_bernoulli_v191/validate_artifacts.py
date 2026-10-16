"""Validate local evidence without rerunning filters or reading FMCW data."""
import ast
import csv
from datetime import datetime, timezone
from decimal import Decimal, localcontext
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / "results/external_reproductions/stonesoup_bernoulli_v191"


def load(name):
    return json.loads((OUT / name).read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    config, scalar, tests = load("protocol.json"), load("closed_form_check.json"), load("official_test_summary.json")
    checks = {}
    checks["protocol_frozen_hash_unchanged"] = sha(OUT / "protocol.json") == load("protocol_receipt.json")["protocol_sha256"] == scalar["protocol_sha256"]
    checks["freeze_precedes_scalar_execution"] = config["frozen_at_utc"] < scalar["calculated_at_utc"] < scalar["executed_at_utc"]
    calls = [r for r in tests["reports"] if r["when"] == "call"]
    checks["seven_relevant_official_tests_passed"] = len(calls) == 7 and tests["exit_code"] == 0 and all(r["outcome"] == "passed" for r in tests["reports"])
    checks["official_tests_preceded_gate_freeze"] = tests["utc"] < config["frozen_at_utc"]
    with localcontext() as context:
        context.prec = 60
        r, b, s, pd = [Decimal(str(config[k])) for k in ("r_prev", "p_birth", "p_survival", "P_D")]
        predicted = b * (1 - r) + s * r
        posterior = predicted * (1 - pd) / (1 - predicted * pd)
    checks["independent_decimal_reference"] = abs(float(posterior) - scalar["r_empty_binary64"]) < 1e-15 and float(predicted) == scalar["official_predictor_r_pred"]
    checks["legal_missed_container_trace"] = scalar["legal_hypothesis_container"] == "MultipleHypothesis" and scalar["legal_hypothesis_count"] == 1 and scalar["measurement_type"] == "MissedDetection" and not scalar["any_hypotheses"]
    checks["mismatch_preserved_not_overwritten"] = not scalar["updater_matches_empty_scan_closed_form"] and scalar["official_updater_r_post"] == scalar["official_predictor_r_pred"] and scalar["absolute_empty_scan_discrepancy"] > config["tolerance_absolute"]
    checks["single_step_probabilities_finite_bounded"] = all(math.isfinite(scalar[k]) and 0 <= scalar[k] <= 1 for k in ("official_predictor_r_pred", "official_updater_r_post"))
    for stage in ("prediction", "update"):
        p = scalar[stage + "_particles"]
        weights = p["weights"]
        checks[stage + "_weights_valid"] = len(weights) == p["count"] and all(math.isfinite(w) and w >= 0 for w in weights) and math.isclose(sum(weights), 1, abs_tol=config["tolerance_absolute"], rel_tol=0)
        checks[stage + "_ESS_recomputed"] = math.isclose(1 / sum(w * w for w in weights), p["effective_sample_size"], rel_tol=1e-12)
    identity = load("source_identity.json")
    upstream = HERE / "upstream"
    git = lambda *args: subprocess.check_output(["git", "-C", str(upstream), *args], text=True).strip()
    checks["tag_and_HEAD_still_expected"] = git("rev-parse", "HEAD") == git("rev-parse", "refs/tags/v1.9.1^{commit}") == identity["expected_commit"]
    checks["locked_source_hashes_unchanged"] = all(sha(upstream / path) == expected for path, expected in identity["source_files_sha256"].items())
    checks["no_upstream_tracked_edits"] = not git("diff", "HEAD", "--name-only")
    checks["installed_core_identity_matches"] = identity["installed_core_matches_locked_source"]
    checks["no_project_commit_created"] = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip() == identity["root_project_head_before_report_changes"]
    status = load("execution_status.json")
    for name, note in status["files"].items():
        with (OUT / name).open() as file:
            rows = list(csv.reader(file))
        checks[name + "_explicitly_NOT_RUN"] = note["status"] == "NOT_RUN" and note["data_rows"] == 0 and len(rows) == 1
    helpers = list(HERE.glob("*.py"))
    for helper in helpers:
        ast.parse(helper.read_text(), filename=str(helper))
    checks["helpers_parse"] = True
    missing = []
    docs = [ROOT / "FMCW_STONESOUP_BERNOULLI_REPRODUCTION.md", HERE / "README.md", OUT / "README.md", OUT / "source_semantics.md", OUT / "independent_review.md"]
    for document in docs:
        for target in re.findall(r"\]\(([^)]+)\)", document.read_text()):
            if "://" in target:
                continue
            path = target.split("#")[0]
            if path and not (document.parent / path).exists():
                if path != "results/external_reproductions/stonesoup_bernoulli_v191/validation.json":
                    missing.append([str(document.relative_to(ROOT)), path])
    # validation.json and artifact_manifest.json are materialized by this task below.
    missing = [m for m in missing if not m[1].endswith("artifact_manifest.json")]
    checks["local_markdown_links_valid"] = not missing
    assert all(checks.values()), {k: v for k, v in checks.items() if not v}
    validation = {
        "validated_at_utc": datetime.now(timezone.utc).isoformat(),
        "evidence_integrity_passed": True,
        "full_reproduction_success": False,
        "status": "STOPPED_AT_EMPTY_SCAN_SEMANTIC_GATE",
        "checks": checks,
        "scalar_reference_independently_recalculated": str(posterior),
        "gate_formula_agreement": False,
        "success_criteria": {
            "1_official_tests": "PASS: 7 relevant standard tests; visibility extension/non-Bernoulli deselection justified",
            "2_version_identity": "PASS",
            "3_existence_finite_bounded": "PASS_SINGLE_SCALAR_ONLY; NOT_TESTED_MULTIFRAME",
            "4_particle_weights": "PASS_SINGLE_SCALAR_ONLY; NOT_TESTED_MULTIFRAME",
            "5_legal_missed_handling": "Representation resolved and executed; Bayesian existence decay NOT_REPRODUCED",
            "6_closed_form_model_equivalence": "FAIL: same model assumptions, source guard skips correction; not a model-dependent difference",
            "7_non_oracle_generation": "PASS_SCALAR_INPUT; A/B scenario generation NOT_RUN",
            "8_preregistration": "PASS_GATE_ONLY; full scenario NOT_FROZEN due STOP",
            "9_no_FMCW_evidence": "Filter/helpers use only this protocol and official source; no DEV evidence for parameters or success",
        },
        "scope_limits": ["No additional filter run in this validator", "No A/B/C or multi-frame decay result", "No full-machine I/O forensic claim; evidence scope is helper/source/command review", "PNG visually inspected; plots show one executed scalar case only"],
        "upstream_algorithm_modified": False,
        "FMCW_data_or_performance_used_for_parameters_or_judgment": False,
        "commit_push_performed": False,
        "missing_local_links": missing,
    }
    (OUT / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"evidence_checks_passed": len(checks), "full_reproduction_success": False,
                      "reason": "legal empty-scan posterior mismatch"}))
