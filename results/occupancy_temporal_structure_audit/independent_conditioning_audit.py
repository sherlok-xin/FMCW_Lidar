#!/usr/bin/env python3
"""Rebuild post-freeze exact-H contrasts and the kill decision independently.

Reads fixed IDs and the independently validated feature table.  It neither
imports interpretation code nor uses its pair/summary outputs as inputs to
the reconstruction.  Auxiliary p-bin mixtures are explicitly diagnosed.
"""
from pathlib import Path
from datetime import datetime, timezone
from collections import Counter, defaultdict
import csv
import hashlib
import json
import time

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SCENE = ROOT / "results/scene_spatiotemporal_background_audit"
CIDS = ("100m_cross_5", "100m_cross_10", "200m_cross_5", "200m_cross_10", "300m_cross_10")
FEATURES = ("age_s", "longest_previous_run", "previous_episodes", "recent_5_prior_frame_fraction")
CONTROLS = ("fixed_silver_feasibility_witness", "original_unconditional_winner", "background_only_winner", "joint_E18_winner", "historical_persistent_clutter_reference")
KILL_CONTROLS = {"original_unconditional_winner", "background_only_winner", "historical_persistent_clutter_reference"}


def rows(path):
    with open(path, newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def number(row, field):
    return float(row[field])


def cell(row):
    return tuple(int(row[f"cell_{axis}"]) for axis in "xyz")


def equal_record(actual, expected, identity):
    for field, value in expected.items():
        try:
            av, ev = float(actual[field]), float(value)
        except (TypeError, ValueError):
            assert str(actual[field]) == str(value), (identity, field, actual[field], value)
        else:
            assert av == ev or (np.isnan(av) and np.isnan(ev)), (identity, field, av, ev)


def keyed(path, fields):
    result = {}
    for row in rows(path):
        key = tuple(row[field] for field in fields)
        assert key not in result, (path, key)
        result[key] = row
    return result


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"finite_n": 0, **{key: np.nan for key in ("q25", "median", "q75", "iqr", "min", "max")}}
    q25, median, q75 = np.quantile(values, [.25, .5, .75])
    return {"finite_n": len(values), "q25": q25, "median": median, "q75": q75, "iqr": q75 - q25, "min": min(values), "max": max(values)}


def clean(obj):
    if isinstance(obj, dict):
        return {key: clean(value) for key, value in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(value) for value in obj]
    if isinstance(obj, (float, np.floating)):
        return float(obj) if np.isfinite(obj) else None
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj


def main():
    started = time.perf_counter()
    manifest = json.loads((OUT / "interpretation_manifest.json").read_text())
    receipt = json.loads((OUT / "TEMPORAL_FEATURES_COMPLETE.json").read_text())
    protocol = json.loads((OUT / "protocol.json").read_text())
    assert manifest["utc_started"] > receipt["utc"]
    assert manifest["features_sha256"] == sha(OUT / "all_node_temporal_features.csv")
    for relpath, digest in manifest["join_inputs"].items():
        assert sha(ROOT / relpath) == digest
    feature_rows = rows(OUT / "all_node_temporal_features.csv")
    feature = {(r["condition_id"], r["family"], r["node_id"]): r for r in feature_rows}
    assert len(feature) == len(feature_rows) == 37117
    anchors = rows(SCENE / "anchor_node_join.csv")
    assert len(anchors) == 80
    anchor_by_frame = {(r["condition_id"], int(r["frame_index"])): r for r in anchors}
    assert len(anchor_by_frame) == 80

    paths = {}
    arm_group = {"original": "original_unconditional_winner", "background_only": "background_only_winner", "background_motion_E18": "joint_E18_winner"}
    for r in rows(ROOT / "results/e18_mechanism_generalization_audit/selected_paths.csv"):
        if r["family"] == "fine_4cm_pregeometry" and r["arm"] in arm_group:
            key = (r["condition_id"], int(r["frame_index"]), arm_group[r["arm"]])
            assert key not in paths
            paths[key] = feature[(r["condition_id"], r["family"], r["node_id"])]
    for r in rows(ROOT / "results/background_motion_dp_mvp/fixed_reference_paths.csv"):
        if r["family"] == "fine_4cm_pregeometry" and r["path_type"] == "silver_feasible":
            group = "fixed_silver_feasibility_witness"
        elif r["family"] == "historical_1m_graph" and r["path_type"] == "top_clutter_full":
            group = "historical_persistent_clutter_reference"
        else:
            continue
        key = (r["condition_id"], int(r["frame_index"]), group)
        assert key not in paths
        paths[key] = feature[(r["condition_id"], r["family"], r["node_id"])]
    for (cid, frame, _), pathnode in paths.items():
        assert int(pathnode["frame_index"]) == frame and pathnode["condition_id"] == cid

    actual_pairs = keyed(OUT / "same_frame_exact_H_pairs.csv", ("condition_id", "frame_index", "comparison_group", "feature"))
    actual_frames = keyed(OUT / "kill_frame_contrasts.csv", ("condition_id", "frame_index", "feature"))
    expected_pairs = []
    expected_frames = []
    duplicate_control_cell_occurrences = 0
    for (cid, frame), anchor_join in sorted(anchor_by_frame.items()):
        an = feature[(cid, "fine_4cm_pregeometry", anchor_join["nearest_node_id"])]
        assert int(an["frame_index"]) == frame
        h = int(an["prior_hits"])
        for group in CONTROLS:
            cn = paths.get((cid, frame, group))
            same_h = cn is not None and int(cn["prior_hits"]) == h
            same_cell = cn is not None and cell(cn) == cell(an)
            if same_h:
                assert number(cn, "p_beta11") == number(an, "p_beta11")
                assert number(cn, "prior_frames") == number(an, "prior_frames") == frame
            for quantity in FEATURES:
                av = number(an, quantity)
                cv = number(cn, quantity) if cn is not None else np.nan
                supported = same_h and np.isfinite(av) and np.isfinite(cv)
                if supported and same_cell:
                    assert av == cv  # A common scene cell is deterministic, even for different node IDs.
                status = ("PATH_FRAME_MISSING" if cn is None else "NONMATCHING_H" if not same_h else "FEATURE_UNKNOWN" if not supported else "IDENTICAL_CELL_DETERMINISTIC_TIE" if same_cell else "DISTINCT_CELL_EXACT_FREQUENCY")
                expected = {"condition_id": cid, "frame_index": frame, "comparison_group": group, "feature": quantity, "anchor_node_id": an["node_id"], "comparison_family": cn["family"] if cn is not None else "MISSING", "comparison_node_id": cn["node_id"] if cn is not None else "MISSING", "anchor_H": h, "comparison_H": int(cn["prior_hits"]) if cn is not None else "UNKNOWN", "F": frame, "p_anchor": number(an, "p_beta11"), "p_comparison": number(cn, "p_beta11") if cn is not None else np.nan, "same_frame_exact_H": int(same_h), "same_centroid_cell": int(same_cell), "anchor_value": av, "comparison_value": cv, "occupancy_conditioned_difference": av - cv if supported else np.nan, "status": status}
                key = (cid, str(frame), group, quantity)
                equal_record(actual_pairs[key], expected, key)
                expected_pairs.append(expected)

        for quantity in FEATURES:
            min_h = 2 if quantity in ("longest_previous_run", "previous_episodes") else 1
            av = number(an, quantity)
            eligible = frame >= 5 and h >= min_h and np.isfinite(av)
            by_cell = defaultdict(list)
            if eligible:
                for group in KILL_CONTROLS:
                    cn = paths.get((cid, frame, group))
                    if cn is not None and int(cn["prior_hits"]) == h and cell(cn) != cell(an) and np.isfinite(number(cn, quantity)):
                        assert number(cn, "prior_frames") == frame and number(cn, "p_beta11") == number(an, "p_beta11")
                        by_cell[cell(cn)].append(number(cn, quantity))
            for values in by_cell.values():
                assert len(set(values)) == 1  # Same cell features are equal across graph families/path memberships.
                duplicate_control_cell_occurrences += len(values) - 1
            comparator = np.median([values[0] for values in by_cell.values()]) if by_cell else np.nan
            expected = {"condition_id": cid, "frame_index": frame, "feature": quantity, "anchor_node_id": an["node_id"], "H": h, "F": frame, "p_beta11": number(an, "p_beta11"), "eligible_non_degenerate_warm_anchor": int(eligible), "distinct_exact_H_control_cells": len(by_cell), "anchor_value": av, "control_median_value": comparator, "frame_median_difference": av - comparator if by_cell else np.nan, "support": "AVAILABLE" if by_cell else "UNKNOWN_NO_ELIGIBLE_DISTINCT_CELL_COMMON_SUPPORT"}
            key = (cid, str(frame), quantity)
            equal_record(actual_frames[key], expected, key)
            expected_frames.append(expected)
    assert len(actual_pairs) == len(expected_pairs) == 1600
    assert len(actual_frames) == len(expected_frames) == 320

    actual_sequences = keyed(OUT / "kill_sequence_summary.csv", ("condition_id", "feature"))
    independently_derived = []
    independent_kill_features = []
    for quantity in FEATURES:
        signs = []
        valid_all = True
        for cid in CIDS:
            frame_rows = [r for r in expected_frames if r["condition_id"] == cid and r["feature"] == quantity]
            finite = np.array([r["frame_median_difference"] for r in frame_rows if np.isfinite(r["frame_median_difference"])])
            summary = stats(finite)
            tol = protocol["numerical_zero_tolerance"][quantity]
            sign = int(np.sign(summary["median"])) if len(finite) and abs(summary["median"]) > tol else 0
            agreement = float(np.count_nonzero(finite * sign > tol) / len(finite)) if len(finite) and sign else 0. if len(finite) else np.nan
            valid = len(finite) >= 3 and sign != 0 and agreement >= 2 / 3
            reason = "INSUFFICIENT_COMMON_SUPPORT" if len(finite) < 3 else "ZERO_MEDIAN" if not sign else "DIRECTION_UNSTABLE" if agreement < 2 / 3 else "PASS"
            expected = {"condition_id": cid, "feature": quantity, "finite_distinct_paired_frames": len(finite), "eligible_anchor_frames": sum(r["eligible_non_degenerate_warm_anchor"] for r in frame_rows), "median_direction": sign, "same_direction_fraction": agreement, "sequence_pass": int(valid), "failure_reason": reason, **summary}
            equal_record(actual_sequences[(cid, quantity)], expected, (cid, quantity))
            independently_derived.append(expected)
            signs.append(sign)
            valid_all &= valid
        shared_sign = len(set(signs)) == 1 and signs[0] != 0
        independent_kill_features.append({"feature": quantity, "passes_all_five": bool(valid_all and shared_sign), "same_nonzero_direction_all_five": bool(shared_sign), "sequence_directions": signs})
    decision = json.loads((OUT / "kill_decision.json").read_text())
    assert decision["features"] == independent_kill_features
    status = "STABLE_CONDITIONAL_PHENOMENON_ONLY_NOT_SCORE_AUTHORIZATION" if any(r["passes_all_five"] for r in independent_kill_features) else "E18_OCCUPANCY_SUFFICIENT_AT_CURRENT_EVIDENCE_LEVEL_NO_E21_BACKGROUND_MODEL"
    assert decision["status"] == status

    # Auxiliary narrow bins do mix F and H.  They are not inputs to the above
    # reconstruction, which controls exact F and H before comparing values.
    grouped_bins = defaultdict(list)
    for r in rows(OUT / "population_temporal_features.csv"):
        if int(float(r["prior_frames"])) >= 5:
            grouped_bins[(r["condition_id"], r["group"], r["p_bin"])].append(r)
    bin_mixture = []
    for (cid, group, pbin), rs in sorted(grouped_bins.items()):
        fs = {int(float(r["prior_frames"])) for r in rs}
        hs = {int(float(r["prior_hits"])) for r in rs}
        if len(fs) > 1 or len(hs) > 1:
            bin_mixture.append({"condition_id": cid, "group": group, "p_bin": pbin, "nodes": len(rs), "distinct_F": len(fs), "distinct_H": len(hs), "min_F": min(fs), "max_F": max(fs)})
    cases = []
    for cid in ("100m_cross_10", "300m_cross_10"):
        for quantity in FEATURES:
            min_h = 2 if quantity in ("longest_previous_run", "previous_episodes") else 1
            selected = [r for r in expected_pairs if r["condition_id"] == cid and r["comparison_group"] == "background_only_winner" and r["F"] >= 5 and r["anchor_H"] >= min_h and r["same_frame_exact_H"] and not r["same_centroid_cell"] and r["feature"] == quantity and np.isfinite(r["occupancy_conditioned_difference"])]
            values = [r["occupancy_conditioned_difference"] for r in selected]
            cases.append({"condition_id": cid, "feature": quantity, "nondegenerate_distinct_cell_same_frame_exact_H_frames": len(selected), "frame_indices": [r["frame_index"] for r in selected], "anchor_minus_background": stats(values), "interpretation": "local descriptive distinction; no path rescore or full-path-ranking explanation"})
    output = {"status": "TESTED_INDEPENDENT_CONDITIONING_AUDIT_PASS", "utc": datetime.now(timezone.utc).isoformat(), "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/independent_conditioning_audit.py", "validator_sha256": sha(__file__), "same_frame_exact_H_pair_rows_checked": len(expected_pairs), "kill_frame_rows_checked": len(expected_frames), "kill_sequence_rows_checked": len(actual_sequences), "duplicate_control_cell_occurrences_removed": duplicate_control_cell_occurrences, "pair_status_counts": dict(Counter(r["status"] for r in expected_pairs)), "same_cell_not_kill_evidence": True, "nonmatching_H_not_conditioned_evidence": True, "all_unknown_common_support_remains_NaN": True, "cold_start_and_H0_H1_degeneracy_excluded_from_kill": True, "auxiliary_warm_bins_mixing_F_or_H": len(bin_mixture), "auxiliary_bin_mixtures_examples": bin_mixture[:12], "primary_decision_reconstructed_without_bin_files": True, "independently_reconstructed_kill_status": status, "independently_reconstructed_kill_features": independent_kill_features, "independently_reconstructed_sequence_results": independently_derived, "background_only_failure_case_support": cases, "checked_output_sha256": {name: sha(OUT / name) for name in ("same_frame_exact_H_pairs.csv", "kill_frame_contrasts.csv", "kill_sequence_summary.csv", "kill_decision.json", "frequency_bin_contrasts.csv")}, "elapsed_s": time.perf_counter() - started}
    (OUT / "independent_conditioning_audit.json").write_text(json.dumps(clean(output), indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(f"PASS: {len(expected_pairs)} pairs, {len(expected_frames)} frame summaries, {len(actual_sequences)} sequence summaries; {status}")


if __name__ == "__main__":
    main()
