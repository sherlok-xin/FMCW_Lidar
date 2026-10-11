#!/usr/bin/env python3
"""Synthesize E20 candidate evidence; no thresholds, graphs or solver execution."""
from __future__ import annotations

import csv
import hashlib
import json
import platform
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/negative_hover_evidence_audit"
FIVE = {"100m_cross_5", "100m_cross_10", "200m_cross_5", "200m_cross_10", "300m_cross_10"}


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def iso(value):
    return datetime.fromtimestamp(float(value), timezone.utc).isoformat()


def main():
    names = ["candidate_windows.csv", "stress_execution_decision.json", "negative_stress_winners.csv",
             "qualified_hover_score_trace.csv", "window_inventory_execution.json"]
    for name in names:
        if (OUT / name).exists():
            raise FileExistsError(f"Refusing to overwrite {OUT / name}")
    started = datetime.now(timezone.utc).isoformat()
    inputs = [OUT / "raw_bag_inventory.csv", OUT / "raw_bag_inventory.json",
              OUT / "gnss_condition_summary.csv", OUT / "gnss_adjacent_segments.csv",
              OUT / "protocol.json", ROOT / "gt_manifest.csv",
              ROOT / "results/research_dev_v0/silver_labels.csv",
              ROOT / "results/data_archaeology/cross/uav_pseudo_labels.csv",
              ROOT / "results/e18_mechanism_generalization_audit/short_slow_windows.csv",
              ROOT / "results/calibration_closure/candidate_summary.csv"]
    before = {str(p.relative_to(ROOT)): digest(p) for p in inputs}
    manifest = rows(ROOT / "gt_manifest.csv")
    by_id = {r["condition_id"]: r for r in manifest}
    silver = rows(ROOT / "results/research_dev_v0/silver_labels.csv")
    observed = {(r["condition_id"], int(r["frame_index"])): r for r in silver}
    result = []

    def add(uid, kind, source, locator, level, start="UNKNOWN", end="UNKNOWN",
            clock="UNKNOWN", cid="UNKNOWN", lidar="UNKNOWN", summary="", failure=""):
        result.append({"window_id": uid, "candidate_kind": kind, "source_path": source,
            "source_locator": locator, "condition_id": cid, "evidence_level": level,
            "time_start": start, "time_end": end, "time_reference": clock,
            "lidar_availability": lidar, "independent_evidence": summary,
            "independently_certified_target_presence": "UNKNOWN",
            "independently_certified_target_absence": "UNKNOWN",
            "independently_certified_hover_or_slow": "UNKNOWN",
            "clock_link_to_lidar": "VERIFIED_BAG_RECORD_TIME" if clock == "BAG_RECORD_UTC" else "UNKNOWN",
            "frozen_E18_bag_graph_available": cid in FIVE,
            "stress_qualified": False, "formal_evaluation_qualified": False,
            "qualification_failure": failure, "stress_execution": "NOT_RUN_DATA_GAP"})

    for index, r in enumerate(rows(OUT / "raw_bag_inventory.csv"), 1):
        cid = next((m["condition_id"] for m in manifest
                    if r["source_path"] == m["archive_path"] or
                    (r["source_kind"] == "STANDALONE_BAG" and
                     r["source_path"].endswith(m["bag_member"]))), "UNKNOWN")
        end = r["record_end_utc"]
        if cid in by_id:
            end = by_id[cid]["lidar_bag_end_utc"]
        source = r["source_path"] + ("::" + r["archive_member"] if r["archive_member"] else "")
        add(f"bag_metadata_{index:02d}", "POTENTIAL_ABSENCE_UNCERTIFIED", source,
            "raw_bag_inventory.csv row " + str(index + 1), "L0", r["record_start_utc"], end,
            "BAG_RECORD_UTC", cid, "RAW_BAG_PRESENT_METADATA_ONLY",
            "Only LiDAR PointCloud2 topic metadata; filenames do not certify flight state",
            "No independent absence event/annotation; unlabelled and no-return are not negatives")
    for r in rows(OUT / "gnss_condition_summary.csv"):
        add("gnss_" + r["condition_id"], "GNSS_FLIGHT_RECORD_UNCERTIFIED_STATE",
            "bag/20260205/飞行轨迹坐标.xlsx",
            r["source_sheet"] + "!" + r["first_cell"] + ":" + r["last_cell"], "L2",
            r["utc_start"], r["utc_end"], "GPS_WEEK_TOW_UTC_MINUS18S", r["condition_id"],
            "MATCHED_BAG_PRESENT_TIME_LINK_PROVISIONAL",
            f"Independent position record, {r['sample_count']} samples; no armed/landed/hover events",
            "Sparse endpoint chords; unresolved absolute clock/pose/FOV; no certified state interval")
    slowest = min(rows(OUT / "gnss_adjacent_segments.csv"), key=lambda r: float(r["chord_speed_mps"]))
    add("gnss_minimum_chord", "NEAR_ORIGIN_SLOW_HINT_NOT_HOVER",
        slowest["source_path"], slowest["source_sheet"] + "!" + slowest["from_cell"] + ":" + slowest["to_cell"],
        "L2", slowest["utc_start"], slowest["utc_end"], "GPS_WEEK_TOW_UTC_MINUS18S",
        slowest["condition_id"], "MATCHED_BAG_PRESENT_TIME_LINK_PROVISIONAL",
        f"Endpoint chord speed {float(slowest['chord_speed_mps']):.9f} m/s; A2 equals device A85",
        "No instantaneous motion/flight-state uncertainty or independent bag synchronization; origin is not absence")
    photo = json.loads((OUT / "raw_bag_inventory.json").read_text())["scene_photo"]
    add("scene_photo", "INDEPENDENT_STILL_SETUP_NOT_WINDOW", photo["path"],
        "EXIF DateTimeOriginal/OffsetTimeOriginal/Subsec + visual inspection", "L2",
        "2026-02-05T14:16:50.957+08:00", "2026-02-05T14:16:50.957+08:00", "PHOTO_EXIF_NOT_SYNCHRONIZED",
        lidar="NO_CONTEMPORANEOUS_FEBRUARY_BAG", summary="Quadrotor and LiDAR setup visible in a still photo; no airborne interval established",
        failure="Photo precedes earliest February bag; separate EXIF GPS clock differs; no interval/visibility/flight event")
    seen = set()
    for r in rows(ROOT / "results/e18_mechanism_generalization_audit/short_slow_windows.csv"):
        if r["window_id"] in seen:
            continue
        seen.add(r["window_id"])
        cid = r["condition_id"]
        a, b = int(r["first_frame"]), int(r["last_frame"])
        add("prior_silver_" + r["window_id"], "PRIOR_E19_SLOW_SILVER_WINDOW",
            "results/research_dev_v0/silver_labels.csv", f"{cid} frames {a}..{b}; original E19 selector", "L1",
            iso(observed[cid, a]["bag_time_unix"]), iso(observed[cid, b]["bag_time_unix"]),
            "BAG_RECORD_UTC", cid, "FROZEN_GRAPH_AND_RAW_BAG_AVAILABLE",
            "NO independent evidence: LiDAR-derived observed center changes",
            "Retrospective DEV_SILVER, not independently authenticated physical hover/slow state")
    # Descriptive source locators only, not new annotations or a detector selector.
    for cid, a, b in (("200m_cross_5", 0, 21), ("200m_cross_10", 30, 47), ("100m_cross_10", 18, 26)):
        add(f"transition_shape_{cid}_{a}_{b}", "MOVING_SLOWING_NEAR_STATIC_SILVER_SHAPE",
            "results/research_dev_v0/silver_labels.csv", f"{cid} contiguous observed frames {a}..{b}", "L1",
            iso(observed[cid, a]["bag_time_unix"]), iso(observed[cid, b]["bag_time_unix"]),
            "BAG_RECORD_UTC", cid, "FROZEN_GRAPH_AND_RAW_BAG_AVAILABLE",
            "NO independent evidence; all adjacent real-dt speeds retained in silver_motion_segments.csv",
            "Return-center trajectory shape is not UAV hover identity; no new score execution")
    legacy = rows(ROOT / "results/data_archaeology/cross/uav_pseudo_labels.csv")[:4]
    add("legacy_cross_frames0_3", "LEGACY_NEAR_STATIC_PSEUDO_CENTER",
        "results/data_archaeology/cross/uav_pseudo_labels.csv", "frames 0..3", "L1",
        iso(legacy[0]["bag_time"]), iso(legacy[-1]["bag_time"]), "BAG_RECORD_UTC",
        lidar="NOVEMBER_RAW_BAG_PRESENT_NO_FROZEN_E18_GRAPH",
        summary="NO independent evidence; manually initialized/ROI-derived pseudo centers",
        failure="Uncertified identity and hover; no existing frozen E18 graph")
    examples = {"100m_cross_10_e0001", "200m_cross_5_e0001", "300m_roundtrip_10_e0001"}
    for r in rows(ROOT / "results/calibration_closure/candidate_summary.csv"):
        if r["candidate_id"] in examples:
            add("unselected_" + r["candidate_id"], "UNSELECTED_SLOW_LIDAR_CANDIDATE",
                "results/calibration_closure/candidate_summary.csv", r["candidate_id"], "L1",
                cid=r["condition_id"], lidar="MATCHED_RAW_BAG_PRESENT",
                summary=f"NO independent identity; LiDAR candidate median speed {r['median_speed_mps']} m/s",
                failure="Not a closure-selected silver target; do not relabel; interval certification UNKNOWN")
    assert len(result) == 80, len(result)
    assert all(not r["stress_qualified"] and not r["formal_evaluation_qualified"] for r in result)
    with (OUT / "candidate_windows.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(result[0]))
        w.writeheader()
        w.writerows(result)
    # Header-only schemas are deliberately not zero measurements.
    schemas = {"negative_stress_winners.csv": ["window_id", "score", "duration_s", "node_score",
        "background_term", "motion_penalty", "gap_penalty", "path_properties", "status"],
        "qualified_hover_score_trace.csv": ["window_id", "bag_record_time_utc", "frame_index",
        "target_neighborhood_score", "background_term", "motion_penalty", "status"]}
    for name, columns in schemas.items():
        with (OUT / name).open("w", encoding="utf-8-sig", newline="") as f:
            csv.writer(f).writerow(columns)
    decision = {"status": "DATA_GAP", "candidate_inventory_rows": len(result),
        "rows_are_not_independent_sequences": True,
        "evidence_level_counts": dict(Counter(r["evidence_level"] for r in result)),
        "qualified_negative_windows": [], "qualified_slow_hover_windows": [],
        "E18_stress_execution": "NOT_RUN_DATA_GAP", "new_solver_runs": 0,
        "negative_winner_score_duration_components": "UNKNOWN",
        "independent_moving_slow_hover_score_degradation": "UNKNOWN",
        "operational_false_alarms_precision": "UNKNOWN", "threshold_selected": "NONE",
        "prior_E19_traces": "OLD_STORED_DIAGNOSTIC_NOT_NEW_STRESS",
        "reason": "No independent target-state window tied to LiDAR time/FOV; silver does not qualify"}
    (OUT / names[1]).write_text(json.dumps(decision, ensure_ascii=False, indent=2) + "\n")
    after = {str(p.relative_to(ROOT)): digest(p) for p in inputs}
    assert before == after
    execution = {"command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python " + str(Path(__file__).relative_to(ROOT)),
        "started_utc": started, "finished_utc": datetime.now(timezone.utc).isoformat(),
        "script_sha256": digest(Path(__file__)), "environment": {"python": sys.version, "platform": platform.platform()},
        "input_sha256": before, "inputs_unchanged": True, "new_E18_solver_runs": 0,
        "output_hashes": {name: digest(OUT / name) for name in names[:-1]}}
    (OUT / names[-1]).write_text(json.dumps(execution, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(decision))


if __name__ == "__main__":
    main()
