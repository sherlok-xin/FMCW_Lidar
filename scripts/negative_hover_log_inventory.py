#!/usr/bin/env python3
"""Read-only source/history inventory; no detector, graph solver, or new stress run.

Exports every observed silver step and every frame-contiguous fragment without a
new slow threshold.  The score trace is copied from stored E19 witness CSVs, not
recomputed with E18.  Nothing here certifies target absence or physical hover.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SILVER = ROOT / "results/research_dev_v0/silver_labels.csv"
E19 = ROOT / "results/e18_mechanism_generalization_audit"
DEFAULT_OUT = ROOT / "results/negative_hover_evidence_audit"
OUTPUT_NAMES = (
    "log_source_inventory.csv", "silver_motion_segments.csv",
    "silver_continuous_fragments.csv", "prior_e19_slow_score_trace.csv",
    "log_evidence_execution.json", "log_history_coverage.json",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def display_path(path):
    path = Path(path)
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def csv_rows(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows, fields):
    with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def utc(timestamp):
    return dt.datetime.fromtimestamp(float(timestamp), dt.timezone.utc).isoformat()


def position(row):
    return [float(row[key]) for key in ("center_x_m", "center_y_m", "center_z_m")]


def run_git(args):
    command = ["git", *args]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=True)
    return {"command": shlex.join(command), "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(ROOT)
    for name in OUTPUT_NAMES:
        if (output / name).exists():
            raise FileExistsError(f"Refusing to overwrite existing artifact: {output / name}")
    started = time.monotonic()
    inputs = {}
    source_inventory = []

    def note(item_id, path, first, last, source_type, interpretation,
             presence="UNKNOWN", absence="UNKNOWN", slow="UNKNOWN", hover="UNKNOWN"):
        path = Path(path)
        if not path.is_absolute():
            path = ROOT / path
        lines = path.read_text(encoding="utf-8-sig").splitlines()
        if not 1 <= first <= last <= len(lines):
            raise ValueError(f"Invalid source locator: {path}:{first}-{last}")
        identity = sha256(path)
        inputs[display_path(path)] = identity
        source_inventory.append({
            "item_id": item_id, "source_path": display_path(path),
            "source_locator": f"{display_path(path)}:{first}-{last}",
            "source_sha256": identity, "source_type": source_type,
            "interpretation": interpretation, "target_presence_evidence": presence,
            "target_absence_certification": absence, "physical_slow_certification": slow,
            "physical_hover_certification": hover,
            "independent_timestamp_linked_flight_state": "NOT_ESTABLISHED_BY_THIS_SOURCE",
            "formal_evaluation_eligible": "NO_FROM_THIS_SOURCE_ALONE",
        })

    note("legacy_readme", "ReadMe.md", 24, 31, "ALGORITHM_DESCRIPTION",
         "Historical occupancy/neighborhood and tracking logic, not acquisition events.")
    note("github_readme", "README.md", 1, 1, "PROJECT_TITLE_ONLY",
         "No flight-state, presence, absence, hover, or synchronization annotation.")
    note("recovered_user_cross", "docs/RECOVERED_CODEX_SESSION_2026-09-18.md", 92, 103,
         "RECOVERED_ORIGINAL_USER_VISUAL_OBSERVATION",
         "User reports target-like sparse returns seen in RViz; no timed hover/absence state.",
         presence="USER_REPORTED_TARGET_LIKE_CROSS_RETURNS_NOT_INDEPENDENT_FRAME_GT")
    note("recovered_user_approach", "docs/RECOVERED_CODEX_SESSION_2026-09-18.md", 113, 123,
         "RECOVERED_ORIGINAL_USER_REQUEST",
         "Longitudinal dataset identified; no timed hover/absence or takeoff/landing record.",
         presence="USER_REPORTED_LONGITUDINAL_TARGET_DATA_NOT_FRAME_GT")
    note("prompt_presence_examples", "docs/RECOVERED_PROJECT_ARCHAEOLOGY_PROMPT.md", 164, 181,
         "REQUEST_TEMPLATE_NOT_ACQUISITION_RECORD",
         "Requested presence/absence inventory, not evidence that such labels exist.")
    note("prompt_hover_example", "docs/RECOVERED_PROJECT_ARCHAEOLOGY_PROMPT.md", 424, 449,
         "REQUEST_TEMPLATE_NOT_ACQUISITION_RECORD",
         "Hovering is an example failure category; text explicitly says derive from data.")
    note("v0_no_external_event", "FMCW_GT_Alignment_and_Benchmark_v0.md", 43, 46,
         "PRIOR_DATA_ALIGNMENT_AUDIT",
         "No camera/trigger evidence authenticated bag content correspondence.")
    note("v0_opaque_fields", "FMCW_GT_Alignment_and_Benchmark_v0.md", 58, 63,
         "PRIOR_GNSS_PARSING_AUDIT",
         "N/E/V/Q and unnamed fields remain opaque; not flight-state or speed labels.")
    note("v0_clock_limit", "FMCW_GT_Alignment_and_Benchmark_v0.md", 65, 86,
         "PRIOR_TIME_ALIGNMENT_AUDIT",
         "Bag/header/GNSS clocks differ; interval-derived offset not independent sync.")
    note("v0_no_surveyed_pose", "FMCW_GT_Alignment_and_Benchmark_v0.md", 88, 92,
         "PRIOR_SPATIAL_ALIGNMENT_AUDIT",
         "No measured heading/extrinsic/lever arm; scene still does not establish timed FOV.")
    note("v0_takeoff_area_speculation", "FMCW_GT_Alignment_and_Benchmark_v0.md", 156, 156,
         "PRIOR_SPECULATIVE_INTERPRETATION",
         "Possible takeoff/landing-area background is not evidence of takeoff/landing events.")
    note("closure_presence_and_failures", "FMCW_Calibration_Closure.md", 18, 32,
         "PRIOR_LIDAR_GNSS_CLOSURE_RESULT",
         "Five cross conditions have strong trajectory evidence; failed cases not absence.",
         presence="PROVISIONAL_LIDAR_GNSS_TRAJECTORY_EVIDENCE_NOT_FLIGHT_STATE")
    note("closure_time_unreliable", "FMCW_Calibration_Closure.md", 55, 63,
         "PRIOR_CALIBRATION_LIMITATION",
         "No shared hardware event/survey; offsets have boundary/alias ambiguity.")
    note("closure_aliases", "FMCW_Calibration_Closure.md", 143, 148,
         "PRIOR_TIME_OFFSET_DIAGNOSTIC",
         "Cross200 boundary offsets and repeated-flight aliases prevent timed hover mapping.")
    note("superseded_100m_no_return", "FMCW_Calibration_Closure.md", 150, 162,
         "PRIOR_RESULT_SUPERSEDING_OLD_INTERPRETATION",
         "100m_cross_10 contains target-like returns; old no-return conclusion not negative.",
         presence="OBSERVED_TRAJECTORY_EVIDENCE")
    note("closure_insufficient_not_absent", "FMCW_Calibration_Closure.md", 164, 169,
         "PRIOR_EVIDENCE_BOUNDARY",
         "Insufficient roundtrip trajectory evidence cannot establish absence/wrong package.")
    note("prior_e12_negative_decision", "results/temporal_evidence_mvp/negative_window_audit.json",
         1, 11, "PRIOR_EXPERIMENT_DECISION_NOT_NEW_ACQUISITION_EVIDENCE",
         "Unlabeled silver-exterior frames expressly rejected as negatives.")
    note("prior_e13_negative_decision", "results/published_baseline_dp_mvp/negative_window_audit.json",
         1, 11, "PRIOR_EXPERIMENT_DECISION_NOT_NEW_ACQUISITION_EVIDENCE",
         "No reliable negative labels; this is an earlier decision, not an independent source.")
    global_project = Path("/home/xin/.claude/projects/-home-xin-FMCW-LIDAR/memory/project-overview.md")
    if global_project.is_file():
        note("claude_project_overview", global_project, 65, 75,
             "OLDER_PROJECT_MEMORY_ALGORITHM_DESCRIPTION",
             "Tracker stationary-removal rules do not certify actual hover or target absence.")
    else:
        source_inventory.append({"item_id": "claude_project_overview",
                                 "source_path": str(global_project),
                                 "source_locator": "UNAVAILABLE", "source_sha256": "UNKNOWN",
                                 "source_type": "UNAVAILABLE_OPTIONAL_PROJECT_MEMORY",
                                 "interpretation": "Not present in this checkout environment.",
                                 "target_presence_evidence": "UNKNOWN",
                                 "target_absence_certification": "UNKNOWN",
                                 "physical_slow_certification": "UNKNOWN",
                                 "physical_hover_certification": "UNKNOWN",
                                 "independent_timestamp_linked_flight_state": "UNKNOWN",
                                 "formal_evaluation_eligible": "NO_FROM_THIS_SOURCE_ALONE"})

    inputs[display_path(SILVER)] = sha256(SILVER)
    labels = csv_rows(SILVER)
    grouped = {}
    for row_number, row in enumerate(labels, 2):
        assert row["label_status"] == "DEV_SILVER"
        assert row["paper_ground_truth"] == "False" and row["interpolated"] == "False"
        row["source_csv_line"] = row_number
        grouped.setdefault(row["condition_id"], []).append(row)
    steps, fragments = [], []
    for condition, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: int(row["frame_index"]))
        runs = [[rows[0]]]
        for left, right in zip(rows, rows[1:]):
            frame0, frame1 = int(left["frame_index"]), int(right["frame_index"])
            delta_t = float(right["bag_time_unix"]) - float(left["bag_time_unix"])
            assert delta_t > 0 and frame1 > frame0
            distance = math.dist(position(left), position(right))
            steps.append({
                "condition_id": condition, "frame_start": frame0, "frame_end": frame1,
                "frame_gap": frame1 - frame0, "consecutive_observed_frames": frame1 == frame0 + 1,
                "bag_time_start_unix": left["bag_time_unix"], "bag_time_end_unix": right["bag_time_unix"],
                "utc_start": utc(left["bag_time_unix"]), "utc_end": utc(right["bag_time_unix"]),
                "dt_s": delta_t, "displacement_m": distance, "observed_center_speed_mps": distance / delta_t,
                "source_file": display_path(SILVER),
                "source_start_locator": f"{display_path(SILVER)}:{left['source_csv_line']}",
                "source_end_locator": f"{display_path(SILVER)}:{right['source_csv_line']}",
                "presence_evidence_level": "RETROSPECTIVE_LIDAR_DERIVED_DEV_SILVER",
                "physical_speed_certification": "UNKNOWN_NOT_INDEPENDENT_UAV_STATE",
                "hover_certification": "UNKNOWN", "formal_evaluation_eligible": "NO",
                "speed_use": "DESCRIPTIVE_ONLY_NO_NEW_SLOW_THRESHOLD",
            })
            if frame1 == frame0 + 1:
                runs[-1].append(right)
            else:
                runs.append([right])
        for run_number, run in enumerate(runs):
            speeds = [math.dist(position(a), position(b)) /
                      (float(b["bag_time_unix"]) - float(a["bag_time_unix"]))
                      for a, b in zip(run, run[1:])]
            ordered = sorted(speeds)
            median = (ordered[len(ordered)//2] if len(ordered)%2 else
                      sum(ordered[len(ordered)//2-1:len(ordered)//2+1])/2) if ordered else "UNKNOWN_SINGLE_OBSERVATION"
            fragments.append({
                "condition_id": condition, "fragment_number": run_number,
                "first_frame": run[0]["frame_index"], "last_frame": run[-1]["frame_index"],
                "observations": len(run), "bag_time_start_unix": run[0]["bag_time_unix"],
                "bag_time_end_unix": run[-1]["bag_time_unix"],
                "utc_start": utc(run[0]["bag_time_unix"]), "utc_end": utc(run[-1]["bag_time_unix"]),
                "duration_s": float(run[-1]["bag_time_unix"]) - float(run[0]["bag_time_unix"]),
                "observed_speed_min_mps": min(speeds) if speeds else "UNKNOWN_SINGLE_OBSERVATION",
                "observed_speed_max_mps": max(speeds) if speeds else "UNKNOWN_SINGLE_OBSERVATION",
                "observed_speed_median_mps": median,
                "source_locator": f"{display_path(SILVER)}:{run[0]['source_csv_line']}-{run[-1]['source_csv_line']}",
                "selection": "ALL_MAXIMAL_FRAME_CONTIGUOUS_OBSERVED_RUNS_NO_SPEED_SELECTION",
                "physical_hover_certification": "UNKNOWN", "formal_evaluation_eligible": "NO",
            })

    slow_file = E19 / "short_slow_windows.csv"
    inputs[display_path(slow_file)] = sha256(slow_file)
    prior_windows = csv_rows(slow_file)
    trace = []
    observed_lookup = {(row["condition_id"], int(row["frame_index"])): row for row in labels}
    trace_sources = []
    for trace_file in sorted((E19 / "witness_background").glob("*.csv")):
        inputs[display_path(trace_file)] = sha256(trace_file)
        trace_sources.append(display_path(trace_file))
        for row_number, row in enumerate(csv_rows(trace_file), 2):
            membership = [w["window_id"] for w in prior_windows
                          if w["family"] == row["family"] and w["condition_id"] == row["condition_id"]
                          and int(w["first_frame"]) <= int(row["frame_index"]) <= int(w["last_frame"])]
            observed = observed_lookup.get((row["condition_id"], int(row["frame_index"])))
            trace.append({**row, "prior_e19_short_slow_window": "|".join(membership) or "NOT_IN_PRIOR_WINDOW",
                          "observed_silver_frame": observed is not None,
                          "silver_source_locator": (f"{display_path(SILVER)}:{observed['source_csv_line']}"
                                                    if observed else "NOT_SILVER_LABEL_FIXED_WITNESS_EXTRA_FRAME"),
                          "source_file": display_path(trace_file),
                          "source_locator": f"{display_path(trace_file)}:{row_number}",
                          "source_sha256": inputs[display_path(trace_file)],
                          "execution_status": "PREVIOUS_E19_STORED_DIAGNOSTIC_NOT_NEW_STRESS_TEST",
                          "trajectory_identity": "PREVIOUS_FIXED_LIDAR_SILVER_WITNESS_NOT_INDEPENDENT_UAV_GT",
                          "physical_hover_certification": "UNKNOWN", "formal_evaluation_eligible": "NO",
                          "motion_penalty_in_this_source": "UNKNOWN_NOT_STORED_HERE_NOT_RECOMPUTED"})

    history_queries = [
        run_git(["log", "--all", "--name-status", "--format=%h %s", "--",
                 "*.mp4", "*.avi", "*.mov", "*.mkv", "*.srt", "*.vtt", "*.xlsx", "*.xls",
                 "*.gpx", "*.kml", "*.nmea", "*.ubx", "*.ulg", "*.tlog", "*flight*", "*hover*",
                 "*takeoff*", "*landing*", "*GNSS*", "*RTK*", "*GPS*", ":!.local/**",
                 ":!.claude/settings*", ":!data/.claude/settings*", ":!fmcw/**", ":!results/**"]),
        run_git(["log", "--all", "--diff-filter=D", "--name-status", "--format=%h %s", "--",
                 "*.md", "*.csv", "*.json", "*.sh", "*.log", "*.txt", "bag/**", "data/**",
                 ":!.local/**", ":!.claude/settings*", ":!data/.claude/settings*", ":!fmcw/**"]),
        run_git(["log", "--all", "--name-status", "--format=%h %s", "--", "bag/**", "data/**",
                 ":!data/.claude/settings*"]),
    ]
    project_note_files = []
    for folder in (ROOT / ".claude", ROOT / "data/.claude"):
        if folder.is_dir():
            project_note_files.extend(display_path(path) for path in sorted(folder.rglob("*"))
                                      if path.is_file() and "settings" not in path.name.lower()
                                      and "credentials" not in path.name.lower())
    for path, identity in inputs.items():
        actual = Path(path) if Path(path).is_absolute() else ROOT / path
        if sha256(actual) != identity:
            raise RuntimeError(f"Input changed during inventory: {path}")
    assert len(steps) == len(labels) - len(grouped)
    assert sum(int(row["observations"]) for row in fragments) == len(labels)
    assert len(trace_sources) == 2 * len(grouped), "Expected two saved E19 witness families, no new witnesses"
    assert sum(row["observed_silver_frame"] for row in trace) == 2 * len(labels)
    output.mkdir(parents=True, exist_ok=True)
    datasets = (("log_source_inventory.csv", source_inventory), ("silver_motion_segments.csv", steps),
                ("silver_continuous_fragments.csv", fragments), ("prior_e19_slow_score_trace.csv", trace))
    for name, rows in datasets:
        write_csv(output / name, rows, list(rows[0]))
    history = {
        "status": "READ_ONLY_FILENAME_LEVEL_GIT_HISTORY_AUDIT",
        "queries": history_queries,
        "history_media_or_flight_state_path_matches": bool(history_queries[0]["stdout"].strip()),
        "history_deleted_project_record_path_matches": bool(history_queries[1]["stdout"].strip()),
        "project_local_nonsettings_note_files": project_note_files,
        "settings_credentials_or_local_blobs_read": False,
        "deleted_blobs_restored": False,
        "limitations": ["Path-level Git evidence is limited to committed project records.",
                        "Raw bags and PCDs are not certified target-absent by their filenames or missing logs.",
                        "No local session history, configuration/credentials, or deleted .local blobs were opened."]}
    (output / "log_history_coverage.json").write_text(json.dumps(history, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    execution = {
        "status": "TESTED_DESCRIPTIVE_INVENTORY_NOT_INDEPENDENT_FLIGHT_STATE_CERTIFICATION",
        "utc_completed": dt.datetime.now(dt.timezone.utc).isoformat(),
        "command": shlex.join([sys.executable, display_path(Path(__file__)), "--out", display_path(output)]),
        "source_sha256": sha256(__file__), "git_head": run_git(["rev-parse", "HEAD"])["stdout"].strip(),
        "python": sys.version, "platform": platform.platform(), "runtime_s": time.monotonic() - started,
        "inputs_sha256": inputs, "all_read_inputs_unchanged_during_run": True,
        "counts": {"source_inventory": len(source_inventory), "silver_observations": len(labels),
                   "silver_sequences": len(grouped), "all_adjacent_observation_steps": len(steps),
                   "gapped_observation_steps": sum(not r["consecutive_observed_frames"] for r in steps),
                   "all_frame_contiguous_fragments": len(fragments), "prior_e19_witness_trace_rows": len(trace),
                   "prior_witness_observed_silver_rows": sum(row["observed_silver_frame"] for row in trace),
                   "prior_witness_extra_nonlabel_rows": sum(not row["observed_silver_frame"] for row in trace)},
        "reliable_target_absence_from_log_branch": "DATA_GAP",
        "independent_physical_hover_from_log_branch": "DATA_GAP",
        "new_e18_stress_run": "NOT_RUN_NO_INDEPENDENT_FLIGHT_STATE_CERTIFICATION",
        "doppler_used": False, "new_slow_threshold": "NONE", "source_files_modified": False,
        "labels_graph_scores_weights_or_gates_modified": False,
        "raw_bag_or_cloud_full_decode": "NOT_RUN_BY_THIS_LOG_BRANCH",
        "preliminary_failed_attempt": {
            "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_log_inventory.py",
            "source_sha256": "1b09d08616628219ecb8dc59491e53427c193b152457fa3182908b3d658f48bb",
            "exit_code": 1, "error": "AssertionError: Expected two saved E19 witness families, no new witnesses",
            "cause": "Incorrect inventory-only count assumption: stored witnesses have 168 nodes, including 8 extra nonlabel graph nodes, not 160 labels.",
            "artifacts_written_before_failure": False,
            "correction": "Check observed label membership and source-file count; preserve every stored witness node and mark extras NOT_SILVER_LABEL.",
            "frozen_e18_or_input_changes": False},
        "outputs_sha256": {name: sha256(output / name) for name in OUTPUT_NAMES
                           if name != "log_evidence_execution.json"},
        "limitations": ["Adjacent speeds are observed LiDAR-center differences, not independently measured UAV velocities.",
                        "Every silver observation and every adjacent pair is exported, including gaps; none is promoted to negative or hover GT.",
                        "Every maximal frame-contiguous observed run is exported, without speed-based selection.",
                        "Stored E19 witness node/background values are copied unchanged; no E18 solve or motion score recomputation occurs.",
                        "Prior reports/decisions and project memory are not independent collection event records.",
                        "Absence of a committed/video/log record is not evidence of UAV absence."]}
    (output / "log_evidence_execution.json").write_text(json.dumps(execution, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": execution["status"], "counts": execution["counts"],
                      "out": display_path(output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
