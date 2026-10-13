#!/usr/bin/env python3
"""Read-only geometry annotations for an oracle-conditioned descriptive audit.

This module does not fit clocks/extrinsics, generate detections, interpolate
silver observations, or write artifacts.  Existing closure solutions are
exploratory localization scenarios, not calibrated uncertainty intervals.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

try:
    import build_gt_benchmark_v0 as existing_gt
except ModuleNotFoundError:
    from scripts import build_gt_benchmark_v0 as existing_gt


SETTINGS = {
    "epsilon": 1e-9,
    "eta_bin_edges": [0.0, 0.1, 0.2, 0.5, 1.0],
    "range_bin_width_m": 25.0,
    "existing_closure_near_best_composite_delta": 0.5,
    "existing_closure_near_best_position_rmse_delta_m": 2.0,
    "existing_anchor_neighborhood_radius_m": 3.0,
    "knot_equality_tolerance_s": 1e-6,
    "uncertainty_interpretation": "existing-fit sensitivity scenarios; not confidence intervals or certified bounds",
    "velocity_interpretation": "GNSS endpoint chord divided by sample interval; not instantaneous speed",
    "hover_interpretation": "UNKNOWN; GNSS endpoint chords do not certify hover",
    "aspect_interpretation": "UNKNOWN; no independently recorded UAV attitude",
    "measured_doppler_sign_convention": "historical dataset evidence: measured velocity approximately minus increasing geometric range-rate",
    "score_threshold": None,
}

PASS_CROSS = (
    "100m_cross_5", "200m_cross_5", "100m_cross_10",
    "200m_cross_10", "300m_cross_10",
)


def _csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def eta_bin(eta: float | None) -> str:
    if eta is None or not np.isfinite(eta):
        return "UNKNOWN"
    value = float(eta)
    edges = SETTINGS["eta_bin_edges"]
    for low, high in zip(edges[:-1], edges[1:]):
        if low <= value < high or (high == edges[-1] and value == high):
            return f"[{low:g},{high:g}{']' if high == edges[-1] else ')'}"
    return "OUTSIDE_PHYSICAL_INTERVAL"


def piecewise_state(source_t: np.ndarray, source_xyz: np.ndarray,
                    query_time: float, rotation: np.ndarray,
                    translation: np.ndarray) -> dict[str, Any] | None:
    """Return a chord-supported state, or None outside closed GNSS support.

    Interior knots use the right-hand chord, explicitly flagging their
    nondifferentiability.  The last endpoint uses the final left-hand chord.
    No endpoint clamping or extrapolation is allowed.
    """
    source_t = np.asarray(source_t, dtype=float)
    source_xyz = np.asarray(source_xyz, dtype=float)
    if source_t.ndim != 1 or source_xyz.shape != (len(source_t), 3):
        raise ValueError("GNSS arrays must be time[N], xyz[N,3]")
    if len(source_t) < 2 or not np.all(np.isfinite(source_t)) or not np.all(np.isfinite(source_xyz)):
        raise ValueError("at least two finite GNSS samples are required")
    if not np.all(np.diff(source_t) > 0):
        raise ValueError("GNSS sample times must be strictly increasing")
    query_time = float(query_time)
    if not np.isfinite(query_time) or query_time < source_t[0] or query_time > source_t[-1]:
        return None
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    index = min(int(np.searchsorted(source_t, query_time, side="right") - 1), len(source_t) - 2)
    dt = float(source_t[index + 1] - source_t[index])
    alpha = float((query_time - source_t[index]) / dt)
    enu = source_xyz[index] + alpha * (source_xyz[index + 1] - source_xyz[index])
    velocity_enu = (source_xyz[index + 1] - source_xyz[index]) / dt
    position = rotation @ enu + translation
    velocity = rotation @ velocity_enu
    radius = float(np.linalg.norm(position))
    speed = float(np.linalg.norm(velocity))
    epsilon = float(SETTINGS["epsilon"])
    if radius > epsilon:
        line_of_sight = position / radius
        range_rate = float(np.dot(line_of_sight, velocity))
        tangential_velocity = velocity - range_rate * line_of_sight
        tangential_speed = float(np.linalg.norm(tangential_velocity))
        eta = float(abs(range_rate) / (speed + epsilon))
        regime = ("CROSS_DOMINANT" if tangential_speed > abs(range_rate) else "RADIAL_DOMINANT")
        if speed <= epsilon:
            regime = "UNDEFINED_ZERO_CHORD_SPEED"
    else:
        line_of_sight = np.full(3, np.nan)
        range_rate = eta = tangential_speed = None
        regime = "UNDEFINED_SENSOR_ORIGIN"
    interior_knot = bool(np.any(np.abs(source_t[1:-1] - query_time) <= SETTINGS["knot_equality_tolerance_s"]))
    return {
        "segment_index": index,
        "query_gps_time_unix_s": query_time,
        "segment_start_gps_unix_s": float(source_t[index]),
        "segment_end_gps_unix_s": float(source_t[index + 1]),
        "segment_dt_s": dt,
        "segment_fraction": alpha,
        "time_since_left_knot_s": float(query_time - source_t[index]),
        "time_to_right_knot_s": float(source_t[index + 1] - query_time),
        "nearest_gnss_knot_distance_s": float(np.min(np.abs(source_t - query_time))),
        "at_interior_gnss_knot": interior_knot,
        "range_rate_derivative_status": "RIGHT_CHORD_AT_NONDIFFERENTIABLE_KNOT" if interior_knot else "PIECEWISE_CHORD_PROXY",
        "enu": enu,
        "position": position,
        "velocity_enu": velocity_enu,
        "velocity": velocity,
        "line_of_sight": line_of_sight,
        "range_m": radius,
        "device_origin_enu_range_m": float(np.linalg.norm(enu)),
        "speed_mps": speed,
        "range_rate_mps": range_rate,
        "tangential_speed_mps": tangential_speed,
        "eta": eta,
        "eta_bin": eta_bin(eta),
        "regime": regime,
        "range_bin_lower_m": int(math.floor(radius / SETTINGS["range_bin_width_m"]) * SETTINGS["range_bin_width_m"]),
    }


def _flatten_state(state: dict[str, Any]) -> dict[str, Any]:
    arrays = {"enu", "position", "velocity_enu", "velocity", "line_of_sight"}
    result = {key: value for key, value in state.items() if key not in arrays}
    for axis, value in zip("xyz", state["position"]):
        result[f"center_{axis}_m"] = float(value)
    for axis, value in zip("xyz", state["velocity"]):
        result[f"velocity_{axis}_mps"] = float(value)
    for axis, value in zip("xyz", state["line_of_sight"]):
        result[f"los_{axis}"] = float(value) if np.isfinite(value) else None
    for axis, value in zip("enu", state["enu"]):
        result[f"enu_{axis}_m"] = float(value)
    for axis, value in zip("enu", state["velocity_enu"]):
        result[f"enu_velocity_{axis}_mps"] = float(value)
    speed = state["speed_mps"]
    for axis, value in zip("xyz", state["velocity"]):
        result[f"flight_direction_unit_{axis}"] = float(value / speed) if speed > SETTINGS["epsilon"] else None
    return result


def load_geometry(root: str | Path) -> dict[str, Any]:
    """Load immutable prior artifacts and directly parse the original workbook."""
    root = Path(root).resolve()
    closure = root / "results/calibration_closure"
    workbook = root / "bag/20260205/飞行轨迹坐标.xlsx"
    alignment_path = closure / "alignment.json"
    alignment = json.loads(alignment_path.read_text(encoding="utf-8"))
    parsed, device = existing_gt.parse_workbook(workbook)
    alignment_rows = _csv(closure / "alignment_results.csv")
    profiles = _csv(closure / "offset_profiles.csv")
    diagnostics = _csv(closure / "offset_diagnostics.csv")
    provenance_path = root / "results/negative_hover_evidence_audit/gnss_samples.csv"
    provenance_rows = _csv(provenance_path)
    provenance = {(row["condition_id"], int(row["gnss_index"])): row for row in provenance_rows}
    if len(provenance) != len(provenance_rows):
        raise ValueError("duplicate GNSS source rows in E20 provenance")
    nominal = alignment["full_6dof"]
    poses = [{"pose_id": "nominal_full_6dof", "rotation": np.asarray(nominal["rotation_matrix"]),
              "translation": np.asarray(nominal["translation_m"]), "omitted_condition": "NONE"}]
    for omitted, model in sorted(alignment["leave_one_out"].items()):
        poses.append({"pose_id": f"loo_omit_{omitted}", "rotation": np.asarray(model["rotation_matrix"]),
                      "translation": np.asarray(model["translation_m"]), "omitted_condition": omitted})
    for pose in poses:
        if pose["rotation"].shape != (3, 3) or pose["translation"].shape != (3,):
            raise ValueError("invalid saved calibration pose")
        if not np.allclose(pose["rotation"] @ pose["rotation"].T, np.eye(3), rtol=0, atol=1e-9):
            raise ValueError("saved rotation is not orthonormal")

    gnss_rows, gnss_segments, sequences, summaries = [], [], {}, []
    checks = []

    def check_source(cid: str, point: Any) -> dict[str, str]:
        index = int(point.index if hasattr(point, "index") else point["index"])
        previous = provenance[(cid, index)]
        direct = vars(point) if hasattr(point, "index") else point
        mapping = {
            "gps_week": "gps_week", "tow_s": "gps_tow_s", "lat_deg": "lat_deg",
            "lon_deg": "lon_deg", "ellh_m": "ellh_m", "n_raw": "opaque_N_raw",
            "e_raw": "opaque_E_raw", "v_raw": "opaque_V_raw", "q_raw": "opaque_Q_raw",
            "unknown_1": "opaque_unlabelled_1", "unknown_2": "opaque_unlabelled_2", "unknown_3": "opaque_unlabelled_3",
        }
        for direct_key, previous_key in mapping.items():
            if not math.isclose(float(direct[direct_key]), float(previous[previous_key]), rel_tol=0, abs_tol=1e-10):
                raise ValueError(f"workbook/E20 mismatch {cid}/{index}/{direct_key}")
        expected_time = existing_gt.gps_to_unix(int(direct["gps_week"]), float(direct["tow_s"]))
        if not math.isclose(expected_time, float(previous["gps_utc_unix_s"]), rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"workbook/E20 time mismatch {cid}/{index}")
        if hasattr(point, "enu_e_m"):
            for key in ("enu_e_m", "enu_n_m", "enu_u_m"):
                if not math.isclose(float(direct[key]), float(previous[key]), rel_tol=0, abs_tol=1e-7):
                    raise ValueError(f"workbook/E20 ENU mismatch {cid}/{index}/{key}")
        checks.append({"condition_id": cid, "gnss_index": index, "source_cell": previous["source_cell"],
                       "direct_workbook_matches_E20": True})
        return previous

    for cid, title, archive, motion, nominal_range, nominal_speed in existing_gt.CONDITION_SPECS:
        points = parsed[title]
        times = np.asarray([point.gps_unix_s for point in points])
        xyz = np.asarray([[point.enu_e_m, point.enu_n_m, point.enu_u_m] for point in points])
        if len(points) < 2 or not np.all(np.diff(times) > 0):
            raise ValueError(f"insufficient/nonmonotonic GNSS for {cid}")
        source_rows = []
        for point in points:
            previous = check_source(cid, point)
            row = dict(previous)
            row.update({"direct_workbook_crosscheck": "PASS", "positional_accuracy_status": "UNKNOWN",
                        "attitude_status": "UNKNOWN", "independent_presence_certificate": "UNKNOWN",
                        "formal_evaluation_qualified": False})
            gnss_rows.append(row)
            source_rows.append(row)
        for index in range(len(points) - 1):
            dt = float(times[index + 1] - times[index])
            chord = (xyz[index + 1] - xyz[index]) / dt
            gnss_segments.append({
                "condition_id": cid, "condition_title": title, "segment_index": index,
                "left_source_cell": source_rows[index]["source_cell"], "right_source_cell": source_rows[index + 1]["source_cell"],
                "gps_start_unix_s": float(times[index]), "gps_end_unix_s": float(times[index + 1]),
                "segment_dt_s": dt, "chord_velocity_e_mps": float(chord[0]),
                "chord_velocity_n_mps": float(chord[1]), "chord_velocity_u_mps": float(chord[2]),
                "chord_speed_mps": float(np.linalg.norm(chord)), "speed_status": "ENDPOINT_CHORD_PROXY",
                "instantaneous_speed_status": "UNKNOWN", "hover_status": "UNKNOWN", "attitude_status": "UNKNOWN",
            })
        result_rows = [row for row in alignment_rows if row["condition_id"] == cid and row["model"] in {"full_6dof", "full_6dof_frozen"}]
        if len(result_rows) > 1:
            raise ValueError(f"multiple full-model primary alignment rows for {cid}")
        result = result_rows[0] if result_rows else None
        passed = result is not None and result["closure_pass"] == "True"
        if passed != (cid in PASS_CROSS):
            raise ValueError(f"closure cohort identity changed for {cid}")
        offset = float(result["time_offset_s"]) if result else None
        pp = [row for row in profiles if row["condition_id"] == cid and row["model"] == "full_6dof_frozen"]
        if pp:
            best = min(pp, key=lambda row: float(row["composite"]))
            near = [float(row["time_offset_s"]) for row in pp
                    if float(row["composite"]) <= float(best["composite"]) + SETTINGS["existing_closure_near_best_composite_delta"]
                    and float(row["position_rmse_m"]) <= float(best["position_rmse_m"]) + SETTINGS["existing_closure_near_best_position_rmse_delta_m"]]
        else:
            near = []
        offsets = sorted(set(near + ([offset] if offset is not None else [])))
        scenarios = [{"scenario_id": f"{pose['pose_id']}__offset_{oi:03d}",
                      "pose_id": pose["pose_id"], "rotation": pose["rotation"], "translation": pose["translation"],
                      "offset_s": scenario_offset, "is_nominal": pose["pose_id"] == "nominal_full_6dof" and scenario_offset == offset,
                      "offset_is_saved_nominal": scenario_offset == offset,
                      "offset_is_existing_near_best": scenario_offset in near}
                     for oi, scenario_offset in enumerate(offsets) for pose in poses]
        diagnostic = next((row for row in diagnostics if row["condition_id"] == cid and row["model"] == "full_6dof_frozen"), {})
        status = ("CLOSURE_PASS_CROSS_EXPLORATORY" if passed else
                  "EXCLUDED_CLOSURE_FAIL_OR_AMBIGUOUS" if result else "EXCLUDED_NO_QUALIFYING_ALIGNMENT")
        sequence = {
            "condition_id": cid, "condition_title": title, "archive": archive, "motion": motion,
            "nominal_range_m": nominal_range, "nominal_speed_mps": nominal_speed,
            "closure_pass": passed, "cohort_status": status, "alignment_result": result,
            "source_t": times, "source_xyz": xyz, "gnss_points": points, "gnss_source_rows": source_rows,
            "time_offset_s": offset, "near_best_offsets_s": near, "scenario_offsets_s": offsets,
            "scenarios": scenarios, "offset_diagnostics": diagnostic,
        }
        sequences[cid] = sequence
        summaries.append({
            "condition_id": cid, "condition_title": title, "archive": archive, "motion": motion,
            "nominal_range_m": nominal_range, "nominal_speed_mps": nominal_speed,
            "cohort_status": status, "closure_pass": passed,
            "gnss_sample_count": len(points), "gnss_segment_count": len(points) - 1,
            "gnss_start_unix_s": float(times[0]), "gnss_end_unix_s": float(times[-1]),
            "gnss_gap_min_s": float(np.min(np.diff(times))), "gnss_gap_max_s": float(np.max(np.diff(times))),
            "time_offset_s": offset, "scenario_count": len(scenarios),
            "near_best_offset_count": len(near), "near_best_offset_low_s": min(near) if near else None,
            "near_best_offset_high_s": max(near) if near else None,
            "offset_boundary_solution": diagnostic.get("boundary_solution", "UNKNOWN"),
            "offset_interior_local_minima": diagnostic.get("interior_local_minima", "UNKNOWN"),
            "nominal_position_rmse_m": float(result["position_rmse_m"]) if result else None,
            "nominal_range_rmse_m": float(result["range_rmse_m"]) if result else None,
            "nominal_doppler_gnss_rmse_mps": float(result["doppler_vs_gnss_rmse_mps"]) if result else None,
            "independent_positional_GT_status": "UNKNOWN", "independent_hover_status": "UNKNOWN",
        })
    device_previous = check_source("device_origin", device)
    gnss_rows.append({**device_previous, "direct_workbook_crosscheck": "PASS",
                      "positional_accuracy_status": "UNKNOWN", "formal_evaluation_qualified": False})
    if len(checks) != len(provenance_rows):
        raise ValueError("direct workbook/E20 row count differs")

    silver_path = root / "results/research_dev_v0/silver_labels.csv"
    trajectory_path = closure / "lidar_trajectories.csv"
    trajectory_rows = _csv(trajectory_path)
    closure_anchors = {(row["candidate_id"], int(row["frame_index"])): row for row in trajectory_rows}
    anchors = {}
    for row in _csv(silver_path):
        key = (row["condition_id"], int(row["frame_index"]))
        if key in anchors:
            raise ValueError(f"duplicate observed silver frame {key}")
        if row["label_status"] != "DEV_SILVER" or row["interpolated"] != "False" or row["paper_ground_truth"] != "False":
            raise ValueError("unexpected silver label evidence status")
        if key[0] not in PASS_CROSS:
            raise ValueError("silver anchor outside frozen five-sequence cohort")
        prior = closure_anchors[(row["candidate_id"], key[1])]
        if not math.isclose(float(row["bag_time_unix"]), float(prior["bag_time_unix_s"]), rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"silver/closure timestamp mismatch {key}")
        for axis in "xyz":
            if not math.isclose(float(row[f"center_{axis}_m"]), float(prior[f"{axis}_m"]), rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"silver/closure center mismatch {key}/{axis}")
        anchors[key] = dict(row)
    if len(anchors) != 80:
        raise ValueError("frozen observed anchor count changed")

    sources = [workbook, alignment_path, closure / "alignment_results.csv", closure / "offset_profiles.csv",
               closure / "offset_diagnostics.csv", closure / "extrinsic_sensitivity.csv", silver_path,
               trajectory_path, provenance_path, root / "scripts/build_gt_benchmark_v0.py"]
    return {
        "root": root, "settings": dict(SETTINGS), "sequences": sequences,
        "sequence_summary": summaries, "rotation": poses[0]["rotation"], "translation": poses[0]["translation"],
        "poses": poses, "anchors": anchors, "gnss_rows": gnss_rows, "gnss_samples": gnss_rows,
        "gnss_segments": gnss_segments, "device_origin": device, "workbook_provenance_checks": checks,
        "source_hashes": {str(path.relative_to(root)): _sha(path) for path in sources},
        "primary_condition_ids": list(PASS_CROSS),
        "evidence_boundary": "oracle-conditioned exploratory description; not independent test evidence or exact per-frame positional GT",
    }


def annotate_frame(context: dict[str, Any], cid: str, bagtime: float,
                   frame_index: int) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Annotate one raw LiDAR frame without any fitted/moved target center."""
    sequence = context["sequences"][cid]
    offset = sequence["time_offset_s"]
    state = (piecewise_state(sequence["source_t"], sequence["source_xyz"], float(bagtime) + offset,
                             context["rotation"], context["translation"]) if offset is not None else None)
    geometry = {
        "condition_id": cid, "frame_index": int(frame_index), "bag_time_unix_s": float(bagtime),
        "cohort_status": sequence["cohort_status"], "closure_pass": sequence["closure_pass"],
        "nominal_range_m": sequence["nominal_range_m"], "nominal_speed_mps": sequence["nominal_speed_mps"],
        "condition_nominal_range_m": sequence["nominal_range_m"], "condition_nominal_speed_mps": sequence["nominal_speed_mps"],
        "nominal_time_offset_s": offset, "gnss_nominal_supported": state is not None,
        "geometry_status": "GNSS_SUPPORTED_PROVISIONAL" if state is not None else "OUTSIDE_GNSS_SUPPORT_OR_NO_ALIGNMENT",
        "gnss_position_sigma_m": None, "gnss_position_sigma_status": "UNKNOWN",
        "clock_offset_sigma_s": None, "clock_offset_sigma_status": "UNKNOWN",
        "extrinsic_sigma_status": "UNKNOWN", "independent_time_anchor_status": "UNKNOWN",
        "interpolation_error_bound_m": None, "interpolation_error_bound_status": "UNKNOWN",
        "UAV_attitude_status": "UNKNOWN", "UAV_body_aspect_status": "UNKNOWN",
        "instantaneous_speed_status": "UNKNOWN", "physical_hover_status": "UNKNOWN",
        "antenna_to_UAV_center_lever_arm_status": "UNKNOWN", "independent_positional_GT": False,
        "uncertainty_scenarios_status": SETTINGS["uncertainty_interpretation"],
        "velocity_semantics": SETTINGS["velocity_interpretation"],
        "near_best_offset_low_s": min(sequence["near_best_offsets_s"]) if sequence["near_best_offsets_s"] else None,
        "near_best_offset_high_s": max(sequence["near_best_offsets_s"]) if sequence["near_best_offsets_s"] else None,
        "offset_boundary_solution": sequence["offset_diagnostics"].get("boundary_solution", "UNKNOWN"),
        "offset_interior_local_minima": sequence["offset_diagnostics"].get("interior_local_minima", "UNKNOWN"),
    }
    if state is not None:
        flat = _flatten_state(state)
        geometry.update(flat)
        geometry.update({f"nominal_{key}": value for key, value in flat.items() if key not in {"range_m", "speed_mps"}})
        geometry["nominal_predicted_range_m"] = flat["range_m"]
        geometry["nominal_chord_speed_mps"] = flat["speed_mps"]
        left = sequence["gnss_source_rows"][state["segment_index"]]
        right = sequence["gnss_source_rows"][state["segment_index"] + 1]
        geometry.update({"gnss_source_path": left["source_path"], "gnss_source_sheet": left["source_sheet"],
                         "gnss_left_source_cell": left["source_cell"], "gnss_right_source_cell": right["source_cell"]})
    valid = []
    for scenario in sequence["scenarios"]:
        candidate_state = piecewise_state(sequence["source_t"], sequence["source_xyz"],
                                          float(bagtime) + scenario["offset_s"], scenario["rotation"], scenario["translation"])
        if candidate_state is None:
            continue
        row = {"condition_id": cid, "frame_index": int(frame_index), "bag_time_unix_s": float(bagtime),
               "scenario_id": scenario["scenario_id"], "pose_id": scenario["pose_id"],
               "offset_s": scenario["offset_s"], "is_nominal": scenario["is_nominal"],
               "offset_is_saved_nominal": scenario["offset_is_saved_nominal"],
               "offset_is_existing_near_best": scenario["offset_is_existing_near_best"],
               "scenario_uncertainty_status": "SENSITIVITY_NOT_CALIBRATED_CI", **_flatten_state(candidate_state)}
        valid.append(row)
    geometry["valid_scenario_count"] = len(valid)
    geometry["possible_scenario_count"] = len(sequence["scenarios"])
    geometry["all_existing_scenarios_supported"] = bool(valid) and len(valid) == len(sequence["scenarios"])
    if state is not None and valid:
        scenario_xyz = np.asarray([[row[f"center_{axis}_m"] for axis in "xyz"] for row in valid])
        shifts = np.linalg.norm(scenario_xyz - state["position"], axis=1)
        geometry["scenario_center_shift_max_m"] = float(np.max(shifts))
        geometry["scenario_center_shift_median_m"] = float(np.median(shifts))
        geometry["scenario_range_min_m"] = min(row["range_m"] for row in valid)
        geometry["scenario_range_max_m"] = max(row["range_m"] for row in valid)
        etas = [row["eta"] for row in valid if row["eta"] is not None]
        geometry["scenario_eta_min"] = min(etas) if etas else None
        geometry["scenario_eta_max"] = max(etas) if etas else None
        geometry["scenario_regime_count"] = len({row["regime"] for row in valid})
    anchor = context["anchors"].get((cid, int(frame_index)))
    geometry["observed_closure_anchor_available"] = anchor is not None
    geometry["observed_closure_anchor_status"] = "DEV_SILVER_OBSERVED_NOT_POINT_GT" if anchor else "NO_OBSERVED_ANCHOR_NOT_A_CERTIFIED_MISS"
    if anchor is not None:
        if not math.isclose(float(anchor["bag_time_unix"]), float(bagtime), rel_tol=0, abs_tol=1e-5):
            raise ValueError(f"bag frame timestamp does not match saved observed anchor {cid}/{frame_index}")
        anchor_xyz = np.asarray([float(anchor[f"center_{axis}_m"]) for axis in "xyz"])
        for axis, value in zip("xyz", anchor_xyz):
            geometry[f"anchor_center_{axis}_m"] = float(value)
        geometry["anchor_candidate_id"] = anchor["candidate_id"]
        geometry["anchor_raw_support_prior"] = int(anchor["evidence_raw_points"])
        geometry["anchor_range_m"] = float(np.linalg.norm(anchor_xyz))
        geometry["anchor_minus_nominal_center_distance_m"] = float(np.linalg.norm(anchor_xyz - state["position"])) if state else None
        if valid:
            distances = [float(np.linalg.norm(anchor_xyz - np.asarray([row[f"center_{axis}_m"] for axis in "xyz"]))) for row in valid]
            geometry["anchor_min_existing_scenario_center_distance_m"] = min(distances)
            geometry["anchor_in_existing_scenario_3m_union"] = min(distances) <= SETTINGS["existing_anchor_neighborhood_radius_m"]
        else:
            geometry["anchor_min_existing_scenario_center_distance_m"] = None
            geometry["anchor_in_existing_scenario_3m_union"] = None
    return geometry, valid
