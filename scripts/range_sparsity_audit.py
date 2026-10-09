#!/usr/bin/env python3
"""Descriptive raw-cloud audit; never invokes a detector or tracker.

Existing observed centers are immutable spatial references, NOT point-level GT.
The nested balls quantify target-neighborhood proxies and their sensitivity.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import scipy
from scipy.spatial import cKDTree, distance
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.stats import rankdata

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "results/range_sparsity_audit"
DEV_LABELS = ROOT / "results/research_dev_v0/silver_labels.csv"
LEGACY = {
    "legacy_cross_20251119": (
        "cross", "cross/baseline_reverse/baseline_uav_frames.csv",
        "81-pm-cross-x10_2025-11-19-15-57-13", "cross", None,
    ),
    "legacy_longitudinal_20251119": (
        "longitudinal", "longitudinal/baseline_reverse/longitudinal_uav_frames.csv",
        "81-82-pm-x10_2025-11-19-15-38-07", "approach", None,
    ),
}
SETTINGS = {
    "version": 1,
    "diagnostic_only": True,
    "target_definition": "existing observed center; raw finite/nonzero xyz in nested 3D balls, NOT point identity GT",
    "target_radii_m": [0.5, 1.0, 3.0],
    "primary_radius_m": 3.0,
    "primary_radius_provenance": "unchanged research_dev_v0 neighborhood; 1m/0.5m sensitivity, no recentering",
    "voxel_sizes_m": [0.04, 1.0],
    "voxel_origin_m": [0.0, 0.0, 0.0],
    "voxel_frame": "sensor-relative [x,z,-y], not provisional gravity/ENU coordinates",
    "clutter_shell_inner_m": 3.0,
    "clutter_shell_outer_m": [15.0, 30.0],
    "clutter_definition": "unlabeled surrounding raw returns, not confirmed non-target negatives",
    "clutter_density_denominator": "complete spherical shell volume; scan/FOV/occlusion not corrected",
    "intensity_subset_inclusive": [10.0, 250.0],
    "broad_roi_m": {"x": [8.0, 510.0], "y": [-200.0, 200.0], "z": [-60.0, 60.0]},
    "range_bin_width_m": 25.0,
    "spacing_singleton": "UNKNOWN / empty numeric CSV field, never zero",
    "spacing_duplicates": "raw NN includes coincident returns; unique-xyz NN also reported",
    "pairwise_limit_points": 2000,
    "inference": "descriptive Spearman rho only; no p-values, no independent-frame assumption",
    "selection": "80 frozen Feb cross observations + 48 legacy measured cross + 20 legacy measured approach; no interpolation",
    "excluded": "legacy prediction-only rows; unresolved Feb roundtrips; old heuristic/interpolated 300m labels",
    "randomness": "none",
}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: ("" if isinstance(v, (float, np.floating)) and not np.isfinite(v) else v)
                        for k, v in row.items()})


def label_files():
    return [DEV_LABELS] + [ROOT / "results/data_archaeology" / v[1] for v in LEGACY.values()]


def protected_hashes():
    # Include ALL existing tracked Python implementations, not just main baseline.
    names = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    paths = [ROOT / n for n in names if n.endswith(".py") and n != "scripts/range_sparsity_audit.py"]
    paths += label_files() + [ROOT / "results" / group / "protocol.json" for group in
                             ["research_dev_v0", "temporal_evidence_mvp", "published_baseline_dp_mvp", "dp_failure_forensics"]]
    return {str(p.relative_to(ROOT)): sha256(p) for p in sorted(set(paths)) if p.is_file()}


def freeze(out):
    protocol = {**SETTINGS, "label_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in label_files()}}
    out.mkdir(parents=True, exist_ok=True)
    p = out / "protocol.json"
    if p.exists() and json.loads(p.read_text()) != protocol:
        raise RuntimeError("existing diagnostic protocol differs; refusing to change it")
    if not p.exists():
        p.write_text(json.dumps(protocol, ensure_ascii=False, indent=2) + "\n")
    print("FROZEN diagnostic definitions before raw-cloud statistics", flush=True)


def pcd_binary(path):
    """Read only header-declared named fields; ignore and RECORD legacy padding."""
    header = {}
    with Path(path).open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD missing DATA")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if header["DATA"] != ["binary"]:
            raise ValueError("audit raw PCD reader only supports binary, not filtered/compressed data")
        types = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 4): "<i4", ("U", 4): "<u4"}
        dtype = np.dtype([(name, types[(kind, int(size))]) for name, kind, size, count in
                          zip(header["FIELDS"], header["TYPE"], header["SIZE"], header["COUNT"])
                          if int(count) == 1])
        if any(int(x) != 1 for x in header["COUNT"]):
            raise ValueError("unsupported vector PCD fields")
        count = int(header["POINTS"][0])
        body = f.read(count * dtype.itemsize)
        if len(body) != count * dtype.itemsize:
            raise ValueError("truncated PCD")
        trailing = f.read()
        arr = np.frombuffer(body, dtype=dtype, count=count)
    return arr, {"pcd_declared_points": count, "trailing_bytes": len(trailing),
                 "trailing_all_zero": not any(trailing), "pcd_fields": " ".join(header["FIELDS"])}


def centers():
    rows = []
    for old in read_csv(DEV_LABELS):
        rows.append({
            "sequence_id": old["condition_id"], "cohort": "feb2026_DEV_SILVER", "motion": "cross",
            "nominal_range_m": int(old["nominal_range_m"]), "frame_index": int(old["frame_index"]),
            "label_status": old["label_status"], "time_s": float(old["bag_time_unix"]),
            "time_source": "bag_record_time", "center_x_m": float(old["center_x_m"]),
            "center_y_m": float(old["center_y_m"]), "center_z_m": float(old["center_z_m"]),
            "label_file": str(DEV_LABELS.relative_to(ROOT)), "legacy_target_cluster_points": "",
            "legacy_total_dynamic_points": "", "closure_evidence_raw_points": int(old["evidence_raw_points"]),
            "closure_evidence_1m_voxels": int(old["evidence_voxels"]),
        })
    for sid, (_, csvpath, dirname, motion, nominal) in LEGACY.items():
        for old in read_csv(ROOT / "results/data_archaeology" / csvpath):
            if old.get("predicted", "False").lower() == "true":
                continue
            frame = int(old["frame"])
            filtered_files = sorted((ROOT / "data" / (dirname + "_filtered_data")).glob("*.pcd"))
            # Exact filename matching, NOT old truncated CSV time or raw index.
            name = filtered_files[frame].name.removeprefix("filtered_")
            path = ROOT / "data" / dirname / name
            if not path.is_file() or int(float(name[:-4])) != int(float(old["timestamp"])):
                raise ValueError(f"legacy mapping mismatch {sid} {frame}")
            rows.append({
                "sequence_id": sid, "cohort": "legacy20251119_RETROSPECTIVE_PSEUDO", "motion": motion,
                "nominal_range_m": "" if nominal is None else nominal, "frame_index": frame,
                "label_status": "RETROSPECTIVE_TARGET_LIKE_NOT_EXTERNAL_GT", "time_s": float(name[:-4]),
                "time_source": "full_PCD_filename_timestamp_absolute_epoch_UNVERIFIED",
                "center_x_m": float(old["cx"]), "center_y_m": float(old["cy"]), "center_z_m": float(old["cz"]),
                "label_file": "results/data_archaeology/" + csvpath,
                "legacy_target_cluster_points": int(old["n_points"]),
                "legacy_total_dynamic_points": int(old["dynamic_points"]),
                "closure_evidence_raw_points": "", "closure_evidence_1m_voxels": "",
                "raw_pcd": str(path.relative_to(ROOT)), "legacy_truncated_time_s": float(old["timestamp"]),
            })
    return sorted(rows, key=lambda x: (x["sequence_id"], x["frame_index"]))


def quantiles(values, prefix):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {f"{prefix}_{k}": float("nan") for k in ("q25", "median", "q75")}
    return dict(zip([f"{prefix}_{k}" for k in ("q25", "median", "q75")], np.quantile(values, [.25, .5, .75])))


def occupied(points, voxel):
    return len(np.unique(np.floor(points / voxel).astype(np.int64), axis=0)) if len(points) else 0


def spacing(points):
    n = len(points)
    unique = np.unique(points, axis=0)
    row = {"raw_points": n, "unique_xyz_points": len(unique),
           "coincident_return_count": n - len(unique), "spacing_defined": n >= 2,
           "spacing_status": "MEASURED" if n >= 2 else "UNKNOWN_singleton_or_empty",
           "pairwise_status": "MEASURED" if 2 <= n <= SETTINGS["pairwise_limit_points"] else "UNKNOWN"}
    for voxel in SETTINGS["voxel_sizes_m"]:
        row[f"occupied_voxels_{voxel:g}m"] = occupied(points, voxel)
    for name in ("span_x_m", "span_y_m", "span_z_m", "diameter_m", "mst_connectivity_edge_max_m"):
        row[name] = float("nan")
    row.update(quantiles([], "nn_m"))
    row.update(quantiles([], "unique_nn_m"))
    row.update(quantiles([], "pairwise_m"))
    if n:
        row.update(dict(zip(("span_x_m", "span_y_m", "span_z_m"), np.ptp(points, axis=0))))
        if n == 1:
            row["diameter_m"] = 0.0  # observed support lower bound, NOT physical object extent
    if n >= 2:
        row.update(quantiles(cKDTree(points).query(points, k=2)[0][:, 1], "nn_m"))
    if len(unique) >= 2:
        row.update(quantiles(cKDTree(unique).query(unique, k=2)[0][:, 1], "unique_nn_m"))
    if 2 <= n <= SETTINGS["pairwise_limit_points"]:
        pairs = distance.pdist(points)
        row.update(quantiles(pairs, "pairwise_m"))
        row["diameter_m"] = float(pairs.max())
        if len(unique) >= 2:
            # Sparse csgraph preserves small nonzero distances; remove exact duplicates first.
            matrix = scipy.sparse.csr_matrix(distance.squareform(distance.pdist(unique)))
            edges = minimum_spanning_tree(matrix).data
            row["mst_connectivity_edge_max_m"] = float(edges.max())
    return row


def rho(x, y):
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    x, y = x[mask], y[mask]
    if len(x) < 3 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return float("nan"), len(x)
    return float(np.corrcoef(rankdata(x), rankdata(y))[0, 1]), len(x)


def measure(arr, label, meta):
    xyz = np.column_stack((arr["x"], arr["z"], -arr["y"])).astype(np.float64)
    intensity, velocity = arr["intensity"].astype(float), arr["velocity"].astype(float)
    valid = np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
    source_indices = np.flatnonzero(valid)
    xyz, intensity, velocity = xyz[valid], intensity[valid], velocity[valid]
    center = np.array([label[f"center_{a}_m"] for a in "xyz"])
    ranges = np.linalg.norm(xyz, axis=1)
    r = float(np.linalg.norm(center))
    d = np.linalg.norm(xyz - center, axis=1)
    intensity_mask = np.isfinite(intensity) & (intensity >= 10) & (intensity <= 250)
    roi = (xyz[:, 0] >= 8) & (xyz[:, 0] <= 510) & (np.abs(xyz[:, 1]) <= 200) & (np.abs(xyz[:, 2]) <= 60)
    outer = xyz[d >= 3.0]
    outer_tree = cKDTree(outer) if len(outer) else None
    context = {
        "organized_samples": len(arr), "valid_nonzero_raw_scene_points": len(xyz),
        "broad_roi_raw_points": int(roi.sum()), "broad_roi_intensity_points": int((roi & intensity_mask).sum()),
        "center_to_nearest_outside_3m_m": float(d[d >= 3].min()) if len(outer) else float("nan"),
    }
    clutter_rows = []
    for radius in SETTINGS["clutter_shell_outer_m"]:
        shell = (d >= 3) & (d < radius)
        volume = 4 * math.pi / 3 * (radius**3 - 3**3)
        for subset, mask in [("raw", shell), ("intensity_10_250", shell & intensity_mask)]:
            nn = cKDTree(xyz[mask]).query(xyz[mask], k=2)[0][:, 1] if mask.sum() >= 2 else []
            cr = {**label, "actual_target_range_m": r, "shell_inner_m": 3.0, "shell_outer_m": radius,
                  "subset": subset, "clutter_points": int(mask.sum()), "shell_volume_m3": volume,
                  "clutter_density_points_per_m3": float(mask.sum() / volume),
                  "clutter_nn_status": "MEASURED" if mask.sum() >= 2 else "UNKNOWN_less_than_2_points",
                  **quantiles(nn, "clutter_nn_m"), **quantiles(intensity[mask], "clutter_intensity"),
                  **quantiles(velocity[mask], "clutter_doppler_mps")}
            clutter_rows.append(cr)
            if subset == "raw":
                context[f"clutter_{radius:g}m_points"] = cr["clutter_points"]
                context[f"clutter_{radius:g}m_density"] = cr["clutter_density_points_per_m3"]
                context[f"clutter_{radius:g}m_nn_median_m"] = cr["clutter_nn_m_median"]
    frame_rows, point_rows = [], []
    for radius in SETTINGS["target_radii_m"]:
        mask = d <= radius
        target = xyz[mask]
        nearest_clutter = outer_tree.query(target, k=1)[0] if len(target) and outer_tree is not None else []
        row = {**label, "actual_target_range_m": r, "target_radius_m": radius,
               **context, **spacing(target), "intensity_10_250_points": int((mask & intensity_mask).sum()),
               "range_thickness_m": float(np.ptp(ranges[mask])) if mask.any() else float("nan"),
               "nearest_target_to_outside_3m_m": float(np.min(nearest_clutter)) if len(nearest_clutter) else float("nan"),
               **quantiles(intensity[mask], "intensity"), **quantiles(velocity[mask], "doppler_mps"),
               "xyz_finite_intensity_nonfinite_count": int(np.sum(mask & ~np.isfinite(intensity))),
               "xyz_finite_doppler_nonfinite_count": int(np.sum(mask & ~np.isfinite(velocity)))}
        frame_rows.append(row)
        if radius == SETTINGS["primary_radius_m"]:
            for idx in np.flatnonzero(mask):
                point_rows.append({"sequence_id": label["sequence_id"], "frame_index": label["frame_index"],
                                   "time_s": label["time_s"], "raw_source_point_index": int(source_indices[idx]),
                                   "x_m": xyz[idx, 0], "y_m": xyz[idx, 1], "z_m": xyz[idx, 2],
                                   "range_m": ranges[idx], "center_distance_m": d[idx],
                                   "intensity": intensity[idx], "measured_doppler_mps": velocity[idx],
                                   "point_identity_status": "TARGET_NEIGHBORHOOD_PROXY_NOT_POINT_GT"})
    return frame_rows, clutter_rows, point_rows, {**label, **meta, **context}


SUMMARY_METRICS = ["actual_target_range_m", "raw_points", "occupied_voxels_0.04m", "occupied_voxels_1m",
                   "diameter_m", "nn_m_median", "unique_nn_m_median", "pairwise_m_median",
                   "mst_connectivity_edge_max_m", "intensity_median", "doppler_mps_median",
                   "nearest_target_to_outside_3m_m", "clutter_15m_points", "clutter_15m_density",
                   "clutter_15m_nn_median_m", "clutter_30m_points", "clutter_30m_density", "clutter_30m_nn_median_m"]


def summaries(rows, keys):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[k] for k in keys)].append(row)
    result = []
    for key, group in groups.items():
        out = dict(zip(keys, key))
        out.update({"observed_frames": len(group), "zero_proxy_frames": sum(x["raw_points"] == 0 for x in group),
                    "singleton_proxy_frames": sum(x["raw_points"] == 1 for x in group),
                    "spacing_defined_frames": sum(x["raw_points"] >= 2 for x in group)})
        for metric in SUMMARY_METRICS:
            v = [float(x[metric]) for x in group]
            out.update(quantiles(v, metric))
            finite = np.asarray(v)[np.isfinite(v)]
            out[f"{metric}_defined_frames"] = len(finite)
            if len(finite):
                out[f"{metric}_min"], out[f"{metric}_max"] = float(finite.min()), float(finite.max())
        result.append(out)
    return result


def trends(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["sequence_id"], row["target_radius_m"])].append(row)
    result = []
    for (sid, radius), group in groups.items():
        group.sort(key=lambda x: x["time_s"])
        dt = np.diff([x["time_s"] for x in group])
        dr = np.diff([x["actual_target_range_m"] for x in group])
        dc = np.diff([x["raw_points"] for x in group])
        approach = dr < 0
        for metric in SUMMARY_METRICS[1:]:
            value, n = rho([x["actual_target_range_m"] for x in group], [x[metric] for x in group])
            result.append({"sequence_id": sid, "cohort": group[0]["cohort"], "motion": group[0]["motion"],
                           "target_radius_m": radius, "metric": metric, "spearman_rho_descriptive": value,
                           "paired_defined_frames": n, "p_value": "NOT_COMPUTED_serially_dependent_selected_frames",
                           "range_span_m": float(np.ptp([x["actual_target_range_m"] for x in group])),
                           "observed_dt_min_s": float(dt.min()), "observed_dt_max_s": float(dt.max()),
                           "approaching_intervals": int(approach.sum()),
                           "approach_count_up_intervals": int(np.sum(approach & (dc > 0))),
                           "approach_count_equal_intervals": int(np.sum(approach & (dc == 0))),
                           "approach_count_down_intervals": int(np.sum(approach & (dc < 0)))})
    return result


def audit(out):
    started = datetime.now(timezone.utc).isoformat()
    protocol = json.loads((out / "protocol.json").read_text())
    expected = {**SETTINGS, "label_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in label_files()}}
    if protocol != expected:
        raise RuntimeError("frozen definitions/labels changed")
    before = protected_hashes()
    labels = centers()
    frames, clutter, points, manifest = [], [], [], []
    input_files = {}

    def collect(arr, label, meta):
        a, b, c, d = measure(arr, label, meta)
        frames.extend(a); clutter.extend(b); points.extend(c); manifest.append(d)

    for label in labels:
        if "raw_pcd" in label:
            path = ROOT / label["raw_pcd"]
            arr, meta = pcd_binary(path)
            collect(arr, label, meta)
            input_files[label["raw_pcd"]] = {"size_bytes": path.stat().st_size, "sha256": sha256(path)}
    print("legacy PCD: 68 measured frames audited; prediction-only rows excluded", flush=True)
    from build_gt_benchmark_v0 import CONDITION_SPECS, iter_condition_pointclouds, DATA_ROOT
    lookup = defaultdict(dict)
    for label in labels:
        if "raw_pcd" not in label:
            lookup[label["sequence_id"]][label["frame_index"]] = label
    for spec in CONDITION_SPECS:
        sid = spec[0]
        if sid not in lookup:
            continue
        path = DATA_ROOT / spec[2]
        input_files[str(path.relative_to(ROOT))] = {"size_bytes": path.stat().st_size, "sha256": sha256(path)}
        count = 0
        for index, (arr, meta) in enumerate(iter_condition_pointclouds(spec)):
            if index in lookup[sid]:
                label = lookup[sid][index]
                if abs(float(meta["bag_time"]) - label["time_s"]) > 1e-5 or int(meta["point_step"]) != 80:
                    raise ValueError("bag identity/time/coordinate mismatch")
                collect(arr, label, {"raw_archive": str(path.relative_to(ROOT)), **meta})
                count += 1
            if index >= max(lookup[sid]):
                break
        if count != len(lookup[sid]):
            raise ValueError("missing DEV observations")
        print(f"{sid}: {count} raw measured frames audited", flush=True)
    frames.sort(key=lambda x: (x["sequence_id"], x["frame_index"], x["target_radius_m"]))
    for group in set((r["sequence_id"], r["target_radius_m"]) for r in frames):
        subset = [r for r in frames if (r["sequence_id"], r["target_radius_m"]) == group]
        rr = [r["actual_target_range_m"] for r in subset]
        ts = [r["time_s"] for r in subset]
        drdt = np.gradient(rr, ts)
        for row, rate in zip(subset, drdt):
            row["observed_center_range_rate_mps"] = float(rate)
            row["minus_measured_doppler_minus_range_rate_mps"] = -row["doppler_mps_median"] - float(rate)
            row["range_bin_lower_m"] = int(math.floor(row["actual_target_range_m"] / 25) * 25)
    write_csv(out / "frame_metrics.csv", frames)
    write_csv(out / "clutter_metrics.csv", clutter)
    write_csv(out / "target_neighborhood_points.csv", points)
    write_csv(out / "frame_manifest.csv", manifest)
    write_csv(out / "sequence_summary.csv", summaries(frames, ["sequence_id", "cohort", "motion", "nominal_range_m", "target_radius_m"]))
    feb = [r for r in frames if r["cohort"] == "feb2026_DEV_SILVER"]
    write_csv(out / "nominal_range_summary.csv", summaries(feb, ["nominal_range_m", "target_radius_m"]))
    write_csv(out / "actual_range_bins.csv", summaries(frames, ["sequence_id", "cohort", "motion", "target_radius_m", "range_bin_lower_m"]))
    write_csv(out / "range_trends.csv", trends(frames))
    # Independent, previously recorded 3m survival counts are a validation oracle, not a tuning signal.
    old_stage = {(r["condition_id"], int(r["frame_index"])): r for r in read_csv(ROOT / "results/research_dev_v0/stage_survival.csv")}
    checks = []
    for r in frames:
        if r["target_radius_m"] != 3:
            continue
        old = old_stage.get((r["sequence_id"], r["frame_index"]))
        if old:
            checks.append({"sequence_id": r["sequence_id"], "frame_index": r["frame_index"],
                           "check": "raw_3m_vs_existing_survival", "current_count": r["raw_points"],
                           "reference_count": int(old["raw_target_points"]),
                           "passed": r["raw_points"] == int(old["raw_target_points"])})
    if len(checks) != 80 or not all(c["passed"] for c in checks):
        raise ValueError("historical raw survival validation failed")
    write_csv(out / "count_validation.csv", checks)
    after = protected_hashes()
    if before != after:
        raise RuntimeError("protected research code/labels/protocol changed during diagnostic")
    (out / "protected_files.json").write_text(json.dumps({"before": before, "after": after, "identical": True}, indent=2) + "\n")
    record = {
        "started_at_utc": started, "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "working_directory": str(ROOT),
        "parent_git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip(),
        "git_status_at_completion": subprocess.check_output(["git", "status", "--short"], cwd=ROOT).decode(),
        "script_sha256": sha256(Path(__file__)), "protocol_sha256": sha256(out / "protocol.json"),
        "python": sys.version, "platform": platform.platform(), "numpy": np.__version__, "scipy": scipy.__version__,
        "inputs": input_files, "label_sha256": protocol["label_sha256"],
        "observed_frames": len(manifest), "frame_radius_rows": len(frames), "point_proxy_rows": len(points),
        "detector_or_tracker_executed": False, "protected_files_identical": True,
        "validation_raw_count_matches": len(checks),
        "output_sha256": {p.name: sha256(p) for p in sorted(out.glob("*.csv"))},
    }
    (out / "run_manifest.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    print(f"DONE {len(manifest)} observed frames, {len(frames)} radius rows, {len(points)} raw proxy points", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--phase", choices=["freeze", "audit"], required=True)
    p.add_argument("--output", type=Path, default=DEFAULT_OUT)
    args = p.parse_args()
    if args.phase == "freeze":
        freeze(args.output)
    else:
        audit(args.output)


if __name__ == "__main__":
    main()
