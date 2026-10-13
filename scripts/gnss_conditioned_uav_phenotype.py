#!/usr/bin/env python3
"""Oracle-conditioned descriptive extraction. No detector, tracker or model fit.

Two different observation products are retained: uncertain GNSS neighborhoods
on all nominally supported frames, and previously localized closure anchors.
Neither provides point identity GT or independent evaluation evidence.
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
from scipy.spatial import cKDTree
from scipy.stats import spearmanr

import gnss_phenotype_geometry as geometry
import gnss_phenotype_io as rich_io
import gnss_phenotype_statistics as stats

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/gnss_conditioned_uav_phenotype"
SEQUENCES = ["100m_cross_5", "200m_cross_5", "100m_cross_10", "200m_cross_10", "300m_cross_10"]
SETTINGS = {
    "study": "GNSS_CONDITIONED_UAV_PHENOTYPE", "version": 1,
    "purpose": "descriptive, oracle-conditioned, development evidence; NOT a detector or benchmark",
    "sequences": SEQUENCES, "gnss_radii_m": [1.0, 3.0, 10.0], "anchor_radii_m": [1.0, 3.0],
    "scenario_union_radius_m": 3.0, "voxel_sizes_m": [0.04, 1.0],
    "voxel_origin": [0, 0, 0], "voxel_frame": "raw [sensor_x,sensor_z,-sensor_y]",
    "control_azimuth_rotations_deg": [-20.0, -10.0, 10.0, 20.0],
    "control_selection": "geometry only, exact same range/elevation/shape; reject overlaps before counting",
    "control_exclusion": "scenario union 3m + nominal GNSS 3m + saved closure anchor 3m + analyzed region",
    "control_status": "LOCAL_CLUTTER_PROXY; NOT independently certified target-absent; FOV/occlusion UNKNOWN",
    "xyz_validity": "finite x/y/z and at least one nonzero coordinate; no intensity/height/motion filters",
    "alignment": "existing shared six-DoF and per-sequence offset, unchanged; no optimization",
    "uncertainty": "existing near-best offset grid and leave-one-out extrinsics, not calibrated confidence bounds",
    "gnss": "piecewise linear, within sample endpoints only; segment chord velocity, not instantaneous GT",
    "eta_eps": 1e-9, "eta_bins": [0.0, 0.1, 0.2, 0.5, 1.0], "range_bin_width_m": 25,
    "doppler_sign": "dataset convention measured velocity ~= -dr/dt; also retain opposite-sign diagnostic",
    "point_time": "record only; undocumented unit/anchor, no deskew",
    "pairwise_limit": 2000, "sampling": "all frames, all selected region points; no random subsampling",
    "matched_effect": "equal frame weight, equal eligible controls per frame; AUC=P(U>C)+0.5P(tie), delta=2AUC-1",
    "inference": "no p-values, IID confidence intervals, score thresholds, fitted weights or classifiers",
    "missingness": "region-empty runs, not sensor missed detections; break GNSS support/segment/frame-index gaps or >2 median bag dt",
    "range_conditioning": "range_bin_m uses provisional GNSS-to-sensor range; retain region_reference_range_m and its bin separately",
    "expected_counts": {"100m_cross_5": [32, 27, 19], "200m_cross_5": [83, 22, 22],
                        "100m_cross_10": [46, 30, 11], "200m_cross_10": [48, 39, 18], "300m_cross_10": [63, 26, 10]},
    "preflight": "first-frame schema inspected before freeze; no full-cohort outcome/parameter selection",
}
FEATURES = ["raw_points", "unique_xyz_points", "occupied_voxels_0.04m", "occupied_voxels_1m", "diameter_m",
            "radial_span_m", "tangential_rms_m", "unique_nn_m_median", "mst_connectivity_edge_max_m",
            "intensity_median", "velocity_median", "velocity0_median", "velocity_abs_median",
            "velocity0_abs_median", "doppler_abs_residual_mps", "time_span", "ring_unique_count",
            "range_thickness_m", "span_x_m", "span_y_m", "span_z_m", "radial_rms_m",
            "raw_cov_eigenvalue_1_m2", "raw_cov_eigenvalue_2_m2", "raw_cov_eigenvalue_3_m2",
            "nn_m_median", "unique_pairwise_m_median", "coincident_return_count",
            "acceleration_median", "surface_evenness_median", "velocity_direction_median",
            "acceleration_direction_median", "voxel_0.04m_mean_collapse_fraction",
            "voxel_1m_mean_collapse_fraction", "voxel_0.04m_mean_diameter_m", "voxel_1m_mean_diameter_m"]


def utc():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean(v) for v in value]
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


def jwrite(path, obj):
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite {path}")
    path.write_text(json.dumps(clean(obj), ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def cwrite(path, rows):
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite {path}")
    rows = [clean(r) for r in rows]
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else v for k, v in r.items()})


def cread(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if v == "":
                r[k] = None
            elif v in ["True", "False"]:
                r[k] = v == "True"
            else:
                try:
                    r[k] = float(v)
                except ValueError:
                    pass
    return rows


def frozen_sources():
    # Existing E20's 302 protected scientific inputs include E18 trees/labels.
    released = json.loads((ROOT / "results/negative_hover_evidence_audit/protected_input_identity.json").read_text())
    mismatches = [p for p, h in released.items() if not (ROOT / p).is_file() or sha(ROOT / p) != h]
    if mismatches:
        raise RuntimeError(f"Released scientific inputs changed: {mismatches}")
    extra = ["results/calibration_closure/alignment.json",
             "results/calibration_closure/offset_profiles.csv", "results/calibration_closure/extrinsic_sensitivity.csv",
             "results/calibration_closure/offset_diagnostics.csv", "results/calibration_closure/alignment_results.csv",
             "results/calibration_closure/lidar_trajectories.csv", "results/negative_hover_evidence_audit/gnss_samples.csv"]
    py = subprocess.check_output(["git", "ls-files", "--", "*.py"], cwd=ROOT, text=True).splitlines()
    extra += [f"results/calibration_closure/raw_voxel_cache/{cid}.json" for cid in SEQUENCES]
    released.update({p: sha(ROOT / p) for p in extra + py if (ROOT / p).is_file()})
    return released


def code_hashes():
    paths = [Path(__file__)] + [ROOT / "scripts" / f"gnss_phenotype_{n}.py" for n in ["geometry", "io", "statistics"]]
    return {str(p.relative_to(ROOT)): sha(p) for p in paths}


def freeze(out):
    out.mkdir(parents=True, exist_ok=True)
    jwrite(out / "protocol.json", {**SETTINGS, "frozen_utc": utc(), "code_sha256": code_hashes(),
                                  "source_sha256": frozen_sources(), "git_head": subprocess.check_output(
                                      ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()})
    print("FROZEN descriptive definitions; no weights/thresholds/models selected", flush=True)


def verify(out):
    p = json.loads((out / "protocol.json").read_text())
    mismatches = [name for name, h in {**p["source_sha256"], **p["code_sha256"]}.items() if sha(ROOT / name) != h]
    if mismatches:
        raise RuntimeError(f"Frozen input/code changed: {mismatches}")
    return p


def rotation_z(angle):
    a = math.radians(angle)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def disjoint(centers_a, ra, centers_b, rb):
    return not len(centers_b) or bool(np.all(np.linalg.norm(np.asarray(centers_a)[:, None, :] - np.asarray(centers_b)[None, :, :], axis=2) > ra + rb))


def make_regions(g, scenarios):
    """Construct geometry-only matched proxies; no measured cloud information."""
    target, controls, rejected = [], [], []
    nominal = np.array([g.get(f"center_{k}_m", np.nan) for k in "xyz"], float)
    anchor = np.array([g.get(f"anchor_center_{k}_m", np.nan) for k in "xyz"], float)
    sc = np.array([[s[f"center_{k}_m"] for k in "xyz"] for s in scenarios], float).reshape(-1, 3)
    tube = sc.copy()
    if np.all(np.isfinite(nominal)):
        tube = np.vstack([tube, nominal])
        for radius in SETTINGS["gnss_radii_m"]:
            target.append((f"GNSS_{radius:g}m", nominal[None, :], radius, nominal))
        if len(sc):
            target.append(("SCENARIO_UNION_3m", sc, 3.0, nominal))
    if np.all(np.isfinite(anchor)):
        tube = np.vstack([tube, anchor])
        for radius in SETTINGS["anchor_radii_m"]:
            target.append((f"ANCHOR_{radius:g}m", anchor[None, :], radius, anchor))
    for name, centers, radius, reference in target:
        for angle in SETTINGS["control_azimuth_rotations_deg"]:
            rotated = centers @ rotation_z(angle).T
            rid = f"{name}_az{angle:+g}"
            if not disjoint(rotated, radius, tube, 3.0) or not disjoint(rotated, radius, centers, radius):
                rejected.append({"alignment_scenario": name, "region_id": rid, "angle_deg": angle,
                                 "reason": "overlap_uncertainty_tube_or_analyzed_region"})
                continue
            controls.append((name, rotated, radius, reference @ rotation_z(angle).T, rid, angle))
    regions = [(n, c, r, ref, n, 0.0, "UAV_CONDITIONED") for n, c, r, ref in target]
    regions += [(n, c, r, ref, rid, a, "LOCAL_CLUTTER_PROXY") for n, c, r, ref, rid, a in controls]
    return regions, rejected


def region_indices(tree, centers, radius):
    found = tree.query_ball_point(centers, radius)
    if not len(found) or not any(len(x) for x in found):
        return np.empty(0, dtype=np.int64)
    return np.unique(np.concatenate(found)).astype(np.int64)


def point_record_export(arr, union, names, offsets, member_indices, meta):
    """Lossless export even for out-of-order/overlapping PointField layouts.

    NPY cannot serialize such structured dtypes. Store complete original point
    bytes plus native named columns and the schema instead; no float coercion.
    This serialization-only correction does not change region membership.
    """
    payload = {"raw_slot_indices": union,
               "raw_point_bytes": arr.view(np.uint8).reshape(len(arr), arr.dtype.itemsize)[union],
               "field_schema_json": np.asarray(json.dumps(clean(meta.get("field_schema", [])))),
               "region_names": np.asarray(names), "region_offsets": offsets,
               "region_record_indices": member_indices}
    payload.update({"field__" + name: arr[name][union] for name in arr.dtype.names})
    return payload


def point_features(arr, idx, xyz, center, g):
    row = stats.spatial_descriptors(xyz[idx], center, voxel_sizes=(0.04, 1.0), pairwise_limit=2000)
    row["raw_points"] = len(idx)
    for name in ["intensity", "velocity", "velocity0", "time", "acceleration", "surface_evenness", "velocity_direction", "acceleration_direction", "ring"]:
        if name not in arr.dtype.names:
            continue
        values = np.asarray(arr[name][idx])
        row.update(stats.numeric_descriptors(values, name))
        finite = values[np.isfinite(values)]
        row[name + "_unique_count"] = len(np.unique(finite))
        row[name + "_span"] = float(np.ptp(finite)) if len(finite) else np.nan
        if name in ["velocity", "velocity0"]:
            row.update(stats.numeric_descriptors(np.abs(values), name + "_abs"))
            expected = g.get("nominal_range_rate_mps")
            med = float(np.median(finite)) if len(finite) else np.nan
            if expected is not None and np.isfinite(expected):
                residual = -med - expected
                row[name + "_signed_range_rate_residual_mps"] = residual
                row[name + "_opposite_sign_residual_mps"] = med - expected
                if name == "velocity":
                    row["doppler_abs_residual_mps"] = abs(residual)
                if "scenario_range_rate_min_mps" in g:
                    row[name + "_residual_sensitivity_min_mps"] = -med - g["scenario_range_rate_max_mps"]
                    row[name + "_residual_sensitivity_max_mps"] = -med - g["scenario_range_rate_min_mps"]
    return row


def extract(out):
    protocol = verify(out)
    ctx = geometry.load_geometry(ROOT)
    if (out / "extraction_started.json").exists():
        raise FileExistsError("Existing extraction attempt preserved; no automatic overwrite/retry")
    started = time.perf_counter()
    jwrite(out / "extraction_started.json", {"utc": utc(), "command": sys.argv, "python": sys.version,
           "numpy": np.__version__, "scipy": scipy.__version__, "platform": platform.platform(),
           "protocol_sha256": sha(out / "protocol.json")})
    cwrite(out / "gnss_source_samples.csv", ctx["gnss_rows"])
    cwrite(out / "gnss_source_segments.csv", ctx["gnss_segments"])
    cwrite(out / "alignment_sequence_summary.csv", ctx["sequence_summary"])
    jwrite(out / "workbook_crosscheck.json", {"checks": ctx["workbook_provenance_checks"],
           "source_sha256": ctx["source_hashes"], "device_origin": ctx["device_origin"]})
    inventories = []
    for cid, seq in ctx["sequences"].items():
        inventories.append({"sequence_id": cid, "included": cid in SEQUENCES,
                            "closure_status": seq.get("cohort_status"), "reason": "five closure-PASS cross sequences" if cid in SEQUENCES else "failed/ambiguous; no raw-return analysis or negative inference"})
    cwrite(out / "sequence_inventory.csv", inventories)
    all_frames, all_regions, all_scenarios, all_fields, all_relations, all_rejected, sources = [], [], [], [], [], [], []
    try:
        manifest = {r["condition_id"]: r for r in cread(ROOT / "gt_manifest.csv")}
        for cid in SEQUENCES:
            seq = ctx["sequences"][cid]
            archive = ROOT / "bag/20260205" / seq["archive"]
            before = archive.stat()
            archive_sha = sha(archive)
            prior_frames = json.loads((ROOT / f"results/calibration_closure/raw_voxel_cache/{cid}.json").read_text())["frames"]
            audit = {}
            n = 0
            (out / "point_records" / cid).mkdir(parents=True, exist_ok=True)
            for arr, meta in rich_io.iter_rich_frames(archive, audit=audit):
                fi, bt = int(meta["frame_index"]), float(meta["bag_time"])
                prior = prior_frames[fi]
                if (int(prior["frame_index"]) != fi or int(prior["seq"]) != meta["seq"] or
                        abs(float(prior["bag_time"]) - bt) > 1e-6 or
                        abs(float(prior["header_time"]) - meta["header_time"]) > 1e-6 or
                        meta["topic"] != manifest[cid]["topic"]):
                    raise RuntimeError(f"Raw frame identity differs from closure/manifest {cid}/{fi}")
                base = {"sequence_id": cid, "frame_index": fi, "time_s": bt, "nominal_range_m": int(seq["nominal_range_m"])}
                g, scenarios = geometry.annotate_frame(ctx, cid, bt, fi)
                base.update(g)
                base["gnss_segment_id"] = g.get("segment_index", "UNKNOWN")
                base["gnss_supported"] = g["gnss_nominal_supported"]
                base["anchor_available"] = g["observed_closure_anchor_available"]
                base["range_bin_m"] = g.get("range_bin_lower_m", "UNKNOWN")
                if g.get("speed_mps", 0) > 0:
                    base["motion_los_angle_deg"] = math.degrees(math.acos(np.clip(g["range_rate_mps"] / g["speed_mps"], -1, 1)))
                if scenarios:
                    rates = [s["range_rate_mps"] for s in scenarios if s["range_rate_mps"] is not None]
                    base["scenario_range_rate_min_mps"] = min(rates) if rates else np.nan
                    base["scenario_range_rate_max_mps"] = max(rates) if rates else np.nan
                    g.update({k: base[k] for k in ["scenario_range_rate_min_mps", "scenario_range_rate_max_mps"]})
                mask = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"]) & ((arr["x"] != 0) | (arr["y"] != 0) | (arr["z"] != 0))
                slots = np.flatnonzero(mask)
                if len(arr) != prior["total_samples"] or len(slots) != prior["valid_returns"]:
                    raise RuntimeError(f"Raw return accounting differs from closure {cid}/{fi}")
                valid_arr = arr[mask]
                xyz = np.column_stack([valid_arr["x"], valid_arr["z"], -valid_arr["y"]]).astype(float)
                frame = {**base, **{k: v for k, v in meta.items() if k not in ["fields"]},
                         "schema_json": meta.get("field_schema"), "all_slots": len(arr), "finite_nonzero_xyz": len(slots)}
                all_frames.append(frame)
                for s in scenarios:
                    all_scenarios.append({**base, **s})
                for field in rich_io.scalar_field_audit(arr, mask):
                    all_fields.append({**{k: base[k] for k in ["sequence_id", "frame_index", "time_s"]}, **field})
                all_relations.append({**{k: base[k] for k in ["sequence_id", "frame_index", "time_s"]},
                                      **rich_io.field_relationship_audit(arr, mask, meta)})
                regions, rejected = make_regions(g, scenarios)
                all_rejected.extend({**base, **r} for r in rejected)
                if regions:
                    tree = cKDTree(xyz)
                    memberships, names = [], []
                    for name, centers, radius, reference, rid, angle, role in regions:
                        idx = region_indices(tree, centers, radius)
                        r = {**base, "alignment_scenario": name, "region_role": role, "region_id": rid,
                             "control_angle_deg": angle, "region_radius_m": radius, "region_center_count": len(centers),
                             "region_reference_x_m": reference[0], "region_reference_y_m": reference[1], "region_reference_z_m": reference[2],
                             "region_reference_range_m": np.linalg.norm(reference),
                             "region_reference_range_bin_m": int(math.floor(np.linalg.norm(reference) / 25) * 25),
                             "independent_target_point_identity": "UNKNOWN", "formal_evaluation_qualified": False}
                        r.update(point_features(valid_arr, idx, xyz, reference, g))
                        all_regions.append(r)
                        memberships.append(slots[idx])
                        names.append(rid)
                    union = np.unique(np.concatenate(memberships))
                    offsets = np.r_[0, np.cumsum([len(x) for x in memberships])]
                    member_indices = np.concatenate([np.searchsorted(union, x) for x in memberships])
                    np.savez_compressed(out / "point_records" / cid / f"frame_{fi:04d}.npz",
                                        **point_record_export(arr, union, names, offsets, member_indices, meta))
                n += 1
                if n % 10 == 0:
                    print(f"{cid}: decoded {n} frames; selected rows {len(all_regions)}", flush=True)
            after = archive.stat()
            if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
                raise RuntimeError(f"Raw archive changed during reading: {archive}")
            expected_frames, expected_gnss, expected_anchors = SETTINGS["expected_counts"][cid]
            seq_frames = [f for f in all_frames if f["sequence_id"] == cid]
            if (n != expected_frames or n != len(prior_frames) or
                    sum(f["gnss_supported"] for f in seq_frames) != expected_gnss or
                    sum(f["anchor_available"] for f in seq_frames) != expected_anchors or
                    not audit.get("complete_member_hash") or not audit.get("zip_crc_verified_at_eof") or
                    audit["bytes_read"] != audit["bag_uncompressed_bytes"]):
                raise RuntimeError(f"Sequence/EOF accounting failed for {cid}")
            sources.append({"sequence_id": cid, "archive_path": str(archive.relative_to(ROOT)), "archive_sha256": archive_sha,
                            "archive_size_bytes": before.st_size, "archive_mtime_ns": before.st_mtime_ns,
                            "frames_decoded": n, **audit})
            print(f"COMPLETE {cid}: {n} raw frames, full member hash/EOF audited", flush=True)
        verify(out)
        for name, rows in [("frames.csv", all_frames), ("region_features.csv", all_regions), ("alignment_scenarios.csv", all_scenarios),
                           ("rich_field_distributions.csv", all_fields), ("rich_field_relationships.csv", all_relations),
                           ("control_exclusions.csv", all_rejected), ("raw_source_identity.csv", sources)]:
            cwrite(out / name, rows)
        jwrite(out / "extraction_completed.json", {"utc": utc(), "elapsed_s": time.perf_counter() - started,
               "frames": len(all_frames), "region_rows": len(all_regions), "source_audits": sources,
               "protected_inputs_unchanged": len(protocol["source_sha256"]), "code_unchanged": True})
    except BaseException as exc:
        jwrite(out / "extraction_failed.json", {"utc": utc(), "exception": repr(exc), "elapsed_s": time.perf_counter() - started,
                                               "completed_source_audits": sources, "frames_seen": len(all_frames)})
        raise


def finite_values(rows, feature):
    return np.asarray([r[feature] for r in rows if isinstance(r.get(feature), (float, int)) and np.isfinite(r[feature])], float)


def summarize(out):
    verify(out)
    regions = cread(out / "region_features.csv")
    frames = cread(out / "frames.csv")
    targets = [r for r in regions if r["region_role"] == "UAV_CONDITIONED"]
    probabilities, summaries, correlations = [], [], []
    for grouping in [["sequence_id", "alignment_scenario"], ["nominal_range_m", "alignment_scenario"],
                     ["range_bin_m", "alignment_scenario"], ["eta_bin", "alignment_scenario"],
                     ["sequence_id", "gnss_segment_id", "alignment_scenario"]]:
        groups = defaultdict(list)
        for r in targets:
            groups[tuple(r.get(k) for k in grouping)].append(r)
        for key, rs in groups.items():
            label = {**dict(zip(grouping, key)), "grouping": "+".join(grouping), "frames": len(rs),
                     "probability_status": "empirical region occupancy, NOT UAV detection probability"}
            counts = np.asarray([r["raw_points"] for r in rs])
            probabilities.append({**label, **{f"n_{s}": int(m.sum()) for s, m in
                [("0", counts == 0), ("1", counts == 1), ("2", counts == 2), ("gt2", counts > 2)]},
                **{f"p_{s}": float(m.mean()) for s, m in [("0", counts == 0), ("1", counts == 1), ("2", counts == 2), ("gt2", counts > 2)]}})
            for feature in FEATURES + ["range_m", "eta", "speed_mps", "range_rate_mps"]:
                vals = finite_values(rs, feature)
                summaries.append({**label, "feature": feature, "n_valid": len(vals),
                                  **stats.numeric_descriptors(vals, "value")})
    effects = []
    for grouping in [["sequence_id"], ["nominal_range_m"], ["range_bin_m"], ["eta_bin"]]:
        effects.extend(stats.matched_effects(regions, FEATURES, grouping + ["alignment_scenario"]))
    # Within-sequence associations avoid substituting nominal group differences
    # for a controlled range law. Correlations remain exploratory and confounded.
    for sid in SEQUENCES:
        for definition in sorted(set(r["alignment_scenario"] for r in targets)):
            rs = [r for r in targets if r["sequence_id"] == sid and r["alignment_scenario"] == definition]
            for covariate in ["range_m", "region_reference_range_m", "eta", "motion_los_angle_deg"]:
                for feature in FEATURES:
                    pairs = [(r.get(covariate), r.get(feature)) for r in rs]
                    pairs = [(a, b) for a, b in pairs if isinstance(a, (float, int)) and isinstance(b, (float, int)) and np.isfinite(a) and np.isfinite(b)]
                    rho = np.nan
                    if len(pairs) >= 3:
                        a, b = np.asarray(pairs).T
                        if np.ptp(a) > 0 and np.ptp(b) > 0:
                            rho = float(spearmanr(a, b).statistic)
                    correlations.append({"sequence_id": sid, "alignment_scenario": definition, "covariate": covariate,
                                         "feature": feature, "n_pairs": len(pairs), "spearman_rho": rho,
                                         "status": "descriptive; temporal dependence and interpolation confounding; no p-value"})
    # Include supported missing observations as gaps of prior localization only.
    gaps = []
    for sid in SEQUENCES:
        rs = [r for r in frames if r["sequence_id"] == sid]
        for r in rs:
            gaps.append({**r, "alignment_scenario": "SAVED_LOCALIZATION_AVAILABILITY", "region_id": "SAVED_LOCALIZATION_AVAILABILITY",
                         "closure_supported": bool(r.get("anchor_available", False)),
                         "raw_points": 1 if r.get("anchor_available", False) else 0})
    runs = stats.zero_run_records(targets + gaps)
    cwrite(out / "support_frequencies.csv", probabilities)
    cwrite(out / "descriptive_summaries.csv", summaries)
    cwrite(out / "matched_exploratory_effects.csv", effects)
    cwrite(out / "within_sequence_associations.csv", correlations)
    cwrite(out / "region_empty_runs.csv", runs)
    checks = {"frame_count": len(frames), "GNSS_supported": sum(bool(r.get("gnss_supported")) for r in frames),
              "saved_anchor_frames": sum(bool(r.get("anchor_available")) for r in frames),
              "region_count": len(regions), "controls": sum(r["region_role"] == "LOCAL_CLUTTER_PROXY" for r in regions),
              "all_protected_sources_and_code_unchanged": True, "no_detector_tracker_or_calibration_fit_invoked": True,
              "negative_labels_created": 0, "labels_changed": False, "score_threshold_selected": False,
              "finished_utc": utc()}
    jwrite(out / "validation_checks.json", checks)
    print(json.dumps(checks), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["freeze", "extract", "summarize", "verify"])
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    if args.phase == "freeze":
        freeze(args.out)
    elif args.phase == "extract":
        extract(args.out)
    elif args.phase == "summarize":
        summarize(args.out)
    else:
        verify(args.out)
        print("Frozen source/code identity unchanged")


if __name__ == "__main__":
    main()
