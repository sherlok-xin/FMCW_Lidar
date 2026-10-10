#!/usr/bin/env python3
"""Read-only E15/E16 geometry audit. No detector, threshold search or tuning.

Reuses raw proxy descriptors and fixed silver hit flags; reconstructs cell means
only from identical filtered raw proxy children. Saved full-scene component
extent is kept separate from raw-ball geometry. No clustering is executed.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import platform
import subprocess
from pathlib import Path
import time

import numpy as np
import scipy

import dbscan_fragmentation_attribution as source
import range_sparsity_audit as raw_audit

ROOT = source.ROOT
OUT = ROOT / "results/geometry_invariance_audit"
ATTR = source.OUT
RAW = source.AUDIT
SELF = Path(__file__).resolve()
RADII = (1.0, 3.0)
RAW_FIELDS = {
    "raw_point_count": "raw_points", "raw_cells_4cm": "occupied_voxels_0.04m", "raw_cells_1m": "occupied_voxels_1m",
    "raw_unique_xyz_count": "unique_xyz_points", "raw_coincident_returns": "coincident_return_count",
    **{f"raw_{k}": k for k in ("span_x_m", "span_y_m", "span_z_m", "diameter_m", "range_thickness_m",
       "nn_m_median", "unique_nn_m_median", "mst_connectivity_edge_max_m", "pairwise_m_median",
       "intensity_q25", "intensity_median", "intensity_q75")},
}
REP_FIELDS = ("same_input_point_filtered_raw_support", "same_input_point_filtered_4cm_cells",
              "same_input_point_filtered_1m_cells", "actual_fine_cache_voxels", "actual_fine_cache_raw_support",
              "existing_1m_postfilter_voxels", "existing_1m_postfilter_raw_support", "pure_cell_collapse_fraction")
SHAPE_KEYS = ("span_x_m", "span_y_m", "span_z_m", "max_axis_span_m", "diameter_m", "range_thickness_m",
              "nn_m_median", "unique_nn_m_median", "mst_connectivity_edge_max_m")
METRICS = list(RAW_FIELDS) + ["raw_max_axis_span_m"] + list(REP_FIELDS)
METRICS += [f"{stage}_{k}" for stage in ("filtered_raw_proxy", "fine_proxy_means", "coarse_proxy_means") for k in SHAPE_KEYS]
PROTOCOL = {
    "version": 1, "status": "FROZEN_DESCRIPTIVE_DEV_SILVER_NOT_THRESHOLD_SELECTION",
    "observations": "all 80 existing observed research_dev_v0 silver labels; no other trajectories",
    "primary_radius_m": 3.0, "sensitivity_radius_m": 1.0,
    "source_geometry": "E15 raw finite/nonzero xyz center balls; not UAV point identity GT",
    "coordinates": "sensor-relative [x,z,-y]; do not refit or rotate geometry",
    "hit_status": "read E16 candidate-stage silver hit; fixed/adaptive/minPts1/2 agree; not target-vs-clutter labels",
    "metrics": METRICS,
    "summary": "frame-level median/Q25/Q75/min/max/valid denominator; nominal group, sequence, hit/miss and nominal-by-hit",
    "range_stability": "max/min of positive 100/200/300 group medians only; descriptive, not a score or feature ranking",
    "range_association": "descriptive Spearman rank correlation overall and within five conditions; no p-values",
    "raw_support": "counts of raw returns summed into voxels; not another independent evidence source",
    "cell_mean_geometry": "identical E16 point-filtered raw proxy children -> fixed zero-origin floor(xyz/.04 or /1) arithmetic means; no detector inputs changed",
    "cell_mean_limit": "isolated proxy reconstruction is not whole-scene fine-component geometry or exact old-cache lineage",
    "component_extent": "saved E16 minPts1 fixed/adaptive full-scene component nearest silver center; chosen for descriptor lookup only, not relabeling",
    "component_selection_limit": "nearest-center saved component is a retrospective diagnostic proxy, not certified UAV point membership",
    "component_hit_validation": "reproduce recorded frame hit by existence of any accepted saved component within existing 3m gate; nearest component can be rejected in a hit frame",
    "existing_rules_only": "2..32 voxel, raw<=120, max-axis<6m are READ as existing rejection reasons; no new rejection rule",
    "degenerate_geometry": "singleton observed extent/diameter=0 is a sampled-support lower bound; NN/MST UNKNOWN",
    "inference": "no negative GT; no operational discrimination, no chosen threshold, no sweep, no algorithm/baseline run",
    "randomness": "NONE",
}


def f(value):
    return float(value) if value not in (None, "", "UNKNOWN") else float("nan")


def write_csv(name, rows):
    cleaned = [{k: ("" if isinstance(v, (float, np.floating)) and not np.isfinite(v) else v)
                for k, v in row.items()} for row in rows]
    source.write_csv(OUT / f"{name}.csv", cleaned)


def protected():
    paths = source.protected_paths() + list(ATTR.glob("*.csv")) + list(ATTR.glob("*.json")) + [ATTR / "FROZEN"]
    paths += [ROOT / p for p in ("FMCW_Range_Sparsity_Audit.md", "FMCW_DBSCAN_Fragmentation_Attribution.md",
               "scripts/dbscan_fragmentation_attribution.py", "scripts/summarize_dbscan_fragmentation.py",
               "tests/test_dbscan_fragmentation_attribution.py")]
    return source.hashes(paths)


def freeze():
    if OUT.exists():
        raise RuntimeError("new audit output already exists; do not overwrite")
    OUT.mkdir(parents=True)
    source.write_json(OUT / "protocol.json", PROTOCOL)
    source.write_json(OUT / "input_identity.json", protected())
    source.write_json(OUT / "freeze_identity.json", {
        "utc": datetime.now(timezone.utc).isoformat(), "script_sha256": source.published.sha256(SELF),
        "protocol_sha256": source.published.sha256(OUT / "protocol.json"),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True)})
    (OUT / "FROZEN").write_text("Descriptive only; no new thresholds or algorithm changes.\n")
    print("geometry definitions frozen", flush=True)


def cell_means(xyz, size):
    if not len(xyz):
        return np.empty((0, 3)), np.zeros(0, dtype=int)
    _, inv, counts = np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
    means = np.column_stack([np.bincount(inv, weights=xyz[:, j]) / counts for j in range(3)])
    return means, counts


def describe(xyz):
    result = raw_audit.spacing(np.asarray(xyz, dtype=float))
    spans = [result[f"span_{a}_m"] for a in "xyz"]
    result["max_axis_span_m"] = max(spans) if len(xyz) else float("nan")
    result["range_thickness_m"] = float(np.ptp(np.linalg.norm(xyz, axis=1))) if len(xyz) else float("nan")
    return {k: result[k] for k in SHAPE_KEYS}


def summarize(values):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    row = {"n_defined": len(v), "status": "DESCRIPTIVE" if len(v) else "UNKNOWN_no_defined_values"}
    row.update({k: float(x) for k, x in zip(("min", "q25", "median", "q75", "max"),
                    np.quantile(v, [0, .25, .5, .75, 1]) if len(v) else [np.nan]*5)})
    return row


def lookup_saved_components(rows, center):
    """Diagnostic lookup only; nearest component need not explain frame hit."""
    distances = [float(np.linalg.norm(np.array([float(r[f"center_{a}_m"]) for a in "xyz"]) - center)) for r in rows]
    index = int(np.argmin(distances))
    accepted_within_gate = sum(r["accepted"] == "True" and d <= 3.0 for r, d in zip(rows, distances))
    return rows[index], distances[index], accepted_within_gate


def grouped(rows):
    yield "all", "ALL", rows
    for nominal in (100, 200, 300):
        rr = [r for r in rows if r["nominal_range_m"] == nominal]
        yield "nominal_range", str(nominal), rr
        for status in ("HIT", "MISS"):
            yield "nominal_by_hit", f"{nominal}_{status}", [r for r in rr if r["hit_status"] == status]
    for status in ("HIT", "MISS"):
        yield "hit_status", status, [r for r in rows if r["hit_status"] == status]
    for cid in source.published.CONDITIONS:
        yield "condition", cid, [r for r in rows if r["condition_id"] == cid]


def run():
    if not (OUT / "FROZEN").exists() or (OUT / "run_manifest.json").exists():
        raise RuntimeError("requires frozen, not already completed output")
    identity = json.loads((OUT / "freeze_identity.json").read_text())
    assert identity["script_sha256"] == source.published.sha256(SELF)
    assert json.loads((OUT / "protocol.json").read_text()) == PROTOCOL
    before = json.loads((OUT / "input_identity.json").read_text())
    assert protected() == before
    tick, utc = time.perf_counter(), datetime.now(timezone.utc).isoformat()
    labels = source.published.load_labels()
    assert len(labels) == 80
    raw = {(r["sequence_id"], int(r["frame_index"]), float(r["target_radius_m"])): r
           for r in source.read_csv(RAW / "frame_metrics.csv")}
    rep = {(r["condition_id"], int(r["frame_index"]), float(r["radius_m"])): r
           for r in source.read_csv(ATTR / "representation_support.csv")}
    hits = defaultdict(dict)
    for r in source.read_csv(ATTR / "frame_metrics.csv"):
        if r["silver_labeled"] == "1":
            hits[(r["condition_id"], int(r["frame_index"]))][r["method"]] = int(r["silver_hit"])
    point_rows = defaultdict(list)
    for r in source.read_csv(RAW / "target_neighborhood_points.csv"):
        point_rows[(r["sequence_id"], int(r["frame_index"]))].append(r)
    component_rows = defaultdict(list)
    for r in source.read_csv(ATTR / "cluster_metrics.csv"):
        if r["method"] in ("fixed_minPts1", "adaptive_minPts1"):
            component_rows[(r["condition_id"], int(r["frame_index"]), r["schedule"])].append(r)
    observations, components, validation = [], [], []
    for label in labels:
        cid, fi = label["condition_id"], int(label["frame_index"])
        values = hits[(cid, fi)]
        assert len(values) == 4 and len(set(values.values())) == 1
        hit = next(iter(values.values()))
        gt = np.array([float(label[f"center_{a}_m"]) for a in "xyz"])
        meta = {"condition_id": cid, "nominal_range_m": int(label["nominal_range_m"]), "frame_index": fi,
                "bag_time_unix": float(label["bag_time_unix"]), "actual_range_m": float(np.linalg.norm(gt)),
                "hit_status": "HIT" if hit else "MISS", "label_status": "DEV_SILVER_NOT_POINT_GT"}
        for radius in RADII:
            original, counts = raw[(cid, fi, radius)], rep[(cid, fi, radius)]
            np.testing.assert_array_equal(gt, [float(original[f"center_{a}_m"]) for a in "xyz"])
            assert float(original["time_s"]) == meta["bag_time_unix"]
            row = {**meta, "radius_m": radius, **{k: f(original[v]) for k, v in RAW_FIELDS.items()},
                   **{k: f(counts[k]) for k in REP_FIELDS}}
            row["raw_max_axis_span_m"] = max(row[f"raw_span_{a}_m"] for a in "xyz")
            pts = [p for p in point_rows[(cid, fi)] if float(p["center_distance_m"]) <= radius]
            assert len(pts) == row["raw_point_count"]
            raw_xyz = np.array([[float(p[f"{a}_m"]) for a in "xyz"] for p in pts]).reshape(-1, 3)
            recomputed = describe(raw_xyz)
            for k in SHAPE_KEYS:
                expected = row[f"raw_{k}"]
                assert (not np.isfinite(expected) and not np.isfinite(recomputed[k])) or np.isclose(expected, recomputed[k], rtol=0, atol=1e-12), (cid, fi, radius, k)
            filtered = source.filtered_raw_proxy(pts)
            assert len(filtered) == row["same_input_point_filtered_raw_support"]
            fine, fine_support = cell_means(filtered, .04)
            coarse, coarse_support = cell_means(filtered, 1.)
            assert len(fine) == row["same_input_point_filtered_4cm_cells"]
            assert len(coarse) == row["same_input_point_filtered_1m_cells"]
            assert fine_support.sum() == coarse_support.sum() == len(filtered)
            for name, xyz in (("filtered_raw_proxy", filtered), ("fine_proxy_means", fine), ("coarse_proxy_means", coarse)):
                row.update({f"{name}_{k}": v for k, v in describe(xyz).items()})
            row["raw_geometry_observability"] = "DEGENERATE_SINGLETON_NOT_BODY_SIZE" if len(pts) == 1 else "SAMPLED_SUPPORT_NOT_BODY_SIZE"
            observations.append(row)
        # Look up existing whole-scene descriptor only; do NOT reconstruct or run DBSCAN.
        for schedule in ("fixed", "adaptive"):
            rr = component_rows[(cid, fi, schedule)]
            nearest, distance, accepted_count = lookup_saved_components(rr, gt)
            assert distance <= 3, (cid, fi, schedule, "component selection unobservable")
            # Replay the old existence-based hit, not the nearest component's status.
            assert (accepted_count > 0) == bool(hit)
            components.append({**meta, "schedule": schedule, "component_id": int(nearest["component_id"]),
                 "lookup": "NEAREST_SAVED_E16_COMPONENT_NOT_POINT_MEMBERSHIP_GT", "center_distance_m": distance,
                 "accepted_components_within_existing_3m_gate": accepted_count,
                 "nearest_component_agrees_with_frame_hit": (nearest["accepted"] == "True") == bool(hit),
                 "voxels": int(nearest["voxels"]), "raw_support": float(nearest["raw_support"]),
                 "max_axis_extent_m": float(nearest["extent_m"]), "accepted": nearest["accepted"],
                 **{k: nearest[k] for k in ("too_small", "too_many_voxels", "too_much_raw_support", "too_large_extent", "out_of_range")}})
        validation.append({**meta, "four_hit_flags_agree": True, "raw_descriptors_reproduced_both_radii": True,
                           "cell_counts_and_support_match_E16": True, "time_center_match_frozen_labels": True})
    summary, stability, trends = [], [], []
    for radius in RADII:
        rr = [r for r in observations if r["radius_m"] == radius]
        for kind, name, group in grouped(rr):
            for feature in METRICS:
                summary.append({"radius_m": radius, "group_kind": kind, "group": name, "n_observations": len(group),
                                "feature": feature, **summarize([r[feature] for r in group])})
        for feature in METRICS:
            med = {g: float(np.median([r[feature] for r in rr if r["nominal_range_m"] == g and np.isfinite(r[feature])]))
                   for g in (100, 200, 300) if any(np.isfinite(r[feature]) and r["nominal_range_m"] == g for r in rr)}
            ratio = max(med.values()) / min(med.values()) if len(med) == 3 and min(med.values()) > 0 else float("nan")
            stability.append({"radius_m": radius, "feature": feature, **{f"median_{g}": med.get(g, np.nan) for g in (100, 200, 300)},
                              "max_min_positive_group_median_ratio": ratio,
                              "ratio_status": "DESCRIPTIVE_NOT_FEATURE_SELECTION" if np.isfinite(ratio) else "UNKNOWN_zero_or_missing_group_median"})
        for group in ("ALL", *source.published.CONDITIONS):
            gr = rr if group == "ALL" else [r for r in rr if r["condition_id"] == group]
            for feature in METRICS:
                rho, n = raw_audit.rho([r["actual_range_m"] for r in gr], [r[feature] for r in gr])
                trends.append({"radius_m": radius, "condition_id": group, "feature": feature, "n_defined": n,
                               "spearman_rho": rho, "inference": "DESCRIPTIVE_NO_PVALUE_NO_CAUSAL_LAW"})
    primary = [r for r in observations if r["radius_m"] == 3]
    assert sum(r["hit_status"] == "MISS" for r in primary) == 9
    comp_fixed = {(r["condition_id"], r["frame_index"]): r for r in components if r["schedule"] == "fixed"}
    misses = [{**r, **{f"fixed_component_{k}": v for k, v in comp_fixed[(r["condition_id"], r["frame_index"])].items()
                      if k not in ("condition_id", "frame_index", "label_status", "hit_status")}}
              for r in primary if r["hit_status"] == "MISS"]
    assert sum(r["fixed_component_too_many_voxels"] == "True" for r in misses) == 8
    assert sum(r["fixed_component_too_small"] == "True" for r in misses) == 1
    for name, rows in (("observation_features", observations), ("feature_summary", summary), ("range_stability", stability),
                       ("range_associations", trends), ("existing_component_geometry", components), ("miss_geometry", misses), ("join_validation", validation)):
        write_csv(name, rows)
    assert protected() == before
    source.write_json(OUT / "verification.json", {
        "protected_files": len(before), "all_unchanged": True, "observations": 80, "radius_rows": len(observations),
        "frozen_hit_frames": 71, "frozen_miss_frames": 9, "miss_max32": 8, "miss_min2": 1,
        "raw_feature_reconstruction_tolerance_m": 1e-12, "raw_feature_reconstruction_all_pass": True,
        "recorded_cell_support_reproduced": True, "no_clustering_or_detector_invoked": True,
        "no_new_thresholds_or_sweeps": True, "no_pvalues": True, "negative_label_status": "NONE_RELIABLE",
        "uav_vs_clutter_hard_rejection_validation": "UNKNOWN_no_independent_negative_or_point_GT"})
    source.write_json(OUT / "run_manifest.json", {
        "utc_start": utc, "utc_end": datetime.now(timezone.utc).isoformat(), "elapsed_s": time.perf_counter()-tick,
        "freeze": identity, "environment": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__, "platform": platform.platform()},
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/geometry_invariance_audit.py run",
        "protected_files_unchanged": True, "output_sha256": source.hashes(list(OUT.glob("*.csv")))})
    print(f"audit complete: {len(observations)} radius rows, 80 observations, 9 preserved misses; {len(before)} protected unchanged", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("freeze", "run"))
    args = parser.parse_args()
    freeze() if args.phase == "freeze" else run()
