#!/usr/bin/env python3
"""Standalone DEV/SILVER candidate-stage attribution; no baseline writes.

Only epsilon and DBSCAN min_samples change. Published geometric nmin=2 is
intentionally retained. Silver neighborhoods are used AFTER full-scene DBSCAN.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import scipy
import sklearn
from sklearn.cluster import DBSCAN

import published_baseline_dp_mvp as published
import sparse_uav_mvp as sparse

ROOT = published.ROOT
OUT = ROOT / "results/dbscan_fragmentation_attribution"
AUDIT = ROOT / "results/range_sparsity_audit"
SELF = Path(__file__).resolve()
BASE = published.PROTOCOL["published_structure_baseline"]
CFG = BASE["project_adapted"]
VARIANTS = [(schedule, minpts) for schedule in ("fixed", "adaptive") for minpts in (1, 2)]
PROTOCOL = {
    "version": 1, "status": "FROZEN_READ_ONLY_DEV_SILVER_DIAGNOSTIC_NOT_METHOD_SELECTION",
    "conditions": list(published.CONDITIONS), "randomness": "NONE",
    "variants": [{"schedule": s, "dbscan_min_samples": m} for s, m in VARIANTS],
    "fixed_eps_m": 0.6,
    "adaptive_eps": "0.6+0.004*max(unweighted_mean_range_of_all_frame_fine_voxels-10,0)",
    "adaptive_scope": "ONE_GLOBAL_SCALAR_PER_FRAME; NOT_PER_POINT_OR_TARGET_RANGE",
    "source_published_structure": BASE,
    "geometry": {k: CFG[k] for k in ("cluster_min_voxels", "cluster_max_voxels",
                                     "cluster_max_raw_points", "cluster_max_axis_extent_m", "cluster_range_m")},
    "geometry_nmin_independent_of_minpts": True,
    "centroid": "unweighted component-wise median for >=3 voxels; arithmetic mean otherwise",
    "input": "existing raw_fine_cache only; never rebuild or alter",
    "stage": "DBSCAN -> geometric validation ONLY; no temporal/jump/tracking/Doppler gates",
    "silver_match_radius_m": 3.0, "neighborhood_radii_m": [1.0, 3.0],
    "connectivity": "induced epsilon graph within silver ball, min_samples=1; empty UNKNOWN; singleton connected",
    "full_scene_components": "also report whole-frame components intersecting ball; bridges may be clutter",
    "large_component": "union(voxels>32, raw_support>120, max_axis_extent>=6m); counts overlap individually",
    "recall_denominator": "80 observed frozen labels only; no interpolation",
    "volume_denominator": "all 272 LiDAR frames; not false alarms/precision",
    "representation": [
        "exact raw-neighborhood 4cm/1m grid occupancy on identical point-level filtered proxy support",
        "actual whole-frame 4cm and existing postfilter 1m cache ball counts (stage-order confounded)",
        "coarsen same whole-frame fine voxel means to 1m; no clustering (4cm means can cross 1m boundaries)",
    ],
    "attribution": "new silver match is proxy fragmentation repair only if adaptive matched full-scene component contains >=2 fixed induced proxy components and induced graph becomes connected",
    "limits": "DEV/SILVER center balls are not point GT; operational precision and independent UAV fragmentation UNKNOWN",
    "no_parameter_selection": True,
}


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"no rows: {path}")
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def hashes(paths):
    return {str(p.relative_to(ROOT)): published.sha256(p) for p in sorted(set(paths))}


def protected_paths():
    previous = json.loads((AUDIT / "protected_files.json").read_text())["before"]
    paths = [ROOT / p for p in previous]
    paths += list(AUDIT.glob("*.csv")) + [AUDIT / "protocol.json", AUDIT / "run_manifest.json"]
    paths += [ROOT / "scripts/range_sparsity_audit.py", published.OUT / "candidate_stage_recall.csv",
              published.CLOSURE / "alignment.json"]
    for cid in published.CONDITIONS:
        for folder in (published.CACHE, sparse.RAW_VOXEL):
            paths += [folder / f"{cid}.npz", folder / f"{cid}.json"]
    return paths


def freeze():
    if (OUT / "protocol.json").exists():
        raise RuntimeError("diagnostic already frozen; do not overwrite")
    OUT.mkdir(parents=True, exist_ok=False)
    source_protocol = json.loads((published.OUT / "protocol.json").read_text())
    assert source_protocol["published_structure_baseline"] == BASE
    for cid in published.CONDITIONS:
        meta = json.loads((published.CACHE / f"{cid}.json").read_text())
        assert meta["voxel_m"] == 0.04
        assert meta["npz_sha256"] == published.sha256(published.CACHE / f"{cid}.npz")
    write_json(OUT / "protocol.json", PROTOCOL)
    write_json(OUT / "input_identity.json", hashes(protected_paths()))
    write_json(OUT / "freeze_identity.json", {
        "utc": datetime.now(timezone.utc).isoformat(), "script_sha256": published.sha256(SELF),
        "protocol_sha256": published.sha256(OUT / "protocol.json"),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
    })
    (OUT / "FROZEN").write_text("No selection, no baseline/cache/label writes.\n")
    print(f"frozen {OUT}", flush=True)


def eps_for(values, schedule):
    if schedule == "fixed" or not len(values):
        return 0.6
    return 0.6 + CFG["dbscan_alpha"] * max(float(np.mean(np.linalg.norm(values[:, :3], axis=1))) - 10.0, 0.0)


def cluster_frame(values, eps, minpts):
    labels = DBSCAN(eps=eps, min_samples=minpts, algorithm="kd_tree").fit_predict(values[:, :3]) if len(values) else np.zeros(0, dtype=int)
    clusters = {}
    for lab in np.unique(labels):
        if lab < 0:
            continue
        c = values[labels == lab]
        n, raw = len(c), float(c[:, 3].sum())
        extent = float(np.max(np.ptp(c[:, :3], axis=0)))
        center = np.median(c[:, :3], axis=0) if n >= 3 else np.mean(c[:, :3], axis=0)
        distance = float(np.linalg.norm(center))
        flags = {
            "too_small": n < CFG["cluster_min_voxels"],
            "too_many_voxels": n > CFG["cluster_max_voxels"],
            "too_much_raw_support": raw > CFG["cluster_max_raw_points"],
            "too_large_extent": extent >= CFG["cluster_max_axis_extent_m"],
            "out_of_range": not (CFG["cluster_range_m"][0] <= distance <= CFG["cluster_range_m"][1]),
        }
        clusters[int(lab)] = {
            "voxels": n, "raw_support": raw, "extent_m": extent, "center": center,
            "range_m": distance, "accepted": not any(flags.values()),
            "large": any(flags[k] for k in ("too_many_voxels", "too_much_raw_support", "too_large_extent")),
            **flags,
        }
    return labels, clusters


def induced_components(xyz, eps):
    if not len(xyz):
        return np.zeros(0, dtype=int)
    return DBSCAN(eps=eps, min_samples=1, algorithm="kd_tree").fit_predict(xyz)


def neighborhood(values, labels, clusters, gt, radius, eps):
    mask = np.linalg.norm(values[:, :3] - gt, axis=1) <= radius
    induced = induced_components(values[mask, :3], eps)
    ids, counts = np.unique(induced, return_counts=True)
    full = sorted(set(int(x) for x in labels[mask] if x >= 0))
    matches = [lab for lab, c in clusters.items() if c["accepted"] and np.linalg.norm(c["center"] - gt) <= 3.0]
    reasons = Counter(k for lab in full for k in ("too_small", "too_many_voxels", "too_much_raw_support", "too_large_extent", "out_of_range") if clusters[lab][k])
    row = {
        "radius_m": radius, "proxy_voxels": int(mask.sum()), "proxy_raw_support": float(values[mask, 3].sum()),
        "induced_components": len(ids), "induced_connected": int(len(ids) == 1) if len(ids) else "UNKNOWN",
        "induced_singleton_components": int(np.sum(counts == 1)),
        "largest_induced_fraction": float(max(counts) / len(induced)) if len(induced) else "UNKNOWN",
        "full_scene_components_touching_proxy": len(full), "noise_proxy_voxels": int(np.sum(labels[mask] < 0)),
        "accepted_components_touching_proxy": sum(clusters[x]["accepted"] for x in full),
        "large_components_touching_proxy": sum(clusters[x]["large"] for x in full),
        "silver_hit_3m_centroid": int(bool(matches)),
        "matched_component_ids": ";".join(map(str, matches)),
        "proxy_rejection_flags": json.dumps(dict(reasons), sort_keys=True),
        "matched_component_max_outside_proxy_voxels": max((clusters[x]["voxels"] - np.sum(labels[mask] == x) for x in matches), default=0),
    }
    return row, mask, induced, matches


def coarsen_fine(values):
    if not len(values):
        return np.zeros((0, 6), dtype=np.float64)
    _, inv = np.unique(np.floor(values[:, :3] / 1.0).astype(np.int64), axis=0, return_inverse=True)
    support = np.bincount(inv, weights=values[:, 3])
    result = np.column_stack([np.bincount(inv, weights=values[:, j] * values[:, 3]) / support
                              for j in (0, 1, 2)])
    return np.column_stack([result, support,
                            np.bincount(inv, weights=values[:, 4] * values[:, 3]) / support,
                            np.bincount(inv, weights=values[:, 5] * values[:, 3]) / support])


def filtered_raw_proxy(points):
    xyz = np.asarray([[float(p[f"{a}_m"]) for a in "xyz"] for p in points], dtype=np.float32).reshape(-1, 3)
    intensity = np.asarray([float(p["intensity"]) for p in points], dtype=np.float32)
    velocity = np.asarray([float(p["measured_doppler_mps"]) for p in points], dtype=np.float32)
    roi, (lo, hi) = CFG["broad_roi_m"], CFG["intensity_range_inclusive"]
    rotation, translation = published._alignment()
    keep = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity) & np.isfinite(velocity)
    for j, axis in enumerate("xyz"):
        keep &= (xyz[:, j] >= roi[axis][0]) & (xyz[:, j] <= roi[axis][1])
    keep &= (intensity >= lo) & (intensity <= hi)
    keep &= (rotation.T @ (xyz - translation).T)[2] > CFG["leveled_height_gt_m"]
    return xyz[keep]


def representation_rows(cid, frame_labels, fine_frames, coarse_frames, points_by_frame, audit_by_key):
    rows = []
    for label in frame_labels:
        fi = int(label["frame_index"])
        gt = np.asarray([float(label[f"center_{a}_m"]) for a in "xyz"])
        fine, old, coarse = fine_frames[fi], coarse_frames[fi], coarsen_fine(fine_frames[fi])
        raw_points = points_by_frame[(cid, fi)]
        for radius in (1.0, 3.0):
            points = [p for p in raw_points if float(p["center_distance_m"]) <= radius]
            raw = filtered_raw_proxy(points)
            source = audit_by_key[(cid, fi, radius)]
            assert len(points) == int(source["raw_points"])
            occupied = {size: len(np.unique(np.floor(raw / size).astype(np.int64), axis=0)) for size in (0.04, 1.0)}
            row = {
                "condition_id": cid, "nominal_range_m": int(label["nominal_range_m"]), "frame_index": fi,
                "bag_time_unix": float(label["bag_time_unix"]), "radius_m": radius,
                "audit_raw_points": len(points), "audit_raw_4cm_cells": int(source["occupied_voxels_0.04m"]),
                "audit_raw_1m_cells": int(source["occupied_voxels_1m"]),
                "same_input_point_filtered_raw_support": len(raw),
                "same_input_point_filtered_4cm_cells": occupied[0.04],
                "same_input_point_filtered_1m_cells": occupied[1.0],
                "pure_cell_collapse_fraction": 1 - occupied[1.0] / occupied[0.04] if occupied[0.04] else "UNKNOWN",
                "same_input_4cm_below_geometric_nmin": int(occupied[0.04] < 2),
                "same_input_1m_below_geometric_nmin": int(occupied[1.0] < 2),
                "same_input_4cm_over_max32": int(occupied[0.04] > 32),
            }
            for prefix, arr in (("actual_fine_cache", fine), ("existing_1m_postfilter", old), ("same_fine_means_coarsened_1m", coarse)):
                mask = np.linalg.norm(arr[:, :3] - gt, axis=1) <= radius
                row[prefix + "_voxels"] = int(mask.sum())
                row[prefix + "_raw_support"] = float(arr[mask, 3].sum())
            rows.append(row)
    return rows


def transition(fixed, adaptive, fixed_labels, adaptive_labels, mask, induced_fixed, induced_adaptive, adaptive_matches):
    gained = not fixed["silver_hit_3m_centroid"] and bool(adaptive["silver_hit_3m_centroid"])
    lost = bool(fixed["silver_hit_3m_centroid"]) and not adaptive["silver_hit_3m_centroid"]
    merged = max((len(set(induced_fixed[adaptive_labels[mask] == lab])) for lab in adaptive_matches), default=0)
    if gained:
        if fixed["induced_components"] > 1 and adaptive["induced_components"] == 1 and merged >= 2:
            attribution = "PROXY_FRAGMENTATION_REPAIR"
        elif fixed["induced_components"] == 1:
            attribution = "ALREADY_CONNECTED_GEOMETRY_OR_CENTROID_CHANGE"
        elif merged >= 2:
            attribution = "FULL_SCENE_BRIDGE_NOT_INDUCED_CONNECTIVITY_REPAIR"
        else:
            attribution = "OTHER_PERMISSIVENESS_OR_CENTROID_CHANGE"
    elif lost:
        attribution = "LOST_MATCH"
    else:
        attribution = "RETAINED_HIT" if fixed["silver_hit_3m_centroid"] else "RETAINED_MISS"
    return {
        "fixed_hit": fixed["silver_hit_3m_centroid"], "adaptive_hit": adaptive["silver_hit_3m_centroid"],
        "gained": int(gained), "lost": int(lost), "attribution": attribution,
        "fixed_induced_components": fixed["induced_components"], "adaptive_induced_components": adaptive["induced_components"],
        "fixed_proxy_voxels": fixed["proxy_voxels"], "max_fixed_proxy_components_in_adaptive_match": merged,
        "fixed_proxy_rejection_flags": fixed["proxy_rejection_flags"], "adaptive_proxy_rejection_flags": adaptive["proxy_rejection_flags"],
        "fixed_proxy_noise_voxels": fixed["noise_proxy_voxels"], "adaptive_proxy_noise_voxels": adaptive["noise_proxy_voxels"],
        "fixed_accepted_components_touching_proxy": fixed["accepted_components_touching_proxy"],
        "adaptive_matched_max_outside_proxy_voxels": adaptive["matched_component_max_outside_proxy_voxels"],
    }


def run():
    if not (OUT / "FROZEN").exists() or (OUT / "run_manifest.json").exists():
        raise RuntimeError("requires frozen, not already completed output directory")
    frozen = json.loads((OUT / "freeze_identity.json").read_text())
    assert published.sha256(SELF) == frozen["script_sha256"]
    assert json.loads((OUT / "protocol.json").read_text()) == PROTOCOL
    before = json.loads((OUT / "input_identity.json").read_text())
    assert hashes(protected_paths()) == before
    started_utc, started = datetime.now(timezone.utc).isoformat(), time.perf_counter()
    all_labels = published.load_labels()
    assert len(all_labels) == 80
    points_by_frame = defaultdict(list)
    for p in read_csv(AUDIT / "target_neighborhood_points.csv"):
        points_by_frame[(p["sequence_id"], int(p["frame_index"]))].append(p)
    audit_by_key = {(r["sequence_id"], int(r["frame_index"]), float(r["target_radius_m"])): r
                    for r in read_csv(AUDIT / "frame_metrics.csv")}
    frames_out, clusters_out, silver_out, transitions_out, reps_out, replay = [], [], [], [], [], []
    original = {r["condition_id"]: r for r in read_csv(published.OUT / "candidate_stage_recall.csv")
                if r["method"] == "Khosravi_reconstruction_pre_temporal"}
    for cid in published.CONDITIONS:
        fine_frames, times = published.load_fine_frames(cid)
        old_frames, old_times = sparse._load_raw_voxel_frames(cid)
        assert np.array_equal(times, old_times)
        truth = [x for x in all_labels if x["condition_id"] == cid]
        truth_by_frame = {int(x["frame_index"]): x for x in truth}
        nominal = int(truth[0]["nominal_range_m"])
        reps_out.extend(representation_rows(cid, truth, fine_frames, old_frames, points_by_frame, audit_by_key))
        for fi, values in enumerate(fine_frames):
            epses = {s: eps_for(values, s) for s in ("fixed", "adaptive")}
            results, diagnostic = {}, {}
            for schedule, minpts in VARIANTS:
                tick = time.perf_counter()
                labels, clusters = cluster_frame(values, epses[schedule], minpts)
                method = f"{schedule}_minPts{minpts}"
                ident = {"method": method, "schedule": schedule, "minpts": minpts,
                         "condition_id": cid, "nominal_range_m": nominal, "frame_index": fi, "bag_time_unix": float(times[fi])}
                for lab, c in clusters.items():
                    clusters_out.append({**ident, "component_id": lab, "eps_m": epses[schedule],
                                         **{k: v for k, v in c.items() if k != "center"},
                                         **{f"center_{a}_m": float(c["center"][j]) for j, a in enumerate("xyz")}})
                row = {**ident, "eps_m": epses[schedule], "input_voxels": len(values),
                       "noise_voxels": int(np.sum(labels < 0)), "dbscan_components": len(clusters),
                       "pregeometry_singletons": sum(c["voxels"] == 1 for c in clusters.values()),
                       "accepted_candidates": sum(c["accepted"] for c in clusters.values()),
                       "large_components": sum(c["large"] for c in clusters.values()),
                       "geometry_min2_rejections": sum(c["too_small"] for c in clusters.values()),
                       "geometry_max32_rejections": sum(c["too_many_voxels"] for c in clusters.values()),
                       "geometry_raw120_rejections": sum(c["too_much_raw_support"] for c in clusters.values()),
                       "geometry_extent6_rejections": sum(c["too_large_extent"] for c in clusters.values()),
                       "silver_labeled": int(fi in truth_by_frame), "silver_hit": "",
                       "elapsed_s": time.perf_counter() - tick}
                if fi in truth_by_frame:
                    gt = np.asarray([float(truth_by_frame[fi][f"center_{a}_m"]) for a in "xyz"])
                    for radius in (1.0, 3.0):
                        nr, mask, induced, matches = neighborhood(values, labels, clusters, gt, radius, epses[schedule])
                        silver_out.append({**ident, "eps_m": epses[schedule], **nr})
                        diagnostic[(schedule, minpts, radius)] = (nr, mask, induced, matches)
                    row["silver_hit"] = diagnostic[(schedule, minpts, 3.0)][0]["silver_hit_3m_centroid"]
                frames_out.append(row)
                results[(schedule, minpts)] = (labels, clusters)
            # minPts=1 vs 2 non-singleton sets must be identical, despite component relabeling.
            for schedule in ("fixed", "adaptive"):
                a = [np.flatnonzero(results[(schedule, 1)][0] == lab).tolist() for lab, c in results[(schedule, 1)][1].items() if c["accepted"]]
                b = [np.flatnonzero(results[(schedule, 2)][0] == lab).tolist() for lab, c in results[(schedule, 2)][1].items() if c["accepted"]]
                assert sorted(a) == sorted(b)
            if fi in truth_by_frame:
                for minpts in (1, 2):
                    for radius in (1.0, 3.0):
                        fr, mask, icf, _ = diagnostic[("fixed", minpts, radius)]
                        ar, _, ica, matches = diagnostic[("adaptive", minpts, radius)]
                        transitions_out.append({"condition_id": cid, "nominal_range_m": nominal, "frame_index": fi,
                                                "radius_m": radius, "minpts": minpts,
                                                **transition(fr, ar, results[("fixed", minpts)][0], results[("adaptive", minpts)][0], mask, icf, ica, matches)})
            if fi == 0 or (fi + 1) % 20 == 0:
                print(f"{cid} frame {fi+1}/{len(fine_frames)}, total elapsed {time.perf_counter()-started:.1f}s", flush=True)
        rr = [x for x in frames_out if x["condition_id"] == cid and x["method"] == "adaptive_minPts2"]
        got = (sum(int(x["silver_hit"]) for x in rr if x["silver_labeled"]), sum(x["accepted_candidates"] for x in rr))
        expected = (int(original[cid]["detected_frames"]), int(original[cid]["candidate_count"]))
        assert got == expected, (cid, got, expected)
        replay.append({"condition_id": cid, "detected_frames": got[0], "candidate_count": got[1], "matches_recorded_E13": True})
    summaries = []
    for schedule, minpts in VARIANTS:
        for group in (100, 200, 300, "ALL"):
            rr = [r for r in frames_out if r["schedule"] == schedule and r["minpts"] == minpts and (group == "ALL" or r["nominal_range_m"] == group)]
            sr = [r for r in silver_out if r["schedule"] == schedule and r["minpts"] == minpts and r["radius_m"] == 3.0 and (group == "ALL" or r["nominal_range_m"] == group)]
            tr = [r for r in transitions_out if r["minpts"] == minpts and r["radius_m"] == 3.0 and (group == "ALL" or r["nominal_range_m"] == group)]
            counts = sum(r["accepted_candidates"] for r in rr)
            summaries.append({
                "schedule": schedule, "minpts": minpts, "nominal_range_m": group,
                "lidar_frames": len(rr), "silver_frames": len(sr), "silver_hits": sum(r["silver_hit_3m_centroid"] for r in sr),
                "candidate_stage_silver_recall": sum(r["silver_hit_3m_centroid"] for r in sr)/len(sr),
                "candidate_count": counts, "candidates_per_frame": counts/len(rr),
                "pregeometry_components": sum(r["dbscan_components"] for r in rr),
                "pregeometry_singletons": sum(r["pregeometry_singletons"] for r in rr),
                "noise_voxels": sum(r["noise_voxels"] for r in rr),
                "large_components": sum(r["large_components"] for r in rr),
                "large_components_per_frame": sum(r["large_components"] for r in rr)/len(rr),
                "proxy_induced_connected_frames": sum(r["induced_connected"] == 1 for r in sr),
                "adaptive_gained_frames": sum(r["gained"] for r in tr), "adaptive_lost_frames": sum(r["lost"] for r in tr),
                "adaptive_proxy_fragmentation_repair_gains": sum(r["attribution"] == "PROXY_FRAGMENTATION_REPAIR" for r in tr),
                "eps_m_median": float(np.median([r["eps_m"] for r in rr])),
                "elapsed_s": sum(r["elapsed_s"] for r in rr), "label_status": "DEV_SILVER_NOT_POINT_GT",
            })
    for name, rows in (("frame_metrics", frames_out), ("cluster_metrics", clusters_out), ("silver_components", silver_out),
                       ("adaptive_transitions", transitions_out), ("representation_support", reps_out), ("summary_by_range", summaries), ("replay_validation", replay)):
        write_csv(OUT / f"{name}.csv", rows)
    after = hashes(protected_paths())
    assert after == before, "protected source/cache/labels/artifacts changed"
    write_json(OUT / "verification.json", {"protected_files": len(before), "all_unchanged": True,
               "minpts_accepted_component_equivalence_all_frames": True, "E13_replay_all_five_exact": True,
               "label_rows": len(all_labels), "frame_rows": len(frames_out), "component_rows": len(clusters_out),
               "silver_rows": len(silver_out), "transition_rows": len(transitions_out), "representation_rows": len(reps_out)})
    write_json(OUT / "run_manifest.json", {
        "utc_start": started_utc, "utc_end": datetime.now(timezone.utc).isoformat(), "elapsed_s": time.perf_counter()-started,
        "freeze": frozen, "environment": {"python": platform.python_version(), "numpy": np.__version__,
            "scipy": scipy.__version__, "sklearn": sklearn.__version__, "platform": platform.platform()},
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/dbscan_fragmentation_attribution.py run",
        "input_identity_file": "input_identity.json", "output_sha256": hashes(list(OUT.glob("*.csv"))),
        "protected_unchanged": True,
    })
    print(json.dumps(summaries, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("freeze", "run"))
    args = parser.parse_args()
    freeze() if args.phase == "freeze" else run()
