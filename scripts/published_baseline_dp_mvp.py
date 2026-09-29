#!/usr/bin/env python3
"""Independent published-structure baseline and minimal candidate-graph DP study.

This is a DEV/silver-label experiment.  It does not modify the historical
detector, the project-adapted B baseline, or labels.  The Khosravi baseline is
an explicitly documented reconstruction: the paper gives the processing
stages and some DBSCAN/voxel parameters, but omits several numeric thresholds.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import build_gt_benchmark_v0 as rawbag
import sparse_uav_mvp as sparse


OUT = ROOT / "results" / "published_baseline_dp_mvp"
CACHE = OUT / "raw_fine_cache"
DEV = ROOT / "results" / "research_dev_v0"
PREVIOUS = ROOT / "results" / "temporal_evidence_mvp"
CLOSURE = ROOT / "results" / "calibration_closure"
CONDITIONS = tuple(sparse.SELECTED)
PAPER_URL = "https://arxiv.org/abs/2603.11586"
PAPER_PDF_SHA256 = "68e5e1aba52abd00d085de91c3d55ff31dcfa4d7a26b37a0f5405e760cdd8174"


PROTOCOL = {
    "protocol_version": 1,
    "status": "FROZEN_DEV_SILVER_NOT_PAPER_GT",
    "random_seed": 20260929,
    "labels": "unchanged research_dev_v0/silver_labels.csv",
    "negative_windows": "NONE_RELIABLE",
    "paper": {
        "citation": "Khosravi, Ventura, Basiri, Unsupervised LiDAR-Based Multi-UAV Detection and Tracking Under Extreme Sparsity, 2026",
        "url": PAPER_URL,
        "pdf_sha256": PAPER_PDF_SHA256,
        "scope_warning": "paper experiments use Livox Mid-360 at 5-25 m; this project uses ground-based FMCW LiDAR at nominal 100-300 m",
        "reproduction_status": "STRUCTURE_FAITHFUL_RECONSTRUCTION_NOT_EXACT_REPRODUCTION",
    },
    "published_structure_baseline": {
        "stage_order": [
            "ROI and VoxelGrid",
            "range-adaptive DBSCAN",
            "geometric validation",
            "spatial-jump validation",
            "M-of-K temporal consistency",
        ],
        "paper_reported": {
            "source_configuration": "Table I Config C for eps0/minPts/voxel; rref from Eq. 1 text",
            "voxel_m": 0.04,
            "dbscan_eps0_m": 0.60,
            "dbscan_min_pts": 2,
            "dbscan_reference_range_m": 10.0,
            "eps_equation": "eps(r)=eps0+alpha*max(r-rref,0)",
            "centroid": "component-wise median for cluster size >=3; arithmetic mean below 3",
        },
        "paper_not_numerically_specified": [
            "alpha", "hmin", "rmax", "rexcl", "nmin", "nmax", "emax",
            "tau_min", "vmax", "M", "K", "d_cons", "T_cons",
        ],
        "paper_configuration_limit": (
            "Table I has range-adaptive alpha>0 only for C/D and Layer 3 only for S1/S4/MR; "
            "no reported row combines both.  The required full chain is therefore a reconstruction."
        ),
        "project_adapted": {
            "coordinate_transform_rich80": "[x,z,-y]",
            "intensity_range_inclusive": [10.0, 250.0],
            "broad_roi_m": {"x": [8.0, 510.0], "y": [-200.0, 200.0], "z": [-60.0, 60.0]},
            "leveled_height_gt_m": 10.0,
            "level_transform": "R_full_6dof.T@(p_lidar-t_full_6dof)",
            "dbscan_alpha": 0.004,
            "dbscan_eps_cap": "NONE; follows the published equation",
            "cluster_min_voxels": 2,
            "cluster_max_voxels": 32,
            "cluster_max_raw_points": 120,
            "cluster_max_axis_extent_m": 6.0,
            "cluster_range_m": [8.0, 510.0],
            "jump_tau_min_m": 3.0,
            "jump_vmax_mps": 25.0,
            "temporal_M": 2,
            "temporal_K_prior_observations": 3,
            "temporal_d_cons_m": 30.0,
            "temporal_T_cons_s": 3.5,
            "association": "one-to-one greedy nearest last centroid; real bag dt",
            "M_interpretation": "current output requires at least M qualifying prior observations (M+1 total hits)",
        },
    },
    "candidate_graph_dp": {
        "input": "same frozen 1 m range-adaptive minPts=1 candidates used by temporal_evidence_mvp",
        "doppler": "NOT_USED",
        "node_score": "0.50 + 0.30*exp(-extent/2m) + 0.20*min(log1p(raw_support)/log(6),1)",
        "node_threshold": "NONE",
        "edge_max_frame_gap": 3,
        "edge_max_dt_s": 3.5,
        "edge_max_speed_mps": 25.0,
        "edge_max_acceleration_mps2": 15.0,
        "edge_nearest_predecessors_per_prior_frame": 12,
        "gap_penalty_per_skipped_frame": 0.25,
        "path_min_nodes": 3,
        "path_min_duration_s": 2.0,
        "output": "single highest-score admissible path per bag; one UAV is assumed by the DEV flight design",
        "algorithm": "second-order Viterbi over pruned candidate DAG edge states",
    },
    "evaluation": {
        "label_radius_m": 3.0,
        "report_operational_precision": False,
        "reason": "no reliable negative labels; unlabeled frames cannot be asserted target-absent",
        "primary_methods": [
            "A_prime_legacy_corrected", "B_khosravi_current",
            "Khosravi_2026_structure_reconstruction", "candidate_graph_DP",
        ],
        "prior_greedy_references": ["fixed_soft@0.68_FORMAL", "fixed_soft@0.55_DIAGNOSTIC"],
    },
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = fields or (list(rows[0]) if rows else [])
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        return "UNKNOWN"


def git_dirty() -> bool:
    try:
        return bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip())
    except Exception:
        return True


def freeze() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    payload = {
        **PROTOCOL,
        "selected_conditions": list(CONDITIONS),
        "source_sha256": {
            "silver_labels": sha256(DEV / "silver_labels.csv"),
            "research_dev_protocol": sha256(DEV / "protocol.json"),
            "research_dev_addendum": sha256(DEV / "input_protocol_addendum.json"),
            "previous_temporal_protocol": sha256(PREVIOUS / "protocol.json"),
            "previous_temporal_results": sha256(PREVIOUS / "benchmark_results.csv"),
            "previous_score_tradeoff": sha256(PREVIOUS / "score_tradeoff.csv"),
            "calibration_alignment": sha256(CLOSURE / "alignment.json"),
        },
    }
    path = OUT / "protocol.json"
    if path.exists() and json.loads(path.read_text(encoding="utf-8")) != payload:
        raise RuntimeError("protocol.json differs; refusing silent retuning")
    if not path.exists():
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    rows = []
    reported = PROTOCOL["published_structure_baseline"]["paper_reported"]
    adapted = PROTOCOL["published_structure_baseline"]["project_adapted"]
    for key, value in reported.items():
        rows.append({"parameter": key, "value": json.dumps(value, ensure_ascii=False),
                     "provenance": "PAPER_REPORTED", "note": "Khosravi et al. 2026"})
    for key in PROTOCOL["published_structure_baseline"]["paper_not_numerically_specified"]:
        rows.append({"parameter": key, "value": "UNKNOWN", "provenance": "PAPER_NOT_GIVEN",
                     "note": "not numerically specified in the public paper"})
    for key, value in adapted.items():
        rows.append({"parameter": key, "value": json.dumps(value, ensure_ascii=False),
                     "provenance": "PROJECT_ADAPTATION", "note": "frozen before DEV comparison"})
    write_csv(OUT / "parameter_provenance.csv", rows)
    (OUT / "negative_window_audit.json").write_text(json.dumps({
        "status": "NO_RELIABLE_NEGATIVE_WINDOWS",
        "operational_precision_reported": False,
        "false_alarms_per_frame_reported": False,
        "pr_curve_reported": False,
        "allowed_volume_metrics": ["input candidates per LiDAR frame", "outputs per LiDAR frame",
                                   "unmatched outputs per labeled positive frame"],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    (OUT / "FROZEN").write_text(
        "published-baseline and candidate-graph DP parameters frozen before DEV comparison\n",
        encoding="utf-8",
    )
    print("frozen published_baseline_dp_mvp protocol")


def _condition_spec(condition_id: str):
    return next(x for x in rawbag.CONDITION_SPECS if x[0] == condition_id)


def _alignment() -> tuple[np.ndarray, np.ndarray]:
    full = json.loads((CLOSURE / "alignment.json").read_text(encoding="utf-8"))["full_6dof"]
    return np.asarray(full["rotation_matrix"], dtype=float), np.asarray(full["translation_m"], dtype=float)


def _voxelize(points: np.ndarray, intensity: np.ndarray, velocity: np.ndarray, voxel_m: float) -> np.ndarray:
    if not len(points):
        return np.zeros((0, 6), dtype=np.float32)
    keys = np.floor(points / voxel_m).astype(np.int32)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    n = int(inverse.max()) + 1
    count = np.bincount(inverse, minlength=n).astype(np.float64)
    out = np.empty((n, 6), dtype=np.float64)
    for j in range(3):
        out[:, j] = np.bincount(inverse, weights=points[:, j], minlength=n) / count
    out[:, 3] = count
    out[:, 4] = np.bincount(inverse, weights=intensity, minlength=n) / count
    out[:, 5] = np.bincount(inverse, weights=velocity, minlength=n) / count
    return out.astype(np.float32)


def build_fine_cache(force: bool = False) -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before cache")
    CACHE.mkdir(parents=True, exist_ok=True)
    cfg = PROTOCOL["published_structure_baseline"]
    adapted = cfg["project_adapted"]
    voxel_m = cfg["paper_reported"]["voxel_m"]
    rotation, translation = _alignment()
    roi = adapted["broad_roi_m"]
    ilo, ihi = adapted["intensity_range_inclusive"]
    for condition_id in CONDITIONS:
        npz_path, json_path = CACHE / f"{condition_id}.npz", CACHE / f"{condition_id}.json"
        if npz_path.exists() and json_path.exists() and not force:
            print(f"fine cache {condition_id}: exists")
            continue
        frames, metadata, counts = [], [], []
        started = time.perf_counter()
        for fi, (arr, meta) in enumerate(rawbag.iter_condition_pointclouds(_condition_spec(condition_id))):
            x = np.asarray(arr["x"], dtype=np.float32).reshape(-1)
            sy = np.asarray(arr["y"], dtype=np.float32).reshape(-1)
            sz = np.asarray(arr["z"], dtype=np.float32).reshape(-1)
            intensity = np.asarray(arr["intensity"], dtype=np.float32).reshape(-1)
            velocity = np.asarray(arr["velocity"], dtype=np.float32).reshape(-1)
            if int(meta["point_step"]) == 80:
                y, z = sz, -sy
            else:
                y, z = sy, sz
            xyz = np.column_stack([x, y, z])
            finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity) & np.isfinite(velocity)
            keep = finite & (np.einsum("ij,ij->i", xyz, xyz) > 1e-8)
            keep &= ((xyz[:, 0] >= roi["x"][0]) & (xyz[:, 0] <= roi["x"][1]) &
                     (xyz[:, 1] >= roi["y"][0]) & (xyz[:, 1] <= roi["y"][1]) &
                     (xyz[:, 2] >= roi["z"][0]) & (xyz[:, 2] <= roi["z"][1]) &
                     (intensity >= ilo) & (intensity <= ihi))
            leveled_z = (rotation.T @ (xyz - translation).T)[2]
            keep &= leveled_z > adapted["leveled_height_gt_m"]
            values = _voxelize(xyz[keep], intensity[keep], velocity[keep], voxel_m)
            frames.append(values)
            metadata.append({"frame_index": fi, "bag_time": float(meta["bag_time"]),
                             "header_time": float(meta["header_time"]), "point_step": int(meta["point_step"])})
            counts.append({"frame_index": fi, "raw_records": int(len(xyz)),
                           "roi_intensity_height_points": int(keep.sum()), "fine_voxels": int(len(values))})
            if fi == 0 or (fi + 1) % 20 == 0:
                print(f"fine cache {condition_id}: frame={fi+1} elapsed={time.perf_counter()-started:.1f}s", flush=True)
        offsets = np.zeros(len(frames) + 1, dtype=np.int64)
        offsets[1:] = np.cumsum([len(x) for x in frames])
        values = np.concatenate(frames) if frames else np.zeros((0, 6), dtype=np.float32)
        np.savez_compressed(npz_path, values=values, offsets=offsets,
                            times=np.asarray([x["bag_time"] for x in metadata], dtype=np.float64))
        json_path.write_text(json.dumps({
            "condition_id": condition_id, "frames": metadata, "counts": counts,
            "voxel_m": voxel_m, "npz_sha256": sha256(npz_path),
            "source_archive": str(rawbag.DATA_ROOT / _condition_spec(condition_id)[2]),
            "elapsed_s": time.perf_counter() - started,
        }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"fine cache {condition_id}: {len(frames)} frames, {len(values)} voxels")


def load_fine_frames(condition_id: str) -> tuple[list[np.ndarray], np.ndarray]:
    path = CACHE / f"{condition_id}.npz"
    if not path.exists():
        raise FileNotFoundError(f"missing {path}; run cache")
    with np.load(path) as z:
        values, offsets, times = z["values"].copy(), z["offsets"].copy(), z["times"].copy()
    return [values[offsets[i]:offsets[i+1]] for i in range(len(offsets)-1)], times


def published_clusters(frames: list[np.ndarray]) -> tuple[list[list[dict]], float]:
    cfg = PROTOCOL["published_structure_baseline"]
    paper, adapted = cfg["paper_reported"], cfg["project_adapted"]
    outputs = []
    started = time.perf_counter()
    for values in frames:
        if not len(values):
            outputs.append([])
            continue
        ranges = np.linalg.norm(values[:, :3], axis=1)
        mean_range = float(np.mean(ranges))
        eps = paper["dbscan_eps0_m"] + adapted["dbscan_alpha"] * max(
            mean_range - paper["dbscan_reference_range_m"], 0.0)
        labels = DBSCAN(eps=eps, min_samples=paper["dbscan_min_pts"], algorithm="kd_tree").fit_predict(values[:, :3])
        current = []
        for lab in np.unique(labels):
            if lab < 0:
                continue
            c = values[labels == lab]
            voxels = len(c)
            raw_support = float(c[:, 3].sum())
            extent = float(np.max(np.ptp(c[:, :3], axis=0)))
            if not (adapted["cluster_min_voxels"] <= voxels <= adapted["cluster_max_voxels"]):
                continue
            if raw_support > adapted["cluster_max_raw_points"] or extent >= adapted["cluster_max_axis_extent_m"]:
                continue
            center = np.median(c[:, :3], axis=0) if voxels >= 3 else np.mean(c[:, :3], axis=0)
            distance = float(np.linalg.norm(center))
            if not (adapted["cluster_range_m"][0] <= distance <= adapted["cluster_range_m"][1]):
                continue
            current.append({
                "center": center, "raw_points": int(round(raw_support)), "voxels": voxels,
                "extent": extent, "intensity": float(np.average(c[:, 4], weights=c[:, 3])),
                "radial_velocity": float(np.average(c[:, 5], weights=c[:, 3])), "eps": eps,
            })
        outputs.append(current)
    return outputs, time.perf_counter() - started


def published_temporal(candidates: list[list[dict]], times: np.ndarray) -> tuple[list[list[dict]], float]:
    cfg = PROTOCOL["published_structure_baseline"]["project_adapted"]
    tracks: dict[int, dict] = {}
    next_id = 0
    outputs = []
    started = time.perf_counter()
    for fi, source in enumerate(candidates):
        now = float(times[fi])
        current = copy.deepcopy(source)
        active = [tid for tid, tr in tracks.items() if 0 < now - tr["times"][-1] <= cfg["temporal_T_cons_s"]]
        options = []
        if active and current:
            last = np.asarray([tracks[tid]["positions"][-1] for tid in active])
            tree = cKDTree(last)
            k = min(4, len(active))
            for ci, cand in enumerate(current):
                distances, indices = tree.query(cand["center"], k=k)
                for distance, ai in zip(np.atleast_1d(distances), np.atleast_1d(indices)):
                    tid = active[int(ai)]
                    dt = now - tracks[tid]["times"][-1]
                    gate = max(cfg["jump_tau_min_m"], cfg["jump_vmax_mps"] * dt)
                    if distance <= gate:
                        options.append((float(distance), ci, tid))
        assigned_c, assigned_t = set(), set()
        for _, ci, tid in sorted(options):
            if ci in assigned_c or tid in assigned_t:
                continue
            tracks[tid]["frames"].append(fi)
            tracks[tid]["times"].append(now)
            tracks[tid]["positions"].append(current[ci]["center"])
            current[ci]["track_id"] = tid
            assigned_c.add(ci); assigned_t.add(tid)
        for ci, cand in enumerate(current):
            if ci in assigned_c:
                continue
            tid = next_id; next_id += 1
            tracks[tid] = {"frames": [fi], "times": [now], "positions": [cand["center"]]}
            cand["track_id"] = tid
        accepted = []
        for cand in current:
            tr = tracks[cand["track_id"]]
            prior_times = tr["times"][:-1][-cfg["temporal_K_prior_observations"]:]
            prior_positions = tr["positions"][:-1][-cfg["temporal_K_prior_observations"]:]
            support = sum(
                now - t < cfg["temporal_T_cons_s"] and np.linalg.norm(cand["center"] - p) < cfg["temporal_d_cons_m"]
                for t, p in zip(prior_times, prior_positions)
            )
            cand["temporal_prior_support"] = int(support)
            if support >= cfg["temporal_M"]:
                accepted.append(cand)
        outputs.append(accepted)
    return outputs, time.perf_counter() - started


def node_score(candidate: dict) -> float:
    compact = math.exp(-float(candidate["extent"]) / 2.0)
    support = min(math.log1p(float(candidate["raw_points"])) / math.log(6.0), 1.0)
    return 0.50 + 0.30 * compact + 0.20 * support


def candidate_graph_dp(candidates: list[list[dict]], times: np.ndarray) -> tuple[list[list[dict]], dict, float]:
    """Return the best second-order admissible path; Doppler is deliberately absent."""
    cfg = PROTOCOL["candidate_graph_dp"]
    started = time.perf_counter()
    nodes, by_frame = [], []
    for fi, frame in enumerate(candidates):
        ids = []
        for ci, candidate in enumerate(frame):
            nid = len(nodes)
            nodes.append({"id": nid, "frame": fi, "time": float(times[fi]), "candidate_index": ci,
                          "center": np.asarray(candidate["center"]), "score": node_score(candidate)})
            ids.append(nid)
        by_frame.append(ids)

    incoming: dict[int, list[int]] = defaultdict(list)
    edges = []
    edge_score, edge_parent, edge_nodes = {}, {}, {}
    for fi in range(len(by_frame)):
        for gap in range(1, cfg["edge_max_frame_gap"] + 1):
            pi = fi - gap
            if pi < 0 or not by_frame[pi] or not by_frame[fi]:
                continue
            dt = float(times[fi] - times[pi])
            if dt <= 0 or dt > cfg["edge_max_dt_s"]:
                continue
            prior_ids = by_frame[pi]
            prior_xyz = np.asarray([nodes[u]["center"] for u in prior_ids])
            tree = cKDTree(prior_xyz)
            k = min(cfg["edge_nearest_predecessors_per_prior_frame"], len(prior_ids))
            radius = cfg["edge_max_speed_mps"] * dt
            for v in by_frame[fi]:
                distances, indexes = tree.query(nodes[v]["center"], k=k, distance_upper_bound=radius)
                for distance, j in zip(np.atleast_1d(distances), np.atleast_1d(indexes)):
                    if not np.isfinite(distance) or int(j) >= len(prior_ids):
                        continue
                    u = prior_ids[int(j)]
                    eid = len(edges)
                    velocity = (nodes[v]["center"] - nodes[u]["center"]) / dt
                    edges.append({"u": u, "v": v, "dt": dt, "velocity": velocity,
                                  "gap_penalty": cfg["gap_penalty_per_skipped_frame"] * (gap - 1)})
                    incoming[v].append(eid)

    for eid, edge in enumerate(edges):
        u, v = edge["u"], edge["v"]
        base = nodes[u]["score"] + nodes[v]["score"] - edge["gap_penalty"]
        best_score, best_parent, best_count = base, None, 2
        for peid in incoming.get(u, []):
            if peid not in edge_score:
                continue
            previous = edges[peid]
            accel_dt = 0.5 * (previous["dt"] + edge["dt"])
            acceleration = float(np.linalg.norm(edge["velocity"] - previous["velocity"]) / max(accel_dt, 1e-9))
            if acceleration > cfg["edge_max_acceleration_mps2"]:
                continue
            score = edge_score[peid] + nodes[v]["score"] - edge["gap_penalty"]
            count = edge_nodes[peid] + 1
            if score > best_score:
                best_score, best_parent, best_count = score, peid, count
        edge_score[eid] = best_score
        edge_parent[eid] = best_parent
        edge_nodes[eid] = best_count

    qualifying = []
    for eid, count in edge_nodes.items():
        if count < cfg["path_min_nodes"]:
            continue
        end = nodes[edges[eid]["v"]]
        cursor, start = eid, None
        while cursor is not None:
            start = nodes[edges[cursor]["u"]]
            cursor = edge_parent[cursor]
        if end["time"] - start["time"] >= cfg["path_min_duration_s"]:
            qualifying.append(eid)

    selected_ids = []
    best_eid = max(qualifying, key=lambda e: edge_score[e]) if qualifying else None
    if best_eid is not None:
        reverse = [edges[best_eid]["v"]]
        cursor = best_eid
        while cursor is not None:
            reverse.append(edges[cursor]["u"])
            cursor = edge_parent[cursor]
        selected_ids = list(reversed(reverse))

    outputs = [[] for _ in candidates]
    for nid in selected_ids:
        node = nodes[nid]
        candidate = copy.deepcopy(candidates[node["frame"]][node["candidate_index"]])
        candidate["track_id"] = 0
        candidate["evidence_score"] = node["score"]
        candidate["dp_path_score"] = edge_score[best_eid] if best_eid is not None else ""
        outputs[node["frame"]].append(candidate)
    diagnostics = {
        "nodes": len(nodes), "edges": len(edges), "qualifying_paths": len(qualifying),
        "selected_path_nodes": len(selected_ids),
        "selected_path_duration_s": (nodes[selected_ids[-1]]["time"] - nodes[selected_ids[0]]["time"] if selected_ids else 0.0),
        "selected_path_score": edge_score[best_eid] if best_eid is not None else None,
    }
    if selected_ids:
        path_xyz = np.asarray([nodes[nid]["center"] for nid in selected_ids])
        path_times = np.asarray([nodes[nid]["time"] for nid in selected_ids])
        steps = np.linalg.norm(np.diff(path_xyz, axis=0), axis=1)
        step_dt = np.diff(path_times)
        diagnostics.update({
            "selected_displacement_m": float(np.linalg.norm(path_xyz[-1] - path_xyz[0])),
            "selected_path_length_m": float(np.sum(steps)),
            "selected_speed_median_mps": float(np.median(steps / step_dt)) if len(steps) else 0.0,
            "selected_range_median_m": float(np.median(np.linalg.norm(path_xyz, axis=1))),
        })
    else:
        diagnostics.update({
            "selected_displacement_m": 0.0, "selected_path_length_m": 0.0,
            "selected_speed_median_mps": 0.0, "selected_range_median_m": "",
        })
    return outputs, diagnostics, time.perf_counter() - started


def load_labels() -> list[dict]:
    return read_csv(DEV / "silver_labels.csv")


def evaluate(method: str, condition_id: str, outputs: list[list[dict]], labels: list[dict],
             runtime_s: float, input_candidates: int) -> tuple[dict, list[dict]]:
    truth = sorted([x for x in labels if x["condition_id"] == condition_id], key=lambda x: int(x["frame_index"]))
    found, errors, frame_rows = [], [], []
    unmatched = 0
    for label in truth:
        fi = int(label["frame_index"])
        gt = np.asarray([float(label["center_x_m"]), float(label["center_y_m"]), float(label["center_z_m"])])
        current = outputs[fi]
        distances = np.asarray([np.linalg.norm(x["center"] - gt) for x in current], dtype=float)
        matches = np.flatnonzero(distances <= sparse.P["label_radius_m"])
        detected = bool(len(matches))
        found.append(detected)
        best = int(matches[np.argmin(distances[matches])]) if detected else -1
        if detected:
            errors.append(float(distances[best]))
        unmatched += len(current) - int(detected)
        frame_rows.append({
            "method": method, "condition_id": condition_id, "nominal_range_m": label["nominal_range_m"],
            "frame_index": fi, "detected": detected, "candidate_count": len(current),
            "unmatched_candidates": len(current) - int(detected),
            "center_error_m": float(distances[best]) if detected else "",
            "matched_track_id": current[best].get("track_id", "") if detected else "",
        })
    total_outputs = sum(map(len, outputs))
    return {
        "method": method, "condition_id": condition_id, "nominal_range_m": int(truth[0]["nominal_range_m"]),
        "lidar_frames": len(outputs), "label_frames": len(truth), "detected_frames": int(sum(found)),
        "frame_recall": float(np.mean(found)),
        "input_candidates_all_frames": input_candidates,
        "input_candidates_per_lidar_frame": input_candidates / len(outputs),
        "output_candidates_all_frames": total_outputs,
        "output_candidates_per_lidar_frame": total_outputs / len(outputs),
        "candidate_reduction_fraction": 1 - total_outputs / input_candidates if input_candidates else "",
        "unmatched_outputs_labeled_frames": unmatched,
        "unmatched_outputs_per_labeled_frame": unmatched / len(truth),
        "center_error_mean_m": float(np.mean(errors)) if errors else "",
        "runtime_s": runtime_s,
        "operational_precision": "UNKNOWN_NO_RELIABLE_NEGATIVE_LABELS",
    }, frame_rows


def _normalize_previous(row: dict) -> dict:
    frames = int(row["lidar_frames"])
    inputs = int(row["input_candidates_all_frames"])
    return {
        "method": row["method"], "condition_id": row["condition_id"],
        "nominal_range_m": int(row["nominal_range_m"]), "lidar_frames": frames,
        "label_frames": int(row["label_frames"]), "detected_frames": int(row["detected_frames"]),
        "frame_recall": float(row["frame_recall"]), "input_candidates_all_frames": inputs,
        "input_candidates_per_lidar_frame": inputs / frames,
        "output_candidates_all_frames": int(row["output_candidates_all_frames"]),
        "output_candidates_per_lidar_frame": float(row["output_candidates_per_lidar_frame"]),
        "candidate_reduction_fraction": row["candidate_reduction_fraction"],
        "unmatched_outputs_labeled_frames": int(row["unmatched_candidates_labeled_frames"]),
        "unmatched_outputs_per_labeled_frame": float(row["unmatched_candidates_per_labeled_frame"]),
        "center_error_mean_m": row["center_error_mean_m"], "runtime_s": float(row["runtime_s"]),
        "operational_precision": "UNKNOWN_NO_RELIABLE_NEGATIVE_LABELS",
    }


def aggregate(metrics: list[dict], method: str, distance: int) -> dict:
    rows = [x for x in metrics if x["method"] == method and x["condition_id"] != "ALL" and int(x["nominal_range_m"]) == distance]
    frames = sum(int(x["lidar_frames"]) for x in rows)
    labels = sum(int(x["label_frames"]) for x in rows)
    detected = sum(int(x["detected_frames"]) for x in rows)
    inputs = sum(int(x["input_candidates_all_frames"]) for x in rows)
    outputs = sum(int(x["output_candidates_all_frames"]) for x in rows)
    unmatched = sum(int(x["unmatched_outputs_labeled_frames"]) for x in rows)
    return {
        "method": method, "condition_id": "ALL", "nominal_range_m": distance,
        "lidar_frames": frames, "label_frames": labels, "detected_frames": detected,
        "frame_recall": detected / labels if labels else "",
        "input_candidates_all_frames": inputs, "input_candidates_per_lidar_frame": inputs / frames if frames else "",
        "output_candidates_all_frames": outputs, "output_candidates_per_lidar_frame": outputs / frames if frames else "",
        "candidate_reduction_fraction": 1 - outputs / inputs if inputs else "",
        "unmatched_outputs_labeled_frames": unmatched,
        "unmatched_outputs_per_labeled_frame": unmatched / labels if labels else "",
        "center_error_mean_m": "", "runtime_s": sum(float(x["runtime_s"]) for x in rows),
        "operational_precision": "UNKNOWN_NO_RELIABLE_NEGATIVE_LABELS",
    }


def detection_rows(method: str, condition_id: str, outputs: list[list[dict]]) -> list[dict]:
    rows = []
    for fi, frame in enumerate(outputs):
        for candidate in frame:
            rows.append({
                "method": method, "condition_id": condition_id, "frame_index": fi,
                "track_id": candidate.get("track_id", ""), "center_x_m": candidate["center"][0],
                "center_y_m": candidate["center"][1], "center_z_m": candidate["center"][2],
                "raw_points": candidate["raw_points"], "voxels": candidate["voxels"],
                "extent_m": candidate["extent"], "eps_m": candidate.get("eps", ""),
                "temporal_prior_support": candidate.get("temporal_prior_support", ""),
                "node_score": candidate.get("evidence_score", ""),
                "path_score": candidate.get("dp_path_score", ""),
            })
    return rows


def candidate_stage_recall(method: str, condition_id: str, candidates: list[list[dict]], labels: list[dict]) -> dict:
    truth = [x for x in labels if x["condition_id"] == condition_id]
    detected = 0
    for label in truth:
        fi = int(label["frame_index"])
        gt = np.asarray([float(label["center_x_m"]), float(label["center_y_m"]), float(label["center_z_m"])])
        detected += int(any(np.linalg.norm(c["center"] - gt) <= sparse.P["label_radius_m"] for c in candidates[fi]))
    return {
        "method": method, "condition_id": condition_id, "nominal_range_m": int(truth[0]["nominal_range_m"]),
        "lidar_frames": len(candidates), "label_frames": len(truth), "detected_frames": detected,
        "candidate_stage_recall": detected / len(truth), "candidate_count": sum(map(len, candidates)),
        "candidates_per_lidar_frame": sum(map(len, candidates)) / len(candidates),
        "status": "DIAGNOSTIC_NOT_AN_OPERATIONAL_DETECTION_OUTPUT",
    }


def prior_greedy_references() -> list[dict]:
    rows = []
    formal = [x for x in read_csv(PREVIOUS / "benchmark_results.csv")
              if x["method"] == "fixed_soft" and x["condition_id"] != "ALL"]
    diagnostic = [x for x in read_csv(PREVIOUS / "score_tradeoff.csv")
                  if x["method"] == "fixed_soft" and x["condition_id"] != "ALL" and abs(float(x["score_threshold"]) - 0.55) < 1e-9]
    for status, source in (("FORMAL_FROZEN", formal), ("POST_HOC_DIAGNOSTIC", diagnostic)):
        for distance in (100, 200, 300):
            rr = [x for x in source if int(x["nominal_range_m"]) == distance]
            frames = sum(int(x["lidar_frames"]) for x in rr)
            labels = sum(int(x["label_frames"]) for x in rr)
            detections = sum(int(x["detected_frames"]) for x in rr)
            outputs = sum(int(x["output_candidates_all_frames"]) for x in rr)
            rows.append({
                "reference": "fixed_soft", "threshold": 0.68 if status == "FORMAL_FROZEN" else 0.55,
                "status": status, "nominal_range_m": distance, "lidar_frames": frames,
                "label_frames": labels, "detected_frames": detections,
                "frame_recall": detections / labels if labels else "",
                "output_candidates_all_frames": outputs,
                "output_candidates_per_lidar_frame": outputs / frames if frames else "",
                "interpretation_limit": "same DEV labels; 0.55 was inspected after the formal run",
            })
    return rows


def run() -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before experiment")
    labels = load_labels()
    metrics, frame_rows, detections, graph_rows, stage_rows = [], [], [], [], []
    previous = read_csv(PREVIOUS / "benchmark_results.csv")
    for row in previous:
        if row["method"] in {"A_prime_legacy_corrected", "B_khosravi_current"} and row["condition_id"] != "ALL":
            metrics.append(_normalize_previous(row))

    for condition_id in CONDITIONS:
        fine_frames, fine_times = load_fine_frames(condition_id)
        kh_candidates, cluster_s = published_clusters(fine_frames)
        stage_rows.append(candidate_stage_recall("Khosravi_reconstruction_pre_temporal", condition_id,
                                                kh_candidates, labels))
        kh_outputs, temporal_s = published_temporal(kh_candidates, fine_times)
        metric, rows = evaluate("Khosravi_2026_structure_reconstruction", condition_id, kh_outputs,
                                labels, cluster_s + temporal_s, sum(map(len, kh_candidates)))
        metrics.append(metric); frame_rows.extend(rows)
        detections.extend(detection_rows("Khosravi_2026_structure_reconstruction", condition_id, kh_outputs))

        source_frames, times = sparse._load_raw_voxel_frames(condition_id)
        singleton, cluster_time = sparse.cluster_frames(source_frames, min_pts=1, adaptive=True)
        stage_rows.append(candidate_stage_recall("minPts1_singleton_pre_DP", condition_id, singleton, labels))
        dp_outputs, diagnostics, dp_time = candidate_graph_dp(singleton, times)
        metric, rows = evaluate("candidate_graph_DP", condition_id, dp_outputs, labels,
                                cluster_time + dp_time, sum(map(len, singleton)))
        metrics.append(metric); frame_rows.extend(rows)
        detections.extend(detection_rows("candidate_graph_DP", condition_id, dp_outputs))
        graph_rows.append({"condition_id": condition_id, **diagnostics,
                           "candidate_generation_runtime_s": cluster_time, "dp_runtime_s": dp_time})
        print(f"published+DP {condition_id}: complete")

    methods = PROTOCOL["evaluation"]["primary_methods"]
    for method in methods:
        for distance in (100, 200, 300):
            metrics.append(aggregate(metrics, method, distance))
    write_csv(OUT / "benchmark_results.csv", metrics)
    write_csv(OUT / "frame_results.csv", frame_rows)
    write_csv(OUT / "candidate_detections.csv", detections)
    write_csv(OUT / "graph_diagnostics.csv", graph_rows)
    write_csv(OUT / "candidate_stage_recall.csv", stage_rows)
    write_csv(OUT / "prior_greedy_reference.csv", prior_greedy_references())
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "git_head_before_run": git_head(),
        "git_dirty_before_run": git_dirty(), "script_sha256": sha256(Path(__file__)),
        "protocol_sha256": sha256(OUT / "protocol.json"), "labels_sha256": sha256(DEV / "silver_labels.csv"),
        "commands": ["python3 scripts/published_baseline_dp_mvp.py freeze",
                     "python3 scripts/published_baseline_dp_mvp cache",
                     "python3 scripts/published_baseline_dp_mvp run"],
        "fine_cache": {cid: sha256(CACHE / f"{cid}.npz") for cid in CONDITIONS},
    }
    (OUT / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["freeze", "cache", "run", "all"])
    parser.add_argument("--force-cache", action="store_true")
    args = parser.parse_args()
    if args.action in {"freeze", "all"}:
        freeze()
    if args.action in {"cache", "all"}:
        build_fine_cache(force=args.force_cache)
    if args.action in {"run", "all"}:
        run()


if __name__ == "__main__":
    main()
