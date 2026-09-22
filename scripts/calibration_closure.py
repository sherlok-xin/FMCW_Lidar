#!/usr/bin/env python3
"""Independent LiDAR trajectory search and GNSS/LiDAR calibration closure.

This diagnostic deliberately does not import or modify the detection/tracking
baseline.  Candidate trajectories are generated from LiDAR measurements only;
GNSS is first used in the later alignment phase.  Raw bags are opened read-only
through the audited parser in ``build_gt_benchmark_v0.py``.

The workflow is split into phases because decoding the nine ZIP-contained bags
is expensive::

    python scripts/calibration_closure.py --phase cache
    python scripts/calibration_closure.py --phase search
    python scripts/calibration_closure.py --phase align

All thresholds are common to all conditions and recorded in ``protocol.json``.
They are not adjusted using GNSS agreement.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import differential_evolution, least_squares
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN

from build_gt_benchmark_v0 import (
    CONDITION_SPECS,
    ROOT,
    WORKBOOK,
    iter_condition_pointclouds,
    parse_workbook,
)


OUT = ROOT / "results" / "calibration_closure"
CACHE = OUT / "raw_voxel_cache"
PLOTS = OUT / "plots"

# Frozen before reading GNSS alignment residuals.  Bounds describe the broad
# valid forward sensing volume rather than any named 100/200/300 m trajectory.
PROTOCOL = {
    "protocol_version": 4,
    "random_seed": 20260922,
    "candidate_generation_uses_gnss": False,
    "coordinate_transform_rich80": "[x,z,-y]",
    "valid_volume_m": {"x": [0.0, 520.0], "y": [-250.0, 250.0], "z": [-100.0, 120.0]},
    "exclude_zero_placeholders": True,
    "voxel_size_m": 1.0,
    "max_voxel_frame_fraction_primary": 0.10,
    "max_voxel_frame_fraction_sensitivity": [0.05, 0.20],
    "component_eps_m": 2.25,
    "component_max_extent_m": 6.0,
    "component_max_points": 120,
    "component_max_voxels": 32,
    "component_min_range_m": 8.0,
    "component_max_range_m": 510.0,
    "max_components_per_frame": 1500,
    "association_initial_speed_mps": 18.0,
    "association_max_speed_mps": 25.0,
    "association_base_gate_m": 3.0,
    "association_prediction_acceleration_mps2": 5.0,
    "association_max_gap_frames": 2,
    "min_track_observations": 5,
    "min_track_span_frames": 6,
    "min_median_speed_mps": 0.5,
    "max_median_speed_mps": 20.0,
    "max_p90_acceleration_mps2": 8.0,
    "min_median_direction_cosine": 0.30,
    "max_median_doppler_residual_mps": 4.0,
    "evidence_min_points_per_observation": 3,
    "evidence_min_supported_observations": 5,
    "evidence_max_supported_gap_frames": 3,
    "calibration_conditions": [
        "300m_roundtrip_5", "300m_roundtrip_10",
        "100m_cross_5", "200m_cross_5", "300m_cross_5",
    ],
    "validation_conditions": [
        "300m_roundtrip_13",
        "100m_cross_10", "200m_cross_10", "300m_cross_10",
    ],
    "offset_search_margin_from_interval_midpoint_s": 40.0,
    "position_huber_scale_m": 5.0,
    "closure_position_rmse_m": 10.0,
    "closure_range_rmse_m": 5.0,
    "closure_direction_median_deg": 35.0,
    "closure_doppler_rmse_mps": 3.0,
    "closure_min_path_fraction": 0.15,
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_protocol() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    payload = {
        **PROTOCOL,
        "frozen_utc": "2026-09-22T00:00:00+00:00",
        "note": "Thresholds are common across all bags and fixed before GNSS alignment scoring.",
    }
    path = OUT / "protocol.json"
    if path.exists():
        old = json.loads(path.read_text(encoding="utf-8"))
        if old != payload:
            raise RuntimeError("protocol.json differs from the frozen protocol; refusing silent retuning")
    else:
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _frame_voxels(arr: np.ndarray, point_step: int) -> tuple[np.ndarray, dict[str, int]]:
    x = np.asarray(arr["x"], dtype=np.float64).reshape(-1)
    sy = np.asarray(arr["y"], dtype=np.float64).reshape(-1)
    sz = np.asarray(arr["z"], dtype=np.float64).reshape(-1)
    intensity = np.asarray(arr["intensity"], dtype=np.float64).reshape(-1)
    velocity = np.asarray(arr["velocity"], dtype=np.float64).reshape(-1)
    if point_step == 80:
        y, z = sz, -sy
    else:
        y, z = sy, sz
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity) & np.isfinite(velocity)
    nonzero = finite & ((x != 0.0) | (y != 0.0) | (z != 0.0))
    bounds = PROTOCOL["valid_volume_m"]
    valid = (nonzero &
             (x >= bounds["x"][0]) & (x <= bounds["x"][1]) &
             (y >= bounds["y"][0]) & (y <= bounds["y"][1]) &
             (z >= bounds["z"][0]) & (z <= bounds["z"][1]))
    idx = np.flatnonzero(valid)
    stats = {
        "total_samples": int(len(x)), "finite_samples": int(finite.sum()),
        "nonzero_returns": int(nonzero.sum()), "valid_returns": int(valid.sum()),
    }
    if not len(idx):
        return np.empty((0, 11), dtype=np.float64), stats

    xyz = np.column_stack([x[idx], y[idx], z[idx]])
    lo = np.array([bounds["x"][0], bounds["y"][0], bounds["z"][0]], dtype=float)
    size = float(PROTOCOL["voxel_size_m"])
    q = np.floor((xyz - lo) / size).astype(np.int32)
    shape = np.ceil((np.array([bounds["x"][1], bounds["y"][1], bounds["z"][1]]) - lo) / size).astype(np.int64) + 1
    linear = (q[:, 0].astype(np.int64) * shape[1] * shape[2] +
              q[:, 1].astype(np.int64) * shape[2] + q[:, 2])
    ids, inverse, counts = np.unique(linear, return_inverse=True, return_counts=True)
    n = len(ids)
    denom = counts.astype(np.float64)
    means = np.column_stack([
        np.bincount(inverse, weights=xyz[:, j], minlength=n) / denom for j in range(3)
    ])
    imean = np.bincount(inverse, weights=intensity[idx], minlength=n) / denom
    vmean = np.bincount(inverse, weights=velocity[idx], minlength=n) / denom
    v2mean = np.bincount(inverse, weights=velocity[idx] ** 2, minlength=n) / denom
    vstd = np.sqrt(np.maximum(0.0, v2mean - vmean ** 2))
    out = np.column_stack([ids, means, counts, imean, vmean, vstd, q[:, :3][np.unique(linear, return_index=True)[1]]])
    # columns: voxel_id,x,y,z,point_count,intensity_mean,velocity_mean,
    # velocity_std,qx,qy,qz
    # Keep float64 because voxel_id can exceed float32's exact integer range.
    return out.astype(np.float64), stats


def cache_condition(spec: tuple[str, str, str, str, int, int], force: bool = False) -> None:
    condition_id = spec[0]
    CACHE.mkdir(parents=True, exist_ok=True)
    npz_path = CACHE / f"{condition_id}.npz"
    json_path = CACHE / f"{condition_id}.json"
    if npz_path.exists() and json_path.exists() and not force:
        print(f"cache {condition_id}: exists", flush=True)
        return
    rows, offsets, frames = [], [0], []
    started = time.time()
    for frame_index, (arr, meta) in enumerate(iter_condition_pointclouds(spec)):
        voxels, stats = _frame_voxels(arr, int(meta["point_step"]))
        rows.append(voxels)
        offsets.append(offsets[-1] + len(voxels))
        frames.append({
            "frame_index": frame_index, "bag_time": float(meta["bag_time"]),
            "header_time": float(meta["header_time"]), "seq": int(meta["seq"]),
            "voxel_count": int(len(voxels)), **stats,
        })
        if frame_index == 0 or (frame_index + 1) % 10 == 0:
            print(f"cache {condition_id}: {frame_index + 1} frames, {time.time()-started:.1f}s", flush=True)
    values = np.concatenate(rows, axis=0) if rows else np.empty((0, 11), dtype=np.float64)
    np.savez_compressed(npz_path, values=values, offsets=np.asarray(offsets, dtype=np.int64))
    meta = {
        "condition_id": condition_id, "frames": frames,
        "cache_npz": str(npz_path.relative_to(ROOT)),
        "elapsed_s": time.time() - started,
        "columns": ["voxel_id", "x", "y", "z", "point_count", "intensity_mean",
                    "velocity_mean", "velocity_std", "qx", "qy", "qz"],
    }
    json_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"cache {condition_id}: complete {len(frames)} frames, {len(values)} voxels", flush=True)


def cache_all(force: bool = False, only: str | None = None) -> None:
    for spec in CONDITION_SPECS:
        if only and spec[0] != only:
            continue
        cache_condition(spec, force=force)


def _load_cache(condition_id: str) -> tuple[dict, np.ndarray, np.ndarray]:
    meta = json.loads((CACHE / f"{condition_id}.json").read_text(encoding="utf-8"))
    with np.load(CACHE / f"{condition_id}.npz") as z:
        return meta, z["values"].copy(), z["offsets"].copy()


def _component_rows(condition_id: str, fraction: float) -> tuple[list[np.ndarray], dict]:
    meta, values, offsets = _load_cache(condition_id)
    nframes = len(meta["frames"])
    ids = values[:, 0].astype(np.int64)
    unique, frequency = np.unique(ids, return_counts=True)
    freq_map = dict(zip(unique.tolist(), frequency.tolist()))
    max_frames = max(2, int(math.ceil(fraction * nframes)))
    by_frame: list[np.ndarray] = []
    diagnostics = {"condition_id": condition_id, "frame_count": nframes,
                   "frequency_fraction": fraction, "max_frames": max_frames,
                   "frames": []}
    for fi in range(nframes):
        raw = values[offsets[fi]:offsets[fi + 1]]
        if not len(raw):
            by_frame.append(np.empty((0, 12), dtype=float))
            continue
        occ = np.asarray([freq_map[int(v)] for v in raw[:, 0]], dtype=float)
        dynamic = raw[occ <= max_frames]
        if not len(dynamic):
            by_frame.append(np.empty((0, 12), dtype=float))
            continue
        labels = DBSCAN(eps=float(PROTOCOL["component_eps_m"]), min_samples=1,
                        algorithm="kd_tree").fit_predict(dynamic[:, 1:4])
        components = []
        for label in np.unique(labels):
            c = dynamic[labels == label]
            weights = c[:, 4]
            point_count = float(weights.sum())
            extent = float(np.linalg.norm(np.ptp(c[:, 1:4], axis=0)))
            center = np.average(c[:, 1:4], axis=0, weights=weights)
            distance = float(np.linalg.norm(center))
            if (len(c) > PROTOCOL["component_max_voxels"] or
                    point_count > PROTOCOL["component_max_points"] or
                    extent > PROTOCOL["component_max_extent_m"] or
                    distance < PROTOCOL["component_min_range_m"] or
                    distance > PROTOCOL["component_max_range_m"]):
                continue
            intensity = float(np.average(c[:, 5], weights=weights))
            velocity = float(np.average(c[:, 6], weights=weights))
            persistence = float(np.average([freq_map[int(v)] / nframes for v in c[:, 0]], weights=weights))
            # LiDAR-only ordering; no nominal range, GNSS, or condition label.
            objectness = (math.log1p(point_count) - 1.5 * persistence - 0.15 * extent)
            components.append([*center, point_count, len(c), extent, intensity,
                               velocity, persistence, objectness, distance])
        components.sort(key=lambda row: row[9], reverse=True)
        limited = np.asarray(components[:int(PROTOCOL["max_components_per_frame"])], dtype=float)
        if limited.size == 0:
            limited = np.empty((0, 12), dtype=float)
        by_frame.append(limited)
        diagnostics["frames"].append({"frame_index": fi, "rare_voxels": int(len(dynamic)),
                                      "components": int(len(components)), "retained": int(len(limited))})
    return by_frame, diagnostics


def _track_components(condition_id: str, components: list[np.ndarray], meta: dict) -> list[dict]:
    times = np.asarray([x["bag_time"] for x in meta["frames"]], dtype=float)
    tracks: dict[int, dict] = {}
    active: set[int] = set()
    next_id = 0
    max_gap = int(PROTOCOL["association_max_gap_frames"])
    for fi, nodes in enumerate(components):
        active_ids = [tid for tid in sorted(active) if fi - tracks[tid]["last_frame"] <= max_gap + 1]
        if active_ids and len(nodes):
            cost = np.full((len(active_ids), len(nodes)), 1e9, dtype=float)
            for ai, tid in enumerate(active_ids):
                tr = tracks[tid]
                dt = times[fi] - tr["times"][-1]
                if dt <= 0:
                    continue
                if len(tr["positions"]) >= 2:
                    prev_dt = tr["times"][-1] - tr["times"][-2]
                    vel = ((tr["positions"][-1] - tr["positions"][-2]) / prev_dt
                           if prev_dt > 0 else np.zeros(3))
                    pred = tr["positions"][-1] + vel * dt
                    gate = (PROTOCOL["association_base_gate_m"] +
                            0.5 * PROTOCOL["association_prediction_acceleration_mps2"] * dt ** 2)
                else:
                    pred = tr["positions"][-1]
                    gate = PROTOCOL["association_base_gate_m"] + PROTOCOL["association_initial_speed_mps"] * min(dt, 1.5)
                distance = np.linalg.norm(nodes[:, :3] - pred, axis=1)
                allowed = distance <= gate
                cost[ai, allowed] = distance[allowed] - 0.05 * nodes[allowed, 9]
            rr, cc = linear_sum_assignment(cost)
            assigned_nodes = set()
            for ai, ni in zip(rr, cc):
                if cost[ai, ni] >= 1e8:
                    continue
                tid = active_ids[ai]
                tr = tracks[tid]
                tr["frames"].append(fi); tr["times"].append(times[fi])
                tr["positions"].append(nodes[ni, :3].copy()); tr["nodes"].append(nodes[ni].copy())
                tr["last_frame"] = fi
                assigned_nodes.add(int(ni))
            for ni, node in enumerate(nodes):
                if ni in assigned_nodes:
                    continue
                tracks[next_id] = {"track_id": next_id, "frames": [fi], "times": [times[fi]],
                                   "positions": [node[:3].copy()], "nodes": [node.copy()], "last_frame": fi}
                active.add(next_id); next_id += 1
        else:
            for node in nodes:
                tracks[next_id] = {"track_id": next_id, "frames": [fi], "times": [times[fi]],
                                   "positions": [node[:3].copy()], "nodes": [node.copy()], "last_frame": fi}
                active.add(next_id); next_id += 1
        active = {tid for tid in active if fi - tracks[tid]["last_frame"] <= max_gap}

    accepted = []
    for tr in tracks.values():
        if len(tr["frames"]) < PROTOCOL["min_track_observations"]:
            continue
        if tr["frames"][-1] - tr["frames"][0] + 1 < PROTOCOL["min_track_span_frames"]:
            continue
        p = np.asarray(tr["positions"]); t = np.asarray(tr["times"])
        dt = np.diff(t); dp = np.diff(p, axis=0)
        good = dt > 0
        speeds = np.linalg.norm(dp[good], axis=1) / dt[good]
        if not len(speeds):
            continue
        velocity_vectors = dp[good] / dt[good, None]
        acc = (np.linalg.norm(np.diff(velocity_vectors, axis=0), axis=1) /
               np.maximum(np.diff(t[:-1][good]), 1e-6)) if len(velocity_vectors) >= 2 else np.array([0.0])
        median_speed = float(np.median(speeds))
        p90_acc = float(np.quantile(acc, 0.9)) if len(acc) else 0.0
        if not (PROTOCOL["min_median_speed_mps"] <= median_speed <= PROTOCOL["max_median_speed_mps"]):
            continue
        if p90_acc > PROTOCOL["max_p90_acceleration_mps2"]:
            continue
        nodes = np.asarray(tr["nodes"])
        if len(velocity_vectors) >= 2:
            denom = (np.linalg.norm(velocity_vectors[:-1], axis=1) *
                     np.linalg.norm(velocity_vectors[1:], axis=1))
            direction_cosine = np.divide(
                np.sum(velocity_vectors[:-1] * velocity_vectors[1:], axis=1), denom,
                out=np.zeros_like(denom), where=denom > 1e-9)
            median_direction_cosine = float(np.median(direction_cosine))
        else:
            median_direction_cosine = 1.0
        ranges = np.linalg.norm(p, axis=1)
        range_rate = np.diff(ranges)[good] / dt[good]
        measured_rv = 0.5 * (nodes[:-1, 7][good] + nodes[1:, 7][good])
        doppler_residual = np.abs(-measured_rv - range_rate)
        median_doppler_residual = float(np.median(doppler_residual))
        if median_direction_cosine < PROTOCOL["min_median_direction_cosine"]:
            continue
        if median_doppler_residual > PROTOCOL["max_median_doppler_residual_mps"]:
            continue
        tr["summary"] = {
            "observations": len(p), "span_frames": tr["frames"][-1] - tr["frames"][0] + 1,
            "coverage": len(p) / (tr["frames"][-1] - tr["frames"][0] + 1),
            "path_length_m": float(np.linalg.norm(dp, axis=1).sum()),
            "displacement_m": float(np.linalg.norm(p[-1] - p[0])),
            "median_speed_mps": median_speed, "p90_acceleration_mps2": p90_acc,
            "median_direction_cosine": median_direction_cosine,
            "median_doppler_residual_mps": median_doppler_residual,
            "median_points": float(np.median(nodes[:, 3])),
            "median_extent_m": float(np.median(nodes[:, 5])),
            "median_intensity": float(np.median(nodes[:, 6])),
            "median_radial_velocity_mps": float(np.median(nodes[:, 7])),
            "median_persistence": float(np.median(nodes[:, 8])),
        }
        # Independent rank rewards observation support, coverage, and smoothness.
        s = tr["summary"]
        tr["lidar_score"] = (s["observations"] * s["coverage"] *
                             math.log1p(s["path_length_m"]) /
                             ((1.0 + s["p90_acceleration_mps2"]) *
                              (1.0 + s["median_doppler_residual_mps"])))
        accepted.append(tr)
    accepted.sort(key=lambda x: x["lidar_score"], reverse=True)
    return accepted


def search_condition(spec, fractions: list[float]) -> None:
    condition_id = spec[0]
    meta, _, _ = _load_cache(condition_id)
    all_tracks = []
    diag = {"condition_id": condition_id, "searches": []}
    for fraction in fractions:
        components, component_diag = _component_rows(condition_id, fraction)
        tracks = _track_components(condition_id, components, meta)
        for rank, tr in enumerate(tracks, 1):
            tr["fraction"] = fraction; tr["rank_within_fraction"] = rank
        all_tracks.extend(tracks)
        diag["searches"].append({"fraction": fraction,
                                 "accepted_tracks": len(tracks),
                                 "component_diagnostics": component_diag})
        print(f"search {condition_id}: fraction={fraction:.2f}, tracks={len(tracks)}", flush=True)
    # Deduplicate the same path recovered at different persistence thresholds.
    unique = []
    for tr in sorted(all_tracks, key=lambda x: x["lidar_score"], reverse=True):
        p = np.asarray(tr["positions"]); frames = np.asarray(tr["frames"])
        duplicate = False
        for kept in unique:
            common, ia, ib = np.intersect1d(frames, kept["frames"], return_indices=True)
            if len(common) >= 3:
                d = np.linalg.norm(p[ia] - np.asarray(kept["positions"])[ib], axis=1)
                if np.median(d) < 2.0:
                    duplicate = True; break
        if not duplicate:
            unique.append(tr)
    for rank, tr in enumerate(unique, 1):
        tr["candidate_id"] = f"{condition_id}_c{rank:04d}"
        tr["lidar_rank"] = rank
    _write_condition_tracks(condition_id, unique)
    evidence = _extract_evidence_segments(condition_id, unique)
    _write_condition_tracks(condition_id, evidence, prefix="evidence_trajectories")
    (OUT / f"search_{condition_id}.json").write_text(json.dumps({
        **diag, "unique_tracks": len(unique), "evidence_tracks": len(evidence),
        "top_summaries": [{"candidate_id": x["candidate_id"], "lidar_rank": x["lidar_rank"],
                            "lidar_score": x["lidar_score"], **x["summary"]} for x in unique[:50]],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"search {condition_id}: unique={len(unique)}, evidence={len(evidence)}", flush=True)


def _extract_evidence_segments(condition_id: str, tracks: list[dict]) -> list[dict]:
    """Split tracklets into independently supported multi-point segments.

    This is deliberately LiDAR-only.  It prevents a dense target-like segment
    from being promoted together with a later chain of unrelated singleton
    clutter, while retaining the unsplit candidates for audit.
    """
    segments = []
    min_points = int(PROTOCOL["evidence_min_points_per_observation"])
    min_support = int(PROTOCOL["evidence_min_supported_observations"])
    max_gap = int(PROTOCOL["evidence_max_supported_gap_frames"])
    for parent in tracks:
        frames = np.asarray(parent["frames"])
        times = np.asarray(parent["times"])
        positions = np.asarray(parent["positions"])
        nodes = np.asarray(parent["nodes"])
        support = np.flatnonzero(nodes[:, 3] >= min_points)
        if len(support) < min_support:
            continue
        groups = np.split(support, np.flatnonzero(np.diff(frames[support]) > max_gap) + 1)
        for group in groups:
            if len(group) < min_support:
                continue
            a, b = int(group[0]), int(group[-1]) + 1
            f, t, p, n = frames[a:b], times[a:b], positions[a:b], nodes[a:b]
            if len(f) < PROTOCOL["min_track_observations"] or f[-1] - f[0] + 1 < PROTOCOL["min_track_span_frames"]:
                continue
            dt = np.diff(t); good = dt > 0
            if not np.any(good):
                continue
            dp = np.diff(p, axis=0)
            speeds = np.linalg.norm(dp[good], axis=1) / dt[good]
            vv = dp[good] / dt[good, None]
            acc = (np.linalg.norm(np.diff(vv, axis=0), axis=1) /
                   np.maximum(np.diff(t[:-1][good]), 1e-6)) if len(vv) >= 2 else np.array([0.0])
            median_speed = float(np.median(speeds))
            p90_acc = float(np.quantile(acc, 0.9)) if len(acc) else 0.0
            if not (PROTOCOL["min_median_speed_mps"] <= median_speed <= PROTOCOL["max_median_speed_mps"]):
                continue
            if p90_acc > PROTOCOL["max_p90_acceleration_mps2"]:
                continue
            if len(vv) >= 2:
                denom = np.linalg.norm(vv[:-1], axis=1) * np.linalg.norm(vv[1:], axis=1)
                cosines = np.divide(np.sum(vv[:-1] * vv[1:], axis=1), denom,
                                    out=np.zeros_like(denom), where=denom > 1e-9)
                direction = float(np.median(cosines))
            else:
                direction = 1.0
            rr = np.diff(np.linalg.norm(p, axis=1))[good] / dt[good]
            rv = 0.5 * (n[:-1, 7][good] + n[1:, 7][good])
            doppler = float(np.median(np.abs(-rv - rr)))
            if direction < PROTOCOL["min_median_direction_cosine"] or doppler > PROTOCOL["max_median_doppler_residual_mps"]:
                continue
            summary = {
                "observations": len(p), "supported_observations": int(len(group)),
                "support_fraction": float(len(group) / len(p)),
                "span_frames": int(f[-1] - f[0] + 1), "coverage": float(len(p) / (f[-1] - f[0] + 1)),
                "path_length_m": float(np.linalg.norm(dp, axis=1).sum()),
                "displacement_m": float(np.linalg.norm(p[-1] - p[0])),
                "median_speed_mps": median_speed, "p90_acceleration_mps2": p90_acc,
                "median_direction_cosine": direction, "median_doppler_residual_mps": doppler,
                "median_points": float(np.median(n[:, 3])), "median_extent_m": float(np.median(n[:, 5])),
                "median_intensity": float(np.median(n[:, 6])),
                "median_radial_velocity_mps": float(np.median(n[:, 7])),
                "median_persistence": float(np.median(n[:, 8])),
            }
            score = (summary["observations"] * summary["coverage"] * summary["support_fraction"] *
                     math.log1p(summary["path_length_m"]) * math.log1p(summary["median_points"]) /
                     ((1.0 + p90_acc) * (1.0 + doppler)))
            segments.append({"frames": f.tolist(), "times": t.tolist(), "positions": p.tolist(),
                             "nodes": n.tolist(), "last_frame": int(f[-1]), "summary": summary,
                             "lidar_score": score, "parent_candidate_id": parent["candidate_id"]})
    segments.sort(key=lambda x: x["lidar_score"], reverse=True)
    unique = []
    for tr in segments:
        f = np.asarray(tr["frames"]); p = np.asarray(tr["positions"])
        duplicate = False
        for kept in unique:
            common, ia, ib = np.intersect1d(f, kept["frames"], return_indices=True)
            if len(common) >= 3 and np.median(np.linalg.norm(p[ia] - np.asarray(kept["positions"])[ib], axis=1)) < 2.0:
                duplicate = True; break
        if not duplicate:
            unique.append(tr)
    for rank, tr in enumerate(unique, 1):
        tr["candidate_id"] = f"{condition_id}_e{rank:04d}"
        tr["lidar_rank"] = rank
    return unique


def _write_condition_tracks(condition_id: str, tracks: list[dict], prefix: str = "trajectories") -> None:
    path = OUT / f"{prefix}_{condition_id}.npz"
    payload = {"candidate_ids": np.asarray([x["candidate_id"] for x in tracks], dtype=str),
               "lidar_scores": np.asarray([x["lidar_score"] for x in tracks], dtype=float)}
    for i, tr in enumerate(tracks):
        payload[f"frames_{i}"] = np.asarray(tr["frames"], dtype=np.int32)
        payload[f"times_{i}"] = np.asarray(tr["times"], dtype=float)
        payload[f"positions_{i}"] = np.asarray(tr["positions"], dtype=float)
        payload[f"nodes_{i}"] = np.asarray(tr["nodes"], dtype=float)
        payload[f"summary_{i}"] = np.asarray(json.dumps(tr["summary"]))
    np.savez_compressed(path, **payload)


def search_all(only: str | None = None) -> None:
    fractions = [PROTOCOL["max_voxel_frame_fraction_primary"],
                 *PROTOCOL["max_voxel_frame_fraction_sensitivity"]]
    for spec in CONDITION_SPECS:
        if only and spec[0] != only:
            continue
        search_condition(spec, fractions)


def _load_tracks(condition_id: str, prefix: str = "evidence_trajectories") -> list[dict]:
    path = OUT / f"{prefix}_{condition_id}.npz"
    with np.load(path) as z:
        result = []
        for i, candidate_id in enumerate(z["candidate_ids"].tolist()):
            result.append({
                "candidate_id": str(candidate_id), "lidar_rank": i + 1,
                "lidar_score": float(z["lidar_scores"][i]),
                "frames": z[f"frames_{i}"].astype(int), "times": z[f"times_{i}"].astype(float),
                "positions": z[f"positions_{i}"].astype(float), "nodes": z[f"nodes_{i}"].astype(float),
                "summary": json.loads(str(z[f"summary_{i}"])),
            })
        return result


def _rotation(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def _gnss_arrays(condition_cn: str, gnss: dict) -> tuple[np.ndarray, np.ndarray]:
    rows = gnss[condition_cn]
    return (np.asarray([x.gps_unix_s for x in rows], dtype=float),
            np.asarray([[x.enu_e_m, x.enu_n_m, x.enu_u_m] for x in rows], dtype=float))


def _interp(t: np.ndarray, source_t: np.ndarray, source_xyz: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(t, source_t, source_xyz[:, j]) for j in range(3)])


def _condition_midpoint(condition_id: str, source_t: np.ndarray) -> float:
    meta = json.loads((CACHE / f"{condition_id}.json").read_text(encoding="utf-8"))
    bag_start = float(meta["frames"][0]["bag_time"])
    bag_end = float(meta["frames"][-1]["bag_time"])
    return 0.5 * ((source_t[0] + source_t[-1]) - (bag_start + bag_end))


def _offset_bounds(candidate: dict, source_t: np.ndarray, midpoint: float) -> tuple[float, float] | None:
    margin = float(PROTOCOL["offset_search_margin_from_interval_midpoint_s"])
    lo = max(midpoint - margin, float(source_t[0] - candidate["times"][0]))
    hi = min(midpoint + margin, float(source_t[-1] - candidate["times"][-1]))
    return (lo, hi) if hi >= lo else None


def _metrics(candidate: dict, source_t: np.ndarray, source_xyz: np.ndarray,
             offset: float, rotation: np.ndarray, translation: np.ndarray) -> dict:
    query = candidate["times"] + offset
    enu = _interp(query, source_t, source_xyz)
    predicted = enu @ rotation.T + translation
    observed = candidate["positions"]
    residual = observed - predicted
    position_rmse = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    range_residual = np.linalg.norm(observed, axis=1) - np.linalg.norm(predicted, axis=1)
    range_rmse = float(np.sqrt(np.mean(range_residual ** 2)))
    dt = np.diff(candidate["times"])
    good = dt > 0
    if np.any(good):
        vo = np.diff(observed, axis=0)[good] / dt[good, None]
        vp = np.diff(predicted, axis=0)[good] / dt[good, None]
        denom = np.linalg.norm(vo, axis=1) * np.linalg.norm(vp, axis=1)
        cosines = np.divide(np.sum(vo * vp, axis=1), denom, out=np.zeros_like(denom), where=denom > 1e-9)
        direction_deg = float(np.median(np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))))
        predicted_rr = np.diff(np.linalg.norm(predicted, axis=1))[good] / dt[good]
        observed_rr = np.diff(np.linalg.norm(observed, axis=1))[good] / dt[good]
        rv = 0.5 * (candidate["nodes"][:-1, 7][good] + candidate["nodes"][1:, 7][good])
        doppler_pred_rmse = float(np.sqrt(np.mean((-rv - predicted_rr) ** 2)))
        doppler_lidar_rmse = float(np.sqrt(np.mean((-rv - observed_rr) ** 2)))
    else:
        direction_deg = doppler_pred_rmse = doppler_lidar_rmse = float("nan")
    gnss_path = float(np.linalg.norm(np.diff(source_xyz, axis=0), axis=1).sum())
    path_fraction = float(candidate["summary"]["path_length_m"] / gnss_path) if gnss_path > 0 else 0.0
    composite = (position_rmse / PROTOCOL["closure_position_rmse_m"] +
                 range_rmse / PROTOCOL["closure_range_rmse_m"] +
                 direction_deg / PROTOCOL["closure_direction_median_deg"] +
                 doppler_pred_rmse / PROTOCOL["closure_doppler_rmse_mps"])
    return {
        "position_rmse_m": position_rmse, "range_rmse_m": range_rmse,
        "direction_median_deg": direction_deg,
        "doppler_vs_gnss_rmse_mps": doppler_pred_rmse,
        "doppler_vs_lidar_rmse_mps": doppler_lidar_rmse,
        "path_fraction_of_gnss": path_fraction, "composite": float(composite),
        "predicted": predicted, "observed": observed, "query_gps_time": query,
    }


def _shape_select(condition_id: str, condition_cn: str, candidates: list[dict], gnss: dict,
                  yaw0: float) -> tuple[dict | None, list[dict]]:
    source_t, source_xyz = _gnss_arrays(condition_cn, gnss)
    gnss_path = float(np.linalg.norm(np.diff(source_xyz, axis=0), axis=1).sum())
    midpoint = _condition_midpoint(condition_id, source_t)
    rot = _rotation(yaw0)
    scored = []
    for candidate in candidates:
        if candidate["summary"]["observations"] < 10 or candidate["summary"]["path_length_m"] < 15.0:
            continue
        if gnss_path <= 0 or candidate["summary"]["path_length_m"] / gnss_path < PROTOCOL["closure_min_path_fraction"]:
            continue
        bounds = _offset_bounds(candidate, source_t, midpoint)
        if bounds is None:
            continue
        best = None
        for offset in np.arange(bounds[0], bounds[1] + 1e-9, 0.25):
            enu = _interp(candidate["times"] + offset, source_t, source_xyz)
            base = enu @ rot.T
            translation = np.median(candidate["positions"] - base, axis=0)
            m = _metrics(candidate, source_t, source_xyz, float(offset), rot, translation)
            # Centered shape score: translation is not accepted as a per-bag
            # extrinsic; it is used only for calibration-candidate assignment.
            score = m["position_rmse_m"] + 0.10 * m["direction_median_deg"]
            row = {"candidate_id": candidate["candidate_id"], "candidate": candidate,
                   "offset_s": float(offset), "shape_score": float(score),
                   "shape_position_rmse_m": m["position_rmse_m"],
                   "shape_direction_median_deg": m["direction_median_deg"]}
            if best is None or row["shape_score"] < best["shape_score"]:
                best = row
        if best is not None:
            scored.append(best)
    scored.sort(key=lambda x: (x["shape_score"], x["candidate"]["lidar_rank"]))
    return (scored[0] if scored else None), scored


def _fit_model(selected: dict[str, dict], condition_cn: dict[str, str], gnss: dict,
               model: str, initial_offsets: dict[str, float]) -> dict:
    ids = sorted(selected)
    full = model == "full_6dof"
    n_pose = 6 if full else 4
    # yaw[,pitch,roll], tx,ty,tz, then one offset per condition
    yaw0 = math.radians(173.2016645969914)
    if full:
        x0 = [yaw0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lower = [math.radians(140), math.radians(-15), math.radians(-15), -50, -50, -50]
        upper = [math.radians(210), math.radians(15), math.radians(15), 50, 50, 50]
    else:
        x0 = [yaw0, 0.0, 0.0, 0.0]
        lower = [math.radians(140), -50, -50, -50]
        upper = [math.radians(210), 50, 50, 50]
    bounds_by_id = {}
    for cid in ids:
        st, _ = _gnss_arrays(condition_cn[cid], gnss)
        midpoint = _condition_midpoint(cid, st)
        b = _offset_bounds(selected[cid], st, midpoint)
        if b is None:
            raise RuntimeError(f"no offset overlap for {cid}")
        bounds_by_id[cid] = b
        x0.append(float(np.clip(initial_offsets[cid], b[0], b[1])))
        lower.append(b[0]); upper.append(b[1])

    # Translation initialization from all selected trajectory points.
    rot0 = _rotation(yaw0)
    translations = []
    for cid in ids:
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        gnss_path = float(np.linalg.norm(np.diff(sx, axis=0), axis=1).sum())
        base = _interp(selected[cid]["times"] + x0[n_pose + ids.index(cid)], st, sx) @ rot0.T
        translations.append(selected[cid]["positions"] - base)
    t0 = np.median(np.concatenate(translations, axis=0), axis=0)
    if full:
        x0[3:6] = t0.tolist()
    else:
        x0[1:4] = t0.tolist()

    def unpack(x):
        if full:
            rot = _rotation(x[0], x[1], x[2]); trans = np.asarray(x[3:6]); start = 6
        else:
            rot = _rotation(x[0]); trans = np.asarray(x[1:4]); start = 4
        return rot, trans, {cid: float(x[start + i]) for i, cid in enumerate(ids)}

    def residuals(x):
        rot, trans, offsets = unpack(x)
        out = []
        for cid in ids:
            cand = selected[cid]
            st, sx = _gnss_arrays(condition_cn[cid], gnss)
            pred = _interp(cand["times"] + offsets[cid], st, sx) @ rot.T + trans
            obs = cand["positions"]
            out.extend(((obs - pred) / PROTOCOL["position_huber_scale_m"]).ravel())
            dt = np.diff(cand["times"]); good = dt > 0
            if np.any(good):
                rr = np.diff(np.linalg.norm(pred, axis=1))[good] / dt[good]
                rv = 0.5 * (cand["nodes"][:-1, 7][good] + cand["nodes"][1:, 7][good])
                out.extend((0.5 * (-rv - rr) / PROTOCOL["closure_doppler_rmse_mps"]).tolist())
        return np.asarray(out)

    result = least_squares(residuals, np.asarray(x0), bounds=(np.asarray(lower), np.asarray(upper)),
                           loss="soft_l1", f_scale=1.0, max_nfev=5000)
    rot, trans, offsets = unpack(result.x)
    if full:
        yaw, pitch, roll = result.x[:3]
    else:
        yaw, pitch, roll = result.x[0], 0.0, 0.0
    per_condition = {}
    for cid in ids:
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        per_condition[cid] = _metrics(selected[cid], st, sx, offsets[cid], rot, trans)
    return {"model": model, "success": bool(result.success), "cost": float(result.cost),
            "yaw_deg": math.degrees(yaw), "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll), "translation_m": trans.tolist(),
            "offsets_s": offsets, "rotation_matrix": rot.tolist(),
            "per_condition": per_condition, "optimizer_message": result.message}


def _jsonable_metrics(m: dict) -> dict:
    return {k: v for k, v in m.items() if k not in {"predicted", "observed", "query_gps_time"}}


def run_alignment() -> dict:
    gnss, _ = parse_workbook(WORKBOOK)
    condition_cn = {x[0]: x[1] for x in CONDITION_SPECS}
    calibration_ids = list(PROTOCOL["calibration_conditions"])
    validation_ids = list(PROTOCOL["validation_conditions"])
    yaw0 = math.radians(173.2016645969914)
    selected, selection_audit, initial_offsets = {}, {}, {}
    for cid in calibration_ids:
        best, scored = _shape_select(cid, condition_cn[cid], _load_tracks(cid), gnss, yaw0)
        selection_audit[cid] = [{k: v for k, v in row.items() if k != "candidate"} for row in scored]
        if best is not None:
            selected[cid] = best["candidate"]
            initial_offsets[cid] = best["offset_s"]
            print(f"align calibration select {cid}: {best['candidate_id']} shape={best['shape_score']:.3f}", flush=True)
        else:
            print(f"align calibration select {cid}: no eligible trajectory", flush=True)
    if len(selected) < 3:
        raise RuntimeError("fewer than three calibration conditions have eligible LiDAR evidence")

    full_initial = _fit_model(selected, condition_cn, gnss, "full_6dof", initial_offsets)
    # One deterministic reassignment using the shared initial full-pose fit.
    rot0 = np.asarray(full_initial["rotation_matrix"]); trans0 = np.asarray(full_initial["translation_m"])
    for cid in list(selected):
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        gnss_path = float(np.linalg.norm(np.diff(sx, axis=0), axis=1).sum())
        midpoint = _condition_midpoint(cid, st)
        best = None
        for candidate in _load_tracks(cid):
            if candidate["summary"]["observations"] < 10 or candidate["summary"]["path_length_m"] < 15.0:
                continue
            if gnss_path <= 0 or candidate["summary"]["path_length_m"] / gnss_path < PROTOCOL["closure_min_path_fraction"]:
                continue
            bounds = _offset_bounds(candidate, st, midpoint)
            if bounds is None:
                continue
            for offset in np.arange(bounds[0], bounds[1] + 1e-9, 0.25):
                m = _metrics(candidate, st, sx, float(offset), rot0, trans0)
                if best is None or m["composite"] < best["metrics"]["composite"]:
                    best = {"candidate": candidate, "offset": float(offset), "metrics": m}
        if best is not None:
            selected[cid] = best["candidate"]; initial_offsets[cid] = best["offset"]
            print(f"align shared reselect {cid}: {best['candidate']['candidate_id']} composite={best['metrics']['composite']:.3f}", flush=True)

    planar = _fit_model(selected, condition_cn, gnss, "planar", initial_offsets)
    full = _fit_model(selected, condition_cn, gnss, "full_6dof", initial_offsets)
    leave_one_out = {}
    if len(selected) >= 4:
        for omitted in sorted(selected):
            subset = {cid: candidate for cid, candidate in selected.items() if cid != omitted}
            subset_offsets = {cid: initial_offsets[cid] for cid in subset}
            leave_one_out[omitted] = _fit_model(subset, condition_cn, gnss, "full_6dof", subset_offsets)
    validation = {}
    for cid in validation_ids:
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        midpoint = _condition_midpoint(cid, st)
        candidates = _load_tracks(cid)
        rows = []
        for candidate in candidates:
            bounds = _offset_bounds(candidate, st, midpoint)
            if bounds is None:
                continue
            best = None
            for offset in np.arange(bounds[0], bounds[1] + 1e-9, 0.25):
                m = _metrics(candidate, st, sx, float(offset), np.asarray(full["rotation_matrix"]),
                             np.asarray(full["translation_m"]))
                if best is None or m["composite"] < best["metrics"]["composite"]:
                    best = {"candidate_id": candidate["candidate_id"], "lidar_rank": candidate["lidar_rank"],
                            "lidar_score": candidate["lidar_score"], "offset_s": float(offset),
                            "summary": candidate["summary"], "metrics": m, "candidate": candidate}
            if best is not None:
                rows.append(best)
        rows.sort(key=lambda x: x["metrics"]["composite"])
        validation[cid] = rows
        if rows:
            print(f"align validation {cid}: {rows[0]['candidate_id']} composite={rows[0]['metrics']['composite']:.3f}", flush=True)

    payload = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "protocol_sha256": sha256(OUT / "protocol.json"),
        "calibration_selected": {cid: {"candidate_id": c["candidate_id"], "lidar_rank": c["lidar_rank"],
                                          "summary": c["summary"]} for cid, c in selected.items()},
        "calibration_failures": [cid for cid in calibration_ids if cid not in selected],
        "selection_audit": selection_audit,
        "planar": {**{k: v for k, v in planar.items() if k != "per_condition"},
                   "per_condition": {cid: _jsonable_metrics(m) for cid, m in planar["per_condition"].items()}},
        "full_6dof": {**{k: v for k, v in full.items() if k != "per_condition"},
                      "per_condition": {cid: _jsonable_metrics(m) for cid, m in full["per_condition"].items()}},
        "leave_one_out": {
            omitted: {**{k: v for k, v in model.items() if k != "per_condition"},
                      "per_condition": {cid: _jsonable_metrics(m) for cid, m in model["per_condition"].items()}}
            for omitted, model in leave_one_out.items()
        },
        "validation": {cid: [{**{k: v for k, v in row.items() if k not in {"candidate", "metrics"}},
                               "metrics": _jsonable_metrics(row["metrics"])} for row in rows]
                       for cid, rows in validation.items()},
    }
    (OUT / "alignment.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_machine_readable(payload, selected, planar, full, validation)
    _write_plots(gnss, condition_cn, selected, full, validation)
    return payload


def _closure_pass(metrics: dict) -> bool:
    return (metrics["position_rmse_m"] <= PROTOCOL["closure_position_rmse_m"] and
            metrics["range_rmse_m"] <= PROTOCOL["closure_range_rmse_m"] and
            metrics["direction_median_deg"] <= PROTOCOL["closure_direction_median_deg"] and
            metrics["doppler_vs_gnss_rmse_mps"] <= PROTOCOL["closure_doppler_rmse_mps"] and
            metrics["path_fraction_of_gnss"] >= PROTOCOL["closure_min_path_fraction"])


def _write_machine_readable(payload, selected, planar, full, validation) -> None:
    summary_rows, trajectory_rows, alignment_rows = [], [], []
    for spec in CONDITION_SPECS:
        cid = spec[0]
        for tr in _load_tracks(cid):
            summary_rows.append({"condition_id": cid, "candidate_id": tr["candidate_id"],
                                 "lidar_rank": tr["lidar_rank"], "lidar_score": tr["lidar_score"],
                                 **tr["summary"]})
            for fi, t, p, node in zip(tr["frames"], tr["times"], tr["positions"], tr["nodes"]):
                trajectory_rows.append({"condition_id": cid, "candidate_id": tr["candidate_id"],
                                        "lidar_rank": tr["lidar_rank"], "frame_index": int(fi),
                                        "bag_time_unix_s": t, "x_m": p[0], "y_m": p[1], "z_m": p[2],
                                        "point_count": node[3], "voxel_count": node[4], "extent_m": node[5],
                                        "intensity_mean": node[6], "radial_velocity_mean_mps": node[7],
                                        "voxel_persistence_fraction": node[8]})
    for model_name, model in (("planar", planar), ("full_6dof", full)):
        for cid, metrics in model["per_condition"].items():
            alignment_rows.append({"condition_id": cid, "split": "calibration", "model": model_name,
                                   "candidate_id": selected[cid]["candidate_id"],
                                   "lidar_rank": selected[cid]["lidar_rank"],
                                   "time_offset_s": model["offsets_s"][cid],
                                   **_jsonable_metrics(metrics), "closure_pass": _closure_pass(metrics)})
    for cid, rows in validation.items():
        if rows:
            row = rows[0]
            alignment_rows.append({"condition_id": cid, "split": "validation", "model": "full_6dof_frozen",
                                   "candidate_id": row["candidate_id"], "lidar_rank": row["lidar_rank"],
                                   "time_offset_s": row["offset_s"], **_jsonable_metrics(row["metrics"]),
                                   "closure_pass": _closure_pass(row["metrics"])})
        else:
            alignment_rows.append({"condition_id": cid, "split": "validation", "model": "full_6dof_frozen",
                                   "candidate_id": "", "lidar_rank": "", "time_offset_s": "",
                                   "closure_pass": False})
    condition_cn = {x[0]: x[1] for x in CONDITION_SPECS}
    gnss, _ = parse_workbook(WORKBOOK)
    profile_rows, offset_rows = [], []
    full_rot = np.asarray(full["rotation_matrix"]); full_trans = np.asarray(full["translation_m"])
    planar_rot = np.asarray(planar["rotation_matrix"]); planar_trans = np.asarray(planar["translation_m"])
    chosen = {cid: (candidate, "calibration") for cid, candidate in selected.items()}
    chosen.update({cid: (rows[0]["candidate"], "validation") for cid, rows in validation.items() if rows})
    for cid, (candidate, split) in chosen.items():
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        midpoint = _condition_midpoint(cid, st)
        bounds = _offset_bounds(candidate, st, midpoint)
        if bounds is None:
            continue
        model_profiles = []
        for model_name, rot, trans in (("full_6dof_frozen", full_rot, full_trans),
                                       ("planar_frozen", planar_rot, planar_trans)):
            rows = []
            for offset in np.arange(bounds[0], bounds[1] + 1e-9, 0.25):
                m = _metrics(candidate, st, sx, float(offset), rot, trans)
                row = {"condition_id": cid, "split": split, "model": model_name,
                       "candidate_id": candidate["candidate_id"], "lidar_rank": candidate["lidar_rank"],
                       "time_offset_s": float(offset), **_jsonable_metrics(m)}
                profile_rows.append(row); rows.append(row)
            best = min(rows, key=lambda x: x["composite"])
            near = [x for x in rows if x["composite"] <= best["composite"] + 0.5 and
                    x["position_rmse_m"] <= best["position_rmse_m"] + 2.0]
            values = np.asarray([x["composite"] for x in rows])
            local_minima = int(np.sum((values[1:-1] < values[:-2]) & (values[1:-1] < values[2:])))
            offset_rows.append({
                "condition_id": cid, "split": split, "model": model_name,
                "candidate_id": candidate["candidate_id"], "lidar_rank": candidate["lidar_rank"],
                "search_low_s": bounds[0], "search_high_s": bounds[1],
                "best_offset_s": best["time_offset_s"], "best_composite": best["composite"],
                "best_position_rmse_m": best["position_rmse_m"], "best_range_rmse_m": best["range_rmse_m"],
                "near_best_low_s": min(x["time_offset_s"] for x in near),
                "near_best_high_s": max(x["time_offset_s"] for x in near),
                "near_best_width_s": max(x["time_offset_s"] for x in near) - min(x["time_offset_s"] for x in near),
                "boundary_solution": (abs(best["time_offset_s"] - bounds[0]) <= 0.26 or
                                      abs(best["time_offset_s"] - bounds[1]) <= 0.26),
                "interior_local_minima": local_minima,
            })
            if split == "validation" and model_name == "planar_frozen":
                alignment_rows.append({"condition_id": cid, "split": split, "model": model_name,
                                       "candidate_id": candidate["candidate_id"],
                                       "lidar_rank": candidate["lidar_rank"],
                                       "time_offset_s": best["time_offset_s"],
                                       **{k: best[k] for k in _jsonable_metrics(_metrics(
                                           candidate, st, sx, best["time_offset_s"], rot, trans))},
                                       "closure_pass": _closure_pass(best)})
    sensitivity_rows = []
    for omitted, model in payload.get("leave_one_out", {}).items():
        sensitivity_rows.append({"omitted_condition": omitted, "yaw_deg": model["yaw_deg"],
                                 "pitch_deg": model["pitch_deg"], "roll_deg": model["roll_deg"],
                                 "translation_x_m": model["translation_m"][0],
                                 "translation_y_m": model["translation_m"][1],
                                 "translation_z_m": model["translation_m"][2], "cost": model["cost"]})
    for path, rows in ((OUT / "candidate_summary.csv", summary_rows),
                       (OUT / "lidar_trajectories.csv", trajectory_rows),
                       (OUT / "alignment_results.csv", alignment_rows),
                       (OUT / "offset_profiles.csv", profile_rows),
                       (OUT / "offset_diagnostics.csv", offset_rows),
                       (OUT / "extrinsic_sensitivity.csv", sensitivity_rows)):
        fields = sorted({key for row in rows for key in row})
        with path.open("w", newline="", encoding="utf-8-sig") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields); writer.writeheader(); writer.writerows(rows)


def _write_candidate_plots() -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    for spec in CONDITION_SPECS:
        cid = spec[0]
        tracks = _load_tracks(cid)[:5]
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for tr in tracks:
            p = tr["positions"]
            label = f"rank {tr['lidar_rank']}"
            axes[0].plot(p[:, 0], p[:, 1], "o-", ms=3, label=label)
            axes[1].plot(tr["frames"], np.linalg.norm(p, axis=1), "o-", ms=3, label=label)
        axes[0].set(xlabel="x [m]", ylabel="y [m]", title="LiDAR-only plan view")
        axes[0].axis("equal"); axes[0].legend(fontsize=7)
        axes[1].set(xlabel="frame", ylabel="range [m]", title="LiDAR-only range")
        axes[1].legend(fontsize=7)
        fig.suptitle(cid); fig.tight_layout()
        fig.savefig(PLOTS / f"{cid}_lidar_candidates.png", dpi=160); plt.close(fig)


def _write_plots(gnss, condition_cn, selected, full, validation) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    _write_candidate_plots()
    rot = np.asarray(full["rotation_matrix"]); trans = np.asarray(full["translation_m"])
    for spec in CONDITION_SPECS:
        cid = spec[0]
        if cid in selected:
            candidate = selected[cid]; offset = full["offsets_s"][cid]
        elif validation.get(cid):
            candidate = validation[cid][0]["candidate"]; offset = validation[cid][0]["offset_s"]
        else:
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.axis("off")
            ax.text(0.5, 0.5, "No eligible trajectory for GNSS alignment",
                    ha="center", va="center", fontsize=13)
            ax.set_title(cid)
            fig.tight_layout(); fig.savefig(PLOTS / f"{cid}_alignment.png", dpi=160); plt.close(fig)
            continue
        st, sx = _gnss_arrays(condition_cn[cid], gnss)
        m = _metrics(candidate, st, sx, offset, rot, trans)
        obs, pred = m["observed"], m["predicted"]
        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        axes[0].plot(obs[:, 0], obs[:, 1], "o-", ms=3, label="LiDAR")
        axes[0].plot(pred[:, 0], pred[:, 1], "x--", ms=3, label="GNSS transformed")
        axes[0].set(xlabel="x [m]", ylabel="y [m]", title="plan view"); axes[0].axis("equal"); axes[0].legend()
        axes[1].plot(np.linalg.norm(obs, axis=1), label="LiDAR")
        axes[1].plot(np.linalg.norm(pred, axis=1), label="GNSS transformed")
        axes[1].set(xlabel="trajectory observation", ylabel="range [m]", title="range"); axes[1].legend()
        axes[2].plot(obs[:, 2], label="LiDAR")
        axes[2].plot(pred[:, 2], label="GNSS transformed")
        axes[2].set(xlabel="trajectory observation", ylabel="z [m]", title="height"); axes[2].legend()
        fig.suptitle(f"{cid}: {candidate['candidate_id']}")
        fig.tight_layout(); fig.savefig(PLOTS / f"{cid}_alignment.png", dpi=160); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", required=True, choices=("cache", "search", "align"))
    parser.add_argument("--condition", choices=[x[0] for x in CONDITION_SPECS])
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    write_protocol()
    if args.phase == "cache":
        cache_all(force=args.force, only=args.condition)
    elif args.phase == "search":
        search_all(only=args.condition)
    elif args.phase == "align":
        if args.condition:
            raise ValueError("--condition is not supported for joint alignment")
        run_alignment()


if __name__ == "__main__":
    main()
