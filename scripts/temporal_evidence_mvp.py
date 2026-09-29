#!/usr/bin/env python3
"""Lightweight soft temporal evidence study on frozen research_dev_v0 labels.

The historical detector/tracker is never modified.  A-prime recomputes the
legacy background/neighborhood/DBSCAN stages from the pre-stage raw-voxel
cache in provisional yaw-preserving leveled coordinates.  Soft accumulators
use singleton-capable candidate generation and real bag time deltas.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))
import sparse_uav_mvp as sparse


OUT = ROOT / "results" / "temporal_evidence_mvp"
SOURCE_DEV = ROOT / "results" / "research_dev_v0"
RAW_VOXEL = ROOT / "results" / "calibration_closure" / "raw_voxel_cache"
ALIGNMENT = ROOT / "results" / "calibration_closure" / "alignment.json"

CONDITIONS = tuple(sparse.SELECTED)
THRESHOLDS = (0.50, 0.55, 0.60, 0.65, 0.68, 0.70, 0.75, 0.80, 0.85)

PROTOCOL = {
    "protocol_version": 1,
    "status": "FROZEN_DEV_SILVER_NOT_PAPER_GT",
    "random_seed": 20260929,
    "labels": "unchanged research_dev_v0/silver_labels.csv",
    "negative_windows": "NONE_RELIABLE",
    "negative_window_reason": (
        "all included bags are target-flight bags and LiDAR silver trajectories are fragmented; "
        "frames outside silver observations cannot be asserted target-absent"
    ),
    "input": {
        "source": "calibration_closure 1 m pre-stage raw-voxel cache",
        "intensity_mean_range_inclusive": [10.0, 250.0],
        "leveled_height_gt_m": 10.0,
        "level_transform": "q=Rz(yaw_full)@R_full.T@(p_lidar-t_full)",
        "note": "raw-voxel input is shared across new methods; no old background/neighborhood cache",
    },
    "legacy_corrected": {
        "grid_bounds_m": {"x": [0.0, 500.0], "y": [-150.0, 150.0], "z": [-5.0, 50.0]},
        "voxel_m": 1.0,
        "background_init_frames": 8,
        "background_probability_threshold": 0.05,
        "neighborhood_static_ratio_gt": 0.6,
        "dbscan_eps_m": 2.0,
        "dbscan_min_pts": 1,
    },
    "singleton_candidates": {
        "min_pts": 1,
        "range_adaptive_dbscan": "inherit frozen research_dev_v0 parameters",
        "geometry_validation": "inherit frozen research_dev_v0 parameters",
    },
    "association": {
        "real_bag_dt": True,
        "tau_min_m": 3.0,
        "vmax_mps": 25.0,
        "nearest_track_queries": 6,
        "velocity_change_scale_mps": 8.0,
        "preferred_motion_scale_mps": 1.0,
    },
    "soft_score": {
        "formula": "0.45*persistence + 0.35*motion_consistency + 0.20*compactness",
        "persistence": "1-exp(-0.5*sum(exp(-age/(window/2))))",
        "motion": "weighted mean of soft nonzero-speed and prediction/velocity consistency",
        "compactness": "exp(-extent/2m)",
        "formal_threshold": 0.68,
        "threshold_sweep_diagnostic_only": list(THRESHOLDS),
    },
    "windows": {
        "fixed_s": 4.0,
        "adaptive_far_or_sparse_s": 6.0,
        "adaptive_dense_or_fast_s": 3.0,
        "adaptive_default_s": 4.0,
        "far_range_m": 250.0,
        "sparse_raw_support_le": 5,
        "dense_raw_support_ge": 20,
        "fast_speed_ge_mps": 8.0,
    },
    "doppler": {
        "hard_gate": False,
        "sign": "-measured radial velocity",
        "residual_scale_mps": 4.0,
        "soft_adjustment": "base_score + 0.15*eta*(exp(-residual/4)-0.5)",
    },
    "formal_methods": [
        "A_prime_legacy_corrected",
        "B_khosravi_current",
        "fixed_soft",
        "adaptive_soft",
        "adaptive_soft_doppler",
    ],
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
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def level_transform() -> tuple[np.ndarray, np.ndarray]:
    full = json.loads(ALIGNMENT.read_text(encoding="utf-8"))["full_6dof"]
    r_full = np.asarray(full["rotation_matrix"], dtype=float)
    translation = np.asarray(full["translation_m"], dtype=float)
    transform = _rotation_z(math.radians(float(full["yaw_deg"]))) @ r_full.T
    return transform, translation


def lidar_to_level(points: np.ndarray) -> np.ndarray:
    transform, translation = level_transform()
    return (transform @ (points - translation).T).T


def level_to_lidar(points: np.ndarray) -> np.ndarray:
    transform, translation = level_transform()
    return (transform.T @ points.T).T + translation


def freeze() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    payload = {
        **PROTOCOL,
        "selected_conditions": list(CONDITIONS),
        "source_sha256": {
            "silver_labels": sha256(SOURCE_DEV / "silver_labels.csv"),
            "research_dev_protocol": sha256(SOURCE_DEV / "protocol.json"),
            "research_dev_addendum": sha256(SOURCE_DEV / "input_protocol_addendum.json"),
            "research_dev_script": sha256(ROOT / "scripts" / "sparse_uav_mvp.py"),
            "calibration_alignment": sha256(ALIGNMENT),
        },
    }
    path = OUT / "protocol.json"
    if path.exists():
        old = json.loads(path.read_text(encoding="utf-8"))
        if old != payload:
            raise RuntimeError("protocol.json differs; refusing silent retuning")
    else:
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    negative = {
        "status": "NO_RELIABLE_NEGATIVE_WINDOWS",
        "reason": PROTOCOL["negative_window_reason"],
        "false_alarm_rate_reported": False,
        "pr_curve_reported": False,
        "allowed_metrics": [
            "unmatched candidates per labeled positive frame",
            "all output candidates per LiDAR frame",
            "recall on frozen silver observations",
        ],
    }
    (OUT / "negative_window_audit.json").write_text(
        json.dumps(negative, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (OUT / "FROZEN").write_text(
        "temporal evidence parameters frozen before data comparison\n", encoding="utf-8"
    )
    print("frozen temporal_evidence_mvp protocol")


def load_labels() -> list[dict]:
    with (SOURCE_DEV / "silver_labels.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def load_leveled_frames(condition_id: str) -> tuple[list[np.ndarray], np.ndarray]:
    """Return qx,qy,qz,raw_support,intensity,radial_velocity after intensity/height."""
    meta = json.loads((RAW_VOXEL / f"{condition_id}.json").read_text(encoding="utf-8"))
    with np.load(RAW_VOXEL / f"{condition_id}.npz") as z:
        values, offsets = z["values"].copy(), z["offsets"].copy()
    frames = []
    lo, hi = PROTOCOL["input"]["intensity_mean_range_inclusive"]
    for i in range(len(offsets)-1):
        a = values[offsets[i]:offsets[i+1]]
        q = lidar_to_level(a[:, 1:4])
        keep = ((a[:, 5] >= lo) & (a[:, 5] <= hi) &
                (q[:, 2] > PROTOCOL["input"]["leveled_height_gt_m"]))
        frames.append(np.column_stack([q[keep], a[keep, 4:7]]))
    times = np.asarray([float(x["bag_time"]) for x in meta["frames"]])
    return frames, times


class LegacyStageFilter:
    def __init__(self):
        cfg = PROTOCOL["legacy_corrected"]
        bounds = cfg["grid_bounds_m"]
        self.bounds = tuple((float(bounds[k][0]), float(bounds[k][1])) for k in ("x", "y", "z"))
        self.voxel = float(cfg["voxel_m"])
        self.shape = tuple(int(math.ceil((hi-lo)/self.voxel)) for lo, hi in self.bounds)
        self.counts = np.zeros(int(np.prod(self.shape)), dtype=np.int32)
        self.frame_count = 0

    def indices(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        low = np.asarray([x[0] for x in self.bounds])
        q = np.floor((points-low)/self.voxel).astype(np.int64)
        valid = np.all((q >= 0) & (q < np.asarray(self.shape)), axis=1)
        idx = np.full(len(points), -1, dtype=np.int64)
        idx[valid] = q[valid, 0]*self.shape[1]*self.shape[2] + q[valid, 1]*self.shape[2] + q[valid, 2]
        return idx, valid, q

    def neighborhood(self, dynamic: np.ndarray, idx: np.ndarray, q: np.ndarray) -> np.ndarray:
        if not np.any(dynamic):
            return dynamic.copy()
        rows = np.flatnonzero(dynamic)
        dyn_idx = idx[dynamic]
        deltas = np.asarray([(x, y, z) for x in (-1, 0, 1)
                            for y in (-1, 0, 1) for z in (-1, 0, 1)], dtype=np.int64)
        nynz = self.shape[1]*self.shape[2]
        offsets = deltas[:, 0]*nynz + deltas[:, 1]*self.shape[2] + deltas[:, 2]
        neighbors = dyn_idx[:, None] + offsets[None, :]
        coords = q[dynamic][:, None, :] + deltas[None, :, :]
        inbound = np.all((coords >= 0) & (coords < np.asarray(self.shape)), axis=2)
        neighbors = np.clip(neighbors, 0, len(self.counts)-1)
        occupied = (self.counts[neighbors] > 0) & inbound
        dynamic_neighbor = np.isin(neighbors.ravel(), dyn_idx).reshape(neighbors.shape) & inbound
        occupied_n = occupied.sum(axis=1)
        static_n = (occupied & ~dynamic_neighbor).sum(axis=1)
        ratio = np.divide(static_n, occupied_n, out=np.zeros_like(static_n, dtype=float), where=occupied_n > 0)
        refined = dynamic.copy()
        refined[rows[(occupied_n > 0) & (ratio > PROTOCOL["legacy_corrected"]["neighborhood_static_ratio_gt"])]] = False
        return refined

    def process(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.frame_count += 1
        idx, valid, q = self.indices(points)
        if self.frame_count <= PROTOCOL["legacy_corrected"]["background_init_frames"]:
            pre = np.zeros(len(points), dtype=bool)
            post = pre.copy()
            static = valid
        else:
            prob = np.zeros(len(points), dtype=float)
            prob[valid] = self.counts[idx[valid]]/self.frame_count
            pre = (prob < PROTOCOL["legacy_corrected"]["background_probability_threshold"]) & valid
            post = self.neighborhood(pre, idx, q)
            static = (~post) & valid
        self.counts[np.unique(idx[static & (idx >= 0)])] += 1
        return pre, post


def legacy_corrected_candidates(condition_id: str, labels: list[dict]) -> tuple[list[list[dict]], np.ndarray, list[dict], float]:
    frames, times = load_leveled_frames(condition_id)
    stage = LegacyStageFilter()
    candidates: list[list[dict]] = []
    stage_rows: list[dict] = []
    label_lookup = {int(x["frame_index"]): x for x in labels if x["condition_id"] == condition_id}
    started = time.perf_counter()
    for fi, values in enumerate(frames):
        pre, post = stage.process(values[:, :3]) if len(values) else (np.zeros(0, bool), np.zeros(0, bool))
        selected = values[post]
        current = []
        if len(selected):
            cfg = PROTOCOL["legacy_corrected"]
            lab = DBSCAN(eps=cfg["dbscan_eps_m"], min_samples=cfg["dbscan_min_pts"]).fit_predict(selected[:, :3])
            for value in np.unique(lab):
                c = selected[lab == value]
                center_q = np.average(c[:, :3], axis=0, weights=c[:, 3])
                center = level_to_lidar(center_q[None, :])[0]
                current.append({
                    "center": center, "raw_points": int(round(c[:, 3].sum())), "voxels": len(c),
                    "extent": float(np.max(np.ptp(c[:, :3], axis=0))),
                    "intensity": float(np.average(c[:, 4], weights=c[:, 3])),
                    "radial_velocity": float(np.average(c[:, 5], weights=c[:, 3])),
                    "evidence_score": math.nan,
                })
        candidates.append(current)
        if fi in label_lookup:
            row = label_lookup[fi]
            center = np.asarray([float(row["center_x_m"]), float(row["center_y_m"]), float(row["center_z_m"])])
            center_q = lidar_to_level(center[None, :])[0]
            dist2 = np.sum((values[:, :3]-center_q)**2, axis=1) if len(values) else np.empty(0)
            target = dist2 <= sparse.P["label_radius_m"]**2
            stage_rows.append({
                "condition_id": condition_id, "frame_index": fi,
                "input_target_raw_support": int(round(values[target, 3].sum())) if len(values) else 0,
                "background_target_raw_support": int(round(values[pre & target, 3].sum())) if len(values) else 0,
                "neighborhood_target_raw_support": int(round(values[post & target, 3].sum())) if len(values) else 0,
                "dbscan_target_raw_support": int(round(values[post & target, 3].sum())) if len(values) else 0,
                "background_frame_count": stage.frame_count,
            })
    return candidates, times, stage_rows, time.perf_counter()-started


def adaptive_window(candidate: dict, speed: float) -> float:
    cfg = PROTOCOL["windows"]
    distance = float(np.linalg.norm(candidate["center"]))
    if distance >= cfg["far_range_m"] or candidate["raw_points"] <= cfg["sparse_raw_support_le"]:
        return float(cfg["adaptive_far_or_sparse_s"])
    if candidate["raw_points"] >= cfg["dense_raw_support_ge"] or speed >= cfg["fast_speed_ge_mps"]:
        return float(cfg["adaptive_dense_or_fast_s"])
    return float(cfg["adaptive_default_s"])


def _score_track(track: dict, now: float, window_s: float, doppler: bool) -> tuple[float, dict]:
    ages = now-np.asarray(track["times"], dtype=float)
    keep = ages <= window_s+1e-9
    ages = ages[keep]
    weights = np.exp(-ages/max(window_s/2.0, 1e-6))
    persistence = 1.0-math.exp(-0.5*float(weights.sum()))
    motion = float(np.average(np.asarray(track["motion_quality"])[keep], weights=weights))
    compactness = float(np.average(np.asarray(track["compact_quality"])[keep], weights=weights))
    base_score = 0.45*persistence + 0.35*motion + 0.20*compactness
    residual = float(track["doppler_residual"][-1])
    eta = float(track["eta"][-1])
    adjustment = 0.0
    if doppler and math.isfinite(residual) and math.isfinite(eta):
        quality = math.exp(-residual/PROTOCOL["doppler"]["residual_scale_mps"])
        adjustment = 0.15*eta*(quality-0.5)
    score = float(np.clip(base_score+adjustment, 0.0, 1.0))
    return score, {
        "persistence_score": persistence, "motion_score": motion,
        "compactness_score": compactness, "doppler_adjustment": adjustment,
        "doppler_residual_mps": residual, "eta": eta, "window_s": window_s,
    }


def soft_accumulate(candidates: list[list[dict]], times: np.ndarray, adaptive: bool,
                    doppler: bool) -> tuple[list[list[dict]], list[list[dict]], float]:
    """Return thresholded outputs and all scored current-frame candidates."""
    started = time.perf_counter()
    tracks: dict[int, dict] = {}
    next_id = 0
    accepted_frames: list[list[dict]] = []
    scored_frames: list[list[dict]] = []
    for fi, source in enumerate(candidates):
        current = [dict(x) for x in source]
        now = float(times[fi])
        active_ids = []
        for tid, tr in tracks.items():
            gap = now-tr["times"][-1]
            permitted = tr["window_s"]
            if gap <= permitted+1e-9:
                active_ids.append(tid)
        options = []
        if active_ids and current:
            predictions, gates = [], []
            for tid in active_ids:
                tr = tracks[tid]
                dt = now-tr["times"][-1]
                if len(tr["positions"]) >= 2:
                    prev_dt = tr["times"][-1]-tr["times"][-2]
                    velocity = ((tr["positions"][-1]-tr["positions"][-2])/prev_dt
                                if prev_dt > 1e-6 else np.zeros(3))
                else:
                    velocity = np.zeros(3)
                predictions.append(tr["positions"][-1]+velocity*max(dt, 0.0))
                gates.append(max(PROTOCOL["association"]["tau_min_m"],
                                 PROTOCOL["association"]["vmax_mps"]*max(dt, 0.0)))
            tree = cKDTree(np.asarray(predictions))
            k = min(PROTOCOL["association"]["nearest_track_queries"], len(predictions))
            for ci, candidate in enumerate(current):
                distances, indices = tree.query(candidate["center"], k=k)
                for distance, ai in zip(np.atleast_1d(distances), np.atleast_1d(indices)):
                    ai = int(ai); tid = active_ids[ai]
                    if float(distance) > gates[ai]:
                        continue
                    tr = tracks[tid]
                    dt = now-tr["times"][-1]
                    implied = ((candidate["center"]-tr["positions"][-1])/dt
                               if dt > 1e-6 else np.zeros(3))
                    speed = float(np.linalg.norm(implied))
                    if speed > PROTOCOL["association"]["vmax_mps"]:
                        continue
                    if len(tr["positions"]) >= 2:
                        prev_dt = tr["times"][-1]-tr["times"][-2]
                        prev_v = ((tr["positions"][-1]-tr["positions"][-2])/prev_dt
                                  if prev_dt > 1e-6 else np.zeros(3))
                        velocity_change = float(np.linalg.norm(implied-prev_v))
                    else:
                        velocity_change = 0.0
                    cost = (float(distance)/max(gates[ai], 1e-6) +
                            0.25*velocity_change/PROTOCOL["association"]["velocity_change_scale_mps"])
                    options.append((cost, ci, tid, implied, float(distance), velocity_change))
        assigned_c, assigned_t, current_track = set(), set(), {}
        association = {}
        for cost, ci, tid, implied, residual_position, velocity_change in sorted(options, key=lambda x: x[0]):
            if ci in assigned_c or tid in assigned_t:
                continue
            assigned_c.add(ci); assigned_t.add(tid); current_track[ci] = tid
            association[ci] = (implied, residual_position, velocity_change, cost)
        for ci, candidate in enumerate(current):
            if ci not in current_track:
                tid = next_id; next_id += 1
                speed = 0.0
                window_s = adaptive_window(candidate, speed) if adaptive else PROTOCOL["windows"]["fixed_s"]
                tracks[tid] = {
                    "times": [now], "positions": [candidate["center"]], "frames": [fi],
                    "motion_quality": [0.5],
                    "compact_quality": [math.exp(-candidate["extent"]/2.0)],
                    "doppler_residual": [math.nan], "eta": [math.nan], "window_s": window_s,
                }
                current_track[ci] = tid
            else:
                tid = current_track[ci]
                tr = tracks[tid]
                implied, position_residual, velocity_change, _ = association[ci]
                speed = float(np.linalg.norm(implied))
                preferred_motion = 1.0-math.exp(-speed/PROTOCOL["association"]["preferred_motion_scale_mps"])
                if len(tr["positions"]) >= 2:
                    prediction_quality = math.exp(-position_residual/3.0)
                    velocity_quality = math.exp(-velocity_change/PROTOCOL["association"]["velocity_change_scale_mps"])
                    motion_quality = preferred_motion*prediction_quality*velocity_quality
                else:
                    motion_quality = preferred_motion
                er = candidate["center"]/max(float(np.linalg.norm(candidate["center"])), 1e-9)
                expected = float(np.dot(er, implied))
                residual = abs(-float(candidate["radial_velocity"])-expected)
                eta = abs(expected)/(speed+1e-9)
                window_s = adaptive_window(candidate, speed) if adaptive else PROTOCOL["windows"]["fixed_s"]
                tr["times"].append(now); tr["positions"].append(candidate["center"]); tr["frames"].append(fi)
                tr["motion_quality"].append(motion_quality)
                tr["compact_quality"].append(math.exp(-candidate["extent"]/2.0))
                tr["doppler_residual"].append(residual); tr["eta"].append(eta); tr["window_s"] = window_s
            tr = tracks[current_track[ci]]
            score, parts = _score_track(tr, now, tr["window_s"], doppler)
            candidate.update(parts)
            candidate["evidence_score"] = score
            candidate["track_id"] = current_track[ci]
        scored_frames.append(current)
        accepted_frames.append([x for x in current if x["evidence_score"] >= PROTOCOL["soft_score"]["formal_threshold"]])
    return accepted_frames, scored_frames, time.perf_counter()-started


def _gap_metrics(found: list[bool]) -> tuple[int, int, int]:
    best = gaps = recovered = current = 0
    for i, value in enumerate(found):
        if value:
            if current and i-current-1 >= 0 and found[i-current-1]:
                gaps += 1; recovered += 1
            current = 0
        else:
            current += 1; best = max(best, current)
    if current and len(found)-current-1 >= 0 and found[len(found)-current-1]:
        gaps += 1
    return gaps, recovered, best


def evaluate(method: str, condition_id: str, outputs: list[list[dict]], labels: list[dict],
             runtime_s: float, input_candidates: int, threshold: float | None = None) -> tuple[dict, list[dict]]:
    truth = sorted([x for x in labels if x["condition_id"] == condition_id], key=lambda x: int(x["frame_index"]))
    found, errors, matched_ids, rows = [], [], defaultdict(set), []
    false_candidates = 0
    for label in truth:
        fi = int(label["frame_index"])
        gt = np.asarray([float(label["center_x_m"]), float(label["center_y_m"]), float(label["center_z_m"])])
        current = outputs[fi]
        distances = np.asarray([np.linalg.norm(x["center"]-gt) for x in current])
        matches = np.flatnonzero(distances <= sparse.P["label_radius_m"])
        detected = bool(len(matches)); found.append(detected)
        best = int(matches[np.argmin(distances[matches])]) if detected else -1
        if detected:
            errors.append(float(distances[best]))
            matched_ids[int(current[best]["track_id"])].add(fi)
        false_candidates += len(current)-int(detected)
        rows.append({
            "method": method, "condition_id": condition_id,
            "nominal_range_m": label["nominal_range_m"], "frame_index": fi,
            "detected": detected, "candidate_count": len(current),
            "unmatched_candidates": len(current)-int(detected),
            "center_error_m": float(distances[best]) if detected else "",
            "matched_track_id": int(current[best]["track_id"]) if detected else "",
            "matched_evidence_score": current[best].get("evidence_score", "") if detected else "",
        })
    gaps, recovered, longest = _gap_metrics(found)
    e = np.asarray(errors, dtype=float)
    output_count = sum(map(len, outputs))
    metric = {
        "method": method, "condition_id": condition_id,
        "nominal_range_m": int(truth[0]["nominal_range_m"]),
        "score_threshold": threshold if threshold is not None else "",
        "lidar_frames": len(outputs), "label_frames": len(truth), "detected_frames": int(sum(found)),
        "frame_recall": float(np.mean(found)),
        "unmatched_candidates_labeled_frames": false_candidates,
        "unmatched_candidates_per_labeled_frame": false_candidates/len(truth),
        "output_candidates_all_frames": output_count,
        "output_candidates_per_lidar_frame": output_count/len(outputs),
        "input_candidates_all_frames": input_candidates,
        "candidate_reduction_fraction": 1-output_count/input_candidates if input_candidates else "",
        "center_error_mean_m": float(np.mean(e)) if len(e) else "",
        "track_coverage_best_id": max((len(x) for x in matched_ids.values()), default=0)/len(truth),
        "longest_miss_silver_observations": longest,
        "gap_count": gaps, "recovered_gap_count": recovered,
        "runtime_s": runtime_s,
        "false_alarms_per_negative_frame": "UNKNOWN_NO_RELIABLE_NEGATIVE_WINDOWS",
    }
    return metric, rows


def aggregate_range(metrics: list[dict], method: str, distance: int) -> dict:
    rr = [x for x in metrics if x["method"] == method and int(x["nominal_range_m"]) == distance]
    labels = sum(int(x["label_frames"]) for x in rr)
    detected = sum(int(x["detected_frames"]) for x in rr)
    unmatched = sum(int(x["unmatched_candidates_labeled_frames"]) for x in rr)
    frames = sum(int(x["lidar_frames"]) for x in rr)
    outputs = sum(int(x["output_candidates_all_frames"]) for x in rr)
    inputs = sum(int(x["input_candidates_all_frames"]) for x in rr)
    return {
        "method": method, "condition_id": "ALL", "nominal_range_m": distance,
        "score_threshold": rr[0]["score_threshold"] if rr else "", "lidar_frames": frames,
        "label_frames": labels, "detected_frames": detected,
        "frame_recall": detected/labels if labels else "",
        "unmatched_candidates_labeled_frames": unmatched,
        "unmatched_candidates_per_labeled_frame": unmatched/labels if labels else "",
        "output_candidates_all_frames": outputs,
        "output_candidates_per_lidar_frame": outputs/frames if frames else "",
        "input_candidates_all_frames": inputs,
        "candidate_reduction_fraction": 1-outputs/inputs if inputs else "",
        "center_error_mean_m": "", "track_coverage_best_id": "",
        "longest_miss_silver_observations": max((int(x["longest_miss_silver_observations"]) for x in rr), default=0),
        "gap_count": sum(int(x["gap_count"]) for x in rr),
        "recovered_gap_count": sum(int(x["recovered_gap_count"]) for x in rr),
        "runtime_s": sum(float(x["runtime_s"]) for x in rr),
        "false_alarms_per_negative_frame": "UNKNOWN_NO_RELIABLE_NEGATIVE_WINDOWS",
    }


def candidate_rows(method: str, condition_id: str, outputs: list[list[dict]]) -> list[dict]:
    rows = []
    for fi, current in enumerate(outputs):
        for c in current:
            rows.append({
                "method": method, "condition_id": condition_id, "frame_index": fi,
                "track_id": c["track_id"], "center_x_m": c["center"][0],
                "center_y_m": c["center"][1], "center_z_m": c["center"][2],
                "raw_points": c["raw_points"], "voxels": c["voxels"], "extent_m": c["extent"],
                "radial_velocity_mps": c["radial_velocity"],
                "evidence_score": c.get("evidence_score", ""),
                "persistence_score": c.get("persistence_score", ""),
                "motion_score": c.get("motion_score", ""),
                "compactness_score": c.get("compactness_score", ""),
                "doppler_adjustment": c.get("doppler_adjustment", ""),
                "doppler_residual_mps": c.get("doppler_residual_mps", ""),
                "eta": c.get("eta", ""), "window_s": c.get("window_s", ""),
            })
    return rows


def run() -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before experiment")
    labels = load_labels()
    metrics, frame_rows, detections, stage_rows, tradeoff = [], [], [], [], []
    methods = ["A_prime_legacy_corrected", "B_khosravi_current", "fixed_soft",
               "adaptive_soft", "adaptive_soft_doppler"]
    for condition_id in CONDITIONS:
        legacy, legacy_times, legacy_stage, legacy_runtime = legacy_corrected_candidates(condition_id, labels)
        stage_rows.extend(legacy_stage)
        legacy_out, link_time = sparse.temporal_validate(
            copy.deepcopy(legacy), legacy_times, temporal=False, doppler="none", current_a=True
        )
        metric, rows = evaluate("A_prime_legacy_corrected", condition_id, legacy_out, labels,
                                legacy_runtime+link_time, sum(map(len, legacy)))
        metrics.append(metric); frame_rows.extend(rows); detections.extend(candidate_rows("A_prime_legacy_corrected", condition_id, legacy_out))

        source_frames, times = sparse._load_raw_voxel_frames(condition_id)
        b_candidates, b_cluster_time = sparse.cluster_frames(source_frames, min_pts=2, adaptive=True)
        b_out, b_temporal_time = sparse.temporal_validate(copy.deepcopy(b_candidates), times, temporal=True, doppler="none")
        metric, rows = evaluate("B_khosravi_current", condition_id, b_out, labels,
                                b_cluster_time+b_temporal_time, sum(map(len, b_candidates)))
        metrics.append(metric); frame_rows.extend(rows); detections.extend(candidate_rows("B_khosravi_current", condition_id, b_out))

        singleton, singleton_cluster_time = sparse.cluster_frames(source_frames, min_pts=1, adaptive=True)
        raw_out, raw_link_time = sparse.temporal_validate(copy.deepcopy(singleton), times, temporal=False, doppler="none", current_a=True)
        metric, _ = evaluate("minPts1_unaccumulated_DIAGNOSTIC", condition_id, raw_out, labels,
                             singleton_cluster_time+raw_link_time, sum(map(len, singleton)))
        metrics.append(metric)

        for method, adaptive, doppler in (
            ("fixed_soft", False, False),
            ("adaptive_soft", True, False),
            ("adaptive_soft_doppler", True, True),
        ):
            formal, scored, accumulate_time = soft_accumulate(singleton, times, adaptive=adaptive, doppler=doppler)
            total_runtime = singleton_cluster_time+accumulate_time
            metric, rows = evaluate(method, condition_id, formal, labels, total_runtime, sum(map(len, singleton)),
                                    threshold=PROTOCOL["soft_score"]["formal_threshold"])
            metrics.append(metric); frame_rows.extend(rows); detections.extend(candidate_rows(method, condition_id, formal))
            for threshold in THRESHOLDS:
                selected = [[x for x in frame if x["evidence_score"] >= threshold] for frame in scored]
                sweep_metric, _ = evaluate(method, condition_id, selected, labels, total_runtime,
                                           sum(map(len, singleton)), threshold=threshold)
                tradeoff.append(sweep_metric)
        print(f"temporal evidence {condition_id}: complete")

    for method in methods + ["minPts1_unaccumulated_DIAGNOSTIC"]:
        for distance in (100, 200, 300):
            metrics.append(aggregate_range(metrics, method, distance))
    write_csv(OUT / "benchmark_results.csv", metrics)
    write_csv(OUT / "frame_results.csv", frame_rows)
    write_csv(OUT / "candidate_detections.csv", detections)
    write_csv(OUT / "legacy_corrected_stage_survival.csv", stage_rows)
    write_csv(OUT / "score_tradeoff.csv", tradeoff)
    run_manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_head": _git_head(),
        "git_dirty": _git_dirty(),
        "script_sha256": sha256(Path(__file__)),
        "protocol_sha256": sha256(OUT / "protocol.json"),
        "labels_sha256": sha256(SOURCE_DEV / "silver_labels.csv"),
        "command": "python3 scripts/temporal_evidence_mvp.py run",
    }
    (OUT / "run_manifest.json").write_text(json.dumps(run_manifest, indent=2), encoding="utf-8")
    summarize()


def _git_head() -> str:
    import subprocess
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def _git_dirty() -> bool:
    import subprocess
    return bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip())


def summarize() -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with (OUT / "benchmark_results.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        metrics = list(csv.DictReader(fh))
    formal = PROTOCOL["formal_methods"]
    figdir = OUT / "figures"; figdir.mkdir(exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for method in formal:
        rr = sorted([x for x in metrics if x["method"] == method and x["condition_id"] == "ALL"],
                    key=lambda x: int(x["nominal_range_m"]))
        x = [int(r["nominal_range_m"]) for r in rr]
        axs[0].plot(x, [float(r["frame_recall"]) for r in rr], marker="o", label=method)
        axs[1].plot(x, [float(r["unmatched_candidates_per_labeled_frame"]) for r in rr], marker="o", label=method)
    axs[0].set(xlabel="nominal range (m)", ylabel="silver-frame recall", ylim=(-.03, 1.03))
    axs[1].set(xlabel="nominal range (m)", ylabel="unmatched candidates / labeled frame")
    for ax in axs: ax.grid(alpha=.25)
    axs[0].legend(fontsize=7); fig.tight_layout(); fig.savefig(figdir / "method_comparison.png", dpi=180); plt.close(fig)

    with (OUT / "score_tradeoff.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        trade = list(csv.DictReader(fh))
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    for method in ("fixed_soft", "adaptive_soft", "adaptive_soft_doppler"):
        points = []
        for threshold in THRESHOLDS:
            rr = [x for x in trade if x["method"] == method and abs(float(x["score_threshold"])-threshold) < 1e-9]
            labels = sum(int(x["label_frames"]) for x in rr)
            detected = sum(int(x["detected_frames"]) for x in rr)
            frames = sum(int(x["lidar_frames"]) for x in rr)
            outputs = sum(int(x["output_candidates_all_frames"]) for x in rr)
            points.append((outputs/frames, detected/labels, threshold))
        ax.plot([x[0] for x in points], [x[1] for x in points], marker="o", label=method)
        for x, y, threshold in points:
            if threshold in (0.50, 0.68, 0.85):
                ax.annotate(f"{threshold:.2f}", (x, y), fontsize=7)
    ax.set(xlabel="output candidates / LiDAR frame", ylabel="silver-frame recall", ylim=(-.03, 1.03))
    ax.grid(alpha=.25); ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(figdir / "score_tradeoff.png", dpi=180); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("freeze", "run", "summarize", "all"))
    args = parser.parse_args()
    if args.command in ("freeze", "all"):
        freeze()
    if args.command in ("run", "all"):
        run()
    if args.command == "summarize":
        summarize()


if __name__ == "__main__":
    main()
