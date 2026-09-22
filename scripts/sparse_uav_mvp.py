#!/usr/bin/env python3
"""Frozen DEV/silver benchmark for sparse FMCW-LiDAR UAV detection.

This module is intentionally separate from the repository detector/tracker.  It
does not modify the baseline.  Labels are the LiDAR-only evidence trajectories
selected by the calibration-closure study; consequently they are DEV/silver
labels, not independent paper ground truth.
"""

from __future__ import annotations

import argparse
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
OUT = ROOT / "results" / "research_dev_v0"
CLOSURE = ROOT / "results" / "calibration_closure"
RAW_VOXEL = CLOSURE / "raw_voxel_cache"
V0_CACHE = ROOT / "results" / "gt_benchmark_v0" / "cache"

SELECTED = {
    "100m_cross_5": "100m_cross_5_e0001",
    "100m_cross_10": "100m_cross_10_e0003",
    "200m_cross_5": "200m_cross_5_e0003",
    "200m_cross_10": "200m_cross_10_e0001",
    "300m_cross_10": "300m_cross_10_e0003",
}

P = {
    "protocol_version": 1,
    "status": "DEV_SILVER_NOT_PAPER_GT",
    "random_seed": 20260922,
    "coordinate_transform_rich80": "[x,z,-y]",
    "label_source": "calibration-closure selected LiDAR evidence trajectories",
    "label_interpolation": "none; observed evidence frames only",
    "label_radius_m": 3.0,
    "broad_roi_m": {"x": [8.0, 510.0], "y": [-200.0, 200.0], "z": [-60.0, 60.0]},
    "intensity_range_inclusive": [10.0, 250.0],
    "detector_input_voxel_m": 1.0,
    "detector_input_note": "existing closure raw voxel cache; intensity is voxel mean",
    "fixed_dbscan_eps_m": 2.0,
    "adaptive_dbscan_eps0_m": 0.8,
    "adaptive_dbscan_alpha": 0.004,
    "adaptive_dbscan_reference_m": 10.0,
    "adaptive_dbscan_eps_cap_m": 2.5,
    "cluster_max_voxels": 32,
    "cluster_max_raw_points": 120,
    "cluster_max_extent_m": 6.0,
    "cluster_range_m": [8.0, 510.0],
    "spatial_jump_tau_min_m": 3.0,
    "spatial_jump_vmax_mps": 25.0,
    "temporal_m": 2,
    "temporal_k": 3,
    "max_track_gap_frames": 2,
    "doppler_sign": "compare -measured_velocity with predicted Cartesian velocity projected on LOS",
    "doppler_fixed_gate_mps": 4.0,
    "doppler_aware_gate": "eta*abs(residual)<=4; eta=abs(er.T@v)/(norm(v)+eps)",
    "formal_methods": {
        "A": "current preprocessing through neighborhood+DBSCAN; real-time candidate linking",
        "B": "adaptive DBSCAN + geometric/jump/M-of-K; no Doppler",
        "C": "B + fixed Doppler consistency",
        "D": "B + observability-aware Doppler consistency",
    },
}

INPUT_ADDENDUM = {
    "addendum_version": 1,
    "reason": "Khosravi-style ROI includes ground removal; raw LiDAR z is not gravity aligned",
    "applies_to": ["B", "C", "D", "ablations"],
    "does_not_apply_to": ["A_current"],
    "transform": "enu_like = R_full_6dof.T @ (lidar_xyz - t_full_6dof)",
    "leveled_height_gt_m": 10.0,
    "threshold_provenance": "unchanged legacy z>10 threshold, applied in the physically appropriate leveled frame",
    "status": "PROVISIONAL because the shared 6-DoF calibration is provisional",
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_evidence(condition_id: str, candidate_id: str) -> dict:
    with np.load(CLOSURE / f"evidence_trajectories_{condition_id}.npz") as z:
        ids = z["candidate_ids"].astype(str).tolist()
        i = ids.index(candidate_id)
        return {
            "frames": z[f"frames_{i}"].copy(),
            "times": z[f"times_{i}"].copy(),
            "positions": z[f"positions_{i}"].copy(),
            "nodes": z[f"nodes_{i}"].copy(),
            "summary": json.loads(str(z[f"summary_{i}"].item())),
        }


def build_label_rows() -> list[dict]:
    rows: list[dict] = []
    for condition_id, candidate_id in SELECTED.items():
        nominal_range = int(condition_id.split("m_")[0])
        nominal_speed = int(condition_id.rsplit("_", 1)[1])
        ev = load_evidence(condition_id, candidate_id)
        for frame, stamp, xyz, node in zip(ev["frames"], ev["times"], ev["positions"], ev["nodes"]):
            rows.append({
                "condition_id": condition_id,
                "candidate_id": candidate_id,
                "nominal_range_m": nominal_range,
                "nominal_speed_mps": nominal_speed,
                "frame_index": int(frame),
                "bag_time_unix": float(stamp),
                "center_x_m": float(xyz[0]),
                "center_y_m": float(xyz[1]),
                "center_z_m": float(xyz[2]),
                "evidence_raw_points": int(round(node[3])),
                "evidence_voxels": int(round(node[4])),
                "evidence_extent_m": float(node[5]),
                "evidence_intensity_mean": float(node[6]),
                "evidence_radial_velocity_mean_mps": float(node[7]),
                "label_status": "DEV_SILVER",
                "paper_ground_truth": False,
                "interpolated": False,
            })
    return sorted(rows, key=lambda r: (r["condition_id"], r["frame_index"]))


def freeze() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    labels = build_label_rows()
    label_path = OUT / "silver_labels.csv"
    if label_path.exists():
        with label_path.open("r", newline="", encoding="utf-8-sig") as fh:
            old = list(csv.DictReader(fh))
        comparable = [{k: str(v) for k, v in row.items()} for row in labels]
        if old != comparable:
            raise RuntimeError("silver_labels.csv differs; refusing to change frozen DEV labels")
    else:
        write_csv(label_path, labels)

    alignment = json.loads((CLOSURE / "alignment.json").read_text(encoding="utf-8"))
    manifest = []
    for cid, candidate_id in SELECTED.items():
        meta = json.loads((RAW_VOXEL / f"{cid}.json").read_text(encoding="utf-8"))
        subset = "calibration" if cid in alignment["calibration_selected"] else "validation"
        manifest.append({
            "condition_id": cid,
            "candidate_id": candidate_id,
            "split_in_closure": subset,
            "dev_label_status": "DEV_SILVER_NOT_PAPER_GT",
            "label_frames": sum(x["condition_id"] == cid for x in labels),
            "lidar_frames": len(meta["frames"]),
            "lidar_start_unix": meta["frames"][0]["bag_time"],
            "lidar_end_unix": meta["frames"][-1]["bag_time"],
            "raw_voxel_cache": str((RAW_VOXEL / f"{cid}.npz").relative_to(ROOT)),
            "evidence_file": str((CLOSURE / f"evidence_trajectories_{cid}.npz").relative_to(ROOT)),
        })
    write_csv(OUT / "dev_manifest.csv", manifest)

    core = {
        **P,
        "selected_candidates": SELECTED,
        "source_sha256": {
            "calibration_alignment": sha256(CLOSURE / "alignment.json"),
            "calibration_protocol": sha256(CLOSURE / "protocol.json"),
            "build_gt_benchmark_v0.py": sha256(ROOT / "scripts" / "build_gt_benchmark_v0.py"),
            "target_track_without_initialize_and_grid.py": sha256(ROOT / "target_track_without_initialize_and_grid.py"),
        },
    }
    protocol_path = OUT / "protocol.json"
    if protocol_path.exists():
        old = json.loads(protocol_path.read_text(encoding="utf-8"))
        if old != core:
            raise RuntimeError("protocol.json differs; refusing silent retuning")
    else:
        protocol_path.write_text(json.dumps(core, ensure_ascii=False, indent=2), encoding="utf-8")
    (OUT / "FROZEN").write_text(
        "research_dev_v0 labels and parameters frozen before formal comparison\n", encoding="utf-8"
    )
    addendum = {
        **INPUT_ADDENDUM,
        "calibration_alignment_sha256": sha256(CLOSURE / "alignment.json"),
    }
    addendum_path = OUT / "input_protocol_addendum.json"
    if addendum_path.exists():
        old_addendum = json.loads(addendum_path.read_text(encoding="utf-8"))
        if old_addendum != addendum:
            raise RuntimeError("input_protocol_addendum.json differs; refusing silent retuning")
    else:
        addendum_path.write_text(json.dumps(addendum, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"frozen: {len(labels)} labels across {len(SELECTED)} sequences")


def read_labels() -> list[dict]:
    with (OUT / "silver_labels.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def _stage_by_frame(condition_id: str, stage: str, nframes: int) -> list[np.ndarray]:
    with np.load(V0_CACHE / f"{condition_id}.npz") as z:
        frame = z[f"{stage}_frame"].copy()
        values = z[f"{stage}_values"].copy()
    return [values[frame == i] for i in range(nframes)]


def audit_survival() -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before audit")
    if str(ROOT / "scripts") not in sys.path:
        sys.path.insert(0, str(ROOT / "scripts"))
    from build_gt_benchmark_v0 import CONDITION_SPECS, iter_condition_pointclouds

    labels = read_labels()
    lookup = {(x["condition_id"], int(x["frame_index"])): x for x in labels}
    specs = {x[0]: x for x in CONDITION_SPECS}
    rows = []
    radius2 = float(P["label_radius_m"]) ** 2
    alignment = json.loads((CLOSURE / "alignment.json").read_text(encoding="utf-8"))["full_6dof"]
    rotation = np.asarray(alignment["rotation_matrix"], dtype=float)
    translation = np.asarray(alignment["translation_m"], dtype=float)
    for cid in SELECTED:
        meta = json.loads((V0_CACHE / f"{cid}.json").read_text(encoding="utf-8"))
        background = _stage_by_frame(cid, "background", len(meta["frames"]))
        neighborhood = _stage_by_frame(cid, "neighborhood", len(meta["frames"]))
        started = time.perf_counter()
        for fi, (arr, pc_meta) in enumerate(iter_condition_pointclouds(specs[cid])):
            label = lookup.get((cid, fi))
            if label is None:
                continue
            gt = np.array([float(label["center_x_m"]), float(label["center_y_m"]), float(label["center_z_m"])])
            x = np.asarray(arr["x"], dtype=np.float32).reshape(-1)
            sy = np.asarray(arr["y"], dtype=np.float32).reshape(-1)
            sz = np.asarray(arr["z"], dtype=np.float32).reshape(-1)
            intensity = np.asarray(arr["intensity"], dtype=np.float32).reshape(-1)
            velocity = np.asarray(arr["velocity"], dtype=np.float32).reshape(-1)
            y, z = (sz, -sy) if int(pc_meta["point_step"]) == 80 else (sy, sz)
            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity) & np.isfinite(velocity)
            nonzero = (x * x + y * y + z * z) > 1e-8
            target = finite & nonzero & (((x-gt[0])**2 + (y-gt[1])**2 + (z-gt[2])**2) <= radius2)
            target_i = target & (intensity >= P["intensity_range_inclusive"][0]) & (intensity <= P["intensity_range_inclusive"][1])
            target_h = target_i & (z > 10.0)
            leveled_z = (rotation.T @ (np.column_stack([x, y, z]) - translation).T)[2]
            target_level_h = target_i & (leveled_z > INPUT_ADDENDUM["leveled_height_gt_m"])
            b = background[fi]
            n = neighborhood[fi]
            bc = int(np.sum(np.sum((b[:, :3] - gt) ** 2, axis=1) <= radius2)) if len(b) else 0
            nc = int(np.sum(np.sum((n[:, :3] - gt) ** 2, axis=1) <= radius2)) if len(n) else 0
            rows.append({
                "condition_id": cid, "nominal_range_m": label["nominal_range_m"],
                "frame_index": fi, "center_z_m": float(gt[2]),
                "raw_target_points": int(target.sum()),
                "intensity_target_points": int(target_i.sum()),
                "z_gt_10_target_points": int(target_h.sum()),
                "leveled_z_gt_10_target_points": int(target_level_h.sum()),
                "background_target_points": bc,
                "neighborhood_target_points": nc,
                "dbscan_target_points": nc,
                "raw_to_intensity_survival": float(target_i.sum()/target.sum()) if target.sum() else "",
                "intensity_to_z_survival": float(target_h.sum()/target_i.sum()) if target_i.sum() else "",
                "raw_intensity_median": float(np.median(intensity[target])) if target.any() else "",
                "raw_velocity_median_mps": float(np.median(velocity[target])) if target.any() else "",
            })
        print(f"audit {cid}: {sum(x['condition_id']==cid for x in rows)} labels, {time.perf_counter()-started:.1f}s")
    write_csv(OUT / "stage_survival.csv", rows)


def _load_raw_voxel_frames(condition_id: str) -> tuple[list[np.ndarray], np.ndarray]:
    meta = json.loads((RAW_VOXEL / f"{condition_id}.json").read_text(encoding="utf-8"))
    with np.load(RAW_VOXEL / f"{condition_id}.npz") as z:
        values, offsets = z["values"].copy(), z["offsets"].copy()
    frames = []
    alignment = json.loads((CLOSURE / "alignment.json").read_text(encoding="utf-8"))["full_6dof"]
    rotation = np.asarray(alignment["rotation_matrix"], dtype=float)
    translation = np.asarray(alignment["translation_m"], dtype=float)
    lo, hi = P["intensity_range_inclusive"]
    roi = P["broad_roi_m"]
    for i in range(len(offsets)-1):
        a = values[offsets[i]:offsets[i+1]]
        keep = ((a[:, 1] >= roi["x"][0]) & (a[:, 1] <= roi["x"][1]) &
                (a[:, 2] >= roi["y"][0]) & (a[:, 2] <= roi["y"][1]) &
                (a[:, 3] >= roi["z"][0]) & (a[:, 3] <= roi["z"][1]) &
                (a[:, 5] >= lo) & (a[:, 5] <= hi))
        leveled_z = (rotation.T @ (a[:, 1:4] - translation).T)[2]
        keep &= leveled_z > INPUT_ADDENDUM["leveled_height_gt_m"]
        # x,y,z, raw-point support, intensity, measured radial velocity
        frames.append(a[keep][:, [1, 2, 3, 4, 5, 6]])
    times = np.asarray([x["bag_time"] for x in meta["frames"]], dtype=float)
    return frames, times


def _weighted_median(values: np.ndarray, weights: np.ndarray) -> np.ndarray:
    out = []
    for j in range(values.shape[1]):
        order = np.argsort(values[:, j])
        v, w = values[order, j], weights[order]
        out.append(float(v[np.searchsorted(np.cumsum(w), 0.5*np.sum(w), side="left")]))
    return np.asarray(out)


def cluster_frames(frames: list[np.ndarray], min_pts: int, adaptive: bool) -> tuple[list[list[dict]], float]:
    result: list[list[dict]] = []
    started = time.perf_counter()
    for values in frames:
        if not len(values):
            result.append([]); continue
        mean_range = float(np.mean(np.linalg.norm(values[:, :3], axis=1)))
        if adaptive:
            eps = min(P["adaptive_dbscan_eps_cap_m"],
                      P["adaptive_dbscan_eps0_m"] + P["adaptive_dbscan_alpha"] *
                      max(mean_range - P["adaptive_dbscan_reference_m"], 0.0))
        else:
            eps = P["fixed_dbscan_eps_m"]
        labels = DBSCAN(eps=eps, min_samples=min_pts, algorithm="kd_tree").fit_predict(values[:, :3])
        candidates = []
        for label in np.unique(labels):
            if label < 0:
                continue
            c = values[labels == label]
            raw_points = float(c[:, 3].sum())
            extent = float(np.max(np.ptp(c[:, :3], axis=0)))
            if (len(c) > P["cluster_max_voxels"] or raw_points > P["cluster_max_raw_points"] or
                    extent >= P["cluster_max_extent_m"]):
                continue
            center = (_weighted_median(c[:, :3], c[:, 3]) if raw_points >= 3
                      else np.average(c[:, :3], axis=0, weights=c[:, 3]))
            distance = float(np.linalg.norm(center))
            if not (P["cluster_range_m"][0] <= distance <= P["cluster_range_m"][1]):
                continue
            candidates.append({
                "center": center, "raw_points": int(round(raw_points)), "voxels": len(c),
                "extent": extent, "intensity": float(np.average(c[:, 4], weights=c[:, 3])),
                "radial_velocity": float(np.average(c[:, 5], weights=c[:, 3])), "eps": eps,
            })
        result.append(candidates)
    return result, time.perf_counter() - started


def current_candidates(condition_id: str, nframes: int) -> tuple[list[list[dict]], float]:
    frames = _stage_by_frame(condition_id, "neighborhood", nframes)
    result = []
    started = time.perf_counter()
    for values in frames:
        if not len(values):
            result.append([]); continue
        labels = DBSCAN(eps=2.0, min_samples=1).fit_predict(values[:, :3])
        candidates = []
        for lab in np.unique(labels):
            c = values[labels == lab]
            candidates.append({
                "center": c[:, :3].mean(axis=0), "raw_points": len(c), "voxels": len(c),
                "extent": float(np.max(np.ptp(c[:, :3], axis=0))),
                "intensity": float(c[:, 3].mean()), "radial_velocity": float(c[:, 4].mean()),
                "eps": 2.0,
            })
        result.append(candidates)
    return result, time.perf_counter() - started


def temporal_validate(candidates: list[list[dict]], times: np.ndarray, temporal: bool,
                      doppler: str, current_a: bool = False) -> tuple[list[list[dict]], float]:
    started = time.perf_counter()
    tracks: dict[int, dict] = {}
    next_id = 0
    outputs: list[list[dict]] = []
    for fi, current in enumerate(candidates):
        now = times[fi]
        active_ids = [tid for tid, tr in tracks.items()
                      if fi - tr["last_frame"] <= P["max_track_gap_frames"] + 1]
        assigned_c, assigned_t = set(), set()
        current_track: dict[int, int] = {}
        options = []
        if active_ids and current:
            preds, gates = [], []
            for tid in active_ids:
                tr = tracks[tid]
                dt = now - tr["times"][-1]
                if len(tr["positions"]) >= 2:
                    pdt = tr["times"][-1] - tr["times"][-2]
                    vel = ((tr["positions"][-1] - tr["positions"][-2]) / pdt
                           if pdt > 1e-6 else np.zeros(3))
                else:
                    vel = np.zeros(3)
                preds.append(tr["positions"][-1] + vel * max(dt, 0.0))
                gates.append(max(P["spatial_jump_tau_min_m"], P["spatial_jump_vmax_mps"] * max(dt, 0.0)))
            tree = cKDTree(np.asarray(preds))
            k = min(4, len(preds))
            for ci, cand in enumerate(current):
                d, jj = tree.query(cand["center"], k=k)
                for distance, aj in zip(np.atleast_1d(d), np.atleast_1d(jj)):
                    if distance > gates[int(aj)]:
                        continue
                    tid = active_ids[int(aj)]
                    tr = tracks[tid]
                    residual = eta = 0.0
                    if len(tr["positions"]) >= 2:
                        dtv = tr["times"][-1] - tr["times"][-2]
                        vel = ((tr["positions"][-1]-tr["positions"][-2])/dtv
                               if dtv > 1e-6 else np.zeros(3))
                        er = cand["center"] / max(np.linalg.norm(cand["center"]), 1e-9)
                        expected = float(np.dot(er, vel))
                        residual = abs(-cand["radial_velocity"] - expected)
                        eta = abs(expected) / (float(np.linalg.norm(vel)) + 1e-9)
                        effective = residual if doppler == "fixed" else eta * residual
                        if doppler != "none" and effective > P["doppler_fixed_gate_mps"]:
                            continue
                    penalty = 0.0 if doppler == "none" else 0.5 * (residual if doppler == "fixed" else eta*residual)
                    options.append((float(distance)+penalty, ci, tid, residual, eta))
        for _, ci, tid, residual, eta in sorted(options):
            if ci in assigned_c or tid in assigned_t:
                continue
            cand = current[ci]
            cand["doppler_residual_mps"] = residual; cand["eta"] = eta
            tr = tracks[tid]
            tr["frames"].append(fi); tr["times"].append(now); tr["positions"].append(cand["center"])
            tr["last_frame"] = fi; tr["candidate_indices"].append(ci)
            assigned_c.add(ci); assigned_t.add(tid)
            current_track[ci] = tid
        for ci, cand in enumerate(current):
            if ci not in assigned_c:
                tid = next_id; next_id += 1
                tracks[tid] = {"frames": [fi], "times": [now], "positions": [cand["center"]],
                               "last_frame": fi, "candidate_indices": [ci]}
                cand["doppler_residual_mps"] = math.nan; cand["eta"] = math.nan
                assigned_t.add(tid)
                current_track[ci] = tid
            else:
                tid = current_track[ci]
            cand["track_id"] = tid
        accepted = []
        for cand in current:
            tr = tracks[cand["track_id"]]
            if current_a or not temporal:
                accepted.append(cand)
                continue
            recent = sum((fi - f) < P["temporal_k"] for f in tr["frames"])
            if recent >= P["temporal_m"]:
                accepted.append(cand)
        outputs.append(accepted)
    return outputs, time.perf_counter() - started


def _gap_metrics(found: list[bool]) -> tuple[int, int, int]:
    longest = gaps = recovered = run = 0
    for i, value in enumerate(found):
        if value:
            if run and i-run-1 >= 0 and found[i-run-1]:
                gaps += 1; recovered += 1
            run = 0
        else:
            run += 1; longest = max(longest, run)
    if run and len(found)-run-1 >= 0 and found[len(found)-run-1]:
        gaps += 1
    return gaps, recovered, longest


def evaluate(method: str, cid: str, outputs: list[list[dict]], labels: list[dict],
             runtime_s: float, raw_candidates: int) -> tuple[dict, list[dict]]:
    truth = sorted([x for x in labels if x["condition_id"] == cid], key=lambda x: int(x["frame_index"]))
    found, errors, matched_ids, per_frame = [], [], defaultdict(set), []
    fp = tp = evaluated = 0
    for row in truth:
        fi = int(row["frame_index"]); gt = np.array([float(row["center_x_m"]), float(row["center_y_m"]), float(row["center_z_m"])])
        cand = outputs[fi]
        distances = np.asarray([np.linalg.norm(x["center"]-gt) for x in cand], dtype=float)
        matches = np.flatnonzero(distances <= P["label_radius_m"])
        ok = len(matches) > 0; found.append(ok); evaluated += len(cand)
        best = int(matches[np.argmin(distances[matches])]) if ok else -1
        if ok:
            tp += 1; errors.append(float(distances[best])); matched_ids[int(cand[best]["track_id"])].add(fi)
        fp += len(cand) - int(ok)
        per_frame.append({
            "method": method, "condition_id": cid, "nominal_range_m": row["nominal_range_m"],
            "frame_index": fi, "label_present": True, "detected": ok,
            "candidate_count": len(cand), "false_candidate_count": len(cand)-int(ok),
            "center_error_m": float(distances[best]) if ok else "",
            "matched_track_id": int(cand[best]["track_id"]) if ok else "",
            "eta": cand[best].get("eta", "") if ok else "",
            "doppler_residual_mps": cand[best].get("doppler_residual_mps", "") if ok else "",
        })
    gaps, recovered, longest = _gap_metrics(found)
    e = np.asarray(errors)
    metric = {
        "method": method, "condition_id": cid, "nominal_range_m": int(truth[0]["nominal_range_m"]),
        "label_frames": len(truth), "detected_frames": sum(found),
        "frame_recall": float(np.mean(found)) if found else "",
        "precision": tp/(tp+fp) if tp+fp else "", "false_candidates": fp,
        "false_candidates_per_label_frame": fp/len(truth) if truth else "",
        "center_error_mean_m": float(e.mean()) if len(e) else "",
        "center_error_rmse_m": float(np.sqrt(np.mean(e**2))) if len(e) else "",
        "track_coverage_best_id": max((len(v) for v in matched_ids.values()), default=0)/len(truth) if truth else "",
        "longest_miss_labels": longest, "gap_count": gaps, "gap_recovered": recovered,
        "gap_recovery_rate": recovered/gaps if gaps else "",
        "candidate_count_all_frames": sum(map(len, outputs)), "raw_candidate_count_all_frames": raw_candidates,
        "runtime_s": runtime_s, "runtime_ms_per_frame": 1000*runtime_s/len(outputs),
    }
    return metric, per_frame


def aggregate(metrics: list[dict], method: str, nominal_range: int) -> dict:
    rows = [x for x in metrics if x["method"] == method and int(x["nominal_range_m"]) == nominal_range]
    labels = sum(int(x["label_frames"]) for x in rows); detected = sum(int(x["detected_frames"]) for x in rows)
    fp = sum(int(x["false_candidates"]) for x in rows); candidates = detected + fp
    weights = np.asarray([int(x["detected_frames"]) for x in rows], dtype=float)
    means = np.asarray([float(x["center_error_mean_m"] or 0) for x in rows])
    return {
        "method": method, "condition_id": "ALL", "nominal_range_m": nominal_range,
        "label_frames": labels, "detected_frames": detected, "frame_recall": detected/labels if labels else "",
        "precision": detected/candidates if candidates else "", "false_candidates": fp,
        "false_candidates_per_label_frame": fp/labels if labels else "",
        "center_error_mean_m": float(np.average(means, weights=weights)) if weights.sum() else "",
        "center_error_rmse_m": "", "track_coverage_best_id": "",
        "longest_miss_labels": max((int(x["longest_miss_labels"]) for x in rows), default=0),
        "gap_count": sum(int(x["gap_count"]) for x in rows),
        "gap_recovered": sum(int(x["gap_recovered"]) for x in rows),
        "gap_recovery_rate": "", "candidate_count_all_frames": sum(int(x["candidate_count_all_frames"]) for x in rows),
        "raw_candidate_count_all_frames": sum(int(x["raw_candidate_count_all_frames"]) for x in rows),
        "runtime_s": sum(float(x["runtime_s"]) for x in rows),
        "runtime_ms_per_frame": "",
    }


def summarize_outputs() -> None:
    """Derive observability strata and diagnostic figures from frozen outputs."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = read_labels()
    with (OUT / "frame_results.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        frame_results = list(csv.DictReader(fh))
    result_lookup = {(x["method"], x["condition_id"], int(x["frame_index"])): x
                     for x in frame_results}
    observability = []
    methods = ["B_temporal", "C_fixed_doppler", "D_aware_doppler"]
    for cid in SELECTED:
        rr = sorted([x for x in labels if x["condition_id"] == cid], key=lambda x: int(x["frame_index"]))
        t = np.asarray([float(x["bag_time_unix"]) for x in rr])
        p = np.asarray([[float(x["center_x_m"]), float(x["center_y_m"]), float(x["center_z_m"])] for x in rr])
        v = np.gradient(p, t, axis=0)
        er = p / np.maximum(np.linalg.norm(p, axis=1, keepdims=True), 1e-9)
        eta = np.abs(np.sum(er*v, axis=1)) / (np.linalg.norm(v, axis=1) + 1e-9)
        for row, value in zip(rr, eta):
            band = "low_<0.25" if value < 0.25 else ("mid_0.25-0.60" if value < 0.60 else "high_>=0.60")
            for method in methods:
                result = result_lookup[(method, cid, int(row["frame_index"]))]
                observability.append({
                    "method": method, "condition_id": cid, "nominal_range_m": row["nominal_range_m"],
                    "frame_index": row["frame_index"], "trajectory_eta": float(value),
                    "eta_band": band, "detected": result["detected"],
                    "center_error_m": result["center_error_m"],
                })
    write_csv(OUT / "observability_frames.csv", observability)
    summary = []
    for method in methods:
        for band in ("low_<0.25", "mid_0.25-0.60", "high_>=0.60"):
            rr = [x for x in observability if x["method"] == method and x["eta_band"] == band]
            detected = sum(x["detected"] == "True" for x in rr)
            summary.append({"method": method, "eta_band": band, "label_frames": len(rr),
                            "detected_frames": detected, "frame_recall": detected/len(rr) if rr else ""})
    write_csv(OUT / "observability_summary.csv", summary)

    figdir = OUT / "figures"; figdir.mkdir(exist_ok=True)
    with (OUT / "stage_survival.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        survival = list(csv.DictReader(fh))
    stages = ["raw_target_points", "intensity_target_points", "z_gt_10_target_points",
              "leveled_z_gt_10_target_points", "background_target_points", "neighborhood_target_points"]
    stage_names = ["raw", "intensity", "raw z>10", "leveled z>10", "background", "neighborhood"]
    values = []
    for distance in (100, 200, 300):
        rr = [x for x in survival if int(x["nominal_range_m"]) == distance]
        raw = sum(int(x["raw_target_points"]) for x in rr)
        values.append([sum(int(x[s]) for x in rr)/raw if raw else 0 for s in stages])
    x = np.arange(len(stages)); width = 0.24
    fig, ax = plt.subplots(figsize=(10, 4.8))
    for i, distance in enumerate((100, 200, 300)):
        ax.bar(x+(i-1)*width, values[i], width, label=f"{distance} m")
    ax.set_xticks(x, stage_names, rotation=20, ha="right"); ax.set_ylim(0, 1.08)
    ax.set_ylabel("target-point survival / raw"); ax.legend(); ax.grid(axis="y", alpha=.25)
    fig.tight_layout(); fig.savefig(figdir / "stage_survival.png", dpi=180); plt.close(fig)

    with (OUT / "benchmark_results.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        metrics = list(csv.DictReader(fh))
    formal = ["A_current", "B_temporal", "C_fixed_doppler", "D_aware_doppler"]
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for method in formal:
        rr = [x for x in metrics if x["method"] == method and x["condition_id"] == "ALL"]
        rr.sort(key=lambda x: int(x["nominal_range_m"]))
        axs[0].plot([int(x["nominal_range_m"]) for x in rr], [float(x["frame_recall"] or 0) for x in rr], marker="o", label=method)
        axs[1].plot([int(x["nominal_range_m"]) for x in rr], [float(x["precision"] or 0) for x in rr], marker="o", label=method)
    axs[0].set(ylabel="frame recall", xlabel="nominal range (m)", ylim=(-.03, 1.03))
    axs[1].set(ylabel="positive-frame candidate precision", xlabel="nominal range (m)", ylim=(-.002, .035))
    for ax in axs: ax.grid(alpha=.25)
    axs[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(figdir / "formal_methods.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    bands = ["low_<0.25", "mid_0.25-0.60", "high_>=0.60"]
    for i, method in enumerate(methods):
        rr = {x["eta_band"]: float(x["frame_recall"] or 0) for x in summary if x["method"] == method}
        ax.bar(np.arange(3)+(i-1)*.24, [rr[b] for b in bands], .24, label=method)
    ax.set_xticks(np.arange(3), ["low", "mid", "high"]); ax.set_ylim(0, 1)
    ax.set_ylabel("frame recall"); ax.set_xlabel("trajectory radial observability eta")
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=.25)
    fig.tight_layout(); fig.savefig(figdir / "doppler_observability.png", dpi=180); plt.close(fig)


def run_benchmark() -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before benchmark")
    labels = read_labels()
    main_metrics, main_frames, ablation_metrics = [], [], []
    configs = [(m, a, t, d) for m in (1, 2, 3) for a in (False, True)
               for t in (False, True) for d in ("none", "fixed", "aware")]
    for cid in SELECTED:
        frames, times = _load_raw_voxel_frames(cid)
        current, cluster_s = current_candidates(cid, len(frames))
        out, temporal_s = temporal_validate(current, times, temporal=False, doppler="none", current_a=True)
        metric, pf = evaluate("A_current", cid, out, labels, cluster_s+temporal_s, sum(map(len, current)))
        main_metrics.append(metric); main_frames.extend(pf)
        cluster_cache = {}
        for min_pts, adaptive, temporal, doppler in configs:
            key = (min_pts, adaptive)
            if key not in cluster_cache:
                cluster_cache[key] = cluster_frames(frames, min_pts, adaptive)
            candidates, cs = cluster_cache[key]
            out, ts = temporal_validate(candidates, times, temporal=temporal, doppler=doppler)
            name = f"min{min_pts}_{'adaptive' if adaptive else 'fixed'}_{'temporal' if temporal else 'no_temporal'}_{doppler}"
            metric, pf = evaluate(name, cid, out, labels, cs+ts, sum(map(len, candidates)))
            metric.update({"min_pts": min_pts, "adaptive": adaptive, "temporal": temporal, "doppler": doppler})
            ablation_metrics.append(metric)
            formal = {"min2_adaptive_temporal_none": "B_temporal",
                      "min2_adaptive_temporal_fixed": "C_fixed_doppler",
                      "min2_adaptive_temporal_aware": "D_aware_doppler"}.get(name)
            if formal:
                metric2 = {k: v for k, v in metric.items() if k not in ("min_pts", "adaptive", "temporal", "doppler")}
                metric2["method"] = formal
                main_metrics.append(metric2)
                for x in pf:
                    x = dict(x); x["method"] = formal; main_frames.append(x)
        print(f"benchmark {cid}: complete")
    for method in ["A_current", "B_temporal", "C_fixed_doppler", "D_aware_doppler"]:
        for distance in (100, 200, 300):
            main_metrics.append(aggregate(main_metrics, method, distance))
    write_csv(OUT / "benchmark_results.csv", main_metrics)
    write_csv(OUT / "frame_results.csv", main_frames)
    write_csv(OUT / "ablation_results.csv", ablation_metrics)
    run = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "script_sha256": sha256(Path(__file__)),
        "protocol_sha256": sha256(OUT / "protocol.json"),
        "input_protocol_addendum_sha256": sha256(OUT / "input_protocol_addendum.json"),
        "labels_sha256": sha256(OUT / "silver_labels.csv"),
        "command": "python3 scripts/sparse_uav_mvp.py benchmark",
    }
    (OUT / "run_manifest.json").write_text(json.dumps(run, indent=2), encoding="utf-8")
    summarize_outputs()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("freeze", "audit", "benchmark", "summarize", "all"))
    args = parser.parse_args()
    if args.command in ("freeze", "all"): freeze()
    if args.command in ("audit", "all"): audit_survival()
    if args.command in ("benchmark", "all"): run_benchmark()
    if args.command == "summarize": summarize_outputs()


if __name__ == "__main__":
    main()
