#!/usr/bin/env python3
"""Post-hoc forensics for the frozen candidate-graph DP failure.

The script does not alter or tune the detector/DP.  It reconstructs the exact
frozen graph, verifies the recorded top path, and uses DEV silver observations
only to identify the most label-consistent feasible path for diagnosis.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from sklearn.metrics import roc_auc_score


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import published_baseline_dp_mvp as frozen
import sparse_uav_mvp as sparse
import temporal_evidence_mvp as temporal


OUT = ROOT / "results" / "dp_failure_forensics"
SOURCE = ROOT / "results" / "published_baseline_dp_mvp"
DEV = ROOT / "results" / "research_dev_v0"
CONDITIONS = tuple(sparse.SELECTED)
TOP_K_STATE_WINNERS = 100


PROTOCOL = {
    "protocol_version": 1,
    "status": "POST_HOC_DIAGNOSTIC_DEV_SILVER_NOT_ALGORITHM_SELECTION",
    "algorithm_change": "NONE",
    "dp_parameter_change": "NONE",
    "source_dp_protocol": "results/published_baseline_dp_mvp/protocol.json",
    "candidate_source": "exact frozen 1 m range-adaptive minPts=1 candidates",
    "top_path": "exact replay of frozen DP over the complete bag",
    "silver_feasible_path": {
        "search_span": "inclusive first-to-last silver observation frame",
        "objective": "lexicographically maximize silver-frame hits, then frozen cumulative DP score",
        "match_radius_m": sparse.P["label_radius_m"],
        "constraints": "unchanged frozen graph pruning, dt, speed, acceleration, gap and duration rules",
        "warning": "label-guided diagnostic path; not a deployable output and not independent GT",
    },
    "rank": {
        "top_k": TOP_K_STATE_WINNERS,
        "definition": "unique Viterbi edge-state winner paths within the silver span, sorted by frozen score",
        "limit": "not an exact rank among the exponentially many non-winning raw paths",
        "lower_bound": "1 + number of qualifying edge-state winner scores strictly above the silver path score",
    },
    "doppler_residual": "abs(-measured_radial_velocity - geometric_dr_dt); dataset-observed sign convention",
    "legacy_revisit": {
        "coordinate_frame": "provisional yaw-preserving leveled coordinates used by A-prime",
        "cell_m": 1.0,
        "prior_only": True,
        "cell_revisit_fraction": "prior frames containing an input voxel in the same legacy cell / prior frames",
        "neighborhood_revisit_fraction": "prior frames containing an input voxel in any 3x3x3 neighbor cell / prior frames",
        "background_probability": "legacy static count before current frame / legacy frame_count after current-frame increment",
    },
    "feature_separation": {
        "comparison": "silver-compatible UAV nodes versus top-clutter nodes at the same silver frames",
        "statistics": "pooled ROC AUC plus per-sequence median direction count; descriptive only",
        "no_feature_threshold_selection": True,
    },
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    fields = fields or (list(rows[0]) if rows else [])
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def _git_head() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def freeze() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    payload = {
        **PROTOCOL,
        "selected_conditions": list(CONDITIONS),
        "source_sha256": {
            "frozen_dp_protocol": sha256(SOURCE / "protocol.json"),
            "frozen_dp_results": sha256(SOURCE / "benchmark_results.csv"),
            "frozen_dp_detections": sha256(SOURCE / "candidate_detections.csv"),
            "frozen_dp_script": sha256(ROOT / "scripts" / "published_baseline_dp_mvp.py"),
            "silver_labels": sha256(DEV / "silver_labels.csv"),
            "legacy_protocol": sha256(ROOT / "results" / "temporal_evidence_mvp" / "protocol.json"),
        },
    }
    path = OUT / "protocol.json"
    if path.exists() and json.loads(path.read_text(encoding="utf-8")) != payload:
        raise RuntimeError("forensics protocol differs; refusing silent definition drift")
    if not path.exists():
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    (OUT / "FROZEN").write_text(
        "diagnostic definitions frozen; source DP and all algorithm parameters unchanged\n", encoding="utf-8"
    )
    print("frozen DP failure-forensics definitions")


def build_graph(candidates: list[list[dict]], times: np.ndarray):
    cfg = frozen.PROTOCOL["candidate_graph_dp"]
    nodes, by_frame = [], []
    for fi, frame in enumerate(candidates):
        ids = []
        for ci, candidate in enumerate(frame):
            nid = len(nodes)
            extent = float(candidate["extent"])
            raw = float(candidate["raw_points"])
            compactness_component = 0.30 * math.exp(-extent / 2.0)
            support_component = 0.20 * min(math.log1p(raw) / math.log(6.0), 1.0)
            nodes.append({
                "id": nid, "frame": fi, "time": float(times[fi]), "candidate_index": ci,
                "center": np.asarray(candidate["center"], dtype=float), "candidate": candidate,
                "base_component": 0.50, "compactness_component": compactness_component,
                "support_component": support_component,
                "score": 0.50 + compactness_component + support_component,
            })
            ids.append(nid)
        by_frame.append(ids)

    edges, incoming = [], defaultdict(list)
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
                    edges.append({
                        "id": eid, "u": u, "v": v, "dt": dt, "velocity": velocity,
                        "frame_gap": gap,
                        "gap_penalty": cfg["gap_penalty_per_skipped_frame"] * (gap - 1),
                    })
                    incoming[v].append(eid)
    return nodes, by_frame, edges, incoming


def _reconstruct(eid: int, edges: list[dict], parent: dict[int, int | None]) -> tuple[int, ...]:
    reverse = [edges[eid]["v"]]
    cursor = eid
    while cursor is not None:
        reverse.append(edges[cursor]["u"])
        cursor = parent[cursor]
    return tuple(reversed(reverse))


def ordinary_dp(nodes: list[dict], edges: list[dict], incoming: dict[int, list[int]],
                frame_min: int | None = None, frame_max: int | None = None):
    cfg = frozen.PROTOCOL["candidate_graph_dp"]
    score, parent, count, start = {}, {}, {}, {}
    for eid, edge in enumerate(edges):
        u, v = edge["u"], edge["v"]
        if frame_min is not None and (nodes[u]["frame"] < frame_min or nodes[v]["frame"] < frame_min):
            continue
        if frame_max is not None and (nodes[u]["frame"] > frame_max or nodes[v]["frame"] > frame_max):
            continue
        best_score = nodes[u]["score"] + nodes[v]["score"] - edge["gap_penalty"]
        best_parent, best_count, best_start = None, 2, u
        for peid in incoming.get(u, []):
            if peid not in score:
                continue
            previous = edges[peid]
            accel_dt = 0.5 * (previous["dt"] + edge["dt"])
            acceleration = float(np.linalg.norm(edge["velocity"] - previous["velocity"]) / max(accel_dt, 1e-9))
            if acceleration > cfg["edge_max_acceleration_mps2"]:
                continue
            trial = score[peid] + nodes[v]["score"] - edge["gap_penalty"]
            if trial > best_score:
                best_score = trial
                best_parent = peid
                best_count = count[peid] + 1
                best_start = start[peid]
        score[eid], parent[eid], count[eid], start[eid] = best_score, best_parent, best_count, best_start
    qualifying = [eid for eid in score if count[eid] >= cfg["path_min_nodes"] and
                  nodes[edges[eid]["v"]]["time"] - nodes[start[eid]]["time"] >= cfg["path_min_duration_s"]]
    best = max(qualifying, key=lambda x: score[x]) if qualifying else None
    return {
        "score": score, "parent": parent, "count": count, "start": start,
        "qualifying": qualifying, "best_eid": best,
        "best_path": _reconstruct(best, edges, parent) if best is not None else tuple(),
    }


def silver_lexicographic_dp(nodes: list[dict], edges: list[dict], incoming: dict[int, list[int]],
                            labels: dict[int, np.ndarray], frame_min: int, frame_max: int):
    cfg = frozen.PROTOCOL["candidate_graph_dp"]
    radius = float(sparse.P["label_radius_m"])

    def hit(nid: int) -> int:
        node = nodes[nid]
        gt = labels.get(node["frame"])
        return int(gt is not None and np.linalg.norm(node["center"] - gt) <= radius)

    hits, score, parent, count, start = {}, {}, {}, {}, {}
    for eid, edge in enumerate(edges):
        u, v = edge["u"], edge["v"]
        if nodes[u]["frame"] < frame_min or nodes[v]["frame"] > frame_max:
            continue
        best_hits = hit(u) + hit(v)
        best_score = nodes[u]["score"] + nodes[v]["score"] - edge["gap_penalty"]
        best_parent, best_count, best_start = None, 2, u
        for peid in incoming.get(u, []):
            if peid not in score:
                continue
            previous = edges[peid]
            accel_dt = 0.5 * (previous["dt"] + edge["dt"])
            acceleration = float(np.linalg.norm(edge["velocity"] - previous["velocity"]) / max(accel_dt, 1e-9))
            if acceleration > cfg["edge_max_acceleration_mps2"]:
                continue
            trial_hits = hits[peid] + hit(v)
            trial_score = score[peid] + nodes[v]["score"] - edge["gap_penalty"]
            if (trial_hits, trial_score) > (best_hits, best_score):
                best_hits, best_score = trial_hits, trial_score
                best_parent, best_count, best_start = peid, count[peid] + 1, start[peid]
        hits[eid], score[eid], parent[eid] = best_hits, best_score, best_parent
        count[eid], start[eid] = best_count, best_start
    qualifying = [eid for eid in score if count[eid] >= cfg["path_min_nodes"] and
                  nodes[edges[eid]["v"]]["time"] - nodes[start[eid]]["time"] >= cfg["path_min_duration_s"]]
    best = max(qualifying, key=lambda x: (hits[x], score[x])) if qualifying else None
    return {
        "hits": hits, "score": score, "parent": parent, "count": count,
        "qualifying": qualifying, "best_eid": best,
        "best_path": _reconstruct(best, edges, parent) if best is not None else tuple(),
        "best_hits": hits[best] if best is not None else 0,
        "best_score": score[best] if best is not None else None,
    }


def path_score(node_ids: tuple[int, ...] | list[int], nodes: list[dict], edge_lookup: dict[tuple[int, int], dict]):
    base = sum(nodes[n]["base_component"] for n in node_ids)
    compact = sum(nodes[n]["compactness_component"] for n in node_ids)
    support = sum(nodes[n]["support_component"] for n in node_ids)
    penalty = sum(edge_lookup[(u, v)]["gap_penalty"] for u, v in zip(node_ids[:-1], node_ids[1:]))
    return {"base_total": base, "compactness_total": compact, "support_total": support,
            "edge_gap_penalty_total": penalty, "cumulative_score": base + compact + support - penalty}


def annotate_legacy(nodes: list[dict], by_frame: list[list[int]], condition_id: str) -> None:
    frames, _ = temporal.load_leveled_frames(condition_id)
    stage = temporal.LegacyStageFilter()
    visit_bits: dict[int, int] = defaultdict(int)
    deltas = np.asarray([(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)])
    for fi, values in enumerate(frames):
        centers = np.asarray([nodes[nid]["center"] for nid in by_frame[fi]]) if by_frame[fi] else np.empty((0, 3))
        leveled = temporal.lidar_to_level(centers) if len(centers) else centers
        idx, valid, q = stage.indices(leveled) if len(centers) else (np.empty(0, int), np.empty(0, bool), np.empty((0, 3), int))
        # LegacyStageFilter increments its internal counter at the start of a
        # non-empty frame, so this is the exact denominator it will use next.
        denominator = stage.frame_count + 1
        for local, nid in enumerate(by_frame[fi]):
            node = nodes[nid]
            node["legacy_valid"] = bool(valid[local])
            if not valid[local]:
                for key in ("legacy_static_count_prior", "legacy_background_probability_prior",
                            "cell_revisit_frames_prior", "cell_revisit_fraction_prior",
                            "neighborhood_revisit_frames_prior", "neighborhood_revisit_fraction_prior"):
                    node[key] = math.nan
                node["legacy_static_or_initialization"] = math.nan
                continue
            cell = int(idx[local])
            count_prior = int(stage.counts[cell])
            prob = count_prior / denominator
            node["legacy_static_count_prior"] = count_prior
            node["legacy_background_probability_prior"] = prob
            cell_frames = int(visit_bits.get(cell, 0)).bit_count()
            neighborhood_bits = 0
            for delta in deltas:
                qq = q[local] + delta
                if np.all((qq >= 0) & (qq < np.asarray(stage.shape))):
                    ni = int(qq[0] * stage.shape[1] * stage.shape[2] + qq[1] * stage.shape[2] + qq[2])
                    neighborhood_bits |= visit_bits.get(ni, 0)
            neighborhood_frames = int(neighborhood_bits).bit_count()
            prior_frames = max(fi, 1)
            node["cell_revisit_frames_prior"] = cell_frames
            node["cell_revisit_fraction_prior"] = cell_frames / prior_frames if fi else 0.0
            node["neighborhood_revisit_frames_prior"] = neighborhood_frames
            node["neighborhood_revisit_fraction_prior"] = neighborhood_frames / prior_frames if fi else 0.0
            init = fi < temporal.PROTOCOL["legacy_corrected"]["background_init_frames"]
            static = prob >= temporal.PROTOCOL["legacy_corrected"]["background_probability_threshold"]
            node["legacy_static_or_initialization"] = bool(init or static)

        points = values[:, :3]
        stage.process(points) if len(points) else None
        if len(points):
            pidx, pvalid, _ = stage.indices(points)
            for cell in np.unique(pidx[pvalid]):
                visit_bits[int(cell)] |= 1 << fi


def add_path_kinematics(node_ids: tuple[int, ...] | list[int], nodes: list[dict]) -> list[dict]:
    rows = []
    velocities = []
    for order, nid in enumerate(node_ids):
        node = nodes[nid]
        row = {"node_id": nid, "path_order": order}
        if order == 0:
            row.update({"speed_mps": math.nan, "acceleration_mps2": math.nan,
                        "curvature_deg": math.nan, "geometric_range_rate_mps": math.nan,
                        "doppler_range_rate_residual_mps": math.nan})
        else:
            prev = nodes[node_ids[order - 1]]
            dt = node["time"] - prev["time"]
            velocity = (node["center"] - prev["center"]) / dt
            velocities.append(velocity)
            drdt = (np.linalg.norm(node["center"]) - np.linalg.norm(prev["center"])) / dt
            measured = float(node["candidate"]["radial_velocity"])
            row.update({
                "speed_mps": float(np.linalg.norm(velocity)),
                "geometric_range_rate_mps": float(drdt),
                "doppler_range_rate_residual_mps": abs(-measured - drdt),
            })
            if order >= 2:
                prevprev = nodes[node_ids[order - 2]]
                dt0 = prev["time"] - prevprev["time"]
                v0 = (prev["center"] - prevprev["center"]) / dt0
                adt = 0.5 * (dt0 + dt)
                row["acceleration_mps2"] = float(np.linalg.norm(velocity - v0) / adt)
                nv0, nv1 = np.linalg.norm(v0), np.linalg.norm(velocity)
                if nv0 > 1e-3 and nv1 > 1e-3:
                    cosine = float(np.clip(np.dot(v0, velocity) / (nv0 * nv1), -1.0, 1.0))
                    row["curvature_deg"] = math.degrees(math.acos(cosine))
                else:
                    row["curvature_deg"] = math.nan
            else:
                row["acceleration_mps2"] = math.nan
                row["curvature_deg"] = math.nan
        rows.append(row)
    return rows


def _finite(values) -> np.ndarray:
    out = np.asarray(values, dtype=float)
    return out[np.isfinite(out)]


def _stat(values, fn, default=""):
    a = _finite(values)
    return float(fn(a)) if len(a) else default


def summarize_path(condition_id: str, path_type: str, node_ids: tuple[int, ...] | list[int],
                   nodes: list[dict], edge_lookup: dict[tuple[int, int], dict],
                   labels: dict[int, np.ndarray], span_frames: int) -> tuple[dict, list[dict]]:
    kinematics = add_path_kinematics(node_ids, nodes)
    score = path_score(node_ids, nodes, edge_lookup)
    node_rows = []
    hits = 0
    for nid, kin in zip(node_ids, kinematics):
        node = nodes[nid]
        cand = node["candidate"]
        gt = labels.get(node["frame"])
        dist = float(np.linalg.norm(node["center"] - gt)) if gt is not None else math.nan
        hit = bool(gt is not None and dist <= sparse.P["label_radius_m"])
        hits += int(hit)
        row = {
            "condition_id": condition_id, "path_type": path_type, "path_order": kin["path_order"],
            "node_id": nid, "frame_index": node["frame"], "bag_time": node["time"],
            "center_x_m": node["center"][0], "center_y_m": node["center"][1], "center_z_m": node["center"][2],
            "range_m": float(np.linalg.norm(node["center"])), "raw_support": cand["raw_points"],
            "voxels": cand["voxels"], "extent_m": cand["extent"], "intensity": cand["intensity"],
            "measured_doppler_mps": cand["radial_velocity"], "node_score": node["score"],
            "base_component": node["base_component"], "compactness_component": node["compactness_component"],
            "support_component": node["support_component"], "silver_frame": gt is not None,
            "silver_distance_m": dist, "silver_match": hit,
            "speed_mps": kin["speed_mps"], "acceleration_mps2": kin["acceleration_mps2"],
            "curvature_deg": kin["curvature_deg"], "geometric_range_rate_mps": kin["geometric_range_rate_mps"],
            "doppler_range_rate_residual_mps": kin["doppler_range_rate_residual_mps"],
            "legacy_valid": node.get("legacy_valid", False),
            "legacy_static_count_prior": node.get("legacy_static_count_prior", math.nan),
            "legacy_background_probability_prior": node.get("legacy_background_probability_prior", math.nan),
            "cell_revisit_frames_prior": node.get("cell_revisit_frames_prior", math.nan),
            "cell_revisit_fraction_prior": node.get("cell_revisit_fraction_prior", math.nan),
            "neighborhood_revisit_frames_prior": node.get("neighborhood_revisit_frames_prior", math.nan),
            "neighborhood_revisit_fraction_prior": node.get("neighborhood_revisit_fraction_prior", math.nan),
            "legacy_static_or_initialization": node.get("legacy_static_or_initialization", math.nan),
        }
        node_rows.append(row)
    frames = [nodes[n]["frame"] for n in node_ids]
    times = [nodes[n]["time"] for n in node_ids]
    summary = {
        "condition_id": condition_id, "path_type": path_type, "nodes": len(node_ids),
        "first_frame": min(frames) if frames else "", "last_frame": max(frames) if frames else "",
        "duration_s": max(times) - min(times) if times else 0.0,
        "span_frame_coverage": len(set(frames)) / span_frames if span_frames else "",
        "silver_hits": hits, "silver_labels": len(labels), "silver_recall": hits / len(labels),
        **score,
        "node_score_mean": _stat([r["node_score"] for r in node_rows], np.mean),
        "extent_median_m": _stat([r["extent_m"] for r in node_rows], np.median),
        "raw_support_median": _stat([r["raw_support"] for r in node_rows], np.median),
        "speed_median_mps": _stat([r["speed_mps"] for r in node_rows], np.median),
        "speed_p90_mps": _stat([r["speed_mps"] for r in node_rows], lambda x: np.percentile(x, 90)),
        "acceleration_median_mps2": _stat([r["acceleration_mps2"] for r in node_rows], np.median),
        "acceleration_p90_mps2": _stat([r["acceleration_mps2"] for r in node_rows], lambda x: np.percentile(x, 90)),
        "curvature_median_deg": _stat([r["curvature_deg"] for r in node_rows], np.median),
        "range_median_m": _stat([r["range_m"] for r in node_rows], np.median),
        "measured_doppler_median_mps": _stat([r["measured_doppler_mps"] for r in node_rows], np.median),
        "measured_abs_doppler_median_mps": _stat([abs(r["measured_doppler_mps"]) for r in node_rows], np.median),
        "doppler_residual_median_mps": _stat([r["doppler_range_rate_residual_mps"] for r in node_rows], np.median),
        "legacy_valid_fraction": float(np.mean([bool(r["legacy_valid"]) for r in node_rows])) if node_rows else "",
        "legacy_background_probability_median": _stat([r["legacy_background_probability_prior"] for r in node_rows], np.median),
        "cell_revisit_fraction_median": _stat([r["cell_revisit_fraction_prior"] for r in node_rows], np.median),
        "neighborhood_revisit_fraction_median": _stat([r["neighborhood_revisit_fraction_prior"] for r in node_rows], np.median),
        "legacy_static_or_initialization_fraction": _stat(
            [float(r["legacy_static_or_initialization"]) for r in node_rows
             if isinstance(r["legacy_static_or_initialization"], (bool, np.bool_))], np.mean),
    }
    return summary, node_rows


def verify_recorded_top(condition_id: str, top_nodes: tuple[int, ...], nodes: list[dict]) -> None:
    rows = [x for x in read_csv(SOURCE / "candidate_detections.csv")
            if x["method"] == "candidate_graph_DP" and x["condition_id"] == condition_id]
    recorded = {int(x["frame_index"]): np.asarray([float(x["center_x_m"]), float(x["center_y_m"]), float(x["center_z_m"])]) for x in rows}
    replayed = {nodes[n]["frame"]: nodes[n]["center"] for n in top_nodes}
    if set(recorded) != set(replayed):
        raise AssertionError(f"{condition_id}: replayed top path frames differ from recorded output")
    error = max((float(np.linalg.norm(recorded[f] - replayed[f])) for f in recorded), default=0.0)
    if error > 1e-5:
        raise AssertionError(f"{condition_id}: top path replay mismatch {error}")


def top_state_winners(dp_result: dict, nodes: list[dict], edges: list[dict], limit: int):
    unique, seen = [], set()
    for eid in sorted(dp_result["qualifying"], key=lambda x: dp_result["score"][x], reverse=True):
        path = _reconstruct(eid, edges, dp_result["parent"])
        if path in seen:
            continue
        seen.add(path)
        unique.append((eid, path, dp_result["score"][eid]))
        if len(unique) >= limit:
            break
    return unique


def separation_rows(all_node_rows: list[dict]) -> list[dict]:
    paired = []
    # Use only silver frames, matching the silver-feasible hit node to the top-clutter node at that frame.
    for condition_id in CONDITIONS:
        target = {(int(x["frame_index"])): x for x in all_node_rows
                  if x["condition_id"] == condition_id and x["path_type"] == "silver_feasible" and x["silver_match"]}
        clutter = {(int(x["frame_index"])): x for x in all_node_rows
                   if x["condition_id"] == condition_id and x["path_type"] == "top_clutter_full" and x["silver_frame"]}
        for frame in sorted(set(target) & set(clutter)):
            paired.append((condition_id, target[frame], clutter[frame]))
    features = {
        "node_score": lambda x: x["node_score"],
        "compactness_component": lambda x: x["compactness_component"],
        "support_component": lambda x: x["support_component"],
        "extent_m": lambda x: x["extent_m"],
        "raw_support": lambda x: x["raw_support"],
        "speed_mps": lambda x: x["speed_mps"],
        "acceleration_mps2": lambda x: x["acceleration_mps2"],
        "curvature_deg": lambda x: x["curvature_deg"],
        "range_m": lambda x: x["range_m"],
        "abs_measured_doppler_mps": lambda x: abs(float(x["measured_doppler_mps"])),
        "doppler_residual_mps": lambda x: x["doppler_range_rate_residual_mps"],
        "legacy_background_probability": lambda x: x["legacy_background_probability_prior"],
        "cell_revisit_fraction": lambda x: x["cell_revisit_fraction_prior"],
        "neighborhood_revisit_fraction": lambda x: x["neighborhood_revisit_fraction_prior"],
    }
    rows = []
    for feature, getter in features.items():
        target_values, clutter_values = [], []
        per_condition = defaultdict(lambda: [[], []])
        for cid, target, clutter in paired:
            tv, cv = float(getter(target)), float(getter(clutter))
            if not (np.isfinite(tv) and np.isfinite(cv)):
                continue
            target_values.append(tv); clutter_values.append(cv)
            per_condition[cid][0].append(tv); per_condition[cid][1].append(cv)
        if not target_values:
            continue
        y = np.r_[np.ones(len(target_values)), np.zeros(len(clutter_values))]
        values = np.r_[target_values, clutter_values]
        auc_high = float(roc_auc_score(y, values)) if len(np.unique(values)) > 1 else 0.5
        direction = "target_higher" if auc_high > 0.5 else ("target_lower" if auc_high < 0.5 else "tie")
        consistent = 0
        condition_directions = []
        for cid in CONDITIONS:
            tv, cv = per_condition[cid]
            if not tv:
                condition_directions.append(f"{cid}:NA")
                continue
            delta = float(np.median(tv) - np.median(cv))
            sign = "higher" if delta > 0 else ("lower" if delta < 0 else "tie")
            condition_directions.append(f"{cid}:{sign}")
            consistent += int((direction == "target_higher" and delta > 0) or
                              (direction == "target_lower" and delta < 0))
        rows.append({
            "feature": feature, "paired_frames": len(target_values),
            "target_median": float(np.median(target_values)), "clutter_median": float(np.median(clutter_values)),
            "target_high_auc": auc_high, "separation_auc": max(auc_high, 1.0 - auc_high),
            "pooled_direction": direction, "sequences_consistent_with_pooled_direction": consistent,
            "sequence_directions": ";".join(condition_directions),
            "status": "DESCRIPTIVE_POST_HOC_DEV_SILVER",
        })
    return rows


def score_comparison_rows(summaries: list[dict]) -> list[dict]:
    rows = []
    for condition_id in CONDITIONS:
        lookup = {x["path_type"]: x for x in summaries if x["condition_id"] == condition_id}
        full = lookup["top_clutter_full"]
        clutter = lookup["top_clutter_label_span"]
        uav = lookup["silver_feasible"]
        row = {
            "condition_id": condition_id,
            "full_clutter_nodes": full["nodes"], "full_clutter_score": full["cumulative_score"],
            "span_clutter_nodes": clutter["nodes"], "silver_path_nodes": uav["nodes"],
            "span_clutter_score": clutter["cumulative_score"], "silver_path_score": uav["cumulative_score"],
        }
        for key in ("base_total", "compactness_total", "support_total", "edge_gap_penalty_total",
                    "node_score_mean", "raw_support_median", "speed_median_mps",
                    "acceleration_median_mps2", "curvature_median_deg",
                    "measured_abs_doppler_median_mps", "doppler_residual_median_mps",
                    "legacy_background_probability_median", "cell_revisit_fraction_median",
                    "neighborhood_revisit_fraction_median"):
            row[f"span_clutter_{key}"] = clutter[key]
            row[f"silver_{key}"] = uav[key]
            row[f"clutter_minus_silver_{key}"] = float(clutter[key]) - float(uav[key])
        rows.append(row)
    return rows


def run() -> None:
    if not (OUT / "FROZEN").exists():
        raise RuntimeError("run freeze before forensics")
    labels_all = read_csv(DEV / "silver_labels.csv")
    summaries, node_rows, ranks, graph_rows = [], [], [], []
    for condition_id in CONDITIONS:
        labels_rows = [x for x in labels_all if x["condition_id"] == condition_id]
        labels = {int(x["frame_index"]): np.asarray([float(x["center_x_m"]), float(x["center_y_m"]), float(x["center_z_m"])])
                  for x in labels_rows}
        frame_min, frame_max = min(labels), max(labels)
        candidates_input, times = sparse._load_raw_voxel_frames(condition_id)
        candidates, _ = sparse.cluster_frames(candidates_input, min_pts=1, adaptive=True)
        nodes, by_frame, edges, incoming = build_graph(candidates, times)
        annotate_legacy(nodes, by_frame, condition_id)
        edge_lookup = {(x["u"], x["v"]): x for x in edges}

        full = ordinary_dp(nodes, edges, incoming)
        verify_recorded_top(condition_id, full["best_path"], nodes)
        span = ordinary_dp(nodes, edges, incoming, frame_min, frame_max)
        silver = silver_lexicographic_dp(nodes, edges, incoming, labels, frame_min, frame_max)
        top_span_nodes = tuple(n for n in full["best_path"] if frame_min <= nodes[n]["frame"] <= frame_max)
        if any((u, v) not in edge_lookup for u, v in zip(top_span_nodes[:-1], top_span_nodes[1:])):
            raise AssertionError("top-span restriction unexpectedly broke graph adjacency")

        for path_type, path in (("top_clutter_full", full["best_path"]),
                                ("top_clutter_label_span", top_span_nodes),
                                ("silver_feasible", silver["best_path"])):
            summary, rows = summarize_path(condition_id, path_type, path, nodes, edge_lookup,
                                           labels, frame_max - frame_min + 1)
            summaries.append(summary); node_rows.extend(rows)

        winners = top_state_winners(span, nodes, edges, TOP_K_STATE_WINNERS)
        first_full_hit_rank = ""
        maximum_topk_hits = 0
        silver_tuple = tuple(silver["best_path"])
        silver_exact_rank = ""
        for rank, (_, path, _) in enumerate(winners, 1):
            hits = sum(nodes[n]["frame"] in labels and
                       np.linalg.norm(nodes[n]["center"] - labels[nodes[n]["frame"]]) <= sparse.P["label_radius_m"]
                       for n in path)
            maximum_topk_hits = max(maximum_topk_hits, hits)
            if hits == len(labels) and first_full_hit_rank == "":
                first_full_hit_rank = rank
            if path == silver_tuple and silver_exact_rank == "":
                silver_exact_rank = rank
        silver_score = float(silver["best_score"])
        rank_lower = 1 + sum(span["score"][eid] > silver_score for eid in span["qualifying"])
        ranks.append({
            "condition_id": condition_id, "silver_labels": len(labels),
            "silver_path_hits": silver["best_hits"], "full_silver_path_exists": silver["best_hits"] == len(labels),
            "silver_path_score": silver_score, "top_span_score": span["score"][span["best_eid"]],
            "score_rank_lower_bound_among_state_winners": rank_lower,
            "qualifying_edge_state_winners": len(span["qualifying"]),
            "top_k_state_winners_checked": len(winners),
            "maximum_silver_hits_in_top_k": maximum_topk_hits,
            "first_full_hit_path_rank_in_top_k": first_full_hit_rank,
            "exact_silver_path_rank_in_top_k": silver_exact_rank,
            "exact_rank_among_all_raw_paths": "UNKNOWN_COMBINATORIAL_NOT_RETAINED_BY_ORIGINAL_DP",
        })
        graph_rows.append({
            "condition_id": condition_id, "nodes": len(nodes), "edges": len(edges),
            "full_qualifying_state_winners": len(full["qualifying"]),
            "span_first_frame": frame_min, "span_last_frame": frame_max,
            "span_qualifying_state_winners": len(span["qualifying"]),
            "top_replay_verified": True,
        })
        print(f"forensics {condition_id}: silver feasible {silver['best_hits']}/{len(labels)}, rank lower bound {rank_lower}")

    separation = separation_rows(node_rows)
    comparisons = score_comparison_rows(summaries)
    write_csv(OUT / "path_summary.csv", summaries)
    write_csv(OUT / "path_nodes.csv", node_rows)
    write_csv(OUT / "score_component_comparison.csv", comparisons)
    write_csv(OUT / "rank_diagnostics.csv", ranks)
    write_csv(OUT / "feature_separation.csv", separation)
    write_csv(OUT / "graph_replay.csv", graph_rows)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "git_head_before_run": _git_head(),
        "script_sha256": sha256(Path(__file__)), "protocol_sha256": sha256(OUT / "protocol.json"),
        "source_dp_protocol_sha256": sha256(SOURCE / "protocol.json"),
        "labels_sha256": sha256(DEV / "silver_labels.csv"),
        "command": "python3 scripts/dp_failure_forensics.py run",
    }
    (OUT / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["freeze", "run", "all"])
    args = parser.parse_args()
    if args.action in {"freeze", "all"}:
        freeze()
    if args.action in {"run", "all"}:
        run()


if __name__ == "__main__":
    main()
