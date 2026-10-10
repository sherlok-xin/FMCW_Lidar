#!/usr/bin/env python3
"""Isolated, label-free causal annotations and exact qualified-path DP.

This module does not load research data, generate candidates, or alter the
historical graph. Parameters below are the prospectively specified MVP values.
Background probability is a Beta-smoothed occupancy proxy, not a calibrated
clutter likelihood. The whole-graph optimizer is offline even when annotations
use prior frames only.
"""
from __future__ import annotations

from collections import defaultdict
import math

import numpy as np


MAX_FRAME_GAP = 3
MAX_DT_S = 3.5
MAX_SPEED_MPS = 25.0
MAX_ACCELERATION_MPS2 = 15.0
PATH_MIN_NODES = 3
PATH_MIN_DURATION_S = 2.0
BACKGROUND_WEIGHT = 1.0
Q = "Q"
METHODS = ("original_score", "background_motion")


def _cell(point):
    xyz = np.asarray(point, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("background cell requires finite sensor XYZ")
    return tuple(np.floor(xyz).astype(np.int64).tolist())


def annotate_background(nodes, by_frame, fine_frames):
    """Annotate every node before updating current-frame occupancy.

    Grid: sensor-relative zero-origin 1 m cells. Each occupied cell contributes
    once per frame, including frames with no candidates. Empty frames remain
    in the prior-frame denominator. Singleton nodes are never deleted.
    """
    if len(by_frame) != len(fine_frames):
        raise ValueError("by_frame and fine_frames must cover the same frames")
    prior_hits = defaultdict(int)
    for fi, node_ids in enumerate(by_frame):
        for nid in node_ids:
            node = nodes[nid]
            if int(node["frame"]) != fi:
                raise ValueError("node belongs to a different frame")
            cell = _cell(node["center"])
            hits = int(prior_hits[cell])
            probability = (hits + 1.0) / (fi + 2.0)
            term = BACKGROUND_WEIGHT * (1.0 - 2.0 * probability)
            node.update({
                "background_cell_xyz": cell,
                "background_prior_hits": hits,
                "background_prior_frames": fi,
                "background_p_beta11": probability,
                "background_evidence_term": term,
                "new_score": float(node["score"]) + term,
            })
        values = np.asarray(fine_frames[fi])
        if not len(values):
            continue
        if values.ndim != 2 or values.shape[1] < 3:
            raise ValueError("fine frames must contain XYZ in their first columns")
        xyz = np.asarray(values[:, :3], dtype=float)
        if not np.isfinite(xyz).all():
            raise ValueError("fine-frame XYZ must be finite")
        for cell in np.unique(np.floor(xyz).astype(np.int64), axis=0):
            prior_hits[tuple(cell.tolist())] += 1
    return nodes


def motion_parts(previous, edge):
    """Return the fixed soft speed/direction terms; no Doppler is accessed."""
    adt = 0.5 * (float(previous["dt"]) + float(edge["dt"]))
    if not math.isfinite(adt) or adt <= 0:
        raise ValueError("motion terms require positive real time intervals")
    v0 = np.asarray(previous["velocity"], dtype=float)
    v = np.asarray(edge["velocity"], dtype=float)
    if v.shape != (3,) or v0.shape != (3,) or not np.isfinite(v).all() or not np.isfinite(v0).all():
        raise ValueError("motion terms require finite XYZ edge velocities")
    speed0, speed = float(np.linalg.norm(v0)), float(np.linalg.norm(v))
    product = speed * speed0
    speed_penalty = 0.5 * ((speed - speed0) / (MAX_ACCELERATION_MPS2 * adt)) ** 2
    direction_penalty = 0.5 * (product - float(np.dot(v, v0))) / (product + (1.0 / adt) ** 2)
    return {
        "speed_penalty": max(float(speed_penalty), 0.0),
        "direction_penalty": max(float(direction_penalty), 0.0),
        "accel_dt_s": adt,
        "acceleration_mps2": float(np.linalg.norm(v - v0)) / adt,
        "speed_mps": speed,
        "previous_speed_mps": speed0,
    }


def _admissible_edge(nodes, edge, frame_min, frame_max, speed=None, dt=None):
    u, v = int(edge["u"]), int(edge["v"])
    first, last = int(nodes[u]["frame"]), int(nodes[v]["frame"])
    if frame_min is not None and first < frame_min:
        return False
    if frame_max is not None and last > frame_max:
        return False
    dt = float(edge["dt"]) if dt is None else dt
    speed = float(np.linalg.norm(np.asarray(edge["velocity"], dtype=float))) if speed is None else speed
    return (1 <= last - first <= MAX_FRAME_GAP and
            0 < dt <= MAX_DT_S and math.isfinite(speed) and speed <= MAX_SPEED_MPS)


def _state_key(count, start_frame, duration):
    if count >= PATH_MIN_NODES and duration >= PATH_MIN_DURATION_S:
        return Q
    return (min(count, PATH_MIN_NODES), start_frame)


def _put(states, key, state):
    if key not in states or state["score"] > states[key]["score"]:
        states[key] = state


def solve(nodes, edges, incoming, method="original_score", frame_min=None, frame_max=None):
    """Maximize the specified score over paths satisfying all frozen gates.

    An edge is the second-order velocity state. Until a history meets both
    minimum count and duration, it retains (capped count, start frame). Once
    qualified, all histories at that edge can safely merge into Q: future
    transition score/acceleration only depend on the edge, and qualification
    cannot be lost. This handles negative node scores without discarding a
    legal long path in favor of an unqualified short path.
    """
    if method not in METHODS:
        raise ValueError(f"unknown score method: {method}")
    # start_frame keys are sufficient only when all nodes in a frame have the
    # same acquisition timestamp. Check this contract rather than assume it.
    frame_times = {}
    for node in nodes:
        frame, stamp = int(node["frame"]), float(node["time"])
        if not math.isfinite(stamp):
            raise ValueError("node timestamp must be finite")
        if frame in frame_times and stamp != frame_times[frame]:
            raise ValueError("one timestamp per frame is required")
        frame_times[frame] = stamp
    score_field = "score" if method == "original_score" else "new_score"
    node_scores = [float(node[score_field]) for node in nodes]
    if not all(math.isfinite(x) for x in node_scores):
        raise ValueError("node scores must be finite")
    # Scalar norm calls are deliberately identical to motion_parts. Cache
    # their results, not row-vectorized approximations that may change an ULP
    # at a hard gate or a strict-score tie.
    velocities = [np.asarray(edge["velocity"], dtype=float) for edge in edges]
    edge_dt = [float(edge["dt"]) for edge in edges]
    speeds = [float(np.linalg.norm(v)) for v in velocities]
    edge_order = sorted(range(len(edges)), key=lambda eid: (int(nodes[edges[eid]["v"]]["frame"]), eid))
    states = {}
    for eid in edge_order:
        edge = edges[eid]
        if not _admissible_edge(nodes, edge, frame_min, frame_max, speeds[eid], edge_dt[eid]):
            continue
        u, v = int(edge["u"]), int(edge["v"])
        dt = edge_dt[eid]
        if not math.isclose(float(nodes[v]["time"]) - float(nodes[u]["time"]), dt,
                            rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("edge dt disagrees with acquisition timestamps")
        gap = float(edge["gap_penalty"])
        if not math.isfinite(gap):
            raise ValueError("gap penalty must be finite")
        current = {}
        root = {
            "score": node_scores[u] + node_scores[v] - gap,
            "count": 2, "start_frame": int(nodes[u]["frame"]),
            "start_time": float(nodes[u]["time"]),
            "parent_eid": None, "parent_state_key": None,
            "node_score_total": node_scores[u] + node_scores[v],
            "gap_penalty_total": gap,
            "speed_penalty_total": 0.0, "direction_penalty_total": 0.0,
        }
        _put(current, _state_key(2, root["start_frame"], dt), root)
        for peid in incoming.get(u, []):
            if peid not in states:
                continue
            previous = edges[peid]
            if int(previous["v"]) != u:
                raise ValueError("incoming edge does not terminate at current u")
            adt = 0.5 * (edge_dt[peid] + dt)
            velocity0, velocity = velocities[peid], velocities[eid]
            # Keep the original scalar norm/division operation for the gate.
            acceleration = float(np.linalg.norm(velocity - velocity0)) / adt
            if acceleration > MAX_ACCELERATION_MPS2:
                continue
            sp = dp = 0.0
            if method == "background_motion":
                speed0, speed = speeds[peid], speeds[eid]
                product = speed * speed0
                sp = max(float(0.5 * ((speed - speed0) / (MAX_ACCELERATION_MPS2 * adt)) ** 2), 0.0)
                dp = max(float(0.5 * (product - float(np.dot(velocity, velocity0))) /
                               (product + (1.0 / adt) ** 2)), 0.0)
            for pkey, prior in states[peid].items():
                # Compare in exactly the original arithmetic/strict-tie order
                # before allocating a losing history's large descriptor dict.
                trial_score = prior["score"] + node_scores[v] - gap - sp - dp
                count = prior["count"] + 1
                duration = float(nodes[v]["time"]) - prior["start_time"]
                key = _state_key(count, prior["start_frame"], duration)
                if key in current and not (trial_score > current[key]["score"]):
                    continue
                state = {
                    "score": trial_score,
                    "count": count,
                    "start_frame": prior["start_frame"], "start_time": prior["start_time"],
                    "parent_eid": peid, "parent_state_key": pkey,
                    "node_score_total": prior["node_score_total"] + node_scores[v],
                    "gap_penalty_total": prior["gap_penalty_total"] + gap,
                    "speed_penalty_total": prior["speed_penalty_total"] + sp,
                    "direction_penalty_total": prior["direction_penalty_total"] + dp,
                }
                current[key] = state
        states[eid] = current
    qualifying = [eid for eid in edge_order if eid in states and Q in states[eid]]
    qualified_scores = {eid: states[eid][Q]["score"] for eid in qualifying}
    best_eid = max(qualifying, key=lambda eid: qualified_scores[eid]) if qualifying else None
    result = {
        "states": states, "qualifying": qualifying, "qualified_scores": qualified_scores,
        "best_eid": best_eid, "best_score": qualified_scores[best_eid] if best_eid is not None else None,
        "edges": edges, "method": method,
    }
    result["best_path"] = reconstruct(result, best_eid) if best_eid is not None else tuple()
    return result


def reconstruct(result, eid):
    """Reconstruct the qualified winner of an edge, not an all-path rank."""
    if eid not in result["states"] or Q not in result["states"][eid]:
        raise ValueError("edge has no qualified path winner")
    edges, states = result["edges"], result["states"]
    reverse, key = [int(edges[eid]["v"])], Q
    while eid is not None:
        reverse.append(int(edges[eid]["u"]))
        state = states[eid][key]
        eid, key = state["parent_eid"], state["parent_state_key"]
    return tuple(reversed(reverse))
