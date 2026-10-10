"""Synthetic-only causal/DP correctness tests; no DEV labels are read."""
from collections import defaultdict
import importlib.util
import itertools
from pathlib import Path
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location("background_core", Path(__file__).parents[1] / "scripts/background_motion_core.py")
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


def synthetic_graph(times, positions, scores, new_scores=None):
    nodes, by_frame = [], []
    for fi, (stamp, frame) in enumerate(zip(times, positions)):
        ids = []
        for ci, point in enumerate(frame):
            nid = len(nodes)
            ids.append(nid)
            nodes.append({"id": nid, "frame": fi, "time": float(stamp), "center": np.asarray(point, dtype=float),
                          "score": float(scores[fi][ci]),
                          "new_score": float((new_scores if new_scores is not None else scores)[fi][ci])})
        by_frame.append(ids)
    edges, incoming = [], defaultdict(list)
    for fi in range(len(by_frame)):
        for gap in range(1, 4):
            pi = fi - gap
            if pi < 0:
                continue
            for u, v in itertools.product(by_frame[pi], by_frame[fi]):
                dt = nodes[v]["time"] - nodes[u]["time"]
                if dt <= 0:
                    continue
                vel = (nodes[v]["center"] - nodes[u]["center"]) / dt
                if dt > 3.5 or np.linalg.norm(vel) > 25:
                    continue
                eid = len(edges)
                edges.append({"u": u, "v": v, "dt": dt, "velocity": vel, "gap_penalty": .25 * (gap - 1)})
                incoming[v].append(eid)
    return nodes, by_frame, edges, incoming


def exhaustive(nodes, edges, method):
    """Enumerate every synthetic graph path, independently of DP states."""
    outgoing = defaultdict(list)
    for eid, e in enumerate(edges):
        outgoing[e["u"]].append(eid)
    winners = {}
    field = "score" if method == "original_score" else "new_score"

    def expand(path, path_edges, score):
        if len(path) >= 3 and nodes[path[-1]]["time"] - nodes[path[0]]["time"] >= 2:
            last = path_edges[-1]
            if last not in winners or score > winners[last][0]:
                winners[last] = (score, tuple(path))
        for eid in outgoing[path[-1]]:
            edge = edges[eid]
            v = edge["v"]
            gap = nodes[v]["frame"] - nodes[edge["u"]]["frame"]
            if not (1 <= gap <= 3 and 0 < edge["dt"] <= 3.5 and np.linalg.norm(edge["velocity"]) <= 25):
                continue
            penalty = 0.0
            if path_edges:
                previous = edges[path_edges[-1]]
                dt = .5 * (previous["dt"] + edge["dt"])
                v0, v1 = previous["velocity"], edge["velocity"]
                if np.linalg.norm(v1 - v0) / dt > 15:
                    continue
                if method == "background_motion":
                    s0, s1 = np.linalg.norm(v0), np.linalg.norm(v1)
                    penalty = .5 * ((s1 - s0) / (15 * dt)) ** 2
                    penalty += max(.5 * (s0 * s1 - np.dot(v0, v1)) / (s0 * s1 + (1 / dt) ** 2), 0)
            expand(path + [v], path_edges + [eid], score + nodes[v][field] - edge["gap_penalty"] - penalty)

    for nid in range(len(nodes)):
        expand([nid], [], nodes[nid][field])
    return winners


class BackgroundMotionTests(unittest.TestCase):
    def assert_exhaustive(self, graph, method):
        nodes, _, edges, incoming = graph
        result = core.solve(nodes, edges, incoming, method)
        enumerated = exhaustive(nodes, edges, method)
        self.assertEqual(set(result["qualifying"]), set(enumerated))
        for eid, (score, _) in enumerated.items():
            self.assertAlmostEqual(result["qualified_scores"][eid], score, places=12)
            path = core.reconstruct(result, eid)
            self.assertGreaterEqual(len(path), 3)
            self.assertGreaterEqual(nodes[path[-1]]["time"] - nodes[path[0]]["time"], 2)
        if enumerated:
            self.assertAlmostEqual(result["best_score"], max(v[0] for v in enumerated.values()), places=12)
        else:
            self.assertIsNone(result["best_score"])
            self.assertEqual(result["best_path"], tuple())

    def test_exhaustive_both_methods_with_negative_nodes(self):
        positions = [[[100 + fi, 0, 0], [101, 1 + .4 * fi, 0]] for fi in range(5)]
        old = [[.2, -.3], [-2, .4], [.1, -.8], [.4, .1], [-.1, .5]]
        new = [[-.4, .2], [-.5, -.6], [.3, .1], [-.3, .5], [.2, -.2]]
        graph = synthetic_graph([0, .9, 1.95, 3.05, 4.], positions, old, new)
        for method in core.METHODS:
            with self.subTest(method=method):
                self.assert_exhaustive(graph, method)

    def test_short_winner_cannot_delete_qualified_negative_history(self):
        graph = synthetic_graph([0, 1, 2], [[[100, 0, 0]]] * 3, [[-5], [1], [1]])
        for method in core.METHODS:
            result = core.solve(graph[0], graph[2], graph[3], method)
            self.assertEqual(result["best_path"], (0, 1, 2))
            self.assertEqual(result["best_score"], -3)
            self.assert_exhaustive(graph, method)

    def test_duration_and_capped_count_states_survive_to_qualification(self):
        graph = synthetic_graph([0, .4, .8, 1.2, 1.6, 2.],
                                [[[100, 0, 0]]] * 6, [[-.2]] * 6)
        for method in core.METHODS:
            self.assert_exhaustive(graph, method)
            result = core.solve(graph[0], graph[2], graph[3], method)
            self.assertEqual(graph[0][result["best_path"][0]]["frame"], 0)
            self.assertEqual(graph[0][result["best_path"][-1]]["frame"], 5)

    def test_query_before_update_unique_cell_and_singletons(self):
        nodes = [{"frame": i, "center": np.array([100.1, 0.1, .1]), "score": .9, "voxels": 1}
                 for i in range(3)]
        by_frame = [[0], [1], [2]]
        frames = [np.array([[100.1, .1, .1], [100.2, .2, .2]]),
                  np.zeros((0, 6)), np.array([[100.3, .3, .3]])]
        core.annotate_background(nodes, by_frame, frames)
        self.assertEqual([n["background_prior_hits"] for n in nodes], [0, 1, 1])
        self.assertEqual([n["background_prior_frames"] for n in nodes], [0, 1, 2])
        np.testing.assert_allclose([n["background_p_beta11"] for n in nodes], [.5, 2/3, .5])
        self.assertEqual(len(nodes), 3)
        self.assertTrue(all(n["voxels"] == 1 for n in nodes))

    def test_background_prefix_invariant_to_future_frames(self):
        def nodes(n):
            return [{"frame": i, "center": np.array([100.1 + i, .1, .1]), "score": .8} for i in range(n)]
        frames = [np.array([[100.1 + i, .1, .1]]) for i in range(5)]
        a, b = nodes(3), nodes(5)
        core.annotate_background(a, [[i] for i in range(3)], frames[:3])
        core.annotate_background(b, [[i] for i in range(5)], frames)
        for old, extended in zip(a, b):
            self.assertEqual(old["new_score"], extended["new_score"])
            self.assertEqual(old["background_prior_hits"], extended["background_prior_hits"])

    def test_motion_uses_real_dt_and_static_direction_is_finite(self):
        previous = {"dt": .5, "velocity": np.array([2., 0, 0])}
        edge = {"dt": 1.5, "velocity": np.array([5., 0, 0])}
        parts = core.motion_parts(previous, edge)
        self.assertEqual(parts["accel_dt_s"], 1.)
        self.assertAlmostEqual(parts["speed_penalty"], .5 * (3 / 15) ** 2)
        self.assertEqual(parts["direction_penalty"], 0.)
        static = {"dt": 1., "velocity": np.zeros(3)}
        self.assertEqual(core.motion_parts(static, static)["direction_penalty"], 0.)
        self.assertTrue(all(np.isfinite(v) for v in core.motion_parts(static, static).values()))

    def test_original_method_does_not_apply_motion_score(self):
        graph = synthetic_graph([0, 1, 2], [[[100, 0, 0]], [[101, 0, 0]], [[103, 0, 0]]], [[1]] * 3)
        old = core.solve(graph[0], graph[2], graph[3], "original_score")
        new = core.solve(graph[0], graph[2], graph[3], "background_motion")
        self.assertEqual(old["best_score"], 3.)
        self.assertLess(new["best_score"], old["best_score"])

    def test_acceleration_gate_gap_gate_and_no_doppler_access(self):
        # Both consecutive speeds are legal (0 and 20 m/s), but their
        # acceleration is 20 m/s^2, above the unchanged 15 m/s^2 gate.
        graph = synthetic_graph([0, 1, 2], [[[100, 0, 0]], [[100, 0, 0]], [[120, 0, 0]]], [[1]] * 3)
        self.assertEqual(core.solve(graph[0], graph[2], graph[3])["best_path"], tuple())
        valid = synthetic_graph([0, 1, 2], [[[100, 0, 0]]] * 3, [[1]] * 3)
        for node in valid[0]:
            node["radial_velocity"] = object()
        for method in core.METHODS:
            self.assertEqual(core.solve(valid[0], valid[2], valid[3], method)["best_path"], (0, 1, 2))
        nodes = [{"frame": i, "time": float(i), "score": 1., "center": np.zeros(3)} for i in (0, 4, 5)]
        edges = [{"u": 0, "v": 1, "dt": 1., "velocity": np.zeros(3), "gap_penalty": 0.}]
        self.assertEqual(core.solve(nodes, edges, {1: [0]})["best_path"], tuple())

    def test_time_and_speed_edge_gates_not_score_thresholds(self):
        nodes = [{"frame": i, "time": t, "score": 1., "center": np.zeros(3)}
                 for i, t in enumerate((0., 4., 5.))]
        edges = [{"u": 0, "v": 1, "dt": 4., "velocity": np.zeros(3), "gap_penalty": 0.}]
        self.assertEqual(core.solve(nodes, edges, {1: [0]})["states"], {})
        nodes[1]["time"] = 1.
        edges[0]["dt"] = 1.
        edges[0]["velocity"] = np.array([25.01, 0., 0.])
        self.assertEqual(core.solve(nodes, edges, {1: [0]})["states"], {})

    def test_frame_span_restriction_and_timestamp_contract(self):
        graph = synthetic_graph([0, 1, 2, 3, 4], [[[100, 0, 0]]] * 5, [[1]] * 5)
        result = core.solve(graph[0], graph[2], graph[3], frame_min=1, frame_max=3)
        self.assertEqual(result["best_path"], (1, 2, 3))
        graph[0][1]["time"] = 1.1
        with self.assertRaisesRegex(ValueError, "edge dt disagrees"):
            core.solve(graph[0], graph[2], graph[3])


if __name__ == "__main__":
    unittest.main()
