"""Synthetic-only term-ablation checks; never open DEV data or silver labels."""
from collections import defaultdict
import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import background_motion_core as frozen_core
import e18_mechanism_audit as audit
from test_background_motion_core import synthetic_graph


ARMS = ("original", "background_only", "motion_only", "background_motion_E18")


def enabled(arm):
    return arm in ("background_only", "background_motion_E18"), arm in ("motion_only", "background_motion_E18")


def independent_motion(previous, edge):
    """Write out frozen equations without calling the production term helper."""
    adt = (float(previous["dt"]) + float(edge["dt"])) / 2.0
    v0, v1 = previous["velocity"], edge["velocity"]
    s0, s1 = float(np.linalg.norm(v0)), float(np.linalg.norm(v1))
    speed = max(0.5 * ((s1 - s0) / (15.0 * adt)) ** 2, 0.0)
    direction = max(0.5 * (s0 * s1 - float(np.dot(v0, v1))) /
                    (s0 * s1 + (1.0 / adt) ** 2), 0.0)
    return speed, direction


def enumerate_qualified_paths(nodes, edges, arm, frame_min=None, frame_max=None):
    """Independent all-path enumeration on tiny graphs, including negatives."""
    background, motion = enabled(arm)
    field = "new_score" if background else "score"
    outgoing = defaultdict(list)
    for eid, edge in enumerate(edges):
        outgoing[edge["u"]].append(eid)
    winners = {}

    def visit(path, path_edges, score):
        duration = nodes[path[-1]]["time"] - nodes[path[0]]["time"]
        if len(path) >= 3 and duration >= 2.0:
            last = path_edges[-1]
            if last not in winners or score > winners[last][0]:
                winners[last] = (score, tuple(path))
        for eid in outgoing[path[-1]]:
            edge = edges[eid]
            u, v = edge["u"], edge["v"]
            first, last = nodes[u]["frame"], nodes[v]["frame"]
            if frame_min is not None and first < frame_min:
                continue
            if frame_max is not None and last > frame_max:
                continue
            if not (1 <= last - first <= 3 and 0.0 < edge["dt"] <= 3.5 and
                    np.linalg.norm(edge["velocity"]) <= 25.0):
                continue
            speed = direction = 0.0
            if path_edges:
                previous = edges[path_edges[-1]]
                adt = (previous["dt"] + edge["dt"]) / 2.0
                if np.linalg.norm(edge["velocity"] - previous["velocity"]) / adt > 15.0:
                    continue
                if motion:
                    speed, direction = independent_motion(previous, edge)
            visit(path + [v], path_edges + [eid],
                  score + nodes[v][field] - edge["gap_penalty"] - speed - direction)

    for nid, node in enumerate(nodes):
        if frame_min is not None and node["frame"] < frame_min:
            continue
        if frame_max is not None and node["frame"] > frame_max:
            continue
        visit([nid], [], node[field])
    return winners


def with_parts(graph):
    """Populate reporting fields without needing candidates or research input."""
    nodes, by_frame, edges, incoming = graph
    for node in nodes:
        node["base_component"] = node["score"]
        node["compactness_component"] = 0.0
        node["support_component"] = 0.0
        node["background_evidence_term"] = node["new_score"] - node["score"]
        node["background_p_beta11"] = 0.5 * (1.0 - node["background_evidence_term"])
    return nodes, by_frame, edges, incoming


class DopplerPoison:
    def __float__(self):
        raise AssertionError("Doppler is forbidden in this audit")

    def __array__(self, *args, **kwargs):
        raise AssertionError("Doppler is forbidden in this audit")

    def __deepcopy__(self, memo):
        return self


class E18MechanismAuditTests(unittest.TestCase):
    def assert_matches_enumeration(self, graph, arm, **kwargs):
        nodes, _, edges, incoming = graph
        actual = audit.solve_arm(nodes, edges, incoming, arm, **kwargs)
        expected = enumerate_qualified_paths(nodes, edges, arm, **kwargs)
        self.assertEqual(set(actual["qualifying"]), set(expected))
        for eid, (score, _) in expected.items():
            self.assertAlmostEqual(actual["qualified_scores"][eid], score, places=12)
        if expected:
            self.assertAlmostEqual(actual["best_score"], max(v[0] for v in expected.values()), places=12)
            path = actual["best_path"]
            self.assertGreaterEqual(len(path), 3)
            self.assertGreaterEqual(nodes[path[-1]]["time"] - nodes[path[0]]["time"], 2.0)
        else:
            self.assertIsNone(actual["best_score"])
            self.assertEqual(actual["best_path"], tuple())
        return actual

    def test_four_arm_contract(self):
        self.assertEqual(tuple(audit.ARMS), ARMS)

    def test_all_four_arms_match_independent_exhaustive_real_dt(self):
        positions = [[[100.0 + fi, 0.0, 0.0], [101.0, 1.0 + 0.4 * fi, 0.0]] for fi in range(5)]
        old = [[0.2, -0.3], [-2.0, 0.4], [0.1, -0.8], [0.4, 0.1], [-0.1, 0.5]]
        new = [[-0.4, 0.2], [-0.5, -0.6], [0.3, 0.1], [-0.3, 0.5], [0.2, -0.2]]
        graph = with_parts(synthetic_graph([0.0, 0.9, 1.95, 3.05, 4.15], positions, old, new))
        for arm in ARMS:
            with self.subTest(arm=arm):
                self.assert_matches_enumeration(graph, arm)

    def test_score_term_routing_and_fixed_path_decomposition(self):
        graph = with_parts(synthetic_graph([0.0, 1.2, 2.9],
            [[[100.0, 0.0, 0.0]], [[101.2, 0.0, 0.0]], [[106.3, 0.0, 0.0]]],
            [[0.8], [0.8], [0.8]], [[1.5], [1.5], [1.5]]))
        nodes, _, edges, incoming = graph
        # Identify actual consecutive edges rather than assuming edge order.
        lookup = {(e["u"], e["v"]): e for e in edges}
        speed, direction = independent_motion(lookup[(0, 1)], lookup[(1, 2)])
        for arm in ARMS:
            with self.subTest(arm=arm):
                background, motion = enabled(arm)
                expected = 2.4 + (2.1 if background else 0.0) - (speed + direction if motion else 0.0)
                result = audit.solve_arm(nodes, edges, incoming, arm)
                self.assertEqual(result["best_path"], (0, 1, 2))
                self.assertAlmostEqual(result["best_score"], expected, places=12)
                parts = audit.path_parts_arm((0, 1, 2), nodes, edges, arm)
                self.assertAlmostEqual(parts["cumulative_score"], expected, places=12)
                self.assertAlmostEqual(parts["duration_s"], 2.9, places=12)

    def test_negative_short_history_cannot_hide_qualified_path(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0], [[[100.0, 0.0, 0.0]]] * 3,
            [[-5.0], [1.0], [1.0]], [[-6.0], [0.5], [0.5]]))
        for arm in ARMS:
            with self.subTest(arm=arm):
                result = self.assert_matches_enumeration(graph, arm)
                self.assertEqual(result["best_path"], (0, 1, 2))
                self.assertEqual(result["best_score"], -5.0 if enabled(arm)[0] else -3.0)

    def test_duration_qualification_with_nonunit_dt(self):
        graph = with_parts(synthetic_graph([0.0, 0.4, 0.8, 1.2, 1.6, 2.0],
            [[[100.0, 0.0, 0.0]]] * 6, [[-0.2]] * 6, [[-0.4]] * 6))
        for arm in ARMS:
            with self.subTest(arm=arm):
                result = self.assert_matches_enumeration(graph, arm)
                self.assertEqual(graph[0][result["best_path"][0]]["frame"], 0)
                self.assertEqual(graph[0][result["best_path"][-1]]["frame"], 5)

    def test_nodes_edges_and_incoming_not_mutated(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0, 3.0],
            [[[100.0 + fi, 0.0, 0.0]] for fi in range(4)], [[0.8]] * 4, [[1.4]] * 4))
        before = copy.deepcopy(graph)
        for arm in ARMS:
            audit.solve_arm(graph[0], graph[2], graph[3], arm)
            audit.path_parts_arm((0, 1, 2, 3), graph[0], graph[2], arm)
        self.assertEqual(graph[1], before[1])
        self.assertEqual(dict(graph[3]), dict(before[3]))
        for original, copied in zip(graph[0] + graph[2], before[0] + before[2]):
            self.assertEqual(set(original), set(copied))
            for key in original:
                if isinstance(original[key], np.ndarray):
                    np.testing.assert_array_equal(original[key], copied[key])
                else:
                    self.assertEqual(original[key], copied[key])

    def test_frozen_constants_unchanged(self):
        expected = {"MAX_FRAME_GAP": 3, "MAX_DT_S": 3.5, "MAX_SPEED_MPS": 25.0,
                    "MAX_ACCELERATION_MPS2": 15.0, "PATH_MIN_NODES": 3,
                    "PATH_MIN_DURATION_S": 2.0, "BACKGROUND_WEIGHT": 1.0}
        for name, value in expected.items():
            self.assertEqual(getattr(frozen_core, name), value)
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0], [[[100.0, 0.0, 0.0]]] * 3, [[1.0]] * 3))
        for arm in ARMS:
            audit.solve_arm(graph[0], graph[2], graph[3], arm)
        for name, value in expected.items():
            self.assertEqual(getattr(frozen_core, name), value)

    def test_doppler_poison_fields_not_accessed(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0], [[[100.0, 0.0, 0.0]]] * 3, [[1.0]] * 3))
        for node in graph[0]:
            node.update({"radial_velocity": DopplerPoison(), "doppler": DopplerPoison(), "velocity": DopplerPoison()})
        for edge in graph[2]:
            edge["doppler"] = DopplerPoison()
        for arm in ARMS:
            with self.subTest(arm=arm):
                result = audit.solve_arm(graph[0], graph[2], graph[3], arm)
                self.assertEqual(result["best_path"], (0, 1, 2))
                parts = audit.path_parts_arm(result["best_path"], graph[0], graph[2], arm)
                self.assertTrue(math.isfinite(parts["cumulative_score"]))

    def test_stationary_and_constant_velocity_motion_penalties_zero(self):
        for velocity in (np.zeros(3), np.array([2.0, -1.0, 0.5])):
            times = [0.0, 0.7, 1.8, 3.0]
            positions = [[np.array([100.0, 0.0, 0.0]) + stamp * velocity] for stamp in times]
            graph = with_parts(synthetic_graph(times, positions, [[1.0]] * 4, [[1.3]] * 4))
            original = audit.solve_arm(graph[0], graph[2], graph[3], "original")
            motion = audit.solve_arm(graph[0], graph[2], graph[3], "motion_only")
            self.assertAlmostEqual(original["best_score"], motion["best_score"], places=12)
            lookup = {(e["u"], e["v"]): e for e in graph[2]}
            path = motion["best_path"]
            ee = [lookup[(u, v)] for u, v in zip(path[:-1], path[1:])]
            for prev, edge in zip(ee[:-1], ee[1:]):
                speed, direction = independent_motion(prev, edge)
                self.assertAlmostEqual(speed, 0.0, places=12)
                self.assertAlmostEqual(direction, 0.0, places=12)

    def test_acceleration_gate_remains_for_all_arms(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0],
            [[[100.0, 0.0, 0.0]], [[100.0, 0.0, 0.0]], [[120.0, 0.0, 0.0]]], [[1.0]] * 3))
        for arm in ARMS:
            with self.subTest(arm=arm):
                self.assert_matches_enumeration(graph, arm)

    def test_frame_span_keyword_preserves_frozen_solver_contract(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0, 3.0, 4.0],
            [[[100.0, 0.0, 0.0]]] * 5, [[1.0]] * 5, [[1.4]] * 5))
        for arm in ARMS:
            with self.subTest(arm=arm):
                result = self.assert_matches_enumeration(graph, arm, frame_min=1, frame_max=3)
                self.assertEqual(result["best_path"], (1, 2, 3))

    def test_unsupported_arm_rejected(self):
        graph = with_parts(synthetic_graph([0.0, 1.0, 2.0], [[[100.0, 0.0, 0.0]]] * 3, [[1.0]] * 3))
        with self.assertRaises((ValueError, KeyError)):
            audit.solve_arm(graph[0], graph[2], graph[3], "new_weight_or_doppler")


if __name__ == "__main__":
    unittest.main()
