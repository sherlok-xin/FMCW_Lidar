"""Tests for isolated diagnostic definitions; no research data mutations."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
spec = importlib.util.spec_from_file_location("attribution", Path(__file__).parents[1] / "scripts/dbscan_fragmentation_attribution.py")
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)


def values(xyz):
    p = np.asarray(xyz, dtype=float).reshape(-1, 3)
    return np.column_stack([p, np.ones(len(p)), np.full(len(p), 20.), np.zeros(len(p))])


class AttributionTests(unittest.TestCase):
    def test_minpts_equivalence_with_geometric_nmin(self):
        v = values([[100, 0, 0], [100, .2, 0], [100, 3, 0]])
        l1, c1 = d.cluster_frame(v, .6, 1)
        l2, c2 = d.cluster_frame(v, .6, 2)
        a = sorted(tuple(np.flatnonzero(l1 == x)) for x, c in c1.items() if c["accepted"])
        b = sorted(tuple(np.flatnonzero(l2 == x)) for x, c in c2.items() if c["accepted"])
        self.assertEqual(a, b)
        self.assertEqual(sum(c["too_small"] for c in c1.values()), 1)
        self.assertEqual(int(np.sum(l2 < 0)), 1)

    def test_induced_vs_full_scene_clutter_bridge(self):
        xyz = np.array([[100., -1., 0.], [100., 1., 0.]])
        self.assertEqual(len(set(d.induced_components(xyz, .6))), 2)
        v = values([[100., y, 0.] for y in (-1., -.5, 0., .5, 1.)])
        lab, _ = d.cluster_frame(v, .6, 1)
        self.assertEqual(len(set(lab)), 1)

    def test_large_merge_can_destroy_candidate(self):
        v = values([[100., y, 0.] for y in np.arange(0, 1, .02)])
        _, c = d.cluster_frame(v, .6, 2)
        self.assertEqual(len(c), 1)
        self.assertTrue(c[0]["too_many_voxels"])
        self.assertFalse(c[0]["accepted"])

    def test_same_input_coarsening_preserves_weighted_support(self):
        v = values([[100., .1, 0.], [100., .2, 0.], [100., 1.2, 0.]])
        v[:, 3] = [3, 1, 2]
        c = d.coarsen_fine(v)
        self.assertEqual(len(c), 2)
        self.assertEqual(c[:, 3].sum(), 6)
        self.assertAlmostEqual(c[0, 1], .125)

    def test_eps_is_scene_global_not_target_range(self):
        v = values([[100., 0., 0.], [300., 0., 0.]])
        self.assertEqual(d.eps_for(v, "fixed"), .6)
        self.assertAlmostEqual(d.eps_for(v, "adaptive"), .6 + .004 * 190.)

    def test_empty_and_singleton_connectivity(self):
        self.assertEqual(len(d.induced_components(np.zeros((0, 3)), .6)), 0)
        self.assertEqual(len(set(d.induced_components(np.array([[100., 0., 0.]]), .6))), 1)


if __name__ == "__main__":
    unittest.main()
