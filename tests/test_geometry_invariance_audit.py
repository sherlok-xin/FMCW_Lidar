"""Descriptor tests only, not new detector or threshold tests."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
spec = importlib.util.spec_from_file_location("geometry", Path(__file__).parents[1] / "scripts/geometry_invariance_audit.py")
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)


class GeometryTests(unittest.TestCase):
    def test_singleton_not_zero_spacing(self):
        d = g.describe(np.array([[100., 0., 0.]]))
        self.assertEqual(d["diameter_m"], 0)
        self.assertEqual(d["range_thickness_m"], 0)
        self.assertTrue(np.isnan(d["nn_m_median"]))
        self.assertTrue(np.isnan(d["mst_connectivity_edge_max_m"]))

    def test_compact_cloud_has_representation_dependent_count(self):
        xyz = np.array([[100., y, .1] for y in np.arange(.1, .9, .02)])
        fine, nf = g.cell_means(xyz, .04); coarse, nc = g.cell_means(xyz, 1)
        self.assertGreater(len(fine), 1)
        self.assertEqual(len(coarse), 1)
        self.assertEqual(nf.sum(), nc.sum())
        self.assertGreater(g.describe(xyz)["diameter_m"], 0)
        self.assertEqual(g.describe(coarse)["diameter_m"], 0)

    def test_diameter_rigid_invariance_not_axis_span(self):
        xyz = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
        angle = np.pi/4
        r = np.array([[np.cos(angle), -np.sin(angle), 0.], [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        a, b = g.describe(xyz), g.describe(xyz @ r.T + [100., 3., 2.])
        self.assertAlmostEqual(a["diameter_m"], b["diameter_m"])
        self.assertNotAlmostEqual(a["max_axis_span_m"], b["max_axis_span_m"])

    def test_summary_missing_not_zero(self):
        a = g.summarize([np.nan, 1., 3.]); b = g.summarize([])
        self.assertEqual(a["n_defined"], 2); self.assertEqual(a["median"], 2)
        self.assertEqual(b["n_defined"], 0); self.assertTrue(np.isnan(b["median"]))

    def test_coincident_returns_not_unique_spacing(self):
        xyz = np.array([[100., 0., 0.], [100., 0., 0.], [100., .2, 0.]])
        d = g.describe(xyz)
        self.assertEqual(d["nn_m_median"], 0)
        self.assertAlmostEqual(d["unique_nn_m_median"], .2)

    def test_nearest_rejected_component_does_not_define_frame_miss(self):
        rows = [{"center_x_m": .1, "center_y_m": 0, "center_z_m": 0, "accepted": "False"},
                {"center_x_m": 1., "center_y_m": 0, "center_z_m": 0, "accepted": "True"}]
        nearest, distance, accepted_count = g.lookup_saved_components(rows, np.zeros(3))
        self.assertEqual(nearest["accepted"], "False")
        self.assertEqual(distance, .1)
        self.assertEqual(accepted_count, 1)


if __name__ == "__main__":
    unittest.main()
