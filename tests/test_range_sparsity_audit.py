"""Synthetic tests for diagnostic semantics, NOT detector/tracker tests."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location("audit", Path(__file__).parents[1] / "scripts/range_sparsity_audit.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class DiagnosticTests(unittest.TestCase):
    def test_singleton_unknown_spacing(self):
        r = audit.spacing(np.array([[100., 0., 0.]]))
        self.assertEqual(r["raw_points"], 1)
        self.assertEqual(r["diameter_m"], 0)
        self.assertTrue(np.isnan(r["nn_m_median"]))
        self.assertTrue(np.isnan(r["pairwise_m_median"]))

    def test_spacing_counts_and_duplicate_semantics(self):
        p = np.array([[100., 0., 0.], [100., .1, 0.], [100., .2, 0.]])
        r = audit.spacing(p)
        self.assertAlmostEqual(r["nn_m_median"], .1)
        self.assertAlmostEqual(r["diameter_m"], .2)
        self.assertAlmostEqual(r["mst_connectivity_edge_max_m"], .1)
        self.assertEqual(r["occupied_voxels_1m"], 1)
        r = audit.spacing(np.array([[1., 1., 1.], [1., 1., 1.]]))
        self.assertEqual(r["coincident_return_count"], 1)
        self.assertEqual(r["nn_m_median"], 0)
        self.assertTrue(np.isnan(r["unique_nn_m_median"]))

    def test_named_fields_header_points_and_padding(self):
        dtype = np.dtype([(k, "<f4") for k in ["x", "y", "z", "velocity", "intensity"]])
        arr = np.array([(100., 2., 3., -4., 17.)], dtype=dtype)
        header = b"FIELDS x y z velocity intensity\nSIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nPOINTS 1\nDATA binary\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample.pcd"
            path.write_bytes(header + arr.tobytes() + b"\0" * 7)
            got, meta = audit.pcd_binary(path)
        self.assertEqual(got["velocity"][0], -4)
        self.assertEqual(got["intensity"][0], 17)
        self.assertEqual(len(got), 1)
        self.assertEqual(meta["trailing_bytes"], 7)
        self.assertTrue(meta["trailing_all_zero"])

    def test_empty_shell_and_no_artificial_spacing(self):
        arr = np.array([(100., 0., 0., 17., 2.), (0., 0., 0., 0., 0.)],
                       dtype=[(k, "f4") for k in ["x", "y", "z", "intensity", "velocity"]])
        label = {f"center_{a}_m": v for a, v in zip("xyz", [100., 0., 0.])}
        label.update(sequence_id="synthetic", frame_index=0, time_s=0)
        frames, clutter, points, _ = audit.measure(arr, label, {})
        self.assertEqual(frames[0]["valid_nonzero_raw_scene_points"], 1)
        self.assertEqual(clutter[0]["clutter_points"], 0)
        self.assertEqual(clutter[0]["clutter_density_points_per_m3"], 0)
        self.assertTrue(np.isnan(clutter[0]["clutter_nn_m_median"]))
        self.assertEqual(len(points), 1)


if __name__ == "__main__":
    unittest.main()
