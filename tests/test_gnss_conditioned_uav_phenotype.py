import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import gnss_conditioned_uav_phenotype as study


class RegionTests(unittest.TestCase):
    def geometry(self):
        return {"center_x_m": 100., "center_y_m": 0., "center_z_m": 20.,
                "anchor_center_x_m": 102., "anchor_center_y_m": 1., "anchor_center_z_m": 20.}

    def test_matched_geometry_and_exclusion(self):
        g = self.geometry()
        scenarios = [{"center_x_m": 100., "center_y_m": 0., "center_z_m": 20.},
                     {"center_x_m": 101., "center_y_m": 1., "center_z_m": 20.}]
        regions, rejected = study.make_regions(g, scenarios)
        self.assertEqual(len([r for r in regions if r[-1] == "UAV_CONDITIONED"]), 6)
        self.assertTrue(rejected)  # 10-degree 10m balls overlap at this range.
        originals = {r[0]: r for r in regions if r[-1] == "UAV_CONDITIONED"}
        tube = np.array([[100, 0, 20], [101, 1, 20], [102, 1, 20]])
        for name, centers, radius, ref, rid, angle, role in regions:
            if role == "LOCAL_CLUTTER_PROXY":
                original = originals[name]
                np.testing.assert_allclose(np.linalg.norm(centers, axis=1), np.linalg.norm(original[1], axis=1))
                np.testing.assert_allclose(centers[:, 2], original[1][:, 2])
                self.assertTrue(study.disjoint(centers, radius, tube, 3.))
                self.assertTrue(study.disjoint(centers, radius, original[1], radius))

    def test_empty_outside_support_does_not_generate_regions(self):
        self.assertEqual(study.make_regions({}, []), ([], []))

    def test_saved_anchor_without_gnss_kept_as_distinct_recipe(self):
        g = {k: v for k, v in self.geometry().items() if k.startswith("anchor_")}
        regions, _ = study.make_regions(g, [])
        self.assertEqual(sorted(set(r[0] for r in regions)), ["ANCHOR_1m", "ANCHOR_3m"])

    def test_union_deduplicates_raw_indices(self):
        points = np.array([[0, 0, 0], [1, 0, 0], [10, 0, 0]])
        idx = study.region_indices(cKDTree(points), np.array([[0, 0, 0], [1, 0, 0]]), 1.1)
        np.testing.assert_array_equal(idx, [0, 1])

    def test_sign_convention_retains_both_residuals(self):
        arr = np.array([(-2., -2.)], dtype=[("velocity", "f4"), ("velocity0", "f4")])
        r = study.point_features(arr, np.array([0]), np.array([[100., 0, 0]]), np.array([100., 0, 0]),
                                 {"nominal_range_rate_mps": 2.})
        self.assertEqual(r["velocity_signed_range_rate_residual_mps"], 0.)
        self.assertEqual(r["velocity_opposite_sign_residual_mps"], -4.)

    def test_out_of_order_schema_lossless_npz_export(self):
        dtype = np.dtype({"names": ["x", "ring", "time"], "formats": ["<f4", "<u4", "<f8"],
                          "offsets": [12, 0, 4], "itemsize": 16})
        raw = np.arange(48, dtype=np.uint8).reshape(3, 16)
        arr = raw.reshape(-1).view(dtype)
        arr["ring"] = [2**32 - 1, 5, 6]
        arr["time"] = [1e-12, 1e-12 + 1e-15, 2e-12]
        payload = study.point_record_export(arr, np.array([0, 2]), ["test"], np.array([0, 2]),
                                            np.array([0, 1]), {"field_schema": []})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "points.npz"
            np.savez_compressed(path, **payload)
            with np.load(path, allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved["raw_point_bytes"], raw[[0, 2]])
                np.testing.assert_array_equal(saved["field__ring"], arr["ring"][[0, 2]])
                np.testing.assert_array_equal(saved["field__time"], arr["time"][[0, 2]])
                self.assertEqual(saved["field__time"].dtype.itemsize, 8)


if __name__ == "__main__":
    unittest.main()
