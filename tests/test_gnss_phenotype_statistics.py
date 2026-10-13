"""Synthetic descriptive-statistics tests; not detector/tracker experiments."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location(
    "phenotype_statistics", Path(__file__).parents[1] / "scripts/gnss_phenotype_statistics.py")
stats = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stats)


class NumericTests(unittest.TestCase):
    def test_nonfinite_zero_and_population_std(self):
        result = stats.numeric_descriptors([0, 2, np.nan, np.inf, -np.inf], "v")
        self.assertEqual(result["v_finite_count"], 2)
        self.assertEqual(result["v_nonfinite_count"], 3)
        self.assertEqual(result["v_nan_count"], 1)
        self.assertEqual(result["v_zero_count"], 1)
        self.assertEqual(result["v_median"], 1)
        self.assertEqual(result["v_std"], 1)

    def test_empty_is_not_zero(self):
        result = stats.numeric_descriptors([], "x")
        self.assertEqual(result["x_count"], 0)
        self.assertTrue(np.isnan(result["x_median"]))


class SpatialTests(unittest.TestCase):
    def test_empty_and_singleton(self):
        empty = stats.spatial_descriptors([], [100, 0, 0])
        self.assertEqual(empty["raw_points"], 0)
        self.assertEqual(empty["occupied_voxels_0.04m"], 0)
        self.assertTrue(np.isnan(empty["diameter_m"]))
        one = stats.spatial_descriptors([[100, 0, 0]], [100, 0, 0])
        self.assertEqual(one["diameter_m"], 0)
        self.assertEqual(one["raw_cov_eigenvalue_1_m2"], 0)
        self.assertEqual(one["geometry_status"], "DEGENERATE_NOT_BODY_SIZE")
        self.assertTrue(np.isnan(one["nn_m_median"]))
        self.assertTrue(np.isnan(one["unique_nn_m_median"]))
        self.assertTrue(np.isnan(one["mst_connectivity_edge_max_m"]))

    def test_duplicate_semantics(self):
        result = stats.spatial_descriptors([[100, 0, 0], [100, 0, 0], [100, .2, 0]], [100, 0, 0])
        self.assertEqual(result["raw_points"], 3)
        self.assertEqual(result["unique_xyz_points"], 2)
        self.assertEqual(result["coincident_return_count"], 1)
        self.assertEqual(result["nn_m_median"], 0)
        self.assertAlmostEqual(result["unique_nn_m_median"], .2)
        self.assertAlmostEqual(result["mst_total_length_m"], .2)
        self.assertNotEqual(result["raw_cov_eigenvalue_1_m2"], result["unique_cov_eigenvalue_1_m2"])

    def test_los_shape_centering_not_oracle_error(self):
        result = stats.spatial_descriptors([[100, -1, 0], [100, 1, 0]], [90, 0, 0])
        self.assertAlmostEqual(result["mean_oracle_offset_norm_m"], 10)
        self.assertEqual(result["radial_rms_m"], 0)
        self.assertEqual(result["tangential_rms_m"], 1)
        self.assertEqual(result["tangential_diameter_m"], 2)
        self.assertEqual(result["raw_cov_radial_variance_m2"], 0)
        self.assertEqual(result["raw_cov_tangential_variance_m2"], 1)

    def test_rigid_invariance_of_diameter_and_cov_eigenvalues(self):
        points = np.array([[100, 0, 0], [101, .2, 0], [102, 0, .1]])
        angle = np.pi / 3
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        a = stats.spatial_descriptors(points, [100, 0, 0])
        b = stats.spatial_descriptors(points @ rotation.T + [0, 3, 1], np.array([100, 0, 0]) @ rotation.T + [0, 3, 1])
        self.assertAlmostEqual(a["diameter_m"], b["diameter_m"])
        self.assertAlmostEqual(a["raw_cov_eigenvalue_1_m2"], b["raw_cov_eigenvalue_1_m2"])
        self.assertNotAlmostEqual(a["span_x_m"], b["span_x_m"])

    def test_fine_coarse_representation_count(self):
        points = np.array([[100, y, .1] for y in np.arange(.1, .9, .02)])
        result = stats.spatial_descriptors(points, [100, .5, .1])
        self.assertGreater(result["occupied_voxels_0.04m"], 1)
        self.assertEqual(result["occupied_voxels_1m"], 1)
        self.assertGreater(result["diameter_m"], 0)
        self.assertEqual(result["voxel_1m_mean_diameter_m"], 0)
        self.assertEqual(result["voxel_1m_mean_cov_eigenvalue_1_m2"], 0)
        self.assertEqual(result["voxel_1m_mean_raw_child_support"], len(points))
        self.assertGreater(result["voxel_0.04m_mean_diameter_m"], 0)

    def test_cell_means_use_same_children_not_unique_weighting(self):
        points = [[100.1, .1, .1], [100.1, .1, .1], [100.4, .1, .1], [101.2, .1, .1]]
        result = stats.spatial_descriptors(points, [100, .1, .1])
        # First cell mean = (100.1+100.1+100.4)/3 = 100.2.
        self.assertEqual(result["voxel_1m_mean_count"], 2)
        self.assertAlmostEqual(result["voxel_1m_mean_diameter_m"], 1)
        self.assertEqual(result["voxel_1m_mean_raw_child_support"], 4)
        self.assertEqual(result["voxel_1m_children_max"], 3)

    def test_pairwise_budget_does_not_hide_nn(self):
        result = stats.spatial_descriptors([[100, 0, 0], [100, 1, 0], [100, 2, 0]], [100, 0, 0], pairwise_limit=2)
        self.assertTrue(result["pairwise_budget_exceeded"])
        self.assertTrue(np.isnan(result["diameter_m"]))
        self.assertTrue(np.isnan(result["mst_total_length_m"]))
        self.assertEqual(result["unique_nn_m_median"], 1)

    def test_tiny_mst_edges_survive_sparse_conversion(self):
        result = stats.spatial_descriptors([[100, 0, 0], [100, 1e-10, 0], [100, 2e-10, 0]], [100, 0, 0])
        self.assertEqual(result["mst_edge_count"], 2)
        self.assertAlmostEqual(result["mst_total_length_m"] / 1e-10, 2)

    def test_invalid_coordinates_are_not_silently_filtered(self):
        with self.assertRaises(ValueError):
            stats.spatial_descriptors([[100, np.nan, 0]], [100, 0, 0])

    def test_zero_los_unknown_not_fabricated(self):
        result = stats.spatial_descriptors([[1, 0, 0], [2, 0, 0]], [0, 0, 0])
        self.assertTrue(np.isnan(result["radial_span_m"]))
        self.assertEqual(result["los_status"], "UNKNOWN_ZERO_CENTER")


def region(frame, role, value, **kwargs):
    return {"sequence_id": "S", "frame_index": frame, "time_s": float(frame),
            "alignment_scenario": "nominal", "region_role": role, "region_id": role,
            "gnss_segment_id": 0, "gnss_supported": True, "raw_points": value,
            "feature": value, **kwargs}


class MatchedTests(unittest.TestCase):
    def test_equal_frame_weights_despite_unequal_control_counts(self):
        rows = [region(0, "UAV_CONDITIONED", 2)]
        rows += [region(0, "LOCAL_CLUTTER_PROXY", 1, region_id=f"c{i}") for i in range(9)]
        rows += [region(1, "UAV_CONDITIONED", 0), region(1, "LOCAL_CLUTTER_PROXY", 1)]
        out = stats.matched_effects(rows, ["feature"], ["sequence_id"])[0]
        self.assertEqual(out["matched_auc_higher_uav"], .5)
        self.assertEqual(out["cliffs_delta"], 0)
        self.assertEqual(out["valid_frames"], 2)
        self.assertEqual(out["valid_pairs"], 10)

    def test_fixed_orientation_ties_and_missing_attrition(self):
        rows = [region(0, "UAV_CONDITIONED", 0), region(0, "LOCAL_CLUTTER_PROXY", 1),
                region(1, "UAV_CONDITIONED", 1), region(1, "LOCAL_CLUTTER_PROXY", 1),
                region(2, "UAV_CONDITIONED", np.nan), region(2, "LOCAL_CLUTTER_PROXY", 5),
                region(3, "UAV_CONDITIONED", 1), region(3, "LOCAL_CLUTTER_PROXY", np.nan)]
        out = stats.matched_effects(rows, ["feature"], ["sequence_id"])[0]
        self.assertEqual(out["matched_auc_higher_uav"], .25)
        self.assertEqual(out["cliffs_delta"], -.5)
        self.assertEqual(out["excluded_frames"], 2)
        self.assertEqual(out["nonfinite_target_frames"], 1)
        self.assertEqual(out["frames_without_finite_controls"], 1)
        self.assertEqual(out["excluded_pairs"], 2)

    def test_scenarios_do_not_cross_match(self):
        rows = [region(0, "UAV_CONDITIONED", 1, alignment_scenario="a"),
                region(0, "LOCAL_CLUTTER_PROXY", 0, alignment_scenario="b")]
        out = stats.matched_effects(rows, ["feature"], [])[0]
        self.assertEqual(out["valid_frames"], 0)
        self.assertTrue(np.isnan(out["matched_auc_higher_uav"]))

    def test_duplicate_targets_require_distinct_recipe(self):
        with self.assertRaises(ValueError):
            stats.matched_effects([region(0, "UAV_CONDITIONED", 1), region(0, "UAV_CONDITIONED", 2)], ["feature"], [])

    def test_zero_iqr_not_infinite_effect(self):
        rows = [region(0, "UAV_CONDITIONED", 2), region(0, "LOCAL_CLUTTER_PROXY", 1)]
        out = stats.matched_effects(rows, ["feature"], [])[0]
        self.assertTrue(np.isnan(out["iqr_scale_effect"]))


class RunTests(unittest.TestCase):
    def test_internal_zero_run_real_time_brackets(self):
        rows = [region(0, "UAV_CONDITIONED", 1, time_s=0),
                region(1, "UAV_CONDITIONED", 0, time_s=.7),
                region(2, "UAV_CONDITIONED", 0, time_s=1.6),
                region(3, "UAV_CONDITIONED", 2, time_s=2.5)]
        run = stats.zero_run_records(rows)[0]
        self.assertEqual(run["record_kind"], "ZERO_REGION_RUN")
        self.assertEqual(run["frame_count"], 2)
        self.assertAlmostEqual(run["observed_span_s"], .9)
        self.assertEqual(run["duration_upper_bound_s"], 2.5)
        self.assertFalse(run["left_censored"])
        self.assertFalse(run["right_censored"])

    def test_boundary_runs_are_censored_single_sample_span_zero(self):
        run = stats.zero_run_records([region(0, "UAV_CONDITIONED", 0)])[0]
        self.assertTrue(run["left_censored"])
        self.assertTrue(run["right_censored"])
        self.assertEqual(run["observed_span_s"], 0)
        self.assertTrue(np.isnan(run["duration_upper_bound_s"]))

    def test_unsupported_and_closure_absent_not_sensor_zero(self):
        rows = [region(0, "UAV_CONDITIONED", 0),
                region(1, "UAV_CONDITIONED", 0, gnss_supported=False),
                region(2, "UAV_CONDITIONED", 0, closure_supported=False),
                region(3, "UAV_CONDITIONED", 0)]
        out = stats.zero_run_records(rows)
        gaps = [r for r in out if r["record_kind"] == "OBSERVATION_GAP"]
        runs = [r for r in out if r["record_kind"] == "ZERO_REGION_RUN"]
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]["frame_count"], 2)
        self.assertEqual(len(runs), 2)
        self.assertTrue(all(r["left_censored"] and r["right_censored"] for r in runs))

    def test_segment_change_breaks_run(self):
        rows = [region(0, "UAV_CONDITIONED", 0), region(1, "UAV_CONDITIONED", 0, gnss_segment_id=1)]
        runs = [r for r in stats.zero_run_records(rows) if r["record_kind"] == "ZERO_REGION_RUN"]
        self.assertEqual(len(runs), 2)
        self.assertTrue(runs[0]["right_censored"])
        self.assertTrue(runs[1]["left_censored"])

    def test_frame_outage_and_missing_index_are_gaps(self):
        rows = [region(0, "UAV_CONDITIONED", 0, time_s=0, bag_median_dt_s=1),
                region(1, "UAV_CONDITIONED", 0, time_s=1, bag_median_dt_s=1),
                region(2, "UAV_CONDITIONED", 0, time_s=5, bag_median_dt_s=1),
                region(4, "UAV_CONDITIONED", 0, time_s=6, bag_median_dt_s=1)]
        out = stats.zero_run_records(rows)
        gaps = [r for r in out if r["record_kind"] == "OBSERVATION_GAP"]
        self.assertEqual(len(gaps), 2)
        self.assertTrue(all(r["frame_count"] is None for r in gaps))
        self.assertEqual({r["start_reason"] for r in gaps}, {"FRAME_TIME_OUTAGE", "MISSING_OR_NONCONTIGUOUS_FRAME_INDEX"})
        self.assertEqual(len([r for r in out if r["record_kind"] == "ZERO_REGION_RUN"]), 3)

    def test_controls_cannot_duplicate_bag_cadence(self):
        rows = []
        for frame, time in enumerate([0, .4, 1, 1.8]):
            rows.append(region(frame, "UAV_CONDITIONED", 0, time_s=time))
            rows.append(region(frame, "LOCAL_CLUTTER_PROXY", 1, time_s=time))
        runs = stats.zero_run_records(rows)
        self.assertEqual(len(runs), 1)
        self.assertAlmostEqual(runs[0]["bag_median_dt_s"], .6)


if __name__ == "__main__":
    unittest.main()
