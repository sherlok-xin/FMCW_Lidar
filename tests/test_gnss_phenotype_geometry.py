"""Synthetic checks for descriptive geometry; no detector/tracker experiment."""

import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import gnss_phenotype_geometry as geometry


class PiecewiseGeometryTests(unittest.TestCase):
    def test_radial_chord_and_analytic_range_rate(self):
        state = geometry.piecewise_state(np.array([0., 2.]), np.array([[10., 0., 0.], [14., 0., 0.]]),
                                         1., np.eye(3), np.zeros(3))
        self.assertEqual(state["range_m"], 12.)
        self.assertEqual(state["speed_mps"], 2.)
        self.assertEqual(state["range_rate_mps"], 2.)
        self.assertAlmostEqual(state["eta"], 1., places=8)
        self.assertEqual(state["regime"], "RADIAL_DOMINANT")

    def test_tangential_chord_range_rate_not_total_speed(self):
        state = geometry.piecewise_state(np.array([0., 2.]), np.array([[10., -2., 0.], [10., 2., 0.]]),
                                         1., np.eye(3), np.zeros(3))
        self.assertEqual(state["speed_mps"], 2.)
        self.assertEqual(state["range_rate_mps"], 0.)
        self.assertEqual(state["eta"], 0.)
        self.assertEqual(state["regime"], "CROSS_DOMINANT")

    def test_no_extrapolation_and_valid_endpoints(self):
        args = (np.array([1., 3.]), np.array([[10., 0., 0.], [14., 0., 0.]]))
        self.assertIsNone(geometry.piecewise_state(*args, 1.-1e-8, np.eye(3), np.zeros(3)))
        self.assertIsNone(geometry.piecewise_state(*args, 3.+1e-8, np.eye(3), np.zeros(3)))
        for time in (1., 3.):
            self.assertIsNotNone(geometry.piecewise_state(*args, time, np.eye(3), np.zeros(3)))

    def test_rotation_translation_and_real_segment_dt(self):
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        state = geometry.piecewise_state(np.array([0., 4.]), np.array([[10., 0., 1.], [18., 0., 1.]]),
                                         2., rotation, np.array([0., 3., 0.]))
        np.testing.assert_allclose(state["position"], [0., 17., 1.])
        np.testing.assert_allclose(state["velocity"], [0., 2., 0.])
        self.assertEqual(state["segment_dt_s"], 4.)
        self.assertAlmostEqual(state["range_rate_mps"], 34./math.sqrt(290.))

    def test_knot_selects_right_chord_and_flags_derivative(self):
        state = geometry.piecewise_state(np.array([0., 1., 3.]),
                                         np.array([[10., 0., 0.], [12., 0., 0.], [8., 0., 0.]]),
                                         1., np.eye(3), np.zeros(3))
        self.assertEqual(state["segment_index"], 1)
        self.assertEqual(state["velocity"][0], -2.)
        self.assertTrue(state["at_interior_gnss_knot"])
        self.assertEqual(state["range_rate_derivative_status"], "RIGHT_CHORD_AT_NONDIFFERENTIABLE_KNOT")

    def test_stationary_endpoint_chord_does_not_certify_hover(self):
        state = geometry.piecewise_state(np.array([0., 2.]), np.array([[10., 0., 0.], [10., 0., 0.]]),
                                         1., np.eye(3), np.zeros(3))
        self.assertEqual(state["regime"], "UNDEFINED_ZERO_CHORD_SPEED")
        self.assertEqual(geometry.SETTINGS["hover_interpretation"], "UNKNOWN; GNSS endpoint chords do not certify hover")

    def test_nonmonotonic_times_rejected(self):
        with self.assertRaises(ValueError):
            geometry.piecewise_state(np.array([1., 1.]), np.zeros((2, 3)), 1., np.eye(3), np.zeros(3))


class FrozenArtifactIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = geometry.load_geometry(ROOT)

    def test_original_workbook_and_e20_crosscheck(self):
        self.assertEqual(len(self.context["sequences"]), 9)
        self.assertEqual(len(self.context["gnss_rows"]), 72)
        self.assertEqual(len(self.context["gnss_segments"]), 62)
        self.assertEqual(len(self.context["workbook_provenance_checks"]), 72)
        self.assertEqual(sum(len(self.context["sequences"][cid]["gnss_points"]) for cid in geometry.PASS_CROSS), 20)
        self.assertEqual(len(self.context["anchors"]), 80)

    def test_nominal_offset_is_included_even_when_profile_grid_omits_it(self):
        for cid in geometry.PASS_CROSS:
            sequence = self.context["sequences"][cid]
            self.assertIn(sequence["time_offset_s"], sequence["scenario_offsets_s"])
            nominal = [scenario for scenario in sequence["scenarios"] if scenario["is_nominal"]]
            self.assertEqual(len(nominal), 1)

    def test_no_silver_interpolation_and_geometry_support(self):
        cid = "100m_cross_10"
        anchor = self.context["anchors"][(cid, 18)]
        result, scenarios = geometry.annotate_frame(self.context, cid, float(anchor["bag_time_unix"]), 18)
        self.assertTrue(result["observed_closure_anchor_available"])
        self.assertTrue(result["gnss_nominal_supported"])
        self.assertFalse(result["independent_positional_GT"])
        self.assertEqual(result["nominal_range_m"], 100)
        self.assertEqual(result["nominal_speed_mps"], 10)
        self.assertEqual(result["nominal_predicted_range_m"], result["range_m"])
        self.assertEqual(result["nominal_chord_speed_mps"], result["speed_mps"])
        self.assertGreater(result["anchor_minus_nominal_center_distance_m"], 0)
        self.assertEqual(sum(row["is_nominal"] for row in scenarios), 1)
        self.assertTrue(all(self.context["sequences"][cid]["source_t"][0] <= row["query_gps_time_unix_s"] <= self.context["sequences"][cid]["source_t"][-1] for row in scenarios))
        missing, _ = geometry.annotate_frame(self.context, cid, float(anchor["bag_time_unix"])-1., 17)
        self.assertFalse(missing["observed_closure_anchor_available"])
        self.assertEqual(missing["observed_closure_anchor_status"], "NO_OBSERVED_ANCHOR_NOT_A_CERTIFIED_MISS")

    def test_outside_support_keeps_unknown_not_extrapolated_target(self):
        cid = "100m_cross_5"
        sequence = self.context["sequences"][cid]
        time = float(sequence["source_t"][0] - max(sequence["scenario_offsets_s"]) - 100.)
        result, scenarios = geometry.annotate_frame(self.context, cid, time, -1)
        self.assertFalse(result["gnss_nominal_supported"])
        self.assertNotIn("center_x_m", result)
        self.assertEqual(scenarios, [])
        self.assertEqual(result["clock_offset_sigma_status"], "UNKNOWN")


if __name__ == "__main__":
    unittest.main()
