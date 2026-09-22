import importlib.util
from pathlib import Path

import numpy as np


PATH = Path(__file__).resolve().parents[1] / "scripts" / "sparse_uav_mvp.py"
SPEC = importlib.util.spec_from_file_location("sparse_uav_mvp", PATH)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def candidate(x, velocity=0.0):
    return {
        "center": np.asarray([x, 0.0, 0.0]), "raw_points": 3, "voxels": 1,
        "extent": 0.0, "intensity": 20.0, "radial_velocity": velocity, "eps": 1.0,
    }


def test_weighted_median():
    values = np.asarray([[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]])
    got = M._weighted_median(values, np.asarray([1.0, 4.0]))
    np.testing.assert_allclose(got, values[1])


def test_real_dt_and_m_of_k_linking():
    frames = [[candidate(10.0)], [candidate(15.0)], [candidate(20.0)]]
    out, _ = M.temporal_validate(frames, np.asarray([0.0, 0.5, 1.0]), True, "none")
    assert [len(x) for x in out] == [0, 1, 1]
    assert out[1][0]["track_id"] == out[2][0]["track_id"]


def test_observability_aware_gate_downweights_cross_range_residual():
    # Motion is mostly cross-range at x=100 m, so eta is small.  A 10 m/s
    # Doppler residual fails the fixed gate but is retained by eta weighting.
    base = [[candidate(100.0)], [candidate(100.0)], [candidate(100.0, velocity=-10.0)]]
    for i, y in enumerate((0.0, 5.0, 10.0)):
        base[i][0]["center"] = np.asarray([100.0, y, 0.0])
    fixed, _ = M.temporal_validate([[dict(x) for x in f] for f in base], np.asarray([0.0, 1.0, 2.0]), False, "fixed")
    aware, _ = M.temporal_validate([[dict(x) for x in f] for f in base], np.asarray([0.0, 1.0, 2.0]), False, "aware")
    assert fixed[2][0]["track_id"] != fixed[1][0]["track_id"]
    assert aware[2][0]["track_id"] == aware[1][0]["track_id"]
