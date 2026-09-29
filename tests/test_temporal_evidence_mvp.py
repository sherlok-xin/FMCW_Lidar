import importlib.util
from pathlib import Path

import numpy as np


PATH = Path(__file__).resolve().parents[1] / "scripts" / "temporal_evidence_mvp.py"
SPEC = importlib.util.spec_from_file_location("temporal_evidence_mvp", PATH)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def candidate(x, y=0.0, raw_points=3, extent=0.0, rv=0.0):
    return {
        "center": np.asarray([x, y, 12.0]), "raw_points": raw_points,
        "voxels": 1, "extent": extent, "intensity": 20.0,
        "radial_velocity": rv,
    }


def test_level_transform_round_trip():
    points = np.asarray([[100.0, -20.0, 2.0], [300.0, 40.0, -9.0]])
    np.testing.assert_allclose(M.level_to_lidar(M.lidar_to_level(points)), points, atol=1e-10)


def test_legacy_initialization_is_blind_for_eight_frames():
    stage = M.LegacyStageFilter()
    points = np.asarray([[100.0, 0.0, 12.0]])
    for _ in range(8):
        pre, post = stage.process(points)
        assert not pre.any() and not post.any()


def test_adaptive_window_policy_is_simple_and_ordered():
    assert M.adaptive_window(candidate(300.0, raw_points=10), 2.0) == 6.0
    assert M.adaptive_window(candidate(100.0, raw_points=3), 2.0) == 6.0
    assert M.adaptive_window(candidate(100.0, raw_points=30), 2.0) == 3.0
    assert M.adaptive_window(candidate(100.0, raw_points=10), 9.0) == 3.0
    assert M.adaptive_window(candidate(100.0, raw_points=10), 2.0) == 4.0


def test_soft_accumulator_rejects_singletons_but_keeps_coherent_motion():
    moving = [[candidate(100.0, y=0.0)], [candidate(100.0, y=5.0)], [candidate(100.0, y=10.0)]]
    accepted, scored, _ = M.soft_accumulate(moving, np.asarray([0.0, 1.0, 2.0]), adaptive=False, doppler=False)
    assert len(accepted[0]) == 0
    assert len(accepted[1]) == 1
    assert scored[1][0]["evidence_score"] >= M.PROTOCOL["soft_score"]["formal_threshold"]

    isolated = [[candidate(100.0)], [], [candidate(200.0)]]
    accepted, _, _ = M.soft_accumulate(isolated, np.asarray([0.0, 1.0, 2.0]), adaptive=False, doppler=False)
    assert [len(x) for x in accepted] == [0, 0, 0]


def test_doppler_is_soft_not_a_hard_gate():
    frames = [[candidate(100.0, y=0.0)], [candidate(100.0, y=5.0, rv=100.0)]]
    no_doppler, _, _ = M.soft_accumulate(frames, np.asarray([0.0, 1.0]), adaptive=True, doppler=False)
    with_doppler, scored, _ = M.soft_accumulate(frames, np.asarray([0.0, 1.0]), adaptive=True, doppler=True)
    assert len(no_doppler[1]) == 1
    # The candidate remains represented and linked even if its soft score falls.
    assert scored[1][0]["track_id"] == scored[0][0]["track_id"]
    assert scored[1][0]["doppler_adjustment"] < 0
    assert len(with_doppler[1]) in (0, 1)
