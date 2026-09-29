import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import published_baseline_dp_mvp as m


def candidate(x, raw=1, extent=0.0):
    return {
        "center": np.asarray(x, dtype=float), "raw_points": raw, "voxels": 1,
        "extent": extent, "intensity": 20.0, "radial_velocity": 0.0, "eps": 1.0,
    }


def test_voxelize_aggregates_support_and_fields():
    xyz = np.asarray([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.01, 0.0, 0.0]])
    values = m._voxelize(xyz, np.asarray([10.0, 30.0, 50.0]), np.asarray([1.0, 3.0, 5.0]), 1.0)
    assert values.shape == (2, 6)
    assert sorted(values[:, 3].tolist()) == [1.0, 2.0]
    merged = values[np.argmax(values[:, 3])]
    np.testing.assert_allclose(merged[:3], [0.015, 0.015, 0.015])
    assert merged[4] == 20.0
    assert merged[5] == 2.0


def test_published_temporal_requires_two_prior_hits():
    frames = [[candidate([10 + i, 0, 0], raw=2)] for i in range(3)]
    outputs, _ = m.published_temporal(frames, np.asarray([0.0, 1.0, 2.0]))
    assert [len(x) for x in outputs] == [0, 0, 1]
    assert outputs[2][0]["temporal_prior_support"] == 2


def test_dp_selects_long_admissible_path_without_node_threshold():
    frames = []
    for i in range(5):
        frames.append([candidate([10.0 + i, 0.0, 0.0]), candidate([100.0, 50.0 + 30 * i, 0.0])])
    outputs, diagnostic, _ = m.candidate_graph_dp(frames, np.arange(5, dtype=float))
    selected = [frame[0]["center"][0] for frame in outputs if frame]
    assert selected == [10.0, 11.0, 12.0, 13.0, 14.0]
    assert diagnostic["selected_path_nodes"] == 5
    assert sum(map(len, outputs)) == 5


def test_dp_rejects_acceleration_break():
    frames = [[candidate([10.0, 0, 0])], [candidate([11.0, 0, 0])],
              [candidate([30.0, 0, 0])], [candidate([31.0, 0, 0])]]
    outputs, diagnostic, _ = m.candidate_graph_dp(frames, np.arange(4, dtype=float))
    assert diagnostic["selected_path_nodes"] < 4
    assert sum(map(len, outputs)) < 4
