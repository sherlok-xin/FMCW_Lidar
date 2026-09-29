import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import dp_failure_forensics as f


def candidate(x, raw=1, extent=0.0):
    return {
        "center": np.asarray(x, dtype=float), "raw_points": raw, "voxels": 1,
        "extent": extent, "intensity": 20.0, "radial_velocity": 0.0, "eps": 1.0,
    }


def synthetic_graph():
    frames = [[candidate([10.0 + i, 0, 0]), candidate([100.0, 0.1 * i, 0])]
              for i in range(5)]
    nodes, by_frame, edges, incoming = f.build_graph(frames, np.arange(5, dtype=float))
    return nodes, by_frame, edges, incoming


def test_ordinary_dp_returns_valid_full_path():
    nodes, _, edges, incoming = synthetic_graph()
    result = f.ordinary_dp(nodes, edges, incoming)
    assert [nodes[n]["frame"] for n in result["best_path"]] == list(range(5))


def test_silver_lexicographic_path_maximizes_hits_without_score_change():
    nodes, _, edges, incoming = synthetic_graph()
    labels = {i: np.asarray([10.0 + i, 0, 0]) for i in range(5)}
    result = f.silver_lexicographic_dp(nodes, edges, incoming, labels, 0, 4)
    assert result["best_hits"] == 5
    np.testing.assert_allclose([nodes[n]["center"][0] for n in result["best_path"]], np.arange(10, 15))


def test_score_decomposition_matches_frozen_sum():
    nodes, _, edges, incoming = synthetic_graph()
    result = f.ordinary_dp(nodes, edges, incoming)
    lookup = {(edge["u"], edge["v"]): edge for edge in edges}
    components = f.path_score(result["best_path"], nodes, lookup)
    assert abs(components["cumulative_score"] - result["score"][result["best_eid"]]) < 1e-12
