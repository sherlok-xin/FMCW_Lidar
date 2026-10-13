#!/usr/bin/env python3
"""Independent, anchor-free validation of the frozen descriptive scene audit.

The full check reconstructs each bag/source's history from saved current cells
using separate array state, checks input lineage and all node occupancies, and
compares selected geometry to exhaustive cell distances. It never reads anchor
or GNSS outcomes. Protected outcome files are read as bytes for SHA-256 only.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import time
import unittest

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
spec = importlib.util.spec_from_file_location("frozen_scene_audit", OUT / "scene_audit.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def sorted_cells(cells):
    cells = np.asarray(cells, dtype=np.int32).reshape(-1, 3)
    if not len(cells):
        return cells
    return cells[np.lexsort((cells[:, 2], cells[:, 1], cells[:, 0]))]


def close(actual, expected, label):
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-10,
                               equal_nan=True, err_msg=label)


def brute_geometry(points, cells, size):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    cells = np.asarray(cells, dtype=np.int32).reshape(-1, 3)
    if not len(cells):
        return (np.full(len(points), np.nan), np.full(len(points), np.nan),
                np.zeros(len(points), dtype=int))
    distances, centres, counts = [], [], []
    lo = cells * size
    cc = (cells + .5) * size
    for point in points:
        delta = np.maximum(np.maximum(lo - point, point - (lo + size)), 0)
        squared = np.sum(delta * delta, axis=1)
        centre_squared = np.sum((cc - point) ** 2, axis=1)
        distances.append(float(np.sqrt(squared.min())))
        centres.append(float(np.sqrt(centre_squared.min())))
        counts.append(int((centre_squared <= 25).sum()))
    return np.array(distances), np.array(centres), np.array(counts)


class SyntheticChecks(unittest.TestCase):
    def test_manual_count_empty_frame_age_runs_and_persistence(self):
        hist = audit.History(1.)
        times = np.array([10., 10.4, 12., 13.5, 14.2, 15.])
        a, b = np.array([[0, -1, 2]]), np.array([[3, 4, 5]])
        # The driver collapses repeated raw points to one occupancy per frame.
        first = np.unique(np.repeat(a, 3, axis=0), axis=0)
        ids = hist.register(first)
        h, last, age, run, episodes = hist.query(first, 0, times)
        np.testing.assert_array_equal(h, [0])
        np.testing.assert_array_equal(last, [-1])
        self.assertTrue(np.isnan(age[0]))
        self.assertEqual(len(hist.persistent(0)), 0)
        hist.update(ids, 0)
        # An empty frame increments the denominator without adding hits.
        hist.update(hist.register(np.empty((0, 3), int)), 1)
        current = np.concatenate([a, b])
        ids = hist.register(current)
        h, last, age, run, episodes = hist.query(current, 2, times)
        np.testing.assert_array_equal(h, [1, 0])
        np.testing.assert_array_equal(last, [0, -1])
        close(age, [2., np.nan], "manual age uses bag time")
        np.testing.assert_array_equal(run, [1, 0])
        np.testing.assert_array_equal(episodes, [1, 0])
        self.assertEqual(len(hist.persistent(2)), 0)
        hist.update(ids, 2)
        np.testing.assert_array_equal(hist.persistent(3), a)
        hist.update(hist.register(b), 3)
        h, last, age, run, episodes = hist.query(current, 4, times)
        np.testing.assert_array_equal(h, [2, 2])
        np.testing.assert_array_equal(last, [2, 3])
        close(age, [2.2, .7], "manual revisit ages")
        np.testing.assert_array_equal(run, [1, 2])
        np.testing.assert_array_equal(episodes, [2, 1])
        np.testing.assert_array_equal(sorted_cells(hist.persistent(4)), sorted_cells(current))
        self.assertEqual(len(hist.persistent(5)), 0)

    def test_prior_only_prefix_invariance_including_node_features(self):
        rng = np.random.default_rng(20261013)
        frames = [rng.integers(-3, 4, (8, 3)) for _ in range(8)]
        frames[2] = np.empty((0, 3), int)
        times = np.array([20., 20.4, 21.1, 22.2, 23.4, 24.1, 25.8, 27.])

        def run(sequence):
            hist = audit.History(1.)
            result = []
            for fi, cells in enumerate(sequence):
                cells = np.unique(cells, axis=0)
                ids = hist.register(cells)
                fields = hist.query(cells, fi, times)
                persistent = hist.persistent(fi)
                # Query arbitrary centroids, not just present scene cells.
                values = np.array([[.1, -.2, .3, 2., 0., 0.],
                                   [2.4, 1.1, -.5, 1., 0., 0.]])
                nodes = [dict(node_id=i, source_component_id=i,
                              **{"center_" + a + "_m": values[i, j]
                                 for j, a in enumerate("xyz")}) for i in range(2)]
                features = audit.node_features(nodes, {0: [0], 1: [1]}, values,
                                               hist, fi, times, persistent, .5)
                result.append((cells, fields, persistent, features))
                hist.update(ids, fi)
            return result

        original = run(frames)
        altered = run(frames[:5] + [rng.integers(20, 30, (4, 3)) for _ in range(3)])
        short = run(frames[:5])
        for fi in range(5):
            for other in (altered, short):
                np.testing.assert_array_equal(original[fi][0], other[fi][0])
                for a, b in zip(original[fi][1], other[fi][1]):
                    close(a, b, "prefix query")
                np.testing.assert_array_equal(original[fi][2], other[fi][2])
                for a, b in zip(original[fi][3], other[fi][3]):
                    for key in a:
                        close(a[key], b[key], "prefix node " + key)

    def test_separate_bags_reset_history(self):
        a = audit.History(1.)
        b = audit.History(1.)
        cell = np.array([[1, 2, -3]])
        a.update(a.register(cell), 0)
        a.update(a.register(cell), 1)
        self.assertEqual(a.query(cell, 2, np.array([0., 1., 2.]))[0][0], 2)
        self.assertEqual(b.query(cell, 0, np.array([0.]))[0][0], 0)
        self.assertEqual(len(b.persistent(0)), 0)

    def test_beta_prior_cannot_make_unseen_cells_persistent(self):
        hist = audit.History(1.)
        hist.register(np.array([[0, 0, 0], [3, -2, 1]]))
        for cutoff in (.25, .5, .75):
            self.assertEqual(len(hist.persistent(0, cutoff)), 0)
        empty = audit.scene_geometry([[0, 0, 0]], np.empty((0, 3), int), 1.)
        self.assertTrue(np.isnan(empty[0][0]) and np.isnan(empty[1][0]))
        self.assertEqual(empty[2][0], 0)

    def test_aabb_and_ball_geometry_against_exhaustive_distances(self):
        rng = np.random.default_rng(417)
        for size in (1., .04):
            cells = np.unique(rng.integers(-25, 26, (300, 3)), axis=0)
            points = rng.uniform(-25, 26, (120, 3)) * size
            points = np.concatenate([points, cells[:10] * size,
                                     (cells[:10] + 1.) * size])
            actual = audit.scene_geometry(points, cells, size)
            expected = brute_geometry(points, cells, size)
            for a, b in zip(actual, expected):
                close(a, b, "exact AABB/ball")
        # Closed AABBs include shared boundaries; the 5m sphere includes its edge.
        cells = np.array([[0, 0, 0], [5, 0, 0], [-1, -1, -1]])
        points = np.array([[.5, .5, .5], [1., 1., 1.], [-1., -1., -1.]])
        for a, b in zip(audit.scene_geometry(points, cells, 1.),
                        brute_geometry(points, cells, 1.)):
            close(a, b, "geometry boundaries")

    def test_node_overlap_uses_children_not_centroid(self):
        hist = audit.History(1.)
        cells = np.array([[0, 0, 0]])
        hist.update(hist.register(cells), 0)
        hist.update(hist.register(cells), 1)
        values = np.array([[.1, .2, .3, 2., 0., 0.],
                           [2.1, .2, .3, 1., 0., 0.]])
        nodes = [dict(node_id=0, source_component_id=0,
                      center_x_m=1.1, center_y_m=.2, center_z_m=.3)]
        result = audit.node_features(nodes, {0: [0, 1]}, values, hist, 2,
                                     np.array([10., 11., 12.]), hist.persistent(2), .5)[0]
        self.assertEqual(result["prior_hits"], 0)
        self.assertEqual(result["fine_point_overlap_fraction"], .5)
        self.assertEqual(result["retained_raw_weight_overlap_fraction"], 2 / 3)
        self.assertEqual(result["unique_cell_overlap_fraction"], .5)
        close(result["distance_to_persistent_scene_m"], .1, "centroid distance")


def full_validation():
    receipt_path = OUT / "SCENE_FEATURES_COMPLETE.json"
    receipt = json.loads(receipt_path.read_text())
    freeze = json.loads((OUT / "freeze_manifest.json").read_text())
    assert receipt["label_or_gnss_used"] is False
    assert receipt["source_sha256"] == freeze["source_sha256"] == sha(OUT / "scene_audit.py")
    assert receipt["protocol_sha256"] == freeze["protocol_sha256"] == sha(OUT / "protocol.json")
    for name, expected in receipt["outputs"].items():
        assert sha(OUT / name) == expected, name
    forbidden = ("label", "alignment", "gnss", "silver", "observed_anchor")
    assert not any(any(token in name.lower() for token in forbidden)
                   for name in freeze["compute_inputs"]), "Outcome compute input"
    protected = json.loads((OUT / "protected_inputs.json").read_text())
    for name, expected in protected.items():
        assert sha(ROOT / name) == expected, "Protected input changed: " + name
    for name, expected in freeze["compute_inputs"].items():
        assert sha(ROOT / name) == expected, "Compute input changed: " + name

    with (OUT / "node_scene_features.csv").open(newline="", encoding="utf-8-sig") as stream:
        features = {(r["condition_id"], r["source"], float(r["persistent_cutoff"]),
                     int(r["frame_index"]), int(r["node_id"])): r
                    for r in csv.DictReader(stream)}
    with (OUT / "frame_scene_statistics.csv").open(newline="", encoding="utf-8-sig") as stream:
        frame_rows = {(r["condition_id"], r["source"], int(r["frame_index"])): r
                      for r in csv.DictReader(stream)}
    checked_cells = checked_nodes = checked_geometry = checked_frames = exact_e18 = 0
    population = []
    for cid in audit.CIDS:
        with (ROOT / f"results/background_motion_dp_mvp/graphs/fine_4cm_pregeometry__{cid}__nodes.csv").open(newline="", encoding="utf-8-sig") as stream:
            nodes = list(csv.DictReader(stream))
        byframe = {}
        for n in nodes:
            byframe.setdefault(int(n["frame_index"]), []).append(n)
        with np.load(ROOT / f"results/calibration_closure/raw_voxel_cache/{cid}.npz") as z:
            raw_values, raw_offsets = z["values"], z["offsets"]
        raw_meta = json.loads((ROOT / f"results/calibration_closure/raw_voxel_cache/{cid}.json").read_text())
        with np.load(ROOT / f"results/published_baseline_dp_mvp/raw_fine_cache/{cid}.npz") as z:
            fine_values, fine_offsets, fine_times = z["values"], z["offsets"], z["times"]
        for source in ("raw_1m", "fine_1m", "fine_004m"):
            with np.load(OUT / f"cell_history__{source}__{cid}.npz") as z:
                saved = {key: z[key] for key in z.files}
            with np.load(OUT / f"persistent_snapshots__{source}__{cid}.npz") as z:
                snapshots = {key: z[key] for key in z.files}
            cells, offsets, times = saved["cells"], saved["offsets"], saved["times"]
            size = float(saved["cell_m"])
            np.testing.assert_array_equal(times, fine_times)
            close(times, [r["bag_time"] for r in raw_meta["frames"]], "frame identity")
            np.testing.assert_array_equal(snapshots["times"], times)
            # A global address table only names cells; all statistical state below
            # starts at zero and changes strictly after the current query.
            unique, address = np.unique(cells, axis=0, return_inverse=True)
            lookup = {tuple(c): i for i, c in enumerate(unique)}
            hit = np.zeros(len(unique), np.int32)
            last = np.full(len(unique), -1, np.int32)
            current_run = np.zeros(len(unique), np.int32)
            longest = np.zeros(len(unique), np.int32)
            episodes = np.zeros(len(unique), np.int32)
            for fi in range(len(times)):
                sl = slice(int(offsets[fi]), int(offsets[fi + 1]))
                current, ix = cells[sl], address[sl]
                assert len(np.unique(ix)) == len(ix), "Duplicate cell in one frame"
                close(saved["prior_hits"][sl], hit[ix], "prior H")
                close(saved["prior_rate"][sl], hit[ix] / fi if fi else np.full(len(ix), np.nan), "prior H/F")
                close(saved["p_beta11"][sl], (hit[ix] + 1.) / (fi + 2), "prior Beta")
                close(saved["last_frame"][sl], last[ix], "last frame")
                expected_age = np.full(len(ix), np.nan)
                seen = last[ix] >= 0
                expected_age[seen] = times[fi] - times[last[ix][seen]]
                close(saved["age_s"][sl], expected_age, "bag-time age")
                close(saved["longest_previous_run"][sl], longest[ix], "longest earlier run")
                close(saved["previous_episodes"][sl], episodes[ix], "earlier episodes")
                close(saved["first_visit"][sl], hit[ix] == 0, "first visit")
                close(saved["revisit"][sl], hit[ix] > 0, "revisit")
                keep = (hit >= 2) & ((hit + 1.) / (fi + 2) >= .5)
                expected_persistent = unique[keep]
                ss = slice(int(snapshots["offsets"][fi]), int(snapshots["offsets"][fi + 1]))
                np.testing.assert_array_equal(sorted_cells(snapshots["cells"][ss]), expected_persistent)
                close(saved["persistent"][sl], keep[ix], "current persistent overlap")

                # Reconstruct the original scene source independently.
                if source == "raw_1m":
                    rv = raw_values[raw_offsets[fi]:raw_offsets[fi + 1]]
                    expected_current = rv[:, 8:11].astype(np.int32) + [0, -250, -100]
                    np.testing.assert_array_equal(current, expected_current)
                    close(saved["raw_count"][sl], rv[:, 4], "raw weights")
                    assert int(rv[:, 4].sum()) == raw_meta["frames"][fi]["nonzero_returns"]
                    assert raw_meta["frames"][fi]["valid_returns"] == raw_meta["frames"][fi]["nonzero_returns"]
                    close(saved["fine_count"][sl], np.ones(len(current)), "raw 1m elements")
                else:
                    fv = fine_values[fine_offsets[fi]:fine_offsets[fi + 1]]
                    eq, inv, counts = np.unique(np.floor(fv[:, :3].astype(float) / size).astype(np.int32),
                                                axis=0, return_inverse=True, return_counts=True)
                    np.testing.assert_array_equal(current, eq)
                    close(saved["fine_count"][sl], counts, "fine centre counts")
                    close(saved["raw_count"][sl], np.bincount(inv, weights=fv[:, 3], minlength=len(eq)), "fine retained weights")
                frame = frame_rows[cid, source, fi]
                assert int(frame["current_cells"]) == len(ix)
                assert int(frame["persistent_cells"]) == len(expected_persistent)
                assert int(frame["novel_cells"]) == int((hit[ix] == 0).sum())
                weights = saved["raw_count"][sl]
                if weights.sum():
                    for column, mask in (("previously_occupied_return_fraction", hit[ix] > 0),
                                         ("first_visit_return_fraction", hit[ix] == 0),
                                         ("low_revisit_return_fraction", hit[ix] <= 1),
                                         ("persistent_return_fraction", keep[ix])):
                        close(float(frame[column]), weights[mask].sum() / weights.sum(), column)

                # All existing nodes; every stored cutoff shares the same causal H.
                frame_nodes = byframe.get(fi, [])
                cutoff_counts = {cutoff: int(((hit >= 2) & ((hit + 1.) / (fi + 2) >= cutoff)).sum())
                                 for cutoff in ((.5,) if source == "fine_004m" else (.25, .5, .75))}
                for n in frame_nodes:
                    nid = int(n["node_id"])
                    point = np.array([float(n["center_" + a + "_m"]) for a in "xyz"])
                    key = tuple(np.floor(point / size).astype(np.int32))
                    ni = lookup.get(key)
                    nh = int(hit[ni]) if ni is not None else 0
                    nl = int(last[ni]) if ni is not None else -1
                    for cutoff in ((.5,) if source == "fine_004m" else (.25, .5, .75)):
                        feature = features[cid, source, cutoff, fi, nid]
                        assert int(feature["prior_hits"]) == nh
                        assert int(feature["prior_frames"]) == fi
                        close(float(feature["p_beta11"]), (nh + 1.) / (fi + 2), "all node Beta")
                        close(float(feature["prior_rate"]), nh / fi if fi else np.nan, "all node rate")
                        close(float(feature["age_s"]), times[fi] - times[nl] if nl >= 0 else np.nan, "all node age")
                        assert int(feature["longest_previous_run"]) == (int(longest[ni]) if ni is not None else 0)
                        assert int(feature["previous_episodes"]) == (int(episodes[ni]) if ni is not None else 0)
                        assert int(feature["persistent_cells"]) == cutoff_counts[cutoff]
                        checked_nodes += 1
                    if source == "fine_1m":
                        assert nh == int(n["background_prior_hits"])
                        assert fi == int(n["background_prior_frames"])
                        close(float(n["background_p_beta11"]), (nh + 1.) / (fi + 2), "E18 p")
                        close(float(n["background_evidence_term"]), 1 - 2 * (nh + 1.) / (fi + 2), "E18 b")
                        exact_e18 += 1
                # Fixed first/middle/last frame sample, exhaustive geometry.
                if fi in (0, len(times) // 2, len(times) - 1) and frame_nodes:
                    points = np.array([[float(n["center_" + a + "_m"]) for a in "xyz"] for n in frame_nodes])
                    distances, centre_distances, ball_counts = brute_geometry(points, expected_persistent, size)
                    for i, n in enumerate(frame_nodes):
                        f = features[cid, source, .5, fi, int(n["node_id"])]
                        close(float(f["distance_to_persistent_scene_m"]), distances[i], "saved AABB")
                        close(float(f["nearest_persistent_centre_m"]), centre_distances[i], "saved centre")
                        assert int(f["persistent_centres_within_5m"]) == int(ball_counts[i])
                        checked_geometry += 1

                consecutive = (last[ix] == fi - 1) & (hit[ix] > 0)
                current_run[ix] = np.where(consecutive, current_run[ix] + 1, 1)
                episodes[ix] += ~consecutive
                longest[ix] = np.maximum(longest[ix], current_run[ix])
                hit[ix] += 1
                last[ix] = fi
                checked_cells += len(ix)
                checked_frames += 1
            population.append({"condition_id": cid, "source": source,
                               "frames": len(times), "occupied_cell_records": len(cells),
                               "distinct_cell_addresses": len(unique)})
            print(f"VALIDATED {cid} {source}: {len(times)} frames, {len(cells)} occupied-cell records", flush=True)
    assert checked_nodes == len(features)
    assert exact_e18 == receipt["nodes"]
    assert checked_frames == 3 * receipt["frames"]
    return {"protected_hashes_checked": len(protected), "compute_input_hashes_checked": len(freeze["compute_inputs"]),
            "scene_output_hashes_checked": len(receipt["outputs"]), "frame_source_records_checked": checked_frames,
            "occupied_cell_records_prior_only_checked": checked_cells,
            "node_feature_records_prior_only_checked": checked_nodes,
            "e18_1m_node_annotations_exact": exact_e18,
            "saved_node_geometries_exhaustively_checked": checked_geometry,
            "geometry_sample_rule": "first, floor(N/2), last frame in every bag/source; cutoff .5; all existing nodes in those frames",
            "history_scope": "Every saved occupied-cell record and all node prior histories, independent state per bag/source",
            "no_anchor_or_gnss_outcomes_parsed": True,
            "population": population,
            "limitations": ["Component membership replay assertions are in the frozen compute; this validator independently checks source cell lineage and all node histories, and synthetic child overlap, without re-running the saved component extraction.",
                            "The frozen fine_004m spatial-concentration columns aggregate 0.4m cubes despite the 10m name; corrected 10m summaries must be reported separately."]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("synthetic", "full"), default="full")
    args = parser.parse_args()
    start = time.perf_counter()
    log = io.StringIO()
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SyntheticChecks))
    text = log.getvalue()
    print(text, flush=True)
    (OUT / "validation_tests.txt").write_text(text)
    payload = {"utc": audit.utc(), "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/validate_scene_audit.py --mode " + args.mode,
               "validator_sha256": sha(__file__), "scene_source_sha256": sha(OUT / "scene_audit.py"),
               "synthetic_tests": result.testsRun, "synthetic_failures": len(result.failures), "synthetic_errors": len(result.errors),
               "synthetic_pass": result.wasSuccessful(), "mode": args.mode, "full_artifact_validation": "NOT_RUN"}
    failed = not result.wasSuccessful()
    if not failed and args.mode == "full":
        try:
            payload["full_artifact_validation"] = full_validation()
        except Exception as exc:
            import traceback
            error = traceback.format_exc()
            print(error, flush=True)
            payload["full_artifact_validation"] = {"status": "FAILED", "error": str(exc), "traceback": error}
            failed = True
    payload["elapsed_s"] = time.perf_counter() - start
    payload["status"] = "FAILED" if failed else "TESTED_PASSED"
    (OUT / "validation.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
