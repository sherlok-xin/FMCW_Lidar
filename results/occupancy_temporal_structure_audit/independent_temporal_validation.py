#!/usr/bin/env python3
"""Independent immutable-event validator, with no anchor/path outcome reads.

The production audit maintains incremental dictionaries.  This validator
instead creates complete per-cell event arrays, truncates each by a strict
past-frame search boundary, and derives run/episode quantities from a binary
prefix.  Synthetic checks also exercise the production query/update API.
"""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import importlib.util
import io
import json
import sys
import time
import unittest

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SCENE = ROOT / "results/scene_spatiotemporal_background_audit"
CIDS = ("100m_cross_5", "100m_cross_10", "200m_cross_5", "200m_cross_10", "300m_cross_10")
NUMERIC_FIELDS = (
    "prior_frames", "prior_hits", "prior_rate", "p_beta11", "age_s",
    "longest_previous_run", "previous_episodes", "recent_5_prior_frame_fraction",
    "last_occupied_frame", "recent_window_elapsed_s",
)


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def csvrows(path):
    with open(path, newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def prefix_quantities(complete_events, frame, times):
    """Independent binary-prefix calculation; includes no current/future hit."""
    complete_events = np.asarray(complete_events, dtype=np.int64)
    cutoff = np.searchsorted(complete_events, frame, side="left")
    past = complete_events[:cutoff]
    prefix = np.zeros(frame, dtype=bool)
    prefix[past] = True
    hits = int(prefix.sum())
    last = int(past[-1]) if hits else -1
    if frame:
        starts = np.flatnonzero(np.diff(np.r_[False, prefix, False].astype(np.int8)) == 1)
        ends = np.flatnonzero(np.diff(np.r_[False, prefix, False].astype(np.int8)) == -1)
        longest = int(np.max(ends - starts)) if len(starts) else 0
        episodes = int(len(starts))
    else:
        longest = episodes = np.nan
    probability = (hits + 1) / (frame + 2)
    return {
        "prior_frames": frame,
        "prior_hits": hits,
        "prior_rate": hits / frame if frame else np.nan,
        "p_beta11": probability,
        "age_s": float(times[frame] - times[last]) if last >= 0 else np.nan,
        "longest_previous_run": longest,
        "previous_episodes": episodes,
        "recent_5_prior_frame_fraction": float(prefix[-5:].sum() / 5) if frame >= 5 else np.nan,
        "last_occupied_frame": last,
        "recent_window_elapsed_s": float(times[frame] - times[frame - 5]) if frame >= 5 else np.nan,
        "support_state": "NO_PRIOR_FRAMES" if frame == 0 else "FIRST_VISIT_NO_PREVIOUS_OCCUPANCY" if hits == 0 else "HAS_PREVIOUS_OCCUPANCY",
        "p_bin": int(min(np.floor(probability * 20), 19)) if frame else "COLD_PRIOR",
    }


def equal_numeric(actual, expected, label):
    # Identical timestamp subtractions and fractions must agree exactly.
    for name in NUMERIC_FIELDS:
        av, ev = float(actual[name]), float(expected[name])
        if not ((np.isnan(av) and np.isnan(ev)) or av == ev):
            raise AssertionError((label, name, av, ev))
    for name in ("support_state", "p_bin"):
        if str(actual[name]) != str(expected[name]):
            raise AssertionError((label, name, actual[name], expected[name]))


def production_class():
    spec = importlib.util.spec_from_file_location("frozen_temporal_production", OUT / "temporal_audit.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CellHistory


class SyntheticTemporalTests(unittest.TestCase):
    def setUp(self):
        self.History = production_class()
        self.cell = (2, -1, 7)
        self.times = np.array([0., 1., 2., 3., 4., 5., 6., 7.4, 8.4])

    def history(self, events, upto):
        h = self.History()
        for frame in range(upto):
            h.update(np.array([self.cell], dtype=int) if frame in events else np.empty((0, 3), dtype=int), frame)
        return h

    def test_manual_counts_irregular_time_and_recent_window(self):
        h = self.history({0, 1, 3, 6}, 7)
        q = h.query(self.cell, 7, self.times)
        expected = prefix_quantities([0, 1, 3, 6], 7, self.times)
        equal_numeric(q, expected, "manual")
        self.assertEqual(q["prior_hits"], 4)
        self.assertEqual(q["p_beta11"], 5 / 9)
        self.assertEqual(q["longest_previous_run"], 2)
        self.assertEqual(q["previous_episodes"], 3)
        self.assertEqual(q["recent_5_prior_frame_fraction"], .4)
        self.assertAlmostEqual(q["age_s"], 1.4)
        self.assertAlmostEqual(q["recent_window_elapsed_s"], 5.4)

    def test_f0_prior_not_observed_history(self):
        q = self.History().query(self.cell, 0, self.times)
        equal_numeric(q, prefix_quantities([], 0, self.times), "F0")
        self.assertEqual(q["p_beta11"], .5)
        self.assertEqual(q["p_bin"], "COLD_PRIOR")
        for name in ("age_s", "longest_previous_run", "previous_episodes", "recent_5_prior_frame_fraction"):
            self.assertTrue(np.isnan(q[name]))

    def test_h0_recency_unknown_counts_degenerate(self):
        q = self.history(set(), 7).query(self.cell, 7, self.times)
        equal_numeric(q, prefix_quantities([], 7, self.times), "H0")
        self.assertTrue(np.isnan(q["age_s"]))
        self.assertEqual(q["longest_previous_run"], 0)
        self.assertEqual(q["previous_episodes"], 0)

    def test_h1_run_episode_deterministic_recency_not(self):
        a = self.history({0}, 7).query(self.cell, 7, self.times)
        b = self.history({6}, 7).query(self.cell, 7, self.times)
        self.assertEqual(a["p_beta11"], b["p_beta11"])
        for q in (a, b):
            self.assertEqual(q["longest_previous_run"], 1)
            self.assertEqual(q["previous_episodes"], 1)
        self.assertNotEqual(a["age_s"], b["age_s"])
        self.assertEqual(a["recent_5_prior_frame_fraction"], 0)
        self.assertEqual(b["recent_5_prior_frame_fraction"], .2)

    def test_same_frequency_distinct_run_episode_structure_possible(self):
        a = self.history({0, 1, 2, 3}, 7).query(self.cell, 7, self.times)
        b = self.history({0, 2, 4, 6}, 7).query(self.cell, 7, self.times)
        self.assertEqual(a["prior_hits"], b["prior_hits"])
        self.assertEqual(a["p_beta11"], b["p_beta11"])
        self.assertEqual((a["longest_previous_run"], a["previous_episodes"]), (4, 1))
        self.assertEqual((b["longest_previous_run"], b["previous_episodes"]), (1, 4))

    def test_prefix_current_future_hits_cannot_change_query(self):
        base = [0, 1, 3]
        expected = prefix_quantities(base, 5, self.times)
        for appended in (base + [5], base + [5, 6, 7, 8]):
            equal_numeric(prefix_quantities(appended, 5, self.times), expected, "prefix invariance")
        h = self.history({0, 1, 3, 5, 6, 7}, 5)
        equal_numeric(h.query(self.cell, 5, self.times), expected, "production excludes current")

    def test_window_is_exactly_five_past_frames(self):
        events = [0, 2, 3, 6, 7]
        q = self.history(set(events), 7).query(self.cell, 7, self.times)
        self.assertEqual(q["recent_5_prior_frame_fraction"], .6)  # 2,3,6; excludes 0 and current 7.
        early = self.history({0, 1, 3}, 4).query(self.cell, 4, self.times)
        self.assertTrue(np.isnan(early["recent_5_prior_frame_fraction"]))

    def test_empty_frame_breaks_run_and_bag_reset_discards_history(self):
        first = self.history({0, 1, 3}, 4)
        q = first.query(self.cell, 4, self.times)
        self.assertEqual(q["longest_previous_run"], 2)
        self.assertEqual(q["previous_episodes"], 2)
        fresh = self.History()
        equal_numeric(fresh.query(self.cell, 0, self.times), prefix_quantities([], 0, self.times), "bag reset")
        equal_numeric(fresh.query(self.cell, 4, self.times), prefix_quantities([], 4, self.times), "no cross-bag history")


def validate_all_features():
    receipt_path = OUT / "TEMPORAL_FEATURES_COMPLETE.json"
    if not receipt_path.exists():
        raise RuntimeError("Wait for TEMPORAL_FEATURES_COMPLETE.json before running validation")
    receipt = json.loads(receipt_path.read_text())
    scene_receipt = json.loads((SCENE / "SCENE_FEATURES_COMPLETE.json").read_text())
    for name, wanted in receipt["outputs"].items():
        assert sha(OUT / name) == wanted, name
    rows = csvrows(OUT / "all_node_temporal_features.csv")
    assert len(rows) == receipt["rows"] == 37117
    assert len({(r["condition_id"], r["family"], r["node_id"]) for r in rows}) == len(rows)
    bycid = {cid: [r for r in rows if r["condition_id"] == cid] for cid in CIDS}
    counts = []
    total_cell_records = 0
    for cid in CIDS:
        name = f"cell_history__fine_1m__{cid}.npz"
        assert sha(SCENE / name) == scene_receipt["outputs"][name]
        with np.load(SCENE / name) as saved:
            cells = saved["cells"]
            offsets = saved["offsets"]
            times = saved["times"]
            fields = {key: saved[key] for key in ("prior_hits", "p_beta11", "age_s", "longest_previous_run", "previous_episodes")}
        complete_events = {}
        for frame in range(len(times)):
            current = cells[offsets[frame]:offsets[frame + 1]]
            assert len(current) == len(set(map(tuple, current)))
            for cell in current:
                complete_events.setdefault(tuple(cell), []).append(frame)
        complete_events = {cell: np.array(events, dtype=int) for cell, events in complete_events.items()}
        graphs = {}
        for family in ("fine_4cm_pregeometry", "historical_1m_graph"):
            graph = csvrows(ROOT / f"results/background_motion_dp_mvp/graphs/{family}__{cid}__nodes.csv")
            graphs.update({(family, row["node_id"]): row for row in graph})
        assert len(graphs) == len(bycid[cid])
        for row in bycid[cid]:
            frame = int(row["frame_index"])
            key = (row["family"], row["node_id"])
            original = graphs[key]
            cell = tuple(int(row[f"cell_{axis}"]) for axis in "xyz")
            center_cell = tuple(np.floor([float(original[f"center_{axis}_m"]) for axis in "xyz"]).astype(int))
            assert cell == center_cell
            assert frame == int(original["frame_index"])
            assert float(row["bag_time"]) == times[frame]
            expected = prefix_quantities(complete_events.get(cell, []), frame, times)
            equal_numeric(row, expected, (cid, *key))
            assert expected["prior_hits"] == int(original["background_prior_hits"])
            assert expected["p_beta11"] == float(original["background_p_beta11"])
        # Also check every prior-only cell record, independently of node sampling.
        for frame in range(len(times)):
            for index in range(offsets[frame], offsets[frame + 1]):
                expected = prefix_quantities(complete_events[tuple(cells[index])], frame, times)
                for name in fields:
                    if frame == 0 and name in ("longest_previous_run", "previous_episodes"):
                        assert fields[name][index] == 0  # Legacy event table uses known empty count; new node cold start is UNKNOWN.
                    else:
                        av, ev = float(fields[name][index]), float(expected[name])
                        assert av == ev or (np.isnan(av) and np.isnan(ev)), (cid, frame, index, name)
        total_cell_records += len(cells)
        counts.append({"condition_id": cid, "frames": len(times), "nodes_checked": len(bycid[cid]), "cell_records_checked": len(cells), "unique_scene_cells": len(complete_events)})
        print(f"INDEPENDENT PASS {cid}: {len(bycid[cid])} nodes, {len(cells)} cell records", flush=True)
    return {"nodes_checked": len(rows), "cell_records_checked": total_cell_records, "per_sequence": counts, "feature_receipt_sha256": sha(receipt_path), "feature_table_sha256": sha(OUT / "all_node_temporal_features.csv"), "original_scene_receipt_sha256": sha(SCENE / "SCENE_FEATURES_COMPLETE.json")}


def validate_protected():
    protected = json.loads((OUT / "protected_inputs.json").read_text())
    changed = []
    for relpath, expected in protected.items():
        path = ROOT / relpath
        if not path.is_file() or sha(path) != expected:
            changed.append(relpath)
    assert not changed, changed
    freeze = json.loads((OUT / "freeze_manifest.json").read_text())
    assert sha(OUT / "temporal_audit.py") == freeze["script_sha256"]
    assert sha(OUT / "protocol.json") == freeze["protocol_sha256"]
    for relpath, expected in freeze["compute_inputs"].items():
        assert sha(ROOT / relpath) == expected, relpath
    return {"protected_files_checked": len(protected), "protected_files_changed": changed, "compute_input_files_checked": len(freeze["compute_inputs"]), "protocol_sha256": sha(OUT / "protocol.json"), "production_script_sha256": sha(OUT / "temporal_audit.py")}


def main():
    started = time.perf_counter()
    log = io.StringIO()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(SyntheticTemporalTests)
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    (OUT / "tests.txt").write_text(log.getvalue(), encoding="utf-8")
    print(log.getvalue(), end="")
    if not result.wasSuccessful():
        raise SystemExit(1)
    features = validate_all_features()
    protected = validate_protected()
    output = {
        "status": "TESTED_INDEPENDENT_VALIDATION_PASS",
        "utc": datetime.now(timezone.utc).isoformat(),
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/independent_temporal_validation.py",
        "validator_sha256": sha(__file__),
        "synthetic_tests_run": result.testsRun,
        "synthetic_failures": len(result.failures),
        "synthetic_errors": len(result.errors),
        "synthetic_tests_sha256": sha(OUT / "tests.txt"),
        "independent_method": "complete immutable per-cell event arrays; strict-left search boundary; binary past prefix; run transitions; no incremental production-state reuse",
        "anchor_path_outcomes_read": False,
        **features,
        **protected,
        "elapsed_s": time.perf_counter() - started,
    }
    (OUT / "validation.json").write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
