#!/usr/bin/env python3
"""Reproducible post-run summaries/QA of the frozen attribution, not tuning.

Replays original published candidate generation in memory, audits component
merges, and summarizes both predeclared radii. Never invokes baseline run/cache.
"""
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

import dbscan_fragmentation_attribution as d


def main():
    out = d.OUT
    if (out / "independent_validation.json").exists():
        raise RuntimeError("post-run audit already exists; do not overwrite")
    before = d.hashes(d.protected_paths())
    assert before == json.loads((out / "input_identity.json").read_text())
    started, tick = datetime.now(timezone.utc).isoformat(), time.perf_counter()
    frame = d.read_csv(out / "frame_metrics.csv")
    clusters = d.read_csv(out / "cluster_metrics.csv")
    silver = d.read_csv(out / "silver_components.csv")
    rep = d.read_csv(out / "representation_support.csv")
    transition = d.read_csv(out / "adaptive_transitions.csv")
    stored = defaultdict(list)
    for c in clusters:
        if c["method"] == "adaptive_minPts2" and c["accepted"] == "True":
            stored[(c["condition_id"], int(c["frame_index"]))].append(c)
    all_merges, replay_rows = [], []
    for cid in d.published.CONDITIONS:
        frames, _ = d.published.load_fine_frames(cid)
        original, _ = d.published.published_clusters(frames)
        for fi, (values, candidates) in enumerate(zip(frames, original)):
            rr = stored[(cid, fi)]
            assert len(rr) == len(candidates)
            for old, new in zip(candidates, rr):
                np.testing.assert_array_equal(old["center"], [float(new[f"center_{a}_m"]) for a in "xyz"])
                assert old["voxels"] == int(new["voxels"])
                assert old["raw_points"] == float(new["raw_support"])
                assert old["extent"] == float(new["extent_m"])
                assert old["eps"] == float(new["eps_m"])
            lf, cf = d.cluster_frame(values, .6, 2)
            la, ca = d.cluster_frame(values, d.eps_for(values, "adaptive"), 2)
            mapped_large = 0
            for lab, c in ca.items():
                parents = sorted(set(int(x) for x in lf[la == lab] if x >= 0))
                for parent in parents:
                    # Growing epsilon cannot split any fixed connected component.
                    assert np.all(la[lf == parent] == lab)
                large_parents = sum(cf[x]["large"] for x in parents)
                mapped_large += large_parents
                assert not large_parents or c["large"]
                all_merges.append({
                    "condition_id": cid, "nominal_range_m": int(cid[:3]), "frame_index": fi,
                    "adaptive_component_id": lab, "adaptive_voxels": c["voxels"],
                    "adaptive_raw_support": c["raw_support"], "adaptive_extent_m": c["extent_m"],
                    "adaptive_accepted": int(c["accepted"]), "adaptive_large": int(c["large"]),
                    "fixed_component_parents": len(parents), "fixed_large_parents": large_parents,
                    "fixed_accepted_parents": sum(cf[x]["accepted"] for x in parents),
                    "absorbed_fixed_noise_voxels": int(np.sum(lf[la == lab] < 0)),
                    "newly_large_by_merging": int(c["large"] and large_parents == 0),
                    "old_large_components_coalesced": max(large_parents - 1, 0),
                })
            assert mapped_large == sum(c["large"] for c in cf.values())
        replay_rows.append({"condition_id": cid, "frames_exact": len(frames),
                            "candidates_exact": sum(map(len, original)),
                            "centers_voxels_raw_extent_eps_exact": True})
        print(f"original replay + merge audit {cid}: {len(frames)} frames", flush=True)
    connectivity, representation, changes, merge_summary, misses = [], [], [], [], []
    for group in (100, 200, 300, "ALL"):
        selected = lambda rows: [r for r in rows if group == "ALL" or int(r["nominal_range_m"]) == group]
        for radius in (1., 3.):
            rr = [r for r in selected(rep) if float(r["radius_m"]) == radius]
            row = {"nominal_range_m": group, "radius_m": radius, "silver_frames": len(rr)}
            keys = ("audit_raw_points", "same_input_point_filtered_raw_support", "same_input_point_filtered_4cm_cells",
                    "same_input_point_filtered_1m_cells", "actual_fine_cache_voxels", "existing_1m_postfilter_voxels",
                    "same_fine_means_coarsened_1m_voxels", "pure_cell_collapse_fraction")
            for k in keys:
                vals = [float(r[k]) for r in rr]
                row[k + "_median"] = float(np.median(vals))
                if k.endswith(("cells", "voxels")):
                    row[k + "_below2_frames"] = sum(v < 2 for v in vals)
                    row[k + "_below3_frames"] = sum(v < 3 for v in vals)
            representation.append(row)
            sr = [r for r in selected(silver) if int(r["minpts"]) == 2 and float(r["radius_m"]) == radius]
            for schedule in ("fixed", "adaptive"):
                rs = [r for r in sr if r["schedule"] == schedule]
                connectivity.append({"nominal_range_m": group, "radius_m": radius, "schedule": schedule,
                    "silver_frames": len(rs), "induced_connected": sum(r["induced_connected"] == "1" for r in rs),
                    "induced_fragmented": sum(int(r["induced_components"]) > 1 for r in rs),
                    "empty_proxy": sum(int(r["proxy_voxels"]) == 0 for r in rs),
                    "accepted_component_touching_proxy_frames": sum(int(r["accepted_components_touching_proxy"]) > 0 for r in rs),
                    "large_component_touching_proxy_frames": sum(int(r["large_components_touching_proxy"]) > 0 for r in rs)})
        fr = [r for r in selected(frame) if int(r["minpts"]) == 2]
        fixed = [r for r in fr if r["schedule"] == "fixed"]
        adaptive = [r for r in fr if r["schedule"] == "adaptive"]
        tr = [r for r in selected(transition) if int(r["minpts"]) == 2 and float(r["radius_m"]) == 3.]
        n = len(fixed)
        delta_candidates = sum(int(r["accepted_candidates"]) for r in adaptive) - sum(int(r["accepted_candidates"]) for r in fixed)
        delta_large = sum(int(r["large_components"]) for r in adaptive) - sum(int(r["large_components"]) for r in fixed)
        changes.append({"nominal_range_m": group, "lidar_frames": n, "silver_frames": len(tr),
                        "gained_silver_frames": sum(int(r["gained"]) for r in tr), "lost_silver_frames": sum(int(r["lost"]) for r in tr),
                        "delta_candidate_count": delta_candidates, "delta_candidates_per_frame": delta_candidates/n,
                        "delta_large_component_count": delta_large, "delta_large_components_per_frame": delta_large/n})
        mr = selected(all_merges)
        new_large = sum(r["newly_large_by_merging"] for r in mr)
        coalesced = sum(r["old_large_components_coalesced"] for r in mr)
        assert new_large - coalesced == delta_large
        merge_summary.append({"nominal_range_m": group, "lidar_frames": n,
            "newly_large_components": new_large, "old_large_components_coalesced": coalesced,
            "net_large_component_change": delta_large,
            "fixed_accepted_components_absorbed_into_adaptive_large": sum(r["fixed_accepted_parents"] for r in mr if r["adaptive_large"]),
            "adaptive_accepted_merges_of_2plus_fixed_components": sum(r["adaptive_accepted"] and r["fixed_component_parents"] >= 2 for r in mr)})
    for r in transition:
        if int(r["minpts"]) == 2 and float(r["radius_m"]) == 3. and not int(r["fixed_hit"]):
            misses.append(r)
    for name, rows in (("component_merge_audit", all_merges), ("component_merge_summary", merge_summary),
                       ("representation_by_range", representation), ("connectivity_by_range", connectivity),
                       ("adaptive_delta_by_range", changes), ("miss_attribution", misses), ("original_function_replay", replay_rows)):
        d.write_csv(out / f"{name}.csv", rows)
    assert d.hashes(d.protected_paths()) == before
    qa = {"utc_start": started, "utc_end": datetime.now(timezone.utc).isoformat(), "elapsed_s": time.perf_counter()-tick,
          "script_sha256": d.published.sha256(Path(__file__)),
          "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/summarize_dbscan_fragmentation.py",
          "original_function_replay": replay_rows, "protected_all_unchanged": True,
          "merge_partition_checks_all_frames": True,
          "descriptive_only_no_pvalues": True,
          "fallacy_scan_11_of_11": {
              "Simpson": "nominal groups and all reported", "ecological": "no population or actual-range causal law claimed",
              "Berkson": "selected DEV observed trajectories; selection-conditioned", "collider": "no adjusted regression",
              "base_rate": "no reliable negatives; volume is not precision", "regression_to_mean": "paired identical inputs, no extreme-case selection",
              "survivorship": "missed/weak silver frames preserved; unlabeled misses UNKNOWN", "look_elsewhere": "all four mandated variants and both frozen radii retained",
              "forking_paths": "no result-driven parameter selection; merge summaries post-run descriptive QA",
              "causal": "controlled code-level epsilon contrast only, no physical/operational benefit inferred",
              "reverse_causality": "not a physical causal model"},
          "postprocess_output_sha256": d.hashes([out / f"{n}.csv" for n in ("component_merge_audit", "component_merge_summary", "representation_by_range",
                                 "connectivity_by_range", "adaptive_delta_by_range", "miss_attribution", "original_function_replay")])}
    d.write_json(out / "independent_validation.json", qa)
    print(json.dumps({"delta": changes, "large_merges": merge_summary}, indent=2))


if __name__ == "__main__":
    main()
