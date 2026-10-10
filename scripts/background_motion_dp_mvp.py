#!/usr/bin/env python3
"""Frozen score-only DP comparisons; labels are parsed only in evaluate phase.

Primary: existing E16 fine minPts1 adaptive components before geometry.
Control: exact historical E13 coarse candidates/graph. Never mix their deltas.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np
import scipy
import sklearn

import background_motion_core as core
import dbscan_fragmentation_attribution as attribution
import dp_failure_forensics as forensic
import published_baseline_dp_mvp as published
import sparse_uav_mvp as sparse

ROOT = published.ROOT
OUT = ROOT / "results/background_motion_dp_mvp"
SELF = Path(__file__).resolve()
CORE = Path(core.__file__).resolve()
FAMILIES = ("fine_4cm_pregeometry", "historical_1m_graph")
METHODS = ("original_score", "background_motion")
PROTOCOL = {
    "version": 1, "status": "FROZEN_MVP_DEV_SILVER_NOT_PAPER_GT",
    "primary_family": FAMILIES[0], "families": list(FAMILIES), "methods": list(METHODS),
    "fine_candidates": "READ all saved E16 adaptive_minPts1 components before geometric acceptance; no rerun/reparameterization or acceptance/count/extent/range hard rejection",
    "historical_control": "exact E13 1m candidate generator and graph replay; old hard geometry stays historical only; cannot call this a 4cm graph",
    "graph_rules": published.PROTOCOL["candidate_graph_dp"],
    "graph_rules_scope": "inherited numeric gates/pruning/qualification; nested historical input description does NOT describe primary fine candidates",
    "graph_identity": "both scores share byte-identical frozen nodes/edges within each family; no cross-family score-causality claim",
    "background": {"cell_m": 1.0, "coordinate_frame": "sensor-relative [x,z,-y], zero-origin floor",
        "history_source": "ALL existing post-ROI/intensity/leveled-height 4cm cache voxels, not just accepted candidates",
        "update": "query before update; each occupied cell increments once per frame; include empty frames in prior-frame denominator",
        "prior": "Beta(1,1), neutral first-frame p=0.5; not calibrated P(clutter|data)",
        "p": "(prior occupied frames + 1)/(prior frames + 2)",
        "soft_contrast": "1 - 2*p", "background_weight": 1.0, "deletion": "NONE",
        "limits": "center-cell proxy may miss within-cell shape, centroid cell empty for large components; no neighborhood feature added"},
    "node_score": "original frozen score + 1*(1-2*p); original compactness/support stay soft, not physical-size hard priors",
    "transition": {"real_dt": True, "a_dt": "0.5*(previous_edge_dt+current_edge_dt)",
        "speed_penalty": "0.5*((norm(v)-norm(v_previous))/(15*a_dt))**2",
        "direction_penalty": "0.5*(norm(v)*norm(v_previous)-dot(v,v_previous))/(norm(v)*norm(v_previous)+(1m/a_dt)**2)",
        "motion_weight": 1.0, "root_two_nodes": "no prior velocity, zero motion term",
        "scales": "15 m/s^2 inherited hard acceleration bound; 1m inherited occupancy scale, not measured position uncertainty",
        "no_minimum_speed_bonus_or_gate": True},
    "weights_provenance": "one unit background contrast and one unit motion penalty, generic dimensionless equal weighting; no fit, sweep, label-derived values, or bag/range-specific values; design informed by prior failure diagnosis, not a blind held-out study",
    "search_correctness": "eligibility-aware edge DP: preserve unqualified (capped_count,start_frame) states and one qualified winner; both scores same solver; negative-score qualification bug avoided",
    "doppler": "NOT_USED, neither score nor graph; no intensity term added",
    "labels": "unchanged 80 silver observations, parsed/used ONLY after unconditioned run artifacts complete; bytes may be hashed for integrity earlier",
    "reference_paths": "historical graph: E14 fixed silver_feasible and top_clutter_full paths exactly; fine graph: one old-score lexicographic silver witness and old-score full winner, then FREEZE same paths for both rescores; never optimize silver witness by new score",
    "incomplete_references": "record partial/absent witness and candidate presence, never force 80 hits or move labels; ineligible ranks/margins UNKNOWN; incomplete silver witnesses force STOP_INSUFFICIENT_WITNESS and do not count as systematic improvement; denominator remains five",
    "rank": "within each graph, full-bag and matched silver-span eligible edge-state winner representatives; strict better-score count+1; exact representative rank only if fixed path itself retained; exact combinatorial rank UNKNOWN",
    "ranking_tie_tolerance": 1e-9, "top_k_representatives": 100,
    "output": "one highest-score eligible offline path per bag, no threshold; causal features do not imply online outputs",
    "kill": {"scope": "primary fine graph; historical control separately reported",
        "systematic_improvement": "strict representative exceedance-count reduction AND strict same-span UAV-minus-old-clutter margin increase in at least 4 of 5 sequences each",
        "persistent_old_clutter_dominance": "old fixed reference's same-window new score >= fixed silver witness new score in >=3/5 sequences; historical reference is E14 clutter, fine reference is not pre-assumed clutter",
        "persistent_new_static_dominance": "new unconditioned winner has zero silver hits and median speed <=1m/median real bag dt in >=3/5 sequences; diagnostic static scale, not an algorithm gate",
        "zero_recovery": "new primary unconditioned output zero silver hits across all five sequences",
        "action": "if any failure criterion holds: STOP_DIRECTION; no retuning/extra features/threshold search",
        "invalid_run": "causal, graph, path replay, or exhaustive correctness failures mean IMPLEMENTATION_INVALID, not a method negative"},
    "evaluation": "per sequence and nominal100/200/300: fixed-path score components/rank evidence, unconditioned silver recall/coverage, candidate/output volume, runtime; precision/false alarms UNKNOWN",
    "randomness": "NONE",
}


def jwrite(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def csvwrite(path, rows):
    published.write_csv(path, rows)


def protected():
    identity = json.loads((ROOT / "results/geometry_invariance_audit/input_identity.json").read_text())
    paths = [ROOT / p for p in identity]
    for folder in (ROOT / "results/geometry_invariance_audit", forensic.OUT):
        paths += list(folder.glob("*.csv")) + list(folder.glob("*.json")) + [folder / "FROZEN"]
    paths += [ROOT / p for p in ("FMCW_Geometry_Invariance_Audit.md", "FMCW_DP_Failure_Forensics.md",
         "scripts/dp_failure_forensics.py", "scripts/temporal_evidence_mvp.py", "scripts/sparse_uav_mvp.py")]
    return attribution.hashes(paths)


def freeze():
    if OUT.exists():
        raise RuntimeError("new output already exists, refusing overwrite")
    OUT.mkdir(parents=True)
    jwrite(OUT / "protocol.json", PROTOCOL)
    jwrite(OUT / "input_identity.json", protected())
    jwrite(OUT / "freeze_identity.json", {"utc": datetime.now(timezone.utc).isoformat(),
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py freeze",
        "parent_commit": published.git_head(), "dirty": published.git_dirty(),
        "source_sha256": attribution.hashes([SELF, CORE]),
        "test_sha256": attribution.hashes([ROOT/"tests/test_background_motion_core.py",ROOT/"tests/test_background_motion_wrapper.py"]),
        "protocol_sha256": published.sha256(OUT / "protocol.json")})
    (OUT / "FROZEN").write_text("One fixed score; labels evaluation-only; no DBSCAN/geometry tuning.\n")
    print("score/functions/kill criterion frozen", flush=True)


def check_frozen():
    assert (OUT / "FROZEN").exists()
    assert json.loads((OUT / "protocol.json").read_text()) == PROTOCOL
    identity = json.loads((OUT / "freeze_identity.json").read_text())
    assert identity["source_sha256"] == attribution.hashes([SELF, CORE])
    assert json.loads((OUT / "input_identity.json").read_text()) == protected()
    cfg=published.PROTOCOL["candidate_graph_dp"]
    for attribute,key in (("MAX_FRAME_GAP","edge_max_frame_gap"),("MAX_DT_S","edge_max_dt_s"),
        ("MAX_SPEED_MPS","edge_max_speed_mps"),("MAX_ACCELERATION_MPS2","edge_max_acceleration_mps2"),
        ("PATH_MIN_NODES","path_min_nodes"),("PATH_MIN_DURATION_S","path_min_duration_s")):
        assert getattr(core,attribute)==cfg[key]
    assert core.BACKGROUND_WEIGHT==PROTOCOL["background"]["background_weight"]


def graph_paths(family, cid):
    stem = OUT / "graphs" / f"{family}__{cid}"
    return Path(str(stem) + "__nodes.csv"), Path(str(stem) + "__edges.csv")


def graph():
    check_frozen()
    if (OUT / "graph_index.json").exists():
        raise RuntimeError("graph snapshot exists, refusing overwrite")
    all_fine = defaultdict(list)
    for r in published.read_csv(attribution.OUT / "cluster_metrics.csv"):
        if r["method"] == "adaptive_minPts1":
            all_fine[r["condition_id"]].append(r)
    index = []
    for cid in published.CONDITIONS:
        fine_frames, times = published.load_fine_frames(cid)
        for family in FAMILIES:
            if family == FAMILIES[0]:
                candidates = [[] for _ in times]
                for r in all_fine[cid]:
                    fi = int(r["frame_index"])
                    assert float(r["bag_time_unix"]) == float(times[fi])
                    candidates[fi].append({"center": np.array([float(r[f"center_{a}_m"]) for a in "xyz"]),
                        "raw_points": float(r["raw_support"]), "voxels": int(r["voxels"]), "extent": float(r["extent_m"]),
                        "source_component_id": int(r["component_id"]), "old_geometry_accepted": r["accepted"]})
                for frame in candidates:
                    frame.sort(key=lambda x: x["source_component_id"])
                assert sum(map(len, candidates)) == len(all_fine[cid])
            else:
                old_frames, old_times = sparse._load_raw_voxel_frames(cid)
                np.testing.assert_array_equal(times, old_times)
                candidates, _ = sparse.cluster_frames(old_frames, min_pts=1, adaptive=True)
            nodes, by_frame, edges, incoming = forensic.build_graph(candidates, times)
            if family == FAMILIES[1]:
                original = forensic.ordinary_dp(nodes, edges, incoming)
                forensic.verify_recorded_top(cid, original["best_path"], nodes)
            core.annotate_background(nodes, by_frame, fine_frames)
            node_rows = []
            for n in nodes:
                c = n["candidate"]
                node_rows.append({"node_id": n["id"], "frame_index": n["frame"], "bag_time": n["time"],
                    "candidate_index": n["candidate_index"], **{f"center_{a}_m": n["center"][j] for j, a in enumerate("xyz")},
                    "raw_support": c["raw_points"], "voxels": c["voxels"], "extent_m": c["extent"],
                    "source_component_id": c.get("source_component_id", "UNKNOWN_historical_replay_index"),
                    "old_geometry_accepted": c.get("old_geometry_accepted", "historical_candidate_survived_original_generator"),
                    **{f"background_cell_{a}": n["background_cell_xyz"][j] for j,a in enumerate("xyz")},
                    **{k: n[k] for k in ("score", "base_component", "compactness_component", "support_component",
                        "background_prior_hits", "background_prior_frames", "background_p_beta11", "background_evidence_term", "new_score")}})
            edge_rows = [{"edge_id": e["id"], "u": e["u"], "v": e["v"], "dt_s": e["dt"], "frame_gap": e["frame_gap"],
                          "gap_penalty": e["gap_penalty"], **{f"velocity_{a}_mps": e["velocity"][j] for j, a in enumerate("xyz")}} for e in edges]
            npth, epth = graph_paths(family, cid)
            csvwrite(npth, node_rows); csvwrite(epth, edge_rows)
            index.append({"family": family, "condition_id": cid, "frames": len(times), "nodes": len(nodes), "edges": len(edges),
                "singletons": sum(n["candidate"]["voxels"] == 1 for n in nodes), "label_used": False,
                "original_top_replay": family == FAMILIES[1], "times": times.tolist(),
                "node_sha256": published.sha256(npth), "edge_sha256": published.sha256(epth)})
            print(f"graph {family}/{cid}: {len(nodes)} nodes, {len(edges)} edges", flush=True)
    check_frozen()
    jwrite(OUT / "graph_index.json", {"utc": datetime.now(timezone.utc).isoformat(),
        "command":"PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py graph", "graphs": index})


def load_graph(entry):
    npth, epth = graph_paths(entry["family"], entry["condition_id"])
    assert published.sha256(npth) == entry["node_sha256"] and published.sha256(epth) == entry["edge_sha256"]
    nodes, by_frame = [], [[] for _ in entry["times"]]
    for r in published.read_csv(npth):
        n = {"id": int(r["node_id"]), "frame": int(r["frame_index"]), "time": float(r["bag_time"]),
             "candidate_index": int(r["candidate_index"]), "center": np.array([float(r[f"center_{a}_m"]) for a in "xyz"]),
             "candidate": {"raw_points": float(r["raw_support"]), "voxels": int(r["voxels"]), "extent": float(r["extent_m"])} }
        for k in ("score", "base_component", "compactness_component", "support_component", "background_prior_hits",
                  "background_prior_frames", "background_p_beta11", "background_evidence_term", "new_score"):
            n[k] = float(r[k])
        assert n["id"] == len(nodes)
        nodes.append(n); by_frame[n["frame"]].append(n["id"])
    edges, incoming = [], defaultdict(list)
    for r in published.read_csv(epth):
        e = {"id": int(r["edge_id"]), "u": int(r["u"]), "v": int(r["v"]), "dt": float(r["dt_s"]),
             "frame_gap": int(r["frame_gap"]), "gap_penalty": float(r["gap_penalty"]),
             "velocity": np.array([float(r[f"velocity_{a}_mps"]) for a in "xyz"])}
        assert e["id"] == len(edges)
        edges.append(e); incoming[e["v"]].append(e["id"])
    return nodes, by_frame, edges, incoming


def digest(path):
    return hashlib.sha256(json.dumps(list(path), separators=(",", ":")).encode()).hexdigest()


def pool_rows(dp, edges):
    return [{"edge_id": eid, "score": dp["qualified_scores"][eid], "path_sha256": digest(core.reconstruct(dp, eid))}
            for eid in dp["qualifying"]]


def path_parts(path, nodes, edges, method):
    lookup = {(e["u"], e["v"]): e for e in edges}
    ee = [lookup[(u,v)] for u,v in zip(path[:-1], path[1:])]
    for prev, curr in zip(ee[:-1], ee[1:]):
        accel = np.linalg.norm(curr["velocity"]-prev["velocity"])/(.5*(curr["dt"]+prev["dt"]))
        assert accel <= 15.0 + 1e-10
    totals = {f"{k}_total": sum(n[k] for n in (nodes[i] for i in path)) for k in
              ("base_component", "compactness_component", "support_component", "background_evidence_term")}
    totals["gap_penalty_total"] = sum(e["gap_penalty"] for e in ee)
    speed_p, dir_p = 0.0, 0.0
    if method == "background_motion":
        for prev, curr in zip(ee[:-1], ee[1:]):
            parts = core.motion_parts(prev, curr)
            speed_p += parts["speed_penalty"]; dir_p += parts["direction_penalty"]
    totals["speed_penalty_total"], totals["direction_penalty_total"] = speed_p, dir_p
    totals["cumulative_score"] = (totals["base_component_total"]+totals["compactness_component_total"]+
        totals["support_component_total"]+(totals["background_evidence_term_total"] if method == "background_motion" else 0.0)-
        totals["gap_penalty_total"]-speed_p-dir_p)
    totals["background_applied"] = method == "background_motion"
    totals["path_nodes"] = len(path)
    totals["duration_s"] = nodes[path[-1]]["time"]-nodes[path[0]]["time"] if path else 0.0
    totals["first_frame"] = nodes[path[0]]["frame"] if path else "UNKNOWN_EMPTY_PATH"
    totals["last_frame"] = nodes[path[-1]]["frame"] if path else "UNKNOWN_EMPTY_PATH"
    totals["speed_median_mps"] = float(np.median([np.linalg.norm(e["velocity"]) for e in ee])) if ee else 0.0
    totals["range_median_m"] = float(np.median([np.linalg.norm(nodes[i]["center"]) for i in path])) if path else 0.0
    return totals


def path_rows(path, nodes, meta):
    return [{**meta, "path_order": order, "node_id": nid, "frame_index": nodes[nid]["frame"], "bag_time": nodes[nid]["time"],
             **{f"center_{a}_m": nodes[nid]["center"][j] for j,a in enumerate("xyz")}} for order,nid in enumerate(path)]


def run():
    check_frozen()
    if (OUT / "run_manifest.json").exists(): raise RuntimeError("run complete, refusing overwrite")
    started, utc = time.perf_counter(), datetime.now(timezone.utc).isoformat()
    selected, summaries, top_rows = [], [], []
    for entry in json.loads((OUT / "graph_index.json").read_text())["graphs"]:
        nodes, _, edges, incoming = load_graph(entry)
        for method in METHODS:
            tick = time.perf_counter()
            dp = core.solve(nodes, edges, incoming, method=method)
            path = dp["best_path"]
            if entry["family"] == FAMILIES[1] and method == "original_score":
                forensic.verify_recorded_top(entry["condition_id"], path, nodes)
            parts = path_parts(path, nodes, edges, method)
            if path: assert abs(parts["cumulative_score"]-dp["best_score"]) <= 1e-9
            meta = {"family": entry["family"], "condition_id": entry["condition_id"], "method": method}
            selected += path_rows(path, nodes, meta)
            pool = pool_rows(dp, edges)
            csvwrite(OUT / "winner_pools" / f"{entry['family']}__{entry['condition_id']}__{method}__full.csv", pool)
            for rank,eid in enumerate(sorted(dp["qualifying"], key=lambda e: dp["qualified_scores"][e], reverse=True)[:100],1):
                top_rows += path_rows(core.reconstruct(dp,eid), nodes, {**meta, "representative_rank": rank,
                    "representative_score": dp["qualified_scores"][eid]})
            summaries.append({**meta, "frames": entry["frames"], "candidate_nodes": entry["nodes"], "edges": entry["edges"],
                "candidate_volume_per_frame": entry["nodes"]/entry["frames"], "output_volume_per_frame": len(path)/entry["frames"],
                "qualified_representatives": len(pool), "runtime_s": time.perf_counter()-tick, **parts})
            print(f"score {entry['family']}/{entry['condition_id']}/{method}: done (labels not parsed/used for inference)",flush=True)
    csvwrite(OUT / "selected_paths.csv", selected)
    csvwrite(OUT / "unconditioned_summary.csv", summaries)
    csvwrite(OUT / "top100_full_paths.csv", top_rows)
    check_frozen()
    jwrite(OUT / "run_manifest.json", {"utc_start": utc, "utc_end": datetime.now(timezone.utc).isoformat(),
        "elapsed_s": time.perf_counter()-started, "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py run",
        "labels_parsed_or_used_for_inference": False, "labels_bytes_hashed_for_integrity": True,
        "environment": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__, "sklearn": sklearn.__version__, "platform": platform.platform()},
        "output_sha256": attribution.hashes(list(OUT.glob("*.csv"))+list((OUT/"winner_pools").glob("*.csv")))})


def rank_reference(path, parts, pool, meta):
    if not reference_qualified(path, parts):
        return {**meta, "path_score": parts["cumulative_score"], "representatives": len(pool),
            "strict_exceedance_count": "UNKNOWN_INELIGIBLE_REFERENCE", "score_rank_lower_bound": "UNKNOWN",
            "exact_rank_in_representatives": "UNKNOWN", "reference_retained": False,
            "global_combinatorial_rank": "UNKNOWN", "tie_policy": "competition_rank_strict_greater_1e-9"}
    tol=PROTOCOL["ranking_tie_tolerance"]
    above=sum(float(r["score"]) > parts["cumulative_score"]+tol for r in pool)
    reference_hash=digest(path)
    matched=any(r["path_sha256"]==reference_hash for r in pool)
    return {**meta, "path_score": parts["cumulative_score"], "representatives": len(pool), "strict_exceedance_count": above,
            "score_rank_lower_bound": 1+above, "exact_rank_in_representatives": 1+above if matched else "UNKNOWN_NOT_RETAINED",
            "reference_retained": matched, "global_combinatorial_rank": "UNKNOWN", "tie_policy": "competition_rank_strict_greater_1e-9"}


def reference_qualified(path, parts):
    return len(path) >= core.PATH_MIN_NODES and parts["duration_s"] >= core.PATH_MIN_DURATION_S


def silver_hits(path, nodes, labels):
    return int(sum(nodes[n]["frame"] in labels and np.linalg.norm(nodes[n]["center"]-labels[nodes[n]["frame"]])<=3.0 for n in path))


def evaluate():
    check_frozen()
    if not (OUT/"run_manifest.json").exists() or (OUT/"evaluation_manifest.json").exists():
        raise RuntimeError("requires completed unconditioned run, not existing evaluation")
    for p,sha in json.loads((OUT/"run_manifest.json").read_text())["output_sha256"].items():
        assert published.sha256(ROOT/p)==sha
    labels_all=published.load_labels()  # First label open, after immutable unconditioned run.
    selected_rows=published.read_csv(OUT/"selected_paths.csv")
    runtimes={(r["family"],r["condition_id"],r["method"]):float(r["runtime_s"]) for r in published.read_csv(OUT/"unconditioned_summary.csv")}
    recorded=published.read_csv(forensic.OUT/"path_nodes.csv")
    scores,ranks,references,metrics,pairs,feasibility=[],[],[],[],[],[]
    for entry in json.loads((OUT/"graph_index.json").read_text())["graphs"]:
        family,cid=entry["family"],entry["condition_id"]
        nodes,_,edges,incoming=load_graph(entry)
        labels={int(r["frame_index"]):np.array([float(r[f"center_{a}_m"]) for a in "xyz"]) for r in labels_all if r["condition_id"]==cid}
        lo,hi=min(labels),max(labels)
        if family==FAMILIES[1]:
            paths={kind:tuple(int(r["node_id"]) for r in sorted([r for r in recorded if r["condition_id"]==cid and r["path_type"]==kind], key=lambda r:int(r["path_order"])))
                   for kind in ("silver_feasible","top_clutter_full","top_clutter_label_span")}
            for r in (r for r in recorded if r["condition_id"]==cid):
                np.testing.assert_allclose(nodes[int(r["node_id"])]["center"], [float(r[f"center_{a}_m"]) for a in "xyz"],rtol=0,atol=1e-5)
        else:
            # Reference fixed by OLD score only, never by new score.
            silver=forensic.silver_lexicographic_dp(nodes,edges,incoming,labels,lo,hi)
            paths={"silver_feasible":silver["best_path"], "top_clutter_full":tuple(int(r["node_id"]) for r in sorted([r for r in selected_rows
                if r["family"]==family and r["condition_id"]==cid and r["method"]=="original_score"],key=lambda r:int(r["path_order"])))}
            paths["top_clutter_label_span"]=tuple(n for n in paths["top_clutter_full"] if lo<=nodes[n]["frame"]<=hi)
        def hits(path): return silver_hits(path,nodes,labels)
        witness_hits=hits(paths["silver_feasible"])
        if family==FAMILIES[1]:
            assert witness_hits==len(labels), (family,cid,"historical recorded witness changed")
        witness_frames={nodes[n]["frame"] for n in paths["silver_feasible"] if nodes[n]["frame"] in labels and np.linalg.norm(nodes[n]["center"]-labels[nodes[n]["frame"]])<=3.0}
        reference_role="E14_RECORDED_PERSISTENT_CLUTTER" if family==FAMILIES[1] else "OLD_SCORE_UNCONDITIONED_WINNER_NOT_ASSUMED_CLUTTER"
        for fi,center in sorted(labels.items()):
            distances=[float(np.linalg.norm(n["center"]-center)) for n in nodes if n["frame"]==fi]
            feasibility.append({"family":family,"condition_id":cid,"frame_index":fi,
                "matching_candidates":sum(d<=3.0 for d in distances),"nearest_candidate_m":min(distances) if distances else "UNKNOWN_NO_CANDIDATES",
                "fixed_witness_hit":fi in witness_frames,"witness_complete":witness_hits==len(labels),
                "missing_path_implication":"absence from this witness does NOT prove no graph path exists"})
        for kind,path in paths.items(): references+=path_rows(path,nodes,{"family":family,"condition_id":cid,"path_type":kind,
            "reference_role":reference_role if kind!="silver_feasible" else "SILVER_GUIDED_FIXED_FEASIBILITY_WITNESS",
            "reference_selection":"OLD_SCORE_ONLY_FIXED_FOR_BOTH_RESCORES"})
        for method in METHODS:
            meta={"family":family,"condition_id":cid,"method":method,"nominal_range_m":int(cid.split('m_')[0])}
            span=core.solve(nodes,edges,incoming,method=method,frame_min=lo,frame_max=hi)
            spanpool=pool_rows(span,edges)
            csvwrite(OUT/"winner_pools"/f"{family}__{cid}__{method}__span.csv",spanpool)
            fullpool=published.read_csv(OUT/"winner_pools"/f"{family}__{cid}__{method}__full.csv")
            parts_by_kind={}
            for kind,path in paths.items():
                parts=path_parts(path,nodes,edges,method); parts_by_kind[kind]=parts
                scores.append({**meta,"path_type":kind,"reference_role":reference_role if kind!="silver_feasible" else "SILVER_GUIDED_FIXED_FEASIBILITY_WITNESS",
                    "reference_qualified":reference_qualified(path,parts),"silver_hits":hits(path),"silver_labels":len(labels),**parts})
                ranks.append(rank_reference(path,parts,fullpool if kind=="top_clutter_full" else spanpool,{**meta,"path_type":kind,
                    "rank_scope":"full_bag" if kind=="top_clutter_full" else "matched_silver_span"}))
            selected=tuple(int(r["node_id"]) for r in sorted([r for r in selected_rows if r["family"]==family and r["condition_id"]==cid and r["method"]==method],key=lambda r:int(r["path_order"])))
            selected_parts=path_parts(selected,nodes,edges,method)
            speed_scale=1.0/float(np.median(np.diff(entry["times"])))
            metrics.append({**meta,"silver_hits":hits(selected),"silver_labels":len(labels),"silver_recall":hits(selected)/len(labels),
                "silver_witness_hits":witness_hits,"silver_witness_complete":witness_hits==len(labels),"static_scale_mps":speed_scale,
                "zero_hit_nearstatic_winner": bool(selected) and hits(selected)==0 and selected_parts["speed_median_mps"]<=speed_scale,
                "frames":entry["frames"],"candidate_nodes":entry["nodes"],"output_nodes":len(selected),
                "runtime_s":runtimes[(family,cid,method)],
                "candidate_volume_per_frame":entry["nodes"]/entry["frames"],"output_volume_per_frame":len(selected)/entry["frames"],
                "operational_precision":"UNKNOWN_NO_RELIABLE_NEGATIVES",**selected_parts})
            valid=reference_qualified(paths["silver_feasible"],parts_by_kind["silver_feasible"]) and reference_qualified(paths["top_clutter_label_span"],parts_by_kind["top_clutter_label_span"])
            pairs.append({**meta,"old_reference_role":reference_role,"silver_witness_complete":witness_hits==len(labels),
                "same_span_comparison_eligible":valid,
                "window_first_frame":lo,"window_last_frame":hi,
                "uav_actual_first_frame":parts_by_kind["silver_feasible"]["first_frame"],"uav_actual_last_frame":parts_by_kind["silver_feasible"]["last_frame"],
                "old_reference_actual_first_frame":parts_by_kind["top_clutter_label_span"]["first_frame"],"old_reference_actual_last_frame":parts_by_kind["top_clutter_label_span"]["last_frame"],
                "duration_difference_uav_minus_reference_s":parts_by_kind["silver_feasible"]["duration_s"]-parts_by_kind["top_clutter_label_span"]["duration_s"],
                "same_span_uav_minus_old_clutter":parts_by_kind["silver_feasible"]["cumulative_score"]-parts_by_kind["top_clutter_label_span"]["cumulative_score"] if valid else "UNKNOWN_INELIGIBLE_REFERENCE",
                "full_bag_uav_minus_old_clutter":parts_by_kind["silver_feasible"]["cumulative_score"]-parts_by_kind["top_clutter_full"]["cumulative_score"] if reference_qualified(paths["silver_feasible"],parts_by_kind["silver_feasible"]) else "UNKNOWN_INELIGIBLE_REFERENCE"})
        print(f"evaluation {family}/{cid}: fixed witnesses rescored",flush=True)
    decisions=[]
    for family in FAMILIES:
        rank_improved=margin_improved=dominant=incomplete=not_evaluable=0
        for cid in published.CONDITIONS:
            rk={r["method"]:r for r in ranks if r["family"]==family and r["condition_id"]==cid and r["path_type"]=="silver_feasible"}
            pp={r["method"]:r for r in pairs if r["family"]==family and r["condition_id"]==cid}
            complete=pp[METHODS[1]]["silver_witness_complete"]
            eligible=pp[METHODS[1]]["same_span_comparison_eligible"] and isinstance(rk[METHODS[0]]["strict_exceedance_count"],int) and isinstance(rk[METHODS[1]]["strict_exceedance_count"],int)
            incomplete+=not complete; not_evaluable+=not eligible
            if complete and eligible:
                rank_improved+=rk[METHODS[1]]["strict_exceedance_count"]<rk[METHODS[0]]["strict_exceedance_count"]
                margin_improved+=pp[METHODS[1]]["same_span_uav_minus_old_clutter"]>pp[METHODS[0]]["same_span_uav_minus_old_clutter"]+1e-9
                dominant+=pp[METHODS[1]]["same_span_uav_minus_old_clutter"]<=1e-9
        mm=[m for m in metrics if m["family"]==family and m["method"]==METHODS[1]]
        static=sum(m["zero_hit_nearstatic_winner"] for m in mm); recovered=sum(m["silver_hits"] for m in mm)
        stop=incomplete>0 or not_evaluable>0 or rank_improved<4 or margin_improved<4 or dominant>=3 or static>=3 or recovered==0
        decisions.append({"family":family,"ranking_surrogate_improved_sequences":int(rank_improved),"same_span_margin_improved_sequences":int(margin_improved),
            "old_reference_still_dominant_sequences":int(dominant),"new_zero_hit_nearstatic_winners":int(static),"unconditioned_silver_hits":recovered,
            "incomplete_silver_witnesses":int(incomplete),"not_evaluable_sequences":int(not_evaluable),"sequence_denominator":5,
            "decision":"STOP_INSUFFICIENT_WITNESS" if incomplete or not_evaluable else "STOP_DIRECTION" if stop else "MVP_GATE_PASSED_NOT_OPERATIONAL_VALIDATION","primary":family==FAMILIES[0]})
    grouped=[]
    for family in FAMILIES:
        for method in METHODS:
            for nominal in (100,200,300):
                rows=[r for r in metrics if r["family"]==family and r["method"]==method and r["nominal_range_m"]==nominal]
                grouped.append({"family":family,"method":method,"nominal_range_m":nominal,"sequences":len(rows),
                    "silver_hits":sum(r["silver_hits"] for r in rows),"silver_labels":sum(r["silver_labels"] for r in rows),
                    "silver_recall":sum(r["silver_hits"] for r in rows)/sum(r["silver_labels"] for r in rows),
                    "frames":sum(r["frames"] for r in rows),"candidate_volume_per_frame":sum(r["candidate_nodes"] for r in rows)/sum(r["frames"] for r in rows),
                    "output_volume_per_frame":sum(r["output_nodes"] for r in rows)/sum(r["frames"] for r in rows),
                    "runtime_s":sum(r["runtime_s"] for r in rows),"runtime_s_per_frame":sum(r["runtime_s"] for r in rows)/sum(r["frames"] for r in rows),
                    "precision":"UNKNOWN","volume_is_not_precision":True})
    for name,rows in (("fixed_reference_paths",references),("fixed_path_scores",scores),("fixed_path_ranks",ranks),
                      ("unconditioned_evaluation",metrics),("score_margins",pairs),("reference_feasibility",feasibility),("summary_by_range",grouped),("kill_decision",decisions)):
        csvwrite(OUT/f"{name}.csv",rows)
    check_frozen()
    jwrite(OUT/"evaluation_manifest.json",{"utc":datetime.now(timezone.utc).isoformat(),"command":"PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/background_motion_dp_mvp.py evaluate",
        "protected_files_unchanged":len(protected()),"reference_paths_fixed_for_both_scores":True,"parameters_not_changed":True,
        "primary_decision":decisions[0],"output_sha256":attribution.hashes(list(OUT.glob("*.csv"))+list((OUT/"winner_pools").glob("*.csv")))})
    print(json.dumps(decisions,ensure_ascii=False),flush=True)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase",choices=("freeze","graph","run","evaluate"))
    phase=parser.parse_args().phase
    {"freeze":freeze,"graph":graph,"run":run,"evaluate":evaluate}[phase]()
