#!/usr/bin/env python3
"""Term-only audit of a53702d E18. Never generate or alter its graphs."""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np

import background_motion_core as core
import background_motion_dp_mvp as e18
import published_baseline_dp_mvp as published

ROOT = e18.ROOT
OUT = ROOT / "results/e18_mechanism_generalization_audit"
BASE_COMMIT = "a53702da1dc5357149e3e472a9e489ff08e9afe4"
ARMS = ("original", "background_only", "motion_only", "background_motion_E18")
FLAGS = {"original": (False, False), "background_only": (True, False),
         "motion_only": (False, True), "background_motion_E18": (True, True)}
PROTOCOL = {
    "version": 1, "base_commit": BASE_COMMIT, "status": "FROZEN_TERM_ABLATION_DEV_SILVER",
    "arms": list(ARMS), "families": list(e18.FAMILIES),
    "input": "read only the ten existing E18 nodes/edges CSV snapshots",
    "solver": "unchanged E18 eligibility-aware core.solve, identical order/gates/ties",
    "ablation": "disable entire background or entire motion term by node score-field routing only; enabled terms retain exact E18 unit weights",
    "graph_generation": "FORBIDDEN", "new_weight_or_feature": "FORBIDDEN",
    "doppler": "NOT_USED", "silver_gate_m": 3.0,
    "labels": "hash bytes before run; parse only after all full-bag unconditioned outputs complete",
    "references": "reuse E18 fixed_reference_paths.csv without selecting any new witness",
    "rank": "strict-exceedance terminal-edge winner representatives, tolerance 1e-9; exact global combinatorial rank UNKNOWN",
    "threshold": "NONE; do not choose an operating point",
    "negative_policy": "require explicit reliable target absence evidence; unlabelled/no-return/failed-closure windows are NOT negatives",
    "hover_policy": "require existing identified hover/very-slow UAV evidence and frozen graph; otherwise no empirical stress run, UNKNOWN",
    "short_slow_stress": "separate retrospective cross-silver subwindows only: ALL maximal consecutive observed runs whose EVERY segment speed <= inherited E18 static diagnostic 1m/median full-bag dt, >=3 observations and >=2s; same frozen graph and full-bag prior background, never reset occupancy; not independent hover/generalization",
    "descriptive_comparisons": "four arms by sequence/family/range; paired incremental hit counts and fixed-path score/margin decomposition; no significance tests",
    "randomness": "NONE",
}


def solve_arm(nodes, edges, incoming, arm, **kwargs):
    background, motion = FLAGS[arm]
    # No mutation, reordering, new scoring expression, solver fork or monkeypatch.
    if arm == "background_only":
        view = [dict(n, score=n["new_score"]) for n in nodes]
    elif arm == "motion_only":
        view = [dict(n, new_score=n["score"]) for n in nodes]
    else:
        view = nodes
    return core.solve(view, edges, incoming,
                      method="background_motion" if motion else "original_score", **kwargs)


def path_parts_arm(path, nodes, edges, arm):
    background, motion = FLAGS[arm]
    # Calculate all frozen terms even if disabled, distinguishing raw/applied.
    raw = e18.path_parts(path, nodes, edges, "background_motion")
    old = sum(float(nodes[n]["score"]) for n in path)
    b = raw["background_evidence_term_total"] if background else 0.0
    sp = raw["speed_penalty_total"] if motion else 0.0
    dp = raw["direction_penalty_total"] if motion else 0.0
    xyz = np.asarray([nodes[n]["center"] for n in path])
    velocities = [(nodes[v]["center"] - nodes[u]["center"]) /
                  (nodes[v]["time"] - nodes[u]["time"]) for u, v in zip(path[:-1], path[1:])]
    accelerations = [np.linalg.norm(v1-v0) / (.5 * ((nodes[path[i+1]]["time"]-nodes[path[i]]["time"]) +
                    (nodes[path[i+2]]["time"]-nodes[path[i+1]]["time"])))
                    for i, (v0, v1) in enumerate(zip(velocities[:-1], velocities[1:]))]
    raw.update({"old_node_score_total": old,
        "background_contribution_total": b, "background_applied": background,
        "motion_applied": motion, "speed_penalty_applied": sp, "direction_penalty_applied": dp,
        "motion_penalty_applied": sp+dp,
        "cumulative_score": old+b-raw["gap_penalty_total"]-sp-dp,
        "background_p_median": float(np.median([nodes[n]["background_p_beta11"] for n in path])) if path else "UNKNOWN",
        "background_negative_nodes": sum(nodes[n]["background_evidence_term"] < 0 for n in path),
        "start_to_end_displacement_m": float(np.linalg.norm(xyz[-1]-xyz[0])) if len(xyz) else 0.0,
        "travel_distance_m": float(sum(np.linalg.norm(v-u) for u,v in zip(xyz[:-1],xyz[1:]))) if len(xyz) else 0.0,
        "acceleration_median_mps2": float(np.median(accelerations)) if accelerations else "UNKNOWN",
        "speed_max_mps": float(max(map(np.linalg.norm,velocities))) if velocities else "UNKNOWN",
    })
    return raw


def protected():
    identities = json.loads((e18.OUT / "input_identity.json").read_text())
    identities.update(e18.attribution.hashes([p for p in e18.OUT.rglob("*") if p.is_file()]))
    identities.update(e18.attribution.hashes([ROOT/"FMCW_Background_Motion_DP_MVP.md",
        Path(core.__file__).resolve(), Path(e18.__file__).resolve(),
        ROOT/"tests/test_background_motion_core.py", ROOT/"tests/test_background_motion_wrapper.py"]))
    return identities


def freeze():
    if OUT.exists():
        raise RuntimeError("independent audit directory exists; refusing overwrite")
    e18.check_frozen()
    if published.git_head() != BASE_COMMIT:
        raise RuntimeError("HEAD must be the user-specified frozen E18 commit")
    OUT.mkdir(parents=True)
    e18.jwrite(OUT/"protocol.json", PROTOCOL)
    e18.jwrite(OUT/"input_identity.json", protected())
    e18.jwrite(OUT/"freeze_identity.json", {"utc": datetime.now(timezone.utc).isoformat(),
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py freeze",
        "parent_commit": published.git_head(), "dirty": published.git_dirty(),
        "source_sha256": e18.attribution.hashes([Path(__file__).resolve(), ROOT/"tests/test_e18_mechanism_audit.py"]),
        "protocol_sha256": published.sha256(OUT/"protocol.json")})
    (OUT/"FROZEN").write_text("E18 term-only audit; no graph, label, gate or weight changes.\n")
    print("E19 audit frozen before new four-arm real-data scoring", flush=True)


def check():
    assert (OUT/"FROZEN").is_file()
    assert json.loads((OUT/"protocol.json").read_text()) == PROTOCOL
    for p, sha in json.loads((OUT/"freeze_identity.json").read_text())["source_sha256"].items():
        assert published.sha256(ROOT/p) == sha, p
    for p, sha in json.loads((OUT/"input_identity.json").read_text()).items():
        assert published.sha256(ROOT/p) == sha, p
    e18.check_frozen()


def group_paths(rows, type_key):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["family"], row["condition_id"], row[type_key])].append(row)
    return {k: tuple(int(r["node_id"]) for r in sorted(v,key=lambda x:int(x["path_order"]))) for k,v in grouped.items()}


def run():
    check()
    if (OUT/"run_manifest.json").exists():
        raise RuntimeError("completed run exists; refusing overwrite")
    selected, summary, replay = [], [], []
    previous = group_paths(published.read_csv(e18.OUT/"selected_paths.csv"), "method")
    started, utc = time.perf_counter(), datetime.now(timezone.utc).isoformat()
    for entry in json.loads((e18.OUT/"graph_index.json").read_text())["graphs"]:
        nodes,_,edges,incoming = e18.load_graph(entry)
        family,cid = entry["family"],entry["condition_id"]
        for arm in ARMS:
            tick=time.perf_counter()
            dp=solve_arm(nodes,edges,incoming,arm)
            path=dp["best_path"]
            parts=path_parts_arm(path,nodes,edges,arm)
            assert abs(parts["cumulative_score"]-dp["best_score"]) <= 1e-9
            if arm in ("original", "background_motion_E18"):
                old_method="original_score" if arm=="original" else "background_motion"
                assert path == previous[(family,cid,old_method)], (family,cid,arm,"path replay")
                oldpool=published.read_csv(e18.OUT/"winner_pools"/f"{family}__{cid}__{old_method}__full.csv")
                assert len(oldpool)==len(dp["qualifying"])
                assert all(dp["qualified_scores"][int(r["edge_id"])]==float(r["score"]) for r in oldpool)
                replay.append({"family":family,"condition_id":cid,"arm":arm,"path_sha256":e18.digest(path),
                    "path_exact":True,"all_qualified_scores_exact":True,"qualified_representatives":len(oldpool)})
            meta={"family":family,"condition_id":cid,"arm":arm}
            selected += e18.path_rows(path,nodes,meta)
            summary.append({**meta,"path_sha256":e18.digest(path),"dp_best_score":dp["best_score"],
                "frames":entry["frames"],"candidate_nodes":entry["nodes"],"edges":entry["edges"],
                "candidate_volume_per_frame":entry["nodes"]/entry["frames"],
                "output_volume_per_frame":len(path)/entry["frames"],"qualified_representatives":len(dp["qualifying"]),
                "runtime_s":time.perf_counter()-tick, **parts,
                "target_presence":"UNKNOWN_OUTSIDE_SILVER", "score_threshold":"NONE"})
            # All representative scores suffice to independently recount ranks.
            e18.csvwrite(OUT/"terminal_scores"/f"{family}__{cid}__{arm}__full.csv",
                [{"edge_id":i,"score":dp["qualified_scores"][i]} for i in dp["qualifying"]])
            print(f"full {family}/{cid}/{arm}: score={dp['best_score']:.6f}; labels not loaded",flush=True)
            del dp
    e18.csvwrite(OUT/"selected_paths.csv", selected)
    e18.csvwrite(OUT/"unconditioned_summary.csv", summary)
    e18.csvwrite(OUT/"e18_exact_replay.csv", replay)
    check()
    e18.jwrite(OUT/"run_manifest.json", {"utc_start":utc,"utc_end":datetime.now(timezone.utc).isoformat(),
        "elapsed_s":time.perf_counter()-started,
        "command":"PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py run",
        "labels_parsed_or_used_for_inference":False, "graph_generation":False,
        "environment":{"python":platform.python_version(),"numpy":np.__version__,"platform":platform.platform()},
        "output_sha256":e18.attribution.hashes(list(OUT.glob("*.csv"))+list((OUT/"terminal_scores").glob("*.csv")))})


def rank(path,parts,dp,nodes,edges,meta):
    above=sum(s>parts["cumulative_score"]+1e-9 for s in dp["qualified_scores"].values())
    lookup={(e["u"],e["v"]):i for i,e in enumerate(edges)}
    eid=lookup[(path[-2],path[-1])]
    retained=eid in dp["qualified_scores"] and core.reconstruct(dp,eid)==path
    assert e18.reference_qualified(path,parts)
    return {**meta,"path_score":parts["cumulative_score"],"representatives":len(dp["qualifying"]),
        "strict_exceedance_count":above,"score_rank_lower_bound":above+1,
        "reference_retained":retained,"exact_rank_in_representatives":above+1 if retained else "UNKNOWN_NOT_RETAINED",
        "global_combinatorial_rank":"UNKNOWN"}


def slow_windows(labels, times):
    """Descriptive window selection, not a new graph/detection gate."""
    scale = 1.0 / float(np.median(np.diff(times)))
    runs, current = [], []
    for fi in sorted(labels):
        if current:
            prev = current[-1]
            speed = float(np.linalg.norm(labels[fi]-labels[prev])/(times[fi]-times[prev]))
            if fi != prev+1 or speed > scale:
                runs.append(current)
                current = []
        current.append(fi)
    if current:
        runs.append(current)
    return [run for run in runs if len(run)>=core.PATH_MIN_NODES and
            times[run[-1]]-times[run[0]]>=core.PATH_MIN_DURATION_S], scale


def evaluate():
    check()
    if not (OUT/"run_manifest.json").is_file() or (OUT/"evaluation_manifest.json").exists():
        raise RuntimeError("requires completed run and no previous evaluation")
    for p,sha in json.loads((OUT/"run_manifest.json").read_text())["output_sha256"].items():
        assert published.sha256(ROOT/p)==sha
    labels_all=published.load_labels()
    references=group_paths(published.read_csv(e18.OUT/"fixed_reference_paths.csv"),"path_type")
    selected=group_paths(published.read_csv(OUT/"selected_paths.csv"),"arm")
    summaries={(r["family"],r["condition_id"],r["arm"]):r for r in published.read_csv(OUT/"unconditioned_summary.csv")}
    metrics,scores,ranks,margins,observations=[],[],[],[],[]
    slow_descriptors,slow_outputs,slow_selected,slow_scores=[],[],[],[]
    for entry in json.loads((e18.OUT/"graph_index.json").read_text())["graphs"]:
        nodes,_,edges,incoming=e18.load_graph(entry)
        family,cid=entry["family"],entry["condition_id"]
        labels={int(r["frame_index"]):np.array([float(r[f"center_{a}_m"]) for a in "xyz"])
                for r in labels_all if r["condition_id"]==cid}
        lo,hi=min(labels),max(labels)
        windows,slow_scale=slow_windows(labels,entry["times"])
        for arm in ARMS:
            meta={"family":family,"condition_id":cid,"arm":arm,"nominal_range_m":int(cid.split('m_')[0])}
            dp=solve_arm(nodes,edges,incoming,arm,frame_min=lo,frame_max=hi)
            refparts={}
            for kind in ("silver_feasible","top_clutter_full","top_clutter_label_span"):
                path=references[(family,cid,kind)]
                parts=path_parts_arm(path,nodes,edges,arm); refparts[kind]=parts
                scores.append({**meta,"path_type":kind,"path_sha256":e18.digest(path),**parts,
                    "reference_identity":"E18_FIXED_NOT_RESELECTED", "object_identity":
                    "SILVER_GUIDED_WITNESS" if kind=="silver_feasible" else
                    "E14_PERSISTENT_CLUTTER" if family==e18.FAMILIES[1] else "OLD_WINNER_NOT_ASSUMED_CLUTTER"})
                if kind!="top_clutter_full":
                    ranks.append(rank(path,parts,dp,nodes,edges,{**meta,"path_type":kind,"scope":"matched_silver_span"}))
            margins.append({**meta,"silver_window_first":lo,"silver_window_last":hi,
                "duration_difference_s":refparts["silver_feasible"]["duration_s"]-refparts["top_clutter_label_span"]["duration_s"],
                "uav_minus_old_reference":refparts["silver_feasible"]["cumulative_score"]-refparts["top_clutter_label_span"]["cumulative_score"]})
            outpath=selected[(family,cid,arm)]
            path_by_frame={nodes[n]["frame"]:n for n in outpath}
            hits=e18.silver_hits(outpath,nodes,labels)
            summary=summaries[(family,cid,arm)]
            metrics.append({**summary,**meta,"silver_hits":hits,"silver_labels":len(labels),
                "silver_recall":hits/len(labels),"operational_precision":"UNKNOWN", "false_alarms":"UNKNOWN"})
            for fi,center in sorted(labels.items()):
                nid=path_by_frame.get(fi)
                d=float(np.linalg.norm(nodes[nid]["center"]-center)) if nid is not None else "UNKNOWN_NO_OUTPUT"
                observations.append({**meta,"frame_index":fi,"selected_node":nid if nid is not None else "NONE",
                    "distance_m":d,"silver_hit":isinstance(d,float) and d<=3.0,"match_gate_m":3.0})
            # Raw occupancy diagnostics only on the already frozen witness.
            if arm=="original":
                path=references[(family,cid,"silver_feasible")]
                witness_rows=[]
                for order,nid in enumerate(path):
                    n=nodes[nid]
                    prev=nodes[path[order-1]] if order else None
                    witness_rows.append({"family":family,"condition_id":cid,"path_order":order,"node_id":nid,
                        "frame_index":n["frame"],"time":n["time"],"cell_xyz":json.dumps(np.floor(n["center"]).astype(int).tolist()),
                        "prior_frames":n["background_prior_frames"],"prior_hits":n["background_prior_hits"],
                        "background_p":n["background_p_beta11"],"background_term":n["background_evidence_term"],
                        "old_node_score":n["score"],"augmented_node_score":n["new_score"],
                        "segment_speed_mps":float(np.linalg.norm(n["center"]-prev["center"])/(n["time"]-prev["time"])) if prev else "UNKNOWN_FIRST_NODE",
                        "motion_mode":"CROSS_DEV_NOT_HOVER_GT"})
                e18.csvwrite(OUT/"witness_background"/f"{family}__{cid}.csv",witness_rows)
            del dp
        for window in windows:
            slo,shi=window[0],window[-1]
            window_id=f"{cid}__frames{slo}_{shi}"
            speed_values=[float(np.linalg.norm(labels[v]-labels[u])/(entry["times"][v]-entry["times"][u]))
                          for u,v in zip(window[:-1],window[1:])]
            slow_descriptors.append({"family":family,"condition_id":cid,"window_id":window_id,
                "first_frame":slo,"last_frame":shi,"observations":len(window),
                "duration_s":entry["times"][shi]-entry["times"][slo],
                "silver_speed_min_mps":min(speed_values),"silver_speed_max_mps":max(speed_values),
                "silver_speed_median_mps":float(np.median(speed_values)),"inherited_static_scale_mps":slow_scale,
                "status":"RETROSPECTIVE_SHORT_SLOW_CROSS_DEV_NOT_INDEPENDENT_HOVER_GT"})
            fixed=tuple(n for n in references[(family,cid,"silver_feasible")] if slo<=nodes[n]["frame"]<=shi)
            assert len(fixed)>=core.PATH_MIN_NODES
            for arm in ARMS:
                dp=solve_arm(nodes,edges,incoming,arm,frame_min=slo,frame_max=shi)
                path=dp["best_path"]
                meta={"family":family,"condition_id":cid,"window_id":window_id,"arm":arm}
                parts=path_parts_arm(path,nodes,edges,arm)
                if path:
                    assert abs(parts["cumulative_score"]-dp["best_score"])<=1e-9
                local_labels={fi:labels[fi] for fi in window}
                hits=e18.silver_hits(path,nodes,local_labels)
                slow_outputs.append({**meta,"silver_hits":hits,"silver_labels":len(window),
                    "silver_recall":hits/len(window),**parts,"occupancy_reset":False,
                    "independent_hover_generalization":"UNKNOWN"})
                slow_selected+=e18.path_rows(path,nodes,meta)
                slow_scores.append({**meta,**path_parts_arm(fixed,nodes,edges,arm),
                    "path_sha256":e18.digest(fixed),"reference":"EXISTING_E18_WITNESS_RESTRICTED_ONLY"})
                del dp
        print(f"evaluate {family}/{cid}: unchanged references and 3m gate",flush=True)
    grouped=[]
    for family in e18.FAMILIES:
        for arm in ARMS:
            for nominal in (100,200,300):
                rr=[r for r in metrics if r["family"]==family and r["arm"]==arm and r["nominal_range_m"]==nominal]
                h,n=sum(int(r["silver_hits"]) for r in rr),sum(int(r["silver_labels"]) for r in rr)
                grouped.append({"family":family,"arm":arm,"nominal_range_m":nominal,"sequences":len(rr),
                    "silver_hits":h,"silver_labels":n,"silver_recall":h/n,
                    "candidate_nodes":sum(int(r["candidate_nodes"]) for r in rr),
                    "output_nodes":sum(int(r["path_nodes"]) for r in rr),"frames":sum(int(r["frames"]) for r in rr),
                    "precision":"UNKNOWN_VOLUME_IS_NOT_PRECISION"})
    for name,rows in (("unconditioned_evaluation",metrics),("summary_by_range",grouped),
                      ("fixed_path_scores",scores),("fixed_path_ranks",ranks),
                      ("score_margins",margins),("silver_frame_evaluation",observations),
                      ("short_slow_windows",slow_descriptors),("short_slow_evaluation",slow_outputs),
                      ("short_slow_selected_paths",slow_selected),("short_slow_fixed_path_scores",slow_scores)):
        e18.csvwrite(OUT/f"{name}.csv",rows)
    check()
    e18.jwrite(OUT/"evaluation_manifest.json",{"utc":datetime.now(timezone.utc).isoformat(),
        "command":"PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_audit.py evaluate",
        "protected_files_unchanged":len(protected()),"silver_gate_m":3.0,"reference_reselection":False,
        "output_sha256":e18.attribution.hashes(list(OUT.glob("*.csv"))+list((OUT/"witness_background").glob("*.csv")))})


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("phase",choices=("freeze","run","evaluate"))
    {"freeze":freeze,"run":run,"evaluate":evaluate}[p.parse_args().phase]()
