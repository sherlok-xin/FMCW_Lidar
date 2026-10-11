#!/usr/bin/env python3
"""Descriptive accounting only; never execute a detector/solver or choose weights."""
from datetime import datetime, timezone
from pathlib import Path
import json

import e18_mechanism_audit as audit


def main():
    out=audit.OUT
    audit.check()
    if not (out/"evaluation_manifest.json").is_file() or (out/"summary_manifest.json").exists():
        raise RuntimeError("requires completed evaluation, refuses output overwrite")
    rows=audit.published.read_csv(out/"unconditioned_evaluation.csv")
    indexed={(r["family"],r["condition_id"],r["arm"]):r for r in rows}
    contrasts=[]
    for family in audit.e18.FAMILIES:
        for cid in audit.published.CONDITIONS:
            arms={a:indexed[(family,cid,a)] for a in audit.ARMS}
            h={a:int(r["silver_hits"]) for a,r in arms.items()}
            contrasts.append({"family":family,"condition_id":cid,"silver_labels":int(arms["original"]["silver_labels"]),
                **{f"{a}_hits":h[a] for a in audit.ARMS},
                "background_increment_without_motion":h["background_only"]-h["original"],
                "motion_increment_without_background":h["motion_only"]-h["original"],
                "motion_increment_with_background":h["background_motion_E18"]-h["background_only"],
                "background_increment_with_motion":h["background_motion_E18"]-h["motion_only"],
                "nonadditive_selected_path_hit_interaction":h["background_motion_E18"]-h["background_only"]-h["motion_only"]+h["original"],
                "scope":"CONTROLLED_COMPUTATIONAL_ABLATION_ON_SAME_DEV_GRAPH_NOT_POPULATION_CAUSAL_EFFECT"})
    fixed=audit.published.read_csv(out/"fixed_path_scores.csv")
    fidx={(r["family"],r["condition_id"],r["path_type"],r["arm"]):r for r in fixed}
    parts=[]
    for family in audit.e18.FAMILIES:
        for cid in audit.published.CONDITIONS:
            for kind in ("silver_feasible","top_clutter_full","top_clutter_label_span"):
                rr={a:fidx[(family,cid,kind,a)] for a in audit.ARMS}
                assert len({r["path_sha256"] for r in rr.values()})==1
                s={a:float(r["cumulative_score"]) for a,r in rr.items()}
                b=float(rr["original"]["background_evidence_term_total"])
                m=float(rr["original"]["speed_penalty_total"])+float(rr["original"]["direction_penalty_total"])
                assert abs((s["background_only"]-s["original"])-b)<1e-9
                assert abs((s["motion_only"]-s["original"])+m)<1e-9
                assert abs(s["background_motion_E18"]-s["original"]-b+m)<1e-9
                parts.append({"family":family,"condition_id":cid,"path_type":kind,
                    "path_sha256":rr["original"]["path_sha256"],"background_score_change":b,
                    "motion_score_change":-m,"joint_score_change":b-m,
                    "object_identity":rr["original"]["object_identity"],"exact_fixed_path_additivity":"PASS"})
    frame_rows=audit.published.read_csv(out/"silver_frame_evaluation.csv")
    frames={}
    for row in frame_rows:
        key=(row["family"],row["condition_id"],row["frame_index"])
        frames.setdefault(key,{})[row["arm"]]=row
    changes=[]
    for (family,cid,fi),rr in frames.items():
        hit={a:r["silver_hit"]=="True" for a,r in rr.items()}
        changes.append({"family":family,"condition_id":cid,"frame_index":int(fi),
            **{f"{a}_hit":hit[a] for a in audit.ARMS},
            "background_recovers_original_miss":not hit["original"] and hit["background_only"],
            "motion_recovers_original_miss":not hit["original"] and hit["motion_only"],
            "E18_recovers_background_only_miss":not hit["background_only"] and hit["background_motion_E18"],
            "E18_loses_background_only_hit":hit["background_only"] and not hit["background_motion_E18"]})
    for name,values in (("term_contrasts",contrasts),("fixed_component_differences",parts),("matched_frame_changes",changes)):
        audit.e18.csvwrite(out/f"{name}.csv",values)
    audit.e18.jwrite(out/"summary_manifest.json",{"utc":datetime.now(timezone.utc).isoformat(),
        "command":"PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/e18_mechanism_summaries.py",
        "activity":"descriptive stored-CSV accounting and identities; no solver, weight search, or new scientific experiment",
        "source_sha256":audit.published.sha256(Path(__file__).resolve()),
        "fixed_path_additivity_checks":len(parts),
        "output_sha256":audit.e18.attribution.hashes([out/f"{name}.csv" for name in
            ("term_contrasts","fixed_component_differences","matched_frame_changes")])})
    print(json.dumps(contrasts,indent=2))


if __name__=="__main__":
    main()
