#!/usr/bin/env python3
"""Reverse-analyze the single baseline track that survives the cross sequence."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree, distance
from sklearn.cluster import DBSCAN

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_fmt as bench


def qstats(a):
    a = np.asarray(a, dtype=float)
    if not len(a): return {"n": 0}
    q = np.quantile(a, [.25, .5, .75])
    return {"n": int(len(a)), "mean": float(np.mean(a)), "q25": float(q[0]),
            "median": float(q[1]), "q75": float(q[2]), "min": float(np.min(a)),
            "max": float(np.max(a))}


def cluster_indices(points):
    if not len(points): return []
    labels = DBSCAN(eps=2.0, min_samples=1).fit_predict(points)
    return [np.flatnonzero(labels == label) for label in sorted(set(labels))]


def find_associated_cluster(frame_item, state):
    if state is None or state["predicted"] or not len(frame_item["dynamic"]): return np.array([], dtype=int)
    clusters = frame_item["clusters"]
    centers = np.array([np.mean(frame_item["dynamic"][idx, :2], axis=0) for idx in clusters])
    k = int(np.argmin(np.linalg.norm(centers - np.array(state["center"]), axis=1)))
    if np.linalg.norm(centers[k] - np.array(state["center"])) > .25:
        raise RuntimeError(f"Could not map track center to DBSCAN cluster at frame {frame_item['frame']}")
    return clusters[k]


def point_rows_for_cluster(item, idx, center):
    p, intensity, velocity = item["dynamic"][idx], item["dynamic_i"][idx], item["dynamic_v"][idx]
    r = np.linalg.norm(center)
    phat = center / max(r, 1e-9)
    offsets = p - center
    radial_offset = offsets @ phat
    tangential_offset = np.linalg.norm(offsets - radial_offset[:, None] * phat, axis=1)
    return p, intensity, velocity, radial_offset, tangential_offset


def plot_success(path, frames, target_id, removals):
    fig, axs = plt.subplots(2, 2, figsize=(16, 11), dpi=200, constrained_layout=True)
    histories = {}
    for item in frames:
        for tid, s in item["states"].items():
            histories.setdefault(tid, []).append((item["frame"], *s["center"], s["predicted"]))
    for tid, h in histories.items():
        h = np.asarray(h, dtype=float)
        color, lw, alpha = ("limegreen", 3.2, 1.0) if tid == target_id else ("0.6", 1.0, .55)
        axs[0,0].plot(h[:,1], h[:,2], color=color, lw=lw, alpha=alpha)
        axs[0,0].scatter(h[0,1], h[0,2], color=color, s=18)
        if tid != target_id: axs[0,0].scatter(h[-1,1], h[-1,2], marker="x", color="crimson", s=22)
    axs[0,0].set(xlabel="x (m)", ylabel="y (m)", title="All baseline tracks; green = only final survivor")

    f = np.array([x["frame"] for x in frames])
    active = np.array([len(x["states"]) for x in frames]); dyn = np.array([len(x["dynamic"]) for x in frames])
    axs[0,1].step(f, active, where="mid", label="active baseline tracks")
    ax2 = axs[0,1].twinx(); ax2.plot(f, dyn, color="tab:orange", alpha=.7, label="dynamic points")
    axs[0,1].set(xlabel="frame", ylabel="active tracks", title="15 tracks collapse to one by frame 18")
    ax2.set_ylabel("dynamic points")
    lines = axs[0,1].lines + ax2.lines; axs[0,1].legend(lines, [x.get_label() for x in lines])

    reasons = {"static": [], "lost": []}
    for r in removals: reasons.setdefault(r["reason"], []).append(r["frame"])
    bins = np.arange(7.5, 60.5, 1)
    for reason, vals in reasons.items():
        if vals: axs[1,0].hist(vals, bins=bins, alpha=.65, label=f"removed: {reason}")
    axs[1,0].set(xlabel="frame", ylabel="tracks removed", title="Why the initial false tracks disappear"); axs[1,0].legend()

    first = frames[0]
    sizes = [len(idx) for idx in first["clusters"]]
    centers = np.array([np.mean(first["dynamic"][idx,:2], axis=0) for idx in first["clusters"]])
    target_center = np.array(first["states"][target_id]["center"])
    target_k = int(np.argmin(np.linalg.norm(centers-target_center,axis=1)))
    colors = ["limegreen" if k == target_k else "0.5" for k in range(len(sizes))]
    axs[1,1].bar(np.arange(len(sizes)), sizes, color=colors)
    axs[1,1].set(xlabel="DBSCAN cluster at frame 8", ylabel="points", title="UAV is one of 15 initial clusters, not uniquely selected")
    for ax in axs.ravel(): ax.grid(alpha=.2)
    fig.suptitle("Why the original baseline appears to work on the cross sequence")
    fig.savefig(path, bbox_inches="tight"); plt.close(fig)


def plot_micro(path, rows, frames):
    observed = [r for r in rows if not r["predicted"]]
    f = np.array([r["frame"] for r in rows]); n = np.array([r["n_points"] for r in rows])
    fig, axs = plt.subplots(2, 3, figsize=(18, 10), dpi=200, constrained_layout=True)
    dyn = np.array([r["dynamic_points"] for r in rows])
    axs[0,0].bar(f, n, label="associated UAV cluster"); axs[0,0].plot(f, dyn, color="tab:orange", label="all dynamic points")
    axs[0,0].set(xlabel="frame", ylabel="points", title="A few points persist inside a sparse candidate set"); axs[0,0].legend()
    axs[0,1].plot(f, [r["step_xy"] for r in rows], ".-", label="track step")
    axs[0,1].axhline(2, color="crimson", ls="--", label="static test threshold")
    axs[0,1].set(xlabel="frame", ylabel="m/frame", title="Track step versus the static-counter threshold"); axs[0,1].legend()
    axs[0,2].plot(f, [r["diameter"] for r in rows], ".-", label="3D diameter")
    axs[0,2].plot(f, [r["span_z"] for r in rows], ".-", label="z span")
    axs[0,2].set(xlabel="frame", ylabel="m", title="Within-frame cluster extent"); axs[0,2].legend()
    axs[1,0].fill_between(f, [r["intensity_q25"] for r in rows], [r["intensity_q75"] for r in rows], alpha=.25)
    axs[1,0].plot(f, [r["intensity_median"] for r in rows], ".-")
    axs[1,0].set(xlabel="frame", ylabel="intensity", title="UAV intensity median and IQR")
    axs[1,1].fill_between(f, [r["vr_q25"] for r in rows], [r["vr_q75"] for r in rows], alpha=.25)
    axs[1,1].plot(f, [r["vr_median"] for r in rows], ".-", label="measured median $v_r$")
    axs[1,1].plot(f, [-r["geometric_dr"] for r in rows], label="− geometric dr/dt")
    axs[1,1].axhline(0,color="k",lw=.7); axs[1,1].set(xlabel="frame",ylabel="m/s",title="Radial velocity follows flight phase"); axs[1,1].legend()
    axs[1,2].plot(f, [r["nearest_other_cluster"] for r in rows], ".-")
    axs[1,2].set(xlabel="frame",ylabel="m",title="Distance to nearest other dynamic cluster")
    for ax in axs.ravel(): ax.grid(alpha=.2)
    fig.suptitle("Cross sequence: point-level properties of baseline track ID 1")
    fig.savefig(path,bbox_inches="tight"); plt.close(fig)


def plot_evolution(path, frames, rows, reps):
    by_frame = {x["frame"]: x for x in frames}; by_row = {x["frame"]: x for x in rows}
    fig, axs = plt.subplots(2, len(reps), figsize=(4.2*len(reps), 8), dpi=220, constrained_layout=True)
    for col, frame in enumerate(reps):
        item, row = by_frame[frame], by_row[frame]
        center = np.array([row["cx"],row["cy"],row["cz"]]); idx = np.asarray(row["cluster_indices"],dtype=int)
        p=item["dynamic"]; target=p[idx] if len(idx) else np.empty((0,3)); ti=item["dynamic_i"][idx] if len(idx) else np.array([])
        local=(np.abs(p[:,0]-center[0])<8)&(np.abs(p[:,1]-center[1])<12)&(np.abs(p[:,2]-center[2])<5)
        axs[0,col].scatter(p[local,0],p[local,1],s=12,c="0.75",label="other dynamic")
        if len(target): axs[0,col].scatter(target[:,0],target[:,1],s=38,c=ti,cmap="viridis",vmin=10,vmax=50,edgecolor="k",linewidth=.25)
        else: axs[0,col].scatter(center[0],center[1],marker="x",s=60,c="crimson")
        axs[0,col].set(xlim=(center[0]-8,center[0]+8),ylim=(center[1]-12,center[1]+12),xlabel="x",ylabel="y",title=f"f{frame} BEV: n={len(idx)}")
        axs[1,col].scatter(p[local,0],p[local,2],s=12,c="0.75")
        if len(target): axs[1,col].scatter(target[:,0],target[:,2],s=38,c=ti,cmap="viridis",vmin=10,vmax=50,edgecolor="k",linewidth=.25)
        else: axs[1,col].scatter(center[0],center[2],marker="x",s=60,c="crimson")
        axs[1,col].set(xlim=(center[0]-8,center[0]+8),ylim=(center[2]-5,center[2]+5),xlabel="x",ylabel="z",title=f"f{frame} side")
        for ax in axs[:,col]: ax.grid(alpha=.2)
    fig.suptitle("Actual baseline-associated UAV points; gray = nearby dynamic candidates, x = predicted-only gap")
    fig.savefig(path,bbox_inches="tight"); plt.close(fig)


def plot_microzoom(path, frames, rows, reps):
    by_frame={x["frame"]:x for x in frames}; by_row={x["frame"]:x for x in rows}
    fig,axs=plt.subplots(2,len(reps),figsize=(3.7*len(reps),7),dpi=240,constrained_layout=True)
    for col,frame in enumerate(reps):
        item,row=by_frame[frame],by_row[frame]; idx=np.asarray(row["cluster_indices"],dtype=int)
        p=item["dynamic"][idx]; ii=item["dynamic_i"][idx]; vv=item["dynamic_v"][idx]
        center=np.mean(p,axis=0); phat=center/np.linalg.norm(center)
        transverse=np.array([-phat[1],phat[0],0.]); transverse/=np.linalg.norm(transverse)
        off=p-center; roff=off@phat; toff=off@transverse; zoff=off[:,2]
        axs[0,col].scatter(roff,toff,c=ii,s=55,cmap="viridis",vmin=10,vmax=50,edgecolor="k",linewidth=.3)
        axs[0,col].set(xlim=(-.5,.5),ylim=(-.5,.5),xlabel="radial offset (m)",ylabel="horizontal transverse offset (m)",title=f"f{frame}: n={len(p)}, intensity")
        axs[1,col].scatter(toff,zoff,c=vv,s=55,cmap="coolwarm",vmin=-1,vmax=1,edgecolor="k",linewidth=.3)
        axs[1,col].set(xlim=(-.5,.5),ylim=(-.3,.3),xlabel="horizontal transverse offset (m)",ylabel="z offset (m)",title=f"f{frame}: radial velocity")
        for ax in axs[:,col]: ax.axhline(0,color="0.8",lw=.7); ax.axvline(0,color="0.8",lw=.7); ax.grid(alpha=.18); ax.set_aspect("equal",adjustable="box")
    fig.suptitle("Sub-meter UAV point microstructure after the baseline dynamic filter")
    fig.savefig(path,bbox_inches="tight"); plt.close(fig)


def main():
    root=Path(__file__).resolve().parents[1]
    data=root/"data/81-pm-cross-x10_2025-11-19-15-57-13_filtered_data"
    out=root/"results/data_archaeology/cross/baseline_reverse"; out.mkdir(parents=True,exist_ok=True)
    seq=bench.load_point_cloud_sequence(str(data))
    voxel=bench.OnlineDynamicFilter(voxel_size=1.0,x_range=(0,500),y_range=(-150,150),z_range=(-5,50),prob_threshold=.05,init_frames=8)
    tracker=bench.BaselineMultiTracker(min_points=1,search_radius=15.,radial_threshold=5.,max_association_distance=30.,dbscan_eps=2.)
    removals=[]; current_frame=[None]
    original_should_remove=bench.BaselineSingleTarget.should_remove
    def logging_should_remove(self):
        decision=original_should_remove(self)
        if decision[0]: removals.append({"frame":current_frame[0],"track_id":self.target_id,"reason":decision[1]})
        return decision
    bench.BaselineSingleTarget.should_remove=logging_should_remove
    frames=[]
    for frame,(p,velocity,intensity,ts) in enumerate(zip(seq["points"],seq["velocities"],seq["intensities"],seq["timestamps"])):
        hmask=p[:,2]>10.; hp,hv,hi=p[hmask],velocity[hmask],intensity[hmask]
        dp,_,dv=voxel.process_frame(hp,None,hv)
        if voxel.frame_count<=voxel.init_frames: continue
        if len(dp):
            d, nearest=cKDTree(hp).query(dp,k=1)
            if np.max(d)>1e-5: raise RuntimeError("Dynamic point could not be mapped back to input PCD")
            di=hi[nearest]
            current_frame[0]=frame; tracker.track_frame(dp,dv)
        else:
            di=np.array([])
        states={t.target_id:{"center":tuple(float(x) for x in t.center),"center_3d":tuple(float(x) for x in t.center_3d),
                "n_points_state":int(t.num_points),"mean_vr_state":float(t.radial_velocity),"predicted":bool(t.is_predicted),
                "lost_count":int(t.lost_count),"static_count":int(t.static_count)} for t in tracker.trackers if t.is_active}
        frames.append({"frame":frame,"timestamp":float(ts),"height":hp,"dynamic":dp,"dynamic_i":di,"dynamic_v":dv,
                       "clusters":cluster_indices(dp),"states":states})
    bench.BaselineSingleTarget.should_remove=original_should_remove
    final_ids=[t.target_id for t in tracker.trackers if t.is_active]
    if len(final_ids)!=1: raise RuntimeError(f"Expected one final baseline track, found {final_ids}")
    target_id=final_ids[0]

    pseudo={int(r["frame"]):r for r in csv.DictReader((root/"results/data_archaeology/cross/uav_pseudo_labels.csv").open())}
    rows=[]; target_intensity=[]; target_vr=[]; other_intensity=[]; other_vr=[]; center_list=[]; time_list=[]
    other_cluster_sizes=[]; other_cluster_diameters=[]; target_size_ranks=[]
    for item in frames:
        frame=item["frame"]; state=item["states"].get(target_id)
        if state is None: continue
        idx=find_associated_cluster(item,state)
        center=np.asarray(state["center_3d"],dtype=float)
        if len(idx):
            p,ii,vv,radial_off,tangent_off=point_rows_for_cluster(item,idx,center)
            target_intensity.extend(ii); target_vr.extend(vv)
            mask=np.ones(len(item["dynamic"]),dtype=bool); mask[idx]=False
            other_intensity.extend(item["dynamic_i"][mask]); other_vr.extend(item["dynamic_v"][mask])
            diam=float(np.max(distance.pdist(p))) if len(p)>1 else 0.
            spans=np.ptp(p,axis=0) if len(p)>1 else np.zeros(3)
            iq=np.quantile(ii,[.25,.5,.75]); vq=np.quantile(vv,[.25,.5,.75])
            tc=np.mean(p[:,:2],axis=0)
            other_clusters=[q for q in item["clusters"] if not np.array_equal(q,idx)]
            other_centers=[np.mean(item["dynamic"][q,:2],axis=0) for q in other_clusters]
            for q in other_clusters:
                other_cluster_sizes.append(len(q))
                other_cluster_diameters.append(float(np.max(distance.pdist(item["dynamic"][q]))) if len(q)>1 else 0.)
            rank=1+sum(len(q)>len(idx) for q in other_clusters); target_size_ranks.append(rank)
            near=float(min(np.linalg.norm(np.asarray(q)-tc) for q in other_centers)) if other_centers else float("nan")
        else:
            diam=0.; spans=np.zeros(3); iq=np.array([np.nan]*3); vq=np.array([np.nan]*3); radial_off=tangent_off=np.array([]); near=float("nan"); rank=0
        center_list.append(center); time_list.append(item["timestamp"])
        pcenter=np.array([float(pseudo[frame][k]) for k in ("cx","cy","cz")])
        rows.append({"frame":frame,"timestamp":item["timestamp"],"track_id":target_id,"predicted":state["predicted"],
            "cx":center[0],"cy":center[1],"cz":center[2],"range":float(np.linalg.norm(center)),
            "n_points":int(len(idx)),"dynamic_points":int(len(item["dynamic"])),"target_share":float(len(idx)/len(item["dynamic"])) if len(item["dynamic"]) else 0.,
            "diameter":diam,"span_x":float(spans[0]),"span_y":float(spans[1]),"span_z":float(spans[2]),
            "radial_thickness":float(np.ptp(radial_off)) if len(radial_off)>1 else 0.,
            "tangential_radius_max":float(np.max(tangent_off)) if len(tangent_off) else 0.,
            "intensity_q25":float(iq[0]),"intensity_median":float(iq[1]),"intensity_q75":float(iq[2]),
            "vr_q25":float(vq[0]),"vr_median":float(vq[1]),"vr_q75":float(vq[2]),
            "n_dynamic_clusters":len(item["clusters"]),"cluster_size_rank":rank,
            "nearest_other_cluster":near,"pseudo_center_distance":float(np.linalg.norm(center-pcenter)),
            "static_count":state["static_count"],"lost_count":state["lost_count"],"cluster_indices":idx.tolist()})

    centers=np.asarray(center_list); times=np.asarray(time_list); ranges=np.linalg.norm(centers,axis=1)
    steps=np.r_[np.nan,np.linalg.norm(np.diff(centers[:,:2],axis=0),axis=1)]
    geometric=np.gradient(ranges,times)
    for row,step,dr in zip(rows,steps,geometric): row["step_xy"]=float(step); row["geometric_dr"]=float(dr)

    fieldnames=[k for k in rows[0] if k!="cluster_indices"]
    with (out/"baseline_uav_frames.csv").open("w",newline="",encoding="utf-8") as fh:
        w=csv.DictWriter(fh,fieldnames=fieldnames); w.writeheader(); w.writerows([{k:r[k] for k in fieldnames} for r in rows])
    observed=[r for r in rows if not r["predicted"]]
    summary={
        "status":"reverse analysis of benchmark_fmt.py baseline; target identity cross-checked against trajectory-guided pseudo label",
        "final_surviving_track_id":target_id,"analyzed_frames":[rows[0]["frame"],rows[-1]["frame"]],
        "observed_frames":len(observed),"predicted_only_frames":[r["frame"] for r in rows if r["predicted"]],
        "initial_tracks":len(frames[0]["states"]),"removal_events":removals,
        "point_count":qstats([r["n_points"] for r in observed]),"target_fraction_of_dynamic":qstats([r["target_share"] for r in observed]),
        "cluster_diameter_m":qstats([r["diameter"] for r in observed]),"span_z_m":qstats([r["span_z"] for r in observed]),
        "radial_thickness_m":qstats([r["radial_thickness"] for r in observed]),"tangential_radius_max_m":qstats([r["tangential_radius_max"] for r in observed]),
        "uav_intensity":qstats(target_intensity),"other_dynamic_intensity":qstats(other_intensity),
        "uav_radial_velocity":qstats(target_vr),"other_dynamic_radial_velocity":qstats(other_vr),
        "within_frame_intensity_iqr":qstats([r["intensity_q75"]-r["intensity_q25"] for r in observed]),
        "within_frame_radial_velocity_iqr":qstats([r["vr_q75"]-r["vr_q25"] for r in observed]),
        "other_dynamic_cluster_size":qstats(other_cluster_sizes),"other_dynamic_cluster_diameter_m":qstats(other_cluster_diameters),
        "fraction_frames_target_is_largest_cluster":float(np.mean(np.asarray(target_size_ranks)==1)),
        "fraction_uav_points_abs_vr_le_1":float(np.mean(np.abs(target_vr)<=1)),
        "fraction_other_dynamic_points_abs_vr_le_1":float(np.mean(np.abs(other_vr)<=1)),
        "fraction_uav_points_intensity_gt_20":float(np.mean(np.asarray(target_intensity)>20)),
        "fraction_other_dynamic_points_intensity_gt_20":float(np.mean(np.asarray(other_intensity)>20)),
        "step_xy_m":qstats([r["step_xy"] for r in rows if np.isfinite(r["step_xy"])]),
        "range_m":qstats([r["range"] for r in rows]),"nearest_other_cluster_m":qstats([r["nearest_other_cluster"] for r in observed if np.isfinite(r["nearest_other_cluster"])]),
        "pseudo_center_distance_m":qstats([r["pseudo_center_distance"] for r in observed]),
        "velocity_correlation_negative_measured_vs_geometric":float(np.corrcoef([-r["vr_median"] for r in observed],[r["geometric_dr"] for r in observed])[0,1]),
        "important_code_facts":["Input PCD is already intensity-filtered; baseline association does not use intensity.",
            "DBSCAN min_points=1 creates tracks for singleton clusters.","The used-cluster set is populated but not used to prevent duplicate association.",
            "Predicted-only frames retain the previous num_points/radial_velocity state."]}
    (out/"baseline_reverse_summary.json").write_text(json.dumps(summary,indent=2,allow_nan=True),encoding="utf-8")
    plot_success(out/"baseline_success_mechanism.png",frames,target_id,removals)
    plot_micro(out/"baseline_uav_microstructure.png",rows,frames)
    plot_evolution(out/"baseline_uav_point_evolution.png",frames,rows,[8,17,27,35,41,57])
    plot_microzoom(out/"baseline_uav_point_microzoom.png",frames,rows,[8,17,27,35,41,57])
    print(json.dumps(summary,indent=2,allow_nan=True))


if __name__=="__main__": main()
