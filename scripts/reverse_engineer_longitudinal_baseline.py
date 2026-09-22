#!/usr/bin/env python3
"""Reverse-analyze the original baseline on the longitudinal 81-82 pm sequence."""

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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_fmt as bench
from reverse_engineer_baseline_uav import qstats, cluster_indices, find_associated_cluster


def run_sequence(root):
    data=root/"data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data"
    seq=bench.load_point_cloud_sequence(str(data))
    voxel=bench.OnlineDynamicFilter(voxel_size=1.,x_range=(0,500),y_range=(-150,150),z_range=(-5,50),prob_threshold=.05,init_frames=8)
    tracker=bench.BaselineMultiTracker(min_points=1,search_radius=15.,radial_threshold=5.,max_association_distance=30.,dbscan_eps=2.)
    removals=[]; current=[None]
    original=bench.BaselineSingleTarget.should_remove
    def logged(self):
        decision=original(self)
        if decision[0]: removals.append({"frame":current[0],"track_id":self.target_id,"reason":decision[1]})
        return decision
    bench.BaselineSingleTarget.should_remove=logged
    frames=[]; observed_counts={}
    for frame,(p,v,i,ts) in enumerate(zip(seq["points"],seq["velocities"],seq["intensities"],seq["timestamps"])):
        hmask=p[:,2]>10.; hp,hv,hi=p[hmask],v[hmask],i[hmask]
        dp,_,dv=voxel.process_frame(hp,None,hv)
        if voxel.frame_count<=8: continue
        if len(dp):
            d,nearest=cKDTree(hp).query(dp,k=1)
            if np.max(d)>1e-5: raise RuntimeError("Could not map dynamic point to PCD")
            di=hi[nearest]; current[0]=frame; tracker.track_frame(dp,dv)
        else: di=np.array([])
        states={}
        for t in tracker.trackers:
            if not t.is_active: continue
            states[t.target_id]={"center":tuple(float(x) for x in t.center),"center_3d":tuple(float(x) for x in t.center_3d),
                "predicted":bool(t.is_predicted),"lost_count":int(t.lost_count),"static_count":int(t.static_count),
                "n_points_state":int(t.num_points),"mean_vr_state":float(t.radial_velocity)}
            if not t.is_predicted: observed_counts[t.target_id]=observed_counts.get(t.target_id,0)+1
        frames.append({"frame":frame,"timestamp":float(ts),"height":hp,"dynamic":dp,"dynamic_i":di,"dynamic_v":dv,
                       "clusters":cluster_indices(dp),"states":states})
    bench.BaselineSingleTarget.should_remove=original
    target_id=max(observed_counts,key=observed_counts.get)
    return frames,removals,target_id,tracker


def extract_rows(frames,target_id):
    rows=[]; target_i=[];target_v=[];other_i=[];other_v=[];other_sizes=[];other_diams=[];ranks=[]
    for item in frames:
        state=item["states"].get(target_id)
        if state is None: continue
        idx=find_associated_cluster(item,state); p=item["dynamic"][idx]; ii=item["dynamic_i"][idx]; vv=item["dynamic_v"][idx]
        center=np.mean(p,axis=0); target_i.extend(ii);target_v.extend(vv)
        mask=np.ones(len(item["dynamic"]),dtype=bool);mask[idx]=False;other_i.extend(item["dynamic_i"][mask]);other_v.extend(item["dynamic_v"][mask])
        others=[q for q in item["clusters"] if not np.array_equal(q,idx)]
        for q in others:
            other_sizes.append(len(q));other_diams.append(float(np.max(distance.pdist(item["dynamic"][q]))) if len(q)>1 else 0.)
        rank=1+sum(len(q)>len(idx) for q in others);ranks.append(rank)
        centers=[np.mean(item["dynamic"][q,:2],axis=0) for q in others]
        near=float(min(np.linalg.norm(c-center[:2]) for c in centers)) if centers else float("nan")
        diam=float(np.max(distance.pdist(p))) if len(p)>1 else 0.;span=np.ptp(p,axis=0) if len(p)>1 else np.zeros(3)
        phat=center/np.linalg.norm(center);off=p-center;roff=off@phat;tang=np.linalg.norm(off-roff[:,None]*phat,axis=1)
        iq=np.quantile(ii,[.25,.5,.75]);vq=np.quantile(vv,[.25,.5,.75])
        duplicate_ids=[]
        for tid,s in item["states"].items():
            if tid==target_id or s["predicted"]: continue
            if np.linalg.norm(np.asarray(s["center"])-center[:2])<.25: duplicate_ids.append(tid)
        rows.append({"frame":item["frame"],"timestamp":item["timestamp"],"track_id":target_id,
            "cx":float(center[0]),"cy":float(center[1]),"cz":float(center[2]),"range":float(np.linalg.norm(center)),
            "n_points":len(idx),"dynamic_points":len(item["dynamic"]),"target_share":len(idx)/len(item["dynamic"]),
            "diameter":diam,"span_x":float(span[0]),"span_y":float(span[1]),"span_z":float(span[2]),
            "radial_thickness":float(np.ptp(roff)) if len(roff)>1 else 0.,"tangential_radius_max":float(np.max(tang)),
            "intensity_q25":float(iq[0]),"intensity_median":float(iq[1]),"intensity_q75":float(iq[2]),
            "vr_q25":float(vq[0]),"vr_median":float(vq[1]),"vr_q75":float(vq[2]),
            "n_dynamic_clusters":len(item["clusters"]),"cluster_size_rank":rank,"nearest_other_cluster":near,
            "duplicate_track_ids":";".join(map(str,duplicate_ids)),"cluster_indices":idx.tolist()})
    centers=np.array([[r["cx"],r["cy"],r["cz"]] for r in rows]);times=np.array([r["timestamp"] for r in rows]);ranges=np.linalg.norm(centers,axis=1)
    steps=np.r_[np.nan,np.linalg.norm(np.diff(centers[:,:2],axis=0),axis=1)];dr=np.gradient(ranges,times)
    for r,s,g in zip(rows,steps,dr):r["step_xy"]=float(s);r["geometric_dr"]=float(g)
    aux={"target_i":np.asarray(target_i),"target_v":np.asarray(target_v),"other_i":np.asarray(other_i),"other_v":np.asarray(other_v),
         "other_sizes":other_sizes,"other_diams":other_diams,"ranks":ranks}
    return rows,aux


def plot_success(path,frames,target_id,removals):
    fig,axs=plt.subplots(2,2,figsize=(16,11),dpi=200,constrained_layout=True);hist={}
    for item in frames:
        for tid,s in item["states"].items():hist.setdefault(tid,[]).append((item["frame"],*s["center"],s["predicted"]))
    for tid,h in hist.items():
        h=np.asarray(h,float);c="limegreen" if tid==target_id else "0.6";lw=3 if tid==target_id else 1
        axs[0,0].plot(h[:,1],h[:,2],color=c,lw=lw,alpha=1 if tid==target_id else .6,label=f"ID {tid}" if tid in (target_id,1) else None)
        if tid!=target_id:axs[0,0].scatter(h[-1,1],h[-1,2],marker="x",color="crimson",s=28)
    axs[0,0].set(xlabel="x (m)",ylabel="y (m)",title="Track histories; green = continuous longitudinal target");axs[0,0].legend()
    f=np.array([x["frame"] for x in frames]);active=np.array([len(x["states"]) for x in frames]);dyn=np.array([len(x["dynamic"]) for x in frames])
    axs[0,1].step(f,active,where="mid",label="active baseline tracks");ax2=axs[0,1].twinx();ax2.plot(f,dyn,color="tab:orange",label="dynamic points")
    axs[0,1].set(xlabel="frame",ylabel="active tracks",title="Six initial tracks fall to two IDs");ax2.set_ylabel("dynamic points")
    lines=axs[0,1].lines+ax2.lines;axs[0,1].legend(lines,[x.get_label() for x in lines])
    vals={"lost":[],"static":[]}
    for x in removals:vals.setdefault(x["reason"],[]).append(x["frame"])
    for reason,v in vals.items():
        if v:axs[1,0].hist(v,bins=np.arange(7.5,28.5,1),alpha=.65,label=reason)
    axs[1,0].set(xlabel="frame",ylabel="tracks removed",title="Four false tracks expire through lost_count");axs[1,0].legend()
    dup=[]
    for item in frames:
        ts=item["states"].get(target_id);n=0
        if ts and not ts["predicted"]:
            for tid,s in item["states"].items():
                if tid!=target_id and not s["predicted"] and np.linalg.norm(np.array(s["center"])-np.array(ts["center"]))<.25:n+=1
        dup.append(n)
    axs[1,1].bar(f,dup,color="crimson");axs[1,1].set(xlabel="frame",ylabel="extra IDs on target cluster",title="ID 1 duplicates the UAV from frame 17")
    for ax in axs.ravel():ax.grid(alpha=.2)
    fig.suptitle("Longitudinal sequence: baseline success and duplicate-association caveat")
    fig.savefig(path,bbox_inches="tight");plt.close(fig)


def plot_micro(path,rows):
    f=np.array([r["frame"] for r in rows]);fig,axs=plt.subplots(2,3,figsize=(18,10),dpi=200,constrained_layout=True)
    axs[0,0].bar(f,[r["n_points"] for r in rows],label="UAV cluster");axs[0,0].plot(f,[r["dynamic_points"] for r in rows],color="tab:orange",label="all dynamic");axs[0,0].legend();axs[0,0].set(xlabel="frame",ylabel="points",title="Target support inside dynamic candidates")
    axs[0,1].plot(f,[r["range"] for r in rows],".-");axs[0,1].set(xlabel="frame",ylabel="range (m)",title="Longitudinal approach: range decreases continuously")
    axs[0,2].plot(f,[r["diameter"] for r in rows],".-",label="3D diameter");axs[0,2].plot(f,[r["span_z"] for r in rows],".-",label="z span");axs[0,2].legend();axs[0,2].set(xlabel="frame",ylabel="m",title="Within-frame cluster extent")
    axs[1,0].fill_between(f,[r["intensity_q25"] for r in rows],[r["intensity_q75"] for r in rows],alpha=.25);axs[1,0].plot(f,[r["intensity_median"] for r in rows],".-");axs[1,0].set(xlabel="frame",ylabel="intensity",title="Intensity median and IQR")
    axs[1,1].fill_between(f,[r["vr_q25"] for r in rows],[r["vr_q75"] for r in rows],alpha=.25);axs[1,1].plot(f,[r["vr_median"] for r in rows],".-",label="measured $v_r$");axs[1,1].plot(f,[-r["geometric_dr"] for r in rows],label="− geometric dr/dt");axs[1,1].legend();axs[1,1].set(xlabel="frame",ylabel="m/s",title="Radial velocity is strong in longitudinal flight")
    axs[1,2].plot(f,[r["nearest_other_cluster"] for r in rows],".-");axs[1,2].set(xlabel="frame",ylabel="m",title="Nearest other dynamic cluster")
    for ax in axs.ravel():ax.grid(alpha=.2)
    fig.suptitle("Longitudinal UAV point properties")
    fig.savefig(path,bbox_inches="tight");plt.close(fig)


def plot_microzoom(path,frames,rows,reps):
    fm={x["frame"]:x for x in frames};rm={x["frame"]:x for x in rows};fig,axs=plt.subplots(2,len(reps),figsize=(3.7*len(reps),7),dpi=240,constrained_layout=True)
    for col,frame in enumerate(reps):
        item,row=fm[frame],rm[frame];idx=np.asarray(row["cluster_indices"],int);p=item["dynamic"][idx];ii=item["dynamic_i"][idx];vv=item["dynamic_v"][idx]
        c=np.mean(p,axis=0);ph=c/np.linalg.norm(c);tr=np.array([-ph[1],ph[0],0.]);tr/=np.linalg.norm(tr);off=p-c;ro=off@ph;to=off@tr;zo=off[:,2]
        axs[0,col].scatter(ro,to,c=ii,s=55,cmap="viridis",vmin=10,vmax=50,edgecolor="k",linewidth=.3);axs[0,col].set(xlim=(-.6,.6),ylim=(-.6,.6),xlabel="radial offset",ylabel="transverse offset",title=f"f{frame}: n={len(p)}, intensity")
        axs[1,col].scatter(to,zo,c=vv,s=55,cmap="coolwarm",vmin=4,vmax=11,edgecolor="k",linewidth=.3);axs[1,col].set(xlim=(-.6,.6),ylim=(-.35,.35),xlabel="transverse offset",ylabel="z offset",title=f"f{frame}: $v_r$")
        for ax in axs[:,col]:ax.axhline(0,color=".8",lw=.7);ax.axvline(0,color=".8",lw=.7);ax.grid(alpha=.18);ax.set_aspect("equal",adjustable="box")
    fig.suptitle("Longitudinal UAV sub-meter point microstructure")
    fig.savefig(path,bbox_inches="tight");plt.close(fig)


def plot_comparison(path,long_rows,cross_rows):
    fig,axs=plt.subplots(2,2,figsize=(15,11),dpi=200,constrained_layout=True)
    for rows,label,color in [(cross_rows,"cross","tab:blue"),(long_rows,"longitudinal","tab:orange")]:
        c=np.array([[float(r["cx"]),float(r["cy"])] for r in rows]);axs[0,0].plot(c[:,0],c[:,1],".-",label=label,color=color)
        u=np.linspace(0,1,len(rows));axs[0,1].plot(u,[float(r["range"]) for r in rows],label=label,color=color)
        axs[1,0].plot(u,[float(r["vr_median"]) for r in rows],label=label,color=color)
    axs[0,0].set(xlabel="x (m)",ylabel="y (m)",title="Geometry of the two flight modes");axs[0,0].legend()
    axs[0,1].set(xlabel="normalized analyzed time",ylabel="range (m)",title="Range evolution");axs[0,1].legend()
    axs[1,0].set(xlabel="normalized analyzed time",ylabel="median radial velocity (m/s)",title="Measured radial velocity");axs[1,0].legend()
    axs[1,1].boxplot([[float(r["n_points"]) for r in cross_rows],[float(r["n_points"]) for r in long_rows]],tick_labels=["cross","longitudinal"])
    axs[1,1].set(ylabel="associated points/frame",title="Point-count distribution")
    for ax in axs.ravel():ax.grid(alpha=.2)
    fig.suptitle("Cross versus longitudinal UAV evidence under the same baseline")
    fig.savefig(path,bbox_inches="tight");plt.close(fig)


def main():
    root=Path(__file__).resolve().parents[1];out=root/"results/data_archaeology/longitudinal/baseline_reverse";out.mkdir(parents=True,exist_ok=True)
    frames,removals,target_id,tracker=run_sequence(root);rows,aux=extract_rows(frames,target_id)
    fields=[k for k in rows[0] if k!="cluster_indices"]
    with (out/"longitudinal_uav_frames.csv").open("w",newline="",encoding="utf-8") as fh:
        w=csv.DictWriter(fh,fieldnames=fields);w.writeheader();w.writerows([{k:r[k] for k in fields} for r in rows])
    duplicate_frames=[r["frame"] for r in rows if r["duplicate_track_ids"]]
    summary={"status":"trajectory-guided pseudo ground truth; no external synchronized ground truth","target_track_id":target_id,
        "analyzed_frames":[rows[0]["frame"],rows[-1]["frame"]],"observed_frames":len(rows),"initial_tracks":len(frames[0]["states"]),
        "final_active_track_ids":[t.target_id for t in tracker.trackers],"removal_events":removals,"duplicate_association_frames":duplicate_frames,
        "point_count":qstats([r["n_points"] for r in rows]),"target_fraction_of_dynamic":qstats([r["target_share"] for r in rows]),
        "cluster_diameter_m":qstats([r["diameter"] for r in rows]),"span_z_m":qstats([r["span_z"] for r in rows]),
        "uav_intensity":qstats(aux["target_i"]),"other_dynamic_intensity":qstats(aux["other_i"]),
        "uav_radial_velocity":qstats(aux["target_v"]),"other_dynamic_radial_velocity":qstats(aux["other_v"]),
        "within_frame_vr_iqr":qstats([r["vr_q75"]-r["vr_q25"] for r in rows]),"range_m":qstats([r["range"] for r in rows]),
        "step_xy_m":qstats([r["step_xy"] for r in rows if np.isfinite(r["step_xy"])]),"nearest_other_cluster_m":qstats([r["nearest_other_cluster"] for r in rows if np.isfinite(r["nearest_other_cluster"])]),
        "other_cluster_size":qstats(aux["other_sizes"]),"other_cluster_diameter_m":qstats(aux["other_diams"]),
        "fraction_frames_target_is_largest_cluster":float(np.mean(np.asarray(aux["ranks"])==1)),
        "fraction_uav_points_intensity_gt_20":float(np.mean(aux["target_i"]>20)),"fraction_other_points_intensity_gt_20":float(np.mean(aux["other_i"]>20)),
        "velocity_correlation_negative_measured_vs_geometric":float(np.corrcoef([-r["vr_median"] for r in rows],[r["geometric_dr"] for r in rows])[0,1]),
        "critical_caveat":"Final IDs 1 and 3 are not two targets; ID 1 associates to the same target cluster as ID 3 from frame 17 because used cluster indices are not excluded."}
    (out/"longitudinal_reverse_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    plot_success(out/"longitudinal_success_mechanism.png",frames,target_id,removals);plot_micro(out/"longitudinal_uav_microstructure.png",rows);plot_microzoom(out/"longitudinal_uav_point_microzoom.png",frames,rows,[8,10,14,17,22,27])
    cross=list(csv.DictReader((root/"results/data_archaeology/cross/baseline_reverse/baseline_uav_frames.csv").open()))
    cross=[r for r in cross if r["predicted"]=="False"]
    plot_comparison(out/"cross_vs_longitudinal.png",rows,cross)
    print(json.dumps(summary,indent=2))


if __name__=="__main__":main()
