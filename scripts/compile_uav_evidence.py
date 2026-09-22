#!/usr/bin/env python3
"""Compile pseudo-label evidence and make target-centered diagnostic panels."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import spearmanr


def describe(a):
    a = np.asarray(a, dtype=float)
    if not len(a): return {"n": 0}
    q = np.quantile(a, [.25, .5, .75])
    return {"n": int(len(a)), "mean": float(np.mean(a)), "q25": float(q[0]),
            "median": float(q[1]), "q75": float(q[2]), "min": float(np.min(a)),
            "max": float(np.max(a))}


def load_rows(path):
    rows = list(csv.DictReader(path.open()))
    for r in rows:
        for k in ("frame", "source_frame", "raw", "intensity", "height", "background", "neighborhood", "cluster"):
            r[k] = int(r[k])
        for k in ("cx", "cy", "cz", "header_time", "median_vr", "mean_intensity", "range"):
            r[k] = float(r[k])
    return rows


def range_bins(rows):
    rng = np.array([r["range"] for r in rows]); n = np.array([r["height"] for r in rows])
    edges = np.linspace(rng.min(), rng.max() + 1e-6, 5)
    result = []
    for a, b in zip(edges[:-1], edges[1:]):
        q = n[(rng >= a) & (rng < b)]
        result.append({"range_m": [float(a), float(b)], "frames": int(len(q)),
                       "mean_points": float(q.mean()), "median_points": float(np.median(q)),
                       "min": int(q.min()), "max": int(q.max())})
    rho, p = spearmanr(rng, n)
    return {"bins": result, "spearman_rho": float(rho), "p": float(p),
            "warning": "Within-sequence association only; distance is confounded with flight phase and path revisits."}


def velocity_check(rows):
    c = np.array([[r["cx"], r["cy"], r["cz"]] for r in rows])
    t = np.array([r["header_time"] for r in rows]); t -= t[0]
    measured = np.array([r["median_vr"] for r in rows])
    geom = np.gradient(np.linalg.norm(c, axis=1), t)
    ok = np.isfinite(measured) & np.isfinite(geom)
    return {"correlation_measured_vs_geometric": float(np.corrcoef(measured[ok], geom[ok])[0, 1]),
            "correlation_negative_measured_vs_geometric": float(np.corrcoef(-measured[ok], geom[ok])[0, 1]),
            "median_abs_frame_median_vr": float(np.nanmedian(np.abs(measured))),
            "fraction_frames_abs_median_vr_le_0_25": float(np.nanmean(np.abs(measured) <= .25)),
            "interpretation": "Measured sign is opposite to increasing range; magnitude and changes closely follow geometric radial motion."}


def plot_zoom(path, name, cache, rows, frames):
    fig, axs = plt.subplots(3, len(frames), figsize=(4.8*len(frames), 12), dpi=200,
                            constrained_layout=True)
    for col, frame in enumerate(frames):
        item, row = cache[frame], rows[frame]
        c = np.array([row["cx"], row["cy"], row["cz"]])
        p, iv, ii = item["height"], item["height_v"], item["height_i"]
        local = (np.abs(p[:, 0]-c[0]) <= 8) & (np.abs(p[:, 1]-c[1]) <= 15) & (np.abs(p[:, 2]-c[2]) <= 5)
        q, qv, qi = p[local], iv[local], ii[local]
        ax = axs[0, col]
        sc = ax.scatter(q[:, 0], q[:, 1], c=qv, s=9, cmap="coolwarm", vmin=-5, vmax=5,
                        linewidths=0) if len(q) else None
        ax.add_patch(plt.Circle((c[0], c[1]), 1, fill=False, color="lime", lw=2))
        ax.set(xlim=(c[0]-8,c[0]+8), ylim=(c[1]-15,c[1]+15), xlabel="x", ylabel="y",
               title=f"f{frame}: height BEV, $v_r$")
        ax = axs[1, col]
        if len(q): ax.scatter(q[:, 0], q[:, 2], c=qi, s=9, cmap="viridis", vmin=10, vmax=50, linewidths=0)
        ax.add_patch(plt.Circle((c[0], c[2]), 1, fill=False, color="red", lw=2))
        ax.set(xlim=(c[0]-8,c[0]+8), ylim=(c[2]-5,c[2]+5), xlabel="x", ylabel="z",
               title=f"f{frame}: height side, intensity")
        ax = axs[2, col]
        n = item["neighborhood"]
        nl = (np.abs(n[:,0]-c[0])<=8)&(np.abs(n[:,1]-c[1])<=15)&(np.abs(n[:,2]-c[2])<=5) if len(n) else np.array([],dtype=bool)
        if len(n): ax.scatter(n[nl,0], n[nl,1], c=item["neighborhood_v"][nl], s=14,
                              cmap="coolwarm", vmin=-5, vmax=5, linewidths=0)
        ax.add_patch(plt.Circle((c[0], c[1]), 1, fill=False, color="lime", lw=2))
        ax.set(xlim=(c[0]-8,c[0]+8), ylim=(c[1]-15,c[1]+15), xlabel="x", ylabel="y",
               title=f"f{frame}: final candidates")
        for a in axs[:, col]: a.grid(alpha=.2)
    fig.suptitle(f"{name}: target-centered views; circle = 1 m pseudo-label radius")
    fig.savefig(path, bbox_inches="tight"); plt.close(fig)


def export_point_labels(path, cache, rows):
    fields = ["frame", "source_frame", "header_time", "bag_time", "x", "y", "z",
              "range", "radial_velocity", "intensity", "label_status"]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields); writer.writeheader()
        for item, row in zip(cache, rows):
            p = item["height"]
            center = np.array([row["cx"], row["cy"], row["cz"]])
            mask = np.linalg.norm(p-center, axis=1) <= 1.0
            for xyz, velocity, intensity in zip(p[mask], item["height_v"][mask], item["height_i"][mask]):
                writer.writerow({"frame": row["frame"], "source_frame": row["source_frame"],
                    "header_time": row["header_time"], "bag_time": row.get("bag_time", ""),
                    "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
                    "range": float(np.linalg.norm(xyz)), "radial_velocity": float(velocity),
                    "intensity": float(intensity),
                    "label_status": "trajectory-guided pseudo ground truth"})


def plot_temporal(path, name, rows):
    f = np.array([r["frame"] for r in rows]); c = np.array([[r["cx"],r["cy"],r["cz"]] for r in rows])
    n = np.array([r["height"] for r in rows]); t = np.array([r["header_time"] for r in rows]); t -= t[0]
    rng = np.linalg.norm(c, axis=1); geom = np.gradient(rng, t)
    measured = np.array([r["median_vr"] for r in rows])
    fig, axs = plt.subplots(2, 2, figsize=(15, 10), dpi=200, constrained_layout=True)
    sc = axs[0,0].scatter(c[:,0],c[:,1],c=f,s=25+n*3,cmap="turbo")
    axs[0,0].plot(c[:,0],c[:,1],alpha=.4); axs[0,0].set(xlabel="x (m)",ylabel="y (m)",title="Pseudo UAV center (size = point count)")
    fig.colorbar(sc,ax=axs[0,0],label="frame")
    axs[0,1].plot(f,c[:,1],label="y"); axs[0,1].plot(f,c[:,2],label="z")
    axs[0,1].set(xlabel="frame",ylabel="position (m)",title="Trajectory coordinates"); axs[0,1].legend()
    axs[1,0].bar(f,n); axs[1,0].set(xlabel="frame",ylabel="pseudo UAV points",title="Intensity+height point count")
    axs[1,1].plot(f,geom,label="geometric dr/dt (header-time delta)")
    axs[1,1].plot(f,-measured,".-",label="− measured median $v_r$")
    axs[1,1].axhline(0,color="k",lw=.7); axs[1,1].set(xlabel="frame",ylabel="m/s",title="Velocity sign convention and geometric consistency"); axs[1,1].legend()
    for ax in axs.ravel(): ax.grid(alpha=.2)
    fig.suptitle(f"{name}: trajectory-guided pseudo label")
    fig.savefig(path,bbox_inches="tight"); plt.close(fig)


def main():
    root = Path(__file__).resolve().parents[1]
    out = root / "results/data_archaeology"
    config = {"cross": [12, 28, 39, 55], "300m": [9, 18, 28, 42]}
    result = {}
    for name, reps in config.items():
        seq = out / name
        rows = load_rows(seq / "uav_pseudo_labels.csv")
        feat = np.load(seq / "uav_feature_samples.npz")
        cache = np.load(seq / "stage_cache.npz", allow_pickle=True)["frames"]
        plot_zoom(seq / f"{name}_uav_zoom_sequence.png", name, cache, rows, reps)
        plot_temporal(seq / f"{name}_uav_temporal_track.png", name, rows)
        export_point_labels(seq / "uav_pseudo_points.csv", cache, rows)
        totals = {k: int(sum(r[k] for r in rows)) for k in ("raw","intensity","height","background","neighborhood","cluster")}
        raw = max(totals["raw"], 1)
        result[name] = {
            "label_status": "manual/trajectory-guided pseudo ground truth; not synchronized true ground truth",
            "survival_totals": totals,
            "survival_fraction_of_raw": {k: v/raw for k,v in totals.items()},
            "uav_features": {"intensity": describe(feat["uav_i"]), "radial_velocity": describe(feat["uav_v"]),
                             "range": describe(feat["uav_r"])},
            "nearby_clutter_features": {"intensity": describe(feat["clutter_i"]),
                                        "radial_velocity": describe(feat["clutter_v"]),
                                        "range": describe(feat["clutter_r"])},
            "velocity_geometry": velocity_check(rows),
            "point_count_vs_range": range_bins(rows),
        }
    (out / "analysis_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
