#!/usr/bin/env python3
"""Create transparent manual/trajectory-guided pseudo labels and survival plots."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

sys.path.insert(0, str(Path(__file__).resolve().parent))
from data_archaeology import StageFilter, iter_bag_messages, transform_xyz


def centers_from_cache(cache, roi, eps=.8):
    observed = np.full((len(cache), 3), np.nan)
    support = np.zeros(len(cache), dtype=int)
    for k, item in enumerate(cache):
        p = item["height"]
        mask = ((p[:, 0] >= roi[0]) & (p[:, 0] <= roi[1]) &
                (p[:, 1] >= roi[2]) & (p[:, 1] <= roi[3]) &
                (p[:, 2] >= roi[4]) & (p[:, 2] <= roi[5]))
        q = p[mask]
        if len(q) < 2:
            continue
        labels = DBSCAN(eps=eps, min_samples=2).fit_predict(q)
        clusters = [(np.sum(labels == x), q[labels == x]) for x in set(labels) if x != -1]
        if clusters:
            n, best = max(clusters, key=lambda x: x[0])
            observed[k] = np.mean(best, axis=0)
            support[k] = n
    known = np.flatnonzero(np.isfinite(observed[:, 0]))
    centers = observed.copy()
    for dim in range(3):
        centers[:, dim] = np.interp(np.arange(len(cache)), known, observed[known, dim])
    return centers, observed, support


def dist_mask(points, center, radius=1.0):
    return np.linalg.norm(points - center, axis=1) <= radius


def cache_survival(cache, centers, radius=1.0):
    rows = []
    target_i, target_v, target_r = [], [], []
    clutter_i, clutter_v, clutter_r = [], [], []
    for item, center in zip(cache, centers):
        hp, hi, hv = item["height"], item["height_i"], item["height_v"]
        target = dist_mask(hp, center, radius)
        d = np.linalg.norm(hp - center, axis=1)
        clutter = (d >= 3.0) & (d <= 15.0) & (np.abs(hp[:, 2] - center[2]) <= 3.0)
        bp, np_ = item["background"], item["neighborhood"]
        btarget, ntarget = dist_mask(bp, center, radius), dist_mask(np_, center, radius)
        rows.append({
            "frame": int(item["frame"]), "cx": float(center[0]), "cy": float(center[1]),
            "cz": float(center[2]), "height": int(target.sum()),
            "background": int(btarget.sum()), "neighborhood": int(ntarget.sum()),
            "cluster": int(ntarget.sum()),
        })
        if target.any():
            target_i.append(hi[target]); target_v.append(hv[target]); target_r.append(np.linalg.norm(hp[target], axis=1))
        if clutter.any():
            clutter_i.append(hi[clutter]); clutter_v.append(hv[clutter]); clutter_r.append(np.linalg.norm(hp[clutter], axis=1))
    features = {
        "uav_i": np.concatenate(target_i) if target_i else np.array([]),
        "uav_v": np.concatenate(target_v) if target_v else np.array([]),
        "uav_r": np.concatenate(target_r) if target_r else np.array([]),
        "clutter_i": np.concatenate(clutter_i) if clutter_i else np.array([]),
        "clutter_v": np.concatenate(clutter_v) if clutter_v else np.array([]),
        "clutter_r": np.concatenate(clutter_r) if clutter_r else np.array([]),
    }
    return rows, features


def raw_counts(spec, centers, rows, radius=1.0):
    bag, conn, start, limit = spec
    filt = StageFilter()
    target_raw_i, target_raw_v = [], []
    for source_frame, (values, meta) in enumerate(iter_bag_messages(bag, conn, start + limit)):
        if source_frame < start:
            continue
        frame = source_frame - start
        points = transform_xyz(values) if meta["point_step"] == 80 else np.column_stack(
            [values["x"], values["y"], values["z"]])
        intensity, velocity = values["intensity"], values["velocity"]
        finite = np.isfinite(points).all(axis=1) & np.isfinite(intensity) & np.isfinite(velocity)
        local = finite & dist_mask(points, centers[frame], radius)
        intensity_ok = local & (intensity >= 10) & (intensity <= 250)
        height_ok = intensity_ok & (points[:, 2] > 10)
        rows[frame].update({
            "source_frame": source_frame, "header_time": meta["header_time"], "bag_time": meta["bag_time"],
            "raw": int(local.sum()), "intensity": int(intensity_ok.sum()),
            "height_direct": int(height_ok.sum()),
        })
        if local.any():
            target_raw_i.append(intensity[local]); target_raw_v.append(velocity[local])
        print("raw-label", frame, rows[frame]["raw"], rows[frame]["intensity"], rows[frame]["height"], flush=True)
    return {
        "raw_i": np.concatenate(target_raw_i) if target_raw_i else np.array([]),
        "raw_v": np.concatenate(target_raw_v) if target_raw_v else np.array([]),
    }


def point_stats(counts):
    a = np.asarray(counts)
    bins = {
        "0": int(np.sum(a == 0)), "1": int(np.sum(a == 1)), "2": int(np.sum(a == 2)),
        "3-5": int(np.sum((a >= 3) & (a <= 5))),
        "6-10": int(np.sum((a >= 6) & (a <= 10))), ">10": int(np.sum(a > 10)),
    }
    return {"n_frames": len(a), "mean": float(a.mean()), "median": float(np.median(a)),
            "min": int(a.min()), "max": int(a.max()), "bins": bins,
            "bin_fraction": {k: v/len(a) for k, v in bins.items()}}


def plot_track(out, name, rows, features):
    f = np.array([r["frame"] for r in rows]); c = np.array([[r["cx"], r["cy"], r["cz"]] for r in rows])
    n = np.array([r["height"] for r in rows]); rng = np.linalg.norm(c, axis=1)
    dt = np.gradient(f.astype(float)); vel = np.gradient(c, axis=0) / dt[:, None]
    geom_vr = np.sum(vel * c, axis=1) / rng
    fig, axs = plt.subplots(2, 2, figsize=(15, 10), dpi=200, constrained_layout=True)
    sc = axs[0, 0].scatter(c[:, 0], c[:, 1], c=f, s=25+n*3, cmap="turbo")
    axs[0, 0].plot(c[:, 0], c[:, 1], alpha=.4); axs[0, 0].set(xlabel="x (m)", ylabel="y (m)", title="Pseudo UAV center (marker size = point count)")
    fig.colorbar(sc, ax=axs[0, 0], label="frame")
    axs[0, 1].plot(f, c[:, 1], label="y"); axs[0, 1].plot(f, c[:, 2], label="z"); axs[0, 1].set(xlabel="frame", ylabel="position (m)", title="Trajectory coordinates"); axs[0, 1].legend()
    axs[1, 0].bar(f, n); axs[1, 0].set(xlabel="frame", ylabel="pseudo UAV points", title="Intensity+height UAV point count")
    measured = []
    for center, row in zip(c, rows):
        measured.append(row.get("median_vr", np.nan))
    axs[1, 1].plot(f, geom_vr, label="geometric radial velocity")
    axs[1, 1].plot(f, measured, ".-", label="median measured radial velocity")
    axs[1, 1].axhline(0, color="k", lw=.7); axs[1, 1].set(xlabel="frame", ylabel="m/s (assuming 1 s)", title="Radial-velocity sign/scale check"); axs[1, 1].legend()
    for ax in axs.ravel(): ax.grid(alpha=.2)
    fig.suptitle(f"{name}: trajectory-guided pseudo label")
    fig.savefig(out / f"{name}_uav_temporal_track.png", bbox_inches="tight"); plt.close(fig)


def plot_survival(out, name, rows):
    f = np.array([r["frame"] for r in rows])
    keys = ["raw", "intensity", "height", "background", "neighborhood", "cluster"]
    fig, axs = plt.subplots(1, 2, figsize=(16, 6), dpi=200, constrained_layout=True)
    for key in keys: axs[0].plot(f, [r.get(key, 0) for r in rows], label=key)
    axs[0].set(xlabel="frame", ylabel="pseudo UAV points", title="Per-frame survival"); axs[0].legend(ncol=2); axs[0].grid(alpha=.2)
    totals = np.array([sum(r.get(k, 0) for r in rows) for k in keys]); base=max(totals[0],1)
    axs[1].bar(keys, totals/base*100); axs[1].set(ylabel="retained relative to raw (%)", title="Aggregate point survival", ylim=(0,105)); axs[1].tick_params(axis="x", rotation=25); axs[1].grid(axis="y", alpha=.2)
    fig.suptitle(name); fig.savefig(out / f"{name}_uav_survival.png", bbox_inches="tight"); plt.close(fig)


def plot_features(out, name, features):
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.5), dpi=200, constrained_layout=True)
    for ax, u, c, title, bins in [
        (axs[0], features["uav_i"], features["clutter_i"], "Intensity", 40),
        (axs[1], features["uav_v"], features["clutter_v"], "Radial velocity", 60),
        (axs[2], features["uav_r"], features["clutter_r"], "Range", 50)]:
        if len(c): ax.hist(c, bins=bins, density=True, alpha=.5, label=f"nearby clutter n={len(c):,}")
        if len(u): ax.hist(u, bins=bins, density=True, alpha=.7, label=f"pseudo UAV n={len(u):,}")
        ax.set_title(title); ax.legend(); ax.grid(alpha=.2)
    fig.suptitle(f"{name}: pseudo UAV vs nearby same-height clutter")
    fig.savefig(out / f"{name}_uav_vs_clutter.png", bbox_inches="tight"); plt.close(fig)


def main():
    root = Path(__file__).resolve().parents[1]; out = root / "results/data_archaeology"; out.mkdir(parents=True, exist_ok=True)
    configs = {
        "cross": {"roi": (215, 227, -18, 60, 10.5, 14.0), "eps": .8,
                  "bag": (root/"bag/81-82-pm-cross-x10_2025-11-19-15-57-13.bag", 0, 0, 60)},
        "300m": {"roi": (285, 303, -60, 75, 16, 24), "eps": .9,
                 "bag": (root/"bag/300m_v10_ms_2026-03-19-14-11-42/300m_v10_ms_2026-03-19-14-11-42.bag", 0, 7, 50)},
    }
    summaries = {}
    for name, cfg in configs.items():
        cache = np.load(out/name/"stage_cache.npz", allow_pickle=True)["frames"]
        centers, observed, support = centers_from_cache(cache, cfg["roi"], cfg["eps"])
        rows, features = cache_survival(cache, centers)
        raw_features = raw_counts(cfg["bag"], centers, rows)
        for row, item, center in zip(rows, cache, centers):
            mask = dist_mask(item["height"], center, 1.0)
            row["median_vr"] = float(np.median(item["height_v"][mask])) if mask.any() else float("nan")
            row["mean_intensity"] = float(np.mean(item["height_i"][mask])) if mask.any() else float("nan")
            row["range"] = float(np.linalg.norm(center))
            row["center_observed"] = bool(np.isfinite(observed[row["frame"], 0]))
            row["center_support"] = int(support[row["frame"]])
        with (out/name/"uav_pseudo_labels.csv").open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        plot_track(out/name, name, rows, features); plot_survival(out/name, name, rows); plot_features(out/name, name, features)
        summaries[name] = {"point_count_intensity_height": point_stats([r["height"] for r in rows]),
                           "point_count_raw": point_stats([r["raw"] for r in rows]),
                           "stage_totals": {k: sum(r[k] for r in rows) for k in ("raw","intensity","height","background","neighborhood","cluster")},
                           "center_observed_frames": int(np.isfinite(observed[:,0]).sum())}
        np.savez_compressed(out/name/"uav_feature_samples.npz", **features, **raw_features)
    (out/"pseudo_label_summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
