#!/usr/bin/env python3
"""Check the GNSS-consistent 100 m range/height band in the *raw* rich bag."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from data_archaeology import iter_bag_messages, plot_views, transform_xyz


def summary(a):
    a = np.asarray(a)
    if not len(a):
        return {"n": 0}
    return {"n": int(len(a)), "min": float(np.min(a)), "q25": float(np.quantile(a, .25)),
            "median": float(np.median(a)), "q75": float(np.quantile(a, .75)),
            "max": float(np.max(a)), "mean": float(np.mean(a))}


def main():
    root = Path(__file__).resolve().parents[1]
    bag = root / "bag/20260205/100m横飞速度10ms/100m_v10m_s_2026-02-05-15-08-28.bag"
    out = root / "results/data_archaeology/100m"
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    all_band_p, all_band_i, all_band_v, all_low_p, all_low_i, all_low_v = [], [], [], [], [], []
    for frame, (values, meta) in enumerate(iter_bag_messages(bag, 0, 46)):
        p = transform_xyz(values)
        intensity, velocity = values["intensity"], values["velocity"]
        finite = np.isfinite(p).all(axis=1) & np.isfinite(intensity) & np.isfinite(velocity)
        p, intensity, velocity = p[finite], intensity[finite], velocity[finite]
        horizontal_range = np.linalg.norm(p[:, :2], axis=1)
        # Workbook-derived device-to-UAV envelope: about 105--120 m horizontal and
        # 6.9--12.8 m relative height. This deliberately wider band avoids claiming
        # time/yaw synchronization that is not available.
        band = (horizontal_range >= 90) & (horizontal_range <= 130) & (p[:, 2] >= 4) & (p[:, 2] <= 16)
        intensity_ok = band & (intensity >= 10) & (intensity <= 250)
        low_alt = (horizontal_range >= 90) & (horizontal_range <= 130) & (p[:, 2] >= 0) & (p[:, 2] <= 4)
        all_band_p.append(p[band]); all_band_i.append(intensity[band]); all_band_v.append(velocity[band])
        all_low_p.append(p[low_alt]); all_low_i.append(intensity[low_alt]); all_low_v.append(velocity[low_alt])
        rows.append({"frame": frame, "raw_band_n": int(band.sum()),
                     "intensity_band_n": int(intensity_ok.sum()),
                     "low_altitude_n": int(low_alt.sum()),
                     "bag_time": meta["bag_time"], "header_time": meta["header_time"]})
        if frame in (8, 20, 35, 45):
            plot_views(out / f"frame_{frame:03d}_raw_views.png", "100m", frame,
                       p, intensity, velocity, meta)
        print("100m-band", frame, int(band.sum()), int(intensity_ok.sum()), int(low_alt.sum()), flush=True)

    bp = np.vstack(all_band_p) if any(len(x) for x in all_band_p) else np.empty((0, 3))
    bi = np.concatenate(all_band_i) if any(len(x) for x in all_band_i) else np.array([])
    bv = np.concatenate(all_band_v) if any(len(x) for x in all_band_v) else np.array([])
    lp = np.vstack(all_low_p) if any(len(x) for x in all_low_p) else np.empty((0, 3))
    li = np.concatenate(all_low_i) if any(len(x) for x in all_low_i) else np.array([])
    lv = np.concatenate(all_low_v) if any(len(x) for x in all_low_v) else np.array([])

    fig, axs = plt.subplots(2, 3, figsize=(18, 10), dpi=200, constrained_layout=True)
    f = np.arange(len(rows))
    axs[0, 0].plot(f, [r["raw_band_n"] for r in rows], label="raw expected band")
    axs[0, 0].plot(f, [r["intensity_band_n"] for r in rows], label="after intensity [10,250]")
    axs[0, 0].set(xlabel="frame", ylabel="points", title="GNSS-consistent broad band per frame"); axs[0, 0].legend()
    if len(bp):
        s = axs[0, 1].scatter(bp[:, 0], bp[:, 1], c=bi, s=3, cmap="viridis", linewidths=0)
        fig.colorbar(s, ax=axs[0, 1], label="intensity")
        axs[0, 2].scatter(np.linalg.norm(bp[:, :2], axis=1), bp[:, 2], c=bi, s=3,
                          cmap="viridis", linewidths=0)
        axs[1, 0].hist(bi, bins=50); axs[1, 1].hist(bv, bins=50)
    axs[0, 1].set(xlabel="x (m)", ylabel="y (m)", title="All raw points in broad expected band")
    axs[0, 2].set(xlabel="horizontal range (m)", ylabel="z (m)", title="Range–height in broad expected band")
    axs[1, 0].axvline(10, color="r", ls="--"); axs[1, 0].set(xlabel="intensity", ylabel="count", title="Expected-band raw intensity")
    axs[1, 1].axvline(0, color="k", lw=.8); axs[1, 1].set(xlabel="radial velocity", ylabel="count", title="Expected-band radial velocity")
    axs[1, 2].plot(f, [r["low_altitude_n"] for r in rows], color="darkorange")
    axs[1, 2].set(xlabel="frame", ylabel="points", title="90–130 m, low altitude z=0–4 m (comparison)")
    for ax in axs.ravel(): ax.grid(alpha=.2)
    fig.suptitle("100 m: raw-bag check against GNSS-derived distance/height envelope")
    fig.savefig(out / "gnss_expected_band.png", bbox_inches="tight"); plt.close(fig)

    result = {
        "label_status": "GNSS-derived broad search envelope, not synchronized point-level ground truth",
        "gnss_expected_horizontal_range_m": [105, 120],
        "gnss_expected_relative_height_m": [6.9, 12.8],
        "tested_broad_horizontal_range_m": [90, 130],
        "tested_broad_height_m": [4, 16],
        "frames": rows,
        "aggregate": {"expected_band_raw_points": int(len(bp)),
                      "expected_band_after_intensity": int(np.sum((bi >= 10) & (bi <= 250))),
                      "expected_band_intensity": summary(bi),
                      "expected_band_radial_velocity": summary(bv),
                      "low_altitude_points": int(len(lp)),
                      "low_altitude_intensity": summary(li),
                      "low_altitude_radial_velocity": summary(lv)},
    }
    (out / "gnss_expected_band.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
