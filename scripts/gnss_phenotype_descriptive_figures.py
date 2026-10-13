#!/usr/bin/env python3
"""Read-only CSV-derived figures/occupancy tables; no fit or method scoring."""
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import gnss_conditioned_uav_phenotype as study

ROOT, OUT = study.ROOT, study.OUT / "run_02"
COLORS = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7"]


def values(rows, key):
    return np.array([r.get(key, np.nan) if r.get(key) is not None else np.nan for r in rows], float)


def save(fig, name):
    for extension in ["pdf", "png"]:
        path = OUT / "figures" / f"{name}.{extension}"
        if path.exists():
            raise FileExistsError(path)
        fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    study.verify(OUT)
    rows = study.cread(OUT / "region_features.csv")
    targets = [r for r in rows if r["region_role"] == "UAV_CONDITIONED"]
    (OUT / "figures").mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 8,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": .18, "pdf.fonttype": 42})
    # All observed samples, not a fitted curve. Break prior-anchor gaps rather
    # than silently joining them into a continuously observed UAV trajectory.
    fig, axes = plt.subplots(5, 2, figsize=(9, 11), constrained_layout=True)
    for i, sid in enumerate(study.SEQUENCES):
        for j, recipe in enumerate(["GNSS_3m", "ANCHOR_3m"]):
            rs = sorted([r for r in targets if r["sequence_id"] == sid and r["alignment_scenario"] == recipe], key=lambda r: r["time_s"])
            ax = axes[i, j]
            time = values(rs, "time_s")
            time -= min(time) if len(time) else 0
            for k, (feature, label, marker) in enumerate([
                    ("raw_points", "raw stored returns", "o"), ("unique_xyz_points", "unique XYZ", "s"),
                    ("occupied_voxels_0.04m", "4 cm cells", "^"), ("occupied_voxels_1m", "1 m cells", "x")]):
                ax.scatter(time, values(rs, feature), s=10, color=COLORS[k], marker=marker, label=label)
            ax.set_title(f"{sid}: {recipe} ({len(rs)} frames)", fontsize=8)
            ax.set_yscale("symlog", linthresh=1)
            ax.set_xlabel("Seconds from first frame of this recipe")
            ax.set_ylabel("Neighborhood support (not physical point GT)")
    axes[0, 1].legend(fontsize=6, loc="upper right")
    fig.suptitle("Oracle-conditioned support: uncertain GNSS regions vs selected closure observations", fontsize=10)
    save(fig, "support_time_and_representation")

    fig, axes = plt.subplots(2, 2, figsize=(8, 6), constrained_layout=True)
    for i, sid in enumerate(study.SEQUENCES):
        rs = [r for r in targets if r["sequence_id"] == sid and r["alignment_scenario"] == "ANCHOR_3m"]
        for ax, key in zip(axes.flat, ["raw_points", "unique_xyz_points", "diameter_m", "unique_nn_m_median"]):
            ax.scatter(values(rs, "region_reference_range_m"), values(rs, key), color=COLORS[i], s=16, alpha=.7, label=sid)
            ax.set_xlabel("Saved-anchor sensor range (m; provisional)")
            ax.set_ylabel(key.replace("_", " "))
        axes[0, 0].set_yscale("symlog", linthresh=1)
    axes[0, 1].legend(fontsize=6)
    fig.suptitle("Selected observed-anchor neighborhoods: no range-law fit or body-size inference", fontsize=10)
    save(fig, "anchor_support_geometry_vs_range")

    fig, axes = plt.subplots(1, 3, figsize=(10, 3.3), constrained_layout=True)
    for i, sid in enumerate(study.SEQUENCES):
        rs = [r for r in targets if r["sequence_id"] == sid and r["alignment_scenario"] == "ANCHOR_3m"]
        axes[0].scatter(values(rs, "range_rate_mps"), -values(rs, "velocity_median"), color=COLORS[i], s=15, label=sid)
        axes[1].scatter(values(rs, "eta"), values(rs, "velocity_signed_range_rate_residual_mps"), color=COLORS[i], s=15)
        axes[2].scatter(values(rs, "range_m"), values(rs, "velocity_signed_range_rate_residual_mps"), color=COLORS[i], s=15)
    axes[0].set_xlabel("GNSS chord range-rate proxy (m/s)")
    axes[0].set_ylabel("Negative median measured velocity")
    axes[0].legend(fontsize=5)
    axes[1].set_xlabel("Radial observability eta (provisional)")
    axes[2].set_xlabel("GNSS-projected sensor range (m)")
    for ax in axes[1:]:
        ax.set_ylabel("Signed Doppler-template residual (m/s)")
        ax.axhline(0, color="gray", lw=.6)
    fig.suptitle("Low-eta selected observations; chord reference and units/sign remain qualified", fontsize=10)
    save(fig, "doppler_chord_reference")

    occupancy = []
    for group_keys in [["sequence_id", "alignment_scenario"], ["nominal_range_m", "alignment_scenario"],
                       ["range_bin_m", "alignment_scenario"], ["eta_bin", "alignment_scenario"]]:
        groups = defaultdict(list)
        for r in targets:
            groups[tuple(r.get(k) for k in group_keys)].append(r)
        for key, rs in groups.items():
            for feature in ["raw_points", "unique_xyz_points", "occupied_voxels_0.04m", "occupied_voxels_1m"]:
                counts = values(rs, feature)
                masks = [("0", counts == 0), ("1", counts == 1), ("2", counts == 2), ("gt2", counts > 2)]
                occupancy.append({**dict(zip(group_keys, key)), "grouping": "+".join(group_keys), "representation": feature,
                                  "frames": len(rs), **{f"n_{name}": int(mask.sum()) for name, mask in masks},
                                  **{f"p_{name}": float(mask.mean()) for name, mask in masks},
                                  "status": "empirical region occupancy; NOT target detection probability"})
    study.cwrite(OUT / "representation_support_frequencies.csv", occupancy)
    study.jwrite(OUT / "figure_manifest.json", {"utc": study.utc(), "script_sha256": study.sha(Path(__file__)),
                 "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/gnss_phenotype_descriptive_figures.py",
                 "input_sha256": {"region_features.csv": study.sha(OUT / "region_features.csv")},
                 "figures": [str(p.relative_to(OUT)) for p in sorted((OUT / "figures").iterdir())],
                 "fitted_curves": False, "threshold_selection": False, "independent_test_evidence": False})


if __name__ == "__main__":
    main()
