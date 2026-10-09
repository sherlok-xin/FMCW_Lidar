#!/usr/bin/env python3
"""Plot recorded descriptive CSVs only; no data selection or algorithm run."""
from pathlib import Path
import csv
import json
import hashlib
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/range_sparsity_audit"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    with (OUT / "frame_metrics.csv").open(encoding="utf-8-sig", newline="") as f:
        rows = [r for r in csv.DictReader(f) if float(r["target_radius_m"]) == 3]
    sequences = sorted(set(r["sequence_id"] for r in rows))
    palette = plt.get_cmap("tab10")
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    metrics = ["raw_points", "occupied_voxels_1m", "unique_nn_m_median", "pairwise_m_median",
               "mst_connectivity_edge_max_m", "clutter_30m_nn_median_m"]
    names = ["Raw neighborhood returns", "Occupied 1 m voxels", "Unique-xyz target NN median (m)",
             "Pairwise median (m)", "MST maximum edge (m)", "Surrounding 3-30 m NN median (m)"]
    for ax, metric, name in zip(axes.flat, metrics, names):
        for i, sid in enumerate(sequences):
            subset = [r for r in rows if r["sequence_id"] == sid]
            x = [float(r["actual_target_range_m"]) for r in subset]
            y = [float(r[metric]) if r[metric] else np.nan for r in subset]
            ax.scatter(x, y, label=sid, s=17, alpha=.7, color=palette(i))
        ax.set(xlabel="Observed center range (m)", ylabel=name)
        if metric == "clutter_30m_nn_median_m":
            ax.set_yscale("log")
        ax.grid(alpha=.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=3, fontsize=8)
    fig.suptitle("Observed-center neighborhood proxies only; not point-level GT")
    fig.savefig(OUT / "range_microstructure.png", dpi=180)
    plt.close(fig)

    subset = sorted([r for r in rows if r["motion"] == "approach"], key=lambda r: float(r["time_s"]))
    x = [float(r["actual_target_range_m"]) for r in subset]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for metric, label in [("raw_points", "raw proxy"), ("legacy_target_cluster_points", "processed target cluster"),
                          ("legacy_total_dynamic_points", "all dynamic points")]:
        axes[0, 0].plot(x, [float(r[metric]) for r in subset], ".-", label=label)
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].set(ylabel="Point count (three different definitions)")
    for metric, label in [("nn_m_median", "NN median"), ("pairwise_m_median", "pairwise median"),
                          ("mst_connectivity_edge_max_m", "MST max edge")]:
        axes[0, 1].plot(x, [float(r[metric]) if r[metric] else np.nan for r in subset], ".-", label=label)
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].set(ylabel="Raw support spacing (m); singleton missing")
    axes[1, 0].plot(x, [float(r["intensity_median"]) for r in subset], ".-", label="intensity median")
    axes[1, 0].set(ylabel="Intensity (device units)")
    axes[1, 1].plot(x, [float(r["doppler_mps_median"]) for r in subset], ".-", label="measured Doppler")
    axes[1, 1].plot(x, [-float(r["observed_center_range_rate_mps"]) for r in subset], ".-", label="-observed dr/dt")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].set(ylabel="Velocity (dataset m/s convention)")
    for ax in axes.flat:
        ax.invert_xaxis()
        ax.set(xlabel="Observed center range, approach direction (m)")
        ax.grid(alpha=.2)
    fig.suptitle("Legacy approach: 20 retrospective observations, 324 to 195 m")
    fig.savefig(OUT / "longitudinal_approach.png", dpi=180)
    plt.close(fig)

    manifest = {"command": " ".join(sys.argv), "script_sha256": digest(Path(__file__)),
                "source_frame_csv_sha256": digest(OUT / "frame_metrics.csv"), "matplotlib": matplotlib.__version__,
                "output_sha256": {p.name: digest(p) for p in OUT.glob("*.png")}}
    (OUT / "plot_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("rendered 2 diagnostic figures from unchanged recorded CSVs")


if __name__ == "__main__":
    main()
