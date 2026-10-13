#!/usr/bin/env python3
"""Layout-only revision of the support plot; preserves the original figure."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import gnss_conditioned_uav_phenotype as study
import gnss_phenotype_descriptive_figures as figures


def main():
    out = study.OUT / "run_02"
    study.verify(out)
    rows = study.cread(out / "region_features.csv")
    plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 8,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": .18, "pdf.fonttype": 42})
    fig, axes = plt.subplots(5, 2, figsize=(9, 10), constrained_layout=True)
    for i, sid in enumerate(study.SEQUENCES):
        for j, recipe in enumerate(["GNSS_3m", "ANCHOR_3m"]):
            rs = sorted([r for r in rows if r["region_role"] == "UAV_CONDITIONED" and r["sequence_id"] == sid and r["alignment_scenario"] == recipe], key=lambda r: r["time_s"])
            time = figures.values(rs, "time_s")
            time -= min(time)
            ax = axes[i, j]
            for k, (feature, label, marker) in enumerate([
                    ("raw_points", "raw stored returns", "o"), ("unique_xyz_points", "unique XYZ", "s"),
                    ("occupied_voxels_0.04m", "4 cm cells", "^"), ("occupied_voxels_1m", "1 m cells", "x")]):
                ax.scatter(time, figures.values(rs, feature), s=10, color=figures.COLORS[k], marker=marker, label=label)
            ax.set_title(f"{sid} | {recipe} | n={len(rs)}", fontsize=8)
            ax.set_yscale("symlog", linthresh=1)
            ax.set_xlabel("Time from recipe start (s)")
            if j == 0:
                ax.set_ylabel("Return / cell count")
    axes[0, 1].legend(fontsize=6, loc="upper right")
    fig.suptitle("Uncertain GNSS-region occupancy vs selected closure-anchor support\nCounts are not point-identity GT or sensor detection probability", fontsize=10)
    figures.save(fig, "support_time_and_representation_v2")
    study.jwrite(out / "support_figure_v2_manifest.json", {
        "utc": study.utc(), "script_sha256": study.sha(Path(__file__)),
        "imported_plot_script_sha256": study.sha(Path(figures.__file__)),
        "input_sha256": study.sha(out / "region_features.csv"),
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/gnss_phenotype_support_figure_v2.py",
        "change": "layout labels/spacing only; original figures and all data retained",
        "new_files": ["figures/support_time_and_representation_v2.png", "figures/support_time_and_representation_v2.pdf"]})


if __name__ == "__main__":
    main()
