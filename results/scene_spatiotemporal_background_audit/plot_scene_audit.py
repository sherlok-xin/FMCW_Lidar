#!/usr/bin/env python3
"""Render frozen scene diagnostics; anchors are interpretation-only overlays.

All sequence snapshots use floor(number_of_frames/2), selected independently
of anchor availability and winner outcomes. Full cached scene extent is shown.
"""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json
import platform

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import PercentFormatter
from matplotlib.colors import LogNorm
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
FIG = OUT / "figures"
CIDS = ("100m_cross_5", "100m_cross_10", "200m_cross_5", "200m_cross_10", "300m_cross_10")
COLORS = {"raw_1m": "#286d94", "fine_1m": "#247a49", "fine_004m": "#a36b26",
          "persistent": "#74787d", "transient": "#e6a064", "novel": "#ce6928",
          "anchor": "#bf283a", "background_only": "#8652ac", "background_motion_E18": "#185c94"}
INPUTS = {}


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def register(path):
    INPUTS[str(path.relative_to(ROOT))] = sha(path)
    return path


def readcsv(path):
    with open(register(path), encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def loadnpz(path):
    with np.load(register(path)) as z:
        return {k: z[k].copy() for k in z.files}


def framepart(values, fi, key="cells"):
    return values[key][values["offsets"][fi]:values["offsets"][fi + 1]]


def xyz(rows):
    return np.array([[float(r[f"center_{a}_m"]) for a in "xyz"] for r in rows], dtype=float).reshape(-1, 3)


def savefig(fig, stem, pdf_pages=None):
    if pdf_pages is not None:
        pdf_pages.savefig(fig, bbox_inches="tight")
    fig.savefig(FIG / (stem + ".pdf"), bbox_inches="tight")
    fig.savefig(FIG / (stem + ".png"), dpi=300, bbox_inches="tight")
    plt.close(fig)


def decorate(ax):
    ax.grid(True, linewidth=.45, color="#d9dde1", alpha=.6)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)


def trajectories(stats):
    fig, axes = plt.subplots(3, 5, figsize=(15.4, 7.7), sharex="col", sharey="row", layout="constrained")
    for col, cid in enumerate(CIDS):
        for source in ("raw_1m", "fine_1m", "fine_004m"):
            rr = [r for r in stats if r["condition_id"] == cid and r["source"] == source]
            t = np.array([float(r["bag_time"]) for r in rr]); t -= t[0]
            for row, key in enumerate(("persistent_return_fraction", "first_visit_return_fraction", "persistent_lag1_jaccard")):
                axes[row, col].plot(t, [float(r[key]) for r in rr], color=COLORS[source], lw=1.5,
                                    ls=":" if source == "fine_004m" else "-", label=source)
        axes[0, col].set_title(cid.replace("_", " "), fontsize=10)
        axes[2, col].set_xlabel("Bag-record elapsed time (s)")
        for row in range(3):
            decorate(axes[row, col]); axes[row, col].set_ylim(-.025, 1.025)
            axes[row, col].yaxis.set_major_formatter(PercentFormatter(1))
    axes[0, 0].set_ylabel("Current return support\nin persistent past cells")
    axes[1, 0].set_ylabel("Current return support\nin first-visit cells")
    axes[2, 0].set_ylabel("Past persistent-set\nlag-1 Jaccard")
    axes[0, 0].legend(handles=[Line2D([], [], color=COLORS[s], lw=1.6, ls=":" if s == "fine_004m" else "-",
                                          label=l) for s, l in (("raw_1m", "Raw, 1 m"), ("fine_1m", "E18 fine input, 1 m"),
                                                                 ("fine_004m", "Fine, 4 cm sensitivity"))],
                         frameon=False, fontsize=7.3, loc="lower right")
    fig.suptitle("Causal scene persistence, novelty and stabilization", fontsize=13)
    savefig(fig, "scene_persistence_trajectories")


def scene_panel(ax, history, snapshot, fi, anchors, paths, projection, anchor_now=True):
    size = float(history["cell_m"])
    p = (framepart(snapshot, fi) + .5) * size
    c = (framepart(history, fi) + .5) * size
    first = framepart(history, fi, "first_visit")
    nonpersistent = ~framepart(history, fi, "persistent")
    i, j = projection
    if len(p):
        ax.scatter(p[:, i], p[:, j], c=COLORS["persistent"], s=3, marker="s", alpha=.48,
                   linewidths=0, rasterized=True)
    if np.any(nonpersistent):
        v = c[nonpersistent]
        ax.scatter(v[:, i], v[:, j], c=COLORS["transient"], s=3.5, marker=".", alpha=.6,
                   linewidths=0, rasterized=True)
    if np.any(first):
        v = c[first]
        ax.scatter(v[:, i], v[:, j], c=COLORS["novel"], s=5, marker=".", alpha=.85,
                   linewidths=0, rasterized=True)
    for arm, rr in paths.items():
        pp = xyz(rr)
        if len(pp):
            ax.plot(pp[:, i], pp[:, j], color=COLORS[arm], lw=.85, alpha=.85,
                    ls="--" if arm == "background_only" else "-")
            current = xyz([r for r in rr if int(r["frame_index"]) == fi])
            if len(current):
                ax.scatter(current[:, i], current[:, j], color=COLORS[arm], s=24, marker="o",
                           facecolors="none", linewidths=1.2)
    aa = xyz(anchors)
    if len(aa):
        ax.scatter(aa[:, i], aa[:, j], color=COLORS["anchor"], s=10, marker="x", linewidths=.75)
    current = xyz([r for r in anchors if int(r["frame_index"]) == fi])
    if anchor_now and len(current):
        ax.scatter(current[:, i], current[:, j], color=COLORS["anchor"], s=50, marker="x", linewidths=1.4)
    # Extents use complete scene evidence, never anchor-centred crops.
    union = np.vstack([p, c])
    if len(union):
        for k, fn in ((i, ax.set_xlim), (j, ax.set_ylim)):
            lo, hi = np.min(union[:, k]), np.max(union[:, k]); pad = max((hi-lo)*.025, 1.)
            fn(lo-pad, hi+pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"{'xyz'[i]} analysis (m)"); ax.set_ylabel(f"{'xyz'[j]} analysis (m)")
    decorate(ax)


def scene_legend():
    return [Line2D([], [], marker="s", ls="", color=COLORS["persistent"], markersize=4, label="Persistent past cells"),
            Line2D([], [], marker=".", ls="", color=COLORS["transient"], markersize=6, label="Current nonpersistent cells"),
            Line2D([], [], marker=".", ls="", color=COLORS["novel"], markersize=6, label="Current first-visit cells"),
            Line2D([], [], marker="x", ls="", color=COLORS["anchor"], markersize=5, label="Observed anchors, all observed frames"),
            Line2D([], [], ls="--", color=COLORS["background_only"], lw=1.2, label="Background-only saved full path"),
            Line2D([], [], ls="-", color=COLORS["background_motion_E18"], lw=1.2, label="E18 saved full path")]


def scene_projections(histories, snapshots, anchors, paths):
    fig, axes = plt.subplots(5, 3, figsize=(12.7, 14.6), layout="constrained")
    metadata = []
    for row, cid in enumerate(CIDS):
        hh, ss = histories[(cid, "raw_1m")], snapshots[(cid, "raw_1m")]
        fi = len(hh["times"]) // 2
        aa = [r for r in anchors if r["condition_id"] == cid]
        pp = {arm: [r for r in paths if r["condition_id"] == cid and r["arm"] == arm]
              for arm in ("background_only", "background_motion_E18")}
        now = sum(int(r["frame_index"]) == fi for r in aa)
        for col, projection in enumerate(((0, 1), (0, 2), (1, 2))):
            scene_panel(axes[row, col], hh, ss, fi, aa, pp, projection)
            axes[row, col].set_title(f"{cid.replace('_', ' ')} | frame {fi}\n{'xyz'[projection[0]].upper()}{'xyz'[projection[1]].upper()} projection", fontsize=9)
        axes[row, 0].text(.01, .98, f"Current-frame anchor: {'available' if now else 'not observed'}",
                          transform=axes[row, 0].transAxes, va="top", fontsize=7,
                          bbox=dict(facecolor="white", edgecolor="none", alpha=.8))
        metadata.append({"condition_id": cid, "frame_index": fi, "bag_time": float(hh["times"][fi]),
                         "selection_rule": "floor(nframes/2)", "current_anchor_count": now})
    fig.get_layout_engine().set(rect=(0, .105, 1, .885))
    fig.legend(handles=scene_legend(), loc="lower center", bbox_to_anchor=(.5, .04), ncol=3, frameon=False, fontsize=8)
    fig.suptitle("Raw scene at fixed sequence midpoints; full scene projections", fontsize=13)
    fig.text(.5, .012, "Map uses past frames only. Entire saved paths and all observed anchors are offline interpretive overlays.\nCoordinates are sensor analysis [raw x, raw z, -raw y], not surveyed gravity.", ha="center", va="bottom", fontsize=8)
    savefig(fig, "raw_scene_fixed_midpoint_projections")
    return metadata


def competitor_profiles(histories, snapshots, anchors, paths, features):
    fig, axes = plt.subplots(2, 4, figsize=(15.4, 7.0), layout="constrained")
    index = {(r["condition_id"], r["source"], int(r["node_id"])): r for r in features
             if float(r["persistent_cutoff"]) == .5 and r["source"] in ("raw_1m", "fine_1m")}
    for row, cid in enumerate(("100m_cross_10", "300m_cross_10")):
        hh = histories[(cid, "raw_1m")]; fi = len(hh["times"]) // 2; t0 = float(hh["times"][0])
        aa = [r for r in anchors if r["condition_id"] == cid]
        pp = {arm: sorted([r for r in paths if r["condition_id"] == cid and r["arm"] == arm], key=lambda r:int(r["path_order"]))
              for arm in ("background_only", "background_motion_E18")}
        for col, source in enumerate(("raw_1m", "fine_1m")):
            scene_panel(axes[row, col], histories[(cid, source)], snapshots[(cid, source)], fi, aa, pp, (0, 2))
            axes[row, col].set_title(f"{cid.replace('_', ' ')} | frame {fi}\n{'Full raw map' if col == 0 else 'Legacy E18 fine-input map'}", fontsize=9)
        for arm, rr in pp.items():
            tt = [float(r["bag_time"])-t0 for r in rr]
            for source, style in (("raw_1m", "-"), ("fine_1m", ":")):
                ff = [index[(cid, source, int(r["node_id"]))] for r in rr]
                axes[row, 2].plot(tt, [float(r["p_beta11"]) for r in ff], color=COLORS[arm], ls=style, lw=1.3)
                axes[row, 3].plot(tt, [float(r["distance_to_persistent_scene_m"]) for r in ff], color=COLORS[arm], ls=style, lw=1.3)
        for col in (2, 3):
            axes[row, col].axvline(float(hh["times"][fi])-t0, color="#888888", lw=.6, ls="--")
            axes[row, col].set_xlabel("Bag-record elapsed time (s)"); decorate(axes[row, col])
        axes[row, 2].set_ylabel("Centroid occupancy, Beta p"); axes[row, 2].set_ylim(-.025, 1.025)
        axes[row, 3].set_ylabel("Distance to persistent scene (m)"); axes[row, 3].set_ylim(bottom=0)
        axes[row, 2].set_title("Location history: solid raw / dotted fine", fontsize=9)
        axes[row, 3].set_title("Cell-AABB distance: solid raw / dotted fine", fontsize=9)
    fig.get_layout_engine().set(rect=(0, .16, 1, .83))
    fig.legend(handles=scene_legend(), loc="lower center", bbox_to_anchor=(.5, .065), ncol=3, frameon=False, fontsize=8)
    fig.suptitle("Existing competitor paths: scene attachment and occupancy", fontsize=13)
    fig.text(.5, .012, "Maps at fixed midpoints; paths are unchanged offline full-bag winners. Features at each path node use only its earlier frames.\nBlank distance segments denote an empty causal persistent set, not zero distance. No path rescore or object identity is inferred.", ha="center", va="bottom", fontsize=8)
    savefig(fig, "competitor_scene_profiles")


POPULATIONS = ("all_fine_nodes", "nearest_anchor_node", "winner_original", "winner_background_only", "winner_background_motion_E18")
POP_LABELS = ("All fine candidates", "Nearest anchor nodes", "Original winners", "Background-only winners", "E18 winners")
POP_COLORS = ("#74787d", COLORS["anchor"], "#877428", COLORS["background_only"], COLORS["background_motion_E18"])


def population_groups(populations):
    groups = {}
    for cid in CIDS:
        for group in POPULATIONS:
            groups[(cid, group)] = [r for r in populations if r["condition_id"] == cid and r["group"] == group
                                   and r["source"] == "raw_1m" and float(r["cutoff"]) == .5]
    return groups


def distance_contrasts(populations, pdf_pages):
    groups = population_groups(populations)
    fig, axes = plt.subplots(1, 5, figsize=(15.7, 5.3), sharex=True, sharey=True, layout="constrained")
    finite_n = []
    for col, cid in enumerate(CIDS):
        ax = axes[col]
        medians = {}
        for pos, (group, label, color) in enumerate(zip(POPULATIONS, POP_LABELS, POP_COLORS)):
            rr = groups[(cid, group)]
            v = np.array([float(r["distance_to_persistent_scene_m"]) for r in rr]); good = np.isfinite(v); d = v[good]
            missing = int((~good).sum())
            if len(d):
                lo, q25, median, q75, hi = np.quantile(d, [.05, .25, .5, .75, .95])
                ax.plot([lo, hi], [pos, pos], color=color, lw=1)
                ax.plot([q25, q75], [pos, pos], color=color, lw=6, solid_capstyle="butt", alpha=.7)
                ax.plot(median, pos, marker="o", color=color, markersize=4, markeredgecolor="white", markeredgewidth=.5)
                ax.plot([d.min(), d.max()], [pos, pos], ls="", marker="|", color=color, alpha=.55, markersize=6)
                medians[group] = float(median)
            ax.text(.98, (4.75-pos)/5.5, f"n={len(d)}; missing={missing}", transform=ax.transAxes,
                    ha="right", va="center", fontsize=7.3, color=color)
            finite_n.append({"condition_id": cid, "population": group, "finite_distance_n": int(good.sum()),
                             "missing_distance_n": missing, "population_n": len(rr),
                             "missing_frame_indices": sorted({int(r["frame_index"]) for r, g in zip(rr, good) if not g})})
        ax.set_title(cid.replace("_", " ") + f"\nAnchor median {medians['nearest_anchor_node']:.2f} m\nBG-only median {medians['winner_background_only']:.2f} m", fontsize=9)
        ax.set_xlim(-1.8, 51); ax.set_ylim(4.65, -.85); ax.set_yticks(range(5), POP_LABELS)
        ax.set_xlabel("Distance to persistent raw scene (m)")
        decorate(ax)
    fig.get_layout_engine().set(rect=(0, .155, 1, .835))
    fig.suptitle("Descriptive distance distributions across the five existing sequences", fontsize=13)
    fig.legend(handles=[Line2D([], [], color="#666666", lw=6, label="IQR"),
                        Line2D([], [], color="#666666", lw=1, label="5th-95th percentiles"),
                        Line2D([], [], color="#666666", ls="", marker="o", label="Median"),
                        Line2D([], [], color="#666666", ls="", marker="|", label="Min/max")],
               loc="lower center", bbox_to_anchor=(.5, .065), ncol=4, frameon=False, fontsize=8)
    fig.text(.5, .008, "Fixed raw 1 m scene and preregistered persistence rule; missing distances arise during cold start (frames 0-1).\nAll candidates include anchor matches; populations cover different frames. Observed DEV/SILVER anchors are not GT. No classification performance or detector gate.",
             ha="center", va="bottom", fontsize=8)
    savefig(fig, "distance_population_contrasts", pdf_pages)
    return finite_n


def occupancy_distance(populations, associations, pdf_pages):
    groups = population_groups(populations)
    fig, axes = plt.subplots(1, 5, figsize=(15.7, 5.1), sharex=True, sharey=True, layout="constrained")
    hexes = []
    for col, cid in enumerate(CIDS):
        ax = axes[col]
        rr = groups[(cid, "all_fine_nodes")]
        p = np.array([float(r["e18_p_beta11"]) for r in rr]); d = np.array([float(r["distance_to_persistent_scene_m"]) for r in rr])
        good = np.isfinite(p) & np.isfinite(d)
        hb = ax.hexbin(p[good], d[good], gridsize=(24, 18), extent=(0, 1, 0, 50), mincnt=1,
                       cmap="Greys", linewidths=0, rasterized=True)
        hexes.append(hb)
        for group, marker, color, size in (("winner_background_only", "o", COLORS["background_only"], 15),
                                            ("winner_background_motion_E18", ".", COLORS["background_motion_E18"], 13),
                                            ("nearest_anchor_node", "x", COLORS["anchor"], 22)):
            ss = groups[(cid, group)]
            pp = np.array([float(r["e18_p_beta11"]) for r in ss]); dd = np.array([float(r["distance_to_persistent_scene_m"]) for r in ss])
            mask = np.isfinite(pp) & np.isfinite(dd)
            if marker == "o":
                ax.scatter(pp[mask], dd[mask], s=size, marker=marker, facecolors="none", edgecolors=color, linewidths=.65)
            else:
                ax.scatter(pp[mask], dd[mask], s=size, marker=marker, color=color, linewidths=.75)
        corr = next(r for r in associations if r["condition_id"] == cid and r["source"] == "raw_1m")
        rho = float(corr["spearman_distance_vs_e18_fine_p"])
        ax.set_title(cid.replace("_", " ") + f"\nSpearman rho={rho:.2f}; finite n={int(good.sum())}", fontsize=9)
        ax.set_xlim(-.025, 1.025); ax.set_ylim(-1, 51)
        ax.set_xlabel("Existing E18 fine occupancy, Beta p")
        decorate(ax)
    vmax = max(float(np.max(h.get_array())) for h in hexes)
    norm = LogNorm(vmin=1, vmax=vmax)
    for h in hexes:
        h.set_norm(norm)
    cb = fig.colorbar(hexes[0], ax=axes, shrink=.72, pad=.012, aspect=25)
    cb.set_label("All fine candidate nodes per hexagon (log)", fontsize=8)
    axes[0].set_ylabel("Distance to persistent raw scene (m)")
    fig.get_layout_engine().set(rect=(0, .16, 1, .83))
    fig.suptitle("Existing occupancy versus scene distance: related signals with residual spread", fontsize=13)
    fig.legend(handles=[Line2D([], [], marker="x", ls="", color=COLORS["anchor"], label="Nearest anchor nodes"),
                        Line2D([], [], marker="o", ls="", markerfacecolor="none", color=COLORS["background_only"], label="Background-only winners"),
                        Line2D([], [], marker=".", ls="", color=COLORS["background_motion_E18"], label="E18 winners")],
               loc="lower center", bbox_to_anchor=(.5, .065), ncol=3, frameon=False, fontsize=8)
    fig.text(.5, .008, "Each node feature is causal; complete populations and interpretive overlays are offline. Cold-start undefined distances are excluded.\nTemporal dependence and unequal observation windows remain; the plot is a descriptive proxy contrast. Silver is not GT; no score, threshold or path changes.",
             ha="center", va="bottom", fontsize=8)
    savefig(fig, "occupancy_distance_nonredundancy", pdf_pages)


def main():
    receipt_path = OUT / "SCENE_FEATURES_COMPLETE.json"
    if not receipt_path.exists():
        raise RuntimeError("scene-feature receipt required before reading anchors or winner paths")
    receipt = json.loads(register(receipt_path).read_text())
    if receipt["label_or_gnss_used"] is not False:
        raise RuntimeError("receipt must attest label-free scene computation")
    # Verify immutable scene files before enabling interpretive input reads.
    for name, digest in receipt["outputs"].items():
        if sha(OUT / name) != digest:
            raise RuntimeError("scene receipt mismatch: " + name)
    FIG.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.labelsize": 8,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "pdf.fonttype": 42,
                         "ps.fonttype": 42, "savefig.facecolor": "white", "axes.linewidth": .65})
    stats = readcsv(OUT / "frame_scene_statistics.csv")
    features = readcsv(OUT / "node_scene_features.csv")
    histories, snapshots = {}, {}
    for cid in CIDS:
        for source in ("raw_1m", "fine_1m"):
            histories[(cid, source)] = loadnpz(OUT / f"cell_history__{source}__{cid}.npz")
            snapshots[(cid, source)] = loadnpz(OUT / f"persistent_snapshots__{source}__{cid}.npz")
    # This read is deliberately after verification of complete frozen features.
    anchors = readcsv(ROOT / "results/research_dev_v0/silver_labels.csv")
    paths = [r for r in readcsv(ROOT / "results/e18_mechanism_generalization_audit/selected_paths.csv")
             if r["family"] == "fine_4cm_pregeometry"]
    trajectories(stats)
    selected = scene_projections(histories, snapshots, anchors, paths)
    competitor_profiles(histories, snapshots, anchors, paths, features)
    populations = readcsv(OUT / "population_scene_features.csv")
    associations = readcsv(OUT / "occupancy_distance_associations.csv")
    # Hash the tabulated descriptive summaries as companion provenance.
    register(OUT / "population_distribution_summary.csv")
    register(OUT / "anchor_same_frame_contrasts.csv")
    with PdfPages(FIG / "descriptive_scene_contrasts.pdf") as pdf:
        finite_n = distance_contrasts(populations, pdf)
        occupancy_distance(populations, associations, pdf)
    manifest = {"utc": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
                "matplotlib": matplotlib.__version__, "numpy": np.__version__,
                "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/plot_scene_audit.py",
                "plot_source_sha256": sha(__file__), "scene_receipt_verified_before_anchor_read": True,
                "scene_complete_utc": receipt["utc"], "selected_frames": selected,
                "descriptive_contrast_finite_and_missing_counts": finite_n,
                "spatial_crop": "full persistent/current scene extent; no GNSS or anchor ROI",
                "maps": "past-only causal snapshots", "paths_and_anchor_overlays": "offline, interpretation only",
                "skipped_metric": "fine_004m spatial_concentration is omitted (stored cube is 0.4m)",
                "input_sha256": INPUTS, "figure_sha256": {p.name: sha(p) for p in sorted(FIG.glob("*"))}}
    (OUT / "figure_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"figures": len(manifest["figure_sha256"]), "selected_frames": selected}))


if __name__ == "__main__":
    main()
