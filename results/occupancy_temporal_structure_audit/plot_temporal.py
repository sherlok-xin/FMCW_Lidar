#!/usr/bin/env python3
"""Render descriptive temporal-occupancy audit outputs; never derive a score.

Run only after the feature-completion and outcome-interpretation receipts exist.
The frozen window, conditioning, populations and kill rule are unchanged.
"""
from __future__ import annotations

import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
import numpy as np


ROOT = Path(__file__).resolve().parent
SEQUENCES = ["100m_cross_5", "100m_cross_10", "200m_cross_5", "200m_cross_10", "300m_cross_10"]
FEATURES = ["age_s", "longest_previous_run", "previous_episodes", "recent_5_prior_frame_fraction"]
LABELS = ["Time since last occupancy\n(seconds)", "Longest previous run\n(recorded frames)", "Previous episodes\n(count)", "Recent occupancy fraction\n(5 preceding frames)"]
SHORT_LABELS = ["Recency (s)", "Longest run (frames)", "Episodes (count)", "Recent fraction"]
BLUE = "#176B9D"
ORANGE = "#B65428"
GREY = "#E9ECEF"
TEXT = "#202932"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(name: str) -> list[dict[str, str]]:
    with (ROOT / name).open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite(value: str) -> bool:
    return np.isfinite(float(value))


def save(fig: plt.Figure, stem: str) -> list[str]:
    names = [f"{stem}.pdf", f"{stem}.png"]
    for name in names:
        fig.savefig(ROOT / name, dpi=300, facecolor="white")
    plt.close(fig)
    return names


def support_direction(summary: list[dict[str, str]]) -> list[str]:
    indexed = {(r["condition_id"], r["feature"]): r for r in summary}
    fig, ax = plt.subplots(figsize=(13.2, 7.0))
    fig.subplots_adjust(left=.18, right=.985, top=.81, bottom=.25)
    ax.set_xlim(-.5, 4.5)
    ax.set_ylim(3.5, -.5)
    ax.set_xticks(range(5), [x.replace("_cross_", " m / cross ") for x in SEQUENCES])
    ax.xaxis.tick_top()
    ax.tick_params(axis="x", length=0, pad=11)
    ax.set_yticks(range(4), LABELS)
    ax.tick_params(axis="y", length=0, pad=12)
    for spine in ax.spines.values():
        spine.set_visible(False)
    for j, feature in enumerate(FEATURES):
        for i, sequence in enumerate(SEQUENCES):
            row = indexed[sequence, feature]
            n = int(row["finite_distinct_paired_frames"])
            eligible = int(row["eligible_anchor_frames"])
            median = float(row["median"])
            sign = int(row["median_direction"])
            fill = GREY if n == 0 else ("#DDEEF6" if sign > 0 else "#F6E5DC" if sign < 0 else "#F3F3F3")
            ax.add_patch(Rectangle((i-.5, j-.5), 1, 1, facecolor=fill, edgecolor="white", linewidth=3))
            if 0 < n < 3:
                ax.add_patch(Rectangle((i-.5, j-.5), 1, 1, facecolor="none", edgecolor="#CDD1D5", linewidth=0, hatch="///"))
            if n == 0:
                headline = "UNKNOWN"
                detail = f"0 / {eligible} eligible frames"
                colour = "#697680"
            else:
                value = f"{median:+.2f}" if feature == "age_s" else f"{median:+.2g}"
                unit = " s" if feature == "age_s" else " frames" if feature == "longest_previous_run" else ""
                headline = r"$\Delta$ = " + value + unit
                detail = f"{n} / {eligible} frames; {float(row['same_direction_fraction']):.0%} same sign"
                colour = BLUE if sign > 0 else ORANGE if sign < 0 else TEXT
            ax.text(i, j-.12, headline, ha="center", va="center", fontsize=12, fontweight="bold", color=colour)
            ax.text(i, j+.18, detail, ha="center", va="center", fontsize=8.4, color=TEXT)
            if 0 < n < 3:
                ax.text(i, j+.36, "LOW SUPPORT (n < 3)", ha="center", va="center", fontsize=7.3, color=TEXT)
    fig.text(.18, .945, "Same occupancy frequency leaves sparse distinct-cell temporal contrasts", fontsize=17, fontweight="bold", color=TEXT)
    fig.text(.18, .895, "Nearest observed-anchor node minus deduplicated original / background-only / historical-reference cells", fontsize=10, color=TEXT)
    fig.legend(handles=[Patch(facecolor="#DDEEF6", label="Positive median difference"), Patch(facecolor="#F6E5DC", label="Negative median difference"), Patch(facecolor=GREY, label="UNKNOWN: no finite distinct-cell pair"), Patch(facecolor="white", edgecolor="#CDD1D5", hatch="///", label="Finite contrast, fewer than 3 frames")], loc="lower center", bbox_to_anchor=(.56, .155), ncol=2, frameon=False, fontsize=9)
    fig.text(.18, .12, "Exact same frame and H imply identical F and p = (H + 1)/(F + 2). All contrasts require F >= 5 and different centroid cells.", fontsize=9, color=TEXT)
    fig.text(.18, .085, "Recency / recent fraction retain H >= 1; run / episodes exclude H=0 and H=1 (require H >= 2). Same-cell ties are excluded.", fontsize=9, color=TEXT)
    fig.text(.18, .05, "No quantity meets the frozen common-support and same-direction criterion in all five sequences. Missing support is not a zero effect.", fontsize=9, color=TEXT)
    fig.text(.18, .017, "Observed anchors are selected DEV_SILVER; distinct cells do not certify object identity or independent samples.", fontsize=9, color=TEXT)
    return save(fig, "temporal_support_direction")


def bin_vs_exact(bins: list[dict[str, str]], pairs: list[dict[str, str]]) -> list[str]:
    selected_bins = [r for r in bins if r["period"] == "warm_Fge5" and r["comparison_group"] == "background_only_winner" and finite(r["median_difference_anchor_minus_comparison"])]
    selected_pairs = [r for r in pairs if r["comparison_group"] == "background_only_winner" and r["status"] == "DISTINCT_CELL_EXACT_FREQUENCY" and int(r["F"]) >= 5 and int(r["anchor_H"]) >= (2 if r["feature"] in FEATURES[1:3] else 1) and finite(r["occupancy_conditioned_difference"])]
    fig, axes = plt.subplots(4, 5, figsize=(14.5, 9.8), sharex=True, sharey="row")
    fig.subplots_adjust(left=.095, right=.985, bottom=.23, top=.86, wspace=.16, hspace=.32)
    for j, feature in enumerate(FEATURES):
        values = [float(r["median_difference_anchor_minus_comparison"]) for r in selected_bins if r["feature"] == feature] + [float(r["occupancy_conditioned_difference"]) for r in selected_pairs if r["feature"] == feature]
        maximum = max([abs(x) for x in values] + [.01])
        for i, sequence in enumerate(SEQUENCES):
            ax = axes[j, i]
            b = [r for r in selected_bins if r["condition_id"] == sequence and r["feature"] == feature]
            p = [r for r in selected_pairs if r["condition_id"] == sequence and r["feature"] == feature]
            ax.axhline(0, color="#B8C0C7", linewidth=.7, zorder=0)
            for row in b:
                x = (float(row["p_left"]) + float(row["p_right"]))/2
                y = float(row["median_difference_anchor_minus_comparison"])
                small = min(int(row["anchor_finite_n"]), int(row["comparison_finite_n"])) < 3
                ax.scatter([x], [y], s=33 if not small else 23, facecolor="none", edgecolor="#5D6972" if not small else "#A7AFB5", linewidth=1.1, zorder=2)
            for row in p:
                ax.scatter([float(row["p_anchor"])], [float(row["occupancy_conditioned_difference"])], marker="s", s=35, facecolor=BLUE, edgecolor="white", linewidth=.5, zorder=3)
            ax.text(.97, .94, f"Exact-H n = {len(p)}" if p else "Exact-H: UNKNOWN", transform=ax.transAxes, ha="right", va="top", fontsize=8.5, color=BLUE if p else "#697680")
            ax.set_xlim(0, 1)
            ax.set_ylim(-maximum*1.17, maximum*1.22)
            ax.set_xticks([0, .25, .5, .75, 1])
            ax.tick_params(labelsize=8, length=3)
            ax.grid(axis="x", color="#EFF1F3", linewidth=.5)
            ax.spines["right"].set_visible(False)
            ax.spines["top"].set_visible(False)
            ax.spines["left"].set_color("#ADB5BD")
            ax.spines["bottom"].set_color("#ADB5BD")
            if i == 0:
                ax.set_ylabel(SHORT_LABELS[j] + "\nanchor minus competitor", fontsize=9)
            if j == 0:
                ax.set_title(sequence.replace("_cross_", " m / cross "), fontsize=10, pad=11)
            if j == 3:
                ax.set_xlabel(r"$p_{\mathrm{Beta}(1,1)}$", fontsize=10)
    fig.text(.095, .95, "Frequency-bin contrasts change when exposure and cell identity are controlled", fontsize=16, fontweight="bold", color=TEXT)
    fig.text(.095, .905, "Nearest observed-anchor nodes versus the existing background-only winner; no path rescoring or score changes", fontsize=10, color=TEXT)
    fig.legend(handles=[Line2D([], [], marker="o", color="none", markeredgecolor="#5D6972", markerfacecolor="none", label="Fixed width-0.05 p bin: difference of medians (F may differ)"), Line2D([], [], marker="s", color="none", markerfacecolor=BLUE, markeredgecolor="white", label="Same-frame, exact-H, distinct-cell pair: paired difference")], loc="lower center", bbox_to_anchor=(.54, .123), ncol=1, frameon=False, fontsize=9)
    fig.text(.095, .105, "All plotted histories have F >= 5. Small pale circles have fewer than 3 finite observations in at least one population.", fontsize=9, color=TEXT)
    fig.text(.095, .076, "Exact pairs retain H >= 1 for recency / recent and require H >= 2 for run / episodes; same-cell ties are excluded.", fontsize=9, color=TEXT)
    fig.text(.095, .047, "Missing p-bin support is blank, not zero. Recent fraction uses 5 prior frames (approximately 5 s at recorded cadence, not a 5 s time window).", fontsize=9, color=TEXT)
    fig.text(.095, .017, "Observed anchors are selected DEV_SILVER; distinct cells do not certify object identity or independent samples.", fontsize=9, color=TEXT)
    return save(fig, "temporal_bins_vs_exact_frequency")


def main() -> None:
    for required in ["FROZEN", "TEMPORAL_FEATURES_COMPLETE.json", "interpretation_manifest.json"]:
        if not (ROOT / required).exists():
            raise RuntimeError(f"Required frozen-stage receipt missing: {required}")
    protocol = json.loads((ROOT / "protocol.json").read_text())
    assert protocol["recent_window_frames"] == 5
    assert protocol["sequences"] == SEQUENCES
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "pdf.fonttype": 42, "ps.fonttype": 42, "axes.labelcolor": TEXT, "xtick.color": TEXT, "ytick.color": TEXT})
    outputs = support_direction(rows("kill_sequence_summary.csv"))
    outputs += bin_vs_exact(rows("frequency_bin_contrasts.csv"), rows("same_frame_exact_H_pairs.csv"))
    inputs = ["protocol.json", "TEMPORAL_FEATURES_COMPLETE.json", "interpretation_manifest.json", "kill_sequence_summary.csv", "frequency_bin_contrasts.csv", "same_frame_exact_H_pairs.csv"]
    manifest = {"utc": datetime.now(timezone.utc).isoformat(), "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/plot_temporal.py", "source_sha256": sha(Path(__file__)), "input_sha256": {name: sha(ROOT/name) for name in inputs}, "output_sha256": {name: sha(ROOT/name) for name in outputs}, "figures": {"temporal_support_direction": "Frozen kill-pool per-frame median contrasts, support denominator, missing support, and low-n flags; no cross-feature magnitude colour scale", "temporal_bins_vs_exact_frequency": "Fixed warm p-bin median differences versus nondegenerate same-frame/H distinct-cell paired differences against background-only winners; entire p range shown, blanks preserved"}, "not_claimed": ["classification performance", "statistical significance", "independent object identity", "zero effect from missing support", "held-out preregistration"], "visual_qa": "PENDING_RENDERED_IMAGE_REVIEW"}
    (ROOT / "figure_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")


if __name__ == "__main__":
    main()
