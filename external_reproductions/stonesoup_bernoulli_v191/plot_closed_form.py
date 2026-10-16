"""Plot the executed scalar counterexample only; never invent a trajectory."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "results/external_reproductions/stonesoup_bernoulli_v191"
data = json.loads((OUT / "closed_form_check.json").read_text())
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "pdf.fonttype": 42, "ps.fonttype": 42})
fig, axes = plt.subplots(1, 2, figsize=(9.3, 3.7), gridspec_kw={"width_ratios": [1.7, 1]})
values = [data["official_predictor_r_pred"], data["r_empty_binary64"],
          data["official_updater_r_post"]]
axes[0].bar([0, 1, 2], values, width=0.62, color=["#64748b", "#0072b2", "#d55e00"])
axes[0].set_xticks([0, 1, 2], ["Official\nprediction", "Bayes reference\nempty scan", "Official\nempty update"])
axes[0].set_ylabel("Existence probability")
axes[0].set_ylim(0, 0.62)
for index, value in enumerate(values):
    axes[0].text(index, value + 0.015, f"{value:.7f}", ha="center", fontsize=10)
axes[0].set_title("(a) One legal empty-scan update", loc="left", fontsize=11)
axes[0].grid(axis="y", alpha=0.18)
axes[0].set_axisbelow(True)
sums = [data["prediction_particles"]["weights_sum"], data["update_particles"]["weights_sum"]]
axes[1].bar([0, 1], sums, width=0.55, color=["#64748b", "#d55e00"])
axes[1].set_xticks([0, 1], ["Prediction\n18 particles", "Update\n9 particles"])
axes[1].set_ylim(0, 1.22)
axes[1].set_ylabel("Sum of conditional particle weights")
for index, value in enumerate(sums):
    axes[1].text(index, value + 0.035, f"{value:.10f}", ha="center", fontsize=9)
axes[1].set_title("(b) Conditional density stays normalized", loc="left", fontsize=10)
axes[1].grid(axis="y", alpha=0.18)
axes[1].set_axisbelow(True)
fig.suptitle("Stone Soup v1.9.1: prerequisite semantic check", fontsize=12, y=0.98)
fig.text(0.5, 0.015, "Fixed inputs; no truth, FMCW data or threshold. Multi-frame experiments NOT RUN.",
         ha="center", fontsize=9)
fig.tight_layout(rect=[0, 0.06, 1, 0.94])
fig.savefig(OUT / "closed_form_check.png", dpi=300, bbox_inches="tight")
fig.savefig(OUT / "closed_form_check.pdf", bbox_inches="tight")
plt.close(fig)
