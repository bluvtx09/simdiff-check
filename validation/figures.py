"""README용 그래프 두 장 (영문). python validation/figures.py → validation/figures/*.png"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RES = Path(__file__).parent / "results"
FIG = Path(__file__).parent / "figures"
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
COL = {"mujoco": "#2a78d6", "pybullet": "#eb6834"}
NAME = {"mujoco": "MuJoCo", "pybullet": "PyBullet"}

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 12,
    "axes.titleweight": "bold", "axes.titlelocation": "left", "axes.titlepad": 30,
})


def bars(ax, cats, vals, ns):
    x = np.arange(len(cats))
    w = 0.38
    for k, eng in enumerate(["mujoco", "pybullet"]):
        xs = x + (k - 0.5) * w
        v = vals[eng]
        ax.bar(xs, np.nan_to_num(v), w - 0.02, color=COL[eng], label=NAME[eng], zorder=3,
               edgecolor=SURFACE, linewidth=1)
        for xi, vi, ni in zip(xs, v, ns[eng]):
            if not np.isnan(vi):
                ax.annotate(f"{vi:.0f}%", (xi, vi), xytext=(0, 3), textcoords="offset points",
                            ha="center", fontsize=8.5, color=INK)
    ax.set_xticks(x, [f"{c}\nn={a:,} / {b:,}" for c, a, b in zip(cats, ns["mujoco"], ns["pybullet"])])
    ax.tick_params(axis="x", length=0, pad=6)
    ax.set_ylim(0, 108)
    ax.legend(frameon=False, loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2, fontsize=9)


def fig_linter():
    cats = ["passes\nall rules", "1 warning", "2 warnings", "3 warnings", "any error"]
    vals, ns = {}, {}
    for eng in ["mujoco", "pybullet"]:
        d = pd.read_csv(RES / f"v2_{eng}.csv")
        d["rules"] = d.rules.fillna("")
        a = d.err_pct.abs()
        k = d.rules.apply(lambda x: len([r for r in x.split(",") if r]))
        groups = [~d.flagged, (d.n_error == 0) & (k == 1), (d.n_error == 0) & (k == 2),
                  (d.n_error == 0) & (k >= 3), d.n_error > 0]
        vals[eng] = [100 * (a[g] > 1).mean() if g.sum() else np.nan for g in groups]
        ns[eng] = [int(g.sum()) for g in groups]
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    bars(ax, cats, vals, ns)
    ax.set_ylabel("scenes with > 1% error")
    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter())
    ax.set_title("Scenes that pass the linter were never off by more than 1%")
    fig.text(0.01, 0.015, "8,000 unseen sliding-box scenes (new shapes, 20% with common mistakes). "
             "Error: stop distance vs analytic. n: MuJoCo / PyBullet", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(FIG / "linter_vs_error.png", dpi=180)
    plt.close(fig)


def fig_selfcheck():
    cats = ["observed order ≈ 1\n(estimate trusted)", "observed order off\n(flagged)", "config mistake\npresent"]
    vals, ns = {}, {}
    for eng in ["mujoco", "pybullet"]:
        d = pd.read_csv(RES / f"v3_box_{eng}.csv")
        r = d.actual / d.est.replace(0, np.nan)
        ok = (r >= 0.5) & (r <= 2)
        clean = ~d.trap
        groups = [clean & d.reliable, clean & ~d.reliable, d.trap]
        vals[eng] = [100 * ok[g].mean() for g in groups]
        ns[eng] = [int(g.sum()) for g in groups]
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    bars(ax, cats, vals, ns)
    ax.set_ylabel("estimates within 2x of true error")
    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter())
    ax.set_title("dt vs dt/2 estimates numerical error, but not setting mistakes")
    fig.text(0.01, 0.015, "Same scenes, also run at dt/2 and dt/4. With a setting mistake the true error is "
             "15-300x the estimate. n: MuJoCo / PyBullet", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(FIG / "selfcheck_calibration.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    FIG.mkdir(exist_ok=True)
    fig_linter()
    fig_selfcheck()
