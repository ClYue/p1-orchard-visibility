"""
生成论文图 3–8（Drones/MDPI 投稿版，英文标注，600 dpi 可调）。

数据来源（全部现成，脚本不重算模型）：
  Fig.3  figures_viz/vis_lut.npz + fig_profiles.json (b sweep)
  Fig.4  fig_profiles.json (offset sweep) + e1b_results.json
  Fig.5  ../02_P1_论文/code/pareto_lut.json
  Fig.6  ../02_P1_论文/code/joint_lut.json
  Fig.7  ../02_P1_论文/code/metrics_lut_*_k0.2.json (exp2)
  Fig.8  figures_viz/nees_series.json

用法：
  python figs_3to8.py                 # 全部，300 dpi
  python figs_3to8.py --dpi 600       # 投稿终稿
  python figs_3to8.py --only 3 5      # 只出指定图
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
E1 = HERE.parents[1]                      # 08_Gazebo仿真
VIZ = E1 / "p1_gz_sim" / "figures_viz"
CODE = E1.parent / "02_P1_论文" / "code"

# ---------------- global style ----------------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8.0,
    "axes.labelsize": 8.0,
    "axes.titlesize": 8.0,
    "xtick.labelsize": 7.2,
    "ytick.labelsize": 7.2,
    "legend.fontsize": 7.0,
    "axes.linewidth": 0.7,
    "xtick.major.width": 0.7,
    "ytick.major.width": 0.7,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "lines.linewidth": 1.3,
    "figure.dpi": 150,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

C_PROP = "#c1272d"
C_AXIAL = "#1f6fb2"
C_LAT = "#2e8b57"
C_HOVER = "#7a7a7a"
C_RAND = "#7b5ea7"
C_NAIVE = "#d98c00"
C_CI = "#1f6fb2"

CASES = {"alley_center": "alley", "canopy_edge": "edge"}
DPI = 300


def save(fig, name: str) -> None:
    p = VIZ / name
    fig.savefig(p, dpi=DPI)
    plt.close(fig)
    print(f"  -> {p.name}  ({p.stat().st_size/1e3:.0f} kB)")


def tag(ax, s: str, x: float = 0.015, y: float = 0.965) -> None:
    ax.text(x, y, s, transform=ax.transAxes, ha="left", va="top",
            fontsize=8.2, fontweight="bold")


# =======================  Fig. 3  =======================
def fig3() -> None:
    z = np.load(E1 / "vis_lut.npz")
    GX, GY = z["GX_alley_center"], z["GY_alley_center"]
    PV = z["pvis_alley_center"]
    prof = json.loads((VIZ / "fig_profiles.json").read_text(encoding="utf-8"))
    g = np.array(prof["grid"])

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.95),
                             gridspec_kw=dict(width_ratios=[1.12, 1.0], wspace=0.30))

    # ---- (a) calibrated visibility field
    ax = axes[0]
    im = ax.imshow(PV, origin="lower", extent=(-15, 15, -15, 15), cmap="viridis",
                   vmin=0, vmax=1, interpolation="nearest", aspect="equal")
    cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
    cb.set_label(r"$p_{\mathrm{vis}}$", fontsize=7.6)
    cb.ax.tick_params(labelsize=6.6)

    ax.add_patch(Rectangle((-0.25, -0.16), 0.5, 0.32, fc="white", ec="k", lw=0.6, zorder=6))
    for sx in (0.5, -0.5):
        ax.plot([sx], [0], marker="s", ms=2.6, color="#ff7f0e", mec="k", mew=0.3, zorder=7)
    ax.annotate("", xy=(5.6, 0), xytext=(1.4, 0),
                arrowprops=dict(arrowstyle="-|>", color="white", lw=1.2, mutation_scale=9), zorder=8)
    ax.text(5.9, 0.45, "along-row $x$", color="0.15", fontsize=7.0, zorder=8,
            bbox=dict(fc="white", ec="none", alpha=0.55, pad=1.0))
    ax.annotate("", xy=(0, 5.4), xytext=(0, 1.5),
                arrowprops=dict(arrowstyle="-|>", color="white", lw=1.2, mutation_scale=9), zorder=8)
    ax.text(0.45, 5.7, "across-row $y$", color="white", fontsize=7.0, zorder=8)
    ax.set_xlabel("UAV offset from robot / m")
    ax.set_ylabel("UAV offset from robot / m")
    ax.set_xlim(-9.5, 9.5); ax.set_ylim(-9.5, 9.5)
    tag(ax, "(a)")

    # ---- (b) across-row profiles vs canopy width
    ax = axes[1]
    cols = plt.cm.viridis(np.linspace(0.1, 0.85, len(prof["b_sweep"]["rows"])))
    ax.axhline(0.5, color="0.55", ls=(0, (3, 2)), lw=0.7, zorder=1)
    ax.text(7.55, 0.52, r"$p_{\mathrm{vis}}=0.5$", fontsize=6.6, color="0.35", ha="right")
    for col, row in zip(cols, prof["b_sweep"]["rows"]):
        y = np.abs(g)
        m = y <= 8.0
        ax.plot(y[m], np.array(row["across"])[m], color=col, lw=1.4,
                label=f"$b$ = {row['canopy_b']:.1f} m")
        y50 = row["Y50_across"]
        ax.plot([y50], [0.5], "o", ms=3.2, color=col, mec="white", mew=0.5, zorder=5)
    ax.set_xlabel(r"across-row offset $|y|$ / m")
    ax.set_ylabel(r"$p_{\mathrm{vis}}$")
    ax.set_xlim(0, 8); ax.set_ylim(-0.02, 1.05)
    ax.legend(loc="upper right", frameon=False, handlelength=1.4, borderaxespad=0.2)
    ax.annotate("canopy window\ncloses", xy=(2.5, 0.52), xytext=(4.3, 0.70),
                fontsize=6.8, color="0.25",
                arrowprops=dict(arrowstyle="->", lw=0.6, color="0.45",
                                connectionstyle="arc3,rad=0.18"))
    tag(ax, "(b)", x=0.66, y=0.30)
    save(fig, "fig3_visibility.png")


# =======================  Fig. 4  =======================
def fig4() -> None:
    prof = json.loads((VIZ / "fig_profiles.json").read_text(encoding="utf-8"))
    g = np.array(prof["grid"])
    rows = prof["offset_sweep"]["rows"]
    styles = [("#4d4d4d", (0, (5, 2)), 1.5), ("#c98a2b", "-", 1.1), (C_PROP, "-", 1.45)]

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.75),
                             gridspec_kw=dict(width_ratios=[1.35, 1.0], wspace=0.32))

    ax = axes[0]
    ax.axhline(0.5, color="0.55", ls=(0, (3, 2)), lw=0.7)
    for (col, ls, lw), r in zip(styles, rows):
        m = np.abs(g) <= 10
        lab = f"$d$ = {r['offset']:.1f} m"
        if r["offset"] == 1.0:
            lab += "  (markers at hedge)"
        ax.plot(g[m], np.array(r["along"])[m], color=col, ls=ls, lw=lw, label=lab)
    ax.set_xlabel(r"along-row offset $x$ / m")
    ax.set_ylabel(r"$p_{\mathrm{vis}}$")
    ax.set_xlim(-10, 10); ax.set_ylim(-0.02, 1.05)
    ax.legend(loc="lower left", frameon=False, handlelength=1.4)
    ax.annotate(r"$X_{50}$ = 4.0 m", xy=(4.0, 0.5), xytext=(5.2, 0.30),
                fontsize=6.8, color=C_PROP,
                arrowprops=dict(arrowstyle="->", lw=0.6, color=C_PROP))
    tag(ax, "(a)", x=0.87, y=0.90)

    # ---- (b) descriptors vs offset
    ax = axes[1]
    d = np.array([r["offset"] for r in rows])
    x50 = np.array([min(r["X50_along"], 15.0) for r in rows])
    p0 = np.array([r["p_nadir"] for r in rows])
    ax.bar(d, x50, width=0.28, color="#c9d4e0", edgecolor="#5b7796", lw=0.6,
           label=r"$X_{50}$ (left)")
    for xi, vi, capped in zip(d, x50, [r["X50_capped"] for r in rows]):
        s = "≥15" if capped else f"{vi:.1f}"
        ax.text(xi, vi + 0.35, s, ha="center", va="bottom", fontsize=6.8, color="#33506b")
    ax.set_ylabel(r"$X_{50}$ / m")
    ax.set_ylim(0, 18)
    ax.set_xlabel("marker lateral offset $d$ / m")
    ax.set_xticks(d)

    ax2 = ax.twinx()
    ax2.plot(d, p0, "o-", color=C_RAND, ms=4, lw=1.2, label=r"$p_0$ (right)")
    for xi, vi in zip(d, p0):
        ax2.text(xi, vi - 0.045, f"{vi:.2f}", ha="center", va="top",
                 fontsize=6.4, color=C_RAND)
    ax2.set_ylabel(r"$p_{\mathrm{vis}}$ at nadir, $p_0$")
    ax2.set_ylim(0.4, 1.12)
    ax2.tick_params(axis="y", colors=C_RAND)
    ax2.spines["right"].set_color(C_RAND)
    ax2.yaxis.label.set_color(C_RAND)
    tag(ax, "(b)")
    save(fig, "fig4_offset.png")


# =======================  Fig. 5  =======================
def fig5() -> None:
    par = json.loads((CODE / "pareto_lut.json").read_text(encoding="utf-8"))
    marks = {"hover": ("s", C_HOVER, "Hover"),
             "follow10": ("^", C_LAT, r"Constant distance (10 m lateral)"),
             "axis10": ("D", C_AXIAL, r"Constant look-ahead (10 m axial)"),
             "random": ("v", C_RAND, "Random manoeuvre")}

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 3.05), sharey=False,
                             gridspec_kw=dict(wspace=0.20))
    handles = None
    for ax, case in zip(axes, ("alley_center", "canopy_edge")):
        r = par[case]
        ours = sorted(r["ours"], key=lambda o: o["dist"])
        x = [o["dist"] for o in ours]
        y = [o["head"] for o in ours]
        ax.plot(x, y, "-", color=C_PROP, lw=1.2, zorder=4, alpha=0.9)
        sc = ax.scatter(x, y, c=[o["w_move"] for o in ours], cmap="Blues_r",
                        s=26, edgecolor=C_PROP, linewidth=0.5, zorder=5, vmin=-0.004)
        for o in ours:
            if o["w_move"] in (0.0, 0.001, 0.004, 0.02, 0.05):
                ax.annotate(rf"$w$={o['w_move']:g}", (o["dist"], o["head"]),
                            fontsize=6.1, xytext=(4, -1), textcoords="offset points",
                            color="0.25")
        for key, (mk, col, lab) in marks.items():
            b = r["baselines"][key]
            ax.scatter([b["dist"]], [b["head"]], marker=mk, s=34, color=col,
                       zorder=6, edgecolor="white", linewidth=0.5)
        ax.set_xlabel("UAV flight distance / m")
        ax.set_xlim(-12, 480)
        ax.set_ylim(0.0, 1.45)
        ax.grid(alpha=0.25, lw=0.5)
        tag(ax, f"({'ab'[list(axes).index(ax)]}) {CASES[case]}", x=0.985, y=0.975)
        axes[list(axes).index(ax)].texts[-1].set_ha("right")
    axes[0].set_ylabel(r"Heading RMSE / $^\circ$")
    axes[1].set_yticklabels([])

    handles = [Line2D([], [], color=C_PROP, marker="o", ms=3.4, lw=1.1, label="Proposed (sweep over $w$)")]
    handles += [Line2D([], [], color=c, marker=m, ls="none", ms=4.2, mec="white", mew=0.5, label=l)
                for m, c, l in marks.values()]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, -0.115), handlelength=1.5, columnspacing=1.4)
    save(fig, "fig5_pareto.png")


# =======================  Fig. 6  =======================
def fig6() -> None:
    j = json.loads((CODE / "joint_lut.json").read_text(encoding="utf-8"))

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.95),
                             gridspec_kw=dict(width_ratios=[1.25, 1.0], wspace=0.34))
    ax = axes[0]
    for case, col, mk in (("alley_center", C_PROP, "o"), ("canopy_edge", C_RAND, "s")):
        ours = sorted(j[case]["ours"], key=lambda o: o["kappa"])
        k = [o["kappa"] for o in ours]
        ax.plot(k, [o["head"] for o in ours], mk + "-", color=col, ms=3.4, lw=1.25,
                label=f"heading, {CASES[case]}")
        b = j[case]["baselines"]["axis10"]
        ax.axhline(b["head"], color=col, ls=(0, (4, 2.5)), lw=0.8, alpha=0.75)
    ax.text(0.10, 0.665, r"axial baseline 0.72$^\circ$", fontsize=6.4, color=C_PROP, ha="left")
    ax.set_xlabel(r"position weight $\kappa$")
    ax.set_ylabel(r"Heading RMSE / $^\circ$")
    ax.set_xlim(-0.03, 0.65); ax.set_ylim(0.2, 0.85)

    ax2 = ax.twinx()
    for case, col, mk in (("alley_center", C_PROP, "o"), ("canopy_edge", C_RAND, "s")):
        ours = sorted(j[case]["ours"], key=lambda o: o["kappa"])
        ax2.plot([o["kappa"] for o in ours], [o["pos"] for o in ours], mk + ":",
                 color=col, ms=2.8, lw=1.0, alpha=0.75, mfc="white")
    ax2.set_ylabel(r"Position RMSE / m", color="0.35")
    ax2.set_ylim(0.0, 0.09)
    ax2.tick_params(axis="y", colors="0.35")
    ax2.spines["right"].set_color("0.55")
    ax2.yaxis.label.set_color("0.35")
    ax.annotate(r"$\kappa=0.2$" + "\nplateau starts", xy=(0.2, 0.33), xytext=(0.30, 0.55),
                fontsize=6.6, color="0.25",
                arrowprops=dict(arrowstyle="->", lw=0.6, color="0.45",
                                connectionstyle="arc3,rad=-0.2"))
    h1 = [Line2D([], [], color=C_PROP, marker="o", ms=3.2, lw=1.1, label="heading (left)"),
          Line2D([], [], color="0.45", ls=(0, (4, 2.5)), lw=0.9, label="axial baseline (left)"),
          Line2D([], [], color="0.45", marker="o", ms=2.6, lw=0.9, ls=":", mfc="white",
                 label="position (right)")]
    ax.legend(handles=h1, loc="upper right", frameon=False, handlelength=1.6)
    tag(ax, "(a)")

    ax = axes[1]
    for case, col, mk in (("alley_center", C_PROP, "o"), ("canopy_edge", C_RAND, "s")):
        ours = sorted(j[case]["ours"], key=lambda o: o["kappa"])
        ax.plot([o["kappa"] for o in ours], [o["pvis"] for o in ours], mk + "-",
                color=col, ms=3.4, lw=1.25, label=CASES[case])
    ax.axvspan(0.0, 0.05, color="0.85", alpha=0.45, lw=0)
    ax.text(0.028, 0.70, "poor-visibility\nregime", fontsize=6.3, color="0.3", ha="center")
    ax.set_xlabel(r"position weight $\kappa$")
    ax.set_ylabel(r"mean $p_{\mathrm{vis}}$")
    ax.set_xlim(-0.03, 0.65); ax.set_ylim(0.2, 1.06)
    ax.legend(loc="lower right", frameon=False)
    tag(ax, "(b)")
    save(fig, "fig6_joint.png")


# =======================  Fig. 7  =======================
def fig7() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.8), sharey=True,
                             gridspec_kw=dict(wspace=0.08))
    style = {"follow10": (C_LAT, "^", "-", r"Constant distance, 10 m lateral"),
             "axis10": (C_AXIAL, "D", "-", r"Constant look-ahead, 10 m axial"),
             "ours": (C_PROP, "o", "-", "Proposed")}
    for ax, case in zip(axes, ("alley_center", "canopy_edge")):
        d = json.loads((CODE / f"metrics_lut_{case}_k0.2.json").read_text(encoding="utf-8"))
        for key, (col, mk, ls, lab) in style.items():
            rows = d["exp2"][key]
            x = [r["p_loss"] * 100 for r in rows]
            y = [r["head_rmse"] for r in rows]
            ax.plot(x, y, ls, color=col, marker=mk, ms=3.4, lw=1.25, label=lab)
            if key == "ours":
                ax.annotate(f"+{(y[-1]/y[0]-1)*100:.0f}%", (x[-1], y[-1]),
                            xytext=(-2, 6), textcoords="offset points",
                            fontsize=6.4, color=col, ha="right")
        ax.set_xlabel("packet loss / %")
        ax.set_xlim(-3, 53)
        ax.set_ylim(0.25, 1.45)
        ax.set_xticks([0, 10, 20, 35, 50])
        ax.grid(alpha=0.25, lw=0.5)
        tag(ax, f"({'ab'[list(axes).index(ax)]}) {CASES[case]}", x=0.985, y=0.975)
        axes[list(axes).index(ax)].texts[-1].set_ha("right")
    axes[0].set_ylabel(r"Heading RMSE / $^\circ$")
    axes[0].legend(loc="center", frameon=False, handlelength=1.6,
                   bbox_to_anchor=(0.52, 0.56))
    save(fig, "fig7_loss.png")


# =======================  Fig. 8  =======================
def fig8() -> None:
    data = json.loads((VIZ / "nees_series.json").read_text(encoding="utf-8"))
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.95),
                             gridspec_kw=dict(width_ratios=[1.30, 1.0], wspace=0.30))

    # ---- (a) NEES vs time, median and IQR over seeds, log scale
    ax = axes[0]
    case = "alley_center"
    naive = np.array(data["cases"][case]["naive"])
    ci = np.array(data["cases"][case]["ci"])
    t = np.arange(naive.shape[1]) * data["dt"]
    for arr, col, lab in ((naive, C_NAIVE, "naive fusion"), (ci, C_CI, "covariance intersection")):
        med = np.median(arr, axis=0)
        q1 = np.percentile(arr, 25, axis=0)
        q3 = np.percentile(arr, 75, axis=0)
        ax.fill_between(t, np.maximum(q1, 0.25), q3, color=col, alpha=0.20, lw=0)
        ax.plot(t, med, color=col, lw=1.0, label=lab)
    ax.axhline(data["cases"][case]["chi2_bound_95"], color="k", ls=(0, (4, 2.5)), lw=0.8)
    ax.text(3, data["cases"][case]["chi2_bound_95"] * 1.12, "95% bound (5.99)",
            fontsize=6.4, color="k")
    ax.set_yscale("log")
    ax.set_xlabel("time / s")
    ax.set_ylabel("NEES (2 DoF)")
    ax.set_xlim(0, t[-1]); ax.set_ylim(0.25, 40)
    ax.set_yticks([0.3, 1, 3, 10, 30])
    ax.set_yticklabels(["0.3", "1", "3", "10", "30"])
    ax.legend(loc="upper left", frameon=False, ncol=1, handlelength=1.6)
    ax.set_title(f"(a) {CASES[case]}, median and IQR of {data['n_seeds']} seeds",
                 loc="left", fontsize=7.6, pad=3)

    # ---- (b) exceedance rate
    ax = axes[1]
    xs = np.arange(2)
    w = 0.36
    for i, (key, col, lab) in enumerate((("naive", C_NAIVE, "naive"), ("ci", C_CI, "CI"))):
        vals = [data["cases"][c]["exceedance"][0 if key == "naive" else 1] * 100
                for c in ("alley_center", "canopy_edge")]
        bars = ax.bar(xs + (i - 0.5) * w, vals, width=w, color=col, label=lab,
                      edgecolor="white", lw=0.5)
        for b_, v in zip(bars, vals):
            ax.text(b_.get_x() + b_.get_width() / 2, v + 0.25, f"{v:.1f}%",
                    ha="center", va="bottom", fontsize=6.5)
    ax.axhline(5.0, color="k", ls=(0, (4, 2.5)), lw=0.8)
    ax.text(1.46, 5.3, "nominal 5%", fontsize=6.4, ha="right")
    ax.set_xticks(xs)
    ax.set_xticklabels(["alley", "edge"])
    ax.set_ylabel("steps above 95% bound / %")
    ax.set_ylim(0, 13.5)
    ax.legend(frameon=False, loc="upper right", bbox_to_anchor=(1.0, 1.0))
    ax.set_title("(b) exceedance rate", loc="left", fontsize=7.6, pad=3)
    save(fig, "fig8_nees.png")


def main() -> None:
    global DPI
    ap = argparse.ArgumentParser()
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args()
    DPI = a.dpi
    todo = {"3": fig3, "4": fig4, "5": fig5, "6": fig6, "7": fig7, "8": fig8}
    for k in (a.only or todo):
        if k in todo:
            print(f"[fig{k}] {DPI} dpi")
            todo[k]()


if __name__ == "__main__":
    main()
