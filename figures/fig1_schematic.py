# -*- coding: utf-8 -*-
"""
图 1 示意图（论文正文用）
=========================
设计目标：读者 3 秒看懂"几何最优（沿行）与可见最优（跨行）方向正交"。
不是写实渲染——写实交给 Blender（见 tools/BLENDER_README.md）。

(a) 俯视平面图：树行、通道、UGV 双标志、两个 UAV 站位、可见/被遮挡视线、
    以及两个方向的可辨识度说明（cos² 结构）。
(b) 标定后的可见率场（来自 vis_lut.npz 的 alley_center 工况），标出两站位与 0.5 等值线。

所有几何与 E1 一致（行距 4 m、株距 2 m、冠层 1.2 m 半径、标志基线 1.0 m、站位 z=6 m）。
用法：python fig1_schematic.py
输出：../figures_viz/fig1_schematic.png（300 dpi）
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle, FancyArrowPatch, Circle
from matplotlib.lines import Line2D

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / "tools"))
from gen_orchard import CFG, SEED, build_tree_centers  # noqa: E402

OUT = _HERE.parent / "figures_viz"
LUT = _HERE.parents[1] / "vis_lut.npz"

C_CANOPY = "#4e8c58"
C_CANOPY_EDGE = "#2f5d38"
C_GROUND = "#f6f4ec"
C_ALLEY = "#e9e3cf"
C_UGV = "#2c3e57"
C_MARKER = "#e07b22"
C_OK = "#1f9d55"
C_BAD = "#d1495b"
C_TXT = "#1f2933"


def draw_quad(ax, xy, color, label=None, r=0.55, label_dy=1.0):
    x, y = xy
    ax.add_patch(Circle((x, y), r * 0.42, fc="white", ec=color, lw=1.6, zorder=6))
    for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        ax.plot([x + dx * r * 0.25, x + dx * r * 0.95],
                [y + dy * r * 0.25, y + dy * r * 0.95],
                color=color, lw=1.6, solid_capstyle="round", zorder=6)
    if label:
        ax.text(x, y + label_dy, label, ha="center", va="bottom",
                fontsize=7.4, color=color, weight="bold", zorder=7)


def panel_a(ax):
    """Plan view, cropped to the alley plus one row each side (legible at 6.85 in)."""
    cfg = dict(CFG)
    trees = build_tree_centers(cfg, np.random.default_rng(SEED))
    a = cfg["canopy_a"]

    ax.set_facecolor(C_GROUND)
    ax.add_patch(Rectangle((-22, -2), 44, 4, fc=C_ALLEY, ec="none", zorder=0))
    for t in trees:
        ax.add_patch(Ellipse((t[0], t[1]), 2 * a, 2 * a, fc=C_CANOPY, alpha=0.55,
                             ec=C_CANOPY_EDGE, lw=0.5, zorder=2))

    # ---- UGV and the two marker boards
    ax.add_patch(Rectangle((-0.25, -0.18), 0.5, 0.36, fc=C_UGV, ec="none", zorder=5))
    for sx in (0.5, -0.5):
        ax.add_patch(Rectangle((sx - 0.035, -0.13), 0.07, 0.26, fc=C_MARKER, ec="none", zorder=6))
    ax.annotate("", xy=(0.5, -0.62), xytext=(-0.5, -0.62),
                arrowprops=dict(arrowstyle="<->", color=C_TXT, lw=0.7, shrinkA=0, shrinkB=0))
    ax.text(0.0, -0.75, "$L$ = 1.0 m", ha="center", va="top", fontsize=6.0, color=C_TXT)
    ax.add_patch(FancyArrowPatch((0.32, 0.0), (1.10, 0.0), arrowstyle="-|>", mutation_scale=8,
                                 color=C_UGV, lw=0.9, zorder=6))
    ax.text(-0.30, 0.34, "UGV", fontsize=6.2, color=C_UGV, ha="right", va="bottom")

    # ---- stations and lines of sight
    st1, st2 = (2.0, 0.0), (0.0, 4.0)
    for m in (0.5, -0.5):
        ax.plot([st1[0], m], [st1[1], 0.0], color=C_OK, lw=1.5, alpha=0.95, zorder=4)
        ax.plot([st2[0], m], [st2[1], 0.0], color=C_BAD, lw=1.2, ls=(0, (3.5, 2.2)),
                alpha=0.95, zorder=4)
    draw_quad(ax, st1, C_OK, None, r=0.42)
    draw_quad(ax, st2, C_BAD, None, r=0.42)
    ax.text(st1[0] + 0.62, st1[1], "S1", fontsize=7.0, color=C_OK, va="center", weight="bold")
    ax.text(st2[0] + 0.60, st2[1], "S2", fontsize=7.0, color=C_BAD, va="center", weight="bold")

    # ---- row spacing arrow inside the alley
    ax.annotate("", xy=(-5.2, -2.0), xytext=(-5.2, 2.0),
                arrowprops=dict(arrowstyle="<->", color="#8a7f60", lw=0.8, shrinkA=0, shrinkB=0))
    ax.text(-4.95, 0.0, "4 m", ha="left", va="center", fontsize=6.0, color="#6f6450")

    ax.set_xlim(-8.6, 8.6)
    ax.set_ylim(-6.4, 6.4)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color("#c9c3b2")
    ax.set_title("(a)  Two candidate observation stations", fontsize=7.2, color=C_TXT, pad=4)


def panel_b(ax):
    d = np.load(LUT, allow_pickle=False)
    gx = d["GX_alley_center"][0, :]
    gy = d["GY_alley_center"][:, 0]
    p = d["pvis_alley_center"]

    im = ax.pcolormesh(gx, gy, p, cmap="viridis", shading="auto", vmin=0, vmax=1)
    ax.contour(gx, gy, p, levels=[0.5], colors="white", linewidths=0.9, linestyles="dashed")
    ax.plot([], [], color="white", lw=0.9, ls="dashed", label="$p_{vis}$ = 0.5")
    ax.legend(loc="lower right", fontsize=6.0, framealpha=0.85, handlelength=1.4)

    ax.plot(2.0, 0.0, marker="o", ms=5.4, mfc="none", mec=C_OK, mew=1.5, zorder=6)
    ax.plot(0.0, 4.0, marker="o", ms=5.4, mfc="none", mec=C_BAD, mew=1.5, zorder=6)
    ax.plot(0, 0, marker="s", ms=4.2, color="white", mec=C_TXT, mew=0.8, zorder=6)
    ax.text(0.45, -1.45, "UGV", color="#3a3a3a", fontsize=6.0, zorder=7)
    ax.text(2.55, 0.55, "S1", color=C_OK, fontsize=6.8, weight="bold", zorder=7)
    ax.text(0.55, 4.55, "S2", color=C_BAD, fontsize=6.8, weight="bold", zorder=7)

    ax.set_xlim(-12, 12); ax.set_ylim(-12, 12)
    ax.set_aspect("equal")
    ax.set_xlabel("along-row offset $x$ / m", fontsize=7.0)
    ax.set_ylabel("across-row offset $y$ / m", fontsize=7.0)
    ax.tick_params(labelsize=6.2)
    ax.set_title("(b)  Calibrated visibility field", fontsize=7.2, color=C_TXT, pad=4)
    cb = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label("$p_{vis}$", fontsize=7.0)
    cb.ax.tick_params(labelsize=6.2)


def main():
    fig, axes = plt.subplots(1, 2, figsize=(6.85, 3.55), dpi=300,
                             gridspec_kw=dict(width_ratios=[1.15, 1.0], wspace=0.30))
    panel_a(axes[0])
    panel_b(axes[1])
    fig.patch.set_facecolor("white")
    out = OUT / "fig1_schematic.png"
    fig.savefig(out, dpi=600, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"[fig1] -> {out}")


if __name__ == "__main__":
    main()
