# -*- coding: utf-8 -*-
"""
图 2：航向可辨识度的几何结构（支撑 §2.2 三个命题）
=================================================
(a) 固定站距 r=6 m 时，Fθθ 随视线方位的变化（极坐标）——cos² 双瓣，
    沿轴最强、正侧向最弱；两条曲线：r=6 m 与 r=2 m，展示"有限距离残差"。
(b) Fθθ 随站距 r 的变化（对数纵轴）——沿轴 ∝ 1/r²，正侧向下降快得多，
    并给出与解析渐近式 (L/2r)²/2 的对照。

用法：python fig2_identifiability.py
输出：../figures_viz/fig2_identifiability.png（300 dpi）
"""
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

OUT = Path(__file__).resolve().parent.parent / "figures_viz"
L, SB = 1.0, 0.008


def f_theta(r, ang, L=L, sb=SB):
    """无人机位于 (r cos a, r sin a)，车体 θ=0，标志点 (±L/2, 0)。"""
    u = np.array([r * math.cos(ang), r * math.sin(ang)])
    tot = 0.0
    for s, m in ((1.0, np.array([L / 2, 0.0])), (-1.0, np.array([-L / 2, 0.0]))):
        d = m - u
        rr = float(np.linalg.norm(d))
        b = math.atan2(d[1], d[0])
        tot += (s * (L / 2) / rr * math.cos(b)) ** 2
    return tot / sb ** 2


def main():
    fig = plt.figure(figsize=(6.85, 3.05), dpi=300)
    ax1 = fig.add_subplot(1, 2, 1, projection="polar")
    ax2 = fig.add_subplot(1, 2, 2)

    ang = np.linspace(-np.pi, np.pi, 361)
    for r, colr, ls in ((6.5, "#2c7fb8", "-"), (2.0, "#d1495b", "--")):
        vals = np.array([f_theta(r, a) for a in ang])
        vals = vals / vals[0]          # 归一化到各自轴向值，只比较形状
        ax1.plot(ang, vals, color=colr, lw=1.8, ls=ls,
                 label=f"$r$ = {r:g} m  (waist {vals[90]:.1%})")
    ax1.set_theta_zero_location("E")
    ax1.set_theta_direction(1)
    ax1.set_rlabel_position(112)
    ax1.set_rticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax1.tick_params(axis="y", labelsize=6.0)
    ax1.set_title("(a)  Normalised identifiability (shape only)", fontsize=7.2, pad=8)
    ax1.tick_params(labelsize=6.4)
    ax1.legend(loc="lower left", bbox_to_anchor=(0.52, -0.14), fontsize=6.0, frameon=False)
    rr = np.linspace(1.6, 16, 300)
    ax_axis = np.array([f_theta(x, 0.0) for x in rr])
    ax_lat = np.array([f_theta(x, np.pi / 2) for x in rr])
    ax2.loglog(rr, ax_axis, color="#2c7fb8", lw=1.9, label="axial station (0°)")
    ax2.loglog(rr, ax_lat, color="#d1495b", lw=1.9, label="lateral station (90°)")
    ax2.loglog(rr, 0.5 * (L / (2 * rr)) ** 2 * ax_axis, color="#d1495b", lw=1.0, ls=":",
               label="asymptote: $\\frac{1}{2}(L/2r)^2\\,F^{ax}$")
    ax2.set_xlabel("station distance $r$ / m", fontsize=6.6)
    ax2.set_ylabel("$F_{\\theta\\theta}$ (rad$^{-2}$)", fontsize=6.6)
    ax2.grid(alpha=0.3, which="major")
    ax2.set_xticks([2, 3, 5, 8, 12, 16])
    ax2.xaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
    ax2.xaxis.set_minor_formatter(mticker.NullFormatter())
    ax2.tick_params(labelsize=6.6)
    ax2.legend(fontsize=6.6, frameon=False)
    ax2.set_title("(b)  Information versus station distance", fontsize=7.2)
    ax2.annotate("lateral / axial:\n4.6% ($r$=2 m), 0.17% ($r$=12 m)",
                 xy=(6.0, 2.6e2), fontsize=6.0, color="#1f2933", ha="left")

    fig.tight_layout()
    out = OUT / "fig2_identifiability.png"
    fig.savefig(out, dpi=600, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"[fig2] -> {out}")


if __name__ == "__main__":
    main()
