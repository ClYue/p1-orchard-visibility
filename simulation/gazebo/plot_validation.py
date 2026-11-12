"""Gazebo 验证图：与 plot_publication.py 同一套版式与配色。

(a) 站位级一致：模型透射率 对 渲染可见比，按半径分色。
(b) 概率校准：按预测分箱的平均透射率 对 实测双标记检出率。

两面板都用散点 + 虚线参考，不引入原图没有的元素（无网格、无柱状、无图内注释）。
"""

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "figures"

# 与 make_world.py 一致：相机 6.0 m、标记球心 0.3 m，120° 全视场 -> 足迹半径 5.7*tan60
CAM_Z, MARKER_Z, HALF_FOV_DEG = 6.0, 0.30, 60.0
HALF_FOV_RADIUS = (CAM_Z - MARKER_Z) * np.tan(np.deg2rad(HALF_FOV_DEG))

INK = "#202730"
MUTED = "#59646F"
# 与 plot_publication.py 相同的 Okabe-Ito 系配色
SERIES = [
    (2.0, INK, "o"),
    (3.0, "#0072B2", "s"),
    (4.5, "#D55E00", "^"),
    (6.5, "#009E73", "D"),
    (9.0, "#7557A6", "v"),
]


def configure():
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 8.5, "axes.labelsize": 8.5,
        "axes.titlesize": 9, "axes.titleweight": "bold", "axes.labelcolor": INK,
        "text.color": INK, "xtick.color": INK, "ytick.color": INK,
        "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
        "axes.edgecolor": MUTED, "axes.linewidth": 0.65,
        "axes.spines.top": False, "axes.spines.right": False,
        "xtick.major.size": 3, "ytick.major.size": 3,
        "pdf.fonttype": 42, "svg.fonttype": "none",
        "savefig.facecolor": "white",
    })


def load():
    """只取相机足迹内的站位。

    门限是**水平**半径 5.7*tan60 = 9.873 m（§2.1 的 camera gate），
    CSV 里的 d3d 是三维斜距，必须先投影回水平面再比。直接拿 d3d 比会把
    水平 9.0 m 的 8 个站位误删，样本变成 32 个（正确为 40 个）。
    """
    rows = []
    with (HERE / "visibility_gazebo.csv").open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "tag" else float(v)) for k, v in r.items()})
    drop = CAM_Z - MARKER_Z
    for r in rows:
        r["horiz"] = (r["d3d"] ** 2 - drop ** 2) ** 0.5
    return [r for r in rows if r["horiz"] <= HALF_FOV_RADIUS]


def panel_scatter(ax, p, v, rad):
    for radius, color, marker in SERIES:
        m = rad == radius
        if not m.any():
            continue
        ax.scatter(p[m], v[m], s=30, marker=marker, facecolor="none",
                   edgecolor=color, linewidth=1.1, alpha=.9, zorder=3)
    ax.plot([0, 1], [0, 1], ls=(0, (3, 2)), lw=.8, color="#A7B0B8", zorder=1)
    ax.set(xlim=(-.04, 1.04), ylim=(-.05, 1.08),
           xticks=[0, .25, .5, .75, 1], yticks=[0, .25, .5, .75, 1],
           xlabel="Predicted transmittance", ylabel="Rendered visible fraction")
    ax.set_title("(a)  Station-level agreement", loc="left", pad=6)
    handles = [Line2D([], [], marker=mk, ls="", ms=4.4, mfc="none", mec=c,
                      mew=1.1, label="r = %g m" % radius)
               for radius, c, mk in SERIES if (rad == radius).any()]
    ax.legend(handles=handles, fontsize=7, frameon=False, loc="lower right",
              handletextpad=.25, borderaxespad=.1, labelspacing=.3)


def panel_calibration(ax, rows):
    """按站位半径对比「模型判为可检出」与「渲染实际双标记可见」的比例。

    两组点横向错开 0.13 m，重合即一致；比按 p 分箱的校准图信息量大，
    且直接对应论文里"站位半径"这个变量。
    """
    radii = sorted({r["radius"] for r in rows})
    xs, pred, obs = [], [], []
    for rad in radii:
        g = [r for r in rows if r["radius"] == rad]
        xs.append(rad)
        pred.append(sum(1 for r in g if r["p_model"] >= 0.5) / len(g))
        obs.append(sum(r["both_seen"] for r in g) / len(g))
    xs = np.asarray(xs, dtype=float)
    ax.scatter(xs - 0.13, pred, marker="o", s=34, facecolor="none",
               edgecolor=INK, linewidth=1.2, zorder=3)
    ax.scatter(xs + 0.13, obs, marker="s", s=30, facecolor="none",
               edgecolor="#0072B2", linewidth=1.2, zorder=3)
    ax.set(xlim=(1.2, 9.8), ylim=(-.06, 1.10), xticks=radii,
           yticks=[0, .25, .5, .75, 1],
           xlabel="Station radius (m)",
           ylabel="Fraction with both markers visible")
    ax.set_title("(b)  Detection rate versus radius", loc="left", pad=6)
    handles = [
        Line2D([], [], marker="o", ls="", ms=4.6, mfc="none", mec=INK, mew=1.2,
               label="Model, transmittance ≥ 0.5"),
        Line2D([], [], marker="s", ls="", ms=4.2, mfc="none", mec="#0072B2", mew=1.2,
               label="Gazebo rendering"),
    ]
    ax.legend(handles=handles, fontsize=7, frameon=False, loc="upper right",
              handletextpad=.25, borderaxespad=.1)
    return xs, pred, obs


def avg_rank(v):
    """平均秩：visible_frac 有大量并列 1.0，序数秩会算错。"""
    o = np.argsort(v, kind="mergesort")
    r = np.empty(len(v), dtype=float)
    i = 0
    while i < len(o):
        j = i
        while j + 1 < len(o) and v[o[j + 1]] == v[o[i]]:
            j += 1
        r[o[i:j + 1]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return r


def main():
    configure()
    rows = load()
    p = np.array([r["p_model"] for r in rows])
    v = np.array([r["visible_frac"] for r in rows])
    det = np.array([r["both_seen"] for r in rows])
    rad = np.array([r["radius"] for r in rows])

    fig, axes = plt.subplots(1, 2, figsize=(6.53, 3.2))
    panel_scatter(axes[0], p, v, rad)
    xs, pred, obs = panel_calibration(axes[1], rows)
    fig.tight_layout()

    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "Figure5.png", dpi=600)
    fig.savefig(OUT / "Figure5.pdf")
    print("已保存", OUT / "Figure5.png")
    print("站位 %d 个（水平距 ≤ %.3f m）" % (len(rows), HALF_FOV_RADIUS))
    print("Pearson(p, 可见比) = %.4f" % np.corrcoef(p, v)[0, 1])
    print("Spearman(p, 可见比) = %.4f"
          % np.corrcoef(avg_rank(p), avg_rank(v))[0, 1])
    print("p>=0.5 双见 %d/%d | p<0.5 双见 %d/%d"
          % (det[p >= .5].sum(), int((p >= .5).sum()),
             det[p < .5].sum(), int((p < .5).sum())))
    print("半径 %s\n模型检出率 %s\n渲染检出率 %s"
          % (xs.tolist(), [round(x, 3) for x in pred], [round(x, 3) for x in obs]))


if __name__ == "__main__":
    main()
