"""Generate the three publication figures from the saved P1 main experiment."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator, FuncFormatter, NullFormatter
import numpy as np


ROOT = Path(__file__).resolve().parent
SCENES = {"center": "Alley centre", "edge": "Canopy edge"}
SCENE_STYLE = {"center": ("#0072B2", "o"), "edge": ("#D55E00", "s")}
POLICIES = {
    "lut": ("Lookup table", "#0072B2", "o", "-"),
    "gaussian": ("Bounded Gaussian", "#D55E00", "^", "--"),
    "uniform": ("Uniform prior", "#7557A6", "D", ":"),
}
INK = "#202730"
MUTED = "#59646F"



def smooth_curve(y, per_seg=14):
    """等间隔序列的 Catmull-Rom 插值，返回 (索引, 平滑值)。

    曲线经过每个实测点，只把点与点之间画成平滑过渡而非直线段，
    避免"用直线连接相邻点"，也避免引入拟合模型。
    """
    y = np.asarray(y, dtype=float)
    if len(y) < 3:
        return np.arange(len(y), dtype=float), y
    ye = np.concatenate([[y[0]], y, [y[-1]]])
    xs, ys = [], []
    for i in range(len(y) - 1):
        p0, p1, p2, p3 = ye[i], ye[i + 1], ye[i + 2], ye[i + 3]
        t = np.linspace(0.0, 1.0, per_seg, endpoint=False)
        ys.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                         + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                         + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
        xs.append(i + t)
    xs.append(np.array([len(y) - 1.0]))
    ys.append(np.array([y[-1]]))
    return np.concatenate(xs), np.concatenate(ys)


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


def save(fig, folder, number):
    fig.savefig(folder / f"Figure{number}.png", dpi=600)
    fig.savefig(folder / f"Figure{number}.pdf")
    plt.close(fig)


def log_ticks(ax, ticks):
    ax.set_yscale("log")
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g}"))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.tick_params(axis="y", which="minor", length=0)


def visibility_figure(study, input_dir, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(6.53, 5.38), sharex=True, sharey=True)
    fig.subplots_adjust(left=.125, right=.856, bottom=.103, top=.925, wspace=.13, hspace=.24)
    with np.load(input_dir / "priors.npz") as maps:
        grid = maps["grid"]
        x, y = np.meshgrid(grid, grid)
        for i, key in enumerate(SCENES):
            fit = study["scenes"][key]
            amp, sx, sy = fit["fit"]
            surfaces = (maps[key], amp * np.exp(-(x / sx)**2 - (y / sy)**2))
            for j, (ax, surface) in enumerate(zip(axes[i], surfaces)):
                panel = "abcd"[2*i+j]
                label = "Centre" if key == "center" else "Edge"
                method = "Lookup table" if j == 0 else "Gaussian fit"
                ax.set_title(f"({panel})  {label} · {method}", loc="left", pad=6)
                heat = ax.pcolormesh(grid, grid, surface, cmap="viridis",
                                     vmin=0, vmax=1, shading="nearest")
                ax.set(xlim=(-15, 15), ylim=(-15, 15), aspect="equal",
                       xticks=[-10, 0, 10], yticks=[-10, 0, 10])
                ax.contour(x, y, surface, [0.5], colors="white", linewidths=.65,
                           linestyles="dashed")
                ax.plot(0, 0, marker="+", ms=7, mew=1.0, color="#FAFAFA")
                # BE 指南：回归系数与符号说明不得写在图形区域内，p0 与 R² 改由题注给出
        for ax in axes[:, 0]:
            ax.set_ylabel("Across-row offset (m)")
        fig.supxlabel("Along-row offset (m)", y=.025, x=.49, fontsize=8.5)
        cax = fig.add_axes([.883, .235, .024, .54])
        cb = fig.colorbar(heat, cax=cax, ticks=np.linspace(0, 1, 6))
        cb.set_label("Pair visibility before camera gate")
        cb.outline.set_linewidth(.55)
    save(fig, output_dir, 1)


def tradeoff_figure(study, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(6.53, 5.4), sharex=True, sharey="col")
    fig.subplots_adjust(left=.125, right=.975, bottom=.10, top=.83, wspace=.26, hspace=.30)
    for i, (key, label) in enumerate(SCENES.items()):
        rows = study["scenes"][key]["results"]
        fixed = np.array([r["mean"] for r in rows if r["model"] == "fixed"])
        best = fixed[fixed[:, 1].argmin()]
        for j, (ax, index) in enumerate(zip(axes[i], (1, 0))):
            panel = "abcd"[2*i+j]
            ax.set_title(f"({panel})  {label}", loc="left", pad=5)
            ax.scatter(fixed[:, 2], fixed[:, index], s=15, facecolors="#DFE4E9",
                       edgecolors="#697680", linewidths=.55, zorder=2)
            for model, (_, color, marker, line) in POLICIES.items():
                settings = [r for r in rows if r["model"] == model]
                points = np.array([r["mean"] for r in settings])
                ax.plot(points[:, 2], points[:, index], color=color, ls="",
                        marker=marker, ms=4.6, zorder=3)
            ax.scatter(best[2], best[index], marker="*", s=92, facecolors=INK,
                       edgecolors="white", linewidths=.55, zorder=5)
            lookup = next(r["mean"] for r in rows if r["model"] == "lut" and r["weight"] == .001)
            ax.scatter(lookup[2], lookup[index], s=63, facecolors="none",
                       edgecolors=POLICIES["lut"][1], linewidths=.85, zorder=5)
            log_ticks(ax, ([.3, .5, 1, 2, 5, 10, 20] if j == 0
                           else [.01, .03, .1, .3, 1, 3]))
            ax.set(ylabel="Heading RMSE (°)" if j == 0 else "Position RMSE (m)",
                   xlim=(-3, 115), xticks=np.arange(0, 101, 20))
            ax.set_axisbelow(True)
    fig.supxlabel("Cumulative UAV flight distance (m)", y=.025, fontsize=8.5)
    handles = [Line2D([], [], marker="o", ls="", ms=4.2, mfc="#DFE4E9", mec="#697680",
                      label="Fixed station"),
               Line2D([], [], marker="*", ls="", ms=8, color=INK, label="Lowest heading RMSE among fixed"),
               *(Line2D([], [], marker=m, ls="", color=c, ms=4.6, label=n)
                 for n, c, m, ls in POLICIES.values()),
               Line2D([], [], marker="o", ls="", ms=6, mfc="none", mec=POLICIES["lut"][1],
                      label=r"Lookup table at $w=0.001$")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.52, .974), ncol=3,
               frameon=False, fontsize=7.4, columnspacing=1.1, handlelength=2.1)
    save(fig, output_dir, 2)


def azimuth_figure(study, output_dir):
    fig, axes = plt.subplots(2, 1, figsize=(6.53, 4.45), sharex=True)
    fig.subplots_adjust(left=.125, right=.970, bottom=.195, top=.866, hspace=.34)
    for key, label in SCENES.items():
        rows = [r for r in study["scenes"][key]["results"]
                if r["model"] == "fixed" and r["radius"] == 2.0]
        rows.sort(key=lambda r: r["angle"])
        angles = np.array([r["angle"] for r in rows])
        samples = np.array([r["samples"] for r in rows])
        color, marker = SCENE_STYLE[key]
        seeds = samples.shape[1]
        draws = np.random.default_rng(91).integers(seeds, size=(5000, seeds))
        for j, (ax, index, scale) in enumerate(zip(axes, (1, 3), (1, 100))):
            y = scale * samples[:, :, index]
            mean = y.mean(axis=1)
            low, high = np.quantile(y[:, draws].mean(axis=2), [.025, .975], axis=1)
            ax.errorbar(angles, mean, yerr=[mean-low, high-mean], color=color,
                        marker=marker, ls="none", ms=4.6,
                        elinewidth=.8, capsize=2.2, label=label)
            xi, yi = smooth_curve(mean)
            ax.plot(np.interp(xi, np.arange(len(angles), dtype=float), angles.astype(float)), yi,
                    color=color, lw=1.05, ls="--" if key == "edge" else "-",
                    alpha=.9, zorder=2, solid_capstyle="round")
    for ax in axes:
        ax.axvspan(30, 150, color="#F9EEE5", lw=0, zorder=0)
        ax.set_axisbelow(True)
    axes[0].set_title("(a)  Heading error", loc="left", pad=5)
    axes[1].set_title("(b)  Paired marker detections", loc="left", pad=5)
    log_ticks(axes[0], [.3, .5, 1, 2, 5, 10])
    axes[0].set(ylabel="Heading RMSE (°)", ylim=(.27, 18))
    axes[1].set(ylabel="Pair detection (%)", ylim=(-4, 108), yticks=[0, 25, 50, 75, 100],
                xlim=(-11, 326), xlabel="Requested station azimuth in the vehicle frame")
    angles = np.arange(0, 360, 45)
    directions = {0: "Forward", 90: "Left", 180: "Rear", 270: "Right"}
    axes[1].set_xticks(angles, [f"{a}°" + ("\n" + directions[a] if a in directions else "") for a in angles])
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center",
               bbox_to_anchor=(.53, .973), ncol=2, frameon=False)
    save(fig, output_dir, 3)


def path_figure(summary, output_dir):
    """Lateral path offset: the plateau, the climb by the edge, and the azimuth switch."""
    rows = sorted(summary["rows"], key=lambda r: r["offset"])
    x = np.array([r["offset"] for r in rows])
    best = np.array([r["best_heading"] for r in rows])
    ang = np.array([r["best_angle"] for r in rows], float)

    plt.rcParams.update({"font.size": 9.6, "axes.labelsize": 9.6,
                         "axes.titlesize": 10.0, "xtick.labelsize": 8.8,
                         "ytick.labelsize": 8.8})
    fig, axes = plt.subplots(1, 2, figsize=(6.53, 3.05), sharex=True,
                             gridspec_kw={"width_ratios": [1.32, 1]})
    fig.subplots_adjust(left=.105, right=.975, bottom=.175, top=.755, wspace=.27)

    for ax in axes:
        ax.axvspan(-.4, .4, color="#EDF0F3", zorder=0)
        for xx in (-.8, .8):
            ax.axvline(xx, color=MUTED, lw=.7, ls=(0, (4, 3)), zorder=1)
        ax.set_axisbelow(True)
        ax.set_xlim(-1.44, 1.44)
        ax.set_xticks([-1.2, -.8, -.4, 0, .4, .8, 1.2])

    ax = axes[0]
    ax.plot(x, best, color="#0072B2", lw=1.5, marker="o", ms=3.4, zorder=4,
            solid_capstyle="round")
    ax.set_ylim(.30, .78)
    ax.set_yticks([.3, .4, .5, .6, .7])
    ax.set(ylabel="Heading RMSE (°)")
    ax.set_title("(a)  Cost of leaving the axis", loc="left", pad=6)
    ax.set_ylim(.30, .80)
    ax.annotate("+28%", xy=(-.80, .468), xytext=(-1.16, .725), fontsize=8.6,
                color=INK, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", lw=.7, color=MUTED,
                                shrinkA=2, shrinkB=4))
    ax.annotate("+43%", xy=(.80, .520), xytext=(1.02, .725), fontsize=8.6,
                color=INK, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", lw=.7, color=MUTED,
                                shrinkA=2, shrinkB=4))

    ax = axes[1]
    ax.step(x, ang, where="mid", color="#D55E00", lw=1.3, zorder=3)
    ax.plot(x, ang, "o", color="#D55E00", ms=3.3, zorder=4)
    ax.set_yticks([0, 135, 225])
    ax.set_yticklabels(["0°", "135°", "225°"])
    ax.set_ylim(-45, 268)
    ax.set(ylabel="Best azimuth")
    ax.set_title("(b)  Which azimuth wins", loc="left", pad=6)
    # 与 (a) 共享横轴，刻度只标一次，否则窄 panel 里数字会挤成一团
    ax.tick_params(axis="x", labelbottom=False)

    fig.supxlabel("Lateral path offset from the alley axis (m)", y=.035, fontsize=8.5)
    save(fig, output_dir, 4)


def crown_figure(study, input_dir, output_dir):
    """Compare the same LUT motion weight across the crown-width sweep."""
    widths = [1.2, 1.5, 1.8, 2.0]
    means, lows, highs = [], [], []
    rng = np.random.default_rng(91)
    for width in widths:
        records = study if width == 1.2 else json.loads(
            (input_dir.parent / f"results_crown_b{width:.1f}" / "study.json").read_text(encoding="utf-8"))
        rows = records["scenes"]["edge"]["results"]
        fixed = min((r for r in rows if r["model"] == "fixed"), key=lambda r: r["mean"][1])
        lut = next(r for r in rows if r["model"] == "lut" and r["weight"] == .001)
        for r in (fixed, lut):
            s = np.array(r["samples"])                    # (30 seeds, 5 metrics)
            idx = rng.integers(s.shape[0], size=(5000, s.shape[0]))
            boot = s[idx].mean(axis=1)                    # (5000, 5)
            lo, hi = np.quantile(boot, [.025, .975], axis=0)
            means.append(s.mean(axis=0))
            lows.append(s.mean(axis=0) - lo)
            highs.append(hi - s.mean(axis=0))
    means = np.array(means).reshape(4, 2, 5)
    lows = np.array(lows).reshape(4, 2, 5)
    highs = np.array(highs).reshape(4, 2, 5)
    fig, axes = plt.subplots(1, 2, figsize=(6.53, 3.2), sharex=True)
    fig.subplots_adjust(left=.115, right=.97, bottom=.17, top=.76, wspace=.32)
    labels = ["Best-heading fixed station", r"Lookup table ($w=0.001$)"]
    for j, (ax, metric, title, ylabel) in enumerate(zip(
        axes, [1, 2], ["Heading accuracy", "Flight distance"], ["Heading RMSE (°)", "UAV flight distance (m)"]
    )):
        for method, (color, marker, _ls) in enumerate([(INK, "*", ""), ("#0072B2", "o", "")]):
            ax.errorbar(widths, means[:, method, metric],
                        yerr=[lows[:, method, metric], highs[:, method, metric]],
                        color=color, marker=marker, ls="none",
                        ms=6.5 if method == 0 else 4.8, elinewidth=.8, capsize=2.2,
                        label=labels[method])
            xi, yi = smooth_curve(means[:, method, metric])
            ax.plot(np.interp(xi, np.arange(len(widths), dtype=float), np.array(widths, float)), yi,
                    color=color, lw=1.05, alpha=.9, zorder=2, solid_capstyle="round")
        ax.set_title(f"({'ab'[j]})  {title}", loc="left", pad=6)
        ax.set(ylabel=ylabel, xticks=widths, xlim=(1.14, 2.06))
        ax.set_ylim(bottom=0)
        ax.set_axisbelow(True)
    fig.supxlabel("Across-row crown semi-axis (m)", y=.035, fontsize=8.5)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center",
               bbox_to_anchor=(.53, .956), ncol=2, frameon=False, fontsize=8)
    save(fig, output_dir, 5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results")
    parser.add_argument("--output", type=Path, default=ROOT / "figures")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    study = json.loads((args.input / "study.json").read_text(encoding="utf-8"))
    configure()
    visibility_figure(study, args.input, args.output)
    tradeoff_figure(study, args.output)
    azimuth_figure(study, args.output)
    path_summary = args.input.parent / "results_path_offset" / "summary.json"
    if path_summary.exists():
        path_figure(json.loads(path_summary.read_text(encoding="utf-8")), args.output)
    crown_figure(study, args.input, args.output)


if __name__ == "__main__":
    main()
