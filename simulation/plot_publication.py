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
SCENES = {"center": "Alley center", "edge": "Canopy edge"}
SCENE_STYLE = {"center": ("#0072B2", "o"), "edge": ("#D55E00", "s")}
POLICIES = {
    "lut": ("Lookup table", "#0072B2", "o", "-"),
    "gaussian": ("Bounded Gaussian", "#D55E00", "^", "--"),
    "uniform": ("Uniform prior", "#7557A6", "D", ":"),
}
INK = "#202730"
MUTED = "#59646F"


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
        center = int(np.flatnonzero(np.isclose(grid, 0))[0])
        for i, key in enumerate(SCENES):
            fit = study["scenes"][key]
            amp, sx, sy = fit["fit"]
            surfaces = (maps[key], amp * np.exp(-(x / sx)**2 - (y / sy)**2))
            for j, (ax, surface) in enumerate(zip(axes[i], surfaces)):
                panel = "abcd"[2*i+j]
                label = "Center" if key == "center" else "Edge"
                method = "Lookup table" if j == 0 else "Gaussian fit"
                ax.set_title(f"({panel})  {label} · {method}", loc="left", pad=6)
                heat = ax.pcolormesh(grid, grid, surface, cmap="viridis",
                                     vmin=0, vmax=1, shading="nearest")
                ax.set(xlim=(-15, 15), ylim=(-15, 15), aspect="equal",
                       xticks=[-10, 0, 10], yticks=[-10, 0, 10])
                ax.contour(x, y, surface, [0.5], colors="white", linewidths=.65,
                           linestyles="dashed")
                ax.plot(0, 0, marker="+", ms=7, mew=1.0, color="#FAFAFA")
                stamp = (f"$p_0$ = {surface[center, center]:.3f}" if j == 0
                         else f"$R^2$ = {fit['r2']:.3f}")
                ax.text(.035, .055, stamp, transform=ax.transAxes, color="white",
                        fontsize=8, bbox=dict(facecolor="#253043", edgecolor="none", pad=2.5))
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
                ax.plot(points[:, 2], points[:, index], color=color, ls=line, marker=marker,
                        lw=1.05, ms=4, zorder=3)
            ax.scatter(best[2], best[index], marker="*", s=92, facecolors=INK,
                       edgecolors="white", linewidths=.55, zorder=5)
            lookup = next(r["mean"] for r in rows if r["model"] == "lut" and r["weight"] == .001)
            ax.scatter(lookup[2], lookup[index], s=63, facecolors="none",
                       edgecolors=POLICIES["lut"][1], linewidths=.85, zorder=5)
            log_ticks(ax, ([.3, .5, 1, 2, 5, 10, 20] if j == 0
                           else [.01, .03, .1, .3, 1, 3]))
            ax.set(ylabel="Heading RMSE (°)" if j == 0 else "Position RMSE (m)",
                   xlim=(-3, 115), xticks=np.arange(0, 101, 20))
            ax.grid(axis="y", color="#DFE5E9", linewidth=.55)
            ax.set_axisbelow(True)
    fig.supxlabel("Cumulative UAV flight distance (m)", y=.025, fontsize=8.5)
    handles = [Line2D([], [], marker="o", ls="", ms=4.2, mfc="#DFE4E9", mec="#697680",
                      label="Fixed station"),
               Line2D([], [], marker="*", ls="", ms=8, color=INK, label="Lowest heading RMSE among fixed"),
               *(Line2D([], [], marker=m, ls=ls, color=c, ms=4, label=n)
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
                        marker=marker, ls="-" if key == "center" else "--", ms=4.2,
                        lw=1.25, elinewidth=.75, capsize=2, label=label)
    for ax in axes:
        ax.axvspan(30, 150, color="#F9EEE5", lw=0, zorder=0)
        ax.grid(axis="y", color="#DFE5E9", linewidth=.55)
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


def crown_figure(study, input_dir, output_dir):
    """Compare the same LUT motion weight across the crown-width sweep."""
    widths = [1.2, 1.5, 1.8, 2.0]
    means = []
    for width in widths:
        records = study if width == 1.2 else json.loads(
            (input_dir.parent / f"results_crown_b{width:.1f}" / "study.json").read_text(encoding="utf-8"))
        rows = records["scenes"]["edge"]["results"]
        fixed = min((r for r in rows if r["model"] == "fixed"), key=lambda r: r["mean"][1])
        lut = next(r for r in rows if r["model"] == "lut" and r["weight"] == .001)
        means.append([fixed["mean"], lut["mean"]])
    means = np.array(means)
    fig, axes = plt.subplots(1, 2, figsize=(6.53, 3.2), sharex=True)
    fig.subplots_adjust(left=.115, right=.97, bottom=.17, top=.76, wspace=.32)
    labels = ["Best-heading fixed station", r"Lookup table ($w=0.001$)"]
    for j, (ax, metric, title, ylabel) in enumerate(zip(
        axes, [1, 2], ["Heading accuracy", "Flight distance"], ["Heading RMSE (°)", "UAV flight distance (m)"]
    )):
        for method, (color, marker, ls) in enumerate([(INK, "*", "--"), ("#0072B2", "o", "-")]):
            ax.plot(widths, means[:, method, metric], color=color, marker=marker,
                    ls=ls, lw=1.2, ms=6 if method == 0 else 4.5, label=labels[method])
        ax.set_title(f"({'ab'[j]})  {title}", loc="left", pad=6)
        ax.set(ylabel=ylabel, xticks=widths, xlim=(1.14, 2.06))
        ax.set_ylim(bottom=0)
        ax.grid(axis="y", color="#DFE5E9", linewidth=.55)
        ax.set_axisbelow(True)
    fig.supxlabel("Across-row crown semi-axis (m)", y=.035, fontsize=8.5)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center",
               bbox_to_anchor=(.53, .956), ncol=2, frameon=False, fontsize=8)
    save(fig, output_dir, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results")
    parser.add_argument("--output", type=Path, default=ROOT / "submission" / "figures_publication")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    study = json.loads((args.input / "study.json").read_text(encoding="utf-8"))
    configure()
    visibility_figure(study, args.input, args.output)
    tradeoff_figure(study, args.output)
    azimuth_figure(study, args.output)
    crown_figure(study, args.input, args.output)


if __name__ == "__main__":
    main()
