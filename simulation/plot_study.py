"""Regenerate the manuscript figures from saved paired simulation results."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


RESULTS = Path(__file__).resolve().parent / "results"
NAMES = {"center": "Alley center", "edge": "Canopy edge"}
COLORS = {"lut": "#007e80", "gaussian": "#b35228", "uniform": "#604a9d"}
LABELS = {"lut": "Lookup table", "gaussian": "Bounded Gaussian", "uniform": "Uniform prior"}


def save(figure, name):
    figure.savefig(RESULTS / f"{name}.png", dpi=600, bbox_inches="tight", pad_inches=0.12)
    plt.close(figure)


def main():
    study = json.loads((RESULTS / "study.json").read_text(encoding="utf-8"))
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    with np.load(RESULTS / "priors.npz") as maps:
        grid = maps["grid"]
        x, y = np.meshgrid(grid, grid)
        fig, axes = plt.subplots(2, 2, figsize=(7.0, 6.0), layout="constrained")
        for row, (name, scene) in enumerate(study["scenes"].items()):
            a, sx, sy = scene["fit"]
            surfaces = (maps[name], a * np.exp(-(x / sx)**2 - (y / sy)**2))
            for col, surface in enumerate(surfaces):
                ax = axes[row, col]
                im = ax.pcolormesh(grid, grid, surface, vmin=0, vmax=1, cmap="viridis")
                ax.set(aspect="equal", xlabel="Along-row offset (m)", ylabel="Across-row offset (m)",
                       title=f"{NAMES[name]}: " + ("ray prior" if col == 0 else "Gaussian fit"))
        fig.colorbar(im, ax=axes, label="Probability of observing both markers", shrink=0.8)
        save(fig, "figure1_priors")

    fig, axes = plt.subplots(2, 2, figsize=(8.0, 6.3), layout="constrained")
    for row, (name, scene) in enumerate(study["scenes"].items()):
        fixed = np.array([r["mean"] for r in scene["results"] if r["model"] == "fixed"])
        for ax, idx, label in zip(axes[row], (1, 0), ("Heading RMSE (°)", "Position RMSE (m)")):
            ax.scatter(fixed[:, 2], fixed[:, idx], s=14, color="#666666", alpha=0.55, label="Fixed station")
            best = fixed[fixed[:, 1].argmin()]
            ax.scatter(best[2], best[idx], s=80, marker="*", color="#202020", zorder=5)
            for model, color in COLORS.items():
                points = np.array([r["mean"] for r in scene["results"] if r["model"] == model])
                ax.plot(points[:, 2], points[:, idx], "o-", color=color, ms=4,
                        lw=1.0, label=LABELS[model])
            ax.set(xlabel="Flight distance (m)", ylabel=label, yscale="log", title=NAMES[name])
            ax.grid(True, axis="y", alpha=0.2)
    axes[0, 1].legend(loc="best", fontsize=8)
    save(fig, "figure2_tradeoffs")

    figure3(study)


def figure3(study):
    """Heading error versus station azimuth at a fixed 2 m distance."""
    colors = {"center": "#007e80", "edge": "#b35228"}
    markers = {"center": "o", "edge": "s"}
    fig, ax = plt.subplots(figsize=(6.6, 3.4), layout="constrained")
    for name, scene in study["scenes"].items():
        rows = [r for r in scene["results"]
                if r["model"] == "fixed" and abs(r["radius"] - 2.0) < 1e-6]
        rows.sort(key=lambda r: r["angle"])
        ang = np.array([r["angle"] for r in rows])
        head = np.array([r["mean"][1] for r in rows])
        det = np.array([r["mean"][3] for r in rows])
        ax.plot(ang, head, "-", color=colors[name], lw=1.4, zorder=2, label=NAMES[name])
        sc = ax.scatter(ang, head, c=det, cmap="viridis", vmin=0, vmax=1, s=46,
                        marker=markers[name], edgecolors=colors[name], linewidths=1.0, zorder=3)
        i = int(np.argmin(head))
        ax.annotate(f"{head[i]:.2f}\u00b0", (ang[i], head[i]), fontsize=7.5,
                    color=colors[name], xytext=(6, -14), textcoords="offset points")
    ax.set_yscale("log")
    ax.set_xticks(range(0, 360, 45))
    ax.set_xticklabels([f"{a}\u00b0" for a in range(0, 360, 45)])
    ax.set_xlabel("Station azimuth in the vehicle frame (0\u00b0 = behind, along the row)")
    ax.set_ylabel("Heading RMSE (\u00b0)")
    ax.set_ylim(0.25, 20)
    ax.set_yticks([0.3, 0.5, 1, 2, 5, 10])
    ax.set_yticklabels(["0.3", "0.5", "1", "2", "5", "10"])
    ax.grid(axis="y", color="#dddddd", lw=0.5, which="major")
    ax.legend(frameon=False, loc="upper left", fontsize=8.5)
    ax.axvspan(30, 150, color="#b35228", alpha=0.06, zorder=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02, fraction=0.045)
    cb.set_label("Pair detection fraction", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    save(fig, "figure3_azimuth")


if __name__ == "__main__":
    main()
