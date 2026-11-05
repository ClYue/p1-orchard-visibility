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


if __name__ == "__main__":
    main()
