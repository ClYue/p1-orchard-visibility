# Does Visibility Modeling Change UAV Viewpoint Selection?

Code and data accompanying the manuscript:

> **Does Visibility Modeling Change UAV Viewpoint Selection? A Controlled Study in Simulated Orchards**
> Chunlong Yue, Qian Li, Junyi Zhang, Kaikai Zhou, Bo Peng
> College of Energy and Intelligent Engineering, Henan University of Animal Husbandry and Economy, Zhengzhou, China

## Repository layout

| Directory | Contents |
|---|---|
| `simulation/` | **The current version.** Simulation, plotting code, calibrated priors, seed-level results, figures, and the manuscript. This is what the paper reports. |


## Quick start

```bash
cd simulation
pip install -r requirements.txt
python run_study.py          # ~80 s, writes results/study.json
python plot_study.py         # regenerates Figures 1-3
```

The full reproduction, including the trial-length and crown-width sweeps, is described in `revised/README.md`.

## What the first version got wrong

An earlier simulation generated marker detections by consulting the same visibility lookup table that the planner used to choose its station. A policy was therefore scored against its own prior, which flatters the adaptive strategy and is easy to miss on inspection.

`simulation/` separates the two. The planner is given a prior; detections are drawn from ray intersections computed at the true marker positions in a fixed orchard. The effect is not cosmetic: under the old arrangement the adaptive policy appeared to beat a tuned fixed station, and under the new one it does not. The corrected comparison is the subject of the manuscript, which reports a trade in flight distance rather than a dominance in accuracy.

## License and citation

See `LICENSE`. The first version remains in the Git history (before commit `f65615b`) so that the correction described above can be checked by anyone who has cited the earlier results.
