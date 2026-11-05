# P1 simulation package

This directory reproduces every number and figure reported in the manuscript "Visibility-Aware UAV Viewpoint Selection for Ground Robot Localization in Simulated Orchards".

## Contents

| Path | Description |
|---|---|
| `run_study.py` | The simulation. Compares 48 fixed stations (8 azimuths × 6 distances) with 12 adaptive configurations (three visibility priors × four motion weights) in two orchard scenarios, over 30 paired seeds each. |
| `plot_study.py` | Regenerates Figures 1–3 from `results/study.json`. |
| `results/study.json` | Column definitions, fitted priors, policy settings, means, and the 30 seed-level records per policy and scene. Seed order is shared across all policies. |
| `results/priors.npz` | The grid and both pair-visibility lookup tables, before the camera gate. |
| `results/figure1_priors.png` | Ray-cast field and its bounded-Gaussian fit, both scenarios. |
| `results/figure2_tradeoffs.png` | Heading and position error against flight distance. |
| `results/figure3_azimuth.png` | Heading error against azimuth at a fixed 2 m station distance. |
| `results_sensitivity_150s/` | The same experiment with the run lengthened to 150 s (see below). |

## Reproduce

Requires Python 3.10 or newer.

```sh
python -m pip install -r requirements.txt
python run_study.py          # 50 s runs; writes results/study.json
python plot_study.py         # regenerates Figures 1-3
```

The full 50 s experiment takes about 80 s on a single CPU core and is deterministic: every random draw is keyed to its seed.

## Trial length

The manuscript reports 50 s trials. Over that horizon the vehicle advances 30 m along the row, which is the length of one bay in a commercial hedgerow orchard. Because the conclusion of the paper is a comparison rather than an absolute accuracy claim, the whole experiment was repeated with the horizon extended to 150 s (90 m of travel):

```sh
python run_study.py --steps 750 --seeds 30 --output results_sensitivity_150s
```

The ordering of the policies is unchanged and the gap widens:

| Scenario | Horizon | Best fixed station (°) | Lookup table at matched flight (°) |
|---|---|---|---|
| Center | 50 s | 0.359 | 0.442 |
| Center | 150 s | 0.321 | 0.480 |
| Edge | 50 s | 0.562 | 0.749 |
| Edge | 150 s | 0.461 | 0.704 |

A longer run helps both strategies, because more observations accumulate, but it helps the fixed station more: the adaptive penalty grows from 0.083° to 0.159° at the center and from 0.187° to 0.243° at the edge. The tree rows extend automatically with the horizon, so the platform never leaves the orchard.

## Crown width

The across-row crown semi-axis is exposed as a parameter, so the visibility field can be swept without editing the code:

```sh
python run_study.py --crown-b 1.8 --output results_crown_b1.8
```

Four widths are reported: 1.2 m (`results/`), and 1.5, 1.8 and 2.0 m (`results_crown_b*/`). Widening the crown drives the nadir probability at the edge from 0.339 to 0.018 and the across-row half-visibility distance from 4.5 m to zero, while the best fixed station degrades from 0.562° to 1.396°. The adaptive penalty at matched flight distance narrows but never reverses.

## Paired confidence intervals

To reproduce the edge intervals, subtract the per-seed samples of the 225°, 2 m fixed station from the samples of the lookup-table policy at weight 0.001. With `numpy.random.default_rng(91)`, draw 5,000 rows of 30 seed indices with replacement, take the mean paired difference for each row, and report its 2.5th and 97.5th percentiles.

## Notes on the model

- The planner never sees the observation process. Detections are Bernoulli draws from `exp(-kℓ₁)·exp(-kℓ₂)`, where ℓᵢ are the in-crown path lengths obtained by intersecting the segment from each true marker position with the ellipsoids of the fixed orchard.
- The camera gate is `0.35 m ≤ range ≤ 5.7·tan(60°) m`. Below 0.35 m the azimuth of a single marker becomes ill-conditioned; beyond the far bound the pair falls outside the assumed field of view.
- The fixed-station family is evaluated with the same estimator and the same speed limit as the adaptive policies, and its target is recomputed from the *estimated* pose every five steps, so the baseline is a realistic follower rather than an oracle.

## Before submission

`Manuscript.md` is the release manuscript. Confirm the author contributions, the funding statement and the AI-use disclosure against the actual record, and check both declarations in the selected cover letter.
