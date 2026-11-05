# P1 revised simulation and manuscript

The experiment compares 48 fixed stations and 12 adaptive configurations in each of two orchard scenarios. The 30 paired seeds per scenario give 3,600 simulated missions. The study models a known UAV pose, idealized azimuths, and ray-based marker detection.

## Reproduce

From this directory, in an environment with Python 3.10 or newer:

```sh
python -m pip install -r requirements.txt
python run_study.py
python plot_study.py
```

- `results/study.json` contains the column definitions, fitted priors, policy settings, means, and 30 seed-level records per policy and scene. The seed order is shared between all policies.
- `results/priors.npz` contains the grid and both lookup tables before the camera gate.
- The two PNGs are generated at 600 dpi. The same saved results generate both figures and the manuscript values.
- To reproduce the edge intervals, subtract the samples of the 225°, 2 m fixed station from the samples of the LUT policy with weight 0.001. With NumPy `default_rng(91)`, draw 5,000 rows of 30 seed indices with replacement, take the mean paired difference for each row, and report its 2.5th and 97.5th percentiles.

The release manuscript is in `Manuscript.md` and the two draft cover letters are separate. Before submission, confirm the author contributions, funding and AI disclosure for this revision, confirm both declarations in the selected cover letter, and upload the supplement with the manuscript.
