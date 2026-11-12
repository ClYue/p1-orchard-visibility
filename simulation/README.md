# P1 simulation package

This package accompanies *Ground Paths Decide Which UAV Viewpoint Is Best in Orchard Localisation*. It compares 48 fixed stations and 12 adaptive configurations in two orchard scenarios, with 30 paired seeds per scenario.

## Package layout

| Path | Contents |
|---|---|
| `run_study.py` | simulation and estimator; writes one results folder per run |
| `plot_publication.py` | Figures 1–4, written to `figures/` |
| `gazebo/` | Gazebo rebuilt orchard, rendering, validation (Figure 5) |
| `results*/` | simulation output: `study.json` plus `priors.npz` |
| `figures/` | rendered Figure 1–5 as PNG and vector PDF |
| `tools/` | build, audit, preview export, reference check and tree-snapshot scripts |
| `../00_SUBMIT/` | the submission package (project root, next to `simulation/`): `1_上传/` to upload, `2_核对/` previews, `3_核查记录/` audit records |
| `报告/` | experiment and review reports (Chinese) |

`../00_SUBMIT/` is only meaningful for submission. Rebuilding it requires Word and
`pywin32`; the scripts pass a single docx through XML-level edits rather than
regenerating it, because the manuscript contains 134 OMML equations and the
MDPI style set.

## Reproduce Figures 1–4

The package includes the saved inputs needed to generate the figures. Run these commands in the directory containing the scripts. Python 3.10 or newer is required.

~~~sh
python -m pip install -r requirements.txt
python plot_publication.py --output figures
~~~

The plotting script writes 600 dpi PNG and vector PDF files. The supplied captions identify the plotted quantities and uncertainty intervals.

## Reproduce the Gazebo visibility check (Section 3.5)

The `gazebo/` scripts rebuild the orchard in Gazebo Sim 6.18 and compare rendered
marker visibility with the model's predicted transmittance.

~~~sh
cd gazebo
python make_world.py                 # writes orchard.sdf and sites.json
python render_sites.py --out shots   # 48 stations, one image each
python analyze.py --shots shots --csv visibility_gazebo.csv
python plot_validation.py            # Figure 5
~~~

`render_sites.py` needs Gazebo Sim with the `ogre` rendering engine and, on WSL,
`DISPLAY=:0`. No ROS installation is required: frames are read through
`ign topic --json-output`. `analyze.py` imports `run_study.py` from the parent
directory for the model's transmittance function.

## Recompute the simulation results

~~~sh
python run_study.py --seeds 30 --output results
python run_study.py --seeds 30 --steps 750 --output results_sensitivity_150s
python run_study.py --seeds 30 --crown-b 1.5 --output results_crown_b1.5
python run_study.py --seeds 30 --crown-b 1.8 --output results_crown_b1.8
python run_study.py --seeds 30 --crown-b 2.0 --output results_crown_b2.0
~~~

Each results folder contains a JSON file with column definitions, configuration means, and seed-level values. It also contains an NPZ file with the grid and lookup tables before the camera gate. Seed order is shared within each case. In the vehicle frame 0° is forward, 180° is rearward, and positive angles are counterclockwise.

The 150 s case extends the orchard rows and generates a different realization. Comparisons with the 50 s case describe the longer scenario. In the crown-width sweep, the adaptive motion weight is held at 0.001 and the best-heading fixed station is selected separately at each width. Flight distance is an observed outcome.

To reproduce the intervals in Section 3.2, subtract the seed-level samples of the 225°, 2 m fixed station from the corresponding samples of the lookup-table policy at weight 0.001. Draw 5,000 samples of 30 seeds with replacement using NumPy default_rng(91). The 2.5th and 97.5th percentiles of the resampled mean differences give the interval. Figure 3 uses pointwise percentile bootstrap intervals for each plotted mean.

The estimator receives a pair only when both markers meet the camera gate and pass the joint Bernoulli draw. The draw probability is the product of the two ray transmittances. The observation generator uses a separate orchard realization from the planner's prior.
