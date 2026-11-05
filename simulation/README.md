# P1 simulation package

This package accompanies *Canopy Visibility Constrains UAV Viewpoint Selection: A Simulation Study in Orchards*. It compares 48 fixed stations and 12 adaptive configurations in two orchard scenarios, with 30 paired seeds per scenario.

## Reproduce Figures 1–4

The package includes the saved inputs needed to generate the figures. Run these commands in the directory containing the scripts. Python 3.10 or newer is required.

~~~sh
python -m pip install -r requirements.txt
python plot_publication.py --output figures
~~~

The plotting script writes 600 dpi PNG and vector PDF files. The supplied captions identify the plotted quantities and uncertainty intervals.

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
