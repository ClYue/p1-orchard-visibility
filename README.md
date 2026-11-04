# Visibility-Calibrated Observation Positioning for UAV-Assisted Ground Localization in Orchards

Code, calibrated visibility tables, and Gazebo assets accompanying the manuscript:

> **Where to Look Matters More Than How Often: Visibility-Calibrated Observation Positioning for UAV-Assisted Ground Localization in Orchards**
> Chunlong Yue, Qian Li, Junyi Zhang, Kaikai Zhou, Bo Peng
> College of Energy and Intelligent Engineering, Henan University of Animal Husbandry and Economy, Zhengzhou, China

## What this repository contains

| Directory | Contents |
|---|---|
| `code/` | The numerical simulator and the three sweep drivers (Pareto, joint heading–position, baseline station-distance) |
| `data/` | `vis_lut.npz` — the calibrated visibility lookup tables for both scenarios — and all result JSON files behind the tables in Section 3 |
| `figures/` | The eight manuscript figures (600 dpi PNG) and the scripts that generate them |
| `gazebo/` | A Gazebo Fortress world with the 126-tree parameterised orchard, a differential-drive platform with a 1.0 m marker baseline, and the world generator |
| `docs/` | The method note (Chinese) and the manuscript source |

**Scope note.** Every quantitative result in the manuscript comes from the numerical simulator in `code/`.
The Gazebo assets fix the orchard geometry in executable form and allow the ray-casting calibration to be
re-run; they are **not** used to produce the numbers in Section 3. No field data are included, because no
field campaign has been performed — this is stated in Section 4.5 of the manuscript.

## Quick start

Requires Python ≥ 3.10 with `numpy`, `scipy`, `matplotlib`.

```bash
pip install -r requirements.txt

# main comparison, packet-loss sweep, and fusion consistency (15 seeds)
python code/p1_sim.py --vis lut --case alley_center --wpos 0.2 --seeds 15
python code/p1_sim.py --vis lut --case canopy_edge  --wpos 0.2 --seeds 15

# accuracy–range Pareto sweep
python code/p1_pareto_sweep.py

# joint heading–position weight sweep
python code/p1_joint_sweep.py

# station-tuned baselines
python code/p1_baseline_sweep.py
```

### Path adjustments

The scripts were developed inside a larger project tree. When running from this repository, adjust these
constants to match the layout above:

| File | Constant | Point it to |
|---|---|---|
| `code/p1_sim.py` | `--lut` argument / `OUT` | `data/vis_lut.npz`; results written next to the script |
| `figures/figs_3to8.py` | `VIZ`, `CODE` | `figures/` and `data/` of this repository |
| `figures/export_fig_data.py` | `E1_DIR` | `data/` of this repository |
| `figures/fig1_schematic.py`, `fig2_identifiability.py` | `OUT` | `figures/` |

The result JSON files in `data/` are the exact files used for the manuscript tables, so the figures can be
regenerated without re-running any simulation.

## Regenerating the Gazebo world

```bash
python gazebo/tools/gen_orchard.py     # writes the 126-tree orchard world
```

Tested with Gazebo Fortress 6.18 and ROS 2 Humble (`ros_gz`). The world runs headless; see
`gazebo/tools/test_m3.sh` for the cmd_vel drive check.

## Citation

If you use the calibrated visibility tables or the simulator, please cite the manuscript once it is published.
Until then, cite this repository.

## License

MIT — see `LICENSE`.

## Contact

Chunlong Yue — edwardyue2011@gmail.com
