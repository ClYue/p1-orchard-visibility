# Cover letter

**Manuscript:** Visibility-Aware UAV Viewpoint Selection for Ground Robot Localization in Simulated Orchards
**Article type:** Article
**Journal:** *Drones*

Dear Editors,

We submit the above manuscript for consideration as an Article in *Drones*.

**The problem.** A single UAV can keep a ground robot localized in an orchard by observing two markers mounted on its body, but the canopy decides whether those markers are visible from any given station. Existing work on observability-aware viewpoint selection ranks candidate stations by how strongly the geometry constrains the estimate, and treats occlusion as a binary feasibility constraint. The question we address is narrower and, we think, prior to that one: how much does the *representation* of canopy visibility change which station is selected, and what does that cost in localization accuracy and flight distance?

**What we did.** We calibrate the pair-visibility field of a parameterized hedgerow orchard by ray casting, then drive one policy and one filter with three representations of that field: the ray-cast lookup table, a bounded separable Gaussian fitted to it, and a constant prior. These are compared against 48 fixed stations spanning eight azimuths and six distances, over 30 paired seeds in two scenarios that differ only in the vehicle's lateral offset. Three design choices are worth flagging:

1. **Detections are generated independently of the planner.** A policy is never scored against its own prior: marker visibility is drawn from ray intersections computed at the true marker positions in a fixed orchard. This removes a circularity that is easy to introduce, and hard to detect, in simulation studies of active sensing.
2. **The fixed-station baseline is allowed the same freedom as the planner.** It is swept over azimuth as well as distance, and its target is recomputed from the estimated pose every five steps (1 s), so the comparison does not rest on a deliberately weakened reference.
3. **The full study is reproducible from the supplement** in about 80 s on one core.

**What we found.** Three results seem worth the reader's attention.

- **The visibility field of a hedgerow orchard is asymmetric, not merely narrow.** With the vehicle on the alley centerline the field is a symmetric stripe: the pair is fully visible along the row at every distance, and visibility falls to half at 2.5 m across it. Moving the vehicle 1 m toward one row changes the structure: the nadir probability drops to 0.339, the row-aligned line never reaches 0.5 anywhere, the near-side direction becomes unusable at every distance, and the usable region moves to the far side. A symmetric separable Gaussian cannot express this, and its fit degrades from R² = 0.924 to 0.314.
- **At the canopy edge, the direction of the station matters more than its distance.** Holding the distance at 2 m and varying only azimuth, heading error spans 0.562° to 8.792°—a factor of 15.6—against a factor of 6.3 for distance over the usable range. On the centerline path the same operation spans only a factor of 2.1. A distance-decay model, which carries no directional information, cannot represent this.
- **Adaptive selection buys flight distance, not accuracy, once the baseline is tuned.** At matched flight distance the lookup-table policy uses 4.87 m less flight—26.4 m against 31.2 m, a reduction of 15.6% on unrounded values—while increasing heading RMSE from 0.562° to 0.749°. We report this as a trade rather than a dominance, and the manuscript states which configuration favours each strategy.

**Why *Drones*.** The manuscript concerns where an aerial platform should station itself to serve a ground task, which sits within the journal's scope in navigation, mission planning and agricultural operation. *Drones* has published UAV–UGV cooperative localization work, and the questions raised here—station geometry, occlusion, the cost of adaptivity—extend that line rather than duplicating it. We have also been explicit about the limits: this is a simulation study with idealized ground paths, a known UAV pose and ray-based detection, and the manuscript states what a field validation would require.

**Declarations.** The manuscript has not been published previously and is not under consideration elsewhere. All authors have read and approved the submitted version and agree to its submission to *Drones*. The authors declare no conflicts of interest. The simulation code, calibrated priors and seed-level outputs are provided as supplementary material and archived at https://github.com/ClYue/p1-orchard-visibility.

We would be glad to suggest reviewers with expertise in aerial–ground cooperative localization, agricultural robotics or observability-aware planning if that would be helpful.

Sincerely,

**Chunlong Yue**
College of Energy and Intelligent Engineering
Henan University of Animal Husbandry and Economy, Zhengzhou 450046, China
edwardyue2011@gmail.com
ORCID: 0009-0009-1272-7294
