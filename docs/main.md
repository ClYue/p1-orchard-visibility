# Where to Look Matters More Than How Often: Visibility-Calibrated Observation Positioning for UAV-Assisted Ground Localization in Orchards

**Chunlong Yue ¹\*, Qian Li ¹, Junyi Zhang ¹, Kaikai Zhou ¹ and Bo Peng ¹**

¹ College of Energy and Intelligent Engineering, Henan University of Animal Husbandry and Economy, Zhengzhou 450046, China
\* Correspondence: edwardyue2011@gmail.com; ORCID: 0009-0009-1272-7294

**Abstract:** UAV-assisted localization is a promising fallback for ground robots in GNSS-denied orchards. Yet the UAV's station is usually chosen by geometry alone, and marker visibility through the canopy is ignored or modeled as isotropic. We show that these two requirements conflict when the markers sit near a canopy face. This is a mounting decision, not a property of the orchard. We resolve it with a calibrated, direction-dependent visibility model. Ray-casting calibration in a parameterized hedgerow orchard shows that the along-row sight line is never attenuated for markers on the alley centerline, whereas across-row visibility collapses beyond 2.5 m. The conventional product-Gaussian model is shape-mismatched, with fitted amplitudes above unity and negative R². We embed the calibrated field in a one-step-lookahead planner whose cost couples the predicted heading and position standard deviations. Over 15 seeds, it yields 0.32° heading and 0.017 m position RMSE with 99 m of flight, against 0.51°, 0.025 m and 185 m for the best station-tuned baseline. Covariance-intersection fusion keeps the NEES exceedance at 0.6% and 0.5%, versus 7.1% and 6.2% for naive fusion. Heading stays within 0.32-0.38° under 0-50% packet loss.

**Keywords:** UAV-UGV cooperative localization; GNSS-denied orchard; observation positioning; visibility calibration; heading identifiability; covariance intersection

**Highlights:**
- Along-row visibility of alley-centre markers is never attenuated in hedgerows.
- Geometry and canopy visibility conflict only when markers sit near a canopy face.
- A calibrated visibility lookup table replaces the product-Gaussian assumption.
- Joint heading-position cost beats the best station-tuned baseline 1.6x.

## 1. Introduction

A ground robot working a hedgerow orchard has to know two things continuously: where it is across the row, to a few centimetres, and which way it is pointing, to a fraction of a degree. Both are normally supplied by GNSS. Under a mature canopy, neither is. The canopy does not merely add noise to the fix. It removes satellites from view and multiplies the surviving signals by multipath. Single-point positioning therefore degrades by an order of magnitude in both coniferous and broadleaved stands [16]. Even multi-constellation, multi-epoch processing recovers only part of the loss [17]. Raising the receiver quality does not solve it either — low-cost receivers with adapted RTKLIB still struggle to hold a fixed solution under tree cover [18]. The consequence is visible in the field: an autonomous orchard vehicle that performs well on the headland degrades markedly once it enters the row [19]. The problem is structural, and it is the row itself that creates it.

What distinguishes the orchard from other GNSS-denied settings is that its occlusion is not random. Row spacing, tree spacing, canopy dimensions and pruning style are design parameters of the orchard: known to the grower, and roughly constant over a season. A camera looking down through such a structure sees a visibility pattern that is periodic along the row and sharply bounded across it — a pattern that can, in principle, be computed from geometry rather than discovered by trial. If that is true, then the question of where an aerial observer should stand is answerable before the flight, and the answer is a property of the orchard rather than of the mission.

The established response is to fuse whatever is left. Visual-inertial odometry, wheel odometry and a low-grade IMU, tied loosely to the occasional GNSS fix, keep position error bounded over a working pass [20,21]. These systems work, and they are the reason orchard robots can operate at all. But they leave one channel weak. Position is constrained by several independent sources; heading is constrained by the gyro alone. Adding sensors does not close that gap: an IMU constrains attitude rate and wheel odometry constrains planar motion, but neither observes the heading error itself. Only an external bearing to a known point does. This is elementary in the estimation literature but easy to forget in the agricultural one. The bias is therefore observable in principle yet unobserved in practice whenever the only external reference is itself unavailable. A bias of 0.01 rad/s, entirely typical of a consumer-grade unit, accumulates to 0.6° per second of dead reckoning. Meanwhile the agricultural localisation literature reports position RMSE almost exclusively [12,19,20]; heading appears in the discussion of end-of-row manoeuvring [14] but rarely in the results table. The channel that matters most for in-row work, such as aligning a sprayer to a canopy face or turning cleanly at the headland, is consequently the one least often measured and least often reported.

An aerial platform changes the geometry of the problem, because a UAV flying above the canopy can see what the ground vehicle cannot. Air–ground cooperative localisation has consequently become an active line. UAVs and UGVs now share position estimates in GNSS-challenged terrain [11]. Other systems fuse visual-inertial and ultra-wideband observations without anchors [22], or let the aerial platform carry the good GNSS fix and pass it to the ground platform [23]. Degradation-aware fusion uses LiDAR detections of the partner robot as a relative observation [24], and tethered "marsupial" architectures couple the pair physically [25]. Related work uses the aerial view for semantics [26], minimises the sensing suite that still closes the loop [27], or plans the pair's path jointly in semi-structured agricultural environments [28]. Orchard-specific instances of the same idea map the environment with a UGV–UAV pair [13], and benchmarks [30], extreme-environment deployments [31] and recent reviews [15,32] confirm that the architecture is no longer the open question. What remains open is where the aerial platform should be. In most of these systems the station is prescribed — follow at a fixed offset, orbit, sweep a coverage pattern — or optimised for coverage and connectivity [28,29]. The question "from where can the markers actually be seen" is either not asked, or answered by a binary line-of-sight test against a map.

The tools for answering it properly already exist, in two separate halves. On the estimation side, the identifiability of relative pose from bearing measurements has a clean analytic structure. Two bearings to a known baseline constrain position and heading simultaneously. The heading information carries a $\cos^2$ dependence on the viewing direction, so some geometries are informative and others are not [1,4]. Closed-form estimators now exist for the 4-DoF case, together with an explicit characterisation of the motion patterns — collinear, shape-preserving — under which they degenerate [7], and the same analysis has been carried to SE(2) with a comparison of filter and pose-graph formulations [9]. Enriching the measurement set with bearing rates removes some of these degeneracies at the cost of additional sensing [8]. On the planning side, observability-aware trajectory optimisation treats an information metric as a cost term [5], and the COP framework makes explicit the trade-off between tracking quality and estimation quality that such formulations inherit [6]. Related formulations steer a formation by range-only observability [37], enhance identifiability through bio-inspired motion [38], plan paths that maximise the information gathered along them, including the case where one agent acts as a moving beacon for another [40], or make bearing-only geolocation practical on low-cost monocular platforms [39]. Where two estimates of the same state are fused, the consistency literature supplies the remedy for the resulting correlation: covariance intersection [36] and its later variants [2,3,10,33,34,35]. Taken together, this is a mature toolkit. We use one element of it, the single-step heading information of a two-bearing baseline, and we call it *identifiability* rather than observability, because it is an information measure at one instant, not a property of the nonlinear system's observable subspace [1].

What the toolkit assumes, and what the orchard violates, is that a measurement which is geometrically available is also physically obtainable. The planning papers adopt a visibility model, almost always a separable exponential with two correlation lengths, and do not calibrate it against the scene it describes [5,6]. The agricultural papers do calibrate, but they calibrate the effect of the canopy on GNSS [16–18], not on a camera looking down through it. To our knowledge no study measures the *visual* visibility field of a hedgerow orchard and places it inside the observation-planning objective; and none reports the heading consequence of doing so. Where a visibility test does appear in the planning literature, it is binary: a sight line either intersects an obstacle or it does not. For a single ray that is the right model; for a camera looking through a canopy at a marker a few centimetres across it is not. Partial occlusion, leaf clutter and the finite size of the target all matter, and the quantity that enters the estimator is a probability, not a flag. The gap is therefore not a missing algorithm. It is a missing measurement, and it sits exactly at the junction of two literatures that rarely cite each other.

This paper closes that junction for one concrete case: a ground robot carrying a two-marker baseline, and a single UAV that observes it from above. We do three things. First, we calibrate the visibility field by ray casting through a parameterised hedgerow orchard, and show that the conventional separable-exponential model is not merely imprecise but the wrong shape — its fitted amplitude exceeds unity and its coefficient of determination is negative in every canopy regime. Second, we embed the calibrated field, as a lookup table, in a one-step observation-position planner whose cost couples the predicted heading and position standard deviations, and show that the coupling is not cosmetic: the heading-only variant selects stations that are geometrically excellent and practically invisible. Third, we quantify what happens when the two correlated estimates are fused, and show that calibrating visibility improves consistency as well as accuracy. The contributions are:

1. **A measured visibility model for a hedgerow orchard.** Ray-casting calibration over 20 jittered orchard realisations, reported as a lookup table rather than a fitted analytic form, together with the evidence that the analytic form conventionally used in this literature fails on both shape and parameter stability (Section 2.3–2.4, Figure 3).
2. **The observation that along-row occlusion is a consequence of how the markers are mounted.** For markers on the alley centreline the along-row sight line is never attenuated at any UAV altitude between 2 and 6 m; along-row decay appears only when the markers are displaced toward a canopy face. This inverts the working assumption that the geometrically optimal direction is the most occluded one (Section 3.1, Figure 4).
3. **A planner whose cost couples heading and position**, and the demonstration that the position term is what converts parity with a fixed-station baseline into dominance — 0.32° heading and 0.017 m position at 99 m of flight, against 0.51°, 0.025 m and 185 m for the same baseline tuned over station distance (Sections 2.5, 3.4, 3.7).
4. **A consistency account of the fused estimate**, with naive fusion exceeding the 95% NEES bound in 7.1% of steps and covariance intersection reducing it to 0.6% (Section 3.6, Figure 8).

The study is deliberately numerical. Every quantitative result comes from a purpose-built simulator driven by the calibrated visibility field, and the Gazebo assets we release fix the geometry in executable form without producing any of the numbers in Section 3 (Section 2.7). We state this at the outset because the paper's weakest assumption is the orchard model itself, and Section 4.6 sets out the field measurement that would replace it.

## 2. Materials and Methods

### 2.1. Problem Formulation and Notation

We consider a ground robot that must estimate its planar pose while traversing an orchard alley under complete GNSS denial, and a single UAV that observes the robot from above and communicates bearing measurements over a wireless link. The ground robot is equipped with wheel odometry and a gyroscope; its heading is corrupted by an unknown, slowly varying gyro bias. Two passive marker boards are mounted on the robot, one at the front and one at the rear, separated by a known baseline $L$. The UAV carries a downward-looking camera, detects both markers with a fiducial-marker detector [41], and reports their bearings in its own image frame. Table 1 summarises the notation.

**Table 1.** Notation.

| Symbol | Meaning | Value / note |
|---|---|---|
| $\mathbf{p}=(x,y)$ | ground robot position | estimated |
| $\theta$ | ground robot heading | estimated; dominant error source |
| $b$ | gyroscope bias | estimated; rad/s |
| $\mathbf{x}=(\mathbf{p},\theta,b)$ | augmented state, 4-D | — |
| $\mathbf{u}$ | UAV horizontal position | treated as known from its own navigation |
| $L$ | marker baseline | 1.0 m |
| $\beta_i$ | bearing to marker $i$, $i\in\{1,2\}$ | measurement |
| $\sigma_\beta$ | bearing noise standard deviation | 0.008 rad |
| $r_i$ | range from UAV to marker $i$ | — |
| $r$ | observation station distance, $\lVert\mathbf{u}-\mathbf{p}\rVert$ | design variable |
| $p_{\mathrm{vis}}(\mathbf{u})$ | probability that both markers are visible from $\mathbf{u}$ | calibrated, see §2.3 |
| $\kappa$, $w$ | position and motion weights in the planner cost | 0.2, 0.004 |

The ground platform obeys the nonholonomic motion model

$$
\dot{x}=v\cos\theta,\qquad \dot{y}=v\sin\theta,\qquad \dot{\theta}=\omega_m-b,
\tag{1}
$$

where $v$ is the odometric speed and $\omega_m$ the raw gyro reading. Because the bias enters only through the heading channel, it can be resolved exclusively by external observations; this is the structural reason why the observation geometry, and not merely the number of observations, determines estimation quality.

Marker positions follow from the pose,

$$
\mathbf{m}_1=\mathbf{p}+\tfrac{L}{2}\mathbf{e}(\theta),\qquad
\mathbf{m}_2=\mathbf{p}-\tfrac{L}{2}\mathbf{e}(\theta),\qquad
\mathbf{e}(\theta)=(\cos\theta,\sin\theta)^{\top},
\tag{2}
$$

and each bearing is

$$
\beta_i=\operatorname{atan2}\!\left(m_{i,y}-u_y,\;m_{i,x}-u_x\right)+\varepsilon_i,
\qquad \varepsilon_i\sim\mathcal{N}(0,\sigma_\beta^2).
\tag{3}
$$

Two markers, rather than one, are used deliberately. A single bearing constrains position only and leaves heading unidentifiable; two bearings constrain position and heading simultaneously and, as shown next, endow the heading information with a clean geometric structure. Figure 1 illustrates the layout assumed throughout: the robot on the alley centreline, and two candidate aerial stations, one on the alley axis and one across the neighbouring row.

**Figure 1.** Observation geometry and the calibrated visibility field. **(a)** Plan view of the hedgerow orchard (row spacing 4.0 m, tree spacing 2.0 m) with the ground robot on the alley centreline, its two marker boards separated by *L* = 1.0 m, and two candidate UAV stations: **S1** on the alley axis (green solid lines) and **S2** across the neighbouring row (red dashed lines). **(b)** Calibrated visibility *p*<sub>vis</sub> at *z* = 6 m for the open-alley scenario, averaged over 20 jittered orchard realisations; the dashed contour is *p*<sub>vis</sub> = 0.5. The alley is an unobstructed corridor (S1 lies inside it, *p*<sub>vis</sub> = 1.00), whereas the across-row station falls in the shadow of the neighbouring canopy (*p*<sub>vis</sub> = 0.12). The two stations are therefore the geometric optimum and the visibility optimum of the same problem, and they are spatially orthogonal.

We make three assumptions explicit. (i) The UAV position is known to the estimator, either from its own GNSS/INS outside the canopy or from a surveyed flight plan. This asymmetry is what makes the UAV useful in the target application, and it is also exploited by map-sharing air–ground frameworks in GNSS-challenged environments [11]. (ii) Marker detection is modelled as a bearing measurement with isotropic Gaussian noise; detection failures are absorbed into $p_{\mathrm{vis}}$. (iii) The orchard is geometrically stationary over a mission, so visibility may be pre-computed rather than estimated online.

---

### 2.2. Heading Identifiability and Its Geometric Structure

Linearising (3) about the state gives the measurement Jacobian. With $\mathbf{d}_i=\mathbf{m}_i-\mathbf{u}$, $r_i=\lVert\mathbf{d}_i\rVert$ and $s_1=+1$, $s_2=-1$,

$$
\frac{\partial\beta_i}{\partial\mathbf{p}}
=\frac{1}{r_i}\left(-\sin\beta_i,\;\cos\beta_i\right),
\qquad
\frac{\partial\beta_i}{\partial\theta}
=s_i\,\frac{L}{2r_i}\cos(\beta_i-\theta),
\qquad
\frac{\partial\beta_i}{\partial b}=0 .
\tag{4}
$$

The heading block of the single-step Fisher information therefore reads

$$
\boxed{\;
F_{\theta\theta}(\mathbf{u})
=\frac{1}{\sigma_\beta^{2}}\sum_{i=1}^{2}\left(\frac{\partial\beta_i}{\partial\theta}\right)^{2}
=\left(\frac{L}{2\sigma_\beta}\right)^{2}
\sum_{i=1}^{2}\frac{\cos^{2}(\beta_i-\theta)}{r_i^{2}}
\;}
\tag{5}
$$

Two remarks are necessary. First, the two terms must be squared individually before summation. Summing the two Jacobian entries first and then squaring produces a spurious cancellation along the vehicle axis, which reverses the conclusion of the entire analysis; this error is easy to make and was committed in our own first implementation. Second, (5) is a single-step, static quantity — an information measure, not a dynamic observability property of the nonlinear system. We therefore refer to it throughout as *heading identifiability* rather than observability; the distinction matters because a filter can be consistent in the observability sense while the instantaneous information is arbitrarily small, and vice versa.

Three consequences follow directly from (5).

*Distance decay.* For $r\gg L$, $\cos(\beta_i-\theta)$ varies slowly with the station, so $F_{\theta\theta}\propto r^{-2}$ and hence $\sigma_\theta\propto r$: doubling the station distance halves the heading accuracy. This is a statement about information per observation, not about the optimum of a mission, because the visibility term in §2.3 also depends on $r$.

*Directional sensitivity.* The factor $\cos^{2}(\beta_i-\theta)$ is the squared cosine of the angle between the line of sight and the vehicle's longitudinal axis. Along-axis viewing ($\beta_i-\theta\in\{0,\pi\}$) gives unit weight; a 45° oblique view halves it; lateral viewing ($\beta_i-\theta\approx\pm\pi/2$) suppresses it. The mechanism is elementary: when the vehicle rotates about its centre, the markers translate perpendicular to the axis, and that translation converts into a bearing change only if the line of sight is also perpendicular to the translation, i.e., axial. Figure 2a shows the resulting pattern: two lobes aligned with the vehicle axis and a waist at $\pm90°$ that never reaches zero, while Figure 2b plots the absolute information against station distance for both directions.

**Figure 2.** Heading identifiability of a two-marker baseline. **(a)** Normalised Fisher information *F*<sub>θθ</sub> as a function of viewing bearing, for two station distances. Curves are normalised to their own axial value so that only the shape is compared; the two lobes are aligned with the vehicle axis and the waist at ±90° never reaches zero because *L* is finite (waist 4.6% at *r* = 2 m, 0.6% at *r* = 6.5 m). **(b)** Absolute information versus station distance on logarithmic axes: axial viewing (0°) and lateral viewing (90°) both decay as *r*<sup>−2</sup>, and the ratio approaches ½(*L*/2*r*)². The lateral channel is thus asymptotically, not absolutely, degenerate — the distinction matters at the short station distances that are flyable inside an orchard.

*Asymptotic lateral degeneracy.* At an exactly lateral station the two markers are symmetric about the line of sight, hence $\beta_1-\theta=\arctan(2r/L)$ and $\beta_2-\theta=\pi-\arctan(2r/L)$, and therefore

$$
\cos^{2}(\beta_i-\theta)=\frac{(L/2)^{2}}{(L/2)^{2}+r^{2}}\neq 0,
\qquad
\frac{F^{\mathrm{lat}}_{\theta\theta}}{F^{\mathrm{ax}}_{\theta\theta}}
\approx\frac{1}{2}\left(\frac{L}{2r}\right)^{2}.
\tag{6}
$$

Lateral viewing is thus *asymptotically* degenerate, not degenerate. With $L=1.0$ m the residual fraction is 4.6% at $r=2$ m, 0.25% at $r=10$ m and 0.17% at $r=12$ m. We emphasise this because the claim "lateral is uninformative" is often stated as an exact result. It is not. At the short station distances that are flyable in an orchard, the lateral channel retains a few percent of the axial information. That residue is what makes short cross-row stations worth considering.

*Relation to existing formulations.* Bearing-only relative localisation has been studied extensively for multi-robot teams, both with known and anonymous data association [4,7]; the recurring finding is that the estimation problem is well posed only under sufficient relative motion, and that degenerate formation geometries (collinear, shape-preserving) must be detected rather than assumed away [7,9]. A parallel line of work optimises the trajectory for estimation quality, either by embedding an observability metric in the planning objective [5,6] or by enriching the measurement set with bearing rates [8]. Our formulation differs in what is held fixed and what is optimised. The measurement set is fixed and minimal — two bearings to a known baseline — and the environmental structure, rather than the excitation of the platform, is the binding constraint: the planner cannot choose an arbitrary trajectory because most of the surrounding space is occluded. Equation (5) is therefore used not as a trajectory-design objective in free space, but as a *one-step* criterion evaluated against a calibrated visibility field. The interesting question is then how the two competing requirements trade off in a specific orchard geometry, not how to excite the system in general.

---

### 2.3. Calibrating Canopy Visibility in a Parameterized Orchard

The heading analysis says where to look; whether the markers can be seen there is a separate, geometric question. Rather than postulate an occlusion model, we calibrate one by ray casting through a parameterized orchard.

*Orchard model.* Hedgerows run along $x$ with row spacing 4.0 m, tree spacing 2.0 m along the row, and tree positions perturbed by $\mathcal{N}(0,0.15^2)$ m to represent planting irregularity. These values follow the hedgerow geometry reported for commercial orchards in which ground and aerial platforms have been deployed for mapping and navigation [13–15]. Each canopy is an ellipsoid with semi-axes $(1.2,\,b,\,1.5)$ m centred at 2.0 m height, so the canopy band occupies $z\in[0.5,3.5]$ m; trunks are omitted because the sight lines of interest pass well above them. The robot travels along the alley centreline $(y=0)$ between two hedgerows; markers sit at $z=0.3$ m. Two canopy regimes are considered. The open alley ($b=1.2$ m) has canopies that do not meet over the alley; the closed canopy ($b=2.0$ m) has canopies touching above it. These bracket the geometries of modern hedgerow orchards.

*Occlusion computation.* For a UAV station $\mathbf{u}$ and a marker $\mathbf{m}$, the sight segment is intersected analytically with every canopy ellipsoid under the affine map that turns each ellipsoid into a unit sphere; the accumulated path length $\ell$ inside canopies gives an optical depth $\tau=k\ell$ and a visibility

$$
p_{\mathrm{vis}}(\mathbf{u})=\tfrac{1}{2}\sum_{i\in\{1,2\}}\exp\!\left(-k\,\ell_i(\mathbf{u})\right),
\qquad k=1.0\ \mathrm{m^{-1}},
\tag{7}
$$

averaged over the two markers. Stations are scanned on a $0.5$ m grid over $[-15,15]^2$ m at $z=6$ m, and each configuration is averaged over 20 independently jittered orchards.

*Non-parametric descriptors.* Because the resulting visibility maps are not Gaussian in shape, we report two descriptors that make no shape assumption: $X_{50}$ and $Y_{50}$, the largest along-row and across-row offsets at which $p_{\mathrm{vis}}\ge 0.5$, plus the nadir visibility $p_0=p_{\mathrm{vis}}(\mathbf{0})$.

The calibration produces a result that contradicts the working assumption in much of the literature. For markers on the alley centreline the along-row sight line never leaves the alley plane and therefore never enters a canopy: $X_{50}$ saturates at the scan boundary (15 m) for every canopy width $b\in\{1.2,1.5,1.8,2.0\}$ m and every UAV altitude $z\in\{2,3,4.5,6\}$ m. Occlusion is one-sided: $Y_{50}$ collapses from 2.5 m in the open alley to 0.5 m under a closed canopy, and the across-row window narrows as the UAV descends (2.5 m at $z=6$ m versus 1.0 m at $z=2$ m). To recover the along-row attenuation that motivates axial viewing at all, the markers must be displaced toward a canopy face: a lateral offset of 1.0 m (canopy inner edge at 0.8 m) restores a finite $X_{50}=4.0$ m, at the cost of $p_0=0.55$ and a narrowed cross-row window ($Y_{50}=3.5$ m). We therefore carry two calibrated scenarios through the rest of the paper: `alley` (markers on the centreline) and `edge` (markers 1.0 m toward the hedge).

---

### 2.4. A Lookup-Table Visibility Model

The conventional parameterisation in this literature — inherited from visibility models used in observability-aware planning and from occlusion models adopted in agricultural localisation studies [5,6,12–14] — is a separable exponential, $p_{\mathrm{vis}}=\exp(-(d_x/\lambda_x)^{2}-(d_y/\lambda_y)^{2})$, whose correlation lengths $\lambda_x,\lambda_y$ are usually quoted without derivation. We fitted this family to the calibrated maps in the linear least-squares sense, with the amplitude treated as a free parameter, over the full scan grid.

The family is rejected on two independent grounds. First, it is shape-mismatched. The calibrated maps have a plateau-and-cliff structure: visibility near unity in the alley, a sharp drop at the hedge, and a low plateau beyond. No product of two Gaussians can represent this. The fit returns amplitudes up to 1.18, which is impossible for a probability, and negative coefficients of determination in every regime ($R^2\in[-1.07,-0.57]$). Second, the fitted lengths are unstable: $\lambda_x$ drifts by up to a factor of 1.7 as the extinction coefficient $k$ varies over $[0.5,2.0]$ m⁻¹, whereas the non-parametric descriptors are essentially invariant. Reporting $\lambda$ as a calibrated quantity would therefore be misleading.

We instead store the calibrated map itself as a lookup table and query it bilinearly,

$$
p_{\mathrm{vis}}(\mathbf{u})=\mathcal{B}\!\left[\,P\,\right]\left(\mathbf{u}-\hat{\mathbf{p}}\right),
\tag{8}
$$

where $P$ is the 61×61 grid of §2.3 evaluated at the current position estimate, with edge clamping outside the stored domain. One table is stored per scenario (`alley`, `edge`); the table is a property of the orchard, not of the mission, so it is computed once offline (about 2 min on a laptop CPU) and thereafter queried in constant time. This replaces a false analytical assumption with a measured one at negligible online cost, and it is the component that the fixed-station baselines in §3 lack.

---

### 2.5. Observation-Position Planning

The planner chooses the next UAV station from a finite candidate set and is re-solved at every control step. Candidates are placed on 20 bearings × 6 station distances $\{2,3,4.5,6.5,9,12\}$ m around the current position estimate, giving $|\mathcal{U}|=120$ points — small enough for real-time use, dense enough to cover both the axial and the lateral directions.

The cost combines the two posterior standard deviations with the manoeuvre effort,

$$
\mathbf{u}^{\star}=\arg\min_{\mathbf{u}\in\mathcal{U}}\;
\sqrt{\big[\mathbf{P}_f(\mathbf{u})\big]_{\theta\theta}}
\;+\;\kappa\sqrt{\operatorname{tr}\big(\big[\mathbf{P}_f(\mathbf{u})\big]_{pp}\big)}
\;+\;w\,\lVert\mathbf{u}-\mathbf{u}_k\rVert,
\tag{9}
$$

$$
\mathbf{P}_f(\mathbf{u})=\Big(\mathbf{P}_{\mathrm{pred}}^{-1}
+p_{\mathrm{vis}}(\mathbf{u})\,\mathbf{H}^{\!\top}\mathbf{R}^{-1}\mathbf{H}\Big)^{-1},
\tag{10}
$$

with $\mathbf{H}$ linearised at the *predicted* state rather than at ground truth, $\mathbf{P}_{\mathrm{pred}}$ the predicted covariance after the motion update, and $\mathbf{R}=\sigma_\beta^2\mathbf{I}$. The visibility enters (10) as a multiplicative weight on the information contribution, i.e., as the expected fraction of steps in which the observation is actually obtained.

The position term is not decoration. Minimising heading variance alone drives the UAV toward stations that are geometrically favourable but poorly visible: the heading-only variant has a mean per-step visibility of 0.31. Position and heading errors are coupled through the off-diagonal blocks of $\mathbf{P}_f$, so degrading position degrades heading as well. Adding the position term with $\kappa=0.2$ returns the vehicle to well-visible short-range stations (mean visibility 0.99 in the open alley and 0.84 with the markers at the canopy edge) and improves *both* components; the ablation is reported in §3.4. Algorithm 1 summarises the procedure.

```
Algorithm 1  Identifiability-driven observation positioning (one-step lookahead)
Input : estimate x̂, covariance P, current UAV position u_k, weights κ, w
Output: next observation station u*
1  x̂⁺, P⁺ ← EKF predict(x̂, P, odometry, gyro)
2  for each candidate u ∈ U(x̂) do
3      p_vis ← bilinear lookup of the calibrated visibility table at (u − x̂_p)
4      H ← bearing Jacobian of the two markers, linearised at x̂⁺
5      I ← (P⁺)⁻¹ + p_vis · Hᵀ R⁻¹ H
6      σ_θ ← sqrt([I⁻¹]_θθ);  σ_p ← sqrt(tr([I⁻¹]_pp))
7      cost(u) ← σ_θ + κ σ_p + w ||u − u_k||
8  end for
9  return argmin cost
```

Each decision requires 120 inversions of a 4×4 matrix; measured on a laptop CPU the complete loop (EKF prediction, 120 evaluations, selection) runs at about 0.15 ms per step, two orders of magnitude below the 50 ms navigation budget targeted by the application.

---

### 2.6. Consistency-Preserving Fusion

The UAV-side and the link-side estimators share the same bearings, hence their errors are correlated; fusing them as independent double-counts the information and yields overconfident covariances [2,3,10]. This is the same failure mode that motivated observability-constrained filter design in cooperative localisation [1]: the filter does not diverge immediately, it merely becomes confidently wrong, and the error surfaces later as a data-association or manoeuvre failure. We therefore fuse with covariance intersection,

$$
\mathbf{P}_f^{-1}=\omega\mathbf{P}_1^{-1}+(1-\omega)\mathbf{P}_2^{-1},
\qquad \omega=\tfrac{1}{2},
\tag{11}
$$

which is consistent for any unknown correlation at the price of conservatism, and we quantify consistency with the normalised estimation error squared over the position block,

$$
\mathrm{NEES}_k=(\hat{\mathbf{p}}_k-\mathbf{p}_k)^{\top}\mathbf{P}_{f,k}^{-1}(\hat{\mathbf{p}}_k-\mathbf{p}_k),
\tag{12}
$$

whose 95% upper bound is 5.991 for two degrees of freedom, corresponding to a nominal exceedance rate of 5%.

---

### 2.7. Simulation Setup and Evaluation Protocol

All quantitative results reported in Section 3 come from a purpose-built numerical simulator that reproduces (1)–(12) at $\Delta t=0.1$ s over $T=150$ s per trial. The robot follows a speed profile $v=0.6\,(1+0.05\sin(2\pi t/17))$ m/s with a yaw rate $\omega=0.30\sin(2\pi t/30)$ rad/s, and carries a gyro bias of 0.010 rad/s with a random walk of $2\times10^{-5}$ rad/s per step; both speed and yaw rate are corrupted before entering the filter. Initial uncertainties are 0.20 m in position, 0.02 rad in heading and 0.010 rad/s in bias. The UAV is a kinematic point mass limited to 3 m/s and 20 m from the robot; its motion is not otherwise modelled, because the question addressed here is where to observe, not how to fly. Table 2 lists the parameters.

**Table 2.** Simulation parameters.

| Parameter | Value |
|---|---|
| Time step / horizon | 0.1 s / 150 s |
| Robot speed | 0.6 m/s, ±5% sinusoidal |
| Yaw rate | 0.30 sin(2πt/30) rad/s |
| Gyro bias | 0.010 rad/s, random walk 2×10⁻⁵ rad/s |
| Bearing noise $\sigma_\beta$ | 0.008 rad |
| Marker baseline $L$ | 1.0 m |
| Candidate stations | 20 bearings × 6 distances = 120 |
| Planner weights | $\kappa=0.2$, $w=0.004$ |
| Visibility table | 61×61 grid, 0.5 m spacing, ±15 m, $k=1.0$ m⁻¹ |
| Seeds per configuration | 15 |
| Packet-loss levels | 0, 10, 20, 35, 50% |

Four baselines are compared. *Hover* keeps the UAV fixed. *Constant-distance following* places it 10 m to the side of the robot in the body frame — the industry convention — and *constant look-ahead* places it 10 m ahead in the body frame. Both constant-station baselines are additionally swept over $r\in\{5,8,10,12,15,20\}$ m and reported at their best setting, so that the comparison does not depend on an arbitrary choice of station distance. *Random manoeuvring* redirects the UAV to a random station every 40 steps. All body-frame offsets are computed with respect to the *estimated* pose, as they would be on a real system; computing them with respect to the true pose would silently turn the baselines into cheating variants. Reported metrics are heading RMSE (deg), position RMSE (m), mean per-step visibility, and UAV flight distance (m); each configuration is repeated over 15 seeds and we report means, with standard deviations in the figures.

For reproducibility we additionally provide a Gazebo Fortress world containing the same parameterised orchard (126 trees at the geometry of §2.3) and a differential-drive platform with a 1.0 m marker baseline, together with the ROS 2 launch files and the visibility tables. These assets are not used to produce the numbers in Section 3 — the closed-loop Gazebo experiments are ongoing work, described in Section 5 — but they fix the geometry in an executable form and allow the calibration of §2.3 to be re-run end-to-end.

The reported metrics are defined over the $N=1500$ steps of a trial as

$$
\mathrm{RMSE}_\theta=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\left(\hat{\theta}_k-\theta_k\right)^{2}},
\qquad
\mathrm{RMSE}_p=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\left\lVert\hat{\mathbf{p}}_k-\mathbf{p}_k\right\rVert^{2}},
\qquad
\bar{p}_{\mathrm{vis}}=\frac{1}{N}\sum_{k=1}^{N}p_{\mathrm{vis}}(\mathbf{u}_k),
\tag{13}
$$

with the heading error wrapped to $(-\pi,\pi]$, and the flight distance is the accumulated UAV path length. Heading and position are reported separately rather than as a single composite score because, as §3 shows, they are not minimised by the same station and the trade-off is the substance of the design problem. Trials that differ only in random seed share the same orchard instance, so variations reported in the figures are due to sensing and motion noise rather than to orchard geometry.

## 3. Results

### 3.1. Calibrated visibility: a one-sided occlusion structure

Figure 3a shows the calibrated visibility field for the open-alley scenario. Three features are immediately visible, and all three contradict the assumptions commonly made in this literature.

**Figure 3.** Calibrated visibility and its one-sided occlusion structure. **(a)** Visibility field for the open-alley scenario (*k* = 1.0 m⁻¹, *z* = 6 m, mean of 20 realisations). The bright band along *y* = 0 is the alley corridor; occlusion sets in abruptly beyond the hedge line. **(b)** Across-row profiles for four canopy semi-axes *b*. Widening the canopy narrows the across-row window from *Y*<sub>50</sub> = 2.5 m (*b* = 1.2 m) to 0.5 m (*b* = 2.0 m, canopies meeting above the alley); the dots mark the *p*<sub>vis</sub> = 0.5 crossing. Along-row profiles are flat at unity in every case and are therefore not shown; see Table 3.

First, **the alley is an unobstructed viewing corridor**. Along the alley centreline the sight line from a UAV at any altitude between 2 m and 6 m to the markers never enters a canopy, because the segment stays in the plane $y=0$ while the canopy inner faces sit at $y=\pm0.8$ m. Consequently $X_{50}$, the largest along-row offset at which $p_{\mathrm{vis}}\ge0.5$, saturates at the 15 m scan boundary in *every* configuration tested: canopy semi-axis $b\in\{1.2,1.5,1.8,2.0\}$ m, UAV altitude $z\in\{2,3,4.5,6\}$ m, and extinction $k\in\{0.5,\dots,2.0\}$ m⁻¹ (Table 3, columns $X_{50}$). Along-row occlusion, in other words, is not a property of the orchard at all for alley-centre markers; it is a property of markers placed near a hedgerow.

Second, **occlusion is entirely one-sided, and it sets in as a cliff rather than a decay**. The across-row half-visibility distance $Y_{50}$ (Figure 3b) falls from 2.5 m in the open alley to 0.5 m under a closed canopy ($b=2.0$ m, canopies meeting above the alley), with the nadir visibility $p_0$ dropping from 1.00 to 0.81. Lower flight makes the cross-row view worse, not better: $Y_{50}$ is 2.5 m at $z=6$ m and 1.0 m at $z=2$ m, because a descending UAV crosses the canopy band at a shallower angle. The extinction coefficient, by contrast, barely moves the descriptors: $Y_{50}$ stays between 2.5 m and 3.0 m over a fourfold change in $k$. The map is therefore a geometric object rather than a photometric one.

**Table 3.** Visibility descriptors (means over 20 orchard realisations; `alley` scenario, $k=1.0$ m⁻¹). $X_{50}$ is a lower bound where marked.

**(a) Canopy width sweep**

| Canopy semi-axis $b$ (m) | $Y_{50}$ (m) | $p_0$ | $X_{50}$ (m) |
|---|---|---|---|
| 1.2 (open alley) | 2.5 | 1.00 | ≥15 |
| 1.5 | 2.0 | 1.00 | ≥15 |
| 1.8 | 1.0 | 0.94 | ≥15 |
| 2.0 (closed canopy) | 0.5 | 0.81 | ≥15 |

**(b) UAV altitude sweep**

| UAV altitude $z$ (m) | $Y_{50}$ (m) | $X_{50}$ (m) |
|---|---|---|
| 2.0 | 1.0 | ≥15 |
| 3.0 | 1.0 | ≥15 |
| 4.5 | 1.5 | ≥15 |
| 6.0 | 2.5 | ≥15 |

Third, **the along-row attenuation that motivates axial viewing can be recovered, but only by moving the markers**. Figure 4 reports the offset sweep: displacing the markers 1.0 m toward the hedge (canopy inner edge at 0.8 m) brings $X_{50}$ down from ≥15 m to 4.0 m, at the cost of $p_0=0.55$ and a narrowed cross-row window ($Y_{50}=3.5$ m); a 0.5 m offset changes nothing ($X_{50}$ still saturated). This is the quantitative version of a qualitative claim that is often asserted rather than measured: axial occlusion appears when the vehicle hugs the hedge, not when it drives down the middle of the alley. Both regimes are carried through the remainder of the paper as the `alley` and `edge` scenarios.

**Figure 4.** Along-row attenuation appears only when the markers approach a canopy face. **(a)** Along-row visibility profiles for three marker lateral offsets *d* (profile taken along the marker line *y* = *d*). With the markers on the centreline (*d* = 0) or 0.5 m off it, visibility remains unity across the whole scan range; at *d* = 1.0 m — just inside the canopy inner edge at 0.8 m — attenuation appears and *X*<sub>50</sub> drops to 4.0 m. **(b)** The two descriptors as a function of *d*: *X*<sub>50</sub> (bars, left axis; ≥15 m means no measurable decay within the scan range) and the nadir visibility *p*<sub>0</sub> (line, right axis). Recovering along-row occlusion costs roughly 45% of the nadir visibility, which is why the paper carries both regimes (`alley`, `edge`) instead of assuming one.

Finally, the conventional separable-exponential model is rejected on two independent grounds. Fitted to the calibrated maps in the linear least-squares sense with a free amplitude, it returns amplitudes up to 1.18, which is impossible for a probability, and negative coefficients of determination in every regime ($R^2\in[-1.07,-0.57]$), because a plateau-and-cliff surface is outside the family. And the fitted lengths are not stable: $\lambda_x$ moves by a factor of 1.7 across the $k$ sweep while $X_{50}$ and $Y_{50}$ do not. Reporting $\lambda$ as a "calibrated" quantity would therefore convey a precision the data does not support, which is why Section 2.4 replaces it with the lookup table.

---

### 3.2. Strategy comparison at the recommended configuration

Table 4 reports the five policies over 15 seeds with $\kappa=0.2$ and $w=0.004$. The pattern is identical in the two visibility scenarios, so the discussion applies to both unless noted.

**Table 4.** Strategy comparison. Heading and position RMSE (mean ± 1σ), mean per-step visibility, and UAV flight distance over 150 s. Recommended configuration ($\kappa=0.2$, $w=0.004$), 15 seeds. Baselines are reported at the conventional 10 m station distance; Section 3.7 reports them at their own best station distance.

| Policy | `alley` heading (°) | `alley` position (m) | `alley` $\bar p_{vis}$ | `alley` distance (m) | `edge` heading (°) | `edge` position (m) |
|---|---|---|---|---|---|---|
| Hover | 1.25 ± 0.35 | 0.136 ± 0.058 | 0.04 | 0 | 1.34 ± 0.45 | 0.134 ± 0.041 |
| Constant distance, 10 m lateral | 1.00 ± 0.10 | 0.060 ± 0.007 | 0.20 | 292 | 1.03 ± 0.21 | 0.060 ± 0.009 |
| Constant look-ahead, 10 m axial | 0.72 ± 0.10 | 0.043 ± 0.006 | 0.27 | 317 | 0.72 ± 0.10 | 0.045 ± 0.006 |
| Random manoeuvre | 1.22 ± 0.25 | 0.139 ± 0.033 | 0.04 | 11 | 1.23 ± 0.27 | 0.136 ± 0.040 |
| **Proposed** | **0.32 ± 0.03** | **0.017 ± 0.002** | **0.99** | **99** | **0.36 ± 0.03** | **0.017 ± 0.002** |

Four observations.

(i) **Lateral following is the worst of the purposeful policies** (1.00° and 1.03°). It is not an unlucky parameter choice: at 10 m the lateral channel retains only 0.25% of the axial heading information (Section 2.2, Equation 6), and the mean visibility along that track is only 0.20. The intuition that following the robot at a constant distance is "safe" fails for a reason that is visible in Figure 1: the safe-looking station is the degenerate one.

(ii) **Axial look-ahead is a strong baseline** (0.72° in both scenarios). It aligns with the geometry, pays for it in visibility (0.27 in the alley, 0.16 at the edge), and accumulates 317 m of flight because the UAV must chase the heading direction as the robot yaws.

(iii) **The proposed policy is both more accurate and cheaper** (0.32°/0.36°, 99 m). Its mean visibility of 0.99 in the alley is not an artefact of the metric: the planner selects short, well-visible stations, and the heading gain follows from the position gain through the cross-covariance terms of Equation 10. In the `edge` scenario visibility is lower (0.84) but heading is essentially unchanged (0.36°), which is the expected behaviour — the markers are partly under the canopy, so more of the time is spent in occlusion, but the chosen geometry is still axial enough.

(iv) **Hover and random are statistically indistinguishable** (1.25 ± 0.35 versus 1.22 ± 0.25 in the alley). The difference is well inside one standard deviation, and we do not claim otherwise; both are dominated by dead reckoning.

The ablation with $\kappa=0$ (heading term only) is worth stating explicitly, because it isolates where the improvement comes from: heading 0.73°/0.76° at 95 m, i.e. statistically the same as the axial baseline but with a third of the flight. Adding the position term is what converts parity into dominance, and Section 3.4 shows why.

---

### 3.3. Accuracy–range trade-off

Sweeping the motion weight $w$ traces the trade-off between heading accuracy and flight distance (Figure 5). At $w=0$ the planner ignores manoeuvre effort entirely: heading reaches 0.22° at the price of 449 m of flight — more than either fixed-station baseline. At $w=0.001$ the policy settles into a well-visible short-range regime and delivers 0.37°/0.38° for 109/102 m. Beyond $w=0.004$ the curve flattens: 0.84°/0.88° at 91.5 m, with no further range saving worth having.

**Figure 5.** Heading accuracy versus UAV flight distance over 150 s trials. The proposed policy is swept over the manoeuvre weight *w* (colour and line, points at *w* = 0, 0.001, 0.004, 0.02, 0.05); the four fixed policies are single points. **(a)** Open-alley scenario; **(b)** markers at the canopy edge. In both scenarios the proposed curve reaches the accuracy of the axial baseline at about one third of its flight distance, and the two unsteered policies (hover, random) are not dominated because they barely fly at all. The heading-only family (*κ* = 0) is shown; adding the position term shifts the whole curve down (Figure 6).

Against the fixed-station baselines this yields Pareto dominance (Table 5): at $w=0.001$ the proposed policy is better than the lateral baseline and the axial baseline in *both* coordinates, achieving their accuracy at roughly one third of the flight distance. Hover and random are not dominated — they fly 0 m and 11 m respectively, which no following policy can beat, and we do not claim they are.

**Table 5.** Pareto check at $\kappa=0$ (heading-only family). "Dominated" requires both lower heading RMSE and no more flight distance. `alley` scenario, 15 seeds.

| Baseline | Baseline (heading, distance) | Dominated? | Best dominating point |
|---|---|---|---|
| Constant distance, 10 m | (1.00°, 292 m) | Yes | $w=0.001$: (0.37°, 109 m) |
| Constant look-ahead, 10 m | (0.72°, 317 m) | Yes | $w=0.001$: (0.37°, 109 m) |
| Hover | (1.25°, 0 m) | No | — |
| Random | (1.22°, 11 m) | No | — |

---

### 3.4. Why the position term is not decoration

Sweeping $\kappa$ at fixed $w=0.004$ produces the monotone improvement summarised in Table 6 and Figure 6. Three regimes are distinguishable.

**Figure 6.** Effect of the position weight *κ* at fixed *w* = 0.004, 15 seeds. **(a)** Heading RMSE (left axis, solid) and position RMSE (right axis, dotted) versus *κ*, for both scenarios; the dashed line is the constant look-ahead baseline at its conventional 10 m station distance. Below *κ* ≈ 0.05 the planner selects geometrically favourable but poorly visible stations; from *κ* = 0.05–0.10 it moves to short, well-visible stations, and both errors fall together. From *κ* = 0.2 the solution saturates, which is why 0.2 is used throughout. **(b)** The corresponding mean per-step visibility: the jump from 0.31 to 0.95–0.99 at the regime switch is the mechanism behind the improvement in (a).

**Table 6.** Joint heading–position trade-off. Recommended $w=0.004$; 15 seeds; `alley` / `edge` scenarios.

| $\kappa$ | Heading (°), alley / edge | Position (m), alley / edge | Distance (m) | $\bar p_{vis}$, alley / edge |
|---|---|---|---|---|
| 0 | 0.73 / 0.76 | 0.070 / 0.073 | 95 | 0.31 / 0.33 |
| 0.05 | 0.50 / 0.51 | 0.023 / 0.023 | 99 | 0.96 / 0.94 |
| 0.10 | 0.45 / 0.44 | 0.021 / 0.022 | 99 | 0.97 / 0.95 |
| **0.20** | **0.32 / 0.36** | **0.017 / 0.017** | 99 | 0.99 / 0.84 |
| 0.35 | 0.32 / 0.36 | 0.017 / 0.017 | 99 | 0.99 / 0.84 |
| 0.60 | 0.33 / 0.36 | 0.016 / 0.017 | 104 | 0.98 / 0.83 |

Below $\kappa\approx0.05$ the policy behaves as in Section 3.2: it selects geometrically favourable stations whose visibility is only 0.31, and both heading and position suffer. Around $\kappa=0.05$–0.10 the planner moves to short, well-visible stations and the mean visibility jumps to 0.95–0.97; the position error falls by a factor of three and, notably, the heading error falls too (0.73°→0.45°). This is the coupling described in Section 2.5: the two estimates share the same measurements, so improving one improves the other through the off-diagonal covariance blocks. From $\kappa=0.2$ onward the solution saturates: $\kappa=0.2$, 0.35 and 0.6 differ by less than the seed-to-seed spread, which is the practical reason for choosing 0.2 — it is the smallest weight that reaches the plateau, leaving margin against orchard variation.

At $\kappa=0.2$ the proposed policy dominates the axial baseline in all three coordinates — heading $2.2\times$/1.99$\times$, position $2.6\times$, distance $0.31\times$ — and the two-dimensional dominance test (lower heading RMSE at no greater flight distance) is satisfied in both scenarios. We regard this as the main quantitative result of the paper, but it should be read together with Section 3.7: the dominance is over a *conventionally tuned* baseline, and the margin narrows once the baseline is allowed to tune its own station distance.

---

### 3.5. Sensitivity to packet loss

Figure 7 reports heading RMSE as the observation link drops frames at random, with all other settings unchanged.

**Figure 7.** Robustness to observation-link packet loss (15 seeds, identical to the rest of the experiments; all other settings at the recommended configuration). Heading RMSE versus the fraction of dropped frames, for the two fixed-station baselines and the proposed policy: **(a)** open-alley scenario, **(b)** markers at the canopy edge. The proposed policy varies by 7–16% across the whole 0–50% range, against 13–21% for the baselines, because a station with mean visibility near unity yields redundant observations. The percentages annotate the 0%→50% change.

In the `alley` scenario the proposed policy moves from 0.32° at 0% loss to 0.38° at 50% loss — a 16% degradation — while the lateral baseline degrades from 1.00° to 1.20° (20%) and the axial baseline from 0.72° to 0.81° (13%, but starting from twice the error). In the `edge` scenario the same pattern holds (0.36°→0.38°, 7%; lateral 1.03°→1.19°, 15%; axial 0.72°→0.87°, 21%). All loss sweeps use the same 15 seeds as the rest of the experiments.

The mechanism is straightforward and worth stating because it generalises: a policy that keeps the mean per-step visibility near 0.99 has redundant observations, so losing half of them changes little; a policy that operates at 0.16–0.27 visibility is already sampling a sparse stream, and further loss is felt directly. Robustness here is a by-product of the station choice, not of any explicit loss model, which is why it appears without being optimised for.

---

### 3.6. Fusion consistency

Figure 8 and Table 7 summarise the consistency experiment over 15 seeds at the recommended configuration ($\kappa=0.2$). Naive fusion of the two position estimates exceeds the 95% NEES bound in 7.1% of steps (alley) and 6.2% (edge), against a nominal 5% — the expected signature of double-counted information. Covariance intersection with $\omega=0.5$ brings the exceedance to 0.6% and 0.5%, roughly an order of magnitude below the nominal rate, at the cost of a slightly larger reported covariance.

**Figure 8.** Consistency of the fused position estimate over 15 seeds. **(a)** NEES versus time for the open-alley scenario (median and inter-quartile range across seeds, logarithmic ordinate): naive fusion of the two correlated position estimates sits systematically above the 95% bound (5.99, dashed), whereas covariance intersection stays below it. **(b)** Fraction of steps exceeding the 95% bound. Naive fusion exceeds it in 7.1% (alley) and 6.2% (edge) of steps against a nominal 5%; covariance intersection brings this to 0.6% and 0.5%, roughly an order of magnitude below the nominal rate.

**Table 7.** Consistency of the fused position estimate (2 degrees of freedom; 95% bound 5.991; 15 seeds × 1500 steps).

| Scenario | Fusion | Mean NEES | Exceedance rate |
|---|---|---|---|
| alley | Naive (independent) | 2.22 | 7.1% |
| alley | Covariance intersection | 1.11 | 0.6% |
| edge | Naive (independent) | 2.15 | 6.2% |
| edge | Covariance intersection | 1.07 | 0.5% |

The magnitude of the inconsistency is worth a remark. Whether naive fusion exceeds the bound by 5% or by 7% depends on how much information each observation is assumed to carry: an over-optimistic visibility model inflates that number, and the resulting covariance error compounds faster than the correlation effect alone. Calibrating visibility is therefore not only an accuracy measure but a consistency measure, which is an argument for calibration that goes beyond the heading results of Section 3.2.

---

### 3.7. Fair comparison against station-tuned baselines

The baselines above are reported at the conventional 10 m station distance, but 10 m is a convention, not an optimum. Table 8 sweeps their station distance over $r\in\{5,8,10,12,15,20\}$ m and reports the best setting of each.

**Table 8.** Baselines at their own best station distance (`alley` scenario, 15 seeds). The best setting is $r=5$ m in every case.

| Baseline | $r=5$ m | $r=10$ m | $r=20$ m | Best (heading / position / distance) |
|---|---|---|---|---|
| Lateral following | 0.80° / 0.038 m | 1.00° / 0.060 m | 1.66° / 0.133 m | 0.80° / 0.038 m / 159 m |
| Axial look-ahead | 0.51° / 0.025 m | 0.72° / 0.043 m | 1.08° / 0.085 m | 0.51° / 0.025 m / 185 m |

Two conclusions follow. First, the station-tuned baselines are considerably stronger than the conventional ones, and the margin of the proposed method shrinks accordingly. Against the best axial baseline (0.51°, 0.025 m, 185 m), the proposed policy achieves 0.32°, 0.017 m and 99 m. These are factors of 1.6 in heading, 1.5 in position and 1.9 in flight distance, rather than the 2.2/2.6/3.2 obtained against the 10 m convention. **All comparisons in the abstract and conclusions use the tuned numbers.** Reporting only the 10 m comparison would overstate the contribution, and a referee would be right to say so.

Second, the sweep independently corroborates the distance-decay analysis of Section 2.2: for both fixed-station families, closer is better, monotonically, until the flight-distance penalty dominates. The optimum of a fixed-station policy is therefore set by the safety margin rather than by an information criterion, which is precisely the freedom that the proposed planner uses.

## 4. Discussion

### 4.1. The binding constraint is geometric, and it is measurable

The visibility model used in observability-aware planning is usually inherited rather than measured: a separable exponential with two correlation lengths, chosen for analytical convenience and rarely checked against the scene it is meant to describe [5,6,12–14]. Our calibration shows that in a hedgerow orchard this family is not merely imprecise, it is the wrong shape. The measured field has a plateau, a cliff and a second low plateau, the fitted amplitude exceeds unity, and the coefficient of determination is negative in every canopy regime (Section 3.1). Those are not numerical accidents; they are the signature of a surface that no product of two Gaussians can represent, because the underlying mechanism is occlusion by discrete objects rather than attenuation through a homogeneous medium.

The practical consequence is that the descriptors which *do* survive the calibration, namely $X_{50}$, $Y_{50}$ and the nadir visibility, behave like geometric properties of the orchard. They barely move when the extinction coefficient changes by a factor of four, which means they can be transferred between seasons, cultivars and camera exposures without recalibration, whereas the fitted correlation lengths cannot. A lookup table of the measured field is therefore not a computational shortcut in place of a "proper" analytical model; it is the more honest of the two representations, and it costs about 2 min offline and constant time online (Section 2.4).

The second, more uncomfortable result is that the direction of anisotropy depends on where the markers are mounted. The literature assumption that along-row sight lines are the most heavily occluded — the premise that makes the geometric optimum and the visibility optimum conflict — holds only when the markers sit near a canopy face. With the markers on the alley centreline, the sight line never leaves the alley plane at any UAV altitude between 2 and 6 m, and along-row attenuation is absent altogether (Section 3.1, Figure 4). This is a mounting decision, not a property of the orchard, and it is worth stating plainly because it inverts the design intuition: a marker board placed in the middle of the vehicle is easier to see along the row than one placed at the vehicle's side.

### 4.2. Why the position term decides the outcome

The ablation in Section 3.4 is, in our view, the most transferable finding of the paper. Minimising the predicted heading variance alone drives the UAV to stations that are geometrically excellent and practically useless: the mean per-step visibility of that variant is 0.31, and both heading and position errors are worse than those of a naive fixed-station policy. The reason is structural rather than incidental. The heading and position blocks of $\mathbf{P}_f$ in Equation (10) are coupled through the off-diagonal terms, so a station that degrades position also degrades heading; and a myopic one-step criterion cannot see that the station it is choosing will be occluded half the time.

Once a position term of weight $\kappa=0.2$ is added, the planner retreats to short, well-visible stations, mean visibility rises to 0.95–0.99, and *both* errors fall by more than a factor of two. The improvement is therefore not a matter of tuning a trade-off between two objectives that genuinely conflict; it is a correction of a cost function that was optimising the wrong quantity. We would expect the same failure mode in any planner that scores candidate viewpoints by an information metric and treats visibility as a multiplicative afterthought rather than as part of the same posterior.

There is a corollary for how such systems are evaluated. A planner of this family can look excellent on heading error alone while operating at 0.3 visibility, and it can look mediocre on heading while being far more robust. Reporting the mean per-step visibility alongside the estimation errors (as we do in Tables 4 and 6) makes the difference visible; reporting only the error hides it.

### 4.3. Relation to observability-aware planning

The closest body of work optimises the trajectory so as to maximise an observability metric, either in closed form or as a control objective [5,6], or enriches the measurement set with bearing rates to remove degenerate directions [8]. Our problem differs in what is fixed and what is free. The measurement set here is minimal and non-negotiable: two bearings to a known baseline. The platform cannot choose an arbitrary trajectory because most of the surrounding space is occluded. What remains free is the *station*, chosen from a small candidate set at every step, and the environment, not the excitation of the platform, is what constrains it.

This is why the planner is deliberately myopic. A receding-horizon or trajectory-level formulation would in principle do better, and the cost of our one-step rule (0.15 ms per decision, Section 2.5) leaves ample room for one. We did not take that step because the experiments here are designed to isolate the effect of the visibility model and of the position term, and a longer horizon would confound the two. It is a limitation of the present study rather than a claim that myopia is optimal, and it is the first thing we would change in a closed-loop implementation.

The consistency result points in the same direction from a different angle. Naive fusion of the two correlated position estimates exceeds the 95% NEES bound in 7.1% of steps, against a nominal 5% — the classical signature of double-counted information [2,3,10], and the same failure mode that observability-constrained filtering was designed to prevent [1]. Covariance intersection restores consistency at the price of conservatism. What is worth noting is that the magnitude of the inconsistency depends on how much information each observation is *assumed* to carry: an over-optimistic visibility model inflates that number and compounds the covariance error. Calibrating visibility is therefore a consistency measure as much as an accuracy measure, which is an argument for calibration that does not appear in the accuracy tables.

### 4.4. Implications for orchard platforms

Translated into platform terms, the results make three concrete suggestions.

First, **mount the markers where the geometry is benign**. A board on the alley-centre line gives an unobstructed axial corridor and therefore the strongest heading information at any altitude; a board near the canopy face buys along-row occlusion for no benefit. The offset sweep of Figure 4 quantifies the trade: recovering the "expected" along-row attenuation costs about 45% of the nadir visibility.

Second, **do not follow at a constant distance**. The industry convention of holding station to the side of the vehicle places the UAV in the asymptotically degenerate direction, where at 10 m only 0.25% of the axial heading information remains (Equation 6). This is the worst of the purposeful policies in our experiments (1.00°/1.03°), and it is worst for a reason that does not depend on the visibility model.

Third, **prefer many short, well-visible looks to few long ones**. The optimal fixed station distance in the sweep of Section 3.7 is the shortest one tested (5 m), and the proposed policy settles at 99 m of flight for the whole 150 s trial — a third of what either fixed-station policy needs. In accuracy terms, a heading error of 0.32° accumulates to about 5.6 cm of lateral drift over a 10 m run, which together with the 1.7 cm position error keeps the platform inside the tolerance usually quoted for in-row spraying and cultivation. The margin is not large, and it is the reason we report the tuned-baseline comparison of Section 3.7 rather than the more flattering 10 m one.

### 4.5. Limitations

1. **No field data.** All numbers come from a numerical simulator driven by a geometrically calibrated visibility model. The calibration is a ray-casting computation over a parameterised orchard, not a measurement in a real one; tree density, cultivar and pruning style enter only through the parameters of Section 2.3.
2. **Two-dimensional visibility.** The field is computed at a single altitude (*z* = 6 m) and the planner queries it in the horizontal plane. The altitude sweep of Table 3 shows that the across-row window narrows as the UAV descends, so a three-dimensional table would be more accurate for low-flying operations; we did not implement one because the candidate set is planar.
3. **Weights tuned on the same scenarios.** $\kappa$ and $w$ were selected on the two scenarios reported here, and although the plateau in Figure 6 suggests the choice is not delicate, an independent tuning set would be the correct test.
4. **Baselines are a fixed-parameter family.** The constant-distance and look-ahead policies have one design parameter each, which we swept; they cannot adapt their station, and the comparison is therefore between families of different richness, not between equally flexible alternatives.
5. **Single UAV, single ground vehicle.** Task allocation and inter-UAV occlusion are out of scope.
6. **UAV position assumed known.** The asymmetry that makes the UAV useful is assumed rather than modelled: the UAV can see its own GNSS fix or follow a surveyed plan above the canopy. In a fully denied environment the UAV's own drift would have to be estimated jointly, which changes the problem.
7. **Idealised flight.** The UAV is a kinematic point mass; wind, energy and obstacle avoidance are not modelled, and no flight controller is in the loop.

### 4.6. Future work

Three extensions follow directly. The first is a closed-loop implementation. The Gazebo Fortress world described in Section 2.7 is already built and runs headless. It contains the same 126-tree geometry, a differential-drive platform with a 1.0 m marker baseline, and the calibrated tables. Closing the loop there would replace the simulator's measurement model with rendered ArUco detections, and would test whether the station-selection policy survives contact with a real detector, latency and a flight controller.

The second is a field campaign with the platforms available to us (a legged robot and a consumer quadrotor), aimed not at reproducing the accuracy numbers but at measuring the visibility field itself: a small grid of hover points, marker detections at each, and a comparison against the ray-cast prediction. That single experiment would convert the weakest assumption of the paper into a measurement, and it is the item we would prioritise before any claim of field readiness.

The third is to make the table adaptive. The orchard is treated here as stationary over a mission, which is reasonable for a single pass but not across seasons or after pruning. Learning the visibility field online from detection outcomes — a small number of Bernoulli observations per station, filtered over the grid already in use — would remove the need for a geometric prior altogether, and would connect this line of work to the learning-based approaches now common in agricultural perception [12].

## 5. Conclusions

Where a UAV should stand to observe a ground robot in a hedgerow orchard is usually decided by geometry alone, on the assumption that a station which is geometrically informative is also physically visible. This paper tested that assumption instead of adopting it, and found it false in a specific and useful way.

Three results carry the argument. First, the occlusion structure of a hedgerow orchard is measurable and one-sided. Ray-casting calibration over twenty jittered orchard realisations shows that, for markers on the alley centreline, the along-row sight line is never attenuated at any UAV altitude between 2 and 6 m. Across-row visibility, by contrast, collapses at the hedge: the half-visibility window shrinks from 2.5 m in an open alley to 0.5 m under a closed canopy. The separable-exponential visibility model assumed in this literature cannot represent that plateau-and-cliff surface. Fitted with a free amplitude, it returns amplitudes above unity and negative coefficients of determination in every regime. Its correlation lengths also drift by a factor of 1.7 across a fourfold change in extinction, while the non-parametric descriptors do not move. Reporting such lengths as "calibrated" would convey a precision the data do not support, which is why the calibrated field is carried through the paper as a lookup table rather than as a fitted law.

Second, along-row occlusion is a consequence of how the markers are mounted, not a property of the orchard. Displacing the markers 1.0 m toward a canopy face restores a finite along-row half-visibility distance of 4.0 m, at the cost of roughly 45% of the nadir visibility. For markers in the middle of the alley, the geometrically optimal direction is also fully visible. This is the natural mounting for a vehicle that must drive between rows. In that case, the "structural conflict" between geometry and visibility that motivates much of this literature simply does not arise.

Third, in the regime where the conflict does arise, the cost function matters more than the visibility model. Minimising predicted heading variance alone drives the UAV to stations whose mean visibility is 0.31; adding a position term of weight 0.2 returns it to well-visible short-range stations and improves heading and position together, because the two are coupled through the off-diagonal blocks of the predicted covariance. At that setting the policy achieves 0.32° heading and 0.017 m position with 99 m of flight, against 0.51°, 0.025 m and 185 m for the same fixed-station baseline tuned over its own station distance. Calibrating visibility also improves consistency: naive fusion of the two correlated estimates exceeds the 95% NEES bound in 7.1% of steps, covariance intersection in 0.6%.

The practical reading is compact. Mount the markers where the geometry is benign; do not follow at a constant distance, because the convention places the aircraft in the asymptotically degenerate direction; and prefer many short, well-visible observations to few long ones. The limits are equally compact. Every number here comes from a simulator driven by a geometric calibration, not from an orchard; the visibility field is two-dimensional; the weights were tuned on the scenarios reported; and the UAV's own position is assumed known. The measurement that would remove the first and largest of these limits needs only a small grid of hover points, marker detections at each, and a tape measure, and is the next thing we intend to do.

## Supplementary Materials

The Gazebo Fortress world (126-tree parameterised orchard), the differential-drive platform model with a 1.0 m
marker baseline, the ROS 2 launch files, and the calibrated visibility tables (`vis_lut.npz`) are provided as
supplementary material.

## Data Availability Statement

The simulation code, the calibrated visibility lookup table (`vis_lut.npz`), the Gazebo world and all
figure-generation scripts are openly available at https://github.com/ClYue/p1-orchard-visibility
(accessed on 4 November 2026).

## Author Contributions

Conceptualization, C.Y.; methodology, C.Y.; software, C.Y. and J.Z.; validation, Q.L. and B.P.; formal analysis,
C.Y. and K.Z.; investigation, Q.L.; data curation, K.Z.; writing—original draft preparation, C.Y.; writing—review
and editing, Q.L. and B.P.; visualization, J.Z.; supervision, C.Y.; funding acquisition, C.Y. All authors have read
and agreed to the published version of the manuscript.

## Funding

This research was funded by the Natural Science Foundation of Henan Province, China, grant number 262300422542.

## Institutional Review Board Statement

Not applicable.

## Informed Consent Statement

Not applicable.

## Acknowledgments

During the preparation of this manuscript, the authors used a large-language-model assistant for drafting and
language editing. The authors reviewed and edited the output and take full responsibility for the content of this
publication.

## Conflicts of Interest

The author declares no conflicts of interest.

## References

1. Huang, G.P.; Trawny, N.; Mourikis, A.I.; Roumeliotis, S.I. Observability-based consistent EKF estimators for multi-robot cooperative localization. *Autonomous Robots* **2011**, *30*, 99–122. https://doi.org/10.1007/s10514-010-9207-y
2. Carrillo-Arce, L.C.; Nerurkar, E.D.; Gordillo, J.L.; Roumeliotis, S.I. Decentralized multi-robot cooperative localization using covariance intersection. In Proceedings of the 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan, 3–7 November 2013; pp. 1412–1417. https://doi.org/10.1109/IROS.2013.6696534
3. Bahr, A.; Walter, M.R.; Leonard, J.J. Consistent cooperative localization. In Proceedings of the 2009 IEEE International Conference on Robotics and Automation (ICRA), Kobe, Japan, 12–17 May 2009; pp. 3415–3422. https://doi.org/10.1109/ROBOT.2009.5152859
4. Stegagno, P.; Cognetti, M.; Oriolo, G.; Bülthoff, H.H.; Franchi, A. Ground and aerial mutual localization using anonymous relative-bearing measurements. *IEEE Transactions on Robotics* **2016**, *32*, 1133–1150. https://doi.org/10.1109/TRO.2016.2593454
5. Grebe, C.; Wise, E.; Kelly, J. Observability-aware trajectory optimization: theory, viability, and state of the art. arXiv:2109.09007, 2021.
6. Böhm, C.; Brault, P.; Delamare, Q.; Robuffo Giordano, P.; Weiss, S. COP: Control and observability-aware planning. In Proceedings of the 2022 International Conference on Robotics and Automation (ICRA), Philadelphia, PA, USA, 23–27 May 2022; pp. 3364–3370. https://doi.org/10.1109/ICRA46639.2022.9812373
7. De, Q.; Zhuang, A.; Zhang, Y.; Qian, Z.; Zou, D. A closed-form 4-DoF inter-robot pose estimator using bearing-only measurements. arXiv:2606.26616, 2026.
8. Zheng, C.; Guo, H.; Zhao, S. A cooperative bearing-rate approach for observability-enhanced target motion estimation. arXiv:2502.08089, 2025.
9. Shin, K.; Sim, H.; Nam, S.; Kim, Y.; Hu, J.; Kim, K.-K.K. Multi-robot relative pose estimation in SE(2) with observability analysis. arXiv:2401.15313, 2024.
10. Tu, C.; Cui, X.; Liu, G.; Lu, M. Weighted covariance intersection for range-based distributed cooperative localization of multi-vehicle systems. arXiv:2508.12207, 2025.
11. Xu, H.; Wang, C.; Bo, Y.; Jiang, C.; Liu, Y.; Yang, S.; Lai, W. An aerial and ground multi-agent cooperative location framework in GNSS-challenged environments. *Remote Sensing* **2022**, *14*, 5055. https://doi.org/10.3390/rs14195055
12. Gao, P.; Fang, J.; He, J.; Ma, S.; Wen, G.; Li, Z. GRU–Transformer hybrid model for GNSS/INS integration in orchard environments. *Agriculture* **2025**, *15*, 1135. https://doi.org/10.3390/agriculture15111135
13. Tagarakis, A.C.; Filippou, E.; Kalaitzidis, D.; Benos, L.; Busato, P.; Bochtis, D. Proposing UGV and UAV systems for 3D mapping of orchard environments. *Sensors* **2022**, *22*, 1571.
14. Peng, C.; Fei, Z.; Vougioukas, S.G. GNSS-free end-of-row detection and headland maneuvering for orchard navigation using a depth camera. *Machines* **2023**, *11*, 84. https://doi.org/10.3390/machines11010084
15. Gu, R.; Luan, S.; Zhao, Y.; Zhao, S.; Kan, X.; Ma, Y.; Teng, T.; Sun, Y.; Li, C.; Sun, X.; Wang, J. Heterogeneous agricultural robots: a review of collaborative sensing and control, challenges and opportunities. *Frontiers in Plant Science* **2026**, 1878431. https://doi.org/10.3389/fpls.2026.1878431
16. Feng, T.; Chen, S.; Feng, Z.; Shen, C.; Tian, Y. Effects of canopy and multi-epoch observations on single-point positioning errors of a GNSS in coniferous and broadleaved forests. *Remote Sensing* **2021**, *13*, 2325. https://doi.org/10.3390/rs13122325
17. Japertas, S.; Mažeika, M. The influence of tree canopy on GPS positioning accuracy. *Scientific Reports* **2026**, *16*, 35943. https://doi.org/10.1038/s41598-026-68024-5
18. Tomaštík, J.; Everett, T. Static positioning under tree canopy using low-cost GNSS receivers and adapted RTKLIB software. *Sensors* **2023**, *23*, 3136. https://doi.org/10.3390/s23063136
19. Han, J.-h.; Park, C.-h.; Jang, Y.Y.; Gu, J.D.; Kim, C.Y. Performance evaluation of an autonomously driven agricultural vehicle in an orchard environment. *Sensors* **2022**, *22*, 114. https://doi.org/10.3390/s22010114
20. Zhou, H.; Wang, J.; Chen, Y.; Hu, L.; Li, Z.; Xie, F.; He, J.; Wang, P. Neural network-based SLAM/GNSS fusion localization algorithm for agricultural robots in orchard GNSS-degraded or denied environments. *Agriculture* **2025**, *15*, 1612. https://doi.org/10.3390/agriculture15151612
21. Reitbauer, E.; Schmied, C. Bridging GNSS outages with IMU and odometry: a case study for agricultural vehicles. *Sensors* **2021**, *21*, 4467. https://doi.org/10.3390/s21134467
22. Xu, H.; Zhao, G.; Bo, Y. UAV-UGV collaborative localization in GNSS-denied large-scale environments: an anchor-free VIO-UWB fusion with adaptive weighting and outlier suppression. *Drones* **2026**, *10*, 175. https://doi.org/10.3390/drones10030175
23. Chen, J.; Wang, X.; Fang, Z.; Gao, M.; Xu, Y.; Zhang, Z. Robust spatial georeferencing for UAV-UGV mobile mapping platforms in urban canyons via asymmetric GNSS/UWB fusion. *Remote Sensing* **2026**, *18*, 1967. https://doi.org/10.3390/rs18121967
24. Pritzl, V.; Yu, X.; Westerlund, T.; Štěpán, P.; Saska, M. Degradation-aware cooperative multi-modal GNSS-denied localization leveraging LiDAR-based robot detections. arXiv:2510.20480, 2025.
25. Martínez-Rozas, S.; Alejo, D.; Carpio, J.J.; Caballero, F.; Merino, L. Long duration inspection of GNSS-denied environments with a tethered UAV-UGV marsupial system. arXiv:2505.23457, 2025.
26. Miller, I.D.; Cladera, F.; Smith, T.; Taylor, C.J.; Kumar, V. Stronger together: air-ground robotic collaboration using semantics. arXiv:2206.14289, 2022.
27. De Silva, A.H.T.E.; Katupitiya, J. UAV-UGV collaborative localisation with minimum sensing. *Sensors* **2024**, *24*, 4629. https://doi.org/10.3390/s24144629
28. Katikaridis, D.; Benos, L.; Kateris, D.; Papageorgiou, E.; Karras, G.; Menexes, I.; Berruto, R.; Sørensen, C.G.; Bochtis, D. Proactive path planning using centralized UAV-UGV coordination in semi-structured agricultural environments. *Applied Sciences* **2026**, *16*, 1143. https://doi.org/10.3390/app16021143
29. Wu, L.; Chen, M.; Liang, X. Air-ground collaborative autonomous exploration and mapping method for complex multi-grain pile environments. *Sensors* **2026**, *26*, 2184. https://doi.org/10.3390/s26072184
30. Boyinine, R.; Ammapalli, J.; Chakraborty, A.; Sharma, R.; Brink, K.; Taylor, C. Navigating together: the CoNaV testbed and framework for benchmarking cooperative localization. *NAVIGATION* **2025**, *72*, e722. https://doi.org/10.33012/navi.722
31. Akhihiero, D.; Olawoye, U.; Das, S.; Gross, J. Cooperative localization for GNSS-denied subterranean navigation: a UAV-UGV team approach. *NAVIGATION* **2024**, *71*, e677. https://doi.org/10.33012/navi.677
32. 耿可可; 成小龙; 丁鹏博; 沈楷传; 马天笑; 殷国栋. 空地协同系统关键技术发展现状综述. *机械工程学报* **2026**, *62*. https://doi.org/10.3901/JME.260208
33. Chang, T.-K.; Chen, K.; Mehta, A. Resilient and consistent multirobot cooperative localization with covariance intersection. arXiv:2108.08789, 2021.
34. Zhou, Y.; Liu, Y.; Wang, X. Distributed 3-D multi-robot cooperative localization: an efficient and consistent approach. *IEEE Robotics and Automation Letters* **2026**, *11*, 2306. https://doi.org/10.1109/LRA.2026.3653287
35. Hao, N.; He, F.; Tian, C.; Yao, Y.; Xia, W. KD-EKF: a consistent cooperative localization estimator based on Kalman decomposition. arXiv:2210.16086, 2022.
36. Julier, S.J.; Uhlmann, J.K. A non-divergent estimation algorithm in the presence of unknown correlations. In Proceedings of the 1997 American Control Conference, Albuquerque, NM, USA, 4-6 June 1997; Volume 4, pp. 2369-2373. https://doi.org/10.1109/ACC.1997.609105
37. Go, H.S.H.; Chong, C.L.; Qian, L.; Liu, H.H.T. Observability-aware control for quadrotor formation flight with range-only measurement. arXiv:2411.03747, 2024.
38. Wang, Q.; Li, Z.; Peng, J.; Lu, K. Bio-inspired observability enhancement method for UAV target localization and sensor bias estimation with bearing-only measurement. *Biomimetics* **2025**, *10*, 336. https://doi.org/10.3390/biomimetics10050336
39. Sun, P.; Tong, S.; Qin, K.; Luo, Z.; Lin, B.; Shi, M. Low-cost real-time remote sensing and geolocation of moving targets via monocular bearing-only micro UAVs. *Remote Sensing* **2025**, *17*, 3836. https://doi.org/10.3390/rs17233836
40. Michaelson, K.; Gandhi, M.; Zanetti, R. Navigation-aware path planning and multi-agent coordination in challenging environments. In Proceedings of the 2025 IEEE/ION Position, Location and Navigation Symposium (PLANS), Salt Lake City, UT, USA, 28 April-1 May 2025; pp. 1319-1326. https://doi.org/10.1109/PLANS61210.2025.11028399
41. Garrido-Jurado, S.; Muñoz-Salinas, R.; Madrid-Cuevas, F.J.; Marín-Jiménez, M.J. Automatic generation and detection of highly reliable fiducial markers under occlusion. *Pattern Recognition* **2014**, *47*, 2280-2292. https://doi.org/10.1016/j.patcog.2014.01.005