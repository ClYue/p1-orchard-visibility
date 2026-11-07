# Cover letter

Dear Editors,

Please consider our manuscript “Canopy Visibility Constrains UAV Viewpoint Selection: A Simulation Study in Orchards” for publication as an Article in *Drones*. The question is one of flight decisions rather than estimator design, which is why we are sending it here rather than to a venue focused on estimation.

A UAV that has to watch a ground robot through an orchard canopy must decide where to stand. Our simulator keeps the canopy explicit: each detection probability comes from a ray cast against the tree crowns, so occlusion is geometric rather than assumed. We used it to ask a narrow question. Does it matter how the planner represents what it can see? The same planner was given three representations of the same field (a lookup table, a fitted Gaussian, a uniform prior), allowed to choose among 120 candidate stations, and then scored against 48 fixed stations swept over eight azimuths and six distances.

The answer depends on the geometry, and not always in the expected direction. On the alley centerline the field can barely be improved on, and a plain forward station at 2 m already tops the fixed set. Shift the ground path 1 m sideways, which puts the robot 0.2 m inside the crown envelope, and the near side becomes almost unusable: paired detections fall to 1.3–2.3% of attempts, and the field there is an off-center band that a centered Gaussian cannot follow. Even so, the lookup-table policy saved 4.87 m of flight (15.6%) and paid 0.187° of heading error for it, while a tuned oblique fixed station stayed more accurate. That negative outcome is, we think, the useful part: it tells a designer when a richer visibility model pays, and it puts a number on the trade between UAV travel and localization accuracy.

The manuscript is original, has not been published previously, and is not under consideration elsewhere. All authors have approved the submission and declare no conflicts of interest. Chunlong Yue is the corresponding author. The supplementary archive contains the simulator, the lookup tables, the seed-level results and the plotting scripts, and the same material is available at https://github.com/ClYue/p1-orchard-visibility.

Sincerely,  
Chunlong Yue, on behalf of all authors  
College of Energy and Intelligent Engineering  
Henan University of Animal Husbandry and Economy, Zhengzhou 450000, China  
Correspondence: ycl@hnuahe.edu.cn (Chunlong Yue)
