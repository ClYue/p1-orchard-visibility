# -*- coding: utf-8 -*-
"""P1 稿件 42 条参考文献的结构化记录（唯一数据源）。

字段约定
    n     稿件中的编号（1–42），必须与 Manuscript.md 参考文献表一致
    type  article | preprint | conference
    a     作者，写法与稿件一致："姓, 名"；中文作者直接写全名
    t     标题
    v     期刊名（article）/ 会议名（conference）
    y     年份
    vol / no / p    卷 / 期 / 页码（或文章号）
    doi   DOI（不含 URL 前缀）
    arx   arXiv 编号
    place / date    会议地点与会期
    lang  zh 时额外给出 t_en（英文题名），稿件里以方括号并列
"""

REFS = [
    dict(n=1, type="article", key="feng2021canopy",
         a=["Feng, T.", "Chen, S.", "Feng, Z.", "Shen, C.", "Tian, Y."],
         t="Effects of canopy and multi-epoch observations on single-point positioning errors of a GNSS in coniferous and broadleaved forests",
         v="Remote Sensing", y=2021, vol="13", p="2325", doi="10.3390/rs13122325"),

    dict(n=2, type="article", key="japertas2026influence",
         a=["Japertas, S.", "Mažeika, M."],
         t="The influence of tree canopy on GPS positioning accuracy",
         v="Scientific Reports", y=2026, vol="16", p="35943", doi="10.1038/s41598-026-68024-5"),

    dict(n=3, type="article", key="tomastik2023static",
         a=["Tomaštík, J.", "Everett, T."],
         t="Static positioning under tree canopy using low-cost GNSS receivers and adapted RTKLIB software",
         v="Sensors", y=2023, vol="23", p="3136", doi="10.3390/s23063136"),

    dict(n=4, type="article", key="han2022performance",
         a=["Han, J.-h.", "Park, C.-h.", "Jang, Y.Y.", "Gu, J.D.", "Kim, C.Y."],
         t="Performance evaluation of an autonomously driven agricultural vehicle in an orchard environment",
         v="Sensors", y=2022, vol="22", p="114", doi="10.3390/s22010114"),

    dict(n=5, type="article", key="zhou2025neural",
         a=["Zhou, H.", "Wang, J.", "Chen, Y.", "Hu, L.", "Li, Z.", "Xie, F.", "He, J.", "Wang, P."],
         t="Neural network-based SLAM/GNSS fusion localization algorithm for agricultural robots in orchard GNSS-degraded or denied environments",
         v="Agriculture", y=2025, vol="15", p="1612", doi="10.3390/agriculture15151612"),

    dict(n=6, type="article", key="gao2025gru",
         a=["Gao, P.", "Fang, J.", "He, J.", "Ma, S.", "Wen, G.", "Li, Z."],
         t="GRU–Transformer hybrid model for GNSS/INS integration in orchard environments",
         v="Agriculture", y=2025, vol="15", p="1135", doi="10.3390/agriculture15111135"),

    dict(n=7, type="article", key="reitbauer2021bridging",
         a=["Reitbauer, E.", "Schmied, C."],
         t="Bridging GNSS outages with IMU and odometry: a case study for agricultural vehicles",
         v="Sensors", y=2021, vol="21", p="4467", doi="10.3390/s21134467"),

    dict(n=8, type="article", key="gu2026heterogeneous",
         a=["Gu, R.", "Luan, S.", "Zhao, Y.", "Zhao, S.", "Kan, X.", "Ma, Y.", "Teng, T.",
            "Sun, Y.", "Li, C.", "Sun, X.", "Wang, J."],
         t="Heterogeneous agricultural robots: a review of collaborative sensing and control, challenges and opportunities",
         v="Frontiers in Plant Science", y=2026, p="1878431", doi="10.3389/fpls.2026.1878431"),

    dict(n=9, type="article", key="geng2026airground", lang="zh",
         a=["耿可可", "成小龙", "丁鹏博", "沈楷传", "马天笑", "殷国栋"],
         t="空地协同系统关键技术发展现状综述",
         t_en="Review of the Development Status of Crucial Technologies for Air-ground Collaboration System",
         v="机械工程学报", y=2026, vol="62", no="10", p="253–272", doi="10.3901/JME.260208"),

    dict(n=10, type="article", key="stegagno2016ground",
         a=["Stegagno, P.", "Cognetti, M.", "Oriolo, G.", "Bülthoff, H.H.", "Franchi, A."],
         t="Ground and aerial mutual localization using anonymous relative-bearing measurements",
         v="IEEE Transactions on Robotics", y=2016, vol="32", p="1133–1151", doi="10.1109/TRO.2016.2593454"),

    dict(n=11, type="article", key="xu2022aerial",
         a=["Xu, H.", "Wang, C.", "Bo, Y.", "Jiang, C.", "Liu, Y.", "Yang, S.", "Lai, W."],
         t="An aerial and ground multi-agent cooperative location framework in GNSS-challenged environments",
         v="Remote Sensing", y=2022, vol="14", p="5055", doi="10.3390/rs14195055"),

    dict(n=12, type="article", key="tagarakis2022proposing",
         a=["Tagarakis, A.C.", "Filippou, E.", "Kalaitzidis, D.", "Benos, L.", "Busato, P.", "Bochtis, D."],
         t="Proposing UGV and UAV systems for 3D mapping of orchard environments",
         v="Sensors", y=2022, vol="22", p="1571"),

    dict(n=13, type="article", key="peng2023gnssfree",
         a=["Peng, C.", "Fei, Z.", "Vougioukas, S.G."],
         t="GNSS-free end-of-row detection and headland maneuvering for orchard navigation using a depth camera",
         v="Machines", y=2023, vol="11", p="84", doi="10.3390/machines11010084"),

    dict(n=14, type="conference", key="carrilloarce2013decentralized",
         a=["Carrillo-Arce, L.C.", "Nerurkar, E.D.", "Gordillo, J.L.", "Roumeliotis, S.I."],
         t="Decentralized multi-robot cooperative localization using covariance intersection",
         v="2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)",
         y=2013, place="Tokyo, Japan", date="3–7 November 2013", p="1412–1417",
         doi="10.1109/IROS.2013.6696534"),

    dict(n=15, type="article", key="huang2011observability",
         a=["Huang, G.P.", "Trawny, N.", "Mourikis, A.I.", "Roumeliotis, S.I."],
         t="Observability-based consistent EKF estimators for multi-robot cooperative localization",
         v="Autonomous Robots", y=2011, vol="30", p="99–122", doi="10.1007/s10514-010-9207-y"),

    dict(n=16, type="article", key="tu2025weighted",
         a=["Tu, C.", "Cui, X.", "Liu, G.", "Lu, M."],
         t="Weighted covariance intersection for range-based distributed cooperative localization of multi-vehicle systems",
         v="IEEE Transactions on Vehicular Technology", y=2026,
         doi="10.1109/TVT.2026.3715359"),

    dict(n=17, type="article", key="chang2021resilient",
         a=["Chang, T.-K.", "Chen, K.", "Mehta, A."],
         t="Resilient and consistent multirobot cooperative localization with covariance intersection",
         v="IEEE Transactions on Robotics", y=2022,
         vol="38", no="1", p="197–208", doi="10.1109/TRO.2021.3104965"),

    dict(n=18, type="conference", key="julier1997nondivergent",
         a=["Julier, S.J.", "Uhlmann, J.K."],
         t="A non-divergent estimation algorithm in the presence of unknown correlations",
         v="Proceedings of the 1997 American Control Conference",
         y=1997, place="Albuquerque, NM, USA", date="4–6 June 1997",
         vol="4", p="2369–2373", doi="10.1109/ACC.1997.609105"),

    dict(n=19, type="article", key="zhou2026distributed",
         a=["Zhou, Y.", "Liu, Y.", "Wang, X."],
         t="Distributed 3-D multi-robot cooperative localization: an efficient and consistent approach",
         v="IEEE Robotics and Automation Letters", y=2026, vol="11", p="2306", doi="10.1109/LRA.2026.3653287"),

    dict(n=20, type="conference", key="hao2022kdekf",
         a=["Hao, N.", "He, F.", "Tian, C.", "Yao, Y.", "Xia, W."],
         t="KD-EKF: a consistent cooperative localization estimator based on Kalman decomposition",
         v="2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)", y=2023,
         p="11064–11070", place="Detroit, MI, USA", doi="10.1109/IROS55552.2023.10341604"),

    dict(n=21, type="article", key="de2026closedform",
         a=["De, Q.", "Zhuang, A.", "Zhang, Y.", "Qian, Z.", "Zou, D."],
         t="A closed-form 4-DoF inter-robot pose estimator using bearing-only measurements",
         v="IEEE Robotics and Automation Letters", y=2026,
         vol="11", no="9", p="10465–10472", doi="10.1109/LRA.2026.3710047"),

    dict(n=22, type="conference", key="zheng2025cooperative",
         a=["Zheng, C.", "Guo, H.", "Zhao, S."],
         t="A cooperative bearing-rate approach for observability-enhanced target motion estimation",
         v="2025 IEEE International Conference on Robotics and Automation (ICRA)", y=2025,
         p="12579–12585", place="Atlanta, GA, USA", doi="10.1109/ICRA55743.2025.11128217"),

    dict(n=23, type="article", key="shin2024multirobot",
         a=["Shin, K.", "Sim, H.", "Nam, S.", "Kim, Y.", "Hu, J.", "Kim, K.-K.K."],
         t="Multi-robot relative pose estimation in SE(2) with observability analysis: a comparison of extended Kalman filtering and robust pose graph optimization",
         v="IEEE Transactions on Intelligent Vehicles", y=2025,
         vol="10", no="8", p="4293–4315", doi="10.1109/TIV.2024.3478290"),

    dict(n=24, type="article", key="desilva2024uavugv",
         a=["De Silva, A.H.T.E.", "Katupitiya, J."],
         t="UAV-UGV collaborative localisation with minimum sensing",
         v="Sensors", y=2024, vol="24", p="4629", doi="10.3390/s24144629"),

    dict(n=25, type="preprint", key="pritzl2025degradation",
         a=["Pritzl, V.", "Yu, X.", "Westerlund, T.", "Štěpán, P.", "Saska, M."],
         t="Degradation-aware cooperative multi-modal GNSS-denied localization leveraging LiDAR-based robot detections",
         y=2025, arx="2510.20480"),

    dict(n=26, type="article", key="martinezrozas2025long",
         a=["Martínez-Rozas, S.", "Alejo, D.", "Carpio, J.J.", "Caballero, F.", "Merino, L."],
         t="Long-duration inspection of GNSS-denied environments with a tethered UAV-UGV marsupial system",
         v="Drones", y=2025,
         vol="9", no="11", p="765", doi="10.3390/drones9110765"),

    dict(n=27, type="article", key="xu2026anchorfree",
         a=["Xu, H.", "Zhao, G.", "Bo, Y."],
         t="UAV-UGV collaborative localization in GNSS-denied large-scale environments: an anchor-free VIO-UWB fusion with adaptive weighting and outlier suppression",
         v="Drones", y=2026, vol="10", p="175", doi="10.3390/drones10030175"),

    dict(n=28, type="article", key="chen2026robust",
         a=["Chen, J.", "Wang, X.", "Fang, Z.", "Gao, M.", "Xu, Y.", "Zhang, Z."],
         t="Robust spatial georeferencing for UAV-UGV mobile mapping platforms in urban canyons via asymmetric GNSS/UWB fusion",
         v="Remote Sensing", y=2026, vol="18", p="1967", doi="10.3390/rs18121967"),

    dict(n=29, type="article", key="miller2022stronger",
         a=["Miller, I.D.", "Cladera, F.", "Smith, T.", "Taylor, C.J.", "Kumar, V."],
         t="Stronger together: air-ground robotic collaboration using semantics",
         v="IEEE Robotics and Automation Letters", y=2022,
         vol="7", no="4", p="9643–9650", doi="10.1109/LRA.2022.3191165"),

    dict(n=30, type="article", key="wu2026airground",
         a=["Wu, L.", "Chen, M.", "Liang, X."],
         t="Air-ground collaborative autonomous exploration and mapping method for complex multi-grain pile environments",
         v="Sensors", y=2026, vol="26", p="2184", doi="10.3390/s26072184"),

    dict(n=31, type="article", key="boyinine2025navigating",
         a=["Boyinine, R.", "Ammapalli, J.", "Chakraborty, A.", "Sharma, R.", "Brink, K.", "Taylor, C."],
         t="Navigating together: the CoNaV testbed and framework for benchmarking cooperative localization",
         v="NAVIGATION", y=2025, vol="72", p="e722", doi="10.33012/navi.722"),

    dict(n=32, type="article", key="katikaridis2026proactive",
         a=["Katikaridis, D.", "Benos, L.", "Kateris, D.", "Papageorgiou, E.", "Karras, G.",
            "Menexes, I.", "Berruto, R.", "Sørensen, C.G.", "Bochtis, D."],
         t="Proactive path planning using centralized UAV-UGV coordination in semi-structured agricultural environments",
         v="Applied Sciences", y=2026, vol="16", p="1143", doi="10.3390/app16021143"),

    dict(n=33, type="article", key="akhihiero2024cooperative",
         a=["Akhihiero, D.", "Olawoye, U.", "Das, S.", "Gross, J."],
         t="Cooperative localization for GNSS-denied subterranean navigation: a UAV-UGV team approach",
         v="NAVIGATION", y=2024, vol="71", p="e677", doi="10.33012/navi.677"),

    dict(n=34, type="conference", key="bahr2009consistent",
         a=["Bahr, A.", "Walter, M.R.", "Leonard, J.J."],
         t="Consistent cooperative localization",
         v="2009 IEEE International Conference on Robotics and Automation (ICRA)",
         y=2009, place="Kobe, Japan", date="12–17 May 2009", p="3415–3422",
         doi="10.1109/ROBOT.2009.5152859"),

    dict(n=35, type="conference", key="grebe2021observability",
         a=["Grebe, C.", "Wise, E.", "Kelly, J."],
         t="Observability-aware trajectory optimization: theory, viability, and state of the art",
         v="2021 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI)", y=2021,
         p="1–8", doi="10.1109/MFI52462.2021.9591177"),

    dict(n=36, type="conference", key="bohm2022cop",
         a=["Böhm, C.", "Brault, P.", "Delamare, Q.", "Robuffo Giordano, P.", "Weiss, S."],
         t="COP: Control and observability-aware planning",
         v="2022 International Conference on Robotics and Automation (ICRA)",
         y=2022, place="Philadelphia, PA, USA", date="23–27 May 2022", p="3364–3370",
         doi="10.1109/ICRA46639.2022.9812373"),

    dict(n=37, type="article", key="go2024observability",
         a=["Go, H.S.H.", "Chong, C.L.", "Qian, L.", "Liu, H.H.T."],
         t="Observability-aware control for quadrotor formation flight with range-only measurement",
         v="Journal of Guidance, Control, and Dynamics", y=2026,
         vol="49", no="7", p="2062–2077", doi="10.2514/1.G009507"),

    dict(n=38, type="article", key="wang2025bioinspired",
         a=["Wang, Q.", "Li, Z.", "Peng, J.", "Lu, K."],
         t="Bio-inspired observability enhancement method for UAV target localization and sensor bias estimation with bearing-only measurement",
         v="Biomimetics", y=2025, vol="10", p="336", doi="10.3390/biomimetics10050336"),

    dict(n=39, type="conference", key="wang2021visibility",
         a=["Wang, Q.", "Gao, Y.", "Ji, J.", "Xu, C.", "Gao, F."],
         t="Visibility-aware trajectory optimization with application to aerial tracking",
         v="IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)",
         y=2021, p="5249–5256", doi="10.1109/IROS51168.2021.9636753"),

    dict(n=40, type="conference", key="michaelson2025navigation",
         a=["Michaelson, K.", "Gandhi, M.", "Zanetti, R."],
         t="Navigation-aware path planning and multi-agent coordination in challenging environments",
         v="2025 IEEE/ION Position, Location and Navigation Symposium (PLANS)",
         y=2025, place="Salt Lake City, UT, USA", date="28 April–1 May 2025", p="1319–1326",
         doi="10.1109/PLANS61210.2025.11028399"),

    dict(n=41, type="article", key="sun2025lowcost",
         a=["Sun, P.", "Tong, S.", "Qin, K.", "Luo, Z.", "Lin, B.", "Shi, M."],
         t="Low-cost real-time remote sensing and geolocation of moving targets via monocular bearing-only micro UAVs",
         v="Remote Sensing", y=2025, vol="17", p="3836", doi="10.3390/rs17233836"),

    dict(n=42, type="article", key="garridojurado2014automatic",
         a=["Garrido-Jurado, S.", "Muñoz-Salinas, R.", "Madrid-Cuevas, F.J.", "Marín-Jiménez, M.J."],
         t="Automatic generation and detection of highly reliable fiducial markers under occlusion",
         v="Pattern Recognition", y=2014, vol="47", p="2280–2292",
         doi="10.1016/j.patcog.2014.01.005"),
]

# 中文刊名/会议名在 Zotero 里保留原文
assert len(REFS) == 42, len(REFS)
assert [r["n"] for r in REFS] == list(range(1, 43))
assert len({r["key"] for r in REFS}) == 42
