"""
导出论文图 3/4/8 所需的曲线数据（只算一次，缓存为 JSON）。

为什么单独跑：
  e1_results.json / e1b_results.json 只存了标量判据（X50/Y50/p0/lambda），
  图 3(b)（跨行窗口随冠层宽度收窄）与图 4（沿行剖面随标志点偏移）需要整条剖面；
  Fig. 8 的 NEES 需要在 p1_sim 里拿逐时刻序列（原脚本只聚合了均值和超界率）。

产物：
  figures_viz/fig_profiles.json   b 扫描 + 标志点偏移扫描的剖面
  figures_viz/nees_series.json    两工况 × 8 种子的逐时刻 NEES（naive / CI）

几何与随机种子完全复用 e1 脚本（seed 0..n-1），不引入新的模型假设。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
E1_DIR = HERE.parents[1]                      # 08_Gazebo仿真
sys.path.insert(0, str(E1_DIR / "tools"))
sys.path.insert(0, str(E1_DIR.parent / "02_P1_论文" / "code"))

from e1_occlusion_calibration import CFG, mean_pvis, half_visibility  # noqa: E402

OUT = E1_DIR / "p1_gz_sim" / "figures_viz"
OUT.mkdir(parents=True, exist_ok=True)

B_LIST = [1.2, 1.5, 1.8, 2.0]
D_LIST = [0.0, 0.5, 1.0]


def export_profiles() -> dict:
    cfg0 = dict(CFG)
    g = np.arange(-cfg0["grid_half"], cfg0["grid_half"] + 1e-9, cfg0["grid_step"])

    b_rows = []
    for b in B_LIST:
        cfg = dict(cfg0, canopy_b=b)
        pv, GX, GY = mean_pvis(cfg, cfg0["extinction_k"], cfg0["n_realizations"])
        i0 = int(np.argmin(abs(GY[:, 0])))       # y = 0（通道中线）
        j0 = int(np.argmin(abs(GX[0, :])))       # x = 0
        h = half_visibility(pv, GX, GY)
        b_rows.append({"canopy_b": b,
                       "along": pv[i0, :].round(4).tolist(),
                       "across": pv[:, j0].round(4).tolist(),
                       **h})
        print(f"[profiles] b={b}: across Y50={h['Y50_across']} p0={h['p_nadir']}")

    d_rows = []
    n_reals_d = max(10, cfg0["n_realizations"] // 2)   # 与 e1b 脚本一致
    for d in D_LIST:
        cfg = dict(cfg0, marker_y_offset=d)
        pv, GX, GY = mean_pvis(cfg, cfg0["extinction_k"], n_reals_d)
        i0 = int(np.argmin(abs(GY[:, 0] - d)))   # 剖面沿标志点所在行 y = d
        j0 = int(np.argmin(abs(GX[0, :])))
        h = half_visibility(pv, GX, GY, y_line=d)
        d_rows.append({"offset": d,
                       "along": pv[i0, :].round(4).tolist(),
                       "across": pv[:, j0].round(4).tolist(),
                       **h})
        print(f"[profiles] d={d}: X50={h['X50_along']} p0={h['p_nadir']}")

    out = {
        "note": "figure data only; scalar criteria live in e1_results.json / e1b_results.json",
        "config": {k: cfg0[k] for k in ("row_spacing", "tree_spacing", "canopy_a",
                                        "canopy_c", "uav_z", "marker_L", "grid_half",
                                        "grid_step", "extinction_k", "seed")},
        "grid": g.round(4).tolist(),
        "b_sweep": {"n_realizations": cfg0["n_realizations"], "rows": b_rows},
        "offset_sweep": {"n_realizations": n_reals_d, "rows": d_rows},
    }
    (OUT / "fig_profiles.json").write_text(
        json.dumps(out, ensure_ascii=False), encoding="utf-8")
    print(f"[profiles] -> {OUT / 'fig_profiles.json'}")
    return out


def export_nees(n_seeds: int = 15, w_pos: float = 0.2) -> None:
    import p1_sim as S

    lut = E1_DIR / "vis_lut.npz"
    store = {}
    # 2026-11-04 修复：p1_sim.CFG 的 w_pos 默认为 0.0，此前未设置 → NEES 实际算在 κ=0（仅航向）
    # 配置下，与正文声称的推荐配置 κ=0.2 不符。现显式固定为推荐配置。
    S.CFG["w_pos"] = w_pos
    for case in ("alley_center", "canopy_edge"):
        S._lut_load(str(lut), case)
        S.VIS["mode"] = "lut"
        S.VIS["case"] = case
        S.PVIS_TAB = S.build_pvis_tab()
        naive, ci = [], []
        for s in range(n_seeds):
            n_, c_ = S.run_consistency(s)
            naive.append(n_.round(4).tolist())
            ci.append(c_.round(4).tolist())
        store[case] = {"naive": naive, "ci": ci,
                       "chi2_bound_95": 5.991,
                       "exceedance": [float((np.array(naive) > 5.991).mean()),
                                      float((np.array(ci) > 5.991).mean())],
                       "mean_nees": [float(np.array(naive).mean()),
                                     float(np.array(ci).mean())]}
        print(f"[nees] {case}: mean {store[case]['mean_nees']}, "
              f"exceed {store[case]['exceedance']}")
    (OUT / "nees_series.json").write_text(
        json.dumps({"n_seeds": n_seeds, "dt": 0.1, "cases": store}), encoding="utf-8")
    print(f"[nees] -> {OUT / 'nees_series.json'}")


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("all", "profiles"):
        export_profiles()
    if what in ("all", "nees"):
        export_nees()
