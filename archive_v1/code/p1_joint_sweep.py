# -*- coding: utf-8 -*-
"""
P1 位置—航向双目标扫描（LUT 可见性）
====================================
问题：固定权重下 ours 航向与 C 打平，但位置精度不如 C（0.070 vs 0.043 m）。
本脚本给代价函数加位置项 κ·σ_p，扫 κ 看能否同时压住航向与位置，
并检验是否出现"二维支配"C 的点（航向 < C 且 位置 < C）。

代价：J = σθ + κ·sqrt(tr(P_pp)) + w·‖u-u_k‖
（σθ ~ 0.012 rad，σp ~ 0.05 m，故 κ 的量级在 0.05–0.6 之间。）

用法：python p1_joint_sweep.py [--seeds 15] [--wmove 0.004]
输出：joint_lut.json、figures_lut_<case>/fig_joint.png
"""
import argparse
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun"]
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p1_sim as S  # noqa: E402

OUT = S.OUT
KAPPA = [0.0, 0.05, 0.1, 0.2, 0.35, 0.6]


def eval_ours(seeds):
    rs = [S.run_metrics("ours", s) for s in range(seeds)]
    return dict(head=float(np.mean([r["head_rmse"] for r in rs])),
                head_std=float(np.std([r["head_rmse"] for r in rs])),
                pos=float(np.mean([r["pos_rmse"] for r in rs])),
                pos_std=float(np.std([r["pos_rmse"] for r in rs])),
                pvis=float(np.mean([r["mean_pvis"] for r in rs])),
                dist=float(np.mean([r["dist"] for r in rs])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=15)
    ap.add_argument("--wmove", type=float, default=0.004)
    args = ap.parse_args()

    lut_path = os.path.normpath(os.path.join(OUT, "..", "..", "08_Gazebo仿真", "vis_lut.npz"))
    S.CFG["w_move"] = args.wmove
    all_res = {}

    for case in ("alley_center", "canopy_edge"):
        S._lut_load(lut_path, case)
        S.VIS["mode"] = "lut"
        S.VIS["case"] = case
        S.PVIS_TAB = S.build_pvis_tab()
        S.FIG = os.path.join(OUT, f"figures_lut_{case}")
        os.makedirs(S.FIG, exist_ok=True)

        base = {}
        for k in ("follow10", "axis10"):
            rs = [S.run_metrics(k, s) for s in range(args.seeds)]
            base[k] = dict(head=float(np.mean([r["head_rmse"] for r in rs])),
                           pos=float(np.mean([r["pos_rmse"] for r in rs])),
                           dist=float(np.mean([r["dist"] for r in rs])))

        pts = []
        for kappa in KAPPA:
            S.CFG["w_pos"] = kappa
            r = eval_ours(args.seeds)
            r["kappa"] = kappa
            pts.append(r)
            print(f"[{case}] kappa={kappa:<5} head={r['head']:.3f} deg  pos={r['pos']:.4f} m  "
                  f"dist={r['dist']:.1f} m  pvis={r['pvis']:.3f}", flush=True)
        S.CFG["w_pos"] = 0.0

        c = base["axis10"]
        dom = [p for p in pts if p["head"] < c["head"] and p["pos"] < c["pos"] and p["dist"] <= c["dist"]]
        all_res[case] = dict(baselines=base, ours=pts,
                             dominates_axis10_2d=bool(dom),
                             best_dom_point=(min(dom, key=lambda p: p["head"] + 100 * p["pos"])["kappa"] if dom else None))

        fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=160)
        sc = ax.scatter([p["pos"] for p in pts], [p["head"] for p in pts],
                        c=[p["dist"] for p in pts], cmap="viridis", s=70, zorder=6)
        fig.colorbar(sc, ax=ax, label="无人机航程 / m")
        ax.plot([p["pos"] for p in pts], [p["head"] for p in pts], "--", color="gray", lw=1, alpha=.7)
        for p in pts:
            ax.annotate(f"κ={p['kappa']}", (p["pos"], p["head"]), fontsize=7,
                        xytext=(5, 4), textcoords="offset points")
        for b, col, lab in (("axis10", "#c44e52", "C 沿轴前视 10 m"), ("follow10", "#55a868", "B 定距跟随 10 m")):
            ax.scatter([base[b]["pos"]], [base[b]["head"]], marker="D" if b == "axis10" else "^",
                       s=70, color=col, label=lab, zorder=7)
        ax.set_xlabel("位置 RMSE / m")
        ax.set_ylabel("航向 RMSE / deg")
        ax.set_title(f"位置—航向双目标（LUT: {case}，w_move={args.wmove}，{args.seeds} 次）", fontsize=10)
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(S.FIG, "fig_joint.png"))
        plt.close(fig)
        print(f"[{case}] 2D 支配 C: {all_res[case]['dominates_axis10_2d']} "
              f"(最佳 κ={all_res[case]['best_dom_point']})", flush=True)

    json.dump(all_res, open(os.path.join(OUT, "joint_lut.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("ALL DONE -> joint_lut.json", flush=True)


if __name__ == "__main__":
    main()
