# -*- coding: utf-8 -*-
"""
P1 精度—航程帕累托扫描（LUT 可见性模型）
=========================================
背景：换成 E1 标定的查找表后，"ours" 策略在固定 w_move=0.004 下
航向精度不再领先 C（沿轴前视），优势只剩航程。
正解不是推翻结果，而是扫出 w_move 的帕累托前沿：
同等航程下 ours 的航向误差是否更低（Pareto 支配）。

用法：python p1_pareto_sweep.py [--seeds 15]
输出：pareto_lut_<case>.json、figures_lut_<case>/fig_pareto.png
"""
import argparse
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p1_sim as S  # noqa: E402

OUT = S.OUT
W_LIST = [0.0, 0.001, 0.004, 0.02, 0.1, 0.5]


def eval_policy(key, seeds):
    rs = [S.run_metrics(key, s) for s in range(seeds)]
    return dict(head=float(np.mean([r["head_rmse"] for r in rs])),
                head_std=float(np.std([r["head_rmse"] for r in rs])),
                pos=float(np.mean([r["pos_rmse"] for r in rs])),
                pvis=float(np.mean([r["mean_pvis"] for r in rs])),
                dist=float(np.mean([r["dist"] for r in rs])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=15)
    args = ap.parse_args()

    lut_path = os.path.normpath(os.path.join(OUT, "..", "..", "08_Gazebo仿真", "vis_lut.npz"))
    base_w = S.CFG["w_move"]
    all_res = {}

    for case in ("alley_center", "canopy_edge"):
        S._lut_load(lut_path, case)
        S.VIS["mode"] = "lut"
        S.VIS["case"] = case
        S.PVIS_TAB = S.build_pvis_tab()
        S.FIG = os.path.join(OUT, f"figures_lut_{case}")
        os.makedirs(S.FIG, exist_ok=True)

        # 基线（固定站距，与 w 无关）
        baselines = {k: eval_policy(k, args.seeds) for k in ("hover", "follow10", "axis10", "random")}
        # ours：扫 w
        ours = []
        for w in W_LIST:
            S.CFG["w_move"] = w
            r = eval_policy("ours", args.seeds)
            r["w_move"] = w
            ours.append(r)
            print(f"[{case}] w={w:<6} head={r['head']:.2f} deg  dist={r['dist']:.1f} m  "
                  f"pos={r['pos']:.3f} m  pvis={r['pvis']:.3f}", flush=True)
        S.CFG["w_move"] = base_w
        all_res[case] = dict(baselines=baselines, ours=ours)

        # 帕累托判定：对每个基线，检查是否存在 w 使 ours 在航程不更差的前提下精度更好
        pareto = []
        for b, rb in baselines.items():
            dom = [o for o in ours if o["dist"] <= rb["dist"] and o["head"] < rb["head"]]
            pareto.append({"baseline": b, "baseline_head": round(rb["head"], 3),
                           "baseline_dist": round(rb["dist"], 1),
                           "dominated_by_ours": bool(dom),
                           "best_ours_w": (min(dom, key=lambda o: o["head"])["w_move"] if dom else None)})
        all_res[case]["pareto_check"] = pareto

        # 图
        fig, ax = plt.subplots(figsize=(6.4, 4.6), dpi=150)
        ax.plot([o["dist"] for o in ours], [o["head"] for o in ours], "o-",
                color="#d1495b", lw=1.6, ms=5, label="E ours（w_move 扫描）")
        for o in ours:
            ax.annotate(f"w={o['w_move']}", (o["dist"], o["head"]), fontsize=6.5,
                        xytext=(3, 3), textcoords="offset points")
        for b, rb in baselines.items():
            ax.scatter([rb["dist"]], [rb["head"]], s=42, marker={"hover": "s", "follow10": "^",
                                                                "axis10": "D", "random": "v"}[b],
                       label={"hover": "A 悬停", "follow10": "B 定距跟随 10 m",
                              "axis10": "C 沿轴前视 10 m", "random": "D 随机"}[b],
                       color={"hover": "#4c72b0", "follow10": "#55a868",
                              "axis10": "#c44e52", "random": "#8172b2"}[b], zorder=5)
        ax.set_xlabel("无人机航程 / m")
        ax.set_ylabel("航向 RMSE / deg")
        ax.set_title(f"精度—航程权衡（LUT: {case}, {args.seeds} seeds）")
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(S.FIG, "fig_pareto.png"))
        plt.close(fig)
        print(f"[{case}] pareto -> {os.path.join(S.FIG, 'fig_pareto.png')}", flush=True)

    out_path = os.path.join(OUT, "pareto_lut.json")
    json.dump(all_res, open(out_path, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    print(f"ALL DONE -> {out_path}", flush=True)


if __name__ == "__main__":
    main()
