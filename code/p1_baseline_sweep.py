# -*- coding: utf-8 -*-
"""
P1 基线站距扫描（应答"自由度不对等"）
=====================================
审稿人可能质疑：本文方法有两个可调权重（κ、w），而 B/C 基线固定 10 m，
比较不对等。本脚本让两条基线也扫站距 r∈{5,8,10,12,15,20} m，
取各自"最好"的那个点，再与本文比较——即给基线同样的最优努力。

用法：python p1_baseline_sweep.py [--seeds 15]
输出：baseline_sweep.json
"""
import argparse
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p1_sim as S  # noqa: E402

# 站距扫描集合必须覆盖规划器的候选范围（p1_sim.RAD 最小 2.0 m），
# 否则固定基线被限制在更大的站距上，对比不公平。
# 2026-11-05 修正：原为 [5,8,10,12,15,20]，最小 5 m。
R_LIST = [2.0, 3.0, 4.5, 5.0, 6.5, 8.0, 9.0, 10.0, 12.0, 15.0, 20.0]


def eval_pol(key, seeds):
    rs = [S.run_metrics(key, s) for s in range(seeds)]
    return dict(head=float(np.mean([r["head_rmse"] for r in rs])),
                pos=float(np.mean([r["pos_rmse"] for r in rs])),
                dist=float(np.mean([r["dist"] for r in rs])),
                pvis=float(np.mean([r["mean_pvis"] for r in rs])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=15)
    args = ap.parse_args()

    # 必须与论文推荐配置一致（κ=0.2, w=0.004），否则 "ours" 会在
    # w_pos=0 的默认值下运行，结果与论文表 4 不可比（实测 0.78° vs 0.34°）。
    S.CFG["w_pos"] = 0.2
    S.CFG["w_move"] = 0.004

    lut = os.path.normpath(os.path.join(S.OUT, "..", "..", "08_Gazebo仿真", "vis_lut.npz"))
    out = {}
    for case in ("alley_center", "canopy_edge"):
        S._lut_load(lut, case)
        S.VIS["mode"] = "lut"
        S.VIS["case"] = case
        S.PVIS_TAB = S.build_pvis_tab()
        rows = []
        for key in ("follow10", "axis10"):
            for r in R_LIST:
                S.CFG["baseline_r"] = r
                m = eval_pol(key, args.seeds)
                m.update(policy=key, r=r)
                rows.append(m)
                print(f"[{case}] {key:9s} r={r:<5} head={m['head']:.3f} pos={m['pos']:.4f} "
                      f"dist={m['dist']:.0f} pvis={m['pvis']:.3f}", flush=True)
        S.CFG["baseline_r"] = 10.0
        # 各基线的最优（按航向，依次航程不劣于本文 99 m×1.5）
        best_head = {}
        for key in ("follow10", "axis10"):
            sub = [m for m in rows if m["policy"] == key]
            best = min(sub, key=lambda m: m["head"])
            best_head[key] = {"head": round(best["head"], 3), "pos": round(best["pos"], 4),
                              "dist": round(best["dist"], 1), "r": best["r"]}
        out[case] = {"sweep": [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()}
                               for m in rows],
                     "best_per_baseline": best_head}
        print(f"[{case}] best: {best_head}", flush=True)

    json.dump(out, open(os.path.join(S.OUT, "baseline_sweep.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("ALL DONE -> baseline_sweep.json")


if __name__ == "__main__":
    main()
