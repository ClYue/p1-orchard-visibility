"""M2 确认运行：冻结比较对象与权重，换新种子重测等距离结论。

独立评审指出，主实验里"最佳固定站"和"等距离权重"都是在同一批 30 个种子上挑出来的
（winner's curse），却用同一批种子报配对区间；0.038° 的区间下限只有 +0.002°，选择偏差
可能吃掉整个效应。

本脚本不改选择，只换数据：固定 225°/2.0 m 站与 w=0.00068，用 seed_offset=1000 起的
100 个新种子（与主实验 0–29 不重叠）重跑，看配对差与区间是否还成立。

顺带报新种子上 48 站里误差最小的站是哪一个——如果不再落在 225°/2.0 m，说明"最优站"
本身就是这批种子的产物。

用法：
    python tools/fresh_seed_check.py --seeds 100
    python tools/fresh_seed_check.py --seeds 5 --dry-run    # 先测速
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
sys.path.insert(0, str(SIM))

import run_study as R  # noqa: E402

OFFSET = 1.0            # edge 路径
BEST = (225.0, 2.0)     # 主实验的最优固定站（方位, 半径）
MATCH_W = 0.00068       # 主实验的等距离权重
SEED_OFFSET = 1000      # 主实验用 0–29，这里从 1000 起
COLS = ["position_rmse_m", "heading_rmse_deg", "flight_m",
        "detection_fraction", "mean_pair_probability"]


def pick(results, model, angle=None, radius=None, weight=None):
    for r in results:
        if r["model"] != model:
            continue
        if angle is not None and abs(r["angle"] - angle) > 1e-9:
            continue
        if radius is not None and abs(r["radius"] - radius) > 1e-9:
            continue
        if weight is not None and abs(r["weight"] - weight) > 1e-12:
            continue
        return np.array(r["samples"], dtype=float)
    raise SystemExit("[中止] 找不到配置 %s %s %s %s" % (model, angle, radius, weight))


def paired(a, b, n=5000, seed=91):
    d = a - b
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(d), size=(n, len(d)))
    means = d[idx].mean(axis=1)
    lo, hi = np.percentile(means, [2.5, 97.5])
    return dict(mean=float(d.mean()), lo=float(lo), hi=float(hi),
                excludes_zero=bool(lo > 0 or hi < 0))


def best_fixed_on(results):
    fixed = [r for r in results if r["model"] == "fixed"]
    best = min(fixed, key=lambda r: r["mean"][1])
    return dict(angle=best["angle"], radius=best["radius"],
                heading=best["mean"][1], detection=best["mean"][3])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--out", type=Path, default=HERE.parent / "results_fresh" / "summary.json")
    ap.add_argument("--dry-run", action="store_true", help="只跑前 5 个种子测速")
    args = ap.parse_args()
    seeds = 5 if args.dry_run else args.seeds

    R.WEIGHTS = (MATCH_W,)
    print("跑 %d 个新种子（seed_offset=%d），权重 %g" % (seeds, SEED_OFFSET, MATCH_W), flush=True)
    scene, _ = R.run_scene(OFFSET, seeds, 1.0, seed_offset=SEED_OFFSET)
    res = scene["results"]

    fixed = pick(res, "fixed", angle=BEST[0], radius=BEST[1])
    lut = pick(res, "lut", weight=MATCH_W)
    if fixed.shape != lut.shape:
        raise SystemExit("[中止] 两个配置的样本数不一致")

    out = dict(seeds=seeds, seed_offset=SEED_OFFSET, scene="edge", offset=OFFSET,
               frozen_station=dict(angle=BEST[0], radius=BEST[1]), frozen_weight=MATCH_W,
               columns=COLS,
               fixed_mean=dict(zip(COLS, fixed.mean(axis=0).tolist())),
               lut_mean=dict(zip(COLS, lut.mean(axis=0).tolist())),
               fixed_samples=fixed.tolist(), lut_samples=lut.tolist(),
               diff={}, best_fixed=best_fixed_on(res))
    for j, name in enumerate(COLS):
        out["diff"][name] = paired(lut[:, j], fixed[:, j])

    print("\n%-22s %10s %10s %12s" % ("指标（LUT − 固定站）", "均值差", "2.5%", "97.5%"))
    for name in COLS:
        d = out["diff"][name]
        print("%-22s %10.4f %10.4f %12.4f %s" % (
            name, d["mean"], d["lo"], d["hi"], "不跨 0" if d["excludes_zero"] else "跨 0"))
    bf = out["best_fixed"]
    print("\n新种子上最优固定站：%.0f° / %.1f m，heading %.3f°，检出 %.3f" % (
        bf["angle"], bf["radius"], bf["heading"], bf["detection"]))
    print("冻结站：%.0f° / %.1f m，heading %.3f°，检出 %.3f" % (
        BEST[0], BEST[1], out["fixed_mean"]["heading_rmse_deg"],
        out["fixed_mean"]["detection_fraction"]))

    if args.dry_run:
        print("\n[dry-run] 未写文件")
        return
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print("\n写入", args.out)


if __name__ == "__main__":
    main()
