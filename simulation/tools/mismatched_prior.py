"""错配先验：规划器按错误走廊建模，评估按真实几何。

论文假设走廊已知（先验 offset 与路径 offset 一致）。这里把两者拆开：
地面路径走真实 offset，可见性表按另一个 offset 建，模拟"设计者把路径位置搞错了 1 m"。

对照三方：
  1. belief_chosen —— 按错误信念会选的那个固定站，在真实路径上的表现（设计者的处境）
  2. oracle_best   —— 按真实几何的最优固定站（设计者够不到的上界）
  3. adaptive      —— 同一错误信念下的 LUT / Gaussian / Uniform 策略

用法：
    python tools/mismatched_prior.py --path edge --belief center
    python tools/mismatched_prior.py --path center --belief edge
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = HERE if (HERE / "run_study.py").exists() else HERE.parent
sys.path.insert(0, str(SIM))
import run_study as R                      # noqa: E402

SCENES = {"center": 0.0, "edge": 1.0}
MODELS = ("lut", "gaussian", "uniform")
COLS = dict(position=0, heading=1, flight=2, detection=3)


def run(path_scene, belief_scene, seeds):
    res, _ = R.run_scene(SCENES[path_scene], seeds, 1.0, prior_offset=SCENES[belief_scene])
    return res


def best_fixed(results):
    f = [r for r in results if r["model"] == "fixed"]
    b = min(f, key=lambda r: r["mean"][1])
    return b


def station(results, angle, radius):
    for r in results:
        if r["model"] == "fixed" and abs(r["angle"] - angle) < 1e-6 and abs(r["radius"] - radius) < 1e-6:
            return r
    raise SystemExit("[中止] 找不到固定站 %g°/%g m" % (angle, radius))


def paired_interval(a, b, key, n=5000, seed=91):
    x = np.asarray(a["samples"])[:, COLS[key]]
    y = np.asarray(b["samples"])[:, COLS[key]]
    d = x - y
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(d), (n, len(d)))
    m = d[idx].mean(axis=1)
    return float(d.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def robust(r, key="heading"):
    """均值被尾部主导时，用分位数描述：中位数、10/90 分位、最大值、超阈值次数。"""
    x = np.asarray(r["samples"])[:, COLS[key]]
    return dict(mean=float(x.mean()), median=float(np.median(x)),
                p10=float(np.percentile(x, 10)), p90=float(np.percentile(x, 90)),
                max=float(x.max()), n=int(len(x)),
                over_5=int((x > 5).sum()), over_10=int((x > 10).sum()))


def row(r):
    return dict(model=r["model"], weight=r["weight"], angle=r["angle"], radius=r["radius"],
                heading=r["mean"][1], position=r["mean"][0], flight=r["mean"][2],
                detection=r["mean"][3])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--path", default="edge", choices=tuple(SCENES))
    ap.add_argument("--belief", default="center", choices=tuple(SCENES))
    ap.add_argument("--belief-station", default=None,
                    help="错误信念下会选的站，格式 角度,半径；缺省自动取信念场景的最优站")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.seeds < 1:
        raise SystemExit("[中止] seeds 必须为正")
    if args.path == args.belief:
        raise SystemExit("[中止] 路径与信念相同，那是已知走廊，不是错配")

    out = args.out or (SIM / "results_mismatch" / ("%s_path_%s_belief" % (args.path, args.belief)))
    out.mkdir(parents=True, exist_ok=True)

    if args.belief_station:
        a, r = (float(x) for x in args.belief_station.split(","))
    else:
        # 信念场景下的最优站——就是设计者按错误信念会选的那一个
        belief = json.loads((SIM / "results" / "study.json").read_text(encoding="utf-8"))
        b = min((x for x in belief["scenes"][args.belief]["results"] if x["model"] == "fixed"),
                key=lambda x: x["mean"][1])
        a, r = b["angle"], b["radius"]

    res = run(args.path, args.belief, args.seeds)
    chosen = station(res["results"], a, r)
    oracle = best_fixed(res["results"])
    actives = [x for x in res["results"] if x["model"] != "fixed"]

    comparisons = []
    for x in actives:
        d = {}
        for key in ("heading", "position", "flight", "detection"):
            m, lo, hi = paired_interval(x, chosen, key)
            d[key] = dict(mean=m, lo=lo, hi=hi)
        comparisons.append(dict(**row(x), vs_belief_chosen=d, robust_heading=robust(x)))

    payload = dict(path=args.path, belief=args.belief, seeds=args.seeds,
                   path_offset=SCENES[args.path], prior_offset=SCENES[args.belief],
                   belief_chosen=row(chosen), oracle_best=row(oracle),
                   belief_chosen_robust=robust(chosen),
                   oracle_best_robust=robust(oracle),
                   comparisons=comparisons)
    (out / "summary.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print("路径 %s（offset %.1f），信念 %s（先验 offset %.1f），%d seeds"
          % (args.path, SCENES[args.path], args.belief, SCENES[args.belief], args.seeds))
    print("  信念选的站 %g°/%g m : head %.3f° pos %.4f m fly %.1f m det %.3f"
          % (chosen["angle"], chosen["radius"], chosen["mean"][1], chosen["mean"][0],
             chosen["mean"][2], chosen["mean"][3]))
    print("  真实最优站 %g°/%g m : head %.3f° pos %.4f m fly %.1f m det %.3f"
          % (oracle["angle"], oracle["radius"], oracle["mean"][1], oracle["mean"][0],
             oracle["mean"][2], oracle["mean"][3]))
    for c in comparisons:
        r = c
        d = c["vs_belief_chosen"]["heading"]
        print("  %-9s w=%-8.6g head %6.3f° fly %5.1f m det %.3f | Δheading %+7.3f [%+7.3f,%+7.3f]"
              % (r["model"], r["weight"], r["heading"], r["flight"], r["detection"],
                 d["mean"], d["lo"], d["hi"]))
    print("→", out / "summary.json")


if __name__ == "__main__":
    main()
