"""等飞行距离对照：把自适应策略的代价权重调到与实际飞行距离和最优固定站一致，再比精度。

论文原来的对比是各策略按自己的权重跑，飞行距离各不相同（LUT 在 w=0.001 飞 26.4 m，
最优固定站在 225°/2 m 飞 31.2 m）。批评者会说这是拿便宜的策略比贵的策略。
本脚本扫一串权重重跑路缘场景，解出使平均飞行距离等于固定站的权重，再用该权重单独确认一次。

用法：
    python tools/matched_distance.py --target 31.2 --seeds 30 --scene edge
    python tools/matched_distance.py --confirm 0.00083 --seeds 30 --scene edge
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = HERE if (HERE / "run_study.py").exists() else HERE.parent   # 仓库里在 tools/，投稿包里在根
sys.path.insert(0, str(SIM))
import run_study as R                      # noqa: E402

# 覆盖需要跨越 31.2 m：w=0.001 飞 26.4 m，w=0.00025 飞 60.6 m。
# 取对数等距，先粗后细。
SCAN = (0.00040, 0.00045, 0.00050, 0.00056, 0.00063, 0.00071, 0.00079, 0.00089, 0.00100)
MODELS = ("lut", "gaussian", "uniform")


def run(weights, seeds, scene):
    R.WEIGHTS = tuple(weights)
    offset = R.SCENES[scene]
    res, _ = R.run_scene(offset, seeds, 1.0)
    return res


def best_fixed(results):
    fixed = [r for r in results if r["model"] == "fixed"]
    return min(fixed, key=lambda r: r["mean"][1])


def rows(results, models):
    out = []
    for r in results:
        if r["model"] in models:
            out.append(dict(model=r["model"], weight=r["weight"],
                            heading=r["mean"][1], position=r["mean"][0],
                            flight=r["mean"][2], detection=r["mean"][3],
                            samples=r["samples"]))
    return sorted(out, key=lambda d: (d["model"], -d["weight"]))


COLS = dict(position=0, heading=1, flight=2, detection=3)


def paired_interval(a_samples, b_samples, key, n=5000, seed=91):
    """配对自举区间，沿用正文 §2.5 的口径：5000 次重抽、default_rng(91)、取 2.5/97.5 分位。"""
    a = np.asarray(a_samples)[:, COLS[key]]
    b = np.asarray(b_samples)[:, COLS[key]]
    d = a - b
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(d), (n, len(d)))
    m = d[idx].mean(axis=1)
    return float(d.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def crossing(points, target, model):
    """在 (weight, flight) 上线性插值解 flight == target。weight 与 flight 单调反向。"""
    p = sorted([q for q in points if q["model"] == model], key=lambda d: d["weight"])
    for a, b in zip(p, p[1:]):
        if (a["flight"] - target) * (b["flight"] - target) <= 0 and a["flight"] != b["flight"]:
            f = (a["flight"] - target) / (a["flight"] - b["flight"])
            return a["weight"] + f * (b["weight"] - a["weight"]), a, b
    return None, None, None


def interp(points, model, weight, key):
    p = sorted([q for q in points if q["model"] == model], key=lambda d: d["weight"])
    for a, b in zip(p, p[1:]):
        if a["weight"] <= weight <= b["weight"]:
            f = (weight - a["weight"]) / (b["weight"] - a["weight"])
            return a[key] + f * (b[key] - a[key])
    raise SystemExit("[中止] 权重 %.6g 不在扫描区间内，无法插值" % weight)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", default="edge", choices=("center", "edge"))
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--target", type=float, default=None,
                    help="目标飞行距离（m）；缺省取该场景最优固定站的平均飞行距离")
    ap.add_argument("--confirm", type=float, default=None,
                    help="只跑这一个权重做确认，不再扫描")
    ap.add_argument("--summary", action="store_true",
                    help="读 scan.json 与 confirm.json，写出 summary.json 与 scan.csv")
    ap.add_argument("--weights", default=None,
                    help="逗号分隔的扫描权重；缺省用 SCAN。中线要覆盖 33.7 m，需比路缘更高的权重")
    ap.add_argument("--out", type=Path, default=None,
                    help="输出目录；缺省 results_matched/<scene>")
    args = ap.parse_args()
    if args.seeds < 1:
        raise SystemExit("[中止] seeds 必须为正")
    if args.out is None:
        args.out = SIM / "results_matched" / args.scene
    args.out.mkdir(parents=True, exist_ok=True)

    if args.summary:
        scan = json.loads((args.out / "scan.json").read_text(encoding="utf-8"))
        conf = json.loads((args.out / "confirm.json").read_text(encoding="utf-8"))
        lut = [r for r in conf["rows"] if r["model"] == "lut"][0]
        fixed = conf["fixed"]

        def frontier(target_key, value):
            """从最便宜的一档往回走，找第一次不差于固定站的那一点。

            只在"需要多飞"时才有意义。若匹配距离上就已优于固定站，说明该指标不需要额外
            代价——返回 needs_extra_travel=False，不拿一个比匹配距离还短的飞行数当"代价"。
            """
            if lut[target_key] <= value:
                return dict(needs_extra_travel=False, note="already better at matched travel")
            p = sorted([q for q in scan["rows"] if q["model"] == "lut"], key=lambda d: d["flight"])
            for q in p:
                if q[target_key] <= value:
                    return dict(needs_extra_travel=True, weight=q["weight"], flight=q["flight"],
                                gain_pct=100 * (q["flight"] / fixed["flight"] - 1),
                                heading=q["heading"], position=q["position"],
                                detection=q["detection"])
            return dict(needs_extra_travel=None, note="not reached within the scanned weights")

        diffs = {}
        fs = conf.get("fixed_samples")
        if fs:
            for key in ("heading", "position", "flight", "detection"):
                d, lo, hi = paired_interval(lut["samples"], fs, key)
                diffs[key] = dict(mean=d, lo=lo, hi=hi)

        summary = dict(
            scene=conf["scene"], seeds=conf["seeds"],
            matched_weight=lut["weight"],
            fixed=dict(model="fixed", angle=fixed["angle"], radius=fixed["radius"],
                       heading=fixed["heading"], position=fixed["position"],
                       flight=fixed["flight"], detection=fixed["detection"]),
            adaptive=dict(model="lut", weight=lut["weight"], heading=lut["heading"],
                          position=lut["position"], flight=lut["flight"],
                          detection=lut["detection"]),
            flight_gap_pct=100 * (lut["flight"] - fixed["flight"]) / fixed["flight"],
            paired_differences_lut_minus_fixed=diffs,
            frontier=dict(position=frontier("position", fixed["position"]),
                          heading=frontier("heading", fixed["heading"])),
        )
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        with (args.out / "scan.csv").open("w", encoding="utf-8") as f:
            f.write("model,weight,heading_rmse_deg,position_rmse_m,flight_m,detection_fraction\n")
            for r in sorted(scan["rows"], key=lambda d: (d["model"], d["weight"])):
                f.write("%s,%.6g,%.4f,%.5f,%.3f,%.4f\n" % (
                    r["model"], r["weight"], r["heading"], r["position"],
                    r["flight"], r["detection"]))
        print(json.dumps(summary, indent=2))
        print("→", args.out / "summary.json", "scan.csv")
        return

    if args.confirm is not None:
        res = run((args.confirm,), args.seeds, args.scene)
        fixed = best_fixed(res["results"])
        payload = dict(kind="confirm", scene=args.scene, seeds=args.seeds,
                       weight=args.confirm,
                       fixed=dict(angle=fixed["angle"], radius=fixed["radius"],
                                  heading=fixed["mean"][1], position=fixed["mean"][0],
                                  flight=fixed["mean"][2], detection=fixed["mean"][3]),
                       fixed_samples=fixed["samples"],
                       rows=rows(res["results"], MODELS))
        (args.out / "confirm.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in payload.items() if k != "rows"}, indent=2))
        for r in payload["rows"]:
            print("  %-9s w=%-9.6g head=%.3f° pos=%.4f m fly=%.1f m det=%.3f"
                  % (r["model"], r["weight"], r["heading"], r["position"], r["flight"], r["detection"]))
        return

    wts = tuple(float(x) for x in args.weights.split(",")) if args.weights else SCAN
    res = run(wts, args.seeds, args.scene)
    fixed = best_fixed(res["results"])
    target = args.target if args.target is not None else fixed["mean"][2]
    pts = rows(res["results"], MODELS)
    payload = dict(kind="scan", scene=args.scene, seeds=args.seeds, weights=wts,
                   target=target,
                   fixed=dict(angle=fixed["angle"], radius=fixed["radius"],
                              heading=fixed["mean"][1], position=fixed["mean"][0],
                              flight=fixed["mean"][2], detection=fixed["mean"][3]),
                   rows=pts, solve={})
    print("最优固定站 %g°/%g m: heading %.3f° pos %.4f m flight %.1f m det %.3f"
          % (fixed["angle"], fixed["radius"], fixed["mean"][1], fixed["mean"][0],
             fixed["mean"][2], fixed["mean"][3]))
    print("目标飞行距离 %.2f m" % target)
    for m in MODELS:
        w, a, b = crossing(pts, target, m)
        if w is None:
            print("  %-9s 扫描区间未跨越目标，需扩大权重范围" % m)
            continue
        est = dict(weight=w, heading=interp(pts, m, w, "heading"),
                   position=interp(pts, m, w, "position"),
                   detection=interp(pts, m, w, "detection"))
        payload["solve"][m] = est
        print("  %-9s 匹配权重 %.6f → 估计 heading %.3f° pos %.4f m det %.3f（插值）"
              % (m, est["weight"], est["heading"], est["position"], est["detection"]))
    for r in pts:
        print("  %-9s w=%-9.6g head=%7.3f° pos=%8.4f fly=%6.1f det=%.3f"
              % (r["model"], r["weight"], r["heading"], r["position"], r["flight"], r["detection"]))
    (args.out / "scan.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print("→", args.out / "scan.json")


if __name__ == "__main__":
    main()
