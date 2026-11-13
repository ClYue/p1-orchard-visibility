"""地面路径横移扫描：把"决策变量"从相机看哪儿换成车走哪儿。

对每个横移量跑一次完整场景（48 个固定站 + 12 个自适应配置），取该横移下
heading 最优的固定站，得到"可达到的精度 vs 路径位置"曲线。

负号与正号对称跑，用来检查两侧是否一致（树是随机抖动的，不会严格对称）。

用法：
    python tools/path_offset.py --seeds 30
    python tools/path_offset.py --seeds 30 --offsets -1.2,-1.0,-0.8,-0.6,-0.4,-0.2,0,0.2,0.4,0.6,0.8,1.0,1.2
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

DEFAULT_OFFSETS = (-1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0,
                   0.2, 0.4, 0.6, 0.8, 1.0, 1.2)
WEIGHT = 0.001


def run(offset, seeds):
    res, _ = R.run_scene(offset, seeds, 1.0)
    return res


def summarise(offset, res):
    fixed = [r for r in res["results"] if r["model"] == "fixed"]
    best = min(fixed, key=lambda r: r["mean"][1])
    worst = max(fixed, key=lambda r: r["mean"][1])
    lut = [r for r in res["results"]
           if r["model"] == "lut" and abs(r["weight"] - WEIGHT) < 1e-9]
    lut = lut[0] if lut else None
    med = float(np.median(np.asarray(best["samples"])[:, 1]))
    return dict(
        offset=offset,
        best_angle=best["angle"], best_radius=best["radius"],
        best_heading=best["mean"][1], best_heading_median=med,
        best_position=best["mean"][0], best_flight=best["mean"][2],
        best_detection=best["mean"][3],
        worst_heading=worst["mean"][1], worst_angle=worst["angle"], worst_radius=worst["radius"],
        spread=worst["mean"][1] - best["mean"][1],
        lut_heading=lut["mean"][1] if lut else None,
        lut_flight=lut["mean"][2] if lut else None,
        lut_detection=lut["mean"][3] if lut else None,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--offsets", default=None, help="逗号分隔的横移量（m）")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.seeds < 1:
        raise SystemExit("[中止] seeds 必须为正")
    offs = (tuple(float(x) for x in args.offsets.split(",")) if args.offsets
            else DEFAULT_OFFSETS)
    out = args.out or (SIM / "results_path_offset")
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for off in offs:
        s = summarise(off, run(off, args.seeds))
        rows.append(s)
        print("offset %+5.2f m | 最优站 %3g°/%g m  head %6.3f° (中位 %6.3f°)  pos %.4f  det %.3f"
              " | 最差方位 head %6.3f° (%g°) | LUT %6.3f° @ %.1f m"
              % (s["offset"], s["best_angle"], s["best_radius"], s["best_heading"],
                 s["best_heading_median"], s["best_position"], s["best_detection"],
                 s["worst_heading"], s["worst_angle"], s["lut_heading"], s["lut_flight"]),
              flush=True)
    payload = dict(seeds=args.seeds, weight=WEIGHT, columns=list(rows[0]), rows=rows)
    (out / "summary.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print("→", out / "summary.json")


if __name__ == "__main__":
    main()
