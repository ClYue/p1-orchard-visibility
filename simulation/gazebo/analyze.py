"""对比 Gazebo 渲染的标记可见性与 pair_probability 的预测。

判据说明：
  pair_probability = exp(-光线穿过冠层椭球的总长度)，被 run_study.py 直接当作
  "成对检测成功概率"。这里用 Gazebo 渲染的标记可见像素数作为独立观测：
  同一半径下，穿过树冠的方位（90/270 度）可见像素应显著少于沿过道方位（0/180 度）。

用法：python3 analyze.py --shots /tmp/shots48 --csv out.csv
"""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

# 与 run_study.py 对齐
MARKER_R = 0.09
IMG_W, IMG_H = 640, 480
CAM_HALF_FOV = math.radians(60.0)
CAM_Z_MODEL_END = 6.0      # pair_probability 里光线终点的 z，相机高度与之对齐
MARKER_Z = 0.30            # 标记球心高度，与 make_world.MARKER_Z 一致


def focal_px():
    return (IMG_W / 2.0) / math.tan(CAM_HALF_FOV)


def marker_mask(bgr):
    """品红标记：R 与 B 都高、G 低。BGR 顺序。"""
    b, g, r = bgr[:, :, 0].astype(int), bgr[:, :, 1].astype(int), bgr[:, :, 2].astype(int)
    m = ((r > 110) & (b > 110) & (g < 110) & (np.abs(r - b) < 90)).astype(np.uint8)
    return m


def two_blobs(mask):
    """返回两个最大连通域的像素数（不足两个则补 0）。"""
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    areas = sorted((int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, n)), reverse=True)
    while len(areas) < 2:
        areas.append(0)
    return areas[0], areas[1]


def expected_area(dist):
    """球在图像上的理论投影面积（像素^2），用于归一化。"""
    rad_px = focal_px() * MARKER_R / max(dist, 1e-6)
    return math.pi * rad_px ** 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", default="/tmp/shots48")
    ap.add_argument("--csv", default="visibility_vs_model.csv")
    ap.add_argument("--outdir", default=".")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    sites = json.loads((here / "sites.json").read_text(encoding="utf-8"))
    shots = Path(args.shots)

    import sys
    sys.path.insert(0, str(here.parent))
    import run_study as R          # noqa: E402

    trees = R.orchard(np.random.default_rng(10000 + 0))
    markers = np.array([[-0.5, 0.0], [0.5, 0.0]])

    rows = []
    for s in sites:
        tag = "r%05.1f_a%03d" % (s["radius"], s["angle_deg"])
        fp = shots / (tag + ".png")
        if not fp.exists():
            continue
        bgr = cv2.imread(str(fp))
        a0, a1 = two_blobs(marker_mask(bgr))
        total = a0 + a1

        uav = np.array([s["x"], s["y"]])
        p = float(R.pair_probability(uav, markers, trees))
        d3d = float(np.hypot(np.hypot(s["x"], s["y"]), CAM_Z_MODEL_END - 0.3))
        exp_area = 2.0 * expected_area(d3d)
        visible = min(1.0, total / exp_area) if exp_area > 0 else 0.0

        rows.append({
            "radius": s["radius"], "angle": s["angle_deg"], "tag": tag,
            "d3d": round(d3d, 3), "blob0": a0, "blob1": a1, "px_total": total,
            "expected_px": round(exp_area, 2), "visible_frac": round(visible, 4),
            "p_model": round(p, 6),
            "both_seen": int(a0 >= 3 and a1 >= 3),
        })

    rows.sort(key=lambda r: (r["radius"], r["angle"]))
    out = Path(args.outdir) / args.csv
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("站位 %d 个 -> %s\n" % (len(rows), out))
    print("%6s %5s %7s %7s %7s %8s %8s" %
          ("半径", "方位", "两标记px", "理论px", "可见比", "模型p", "双见"))
    for r in rows:
        print("%6.1f %5d %7d %7.1f %7.3f %8.4f %8d" %
              (r["radius"], r["angle"], r["px_total"], r["expected_px"],
               r["visible_frac"], r["p_model"], r["both_seen"]))

    # 同一半径内，按方位看遮挡效应
    print("\n=== 同半径下 8 个方位的可见像素（关键对照）===")
    for rad in sorted({r["radius"] for r in rows}):
        grp = [r for r in rows if r["radius"] == rad]
        px = [g["px_total"] for g in grp]
        pm = [g["p_model"] for g in grp]
        print("半径 %4.1f m: 像素 %s" % (rad, px))
        print("            模型p %s" % ["%.3f" % x for x in pm])

    # 相关性（限模型认定的视场内站位）
    #
    # 注意：门限 HALF_FOV_RADIUS 是相机足迹的**水平**半径（5.7 * tan60 = 9.873 m），
    # 而 d3d 是三维斜距。此前直接拿 d3d 与门限比，把水平 9.0 m 的 8 个站位误判为出界，
    # 样本从 40 掉到 32。这里先换回水平距离再筛。
    drop = CAM_Z_MODEL_END - MARKER_Z
    for r in rows:
        r["horiz"] = math.sqrt(max(r["d3d"] ** 2 - drop ** 2, 0.0))
    valid = [r for r in rows if r["horiz"] <= R.HALF_FOV_RADIUS]
    if len(valid) >= 5:
        x = np.array([r["p_model"] for r in valid])
        y = np.array([r["visible_frac"] for r in valid])
        z = np.array([r["px_total"] for r in valid])
        def pearson(a, b):
            return float(np.corrcoef(a, b)[0, 1]) if a.std() > 1e-9 and b.std() > 1e-9 else float("nan")
        def spearman(a, b):
            # 平均秩。visible_frac 有大量并列 1.0，用 argsort(argsort()) 会按原始下标
            # 给并列值排不同名次，算出的是序数相关而非 Spearman ρ（32 站会低到 0.936）。
            def rank(v):
                o = np.argsort(v, kind="mergesort")
                r = np.empty(len(v), dtype=float)
                i = 0
                while i < len(o):
                    j = i
                    while j + 1 < len(o) and v[o[j + 1]] == v[o[i]]:
                        j += 1
                    r[o[i:j + 1]] = (i + j) / 2.0 + 1.0
                    i = j + 1
                return r
            return pearson(rank(a), rank(b))
        print("\n视场内站位 %d 个（水平距 <= %.3f m）" % (len(valid), R.HALF_FOV_RADIUS))
        print("  Pearson(p, 可见比) = %.3f" % pearson(x, y))
        print("  Spearman(p, 可见比) = %.3f" % spearman(x, y))
        print("  Pearson(p, 像素数) = %.3f" % pearson(x, z))

    both = [r for r in valid if r["both_seen"]]
    print("  双标记可见 %d / %d" % (len(both), len(valid)))
    hi = [r for r in valid if r["p_model"] >= 0.5]
    lo = [r for r in valid if r["p_model"] < 0.5]
    print("  模型 p>=0.5 的站位 %d，其中双见 %d"
          % (len(hi), sum(1 for r in hi if r["both_seen"])))
    print("  模型 p<0.5 的站位 %d，其中双见 %d"
          % (len(lo), sum(1 for r in lo if r["both_seen"])))


if __name__ == "__main__":
    main()
