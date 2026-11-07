"""Independent check of the shifted-Gaussian diagnostic in review_20261106.

Three steps, each with a hard check:
  1. Reproduce the centered Gaussian fit and R^2 from run_study.prior().
  2. Fit a Gaussian with a free center on the same LUT grid (bounded least squares).
  3. Re-run the edge scene with the same 30 seeds for three policies only
     (LUT w=0.001, centered Gaussian w=0.001, shifted Gaussian w=0.001) and
     require the first two to match results/study.json exactly before
     reporting the third. No match -> SystemExit, no numbers.

Run:  python verify_shifted_gaussian.py
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import run_study as rs  # noqa: E402

WEIGHT = 0.001
TOL = 5e-4          # agreement with study.json, absolute


def _r2(lut, gridd, params):
    xx, yy = np.meshgrid(gridd, gridd)
    a, sx, sy, cx, cy = params
    model = a * np.exp(-((xx - cx) / sx) ** 2 - ((yy - cy) / sy) ** 2)
    ss_res = float(np.sum((lut - model) ** 2))
    ss_tot = float(np.sum((lut - lut.mean()) ** 2))
    return 1.0 - ss_res / ss_tot


def gaussian_fit_center_free(lut, gridd, base, free, verbose=True, tag=""):
    """Bounded fit of p = A*exp(-((x-cx)/sx)^2 - ((y-cy)/sy)^2).

    `base` supplies the starting point and the values held fixed for any
    parameter not listed in `free`.
    """
    xx, yy = np.meshgrid(gridd, gridd)
    x, y, z = xx.ravel(), yy.ravel(), lut.ravel()
    names = ["A", "sx", "sy", "cx", "cy"]
    idx = [names.index(n) for n in free]
    lo_all = np.array([0.0, 0.5, 0.5, -12.0, -12.0])
    hi_all = np.array([1.0, 150.0, 150.0, 12.0, 12.0])

    def unpack(q):
        p = np.array(base, dtype=float)
        p[idx] = q
        return p

    def resid(q):
        a, sx, sy, cx, cy = unpack(q)
        return a * np.exp(-((x - cx) / sx) ** 2 - ((y - cy) / sy) ** 2) - z

    sol = least_squares(resid, np.array(base, dtype=float)[idx],
                        bounds=(lo_all[idx], hi_all[idx]),
                        xtol=1e-14, ftol=1e-14, gtol=1e-14, max_nfev=20000)
    params = unpack(sol.x)
    r2 = _r2(lut, gridd, params)
    if verbose:
        a, sx, sy, cx, cy = params
        at = "".join("^" if abs(v - b) < 1e-6 else " " for v, b in
                     zip(params, hi_all) if b > 0)
        print(f"[拟合] {tag:26s} A={a:.3f} sx={sx:6.2f} sy={sy:5.2f} "
              f"center=({cx:+.2f},{cy:+.2f})  R^2={r2:.4f}"
              + ("   <- 参数撞界" if at.strip() else ""))
    return tuple(float(v) for v in params), r2


def choose_shifted(estimate, covariance, uav, lut, fits, model, weight):
    """run_study.choose with a per-policy, optionally translatable Gaussian center."""
    requested = estimate[:, None, :2] + rs.OFFSETS
    reachable = rs.move(uav[:, None, :], requested, rs.SPEED)
    candidate = np.concatenate((reachable, uav[:, None, :]), axis=1)
    relative = candidate - estimate[:, None, :2]
    pv = rs.interpolate(lut, relative)
    amp = np.array([fits[m][0] for m in model])[:, None]
    sx = np.array([fits[m][1] for m in model])[:, None]
    sy = np.array([fits[m][2] for m in model])[:, None]
    cx = np.array([fits[m][3] for m in model])[:, None]
    cy = np.array([fits[m][4] for m in model])[:, None]
    gaussian = amp * np.exp(-((relative[..., 0] - cx) / sx) ** 2
                            - ((relative[..., 1] - cy) / sy) ** 2)
    pv = np.where(model[:, None] == "lut", pv, gaussian)
    _, h, distance = rs.measure(candidate, estimate[:, None, :])
    pv *= rs.camera_gate(distance)
    p = covariance[:, None, :, :]
    pht = p @ np.swapaxes(h, -1, -2)
    innovation = h @ pht + np.eye(2) * rs.BEARING_SD ** 2
    reduction = pht @ np.linalg.solve(innovation, np.swapaxes(pht, -1, -2))
    expected = p - pv[..., None, None] * reduction
    sd = np.sqrt(np.maximum(0, np.diagonal(expected, axis1=-2, axis2=-1)))
    cost = (sd[..., 2] + 0.2 * np.hypot(sd[..., 0], sd[..., 1])
            + weight[:, None] * np.linalg.norm(candidate - uav[:, None, :], axis=-1))
    return candidate[np.arange(len(estimate)), cost.argmin(axis=1)]


def run_edge_subset(offset, seeds, policies, lut, fits):
    """Verbatim copy of run_study.run_scene, restricted to the given policies."""
    policy_count = len(policies)
    n = policy_count * seeds
    config = np.repeat(np.array(policies, dtype=object), seeds, axis=0)

    trees = np.stack([rs.orchard(np.random.default_rng(10000 + s)) for s in range(seeds)])
    noise = np.stack([np.random.default_rng(20000 + s).standard_normal((rs.STEPS + 1, 7))
                      for s in range(seeds)], axis=1)
    uniform = np.stack([np.random.default_rng(30000 + s).random(rs.STEPS)
                        for s in range(seeds)], axis=1)
    t = np.arange(rs.STEPS + 1) * rs.DT
    x = 0.6 * t
    y = offset + 0.22 * np.sin(2 * np.pi * t / 18)
    yaw = np.r_[np.arctan2(np.diff(y), np.diff(x)), 0.]
    yaw[-1] = yaw[-2]
    v = np.hypot(np.diff(x), np.diff(y)) / rs.DT
    omega = np.diff(yaw) / rs.DT
    truth = np.column_stack((x, y, yaw))
    initial_sd = np.array([0.20, 0.20, 0.02, 0.01])
    estimate = np.tile(np.r_[truth[0], 0.] + initial_sd * noise[0, :, :4], (policy_count, 1))
    covariance = np.broadcast_to(np.diag(initial_sd ** 2), (n, 4, 4)).copy()
    uav = np.tile([-2.0, offset], (n, 1))
    target = uav.copy()
    metrics = np.zeros((n, 5))
    bias = 0.01 * noise[0, :, 4]
    eye = np.eye(4)
    weight = config[:, 3].astype(float)

    for k in range(rs.STEPS):
        if k % 5 == 0:
            target = choose_shifted(estimate, covariance, uav, lut,
                                    fits, config[:, 0], weight)
        next_uav = rs.move(uav, target, rs.SPEED * rs.DT)
        step = np.linalg.norm(next_uav - uav, axis=-1)
        assert np.max(step) <= rs.SPEED * rs.DT + 1e-10
        metrics[:, 2] += step
        uav = next_uav
        bias += 2e-5 * noise[k + 1, :, 4]
        vm = np.tile(v[k] * (1 + 0.03 * noise[k + 1, :, 0]), policy_count)
        wm = np.tile(omega[k] + bias + 0.01 * noise[k + 1, :, 1], policy_count)
        sin, cos = np.sin(estimate[:, 2]), np.cos(estimate[:, 2])
        f = np.broadcast_to(eye, (n, 4, 4)).copy()
        f[:, 0, 2], f[:, 1, 2], f[:, 2, 3] = -rs.DT * vm * sin, rs.DT * vm * cos, -rs.DT
        estimate[:, 0] += rs.DT * vm * cos
        estimate[:, 1] += rs.DT * vm * sin
        estimate[:, 2] = rs.wrap(estimate[:, 2] + rs.DT * (wm - estimate[:, 3]))
        q = np.diag([(rs.DT * v[k] * 0.03) ** 2] * 2 + [(rs.DT * 0.01) ** 2, (2e-5) ** 2])
        covariance = f @ covariance @ np.swapaxes(f, -1, -2) + q
        nominal_x = np.arange(-18.0, rs.X_EXTENT + 0.01, 2.0)
        column = np.tile(np.abs(nominal_x - x[k + 1]) < 13., 8)
        local_trees = trees[:, column, :]
        true_pose = truth[k + 1]
        beta, _, true_distance = rs.measure(uav, true_pose)
        in_fov = rs.camera_gate(true_distance)
        probability = np.zeros(n)
        valid = np.flatnonzero(in_fov)
        if valid.size:
            probability[valid] = rs.pair_probability(
                uav[valid], rs.markers(true_pose), local_trees[valid % seeds, None, :, :])
        detected = np.tile(uniform[k], policy_count) < probability
        metrics[:, 3] += detected
        metrics[:, 4] += probability
        if np.any(detected):
            pred, h, _ = rs.measure(uav[detected], estimate[detected])
            observed = beta + rs.BEARING_SD * np.tile(noise[k + 1, :, 2:4], (policy_count, 1))
            p = covariance[detected]
            pht = p @ np.swapaxes(h, -1, -2)
            r = np.eye(2) * rs.BEARING_SD ** 2
            gain = np.swapaxes(np.linalg.solve(h @ pht + r, np.swapaxes(pht, -1, -2)), -1, -2)
            estimate[detected] += (gain @ rs.wrap(observed[detected] - pred)[..., None])[..., 0]
            residual = eye - gain @ h
            covariance[detected] = (residual @ p @ np.swapaxes(residual, -1, -2)
                                    + gain @ r @ np.swapaxes(gain, -1, -2))
        metrics[:, 0] += np.sum((estimate[:, :2] - true_pose[:2]) ** 2, axis=-1)
        metrics[:, 1] += rs.wrap(estimate[:, 2] - true_pose[2]) ** 2
    metrics[:, :2] = np.sqrt(metrics[:, :2] / rs.STEPS)
    metrics[:, 1] = np.rad2deg(metrics[:, 1])
    metrics[:, 3:] /= rs.STEPS
    assert np.all(np.isfinite(metrics))
    labels = [c[0] for c in policies]
    arr = metrics.reshape(policy_count, seeds, 5)
    return {lab: arr[i] for i, lab in enumerate(labels)}


def main():
    study = json.loads((ROOT / "results/study.json").read_text(encoding="utf-8"))
    edge = study["scenes"]["edge"]

    lut, fit_c, r2_c = rs.prior(1.0)
    print(f"[第1步] 中心固定高斯  fit={[round(v,4) for v in fit_c]}  R^2={r2_c:.4f}"
          f"  （study.json 记录 R^2={edge['r2']:.4f}）")
    if abs(r2_c - edge["r2"]) > 1e-6:
        sys.exit("自检失败：R^2 未能复现 study.json，不出数")

    print("\n[消融] 把“平移中心”和“形状退化”分开")
    base = (*fit_c, 0.0, 0.0)
    f_shift_only, r2_shift_only = gaussian_fit_center_free(
        lut, rs.GRID, base, ["cx", "cy"], tag="仅平移（A,sx,sy 固定）")
    f_amp_shift, r2_amp_shift = gaussian_fit_center_free(
        lut, rs.GRID, base, ["A", "cx", "cy"], tag="平移 + 幅值（sx,sy 固定）")
    f_all, r2_all = gaussian_fit_center_free(
        lut, rs.GRID, base, ["A", "sx", "sy", "cx", "cy"], tag="平移 + 形状（全自由）")

    fit_s = f_all
    r2_s = r2_all

    fits = {"lut": (*fit_c, 0.0, 0.0), "gaussian": (*fit_c, 0.0, 0.0),
            "gaussian_shift_only": f_shift_only, "gaussian_shift": f_all}

    policies = [("lut", 0., 0., WEIGHT),
                ("gaussian", 0., 0., WEIGHT),
                ("gaussian_shift_only", 0., 0., WEIGHT),
                ("gaussian_shift", 0., 0., WEIGHT)]
    res = run_edge_subset(1.0, study["seeds"], policies, lut, fits)

    ref = {r["model"]: r["mean"] for r in edge["results"]
           if r["model"] != "fixed" and abs(r["weight"] - WEIGHT) < 1e-12}
    print("\n[第2步] 对照复现（必须与 study.json 一致，容差 %.0e）" % TOL)
    for lab, key in (("lut", "lut"), ("gaussian", "gaussian")):
        got = res[lab].mean(axis=0)
        want = ref[key]
        diff = np.abs(got - np.array(want))
        print(f"  {lab:9s} 本次 head={got[1]:.4f} pos={got[0]:.5f} fly={got[2]:.3f} "
              f"det={got[3]:.4f} | 最大差 {diff.max():.2e}")
        if diff.max() > TOL:
            sys.exit(f"自检失败：{lab} 与 study.json 不一致，复现管线有误，不出数")

    print("\n[第3步] 边缘场景 w=%.3f 三组策略（同一批 30 种子）" % WEIGHT)
    print("  策略                     航向RMSE  位置RMSE   航程    检出率")
    for lab, name in (("lut", "LUT"), ("gaussian", "中心固定高斯"),
                      ("gaussian_shift_only", "仅平移中心"),
                      ("gaussian_shift", "平移+形状（Codex 口径）")):
        m = res[lab].mean(axis=0)
        print(f"  {name:22s}  {m[1]:7.3f}  {m[0]:8.4f}  {m[2]:6.2f}  {m[3]*100:5.1f}%")

    rng = np.random.default_rng(0)
    n = len(res["lut"])
    idx = rng.integers(0, n, (5000, n))
    print("\n[配对差值] 相对 LUT（30 种子，5000 次自助，95% 百分位区间）")
    for lab, name in (("gaussian_shift_only", "仅平移中心"), ("gaussian_shift", "平移+形状")):
        d = res[lab] - res["lut"]
        print(f"  {name}:")
        for label, i, unit in (("位置 RMSE (m)", 0, "m"), ("航向 RMSE (°)", 1, "°"),
                               ("航程 (m)", 2, "m"), ("检出率", 3, "")):
            dd = d[:, i]
            bs = dd[idx].mean(axis=1)
            lo, hi = np.percentile(bs, [2.5, 97.5])
            print(f"    {label:14s} {dd.mean():+.5f}  ({lo:+.5f}, {hi:+.5f}) {unit}")


if __name__ == "__main__":
    main()
