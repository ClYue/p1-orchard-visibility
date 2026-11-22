"""Shifted (off-centre) bounded separable Gaussian prior ablation on the edge path.

Directly tests whether allowing the across-row Gaussian centre y_c to shift removes
the failure of the centred Gaussian on the asymmetric canopy-edge visibility field.

Output: simulation/results_shifted_gaussian/summary.json
"""
import json
import sys
from pathlib import Path
import numpy as np

SIM_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM_DIR))
import run_study as rs


def fit_shifted_gaussian(lut):
    scales = np.geomspace(0.5, 150, 96)
    centers_y = np.arange(-8.0, 8.01, 0.25)
    bx = np.exp(-(rs.GRID[None, :] / scales[:, None]) ** 2)
    bx_norm = np.sum(bx ** 2, axis=1)
    lut_ss = np.sum((lut - lut.mean()) ** 2)
    lut_sq = np.sum(lut ** 2)

    best = (np.inf, 0.0, 0.0, 0.0, 0.0)
    for yc in centers_y:
        by = np.exp(-((rs.GRID[None, :] - yc) / scales[:, None]) ** 2)
        cross = bx @ lut.T @ by.T
        norm = np.outer(bx_norm, np.sum(by ** 2, axis=1))
        amp = np.clip(cross / norm, 0.0, 1.0)
        sse = lut_sq - 2 * amp * cross + amp ** 2 * norm
        idx = np.argmin(sse)
        i, j = np.unravel_index(idx, sse.shape)
        if sse[i, j] < best[0]:
            best = (float(sse[i, j]), float(amp[i, j]), float(scales[i]), float(scales[j]), float(yc))
    sse_min, amp, sx, sy, yc = best
    r2 = 1.0 - sse_min / lut_ss
    return (amp, sx, sy, yc), float(r2)


def run_shifted_ablation(offset=1.0, seeds=30):
    lut, centred_fit, centred_r2 = rs.prior(offset)
    shifted_fit, shifted_r2 = fit_shifted_gaussian(lut)
    amp_s, sx_s, sy_s, yc_s = shifted_fit
    print(f"Edge centred Gaussian : R2={centred_r2:.4f}, fit={centred_fit}")
    print(f"Edge shifted Gaussian : R2={shifted_r2:.4f}, A={amp_s:.4f}, lx={sx_s:.2f}, ly={sy_s:.2f}, yc={yc_s:.2f} m")

    weights = list(rs.WEIGHTS) + [0.00068]  # include matched-travel weight for comparison
    policies = [("lut", w) for w in weights] + [("shifted_gaussian", w) for w in weights] + [("centred_gaussian", 0.001)]
    policy_count = len(policies)
    n = policy_count * seeds
    models = np.repeat([p[0] for p in policies], seeds)
    weight = np.repeat([p[1] for p in policies], seeds)

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
    covariance = np.broadcast_to(np.diag(initial_sd**2), (n, 4, 4)).copy()
    uav = np.tile([-2.0, offset], (n, 1))
    target = uav.copy()
    metrics = np.zeros((n, 5))
    bias = 0.01 * noise[0, :, 4]
    eye = np.eye(4)

    amp_c, sx_c, sy_c = centred_fit

    for k in range(rs.STEPS):
        if k % 5 == 0:
            requested = estimate[:, None, :2] + rs.OFFSETS
            reachable = rs.move(uav[:, None, :], requested, rs.SPEED * 1)
            candidate = np.concatenate((reachable, uav[:, None, :]), axis=1)
            relative = candidate - estimate[:, None, :2]
            pv_lut = rs.interpolate(lut, relative)
            pv_cg = amp_c * np.exp(-(relative[..., 0] / sx_c)**2 - (relative[..., 1] / sy_c)**2)
            pv_sg = amp_s * np.exp(-(relative[..., 0] / sx_s)**2 - ((relative[..., 1] - yc_s) / sy_s)**2)
            pv = np.where(models[:, None] == "shifted_gaussian", pv_sg,
                          np.where(models[:, None] == "centred_gaussian", pv_cg, pv_lut))
            _, h, distance = rs.measure(candidate, estimate[:, None, :])
            pv *= rs.camera_gate(distance)
            p = covariance[:, None, :, :]
            pht = p @ np.swapaxes(h, -1, -2)
            innovation = h @ pht + np.eye(2) * rs.BEARING_SD**2
            reduction = pht @ np.linalg.solve(innovation, np.swapaxes(pht, -1, -2))
            expected = p - pv[..., None, None] * reduction
            sd = np.sqrt(np.maximum(0, np.diagonal(expected, axis1=-2, axis2=-1)))
            cost = (sd[..., 2] + rs.POS_WEIGHT * np.hypot(sd[..., 0], sd[..., 1])
                    + weight[:, None] * np.linalg.norm(candidate - uav[:, None, :], axis=-1))
            target = candidate[np.arange(n), cost.argmin(axis=1)]

        next_uav = rs.move(uav, target, rs.SPEED * rs.DT)
        metrics[:, 2] += np.linalg.norm(next_uav - uav, axis=-1)
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
        q = np.diag([(rs.DT * v[k] * 0.03)**2] * 2 + [(rs.DT * 0.01)**2, (2e-5)**2])
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
            r = np.eye(2) * rs.BEARING_SD**2
            gain = np.swapaxes(np.linalg.solve(h @ pht + r, np.swapaxes(pht, -1, -2)), -1, -2)
            estimate[detected] += (gain @ rs.wrap(observed[detected] - pred)[..., None])[..., 0]
            residual = eye - gain @ h
            covariance[detected] = residual @ p @ np.swapaxes(residual, -1, -2) + gain @ r @ np.swapaxes(gain, -1, -2)

        metrics[:, 0] += np.sum((estimate[:, :2] - true_pose[:2])**2, axis=-1)
        metrics[:, 1] += rs.wrap(estimate[:, 2] - true_pose[2])**2

    metrics[:, :2] = np.sqrt(metrics[:, :2] / rs.STEPS)
    metrics[:, 1] = np.rad2deg(metrics[:, 1])
    metrics[:, 3:] /= rs.STEPS

    rows = []
    for (m, w), vals in zip(policies, metrics.reshape(policy_count, seeds, 5)):
        mean = vals.mean(axis=0).tolist()
        rows.append(dict(model=m, weight=w, pos_rmse=mean[0], heading_rmse=mean[1],
                         flight=mean[2], pair_det=mean[3]))
        print(f"  {m:17s} w={w:7.5f} | hdg={mean[1]:.3f} deg | pos={mean[0]:.4f} m | flt={mean[2]:.1f} m | det={mean[3]:.3f}")

    out_dir = SIM_DIR / "results_shifted_gaussian"
    out_dir.mkdir(exist_ok=True)
    out = dict(
        centred_fit=dict(A=centred_fit[0], lx=centred_fit[1], ly=centred_fit[2], r2=centred_r2),
        shifted_fit=dict(A=amp_s, lx=sx_s, ly=sy_s, yc=yc_s, r2=shifted_r2),
        rows=rows,
    )
    (out_dir / "summary.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    print("Saved:", out_dir / "summary.json")


if __name__ == "__main__":
    run_shifted_ablation()
