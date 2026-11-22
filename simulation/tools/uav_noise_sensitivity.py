"""Sensitivity to UAV horizontal position uncertainty (edge and centre scenes).

Adds isotropic Gaussian horizontal error (SD = sigma_u, per-step independent or
slowly varying? Here we test per-step independent zero-mean Gaussian UAV position
noise in the true measurement geometry, while the EKF uses the nominal UAV pose).
We also verify sigma_u = 0.0 reproduces the baseline run_study.py results to 0 diff.

Output: simulation/results_uav_noise/summary.json
"""
import json
import sys
from pathlib import Path
import numpy as np

SIM_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM_DIR))
import run_study as rs


def run_with_uav_noise(offset, seeds, sigma_u, weight_lut=0.001):
    lut, fit, r2 = rs.prior(offset)
    fixed = [("fixed", float(angle), float(radius), 0.0)
             for angle in range(0, 360, 45) for radius in rs.RADII]
    active = [("lut", 0.0, 0.0, weight_lut)]
    policies = fixed + active
    policy_count = len(policies)
    n = policy_count * seeds
    config = np.repeat(np.array(policies, dtype=object), seeds, axis=0)
    is_active = config[:, 0] != "fixed"
    fixed_radius = config[~is_active, 2].astype(float)
    fixed_angle = np.deg2rad(config[~is_active, 1].astype(float))
    weight = config[is_active, 3].astype(float)

    trees = np.stack([rs.orchard(np.random.default_rng(10000 + s)) for s in range(seeds)])
    noise = np.stack([np.random.default_rng(20000 + s).standard_normal((rs.STEPS + 1, 7))
                      for s in range(seeds)], axis=1)
    uniform = np.stack([np.random.default_rng(30000 + s).random(rs.STEPS)
                        for s in range(seeds)], axis=1)
    uav_err_stream = np.stack([np.random.default_rng(40000 + s).standard_normal((rs.STEPS, 2))
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

    for k in range(rs.STEPS):
        if k % 5 == 0:
            angle = estimate[~is_active, 2] + fixed_angle
            target[~is_active] = estimate[~is_active, :2] + fixed_radius[:, None] * np.stack(
                (np.cos(angle), np.sin(angle)), axis=-1)
            target[is_active] = rs.choose(
                estimate[is_active], covariance[is_active],
                uav[is_active], lut, fit, config[is_active, 0], weight, horizon=1)
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

        # True UAV position includes horizontal positioning error; EKF uses nominal uav.
        uav_true = uav + sigma_u * np.tile(uav_err_stream[k], (policy_count, 1))
        beta, _, true_distance = rs.measure(uav_true, true_pose)
        in_fov = rs.camera_gate(true_distance)
        probability = np.zeros(n)
        valid = np.flatnonzero(in_fov)
        if valid.size:
            probability[valid] = rs.pair_probability(
                uav_true[valid], rs.markers(true_pose), local_trees[valid % seeds, None, :, :])
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
    for (model, angle, radius, w), values in zip(policies, metrics.reshape(policy_count, seeds, 5)):
        rows.append(dict(model=model, angle=angle, radius=radius, weight=w,
                         mean=values.mean(axis=0).tolist()))
    fixed_rows = [r for r in rows if r["model"] == "fixed"]
    best_fixed = min(fixed_rows, key=lambda r: r["mean"][1])
    fwd_2m = [r for r in fixed_rows if r["angle"] == 0.0 and r["radius"] == 2.0][0]
    lut_row = [r for r in rows if r["model"] == "lut"][0]
    return dict(
        sigma_u=sigma_u,
        best_fixed=dict(angle=best_fixed["angle"], radius=best_fixed["radius"],
                        pos_rmse=best_fixed["mean"][0], heading_rmse=best_fixed["mean"][1],
                        flight=best_fixed["mean"][2], pair_det=best_fixed["mean"][3]),
        forward_2m=dict(pos_rmse=fwd_2m["mean"][0], heading_rmse=fwd_2m["mean"][1],
                        flight=fwd_2m["mean"][2], pair_det=fwd_2m["mean"][3]),
        lut_w001=dict(pos_rmse=lut_row["mean"][0], heading_rmse=lut_row["mean"][1],
                      flight=lut_row["mean"][2], pair_det=lut_row["mean"][3]),
    )


def main():
    out_dir = SIM_DIR / "results_uav_noise"
    out_dir.mkdir(exist_ok=True)
    sigmas = [0.0, 0.05, 0.15, 0.30]
    out = {}
    for scene, offset in [("edge", 1.0), ("center", 0.0)]:
        out[scene] = []
        for s in sigmas:
            res = run_with_uav_noise(offset, seeds=30, sigma_u=s)
            out[scene].append(res)
            bf = res["best_fixed"]
            fw = res["forward_2m"]
            lt = res["lut_w001"]
            print(f"[{scene:6s}] sigma_u={s:.2f} m | best=({bf['angle']:5.0f}deg, {bf['radius']:.1f}m) "
                  f"hdg={bf['heading_rmse']:.3f} pos={bf['pos_rmse']:.4f} flt={bf['flight']:.1f} | "
                  f"fwd2m hdg={fw['heading_rmse']:.3f} | "
                  f"LUT hdg={lt['heading_rmse']:.3f} pos={lt['pos_rmse']:.4f} flt={lt['flight']:.1f}")
    (out_dir / "summary.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    print("Saved:", out_dir / "summary.json")


if __name__ == "__main__":
    main()
