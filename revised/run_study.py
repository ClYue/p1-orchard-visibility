"""Paired simulation of visibility-aware UAV station selection in fixed orchards."""

import argparse
import json
from pathlib import Path

import numpy as np


DT = 0.2
STEPS = 250
SPEED = 3.0
BEARING_SD = 0.008
HALF_FOV_RADIUS = 5.7 * np.tan(np.deg2rad(60))
MIN_MARKER_RADIUS = 0.35  # Mask the nadir singularity of the azimuth-only model.
RADII = (2, 3, 4.5, 6.5, 9, 12)
GRID = np.arange(-15.0, 15.01, 0.5)
ANGLES = np.arange(20) * 2 * np.pi / 20
OFFSETS = (np.array(RADII)[:, None, None] *
           np.stack((np.cos(ANGLES), np.sin(ANGLES)), axis=-1)).reshape(-1, 2)
SCENES = {"center": 0.0, "edge": 1.0}
WEIGHTS = (0, 0.00025, 0.001, 0.004)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def markers(pose):
    theta = pose[..., 2]
    half_baseline = 0.5 * np.stack((np.cos(theta), np.sin(theta)), axis=-1)
    return pose[..., None, :2] + np.array([1, -1])[:, None] * half_baseline[..., None, :]


def measure(uav, pose):
    delta = markers(pose) - uav[..., None, :]
    beta = np.arctan2(delta[..., 1], delta[..., 0])
    distance = np.linalg.norm(delta, axis=-1)
    gx = -np.sin(beta) / np.maximum(distance, 1e-6)
    gy = np.cos(beta) / np.maximum(distance, 1e-6)
    heading = 0.5 * np.array([1, -1]) * (
        -gx * np.sin(pose[..., 2, None]) + gy * np.cos(pose[..., 2, None]))
    return beta, np.stack((gx, gy, heading, np.zeros_like(beta)), axis=-1), distance


def orchard(rng):
    x, y = np.meshgrid(np.arange(-18.0, 50.01, 2.0), [-14, -10, -6, -2, 2, 6, 10, 14])
    xy = np.stack((x.ravel(), y.ravel()), axis=-1) + rng.normal(0, 0.15, (x.size, 2))
    return np.column_stack((xy, np.full(x.size, 2.0)))


def pair_probability(uav, marker_xy, trees):
    """Direct segment/ellipsoid intersections, one ray per marker.

    Independent detections conditional on geometry and ray transmittances.
    Overlapping ellipsoids contribute additive optical depth.
    """
    radius = np.array([1.2, 1.2, 1.5])
    start = np.concatenate((marker_xy, np.full((*marker_xy.shape[:-1], 1), 0.3)), axis=-1)
    end = np.concatenate((uav, np.full((*uav.shape[:-1], 1), 6.0)), axis=-1)
    ray = end[..., None, :] - start
    unit_ray = ray[..., None, :] / radius
    origin = (start[..., None, :] - trees) / radius
    a = np.sum(unit_ray * unit_ray, axis=-1)
    b = np.sum(origin * unit_ray, axis=-1)
    c = np.sum(origin * origin, axis=-1) - 1
    disc = b * b - a * c
    root = np.sqrt(np.maximum(0, disc))
    lo = np.clip((-b - root) / a, 0, 1)
    hi = np.clip((-b + root) / a, 0, 1)
    length = np.linalg.norm(ray, axis=-1) * np.sum(hi - lo, axis=-1)
    return np.exp(-np.sum(length, axis=-1))


def prior(offset):
    xx, yy = np.meshgrid(GRID, GRID)
    uav = np.stack((xx.ravel(), yy.ravel() + offset), axis=-1)
    maps = []
    for seed in range(20):
        rng = np.random.default_rng(seed)
        trees = orchard(rng)
        phase = rng.uniform(0, 2)
        trees = trees[(np.abs(trees[:, 0] - phase) < 18) & (np.abs(trees[:, 1] - offset) < 18)]
        maps.append(pair_probability(uav + [phase, 0], markers(np.array([phase, offset, 0.])), trees))
    lut = np.mean(maps, axis=0).reshape(xx.shape)
    scales = np.geomspace(0.5, 150, 96)
    basis = np.exp(-(GRID[None, :] / scales[:, None]) ** 2)
    cross = basis @ lut.T @ basis.T
    norm = np.outer(np.sum(basis ** 2, axis=1), np.sum(basis ** 2, axis=1))
    amplitude = np.clip(cross / norm, 0, 1)
    sse = np.sum(lut ** 2) - 2 * amplitude * cross + amplitude ** 2 * norm
    i, j = np.unravel_index(sse.argmin(), sse.shape)
    fit = (float(amplitude[i, j]), float(scales[i]), float(scales[j]))
    r2 = 1 - sse[i, j] / np.sum((lut - lut.mean()) ** 2)
    return lut, fit, float(r2)


def interpolate(lut, relative):
    x, y = np.moveaxis(np.clip((relative - GRID[0]) / 0.5, 0, len(GRID) - 1), -1, 0)
    i = np.minimum(x.astype(int), len(GRID) - 2)
    j = np.minimum(y.astype(int), len(GRID) - 2)
    tx, ty = x - i, y - j
    return ((1 - tx) * (1 - ty) * lut[j, i] + tx * (1 - ty) * lut[j, i + 1]
            + (1 - tx) * ty * lut[j + 1, i] + tx * ty * lut[j + 1, i + 1])


def move(current, target, limit):
    delta = target - current
    scale = np.minimum(1., limit / np.maximum(1e-12, np.linalg.norm(delta, axis=-1)))
    return current + scale[..., None] * delta


def camera_gate(distance):
    return np.all((distance >= MIN_MARKER_RADIUS) & (distance <= HALF_FOV_RADIUS), axis=-1)


def choose(estimate, covariance, uav, lut, fit, model, weight):
    # Evaluate positions reachable within one second and allow the UAV to stay.
    requested = estimate[:, None, :2] + OFFSETS
    reachable = move(uav[:, None, :], requested, SPEED)
    candidate = np.concatenate((reachable, uav[:, None, :]), axis=1)
    relative = candidate - estimate[:, None, :2]
    pv = interpolate(lut, relative)
    amp, sx, sy = fit
    gaussian = amp * np.exp(-(relative[..., 0] / sx)**2 - (relative[..., 1] / sy)**2)
    pv = np.where(model[:, None] == "gaussian", gaussian, pv)
    pv = np.where(model[:, None] == "uniform", lut.mean(), pv)
    _, h, distance = measure(candidate, estimate[:, None, :])
    pv *= camera_gate(distance)
    # Expected posterior covariance for either a pair of observations or no update.
    p = covariance[:, None, :, :]
    pht = p @ np.swapaxes(h, -1, -2)
    innovation = h @ pht + np.eye(2) * BEARING_SD**2
    reduction = pht @ np.linalg.solve(innovation, np.swapaxes(pht, -1, -2))
    expected = p - pv[..., None, None] * reduction
    sd = np.sqrt(np.maximum(0, np.diagonal(expected, axis1=-2, axis2=-1)))
    cost = (sd[..., 2] + 0.2 * np.hypot(sd[..., 0], sd[..., 1])
            + weight[:, None] * np.linalg.norm(candidate - uav[:, None, :], axis=-1))
    return candidate[np.arange(len(estimate)), cost.argmin(axis=1)]


def run_scene(offset, seeds):
    lut, fit, r2 = prior(offset)
    fixed = [("fixed", float(angle), float(radius), 0.0)
             for angle in range(0, 360, 45) for radius in RADII]
    active = [(model, 0., 0., weight)
              for model in ("lut", "gaussian", "uniform") for weight in WEIGHTS]
    policies = fixed + active
    policy_count = len(policies)
    n = policy_count * seeds
    config = np.repeat(np.array(policies, dtype=object), seeds, axis=0)
    is_active = config[:, 0] != "fixed"
    fixed_radius = config[~is_active, 2].astype(float)
    fixed_angle = np.deg2rad(config[~is_active, 1].astype(float))
    weight = config[is_active, 3].astype(float)
    trees = np.stack([orchard(np.random.default_rng(10000 + s)) for s in range(seeds)])
    noise = np.stack([np.random.default_rng(20000 + s).standard_normal((STEPS + 1, 7))
                      for s in range(seeds)], axis=1)
    uniform = np.stack([np.random.default_rng(30000 + s).random(STEPS)
                        for s in range(seeds)], axis=1)
    t = np.arange(STEPS + 1) * DT
    x = 0.6 * t
    y = offset + 0.22 * np.sin(2 * np.pi * t / 18)
    yaw = np.r_[np.arctan2(np.diff(y), np.diff(x)), 0.]
    yaw[-1] = yaw[-2]
    v = np.hypot(np.diff(x), np.diff(y)) / DT
    omega = np.diff(yaw) / DT
    truth = np.column_stack((x, y, yaw))
    initial_sd = np.array([0.20, 0.20, 0.02, 0.01])
    estimate = np.tile(np.r_[truth[0], 0.] + initial_sd * noise[0, :, :4], (policy_count, 1))
    covariance = np.broadcast_to(np.diag(initial_sd**2), (n, 4, 4)).copy()
    uav = np.tile([-2.0, offset], (n, 1))
    target = uav.copy()
    metrics = np.zeros((n, 5))
    bias = 0.01 * noise[0, :, 4]
    eye = np.eye(4)
    for k in range(STEPS):
        if k % 5 == 0:
            angle = estimate[~is_active, 2] + fixed_angle
            target[~is_active] = estimate[~is_active, :2] + fixed_radius[:, None] * np.stack(
                (np.cos(angle), np.sin(angle)), axis=-1)
            target[is_active] = choose(estimate[is_active], covariance[is_active],
                                       uav[is_active], lut, fit, config[is_active, 0], weight)
        next_uav = move(uav, target, SPEED * DT)
        step = np.linalg.norm(next_uav - uav, axis=-1)
        assert np.max(step) <= SPEED * DT + 1e-10
        metrics[:, 2] += step
        uav = next_uav
        bias += 2e-5 * noise[k + 1, :, 4]
        vm = np.tile(v[k] * (1 + 0.03 * noise[k + 1, :, 0]), policy_count)
        wm = np.tile(omega[k] + bias + 0.01 * noise[k + 1, :, 1], policy_count)
        sin, cos = np.sin(estimate[:, 2]), np.cos(estimate[:, 2])
        f = np.broadcast_to(eye, (n, 4, 4)).copy()
        f[:, 0, 2], f[:, 1, 2], f[:, 2, 3] = -DT * vm * sin, DT * vm * cos, -DT
        estimate[:, 0] += DT * vm * cos
        estimate[:, 1] += DT * vm * sin
        estimate[:, 2] = wrap(estimate[:, 2] + DT * (wm - estimate[:, 3]))
        q = np.diag([(DT * v[k] * 0.03)**2] * 2 + [(DT * 0.01)**2, (2e-5)**2])
        covariance = f @ covariance @ np.swapaxes(f, -1, -2) + q
        # Trees remain fixed in world coordinates. Prune only impossible intersections.
        nominal_x = np.arange(-18.0, 50.01, 2.0)
        column = np.tile(np.abs(nominal_x - x[k + 1]) < 13., 8)
        local_trees = trees[:, column, :]
        true_pose = truth[k + 1]
        beta, _, true_distance = measure(uav, true_pose)
        in_fov = camera_gate(true_distance)
        probability = np.zeros(n)
        valid = np.flatnonzero(in_fov)
        if valid.size:
            probability[valid] = pair_probability(
                uav[valid], markers(true_pose), local_trees[valid % seeds, None, :, :])
        assert np.all((0 <= probability) & (probability <= 1))
        # A single shared uniform represents the conditional joint Bernoulli event.
        detected = np.tile(uniform[k], policy_count) < probability
        metrics[:, 3] += detected
        metrics[:, 4] += probability
        if np.any(detected):
            pred, h, _ = measure(uav[detected], estimate[detected])
            observed = beta + BEARING_SD * np.tile(noise[k + 1, :, 2:4], (policy_count, 1))
            p = covariance[detected]
            pht = p @ np.swapaxes(h, -1, -2)
            r = np.eye(2) * BEARING_SD**2
            gain = np.swapaxes(np.linalg.solve(h @ pht + r, np.swapaxes(pht, -1, -2)), -1, -2)
            estimate[detected] += (gain @ wrap(observed[detected] - pred)[..., None])[..., 0]
            residual = eye - gain @ h
            covariance[detected] = residual @ p @ np.swapaxes(residual, -1, -2) + gain @ r @ np.swapaxes(gain, -1, -2)
        metrics[:, 0] += np.sum((estimate[:, :2] - true_pose[:2])**2, axis=-1)
        metrics[:, 1] += wrap(estimate[:, 2] - true_pose[2])**2
    metrics[:, :2] = np.sqrt(metrics[:, :2] / STEPS)
    metrics[:, 1] = np.rad2deg(metrics[:, 1])
    metrics[:, 3:] /= STEPS
    assert np.all(np.isfinite(metrics))
    results = []
    for (model, angle, radius, w), values in zip(policies, metrics.reshape(policy_count, seeds, 5)):
        results.append(dict(model=model, angle=angle, radius=radius, weight=w,
                            mean=values.mean(axis=0).tolist(), samples=values.tolist()))
    return dict(fit=fit, r2=r2, results=results), lut


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results")
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    study = dict(seeds=args.seeds, dt=DT, steps=STEPS,
                 columns=["position_rmse_m", "heading_rmse_deg", "flight_m",
                          "detection_fraction", "mean_pair_probability"], scenes={})
    maps = {}
    for name, offset in SCENES.items():
        study["scenes"][name], maps[name] = run_scene(offset, args.seeds)
        print(f"{name}: completed {len(study['scenes'][name]['results'])} policies", flush=True)
    (args.output / "study.json").write_text(json.dumps(study, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(args.output / "priors.npz", grid=GRID, **maps)


if __name__ == "__main__":
    main()
