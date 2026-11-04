# -*- coding: utf-8 -*-
"""
P1 预研仿真：可辨识度驱动的无人机观测机动
对应论文：GNSS拒止果园环境下无人机辅助地面机器人协同定位

注意：本脚本只做方法可行性与趋势预演，输出为仿真值，不是实测数据。
      所有参数集中在 CFG，便于评审/复现时逐项替换后重跑。
"""
import json, os, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 160
plt.rcParams["savefig.bbox"] = "tight"

OUT = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(OUT, "figures")
os.makedirs(FIG, exist_ok=True)

CFG = dict(
    dt=0.1, T=150.0,
    v0=0.60, v_noise=0.030,
    gyro_noise=0.010,
    bias_true=0.010, bias_rw=2e-5,
    L=1.00,
    sigma_beta=0.008,
    vis_lx=4.5, vis_ly=14.0, vis_min=0.03,
    uav_vmax=3.0, uav_rmax=20.0,
    p0=0.20, s_theta0=0.02, s_b0=0.010,
    w_move=0.004,
    w_pos=0.0,          # 位置项权重（双目标扩展；0 = 仅航向，与旧结果一致）
    baseline_r=10.0,    # B/C 基线的固定观测站距（m）
    q2_mult=10.0,
)
ANG = np.linspace(-math.pi, math.pi, 20, endpoint=False)
RAD = np.array([2.0, 3.0, 4.5, 6.5, 9.0, 12.0])
AG, RG = np.meshgrid(ANG, RAD)
AG, RG = AG.ravel(), RG.ravel()
REL = np.stack([RG * np.cos(AG), RG * np.sin(AG)], 1)          # (M,2)
PVIS_TAB = None  # 由 build_pvis_tab() 按当前可见性模式生成（gauss / lut）


# ---------------- 可见性模型：乘积高斯（旧） / 查找表（M0 裁决 1） ----------------
VIS = {"mode": "gauss", "lut": None, "case": "alley_center"}


def _lut_load(path, case):
    d = np.load(path, allow_pickle=False)
    gx = np.asarray(d[f"GX_{case}"])[0, :].copy()
    gy = np.asarray(d[f"GY_{case}"])[:, 0].copy()
    p = np.asarray(d[f"pvis_{case}"]).copy()
    VIS["lut"] = (gx, gy, p)
    VIS["case"] = case
    print(f"[vis] LUT loaded: {case}, grid={p.shape}, 范围 "
          f"[{gx[0]:.1f},{gx[-1]:.1f}] m, 均值 {p.mean():.3f}", flush=True)


def _lut_pvis(dx, dy):
    """双线性插值，边界外取边界值。dx,dy 为无人机相对地面平台位移。"""
    gx, gy, P = VIS["lut"]
    dx = np.clip(np.asarray(dx, float), gx[0], gx[-1])
    dy = np.clip(np.asarray(dy, float), gy[0], gy[-1])
    i = np.clip(np.searchsorted(gx, dx) - 1, 0, len(gx) - 2)
    j = np.clip(np.searchsorted(gy, dy) - 1, 0, len(gy) - 2)
    tx = (dx - gx[i]) / (gx[i + 1] - gx[i])
    ty = (dy - gy[j]) / (gy[j + 1] - gy[j])
    v = ((1 - tx) * (1 - ty) * P[j, i]
         + tx * (1 - ty) * P[j, i + 1]
         + (1 - tx) * ty * P[j + 1, i]
         + tx * ty * P[j + 1, i + 1])
    return np.maximum(v, 0.0)


def pvis_field(dx, dy):
    if VIS["mode"] == "lut":
        return _lut_pvis(dx, dy)
    return np.maximum(CFG["vis_min"],
                      np.exp(-(np.asarray(dx, float) / CFG["vis_lx"]) ** 2
                             - (np.asarray(dy, float) / CFG["vis_ly"]) ** 2))


def build_pvis_tab():
    return np.asarray(pvis_field(REL[:, 0], REL[:, 1]), float)


PVIS_TAB = build_pvis_tab()


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def markers(x, y, th, L):
    e = np.array([math.cos(th), math.sin(th)])
    p = np.array([x, y])
    return p + 0.5 * L * e, p - 0.5 * L * e


def jac_and_bearings(u, x, y, th, L):
    m1, m2 = markers(x, y, th, L)
    H = np.zeros((2, 4))
    z = np.zeros(2)
    dmx = np.array([-0.5 * L * math.sin(th), -0.5 * L * math.sin(th)])
    dmy = np.array([0.5 * L * math.cos(th), 0.5 * L * math.cos(th)])
    for i, (m, s) in enumerate(((m1, 1.0), (m2, -1.0))):
        d = m - u
        r = math.sqrt(max(d @ d, 1e-9))
        b = math.atan2(d[1], d[0])
        z[i] = b
        hx, hy = -math.sin(b) / r, math.cos(b) / r
        H[i, 0] = hx
        H[i, 1] = hy
        H[i, 2] = hx * s * dmx[i] + hy * s * dmy[i]
    return H, z


def noisy_bearings(u, x, y, th, rng):
    m1, m2 = markers(x, y, th, CFG["L"])
    z = np.array([math.atan2((m1 - u)[1], (m1 - u)[0]), math.atan2((m2 - u)[1], (m2 - u)[0])])
    return z + CFG["sigma_beta"] * rng.standard_normal(2)


def ftheta(u, x, y, th, L, sb):
    H, _ = jac_and_bearings(u, x, y, th, L)
    return float(H[:, 2] @ H[:, 2]) / sb ** 2


def pvis_of(u, x, y):
    return float(np.asarray(pvis_field(u[0] - x, u[1] - y)))


def policy_ours(x, y, th, P, u_cur):
    """一步前瞻：最小化 预测航向标准差 + 机动代价"""
    U = REL + np.array([x, y])
    m1, m2 = markers(x, y, th, CFG["L"])
    dmx = -0.5 * CFG["L"] * math.sin(th)
    dmy = 0.5 * CFG["L"] * math.cos(th)
    H0 = np.linalg.inv(P)
    acc = np.zeros((U.shape[0], 4, 4))
    for m, s in ((m1, 1.0), (m2, -1.0)):
        d = m - U
        r = np.sqrt(np.maximum((d ** 2).sum(1), 1e-9))
        b = np.arctan2(d[:, 1], d[:, 0])
        h = np.zeros((U.shape[0], 4))
        h[:, 0] = -np.sin(b) / r
        h[:, 1] = np.cos(b) / r
        h[:, 2] = h[:, 0] * s * dmx + h[:, 1] * s * dmy
        acc += h[:, :, None] * h[:, None, :]          # 每个标志点各自贡献，不能先相加
    I = H0[None] + PVIS_TAB[:, None, None] * acc / CFG["sigma_beta"] ** 2
    try:
        Pp = np.linalg.inv(I)
    except np.linalg.LinAlgError:
        return u_cur
    cost = (np.sqrt(np.maximum(Pp[:, 2, 2], 1e-12))
            + CFG["w_pos"] * np.sqrt(np.maximum(Pp[:, 0, 0] + Pp[:, 1, 1], 1e-12))
            + CFG["w_move"] * np.linalg.norm(U - u_cur, axis=1))
    return U[int(np.argmin(cost))]


def run(name, seed, p_loss=0.0, record=False):
    rng = np.random.default_rng(seed)
    dt, T = CFG["dt"], CFG["T"]
    N = int(T / dt)
    xt = np.array([0.0, 0.0, 0.0]); b_true = CFG["bias_true"]
    est = np.zeros(4)
    P = np.diag([CFG["p0"] ** 2, CFG["p0"] ** 2, CFG["s_theta0"] ** 2, CFG["s_b0"] ** 2])
    u = np.array([0.0, -10.0])
    dist = 0.0
    if record:
        log = {k: [] for k in ("t", "tx", "ty", "tt", "ex", "ey", "et", "uav", "pvis", "ftheta")}

    for k in range(N):
        t = k * dt
        v = CFG["v0"] * (1.0 + 0.05 * math.sin(2 * math.pi * t / 17.0))
        w = 0.30 * math.sin(2 * math.pi * t / 30.0)
        xt[0] += dt * v * math.cos(xt[2]); xt[1] += dt * v * math.sin(xt[2]); xt[2] += dt * w
        b_true += dt * CFG["bias_rw"] * rng.standard_normal()

        # 无人机机动
        if name == "follow10":       # 车体坐标系下 跨行侧 10 m
            c, s_ = math.cos(est[2]), math.sin(est[2])
            tgt = np.array([est[0] + CFG["baseline_r"] * s_, est[1] - CFG["baseline_r"] * c])
        elif name == "axis10":       # 车体坐标系下 正前方 10 m
            c, s_ = math.cos(est[2]), math.sin(est[2])
            tgt = np.array([est[0] + CFG["baseline_r"] * c, est[1] + CFG["baseline_r"] * s_])
        elif name == "random":
            tgt = None
            if rng.random() < 1 / 40:
                a = rng.uniform(0, 2 * math.pi); rr = rng.uniform(4, 16)
                tgt = np.array([est[0] + rr * math.cos(a), est[1] + rr * math.sin(a)])
        elif name == "ours":
            tgt = policy_ours(est[0], est[1], est[2], P, u)
        else:
            tgt = None
        if tgt is not None:
            d = tgt - u; nd = float(np.linalg.norm(d))
            if nd > 1e-9:
                step = CFG["uav_vmax"] * dt
                mv = d if nd <= step else d / nd * step
                u = u + mv; dist += float(np.linalg.norm(mv))
        rel = u - xt[:2]; nr = float(np.linalg.norm(rel))
        if nr > CFG["uav_rmax"]:
            u = xt[:2] + rel / nr * CFG["uav_rmax"]

        # EKF 预测
        v_m = v * (1.0 + CFG["v_noise"] * rng.standard_normal())
        w_m = w + b_true + CFG["gyro_noise"] * rng.standard_normal()
        th0 = est[2]
        est[0] += dt * v_m * math.cos(th0); est[1] += dt * v_m * math.sin(th0)
        est[2] += dt * (w_m - est[3])
        F = np.array([[1, 0, -dt * v_m * math.sin(th0), 0],
                      [0, 1, dt * v_m * math.cos(th0), 0],
                      [0, 0, 1, -dt], [0, 0, 0, 1]], float)
        Q = np.diag([(dt * CFG["v_noise"] * CFG["v0"]) ** 2, (dt * CFG["v_noise"] * CFG["v0"]) ** 2,
                     (dt * CFG["gyro_noise"]) ** 2, (dt * CFG["bias_rw"]) ** 2])
        P = F @ P @ F.T + Q

        # 观测
        pv = pvis_of(u, xt[0], xt[1])
        if rng.random() < pv and rng.random() >= p_loss:
            zt = noisy_bearings(u, xt[0], xt[1], xt[2], rng)
            H, zp = jac_and_bearings(u, est[0], est[1], est[2], CFG["L"])
            yv = np.array([wrap(zt[0] - zp[0]), wrap(zt[1] - zp[1])])
            S = H @ P @ H.T + np.eye(2) * CFG["sigma_beta"] ** 2
            K = P @ H.T @ np.linalg.inv(S)
            est = est + K @ yv; est[2] = wrap(est[2])
            P = (np.eye(4) - K @ H) @ P; P = 0.5 * (P + P.T)

        if record:
            log["t"].append(t); log["tx"].append(xt[0]); log["ty"].append(xt[1]); log["tt"].append(xt[2])
            log["ex"].append(est[0]); log["ey"].append(est[1]); log["et"].append(est[2])
            log["uav"].append(u.copy()); log["pvis"].append(pv)
            log["ftheta"].append(ftheta(u, xt[0], xt[1], xt[2], CFG["L"], CFG["sigma_beta"]))

    # 误差统计（用最后 20 个存点做简化记录时的近似不可行，故仅在 record 模式下用日志）
    if record:
        ex = np.array(log["ex"]) - np.array(log["tx"]); ey = np.array(log["ey"]) - np.array(log["ty"])
        et = np.array([wrap(a) for a in np.array(log["et"]) - np.array(log["tt"])])
        return dict(pos_rmse=float(np.sqrt(np.mean(ex ** 2 + ey ** 2))),
                    head_rmse=float(np.sqrt(np.mean(et ** 2))) * 180 / math.pi,
                    mean_pvis=float(np.mean(log["pvis"])), dist=dist, log=log)
    return dict(dist=dist)


def run_metrics(name, seed, p_loss=0.0):
    """轻量版：只累计误差，不存日志"""
    rng = np.random.default_rng(seed)
    dt, T = CFG["dt"], CFG["T"]; N = int(T / dt)
    xt = np.array([0.0, 0.0, 0.0]); est = np.zeros(4)
    b_true = CFG["bias_true"]
    P = np.diag([CFG["p0"] ** 2, CFG["p0"] ** 2, CFG["s_theta0"] ** 2, CFG["s_b0"] ** 2])
    u = np.array([0.0, -10.0]); dist = 0.0
    se_p = 0.0; se_h = 0.0; pv_sum = 0.0
    for k in range(N):
        t = k * dt
        v = CFG["v0"] * (1.0 + 0.05 * math.sin(2 * math.pi * t / 17.0))
        w = 0.30 * math.sin(2 * math.pi * t / 30.0)
        xt[0] += dt * v * math.cos(xt[2]); xt[1] += dt * v * math.sin(xt[2]); xt[2] += dt * w
        b_true += dt * CFG["bias_rw"] * rng.standard_normal()

        if name == "follow10":
            c, s_ = math.cos(est[2]), math.sin(est[2])
            tgt = np.array([est[0] + CFG["baseline_r"] * s_, est[1] - CFG["baseline_r"] * c])
        elif name == "axis10":
            c, s_ = math.cos(est[2]), math.sin(est[2])
            tgt = np.array([est[0] + CFG["baseline_r"] * c, est[1] + CFG["baseline_r"] * s_])
        elif name == "random":
            tgt = None
            if rng.random() < 1 / 40:
                a = rng.uniform(0, 2 * math.pi); rr = rng.uniform(4, 16)
                tgt = np.array([est[0] + rr * math.cos(a), est[1] + rr * math.sin(a)])
        elif name == "ours":
            tgt = policy_ours(est[0], est[1], est[2], P, u)
        else:
            tgt = None
        if tgt is not None:
            d = tgt - u; nd = float(np.linalg.norm(d))
            if nd > 1e-9:
                step = CFG["uav_vmax"] * dt
                mv = d if nd <= step else d / nd * step
                u = u + mv; dist += float(np.linalg.norm(mv))
        rel = u - xt[:2]; nr = float(np.linalg.norm(rel))
        if nr > CFG["uav_rmax"]:
            u = xt[:2] + rel / nr * CFG["uav_rmax"]

        v_m = v * (1.0 + CFG["v_noise"] * rng.standard_normal())
        w_m = w + b_true + CFG["gyro_noise"] * rng.standard_normal()
        th0 = est[2]
        est[0] += dt * v_m * math.cos(th0); est[1] += dt * v_m * math.sin(th0); est[2] += dt * (w_m - est[3])
        F = np.array([[1, 0, -dt * v_m * math.sin(th0), 0], [0, 1, dt * v_m * math.cos(th0), 0],
                      [0, 0, 1, -dt], [0, 0, 0, 1]], float)
        Q = np.diag([(dt * CFG["v_noise"] * CFG["v0"]) ** 2, (dt * CFG["v_noise"] * CFG["v0"]) ** 2,
                     (dt * CFG["gyro_noise"]) ** 2, (CFG["bias_rw"]) ** 2])
        P = F @ P @ F.T + Q

        pv = pvis_of(u, xt[0], xt[1]); pv_sum += pv
        if rng.random() < pv and rng.random() >= p_loss:
            zt = noisy_bearings(u, xt[0], xt[1], xt[2], rng)
            H, zp = jac_and_bearings(u, est[0], est[1], est[2], CFG["L"])
            yv = np.array([wrap(zt[0] - zp[0]), wrap(zt[1] - zp[1])])
            S = H @ P @ H.T + np.eye(2) * CFG["sigma_beta"] ** 2
            K = P @ H.T @ np.linalg.inv(S)
            est = est + K @ yv; est[2] = wrap(est[2])
            P = (np.eye(4) - K @ H) @ P; P = 0.5 * (P + P.T)

        se_p += (est[0] - xt[0]) ** 2 + (est[1] - xt[1]) ** 2
        se_h += wrap(est[2] - xt[2]) ** 2

    return dict(pos_rmse=math.sqrt(se_p / N), head_rmse=math.sqrt(se_h / N) * 180 / math.pi,
                mean_pvis=pv_sum / N, dist=dist)


POLICIES = [("hover", "A 悬停（不跟随）"),
            ("follow10", "B 定距跟随（跨行侧 10 m）"),
            ("axis10", "C 沿轴前视（行向 10 m）"),
            ("random", "D 随机机动"),
            ("ours", "E 可辨识度驱动（本文）")]


def run_consistency(seed):
    """一致性检验：两个估计器共享同一批观测，进行位置融合。
       naive = 视为独立直接相乘（重复计信息）；CI = 协方差交集（保守）。
       返回 NEES 时序（2 自由度，95% 界 5.99）。"""
    rng = np.random.default_rng(seed)
    dt, T = CFG["dt"], CFG["T"]; N = int(T / dt)
    xt = np.array([0.0, 0.0, 0.0]); b_true = CFG["bias_true"]
    e1 = np.zeros(4); P1 = np.diag([CFG["p0"] ** 2, CFG["p0"] ** 2, CFG["s_theta0"] ** 2, CFG["s_b0"] ** 2])
    e2 = np.zeros(4); P2 = P1.copy() * 4.0
    u = np.array([0.0, -10.0])
    nees_n, nees_c = [], []
    for k in range(N):
        t = k * dt
        v = CFG["v0"] * (1.0 + 0.05 * math.sin(2 * math.pi * t / 17.0))
        w = 0.30 * math.sin(2 * math.pi * t / 30.0)
        xt[0] += dt * v * math.cos(xt[2]); xt[1] += dt * v * math.sin(xt[2]); xt[2] += dt * w
        tgt = policy_ours(e1[0], e1[1], e1[2], P1, u)
        d = tgt - u; nd = float(np.linalg.norm(d)); step = CFG["uav_vmax"] * dt
        u = u + (d if nd <= step else d / nd * step)
        rel = u - xt[:2]; nr = float(np.linalg.norm(rel))
        if nr > CFG["uav_rmax"]:
            u = xt[:2] + rel / nr * CFG["uav_rmax"]

        v_m = v * (1.0 + CFG["v_noise"] * rng.standard_normal())
        w_m = w + b_true + CFG["gyro_noise"] * rng.standard_normal()
        F = np.array([[1, 0, -dt * v_m * math.sin(xt[2]), 0], [0, 1, dt * v_m * math.cos(xt[2]), 0],
                      [0, 0, 1, -dt], [0, 0, 0, 1]], float)
        Q1 = np.diag([(dt * CFG["v_noise"] * CFG["v0"]) ** 2, (dt * CFG["v_noise"] * CFG["v0"]) ** 2,
                      (dt * CFG["gyro_noise"]) ** 2, (CFG["bias_rw"]) ** 2])
        for est, P, Q in ((e1, P1, Q1), (e2, P2, Q1 * CFG.get("q2_mult", 20.0))):
            est[0] += dt * v_m * math.cos(est[2]); est[1] += dt * v_m * math.sin(est[2])
            est[2] += dt * (w_m - est[3])
            P[:] = F @ P @ F.T + Q
        pv = pvis_of(u, xt[0], xt[1])
        if rng.random() < pv:
            zt = noisy_bearings(u, xt[0], xt[1], xt[2], rng)
            for est, P in ((e1, P1), (e2, P2)):
                H, zp = jac_and_bearings(u, est[0], est[1], est[2], CFG["L"])
                yv = np.array([wrap(zt[0] - zp[0]), wrap(zt[1] - zp[1])])
                S = H @ P @ H.T + np.eye(2) * CFG["sigma_beta"] ** 2
                K = P @ H.T @ np.linalg.inv(S)
                est[:] = est + K @ yv; est[2] = wrap(est[2])
                P[:] = (np.eye(4) - K @ H) @ P; P[:] = 0.5 * (P + P.T)
        # 位置块融合
        I1 = np.linalg.inv(P1[:2, :2]); I2 = np.linalg.inv(P2[:2, :2])
        y1 = I1 @ e1[:2]; y2 = I2 @ e2[:2]
        for wts, bucket in (((1.0, 1.0), nees_n), ((0.5, 0.5), nees_c)):
            If = wts[0] * I1 + wts[1] * I2
            xf = np.linalg.solve(If, wts[0] * y1 + wts[1] * y2)
            err = xf - xt[:2]
            bucket.append(float(err @ If @ err))
    return np.array(nees_n), np.array(nees_c)



def _selftest():
    """极简自检：确认 EKF 与信息量计算可跑"""
    r = run_metrics("ours", 0)
    assert 0.0 < r["head_rmse"] < 90.0, r
    return r


def main():
    import argparse
    ap = argparse.ArgumentParser(description="P1 仿真：策略对比 / 丢包 / 一致性")
    ap.add_argument("--vis", choices=["gauss", "lut"], default="gauss",
                    help="可见性模型：gauss=旧乘积高斯；lut=E1 标定查找表")
    ap.add_argument("--case", choices=["alley_center", "canopy_edge"], default="alley_center",
                    help="LUT 工况（仅 --vis lut 有效）")
    ap.add_argument("--lut", default=os.path.normpath(os.path.join(
        OUT, "..", "..", "08_Gazebo仿真", "vis_lut.npz")), help="vis_lut.npz 路径")
    ap.add_argument("--seeds", type=int, default=15, help="Exp1 重复次数")
    ap.add_argument("--wpos", type=float, default=0.0,
                    help="位置项权重（双目标；0 = 仅航向）")
    args = ap.parse_args()

    global PVIS_TAB, FIG
    nseeds = args.seeds
    CFG["w_pos"] = args.wpos
    if args.vis == "lut":
        _lut_load(args.lut, args.case)
        VIS["mode"] = "lut"
        PVIS_TAB = build_pvis_tab()
        suffix = f"figures_lut_{args.case}"
        if args.wpos:
            suffix += f"_k{args.wpos:g}"
        FIG = os.path.join(OUT, suffix)
        os.makedirs(FIG, exist_ok=True)
    # Exp2/Exp3 与 Exp1 使用相同的种子数（2026-11-04 起统一，原为 nseeds//2 导致丢包实验仅 7 seeds）
    nseeds2 = nseeds
    tag = "" if args.vis == "gauss" else f"_lut_{args.case}"
    if args.wpos:
        tag += f"_k{args.wpos:g}"
    print(f"=== 可见性模型: {args.vis}"
          f"{(' / ' + args.case) if args.vis == 'lut' else ''}, 种子数 {nseeds} ===", flush=True)

    print("=== Exp1 策略对比 ===", flush=True)
    res1 = {}
    for key, label in POLICIES:
        rs = [run_metrics(key, s) for s in range(nseeds)]
        res1[key] = dict(label=label,
                         pos_rmse=float(np.mean([r["pos_rmse"] for r in rs])),
                         pos_std=float(np.std([r["pos_rmse"] for r in rs])),
                         head_rmse=float(np.mean([r["head_rmse"] for r in rs])),
                         head_std=float(np.std([r["head_rmse"] for r in rs])),
                         mean_pvis=float(np.mean([r["mean_pvis"] for r in rs])),
                         dist=float(np.mean([r["dist"] for r in rs])))
        print(f"{label:26s} pos={res1[key]['pos_rmse']:.3f} m  head={res1[key]['head_rmse']:.2f} deg  "
              f"pvis={res1[key]['mean_pvis']:.3f}  dist={res1[key]['dist']:.1f} m", flush=True)

    print("=== Exp2 通信丢包敏感度 ===", flush=True)
    losses = [0.0, 0.1, 0.2, 0.35, 0.5]
    res2 = {}
    for key in ("follow10", "axis10", "ours"):
        res2[key] = []
        for q in losses:
            rs = [run_metrics(key, s, p_loss=q) for s in range(nseeds2)]
            res2[key].append(dict(p_loss=q,
                                  pos_rmse=float(np.mean([r["pos_rmse"] for r in rs])),
                                  head_rmse=float(np.mean([r["head_rmse"] for r in rs]))))
        print(key, [round(v["head_rmse"], 2) for v in res2[key]], flush=True)

    print("=== Exp3 融合一致性（NEES）===", flush=True)
    ns, cs = [], []
    for s in range(nseeds2):
        a, b = run_consistency(s)
        ns.append(a); cs.append(b)
    ns = np.concatenate(ns); cs = np.concatenate(cs)
    bound = 5.991
    res3 = dict(nees_naive_mean=float(ns.mean()), nees_ci_mean=float(cs.mean()),
                naive_above_rate=float((ns > bound).mean()), ci_above_rate=float((cs > bound).mean()),
                chi2_bound_95=bound, nominal_rate=0.05)
    print(res3, flush=True)

    json.dump(dict(cfg={k: v for k, v in CFG.items()},
                   vis_mode=args.vis, vis_case=(args.case if args.vis == "lut" else None),
                   exp1=res1, exp2=res2, exp3=res3),
              open(os.path.join(OUT, f"metrics{tag}.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)

    # ================= 图 1 轨迹与误差 =================
    fig = plt.figure(figsize=(11.2, 4.2))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.15, 1.5])
    ax0 = fig.add_subplot(gs[0, 0]); ax1 = fig.add_subplot(gs[0, 1]); ax2 = fig.add_subplot(gs[0, 2])

    r = run("ours", 3, record=True); lg = r["log"]
    ax0.plot(lg["tx"], lg["ty"], "k-", lw=1.6, label="UGV 真值")
    ax0.plot(lg["ex"], lg["ey"], "--", color="#d1495b", lw=1.1, label="EKF 估计")
    ua = np.array(lg["uav"])
    ax0.plot(ua[:, 0], ua[:, 1], color="#2a9d8f", lw=1.0, alpha=.9, label="UAV 观测位")
    ax0.set_xlabel("x / m"); ax0.set_ylabel("y / m")
    ax0.legend(fontsize=7, loc="upper left"); ax0.grid(alpha=.3)
    ax0.set_title("(a) 水平轨迹（纵向压缩显示）", fontsize=9.5)

    rr = run("follow10", 3, record=True)
    ax1.plot(rr["log"]["tx"], rr["log"]["ty"], "k-", lw=1.6)
    ua = np.array(rr["log"]["uav"])
    ax1.plot(ua[:, 0], ua[:, 1], color="#4c72b0", lw=1.0, label="UAV 观测位")
    ax1.set_xlabel("x / m"); ax1.set_ylabel("y / m")
    ax1.legend(fontsize=7, loc="upper left"); ax1.grid(alpha=.3)
    ax1.set_title("(b) 对照：定距跟随 10 m", fontsize=9.5)

    for key, label in POLICIES:
        rq = run(key, 3, record=True)
        et = np.array([wrap(a) for a in np.array(rq["log"]["et"]) - np.array(rq["log"]["tt"])]) * 180 / math.pi
        Wr = 100
        rms = np.sqrt(np.convolve(et ** 2, np.ones(Wr) / Wr, mode="valid"))
        ax2.plot(rq["log"]["t"][Wr - 1:], rms, lw=1.3, label=label)
    ax2.set_xlabel("时间 / s"); ax2.set_ylabel("航向误差滑动 RMS / deg（窗长 10 s）")
    ax2.grid(alpha=.3); ax2.legend(fontsize=6.5, ncol=1, loc="upper right")
    ax2.set_title("(c) 航向误差时序（同一随机种子）", fontsize=9.5)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig1_trajectory.png")); plt.close(fig)
    print("fig1 done", flush=True)

    # ================= 图 2 期望信息地图 + 实际观测位 =================
    gx = np.linspace(-12, 12, 241); gy = np.linspace(-12, 12, 241)
    GX, GY = np.meshgrid(gx, gy)
    U = np.stack([GX.ravel(), GY.ravel()], 1)
    Facc = np.zeros(U.shape[0])
    m1, m2 = markers(0, 0, 0.0, CFG["L"])
    for m, s in ((m1, 1.0), (m2, -1.0)):
        d = m - U; rr_ = np.sqrt(np.maximum((d ** 2).sum(1), 1e-9))
        b = np.arctan2(d[:, 1], d[:, 0])
        hth = (np.cos(b) / rr_) * s * 0.5 * CFG["L"]
        Facc += hth ** 2
    Finfo = Facc / CFG["sigma_beta"] ** 2
    PVv = np.asarray(pvis_field(GX, GY), float).ravel()
    Z = (PVv * Finfo).reshape(GX.shape)

    fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.9))
    ax = axes[0]
    cset = ax.contourf(gx, gy, np.log10(Z + 1e-6), levels=20, cmap="magma")
    fig.colorbar(cset, ax=ax, shrink=.85, label="log10 期望信息量")
    q = np.array(r["log"]["uav"]) - np.stack([r["log"]["tx"], r["log"]["ty"]], 1)
    ax.scatter(q[:, 0], q[:, 1], s=3, c="#00e5ff", alpha=.45, label="E 可辨识度驱动")
    for key, col in (("follow10", "#7ee787"), ("axis10", "#ffd166")):
        rq = run(key, 3, record=True)
        qq = np.array(rq["log"]["uav"]) - np.stack([rq["log"]["tx"], rq["log"]["ty"]], 1)
        ax.scatter(qq[:, 0], qq[:, 1], s=3, c=col, alpha=.5,
                   label={"follow10": "B 定距跟随 10 m", "axis10": "C 沿轴前视 10 m"}[key])
    ax.plot(0, 0, "w*", ms=15, label="UGV 中心")
    ax.annotate("", xy=(3.4, 0), xytext=(1.0, 0), arrowprops=dict(arrowstyle="->", color="w", lw=1.8))
    _head_note = ("车头方向（遮挡重）" if VIS.get("case") == "canopy_edge" or VIS["mode"] != "lut"
                  else "车头方向（通道开阔，无遮挡）")
    ax.text(3.5, .35, _head_note, color="w", fontsize=8)
    ax.set_xlim(-12, 12); ax.set_ylim(-12, 12); ax.set_aspect("equal")
    ax.set_xlabel("相对 x / m（沿行方向）"); ax.set_ylabel("相对 y / m（跨行方向）")
    ax.legend(fontsize=7.2, loc="upper right", markerscale=1.4, framealpha=.85)
    ax.set_title("(a) 期望信息量场与各策略落点", fontsize=10)

    # (b) 三条射线上的期望信息量随距离变化
    axr = axes[1]
    rr = np.linspace(1.0, 20.0, 200)
    for ang, lab in ((0.0, "沿行方向 (0°)"), (math.pi / 4, "斜 45°"), (math.pi / 2, "跨行方向 (90°)")):
        Uv = np.stack([rr * math.cos(ang), rr * math.sin(ang)], 1)
        FF = np.zeros(len(rr))
        for m, s in ((m1, 1.0), (m2, -1.0)):
            d = m - Uv; rn = np.sqrt(np.maximum((d ** 2).sum(1), 1e-9))
            b = np.arctan2(d[:, 1], d[:, 0])
            FF += ((np.cos(b) / rn) * s * 0.5 * CFG["L"]) ** 2
        PV = np.asarray(pvis_field(Uv[:, 0], Uv[:, 1]), float)
        axr.semilogy(rr, PV * FF / CFG["sigma_beta"] ** 2, lw=1.6, label=lab)
    axr.axvline(10.0, color="gray", ls="--", lw=1.0, label="惯例定距 10 m")
    axr.set_xlabel("观测距离 r / m"); axr.set_ylabel("期望信息量 p$_{vis}$·F$_{\\theta\\theta}$")
    axr.grid(alpha=.3, which="both"); axr.legend(fontsize=8)
    axr.set_title("(b) 三个方向上的信息量-距离曲线", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig2_information_map.png")); plt.close(fig)
    print("fig2 done", flush=True)

    # ================= 图 3 策略对比（横向条形） =================
    labs = [res1[k]["label"] for k, _ in POLICIES][::-1]
    hv = [res1[k]["head_rmse"] for k, _ in POLICIES][::-1]
    hs = [res1[k]["head_std"] for k, _ in POLICIES][::-1]
    pv_ = [res1[k]["pos_rmse"] for k, _ in POLICIES][::-1]
    ps_ = [res1[k]["pos_std"] for k, _ in POLICIES][::-1]
    ypos = np.arange(len(labs))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.0))
    axes[0].barh(ypos, hv, xerr=hs, capsize=3, color="#4c72b0")
    axes[0].set_yticks(ypos); axes[0].set_yticklabels(labs, fontsize=8.5)
    axes[0].set_xlabel("航向 RMSE / deg"); axes[0].grid(axis="x", alpha=.3)
    axes[0].set_title("(a) 航向误差（15 次重复，误差棒 1σ）", fontsize=10)
    axes[1].barh(ypos, pv_, xerr=ps_, capsize=3, color="#dd8452")
    axes[1].set_yticks(ypos); axes[1].set_yticklabels([])
    axes[1].set_xlabel("位置 RMSE / m"); axes[1].grid(axis="x", alpha=.3)
    axes[1].set_title("(b) 位置误差", fontsize=10)
    for a in axes:
        a.invert_yaxis()
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig3_policy_compare.png")); plt.close(fig)
    print("fig3 done", flush=True)

    # ================= 图 4 丢包敏感度 =================
    fig, ax = plt.subplots(figsize=(5.9, 4.2))
    lab = dict(POLICIES)
    for key, vals in res2.items():
        ax.plot(losses, [v["head_rmse"] for v in vals], "o-", ms=4.5, lw=1.5, label=lab[key])
    ax.set_xlabel("通信丢包率"); ax.set_ylabel("航向 RMSE / deg")
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title("通信受限下的性能退化", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig4_loss_sensitivity.png")); plt.close(fig)
    print("fig4 done", flush=True)

    # ================= 图 5 一致性 =================
    W = 200
    kmov_n = np.convolve(ns, np.ones(W) / W, mode="valid")
    kmov_c = np.convolve(cs, np.ones(W) / W, mode="valid")
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.0))
    axes[0].plot(np.arange(len(kmov_n)), kmov_n, lw=1.3, color="#c44e52", label="朴素融合（视为独立）")
    axes[0].plot(np.arange(len(kmov_c)), kmov_c, lw=1.3, color="#4c72b0", label="协方差交集 CI")
    axes[0].axhline(bound, color="k", ls="--", lw=1.0, label="χ² 95% 上界 = 5.99")
    axes[0].axhline(2.0, color="gray", ls=":", lw=1.0, label="理论期望 = 2.00")
    axes[0].set_xlabel("时间步"); axes[0].set_ylabel("NEES 滑动均值（窗长 200）")
    axes[0].grid(alpha=.3); axes[0].legend(fontsize=7.5)
    axes[0].set_title("(a) NEES 一致性追踪", fontsize=10)

    rates = [float((ns > bound).mean()) * 100, float((cs > bound).mean()) * 100, 5.0]
    axes[1].bar(["朴素融合", "协方差交集", "理论值"], rates,
                color=["#c44e52", "#4c72b0", "gray"])
    for i, v in enumerate(rates):
        axes[1].text(i, v + 0.25, f"{v:.1f}%", ha="center", fontsize=9)
    axes[1].set_ylabel("NEES 超过 95% 界的比例 / %")
    axes[1].grid(axis="y", alpha=.3)
    axes[1].set_title("(b) 过度自信程度", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig5_nees.png")); plt.close(fig)
    print("fig5 done", flush=True)

    print("ALL DONE ->", FIG, flush=True)


if __name__ == "__main__":
    main()
