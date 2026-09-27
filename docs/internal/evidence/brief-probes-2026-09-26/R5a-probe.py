"""L7 brief re-check probe (2026-09-26 evening). Mirrors sim::MotorModel (motor_model.hpp:80-106)
and a tank's yaw = (xR - xL) / W. Not project code; numbers only."""
import math
import numpy as np

DT = 0.01
W = 12.5  # calibration_test_rig.hpp:47


def vss(u, g):
    kS, kV, kA = g
    if abs(u) <= kS:
        return 0.0
    return math.copysign((abs(u) - kS) / kV, u)


def adv(v, u, g):
    kS, kV, kA = g
    s = vss(u, g)
    if kA <= 0:
        return s
    d = math.exp(-DT * kV / kA)
    return s + (v - s) * d


def segment(gL, gR, kind, seg=36.0, ramp=2.0, rampMax=10.0, step=6.0, tmax=8.0, coast=True):
    vL = vR = xL = xR = 0.0
    t = 0.0
    rows = []
    sgn = 1.0 if kind.endswith("F") else -1.0
    while True:
        u = sgn * (min(ramp * t, rampMax) if kind.startswith("Q") else step)
        nL, nR = adv(vL, u, gL), adv(vR, u, gR)
        rows.append((u, vL, nL, vR, nR))
        vL, vR = nL, nR
        xL += vL * DT
        xR += vR * DT
        t += DT
        if max(abs(xL), abs(xR)) >= seg or t >= tmax:
            break
    endx = (xL, xR)
    # coast-down on the plant (0 V, both sides): exponential decay with tau = kA/kV
    cL = cR = 0.0
    while abs(vL) > 1e-6 or abs(vR) > 1e-6:
        nL, nR = adv(vL, 0.0, gL), adv(vR, 0.0, gR)
        rows.append((0.0, vL, nL, vR, nR))
        vL, vR = nL, nR
        cL += vL * DT
        cR += vR * DT
        if abs(vL) < 1e-9 and abs(vR) < 1e-9:
            break
    yaw = (endx[1] - endx[0]) / W
    return rows, endx, (cL, cR), t, yaw


def fit(samples, minSpeed=1.0, admitZero=False):
    X, y = [], []
    for u, v0, v1 in samples:
        if (u == 0.0 and not admitZero) or abs(v0) < minSpeed or abs(v1) < minSpeed:
            continue
        X.append([v0, u, math.copysign(1.0, v0)])
        y.append(v1)
    X, y = np.array(X), np.array(y)
    cond = np.linalg.cond(X)
    sol, res, rank, sv = np.linalg.lstsq(X, y, rcond=None)
    a, b, c = sol
    kV = (1 - a) / b
    kS = -c / b
    kA = -kV * DT / math.log(a) if a > 0 else 0.0
    rms = math.sqrt(np.mean((X @ sol - y) ** 2))
    return dict(n=len(y), cond=cond, rank=rank, kS=kS, kV=kV, kA=kA, alpha=a, rms=rms)


def side(rows, s):
    return [(r[0], r[1 + 2 * s], r[2 + 2 * s]) for r in rows]


print("== Finding 1: yaw at segment end, brief's gains L{1.0,.16,.02} R{1.4,.19,.03}")
L, R = (1.0, 0.16, 0.02), (1.4, 0.19, 0.03)
for k in ("QF", "QB", "SF", "SB"):
    rows, endx, c, t, yaw = segment(L, R, k)
    print(f"  {k}: t={t:.2f}s xL={endx[0]:.2f} xR={endx[1]:.2f} yaw={math.degrees(yaw):.1f} deg; plant coast L={c[0]:.2f} R={c[1]:.2f} in")

print("== proposed gains ~5% apart, both orientations")
A, B = (1.0, 0.160, 0.020), (1.1, 0.168, 0.024)
for name, (gl, gr) in (("L fast", (A, B)), ("R fast", (B, A))):
    for k in ("QF", "QB", "SF", "SB"):
        rows, endx, c, t, yaw = segment(gl, gr, k)
        print(f"  {name} {k}: t={t:.2f}s xL={endx[0]:.2f} xR={endx[1]:.2f} yaw={math.degrees(yaw):.1f} deg")

print("== yaw ~ seg*asym/W for asym 0.15:", math.degrees(36 * 0.15 / W), "deg; guard 20 deg <=> asym",
      math.radians(20) * W / 36)

print("== Identifiability: quasi-only vs quasi+step (per side, L gains, kA>0)")
allrows = {k: segment(L, R, k)[0] for k in ("QF", "QB", "SF", "SB")}
q = side(allrows["QF"], 0) + side(allrows["QB"], 0)
qs = q + side(allrows["SF"], 0) + side(allrows["SB"], 0)
print("  quasi only :", fit(q))
print("  quasi+step :", fit(qs))
print("  quasi+step, u=0 admitted:", fit(qs, admitZero=True))
L0 = (1.0, 0.16, 0.0)
all0 = {k: segment(L0, L0, k)[0] for k in ("QF", "QB", "SF", "SB")}
q0 = side(all0["QF"], 0) + side(all0["QB"], 0)
qs0 = q0 + side(all0["SF"], 0) + side(all0["SB"], 0)
print("  kA=0 quasi only :", fit(q0))
print("  kA=0 quasi+step :", fit(qs0))

print("== coast residual on plant, per sample: (1-d)*kS/kV")
d = math.exp(-DT * 0.16 / 0.02)
print("  L:", (1 - d) * 1.0 / 0.16, "in/s ; kA=0:", 1.0 / 0.16)

print("== Finding 4: slope sigma, white heading vs integrated rate noise, 60 s at 100 Hz")
N, T = 6000, 60.0
sh = 0.05  # deg
print("  white heading 0.05 deg: slope sigma =", sh * math.sqrt(12) / (math.sqrt(N) * T) * 60, "deg/min")
sr = 0.5  # deg/s per sample
D = sr ** 2 * DT
print("  RW (0.5 deg/s at 100 Hz): LS slope sigma ~", math.sqrt(1.2 * D / T) * 60, "deg/min; mean-rate",
      math.sqrt(D / T) * 60, "deg/min")
rng = np.random.default_rng(1)
sl, res, dif = [], [], []
t = np.arange(N) * DT
for _ in range(400):
    h = np.cumsum(rng.normal(0, sr * DT, N))
    p = np.polyfit(t, h, 1)
    sl.append(p[0] * 60)
    res.append(np.std(h - np.polyval(p, t)))
    dif.append(np.std(np.diff(h)) / DT)
print("  MC: slope sd", np.std(sl), "deg/min; residual sd about line", np.mean(res), "deg ->",
      "sqrt2*res/dt =", math.radians(math.sqrt(2) * np.mean(res) / DT), "rad/s; diff sd/dt =",
      math.radians(np.mean(dif)), "rad/s")
print("  white-heading sqrt2*sigma/dt =", math.radians(math.sqrt(2) * sh / DT), "rad/s")
for Tm in (60, 120, 300):
    print(f"  RW slope sigma over {Tm}s:", math.sqrt(1.2 * D / Tm) * 60, "deg/min")

print("== D3 exact volts")
print("  79/7 =", 79 / 7, " (11.29-1)/(12/70) =", (11.29 - 1) / (12 / 70))
