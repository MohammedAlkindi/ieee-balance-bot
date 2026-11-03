"""Plain-Python model of the robot for rough gain tuning before hardware.

Body on two wheels, driven by steppers. A stepper sets the wheel's angle
relative to the body (psi), so the input is the commanded relative acceleration
a = R * psi_ddot. Lagrange's equations for a body of mass M (centre of mass at
height L above the axle, inertia I_b about its own centre) on wheels of radius R,
total mass m_w and inertia I_w, rolling without slip, give

    D(theta) * theta_ddot = M g L sin(theta) + M R L sin(theta) theta_dot^2
                            - (A_w + M R L cos(theta)) * a / R
    A_w = (M + m_w) R^2 + I_w,   D = A_w + 2 M R L cos(theta) + M L^2 + I_b

theta is pitch (rad, + = leaning forward). For a point mass on light wheels this
linearises to theta_ddot = g L theta / (L + R)^2 - a / (L + R). Ground speed is
R * (psi_dot + theta_dot). The controller is the same cascaded law the firmware
runs at 500 Hz, fed the commanded wheel speed v (what the step generator makes):

    theta_ref = clamp(Kv * (v_target - v) + Ki * integral(v_target - v), +-THETA_REF_MAX)
    a         = clamp(Kp * (theta - theta_ref) + Kd * theta_dot, +-A_MAX)
    v        += a * dt  (clamped to +-V_MAX)

Run:  python sim.py            (prints recovery metrics for a few scenarios)
      python sim.py --sweep    (grid over Kp and Kd, lists the gains that survive all scenarios)
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass

G = 9.81
DT = 0.002  # 500 Hz


@dataclass
class Gains:
    kp: float = 40.0   # (m/s^2) per rad of pitch error; same defaults as firmware config.h
    kd: float = 3.0    # (m/s^2) per rad/s of pitch rate
    kv: float = 0.10   # rad of target lean per m/s of speed error
    ki: float = 0.05   # rad per (m/s * s)


@dataclass
class Robot:
    L: float = 0.10       # axle to centre of mass, m
    M: float = 1.0        # body mass incl. motors and battery, kg
    body_h: float = 0.25  # body height, for its inertia (treated as a uniform slab)
    R: float = 0.045      # wheel radius, m
    m_w: float = 0.10     # both wheels, kg
    a_max: float = 12.0   # wheel acceleration the steppers can deliver, m/s^2
    v_max: float = 1.0    # wheel speed limit, m/s
    noise_deg: float = 0.15
    delay_steps: int = 1  # one control period of sensing/actuation delay


THETA_REF_MAX = math.radians(8)
CUTOFF = math.radians(35)


def simulate(g: Gains, r: Robot, theta0_deg=0.0, push_dps=0.0, push_at=0.5, seconds=4.0, seed=1):
    """Return (survived, peak_deg, settle_s, final_drift_m)."""
    rng = random.Random(seed)
    th, thd = math.radians(theta0_deg), 0.0
    v = x = integ = 0.0
    I_b = r.M * r.body_h ** 2 / 12
    I_w = 0.5 * r.m_w * r.R ** 2
    A_w = (r.M + r.m_w) * r.R ** 2 + I_w
    pending = [0.0] * (r.delay_steps + 1)
    peak, settled_at = abs(th), None
    sub = 10  # integrate the plant at 5 kHz inside each 500 Hz control step
    for k in range(int(seconds / DT)):
        t = k * DT
        if push_dps and abs(t - push_at) < DT / 2:
            thd += math.radians(push_dps)  # a shove = sudden pitch rate
        # controller sees noisy pitch, clean gyro rate (gyro noise is small at this scale)
        th_meas = th + math.radians(rng.gauss(0, r.noise_deg))
        err_v = 0.0 - v
        integ += err_v * DT
        th_ref = max(-THETA_REF_MAX, min(THETA_REF_MAX, g.kv * err_v + g.ki * integ))
        a_cmd = max(-r.a_max, min(r.a_max, g.kp * (th_meas - th_ref) + g.kd * thd))
        pending.append(a_cmd)
        a = pending.pop(0)
        for _ in range(sub):
            h = DT / sub
            v_new = max(-r.v_max, min(r.v_max, v + a * h))
            a_eff = (v_new - v) / h  # relative wheel acceleration actually commanded
            v = v_new
            s_, c_ = math.sin(th), math.cos(th)
            D = A_w + 2 * r.M * r.R * r.L * c_ + r.M * r.L ** 2 + I_b
            thdd = (r.M * G * r.L * s_ + r.M * r.R * r.L * s_ * thd ** 2
                    - (A_w + r.M * r.R * r.L * c_) * a_eff / r.R) / D
            thd += thdd * h
            th += thd * h
            x += (v + r.R * thd) * h  # ground speed = relative wheel speed + body rotation
        peak = max(peak, abs(th))
        if abs(th) > CUTOFF:
            return False, math.degrees(peak), None, x
        if abs(th) < math.radians(1) and abs(thd) < math.radians(10):
            settled_at = settled_at if settled_at is not None else t
        else:
            settled_at = None
    return True, math.degrees(peak), settled_at, x


SCENARIOS = [
    ("start tilted 5 deg", dict(theta0_deg=5)),
    ("start tilted 10 deg", dict(theta0_deg=10)),
    ("shove 60 deg/s", dict(push_dps=60)),
    ("shove 120 deg/s", dict(push_dps=120)),
]


def report(g: Gains, r: Robot) -> bool:
    ok_all = True
    for name, kw in SCENARIOS:
        ok, peak, settle, drift = simulate(g, r, **kw)
        ok_all &= ok and settle is not None
        status = f"settled in {settle:.2f} s" if ok and settle is not None else ("never settled" if ok else "FELL")
        print(f"  {name:20s} peak {peak:5.1f} deg  {status:18s} drift {drift:+.2f} m")
    return ok_all


def sweep(r: Robot) -> None:
    good = []
    for kp in (15, 20, 25, 30, 40, 50, 60):
        for kd in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0):
            g = Gains(kp=kp, kd=kd)
            if all(simulate(g, r, **kw)[0] and simulate(g, r, **kw)[2] is not None for _, kw in SCENARIOS):
                worst = max(simulate(g, r, **kw)[1] for _, kw in SCENARIOS)
                good.append((worst, kp, kd))
    for worst, kp, kd in sorted(good)[:10]:
        print(f"  kp {kp:4.0f}  kd {kd:3.1f}  worst peak {worst:4.1f} deg")
    if not good:
        print("  no gain pair survived every scenario")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--L", type=float, default=0.10, help="axle to centre-of-mass height, m")
    args = ap.parse_args()
    robot = Robot(L=args.L)
    if args.sweep:
        sweep(robot)
    else:
        print(f"default gains {Gains()} with L = {robot.L} m")
        report(Gains(), robot)
