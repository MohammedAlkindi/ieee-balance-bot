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

    theta_ref = clamp(Kv * (v_target - v) + Ki * integral(v_target - v) + Kff * a_target / g,
                      +-THETA_REF_MAX)
    a         = clamp(Kp * (theta - theta_ref) + Kd * theta_dot, +-A_MAX)
    v        += a * dt  (clamped to +-V_MAX)

Driving: the gamepad stick asks for a speed. The firmware's shaper moves v_target
toward it with limited acceleration a_target and jerk, and the robot leans into
that acceleration: theta_ref += Kff * a_target / g. Without the lean the slow speed
loop leaves the robot ~0.25 m/s behind the stick after 1.5 s.

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
    kff: float = 1.0   # lean into the drive target's acceleration: theta_ref += kff * a_target / g


@dataclass
class Robot:
    L: float = 0.10       # axle to centre of mass, m
    M: float = 1.0        # body mass incl. motors and battery, kg
    body_h: float = 0.25  # body height, for its inertia (treated as a uniform slab)
    R: float = 0.045      # wheel radius, m
    m_w: float = 0.10     # both wheels, kg
    a_max: float = 12.0   # wheel acceleration the steppers can deliver, m/s^2
    v_max: float = 1.0    # wheel speed limit, m/s
    imu_h: float = 0.10   # IMU height above the axle, m
    accel_noise_g: float = 0.004   # MPU-6050 at ~94 Hz bandwidth
    gyro_noise_dps: float = 0.05
    delay_steps: int = 1  # one control period of sensing/actuation delay
    accel_comp: float = 1.0  # firmware subtracts its own commanded acceleration from the accelerometer (0 = off)


THETA_REF_MAX = math.radians(8)
CUTOFF = math.radians(35)
COMP_TAU = 0.5  # complementary filter time constant, s (firmware COMP_FILTER_TAU_S)


@dataclass
class Drive:
    """Gamepad speed shaping, same defaults as firmware config.h DRIVE_*.

    The stick asks for a speed; the target moves toward it with limited
    acceleration and jerk, so the lean the robot needs builds up smoothly.
    """
    accel: float = 0.5   # m/s^2, most the target speeds up or slows down
    jerk: float = 2.0    # m/s^3, how fast that acceleration may change
    gain: float = 3.0    # 1/s, how hard the target closes on the stick


class Shaper:
    """One tick of the firmware's drive shaper (firmware: SpeedShaper in main/main.cpp)."""

    def __init__(self, d: Drive):
        self.d, self.v, self.a = d, 0.0, 0.0

    def step(self, stick_v: float, dt: float) -> tuple[float, float]:
        want = max(-self.d.accel, min(self.d.accel, (stick_v - self.v) * self.d.gain))
        self.a += max(-self.d.jerk * dt, min(self.d.jerk * dt, want - self.a))
        self.v += self.a * dt
        return self.v, self.a


def stick_step(speed, start=0.5, hold=3.0):
    """Push the stick to a speed at start, let go after hold seconds."""
    return lambda t: speed if start <= t < start + hold else 0.0


def simulate(g: Gains, r: Robot, theta0_deg=0.0, push_dps=0.0, push_at=0.5, seconds=4.0, seed=1,
             stick=None, drive=Drive(), stats=None):
    """Return (survived, peak_deg, settle_s, final_drift_m, peak_filter_error_deg, peak_torque_per_motor_Nm).

    The controller sees what the firmware sees: a simulated accelerometer and gyro
    fused by the same complementary filter, not the true pitch. stick(t) is the speed
    the gamepad asks for (default 0, stay in place); the firmware's shaper turns it
    into the speed target. If stats is a dict, it gets the worst ground-speed error
    once the stick has been steady for 1.5 s, and the lowest wheel-speed headroom.
    """
    rng = random.Random(seed)
    th, thd, thdd, xdd = math.radians(theta0_deg), 0.0, 0.0, 0.0
    v = x = integ = 0.0
    a_cmd_prev = 0.0
    I_b = r.M * r.body_h ** 2 / 12
    I_w = 0.5 * r.m_w * r.R ** 2
    A_w = (r.M + r.m_w) * r.R ** 2 + I_w
    alpha = COMP_TAU / (COMP_TAU + DT)
    pitch_est = th
    pending = [0.0] * (r.delay_steps + 1)
    peak, settled_at, peak_err, peak_tau = abs(th), None, 0.0, 0.0
    sub = 10  # integrate the plant 10x faster than the control loop
    shaper = Shaper(drive)
    stick_prev, stick_since, steady_err, headroom = 0.0, 0.0, 0.0, r.v_max
    for k in range(int(seconds / DT)):
        t = k * DT
        want = stick(t) if stick else 0.0
        vt, at = shaper.step(want, DT)
        if want != stick_prev:
            stick_prev, stick_since = want, t
        elif t - stick_since > 1.5:  # stick steady: the robot should be cruising at that speed
            steady_err = max(steady_err, abs(want - (v + r.R * thd)))
        if push_dps and abs(t - push_at) < DT / 2:
            thd += math.radians(push_dps)  # a shove = sudden pitch rate

        # sensors: specific force at the IMU in body axes, plus gyro rate
        s_, c_ = math.sin(th), math.cos(th)
        fx = xdd + r.imu_h * (c_ * thdd - s_ * thd ** 2)
        fy = -r.imu_h * (s_ * thdd + c_ * thd ** 2) + G
        acc_up = (fx * s_ + fy * c_) / G + rng.gauss(0, r.accel_noise_g)
        acc_fwd = -(fx * c_ - fy * s_) / G + rng.gauss(0, r.accel_noise_g)  # + when tilted forward
        rate = thd + math.radians(rng.gauss(0, r.gyro_noise_dps))
        # same complementary filter as the firmware; forward acceleration reads as a backward
        # tilt, so the firmware adds back its own last commanded acceleration
        acc_pitch = math.atan2(acc_fwd + r.accel_comp * a_cmd_prev / G, acc_up)
        pitch_est = alpha * (pitch_est + rate * DT) + (1 - alpha) * acc_pitch
        peak_err = max(peak_err, abs(pitch_est - th))

        err_v = vt - v
        integ = max(-2.0, min(2.0, integ + err_v * DT))
        lean_ff = g.kff * at / G  # lean into the target's acceleration
        th_ref = max(-THETA_REF_MAX, min(THETA_REF_MAX, g.kv * err_v + g.ki * integ + lean_ff))
        a_cmd = max(-r.a_max, min(r.a_max, g.kp * (pitch_est - th_ref) + g.kd * rate))
        a_cmd_prev = a_cmd
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
            xdd = a_eff + r.R * thdd  # axle (ground) acceleration
            # torque both motors put into the wheels, from the body's own equation of motion
            tau = r.M * G * r.L * s_ - r.M * r.L * c_ * xdd - (I_b + r.M * r.L ** 2) * thdd
            peak_tau = max(peak_tau, abs(tau) / 2)
            thd += thdd * h
            th += thd * h
            x += (v + r.R * thd) * h  # ground speed = relative wheel speed + body rotation
        peak = max(peak, abs(th))
        headroom = min(headroom, r.v_max - abs(v))
        if stats is not None:
            stats.update(steady_err=steady_err, headroom=headroom)
        if abs(th) > CUTOFF:
            return False, math.degrees(peak), None, x, math.degrees(peak_err), peak_tau
        if abs(th) < math.radians(1) and abs(thd) < math.radians(10):
            settled_at = settled_at if settled_at is not None else t
        else:
            settled_at = None
    return True, math.degrees(peak), settled_at, x, math.degrees(peak_err), peak_tau


DRIVE_SPEED = 0.4  # firmware DRIVE_SPEED_MAX, full stick

SCENARIOS = [
    ("start tilted 5 deg", dict(theta0_deg=5)),
    ("start tilted 10 deg", dict(theta0_deg=10)),
    ("shove 60 deg/s", dict(push_dps=60)),
    ("shove 120 deg/s", dict(push_dps=120)),
    ("drive forward, stop", dict(stick=stick_step(DRIVE_SPEED, hold=4.0), seconds=10.0)),
    ("drive back, stop", dict(stick=stick_step(-DRIVE_SPEED, hold=4.0), seconds=10.0)),
    ("shove while driving", dict(stick=stick_step(DRIVE_SPEED, hold=4.0), push_dps=90, push_at=3.0, seconds=10.0)),
]


def report(g: Gains, r: Robot) -> bool:
    ok_all = True
    for name, kw in SCENARIOS:
        st = {}
        ok, peak, settle, drift, err, tau = simulate(g, r, stats=st, **kw)
        ok_all &= ok and settle is not None
        status = f"settled in {settle:.2f} s" if ok and settle is not None else ("never settled" if ok else "FELL")
        moved = "moved" if "stick" in kw else "drift"
        lag = f"  speed lag {st['steady_err']:.2f} m/s" if "stick" in kw and not kw.get("push_dps") else ""
        print(f"  {name:20s} peak {peak:5.1f} deg  {status:18s} {moved} {drift:+.2f} m"
              f"  filter err {err:4.1f} deg  torque {tau:.3f} N*m/motor{lag}")
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
    ap.add_argument("--M", type=float, default=1.0, help="body mass incl. motors and battery, kg")
    args = ap.parse_args()
    robot = Robot(L=args.L, M=args.M, imu_h=args.L)
    if args.sweep:
        sweep(robot)
    else:
        print(f"default gains {Gains()} with L = {robot.L} m")
        report(Gains(), robot)
