"""Physics check of the Denge_Araci_V2 controller (design tool, runs on the PC).

Planar wheeled inverted pendulum + linear DC motor (slow-decay PWM) + Coulomb friction,
IMU with noise, bias, a 2 ms delay and linear-acceleration error on the accel angle.
The Controller class is a line-by-line port of runBalance() in Control.ino; if the
firmware control law changes, change it here too. Robot models are guesses (TT and
N20 motors), so this checks signs and gain ranges, not the final tuning.

    python sim_balance.py          # default gains on 3 robot models, exit code 0 = all OK
"""
from __future__ import annotations
import math, random, sys
from dataclasses import dataclass

G = 9.81

@dataclass
class Robot:
    name: str
    M: float        # body mass kg
    L: float        # CoM height above axle m
    r: float        # wheel radius m
    mw: float       # one wheel mass kg
    tau_stall: float  # per motor N*m at 100 % duty
    w_max: float    # no-load wheel speed rad/s at 100 %
    friction: float # Coulomb friction as fraction of stall torque
    cog_offset_deg: float  # true balance angle (sensor frame) because of CoM / mounting
    sensor_h: float = 0.04

ROBOTS = [
    Robot("TT motor 65mm", M=0.40, L=0.06, r=0.0325, mw=0.03, tau_stall=0.10, w_max=26, friction=0.10, cog_offset_deg=3.0),
    Robot("N20 300rpm 43mm", M=0.25, L=0.05, r=0.0215, mw=0.01, tau_stall=0.05, w_max=31, friction=0.08, cog_offset_deg=3.0),
    Robot("TT uzun govde", M=0.50, L=0.10, r=0.0325, mw=0.03, tau_stall=0.10, w_max=26, friction=0.12, cog_offset_deg=-2.0),
]

# Keep in sync with PARAM_TABLE in Params.ino.
DEFAULTS = dict(kp=6.0, ki=100.0, kd=0.4, kv=0.1, kx=0.1, kxi=0.005, max_lean=6.0, trim=3.0,
                vel_tau=0.1, deadband=12.0, max_out=100.0, tau=0.6, d_lpf=25.0,
                fall_angle=40.0)
TRIM_LIMIT, TRIM_LEARN_X_CLAMP, RAMP = 15.0, 20.0, 2.0


def drive_fraction(mag, deadband):
    mag = min(mag, 100.0)
    db = max(0.0, min(deadband, 60.0))
    span = (100 - db) / 100
    if mag < RAMP:
        return (mag / RAMP) * (db + RAMP * span) / 100
    return (db + mag * span) / 100


class Controller:
    """Port of runBalance() + complementary filter."""
    def __init__(self, p):
        self.p = dict(p)
        self.angle = None
        self.v = self.x = self.last_u = 0.0
        self.rate_f = 0.0
        self.target = 0.0
        self.i_term = 0.0

    def step(self, acc_angle, rate, dt):
        p = self.p
        if self.angle is None:
            self.angle = acc_angle
            self.rate_f = rate
        else:
            k = p["tau"] / (p["tau"] + dt)
            self.angle = k * (self.angle + rate * dt) + (1 - k) * acc_angle
        tilt = self.angle - p["trim"]
        if abs(tilt) > p["fall_angle"]:
            return None
        vel_tau = max(p["vel_tau"], dt)
        self.v += (self.last_u - self.v) * dt / vel_tau
        self.x += self.v * dt
        lean = p["max_lean"]
        pos = max(-0.7 * lean, min(0.7 * lean, p["kx"] * self.x))
        target = p["trim"] - pos - p["kv"] * self.v
        self.target = max(p["trim"] - lean, min(p["trim"] + lean, target))
        xl = max(-TRIM_LEARN_X_CLAMP, min(TRIM_LEARN_X_CLAMP, self.x))
        p["trim"] = max(-TRIM_LIMIT, min(TRIM_LIMIT, p["trim"] - p["kxi"] * xl * dt))
        if p["d_lpf"] > 0:
            rc = 1 / (2 * math.pi * p["d_lpf"])
            self.rate_f += (rate - self.rate_f) * dt / (rc + dt)
        else:
            self.rate_f = rate
        err = self.angle - self.target
        self.i_term = max(-p["max_out"], min(p["max_out"], self.i_term + p["ki"] * err * dt))
        u = p["kp"] * err + self.i_term + p["kd"] * self.rate_f
        u = max(-p["max_out"], min(p["max_out"], u))
        self.last_u = u
        return u


def simulate(robot: Robot, params: dict, seconds=25.0, push_at=8.0, push_N=1.5, push_s=0.15,
             start_tilt=1.0, seed=1, flip=None):
    rnd = random.Random(seed)
    ctrl = Controller(params)
    if flip:
        flip(ctrl)
    Mw = 2 * (robot.mw + 0.5 * robot.mw * robot.r ** 2 / robot.r ** 2)  # m + I/r^2, disc wheels
    Ib = robot.M * robot.L ** 2 / 3
    # state: x, xd, th (rad, + = lean toward +x), thd
    th = math.radians(start_tilt)  # physical tilt from vertical; the sensor reads th + cog_offset
    x = xd = thd = 0.0
    dt_sim, dt_ctrl = 0.0002, 0.002
    steps_per = int(dt_ctrl / dt_sim)
    u = 0.0
    delay = [0.0] * int(0.003 / dt_ctrl)  # DLPF latency on gyro rate
    bias = rnd.uniform(-0.3, 0.3)
    xdd_prev = thdd_prev = 0.0
    log = []
    fell_at = None
    t = 0.0
    n = int(seconds / dt_sim)
    for i in range(n):
        if i % steps_per == 0:
            # sensors (physical balance point sits at sensor angle = cog_offset)
            a_fwd = xdd_prev + robot.sensor_h * thdd_prev * math.cos(th)
            acc_angle = math.degrees(th - math.atan2(a_fwd, G)) + robot.cog_offset_deg + rnd.gauss(0, 0.4)
            delay.append(math.degrees(thd) + rnd.gauss(0, 0.08) + bias)
            rate = delay.pop(0)
            out = ctrl.step(acc_angle, rate, dt_ctrl + rnd.uniform(-5e-5, 5e-5))
            if out is None:
                fell_at = t
                break
            u = out
            log.append((t, math.degrees(th), x, ctrl.p["trim"], u, ctrl.x))
        # motors (both wheels identical here)
        duty = math.copysign(drive_fraction(abs(u), params["deadband"]), u)
        w_rel = xd / robot.r - thd
        tau_m = robot.tau_stall * (duty - w_rel / robot.w_max)
        tau_f = robot.friction * robot.tau_stall * math.tanh(w_rel / 0.3)
        tau = 2 * (tau_m - tau_f)
        F_push = push_N if push_at <= t < push_at + push_s else 0.0
        # (M+Mw) xdd + M L cos th thdd = tau/r + F + M L sin th thd^2
        # M L cos th xdd + (Ib + M L^2) thdd = M g L sin th - tau + F*L*cos th
        a11, a12 = robot.M + Mw, robot.M * robot.L * math.cos(th)
        a21, a22 = robot.M * robot.L * math.cos(th), Ib + robot.M * robot.L ** 2
        b1 = tau / robot.r + F_push + robot.M * robot.L * math.sin(th) * thd ** 2
        b2 = robot.M * G * robot.L * math.sin(th) - tau + F_push * robot.L * math.cos(th)
        det = a11 * a22 - a12 * a21
        xdd = (b1 * a22 - a12 * b2) / det
        thdd = (a11 * b2 - a21 * b1) / det
        xd += xdd * dt_sim
        x += xd * dt_sim
        thd += thdd * dt_sim
        th += thd * dt_sim
        xdd_prev, thdd_prev = xdd, thdd
        t += dt_sim
        if abs(th) > math.radians(60):
            fell_at = t
            break
    return log, fell_at


def summarize(robot, params, label, **kw):
    log, fell = simulate(robot, params, **kw)
    if fell is not None:
        return dict(label=label, ok=False, msg=f"DUSTU t={fell:.2f}s")
    push_at = kw.get("push_at", 8.0)
    pre = [r for r in log if 5.0 <= r[0] < push_at]
    post = [r for r in log if r[0] >= push_at]
    end = [r for r in log if r[0] >= log[-1][0] - 3.0]
    ang_rms = math.sqrt(sum((r[1]) ** 2 for r in pre) / len(pre)) if pre else float("nan")
    max_disp = max(abs(r[2]) for r in post) if post else 0
    end_x = sum(r[2] for r in end) / len(end)
    end_trim = end[-1][3]
    ok = abs(end_x) < 0.04
    return dict(label=label, ok=ok, msg=(f"egim RMS {ang_rms:.2f} deg | itmede max yer degistirme {max_disp*100:.1f} cm"
                                          f" | sonda konum {end_x*100:+.1f} cm | trim -> {end_trim:+.2f}"))


def main():
    all_ok = True
    for rb in ROBOTS:
        print(f"\n=== {rb.name} (gercek denge acisi {rb.cog_offset_deg:+.1f} deg) ===")
        # Pushes are scaled to the body mass: dv = velocity the shove gives the robot.
        push = lambda dv: dict(push_N=dv * rb.M / 0.15, push_s=0.15)
        # Push scenarios start from a correct trim (as after trimcal / learning). The
        # wrong-trim case only has to survive: returning home then takes longer than 25 s.
        for label, kw, must_return in [
            ("itme 0.25 m/s", push(0.25), True),
            ("guclu itme 0.4 m/s", push(0.4), True),
            ("trim 3 derece yanlis + itme 0.2 m/s", dict(push(0.2), trim_error=3.0), False),
        ]:
            p = dict(DEFAULTS)
            p["trim"] = rb.cog_offset_deg + kw.pop("trim_error", 0.0)
            res = summarize(rb, p, label, **kw)
            ok = res["ok"] if must_return else not res["msg"].startswith("DUSTU")
            all_ok &= ok
            print(f"  [{'OK ' if ok else 'HATA'}] {label}: {res['msg']}")
    return all_ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)

