"""Render the balance + drive demo as a GIF and a time plot.

Runs the physics model in sim_balance.py (same control law as the firmware) through one
scenario: hold the start point, recover from a shove, drive to a target with the remote
control, turn 180 degrees on the spot, drive back and stop where it started.
Driving moves the outer loop's position/speed setpoint; turning moves the heading-hold
target. The heading plant is a first-order wheel-speed model (the pitch model is planar).
    python animate.py            # writes push_recovery.gif and push_recovery.png
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, Polygon, Rectangle
from matplotlib.transforms import Affine2D

from sim_balance import DEFAULTS, ROBOTS, drive_fraction, simulate

robot = ROBOTS[0]
params = dict(DEFAULTS, trim=robot.cog_offset_deg)
TRACK = 0.15                      # wheel spacing [m]
PUSH_AT, PUSH_DV = 2.5, 0.4
DRIVE_M = 0.40                    # remote drive distance [m]
SECONDS = 23.0

# (start, end, kind, label) — kinds: hold / drive / turn
PHASES = [
    (0.0, 2.5, "hold", "Konum tutma"),
    (2.5, 7.0, "push", "İtme → toparlanma, yerine dönüş"),
    (7.0, 10.0, "drive", "Kumanda: ileri"),
    (10.0, 11.5, "hold", "Hedefte bekleme"),
    (11.5, 14.0, "turn", "Kumanda: yerinde 180° dönüş"),
    (14.0, 17.0, "drive", "Kumanda: başlangıca dönüş"),
    (17.0, 18.5, "hold", "Konum tutma"),
    (18.5, 21.0, "turn", "Kumanda: 180° dönüş"),
    (21.0, SECONDS, "hold", "Başladığı yerde, aynı yönde"),
]


def smooth(a: float) -> float:
    a = min(max(a, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * a)


def smooth_rate(a: float, length: float) -> float:
    return 0.5 * math.pi * math.sin(math.pi * a) / length if 0 < a < 1 else 0.0


def body_path(t: float) -> tuple[float, float]:
    """Distance along the heading [m] and its rate: two forward legs of DRIVE_M."""
    s = v = 0.0
    for start, end in [(7.0, 10.0), (14.0, 17.0)]:
        a = (t - start) / (end - start)
        s += DRIVE_M * smooth(a)
        v += DRIVE_M * smooth_rate(a, end - start)
    return s, v


def heading_target(t: float) -> float:
    return sum(180.0 * smooth((t - a) / (b - a)) for a, b in [(11.5, 14.0), (18.5, 21.0)])


# The controller estimates speed from the motor command (%), not in metres: calibrate the
# scale once with a slow constant-speed run so the setpoint lands where we want.
def calibrate() -> float:
    pct = 20.0
    log, _ = simulate(robot, dict(params), seconds=8.0, push_at=99, push_N=0,
                      ref=lambda t: (pct * max(t - 2, 0), pct if t > 2 else 0.0))
    return (log[-1][2] - log[int(len(log) * 0.5)][2]) / (pct * (log[-1][0] - log[int(len(log) * 0.5)][0]))


M_PER_UNIT = calibrate()
log, fell = simulate(robot, params, seconds=SECONDS, push_at=PUSH_AT,
                     push_N=PUSH_DV * robot.M / 0.15, push_s=0.15,
                     ref=lambda t: tuple(q / M_PER_UNIT for q in body_path(t)))
assert fell is None, "robot fell in simulation"

t = [r[0] for r in log]
dt = t[1] - t[0]
tilt = [r[1] for r in log]
s_body = [r[2] for r in log]
u = [r[4] for r in log]

# Heading: firmware heading-hold law driving a first-order differential wheel-speed model.
yaw, yaw_rate = [0.0], 0.0
for i in range(1, len(t)):
    err = heading_target(t[i]) - yaw[-1]
    cmd = params.get("kyaw", 0.4) * err - params.get("kyaw_d", 0.03) * yaw_rate
    cmd = max(-30.0, min(30.0, cmd))
    duty = lambda c: math.copysign(drive_fraction(abs(c), params["deadband"]), c)
    wheel_diff = (duty(u[i] + cmd) - duty(u[i] - cmd)) * robot.w_max * robot.r
    yaw_rate += (math.degrees(wheel_diff / TRACK) - yaw_rate) * dt / 0.08
    yaw.append(yaw[-1] + yaw_rate * dt)

# World position from distance along the heading.
wx, wy = [0.0], [0.0]
for i in range(1, len(t)):
    ds = s_body[i] - s_body[i - 1]
    wx.append(wx[-1] + ds * math.cos(math.radians(yaw[i])))
    wy.append(wy[-1] + ds * math.sin(math.radians(yaw[i])))

BG, PANEL, FG, MUTED, GRID = "#0b1220", "#111a2e", "#e2e8f0", "#94a3b8", "#1e293b"
ACCENT, GREEN, WARN, RED = "#3b82f6", "#22c55e", "#f59e0b", "#ef4444"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED,
                     "axes.labelcolor": FG, "xtick.color": MUTED, "ytick.color": MUTED,
                     "text.color": FG, "font.size": 10})


def shade_phases(ax):
    for start, end, kind, _ in PHASES:
        if kind in ("drive", "turn"):
            ax.axvspan(start, end, color=ACCENT, alpha=0.08, lw=0)
    ax.axvline(PUSH_AT, color=WARN, ls="--", lw=0.8)


# --- time plot ---------------------------------------------------------------------------
fig, axes = plt.subplots(4, 1, figsize=(8, 7.2), sharex=True)
rows = [(tilt, "Eğim (°)", ACCENT), ([x * 100 for x in wx], "Konum (cm)", GREEN),
        (yaw, "Yön (°)", "#a78bfa"), (u, "Motor çıkışı (%)", WARN)]
for ax, (data, label, color) in zip(axes, rows):
    shade_phases(ax)
    ax.plot(t, data, color=color, lw=1.4)
    ax.set_ylabel(label)
    ax.grid(color=GRID, lw=0.6)
axes[1].plot(t, [body_path(tt)[0] * 100 * math.cos(math.radians(heading_target(tt)))
                 if tt < 14 else (2 * DRIVE_M - body_path(tt)[0]) * 100 for tt in t],
             color=MUTED, lw=1, ls=":", label="kumanda hedefi")
axes[1].legend(loc="upper right", frameon=False, fontsize=9)
axes[0].set_title("Denge + kumanda: itmeye toparlanma, ileri sürüş, 180° dönüş, başlangıca dönüş",
                  loc="left", fontsize=10.5)
axes[-1].set_xlabel("Zaman (s)   (mavi alanlar: kumanda komutu, turuncu çizgi: itme)")
fig.tight_layout()
fig.savefig("push_recovery.png", dpi=120)

# --- animation ---------------------------------------------------------------------------
CM = 100
R = robot.r * CM                        # wheel radius
H, W, D = 17.0, 5.0, 11.0               # body height, length (side), depth (across) [cm]
HALF_TRACK = TRACK * CM / 2

fig2 = plt.figure(figsize=(9.6, 5.4), dpi=80)  # 16:9, same as the other showcase images
side = fig2.add_axes([0.01, 0.06, 0.66, 0.78])
top = fig2.add_axes([0.70, 0.36, 0.28, 0.48])
pad = fig2.add_axes([0.765, 0.04, 0.15, 0.27])

side.set_xlim(-24, 66)
side.set_ylim(-6, 30)
side.set_aspect("equal")
side.axis("off")
side.add_patch(Rectangle((-30, -8), 110, 8, color=PANEL, zorder=0))
side.plot([-30, 80], [0, 0], color=MUTED, lw=1.2, zorder=1)
for xc in range(-20, 70, 10):
    side.plot([xc, xc], [0, -1.2], color=MUTED, lw=0.8)
    side.text(xc, -3.4, f"{xc}", ha="center", fontsize=7, color=MUTED)
side.text(66, -3.4, "cm", ha="right", fontsize=7, color=MUTED)
side.add_patch(Polygon([[0, 0], [0, 7], [3.2, 6], [0, 5]], closed=True, color=GREEN, zorder=1))
side.text(-4.5, 3.5, "başlangıç", ha="right", fontsize=8, color=GREEN)
goal_flag = side.add_patch(Polygon([[40, 0], [40, 7], [43.2, 6], [40, 5]], closed=True,
                                   color=ACCENT, zorder=1, alpha=0))
goal_txt = side.text(48.5, 3.5, "hedef", ha="left", fontsize=8, color=ACCENT, alpha=0)

far_wheel = Ellipse((0, R), 2 * R, 2 * R, color="#334155", zorder=2)
body = FancyBboxPatch((-W / 2, 0), W, H, boxstyle="round,pad=0,rounding_size=1.2",
                      fc="#1e3a5f", ec="#60a5fa", lw=1.5, zorder=3)
battery = Rectangle((-W / 2 + 0.6, 1.5), W - 1.2, 4.2, fc="#334155", ec="#64748b", lw=1, zorder=4)
board = Rectangle((-W / 2 + 0.6, 7.2), W - 1.2, 5.0, fc="#14532d", ec="#22c55e", lw=1, zorder=4)
chip = Rectangle((-0.9, 8.6), 1.8, 1.8, fc="#0f172a", zorder=5)
imu = Rectangle((-0.6, 13.3), 1.2, 1.0, fc="#a78bfa", zorder=5)
led = Circle((W / 2, H - 2.0), 0.55, color=RED, zorder=6)
near_tire = Ellipse((0, R), 2 * R, 2 * R, fc="#0f172a", ec="#cbd5e1", lw=2.2, zorder=7)
hub = Ellipse((0, R), 0.9 * R, 0.9 * R, fc="#475569", ec="#94a3b8", lw=1, zorder=8)
for p in (far_wheel, body, battery, board, chip, imu, led, near_tire, hub):
    side.add_patch(p)
spokes = [side.plot([], [], color="#cbd5e1", lw=1.4, zorder=9)[0] for _ in range(3)]
push_arrow = side.annotate("", xy=(0, 0), xytext=(0, 0))

phase_txt = fig2.text(0.015, 0.94, "", fontsize=13, weight="bold", va="center")
stat_txt = fig2.text(0.015, 0.875, "", fontsize=9.5, color=MUTED, va="center", family="monospace")
fig2.text(0.70, 0.94, "Üstten görünüş", fontsize=10, color=MUTED, va="center")

top.set_xlim(-14, 54)
top.set_ylim(-16, 16)
top.set_aspect("equal")
top.set_facecolor(PANEL)
top.set_xticks([])
top.set_yticks([])
for sp in top.spines.values():
    sp.set_color(GRID)
top.plot(0, 0, "o", color=GREEN, ms=6)
trail = top.plot([], [], color=ACCENT, lw=1.5, alpha=0.7)[0]
top_body = Rectangle((-W / 2 - 1, -HALF_TRACK + 1.5), W + 2, 2 * HALF_TRACK - 3,
                     fc="#1e3a5f", ec="#60a5fa", lw=1.2, zorder=3)
top_wl = Rectangle((-R, HALF_TRACK - 1.2), 2 * R, 2.4, fc="#0f172a", ec="#cbd5e1", zorder=4)
top_wr = Rectangle((-R, -HALF_TRACK - 1.2), 2 * R, 2.4, fc="#0f172a", ec="#cbd5e1", zorder=4)
top_nose = Polygon([[W / 2 + 1, -2.5], [W / 2 + 5, 0], [W / 2 + 1, 2.5]], color=RED, zorder=5)
for p in (top_body, top_wl, top_wr, top_nose):
    top.add_patch(p)

pad.set_xlim(-1.3, 1.3)
pad.set_ylim(-1.3, 1.3)
pad.set_aspect("equal")
pad.axis("off")
pad.add_patch(Circle((0, 0), 1.0, fc=PANEL, ec=MUTED, lw=1))
pad.plot([-1, 1], [0, 0], color=GRID, lw=0.8)
pad.plot([0, 0], [-1, 1], color=GRID, lw=0.8)
knob = Circle((0, 0), 0.28, color=ACCENT, zorder=3)
pad.add_patch(knob)
pad.text(0, -1.28, "kumanda", ha="center", va="top", fontsize=8, color=MUTED)


def phase_at(tt):
    return next(p for p in PHASES if p[0] <= tt < p[1] or p is PHASES[-1])


def draw(i):
    global push_arrow
    tt = t[i]
    th = math.radians(tilt[i])
    psi = math.radians(yaw[i])
    c, sn = math.cos(psi), math.sin(psi)
    x = wx[i] * CM

    # Side view: the robot is a box on two wheels seen from the side; when it turns, the
    # wheels separate on screen and the body's side face narrows (simple projection).
    lean_x = math.sin(th) * c                       # forward lean projected on the screen
    axle = (x, R)
    rot = Affine2D().scale(abs(c) * 1.0 + abs(sn) * D / W, 1).rotate(-math.asin(max(-1, min(1, lean_x)))) \
        .translate(*axle) + side.transData
    face = 1 if c >= 0 else -1
    body.set_transform(rot)
    for p in (battery, board, chip, imu):
        p.set_transform(rot)
    led.set_center((face * W / 2 * abs(c), H - 2.0))
    led.set_transform(Affine2D().rotate(-math.asin(max(-1, min(1, lean_x)))).translate(*axle)
                      + side.transData)
    led.set_alpha(max(abs(c), 0.15))
    near_dx, far_dx = HALF_TRACK * sn, -HALF_TRACK * sn
    for e, dx in ((far_wheel, far_dx), (near_tire, near_dx), (hub, near_dx)):
        e.set_center((x + dx, R))
    near_tire.width = 2 * R * max(abs(c), 0.12)
    hub.width = 0.9 * R * max(abs(c), 0.12)
    far_wheel.width = 2 * R * max(abs(c), 0.12)
    roll = s_body[i] / robot.r
    for k, sp in enumerate(spokes):
        a = roll + k * math.pi / 3
        sp.set_data([x + near_dx - R * 0.9 * math.cos(a) * abs(c), x + near_dx + R * 0.9 * math.cos(a) * abs(c)],
                    [R - R * 0.9 * math.sin(a), R + R * 0.9 * math.sin(a)])

    kind = phase_at(tt)[2]
    in_push = PUSH_AT <= tt < PUSH_AT + 0.5
    push_arrow.remove()
    push_arrow = side.annotate("itme", xy=(x - 4, R + H * 0.7), xytext=(x - 17, R + H * 0.7),
                               color=WARN, fontsize=9, va="center",
                               arrowprops=dict(arrowstyle="-|>", color=WARN, lw=2.2))
    push_arrow.set_visible(in_push)
    goal_on = 6.5 <= tt < 14.0
    goal_flag.set_alpha(1 if goal_on else 0)
    goal_txt.set_alpha(1 if goal_on else 0)

    # Top view.
    trail.set_data([v * CM for v in wx[: i + 1 : 5]], [v * CM for v in wy[: i + 1 : 5]])
    tr = Affine2D().rotate(psi).translate(x, wy[i] * CM) + top.transData
    for p in (top_body, top_wl, top_wr, top_nose):
        p.set_transform(tr)

    # Remote: forward stick while driving, sideways while turning.
    _, v_ref = body_path(tt)
    yaw_ref_rate = (heading_target(tt + 0.05) - heading_target(tt)) / 0.05
    knob.set_center((max(-1, min(1, -yaw_ref_rate / 110)) * 0.7, max(-1, min(1, v_ref / 0.25)) * 0.7))
    knob.set_color(ACCENT if kind in ("drive", "turn") else MUTED)

    color = {"push": WARN, "drive": ACCENT, "turn": "#a78bfa"}.get(kind, GREEN)
    phase_txt.set_text(phase_at(tt)[3])
    phase_txt.set_color(color)
    stat_txt.set_text(f"t={tt:4.1f} s  eğim={tilt[i]:+5.1f}°  konum={x:+5.1f} cm  yön={yaw[i] % 360:5.1f}°")
    return []


step = max(1, int(round(0.05 / dt)))
anim = FuncAnimation(fig2, draw, frames=range(0, len(log), step), blit=False)
anim.save("push_recovery.gif", writer=PillowWriter(fps=20))
print(f"push_recovery.gif / push_recovery.png yazıldı (son konum {wx[-1] * 100:+.1f} cm, yön {yaw[-1]:.1f}°)")
