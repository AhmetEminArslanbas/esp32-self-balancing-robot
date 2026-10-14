"""Render the balance controller's push-recovery as a GIF and a time plot.

Runs the physics model in sim_balance.py (same control law as the firmware), shoves the
robot and draws what happens.
    python animate.py            # writes push_recovery.gif and push_recovery.png
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from sim_balance import DEFAULTS, ROBOTS, simulate

robot = ROBOTS[0]
params = dict(DEFAULTS, trim=robot.cog_offset_deg)
push_at, dv = 3.0, 0.4
log, fell = simulate(robot, params, seconds=12.0, push_at=push_at,
                     push_N=dv * robot.M / 0.15, push_s=0.15)
assert fell is None, "robot fell in simulation"

t = [r[0] for r in log]
tilt = [r[1] for r in log]
x_cm = [r[2] * 100 for r in log]
u = [r[4] for r in log]

BG, FG, MUTED, ACCENT, WARN = "#0b1220", "#e2e8f0", "#94a3b8", "#3b82f6", "#f59e0b"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED,
                     "axes.labelcolor": FG, "xtick.color": MUTED, "ytick.color": MUTED,
                     "text.color": FG, "font.size": 10})

fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
for ax, data, label, color in [(axes[0], tilt, "Eğim (°)", ACCENT),
                               (axes[1], x_cm, "Konum (cm)", "#22c55e"),
                               (axes[2], u, "Motor çıkışı (%)", WARN)]:
    ax.plot(t, data, color=color, lw=1.4)
    ax.axvline(push_at, color=MUTED, ls="--", lw=0.8)
    ax.set_ylabel(label)
    ax.grid(color="#1e293b", lw=0.6)
axes[0].set_title(f"İtme testi: {dv} m/s darbe → toparlanma ve başlangıç noktasına dönüş", loc="left")
axes[-1].set_xlabel("Zaman (s)")
fig.tight_layout()
fig.savefig("push_recovery.png", dpi=120)

# Side-view animation: wheel + body, decimated to ~25 fps.
step = max(1, int(0.04 / (t[1] - t[0])))
frames = range(0, len(log), step)
fig2, ax2 = plt.subplots(figsize=(6.4, 3.6))
ax2.set_xlim(-30, 30)
ax2.set_ylim(-2, 22)
ax2.set_aspect("equal")
ax2.axis("off")
ax2.axhline(0, color=MUTED, lw=1)
scale = 100  # metres -> cm
wheel_r = robot.r * scale
body_len = robot.L * 2 * scale
wheel = plt.Circle((0, wheel_r), wheel_r, color=MUTED, fill=False, lw=2)
ax2.add_patch(wheel)
(body,) = ax2.plot([], [], color=ACCENT, lw=8, solid_capstyle="round")
(home,) = ax2.plot([0, 0], [-1.5, 0], color="#22c55e", lw=2)
label = ax2.text(-29, 20, "", fontsize=10, va="top")
arrow = ax2.annotate("", xy=(0, 0), xytext=(0, 0))


def draw(i):
    global arrow
    tt, th_deg, x = log[i][0], log[i][1], log[i][2] * scale
    th = math.radians(th_deg - robot.cog_offset_deg)
    wheel.center = (x, wheel_r)
    body.set_data([x, x + body_len * math.sin(th)], [wheel_r, wheel_r + body_len * math.cos(th)])
    pushing = push_at <= tt < push_at + 0.4
    arrow.remove()
    arrow = ax2.annotate("itme", xy=(x - 2, wheel_r + body_len * 0.6), xytext=(x - 14, wheel_r + body_len * 0.6),
                         color=WARN, arrowprops=dict(arrowstyle="->", color=WARN, lw=2), alpha=1 if pushing else 0)
    label.set_text(f"t = {tt:4.1f} s   eğim = {th_deg - robot.cog_offset_deg:+5.1f}°   konum = {x:+5.1f} cm")
    return body, wheel, label


anim = FuncAnimation(fig2, draw, frames=frames, blit=False)
anim.save("push_recovery.gif", writer=PillowWriter(fps=25))
print("push_recovery.gif / push_recovery.png yazıldı")
