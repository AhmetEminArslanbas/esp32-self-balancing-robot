"""Serve the robot's tuning page on the PC with simulated telemetry (no hardware needed).

/params comes from the firmware's PARAM_TABLE, /state replays the physics simulation
(push recovery, looping), /cmd answers like the robot would.
    python web_demo.py   ->  http://127.0.0.1:8080
"""
from __future__ import annotations

import json
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from sim_balance import DEFAULTS, ROBOTS, simulate

FIRMWARE = Path(__file__).resolve().parent.parent / "firmware"
PAGE = re.search(r'R"HTML\((.*)\)HTML"', (FIRMWARE / "WebPage.h").read_text(encoding="utf-8"), re.S).group(1)
ROW = re.compile(r'PARAM_ROW\((\w+),\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f,\s*"([^"]*)"\)')
PARAMS = [dict(n=n, lo=float(lo), hi=float(hi), v=float(v), d=d)
          for n, lo, hi, v, d in ROW.findall((FIRMWARE / "Params.ino").read_text(encoding="utf-8"))]

robot = ROBOTS[0]
LOG, _ = simulate(robot, dict(DEFAULTS, trim=robot.cog_offset_deg), seconds=12.0, push_at=4.0,
                  push_N=0.4 * robot.M / 0.15, push_s=0.15)
# Pre-fill the page's chart history so it opens already showing a push recovery.
_hist = [min(LOG, key=lambda r: abs(r[0] - (3.4 + i * 0.03)))[1:5] for i in range(250)]
PREFILL = ("<script>" + json.dumps([dict(a=a, x=x * 100, t=tr, u=u) for a, x, tr, u in _hist]) +
           ".forEach(s=>{push('a',s.a);push('t',s.t);push('u',s.u);push('x',s.x)});</script>")
PAGE = PAGE.replace("loadParams();poll();", "loadParams();poll();</script>" + PREFILL[:-9], 1)
POLL_S = 0.06  # the page polls /state every 60 ms
polls = 0


def state() -> dict:
    global polls
    polls += 1
    t = (polls * POLL_S) % LOG[-1][0]
    row = min(LOG, key=lambda r: abs(r[0] - t))
    _, tilt, x, trim, u, _x_est = row
    return dict(s="BALANCING", a=tilt, t=trim, u=u, x=x * 100, v=0.0, tr=trim, y=0.0,
                hz=500, dtmax=2140, falls=0, vb=7.62, net="AP DengeAraci 192.168.4.1 (simülasyon)")


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/params"):
            body, kind = json.dumps(PARAMS), "application/json"
        elif self.path.startswith("/state"):
            body, kind = json.dumps(state()), "application/json"
        elif self.path.startswith("/cmd"):
            body, kind = "ok (simülasyon)", "text/plain"
        else:
            body, kind = PAGE, "text/html"
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", kind + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    print("http://127.0.0.1:8080")
    ThreadingHTTPServer(("127.0.0.1", 8080), Handler).serve_forever()
