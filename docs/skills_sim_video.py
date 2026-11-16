# Video of the skills simulation (needs ffmpeg). Run from this folder after:
#   python3 skills_sim.py 17 nocheck

import json, math, bisect
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP
from matplotlib.animation import FuncAnimation, FFMpegWriter
from shapely.geometry import box
from shapely import affinity
exec(open("skills_sim_field.py").read())
exec(open("skills_sim_draw.py").read().split("def draw")[0].split("exec(")[0])  # imports only
COL = {"1 red auton": "#c0392b", "2 cross": "#7f8c8d", "3 goal A": "#2471a3", "4 goal B": "#7d3c98", "5 toggle + park": "#1e8449"}
run = json.load(open("run_17_off.json"))
S = run["samples"]; W, L = run["W"], run["L"]; legs = run["legs"]; events = run["events"]
ts = [s[0] for s in S]
gone_at = {n: next(e[0] for e in events if e[1].startswith(n.split()[0])) for n in ("UPPER stack", "LOWER stack")}

def pose(t):
    i = min(max(bisect.bisect_left(ts, t), 1), len(S) - 1)
    a, b = S[i - 1], S[i]
    f = 0 if b[0] == a[0] else max(0, min(1, (t - a[0]) / (b[0] - a[0])))
    dh = ((b[3] - a[3] + 180) % 360) - 180
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), (a[3] + f * dh) % 360, b[4], i

def leg_at(t):
    for lg in legs:
        if lg[2] <= t <= lg[3] + 1e-6: return lg
    return None

def rpoly(x, y, h):
    r = affinity.rotate(box(-W / 2, -L / 2, W / 2, L / 2), -h, origin=(0, 0))
    return affinity.translate(r, x, y)

fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
ax = fig.add_axes([0.02, 0.04, 0.56, 0.92]); side = fig.add_axes([0.6, 0.04, 0.38, 0.92]); side.axis("off")
SPEED = 3.0; FPS = 15
nframes = int(run["t"] / SPEED * FPS) + FPS * 2

static_obs = {}
def background(t):
    ax.clear(); ax.set_facecolor("#ecebe6")
    for k in range(-3, 4):
        ax.plot([k*T, k*T], [-WALL, WALL], c="#cfcdc4", lw=0.6, zorder=1); ax.plot([-WALL, WALL], [k*T, k*T], c="#cfcdc4", lw=0.6, zorder=1)
    for n, g in obs.items():
        if n == "wall": continue
        if n in gone_at and t >= gone_at[n]: continue
        c = "#555"
        if "goal" in n: c = "#d4ac0d" if ("neutral" in n or n.startswith("goal")) else ("#c0392b" if "red" in n else "#2e86c1")
        if "tall" in n: c = "#333"
        if "cluster" in n: c = "#e67e22"
        if "stack" in n: c = "#b7950b"
        if "pin (" in n: c = "#c0392b" if "red" in n else "#2e86c1"
        if "toggle" in n: c = "#17202a"
        if "loader" in n: c = "#f1948a" if "red" in n else "#85c1e9"
        xs, ys = g.exterior.xy; ax.fill(xs, ys, fc=c, ec="k", lw=0.4, zorder=3)
    for n, (gx, gy) in {"A": (T, -2*T), "B": (2*T, -T)}.items():
        ax.text(gx, gy, n, ha="center", va="center", fontsize=8, weight="bold", zorder=4)
    ax.add_patch(plt.Rectangle((-WALL, -WALL), 2*WALL, 2*WALL, fill=False, lw=2, ec="k"))
    ax.set_xlim(-74, 74); ax.set_ylim(-74, 74); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])

def frame(k):
    t = min(k / FPS * SPEED, run["t"])
    background(t)
    x, y, h, part, i = pose(t)
    for j in range(1, i):
        if S[j][4] == S[j-1][4]:
            pass
    xs = [s[1] for s in S[:i]]; ys = [s[2] for s in S[:i]]
    ax.plot(xs + [x], ys + [y], c="#555", lw=1.0, alpha=0.6, zorder=4)
    lg = leg_at(t); allowed = set(lg[6]) if lg else set()
    rp = rpoly(x, y, h); hit = None
    for n, g in obs.items():
        if n in allowed or (n in gone_at and t >= gone_at[n]): continue
        if rp.distance(g) == 0: hit = n; break
    c = "red" if hit else COL[part]
    px, py = rp.exterior.xy
    ax.fill(px, py, fc=c, ec="k", alpha=0.55, lw=1.2, zorder=6)
    fx, fy = x + (L/2)*math.sin(math.radians(h)), y + (L/2)*math.cos(math.radians(h))
    ax.plot([x, fx], [y, fy], c="k", lw=2, zorder=7)
    bx, by = x - 10*math.sin(math.radians(h)), y - 10*math.cos(math.radians(h))
    ax.plot(bx, by, "o", c="white", mec="k", ms=5, zorder=7)
    # side panel
    side.clear(); side.axis("off"); side.set_xlim(0, 1); side.set_ylim(0, 1)
    side.text(0, 0.97, "839X SKILLS  (team plan, simulated)", fontsize=13, weight="bold", va="top")
    clock = "red" if t > 60 else "k"
    side.text(0, 0.90, f"{t:5.1f} s", fontsize=26, weight="bold", va="top", color=clock, family="monospace")
    side.text(0.42, 0.885, "past 60 s!" if t > 60 else "", fontsize=12, color="red", va="top", weight="bold")
    side.text(0, 0.80, f"Part {part}", fontsize=12, color=COL[part], weight="bold", va="top")
    side.text(0, 0.75, (lg[1] if lg else "mechanism"), fontsize=11, va="top")
    if hit: side.text(0, 0.70, f"HITS: {hit}", fontsize=12, color="red", weight="bold", va="top")
    yy = 0.62
    for (et, name) in events:
        if et <= t:
            side.text(0, yy, f"{et:5.1f}s  {name}", fontsize=9.5, va="top", family="monospace", color="red" if et > 60 else "k"); yy -= 0.035
    side.text(0, 0.06, "17 x 17 in robot, 60% speed. Robot size, field element sizes\nand speeds are GUESSES. Black line = front (toggle mech),\nwhite dot = grabber. Shown at 3x speed.", fontsize=8.5, color="#555", va="bottom")
    return []

anim = FuncAnimation(fig, frame, frames=nframes)
anim.save("skills_sim.mp4", writer=FFMpegWriter(fps=FPS, bitrate=1800))
print("done", nframes)
