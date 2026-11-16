# Drawing of the skills simulation. Run from this folder:
#   python3 skills_sim.py 17 nocheck; python3 skills_sim.py 17 nocheck nocluster; python3 skills_sim_draw.py
# Needs: pip install matplotlib shapely
import json, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP
exec(open("skills_sim_field.py").read())
COL = {"1 red auton": "#c0392b", "2 cross": "#7f8c8d", "3 goal A": "#2471a3", "4 goal B": "#7d3c98", "5 toggle + park": "#1e8449"}

def draw(ax, run, title):
    ax.set_facecolor("#ecebe6")
    for n, g in obs.items():
        if n == "wall": continue
        if n in run["removed"]: 
            xs, ys = g.exterior.xy; ax.fill(xs, ys, fc="none", ec="#999", ls=":", lw=0.8); continue
        c = "#555"
        if "goal" in n: c = "#d4ac0d" if ("neutral" in n or n.startswith("goal")) else ("#c0392b" if "red" in n else ("#2e86c1" if "blue" in n else "#333"))
        if "tall" in n: c = "#333"
        if "cluster" in n: c = "#e67e22"
        if "stack" in n: c = "#b7950b"
        if "pin (" in n: c = "#c0392b" if "red" in n else "#2e86c1"
        if "toggle" in n: c = "#17202a"
        if "loader" in n: c = "#f1948a" if "red" in n else "#85c1e9"
        xs, ys = g.exterior.xy; ax.fill(xs, ys, fc=c, ec="k", lw=0.4, zorder=3)
    for k in range(-3, 4):
        ax.plot([k*T, k*T], [-WALL, WALL], c="#cfcdc4", lw=0.6, zorder=1); ax.plot([-WALL, WALL], [k*T, k*T], c="#cfcdc4", lw=0.6, zorder=1)
    ax.add_patch(plt.Rectangle((-WALL, -WALL), 2*WALL, 2*WALL, fill=False, lw=2, ec="k"))
    S = run["samples"]
    for i in range(1, len(S)):
        if S[i][4] != S[i-1][4]: continue
        ax.plot([S[i-1][1], S[i][1]], [S[i-1][2], S[i][2]], c=COL[S[i][4]], lw=1.6, zorder=4, alpha=0.9)
    W, L = run["W"], run["L"]
    def outline(x, y, h, c, a=0.25):
        pts = []
        for (dx, dy) in [(-W/2, -L/2), (W/2, -L/2), (W/2, L/2), (-W/2, L/2)]:
            r = math.radians(h); pts.append((x + dx*math.cos(r) + dy*math.sin(r), y - dx*math.sin(r) + dy*math.cos(r)))
        ax.add_patch(MP(pts, closed=True, fc=c, ec=c, alpha=a, lw=0.8, zorder=5))
        fx, fy = x + (L/2)*math.sin(math.radians(h)), y + (L/2)*math.cos(math.radians(h))
        ax.plot([x, fx], [y, fy], c="k", lw=0.8, zorder=6)
    ev_t = [e[0] for e in run["events"]]
    for et in ev_t:
        s = min(S, key=lambda q: abs(q[0] - et)); outline(s[1], s[2], s[3], COL[s[4]])
    outline(*S[0][1:4], "#000", 0.15)
    for (hx, hy, n) in run["hits"][::4]:
        ax.plot(hx, hy, "x", c="red", ms=5, mew=1.5, zorder=7)
    ax.set_xlim(-74, 74); ax.set_ylim(-74, 74); ax.set_aspect("equal"); ax.set_title(title, fontsize=11)
    ax.set_xticks([-71.4, -47.6, -23.8, 0, 23.8, 47.6, 71.4]); ax.set_yticks([-71.4, -47.6, -23.8, 0, 23.8, 47.6, 71.4])
    ax.tick_params(labelsize=7)

runs = [("run_17_off.json", "As coded, 17 in robot: %.0f s, 11 loads"),
        ("run_17_off_nc.json", "Same, if the cluster at (48,-48) were not there")]
fig, axs = plt.subplots(1, 2, figsize=(15, 7.8))
for ax, (f, title) in zip(axs, runs):
    r = json.load(open(f)); draw(ax, r, title % r["t"] if "%" in title else title)
from matplotlib.lines import Line2D
h = [Line2D([0], [0], c=c, lw=2, label="part " + k) for k, c in COL.items()]
h += [Line2D([0], [0], marker="x", c="red", ls="", label="robot hits something")]
fig.legend(handles=h, loc="lower center", ncol=6, fontsize=9, frameon=False)
fig.suptitle("839X skills (team plan): robot center path, robot outline at each scoring stop. Front line = toggle mech.", fontsize=12)
plt.tight_layout(rect=(0, 0.05, 1, 0.96))
plt.savefig("skills_sim.png", dpi=110)
print("ok")
