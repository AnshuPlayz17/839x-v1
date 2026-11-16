import math, sys
from shapely.geometry import Point, Polygon, box
T=23.8
WALL=3*T
LOADER=(60,-64)
UPPER=(-T,-T)
LOWER=(-2*T,-2*T)
# ---------------- field elements (sizes are GUESSES) ----------------
obs = {}
def circ(name, x, y, r): obs[name] = Point(x, y).buffer(r, 24)
circ("tall goal", 0, 0, 4)
for n, (x, y) in {"red goal 3": (-T, -2*T), "red goal 2": (-2*T, -T), "blue goal": (2*T, T), "blue goal ": (T, 2*T),
                  "goal A": (T, -2*T), "goal B": (2*T, -T), "neutral goal": (-T, 2*T), "neutral goal ": (-2*T, T)}.items():
    circ(n, x, y, 3.5)
for n, (x, y) in {"red pin (-24,0)": (-T, 0), "red pin (0,-24)": (0, -T), "blue pin (0,24)": (0, T), "blue pin (24,0)": (T, 0)}.items():
    circ(n, x, y, 1.5)
for (x, y) in [(-2*T, 2*T), (-T, T), (T, -T), (2*T, -2*T)]:
    obs[f"pin cluster ({x:.0f},{y:.0f})"] = box(x - 7.5, y - 1.25, x + 7.5, y + 1.25).union(box(x - 1.25, y - 7.5, x + 1.25, y + 7.5))
circ("UPPER stack", *UPPER, 2.5); circ("LOWER stack", *LOWER, 2.5)
circ("field stack (24,24)", T, T, 2.5); circ("field stack (48,48)", 2*T, 2*T, 2.5)
for sx in (-1, 1):
    for sy in (-1, 1):
        circ(f"wall stack ({sx*68.7:.0f},{sy*23.5:.0f})", sx * 68.7, sy * 23.5, 2.5)
        circ(f"wall stack ({sx*23.2:.0f},{sy*68.7:.0f})", sx * 23.2, sy * 68.7, 2.5)
obs["bottom toggle"] = box(-5, -WALL, 5, -WALL + 3)
obs["top toggle"] = box(-5, WALL - 3, 5, WALL)
obs["left toggle"] = box(-WALL, -5, -WALL + 3, 5)
obs["right toggle"] = box(WALL - 3, -5, WALL, 5)
obs["red loader (right)"] = box(54, -WALL, 66, LOADER[1])
obs["red loader (left)"] = box(-66, -WALL, -54, LOADER[1])
obs["blue loader (right)"] = box(54, -LOADER[1], 66, WALL)
obs["blue loader (left)"] = box(-66, -LOADER[1], -54, WALL)
outside = box(-200, -200, 200, 200).difference(box(-WALL, -WALL, WALL, WALL))
obs["wall"] = outside
NOCLUSTER=False
if NOCLUSTER: del obs["pin cluster (48,-48)"]

