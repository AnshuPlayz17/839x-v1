"""Top-down simulation of 839X's skills auton (src/autons.cpp).
Mirrors the code step by step. Field element sizes, robot size and speeds are GUESSES."""
import math, sys
from shapely.geometry import Point, Polygon, box
from shapely import affinity

T = 23.8
WALL = 3 * T  # 71.4

# ---------------- robot + speed guesses ----------------
ROBOT_W = float(sys.argv[1]) if len(sys.argv) > 1 else 17.0
ROBOT_L = ROBOT_W
FRONT = ROBOT_L / 2          # FRONT_OFFSET_IN
GRAB = 10.0                  # GRABBER_OFFSET_IN (guess)
TRACK = 12.0
VMAX_100 = 76.6              # in/s at 100% (3.25" wheels at 450 rpm, guess)
PCT = 0.60                   # AUTON_PCT
VMAX = VMAX_100 * PCT
ACC = 60.0                   # in/s^2 (guess)
SETTLE = 0.25                # s at the end of each stop-type move
CLOCK_CHECK = (sys.argv[2] != "nocheck") if len(sys.argv) > 2 else True
LIFT_DPS = 600.0             # elevator deg/s (LIFT_RPM 100 guess)

# ---------------- team's numbers (autons.cpp) ----------------
START = (0, -62.7, 180)
GOAL = (-T, -2 * T); UPPER = (-T, -T); LOWER = (-2 * T, -2 * T)
TOGGLE_RAM, TOGGLE_BACK = 14, 10
PRELOAD_HEADING, PRELOAD_TURN_Y = 89, -2 * T - 2
UPPER_FIX, UPPER_BACK_EXTRA, LOWER_BACK_EXTRA, PRELOAD_BACK_EXTRA = 2, 7, 1, 2
LOADER = (60, -64); LOADER_H = 180
GOAL_A = (T, -2 * T); GOAL_B = (2 * T, -T)
CROSS_DOWN_X, CROSS_LANE_Y, CROSS_END_X = -44, -58.6, 40
RIGHT_LANE_X, ACROSS_Y, UP_LANE_X, PARK_TURN_Y = 58.6, 11.4, 11.4, 14
TOP_TOGGLE = (0, 62.7)
SKILLS_MS, TRIP_MS, FINISH_MS = 59.5, 11.0, 16.0
A1, A2 = (700, 610, 800), (900, 860, 1000)
B2, B3, B4 = (700, 610, 800), (900, 860, 1000), (1100, 1060, 1200)

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
NOCLUSTER = len(sys.argv) > 3 and sys.argv[3] == "nocluster"
if NOCLUSTER: del obs["pin cluster (48,-48)"]

# ---------------- state ----------------
x, y, h = START
t = 0.0
mech_free = 0.0
lift = 0.0
part = "1 red auton"
samples = []          # (t, x, y, h, part)
legs = []             # (part, label, t0, t1, min clearance, nearest, allowed)
removed = set()
hits = []
allowed = set()
events = []
skipped = []

def d2r(a): return math.radians(a)
def wrap(a): return (a + 180) % 360 - 180
def heading_to(px, py, qx, qy): return math.degrees(math.atan2(qx - px, qy - py)) % 360

def robot_poly(x, y, h):
    r = box(-ROBOT_W / 2, -ROBOT_L / 2, ROBOT_W / 2, ROBOT_L / 2)
    r = affinity.rotate(r, -h, origin=(0, 0))
    return affinity.translate(r, x, y)

def check(label, t0):
    """Min clearance over the samples of this leg."""
    best, who = 1e9, ""
    for (ts, sx, sy, sh, _) in samples:
        if ts < t0 - 1e-9: continue
        rp = robot_poly(sx, sy, sh)
        for n, g in obs.items():
            if n in removed or n in allowed: continue
            d = rp.distance(g)
            if d == 0: d = -math.sqrt(rp.intersection(g).area)  # overlap shown as negative
            if d < best: best, who = d, n
            if d < 0: hits.append((sx, sy, n))
    legs.append((part, label, t0, t, best, who, sorted(allowed)))

def rec():
    samples.append((t, x, y, h, part))

def turn_to(target, label="turn"):
    global h, t
    t0 = t
    err = wrap(target - h)
    wmax = math.degrees(2 * VMAX / TRACK); alpha = math.degrees(2 * ACC / TRACK)
    total = abs(err); sgn = 1 if err >= 0 else -1
    ta = wmax / alpha; da = 0.5 * alpha * ta * ta
    dur = 2 * math.sqrt(total / alpha) if total < 2 * da else 2 * ta + (total - 2 * da) / wmax
    n = max(2, int(dur / 0.02)); h0 = h
    for i in range(1, n + 1):
        h = (h0 + sgn * total * i / n) % 360; t = t0 + dur * i / n; rec()
    t += SETTLE; h = target % 360; rec()
    check(label, t0)

def straight(inches, label="drive", vmax=VMAX, timeout=None):
    global x, y, t
    t0 = t
    dist = abs(inches); sgn = 1 if inches >= 0 else -1
    ta = vmax / ACC; da = 0.5 * ACC * ta * ta
    dur = 2 * math.sqrt(dist / ACC) if dist < 2 * da else 2 * ta + (dist - 2 * da) / vmax
    n = max(2, int(dur / 0.02)); x0, y0 = x, y
    for i in range(1, n + 1):
        f = i / n; x = x0 + sgn * dist * f * math.sin(d2r(h)); y = y0 + sgn * dist * f * math.cos(d2r(h)); t = t0 + dur * f; rec()
    t = t0 + (timeout if timeout else dur + SETTLE); rec()
    check(label, t0)

def move(tx, ty, forwards=True, chained=False, pose_h=None, lead=0.6, label="move"):
    """moveToPoint / moveToPose: unicycle that chases a (carrot) point."""
    global x, y, h, t
    t0 = t; v = 0.0; dt = 0.01
    wmax = 2 * VMAX / TRACK * 0.7
    exit_r = 6 if chained else 0.75
    for _ in range(4000):
        d = math.hypot(tx - x, ty - y)
        if d < exit_r: break
        cx, cy = tx, ty
        if pose_h is not None and d > 6:
            cx = tx - lead * d * math.sin(d2r(pose_h)); cy = ty - lead * d * math.cos(d2r(pose_h))
        trav = h if forwards else (h + 180) % 360
        e = wrap(heading_to(x, y, cx, cy) - trav)
        if pose_h is not None and d <= 6:
            e = wrap(pose_h - h) if forwards else wrap(pose_h - h)
            e = e * 0.5 + wrap(heading_to(x, y, tx, ty) - trav) * 0.5
        vd = VMAX * max(math.cos(d2r(e)), 0.0)
        if not chained: vd = min(vd, math.sqrt(2 * ACC * d))
        else: vd = max(vd, 50 / 127 * VMAX_100 * max(math.cos(d2r(e)), 0))
        v = min(vd, v + ACC * dt) if vd > v else max(vd, v - 2 * ACC * dt)
        w = max(-wmax, min(wmax, 6.0 * d2r(e)))
        h = (h + math.degrees(w) * dt) % 360
        s = 1 if forwards else -1
        x += s * v * dt * math.sin(d2r(h)); y += s * v * dt * math.cos(d2r(h)); t += dt
        if int(t / dt) % 3 == 0: rec()
    rec()
    if not chained: t += SETTLE
    check(label, t0)
    if pose_h is not None:
        turn_to(pose_h, label + " (square up)")

# ----- mechanism timing (guesses) -----
def mech_start(dur):
    global mech_free
    mech_free = t + dur
def mech_wait():
    global t
    t = max(t, mech_free); rec()
def lift_to(hgt):
    global lift
    mech_start(abs(hgt - lift) / LIFT_DPS); lift = hgt
def home():
    global lift
    mech_start(lift / LIFT_DPS + 0.5); lift = 0
def grab():
    global lift
    mech_start(0.85); lift += 60
def score(down):  # dip to `down`, let go, small raise
    global lift
    mech_start(abs(lift - down) / LIFT_DPS + 0.7); lift = down + 60
def delay(s):
    global t
    t += s; rec()
def ev(name):
    events.append((t, name))

# ----- the team's helpers -----
def drive(inches, label="drive"): straight(inches, label)
def turn_back_to(px, py, label="turn back"): turn_to((heading_to(x, y, px, py) + 180) % 360, label)
def turn_face(px, py, label="turn"): turn_to(heading_to(x, y, px, py), label)
def back_up_to(px, py, extra=0, label="back onto"):
    dist = math.hypot(px - x, py - y)
    want = GRAB - dist - extra
    stop = ROBOT_L / 2 + 1.0 - dist   # back edge meets the goal post (1 in radius at the top)
    drive(max(want, stop), label)
def grabber_to(px, py, extra=0, label="grabber to"):
    turn_back_to(px, py, label + " (turn)"); back_up_to(px, py, extra, label)
def ram(label):
    global x, y, t
    # drives until the front pushes the toggle in, then waits for the 1 s time limit
    room = (WALL - 1.5 - FRONT) - abs(y)  # front stops when the toggle is pushed in
    straight(max(0.0, min(TOGGLE_RAM, room)), label, timeout=1.0)
def clamp_into_field():
    global x, y
    lim = WALL - 1.5 - FRONT  # toggle pushed in to 1.5 in from the wall
    if h % 360 == 180: y = max(y, -lim)
    if h % 360 == 0: y = min(y, lim)
def score_and_clear(down):
    score(down); mech_wait(); lift_to(250); mech_wait()
def field_grab(hgt):
    lift_to(hgt); mech_wait(); delay(0.25); lift_to(250); mech_wait()

# ===================== the route =====================
def hit_toggle(name):
    global allowed
    allowed = {name, "wall"}
    for i in (1, 2):
        ram(f"ram {name} {i}")
        clamp_into_field(); rec()
        drive(-TOGGLE_BACK, f"back off {name} {i}")
    allowed = set()
    ev(f"{name} hit")

# ---- Part 1: togglePreload + twoStacks ----
rec()
hit_toggle("bottom toggle")
grab()
move(0, PRELOAD_TURN_Y, forwards=False, label="back off the wall")
mech_wait()
turn_to(PRELOAD_HEADING, "preload turn")
allowed = {"red goal 3"}; back_up_to(*GOAL, PRELOAD_BACK_EXTRA, "preload onto red goal"); allowed = set()
score_and_clear(lift - 90); ev("preload scored")
allowed = {"red goal 3"}; drive(8, "off the goal"); allowed = set()
allowed = {"UPPER stack"}; grabber_to(UPPER[0] - UPPER_FIX, UPPER[1], 0, "grabber to UPPER")
field_grab(0); removed.add("UPPER stack"); allowed = set(); ev("UPPER grabbed")
allowed = {"UPPER stack"}; drive(6, "off UPPER"); allowed = set()
allowed = {"red goal 3"}; grabber_to(*GOAL, UPPER_BACK_EXTRA, "UPPER onto red goal"); allowed = set()
score_and_clear(lift - 30); ev("UPPER scored")
allowed = {"red goal 3"}; drive(8, "off the goal"); allowed = set()
turn_face(*UPPER, "turn to UPPER spot"); move(*UPPER, label="to UPPER spot")
allowed = {"LOWER stack"}; grabber_to(*LOWER, 0, "grabber to LOWER")
field_grab(25); removed.add("LOWER stack"); allowed = set(); ev("LOWER grabbed")
allowed = {"LOWER stack"}; drive(6, "off LOWER"); allowed = set()
allowed = {"red goal 3"}; grabber_to(*GOAL, LOWER_BACK_EXTRA, "LOWER onto red goal"); allowed = set()
score_and_clear(lift - 30); ev("LOWER scored")
allowed = {"red goal 3"}; drive(4, "off the goal"); allowed = set()
part1_end = (x, y, h)

# ---- Part 2: cross ----
part = "2 cross"
home()
move(CROSS_DOWN_X, CROSS_LANE_Y, chained=True, label="down left of the red goal")
move(CROSS_END_X, CROSS_LANE_Y, chained=True, label="along the bottom lane")

def to_loader(label="to the loader"):
    global allowed
    home()
    allowed = {"red loader (right)"}
    move(LOADER[0], LOADER[1] + FRONT, pose_h=LOADER_H, lead=0.4, label=label)
    allowed = set()

to_loader("curve into the loader"); ev("at loader")

def pickup():
    global allowed
    allowed = {"red loader (right)"}
    mech_wait(); delay(1.5); grab()
    drive(-6, "back off the loader"); allowed = set()

def loader_stack(goal, gname, spots, tag):
    global allowed
    pickup()
    turn_back_to(*goal, f"turn back to {gname}")
    mech_wait(); lift_to(spots[0])
    allowed = {gname}; back_up_to(*goal, 0, f"onto {gname}")
    mech_wait(); score(spots[1]); mech_wait(); lift_to(spots[2]); mech_wait()
    ev(f"{tag} scored on {gname}")
    drive(6, f"off {gname}"); allowed = set(); home()

def loader_pin(goal, gname, tag):
    global allowed
    pickup()
    turn_back_to(*goal, f"turn back to {gname}")
    mech_wait()
    allowed = {gname}; back_up_to(*goal, PRELOAD_BACK_EXTRA, f"onto {gname}")
    score(lift - 90); mech_wait(); ev(f"{tag} scored on {gname}")
    drive(6, f"off {gname}"); allowed = set(); home()

def time_for(ms):
    return (not CLOCK_CHECK) or (t + ms + FINISH_MS <= SKILLS_MS)

# ---- Part 3: goal A ----
part = "3 goal A"
loads = 0
def trips():
    global part, loads
    loader_stack(GOAL_A, "goal A", A1, "load 1+2 (pin+cup)"); loads += 2
    if not time_for(TRIP_MS): skipped.append("goal A 2nd"); return False
    to_loader(); loader_stack(GOAL_A, "goal A", A2, "load 3+4 (pin+cup)"); loads += 2
    part = "4 goal B"
    if not time_for(TRIP_MS): skipped.append("goal B pin"); return False
    to_loader(); loader_pin(GOAL_B, "goal B", "load 5 (pin)"); loads += 1
    for i, sp in enumerate((B2, B3, B4)):
        if not time_for(TRIP_MS): skipped.append(f"goal B stack {i+1}"); return False
        to_loader(); loader_stack(GOAL_B, "goal B", sp, f"load {loads+1}+{loads+2} (pin+cup)"); loads += 2
    return True
trips()
part4_end = (x, y, h)

# ---- Part 5: finish ----
part = "5 toggle + park"
drive(12 - 6, "12 in off the goal")  # 6 already done after the drop, code does offGoal(12) from on-goal
home()
move(RIGHT_LANE_X, ACROSS_Y, chained=True, label="up the right side")
move(UP_LANE_X, ACROSS_Y, chained=True, label="across above blue pin (24,0)")
move(UP_LANE_X, TOP_TOGGLE[1] - 2 * TOGGLE_BACK, chained=True, label="up the middle")
allowed = {"top toggle"}
move(*TOP_TOGGLE, pose_h=0, lead=0.3, label="square to the top toggle"); allowed = set()
hit_toggle("top toggle")
move(UP_LANE_X, TOP_TOGGLE[1] - 2 * TOGGLE_BACK, forwards=False, chained=True, label="back over to the middle lane")
move(UP_LANE_X, PARK_TURN_Y, forwards=False, label="back down the middle")
allowed = {"tall goal"}; grabber_to(0, 0, 0, "grabber onto tall goal"); allowed = set()
ev("parked")

# ===================== report =====================
print(f"ROBOT {ROBOT_W:.0f} x {ROBOT_L:.0f} in   speed {PCT*100:.0f}%   clock check {'ON' if CLOCK_CHECK else 'OFF'}")
print(f"part 1 ends at x {part1_end[0]:.1f} y {part1_end[1]:.1f} h {part1_end[2]:.0f}")
print(f"part 4 ends at x {part4_end[0]:.1f} y {part4_end[1]:.1f} h {part4_end[2]:.0f}")
print(f"TOTAL {t:.1f} s   loads used {loads}   skipped: {', '.join(skipped) or 'none'}")
for (tt, n) in events: print(f"  {tt:6.1f}s  {n}")
print("Tight or blocked moves (clearance < 1 in):")
for (p, lab, t0, t1, best, who, al) in legs:
    if best < 1.0:
        print(f"  [{p}] {lab:34s} {best:+5.1f} in  {who}")
import json
json.dump({"samples": samples, "legs": legs, "events": events, "W": ROBOT_W, "L": ROBOT_L, "t": t,
           "skipped": skipped, "hits": hits, "loads": loads, "removed": sorted(removed)},
          open(f"run_{int(ROBOT_W)}_{'on' if CLOCK_CHECK else 'off'}{'_nc' if NOCLUSTER else ''}.json", "w"))
