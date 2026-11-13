#!/usr/bin/env python3
"""homing_starts.py -- the FIXED homing starts of Exam #3 rev 2 (operator-approved 2026-11-13; results/EXAM2_SPEC.md section 14.2).

THE ONE PLACE the table lives: tools/exam.sh (through tools/homing_trials.sh --fixed) reads the starts from here, tools/exam_record.py
copies the table into every record (meta.homing_starts), the spec prints `--md`.

Why: until rev 2 the 10 starts of Test 3 were seeded draws (homing_trials.sh --ring: distance uniform 3-5 m, bearing uniform 0-360 deg,
heading uniform +-180 deg, random.seed(seed * 1000 + trial)). The draws were the same in every exam, so they were not noise -- but they
were an accident of one seed: the seed-1 bearings were 4, 28, 80, 89, 123, 193, 214, 221, 260, 274 deg (two clusters, 90 deg empty
behind the desk), distances and headings drawn independently, so "homing" measured one arbitrary sample of the task. A designed table covers the ring evenly and repeats each condition.

The design (goal = the desk edge centre (gx, gy) + the approach heading psi; exam: 0.35, 0, 0 deg):
  bearing   i x 36 deg around the goal, i = 0..9, measured from the approach heading (0 deg = straight behind the desk, 180 deg = in
            front of the edge on the approach line); start = goal + d (cos(psi + bearing), sin(psi + bearing))
  distance  d = 3 m for even i, 5 m for odd i
  heading   relative to the direction FROM the start TO the goal (= psi + bearing + 180 deg), cycling
            i % 3 == 0: "away"  = that direction + 180 deg (the robot's back to the goal: a half turn first)
            i % 3 == 1: "+90"   = that direction + 90 deg  (the goal on the robot's right)
            i % 3 == 2: "-90"   = that direction - 90 deg  (the goal on the robot's left)
Worked, i = 1: bearing 36 deg, d = 5 m -> x = 0.35 + 5 cos 36 = 4.395, y = 5 sin 36 = 2.939; to the goal = 216 deg, +90 -> 306 = -54.0 deg.

  python3 homing_starts.py [--goal "0.35 0 0"] [--n 10]      one "x y yaw_deg" line per start (what homing_trials.sh reads)
  python3 homing_starts.py --md                               the table for the spec
  python3 homing_starts.py --json                             the record block (meta.homing_starts)
"""
import json
import math
import sys

REV = 2                      # the exam revision this table belongs to
N = 10
BEARING_STEP_DEG = 36.0
DISTANCES_M = (3.0, 5.0)     # even i, odd i
HEADINGS = (("away", 180.0), ("+90", 90.0), ("-90", -90.0))   # (name, offset from the direction to the goal, deg), cycled by i % 3
GOAL = (0.35, 0.0, 0.0)      # the exam's goal: desk edge centre x, y [m], approach heading [deg]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def table(goal=GOAL, n=N):
    """[{i, bearing_deg, dist_m, heading, x, y, yaw_deg}], x / y rounded to 1 mm and yaw to 0.1 deg (what is sent to the commander)"""
    gx, gy, psi = goal
    rows = []
    for i in range(n):
        b = BEARING_STEP_DEG * i
        d = DISTANCES_M[i % len(DISTANCES_M)]
        name, off = HEADINGS[i % len(HEADINGS)]
        th = math.radians(psi + b)
        to_goal = psi + b + 180.0
        rows.append({"i": i + 1, "bearing_deg": b, "dist_m": d, "heading": name,
                     "x": round(gx + d * math.cos(th), 3), "y": round(gy + d * math.sin(th), 3), "yaw_deg": round(wrap_deg(to_goal + off), 1)})
    return rows


def meta(goal=GOAL, n=N):
    return {"rev": REV, "goal": list(goal), "n": n, "bearing_step_deg": BEARING_STEP_DEG, "distances_m": list(DISTANCES_M),
            "headings": [list(h) for h in HEADINGS], "source": "unitree_mujoco/mujoco_sim/tools/homing_starts.py",
            "design": "bearing i x 36 deg around the goal from the approach heading; distance 3 m (even i) / 5 m (odd i); heading = the direction "
                      "to the goal + 180 (away) / + 90 / - 90 deg, cycled by i % 3",
            "rows": table(goal, n)}


def main():
    a = sys.argv[1:]
    goal = tuple(float(v) for v in a[a.index("--goal") + 1].split()) if "--goal" in a else GOAL
    n = int(a[a.index("--n") + 1]) if "--n" in a else N
    if "--json" in a:
        print(json.dumps(meta(goal, n)))
    elif "--md" in a:
        print("| trial | bearing ° | distance m | heading rule | x m | y m | yaw ° |\n|---|---|---|---|---|---|---|")
        for r in table(goal, n):
            print(f"| {r['i']} | {r['bearing_deg']:g} | {r['dist_m']:g} | {r['heading']} | {r['x']:.3f} | {r['y']:.3f} | {r['yaw_deg']:.1f} |")
    else:
        for r in table(goal, n):
            print(f"{r['x']:.3f} {r['y']:.3f} {r['yaw_deg']:.1f}")


if __name__ == "__main__":
    main()
