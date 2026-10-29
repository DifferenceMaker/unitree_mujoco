#!/usr/bin/env python3
"""exam_record.py <exam dir> <policy name> <milestone slug> [--tag T --seed S --n N --balance B --instance K]

The per-policy exam record that travels with the milestone (operator 2026-10-29: "save that data and visually reconstruct it in fleetdeck
in the specific policy page"): aspired-isaac-lab/milestone_checkpoints/<slug>/exam/<tag>_<stamp>.json =
  meta        tag, stamp, seed, n, balance, walk, rig (lock-step, sim build), score parts (tools/exam_report.py's formula)
  homing      the 10 trial records (homing_<name>.jsonl) + per trial a 2 Hz copy of its 10 Hz trajectory (t, phase, x, y, yaw, cmd, truth vel)
  stairs      the staircase record (stairs_<name>.jsonl) + its 10 Hz rows (STAIRS_COLS incl. the stance columns)
Sizes: ~150 KB per policy, ~3 MB per 19-row wave -- JSON in git, the PNG sheets are not stored (fleetdeck draws SVG from this file).
Also writes the same file next to the exam's own records (<exam dir>/exam_<name>.json)."""
import argparse, csv, glob, json, os, time

ap = argparse.ArgumentParser()
ap.add_argument("exam_dir"); ap.add_argument("name"); ap.add_argument("slug")
ap.add_argument("--tag", default="exam"); ap.add_argument("--seed", default="1"); ap.add_argument("--n", default="10"); ap.add_argument("--balance", default="")
ap.add_argument("--instance", default=""); ap.add_argument("--ail", default=os.path.expanduser("~/Projects/robot_projects/repos/aspired-isaac-lab"))
a = ap.parse_args()
SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EX, name, slug = a.exam_dir, a.name, a.slug
SFX = f"_{a.instance}" if a.instance else ""
TRAJ = f"{SIM}/logs/homing_traj{SFX}"

def load(p):
    try: return [json.loads(l) for l in open(p) if l.strip()]
    except FileNotFoundError: return []

def traj_rows(fname, every=5, cols=None):
    """the CSV decimated (every k-th row) as a list of lists in the given column order"""
    p = os.path.join(TRAJ, fname) if fname else None
    if not p or not os.path.exists(p):   # the exam's instance is unknown when called by hand: search every traj dir
        cands = glob.glob(f"{SIM}/logs/homing_traj*/{fname}") if fname else []
        if not cands: return None, None
        p = cands[0]
    rows = list(csv.DictReader(open(p)))
    if not rows: return None, None
    cols = cols or list(rows[0].keys())
    out = []
    for i, r in enumerate(rows):
        if i % every: continue
        out.append([r[c] if c in ("phase", "seg") else (round(float(r[c]), 4) if r[c] not in ("", "nan") else None) for c in cols])
    return cols, out

homing = load(f"{EX}/homing_{name}.jsonl")
stairs = load(f"{EX}/stairs_{name}.jsonl"); stairs = stairs[-1] if stairs else None
handover = load(f"{EX}/handover_{name}.jsonl")   # exam Test 0 (2026-10-29): one record per arm pose
H_COLS = ["t", "phase", "x", "y", "yaw_deg", "fx", "fy", "fyaw_deg", "cmd_vx", "cmd_vy", "cmd_wz", "ovx", "ovy", "owz", "along", "lat", "head_deg"]
for r in homing:
    cols, rows = traj_rows(r.get("traj_file"), every=5, cols=H_COLS)   # 10 Hz -> 2 Hz
    r["traj"] = {"hz": 2, "cols": cols, "rows": rows} if rows else None
for r in handover:
    cols, rows = traj_rows(r.get("traj_file"), every=1)
    r["rows"] = {"hz": 10, "cols": cols, "rows": rows} if rows else None
if stairs:
    cols, rows = traj_rows(stairs.get("traj_file"), every=1)           # 10 Hz, every row (the stance sheet needs the lifts)
    stairs["rows"] = {"hz": 10, "cols": cols, "rows": rows} if rows else None

# the score, same formula as exam_report.py (kept in one place there; here only the parts the sheet shows)
import importlib.util, sys
spec = importlib.util.spec_from_file_location("exam_report_lib", os.path.join(SIM, "tools", "exam_score.py"))
score = None
if spec and os.path.exists(spec.origin):
    lib = importlib.util.module_from_spec(spec); spec.loader.exec_module(lib)
    score = lib.score_policy(homing, stairs)

stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
rig = {}
try:
    rig["sim_build"] = time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(f"{SIM}/../simulate/build/unitree_mujoco")))
except OSError:
    pass
rec = {"meta": {"tag": a.tag, "stamp": stamp, "seed": a.seed, "n": a.n, "balance": a.balance, "walk": name, "slug": slug, "mode": "lock-step headless",
                "rig": rig, "score": score, "metrics_doc": "unitree_mujoco/mujoco_sim/results/EXAM_METRICS.md"},
       "homing": homing, "stairs": stairs, "handover": handover}
out_local = f"{EX}/exam_{name}.json"
json.dump(rec, open(out_local, "w"), separators=(",", ":"))
msd = os.path.join(a.ail, "milestone_checkpoints", slug)
if os.path.isdir(msd):
    os.makedirs(os.path.join(msd, "exam"), exist_ok=True)
    out_ms = os.path.join(msd, "exam", f"{a.tag}_{stamp}.json")
    json.dump(rec, open(out_ms, "w"), separators=(",", ":"))
    print(f"exam record -> {out_ms} ({os.path.getsize(out_ms) // 1024} KB)")
else:
    print(f"exam record -> {out_local} ({os.path.getsize(out_local) // 1024} KB); no milestone dir {msd}")
