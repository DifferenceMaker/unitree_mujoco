#!/usr/bin/env python3
"""exam_record.py <exam dir> <policy name> <milestone slug> [--exam 2 --tag T --seed S --n N --balance B --instance K]

The per-policy exam record that travels with the milestone (operator 2026-10-29: "save that data and visually reconstruct it in fleetdeck
in the specific policy page"): aspired-isaac-lab/milestone_checkpoints/<slug>/exam/<tag>_<stamp>.json =
  meta        tag, stamp, seed, n, balance, walk, rig (lock-step, sim build), score parts (tools/exam_report.py's formula)
  homing      the 10 trial records (homing_<name>.jsonl) + per trial a 2 Hz copy of its 10 Hz trajectory (t, phase, x, y, yaw, cmd, truth vel)
  stairs      the staircase record (stairs_<name>.jsonl) + its 10 Hz rows (STAIRS_COLS incl. the stance columns)
Sizes: ~150 KB per policy, ~3 MB per 19-row wave -- JSON in git, the PNG sheets are not stored (fleetdeck draws SVG from this file).
Also writes the same file next to the exam's own records (<exam dir>/exam_<name>.json).
EXAM #2 (--exam 2, 2026-10-30): + ladders (Test 1: the ladder record + its rows at 5 Hz + the stance drawing = tools/stance_fk.py on the
20 s stand's joint state), pushes (Test 2: the push record + the rows around every push at 10 Hz), meta.exam_version 2 and the score by
quality (tools/exam_score.py score_v2: every metric with its value, g / b, ramp and points)."""
import argparse, csv, glob, json, os, time

ap = argparse.ArgumentParser()
ap.add_argument("exam_dir"); ap.add_argument("name"); ap.add_argument("slug")
ap.add_argument("--tag", default="exam"); ap.add_argument("--seed", default="1"); ap.add_argument("--n", default="10"); ap.add_argument("--balance", default="")
ap.add_argument("--stop-dist", default=None, help="homing: torso front -> desk edge [m] (exam.sh --stop-dist)")
ap.add_argument("--exam", default="1"); ap.add_argument("--instance", default=""); ap.add_argument("--ail", default=os.path.expanduser("~/Projects/robot_projects/repos/aspired-isaac-lab"))
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

ladders = pushes = None
if a.exam == "2":
    import subprocess
    ladders = load(f"{EX}/ladders_{name}.jsonl"); ladders = ladders[-1] if ladders else None
    pushes = load(f"{EX}/pushes_{name}.jsonl"); pushes = pushes[-1] if pushes else None
    if ladders:
        cols, rows = traj_rows(ladders.get("traj_file"), every=2)       # 10 -> 5 Hz (340 s): the stance / tracking sheets
        ladders["rows"] = {"hz": 5, "cols": cols, "rows": [[round(v, 3) if isinstance(v, float) else v for v in r] for r in rows]} if rows else None
        # the stance drawing: forward kinematics of the 20 s stand's joint state on the exam body (needs a python with mujoco)
        sq = ladders.get("stance_q")
        scene = None
        try:
            for l in open(f"{EX}/rig_{name}.log"):
                if "[scene] robot_scene ->" in l: scene = l.split("->")[1].split()[0]; break
        except FileNotFoundError:
            pass
        xml = os.path.join(SIM, "..", "unitree_robots", "h1_2", "h1_2_" + (scene or "scene_comx06_armature_hand790.xml")[len("scene_"):].replace("_desk.xml", ".xml"))
        if sq and os.path.exists(xml):
            fkpy = next((c for c in (os.environ.get("EXAM_FK_PY"), os.path.expanduser("~/miniconda3/envs/isaacsim/bin/python"), "python3")
                         if c and subprocess.run([c, "-c", "import mujoco"], capture_output=True).returncode == 0), None)
            if fkpy:
                src, dst = f"{EX}/stance_q_{name}.json", f"{EX}/stance_fk_{name}.json"
                json.dump(sq, open(src, "w"))
                rc = subprocess.run([fkpy, os.path.join(SIM, "tools", "stance_fk.py"), xml, src, dst], capture_output=True, text=True)
                if rc.returncode == 0:
                    ladders["stance_fk"] = json.load(open(dst)); ladders["stance_fk"]["mjcf"] = os.path.basename(xml)
                else:
                    print(f"stance_fk failed: {rc.stderr[-400:]}")
            else:
                print("stance_fk skipped: no python with mujoco (EXAM_FK_PY)")
        ladders.pop("stance_q", None)   # the drawing replaces the raw joint rows (size)
    if pushes:
        cols, rows = traj_rows(pushes.get("traj_file"), every=1)
        if rows:   # 10 Hz around every push: the push window + the last 1 s of its 'pre'
            ti, si = cols.index("t"), cols.index("seg")
            keep = []
            for i, r in enumerate(rows):
                if r[si].startswith("push:"):
                    keep.append(r)
                elif r[si] == "pre":
                    nxt = next((q for q in rows[i:] if q[si] != "pre"), None)
                    if nxt and nxt[si].startswith("push:") and nxt[ti] - r[ti] <= 1.0:
                        keep.append(r)
            pushes["rows"] = {"hz": 10, "cols": cols, "rows": [[round(v, 3) if isinstance(v, float) else v for v in r] for r in keep]}

# the score, same formula as exam_report.py (kept in one place there; here only the parts the sheet shows)
import importlib.util, sys
spec = importlib.util.spec_from_file_location("exam_report_lib", os.path.join(SIM, "tools", "exam_score.py"))
score = None
if spec and os.path.exists(spec.origin):
    lib = importlib.util.module_from_spec(spec); spec.loader.exec_module(lib)
    score = lib.score_v2(handover, ladders, pushes, homing, n_homing=int(a.n)) if a.exam == "2" else lib.score_policy(homing, stairs)

stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
rig = {}
try:
    rig["sim_build"] = time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(f"{SIM}/../simulate/build/unitree_mujoco")))
except OSError:
    pass
rec = {"meta": {"exam_version": int(a.exam), "tag": a.tag, "stamp": stamp, "seed": a.seed, "n": a.n, "balance": a.balance, "walk": name, "slug": slug, "mode": "lock-step headless",
                "stop_dist": float(a.stop_dist) if a.stop_dist is not None else None,
                "rig": rig, "score": score, "metrics_doc": "unitree_mujoco/mujoco_sim/results/EXAM_METRICS.md"},
       "homing": homing, "stairs": stairs, "handover": handover}
if a.exam == "2":
    rec["ladders"], rec["pushes"] = ladders, pushes
    rec["meta"]["metrics_doc"] = "unitree_mujoco/mujoco_sim/results/EXAM2_SPEC.md"
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
