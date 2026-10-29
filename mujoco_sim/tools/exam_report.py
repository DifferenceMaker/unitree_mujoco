#!/usr/bin/env python3
"""exam_report.py <exam dir> [--seed S --n N --balance B --tag T --wall SECONDS --parallel P] -- the ranking table of one exam run.

Reads per policy homing_<name>.jsonl (Test 2 records) and stairs_<name>.jsonl (Test 1 record) from tools/exam.sh, prints the table,
writes <exam dir>/EXAM.md and appends the same block to mujoco_sim/results/EXAM.md.

Score (0-100, transparent, every term bounded):
  tracking  40  mean over the staircase's 12 command segments of min(gain, 1)  (gain = sim-truth achieved / commanded; over-tracking
                is capped, under-tracking costs linearly; a fall or a missing segment counts 0)
  placement 20  1 - min(settled planar mean / 0.10 m, 1)                     (homing: the robot's offset from the stop pose after the stand)
  stillness 20  half: 1 - min(stand-10s drift / 0.10 m, 1)  (staircase's first segment: the zero-command stand walks how far)
                half: 1 - min(|stand after the switch, lateral| / 0.10 m, 1) (homing: what the hold + switch + settle added)
  turn      10  (1 - min(mean pivot index over the |wz| = 0.5 / 0.8 turns / 0.5 rad/s, 1)) x min(mean |gain| at 0.8, 1)  (slide = 0, no turn = 0, stepping turn ~1)
  survival  10  homing arrivals / n, times 0 if the staircase fell
Numbers are FLAGS and a ranking aid; the operator's eye on the rig is the verdict (RL Bible rule 34)."""
import argparse, glob, json, os, statistics as st, sys, datetime

ap = argparse.ArgumentParser()
ap.add_argument("exam_dir"); ap.add_argument("--seed", default="?"); ap.add_argument("--n", default="?"); ap.add_argument("--balance", default="?")
ap.add_argument("--tag", default=""); ap.add_argument("--wall", type=int, default=0); ap.add_argument("--parallel", default="?")
a = ap.parse_args()
EX = a.exam_dir
SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
names = sorted({os.path.basename(p)[len("homing_"):-len(".jsonl")] for p in glob.glob(f"{EX}/homing_*.jsonl")} |
               {os.path.basename(p)[len("stairs_"):-len(".jsonl")] for p in glob.glob(f"{EX}/stairs_*.jsonl")})
times = {}
if os.path.exists(f"{EX}/times.txt"):
    for l in open(f"{EX}/times.txt"):
        w = l.split()
        if len(w) == 2: times[w[0]] = int(w[1])

def load(p):
    try: return [json.loads(l) for l in open(p) if l.strip()]
    except FileNotFoundError: return []
def mean(v, d=None): return st.mean(v) if v else d
def clamp01(x): return max(0.0, min(1.0, x))
import importlib.util
_spec = importlib.util.spec_from_file_location("exam_score", os.path.join(os.path.dirname(os.path.abspath(__file__)), "exam_score.py"))
exam_score = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(exam_score)
SEGS = exam_score.SEGS

rows = []
for nm in names:
    H = load(f"{EX}/homing_{nm}.jsonl"); S = load(f"{EX}/stairs_{nm}.jsonl"); S = S[-1] if S else None
    r = exam_score.score_policy(H, S)
    r["name"] = nm; r["t_s"] = times.get(nm)
    p = r["parts"]; r["parts"] = (p["tracking"], p["placement"], p["stillness"], p["turn"], p["survival"])
    rows.append(r)
rows.sort(key=lambda r: -r["score"])

f = lambda v, p=3: "-" if v is None or (isinstance(v, float) and v != v) else f"{v:.{p}f}"
hdr = (f"| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | "
       f"stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |")
sep = "|" + "---|" * 23
lines = [hdr, sep]
for i, r in enumerate(rows, 1):
    g = r["gains"]
    lines.append(f"| {i} | {r['name']} | **{r['score']}** | {'/'.join(str(x) for x in r['parts'])} | {r['arrived']}/{r['n']} | {f(r.get('planar'))} | "
                 f"{f(r.get('stand_left'))} / {f(r.get('stand_turn'), 1)} | {f(r.get('g_vx'), 2)} | {f(r.get('g_wzL'), 2)}/{f(r.get('g_wzR'), 2)} | {f(r.get('eff'), 2)} | "
                 f"{f(g['vx05'], 2)}/{f(g['vx10'], 2)}/{f(g['vx15'], 2)}/{f(g['vxm10'], 2)} | {f(g['vyL10'], 2)}/{f(g['vyR10'], 2)} | {f(g['wzL03'], 2)}/{f(g['wzR03'], 2)} | "
                 f"{f(g['wzL05'], 2)}/{f(g['wzR05'], 2)} | {f(g['wzL08'], 2)}/{f(g['wzR08'], 2)} | {f(r.get('stand10_drift'))} / {f(r.get('stand10_yaw'), 1)} / {r.get('stand10_lifts', '-')} | "
                 f"{f(r.get('pivot05'), 2)} | {r.get('lifts05') or '-'} | {f(r.get('swing'))} | {('%s@%ss' % (r['fell']['seg'], r['fell']['t'])) if r.get('fell') else ('-' if r.get('stairs') else 'no stairs')} | {r.get('t_s') or '-'} |")
stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
head = (f"## EXAM {a.tag} — {stamp} — {len(rows)} policies, seed {a.seed}, homing n {a.n}, balance {a.balance}, {a.parallel} instances, "
        f"wall {a.wall // 60} min {a.wall % 60} s\n\nScore = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). "
        f"Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line "
        f"(sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: {EX}\n")
block = head + "\n" + "\n".join(lines) + "\n"
print(block)
open(f"{EX}/EXAM.md", "w").write(block)
os.makedirs(f"{SIM}/results", exist_ok=True)
with open(f"{SIM}/results/EXAM.md", "a") as fh:
    if fh.tell() == 0: fh.write("# Standardized walk exam (tools/exam.sh) -- one block per run, newest last\n\n")
    fh.write(block + "\n")
print(f"-> {EX}/EXAM.md and {SIM}/results/EXAM.md")
