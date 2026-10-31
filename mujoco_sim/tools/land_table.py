#!/usr/bin/env python3
"""land_table.py <exam dir> --new <prefix> --old <prefix> [--lognums <json>] -- the "land" tables (skill: land).

From one exam run that covered a new wave and the previous wave on the same rig:
  1. the exam table of the NEW wave (score, parts, key columns), ranked
  2. the comparison with the OLD wave: how many new rows beat the old best / the old median score; per key column the old best vs the new best
     and which new rows beat the old best
Numbers are flags (RL Bible rule 34); the operator's eye on the rig is the verdict. Policy names = the exam's vendored names
(lm7g_trunk, lm7f_kit_gforce, ...)."""
import argparse, glob, importlib.util, json, os, statistics as st

ap = argparse.ArgumentParser()
ap.add_argument("exam_dir"); ap.add_argument("--new", required=True); ap.add_argument("--old", required=True); ap.add_argument("--md", default="")
a = ap.parse_args()
TOOLS = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("exam_score", os.path.join(TOOLS, "exam_score.py"))
es = importlib.util.module_from_spec(spec); spec.loader.exec_module(es)

def load(p):
    try: return [json.loads(l) for l in open(p) if l.strip()]
    except FileNotFoundError: return []

if glob.glob(f"{a.exam_dir}/ladders_*.jsonl"):   # EXAM #2 (2026-10-30): the score by quality (exam_score.score_v2)
    names = sorted(os.path.basename(p)[len("ladders_"):-len(".jsonl")] for p in glob.glob(f"{a.exam_dir}/ladders_*.jsonl"))
    R = {}
    for nm in names:
        L = load(f"{a.exam_dir}/ladders_{nm}.jsonl"); Pu = load(f"{a.exam_dir}/pushes_{nm}.jsonl")
        R[nm] = es.score_v2(load(f"{a.exam_dir}/handover_{nm}.jsonl"), L[-1] if L else None, Pu[-1] if Pu else None, load(f"{a.exam_dir}/homing_{nm}.jsonl"))
    new = {k: v for k, v in R.items() if k.startswith(a.new)}; old = {k: v for k, v in R.items() if k.startswith(a.old)}
    Q = list(es.QUALITIES); out = []; P = out.append
    mk = lambda r, k: next((m for m in r["metrics"] if m["key"] == k), {})
    P(f"## Exam #2 table — {a.new} ({len(new)} rows), same rig and run as {a.old} ({len(old)} rows) · {os.path.basename(a.exam_dir)}"); P("")
    P("| # | policy | score | " + " | ".join(f"{q} /{es.QUALITIES[q]}" for q in Q) + " | falls | track vx / vy / wz | pushes | homing |")
    P("|" + "---|" * (len(Q) + 7))
    for i, (nm, r) in enumerate(sorted(new.items(), key=lambda kv: -kv[1]["score"]), 1):
        P(f"| {i} | {nm} | **{r['score']:.1f}** | " + " | ".join(str(r["parts"][q]) for q in Q) + f" | {len(r['falls'])} | "
          + " / ".join(f"{mk(r, k).get('s', 0):.2f}" for k in ("track_vx", "track_vy", "track_wz")) + f" | {mk(r, 'pushes').get('label', '').split(':')[0]} | {mk(r, 'homing').get('label', '').split(':')[0]} |")
    P("")
    if new and old:
        osc = sorted((r["score"] for r in old.values()), reverse=True); nsc = sorted((r["score"] for r in new.values()), reverse=True)
        ob = max(old.items(), key=lambda kv: kv[1]["score"]); nb = max(new.items(), key=lambda kv: kv[1]["score"]); omed = st.median(osc)
        P(f"## {a.new} vs {a.old} (same rig, same run)"); P("")
        P(f"- **Score:** {sum(1 for x in nsc if x > ob[1]['score'])} of {len(nsc)} {a.new} rows beat the best {a.old} row ({ob[0]} {ob[1]['score']:.1f}); "
          f"{sum(1 for x in nsc if x > omed)} of {len(nsc)} beat the {a.old} median ({omed:.1f}). Best {a.new}: {nb[0]} {nb[1]['score']:.1f}."); P("")
        P(f"| quality | best {a.old} | best {a.new} | {a.new} rows beating the {a.old} best |"); P("|---|---|---|---|")
        for q in Q:
            bo = max(old.items(), key=lambda kv: kv[1]["parts"][q]); bn = max(new.items(), key=lambda kv: kv[1]["parts"][q])
            beat = [k for k, r in sorted(new.items(), key=lambda kv: -kv[1]["parts"][q]) if r["parts"][q] > bo[1]["parts"][q]]
            P(f"| {q} | {bo[0]} ({bo[1]['parts'][q]}) | {bn[0]} ({bn[1]['parts'][q]}) | {', '.join(beat) if beat else 'none'} |")
    txt = "\n".join(out); print(txt)
    if a.md:
        open(a.md, "w").write(txt + "\n"); print(f"\n-> {a.md}")
    raise SystemExit(0)

names = sorted({os.path.basename(p)[len("homing_"):-len(".jsonl")] for p in glob.glob(f"{a.exam_dir}/homing_*.jsonl")} |
               {os.path.basename(p)[len("stairs_"):-len(".jsonl")] for p in glob.glob(f"{a.exam_dir}/stairs_*.jsonl")})
R = {}
for nm in names:
    S = load(f"{a.exam_dir}/stairs_{nm}.jsonl"); R[nm] = es.score_policy(load(f"{a.exam_dir}/homing_{nm}.jsonl"), S[-1] if S else None)
new = {k: v for k, v in R.items() if k.startswith(a.new)}; old = {k: v for k, v in R.items() if k.startswith(a.old)}

f = lambda v, p=3: "–" if v is None or (isinstance(v, float) and v != v) else f"{v:.{p}f}"
# key columns: (label, getter, better = "max" | "min" | "absmin" | "one" (closest to 1))
COLS = [("settled planar m", lambda r: r.get("planar"), "min"),
        ("stand after switch m", lambda r: r.get("stand_left"), "absmin"),
        ("stand-10 s drift m", lambda r: r.get("stand10_drift"), "min"),
        ("gain .05", lambda r: r["gains"].get("vx05"), "one"),
        ("gain .10", lambda r: r["gains"].get("vx10"), "one"),
        ("gain −.10", lambda r: r["gains"].get("vxm10"), "one"),
        ("gain ω +.8", lambda r: r["gains"].get("wzL08"), "one"),
        ("gain ω −.8", lambda r: r["gains"].get("wzR08"), "one"),
        ("pivot rad/s", lambda r: r.get("pivot05"), "min"),
        ("sep trend m", lambda r: r.get("stand10_sep_trend"), "absmin"),
        ("stagger m", lambda r: r.get("stand10_stagger_mean"), "absmin"),
        ("knee jitter", lambda r: r.get("stand10_jitter_knee"), "min")]
def key(kind, v):
    if v is None: return None
    return {"max": -v, "min": v, "absmin": abs(v), "one": abs(v - 1.0)}[kind]

out = []
P = lambda s="": out.append(s)
P(f"## Exam table — {a.new} ({len(new)} rows), same rig and run as {a.old} ({len(old)} rows) · {os.path.basename(a.exam_dir)}")
P("")
P("| # | policy | score | trk / plc / still / turn / surv | arrived | " + " | ".join(c[0] for c in COLS) + " |")
P("|" + "---|" * (5 + len(COLS)))
for i, (nm, r) in enumerate(sorted(new.items(), key=lambda kv: -kv[1]["score"]), 1):
    p = r["parts"]
    P(f"| {i} | {nm} | **{r['score']:.1f}** | {p['tracking']} / {p['placement']} / {p['stillness']} / {p['turn']} / {p['survival']} | {r['arrived']}/{r['n']} | " +
      " | ".join(f(c[1](r), 2 if 'gain' in c[0] or 'pivot' in c[0] else 3) for c in COLS) + " |")
P("")
osc = sorted((r["score"] for r in old.values()), reverse=True); nsc = sorted((r["score"] for r in new.values()), reverse=True)
ob = max(old.items(), key=lambda kv: kv[1]["score"]) if old else None; nb = max(new.items(), key=lambda kv: kv[1]["score"]) if new else None
omed = st.median(osc) if osc else None
P(f"## {a.new} vs {a.old} (same rig, same run)")
P("")
if ob and nb:
    P(f"- **Score:** {sum(1 for s in nsc if s > ob[1]['score'])} of {len(nsc)} {a.new} rows beat the best {a.old} row ({ob[0]} {ob[1]['score']:.1f}); "
      f"{sum(1 for s in nsc if s > omed)} of {len(nsc)} beat the {a.old} median ({omed:.1f}). Best {a.new}: {nb[0]} {nb[1]['score']:.1f}. "
      f"Medians: {a.new} {st.median(nsc):.1f}, {a.old} {omed:.1f}.")
P("")
P(f"| column | best {a.old} (value) | best {a.new} (value) | {a.new} rows beating the {a.old} best |")
P("|---|---|---|---|")
for lab, g, kind in COLS:
    ov = [(k, g(r)) for k, r in old.items() if g(r) is not None]; nv = [(k, g(r)) for k, r in new.items() if g(r) is not None]
    if not ov or not nv: continue
    bo = min(ov, key=lambda kv: key(kind, kv[1])); bn = min(nv, key=lambda kv: key(kind, kv[1]))
    beat = [k for k, v in sorted(nv, key=lambda kv: key(kind, kv[1])) if key(kind, v) < key(kind, bo[1])]
    p = 2 if 'gain' in lab or 'pivot' in lab else 3
    P(f"| {lab} | {bo[0]} ({f(bo[1], p)}) | {bn[0]} ({f(bn[1], p)}) | {', '.join(beat) if beat else 'none'} |")
P("")
P(f"## {a.old} on the same run (for reference)")
P("")
P("| # | policy | score | arrived | stand-10 s drift m | gain .10 | pivot rad/s |")
P("|---|---|---|---|---|---|---|")
for i, (nm, r) in enumerate(sorted(old.items(), key=lambda kv: -kv[1]["score"]), 1):
    P(f"| {i} | {nm} | {r['score']:.1f} | {r['arrived']}/{r['n']} | {f(r.get('stand10_drift'))} | {f(r['gains'].get('vx10'), 2)} | {f(r.get('pivot05'), 2)} |")
txt = "\n".join(out)
print(txt)
if a.md:
    open(a.md, "w").write(txt + "\n"); print(f"\n-> {a.md}")
