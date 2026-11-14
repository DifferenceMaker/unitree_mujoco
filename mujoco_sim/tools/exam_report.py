#!/usr/bin/env python3
"""exam_report.py <exam dir> [--exam 3|2|1 --seed S --n N --balance B --tag T --wall SECONDS --parallel P --rev R] -- the ranking table of one exam run.
(--rev, 2026-11-13: Exam #3 rev 2 -> "EXAM #3 rev 2" in the head, score_e3(rev=2); each policy's exam_<name>.json meta.exam_rev wins.
 Rev 3, 2026-11-14: the same with the rev-3 ladder -- vx to 0.80, wz +-0.60 -- and its rung counts in the text.)

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
ap.add_argument("exam_dir"); ap.add_argument("--exam", default="1"); ap.add_argument("--seed", default="?"); ap.add_argument("--n", default="?"); ap.add_argument("--balance", default="?")
ap.add_argument("--tag", default=""); ap.add_argument("--wall", type=int, default=0); ap.add_argument("--parallel", default="?")
ap.add_argument("--rev", default=None, help="Exam #3 revision exam.sh ran (2 = the repeated P5 stand, 2026-11-13); a policy's own record "
                "(<exam dir>/exam_<name>.json meta.exam_rev) wins; mixed revisions are named in the header")
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

E3 = a.exam == "3"   # EXAM #3 (2026-11-13) = Exam #2's tests + Test 6 (encoder offsets), scored by exam_score.score_e3
if a.exam in ("2", "3"):   # EXAM #2 (2026-10-30): score by quality (exam_score.score_v2), one row per policy
    names = sorted({os.path.basename(p)[len("ladders_"):-len(".jsonl")] for p in glob.glob(f"{EX}/ladders_*.jsonl")} | set(names))
    R = []
    REV = {}   # Exam #3 rev 2 (2026-11-13): per policy (rev, p5_stands) -- the record exam_record.py wrote says how it was examined, else --rev

    def rev_of(nm):
        try:
            m = json.load(open(f"{EX}/exam_{nm}.json")).get("meta") or {}
        except (OSError, ValueError, AttributeError):
            m = {}
        if int(m.get("exam_version") or 0) == 3:
            return int(m.get("exam_rev") or 1), (m.get("det") or {}).get("p5_stands")
        return (int(a.rev) if a.rev else 1), None
    for nm in names:
        H = load(f"{EX}/homing_{nm}.jsonl"); L = load(f"{EX}/ladders_{nm}.jsonl"); P = load(f"{EX}/pushes_{nm}.jsonl"); HO = load(f"{EX}/handover_{nm}.jsonl")
        U = load(f"{EX}/stumble_{nm}.jsonl")   # Test 5 (2026-11-02): key columns only, not scored
        sc = exam_score.score_v4(HO, L[-1] if L else None, P[-1] if P else None, H, n_homing=int(a.n) if a.n != "?" else 10)   # v4 (2026-11-09)
        if E3:
            EN = load(f"{EX}/enc_{nm}.jsonl")
            REV[nm] = rev_of(nm)
            if REV[nm][0] >= 2:   # rev 2: the P5 rows read the worst of the repeated P5 stands (score version 6); rev 3: + the rev-3 ladder (version 7)
                sc = exam_score.score_e3(HO, L[-1] if L else None, P[-1] if P else None, H, enc=EN[-1] if EN else None, n_homing=int(a.n) if a.n != "?" else 10,
                                         rev=REV[nm][0], p5_stands=REV[nm][1])
            else:
                sc = exam_score.score_e3(HO, L[-1] if L else None, P[-1] if P else None, H, enc=EN[-1] if EN else None, n_homing=int(a.n) if a.n != "?" else 10)
        m = {x["key"]: x for x in sc["metrics"]}
        R.append((nm, sc, m, exam_score.ladder_swing(L[-1] if L else None), exam_score.stumble_summary(U[-1] if U else None),
                  exam_score.ladder_scuffs(L[-1] if L else None),
                  {s_["name"]: s_.get("sep_dir") for s_ in ((L[-1] if L else {}) or {}).get("segments", []) if s_.get("name") in ("stand", "walk_stand")}))   # the stands' sep direction (per policy; a loop variable leaked the last policy's on 2026-11-03)
    R.sort(key=lambda t: -t[1]["score"])
    Q = list(exam_score.QUALITIES)
    cm = lambda a: "—" if not a else "/".join("—" if v is None else f"{100 * v:.1f}" for v in a)   # [L, R] m -> "L/R" cm
    def stu(su, rung):
        """Test 5 column: 'apexL/apexR cm · trips · tilt' for one rung, '—' when the record / rung is missing"""
        r = (su or {}).get("rungs", {}).get(rung) if su else None
        if not r: return "—"
        return f"{cm(r['apex'])} · {r['trips']}{'' if not r['fell'] else ' FELL'} · {r['tilt_max'] if r['tilt_max'] is not None else '—'}"
    hdr = ("| # | policy | score | " + " | ".join(f"{q} /{exam_score.QUALITIES[q]}" for q in Q) + " | feasible (failed constraints) | falls (hard) | track vx/vy/wz | pushes held | homing arrived | stand drift m "
           "| stand yaw° / sep dev max·end cm (dir) / stag dev max·end cm | stand after walking: yaw° / sep dev max·end cm (dir) / stag dev max·end cm | z stand m / drop walk·push cm | tilt peak° | self-contact s | arms dev mean·max° "
           "| swing .15 L/R cm | swing .30 L/R cm | scuffs .15 | scuffs .30 | stumble .15: swing L/R cm · trips · tilt° | stumble .30 | stumble falls | t s |")
    lines = [hdr, "|" + "---|" * (len(Q) + 23)]
    def feas(sc):   # v4 constraints: "yes" or "NO: walks_wz 0.62 < 0.9, ..." (p = provisional threshold)
        if not sc.get("constraints"): return "—"
        bad = []
        for c in sc["constraints"]:
            if c["pass"]: continue
            val = "—" if c["value"] is None else f"{c['value']:.3g}"
            rel = "<" if c["sense"] == ">=" else ">"
            bad.append(f"{c['name']} {val} {rel} {c['threshold']:g}" + (" p" if c["provisional"] else ""))
        return "yes" if not bad else "NO: " + ", ".join(bad)
    v = lambda m, k, p=1, f=100.0: "—" if k not in m or m[k]["value"] is None else f"{m[k]['value'] * f:.{p}f}"
    for i, (nm, sc, m, lsw, su, lsc, sdir) in enumerate(R, 1):
        held = m["pushes"]["label"].split("held ")[1].split(":")[0] if "pushes" in m else "-"
        arr = m["homing"]["label"].split("arrived ")[1].split(":")[0] if "homing" in m else "-"
        trk = "/".join(f"{m[k]['s']:.2f}" for k in ("track_vx", "track_vy", "track_wz"))
        inf = (sc.get("v3") or {}).get("info") or {}; nh = len((sc.get("v3") or {}).get("hard") or [])
        tp = inf.get("tilt_peak") or {}
        zs = ("—" if inf.get("z_stand") is None else f"{inf['z_stand']:.3f}") + " / " + ("—" if inf.get("z_drop_walk") is None else f"{100 * inf['z_drop_walk']:.1f}") + "·" + ("—" if inf.get("z_drop_push") is None else f"{100 * inf['z_drop_push']:.1f}")
        lines.append(f"| {i} | {nm} | **{sc['score']}** | " + " | ".join(str(sc["parts"][q]) for q in Q) +
                     f" | {feas(sc)} | {len(sc['falls'])} ({nh}) | {trk} | {held} | {arr} | {m['stand_drift']['value']} "
                     f"| {v(m, 'stand_yaw', 1, 1.0)} / {v(m, 'stand_sep_dev_max')}·{v(m, 'stand_sep_dev_end')} ({sdir.get('stand') or '—'}) / {v(m, 'stand_stag_dev_max')}·{v(m, 'stand_stag_dev_end')} "
                     f"| {v(m, 'wstand_yaw', 1, 1.0)} / {v(m, 'wstand_sep_dev_max')}·{v(m, 'wstand_sep_dev_end')} ({sdir.get('walk_stand') or '—'}) / {v(m, 'wstand_stag_dev_max')}·{v(m, 'wstand_stag_dev_end')} | {zs} "
                     f"| {max(tp.values()) if tp else '—'} | {v(m, 'sc_dur', 2, 1.0)} | {v(m, 'arm_dev_mean', 1, 1.0)}·{v(m, 'arm_dev_max', 1, 1.0)} | {cm(lsw.get('vx+0.15'))} | {cm(lsw.get('vx+0.30'))} "
                     f"| {exam_score.scuff_cell(lsc.get('vx+0.15'))} | {exam_score.scuff_cell(lsc.get('vx+0.30'))} "
                     f"| {stu(su, 'vx+0.15')} | {stu(su, 'vx+0.30')} | {su['falls'] if su else '—'} | {times.get(nm) or '-'} |")
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    ff = os.environ.get("EXAM_FF", "1")
    revlab = ""   # Exam #3 rev 2 (2026-11-13): "EXAM #3 rev 2"; rev 1 without --rev = the header as it always was; mixed revisions said loudly
    revs = sorted({rv for rv, _ in REV.values()}) if E3 else []
    if len(revs) > 1:
        revlab = " MIXED REVISIONS (" + "; ".join(f"rev {r_}: " + ", ".join(n_ for n_, (rv, _) in sorted(REV.items()) if rv == r_) for r_ in revs) + \
                 " -- scores of different revisions do not compare)"
    elif revs and (revs[0] >= 2 or a.rev):
        revlab = f" rev {revs[0]}"
    if E3 and a.rev and revs and revs != [int(a.rev)]:
        revlab += f" (exam.sh said --rev {a.rev}; the records say rev {'/'.join(str(r_) for r_ in revs)})"
    head = (f"## EXAM #{a.exam}{revlab} {a.tag} — {stamp} — {len(R)} policies, seed {a.seed}, homing n {a.n}, balance {a.balance}, {a.parallel} instances, "
            f"wall {a.wall // 60} min {a.wall % 60} s, arm ff {ff}\n\nScore by quality (tools/exam_score.py score_v4 since 2026-11-09 = v3 + rung error e = |v - c| / max(|c|, 0.05) + 48 pushes "
            f"(8 directions) + calibration {exam_score.load_cal_v4()[1]}; 'feasible' = every v4 constraint passes (no_falls, walks_vx/vy/wz >= 90 % of the rungs from 0.10 m/s / 0.2 rad/s at gain >= 0.5, "
            f"track_floor_* and tilt_floor = provisional 'p'); v3: hard fails = height / tilt {exam_score.TILT_FAIL_DEG:g} deg / "
            f"floor contact 20 N / arm envelope count as falls, 'falls (hard)' = all falls (of them hard); the 60 s clean-engagement stand; results/EXAM2_SPEC.md v3 + v4): survival 10, "
            f"stability 30, tracking 20, gait 10, energy 10, pushes 10, homing 10; every metric a ramp between its good and bad value. "
            f"Track = mean rung score per ladder. Swing = the flat ladder's peak foot lift per foot at the 0.15 / 0.30 rungs. Scuffs = the flat ladder's "
            f"scuffed swings / swings (L+R) at 0.15 / 0.30 (a floor-force burst > 20 N between two airborne 20 ms samples = the toe dragged; '—' = a record "
            f"before 2026-11-02 evening). Stumble = Test 5 (rough floor: 0.4-1 cm sphere bumps every 20 cm, seed {a.seed}, NOT scored): ground-corrected swing "
            f"apex L/R, trips (scuffs + catches), peak torso tilt per rung. Records: {EX}\n")
    block = head + "\n" + "\n".join(lines) + "\n"
    if E3:   # Exam #3 (2026-11-13): what the table above means now + Test 6's own table (the v4 columns above keep their places)
        g = lambda x, p=2: "—" if x is None else f"{x:.{p}f}"
        w_ = lambda es, k, p=2: "—" if not (es.get("worst") or {}).get(k) else f"{es['worst'][k][0]:.{p}f} ({es['worst'][k][1]})"
        mrow = lambda sc, k: next((m for m in sc["metrics"] if m["key"] == k), {})
        el = ["| policy | Exam #3 | same run, v4 rules | Test 6 | injection (max err mrad) | falls in patterns / offset pushes not held | P5 stand: drift cm · lifts · rate rad/s "
              "| worst in-band stand drift cm | walk tilt change° (worst P1..P5) | impact ΔBW · roughness Δrad · power ΔW (walk, vs P0) | offset pushes: score · held · energy J (Test 2 same pushes: score · held) "
              "| Test 6 points stab/gait/energy/push | low rungs passed vx · vy · wz (information) | bias (recorded): worst lean° · walk vy mm/s · P5 lean° | Exam #3 constraints |",
              "|" + "---|" * 15]
        R2 = any(rv >= 2 for rv, _ in REV.values())   # rev 2: the P5 stand cell = the worst of the N stands + each stand in brackets
        R3 = any(rv >= 3 for rv, _ in REV.values())   # rev 3 (2026-11-14): the shorter ladder -> its rung counts in the text below
        nr = " / ".join(f"{ax} {n_}" for ax, n_ in exam_score.expected_rungs(exam_score.ladder_tops(3)).items()) if R3 else "30 / 12 / 32"
        if R2: el[0] = el[0].replace("| P5 stand: drift cm · lifts · rate rad/s ", "| P5 stand (rev 2: the WORST of the N stands; each stand drift · lifts in brackets): drift cm · lifts · rate rad/s ")
        for nm, sc, *_ in R:
            es = sc.get("enc") or {}
            wl = (sc.get("e3") or {}).get("walks_low")
            wls = "—" if not wl else " · ".join(f"{wl[ax][0]}/{wl[ax][1]}" for ax in ("vx", "vy", "wz")) + f" = {wl['share']:.2f}"
            ec = [c for c in sc["constraints"] if c["name"].startswith(("enc_", "walks_")) or c["name"] == "stand_quiet"]
            bad = [f"{c['name']} {'—' if c['value'] is None else format(c['value'], '.3g')} {'<' if c['sense'] == '>=' else '>'} {c['threshold']:g}" + (" p" if c["provisional"] else "") for c in ec if not c["pass"]]
            cons = "pass" if not bad else "FAIL: " + ", ".join(bad)
            if not es.get("ran"):
                el.append(f"| {nm} | **{sc['score']}** | {sc.get('score_v4')} | {es.get('result', 'MISSING')} |" + " — |" * 8 + f" {wls} | — | {cons} |"); continue
            X = sc["e3"]["values"]; inj = es["injection"]; sk = es.get("stress")
            op = es.get("pushes") or []; t2 = [p["t2"] for p in op if p.get("t2")]
            pts = {q: sum(m["pts"] for m in sc["metrics"] if m["test"] == "enc" and m["quality"] == q) for q in ("stability", "gait", "energy", "pushes")}
            d5 = (es["patterns"].get(sk) or {}).get("d") or {}
            S5 = (es["patterns"].get(sk) or {}).get("stands") or []   # rev 2: each P5 stand's own drift / lifts ('✗' = fell / hard fail)
            p5e = "" if not S5 else (" (" + " / ".join("✗" if not s_.get("measured") else g(100 * s_["drift_m"], 1) for s_ in S5) + " cm · "
                                     + " / ".join(g(s_.get("lifts"), 0) for s_ in S5) + ")")
            el.append(f"| {nm} | **{sc['score']}** | {sc.get('score_v4')} | {es['result']} | {'ok' if inj['ok'] else 'NOT VERIFIED'} ({1000 * inj['max_err']:.2f}) | {es['falls']} / {es['push_falls']} "
                      f"| {g(None if X['enc_stand_drift'] is None else 100 * X['enc_stand_drift'], 1)} · {g(X['enc_stand_lifts'], 0)} · {g(X['enc_stand_rate'], 3)}{p5e} "
                      f"| {g(None if X['enc_creep'] is None else 100 * X['enc_creep'], 1)} | {g(X['enc_walk_tilt'])} | {g(X['enc_impact'], 3)} · {g(X['enc_smooth'], 4)} · {g(X['enc_power'], 1)} "
                      f"| {g(mrow(sc, 'enc_pushes').get('value'), 3)} · {sum(1 for p in op if p['held'])}/{len(op)} · {g(X['enc_push_energy'], 0)} ({g(mean([t['score'] for t in t2]), 3)} · {sum(1 for t in t2 if t['held'])}/{len(t2)}) "
                      f"| {pts['stability']:.2f} / {pts['gait']:.2f} / {pts['energy']:.2f} / {pts['pushes']:.2f} | {wls} "
                      f"| {w_(es, 'lean')} · {w_(es, 'walk_vy', 1)} · {g(d5.get('lean'))} | {cons} |")
        block += ("\nEXAM #3 (tools/exam_score.py score_e3, results/EXAM2_SPEC.md section 13) = the v4 rows above re-weighted inside the qualities (stability x 25/30, gait x 8/10, energy x 8/10, "
                  "the 48 pushes 8.57 of the 10 push points) + Test 6 — encoder offsets (tools/enc_patterns.py: reported joint angle = true + offset in the sim; P0 zero, P1..P4 inside the trained "
                  "+-0.010 rad band, P5 = the measured residuals unscaled = stress; per pattern a 15 s stand + a 15 s measured walk at vx 0.30, then 8 offset pushes at 1.0 m/s under P5): stability 5 "
                  "(P5 stand drift 1.5, lifts 1, torso rate 0.5, worst in-band stand drift 1, walk tilt change 1), gait 2 (impact, roughness change), energy 2 (offset-push recovery, walking power change), "
                  "pushes 1.43 (the 8 offset pushes, the Test 2 formula); a fall under a pattern or an offset push not held halves survival like any fall. 'same run, v4 rules' = score_v4 of the same record "
                  f"(Exam #3 and Exam #2 scores do not compare). Constraints: walks_vx / vy / wz count EVERY ladder rung ({nr}) at gain >= 0.5, share >= {exam_score.WALK_SHARE:g} (v4 counted from 0.10 m/s / 0.2 rad/s); added: stand_quiet (foot lifts 60 s stand + 120 s stand after walking <= {exam_score.STAND_QUIET_LIFTS:g} p), "
                  f"enc_no_falls, enc_stand_drift <= {exam_score.ENC_STAND_DRIFT_CM:g} cm p, enc_stand_lifts <= {exam_score.ENC_STAND_LIFTS:g} p (p = provisional); the 'feasible' column above includes them. "
                  "The bias numbers (lean, drift vs P0) are recorded, not scored."
                  + (" REV 2 (2026-11-13, score version 6): the P5 stand is entered 3 times (stand; then 8 s walking at 0.30 on perfect encoders, P5 lands again, stand2; "
                     "the same for stand3) and the three P5 stand rows + enc_stand_drift / enc_stand_lifts read the WORST of them per metric (drift, lifts, rate = the max); "
                     "a fall / hard fail in any repeated segment is a P5 fall; every test starts from the standing snapshot; Test 3 starts = the fixed table "
                     "(tools/homing_starts.py). Weights and ramps as rev 1." if R2 else "")
                  + (" REV 3 (2026-11-14, score version 7): rev 2 on the ladder of the trimmed training ranges, vx -0.50..+0.80 and wz +-0.60 "
                     f"(tools/exam_score.py E3_REV3_LADDER_TOPS; vy +-0.30): {nr} rungs; walks_* / track_* / track_floor_* count these rungs, "
                     "track_floor_wz08 reads the wz tops +-0.60; rev-3 scores do not compare with rev 2." if R3 else "")
                  + "\n\n" + "\n".join(el) + "\n")
    print(block)
    open(f"{EX}/EXAM.md", "w").write(block)
    with open(f"{SIM}/results/EXAM.md", "a") as fh:
        fh.write(block + "\n")
    print(f"-> {EX}/EXAM.md and {SIM}/results/EXAM.md")
    sys.exit(0)

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
