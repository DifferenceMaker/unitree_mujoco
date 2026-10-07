#!/usr/bin/env python3
"""stop_test_read.py -- read a `tau_monitor.py --legs --angles --log FILE` recording of the STOP TEST and print, per leg joint,
every plateau (the joint held still at an extreme for >= --hold seconds) with its time, angle and distance to the URDF limit,
then pair LEFT and RIGHT at the same stop: for roll/yaw joints the mirrored readings must SUM to ~0, for pitch joints they must
be EQUAL. A non-zero pair residual is the encoder zero offset between the two sides (the mechanical stops are built identical).

The joint is identified from the data (the one that moves), the direction from the sign; the operator's order only labels.
Usage: python3 stop_test_read.py ~/legs_2026-10-08_stoptest.log [--hold 3] [--tol 0.3] [--min-frac 0.5]
  --hold     seconds a joint must sit still to count as a plateau (default 3)
  --tol      stillness tolerance in degrees (default 0.3)
  --min-frac only report plateaus beyond this fraction of the URDF limit (default 0.5; the hanging rest pose is ~0)
"""
import argparse, re, sys, collections
# URDF limits (deg) of h1_2_comx06_hand790.urdf -- lower/upper per side; None = free (no stop in the model)
LIM = {("hipR", "L"): (-24.64, None), ("hipR", "R"): (None, 24.64),      # inward: L negative, R positive
       ("hipY", "L"): (-24.64, 24.64), ("hipY", "R"): (-24.64, 24.64),
       ("ankR", "L"): (-15.0, 15.0), ("ankR", "R"): (-15.0, 15.0),
       ("ankP", "L"): (-51.41, 30.0), ("ankP", "R"): (-51.41, 30.0),
       ("hipP", "L"): (None, None), ("hipP", "R"): (None, None), ("knee", "L"): (-14.90, 117.46), ("knee", "R"): (-14.90, 117.46)}
MIRROR = {"hipR": -1, "hipY": -1, "ankR": -1, "ankP": +1, "hipP": +1, "knee": +1}   # L and R at the same physical stop: sum (-1) or equal (+1)
ap = argparse.ArgumentParser(); ap.add_argument("log"); ap.add_argument("--hold", type=float, default=3.0); ap.add_argument("--tol", type=float, default=0.3); ap.add_argument("--min-frac", type=float, default=0.5)
a = ap.parse_args()
tpat = re.compile(r"^(\d\d:\d\d:\d\d)"); lpat = re.compile(r"^\s*([LR]) \| (.*)$"); jpat = re.compile(r"(\w+) cmd\s+\S+\s+q\s+([-+]?\d+\.\d+)\s+err\s+\S+\s+tau\s+([-+]?\d+\.\d+|-)")
series = collections.defaultdict(list)   # (joint, side) -> [(t, q)]
t = None
for line in open(a.log, errors="replace"):
    m = tpat.match(line)
    if m: t = m.group(1); continue
    m = lpat.match(line)
    if not m or t is None: continue
    side = m.group(1)
    for j, q, tau in jpat.findall(m.group(2)):
        series[(j, side)].append((t, float(q)))
        if j == "knee" and tau != "-": series[("kneeTau", side)].append((t, abs(float(tau))))
if not series: sys.exit("no leg lines found (run tau_monitor with --legs --angles)")
def plateaus(rows):
    out, i = [], 0
    while i < len(rows):
        k = i
        while k + 1 < len(rows) and abs(rows[k + 1][1] - rows[i][1]) <= a.tol: k += 1
        n = k - i + 1
        if n >= a.hold: out.append((rows[i][0], rows[k][0], n, sum(r[1] for r in rows[i:k + 1]) / n))
        i = k + 1
    return out
found = {}
print(f"{'joint':5s} {'side':4s} {'from':8s} {'to':8s} {'s':>3s} {'angle':>8s}  {'URDF stop':>9s} {'to stop':>8s}")
for (j, side), rows in sorted(series.items()):
    lo, hi = LIM[(j, side)]
    for t0, t1, n, q in plateaus(rows):
        lim = None
        if q < 0 and lo is not None: lim = lo
        if q > 0 and hi is not None: lim = hi
        frac = abs(q) / abs(lim) if lim else 0.0
        if lim is None or frac < a.min_frac: continue
        print(f"{j:5s} {side:4s} {t0:8s} {t1:8s} {n:3d} {q:+8.2f}  {lim:+9.2f} {q - lim:+8.2f}")
        key = (j, "neg" if q < 0 else "pos")
        found.setdefault((j, side, key[1]), []).append(q)
print("\nPAIRS (same physical stop, L vs R):")
for j, sgn in sorted({(j, s) for (j, side, s) in found}):
    # mirrored joints: L at +x pairs with R at -x; pitch joints: same sign
    sL = found.get((j, "L", sgn)); sR = found.get((j, "R", sgn if MIRROR[j] > 0 else ("neg" if sgn == "pos" else "pos")))
    if not sL or not sR: print(f"  {j:5s} {sgn}: only one side reached this stop (L {sL} / R {sR})"); continue
    qL, qR = sum(sL) / len(sL), sum(sR) / len(sR)
    resid = qL + qR if MIRROR[j] < 0 else qL - qR
    verdict = "PASS" if abs(resid) <= 0.5 else "OFFSET"
    print(f"  {j:5s} {sgn}: L {qL:+7.2f}  R {qR:+7.2f}  {'sum' if MIRROR[j] < 0 else 'diff'} {resid:+6.2f} deg  -> {verdict} (pass = within 0.5)")
# FEET FLAT = the final 10 s; valid as an absolute ankle reference ONLY with the weight on the legs (knee |tau| > 20 Nm both sides)
last = sorted(set(t for t, _ in series[("hipR", "L")]))[-10:]
def wmean(k): v = [q for t, q in series[k] if t in last]; return sum(v) / len(v) if v else float("nan")
tL, tR = wmean(("kneeTau", "L")), wmean(("kneeTau", "R"))
loaded = tL > 20 and tR > 20
print(f"\nFEET FLAT (final 10 s {last[0]}..{last[-1]}): knee |tau| L {tL:.1f} / R {tR:.1f} Nm -> {'LOADED, usable as the absolute ankle reference' if loaded else 'UNLOADED (weight on the harness) -> NOT an absolute reference; redo with knee |tau| > 20 Nm'}")
for j in ["hipR", "ankR", "ankP", "knee", "hipY"]:
    L, R = wmean((j, "L")), wmean((j, "R")); print(f"  {j:5s} L {L:+7.2f} R {R:+7.2f}  {'sum' if MIRROR[j] < 0 else 'diff'} {(L + R) if MIRROR[j] < 0 else (L - R):+6.2f}")
