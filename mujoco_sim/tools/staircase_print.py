#!/usr/bin/env python3
"""staircase_print.py <staircase.jsonl> [-1] -- the per-segment table of the last (or every) staircase record."""
import json, sys

path = sys.argv[1]; last_only = "-1" in sys.argv[2:] or len(sys.argv) == 2
recs = [json.loads(l) for l in open(path) if l.strip()]
if last_only: recs = recs[-1:]
for r in recs:
    print(f"STAIRCASE {r['result']}  walk={r['walk']}  balance={r.get('balance')}  start={r.get('start')}  t={r['t_total']} s  fell={r.get('fell')}")
    print(f"{'seg':>6} {'cmd (vx vy wz)':>18} {'gain':>6} {'v_true (vx vy wz)':>24} {'drift':>6} {'fwd/left':>14} {'yaw_chg':>7} {'lifts':>7} {'pivot':>6} {'swing L/R':>11} {'z_min':>5}")
    for s in r["segments"]:
        g = s.get("gain") or {}; gv = next(iter(g.values()), None)
        vt = s.get("v_true") or [float("nan")] * 3; db = s.get("dx_body") or [float("nan")] * 2; sw = s.get("swing_height") or [0, 0]
        piv = s.get("pivot_rad_s")
        print(f"{s['name']:>6} {str(s['cmd']):>18} {('%+.2f' % gv) if gv is not None else '-':>6} {'%+.3f %+.3f %+.3f' % tuple(vt):>24} "
              f"{s.get('drift_xy', float('nan')):>6.3f} {'%+.3f/%+.3f' % tuple(db):>14} {s.get('yaw_change_deg', float('nan')):>7.1f} {str(s['lifts']):>7} "
              f"{('%.2f' % piv) if piv is not None else '-':>6} {'%.2f/%.2f' % tuple(sw):>11} {s.get('z_min', 0):>5.2f}")
