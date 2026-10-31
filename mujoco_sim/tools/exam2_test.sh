#!/usr/bin/env bash
# exam2_test.sh -- exam #2 Tests 1 and 2 (2026-10-30, results/EXAM2_SPEC.md) on the running homing rig.
#   bash mujoco_sim/tools/exam2_test.sh ladders [--start "-3 0 180"] [--timeout 900]
#   bash mujoco_sim/tools/exam2_test.sh pushes  [--start "-3 0 180"] [--timeout 600]
# ladders: teleport, a 20 s stand (stance + the joint state for the stance drawing), then per axis a continuous ladder of 0.05 rungs
#   (4 s each, the last 2.5 s measured) up to the top, a 4 s stop, the same ladder backwards, a 4 s stop: vx 0.05..1.00 / -0.05..-0.50,
#   vy +-0.05..0.30, wz +-0.05..0.80 (340 s of sim). Record -> logs/staircase[$SFX].jsonl (kind "ladders").
# pushes: teleport, 3 s settle, 12 pushes standing (2 s before each) then 12 walking at vx 0.3 (3 s before each): velocity kicks of
#   0.5 / 1.0 / 1.5 m/s front / back / left / right, 5 s measured after each (183 s). Record -> logs/pushes[$SFX].jsonl.
# A fall: recorded, the robot reset on the spot (standing snapshot), the script continues (ladders: the next half; pushes: the next push).
# Start default "-3 0 180": free floor west of the desk, facing away from it (the vx ladder walks ~31 m net to -x).
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHAT="${1:-}"; shift || true
[[ "$WHAT" == ladders || "$WHAT" == pushes ]] || { sed -n '2,12p' "$0"; exit 1; }
START="-3 0 180"; TO=$([[ "$WHAT" == ladders ]] && echo 900 || echo 600)
while [[ $# -gt 0 ]]; do case "$1" in --start) START="$2"; shift 2;; --timeout) TO="$2"; shift 2;; *) echo "unknown arg: $1"; exit 1;; esac; done
SFX="${LOCOSW_INSTANCE:+_$LOCOSW_INSTANCE}"
CMD="$SIM/logs/.homing_cmd$SFX"; LOG="$SIM/logs/stack_locosw_latest$SFX.log"
JL="$SIM/logs/$([[ "$WHAT" == ladders ]] && echo staircase || echo pushes)$SFX.jsonl"
# the commander of THIS run must be up (a stale "latest" symlink fooled the hand-run smokes on 2026-10-29)
for _i in $(seq 1 240); do L2="$(readlink -f "$LOG" 2>/dev/null)"; [[ -n "$L2" && "$L2" -nt "$CMD" ]] && grep -q "\[homing\] pose=" "$L2" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
touch "$JL"; n0=$(wc -l < "$JL")
echo "$WHAT $START" >> "$CMD"
for _i in $(seq 1 $((TO * 2))); do [[ "$(wc -l < "$JL")" -gt "$n0" ]] && break; sleep 0.5; done
[[ "$(wc -l < "$JL")" -gt "$n0" ]] || { echo "!! no $WHAT record within $TO s"; echo "stop" >> "$CMD"; exit 3; }
python3 - "$JL" <<'PY'
import json, sys
r = [json.loads(l) for l in open(sys.argv[1]) if l.strip()][-1]
S = [s for s in r["segments"] if s.get("name") not in ("reset",)]
print(f"{r['kind'].upper()} {r['result']}  t {r['t_total']} s  falls {len(r.get('falls') or [])} {[(f['seg'], f['why']) for f in r.get('falls') or []]}")
for s in S:
    v, e = s.get("v_true") or [None] * 3, s.get("energy") or {}
    extra = ""
    if s.get("stop"): extra = f"  stop: settle {s['stop']['settle_s']} s, overshoot {s['stop']['overshoot_m']} m / {s['stop']['overshoot_deg']} deg, steps {s['stop']['steps']}"
    if s.get("push"): p = s["push"]; extra = f"  held {p['held']} dist {p['dist_m']} m tilt {p['tilt_peak_deg']} deg rec {p['recovery_s']} s steps {p['steps']} foot {p['foot_peak_bw']} BW"
    vs = "" if v[0] is None else f"v ({v[0]:+.3f} {v[1]:+.3f} {v[2]:+.3f})"
    print(f"  {s['name']:22s} cmd {s.get('cmd')} {vs}  tilt {s.get('tilt_mean')} P {e.get('P_W')} W  lifts {s.get('lifts')}{extra}{'  FELL' if s.get('fell') else ''}")
PY
