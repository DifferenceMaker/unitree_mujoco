#!/usr/bin/env bash
# handover.sh -- Test 0 of the standardized walk exam (2026-10-29): the HANDOVER from the balance policy, on the running homing rig.
#   bash mujoco_sim/tools/handover.sh [--start "-3 0 0"] [--poses "default right_up_left_side both_forward"] [--timeout 300]
# Per arm pose the commander: switches to the balance policy, teleports the robot to the start, lets the balance stand 10 s holding the
# arm pose (ramped over 2 s), switches to the walk at a zero command and measures 8 s, switches back to the balance and measures 10 s.
# One record per pose in logs/handover[$SFX].jsonl (segments bal_stand / walk_after / bal_after with the staircase's metrics: lifts,
# drift, body-frame displacement, yaw change, z_min, feet separation / stagger / toe / jitter). Operator 2026-10-29: the exam must start
# where the robot failed on the hardware, with the arms in manipulation poses ("right arm up, left arm at the side").
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START="-3 0 0"; POSES="default right_up_left_side both_forward"; TO=300
while [[ $# -gt 0 ]]; do case "$1" in --start) START="$2"; shift 2;; --poses) POSES="$2"; shift 2;; --timeout) TO="$2"; shift 2;; *) echo "unknown arg: $1"; exit 1;; esac; done
SFX="${LOCOSW_INSTANCE:+_$LOCOSW_INSTANCE}"
CMD="$SIM/logs/.homing_cmd$SFX"; LOG="$SIM/logs/stack_locosw_latest$SFX.log"; JL="$SIM/logs/handover$SFX.jsonl"
for _i in $(seq 1 240); do L2="$(readlink -f "$LOG" 2>/dev/null)"; [[ -n "$L2" && "$L2" -nt "$CMD" ]] && grep -q "\[homing\] pose=" "$L2" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
touch "$JL"; n0=$(wc -l < "$JL"); np=$(wc -w <<< "$POSES")
for p in $POSES; do echo "handover $START $p" >> "$CMD"; done
for _i in $(seq 1 $((TO * 2))); do [[ "$(wc -l < "$JL")" -ge $((n0 + np)) ]] && break; sleep 0.5; done
got=$(( $(wc -l < "$JL") - n0 ))
[[ $got -ge $np ]] || { echo "!! $got of $np handover records within $TO s"; echo "stop" >> "$CMD"; }
python3 - "$JL" "$got" <<'PY'
import json, sys
L = [json.loads(l) for l in open(sys.argv[1]) if l.strip()][-int(sys.argv[2]):] if int(sys.argv[2]) > 0 else []
for r in L:
    print(f"HANDOVER {r['pose']:20s} {r['result']:24s} walk {r['walk']}  switch wait {r.get('switch_wait_s')}")
    for s in r["segments"]:
        print(f"   {s['name']:10s} lifts {s.get('lifts')}  drift {s.get('drift_xy')} m  dx_body {s.get('dx_body')}  yaw {s.get('yaw_change_deg')} deg  z_min {s.get('z_min')}"
              f"  sep {s.get('sep_mean')} (trend {s.get('sep_trend')})  stagger {s.get('stagger_mean')}  knee jitter {s.get('jitter_knee')}")
PY
