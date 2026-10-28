#!/usr/bin/env bash
# staircase.sh -- Test 1 of the standardized walk exam (2026-10-28): the fixed command script on the running homing rig.
#   bash mujoco_sim/tools/staircase.sh [--start "-3 0 0"] [--timeout 200]
# The commander teleports the robot to the start on a planned sim frame, drives the segments (stand 10 s; vx 0.05 / 0.10 / 0.15 / -0.10;
# vy +-0.10; wz +-0.3 / +-0.5, each with a 2 s stand between) and appends ONE record to logs/staircase[$SFX].jsonl: per segment the
# sim-truth gain (achieved / commanded), the stand drift, the yaw change, foot lifts, the pivot index (|foot yaw rate| while loaded)
# and the swing height. Same LOCOSW_INSTANCE as the rig. Prints the per-segment table.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START=""; TO=240
while [[ $# -gt 0 ]]; do case "$1" in --start) START="$2"; shift 2;; --timeout) TO="$2"; shift 2;; *) echo "unknown arg: $1"; exit 1;; esac; done
SFX="${LOCOSW_INSTANCE:+_$LOCOSW_INSTANCE}"
CMD="$SIM/logs/.homing_cmd$SFX"; LOG="$SIM/logs/stack_locosw_latest$SFX.log"; JL="$SIM/logs/staircase$SFX.jsonl"
for _i in $(seq 1 120); do grep -q "\[homing\] pose=" "$LOG" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
touch "$JL"; n0=$(wc -l < "$JL")
echo "stairs $START" | sed 's/ *$//' >> "$CMD"
for _i in $(seq 1 $((TO * 2))); do [[ "$(wc -l < "$JL")" -gt "$n0" ]] && break; sleep 0.5; done
[[ "$(wc -l < "$JL")" -gt "$n0" ]] || { echo "!! no staircase record within $TO s"; echo "stop" >> "$CMD"; exit 3; }
python3 "$SIM/tools/staircase_print.py" "$JL" -1
