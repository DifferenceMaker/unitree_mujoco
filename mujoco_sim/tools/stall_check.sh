#!/usr/bin/env bash
# stall_check.sh <instance> <walk policy> [repeats] -- the lock-step stall check (2026-10-31): a homing rig on LOCOSW_INSTANCE=<instance>
# runs Test 0 (tools/handover.sh, 3 arm poses) <repeats> times and reports the Bridge's missed policy frames (each = a 120 s wait)
# and the joint_cmd_q evictions (the 2026-10-31 fix: a full queue evicts the oldest item instead of dropping MovementModule's
# stamped command). No exam record is written.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K="$1"; WALK="$2"; REP="${3:-1}"
export LOCOSW_INSTANCE="$K"
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
link0="$(readlink "$SIM/logs/stack_locosw_latest_$K.log" 2>/dev/null)"
(cd "$SIM" && HOMING_WALK="$WALK" HOMING_BALANCE=p14h_stancewide HOMING_STOP_DIST=0.25 LOCOSW_LOCKSTEP=1 HOMING_SEED=1 METRICS=0 \
  bash run_mujoco_homing.sh > "$SIM/logs/stall_check_rig_$K.log" 2>&1 &)
for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$K.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
L="$SIM/logs/stack_locosw_latest_$K.log"
for i in $(seq 1 300); do grep -q "\[homing\] pose=" "$L" 2>/dev/null && break; sleep 1; done
t0=$(date +%s)
for r in $(seq 1 "$REP"); do bash "$SIM/tools/handover.sh" --timeout 900 > "$SIM/logs/stall_check_handover_${K}_$r.log" 2>&1; done
t1=$(date +%s)
M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$K".log | head -1)
echo "instance $K walk $WALK: $REP x Test 0 in $((t1 - t0)) s wall | missed: $(grep -c 'no MovementModule command for policy frame' "$L") | evicted: $(grep -o 'evicted the oldest item ([0-9]* so far)' "$L" | tail -1) | $(grep 'HEADLESS\] sim_t' "$M" | tail -1 | grep -o 'sim_t=[0-9.]* s.*cmd_timeouts=[0-9]*')"
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
