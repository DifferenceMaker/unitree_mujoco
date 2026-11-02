#!/usr/bin/env bash
# ff_handover.sh <walk> <balance> <instance> <none|cut|fade> -- Test 0 (the handover, 3 arm poses) on one lock-step rig with the
# ROBOT's arm gravity feedforward emulated (2026-11-02): ActionModule/Utils/arm_ff_only.py (the real robot/gravity_ff.GravityFF)
# runs inside the rig container the whole time, as the ActionModule does on the robot; the Bridge conductor drops it while a walk
# stream owns the arms and the setter then CUTS it after the 0.5 s lapse (cut: BRIDGE_TAU_FF_FADE_S=0, the robot until 11-02) or
# FADES it out over 1.0 s (fade). none = no feedforward (every exam so far). Records -> logs/ff_handover_<time>_<mode>/.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WALK="$1"; BAL="$2"; k="$3"; FF="$4"; OUT="$SIM/logs/ff_handover_$(date +%H-%M-%S)_$FF"; mkdir -p "$OUT"
export LOCOSW_INSTANCE="$k"
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
FADE=$([[ "$FF" == cut ]] && echo 0 || echo 1.0)
(cd "$SIM" && BRIDGE_TAU_FF_FADE_S="$FADE" HOMING_WALK="$WALK" HOMING_BALANCE="$BAL" HOMING_STOP_DIST=0.25 LOCOSW_LOCKSTEP=1 HOMING_SEED=1 METRICS=0 \
  HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120 HOMING_TRIAL_T0=15 bash run_mujoco_homing.sh > "$OUT/rig.log" 2>&1 &)
for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
L="$SIM/logs/stack_locosw_latest_$k.log"
for i in $(seq 1 300); do grep -q "\[homing\] pose=" "$L" 2>/dev/null && break; sleep 1; done
if [[ "$FF" != none ]]; then
  docker exec -d "archb_locosw_$k" bash -c 'source /opt/ros/humble/setup.bash; cd /workspace; PYTHONPATH=/workspace/.global:/workspace/ActionModule/main:${PYTHONPATH:-} /workspace/.venv/ActionModule/bin/python3 ActionModule/Utils/arm_ff_only.py > /tmp/arm_ff_only.log 2>&1'
  sleep 5; docker exec "archb_locosw_$k" bash -c 'cat /tmp/arm_ff_only.log' > "$OUT/arm_ff_only.log" 2>&1
fi
h0=$(wc -l < "$SIM/logs/handover_$k.jsonl" 2>/dev/null || echo 0)
bash "$SIM/tools/handover.sh" --timeout 600 > "$OUT/handover.log" 2>&1
tail -n +"$((h0 + 1))" "$SIM/logs/handover_$k.jsonl" > "$OUT/handover.jsonl"
[[ "$FF" != none ]] && docker exec "archb_locosw_$k" bash -c 'cat /tmp/arm_ff_only.log' > "$OUT/arm_ff_only.log" 2>&1
grep -a "tau_ff\|arms: MUTED\|arms: LIVE\|POLICY SWITCH" "$L" > "$OUT/events.txt" 2>/dev/null
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
echo "$FF: $(grep -c '^HANDOVER' "$OUT/handover.log") records -> $OUT"
