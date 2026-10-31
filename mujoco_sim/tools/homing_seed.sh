#!/usr/bin/env bash
# homing_seed.sh <instance> <walk policy> <seed> <out dir> -- Exam #2 Test 3 alone with another seed (2026-10-31, the noise floor):
# a lock-step rig on LOCOSW_INSTANCE=<instance> with the Exam #2 commander knobs, tools/homing_trials.sh --ring --seed <seed> --n 10,
# the 10 trial records -> <out dir>/homing_<walk>_seed<seed>.jsonl. No milestone record (a homing-only record would become the
# policy's newest exam). Only homing depends on the seed: the handover, ladders and pushes repeat bit for bit on the lock-step rig.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K="$1"; WALK="$2"; SEED="$3"; OUT="$4"; mkdir -p "$OUT"
export LOCOSW_INSTANCE="$K" HOMING_TIMEOUT_S=120   # the driver's per-trial wait follows the commander's 120 s timeout (homing_trials.sh)
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
link0="$(readlink "$SIM/logs/stack_locosw_latest_$K.log" 2>/dev/null)"
(cd "$SIM" && HOMING_WALK="$WALK" HOMING_BALANCE=p14h_stancewide HOMING_STOP_DIST=0.25 LOCOSW_LOCKSTEP=1 HOMING_SEED="$SEED" METRICS=0 \
  HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" bash run_mujoco_homing.sh > "$OUT/rig_${WALK}_seed$SEED.log" 2>&1 &)
for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$K.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
for i in $(seq 1 300); do grep -q "\[homing\] pose=" "$SIM/logs/stack_locosw_latest_$K.log" 2>/dev/null && break; sleep 1; done
h0=$(wc -l < "$SIM/logs/homing_trials_$K.jsonl" 2>/dev/null || echo 0)
HOMING_BENCH_FILE="$OUT/bench.md" bash "$SIM/tools/homing_trials.sh" --n 10 --seed "$SEED" --ring --no-plot --tag "noise_${WALK}_s$SEED" > "$OUT/homing_${WALK}_seed$SEED.log" 2>&1
tail -n +"$((h0 + 1))" "$SIM/logs/homing_trials_$K.jsonl" > "$OUT/homing_${WALK}_seed$SEED.jsonl"
echo "$WALK seed $SEED: $(grep -E '^trials' "$OUT/homing_${WALK}_seed$SEED.log" | tail -1 | cut -c1-60)"
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
