#!/usr/bin/env bash
# wire_check.sh <walk policy> [sim seconds, default 60] [instance A] [instance B] -- lock-step TRANSPORT equivalence check (2026-11-01):
# the same policy on two rigs at once, A on the old wire (LOCOSW_COMPACT=0: full rt/lowstate + rt/lowcmd every frame), B on the
# compact wire (LOCOSW_COMPACT=1), both with SIM_TRACE (per-frame hashes of ctrl / qpos / qvel). Stops both at <sim seconds> (before
# the exam's first item at sim 15 s nothing is commanded, so no wall-clock input enters) and compares the traces frame by frame:
# identical = the transport changes nothing the physics sees. Also prints each rig's rtf and per-frame waits.
# WIRE_A / WIRE_B (default 0 / 1) pick each rig's wire; WIRE_TEST=handover runs Test 0 on both before stopping (the determinism check:
# with WIRE_A=1 WIRE_B=1 the first differing frame shows where a wall-clock input entered).
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WALK="$1"; T="${2:-60}"; A="${3:-1}"; B="${4:-2}"; OUT="$SIM/logs/wire_check_$(date +%H-%M-%S)"; mkdir -p "$OUT"
one() {  # one <instance> <compact 0|1>
  local k="$1"; export LOCOSW_INSTANCE="$k"
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
  local link0; link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
  (cd "$SIM" && LOCOSW_COMPACT="$2" SIM_TRACE="$OUT/trace_$k.txt" HOMING_WALK="$WALK" HOMING_BALANCE=p14h_stancewide HOMING_STOP_DIST=0.25 \
    LOCOSW_LOCKSTEP=1 HOMING_SEED=1 METRICS=0 HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120 bash run_mujoco_homing.sh > "$OUT/rig_$k.log" 2>&1 &)
  for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
  local M=""; for i in $(seq 1 60); do M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log 2>/dev/null | head -1); [[ -n "$M" && "$M" -nt "$OUT" ]] && break; sleep 1; done
  if [[ "${WIRE_TEST:-}" == "handover" ]]; then
    for i in $(seq 1 300); do grep -q "\[homing\] pose=" "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null && break; sleep 1; done
    bash "$SIM/tools/handover.sh" --timeout 600 > "$OUT/handover_$k.log" 2>&1
    grep -a "HANDOVER\|POLICY SWITCH\|arm" "$SIM/logs/stack_locosw_latest_$k.log" > "$OUT/events_$k.txt" 2>/dev/null
  fi
  for i in $(seq 1 900); do
    [[ "${WIRE_TEST:-}" == "handover" ]] && break
    local t; t=$(grep -a 'HEADLESS\] sim_t' "$M" 2>/dev/null | tail -1 | grep -o 'sim_t=[0-9.]*' | cut -d= -f2)
    [[ -n "$t" ]] && awk -v t="$t" -v T="$T" 'BEGIN { exit !(t >= T) }' && break; sleep 1; done
  echo "rig $k (compact $2): $(grep -m1 'first command after' "$M" | cut -c12-) | $(grep -a 'HEADLESS\] sim_t' "$M" | tail -1 | cut -c12-) | missed $(grep -c 'no MovementModule command' "$SIM/logs/stack_locosw_latest_$k.log")" > "$OUT/summary_$k.txt"
  grep -a "compact transport" "$M" | head -1 >> "$OUT/summary_$k.txt"
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
}
one "$A" "${WIRE_A:-0}" & sleep 3; one "$B" "${WIRE_B:-1}"; wait
cat "$OUT"/summary_*.txt
python3 - "$OUT/trace_$A.txt" "$OUT/trace_$B.txt" <<'PY'
import sys
a = [l.split() for l in open(sys.argv[1]) if len(l.split()) == 8]   # a rig stopped mid-write leaves a truncated last line
b = [l.split() for l in open(sys.argv[2]) if len(l.split()) == 8]
n = min(len(a), len(b))
first = next((i for i in range(n) if a[i] != b[i]), None)
if first is None:
    print(f"traces IDENTICAL over {n} common frames (sim {a[n - 1][1]} s)")
else:
    print(f"traces DIFFER at frame {a[first][0]} (sim {a[first][1]} s) of {n} common frames:\n  A {' '.join(a[first])}\n  B {' '.join(b[first])}")
PY
