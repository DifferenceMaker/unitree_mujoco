#!/usr/bin/env bash
# det_check.sh <walk policy> [instance A] [instance B] -- lock-step determinism check (2026-10-31): the same policy on two rigs at once,
# Test 0 (handover) + Test 1 (ladders) on each, then the two records compared field by field. Identical = the rig reproduces bit for bit
# (the seed-to-seed homing spread is then the only noise). Also prints each rig's bring-up wait and its re-published states.
# No exam / milestone record is written.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WALK="$1"; A="${2:-1}"; B="${3:-2}"; OUT="$SIM/logs/det_check_$(date +%H-%M-%S)"; mkdir -p "$OUT"
one() {  # one <instance>
  local k="$1"; export LOCOSW_INSTANCE="$k"
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
  local link0; link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
  (cd "$SIM" && HOMING_WALK="$WALK" HOMING_BALANCE=p14h_stancewide HOMING_STOP_DIST=0.25 LOCOSW_LOCKSTEP=1 HOMING_SEED=1 METRICS=0 \
    HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120 bash run_mujoco_homing.sh > "$OUT/rig_$k.log" 2>&1 &)
  for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
  for i in $(seq 1 300); do grep -q "\[homing\] pose=" "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null && break; sleep 1; done
  local h0 s0; h0=$(wc -l < "$SIM/logs/handover_$k.jsonl" 2>/dev/null || echo 0); s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
  bash "$SIM/tools/handover.sh" --timeout 600 > "$OUT/handover_$k.log" 2>&1
  bash "$SIM/tools/exam2_test.sh" ladders --timeout 900 > "$OUT/ladders_$k.log" 2>&1
  tail -n +"$((h0 + 1))" "$SIM/logs/handover_$k.jsonl" > "$OUT/handover_$k.jsonl"; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$OUT/ladders_$k.jsonl"
  local M; M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log | head -1)
  echo "rig $k: $(grep -m1 'first command after' "$M" | cut -c12-) | $(grep 'HEADLESS\] sim_t' "$M" | tail -1 | grep -o 'rtf=[0-9.]*\|cmd_timeouts=[0-9]*\|republished=[0-9]*' | tr '\n' ' ') | missed $(grep -c 'no MovementModule command' "$SIM/logs/stack_locosw_latest_$k.log")" > "$OUT/summary_$k.txt"
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
}
one "$A" & sleep 8; one "$B"; wait
cat "$OUT"/summary_*.txt
python3 - "$OUT" "$A" "$B" <<'PY'
import json, sys
out, a, b = sys.argv[1:4]
for kind in ("handover", "ladders"):
    ra = [json.loads(l) for l in open(f"{out}/{kind}_{a}.jsonl") if l.strip()]
    rb = [json.loads(l) for l in open(f"{out}/{kind}_{b}.jsonl") if l.strip()]
    strip = lambda r: {k: v for k, v in r.items() if k not in ("traj_file",)}
    same = [strip(x) == strip(y) for x, y in zip(ra, rb)]
    print(f"{kind}: {len(ra)} / {len(rb)} records, identical: {sum(same)}/{len(same)}")
    if not all(same):
        for x, y in zip(ra, rb):
            for s1, s2 in zip(x.get("segments", []), y.get("segments", [])):
                if s1 != s2:
                    print(f"   first difference in segment {s1.get('name')}: v_true {s1.get('v_true')} vs {s2.get('v_true')}"); break
PY
