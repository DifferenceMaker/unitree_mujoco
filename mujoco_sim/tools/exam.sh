#!/usr/bin/env bash
# exam.sh -- THE standardized walk-policy exam, N rigs in parallel (2026-10-28 evening).
#   bash mujoco_sim/tools/exam.sh [--parallel 8] [--seed 1] [--n 10] [--balance p14h_stancewide] [--tag NAME] [--no-stairs] [--no-homing] <slug|name> ...
# Per policy (milestone slug in aspired-isaac-lab/milestone_checkpoints, or a name already vendored under MovementModule/policy):
#   vendor -> a LOCK-STEP headless rig on its own instance (LOCOSW_INSTANCE=k: own DDS domains, container, files) ->
#   Test 1 staircase (tools/staircase.sh: stand, vx 0.05/0.10/0.15/-0.10, vy +-0.10, wz +-0.3/+-0.5; sim-truth gains, drifts,
#   lifts, pivot index) -> Test 2 homing (tools/homing_trials.sh --n N --seed S: teleport, goto the table edge, balance switch,
#   settle; placement, stand drift, command tracking) -> stop -> the next policy from the queue.
# Deterministic: the same seed gives the same starts and, on the lock-step rig, the same batch bit for bit (det11/det12).
# Output: mujoco_sim/logs/exam/<tag>_<stamp>/ (per policy: rig_/stairs_/homing_ logs + the records), EXAM.md there (the ranking,
# tools/exam_report.py) and the same table appended to mujoco_sim/results/EXAM.md. Instances 1..N (instance 0 = the manual rig).
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
AIL="${AIL:-$REPOS/aspired-isaac-lab}"; ASPIRED_W="${ASPIRED:-$REPOS/Aspired_Robot_Project-homing}"; MMP="$ASPIRED_W/MovementModule/policy"
PAR=8; PIN=1; SEED=1; N=10; BALANCE="${HOMING_BALANCE:-p14h_stancewide}"; TAG="exam"; STAIRS=1; HOMING=1
POLICIES=()
while [[ $# -gt 0 ]]; do case "$1" in
  --parallel) PAR="$2"; shift 2;; --seed) SEED="$2"; shift 2;; --n) N="$2"; shift 2;; --balance) BALANCE="$2"; shift 2;; --tag) TAG="$2"; shift 2;;
  --no-stairs) STAIRS=0; shift;; --no-homing) HOMING=0; shift;; --no-pin) PIN=0; shift;; -*) echo "unknown arg: $1"; exit 1;; *) POLICIES+=("$1"); shift;; esac; done
[[ ${#POLICIES[@]} -gt 0 ]] || { sed -n '2,12p' "$0"; exit 1; }
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"; EX="$SIM/logs/exam/${TAG}_$STAMP"; mkdir -p "$EX"
vendor() {  # vendor <slug> -> prints the vendored policy name
  local slug="$1" name src
  name="$(echo "$slug" | sed -E 's/_20[0-9]{2}-[0-9]{2}-[0-9]{2}$//')"
  if [[ -f "$MMP/$name/policy.onnx" ]]; then echo "$name"; return 0; fi
  src="$AIL/milestone_checkpoints/$slug"
  [[ -f "$src/exported/policy.onnx" && -f "$src/params/deploy.yaml" ]] || { echo "!! $slug: no exported/policy.onnx or params/deploy.yaml" >&2; return 1; }
  mkdir -p "$MMP/$name" && cp "$src/exported/policy.onnx" "$MMP/$name/policy.onnx" && cp "$src/params/deploy.yaml" "$MMP/$name/deploy.yaml" \
    && cp "$src/overrides.json" "$MMP/$name/overrides.json" 2>/dev/null; cp "$src/MILESTONE.md" "$MMP/$name/" 2>/dev/null
  echo ">>> vendored $slug -> $MMP/$name (onnx md5 $(md5sum "$MMP/$name/policy.onnx" | cut -c1-12))" >&2
  echo "$name"
}
Q="$EX/queue.txt"; : > "$Q"
for slug in "${POLICIES[@]}"; do name="$(vendor "$slug")" && echo "$name" >> "$Q" || echo "$slug NOT VENDORED" >> "$EX/not_vendored.txt"; done
NP=$(wc -l < "$Q"); [[ $NP -gt 0 ]] || { echo "nothing to examine"; exit 2; }
[[ $PAR -gt $NP ]] && PAR=$NP
echo "##### $(date +%T) EXAM $TAG: $NP policies, $PAR instances (pin $PIN), seed $SEED, n $N, balance $BALANCE, stairs $STAIRS homing $HOMING -> $EX"
T_START=$(date +%s)
pop() { ( flock 9; local l; l=$(head -1 "$Q"); [[ -n "$l" ]] && { tail -n +2 "$Q" > "$Q.tmp" && mv "$Q.tmp" "$Q"; echo "$l"; } ) 9>"$Q.lock"; }

worker() {  # worker <instance k>
  local k="$1" name t0 t1 link0 i
  export LOCOSW_INSTANCE="$k"
  # CPU pinning (7950X: CPU i and i+16 share core i): <= 8 instances -> 2 cores (4 threads) each, else 1 core (2 threads) each
  if [[ $PIN -eq 1 ]]; then
    if [[ $PAR -le 8 ]]; then local a=$(( 2 * (k - 1) )); export LOCOSW_CPUS="$a,$((a + 1)),$((a + 16)),$((a + 17))"
    else local a=$(( (k - 1) % 16 )); export LOCOSW_CPUS="$a,$((a + 16))"; fi
  fi
  while name="$(pop)"; [[ -n "$name" ]]; do
    t0=$(date +%s)
    echo "[$k] $(date +%T) $name: launch${LOCOSW_CPUS:+ on cpus $LOCOSW_CPUS}"
    bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
    link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
    (cd "$SIM" && HOMING_WALK="$name" HOMING_BALANCE="$BALANCE" LOCOSW_LOCKSTEP=1 HOMING_SEED="$SEED" METRICS=0 bash run_mujoco_homing.sh > "$EX/rig_$name.log" 2>&1 &)
    for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
    local L="$SIM/logs/stack_locosw_latest_$k.log"; local ok=0
    for i in $(seq 1 300); do
      grep -q "BALANCE STABLE" "$L" 2>/dev/null && [[ $i -gt 10 ]] && { ok=1; break; }
      grep -q "Traceback\|refused\|sim died" "$L" 2>/dev/null && [[ $i -gt 30 ]] && break; sleep 1; done
    if [[ $ok -ne 1 ]]; then
      echo "[$k] $(date +%T) $name: !! rig did not reach BALANCE STABLE"; grep -E 'Traceback|refused|sim died|ENGAGED' "$L" | tail -3 | sed "s/^/[$k]    /"
      echo "$name NO_STABLE_STAND" >> "$EX/failed.txt"; bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; continue
    fi
    sleep 3
    if [[ $STAIRS -eq 1 ]]; then
      local s0; s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/staircase.sh" > "$EX/stairs_$name.log" 2>&1; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$EX/stairs_$name.jsonl"   # only THIS run's record (never a previous policy's)
      echo "[$k] $(date +%T) $name: staircase $(head -1 "$EX/stairs_$name.log" | cut -c1-80)"
    fi
    if [[ $HOMING -eq 1 ]]; then
      local h0; h0=$(wc -l < "$SIM/logs/homing_trials_$k.jsonl" 2>/dev/null || echo 0)
      HOMING_BENCH_FILE="$EX/bench.md" bash "$SIM/tools/homing_trials.sh" --n "$N" --seed "$SEED" --tag "${TAG}_$name" --no-plot > "$EX/homing_$name.log" 2>&1
      tail -n +"$((h0 + 1))" "$SIM/logs/homing_trials_$k.jsonl" > "$EX/homing_$name.jsonl"   # only THIS batch's records (a missing trial never pulls in an older one)
      echo "[$k] $(date +%T) $name: homing $(grep -E '^trials' "$EX/homing_$name.log" | tail -1 | cut -c1-60) | $(grep -E '^settled' "$EX/homing_$name.log" | tail -1 | cut -c21-110)"
    fi
    M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log 2>/dev/null | head -1); [[ -n "$M" ]] && grep -E 'HEADLESS\] sim_t' "$M" | tail -1 | sed "s/^/[$k]    /"
    bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
    t1=$(date +%s); echo "$name $((t1 - t0))" >> "$EX/times.txt"
    echo "[$k] $(date +%T) $name: done in $((t1 - t0)) s"
  done
}
for k in $(seq 1 "$PAR"); do worker "$k" & sleep "${EXAM_STAGGER_S:-8}"; done
wait
T_END=$(date +%s)
echo "##### $(date +%T) EXAM DONE: $NP policies on $PAR instances in $(( (T_END - T_START) / 60 )) min $(( (T_END - T_START) % 60 )) s"
PYX="${EXAM_PY:-python3}"
"$PYX" "$SIM/tools/exam_report.py" "$EX" --seed "$SEED" --n "$N" --balance "$BALANCE" --tag "$TAG" --wall "$((T_END - T_START))" --parallel "$PAR" | tee "$EX/report.txt"
