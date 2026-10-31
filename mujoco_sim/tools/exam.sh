#!/usr/bin/env bash
# exam.sh -- THE standardized walk-policy exam, N rigs in parallel (2026-10-28 evening).
#   bash mujoco_sim/tools/exam.sh [--exam 2] [--parallel 8] [--seed 1] [--n 10] [--balance p14h_stancewide] [--stop-dist 0.25] [--tag NAME] [--no-stairs] [--no-pushes] [--no-homing] <slug|name> ...
# EXAM #2 (default since 2026-10-30, results/EXAM2_SPEC.md): Test 0 handover -> Test 1 LADDERS (tools/exam2_test.sh ladders: 20 s stand,
#   then vx / vy / wz in 0.05 rungs up to the top, a stop, back down to the negative top, a stop) -> Test 2 PUSHES (tools/exam2_test.sh
#   pushes: 0.5 / 1.0 / 1.5 m/s kicks from 4 sides, standing and walking at 0.3) -> Test 3 HOMING (starts all around the desk, the
#   commander plans around it, turns by the heading error with counted rescue pulses, 120 s timeout). Scored by QUALITY
#   (tools/exam_score.py score_v2: survival 10, stability 30, tracking 20, gait 10, energy 10, pushes 10, homing 10). --exam 1 = the
#   2026-10-29 exam below (staircase, the bang-bang commander), kept to reproduce old records.
# Per policy (milestone slug in aspired-isaac-lab/milestone_checkpoints, or a name already vendored under MovementModule/policy):
#   vendor -> a LOCK-STEP headless rig on its own instance (LOCOSW_INSTANCE=k: own DDS domains, container, files) ->
#   Test 0 handover (tools/handover.sh: 3 arm poses, balance -> walk -> balance) ->
#   Test 1 staircase (tools/staircase.sh: stand, vx 0.05/0.10/0.15/-0.10, vy +-0.10, wz +-0.3/+-0.5/+-0.8; sim-truth gains, drifts,
#   lifts, pivot index) -> Test 2 homing (tools/homing_trials.sh --n N --seed S: teleport, goto the table edge, balance switch,
#   settle; placement, stand drift, command tracking) -> stop -> the next policy from the queue.
# Homing stop: torso front --stop-dist m from the desk edge (operator 2026-10-30: 0.25 m = the working distance; was the
# commander default 0.35 before). The value travels into the rig as HOMING_STOP_DIST and into the record's meta (stop_dist).
# Deterministic: the same seed gives the same starts and, on the lock-step rig, the same batch bit for bit (det11/det12).
# Output: mujoco_sim/logs/exam/<tag>_<stamp>/ (per policy: rig_/stairs_/homing_ logs + the records), EXAM.md there (the ranking,
# tools/exam_report.py) and the same table appended to mujoco_sim/results/EXAM.md. Instances 1..N (instance 0 = the manual rig).
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
AIL="${AIL:-$REPOS/aspired-isaac-lab}"; ASPIRED_W="${ASPIRED:-$REPOS/Aspired_Robot_Project-homing}"; MMP="$ASPIRED_W/MovementModule/policy"
EXAM=2; PUSHES=1; PAR=8; PIN=1; FIRST=1; SEED=1; N=10; BALANCE="${HOMING_BALANCE:-p14h_stancewide}"; TAG="exam"; STAIRS=1; HOMING=1; HANDOVER=1; STOP_DIST=0.25
POLICIES=()
while [[ $# -gt 0 ]]; do case "$1" in
  --exam) EXAM="$2"; shift 2;; --no-pushes) PUSHES=0; shift;; --parallel) PAR="$2"; shift 2;; --seed) SEED="$2"; shift 2;; --stop-dist) STOP_DIST="$2"; shift 2;; --n) N="$2"; shift 2;; --balance) BALANCE="$2"; shift 2;; --tag) TAG="$2"; shift 2;;
  --no-stairs) STAIRS=0; shift;; --no-handover) HANDOVER=0; shift;; --no-homing) HOMING=0; shift;; --no-pin) PIN=0; shift;; --first-instance) FIRST="$2"; shift 2;; -*) echo "unknown arg: $1"; exit 1;; *) POLICIES+=("$1"); shift;; esac; done
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
for slug in "${POLICIES[@]}"; do name="$(vendor "$slug")" && echo "$name $slug" >> "$Q" || echo "$slug NOT VENDORED" >> "$EX/not_vendored.txt"; done
NP=$(wc -l < "$Q"); [[ $NP -gt 0 ]] || { echo "nothing to examine"; exit 2; }
[[ $PAR -gt $NP ]] && PAR=$NP
echo "##### $(date +%T) EXAM #$EXAM $TAG: $NP policies, $PAR instances (pin $PIN), seed $SEED, n $N, balance $BALANCE, stop $STOP_DIST m, stairs/ladders $STAIRS pushes $PUSHES homing $HOMING -> $EX"
if [[ "$EXAM" == 2 ]]; then   # the commander knobs of exam #2 (forwarded into the rig by run_mujoco_loco_switch.sh)
  export HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120   # the desk scene: top 0.80 deep x 1.20 wide behind the edge (0.35, 0)
fi
T_START=$(date +%s)
pop() { ( flock 9; local l; l=$(head -1 "$Q"); [[ -n "$l" ]] && { tail -n +2 "$Q" > "$Q.tmp" && mv "$Q.tmp" "$Q"; echo "$l"; } ) 9>"$Q.lock"; }

worker() {  # worker <instance k>
  local k="$1" name t0 t1 link0 i
  export LOCOSW_INSTANCE="$k"
  # CPU pinning (7950X: CPU i and i+16 share core i): <= 8 instances -> 2 cores (4 threads) each, else 1 core (2 threads) each
  if [[ $PIN -eq 1 ]]; then
    if [[ $PAR -le 8 ]]; then local a=$(( (2 * (k - 1)) % 16 )); export LOCOSW_CPUS="$a,$((a + 1)),$((a + 16)),$((a + 17))"
    else local a=$(( (k - 1) % 16 )); export LOCOSW_CPUS="$a,$((a + 16))"; fi
  fi
  while item="$(pop)"; [[ -n "$item" ]]; do
    name="${item%% *}"; slug="${item##* }"; [[ "$slug" == "$name" ]] && slug="$(ls "$AIL/milestone_checkpoints" | grep -E "^${name}_20[0-9-]+$" | tail -1)"
    t0=$(date +%s)
    echo "[$k] $(date +%T) $name: launch${LOCOSW_CPUS:+ on cpus $LOCOSW_CPUS}"
    bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
    link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
    (cd "$SIM" && HOMING_WALK="$name" HOMING_BALANCE="$BALANCE" HOMING_STOP_DIST="$STOP_DIST" LOCOSW_LOCKSTEP=1 HOMING_SEED="$SEED" METRICS=0 bash run_mujoco_homing.sh > "$EX/rig_$name.log" 2>&1 &)
    for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
    local L="$SIM/logs/stack_locosw_latest_$k.log"; local ok=0
    # ready = the homing commander is up. The first queued item (the staircase) is then sent BEFORE the walk policy engages, so the
    # commander's scheduler starts it on the fixed sim frame TRIAL_T0 = 40 s (it waits for the policy itself, on the sim clock) -- a
    # wall-clock send after sim 40 s made the start frame depend on the host load and two same-seed exams differed (tp5 vs lm7f19).
    for i in $(seq 1 300); do
      grep -q "\[homing\] pose=" "$L" 2>/dev/null && { ok=1; break; }
      grep -q "Traceback\|refused\|sim died" "$L" 2>/dev/null && [[ $i -gt 30 ]] && break; sleep 1; done
    if [[ $ok -ne 1 ]]; then
      echo "[$k] $(date +%T) $name: !! rig / commander did not come up"; grep -E 'Traceback|refused|sim died|ENGAGED' "$L" | tail -3 | sed "s/^/[$k]    /"
      echo "$name NO_COMMANDER" >> "$EX/failed.txt"; bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; continue
    fi
    if [[ $HANDOVER -eq 1 ]]; then   # Test 0 (2026-10-29): balance -> walk -> balance from three arm poses, FIRST (the hardware failure)
      local o0; o0=$(wc -l < "$SIM/logs/handover_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/handover.sh" --timeout 600 > "$EX/handover_$name.log" 2>&1; tail -n +"$((o0 + 1))" "$SIM/logs/handover_$k.jsonl" > "$EX/handover_$name.jsonl"
      echo "[$k] $(date +%T) $name: handover $(grep -c '^HANDOVER' "$EX/handover_$name.log") records $(grep -o 'FAIL[a-z_]*' "$EX/handover_$name.log" | sort | uniq -c | tr '\n' ' ')"
    fi
    if [[ $STAIRS -eq 1 && "$EXAM" == 2 ]]; then   # exam #2 Test 1: the ladders (record in the staircase jsonl, kind "ladders")
      local s0; s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/exam2_test.sh" ladders --timeout 900 > "$EX/ladders_$name.log" 2>&1; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$EX/ladders_$name.jsonl"
      echo "[$k] $(date +%T) $name: ladders $(head -1 "$EX/ladders_$name.log" | cut -c1-90)"
    fi
    if [[ $PUSHES -eq 1 && "$EXAM" == 2 ]]; then   # exam #2 Test 2: the pushes
      local p0; p0=$(wc -l < "$SIM/logs/pushes_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/exam2_test.sh" pushes --timeout 600 > "$EX/pushes_$name.log" 2>&1; tail -n +"$((p0 + 1))" "$SIM/logs/pushes_$k.jsonl" > "$EX/pushes_$name.jsonl"
      echo "[$k] $(date +%T) $name: pushes $(head -1 "$EX/pushes_$name.log" | cut -c1-90) | held $(grep -c 'held True' "$EX/pushes_$name.log")/24"
    fi
    if [[ $STAIRS -eq 1 && "$EXAM" == 1 ]]; then
      local s0; s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/staircase.sh" --timeout 900 > "$EX/stairs_$name.log" 2>&1; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$EX/stairs_$name.jsonl"   # only THIS run's record (never a previous policy's)
      echo "[$k] $(date +%T) $name: staircase $(head -1 "$EX/stairs_$name.log" | cut -c1-80)"
    fi
    if [[ $HOMING -eq 1 ]]; then
      local h0; h0=$(wc -l < "$SIM/logs/homing_trials_$k.jsonl" 2>/dev/null || echo 0)
      HOMING_BENCH_FILE="$EX/bench.md" bash "$SIM/tools/homing_trials.sh" --n "$N" --seed "$SEED" --tag "${TAG}_$name" --no-plot $([[ "$EXAM" == 2 ]] && echo --ring) > "$EX/homing_$name.log" 2>&1
      tail -n +"$((h0 + 1))" "$SIM/logs/homing_trials_$k.jsonl" > "$EX/homing_$name.jsonl"   # only THIS batch's records (a missing trial never pulls in an older one)
      echo "[$k] $(date +%T) $name: homing $(grep -E '^trials' "$EX/homing_$name.log" | tail -1 | cut -c1-60) | $(grep -E '^settled' "$EX/homing_$name.log" | tail -1 | cut -c21-110)"
    fi
    M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log 2>/dev/null | head -1); [[ -n "$M" ]] && grep -E 'HEADLESS\] sim_t' "$M" | tail -1 | sed "s/^/[$k]    /"
    # the exam record that travels with the milestone (fleetdeck draws the sheets from it) -- JSON in git, no PNG
    python3 "$SIM/tools/exam_record.py" "$EX" "$name" "${slug:-$name}" --tag "$TAG" --seed "$SEED" --n "$N" --balance "$BALANCE" --stop-dist "$STOP_DIST" --exam "$EXAM" --instance "$k" 2>&1 | sed "s/^/[$k]    /"
    grep -c "POSE GAP" "$L" 2>/dev/null | sed "s/^/[$k]    pose gaps: /"
    bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
    t1=$(date +%s); echo "$name $((t1 - t0))" >> "$EX/times.txt"
    echo "[$k] $(date +%T) $name: done in $((t1 - t0)) s"
  done
}
for k in $(seq "$FIRST" "$((FIRST + PAR - 1))"); do worker "$k" & sleep "${EXAM_STAGGER_S:-8}"; done   # --first-instance: leave lower instances to other work
wait
T_END=$(date +%s)
echo "##### $(date +%T) EXAM DONE: $NP policies on $PAR instances in $(( (T_END - T_START) / 60 )) min $(( (T_END - T_START) % 60 )) s"
PYX="${EXAM_PY:-python3}"
"$PYX" "$SIM/tools/exam_report.py" "$EX" --exam "$EXAM" --seed "$SEED" --n "$N" --balance "$BALANCE" --tag "$TAG" --wall "$((T_END - T_START))" --parallel "$PAR" | tee "$EX/report.txt"
