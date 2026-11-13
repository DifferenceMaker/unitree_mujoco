#!/usr/bin/env bash
# exam.sh -- THE standardized walk-policy exam, N rigs in parallel (2026-10-28 evening).
#   bash mujoco_sim/tools/exam.sh [--exam 3|2|1, default 3 since 2026-11-13] [--parallel 8] [--seed 1] [--n 10] [--balance SLUG] [--stop-dist 0.25] [--tag NAME] [--no-stairs] [--no-pushes] [--no-homing] [--stumble] <slug|name> ...
# EXAM #3 (2026-11-13, owner: "No flags. This is our new test"; results/EXAM2_SPEC.md sections 12 + 13) = --exam 3 = every test of Exam #2 v4,
#   unchanged and in the same order, THEN Test 6 ENCODER OFFSETS (tools/exam2_test.sh enc): the sim's robot gets the fixed zero-offset patterns of
#   tools/enc_patterns.py (reported joint angle = true + offset; P0 zero, P1..P4 inside the trained +-0.010 rad band, P5 = the measured
#   residuals unscaled = the stress pattern), per pattern a 15 s stand and a 15 s measured walk at vx 0.30, then Test 2's 1.0 m/s kick from
#   the 4 sides, standing and walking, under P5 (the offset pushes): 311 s of sim, ~1 min of wall. Scored by tools/exam_score.py score_e3:
#   the seven quality totals of v4, with Test 6 rows inside survival / stability / gait / energy / pushes, + the constraints walks_low,
#   enc_no_falls, enc_stand_drift, enc_stand_lifts. Records: meta.exam_version 3 (meta.score.version 5). Needs a sim built from 2026-11-13 on.
#   Exam #3 scores do NOT compare with Exam #2 scores (the weights inside the qualities moved); every Exam #3 record also carries
#   meta.score.score_v4 = the same run under the v4 rules.
# --exam 2 = Exam #2 v4 EXACTLY as before 2026-11-13 (no Test 6, score_v4, exam_version 2): reproduces every v4 record; the running
#   Optuna study s1_pilot is scored with it.
# THE DEFAULT: the line "EXAM=..." below. Still 2 on 2026-11-13 (the s1_pilot watcher calls this script without --exam until its stored
#   flags say --exam 2); the owner's decision is 3 -- flip that one value, nothing else depends on it.
#   --balance: default (Exam #2 v4, 2026-11-09) = the rig worktree's MovementModule/policy/CURRENT_BALANCE, read at launch (p16c_sw_enc01 on
#   2026-11-09; was the fixed p14h_stancewide); the slug used goes into every record's meta.balance. HOMING_BALANCE in the env overrides too.
#   EXAM_FF=0 bash ... = no arm gravity feedforward (every exam before 2026-11-03; the default since then runs the robot's ff sender in every rig).
# EXAM #2 (default since 2026-10-30, results/EXAM2_SPEC.md): Test 0 handover -> Test 1 LADDERS (tools/exam2_test.sh ladders: 20 s stand,
#   then vx / vy / wz in 0.05 rungs up to the top, a stop, back down to the negative top, a stop) -> Test 2 PUSHES (tools/exam2_test.sh
#   pushes: 0.5 / 1.0 / 1.5 m/s kicks from 4 sides + (v4, 2026-11-09) the 4 diagonals = 48, standing and walking at 0.3) -> Test 3 HOMING (starts all around the desk, the
#   commander plans around it, turns by the heading error with counted rescue pulses, 120 s timeout) -> Test 5 STUMBLE (2026-11-02,
#   tools/exam2_test.sh stumble: vx 0.15 / 0.30 for 15 s each on a rough floor, 0.4-1 cm sphere bumps (2 cm until 2026-11-02 evening) deterministic
#   per --seed, far from the other tests; swing apex per foot, scuffs / catches = trips, tilt, falls; RECORDED, NOT SCORED yet -- EXAM2_SPEC Test 5).
#   Since 2026-11-02 evening the same swing engine runs on every LADDER segment: the key columns are the FLAT vx 0.15 / 0.30 rungs' scuffed swings
#   ("scuffs .15" = scuffed/swings) beside their swing heights -- the robot's toe drag happened on a flat floor. Scored by QUALITY
#   (tools/exam_score.py score_v4 since 2026-11-09 -- v3 + rung error floor 0.05 + 48 pushes + the constraints block; weights survival 10,
#   stability 30, tracking 20, gait 10, energy 10, pushes 10, homing 10). --exam 1 = the
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
EXAM=3; PUSHES=1; STUMBLE=0; NOMS=0; PAR=8; PIN=1; FIRST=1; SEED=1; N=10; BALANCE="${HOMING_BALANCE:-}"; TAG="exam"; STAIRS=1; HOMING=1; HANDOVER=1; STOP_DIST=0.25
FF="${EXAM_FF:-1}"   # 2026-11-03: the robot's arm gravity feedforward runs in EVERY rig (ActionModule/Utils/arm_ff_only.py in the container, as the ActionModule on the robot); EXAM_FF=0 = the pre-11-03 plant (PD-only arms)
POLICIES=()
while [[ $# -gt 0 ]]; do case "$1" in
  --exam) EXAM="$2"; shift 2;; --no-pushes) PUSHES=0; shift;; --no-stumble) STUMBLE=0; shift;; --stumble) STUMBLE=1; shift;; --no-milestone) NOMS=1; shift;; --parallel) PAR="$2"; shift 2;; --seed) SEED="$2"; shift 2;; --stop-dist) STOP_DIST="$2"; shift 2;; --n) N="$2"; shift 2;; --balance) BALANCE="$2"; shift 2;; --tag) TAG="$2"; shift 2;;
  --no-stairs) STAIRS=0; shift;; --no-handover) HANDOVER=0; shift;; --no-homing) HOMING=0; shift;; --no-pin) PIN=0; shift;; --first-instance) FIRST="$2"; shift 2;; -*) echo "unknown arg: $1"; exit 1;; *) POLICIES+=("$1"); shift;; esac; done
[[ ${#POLICIES[@]} -gt 0 ]] || { sed -n '2,12p' "$0"; exit 1; }
# Exam #3 (2026-11-13) = Exam #2's tests (EXB = the test set this run drives: every "exam #2" branch below reads it) + Test 6 (ENC)
[[ "$EXAM" == 1 || "$EXAM" == 2 || "$EXAM" == 3 ]] || { echo "unknown --exam $EXAM (3 = Exam #3, 2 = Exam #2 v4, 1 = the 2026-10-29 exam)"; exit 1; }
ENC=0; EXB="$EXAM"; [[ "$EXAM" == 3 ]] && { ENC=1; EXB=2; }
# v4 (2026-11-09): the balance partner = the line's CURRENT balance policy of the rig worktree, read now (one value for the whole run)
if [[ -z "$BALANCE" ]]; then
  BALANCE="$(tr -d '[:space:]' < "$MMP/CURRENT_BALANCE" 2>/dev/null)"
  [[ -n "$BALANCE" ]] || { echo "FATAL: no --balance and no $MMP/CURRENT_BALANCE"; exit 1; }
  echo ">>> balance partner = CURRENT_BALANCE of $ASPIRED_W: $BALANCE"
fi
[[ -f "$MMP/$BALANCE/policy.onnx" ]] || { echo "FATAL: balance policy $BALANCE not vendored in $MMP"; exit 1; }
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
[[ "$EXB" == 2 ]] || STUMBLE=0
echo "##### $(date +%T) EXAM #$EXAM $TAG: $NP policies, $PAR instances (pin $PIN), seed $SEED, n $N, balance $BALANCE, stop $STOP_DIST m, stairs/ladders $STAIRS pushes $PUSHES homing $HOMING stumble $STUMBLE arm ff $FF -> $EX"
if [[ "$EXB" == 2 ]]; then   # the commander knobs of exam #2 (forwarded into the rig by run_mujoco_loco_switch.sh)
  export HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120   # the desk scene: top 0.80 deep x 1.20 wide behind the edge (0.35, 0)
  export HOMING_TRIAL_T0="${HOMING_TRIAL_T0:-15}"   # 2026-11-01: the first item at sim 15 s (was 40; the walk engages at ~4.6 s)
  [[ $STUMBLE -eq 1 && "${EXAM_STUMBLE_FIELD:-1}" == 1 ]] && export LOCOSW_STUMBLE_SEED="$SEED"   # Test 5: the launcher generates <scene>_stumble$SEED.xml (the field exists only at x 50..64 m); EXAM_STUMBLE_FIELD=0 = the same script on the flat (control)
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
    grep -o 'robot_scene -> [^ ]*' "$EX/rig_$name.log" 2>/dev/null | head -1 | sed "s/^/[$k]    scene= /"   # the scene THIS rig loaded (a stale MJ_SCENE once leaked a scene)
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
    local balx; balx="$(grep -a -m1 -o 'SHADOW POLICY[^:]*: [^:]*' "$L" | sed 's/.*: //')"   # v4: the balance MovementModule actually loaded
    [[ "$balx" == "$BALANCE" ]] && echo "[$k]    balance in the rig: $balx" || { echo "[$k]    !! balance in the rig '$balx' != $BALANCE"; echo "$name BALANCE_MISMATCH $balx" >> "$EX/failed.txt"; }
    # ARM GRAVITY FEEDFORWARD (2026-11-03, operator: the ff is on in every policy on the robot, walk and balance): the robot's
    # ActionModule streams pinocchio g(q_measured) for both arms to /BridgeModule/arm_tau_ff; the rig runs the same sender
    # (ActionModule/Utils/arm_ff_only.py = robot/gravity_ff.GravityFF, as tools/ff_handover.sh did) inside the container for the whole
    # policy, so the exam tests the plant the robot will run (the lm7j walk rows train with it). The Bridge conductor keeps the ff
    # while the walk owns the arm targets (Aspired af7e173 on MM_loco_switch, cherry-picked to MM_loco_switch_homing). EXAM_FF=0 skips it. The sender is wall-clock (30 Hz) like the arm stream.
    if [[ "$FF" == 1 ]]; then
      docker exec -d "archb_locosw_$k" bash -c 'source /opt/ros/humble/setup.bash; cd /workspace; PYTHONPATH=/workspace/.global:/workspace/ActionModule/main:${PYTHONPATH:-} /workspace/.venv/ActionModule/bin/python3 ActionModule/Utils/arm_ff_only.py > /tmp/arm_ff_only.log 2>&1'
      for i in $(seq 1 20); do docker exec "archb_locosw_$k" grep -q "tau_ff first sample" /tmp/arm_ff_only.log 2>/dev/null && break; sleep 1; done
      if docker exec "archb_locosw_$k" grep -q "tau_ff first sample" /tmp/arm_ff_only.log 2>/dev/null; then
        echo "[$k] $(date +%T) $name: arm ff ON -- $(docker exec "archb_locosw_$k" grep -o "first sample.*" /tmp/arm_ff_only.log | cut -c1-120)"
      else echo "[$k] $(date +%T) $name: !! arm ff requested but no first sample after 20 s (see ff_$name.log)"; echo "$name NO_ARM_FF" >> "$EX/failed.txt"; fi
    else echo "[$k] $(date +%T) $name: arm ff OFF (EXAM_FF=0: PD-only arms, the pre-2026-11-03 plant)"; fi
    if [[ $HANDOVER -eq 1 ]]; then   # Test 0 (2026-10-29): balance -> walk -> balance from three arm poses, FIRST (the hardware failure)
      local o0; o0=$(wc -l < "$SIM/logs/handover_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/handover.sh" --timeout 600 > "$EX/handover_$name.log" 2>&1; tail -n +"$((o0 + 1))" "$SIM/logs/handover_$k.jsonl" > "$EX/handover_$name.jsonl"
      echo "[$k] $(date +%T) $name: handover $(grep -c '^HANDOVER' "$EX/handover_$name.log") records $(grep -o 'FAIL[a-z_]*' "$EX/handover_$name.log" | sort | uniq -c | tr '\n' ' ')"
    fi
    if [[ $STAIRS -eq 1 && "$EXB" == 2 ]]; then   # exam #2 Test 1: the ladders (record in the staircase jsonl, kind "ladders")
      local s0; s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/exam2_test.sh" ladders --timeout 900 > "$EX/ladders_$name.log" 2>&1; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$EX/ladders_$name.jsonl"
      echo "[$k] $(date +%T) $name: ladders $(head -1 "$EX/ladders_$name.log" | cut -c1-90)"
      grep -E '^  vx\+0\.(15|30) ' "$EX/ladders_$name.log" | sed -E "s/^  (vx[^ ]+) .*  sw /[$k]    \1 sw /" | cut -c1-160   # the toe-drag key rungs (2026-11-02)
    fi
    if [[ $PUSHES -eq 1 && "$EXB" == 2 ]]; then   # exam #2 Test 2: the pushes
      local p0; p0=$(wc -l < "$SIM/logs/pushes_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/exam2_test.sh" pushes --timeout 900 > "$EX/pushes_$name.log" 2>&1; tail -n +"$((p0 + 1))" "$SIM/logs/pushes_$k.jsonl" > "$EX/pushes_$name.jsonl"   # v4: 48 pushes = 363 s of sim (v3: 24, 183 s)
      echo "[$k] $(date +%T) $name: pushes $(head -1 "$EX/pushes_$name.log" | cut -c1-90) | held $(grep -c 'held True' "$EX/pushes_$name.log")/$(grep -c ' held ' "$EX/pushes_$name.log")"
    fi
    if [[ $STAIRS -eq 1 && "$EXB" == 1 ]]; then
      local s0; s0=$(wc -l < "$SIM/logs/staircase_$k.jsonl" 2>/dev/null || echo 0)
      bash "$SIM/tools/staircase.sh" --timeout 900 > "$EX/stairs_$name.log" 2>&1; tail -n +"$((s0 + 1))" "$SIM/logs/staircase_$k.jsonl" > "$EX/stairs_$name.jsonl"   # only THIS run's record (never a previous policy's)
      echo "[$k] $(date +%T) $name: staircase $(head -1 "$EX/stairs_$name.log" | cut -c1-80)"
    fi
    if [[ $HOMING -eq 1 ]]; then
      local h0; h0=$(wc -l < "$SIM/logs/homing_trials_$k.jsonl" 2>/dev/null || echo 0)
      HOMING_BENCH_FILE="$EX/bench.md" bash "$SIM/tools/homing_trials.sh" --n "$N" --seed "$SEED" --tag "${TAG}_$name" --no-plot $([[ "$EXB" == 2 ]] && echo --ring) > "$EX/homing_$name.log" 2>&1
      tail -n +"$((h0 + 1))" "$SIM/logs/homing_trials_$k.jsonl" > "$EX/homing_$name.jsonl"   # only THIS batch's records (a missing trial never pulls in an older one)
      echo "[$k] $(date +%T) $name: homing $(grep -E '^trials' "$EX/homing_$name.log" | tail -1 | cut -c1-60) | $(grep -E '^settled' "$EX/homing_$name.log" | tail -1 | cut -c21-110)"
    fi
    if [[ $STUMBLE -eq 1 ]]; then   # exam #2 Test 5 (2026-11-02): LAST, so Tests 0-3 keep their planned frames (their records stay comparable with older runs)
      local u0; u0=$(wc -l < "$SIM/logs/stumble_$k.jsonl" 2>/dev/null || echo 0); local tu0; tu0=$(date +%s)
      bash "$SIM/tools/exam2_test.sh" stumble --timeout 300 > "$EX/stumble_$name.log" 2>&1; tail -n +"$((u0 + 1))" "$SIM/logs/stumble_$k.jsonl" > "$EX/stumble_$name.jsonl"
      echo "[$k] $(date +%T) $name: stumble $(head -1 "$EX/stumble_$name.log" | cut -c1-120) | wall $(( $(date +%s) - tu0 )) s"
      grep -E '^  vx' "$EX/stumble_$name.log" | sed "s/^/[$k]    /" | cut -c1-200
    fi
    if [[ $ENC -eq 1 ]]; then   # EXAM #3 Test 6 (2026-11-13): encoder offsets + the offset pushes, AFTER everything else -- Tests 0-5 keep their
      # frames and their records, and the offsets (zero again at the test's end; the rig stops right after anyway) can never reach another test
      touch "$SIM/logs/enc_$k.jsonl"   # host-owned before the container appends to it (run_mujoco_homing.sh does this for the other tests' files)
      local c0; c0=$(wc -l < "$SIM/logs/enc_$k.jsonl" 2>/dev/null || echo 0); local tc0; tc0=$(date +%s)
      bash "$SIM/tools/exam2_test.sh" enc --timeout 500 > "$EX/enc_$name.log" 2>&1; tail -n +"$((c0 + 1))" "$SIM/logs/enc_$k.jsonl" > "$EX/enc_$name.jsonl"
      echo "[$k] $(date +%T) $name: enc $(head -1 "$EX/enc_$name.log" | cut -c1-120) | offset pushes held $(grep -c 'held True' "$EX/enc_$name.log")/$(grep -c ' held ' "$EX/enc_$name.log") | wall $(( $(date +%s) - tc0 )) s"
      grep -E '^  enc:P[0-9]+:(stand|walk)' "$EX/enc_$name.log" | sed -E "s/^  (enc:[^ ]+) .*  ENC [a-z]+: /[$k]    \1  /" | cut -c1-200
      # the sim's own account of the offsets it applied (frame + values) travels with the exam: the injection's first witness
      local ME; ME=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log 2>/dev/null | head -1); [[ -n "$ME" ]] && grep -a '^\[ENC\]' "$ME" > "$EX/enc_sim_$name.log"
      echo "[$k]    sim [ENC] lines: $(wc -l < "$EX/enc_sim_$name.log" 2>/dev/null || echo 0) (ON $(grep -c 'offsets ON' "$EX/enc_sim_$name.log" 2>/dev/null))"
    fi
    M=$(ls -t "$SIM"/logs/mujoco_locosw_*_"$k".log 2>/dev/null | head -1); [[ -n "$M" ]] && grep -E 'HEADLESS\] sim_t' "$M" | tail -1 | sed "s/^/[$k]    /"
    # the exam record that travels with the milestone (fleetdeck draws the sheets from it) -- JSON in git, no PNG
    python3 "$SIM/tools/exam_record.py" "$EX" "$name" "${slug:-$name}" --tag "$TAG" --seed "$SEED" --n "$N" --balance "$BALANCE" --stop-dist "$STOP_DIST" --exam "$EXAM" --instance "$k" --arm-ff "$FF" $([[ $NOMS -eq 1 ]] && echo "--ail /nonexistent") 2>&1 | sed "s/^/[$k]    /"
    grep -c "POSE GAP" "$L" 2>/dev/null | sed "s/^/[$k]    pose gaps: /"
    if [[ "$FF" == 1 ]]; then   # the ff sender's log + the Bridge's ff events (APPLIED / reverted / MUTED / LIVE) travel with the exam
      docker exec "archb_locosw_$k" cat /tmp/arm_ff_only.log > "$EX/ff_$name.log" 2>/dev/null
      grep -a "tau_ff\|arms: MUTED\|arms: LIVE" "$L" >> "$EX/ff_$name.log" 2>/dev/null
      echo "[$k]    arm ff: applied $(grep -c 'tau_ff APPLIED' "$EX/ff_$name.log") reverted $(grep -c 'reverted to 0' "$EX/ff_$name.log") (a revert = the sender lapsed; expected 0)"
    fi
    bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
    t1=$(date +%s); echo "$name $((t1 - t0))" >> "$EX/times.txt"
    echo "[$k] $(date +%T) $name: done in $((t1 - t0)) s"
  done
}
for k in $(seq "$FIRST" "$((FIRST + PAR - 1))"); do worker "$k" & sleep "${EXAM_STAGGER_S:-3}"; done   # --first-instance: leave lower instances to other work
wait
T_END=$(date +%s)
echo "##### $(date +%T) EXAM DONE: $NP policies on $PAR instances in $(( (T_END - T_START) / 60 )) min $(( (T_END - T_START) % 60 )) s"
PYX="${EXAM_PY:-python3}"
"$PYX" "$SIM/tools/exam_report.py" "$EX" --exam "$EXAM" --seed "$SEED" --n "$N" --balance "$BALANCE" --tag "$TAG" --wall "$((T_END - T_START))" --parallel "$PAR" | tee "$EX/report.txt"
