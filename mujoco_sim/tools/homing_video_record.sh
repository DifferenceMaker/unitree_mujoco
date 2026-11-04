#!/usr/bin/env bash
# homing_video_record.sh -- homing trials WITH THE FULL STATE recorded, for the multi-robot homing video (2026-11-04).
#   bash mujoco_sim/tools/homing_video_record.sh [--out DIR] [--walk lm7j_stopramp] [--balance p14h_stancewide] [--n 12]
#        [--seeds "101 102 103"] [--first-instance 21] [--dmin 3.0] [--dmax 4.5] [--fan 35] [--stop-dist 0.25]
#        [--items "102:5 101:4 ..."] [--work PLAN.json]
# One LOCK-STEP rig per seed (instance first+i, run in parallel), set up as one tools/exam.sh worker: desk scene, exam #2 commander
# knobs (turn law p, HOMING_DESK, 120 s timeout, T0 15 s), the arm gravity feedforward on, stop 0.25 m. Per rig: the passive
# tools/sim_state_recorder.py (rt/sim_base_pose = base pose + all 27 joints at 50 Hz of sim time), then N "trial" items queued on the
# commander at once (teleport -> 2 s -> goto the desk edge -> balance switch -> 10 s settle; the commander sequences them on planned sim
# frames). Starts: distance U(dmin, dmax) from the edge centre, bearing U(-fan, +fan) deg around the approach line IN FRONT of the desk
# (so after the video's rotation about the vertical axis no path crosses the round table), heading U(-180, 180); seeded per trial
# (random.seed(seed * 1000 + t), the homing_trials.sh convention).
# Output DIR (default mujoco_sim/logs/homing_video/<stamp>/): state_<seed>.bin/.json, trials_<seed>.jsonl (the commander's records),
# teleports_<seed>.txt (the commander's teleport lines), rig_<seed>.log, rec_<seed>.log. Render: tools/homing_video_render.py.
# --items "SEED:T ...": ONE trial per rig instead (the trial T of SEED, the same start as trial T of a --seeds batch), every item on its
# own rig in parallel; files keyed <seed>_t<T>. --work PLAN.json (tools/homing_work_plan.py, 2026-11-04 the work video): after the
# trial's ARRIVED record the rig plays the arm sequence (tools/homing_work_driver.py inside the container, sim-clock paced) and the
# recorder keeps running until it ends; its log = work_<key>.jsonl.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"
WALK=lm7j_stopramp; BAL=p14h_stancewide; N=12; SEEDS="101 102 103"; FIRST=21; DMIN=3.0; DMAX=4.5; FAN=35; STOP_DIST=0.25; OUT=""; ITEMS=""; WORK=""
while [[ $# -gt 0 ]]; do case "$1" in
  --out) OUT="$2"; shift 2;; --walk) WALK="$2"; shift 2;; --balance) BAL="$2"; shift 2;; --n) N="$2"; shift 2;; --seeds) SEEDS="$2"; shift 2;;
  --first-instance) FIRST="$2"; shift 2;; --dmin) DMIN="$2"; shift 2;; --dmax) DMAX="$2"; shift 2;; --fan) FAN="$2"; shift 2;;
  --stop-dist) STOP_DIST="$2"; shift 2;; --items) ITEMS="$2"; shift 2;; --work) WORK="$(readlink -f "$2")"; shift 2;;
  *) echo "unknown arg: $1"; exit 1;; esac; done
OUT="${OUT:-$SIM/logs/homing_video/$(date +%Y-%m-%d_%H-%M-%S)}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"   # absolute: the work driver writes into it from the container
# the exam #2 commander knobs (tools/exam.sh), forwarded into the container by run_mujoco_loco_switch.sh
export HOMING_TURN_LAW=p HOMING_DESK="0.8 1.2" HOMING_TIMEOUT_S=120 HOMING_TRIAL_T0=15
echo "##### $(date +%T) homing video record: walk $WALK, balance $BAL, ${ITEMS:+items [$ITEMS]}${ITEMS:-$N trials x seeds [$SEEDS]}, starts $DMIN-$DMAX m, fan +-$FAN deg, stop $STOP_DIST m${WORK:+, arm sequence $WORK} -> $OUT"
printf '{"walk": "%s", "balance": "%s", "n": %s, "seeds": "%s", "items": "%s", "work": "%s", "dmin": %s, "dmax": %s, "fan": %s, "stop_dist": %s, "goal": [0.35, 0.0, 0.0]}\n' \
  "$WALK" "$BAL" "$N" "$SEEDS" "$ITEMS" "$WORK" "$DMIN" "$DMAX" "$FAN" "$STOP_DIST" > "$OUT/meta.json"
[[ -n "$WORK" ]] && cp "$WORK" "$OUT/work_plan.json"

rig() {  # rig <instance k> <seed> <trials "1 2 ..."> <key>
  local k="$1" seed="$2" trials="$3" key="$4" i link0 L REC_PID ok=0 n
  n=$(wc -w <<< "$trials")
  export LOCOSW_INSTANCE="$k"
  local JL="$SIM/logs/homing_trials_$k.jsonl" CMD="$SIM/logs/.homing_cmd_$k"
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; sleep 1
  link0="$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)"
  "$TV_PY" "$SIM/tools/sim_state_recorder.py" --domain $((1 + k)) --out "$OUT/state_$key" > "$OUT/rec_$key.log" 2>&1 &
  REC_PID=$!
  (cd "$SIM" && HOMING_WALK="$WALK" HOMING_BALANCE="$BAL" HOMING_STOP_DIST="$STOP_DIST" LOCOSW_LOCKSTEP=1 HOMING_SEED="$seed" METRICS=0 \
     bash run_mujoco_homing.sh > "$OUT/rig_$key.log" 2>&1 &)
  for i in $(seq 1 60); do [[ "$(readlink "$SIM/logs/stack_locosw_latest_$k.log" 2>/dev/null)" != "$link0" ]] && break; sleep 1; done
  L="$SIM/logs/stack_locosw_latest_$k.log"
  grep -o 'robot_scene -> [^ ]*' "$OUT/rig_$key.log" 2>/dev/null | head -1 | sed "s/^/[$k] scene= /"
  for i in $(seq 1 300); do
    grep -q "\[homing\] pose=" "$L" 2>/dev/null && { ok=1; break; }
    grep -q "Traceback\|refused\|sim died" "$L" 2>/dev/null && [[ $i -gt 30 ]] && break; sleep 1; done
  if [[ $ok -ne 1 ]]; then echo "[$k] !! rig / commander did not come up ($key)"; kill -INT "$REC_PID" 2>/dev/null; bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; return 1; fi
  # the robot's arm gravity feedforward (exam.sh, 2026-11-03: on in every rig)
  docker exec -d "archb_locosw_$k" bash -c 'source /opt/ros/humble/setup.bash; cd /workspace; PYTHONPATH=/workspace/.global:/workspace/ActionModule/main:${PYTHONPATH:-} /workspace/.venv/ActionModule/bin/python3 ActionModule/Utils/arm_ff_only.py > /tmp/arm_ff_only.log 2>&1'
  for i in $(seq 1 20); do docker exec "archb_locosw_$k" grep -q "tau_ff first sample" /tmp/arm_ff_only.log 2>/dev/null && break; sleep 1; done
  docker exec "archb_locosw_$k" grep -q "tau_ff first sample" /tmp/arm_ff_only.log 2>/dev/null && echo "[$k] $(date +%T) $key: arm ff ON" || echo "[$k] !! arm ff: no first sample after 20 s"
  local n0; n0=$(wc -l < "$JL" 2>/dev/null || echo 0)
  if [[ -n "$WORK" ]]; then   # the arm sequence after the trial (the MovementModule venv: rclpy + unitree_sdk2py, the container's DDS env)
    cp "$WORK" "$SIM/logs/.work_plan_$k.json"; : > "$OUT/work_$key.jsonl"
    docker exec -d "archb_locosw_$k" bash -c "source /opt/ros/humble/setup.bash; source /workspace/.venv/MovementModule/bin/activate; cd /workspace; \
      CYCLONEDDS_URI=file:///unitree_mujoco/mujoco_sim/tools/cyclonedds_lo.xml \
      PYTHONPATH=/workspace/.global:/unitree_sdk2_python:/workspace/.venv/BridgeModule/lib/python3.10/site-packages:\${PYTHONPATH:-} \
      python3 /unitree_mujoco/mujoco_sim/tools/homing_work_driver.py --plan /unitree_mujoco/mujoco_sim/logs/.work_plan_$k.json \
        --trials-log /unitree_mujoco/mujoco_sim/logs/homing_trials_$k.jsonl --out /unitree_mujoco/mujoco_sim/${OUT#$SIM/}/work_$key.jsonl \
        > /tmp/homing_work_driver.log 2>&1"
    sleep 2; docker exec "archb_locosw_$k" grep -q "waiting for the trial record" /tmp/homing_work_driver.log 2>/dev/null \
      && echo "[$k] $(date +%T) $key: work driver up" || { echo "[$k] !! work driver did not come up:"; docker exec "archb_locosw_$k" tail -5 /tmp/homing_work_driver.log; }
  fi
  python3 - "$seed" "$trials" "$DMIN" "$DMAX" "$FAN" >> "$CMD" <<'PY'
import math, random, sys
seed, trials, dmin, dmax, fan = int(sys.argv[1]), [int(v) for v in sys.argv[2].split()], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
GX, GY, GPSI = 0.35, 0.0, 0.0
for t in trials:
    random.seed(seed * 1000 + t)
    d = random.uniform(dmin, dmax); u = random.uniform(-fan, fan); th = math.radians(GPSI) + math.pi + math.radians(u)
    print(f"trial {GX + d * math.cos(th):.3f} {GY + d * math.sin(th):.3f} {random.uniform(-180, 180):.1f} {GX} {GY} {GPSI}")
PY
  echo "[$k] $(date +%T) $key: $n trials queued"
  local deadline=$(( $(date +%s) + n * 200 + 120 ))
  while [[ $(date +%s) -lt $deadline ]]; do
    [[ $(( $(wc -l < "$JL") - n0 )) -ge $n ]] && break; sleep 5; done
  tail -n +"$((n0 + 1))" "$JL" > "$OUT/trials_$key.jsonl"
  grep -a "\[homing\] trial: teleport to" "$L" > "$OUT/teleports_$key.txt"
  if [[ -n "$WORK" ]]; then   # the recorder runs until the arm sequence is done (or skipped)
    for i in $(seq 1 240); do grep -q '"kind": "done"\|"kind": "skipped"' "$OUT/work_$key.jsonl" 2>/dev/null && break; sleep 1; done
    docker exec "archb_locosw_$k" tail -3 /tmp/homing_work_driver.log 2>/dev/null | sed "s/^/[$k] /"
  fi
  sleep 3   # the recorder gets the last frames of the final settle
  kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
  echo "[$k] $(date +%T) $key: $(wc -l < "$OUT/trials_$key.jsonl") records ($(grep -c '"ARRIVED"' "$OUT/trials_$key.jsonl") ARRIVED) | $(tail -1 "$OUT/rec_$key.log")"
}
i=0
if [[ -n "$ITEMS" ]]; then
  for it in $ITEMS; do rig $((FIRST + i)) "${it%%:*}" "${it##*:}" "${it%%:*}_t${it##*:}" & i=$((i + 1)); sleep 3; done
else
  for s in $SEEDS; do rig $((FIRST + i)) "$s" "$(seq -s ' ' 1 "$N")" "$s" & i=$((i + 1)); sleep 3; done
fi
wait
echo "##### $(date +%T) done -> $OUT"
