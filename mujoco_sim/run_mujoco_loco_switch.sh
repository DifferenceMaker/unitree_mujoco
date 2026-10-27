#!/usr/bin/env bash
# ============================================================================
# run_mujoco_loco_switch.sh — WALK <-> BALANCE runtime policy switching through
# the REAL Arch B stack in OUR sim (2026-10-12, branch MM_loco_switch).
#
# The robot starts in FixStand on the band (as on the real harness), the WALK
# policy engages at release (kind=walk engages on itself), you drive it
# somewhere, switch to BALANCE on /MovementModule/policy, push it, switch back
# to WALK, reposition, switch to BALANCE again. The walk policy plays the role
# of "Unitree's walking" here: Unitree's controller is closed firmware on pc1
# and does not exist in MuJoCo, so the hardware-only half (BridgeModule
# take/give on /BridgeModule/loco) is NOT exercised by this rig — only the
# switching core is: shadow inference, the standing gate, the hard-cut
# handover, and the push -> reposition -> balance loop.
#
#   bash run_mujoco_loco_switch.sh                        # lm7b_u_ladder (milestone, last known good walk) + p14h_stancewide (vendored)
#   bash run_mujoco_loco_switch.sh keys                   # SECOND terminal: drive the walk; p = policy switch
#   bash run_mujoco_loco_switch.sh --policy <walk>        # --policy = --walk (fleetdeck's spelling, 2026-10-12)
#   bash run_mujoco_loco_switch.sh --walk lm5_heading --balance p16_dr_phys
#   bash run_mujoco_loco_switch.sh --desk --arms          # TABLE scene + the desk rig's arm commander once the
#                                                         # balance policy is active (alt-click the table = reach target,
#                                                         # ctrl+alt-click = anchor); fleetdeck's "policy switchable test"
#   bash tools/loco_switch_scenario.sh                     # the scripted test (3rd terminal)
#   bash run_mujoco_loco_switch.sh stop
#   MM_LOCO_SWITCH=unitree BRIDGE_SIM_FAKE_LOCO=1 bash run_mujoco_loco_switch.sh
#       PLUMBING ONLY (2026-10-20, Aspired branch MM_loco_switch_unitree): 'unitree' on
#       /MovementModule/policy makes MovementModule send the Bridge 'give-stand'; the SIM Bridge
#       fakes the owner flip (no Unitree controller here: the robot SAGS under the shadow-follow),
#       the stick is relayed as 'move', a policy name sends stop/take and the HANDOFF engages.
#       Message flow only -- the physics of the real handover cannot be seen in MuJoCo.
#
# The balance policy must be VENDORED under Aspired_Robot_Project/MovementModule/
# policy/ (default = that repo's CURRENT_BALANCE pointer, p14h_stancewide if the
# file is missing); the walk policy may be vendored OR a milestone name (staged, like the
# walk-line rig). Both should be trained at EQUAL PD gains (MovementModule warns
# otherwise: BridgeModule applies one gain set per session).
#
# Control from any terminal while it runs:
#   switch:  docker exec archb_locosw bash -c 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /MovementModule/policy std_msgs/String "data: p14h_stancewide"'
#   drive:   printf '\033[A' > mujoco_sim/logs/.teleop_keys     (Up = +0.05 m/s per press; space = zero)
#   push:    echo "push 0.5 0" > mujoco_sim/logs/.sim_stdin      (m/s, world frame)
#
# ISOLATION (do not weaken — 2026-07-03 incidents): ROS domain 77 +
# ROS_LOCALHOST_ONLY=1; unitree-SDK DDS domain 1 on lo (real robot = domain 0).
# ============================================================================
set -uo pipefail

REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="${ASPIRED:-$REPOS/Aspired_Robot_Project}"   # a worktree on another MM branch via ASPIRED=... (homing rig: Aspired_Robot_Project-homing)
MUJOCO="$REPOS/unitree_mujoco"
SDK="$REPOS/unitree_sdk2_python"
SIM="$MUJOCO/mujoco_sim"
MJ_BIN="$MUJOCO/simulate/build/unitree_mujoco"

# default balance = the deployment pointer (Aspired_Robot_Project/MovementModule/policy/CURRENT_BALANCE)
CUR_BAL="$(tr -d '[:space:]' < "$ASPIRED/MovementModule/policy/CURRENT_BALANCE" 2>/dev/null)"
PROFILE="run"; WALK="${LOCOSW_WALK:-lm7b_u_ladder}"; BALANCE="${LOCOSW_BALANCE:-${CUR_BAL:-p14h_stancewide}}"; DEBUG=1; METRICS="${METRICS:-1}"
RECORD_FILE=""; DESK=0; ARMS=0
while [[ $# -gt 0 ]]; do case "$1" in
  keys|stop) PROFILE="$1"; shift;;
  --walk|--policy) WALK="$2"; shift 2;;      # --policy: fleetdeck passes every rig its policy this way
  --balance) BALANCE="$2"; shift 2;;
  --record-file) RECORD_FILE="$2"; shift 2;;  # in-sim recording (ARCHB_RECORD_FILE), fleetdeck's record toggle
  --desk)    DESK=1; shift;;                  # the table scene (<scene>_desk.xml variant of the auto scene)
  --arms)    ARMS=1; shift;;                  # arm_ik_commander gated on the balance policy (alt-click reach targets)
  --quiet)   DEBUG=0; shift;;
  --no-metrics) METRICS=0; shift;;
  -h|--help) sed -n '2,32p' "$0"; exit 0;;
  *) echo "unknown arg: $1"; exit 1;;
esac; done

KFIFO="$SIM/logs/.teleop_keys"
SFIFO="$SIM/logs/.sim_stdin"
CONTAINER="archb_locosw"
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"

if [[ "$PROFILE" == "keys" ]]; then
  [[ -p "$KFIFO" ]] || mkfifo "$KFIFO"
  echo "WALK KEYS — this terminal drives the walk policy (arrows = vx/vy, q/e = yaw, space = zero, ESC = quit)."
  echo "            p = POLICY SWITCH (walk <-> balance; zeroes the stick, MovementModule switches when the standing gate holds)"
  stty -icanon min 1 time 0 -echo; trap 'stty sane' EXIT INT TERM
  python3 "$SIM/tools/teleop_keys_feed.py" "$KFIFO"
  exit 0
fi
if [[ "$PROFILE" == "stop" ]]; then
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  pkill -x unitree_mujoco 2>/dev/null || true
  pkill -f "balance_metrics.py" 2>/dev/null || true
  echo "stopped."; exit 0
fi

for f in policy.onnx deploy.yaml; do
  [[ -f "$ASPIRED/MovementModule/policy/$BALANCE/$f" ]] || { echo "FATAL: balance policy must be vendored: $ASPIRED/MovementModule/policy/$BALANCE/$f missing"; exit 2; }
done
[[ -x "$MJ_BIN" ]] || { echo "ERROR: MuJoCo binary not built: $MJ_BIN"; exit 1; }

# ── walk policy: a VENDORED dir under MovementModule/policy, or a MILESTONE ──
# (logs/milestones or the repo archive, same resolver as run_mujoco_walk_line.sh).
# A milestone is STAGED next to a copy of the vendored balance policy in a fresh
# per-run dir that is overlay-mounted over /workspace/MovementModule/policy, so
# MovementModule sees both and nothing lands in the repo tree.
MILESTONES="$REPOS/unitree_rl_lab/logs/milestones"
resolve_ms() {
  [[ -d "$MILESTONES/$1" ]] && { echo "$1"; return; }
  local m
  m=$(ls -dt "$MILESTONES"/*"_${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/"${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/*"$1"* 2>/dev/null | head -1)
  [[ -n "$m" ]] && basename "$m"
}
POLICY_MOUNT="$ASPIRED/MovementModule/policy"; WALK_NAME="$WALK"; WALK_ENV=""
if [[ -f "$ASPIRED/MovementModule/policy/$WALK/policy.onnx" && -f "$ASPIRED/MovementModule/policy/$WALK/deploy.yaml" ]]; then
  echo ">>> [policy] walk = vendored MovementModule/policy/$WALK"
else
  MS="$(resolve_ms "$WALK")"
  if [[ -z "$MS" || ! -f "$MILESTONES/$MS/exported/policy.onnx" ]]; then
    MILESTONES="$REPOS/aspired-isaac-lab/milestone_checkpoints"; MS="$(resolve_ms "$WALK")"
  fi
  [[ -n "$MS" && -f "$MILESTONES/$MS/exported/policy.onnx" && -f "$MILESTONES/$MS/params/deploy.yaml" ]] || {
    echo "FATAL: '$WALK' is neither vendored under MovementModule/policy nor a milestone with exported/policy.onnx + params/deploy.yaml"; exit 2; }
  STAGE="$SIM/logs/.locosw_stage_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$STAGE/$MS"
  cp "$MILESTONES/$MS/exported/policy.onnx" "$STAGE/$MS/policy.onnx"
  cp "$MILESTONES/$MS/params/deploy.yaml"   "$STAGE/$MS/deploy.yaml"
  cp "$MILESTONES/$MS/MILESTONE.md" "$STAGE/$MS/" 2>/dev/null || true
  cp -r "$ASPIRED/MovementModule/policy/$BALANCE" "$STAGE/$BALANCE"
  POLICY_MOUNT="$STAGE"; WALK_NAME="$MS"; WALK_ENV="$MILESTONES/$MS/params/env.yaml"
  echo ">>> [policy] walk = milestone $MILESTONES/$MS, staged with $BALANCE in $STAGE (overlay over MovementModule/policy)"
fi

# ── scene: both default policies train on the hand790 body + Unitree armature ─
SCENE_AUTO="scene_comx06_armature_hand790.xml"
if [[ -n "$WALK_ENV" && -f "$WALK_ENV" ]]; then
  if grep -q "h1_2_comx06_hand790" "$WALK_ENV"; then SCENE_AUTO="scene_comx06_armature_hand790.xml"
  elif grep -qE "armature: 0\.16" "$WALK_ENV"; then SCENE_AUTO="scene_comx06_armature.xml"
  else SCENE_AUTO="scene_comx06.xml"; fi
fi
[[ $DESK -eq 1 ]] && SCENE_AUTO="${SCENE_AUTO%.xml}_desk.xml"   # scene_comx06_armature_hand790_desk.xml etc.
SCENE="${LOCOSW_SCENE:-$SCENE_AUTO}"
[[ -f "$MUJOCO/unitree_robots/h1_2/$SCENE" ]] || { echo "FATAL: scene $SCENE not in unitree_robots/h1_2/"; exit 2; }
case "${SCENE%_desk.xml}.xml" in
  scene_comx06_armature_hand790.xml) XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06_armature_hand790.xml" ;;
  scene_comx06_armature.xml)         XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06_armature.xml" ;;
  *)                                 XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06.xml" ;;
esac
if ! grep -qE "robot_scene: \"$SCENE\"" "$MUJOCO/simulate/config.yaml"; then
  sed -i "s/robot_scene: \"[^\"]*\"/robot_scene: \"$SCENE\"/" "$MUJOCO/simulate/config.yaml"
fi
echo ">>> [scene] robot_scene -> $SCENE (LOCOSW_SCENE overrides)"

# ── sweep leftovers from an aborted run ──────────────────────────────────────
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
pkill -x unitree_mujoco 2>/dev/null && echo ">>> [sweep] killed a leftover unitree_mujoco sim"
pkill -f "balance_metrics.py" 2>/dev/null && echo ">>> [sweep] killed a leftover balance_metrics"

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
MJ_LOG="$LOG_DIR/mujoco_locosw_$STAMP.log"
STACK_LOG="$LOG_DIR/stack_locosw_$STAMP.log"
METRICS_LOG="$LOG_DIR/locosw_metrics_$STAMP.log"
BAND_FLAG="$SIM/logs/.band_release"
BAND_FLAG_CTR="/unitree_mujoco/mujoco_sim/logs/.band_release"
ln -sfn "$STACK_LOG" "$LOG_DIR/stack_locosw_latest.log"
ln -sfn "$METRICS_LOG" "$LOG_DIR/locosw_metrics_latest.log"

cleanup() {
  [[ -n "${METRICS_PID:-}" ]] && { kill -INT "$METRICS_PID" 2>/dev/null; sleep 1; }   # -> RUN SUMMARY (falls, touchdowns)
  [[ -n "${WC_PID:-}" ]] && kill "$WC_PID" 2>/dev/null
  [[ -n "${KHOLD_PID:-}" ]] && kill "$KHOLD_PID" 2>/dev/null
  [[ -n "${SHOLD_PID:-}" ]] && kill "$SHOLD_PID" 2>/dev/null
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  [[ -n "${MJ_PID:-}" ]] && kill "$MJ_PID" 2>/dev/null
  [ -e "$BAND_FLAG" ] && unlink "$BAND_FLAG"
}
trap cleanup EXIT INT TERM

[ -e "$BAND_FLAG" ] && unlink "$BAND_FLAG"
# FIFOs with a holder writer each, so a one-shot `printf > fifo` never gives the
# reader EOF (walk_teleop) and the sim's `push` reader keeps a live stdin.
for _f in "$KFIFO" "$SFIFO"; do [ -e "$_f" ] && unlink "$_f"; done; mkfifo "$KFIFO" "$SFIFO"
# alt-click reach targets: the sim appends "l x y z" lines to logs/.arm_targets (click_target.h default
# path, relative to simulate/); clear a stale file so the commander does not replay last run's clicks
if [[ $ARMS -eq 1 ]]; then
  [ -e "$SIM/logs/.arm_targets" ] && unlink "$SIM/logs/.arm_targets"
  : > "$SIM/logs/.arm_targets"
  # the desk rig's raised start pose (hands ~desk + 10 cm) -- taken up when the balance policy becomes active
  echo "l 0.30 0.25 0.10" >> "$SIM/logs/.arm_targets"; echo "r 0.30 -0.25 0.10" >> "$SIM/logs/.arm_targets"
fi
sleep infinity > "$KFIFO" & KHOLD_PID=$!
sleep infinity > "$SFIFO" & SHOLD_PID=$!

echo ">>> [1] launching unitree_mujoco (h1_2, $SCENE) on lo, domain 1 — stdin <- $SFIFO"
( cd "$MUJOCO/simulate" && env -u WAYLAND_DISPLAY GLFW_PLATFORM=x11 \
    ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" ARCHB_RECORD_FILE="$RECORD_FILE" \
    "$MJ_BIN" -r h1_2 -i 1 -n lo < "$SFIFO" ) >"$MJ_LOG" 2>&1 &
MJ_PID=$!
echo "    pid $MJ_PID, log $MJ_LOG"
sleep 3
if ! kill -0 "$MJ_PID" 2>/dev/null; then
  echo "ERROR: MuJoCo exited during startup — see $MJ_LOG"; tail -3 "$MJ_LOG" | sed 's/^/    /'
  echo "       Aborting: refusing to start the ROS2 stack without a live sim."; exit 1
fi
# WINDOW watchdog (2026-10-12): closing the MuJoCo window did NOT end the sim process here -- its stdin
# is the push FIFO, held open by a writer, so the stdin command thread never sees EOF and the process
# outlived its window (the walk rig's sim gets /dev/null as a background job and exits). The operator
# closes the window to end a test and expects the consoles to follow (as on the walk rig): once the
# sim's window has existed and is gone while the pid lives, kill the sim -> the sim-death watchdog below
# tears the stack down. Needs wmctrl; without it this rig behaves as before.
if command -v wmctrl >/dev/null 2>&1; then
  ( seen=0
    while kill -0 "$MJ_PID" 2>/dev/null; do
      if wmctrl -lp 2>/dev/null | awk -v p="$MJ_PID" '$3==p{f=1} END{exit !f}'; then seen=1
      elif [[ $seen -eq 1 ]]; then
        echo ""; echo ">>> [watchdog] MuJoCo window closed — ending the sim (pid $MJ_PID)"
        kill "$MJ_PID" 2>/dev/null; sleep 3; kill -9 "$MJ_PID" 2>/dev/null; break
      fi
      sleep 1
    done ) &
fi
# sim-death watchdog: never leave a phantom controller without its robot
( while kill -0 "$MJ_PID" 2>/dev/null; do sleep 2; done
  echo ""; echo ">>> [watchdog] sim died — tearing down the stack"
  docker rm -f "$CONTAINER" >/dev/null 2>&1 ) &

if [[ "$METRICS" = "1" && -x "$TV_PY" ]]; then
  "$TV_PY" "$SIM/tools/balance_metrics.py" --iface lo --domain 1 --xml "$XML" \
      --mode switch < /dev/null > "$METRICS_LOG" 2>&1 &
  METRICS_PID=$!
  echo ">>> [1b] balance_metrics headless (pid $METRICS_PID; falls/touchdowns -> $METRICS_LOG)"
fi
# walk_hud gauge feeder (the walk rig's): the container's walk_teleop has no unitree_sdk2py, so a host
# mirror tails the stack log and publishes rt/wirelesscontroller -- the bottom-right vx/vy/wz gauges
# auto-hide without it (missing here until 2026-10-12: "the tracking gauges don't show up")
( sleep 6; "$TV_PY" "$SIM/tools/wc_mirror.py" "$STACK_LOG" 1 lo ) > "$LOG_DIR/wc_mirror_locosw_$STAMP.log" 2>&1 &
WC_PID=$!

echo ">>> [2] REAL Arch B stack: BridgeModule --sim + MovementModule [$WALK_NAME active, $BALANCE shadow] + walk_teleop$([[ $ARMS -eq 1 ]] && echo ' + arm_ik_commander (gated on the balance policy; alt-click = reach target)')"
echo "    switch:  docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /MovementModule/policy std_msgs/String \"data: $BALANCE\"'"
echo "    drive:   bash $SIM/run_mujoco_loco_switch.sh keys   |  printf '\\033[A' > $KFIFO   (p = policy switch)"
echo "    push:    echo \"push 0.5 0\" > $SFIFO"
echo "    scenario: bash $SIM/tools/loco_switch_scenario.sh"
echo "    stack log: $STACK_LOG"
docker run --rm --name "$CONTAINER" --network host --ipc=host \
  -e ROS_DOMAIN_ID="${ARCHB_ROS_DOMAIN:-77}" -e ROS_LOCALHOST_ONLY=1 \
  -e BRIDGE_DDS_DOMAIN=1 \
  -e MODE="${LOCOSW_MODE:-w}" -e ARCHB_DEBUG="$DEBUG" -e ARCHB_ARM_IK="$ARMS" \
  ${HOMING_POSE:+-e HOMING_POSE="$HOMING_POSE"} ${HOMING_BALANCE+-e HOMING_BALANCE="$HOMING_BALANCE"} ${HOMING_VMAX:+-e HOMING_VMAX="$HOMING_VMAX"} ${HOMING_POSE_NOISE_XY:+-e HOMING_POSE_NOISE_XY="$HOMING_POSE_NOISE_XY"} ${HOMING_POSE_NOISE_YAW:+-e HOMING_POSE_NOISE_YAW="$HOMING_POSE_NOISE_YAW"} ${HOMING_POSE_BIAS_XY:+-e HOMING_POSE_BIAS_XY="$HOMING_POSE_BIAS_XY"} ${HOMING_TORSO_FRONT:+-e HOMING_TORSO_FRONT="$HOMING_TORSO_FRONT"} ${HOMING_STOP_DIST:+-e HOMING_STOP_DIST="$HOMING_STOP_DIST"} ${HOMING_HOLD_S:+-e HOMING_HOLD_S="$HOMING_HOLD_S"} ${HOMING_SEED:+-e HOMING_SEED="$HOMING_SEED"} ${HOMING_TRAJ_DIR:+-e HOMING_TRAJ_DIR="$HOMING_TRAJ_DIR"} ${HOMING_TRAJ_HZ:+-e HOMING_TRAJ_HZ="$HOMING_TRAJ_HZ"} \
  -e MM_POLICY="$WALK_NAME" -e MM_SWITCH_POLICIES="$BALANCE" \
  -e MM_SWITCH_BLEND_SEC="${MM_SWITCH_BLEND_SEC:-0.0}" -e MM_SWITCH_HOLD_SEC="${MM_SWITCH_HOLD_SEC:-0.3}" \
  -e MM_SWITCH_GYRO="${MM_SWITCH_GYRO:-0.3}" -e MM_SWITCH_REQUIRE_LOAD="${MM_SWITCH_REQUIRE_LOAD:-1}" \
  -e MM_SWITCH_FORCE_SEC="${MM_SWITCH_FORCE_SEC:-0}" \
  -e MM_LOCO_SWITCH="${MM_LOCO_SWITCH:-aspired}" -e MM_LOCO_GIVE_CMD="${MM_LOCO_GIVE_CMD:-give-stand}" \
  -e MM_LOCO_SWITCH_TIMEOUT_S="${MM_LOCO_SWITCH_TIMEOUT_S:-10}" -e MM_LOCO_SETTLE_S="${MM_LOCO_SETTLE_S:-2.0}" \
  -e MM_LOCO_MOVE_RATE_HZ="${MM_LOCO_MOVE_RATE_HZ:-5}" -e MM_LOCO_MOVE_DUR_S="${MM_LOCO_MOVE_DUR_S:-0.6}" -e MM_LOCO_VMAX="${MM_LOCO_VMAX:-0.5,0.3,0.5}" \
  -e BRIDGE_SIM_FAKE_LOCO="${BRIDGE_SIM_FAKE_LOCO:-0}" \
  -e ARCHB_WALK_ENGAGE_MODE="${ARCHB_WALK_ENGAGE_MODE:-none}" \
  ${ARCHB_ENGAGE_MODE:+-e ARCHB_ENGAGE_MODE="$ARCHB_ENGAGE_MODE"} \
  -e ARCHB_FIXSTAND_SEC="${ARCHB_FIXSTAND_SEC:-1.0}" -e ARCHB_HOLD_SEC="${ARCHB_HOLD_SEC:-3.5}" \
  -e ARCHB_ACTION_CLIP="${ARCHB_ACTION_CLIP:-100.0}" \
  -e ARCHB_YAW_HOLD="${ARCHB_YAW_HOLD:-1}" -e ARCHB_YAW_HOLD_K="${ARCHB_YAW_HOLD_K:-1.0}" -e ARCHB_YAW_HOLD_MAX="${ARCHB_YAW_HOLD_MAX:-0.4}" -e ARCHB_YAW_HOLD_MIN="${ARCHB_YAW_HOLD_MIN:-0.0}" -e ARCHB_YAW_HOLD_DEADBAND="${ARCHB_YAW_HOLD_DEADBAND:-0.03}" \
  -e ARCHB_ENGAGE_BLEND_SEC="${ARCHB_ENGAGE_BLEND_SEC:-0.3}" -e ARCHB_LOAD_STEPS="${ARCHB_LOAD_STEPS:-3}" \
  -e BRIDGE_GETTER_MIN_DT="${BRIDGE_GETTER_MIN_DT:-0.002}" -e BRIDGE_LEG_SLEW_SCALE="${BRIDGE_LEG_SLEW_SCALE:-4.0}" \
  -e BRIDGE_IMU_PERIOD="${BRIDGE_IMU_PERIOD:-0.002}" -e EMERGENCY_SRV="${EMERGENCY_SRV:-0}" \
  -e ARCHB_BAND_RELEASE_FILE="$BAND_FLAG_CTR" \
  -v "$ASPIRED:/workspace" ${ASPIRED_VENV:+-v "$ASPIRED_VENV:/workspace/.venv"} -v "$POLICY_MOUNT:/workspace/MovementModule/policy" \
  -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python" \
  --entrypoint bash ros2-humble-dev /unitree_mujoco/mujoco_sim/tools/_nodes_in_container.sh \
  2>&1 | tee "$STACK_LOG"
