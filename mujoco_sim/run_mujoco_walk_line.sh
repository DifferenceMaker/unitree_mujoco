#!/usr/bin/env bash
# ============================================================================
# run_mujoco_walk_line.sh — the WALK-LINE policy through the REAL Arch B stack,
# in OUR sim (2026-08-24: the .testing python sim ran at ~4 fps / rtf 0.5-0.8
# and every run fell — operator: "forego my colleague's .testing pipeline and
# reuse ours". This is the original arch-B flow: the C++ unitree_mujoco sim at
# true rtf 1.0 on the DDS plane, the colleague's REAL BridgeModule --sim +
# MovementModule on ROS, zero sim shims).
#
#   bash run_mujoco_walk_line.sh                    # walk (lm5_heading)
#   bash run_mujoco_walk_line.sh keys               # SECOND terminal: drive it
#   bash run_mujoco_walk_line.sh --policy lm5_lcpr  # any walk milestone
#
# Chain (identical contracts to the real robot):
#   sim rt/lowstate -> BridgeModule(BRIDGE_SIM=1) -> /BridgeModule/joints_imu_b
#   -> MovementModule (policy_contract KIND=walk: 90-obs velocity_commands,
#      27 actions) -> /BridgeModule/joint_set_body -> BridgeModule -> rt/lowcmd
#   keys terminal -> FIFO -> walk_teleop -> /MovementModule/cmd_vel
#
# Keys (in the 'keys' terminal): arrows = vx / vy, q/e = yaw, space = zero,
# tab = sticky, ESC = quit teleop.
#
# ISOLATION (do not weaken — 2026-07-03 incidents): ROS domain 77 +
# ROS_LOCALHOST_ONLY=1; unitree-SDK DDS domain 1 on lo (real robot = domain 0).
# ============================================================================
set -uo pipefail

REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="$REPOS/Aspired_Robot_Project"
MUJOCO="$REPOS/unitree_mujoco"
SDK="$REPOS/unitree_sdk2_python"
RLLAB="$REPOS/unitree_rl_lab"
SIM="$MUJOCO/mujoco_sim"
MJ_BIN="$MUJOCO/simulate/build/unitree_mujoco"

PROFILE="walk"; POLICY="lm5_heading"; DEBUG=1; RECORD=0
while [[ $# -gt 0 ]]; do case "$1" in
  walk|keys|stop) PROFILE="$1"; shift;;
  --policy) POLICY="$2"; shift 2;;
  --quiet)  DEBUG=0; shift;;
  --record) RECORD=1; shift;;
  -h|--help) sed -n '2,24p' "$0"; exit 0;;
  *) echo "unknown arg: $1 (profiles: walk | keys | stop)"; exit 1;;
esac; done

KFIFO="$SIM/logs/.teleop_keys"
if [[ "$PROFILE" == "keys" ]]; then
  [[ -p "$KFIFO" ]] || mkfifo "$KFIFO"
  echo "WALK KEYS — this terminal now drives the robot (logs stay in the other one)."
  echo "  arrows = vx/vy   q/e = yaw   space = zero   tab = sticky   ESC = quit"
  stty -icanon min 1 time 0 -echo; trap 'stty sane' EXIT INT TERM
  python3 "$SIM/tools/teleop_keys_feed.py" "$KFIFO"
  exit 0
fi
if [[ "$PROFILE" == "stop" ]]; then
  docker ps --format '{{.Names}}' | grep '^archb_walk_' | xargs -r docker rm -f
  pkill -x unitree_mujoco 2>/dev/null || true
  echo "stopped."; exit 0
fi

# ── walk policy staging (overlay-mounted over MovementModule/policy) ─────────
MILESTONES="$RLLAB/logs/milestones"
resolve_ms() {
  [[ -d "$MILESTONES/$1" ]] && { echo "$1"; return; }
  local m
  m=$(ls -dt "$MILESTONES"/*"_${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/"${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/*"$1"* 2>/dev/null | head -1)
  [[ -n "$m" ]] && basename "$m"
}
MS="$(resolve_ms "$POLICY")"
# FALLBACK ROOT (2026-08-25): pod-harvested milestones exist only in the repo
# archive (aspired-isaac-lab/milestone_checkpoints carries exported/ + params/
# via git) — not under this machine's logs/milestones. Same layout, so retry there.
if [[ -z "$MS" || ! -d "$MILESTONES/$MS" ]]; then
  MILESTONES="$REPOS/aspired-isaac-lab/milestone_checkpoints"
  MS="$(resolve_ms "$POLICY")"
  [[ -n "$MS" ]] && echo ">>> [policy] resolved from the repo archive: $MILESTONES/$MS"
fi
[[ -n "$MS" && -d "$MILESTONES/$MS" ]] || { echo "FATAL: no milestone matches --policy '$POLICY' under $MILESTONES"; exit 2; }
for f in exported/policy.onnx params/deploy.yaml; do
  [[ -f "$MILESTONES/$MS/$f" ]] || { echo "FATAL: $MS missing $f"; exit 2; }
done
STAGE="$SIM/logs/.walk_policy_stage"
rm -rf "$STAGE"; mkdir -p "$STAGE/$MS"
cp "$MILESTONES/$MS/exported/policy.onnx" "$STAGE/$MS/policy.onnx"
cp "$MILESTONES/$MS/params/deploy.yaml"   "$STAGE/$MS/deploy.yaml"
cp "$MILESTONES/$MS/MILESTONE.md" "$STAGE/$MS/" 2>/dev/null || true
printf '%s\n' "$MS" > "$STAGE/CURRENT"
echo ">>> [policy] staged $MS -> $STAGE (container sees MovementModule/policy/CURRENT)"

# ── walk scene: comx06 body + rigid floor, NO desk (new-soles-era default) ───
if ! grep -qE 'robot_scene: "scene_comx06.xml"' "$MUJOCO/simulate/config.yaml"; then
  sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_comx06.xml"/' "$MUJOCO/simulate/config.yaml"
  echo ">>> [scene] robot_scene -> scene_comx06.xml (comx06 body, rigid floor, no desk)"
fi

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
MJ_LOG="$LOG_DIR/mujoco_walk_$STAMP.log"
RECORD_FILE="$LOG_DIR/mujoco_walk_rec_$STAMP.mp4"
BAND_FLAG="$SIM/logs/.band_release"
BAND_FLAG_CTR="/unitree_mujoco/mujoco_sim/logs/.band_release"
CONTAINER="archb_walk_$$"

cleanup() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  [[ -n "${MJ_PID:-}" ]] && kill "$MJ_PID" 2>/dev/null
  rm -f "$BAND_FLAG"
}
trap cleanup EXIT INT TERM

rm -f "$BAND_FLAG"
rm -f "$KFIFO"; mkfifo "$KFIFO"    # walk_teleop opens it read-side (nonblocking)

echo ">>> [1] launching unitree_mujoco (h1_2, scene_comx06.xml) on lo, domain 1..."
ulimit -c unlimited 2>/dev/null
( cd "$MUJOCO/simulate" && env -u WAYLAND_DISPLAY GLFW_PLATFORM=x11 \
    ARCHB_RECORD_FILE="$([[ $RECORD = 1 ]] && echo "$RECORD_FILE")" \
    ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" \
    "$MJ_BIN" -r h1_2 -i 1 -n lo ) >"$MJ_LOG" 2>&1 &
MJ_PID=$!
echo "    pid $MJ_PID, log $MJ_LOG"
sleep 3
if ! kill -0 "$MJ_PID" 2>/dev/null; then
  echo "ERROR: MuJoCo exited during startup — see $MJ_LOG"; tail -3 "$MJ_LOG" | sed 's/^/    /'
  echo "       Aborting: refusing to start the ROS2 stack without a live sim."; exit 1
fi

# sim-death watchdog: never leave a phantom controller without its robot
( while kill -0 "$MJ_PID" 2>/dev/null; do sleep 2; done
  echo ""; echo ">>> [watchdog] sim died — tearing down the stack"
  docker rm -f "$CONTAINER" >/dev/null 2>&1 ) &

echo ">>> [2] REAL Arch B stack (BridgeModule --sim + MovementModule walk kind + walk_teleop)"
echo "    drive it:   bash $SIM/run_mujoco_walk_line.sh keys      (second terminal)"
echo "    stack log:  $LOG_DIR/stack_walk_$STAMP.log"
docker run --rm --name "$CONTAINER" --network host --ipc=host \
  -e ROS_DOMAIN_ID="${ARCHB_ROS_DOMAIN:-77}" -e ROS_LOCALHOST_ONLY=1 \
  -e BRIDGE_DDS_DOMAIN=1 \
  -e MODE=w -e ARCHB_DEBUG="$DEBUG" \
  -e ARCHB_FIXSTAND_SEC="${ARCHB_FIXSTAND_SEC:-1.0}" -e ARCHB_HOLD_SEC="${ARCHB_HOLD_SEC:-3.5}" \
  -e ARCHB_ACTION_CLIP="${ARCHB_ACTION_CLIP:-100.0}" \
  -e ARCHB_ENGAGE_BLEND_SEC="${ARCHB_ENGAGE_BLEND_SEC:-0.3}" -e ARCHB_LOAD_STEPS="${ARCHB_LOAD_STEPS:-3}" \
  -e BRIDGE_GETTER_MIN_DT="${BRIDGE_GETTER_MIN_DT:-0.002}" -e BRIDGE_LEG_SLEW_SCALE="${BRIDGE_LEG_SLEW_SCALE:-4.0}" \
  -e BRIDGE_IMU_PERIOD="${BRIDGE_IMU_PERIOD:-0.002}" -e EMERGENCY_SRV="${EMERGENCY_SRV:-0}" \
  -e ARCHB_BAND_RELEASE_FILE="$BAND_FLAG_CTR" \
  -v "$ASPIRED:/workspace" -v "$STAGE:/workspace/MovementModule/policy" \
  -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python" \
  --entrypoint bash ros2-humble-dev /unitree_mujoco/mujoco_sim/tools/_nodes_in_container.sh \
  2>&1 | tee "$LOG_DIR/stack_walk_$STAMP.log"
