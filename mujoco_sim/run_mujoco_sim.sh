#!/usr/bin/env bash
# ============================================================================
# run_mujoco_sim.sh — one-command Architecture B × MuJoCo balance sim.
#
# Brings up the mass-corrected H1-2 in unitree_mujoco on `lo` (host), the
# balance_metrics sidecar headless→logfile (host, tv env), and either:
#   --mode-a  (default) our balance stack with a STATIC default arm pose
#             sim_state_bridge + sim_armpose_pub + MovementModule + consumer
#   --mode-b  FULL integration: our balance legs/torso + the colleague's
#             ActionModule IK-resolved arms (real /BridgeModule/joint_set)
#   --ref     the known-good C++ reference: h1_2_ctrl --network lo (NO ROS2
#             stack) — for the apples-to-apples comparison
#
# The ROS2 nodes run in the ros2-humble-dev container (both repos mounted),
# MuJoCo + metrics run on the host. DDS is CycloneDDS on lo (domain 0), shared
# via --network host. See mujoco_sim/INTEGRATION_DESIGN.md (Addendum A6).
#
# Examples:
#   bash run_mujoco_sim.sh --mode-a                 # balance bring-up
#   bash run_mujoco_sim.sh --mode-b                 # + colleague's IK arms
#   bash run_mujoco_sim.sh --ref                    # C++ reference path
#   bash run_mujoco_sim.sh --mode-a --metrics-mode idle_quiet
#   echo "mode push" > /tmp/archb_metrics.stdin     # label a disturbance mode live
# ============================================================================
set -uo pipefail

# ── paths (edit REPOS if your layout differs) ───────────────────────────────
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="$REPOS/Aspired_Robot_Project"
MUJOCO="$REPOS/unitree_mujoco"
SDK="$REPOS/unitree_sdk2_python"
RLLAB="$REPOS/unitree_rl_lab"
SIM="$MUJOCO/mujoco_sim"
MJ_BIN="$MUJOCO/simulate/build/unitree_mujoco"
CTRL_BIN="$RLLAB/deploy/robots/h1_2/build/h1_2_ctrl"
XML="$MUJOCO/unitree_robots/h1_2/h1_2.xml"
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"
DDS_LO="$SIM/tools/cyclonedds_lo.xml"

# ── args ────────────────────────────────────────────────────────────────────
MODE="a"; GAINS="harness"; METRICS=1; METRICS_MODE="idle_quiet"
while [[ $# -gt 0 ]]; do case "$1" in
  --mode-a) MODE="a"; shift;;
  --mode-b) MODE="b"; shift;;
  --ref)    MODE="ref"; shift;;
  --gains)  GAINS="$2"; shift 2;;
  --metrics-mode) METRICS_MODE="$2"; shift 2;;
  --no-metrics)   METRICS=0; shift;;
  -h|--help) sed -n '2,33p' "$0"; exit 0;;
  *) echo "unknown arg: $1"; exit 1;;
esac; done

# ── architecture banner — make it obvious which path is running ──────────────
echo "============================================================"
case "$MODE" in
  a)   echo " ARCHITECTURE B × MuJoCo  —  MODE A (balance bring-up)"
       echo "   ROS2 balance stack: our legs/torso policy + STATIC default arms" ;;
  b)   echo " ARCHITECTURE B × MuJoCo  —  MODE B (full integration)"
       echo "   ROS2 balance stack: our legs/torso policy + ActionModule IK arms" ;;
  ref) echo " REFERENCE PATH (NOT Architecture B)  —  C++ h1_2_ctrl"
       echo "   known-good controller, for the apples-to-apples comparison" ;;
esac
echo "   model: $XML"
[[ "$MODE" != "ref" ]] && echo "   consumer gains: $GAINS"
echo "============================================================"

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
MJ_LOG="$LOG_DIR/mujoco_$STAMP.log"
METRICS_LOG="$LOG_DIR/balance_metrics_${MODE}_$STAMP.log"
METRICS_FIFO="/tmp/archb_metrics.stdin"
BAND_FLAG="$SIM/logs/.band_release"                            # host path (shared mount, gitignored)
BAND_FLAG_CTR="/unitree_mujoco/mujoco_sim/logs/.band_release"  # same file, container path

[[ -x "$MJ_BIN" ]] || { echo "ERROR: MuJoCo binary not built: $MJ_BIN"; exit 1; }
if ! ls /dev/input/js* >/dev/null 2>&1; then
  echo "NOTE: no joystick at /dev/input/js* — config has use_joystick:1. Push/command"
  echo "      tests need a gamepad; balance still runs. (Edit simulate/config.yaml"
  echo "      use_joystick:0 if the sim refuses to start without one.)"
fi

CONTAINER="archb_sim_$$"; MJ_PID=""; METRICS_PID=""; CTRL_PID=""
cleanup() {
  [[ -n "${_CLEANED:-}" ]] && return; _CLEANED=1   # run once: Ctrl+C fires INT then EXIT
  echo ""; echo ">>> cleaning up..."
  [[ -n "$METRICS_PID" ]] && { kill -INT "$METRICS_PID" 2>/dev/null; sleep 1; }  # → RUN SUMMARY into log
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  [[ -n "$CTRL_PID" ]] && kill "$CTRL_PID" 2>/dev/null || true
  [[ -n "$MJ_PID"  ]] && kill "$MJ_PID"  2>/dev/null || true
  pkill -f "balance_metrics.py" 2>/dev/null || true
  rm -f "$METRICS_FIFO" "$BAND_FLAG"
  [[ "$METRICS" = "1" ]] && echo ">>> metrics log + RUN SUMMARY: $METRICS_LOG"
  echo ">>> done."
}
trap cleanup EXIT INT TERM

# ── 1. MuJoCo (host) ────────────────────────────────────────────────────────
echo ">>> [1] launching unitree_mujoco (h1_2 D-model) on lo, domain 0..."
rm -f "$BAND_FLAG"   # clean slate so a stale flag can't pre-release the band
( cd "$MUJOCO/simulate" && ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" "$MJ_BIN" -r h1_2 -i 0 -n lo ) >"$MJ_LOG" 2>&1 &
MJ_PID=$!
echo "    pid $MJ_PID, log $MJ_LOG  (disable the elastic band in the sim window for free-standing balance)"

# ── 2. metrics sidecar (host, tv env, headless → logfile) ───────────────────
if [[ "$METRICS" = "1" ]]; then
  if [[ -x "$TV_PY" ]]; then
    rm -f "$METRICS_FIFO"; mkfifo "$METRICS_FIFO"
    sleep infinity > "$METRICS_FIFO" &   # hold the write end open so stdin doesn't EOF
    HOLD_PID=$!
    "$TV_PY" "$SIM/tools/balance_metrics.py" --iface lo --domain 0 --xml "$XML" \
        --mode "$METRICS_MODE" < "$METRICS_FIFO" > "$METRICS_LOG" 2>&1 &
    METRICS_PID=$!
    echo ">>> [2] balance_metrics headless (pid $METRICS_PID)"
    echo "    log:   $METRICS_LOG"
    echo "    label: echo \"mode <push|trainingdist|idle_quiet>\" > $METRICS_FIFO   (also: zero | note <txt>)"
  else
    echo ">>> [2] SKIP metrics: tv python not found at $TV_PY (set TV_PY=...)"
  fi
fi

# ── 3. controller / ROS2 stack ──────────────────────────────────────────────
if [[ "$MODE" = "ref" ]]; then
  [[ -x "$CTRL_BIN" ]] || { echo "ERROR: h1_2_ctrl not built: $CTRL_BIN"; exit 1; }
  echo ">>> [3] REFERENCE path: $CTRL_BIN --network lo"
  echo "    (ensure BalancePush.policy_dir → logs/milestones/p7_1b for an apples-to-apples"
  echo "     comparison with MovementModule; rebuild h1_2_ctrl if you changed it.)"
  echo "    R3 FSM on the controller terminal: FixStand → Balance. Ctrl+C here to stop all."
  "$CTRL_BIN" --network lo & CTRL_PID=$!
  wait "$CTRL_PID"
else
  echo ">>> [3] ROS2 stack in container (MODE=$MODE, gains=$GAINS)..."
  docker run --rm --name "$CONTAINER" --network host --ipc=host \
    -e MODE="$MODE" -e GAINS="$GAINS" -e ARCHB_DEBUG="${ARCHB_DEBUG:-0}" \
    -e ARCHB_FIXSTAND_SEC="${ARCHB_FIXSTAND_SEC:-1.5}" -e ARCHB_HOLD_SEC="${ARCHB_HOLD_SEC:-1.0}" -e ARCHB_ACTION_CLIP="${ARCHB_ACTION_CLIP:-5.0}" \
    -e ARCHB_BAND_RELEASE_FILE="$BAND_FLAG_CTR" \
    -v "$ASPIRED:/workspace" -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python" \
    --entrypoint bash ros2-humble-dev /unitree_mujoco/mujoco_sim/tools/_nodes_in_container.sh
fi
