#!/usr/bin/env bash
# _nodes_in_container.sh — bring up the REAL Architecture B v2 stack inside the
# ros2-humble-dev container, against rt/lowstate/rt/lowcmd already on `lo`
# (MuJoCo runs on the host; --network host shares the bus).
#
# REAL modules only — no sim shims. BridgeModule runs in --sim mode (no camera /
# hands / MotionSwitcher) and owns ALL robot DDS I/O; MovementModule attaches to
# it directly via /BridgeModule/joint_set_legs + /BridgeModule/joint_set_arms
# (arch_b_v2 contract) and does its own FixStand->hold->engage bring-up.
#
# Invoked by run_mujoco_sim.sh via docker run. Env in:
#   MODE = a (no arm source: MovementModule falls back to measured arms)
#        | b (arm_ik_commander dots -> /BridgeModule/joint_set_arms)
#        | c (the REAL ActionModule + teleop keyboard: WASD/QE = left hand,
#             IK-resolved by the colleague's stack -> joint_set_arms.
#             Needs docker -it: teleop reads /dev/tty.)
#   ARCHB_FIXSTAND_SEC / ARCHB_HOLD_SEC / ARCHB_ACTION_CLIP / ARCHB_DEBUG /
#   ARCHB_BAND_RELEASE_FILE  -> consumed by MovementModule's bring-up
# Blocks until the container is stopped (SIGTERM) — then kills the children.
source /opt/ros/humble/setup.bash   # before any set -u

export ASPIRED_ROOT=/workspace
export CYCLONEDDS_URI="file:///unitree_mujoco/mujoco_sim/tools/cyclonedds_lo.xml"
MODE="${MODE:-a}"

echo "============================================================"
echo " ARCHITECTURE B v2 (REAL modules: BridgeModule --sim + MovementModule)"
echo "   MODE ${MODE}  (a = arms-hold fallback, b = ActionModule IK arms)"
echo "============================================================"

echo ">>> [container] installing unitree_sdk2py for BridgeModule"
pip install -e /unitree_sdk2_python -q 2>/dev/null || pip install unitree_sdk2py -q 2>/dev/null || true
python3 -c "import unitree_sdk2py" 2>/dev/null || { echo "FATAL: unitree_sdk2py unavailable"; exit 2; }

PIDS=()
cleanup() { [[ -n "${_CLEANED:-}" ]] && return; _CLEANED=1; echo ">>> [container] stopping nodes"; kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

echo ">>> [container] starting REAL BridgeModule (BRIDGE_SIM=1, iface lo, SDK-DDS domain ${BRIDGE_DDS_DOMAIN:-0})"
# BRIDGE_GAINS_FROM_POLICY: Bridge reads ALL 27 PD gains (arms included) from
# the ACTIVE policy's deploy.yaml (MovementModule/policy/CURRENT) — policies
# are gain-adapted; wrong arm gains invalidate the eval (2026-07-06 handoff).
( PYTHONPATH="/workspace/.global:${PYTHONPATH:-}" \
  BRIDGE_GAINS_FROM_POLICY=/workspace/MovementModule/policy \
  BRIDGE_SIM=1 python3 /workspace/BridgeModule/main/main.py lo ) & PIDS+=($!)
sleep 2

if [ "$MODE" = "b" ]; then
  echo ">>> [container] MODE B — arm_ik_commander (red/blue dot Cartesian targets"
  echo "      -> ActionModule ikpy IK -> /BridgeModule/joint_set_arms)."
  echo "      Command from the host:  echo \"l 0.35 0.25 0.10\" >> mujoco_sim/logs/.arm_targets"
  echo "      (also: 'r x y z' | 'default';  ARM_IK_DEMO=1 auto-cycles targets)"
  pip install -q ikpy 2>/dev/null || true   # pure-python IK dep (no ActionModule venv exists)
  ( PYTHONPATH="/workspace/.global:/workspace/ActionModule:${PYTHONPATH:-}" \
    ARM_TARGETS_FILE=/unitree_mujoco/mujoco_sim/logs/.arm_targets \
    python3 /workspace/ActionModule/Utils/arm_ik_commander.py ) & PIDS+=($!)
elif [ "$MODE" = "c" ]; then
  echo ">>> [container] MODE C — REAL ActionModule (his MoveIt/ernest IK stack),"
  echo "      launched FOREGROUND below so THIS terminal's keyboard drives teleop."
  pip install -q ikpy scipy scikit-learn 2>/dev/null || true   # ActionModule deps (no venv exists)
  ln -sfn /workspace/.global /global               # ActionModule hardcodes /global/... paths
else
  echo ">>> [container] MODE A — no arm source; MovementModule uses measured-arms fallback"
fi

echo ">>> [container] starting MovementModule (FixStand->hold->policy -> /BridgeModule/joint_set_legs)"
( source /workspace/.venv/MovementModule/bin/activate
  PYTHONPATH="/workspace/.global:${PYTHONPATH:-}" \
  python3 /workspace/MovementModule/main/main.py ) & PIDS+=($!)

if [ "$MODE" = "c" ]; then
  # ActionModule runs FOREGROUND as this tty's owner: teleop's /dev/tty raw-mode
  # key capture works only for the foreground process group (backgrounded, the
  # keys just echoed into the console — the 2026-07-06 'can't type' failure).
  # When ActionModule exits (ESC then Ctrl+C), the trap tears everything down.
  ( sleep 15
    echo ">>> [container] triggering teleop sequence (/ActionModule/run <- 'teleop')"
    ros2 topic pub --once /ActionModule/run std_msgs/String "data: teleop" >/dev/null 2>&1 ) &
  echo ">>> [container] all nodes up. ActionModule in FOREGROUND — teleop auto-starts in ~15 s."
  echo "      KEYS (type here): w/s=+x/-x  a/d=+y/-y  q/e=+z/-z (left hand),"
  echo "      i/k j/l u/o = roll/pitch/yaw, p = print pose, ESC = quit, then Ctrl+C."
  # No IK_ENGINE override: MoveIt is available in this container, so ActionModule
  # runs the SAME moveit+ernest resolvers as the real robot (full parity —
  # verified live 2026-07-07, "You can start planning now!").
  PYTHONPATH="/workspace/.global:/workspace/ActionModule:${PYTHONPATH:-}" \
  python3 /workspace/ActionModule/main/main.py
else
  echo ">>> [container] all nodes up (MODE=$MODE). Ctrl+C the launcher to stop."
  wait
fi
