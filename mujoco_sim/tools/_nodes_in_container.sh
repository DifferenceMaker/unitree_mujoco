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
#        | b (ActionModule IK arms -> /BridgeModule/joint_set_arms)
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

echo ">>> [container] starting REAL BridgeModule (BRIDGE_SIM=1, iface lo)"
( PYTHONPATH="/workspace/.global:${PYTHONPATH:-}" \
  BRIDGE_SIM=1 python3 /workspace/BridgeModule/main/main.py lo ) & PIDS+=($!)
sleep 2

if [ "$MODE" = "b" ]; then
  echo ">>> [container] MODE B — arm_ik_commander (red/blue dot Cartesian targets"
  echo "      -> ActionModule ikpy IK -> /BridgeModule/joint_set_arms)."
  echo "      Command from the host:  echo \"l 0.35 0.25 0.10\" >> mujoco_sim/logs/.arm_targets"
  echo "      (also: 'r x y z' | 'default';  ARM_IK_DEMO=1 auto-cycles targets)"
  ( source /workspace/.venv/ActionModule/bin/activate 2>/dev/null
    PYTHONPATH="/workspace/.global:/workspace/ActionModule:${PYTHONPATH:-}" \
    ARM_TARGETS_FILE=/unitree_mujoco/mujoco_sim/logs/.arm_targets \
    python3 /workspace/ActionModule/Utils/arm_ik_commander.py ) & PIDS+=($!)
else
  echo ">>> [container] MODE A — no arm source; MovementModule uses measured-arms fallback"
fi

echo ">>> [container] starting MovementModule (FixStand->hold->policy -> /BridgeModule/joint_set_legs)"
( source /workspace/.venv/MovementModule/bin/activate
  PYTHONPATH="/workspace/.global:${PYTHONPATH:-}" \
  python3 /workspace/MovementModule/main/main.py ) & PIDS+=($!)

echo ">>> [container] all nodes up (MODE=$MODE). Ctrl+C the launcher to stop."
wait
