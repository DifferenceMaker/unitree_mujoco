#!/usr/bin/env bash
# _nodes_in_container.sh — bring up the Architecture B ROS2 nodes inside the
# ros2-humble-dev container, against rt/lowstate/rt/lowcmd already on `lo`
# (real MuJoCo runs on the host; --network host shares the bus).
#
# Invoked by run_arch_b_sim.sh via docker run. Env in:
#   MODE = a (static arms, no ActionModule)  |  b (ActionModule IK arms)
#   GAINS = harness | deploy | flat50  (consumer kp/kd regime)
# Blocks until the container is stopped (SIGTERM) — then kills the children.
source /opt/ros/humble/setup.bash   # before any set -u

export ASPIRED_ROOT=/workspace
export CYCLONEDDS_URI="file:///unitree_mujoco/arch_b_sim/tools/cyclonedds_lo.xml"
export PYTHONPATH="/workspace/.global:/workspace/MovementModule/main:${PYTHONPATH:-}"
SIM=/unitree_mujoco/arch_b_sim
MODE="${MODE:-a}"
GAINS="${GAINS:-harness}"

echo ">>> [container] installing unitree_sdk2py for the sim nodes"
pip install -e /unitree_sdk2_python -q 2>/dev/null || pip install unitree_sdk2py -q 2>/dev/null || true
python3 -c "import unitree_sdk2py" 2>/dev/null || { echo "FATAL: unitree_sdk2py unavailable"; exit 2; }

PIDS=()
cleanup() { echo ">>> [container] stopping nodes"; kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

echo ">>> [container] starting sim_state_bridge (rt/lowstate → /BridgeModule/joints_imu + conduct)"
python3 "$SIM/sim_state_bridge.py" & PIDS+=($!)
sleep 1

if [ "$MODE" = "b" ]; then
  echo ">>> [container] MODE B — launching colleague's ActionModule (real IK arms)"
  ( source /workspace/.venv/ActionModule/bin/activate 2>/dev/null
    PYTHONPATH="/workspace/.global:/workspace/ActionModule/main:${PYTHONPATH}" \
    python3 /workspace/ActionModule/main/main.py ) & PIDS+=($!)
  CONSUMER_ARMS=""    # consumer reads joint_set arm slots from ActionModule
else
  echo ">>> [container] MODE A — static default arm pose on /BridgeModule/joint_set"
  python3 "$SIM/sim_armpose_pub.py" & PIDS+=($!)
  CONSUMER_ARMS="--static-arms"
fi

echo ">>> [container] starting MovementModule (balance policy → /MovementModule/policy_action)"
( source /workspace/.venv/MovementModule/bin/activate
  python3 /workspace/MovementModule/main/main.py ) & PIDS+=($!)
sleep 2

echo ">>> [container] starting sim_action_consumer (transform+remap → rt/lowcmd, gains=$GAINS)"
python3 "$SIM/sim_action_consumer.py" --gains "$GAINS" $CONSUMER_ARMS & PIDS+=($!)

echo ">>> [container] all nodes up (MODE=$MODE). Ctrl+C the launcher to stop."
wait
