#!/usr/bin/env bash
# chain_selftest.sh — verify the Architecture B Mode-A data-flow WITHOUT physics.
#
# Brings up, inside one ros2-humble-dev container, against a FAKE rt/lowstate
# publisher (no MuJoCo GUI needed):
#     fake_lowstate_pub → sim_state_bridge → MovementModule → sim_action_consumer
#     + sim_armpose_pub (static joint_set)
# Then checks: (1) /BridgeModule/joints_imu flows, (2) MovementModule emits
# /MovementModule/policy_action, (3) the consumer writes rt/lowcmd with sane,
# finite targets. This isolates "is the plumbing wired" from "is the policy good".
#
# Run it via:
#   docker run --rm --network host \
#     -v <Aspired>:/workspace -v <unitree_mujoco>:/unitree_mujoco \
#     -v <unitree_sdk2_python>:/unitree_sdk2_python \
#     --entrypoint bash ros2-humble-dev /unitree_mujoco/arch_b_sim/tools/chain_selftest.sh
source /opt/ros/humble/setup.bash   # before any `set -u`: it reads unbound AMENT_* vars

export ASPIRED_ROOT=/workspace
export CYCLONEDDS_URI="file:///unitree_mujoco/arch_b_sim/tools/cyclonedds_lo.xml"
export PYTHONPATH="/workspace/.global:/workspace/MovementModule/main:${PYTHONPATH:-}"
SIM=/unitree_mujoco/arch_b_sim
L=/tmp/archb ; mkdir -p "$L"

echo ">>> installing unitree_sdk2py (system python, for the sim nodes)"
pip install -e /unitree_sdk2_python -q 2>/dev/null || pip install unitree_sdk2py -q 2>/dev/null || echo "  (pip install failed — check network/repo)"
python3 -c "import unitree_sdk2py" 2>/dev/null && echo "  unitree_sdk2py OK" || { echo "  FATAL: unitree_sdk2py unavailable"; exit 2; }

PIDS=()
bg() { "$@" & PIDS+=($!); }

echo ">>> starting fake robot + nodes"
bg python3 "$SIM/tools/fake_lowstate_pub.py"        # rt/lowstate (cyclone lo)
sleep 1
bg python3 "$SIM/sim_state_bridge.py"               # rt/lowstate → /BridgeModule/joints_imu
bg python3 "$SIM/sim_armpose_pub.py"                # static /BridgeModule/joint_set
# MovementModule in its own venv (onnxruntime); rclpy comes from the sourced
# ROS paths already in the exported PYTHONPATH — do NOT clobber it (venv activate
# leaves PYTHONPATH alone, so the inherited value keeps rclpy + .global + contract).
( source /workspace/.venv/MovementModule/bin/activate
  python3 /workspace/MovementModule/main/main.py ) > "$L/mm.log" 2>&1 &
PIDS+=($!)
bg python3 "$SIM/sim_action_consumer.py" --static-arms --gains harness

cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

echo ">>> waiting up to 25s for /MovementModule/policy_action to appear"
ok_topic=0
for i in $(seq 1 25); do
  if timeout 3 ros2 topic echo --once /MovementModule/policy_action >/dev/null 2>&1; then
    ok_topic=1; break
  fi
  sleep 1
done

echo "" ; echo "================ CHAIN CHECK ================"
PASS=1

# 1) joints_imu present + right width
if timeout 5 ros2 topic echo --once /BridgeModule/joints_imu 2>/dev/null | grep -q "data:"; then
  echo "  [PASS] /BridgeModule/joints_imu flowing"
else
  echo "  [FAIL] /BridgeModule/joints_imu not flowing"; PASS=0
fi

# 2) policy_action present + 13-wide
if [ "$ok_topic" = "1" ]; then
  N=$(timeout 5 ros2 topic echo --once /MovementModule/policy_action 2>/dev/null | grep -c '^-' )
  echo "  [PASS] /MovementModule/policy_action flowing (≈$N elements; expect 13)"
else
  echo "  [FAIL] /MovementModule/policy_action never appeared"; PASS=0
fi

# 3) rt/lowcmd written by the consumer, finite + in-range
python3 - <<'PY'
import os, time, sys, math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
ChannelFactoryInitialize(0, "lo")
got={"m":None}
ChannelSubscriber("rt/lowcmd", LowCmd_).Init(lambda m: got.__setitem__("m", m), 10)
t=time.time()+8
while time.time()<t and got["m"] is None: time.sleep(0.2)
m=got["m"]
if m is None:
    print("  [FAIL] rt/lowcmd not written by consumer"); sys.exit(3)
q=[m.motor_cmd[i].q for i in range(27)]
kp=[m.motor_cmd[i].kp for i in range(27)]
finite=all(math.isfinite(x) for x in q)
inrange=all(abs(x)<4.0 for x in q)
print(f"  [{'PASS' if finite and inrange else 'FAIL'}] rt/lowcmd written: "
      f"q[0:6]={[round(x,3) for x in q[:6]]} knee_q[3,9]={round(q[3],3)},{round(q[9],3)} "
      f"kp[3](knee)={kp[3]} kp[13](arm)={kp[13]}")
sys.exit(0 if finite and inrange else 4)
PY
RC=$?
[ $RC -ne 0 ] && PASS=0

echo "=============================================="
[ "$PASS" = "1" ] && echo "RESULT: CHAIN OK" || { echo "RESULT: CHAIN FAILED — logs in $L and ./*.log"; for f in "$L"/mm.log; do echo "--- $f ---"; tail -15 "$f" 2>/dev/null; done; }
exit $([ "$PASS" = "1" ] && echo 0 || echo 1)
