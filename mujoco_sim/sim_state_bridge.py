#!/usr/bin/env python3
"""
sim_state_bridge.py — sim-only DDS→ROS2 state bridge for the Architecture B loop.

Stands in for BridgeModule's ROS2-facing side WITHOUT its camera/hand/
MotionSwitcher dependencies (see mujoco_sim/BRIDGEMODULE_SIM_NOTES.md). It:

  • subscribes  rt/lowstate  (unitree_hg LowState_) from unitree_mujoco on `lo`,
  • republishes the SAME 92-float `joints_imu` record BridgeModule produces, on
    ROS2 topic  /BridgeModule/joints_imu  (Float32MultiArray via Ros2_connector),
  • announces   /BridgeModule/conduct = True (latched), so the colleague's
    ActionModule clears its BridgeModule handshake and starts publishing the
    IK-resolved arm pose on /BridgeModule/joint_set.

The 92-float record layout is copied byte-for-byte from
  Aspired_Robot_Project/BridgeModule/main/getters/lococlient/get_joints_imu.py
    [ q(27), dq(27), tau(27), quat(4 wxyz), gyro(3), acc(3), ts(1) ]   = 92
so MovementModule's balance_contract slices (Q/DQ/QUAT/GYRO_SLICE) line up.

DO NOT edit BridgeModule to use this — it lives here in the unitree_mujoco
`mujoco_sim` branch as sim-only scaffolding.

Run (inside the ros2-humble-dev container, ROS2 sourced):
    python3 sim_state_bridge.py --iface lo
Requires on PYTHONPATH:  <Aspired>/.global  (Ros2_connector, Debug_tool)
Requires installed:      unitree_sdk2py, numpy, rclpy
"""

import argparse
import os
import sys
import threading
import time

import numpy as np

# ── make the Aspired project's shared ROS2 connector importable ──────────────
ASPIRED_ROOT = os.environ.get(
    "ASPIRED_ROOT", "/workspace"
)  # the ros2-humble-dev container mounts Aspired_Robot_Project at /workspace
for _p in (os.path.join(ASPIRED_ROOT, ".global"),
           os.path.join(ASPIRED_ROOT, "MovementModule", "main")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Ros2_connector import Ros2  # noqa: E402

NUM_MOTOR = 27
# 27q + 27dq + 27tau + 4quat + 3gyro + 3acc + 1ts
RECORD_LEN = NUM_MOTOR * 3 + 4 + 3 + 3 + 1  # = 92


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="lo", help="DDS network interface (lo=sim)")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain id")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="ROS2 republish rate (Hz); MovementModule reads at 50 Hz")
    args = ap.parse_args()

    # ── Unitree low-level DDS subscriber (rt/lowstate) ───────────────────────
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(args.domain, args.iface)

    latest = {"msg": None}
    lock = threading.Lock()

    def on_lowstate(msg: "LowState_"):
        with lock:
            latest["msg"] = msg

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)
    print(f"[sim_state_bridge] subscribed rt/lowstate on {args.iface} "
          f"(domain {args.domain})", flush=True)

    # ── ROS2 side (impersonate BridgeModule) ─────────────────────────────────
    import rclpy
    rclpy.init()
    ros = Ros2("BridgeModule")
    print("[sim_state_bridge] ROS2 node up as BridgeModule; waiting for "
          "lowstate before announcing /BridgeModule/conduct", flush=True)

    announced = False
    period = 1.0 / args.rate
    next_t = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.0)

            with lock:
                msg = latest["msg"]

            if msg is not None:
                imu = msg.imu_state
                rec = np.array(
                    [*[msg.motor_state[i].q       for i in range(NUM_MOTOR)],
                     *[msg.motor_state[i].dq      for i in range(NUM_MOTOR)],
                     *[msg.motor_state[i].tau_est for i in range(NUM_MOTOR)],
                     *imu.quaternion,      # 4, wxyz
                     *imu.gyroscope,       # 3
                     *imu.accelerometer,   # 3
                     # ts slot: CLOCK_MONOTONIC ms (mod 100 s so it fits float32
                     # with ~ms precision). time.monotonic() is comparable across
                     # processes on one host → downstream can measure obs latency.
                     (time.monotonic() % 100.0) * 1000.0],
                    dtype=np.float32,
                )
                assert rec.shape[0] == RECORD_LEN, rec.shape
                ros.publish_data("joints_imu", rec.tolist(), "f32arr")

                # Announce the handshake only once real data is flowing, so
                # ActionModule doesn't start before state is available.
                if not announced:
                    ros.startup_conductivity_announce(latched=True)
                    announced = True
                    print("[sim_state_bridge] lowstate flowing → announced "
                          "/BridgeModule/conduct=True", flush=True)

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        ros.destroy_node()
        rclpy.shutdown()
        print("[sim_state_bridge] stopped", flush=True)


if __name__ == "__main__":
    main()
