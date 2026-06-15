#!/usr/bin/env python3
"""
fake_lowstate_pub.py — a stand-in for MuJoCo that publishes rt/lowstate
(unitree_hg LowState_) at a fixed pose. NO physics — it does not respond to
rt/lowcmd. Purpose: verify the ROS2/DDS *plumbing* of the Architecture B loop
(sim_state_bridge → joints_imu → MovementModule → policy_action → consumer →
rt/lowcmd) on a box where the GUI MuJoCo can't run, and as a CI smoke fixture.

Publishes the p7_1b default pose (SDK order) upright (quat wxyz = 1,0,0,0,
mode_machine=6), so a correctly-wired chain yields joint_pos_rel≈0 and
projected_gravity≈[0,0,-1] in MovementModule.

Run (needs unitree_sdk2py + numpy; e.g. conda tv, or the container):
    export CYCLONEDDS_URI=file://$PWD/cyclonedds_lo.xml   # lo-only discovery
    PYTHONPATH=<Aspired>/MovementModule/main python3 fake_lowstate_pub.py
"""
import argparse
import os
import sys
import time

import numpy as np

ASPIRED_ROOT = os.environ.get("ASPIRED_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ASPIRED_ROOT, "MovementModule", "main"))

try:
    from balance_contract import OBS_SDK_ORDER, DEFAULT_JOINT_POS
    _have_contract = True
except Exception:
    _have_contract = False

NUM_MOTOR = 27


def sdk_default_pose():
    q = np.zeros(NUM_MOTOR, dtype=np.float32)
    if _have_contract:
        for k, sdk_idx in enumerate(OBS_SDK_ORDER):
            q[int(sdk_idx)] = DEFAULT_JOINT_POS[k]
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--zeros", action="store_true",
                    help="publish all-zero pose instead of the p7_1b default")
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(args.domain, args.iface)
    ls = unitree_hg_msg_dds__LowState_()
    ls.mode_machine = 6
    q = np.zeros(NUM_MOTOR, np.float32) if args.zeros else sdk_default_pose()
    for i in range(NUM_MOTOR):
        ls.motor_state[i].q = float(q[i])
        ls.motor_state[i].dq = 0.0
        ls.motor_state[i].tau_est = 0.0
    ls.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]   # wxyz upright
    ls.imu_state.gyroscope = [0.0, 0.0, 0.0]
    ls.imu_state.accelerometer = [0.0, 0.0, 9.81]

    pub = ChannelPublisher("rt/lowstate", LowState_)
    pub.Init()
    print(f"[fake_lowstate] publishing rt/lowstate on {args.iface} @ {args.rate} Hz "
          f"(pose={'zeros' if args.zeros else 'p7_1b default'}, contract={_have_contract})",
          flush=True)

    period = 1.0 / args.rate
    next_t = time.monotonic()
    n = 0
    try:
        while True:
            pub.Write(ls)
            n += 1
            if n % int(args.rate) == 0:
                print(f"[fake_lowstate] published {n} msgs", flush=True)
            next_t += period
            s = next_t - time.monotonic()
            time.sleep(s if s > 0 else 0.0)
    except KeyboardInterrupt:
        print("[fake_lowstate] stopped", flush=True)


if __name__ == "__main__":
    main()
