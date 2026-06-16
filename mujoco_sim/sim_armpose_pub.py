#!/usr/bin/env python3
"""
sim_armpose_pub.py — static /BridgeModule/joint_set publisher (Mode A only).

For the balance bring-up mode where the colleague's ActionModule is NOT running,
MovementModule still needs /BridgeModule/joint_set to be non-None (it early-
returns otherwise — MovementModule/main/main.py:80-83) so it can build the
`arm_pose_command` obs term. This publishes a constant full-body 27-vec (SDK
order) whose arm slots hold the p7_1b default arm pose.

In Mode B (full integration) DO NOT run this — ActionModule owns
/BridgeModule/joint_set with the real IK-resolved arm pose.

Run (container, ROS2 sourced):  python3 sim_armpose_pub.py
Requires PYTHONPATH: <Aspired>/.global, <Aspired>/MovementModule/main
"""

import argparse
import os
import sys
import time

import numpy as np

ASPIRED_ROOT = os.environ.get("ASPIRED_ROOT", "/workspace")
for _p in (os.path.join(ASPIRED_ROOT, ".global"),
           os.path.join(ASPIRED_ROOT, "MovementModule", "main")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Ros2_connector import Ros2  # noqa: E402
from balance_contract import OBS_SDK_ORDER, DEFAULT_JOINT_POS  # noqa: E402

NUM_MOTOR = 27
JOINT_SET_TOPIC = "/BridgeModule/joint_set"  # fully-qualified: owned by BridgeModule


def sdk_default_pose() -> np.ndarray:
    q = np.zeros(NUM_MOTOR, dtype=np.float32)
    for k, sdk_idx in enumerate(OBS_SDK_ORDER):
        q[int(sdk_idx)] = DEFAULT_JOINT_POS[k]
    return q


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=50.0)
    args = ap.parse_args()

    pose = sdk_default_pose().tolist()

    import rclpy
    rclpy.init()
    ros = Ros2("ArchBSimArmPose")
    print(f"[sim_armpose_pub] publishing static default joint_set on "
          f"{JOINT_SET_TOPIC} @ {args.rate} Hz", flush=True)

    period = 1.0 / args.rate
    next_t = time.monotonic()
    try:
        while rclpy.ok():
            ros.publish_data(JOINT_SET_TOPIC, pose, "f32arr")
            next_t += period
            sleep = next_t - time.monotonic()
            time.sleep(sleep if sleep > 0 else 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        ros.destroy_node()
        rclpy.shutdown()
        print("[sim_armpose_pub] stopped", flush=True)


if __name__ == "__main__":
    main()
