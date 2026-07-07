#!/usr/bin/env python3
"""Standalone MoveIt IK probe — ZERO ActionModule code touched.

Run from a third terminal while the teleop profile is up:

    docker exec <container> bash -lc \
      'source /opt/ros/humble/setup.bash && \
       ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 \
       python3 /unitree_mujoco/mujoco_sim/tools/ik_probe.py'

Answers two questions the teleop failures can't separate:
  1. What does MoveIt's world actually look like right now?  Echoes one
     /joint_states message (names+positions the current-state monitor eats).
  2. Is the KNOWN-GOOD boot pose solvable right now?  Calls /compute_ik
     directly with the exact request the boot go_to_start solved at engage
     ((0.300, 0.300, 0.050) rpy (103.5, -41.5, -1.0) deg, left_arm,
     torso_link frame) — seedless, like the boot call.

If (2) fails while the same call succeeded at engage, the planning scene's
live state is what flipped reachability, and (1) shows exactly which joint
values did it. If (2) succeeds, the difference lives in how the teleop call
is constructed, not in MoveIt.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

BOOT_POSE = (0.300, 0.300, 0.050)          # the pose boot go_to_start solved
BOOT_RPY_DEG = (103.5, -41.5, -1.0)        # left arm variant


def rpy_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main():
    rclpy.init()
    node = Node("ik_probe")

    # 1) one /joint_states snapshot
    got = {}

    def on_js(msg):
        got["msg"] = msg

    node.create_subscription(JointState, "/joint_states", on_js, 10)
    for _ in range(200):                      # up to ~4 s
        rclpy.spin_once(node, timeout_sec=0.02)
        if "msg" in got:
            break
    if "msg" in got:
        js = got["msg"]
        print(f"/joint_states: {len(js.name)} joints")
        for n, p in zip(js.name, js.position):
            print(f"  {n:35s} {p:+.3f}")
    else:
        print("/joint_states: NO MESSAGE in 4 s — bridge not publishing!")

    # 2) seedless /compute_ik with the known-good boot pose
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        print("/compute_ik: SERVICE NOT AVAILABLE")
        rclpy.shutdown()
        sys.exit(1)

    req = GetPositionIK.Request()
    ik = PositionIKRequest()
    ik.group_name = "left_arm"
    ik.ik_link_name = "left_wrist_yaw_link"
    ik.avoid_collisions = False
    ik.timeout.sec = 1
    ps = PoseStamped()
    ps.header.frame_id = "torso_link"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = BOOT_POSE
    qx, qy, qz, qw = rpy_to_quat(*(math.radians(d) for d in BOOT_RPY_DEG))
    ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    ik.pose_stamped = ps
    req.ik_request = ik

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    res = future.result()
    if res is None:
        print("/compute_ik: NO RESPONSE in 5 s")
    elif res.error_code.val == 1:
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        arm = {n: v for n, v in sol.items() if n.startswith("left_")
               and ("shoulder" in n or "elbow" in n or "wrist" in n)}
        print(f"/compute_ik: SUCCESS — boot pose IS solvable right now")
        for n, v in arm.items():
            print(f"  {n:35s} {v:+.3f}")
    else:
        print(f"/compute_ik: FAILED error_code={res.error_code.val} "
              f"(-31=NO_IK_SOLUTION) — live planning-scene state has made "
              f"the boot pose unreachable")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
