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

    # 2) /compute_ik request matrix — replicates the EXACT requests
    #    ActionModule's failing teleop solves sent (poses+rpy+seed from the
    #    2026-07-07 14:24 run's [ik/moveit] FAIL prints), alongside the
    #    known-good boot request. Whichever row flips SUCCESS->FAIL names
    #    the field that breaks it.
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        print("/compute_ik: SERVICE NOT AVAILABLE")
        rclpy.shutdown()
        sys.exit(1)

    LEFT_ARM_JOINTS = [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint",
        "left_wrist_roll_joint", "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ]
    TELEOP_SEED = [-0.24, 0.21, 0.04, 0.54, 1.51, 0.10, 1.01]

    def compute_ik(label, pose, rpy_deg, seed=None):
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = "left_arm"
        ik.ik_link_name = "left_wrist_yaw_link"
        ik.avoid_collisions = False
        ik.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = "torso_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose
        qx, qy, qz, qw = rpy_to_quat(*(math.radians(d) for d in rpy_deg))
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        ik.pose_stamped = ps
        if seed is not None:
            js = JointState()
            js.name = list(LEFT_ARM_JOINTS)
            js.position = [float(v) for v in seed]
            ik.robot_state.joint_state = js
        req.ik_request = ik
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            print(f"{label:28s} NO RESPONSE in 5 s")
        elif res.error_code.val == 1:
            sol = dict(zip(res.solution.joint_state.name,
                           res.solution.joint_state.position))
            arm = [f"{sol[n]:+.2f}" for n in LEFT_ARM_JOINTS if n in sol]
            print(f"{label:28s} SUCCESS  joints=[{','.join(arm)}]")
        else:
            print(f"{label:28s} FAILED error_code={res.error_code.val} "
                  f"(-31=NO_IK_SOLUTION, -21=FRAME_TRANSFORM_FAILURE)")

    print()
    print("workspace-margin matrix (all with the boot orientation "
          f"{BOOT_RPY_DEG} deg, seed = teleop seed):")
    # Round-3 verdict: the LCM/LCC targets were genuinely unreachable and
    # the boot pose sits ON the workspace boundary (z 0.050 solves, 0.049
    # does not). This matrix maps the PLAIN move_r neighborhood — a 5 cm
    # step in each direction with the orientation held — i.e. exactly what
    # each teleop key requests with lCM_TEST=False.
    bx, by, bz = BOOT_POSE
    compute_ik("boot (baseline)", BOOT_POSE, BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("w: x+0.05", (bx + 0.05, by, bz), BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("s: x-0.05", (bx - 0.05, by, bz), BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("a: y+0.05", (bx, by + 0.05, bz), BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("d: y-0.05", (bx, by - 0.05, bz), BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("q: z+0.05", (bx, by, bz + 0.05), BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("e: z-0.05", (bx, by, bz - 0.05), BOOT_RPY_DEG, TELEOP_SEED)
    # context rows: the old table-height start pose (colleague's teleop was
    # tuned here) and mid-height candidates with more margin
    compute_ik("old table pose", (0.250, 0.490, 0.550), BOOT_RPY_DEG)
    compute_ik("candidate (0.30,0.25,0.15)", (0.300, 0.250, 0.150),
               BOOT_RPY_DEG, TELEOP_SEED)
    compute_ik("candidate (0.35,0.25,0.20)", (0.350, 0.250, 0.200),
               BOOT_RPY_DEG, TELEOP_SEED)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
