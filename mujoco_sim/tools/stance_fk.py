#!/usr/bin/env python3
"""stance_fk.py <mjcf> <stance_q.json> <out.json> -- the exam #2 stance drawing (operator 2026-10-30: "record the joints for 20 seconds
and display its 20 second default stance").

Input: the ladder record's stance_q ({"cols": [t, z, qw, qx, qy, qz, q0..], "rows": [...]}, 5 Hz over the 20 s stand; q0.. = the
MJCF joint order after the free joint). Output (JSON, in the robot's heading frame: x forward, y left, z up, origin = the feet
midpoint on the floor):
  bodies        the robot body names (MJCF order)
  mean          [x, y, z] per body for the MEAN pose (mean joint angles, mean base height, mean roll / pitch, yaw removed)
  ghosts        the same for every 10th row (2 s apart), each on its own feet midpoint -- the 20 s spread
  extra         head (shoulder midpoint + 0.22 m along the torso axis) and hand points (wrist + 0.12 m along the wrist x axis) of
                the mean pose -- drawing aids, not model bodies
  com           the whole-robot centre of mass of the mean pose; com_ghosts per ghost
  soles         4 floor corners per foot (the ankle_roll_link sole 26 x 8.6 cm) of the mean pose
  joint_std_deg the joint angle std over the stand, per joint (MJCF joint names in joints)
Needs a python with mujoco (the isaacsim env); exam_record.py finds one."""
import json
import math
import sys

import mujoco
import numpy as np

SOLE = (-0.086, 0.174, 0.043)   # ankle_roll_link sole: x from heel to toe, half width (m), as fleetdeck's footprint


def rpy(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    return roll, pitch


def quat_rp(roll, pitch):
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return [cr * cp, sr * cp, cr * sp, -sr * sp]   # yaw = 0


def main():
    mjcf, src, dst = sys.argv[1:4]
    sq = json.load(open(src))
    rows = np.array(sq["rows"], float)
    m = mujoco.MjModel.from_xml_path(mjcf); d = mujoco.MjData(m)
    nj = m.nq - 7
    Q = rows[:, 6:6 + nj]
    bodies = [m.body(i).name for i in range(1, m.nbody)]
    bid = {n: i for i, n in enumerate([m.body(i).name for i in range(m.nbody)])}
    feet = (bid["left_ankle_roll_link"], bid["right_ankle_roll_link"])
    root = bid["pelvis"]

    def pose(z, roll, pitch, qj):
        d.qpos[:] = 0.0
        d.qpos[2] = z; d.qpos[3:7] = quat_rp(roll, pitch); d.qpos[7:7 + nj] = qj
        mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        mid = 0.5 * (d.xpos[feet[0]][:2] + d.xpos[feet[1]][:2])
        off = np.array([mid[0], mid[1], 0.0])
        pts = (d.xpos[1:] - off).round(4).tolist()
        com = (d.subtree_com[root] - off).round(4).tolist()
        return pts, com, off

    rp = np.array([rpy(*r[2:6]) for r in rows])
    pts, com, off = pose(float(rows[:, 1].mean()), float(rp[:, 0].mean()), float(rp[:, 1].mean()), Q.mean(axis=0))
    # drawing aids on the mean pose
    sh = 0.5 * (d.xpos[bid["left_shoulder_pitch_link"]] + d.xpos[bid["right_shoulder_pitch_link"]])
    head = sh + 0.22 * d.xmat[bid["torso_link"]].reshape(3, 3)[:, 2]
    hands = [d.xpos[bid[w]] + 0.12 * d.xmat[bid[w]].reshape(3, 3)[:, 0] for w in ("left_wrist_yaw_link", "right_wrist_yaw_link")]
    soles = []
    for f in feet:
        R, p = d.xmat[f].reshape(3, 3), d.xpos[f]
        soles.append([((p + R @ np.array([x, y, 0.0])) - off).round(4)[:2].tolist() for x, y in ((SOLE[0], -SOLE[2]), (SOLE[1], -SOLE[2]), (SOLE[1], SOLE[2]), (SOLE[0], SOLE[2]))])
    extra = {"head": (head - off).round(4).tolist(), "hand_L": (hands[0] - off).round(4).tolist(), "hand_R": (hands[1] - off).round(4).tolist()}
    ghosts, com_g = [], []
    for k in range(0, len(rows), 10):
        g, c, _ = pose(float(rows[k, 1]), float(rp[k, 0]), float(rp[k, 1]), Q[k])
        ghosts.append(g); com_g.append(c)
    out = {"frame": "base heading: x forward, y left, z up; origin = feet midpoint on the floor", "n_rows": int(len(rows)),
           "bodies": bodies, "mean": pts, "ghosts": ghosts, "extra": extra, "com": com, "com_ghosts": com_g, "soles": soles,
           "joints": [m.joint(i).name for i in range(1, m.njnt)], "joint_std_deg": np.degrees(Q.std(axis=0)).round(3).tolist(),
           "base": {"z": round(float(rows[:, 1].mean()), 4), "roll_deg": round(math.degrees(rp[:, 0].mean()), 3), "pitch_deg": round(math.degrees(rp[:, 1].mean()), 3)}}
    json.dump(out, open(dst, "w"), separators=(",", ":"))


if __name__ == "__main__":
    main()
