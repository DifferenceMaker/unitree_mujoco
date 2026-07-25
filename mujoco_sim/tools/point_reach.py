"""point_reach.py — desk-line reach tester: torso-frame target points -> arm IK
-> rt/arm_pose_cmd (the controller's decoupled 14-dim external arm interface).

Give each hand a target point RELATIVE TO THE TORSO (x fwd, y left, z up, m).
A damped-least-squares IK (MuJoCo python, kinematics only — no sim stepped)
solves the 7-DoF arm chain; the resulting 14-dim pose is published as
  {"pose":[14 floats], "transition_s": s}   on rt/arm_pose_cmd (String_),
which the deploy controller slews to (External mode) — SAME channel in sim2sim
(iface lo) and on the real robot (robot DDS net). The balance policy reads the
arm command from its obs as always; this is how we test whether a policy TILTS
(hip-hinge) to reach deep/low points.

Interactive commands (stdin):
    l  x y z        set LEFT-hand target (torso frame, meters) + solve + publish
    r  x y z        set RIGHT-hand target
    b  x y z        both arms, y mirrored (left gets +y, right -y)
    t  s            set transition_s (default 3.0)
    table           preset: 1m-table reach (x .45, z -.05). NOTE: arm reach is
                    only ~0.41 m from the shoulder (z +0.42) — ANY table point
                    this far forward is beyond upright reach; the arms extend
                    toward it and only a HINGE-capable policy closes the gap.
    deep            preset: deep table reach (x .65)
    low             preset: low forward reach (x .45, z -.35 — coffee-table)
    d               default arm pose (FixStand arms)
    q               quit

Usage:
    /home/aspired-comp-2/miniconda3/envs/tv/bin/python point_reach.py            # sim2sim (lo)
    ... point_reach.py --iface enp6s0                                            # real robot
    ... point_reach.py --dry-run --left 0.55 0.25 -0.05                          # solve only
"""
import argparse
import json
import sys

import mujoco
import numpy as np

# 14-dim arm order (ArmPosePublisher.h — interleaved L/R):
#  0 L_sh_pitch  1 R_sh_pitch  2 L_sh_roll  3 R_sh_roll  4 L_sh_yaw  5 R_sh_yaw
#  6 L_elb_pitch 7 R_elb_pitch 8 L_elb_roll 9 R_elb_roll 10 L_wr_pitch
# 11 R_wr_pitch 12 L_wr_yaw   13 R_wr_yaw
# NB mujoco-xml names differ from the training URDF: elbow_joint == elbow_pitch,
# wrist_roll_joint == elbow_roll. Same kinematic chain order either way.
ARM_JOINTS = {
    "left":  ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
              "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
              "left_wrist_yaw_joint"],
    "right": ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
              "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
              "right_wrist_yaw_joint"],
}
EE_BODY = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
# chain-order -> interleaved-14 index, per side
IDX14 = {"left": [0, 2, 4, 6, 8, 10, 12], "right": [1, 3, 5, 7, 9, 11, 13]}
# FixStand default arms (deploy config qs): sh_pitch .35, sh_roll +-.25, sh_yaw -.1, elbow .65
DEFAULT_CHAIN = {"left": [0.35, 0.25, -0.1, 0.65, 0.0, 0.0, 0.0],
                 "right": [0.35, -0.25, -0.1, 0.65, 0.0, 0.0, 0.0]}

MODEL_CANDIDATES = [
    "/home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/unitree_robots/h1_2/h1_2_sym.xml",
    "/home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/unitree_robots/h1_2/scene_sym_soft08.xml",
]


class ArmIK:
    def __init__(self):
        for path in MODEL_CANDIDATES:
            try:
                self.model = mujoco.MjModel.from_xml_path(path)
                print(f"[ik] model: {path}")
                break
            except Exception as e:
                print(f"[ik] could not load {path}: {e}")
        else:
            sys.exit("[ik] no model loadable")
        self.data = mujoco.MjData(self.model)
        self.torso_id = self.model.body("torso_link").id
        self.ee_id = {s: self.model.body(EE_BODY[s]).id for s in ("left", "right")}
        self.jinfo = {}
        for side, names in ARM_JOINTS.items():
            qadr, dadr, lo, hi = [], [], [], []
            for n in names:
                j = self.model.joint(n)
                qadr.append(int(j.qposadr[0])); dadr.append(int(j.dofadr[0]))
                lo.append(float(j.range[0])); hi.append(float(j.range[1]))
            self.jinfo[side] = (np.array(qadr), np.array(dadr), np.array(lo), np.array(hi))
        # neutral stance qpos for the whole robot; arms overwritten per solve
        mujoco.mj_forward(self.model, self.data)

    def solve(self, side: str, p_torso: np.ndarray, seed=None, iters=200):
        """DLS position IK: target p (3,) in the TORSO frame -> 7 chain angles."""
        qadr, dadr, lo, hi = self.jinfo[side]
        q = np.array(seed if seed is not None else DEFAULT_CHAIN[side], dtype=float)
        lam2 = 0.01
        for _ in range(iters):
            self.data.qpos[qadr] = q
            mujoco.mj_forward(self.model, self.data)
            R_t = self.data.xmat[self.torso_id].reshape(3, 3)
            p_w = self.data.xpos[self.torso_id] + R_t @ p_torso
            err = p_w - self.data.xpos[self.ee_id[side]]
            if np.linalg.norm(err) < 2e-3:
                break
            jacp = np.zeros((3, self.model.nv)); jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_id[side])
            J = jacp[:, dadr]                                   # (3,7)
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), err)
            q = np.clip(q + 0.5 * dq, lo, hi)
        resid = float(np.linalg.norm(err))
        return q, resid


def main():
    ap = argparse.ArgumentParser(description="torso-frame point -> arm IK -> rt/arm_pose_cmd")
    ap.add_argument("--iface", default="lo", help="DDS iface (sim=lo, real=enp6s0)")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--transition", type=float, default=3.0)
    ap.add_argument("--left", nargs=3, type=float, help="left target x y z (torso frame)")
    ap.add_argument("--right", nargs=3, type=float, help="right target x y z")
    ap.add_argument("--dry-run", action="store_true", help="solve + print only, no DDS")
    args = ap.parse_args()

    ik = ArmIK()
    pose14 = np.zeros(14)
    for s in ("left", "right"):                       # start at the default pose
        pose14[IDX14[s]] = DEFAULT_CHAIN[s]

    pub = None
    if not args.dry_run:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        ChannelFactoryInitialize(args.domain, args.iface)
        pub = ChannelPublisher("rt/arm_pose_cmd", String_)
        pub.Init()
        print(f"[pub] rt/arm_pose_cmd on {args.iface} (domain {args.domain})")

    trans = [args.transition]

    def publish():
        js = json.dumps({"pose": [round(float(v), 4) for v in pose14],
                         "transition_s": trans[0]})
        if pub is not None:
            msg_cls = type(pub).__module__  # noqa: F841
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            pub.Write(String_(data=js))
            print(f"[pub] sent (transition {trans[0]}s)")
        else:
            print(f"[dry] {js}")

    def set_target(side, xyz):
        q, resid = ik.solve(side, np.array(xyz, dtype=float),
                            seed=pose14[IDX14[side]])
        pose14[IDX14[side]] = q
        flag = "" if resid < 0.02 else f"  <-- beyond upright reach (short {resid*100:.0f} cm): arms extend toward it; a hinge policy closes the gap"
        print(f"[ik] {side} -> ({xyz[0]:+.2f},{xyz[1]:+.2f},{xyz[2]:+.2f})  resid {resid*1000:.0f} mm{flag}")
        print(f"     chain: {np.array2string(q, precision=2, suppress_small=True)}")

    # one-shot CLI targets
    if args.left:  set_target("left", args.left)
    if args.right: set_target("right", args.right)
    if args.left or args.right:
        publish()
    if args.dry_run and (args.left or args.right):
        return

    print("commands: l|r|b x y z   t s   table | deep | low   d(efault)   q(uit)")
    for line in sys.stdin:
        tok = line.split()
        if not tok:
            continue
        c = tok[0].lower()
        try:
            if c == "q":
                break
            elif c == "t" and len(tok) == 2:
                trans[0] = float(tok[1]); print(f"[cfg] transition_s={trans[0]}")
                continue
            elif c in ("l", "r", "b") and len(tok) == 4:
                x, y, z = map(float, tok[1:4])
                if c in ("l", "b"): set_target("left", (x, abs(y) if c == "b" else y, z))
                if c in ("r", "b"): set_target("right", (x, -abs(y) if c == "b" else y, z))
            elif c == "table":
                set_target("left", (0.45, 0.22, -0.05)); set_target("right", (0.45, -0.22, -0.05))
            elif c == "deep":
                set_target("left", (0.65, 0.22, -0.05)); set_target("right", (0.65, -0.22, -0.05))
            elif c == "low":
                set_target("left", (0.45, 0.22, -0.35)); set_target("right", (0.45, -0.22, -0.35))
            elif c == "d":
                for s in ("left", "right"): pose14[IDX14[s]] = DEFAULT_CHAIN[s]
                print("[ik] default arm pose")
            else:
                print("?? commands: l|r|b x y z / t s / table / deep / low / d / q")
                continue
            publish()
        except Exception as e:
            print(f"[err] {e}")


if __name__ == "__main__":
    main()
