#!/usr/bin/env python3
"""homing_work_plan.py -- the arm-reach plan for the "work at the table" video (2026-11-04): torso-frame ball targets -> arm IK -> a
joint-space plan for tools/homing_work_driver.py.

The robot stands at its table spot under the balance policy (the homing trial's stop: torso front 0.25 m from the desk edge, the
desk top at the torso origin's height). Balls float above the desk edge in front of it, in the TORSO frame (x fwd, y left, z up, m):
the left hand touches L1, then the right hand R1, then both hands the pair P_L + P_R at once (hands kept apart). Between the reaches
the arms go back to a READY pose (hands raised in front), at the end to the balance policy's trained neutral (DEFAULT).
IK = tools/point_reach.py's damped-least-squares solver (ArmIK) on the hand790 body, solved for the PALM point (--hand-offset in the
wrist_yaw_link frame, the Inspire palm sensor) instead of the wrist origin, with a null-space pull toward the READY posture so the
wrists stay natural. Kinematics only (no sim stepped). Output: plan.json -- keyframes in the BridgeModule order (/BridgeModule/
joint_set_arms: left [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw], then right), the sequence, the balls and the IK residuals.

  ~/miniconda3/envs/tv/bin/python tools/homing_work_plan.py --out logs/homing_video/work_plan.json
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import point_reach  # noqa: E402

ROBOT_XML = os.path.abspath(os.path.join(HERE, "..", "..", "unitree_robots", "h1_2", "h1_2_comx06_armature_hand790.xml"))
# exam Test 0's "default" (MovementModule/Utils/homing_commander.py HANDOVER_POSES): the balance policy's trained neutral
DEFAULT14 = [0.4, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]


class PalmIK(point_reach.ArmIK):
    """point_reach.ArmIK on the hand790 body, for a point fixed in the wrist_yaw_link frame (the palm), + a null-space posture pull"""

    def solve_point(self, side, p_torso, offset, q_nom, R_torso=None, seed=None, iters=600, k_null=0.05, w_rot=0.25):
        """the palm point onto p_torso; with R_torso (3x3, the hand frame's axes in the torso frame) also the hand orientation,
        weighted w_rot (m per rad) -- position first, the orientation as far as the 7 joints and their limits allow"""
        qadr, dadr, lo, hi = self.jinfo[side]
        q = np.array(seed if seed is not None else q_nom, dtype=float)
        lam2 = 0.004
        for _ in range(iters):
            self.data.qpos[qadr] = q
            mujoco.mj_forward(self.model, self.data)
            R_t = self.data.xmat[self.torso_id].reshape(3, 3)
            p_w = self.data.xpos[self.torso_id] + R_t @ p_torso
            R_e = self.data.xmat[self.ee_id[side]].reshape(3, 3)
            pt = self.data.xpos[self.ee_id[side]] + R_e @ offset
            err = p_w - pt
            jacp = np.zeros((3, self.model.nv)); jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.data, jacp, jacr, pt, self.ee_id[side])
            J, e = jacp[:, dadr], err
            if R_torso is not None:
                R_g = R_t @ R_torso
                e_r = 0.5 * sum(np.cross(R_e[:, i], R_g[:, i]) for i in range(3))
                J = np.vstack([J, w_rot * jacr[:, dadr]]); e = np.concatenate([err, w_rot * e_r])
            if np.linalg.norm(err) < 1e-3 and (R_torso is None or np.linalg.norm(e[3:]) < 2e-3):
                break
            n = J.shape[0]
            Jp = J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(n))
            dq = Jp @ e + (np.eye(7) - Jp @ J) @ (k_null * (np.asarray(q_nom) - q))
            q = np.clip(q + 0.5 * dq, lo, hi)
        self.data.qpos[qadr] = q
        mujoco.mj_forward(self.model, self.data)
        R_e = self.data.xmat[self.ee_id[side]].reshape(3, 3)
        rot_err = None
        if R_torso is not None:
            R_rel = (self.data.xmat[self.torso_id].reshape(3, 3) @ R_torso).T @ R_e
            rot_err = float(np.degrees(np.arccos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1))))
        return q, float(np.linalg.norm(err)), rot_err


def hand_frame(side, pitch_deg, yaw_deg=0.0):
    """the hand's axes in the torso frame: fingers (hand x) forward, pitched down pitch_deg and turned outward yaw_deg, the PALM down
    (the Inspire palm faces the hand's -y on the left hand, +y on the right: the thumb side)"""
    p, y = np.radians(pitch_deg), np.radians(yaw_deg) * (1 if side == "left" else -1)
    x = np.array([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), -np.sin(p)])
    up = np.array([0.0, 0.0, 1.0]) if side == "left" else np.array([0.0, 0.0, -1.0])
    yv = up - (up @ x) * x; yv /= np.linalg.norm(yv)
    return np.stack([x, yv, np.cross(x, yv)], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--hand-point", default="0.15,0.02,0", help="the palm SURFACE point in the left wrist_yaw_link frame (m; right: y mirrored)")
    ap.add_argument("--ball-r", type=float, default=0.035, help="ball radius (m): the palm lands on the ball's top")
    ap.add_argument("--ready", default="0.27,0.21,0.10", help="READY palm point, left hand (right mirrored), torso frame")
    ap.add_argument("--l1", default="0.42,0.17,0.13", help="ball L1 centre (left hand), torso frame")
    ap.add_argument("--r1", default="0.42,-0.17,0.13", help="ball R1 centre (right hand)")
    ap.add_argument("--pair", default="0.40,0.25,0.16", help="ball P_L centre (left; P_R mirrored), torso frame")
    ap.add_argument("--pitch", type=float, default=20.0, help="fingers pitched down at a ball (deg)")
    ap.add_argument("--hold", type=float, default=1.0, help="hold at a ball (s)")
    ap.add_argument("--max-rate", type=float, default=0.85, help="peak joint speed of a move (rad/s; the Bridge arm slew is 1.0)")
    ap.add_argument("--min-move", type=float, default=0.9, help="shortest move (s)")
    a = ap.parse_args()

    point_reach.MODEL_CANDIDATES[:] = [ROBOT_XML]
    ik = PalmIK()
    P = lambda s: np.array([float(v) for v in s.split(",")])
    mir = lambda p: p * np.array([1.0, -1.0, 1.0])
    hp = {"left": mir(P(a.hand_point)), "right": P(a.hand_point)}   # the palm faces the hand's -y (left) / +y (right)
    top = np.array([0.0, 0.0, a.ball_r])
    d7 = {"left": np.array(DEFAULT14[:7]), "right": np.array(DEFAULT14[7:])}
    resid = {}

    def solve(name, side, p, pitch, q_nom, yaw=0.0):
        q, res, rot = ik.solve_point(side, p, hp[side], q_nom, R_torso=hand_frame(side, pitch, yaw), seed=q_nom)
        resid[f"{name}_{side}"] = {"pos_mm": round(res * 1000, 1), "rot_deg": round(rot, 1)}
        return q

    # READY first (pulled toward the default), then every reach pulled toward READY
    rdy = {"left": solve("ready", "left", P(a.ready), 10.0, d7["left"]), "right": solve("ready", "right", mir(P(a.ready)), 10.0, d7["right"])}
    keys = {"ready": np.concatenate([rdy["left"], rdy["right"]]), "default": np.array(DEFAULT14)}
    L1, R1, PL = P(a.l1), P(a.r1), P(a.pair)
    keys["L1"] = np.concatenate([solve("L1", "left", L1 + top, a.pitch, rdy["left"]), rdy["right"]])
    keys["R1"] = np.concatenate([rdy["left"], solve("R1", "right", R1 + top, a.pitch, rdy["right"])])
    keys["P"] = np.concatenate([solve("P", "left", PL + top, a.pitch, rdy["left"], yaw=10.0), solve("P", "right", mir(PL) + top, a.pitch, rdy["right"], yaw=10.0)])
    balls = [{"name": "L1", "side": "left", "key": "L1", "c_torso": L1.tolist()},
             {"name": "R1", "side": "right", "key": "R1", "c_torso": R1.tolist()},
             {"name": "P_L", "side": "left", "key": "P", "c_torso": PL.tolist()},
             {"name": "P_R", "side": "right", "key": "P", "c_torso": mir(PL).tolist()}]
    seq = [["move", "ready"], ["hold", 0.5],
           ["move", "L1"], ["hold", a.hold], ["move", "ready"], ["hold", 0.3],
           ["move", "R1"], ["hold", a.hold], ["move", "ready"], ["hold", 0.3],
           ["move", "P"], ["hold", a.hold], ["move", "ready"], ["hold", 0.4],
           ["move", "default"], ["hold", 1.0]]
    plan = {"hand_point": {k: v.tolist() for k, v in hp.items()}, "ball_r": a.ball_r,
            "order": "BridgeModule joint_set_arms: left 7 then right 7 (sh pitch, roll, yaw, elbow, wr roll, pitch, yaw)",
            "max_rate": a.max_rate, "min_move": a.min_move, "keys": {k: [round(float(v), 4) for v in q] for k, q in keys.items()},
            "seq": seq, "balls": balls, "ik_resid": resid, "model": ROBOT_XML}
    for k, v in resid.items():
        print(f"[plan] IK {k:12s} palm {v['pos_mm']} mm, hand orientation {v['rot_deg']} deg off")
    for k, q in keys.items():
        print(f"[plan] key {k:7s} {np.round(q, 2)}")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(plan, open(a.out, "w"), indent=1)
    print(f"[plan] -> {a.out}")


if __name__ == "__main__":
    main()
