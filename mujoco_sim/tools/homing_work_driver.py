#!/usr/bin/env python3
"""homing_work_driver.py -- the "work at the table" arm sequence on a homing rig (2026-11-04, the homing + work video).

Runs INSIDE the rig container (docker exec; tools/homing_video_record.sh --work starts it): the MovementModule venv + /opt/ros/humble,
CYCLONEDDS_URI = the container's loopback config, SIM_DDS_DOMAIN = the rig's SDK domain (the container's own env). It waits for the
rig's homing trial record (the commander writes it at the end of the SETTLE after the balance switch); on ARRIVED it plays the plan of
tools/homing_work_plan.py: min-jerk joint-space moves between the plan's keyframes (each move as long as its largest joint step needs
at --max-rate peak speed: min-jerk peak = 1.875 x step / duration), holds at the balls, PACED ON THE SIM CLOCK (rt/sim_base_pose "t",
one arm command per pose message = 50 Hz of sim time). The arm command goes to /BridgeModule/joint_set_arms, the channel ActionModule
drives on the robot: the Bridge slews the arms (1 rad/s, sim time in lock-step) and MovementModule feeds the commanded slice to the
balance policy as its arm_pose_command. Nothing in MovementModule / the Bridge changes.

Output (--out, JSON lines): {"kind": "plan", ...} = the trial record, t0 / t_end, every segment and per ball its window (t_move0 =
the reach starts, t_arrive = the move ends, t_hold_end = the hand leaves) in sim seconds; {"kind": "done", ...} at the end.
"""
import argparse
import json
import math
import os
import threading
import time

import numpy as np


def minjerk(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--trials-log", required=True, help="the rig's homing trial log (the commander's HOMING_LOG)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--lead", type=float, default=1.0, help="sim s from the trial record to the first move")
    ap.add_argument("--tail", type=float, default=3.0, help="sim s of commands after the sequence (the arms settle on the default)")
    ap.add_argument("--timeout", type=float, default=900.0, help="wall s to wait for the trial record")
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    keys = {k: np.array(v, dtype=float) for k, v in plan["keys"].items()}

    import rclpy
    from Ros2_connector import Ros2
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    rclpy.init()
    ros = Ros2("HomingWork", node_name="homing_work_driver")
    ChannelFactoryInitialize(int(os.environ.get("SIM_DDS_DOMAIN", "1")), os.environ.get("SIM_DDS_IFACE", "lo"))
    latest = {"t": None, "qj": None, "n": 0}
    evt = threading.Event()

    def cb(msg):
        try:
            d = json.loads(msg.data)
            latest["t"], latest["qj"] = float(d["t"]), d.get("qj")
            latest["n"] += 1
            evt.set()
        except Exception:
            pass

    sub = ChannelSubscriber("rt/sim_base_pose", String_)
    sub.Init(cb, 10)
    out = open(a.out, "a")

    n0 = sum(1 for _ in open(a.trials_log)) if os.path.exists(a.trials_log) else 0
    print(f"[work] waiting for the trial record ({a.trials_log}, {n0} lines now)", flush=True)
    rec, w0 = None, time.time()
    while rec is None and time.time() - w0 < a.timeout:
        time.sleep(0.2)
        lines = [l for l in open(a.trials_log)] if os.path.exists(a.trials_log) else []
        if len(lines) > n0:
            rec = json.loads(lines[n0])
    if rec is None or rec.get("result") != "ARRIVED":
        out.write(json.dumps({"kind": "skipped", "trial": rec}) + "\n"); out.close()
        print(f"[work] no ARRIVED record ({rec and rec.get('result')}): nothing to do", flush=True)
        return 0
    evt.clear(); evt.wait(5.0)
    t_now, qj = latest["t"], latest["qj"]
    cur = np.array(qj[13:27], dtype=float)
    t0 = math.ceil((t_now + a.lead) / 0.02) * 0.02
    segs, t, last = [], t0, None
    for op in plan["seq"]:
        if op[0] == "move":
            q1 = keys[op[1]]
            T = max(plan["min_move"], 1.875 * float(np.max(np.abs(q1 - cur))) / plan["max_rate"])
            segs.append({"kind": "move", "key": op[1], "t_a": round(t, 3), "t_b": round(t + T, 3), "q_a": cur.tolist(), "q_b": q1.tolist()})
            t += T; cur = q1; last = op[1]
        else:
            segs.append({"kind": "hold", "key": last, "t_a": round(t, 3), "t_b": round(t + float(op[1]), 3), "q_a": cur.tolist(), "q_b": cur.tolist()})
            t += float(op[1])
    t_end = t
    balls = []
    for b in plan["balls"]:
        i = next(k for k, s in enumerate(segs) if s["kind"] == "move" and s["key"] == b["key"])
        hold = segs[i + 1] if i + 1 < len(segs) and segs[i + 1]["kind"] == "hold" else segs[i]
        balls.append(dict(b, t_move0=segs[i]["t_a"], t_arrive=segs[i]["t_b"], t_hold_end=hold["t_b"]))
    out.write(json.dumps({"kind": "plan", "trial": rec, "t_record": t_now, "t0": t0, "t_end": round(t_end, 3), "q_arm0": [round(v, 4) for v in qj[13:27]],
                          "hand_point": plan["hand_point"], "ball_r": plan["ball_r"], "balls": balls,
                          "segments": [{k: v for k, v in s.items() if k not in ("q_a", "q_b")} for s in segs]}) + "\n")
    out.flush()
    print(f"[work] ARRIVED: sequence {t0:.2f} -> {t_end:.2f} s sim ({len(segs)} segments, {len(balls)} balls)", flush=True)

    def target(tt):
        for s in segs:
            if tt < s["t_b"]:
                if tt < s["t_a"]:
                    return np.array(s["q_a"])
                u = minjerk((tt - s["t_a"]) / max(s["t_b"] - s["t_a"], 1e-6))
                return np.array(s["q_a"]) + u * (np.array(s["q_b"]) - np.array(s["q_a"]))
        return np.array(segs[-1]["q_b"])

    n_pub, n_seen = 0, latest["n"]
    q_hold = np.array(qj[13:27], dtype=float)
    while True:
        if not evt.wait(30.0):
            print("[work] no pose for 30 s: the rig stopped", flush=True); break
        evt.clear()
        if latest["n"] == n_seen:
            continue
        n_seen = latest["n"]; tt = latest["t"]
        q = q_hold if tt < t0 else target(tt)
        ros.publish_data("/BridgeModule/joint_set_arms", [float(v) for v in q], "f32arr")
        n_pub += 1
        if tt >= t_end + a.tail:
            break
    out.write(json.dumps({"kind": "done", "t": latest["t"], "n_pub": n_pub}) + "\n"); out.close()
    print(f"[work] done at {latest['t']:.2f} s sim, {n_pub} arm commands", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
