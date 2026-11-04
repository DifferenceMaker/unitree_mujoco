#!/usr/bin/env python3
"""sim_state_recorder.py -- record the sim's ground-truth FULL state for a kinematic replay (2026-11-04, the homing video).

Source: rt/sim_base_pose (main.cc PublishSidecar, every 10th physics frame = 50 Hz of SIM time, also in lock-step): "t" (sim s),
"seq", "p" (base xyz), "q" (base quat wxyz), "qj" (every qpos after the free joint = all 27 H1-2 joints, the hand790 hands are
rigid). Nothing is added to the sim or the stack: a passive BEST-EFFORT reader (never back-pressures the sim's writer), polled with
take() into a deep KEEP_LAST queue so a slow Python loop loses nothing on the reader side; a lost wire sample shows as a "seq" gap.

Output: <out>.bin = float64 rows [t, seq, px, py, pz, qw, qx, qy, qz, qj...] (row width in <out>.json, written on the first sample),
read with numpy.fromfile(path).reshape(-1, width).

Usage (tv env: cyclonedds + unitree_sdk2py):  ~/miniconda3/envs/tv/bin/python tools/sim_state_recorder.py --domain 22 --out logs/x
Stops on SIGINT/SIGTERM (or --max-s wall seconds).
"""
import argparse
import json
import os
import signal
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, required=True, help="the rig's SDK DDS domain (instance k -> 1 + k)")
    ap.add_argument("--out", required=True, help="output path without extension (.bin + .json)")
    ap.add_argument("--topic", default="rt/sim_base_pose")
    ap.add_argument("--max-s", type=float, default=0.0, help="stop after this many wall seconds (0 = until a signal)")
    ap.add_argument("--dds-config", default=os.path.join(HERE, "cyclonedds_lo.xml"), help="the container's loopback config (unicast peers)")
    a = ap.parse_args()

    os.environ.setdefault("CYCLONEDDS_URI", "file://" + os.path.abspath(a.dds_config))
    a.out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    os.chdir(os.path.dirname(a.out))   # the config's warning trace (cyclonedds.log, cwd-relative) lands next to the recording, not in the repo
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    dp = DomainParticipant(a.domain)
    tp = Topic(dp, a.topic, String_)
    rd = DataReader(dp, tp, qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(4000)))

    stop = {"f": False}
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.__setitem__("f", True))

    fh = None; width = None; n = 0; gaps = 0; last_seq = None; t_wall0 = time.time(); t_last_print = t_wall0; t_sim = None
    print(f"[rec] {a.topic} on domain {a.domain} -> {a.out}.bin", flush=True)
    while not stop["f"]:
        if a.max_s and time.time() - t_wall0 > a.max_s:
            break
        try:
            samples = rd.take(N=512)
        except Exception as e:  # noqa: BLE001
            print(f"[rec] take failed: {e}", flush=True); time.sleep(0.05); continue
        if not samples:
            time.sleep(0.004)
            continue
        rows = []
        for smp in samples:
            try:
                js = json.loads(smp.data)
            except Exception:  # noqa: BLE001  (InvalidSample / a truncated buffer)
                continue
            qj = js.get("qj")
            if qj is None or "p" not in js:
                continue
            seq = js.get("seq", -1)
            if last_seq is not None and seq > last_seq + 1:
                gaps += seq - last_seq - 1
            last_seq = seq
            row = [js["t"], seq, *js["p"], *js["q"], *qj]
            if width is None:
                width = len(row)
                with open(a.out + ".json", "w") as jf:
                    json.dump({"width": width, "cols": ["t", "seq", "px", "py", "pz", "qw", "qx", "qy", "qz"] + [f"qj{k}" for k in range(len(qj))],
                               "topic": a.topic, "domain": a.domain, "nq": 7 + len(qj)}, jf)
                fh = open(a.out + ".bin", "ab")
            if len(row) != width:
                continue
            rows.append(row)
            t_sim = js["t"]
        if rows:
            np.asarray(rows, dtype=np.float64).tofile(fh)
            n += len(rows)
        if time.time() - t_last_print > 30:
            t_last_print = time.time()
            if fh:
                fh.flush()
            print(f"[rec] {n} samples, sim t {t_sim}, seq gaps {gaps}", flush=True)
    if fh:
        fh.close()
    print(f"[rec] done: {n} samples, seq gaps {gaps}, last sim t {t_sim}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
