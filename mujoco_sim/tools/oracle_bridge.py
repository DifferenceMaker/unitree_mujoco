"""oracle_bridge — the sidecar's half of the reward ORACLE (2026-09-10).

balance_metrics.py (tv env, owns DDS) forwards one JSON state datagram per tick
to reward_oracle.py (isaacsim env, owns torch + the Isaac reward functions) over
localhost UDP and hands the returned HUD row string to the sim exactly like the
old RewardLedger.hud_field() did. Same DDS inputs the ledger used:
    rt/sim_base_pose  (String_ JSON, simulate main.cc v3/v4 ground truth)
    rt/anchor_point   (String_ JSON {"p":[x,y,z]} base frame)
    rt/lowcmd         (LowCmd_, the controller's joint targets -> action terms)
    rt/lean_cmd       (String_ JSON {"lean": rad}, the Lean slider)
plus the lowstate q/dq/tau/quat/gyro the sidecar loop already has in hand.

A `meta` message (SDK joint names + ranges from the sidecar's MJCF, MuJoCo body
names) is sent first and re-sent every 5 s so an oracle restarted mid-run picks
the contract up again. If no reply arrives within `wait_s` the previous rows are
reused; if the oracle is silent for > 2 s the HUD field is left empty (the sim
hides a stale ledger after 2 s on its own).
"""
from __future__ import annotations

import json
import socket
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_


class OracleBridge:
    def __init__(self, port: int, joint_names_sdk, jnt_range, body_names, wait_s: float = 0.010):
        self.addr = ("127.0.0.1", int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.wait_s = wait_s
        self.meta = {"type": "meta", "joint_names_sdk": list(joint_names_sdk),
                     "jnt_range": [[float(a), float(b)] for a, b in jnt_range],
                     "body_names": list(body_names)}
        self._meta_t = 0.0
        self._pose = None
        self._anchor = None
        self._qcmd = None
        self._lean = None
        self._last_rows = ""
        self._last_reply_t = 0.0
        self.n_sent = 0
        self.n_recv = 0
        self._pose_sub = ChannelSubscriber("rt/sim_base_pose", String_)
        self._pose_sub.Init(self._on_pose, 10)
        self._anchor_sub = ChannelSubscriber("rt/anchor_point", String_)
        self._anchor_sub.Init(self._on_anchor, 10)
        self._lean_sub = ChannelSubscriber("rt/lean_cmd", String_)
        self._lean_sub.Init(self._on_lean, 10)
        try:
            self._cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
            self._cmd_sub.Init(self._on_lowcmd, 10)
        except Exception:
            self._cmd_sub = None
        print(f"[BRIDGE] reward oracle at udp://{self.addr[0]}:{self.addr[1]}; "
              f"{len(body_names)} bodies, {len(joint_names_sdk)} joints in meta", flush=True)

    # ---- DDS callbacks ----
    def _on_pose(self, msg):
        try:
            self._pose = json.loads(msg.data)
        except Exception:
            pass

    def _on_anchor(self, msg):
        try:
            self._anchor = json.loads(msg.data)["p"]
        except Exception:
            pass

    def _on_lean(self, msg):
        try:
            self._lean = float(json.loads(msg.data)["lean"])
        except Exception:
            pass

    def _on_lowcmd(self, msg):
        try:
            self._qcmd = [float(mc.q) for mc in msg.motor_cmd[:27]]
        except Exception:
            pass

    # ---- per tick ----
    def tick(self, q, dq, tau, quat, gyro) -> str:
        now = time.monotonic()
        if now - self._meta_t > 5.0:
            try:
                self.sock.sendto(json.dumps(self.meta).encode(), self.addr)
            except OSError:
                pass
            self._meta_t = now
        state = {"type": "state", "t": now,
                 "q": [float(x) for x in q], "dq": [float(x) for x in dq], "tau": [float(x) for x in tau],
                 "quat": [float(x) for x in quat], "gyro": [float(x) for x in np.asarray(gyro).tolist()],
                 "pose": self._pose, "anchor": self._anchor, "qcmd": self._qcmd, "lean": self._lean}
        try:
            self.sock.sendto(json.dumps(state).encode(), self.addr)
            self.n_sent += 1
        except OSError:
            return self._stale_rows(now)
        deadline = now + self.wait_s
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                self._last_rows = data.decode(errors="replace")
                self._last_reply_t = time.monotonic()
                self.n_recv += 1
                # drain any older replies queued behind it
                continue
            except BlockingIOError:
                if time.monotonic() >= deadline or self._last_reply_t >= now:
                    break
                time.sleep(0.0005)
        return self._stale_rows(now)

    def _stale_rows(self, now: float) -> str:
        return self._last_rows if now - self._last_reply_t < 2.0 else ""

    def hud_field(self, rows: str) -> str:
        return rows
