#!/usr/bin/env python3
"""wc_mirror — host-side gauge feeder for the walk rig (2026-08-27).

walk_teleop lives in the ROS container, which has no unitree_sdk2py, so it
cannot publish rt/wirelesscontroller for the sim's walk_hud gauges itself.
It DOES print every command change to the stack log:
    [walk_teleop] vx=+0.10 vy=+0.00 wz=+0.00
This mirror tails the stack log, keeps the latest (vx, vy, wz) and publishes
WirelessController_ (ly=vx, lx=-vy, rx=-wz — walk_hud's mapping) at 20 Hz on
the sim DDS domain, so the gauges wake with the first key press and stay live.
Usage: wc_mirror.py <stack_log> [domain=1] [iface=lo]
"""
import re
import sys
import time

log_path = sys.argv[1]
domain = int(sys.argv[2]) if len(sys.argv) > 2 else 1
iface = sys.argv[3] if len(sys.argv) > 3 else "lo"

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_  # noqa: E402

ChannelFactoryInitialize(domain, iface)
pub = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
pub.Init()
msg = WirelessController_(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0)
rx = re.compile(r"\[walk_teleop\] vx=([+-]?[\d.]+) vy=([+-]?[\d.]+) wz=([+-]?[\d.]+)")
print(f"[wc_mirror] tailing {log_path} -> rt/wirelesscontroller (domain {domain}, {iface})", flush=True)
seen = False
with open(log_path, "r", errors="ignore") as f:
    while True:
        line = f.readline()
        if line:
            m = rx.search(line)
            if m:
                msg.ly, msg.lx, msg.rx = float(m.group(1)), -float(m.group(2)), -float(m.group(3))
                seen = True
        else:
            time.sleep(0.05)
        if seen:
            pub.Write(msg)
