"""anchor_monitor.py — HOW WELL is the policy holding the anchor?

Subscribes rt/anchor_point (the sim's base-frame anchor, published ~50 Hz by
anchor_pub.h) and prints, once a second:
    err_xy   horizontal distance between the CURRENT base->anchor vector and
             the NOMINAL one (captured at first sample = the engage pose)
    err_yaw  how far the base has rotated away from facing the anchor (deg)
    rms10    10 s rolling RMS of err_xy — THE hold-quality number
    max10    worst excursion in the window (push recovery depth)
Perfect anchoring = err_xy ~ 0.00-0.03 m forever; a push shows as a spike in
max10 with rms10 recovering — recovery time is readable off the err_xy column.

Usage:  <tv python> anchor_monitor.py            # sim (lo, domain 1)
"""
import argparse, collections, json, math, time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

latest = {"p": None}


def on_msg(msg: String_):
    try:
        latest["p"] = json.loads(msg.data)["p"]
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--domain", type=int, default=1, help="sim DDS domain (launcher uses 1)")
    args = ap.parse_args()
    ChannelFactoryInitialize(args.domain, args.iface)
    sub = ChannelSubscriber("rt/anchor_point", String_)
    sub.Init(on_msg, 10)
    print(f"[anchor] listening rt/anchor_point on {args.iface} domain {args.domain} ...")
    nominal = None
    hist = collections.deque(maxlen=10)
    while True:
        time.sleep(1.0)
        p = latest["p"]
        if p is None:
            continue
        if nominal is None:
            nominal = list(p)
            print(f"[anchor] nominal captured: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) base-frame")
            continue
        err_xy = math.hypot(p[0] - nominal[0], p[1] - nominal[1])
        yaw_now, yaw_nom = math.atan2(p[1], p[0]), math.atan2(nominal[1], nominal[0])
        err_yaw = math.degrees(abs(math.atan2(math.sin(yaw_now - yaw_nom), math.cos(yaw_now - yaw_nom))))
        hist.append(err_xy)
        rms = math.sqrt(sum(e * e for e in hist) / len(hist))
        print(f"err_xy={err_xy:5.3f} m  err_yaw={err_yaw:4.1f} deg  "
              f"rms10={rms:5.3f}  max10={max(hist):5.3f}", flush=True)


if __name__ == "__main__":
    main()
