"""tau_monitor.py — per-motor torque readout for sim2sim (the ROS
lowstate_health_probe, sim-side).

Subscribes rt/lowstate (unitree_hg LowState_, which the MuJoCo bridge fills
with mj_data.ctrl per motor as tau_est) and prints once a second:
    |tau|max=<Nm>(m<idx>)   hipR L(m2)=<Nm> R(m8)=<Nm>   maxT=<C>(m<idx>)
mirroring the pc4 probe's key line so anti-squeeze runs can be judged on the
actual internal-force number, not just the eyeball.

SDK motor order (h1_2, per h1_2_comx06.xml): 0-5 L leg (hip_yaw,hip_pitch,
hip_roll,knee,ankle_pitch,ankle_roll), 6-11 R leg, 12 torso, 13+ arms.
=> hip_roll = m2 (L), m8 (R): the squeeze pair. knee = m3/m9 (~45 Nm normal).

Usage (alongside a running sim2sim; tv env has unitree_sdk2py):
    /home/aspired-comp-2/miniconda3/envs/tv/bin/python tau_monitor.py
    # options: --iface lo --domain 0 --hz 1 --threshold 100
"""
import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

HIP_ROLL_L, HIP_ROLL_R = 2, 8      # the internal-force squeeze pair
KNEE_L, KNEE_R = 3, 9              # reference: ~45 Nm in a normal stand
NAMES = {2: "L_hip_roll", 8: "R_hip_roll", 3: "L_knee", 9: "R_knee"}

_latest = {"msg": None}


def _on_msg(msg: LowState_):
    _latest["msg"] = msg


def main():
    ap = argparse.ArgumentParser(description="sim2sim per-motor torque monitor")
    ap.add_argument("--iface", default="lo", help="DDS interface (sim runs on lo)")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--threshold", type=float, default=100.0,
                    help="flag hip-roll |tau| above this (Nm); 120 = deploy wire cap")
    ap.add_argument("--n_motors", type=int, default=27)
    args = ap.parse_args()

    ChannelFactoryInitialize(args.domain, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(_on_msg, 10)
    print(f"[tau] subscribed rt/lowstate on {args.iface} (domain {args.domain}); "
          f"hip-roll flag threshold {args.threshold} Nm; waiting for data...", flush=True)

    peak_hiproll = 0.0
    period = 1.0 / args.hz
    while True:
        time.sleep(period)
        msg = _latest["msg"]
        if msg is None:
            continue
        ms = msg.motor_state
        tau = [abs(ms[i].tau_est) for i in range(args.n_motors)]
        temp = [ms[i].temperature for i in range(args.n_motors)]

        imax = max(range(args.n_motors), key=lambda i: tau[i])
        tmax = max(range(args.n_motors), key=lambda i: temp[i])
        hr_l, hr_r = tau[HIP_ROLL_L], tau[HIP_ROLL_R]
        peak_hiproll = max(peak_hiproll, hr_l, hr_r)

        flag = "  <== HIP-ROLL SQUEEZE" if max(hr_l, hr_r) > args.threshold else ""
        print(f"|tau|max={tau[imax]:6.1f}(m{imax})  "
              f"hipR L(m2)={hr_l:6.1f} R(m8)={hr_r:6.1f} (peak {peak_hiproll:.0f})  "
              f"knee L={tau[KNEE_L]:.0f} R={tau[KNEE_R]:.0f}  "
              f"maxT={temp[tmax]:.0f}C(m{tmax}){flag}", flush=True)


if __name__ == "__main__":
    main()
