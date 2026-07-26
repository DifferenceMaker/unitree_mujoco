"""tau_monitor.py — per-second torque + lean readout for sim2sim AND sim2real.

Every incoming rt/lowstate message (robot publishes ~500 Hz) is captured in a
callback; once per second the mean over ALL messages in that second is printed.
So each line is a tight 1-second average of hundreds of samples — noise-smoothed
without the temporal smear of a multi-second window. N (samples averaged) is
shown so the reading is auditable.

Prints per line:
  - lean (projected gravity, base tilt): leanFwd (pitch, + = nose-down/forward),
    leanLat (roll) — the DIRECT, mass-free lean readout = the CoM-belief-error.
  - signed ankle-pitch/roll torques + a CoM-offset ESTIMATE from the static
    ankle balance  tau ~= m*g*d :
      CoM_x = (tauP_L + tauP_R)/(m*g)   [fore-aft, the clean one]
      CoM_y = (tauR_L + tauR_R)/(m*g)   [lateral, APPROX — a lateral CoM shift
              also redistributes vertical load between the two feet, which
              ankle-roll torque alone misses]
  - knee + hip_roll |tau| + max motor temp.

MEASURE ON A SETTLED BALANCING POLICY (not FixStand): when it holds near-upright,
leanFwd/leanLat = the residual lean it can't correct = CoM-belief error, and the
mean ankle torque = the true CoM moment.

SDK motor order (h1_2): 0-5 L leg (hip_yaw,hip_pitch,hip_roll,knee,ankle_pitch,
ankle_roll), 6-11 R leg, 12 torso, 13+ arms.
  hip_roll m2/m8 | knee m3/m9 | ankle_pitch m4/m10 | ankle_roll m5/m11

Usage (monitor on the work PC, robot's DDS net; controller runs on pc4):
    /home/aspired-comp-2/miniconda3/envs/tv/bin/python tau_monitor.py \
        --iface enp6s0 --domain 0 --mass 77.0
    # sim2sim: --iface lo ; print twice a second: --hz 2
"""
import argparse
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

HIP_ROLL_L, HIP_ROLL_R = 2, 8
KNEE_L, KNEE_R = 3, 9
ANK_PITCH_L, ANK_PITCH_R = 4, 10
ANK_ROLL_L, ANK_ROLL_R = 5, 11
G = 9.81


def projected_gravity(quat):
    """world -Z in the base frame, from quaternion (w,x,y,z).
    x = fore-aft tilt (+ = pitched forward), y = lateral tilt, z ~ -1 upright."""
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    return (2.0 * (w * y - x * z), -2.0 * (y * z + w * x), 2.0 * (x * x + y * y) - 1.0)


def main():
    ap = argparse.ArgumentParser(description="per-second torque + lean monitor (sim2sim / sim2real)")
    ap.add_argument("--iface", default="lo", help="DDS interface (sim=lo; real robot=enp6s0)")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--hz", type=float, default=1.0, help="PRINT rate; each line averages this window of messages")
    ap.add_argument("--mass", type=float, default=77.0,
                    help="robot mass (kg) for the CoM-offset estimate; scales the mm only "
                         "— lean(proj_grav) is mass-free")
    ap.add_argument("--threshold", type=float, default=100.0, help="flag hip-roll |tau| above this (Nm)")
    ap.add_argument("--n_motors", type=int, default=27)
    args = ap.parse_args()

    mg = args.mass * G
    buf = []
    lock = threading.Lock()

    def on_msg(msg: LowState_):
        try:
            ms = msg.motor_state
            row = (
                *projected_gravity(msg.imu_state.quaternion),        # gx, gy, gz
                float(ms[ANK_PITCH_L].tau_est), float(ms[ANK_PITCH_R].tau_est),
                float(ms[ANK_ROLL_L].tau_est), float(ms[ANK_ROLL_R].tau_est),
                abs(float(ms[KNEE_L].tau_est)), abs(float(ms[KNEE_R].tau_est)),
                abs(float(ms[HIP_ROLL_L].tau_est)), abs(float(ms[HIP_ROLL_R].tau_est)),
                max(max(ms[i].temperature) for i in range(args.n_motors)),
            )
        except Exception:
            return
        with lock:
            buf.append(row)

    ChannelFactoryInitialize(args.domain, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_msg, 10)
    print(f"[tau] subscribed rt/lowstate on {args.iface} (domain {args.domain}); mass={args.mass}kg; "
          f"averaging every {1.0/args.hz:.2f}s. MEASURE ON A SETTLED POLICY. waiting for data...", flush=True)
    print("  t | N    |  leanFwd leanLat |  ankP_L  ankP_R ->CoMx |  ankR_L  ankR_R ->CoMy* | knee LR | hipR LR | maxT",
          flush=True)

    period = 1.0 / args.hz
    peak_hr = 0.0
    t0 = None
    while True:
        time.sleep(period)
        with lock:
            rows = buf[:]
            buf.clear()
        n = len(rows)
        if n == 0:
            print("   -- no rt/lowstate messages this window --", flush=True)
            continue
        if t0 is None:
            t0 = time.monotonic()
        t = time.monotonic() - t0
        m = [sum(col) / n for col in zip(*rows)]
        gx, gy, gz, pL, pR, rL, rR, kL, kR, hL, hR, mt = m
        comx = (pL + pR) / mg * 1000.0
        comy = (rL + rR) / mg * 1000.0
        peak_hr = max(peak_hr, hL, hR)
        flag = "  <== HIP-ROLL SQUEEZE" if max(hL, hR) > args.threshold else ""
        print(f"{t:4.0f}| {n:4d} | {gx:+7.3f} {gy:+7.3f} | {pL:+6.1f} {pR:+6.1f} ->{comx:+5.0f}mm "
              f"| {rL:+6.1f} {rR:+6.1f} ->{comy:+5.0f}mm | {kL:2.0f} {kR:2.0f} | {hL:3.0f} {hR:3.0f} "
              f"| {mt:.0f}C{flag}", flush=True)


if __name__ == "__main__":
    main()
