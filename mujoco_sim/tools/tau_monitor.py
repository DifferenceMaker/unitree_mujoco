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

  - --angles (2026-09-28, the lateral-asymmetry read): a second line per window with the
    MEASURED joint angles L/R (deg) for hip_roll / knee / ankle_roll and the SIGNED hip-roll
    torques. In FixStand with the policy OFF the command is symmetric, so a left/right angle
    difference there is a joint ZERO-OFFSET (calibration), while equal angles with unequal
    torques is a mass (CoM) offset -- the two reasons a leg looks "heavier" separated in one read.
  - MARKS (2026-09-28, the two-scales weigh-in): type the two scale readings "<left> <right>"
    (kg) and press Enter right after the photo. The tool stamps them with the wall clock and the
    lateral tilt of that moment, prints D = left - right and the raw CoM offset
    y_raw = (d/2) * D / (left+right), and once >= 3 marks exist fits D against the tilt:
    the load split follows the CoM ground projection, and a CoM ~0.9 m up moves ~8 kg per
    degree of roll, so the placement-to-placement roll of a PD-held FixStand swamps a 5 mm
    offset; the value of the fit at ZERO tilt is the body's own offset. The slope should
    come out near 2*m*zcom*tan(1 deg)/d (printed as "expected"), which checks the method.
    Enter alone = a mark without scale numbers (tilt + angles only). --log appends everything.
SDK motor order (h1_2): 0-5 L leg (hip_yaw,hip_pitch,hip_roll,knee,ankle_pitch,
ankle_roll), 6-11 R leg, 12 torso, 13+ arms.
  hip_roll m2/m8 | knee m3/m9 | ankle_pitch m4/m10 | ankle_roll m5/m11

Usage (monitor on the work PC, robot's DDS net; controller runs on pc4):
    /home/aspired-comp-2/miniconda3/envs/tv/bin/python tau_monitor.py \
        --iface enp6s0 --domain 0 --mass 77.0
    # sim2sim: --iface lo ; print twice a second: --hz 2
"""
import argparse
from collections import Counter
import math
import sys
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


def parse_mark(text):
    """'39.3 36.55' -> (39.3, 36.55); anything else -> None (a bare Enter is a tilt-only mark)."""
    parts = text.replace(",", " ").split()
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def fit_marks(marks):
    """marks: list of (roll_deg, D_kg). Least squares D = a + b*roll. Returns (a, b, n) or None."""
    pts = [(r, d) for r, d in marks if r is not None and d is not None]
    n = len(pts)
    if n < 3:
        return None
    mr = sum(r for r, _ in pts) / n
    md = sum(d for _, d in pts) / n
    sxx = sum((r - mr) ** 2 for r, _ in pts)
    if sxx < 1e-9:
        return None
    b = sum((r - mr) * (d - md) for r, d in pts) / sxx
    a = md - b * mr
    return a, b, n


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
    ap.add_argument("--angles", action="store_true",
                    help="also print measured hip_roll/knee/ankle_roll angles L/R (deg) + signed hip-roll tau")
    ap.add_argument("--d", type=float, default=0.305,
                    help="foot CENTRE-to-centre separation (m) for the two-scales CoM estimate (2026-09-28: (22+39)/2 cm)")
    ap.add_argument("--zcom", type=float, default=0.9, help="CoM height (m), only for the expected roll slope printout")
    ap.add_argument("--log", default=None, help="append every line and mark to this file")
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
                # temperature[0] = case/NTC (slow — what a hand feels);
                # temperature[-1] = driver's WINDING/junction estimate (fast,
                # load-following — the channel the thermal guard cares about).
                # Track them separately + WHICH motor owns the winding max:
                # a single anonymous max hops between motors and reads as noise
                # (the "105C but the robot is cold" confusion, 2026-08-03).
                max(float(ms[i].temperature[0]) for i in range(args.n_motors)),
                max(float(ms[i].temperature[-1]) for i in range(args.n_motors)),
                float(max(range(args.n_motors), key=lambda i: ms[i].temperature[-1])),
                # --angles columns (always captured, printed on request): measured q (rad)
                # hip_roll L/R, knee L/R, ankle_roll L/R, then SIGNED hip-roll tau L/R
                float(ms[HIP_ROLL_L].q), float(ms[HIP_ROLL_R].q),
                float(ms[KNEE_L].q), float(ms[KNEE_R].q),
                float(ms[ANK_ROLL_L].q), float(ms[ANK_ROLL_R].q),
                float(ms[HIP_ROLL_L].tau_est), float(ms[HIP_ROLL_R].tau_est),
            )
        except Exception:
            return
        with lock:
            buf.append(row)

    try:
        ChannelFactoryInitialize(args.domain, args.iface)
    except Exception:
        import subprocess
        ifaces = subprocess.run(["ls", "/sys/class/net"], capture_output=True, text=True).stdout.split()
        raise SystemExit(f"[tau] DDS init failed on iface '{args.iface}' — available: {', '.join(ifaces)} "
                         f"(robot LAN is usually enp6s0; sim is lo)")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_msg, 10)
    print(f"[tau] subscribed rt/lowstate on {args.iface} (domain {args.domain}); mass={args.mass}kg; "
          f"averaging every {1.0/args.hz:.2f}s. MEASURE ON A SETTLED POLICY. waiting for data...", flush=True)
    print("  clock     t | N    |  leanFwd leanLat |  ankP_L  ankP_R ->CoMx |  ankR_L  ankR_R ->CoMy* | knee LR | hipR LR | case/wind@motor",
          flush=True)

    logf = open(args.log, "a") if args.log else None

    def out(line):
        print(line, flush=True)
        if logf:
            logf.write(line + "\n"); logf.flush()

    last = {}          # the most recent window's values, for the MARK thread
    marks = []         # (roll_deg, D_kg)
    mark_n = [0]
    deg = 57.29577951308232
    slope_expected = 2.0 * args.mass * args.zcom * math.tan(math.radians(1.0)) / args.d   # kg per degree of roll

    def stdin_marks():
        for text in sys.stdin:
            with lock:
                L = dict(last)
            if not L:
                out(">>> MARK ignored: no rt/lowstate data yet"); continue
            mark_n[0] += 1
            roll = L["gy_deg"]
            sc = parse_mark(text)
            head = (f">>> MARK {mark_n[0]} {time.strftime('%H:%M:%S')} | leanLat {L['gy']:+.4f} rad ({roll:+.2f} deg) "
                    f"leanFwd {L['gx']:+.4f} | q hipR L {L['qhL']*deg:+.2f} R {L['qhR']*deg:+.2f} ankR L {L['qaL']*deg:+.2f} R {L['qaR']*deg:+.2f}")
            if sc is None:
                out(head + " | (no scale numbers)"); marks.append((roll, None)); continue
            fl, fr = sc
            D = fl - fr
            y_raw = (args.d / 2.0) * D / (fl + fr) * 1000.0
            out(head + f" | scales L {fl:.2f} R {fr:.2f} sum {fl+fr:.2f} kg  D {D:+.2f} kg -> y_raw {y_raw:+.1f} mm"
                + (f"  !! sum {fl+fr:.1f} vs mass {args.mass:.1f}: robot moving or harness loaded" if abs(fl + fr - args.mass) > 2.0 else ""))
            marks.append((roll, D))
            fit = fit_marks(marks)
            if fit:
                a, b, n = fit
                y0 = (args.d / 2.0) * a / args.mass * 1000.0
                out(f">>> FIT n={n}: D = {a:+.2f} kg + {b:+.2f} kg/deg * roll  ->  body offset at ZERO roll = {y0:+.1f} mm "
                    f"(slope expected ~{slope_expected:+.1f} kg/deg for a CoM {args.zcom:.2f} m up; a slope far from it = the reads are not tracking the tilt)")
    threading.Thread(target=stdin_marks, daemon=True).start()
    out(f"[tau] MARKS: type '<left> <right>' (kg) + Enter after each photo; d={args.d:.3f} m, expected roll slope {slope_expected:+.1f} kg/deg")

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
        gx, gy, gz, pL, pR, rL, rR, kL, kR, hL, hR, tcase, twind, _, qhL, qhR, qkL, qkR, qaL, qaR, thL, thR = m
        hot = Counter(int(r[13]) for r in rows).most_common(1)[0][0]
        comx = (pL + pR) / mg * 1000.0
        comy = (rL + rR) / mg * 1000.0
        peak_hr = max(peak_hr, hL, hR)
        flag = "  <== HIP-ROLL SQUEEZE" if max(hL, hR) > args.threshold else ""
        with lock:
            last.update(gx=gx, gy=gy, gy_deg=math.degrees(math.asin(max(-1.0, min(1.0, gy)))),
                        qhL=qhL, qhR=qhR, qaL=qaL, qaR=qaR, qkL=qkL, qkR=qkR)
        out(f"{time.strftime('%H:%M:%S')} {t:4.0f}| {n:4d} | {gx:+7.3f} {gy:+7.3f} | {pL:+6.1f} {pR:+6.1f} ->{comx:+5.0f}mm "
            f"| {rL:+6.1f} {rR:+6.1f} ->{comy:+5.0f}mm | {kL:2.0f} {kR:2.0f} | {hL:3.0f} {hR:3.0f} "
            f"| case {tcase:.0f}C wind {twind:.0f}C@m{hot}{flag}")
        if args.angles:
            out(f"      q(deg) hipR L {qhL*deg:+6.2f} R {qhR*deg:+6.2f} (L+R {(qhL+qhR)*deg:+5.2f}) | knee L {qkL*deg:5.2f} R {qkR*deg:5.2f} "
                f"(L-R {(qkL-qkR)*deg:+5.2f}) | ankR L {qaL*deg:+6.2f} R {qaR*deg:+6.2f} (L+R {(qaL+qaR)*deg:+5.2f}) "
                f"| hipR tau signed L {thL:+6.1f} R {thR:+6.1f} Nm")


if __name__ == "__main__":
    main()
